package dma

import chisel3._
import chisel3.util._
import dma.DmaConst._

/**
 * DMA Address Generator - Top (명세서 3장 Standard + 4장 Fusion)
 *
 * Arbiter 가 target_ocm 과 trigger 만 주면, RF 에 배선된 base/차원 정보로
 * 해당 타겟의 4KB chunk 분 주소 시퀀스를 스스로 생성한다.
 * fusion_mode 가 1 이면 Phase 교차까지 내부에서 처리한다.
 *
 * ============================================================
 * 구현 범위
 * ============================================================
 *   포함 : UB / WB / NB / PB 의 2D strided transpose, FB 의 circular fetch,
 *          chunk-tile-message 계층 카운터, chunk_last / dram_data_last,
 *          Fusion Phase 교차와 UB 위치 복원, 강제 전환
 *   제외 : TileLink node 연결 - 이 모듈은 순수 주소 스트림만 출력한다
 *          Write path (Put) - 후속
 *
 * 주소 생성기와 TileLink wrapper 를 분리한 이유는, 이 상태에서 Verilator 로
 * (addr, lgSize) 시퀀스만 덤프해 Python 레퍼런스와 대조할 수 있기 때문이다.
 * 버스를 붙인 뒤엔 주소 버그와 프로토콜 버그가 섞여 분리가 어려워진다.
 *
 * ============================================================
 * Phase 를 두 방식으로 구현하는 이유
 * ============================================================
 *   UB     : 같은 주소를 두 번 읽는다  -> nPtr = 1 + save/restore
 *   WB, PB : 다른 주소를 번갈아 읽는다 -> nPtr = 2 (내부 포인터 Vec)
 *
 * UB 에 포인터를 2개 두면 Phase 0 진입 때 ptr1 <- ptr0 복사가 필요해지는데,
 * 그것이 곧 save/restore 이므로 포인터를 늘릴 이득이 없다.
 *
 * 생성기는 OCM 당 하나씩 5개뿐이다. W1/W2, P1/P2 의 독립성은 생성기 내부
 * 포인터 Vec 이 담당하므로 top 에서 slot 을 나눌 필요가 없다.
 *
 * ============================================================
 * 타겟별 파라미터 매핑 (TargetAddrGen 주석 참조)
 * ============================================================
 *   UB : stride = intermNum, rows = rowNum,    cols = intermNum   nPtr=1
 *   WB : stride = colNum,    rows = intermNum, cols = colNum      nPtr=2
 *   PB : WB 와 동일                                               nPtr=2
 *   NB : WB 와 동일 (가정 A2 - 명세서에 base 규정 없음)            nPtr=1
 *   FB : 차원 무관. totalFreqSize 만 사용
 */
class DmaAddrGen(addrW: Int = 32, dimW: Int = 16) extends Module {
  val io = IO(new Bundle {
    // ---- Arbiter 인터페이스 ----
    val targetOcm = Input(UInt(log2Ceil(Ocm.NUM).W))
    val trigger   = Input(Bool())
    val burstDone = Output(Bool())

    // ---- host 인터페이스 ----
    /** RF 갱신 후 전 타겟의 주소를 base 로 되감고 Phase 를 0 으로 초기화한다 */
    val load = Input(Bool())

    // ---- RF 정적 배선 ----
    val cfg = Input(new DmaConfig(addrW, dimW))

    // ---- 메모리 요청 ----
    val req = Decoupled(new MemReq(addrW))

    // ---- OCM 제어 ----
    /** 4KB 를 채웠거나 텐서 경계에 닿음. 해당 OCM 의 Logical Full 유발 */
    val chunkLast = Output(Bool())
    /** 텐서의 진짜 끝. 해당 OCM 의 [LAST_TAG] 유발 후 영구 정지 */
    val tensorLast = Output(Bool())
    /** 위 두 신호가 어느 OCM 을 향한 것인지 */
    val lastOcm = Output(UInt(log2Ceil(Ocm.NUM).W))

    /**
     * 타겟별 정지 상태. Arbiter 는 halted 인 타겟을 grant 후보에서 빼야 한다.
     * 빼지 않아도 아래 즉시 ack 로직이 deadlock 은 막지만, 의미 없는 trigger 가
     * 반복되는 것은 Arbiter 쪽에서 거르는 것이 맞다.
     */
    val halted = Output(Vec(Ocm.NUM, Bool()))

    // ---- 관측용 ----
    val phase = Output(Vec(Ocm.NUM, Bool()))
  })

  // ------------------------------------------------------------
  // 인스턴스 - OCM 당 하나
  // ------------------------------------------------------------
  val ub = Module(new TargetAddrGen(addrW, dimW, nPtr = 1))
  val wb = Module(new TargetAddrGen(addrW, dimW, nPtr = 2))
  val pb = Module(new TargetAddrGen(addrW, dimW, nPtr = 2))
  val nb = Module(new TargetAddrGen(addrW, dimW, nPtr = 1))
  val fb = Module(new FreqAddrGen(addrW))

  val fusion = Module(new FusionCtrl(dimW))

  // ------------------------------------------------------------
  // 차원 배선
  // ------------------------------------------------------------
  ub.io.base        := VecInit(io.cfg.inputAddr)
  ub.io.strideTiles := io.cfg.intermNum
  ub.io.rowTiles    := io.cfg.rowNum
  ub.io.colTiles    := io.cfg.intermNum

  /** W / P / N 은 모두 (intermNum x colNum) tile 레이아웃을 가진다 */
  def wireWeightLike(m: TargetAddrGen): Unit = {
    m.io.strideTiles := io.cfg.colNum
    m.io.rowTiles    := io.cfg.intermNum
    m.io.colTiles    := io.cfg.colNum
  }
  wireWeightLike(wb); wb.io.base := io.cfg.weightAddr
  wireWeightLike(pb); pb.io.base := io.cfg.paramAddr
  wireWeightLike(nb); nb.io.base := VecInit(io.cfg.normAddr)

  fb.io.base          := io.cfg.freqAddr
  fb.io.totalFreqSize := io.cfg.totalFreqSize

  // ------------------------------------------------------------
  // Fusion 제어기 연결
  // ------------------------------------------------------------
  fusion.io.fusionMode := io.cfg.fusionMode
  fusion.io.intermNum  := io.cfg.intermNum
  fusion.io.load       := io.load

  io.phase := fusion.io.phase

  /** phase 가 그대로 포인터 번호가 된다 */
  ub.io.ptrSel := 0.U
  nb.io.ptrSel := 0.U
  wb.io.ptrSel := fusion.io.phase(Ocm.WB).asUInt
  pb.io.ptrSel := fusion.io.phase(Ocm.PB).asUInt

  // save / restore 는 UB 만 쓴다
  ub.io.save    := fusion.io.ubSave
  ub.io.restore := fusion.io.ubRestore
  Seq(wb, pb, nb).foreach { m =>
    m.io.save    := false.B
    m.io.restore := false.B
  }

  // ------------------------------------------------------------
  // 소유권 래치
  //
  // Arbiter 는 한 번에 하나의 타겟에만 버스를 준다. trigger 시점의 OCM 을
  // 래치해 그 chunk 가 끝날 때까지 소유권을 유지한다. Phase 전환은 chunk
  // 경계에서만 일어나므로(가정 A5) 전송 도중 포인터가 바뀌는 일은 없다.
  // ------------------------------------------------------------
  val ownerOcm   = RegInit(0.U(log2Ceil(Ocm.NUM).W))
  val ownerValid = RegInit(false.B)

  val haltedVec = VecInit(ub.io.halted, wb.io.halted, nb.io.halted,
                          pb.io.halted, fb.io.halted)
  io.halted := haltedVec

  /**
   * halted 인 타겟으로 trigger 가 오면 소유권을 주지 않고 다음 cycle 에
   * burstDone 만 돌려준다 (즉시 ack).
   *
   * 이 인터락이 없으면 deadlock 이 된다. 소유권만 잡히고 생성기는 요청을
   * 내지 않으므로 burstDone 이 영원히 오지 않고, Arbiter 가 응답을 기다리며
   * 멈춘다. 시뮬레이터는 타임아웃으로 알려주지만 하드웨어에는 그런 장치가 없다.
   *
   * 명세서상 dram_data_last 후 OCM 은 영구 정지하므로 Arbiter 가 그 타겟을
   * 요청하지 않는 것이 정상이지만, 그것을 강제하는 장치가 없으므로 방어한다.
   */
  val trigToHalted = io.trigger && haltedVec(io.targetOcm)
  val haltedAck    = RegInit(false.B)

  haltedAck := trigToHalted

  when(io.trigger) {
    ownerOcm   := io.targetOcm
    ownerValid := !haltedVec(io.targetOcm)
  }

  // ------------------------------------------------------------
  // start / load 라우팅
  // ------------------------------------------------------------
  def startFor(ocm: Int): Bool = io.trigger && (io.targetOcm === ocm.U)

  ub.io.start := startFor(Ocm.UB)
  wb.io.start := startFor(Ocm.WB)
  nb.io.start := startFor(Ocm.NB)
  pb.io.start := startFor(Ocm.PB)
  fb.io.start := startFor(Ocm.FB)

  Seq(ub, wb, pb, nb).foreach { _.io.load := io.load }
  fb.io.load := io.load

  // ------------------------------------------------------------
  // 출력 MUX
  //
  // 동시에 하나만 running 이므로 단순 선택으로 충분하다.
  // ready 는 소유 타겟에만 전달해 비소유 생성기의 오발행을 막는다.
  // Vec 인덱스는 Ocm 번호 그대로다.
  // ------------------------------------------------------------
  val reqBits = VecInit(ub.io.req.bits, wb.io.req.bits, nb.io.req.bits,
                        pb.io.req.bits, fb.io.req.bits)
  val reqValid = VecInit(ub.io.req.valid, wb.io.req.valid, nb.io.req.valid,
                         pb.io.req.valid, fb.io.req.valid)

  io.req.valid := reqValid(ownerOcm) && ownerValid
  io.req.bits  := reqBits(ownerOcm)

  def grant(ocm: Int): Bool = io.req.ready && ownerValid && (ownerOcm === ocm.U)

  ub.io.req.ready := grant(Ocm.UB)
  wb.io.req.ready := grant(Ocm.WB)
  nb.io.req.ready := grant(Ocm.NB)
  pb.io.req.ready := grant(Ocm.PB)
  fb.io.req.ready := grant(Ocm.FB)

  // ------------------------------------------------------------
  // 완료 신호를 Fusion 제어기로
  // ------------------------------------------------------------
  val doneVec = VecInit(ub.io.burstDone, wb.io.burstDone, nb.io.burstDone,
                        pb.io.burstDone, fb.io.burstDone)
  val chunkVec = VecInit(ub.io.last.chunkLast, wb.io.last.chunkLast,
                         nb.io.last.chunkLast, pb.io.last.chunkLast,
                         fb.io.last.chunkLast)
  /** 생성기가 올린 raw 경계 신호. Fusion 게이팅 전이다 */
  val rawEndVec = VecInit(ub.io.last.tensorLast, wb.io.last.tensorLast,
                          nb.io.last.tensorLast, pb.io.last.tensorLast,
                          fb.io.last.tensorLast)

  fusion.io.burstDone := doneVec
  fusion.io.rawEnd    := rawEndVec

  // ------------------------------------------------------------
  // 최종 출력
  //
  // tensorLast 는 Fusion 게이팅을 거친 값을 쓴다. Phase 0 에서의 경계 도달은
  // 강제 전환 트리거일 뿐 종료가 아니므로 밖으로 나가면 안 된다 (명세서 4.3).
  // ------------------------------------------------------------
  // 즉시 ack 는 burstDone 만 올린다. chunkLast / tensorLast 는 올리지 않는다.
  // 데이터가 오지 않았으므로 OCM 의 Logical Full 을 유발하면 안 된다.
  io.burstDone  := (doneVec(ownerOcm) && ownerValid) || haltedAck
  io.chunkLast  := chunkVec(ownerOcm) && ownerValid
  io.tensorLast := fusion.io.tensorLast(ownerOcm) && ownerValid
  io.lastOcm    := ownerOcm

  when(io.burstDone) { ownerValid := false.B }

  // ------------------------------------------------------------
  // 가드
  // ------------------------------------------------------------
  assert(!(io.trigger && ownerValid && !io.burstDone),
    "Arbiter issued a trigger while the previous chunk is still in flight")

  assert(!(haltedAck && ownerValid),
    "halted ack must not overlap an owned chunk")

  assert(PopCount(reqValid) <= 1.U,
    "more than one address generator is driving requests")

  assert(!(io.tensorLast && !io.chunkLast),
    "tensorLast must imply chunkLast")
}