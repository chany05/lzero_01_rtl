package dma

import chisel3._
import chisel3.util._
import dma.DmaConst._

/**
 * Fusion Mode 제어기 - 명세서 4장
 *
 * ============================================================
 * Fusion 이 하는 일
 * ============================================================
 * 같은 입력 X 에 서로 다른 weight 두 벌을 곱하는 sibling(horizontal) fusion 이다.
 *
 *     Y1 = X * W1   (Phase 0)
 *     Y2 = X * W2   (Phase 1)
 *
 * Y1 은 Y2 와 결합되면 버려지는 중간 텐서이므로 DRAM 에 내리지 않는다.
 * 다만 Y1 전체를 on-chip 에 들고 있을 수는 없으므로, output tile 16개 분량씩
 * 잘라서 Phase 를 번갈아 돈다. 그 조각 단위가 이 제어기의 전환 주기다.
 *
 * ============================================================
 * 전환 조건 (4.1)
 * ============================================================
 * output tile 1개는 K 방향으로 intermNum 개의 input tile 을 누산해야 한다.
 * output tile 16개면
 *
 *     16 * intermNum tiles * 256B = intermNum * 4KB = intermNum chunk
 *
 * 따라서 4KB chunk 를 intermNum 개 보냈을 때 Phase 를 뒤집는다.
 *
 * burst_counter 는 UB / WB / PB 가 **각각 독립**으로 가진다. Arbiter 가
 * hungry 상태에 따라 비대칭으로 버스를 주므로, 전역 카운터 하나로는 각 타겟의
 * 진행도를 추적할 수 없기 때문이다.
 *
 * ============================================================
 * 강제 전환 (4.3)
 * ============================================================
 * 텐서 끝단에서는 burst_counter 가 intermNum 에 닿기 전에 행렬 경계를 만난다.
 * 이때 Phase 를 넘기지 않으면
 *
 *     Y1 의 꼬리 조각만 on-chip 에 남고
 *     -> 대응하는 Y2 조각이 영원히 오지 않아
 *     -> elementwise 결합이 완성되지 않고 deadlock
 *
 * 이 되므로, 카운터가 안 찼어도 강제로 전환한다. 최적화가 아니라 정합성 요구다.
 *
 * ============================================================
 * 최종 종료
 * ============================================================
 * dram_data_last 는 Phase 1 의 텐서 경계에서만 나간다. Phase 0 의 끝은 Y1 조각이
 * 완성된 것일 뿐 최종 출력이 아니므로, 거기서 종료를 알리면 Compute unit 이
 * Y2 를 기다리지 않고 멈춰 버린다.
 *
 * ============================================================
 * TODO (명세서 미확정)
 * ============================================================
 *   - UB / WB / PB 의 Phase 가 서로 어긋날 수 있다. UB 가 먼저 Phase 1 로 갔는데
 *     WB 가 아직 Phase 0 이면 X 의 두 번째 읽기에 W1 이 곱해진다.
 *     전역 barrier 가 필요한지 dataflow 매핑 확정 후 판단할 것.
 *   - WB 의 전환 임계값이 정말 intermNum 인지는, output tile 16개를 행 방향으로
 *     묶는지 열 방향으로 묶는지에 달려 있다. 후자면 16배 차이가 난다.
 */
class FusionCtrl(dimW: Int = 16) extends Module {
  val io = IO(new Bundle {
    val fusionMode = Input(Bool())
    val intermNum  = Input(UInt(dimW.W))
    val load       = Input(Bool())

    /** 타겟별 chunk 완료 펄스. Ocm 인덱스로 접근한다 */
    val burstDone = Input(Vec(Ocm.NUM, Bool()))
    /** 타겟별 "행렬 경계에 닿음" 펄스. 생성기의 raw tensorLast */
    val rawEnd = Input(Vec(Ocm.NUM, Bool()))

    /** 타겟별 현재 Phase. slot 선택과 UB 제어에 쓰인다 */
    val phase = Output(Vec(Ocm.NUM, Bool()))

    /** UB 위치 저장/복원 (명세서 4.2) */
    val ubSave    = Output(Bool())
    val ubRestore = Output(Bool())

    /** 밖으로 내보낼 진짜 종료 신호. Phase 1 경계에서만 뜬다 */
    val tensorLast = Output(Vec(Ocm.NUM, Bool()))
  })

  /** Fusion 에 참여하는 타겟. NB / FB 는 Phase 개념이 없다 */
  val FUSED = Seq(Ocm.UB, Ocm.WB, Ocm.PB)

  val phaseReg = RegInit(VecInit(Seq.fill(Ocm.NUM)(false.B)))
  val cntReg   = RegInit(VecInit(Seq.fill(Ocm.NUM)(0.U(dimW.W))))

  // 기본값 - 비참여 타겟은 항상 Phase 0
  for (i <- 0 until Ocm.NUM) {
    io.phase(i)      := Mux(io.fusionMode, phaseReg(i), false.B)
    io.tensorLast(i) := io.rawEnd(i)
  }

  val saveWire    = WireDefault(false.B)
  val restoreWire = WireDefault(false.B)

  for (t <- FUSED) {
    /** 이번 chunk 로 intermNum 을 채우는가 */
    val counterFull = (cntReg(t) + 1.U) === io.intermNum

    /** 카운터가 찼거나, 못 찼아도 행렬 경계를 만났거나 (4.3 강제 전환) */
    val doSwitch = io.burstDone(t) && io.fusionMode && (counterFull || io.rawEnd(t))

    when(io.burstDone(t) && io.fusionMode) {
      cntReg(t) := Mux(doSwitch, 0.U, cntReg(t) + 1.U)
      when(doSwitch) { phaseReg(t) := !phaseReg(t) }
    }

    /**
     * 종료 게이팅.
     * Phase 0 에서의 경계 도달은 "강제 전환 트리거"일 뿐 종료가 아니다.
     * Phase 1 에서 도달했을 때만 밖으로 내보낸다.
     */
    when(io.fusionMode) {
      io.tensorLast(t) := io.rawEnd(t) && phaseReg(t)
    }

    if (t == Ocm.UB) {
      // Phase 1 로 진입 -> 백업 위치로 복원해 같은 X 를 다시 읽는다
      restoreWire := doSwitch && !phaseReg(t)
      // Phase 0 으로 복귀 -> 현재 위치가 다음 영역의 시작이므로 백업한다
      saveWire    := doSwitch && phaseReg(t)
    }
  }

  io.ubSave    := saveWire
  io.ubRestore := restoreWire

  when(io.load) {
    for (i <- 0 until Ocm.NUM) {
      phaseReg(i) := false.B
      cntReg(i)   := 0.U
    }
  }

  assert(!(io.ubSave && io.ubRestore),
    "UB save/restore collision")
}
