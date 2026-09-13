package dma

import chisel3._
import chisel3.util._
import dma.DmaConst._

/**
 * 2D Strided + Wrap-around 주소 생성기 (UB / WB / NB / PB 공용)
 *
 * ============================================================
 * strided 와 wrap-around 는 같은 동작이 아니다
 * ============================================================
 *
 *   for (col = 0; col < colTiles; col++)      <- wrap-around 가 담당 (바깥 루프)
 *       for (row = 0; row < rowTiles; row++)  <- strided 가 담당 (안쪽 루프)
 *           read_tile(base + col*256 + row*strideTiles*256)
 *
 *   strided     : addr += strideTiles * 256   (누적 가산)
 *   wrap-around : addr  = base + col * 256    (절대 주소 재계산)
 *
 * 예) strideTiles = 4, rowTiles = 3 인 4x3 tile 행렬. 단위는 256B
 *
 *          col0 col1 col2 col3
 *   row0 :   0    1    2    3
 *   row1 :   4    5    6    7
 *   row2 :   8    9   10   11
 *
 *   방문 순서 : 0 -> 4 -> 8 -> [wrap] 1 -> 5 -> 9 -> [wrap] 2 -> ...
 *   주소 변화 :   +4   +4     -7      +4   +4     -7
 *
 * 세로로 훑어 내려가므로, DRAM 에 row-major 로 누운 행렬을 재배치 없이
 * column 단위로 뽑아내게 된다. 이것이 명세서 3.1 의 Tiled Transpose Read 이다.
 *
 * ============================================================
 * nPtr - 내부 포인터 다중화
 * ============================================================
 * 명세서 4.2 는 WB / PB 가 W1/W2, P1/P2 포인터를 독립적으로 유지한다고 규정한다.
 * 이를 모듈 인스턴스 복제가 아니라 내부 포인터 Vec 으로 구현한다.
 *
 *   포인터별 : tileAddr, rowCnt, colCnt, halted
 *   공   유 : msgCnt, tileCnt, 차원 입력, 가산기, 비교기
 *
 * msgCnt / tileCnt 를 공유해도 되는 근거는 가정 A5 다. Phase 전환은 chunk
 * 경계에서만 일어나므로 전환 시점에 두 값은 항상 0 이다.
 *
 *   nPtr = 1 : UB, NB  (UB 는 대신 save/restore 로 같은 위치를 재방문한다)
 *   nPtr = 2 : WB, PB
 *   nPtr = 3 : QKV projection 같은 3-way 확장 시
 *
 * ============================================================
 * 타겟별 파라미터 매핑
 * ============================================================
 * (신호 인가 규약은 아래 load 처리 주석 참조)
 *   UB : X 는 (rowNum x intermNum) tile
 *        -> strideTiles = intermNum, rowTiles = rowNum,    colTiles = intermNum
 *   WB : W 는 (intermNum x colNum) tile
 *        -> strideTiles = colNum,    rowTiles = intermNum, colTiles = colNum
 *
 * stride 값이 다른 이유는 각자 자기 행렬의 가로폭(tile 수)을 쓰기 때문이다.
 * 한 tile-row 를 통째로 건너뛰려면 그 행렬의 가로폭만큼 점프해야 한다.
 */
class TargetAddrGen(addrW: Int = 32, dimW: Int = 16, nPtr: Int = 1) extends Module {
  require(nPtr >= 1)
  val selW = math.max(1, log2Ceil(nPtr))

  val io = IO(new Bundle {
    /** Arbiter 의 trigger 펄스. chunk 1개 발행을 시작/재개한다 */
    val start = Input(Bool())
    /** host 가 RF 를 갱신한 뒤 주는 펄스. 모든 포인터를 base 로 되감는다 */
    val load  = Input(Bool())

    /** 이번 chunk 에 쓸 포인터 번호. Fusion 의 phase 가 그대로 들어온다 */
    val ptrSel = Input(UInt(selW.W))

    // ---- RF 정적 배선 ----
    val base        = Input(Vec(nPtr, UInt(addrW.W)))
    val strideTiles = Input(UInt(dimW.W))
    val rowTiles    = Input(UInt(dimW.W))
    val colTiles    = Input(UInt(dimW.W))

    /**
     * Fusion 용 위치 저장/복원 (명세서 4.2 의 saved_input_addr).
     * nPtr = 1 인 UB 전용이다. 같은 위치를 두 번 읽어야 하므로 포인터를
     * 늘리는 대신 shadow 에 백업했다가 되돌린다.
     *   save    : 현재 위치를 백업     (Phase 0 진입)
     *   restore : 백업 위치로 복원     (Phase 1 진입). halt 도 함께 풀린다
     */
    val save    = Input(Bool())
    val restore = Input(Bool())

    // ---- 출력 ----
    val req  = Decoupled(new MemReq(addrW))
    val last = Output(new LastFlags)

    /** 이번 chunk 의 요청을 모두 버스에 실었음 (Arbiter 에게) */
    val burstDone = Output(Bool())
    val busy      = Output(Bool())
    /** 현재 포인터가 텐서를 소진해 정지 상태인가 */
    val halted    = Output(Bool())
  })

  // ------------------------------------------------------------
  // 상태
  //
  //   loaded  : base 적재 완료 (공유)
  //   running : chunk 발행 중  (공유 - 동시에 한 chunk 만 돈다)
  //   halted  : 텐서 소진      (포인터별 - W1 이 먼저 끝날 수 있다)
  // ------------------------------------------------------------
  val loaded  = RegInit(false.B)
  val running = RegInit(false.B)
  val halted  = RegInit(VecInit(Seq.fill(nPtr)(false.B)))

  /** 포인터별 위치 */
  val tileAddr = RegInit(VecInit(Seq.fill(nPtr)(0.U(addrW.W))))
  val rowCnt   = RegInit(VecInit(Seq.fill(nPtr)(0.U(dimW.W))))
  val colCnt   = RegInit(VecInit(Seq.fill(nPtr)(0.U(dimW.W))))

  /** Fusion 위치 백업본 (nPtr = 1 전용) */
  val shadowAddr = RegInit(0.U(addrW.W))
  val shadowRow  = RegInit(0.U(dimW.W))
  val shadowCol  = RegInit(0.U(dimW.W))

  /** chunk 내부 카운터 - 공유 (가정 A5) */
  val msgCnt  = RegInit(0.U(log2Ceil(MSG_PER_TILE).W))    // 0..3
  val tileCnt = RegInit(0.U(log2Ceil(TILE_PER_CHUNK).W))  // 0..15

  /**
   * nPtr = 1 이면 인덱스를 상수 0 으로 접는다.
   * io.ptrSel 을 그대로 쓰면 크기 1 Vec 에 폭 1 인덱스가 들어가
   * W004(dynamic index too wide) 경고가 접근마다 뜬다.
   */
  val sel = if (nPtr == 1) 0.U else io.ptrSel

  io.busy   := running
  io.halted := halted(sel)

  // ------------------------------------------------------------
  // 요청 생성
  //
  // lgSize 는 항상 6 (64B) 으로 고정한다.
  //   - 행렬이 tile(256B) 정수배라는 가정 A3 하에서 64B 미만 잔여가 없음
  //   - 꼬리 처리는 "size 를 줄이는 것"이 아니라 "발행 개수를 줄이는 것"으로 대체됨
  //     명세서 4.3 의 "잔여 1.5KB 를 한 방에 쏜다" 는 TileLink 에서 불법이다.
  //     a_size 는 2의 거듭제곱 바이트만 표현할 수 있기 때문
  // 가정 A3 가 깨지면 아래처럼 축소해야 한다:
  //   lgSize = min(ctz(addr), log2(remaining), LG_MSG_BYTES)
  // ------------------------------------------------------------
  io.req.valid       := running
  io.req.bits.addr   := tileAddr(sel) + (msgCnt << LG_MSG_BYTES)
  io.req.bits.lgSize := LG_MSG_BYTES.U

  val fire = io.req.fire

  // ------------------------------------------------------------
  // 경계 판정 - 활성 포인터 하나에 대해서만 계산한다
  // ------------------------------------------------------------
  val lastMsgInTile   = msgCnt  === (MSG_PER_TILE - 1).U
  val lastTileInChunk = tileCnt === (TILE_PER_CHUNK - 1).U
  val lastRow         = rowCnt(sel) === (io.rowTiles - 1.U)
  val lastCol         = colCnt(sel) === (io.colTiles - 1.U)

  /** 행렬의 마지막 tile 의 마지막 message */
  val tensorEnd = lastMsgInTile && lastRow && lastCol

  /** 4KB 를 다 채웠거나, 채우기 전에 텐서 경계에 닿았거나 */
  val chunkEnd = (lastMsgInTile && lastTileInChunk) || tensorEnd

  io.last.chunkLast  := fire && chunkEnd
  io.last.tensorLast := fire && tensorEnd
  io.burstDone       := fire && chunkEnd

  // ------------------------------------------------------------
  // 주소 전진
  //
  // tile 단위 row-major 저장이므로 tile (row, col) 의 절대 주소는
  //     addr = base + (row * strideTiles + col) * 256
  // 이다. row * strideTiles 는 transpose 때문이 아니라 2D -> 1D
  // linearization 에서 필연적으로 나오는 항이다.
  //
  // 다만 여기서는 절대식을 쓰지 않는다. row 가 항상 1씩 증가하므로
  //     (row+1)*strideTiles = row*strideTiles + strideTiles
  // 로 곱셈이 덧셈으로 퇴화하고, wrap 시에는 row = 0 이라
  //     addr = base + col * 256      (256 배는 shift)
  // 가 되어 곱셈이 나올 자리가 없다. 변수 x 변수 곱셈기를 한 개 아끼는 셈이다.
  // ------------------------------------------------------------
  /**
   * 다음 위치를 먼저 wire 로 뽑는다.
   *
   * 레지스터에 바로 대입하면 같은 cycle 에 save 를 뜨우는 Fusion 경로가
   * **전진 전** 값을 캡처한다. 레지스터 갱신은 다음 cycle 에 반영되기 때문이다.
   * 그러면 복원 위치가 한 타일 뒤로 밀린다. 두 소비자가 같은 wire 를 보게 한다.
   */
  val nextTileAddr = WireDefault(tileAddr(sel))
  val nextRow      = WireDefault(rowCnt(sel))
  val nextCol      = WireDefault(colCnt(sel))

  when(lastMsgInTile) {
    when(lastRow) {
      // ---- wrap-around : base 로부터 절대 주소 재계산 ----
      nextRow      := 0.U
      nextCol      := colCnt(sel) + 1.U
      nextTileAddr := io.base(sel) + ((colCnt(sel) + 1.U) << log2Ceil(TILE_BYTES))
    }.otherwise {
      // ---- strided : 한 tile-row 만큼 누적 점프 ----
      nextRow      := rowCnt(sel) + 1.U
      nextTileAddr := tileAddr(sel) + (io.strideTiles << log2Ceil(TILE_BYTES))
    }
  }

  when(fire) {
    when(!lastMsgInTile) {
      // tile 내부 - 주소 오프셋은 조합적으로 더해지므로 msgCnt 만 올린다
      msgCnt := msgCnt + 1.U
    }.otherwise {
      msgCnt := 0.U
      // chunk 경계에서 반드시 0 으로 돌아가야 한다. lastTileInChunk 만 보면
      // 텐서가 tile 15 전에 끝나는 tail chunk 에서 tileCnt 가 남아, 다음
      // chunk 가 중간부터 시작해 짧게 끊긴다. chunkEnd = lastTileInChunk
      // || tensorEnd 이므로 tail 도 함께 덮인다.
      tileCnt := Mux(lastTileInChunk || tensorEnd, 0.U, tileCnt + 1.U)

      tileAddr(sel) := nextTileAddr
      rowCnt(sel)   := nextRow
      colCnt(sel)   := nextCol
    }
  }

  // ------------------------------------------------------------
  // 제어
  // ------------------------------------------------------------
  when(io.start && loaded && !halted(sel)) { running := true.B }

  when(fire && chunkEnd)  { running := false.B }
  when(fire && tensorEnd) { halted(sel) := true.B }

  // ---- Fusion save / restore (nPtr = 1 전용) ----
  /**
   * save 는 **전진 후** 위치를 백업한다.
   *
   * save 가 뜨는 cycle 은 chunk 의 마지막 message 가 fire 되는 cycle 이므로,
   * tileAddr 레지스터는 아직 이전 타일을 가리킨다. 그 값을 저장하면 다음
   * Phase 0 이 한 타일 뒤에서 시작해 영역이 어긋난다.
   * fire 가 아닌 save 는 정상 흐름에 없지만, 그때는 next* 가 현재값이라 안전하다.
   */
  when(io.save) {
    shadowAddr := Mux(fire, nextTileAddr, tileAddr(sel))
    shadowRow  := Mux(fire, nextRow,      rowCnt(sel))
    shadowCol  := Mux(fire, nextCol,      colCnt(sel))
  }

  /**
   * restore 는 halt 도 함께 푼다.
   * Phase 0 에서 텐서 경계에 닿은 chunk 는 같은 cycle 에 halted 를 세우는데,
   * 명세서 4.3 의 강제 전환은 바로 그 cycle 에 restore 를 올린다.
   * 아래 대입이 위의 halted 대입보다 뒤에 있어야 한다 (Chisel 은 마지막이 이긴다).
   */
  when(io.restore) {
    tileAddr(sel) := shadowAddr
    rowCnt(sel)   := shadowRow
    colCnt(sel)   := shadowCol
    halted(sel)   := false.B
    msgCnt        := 0.U   // 가정 A5 에 의해 이미 0 이지만 방어적으로
    tileCnt       := 0.U
  }

  /**
   * load 가 최우선이다 (Chisel 은 마지막 대입이 이긴다).
   * running := false 로 chunk 발행을 강제 중단한다. 이 줄이 없으면 발행
   * 도중 load 가 들어왔을 때 주소만 base 로 튄 채 계속 쏘게 된다.
   *
   * 신호 인가 규약: load 와 start 를 같은 cycle 에 주면 start 는 유실된다.
   * loaded 레지스터가 그 cycle 에는 아직 false 이기 때문이다.
   * host driver 는 load 이후 최소 1 cycle 을 띄워야 한다.
   *
   * 주의: 이 cycle 에 이미 fire 된 요청은 되돌릴 수 없다. abort 의미로
   * load 를 쓰려면 wrapper 쪽에서 in-flight source ID 를 무효화해야 한다.
   */
  when(io.load) {
    loaded  := true.B
    running := false.B
    msgCnt  := 0.U
    tileCnt := 0.U
    for (p <- 0 until nPtr) {
      tileAddr(p) := io.base(p)
      rowCnt(p)   := 0.U
      colCnt(p)   := 0.U
      halted(p)   := false.B
    }
    shadowAddr := io.base(0)
    shadowRow  := 0.U
    shadowCol  := 0.U
  }

  // ------------------------------------------------------------
  // 가드
  // ------------------------------------------------------------
  when(io.req.valid) {
    assert(io.req.bits.addr(LG_MSG_BYTES - 1, 0) === 0.U,
      "TileLink violation: addr must be 64B aligned")
    assert(io.rowTiles =/= 0.U && io.colTiles =/= 0.U,
      "rowTiles / colTiles must be nonzero")
  }
  assert(!(io.save && io.restore), "save and restore must not assert together")
  assert(!(io.start && running), "start pulsed while a chunk is still in flight")

  // TODO(perf): rowTiles == 1 이면 매 tile 마다 wrap 경로를 타게 된다.
  //   주소는 결과적으로 linear 이므로 전용 linear 경로를 두면
  //     (a) 인접 message 를 더 큰 Get 하나로 병합 가능
  //     (b) DRAM row buffer hit 율 상승
  //   decode 단계(M = batch = 16 -> rowNum = 1)에서 실제로 발생하는 조건이다.
}