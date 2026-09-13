package dma

import chisel3._
import chisel3.util._
import dma.DmaConst._

/**
 * FB (Frequency Buffer) 전용 주소 생성기 - 명세서 3.3 Smart Circular Fetch
 *
 * UB/WB 와 달리 transpose 도 stride 도 없다. 항상 선형 읽기이며, 차이는
 * "끝에 도달했을 때 무엇을 하는가" 뿐이다.
 *
 *   Cache Mode (totalFreqSize <= 32KB)
 *     전체가 FB 물리 용량에 들어가므로 단 한 번만 선형으로 읽는다.
 *     완료 시 tensorLast 를 올리고 영구 정지한다.
 *     이후 순환은 FB 내부에서 자체적으로 처리하므로 DRAM 접근이 0 이 된다.
 *
 *   Spill Mode (totalFreqSize > 32KB)
 *     용량 초과이므로 DRAM 을 계속 때려야 한다.
 *     누적 바이트가 totalFreqSize 에 도달하면 base 로 되감아 무한 반복한다.
 *     tensorLast 는 발생하지 않는다. host 가 레이어 종료 시 정지시켜야 한다.
 *
 * 주의: totalFreqSize 는 64B 의 정수배여야 한다. 그렇지 않으면 마지막 요청이
 *       텐서 경계를 넘어가 인접 영역을 읽게 된다.
 *
 * 신호 인가 규약:
 *   load 와 start 를 같은 cycle 에 주면 start 가 유실된다. load 는 상태를
 *   sWait 로 만들 뿐이고, sWait 에서의 start 판정은 다음 cycle 부터이기
 *   때문이다. host driver 는 load 이후 최소 1 cycle 을 띄워야 한다.
 */
class FreqAddrGen(addrW: Int = 32) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val load  = Input(Bool())

    val base          = Input(UInt(addrW.W))
    val totalFreqSize = Input(UInt(addrW.W))  // byte 단위

    val req  = Decoupled(new MemReq(addrW))
    val last = Output(new LastFlags)

    val burstDone = Output(Bool())
    val busy      = Output(Bool())
    val halted    = Output(Bool())
  })

  val sInit :: sWait :: sRun :: sHalt :: Nil = Enum(4)
  val state = RegInit(sInit)

  val curAddr = RegInit(0.U(addrW.W))
  val sentBytes = RegInit(0.U(addrW.W))                   // base 부터 누적
  val msgCnt  = RegInit(0.U(log2Ceil(MSG_PER_CHUNK).W))   // 0..63  chunk 내부

  val cacheMode = io.totalFreqSize <= FB_CAPACITY_BYTES.U

  io.busy   := state === sRun
  io.halted := state === sHalt

  io.req.valid       := state === sRun
  io.req.bits.addr   := curAddr
  io.req.bits.lgSize := LG_MSG_BYTES.U

  val fire = io.req.fire

  // ------------------------------------------------------------
  // 경계 판정
  // ------------------------------------------------------------
  /** 이번 요청으로 텐서를 다 읽게 되는가 */
  val freqEnd = (sentBytes + MSG_BYTES.U) >= io.totalFreqSize

  val lastMsgInChunk = msgCnt === (MSG_PER_CHUNK - 1).U

  /** Cache Mode 에서만 진짜 끝이 존재한다. Spill 은 끝없이 순환한다 */
  val tensorEnd = freqEnd && cacheMode

  val chunkEnd = lastMsgInChunk || tensorEnd

  io.last.chunkLast  := fire && chunkEnd
  io.last.tensorLast := fire && tensorEnd
  io.burstDone       := fire && chunkEnd

  // ------------------------------------------------------------
  // 주소 전진
  // ------------------------------------------------------------
  when(fire) {
    // chunk 경계에서 반드시 0 으로 돌아가야 한다 (TargetAddrGen 과 동일 이유).
    // Cache Mode 는 tensorEnd 가 msgCnt 63 이전에 뜰 수 있다.
    msgCnt := Mux(chunkEnd, 0.U, msgCnt + 1.U)

    when(freqEnd) {
      // Spill Mode 되감기. Cache Mode 는 어차피 sHalt 로 가므로 값은 무의미
      curAddr   := io.base
      sentBytes := 0.U
    }.otherwise {
      curAddr   := curAddr + MSG_BYTES.U
      sentBytes := sentBytes + MSG_BYTES.U
    }
  }

  // ------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------
  switch(state) {
    is(sInit) { when(io.load)  { state := sWait } }
    is(sWait) { when(io.start) { state := sRun  } }
    is(sRun) {
      when(fire && tensorEnd) {
        state := sHalt
      }.elsewhen(fire && chunkEnd) {
        state := sWait
      }
    }
    is(sHalt) { when(io.load) { state := sWait } }
  }

  /**
   * load 는 switch 보다 우선한다 (Chisel 은 마지막 대입이 이긴다).
   *
   * chunk 발행 도중(sRun)에 load 가 들어오면 switch 의 sRun 분기는 상태를
   * 그대로 두므로, 이 대입이 없으면 주소만 base 로 튄 채 trigger 없이
   * 계속 발행하게 된다. TargetAddrGen 의 running := false 와 같은 역할이다.
   *
   * 주의: 이 cycle 에 이미 fire 된 요청은 되돌릴 수 없다. 버스로 나간 Get 의
   * 응답이 나중에 D channel 로 돌아오므로, abort 의미로 load 를 쓰려면
   * wrapper 쪽에서 in-flight source ID 를 무효화해야 한다.
   */
  when(io.load) {
    state     := sWait
    curAddr   := io.base
    sentBytes := 0.U
    msgCnt    := 0.U
  }

  when(io.req.valid) {
    assert(io.req.bits.addr(LG_MSG_BYTES - 1, 0) === 0.U,
      "TileLink violation: addr must be 64B aligned")
    assert(io.totalFreqSize(LG_MSG_BYTES - 1, 0) === 0.U,
      "totalFreqSize must be a multiple of 64B")
  }
}