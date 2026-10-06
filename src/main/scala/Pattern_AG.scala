package ag

import chisel3._
import chisel3.util._

/** DMA AG v3 - Pattern AG (공통 nested-loop engine + NCCocm profile)
  *
  *  AG v3 §14 의 "공통 nested-loop engine + profile" 구현 제안을 따름.
  *  loop 는 outer → mid → inner 3중. 주소 = base + outer*cO + mid*cM + inner*cI.
  *  profile 은 어느 index 가 주소에서 빠지는지(replay), LOCAL_CACHE 때 어느 loop 가 1로 수축하는지를 정함.
  *
  *    UB : outer=m, mid=n, inner=k.  n 이 주소에 없음 (row replay).  LC 는 n 수축.
  *    WB : outer=m, mid=n, inner=k.  m 이 주소에 없음 (weight 전체 재공급). LC 는 m 수축.
  *    QB/FB : outer=mid=1, inner 만 사용. rewind 는 loop_end 로 표현.
  *
  *  검증된 SV rev4 (ub_pattern_ag.sv) 와 동작이 같도록 작성함. 포트/폭은 임시값 (명세 미확정).
  */
object Collapse extends Enumeration { val Mid, Outer = Value }

case class AGProfile(
  name:       String,
  target:     Int,            // 0=UB 1=WB 2=QB 3=FB
  addrOuter:  Boolean,        // outer index 가 주소에 기여
  addrMid:    Boolean,        // mid   index 가 주소에 기여
  collapse:   Collapse.Value, // LOCAL_CACHE 때 1로 수축하는 loop (= 재사용 단위)
)
object AGProfile {
  val UB = AGProfile("UB", 0, addrOuter = true,  addrMid = false, collapse = Collapse.Mid)
  val WB = AGProfile("WB", 1, addrOuter = false, addrMid = true,  collapse = Collapse.Outer)
}

case class AGParams(
  addrW:      Int = 40,
  cntW:       Int = 16,
  chunkBytes: Int = 4096,
  localBytes: Int = 16384,
  idW:        Int = 8,
  seqW:       Int = 16,
  streamW:    Int = 32,
) {
  require(isPow2(chunkBytes) && isPow2(localBytes))
}

class AGDescriptor(p: AGParams) extends Bundle {
  val outerCnt, midCnt, innerCnt = UInt(p.cntW.W)
  val base                        = UInt(p.addrW.W)
  val strideOuter, strideMid, strideInner = UInt(p.addrW.W)
  val compact        = Bool()   // 1: tile=16B, 0: dense 256B
  val localCache     = Bool()   // 1: LOCAL_CACHE, 0: STREAM_SPILL
  val backingValid   = Bool()
  val ctxId, tensorId, cacheRegionId = UInt(p.idW.W)
  val cacheable, fillOnMiss = Bool()
}

/** canonical request (AG v3 §9) - IB/DRAM source 모름 */
class CanonicalReq(p: AGParams) extends Bundle {
  val ctxId, tensorId   = UInt(p.idW.W)
  val target            = UInt(3.W)
  val refillId          = UInt(p.seqW.W)   // 4KiB window 번호 (piece 들이 공유)
  val sequenceId        = UInt(p.seqW.W)   // piece 순번
  val dramAddr          = UInt(p.addrW.W)
  val dramBackingValid  = Bool()
  val bytes             = UInt(16.W)
  val dstOffset         = UInt(16.W)
  val groupId, phaseId  = UInt(p.cntW.W)   // outer / mid
  val epoch             = UInt(p.idW.W)
  val newEpoch          = Bool()           // mid==0: 이 group 의 resident chunk 를 IB 에 fill
  val loopEnd, chunkEnd, finalSupply = Bool()
  val cacheRegionId     = UInt(p.idW.W)
  val cacheable, fillOnMiss = Bool()
}

class PatternAG(prof: AGProfile, p: AGParams = AGParams()) extends Module {
  override def desiredName = s"PatternAG_${prof.name}"
  val io = IO(new Bundle {
    val cfgStart = Input(Bool())
    val cfgAbort = Input(Bool())
    val cfg      = Input(new AGDescriptor(p))
    // NCCocm 논리 이벤트: 다음 outer group 공급 허가 (reuse_done / new_group)  (AG v3 §3, Interaction §3)
    val newGroup = Input(Bool())
    // region state: 이전 epoch 의 IB read 전부 인계됨  (Interaction §7, IB v2 §4)
    val oldEpochDrained = Input(Bool())
    val req  = Decoupled(new CanonicalReq(p))
    val busy = Output(Bool())
    val done = Output(Bool())
    val cfgErr = Output(Bool())
  })

  // ---------------- descriptor latch ----------------
  val d       = Reg(new AGDescriptor(p))
  val sh      = RegInit(8.U(4.W))              // tile_bytes = 1 << sh

  // ---------------- cursor (fire 때만 전진) ----------------
  val outer, mid, inner = RegInit(0.U(p.cntW.W))
  val stream = RegInit(0.U(p.streamW.W))       // 누적 tile 수 (SPILL dst 용)
  val wpos   = RegInit(0.U(p.streamW.W))       // 현재 window 안의 tile 위치
  val win    = RegInit(0.U(p.seqW.W))          // window 번호 = refill_id
  val seq    = RegInit(0.U(p.seqW.W))
  val allowedGroups = RegInit(0.U((p.cntW + 1).W))  // NCCocm 이 허가한 outer group 수
  val run    = RegInit(false.B)
  val doneR  = RegInit(false.B)
  val errR   = RegInit(false.B)

  // ---------------- 효과 loop 수 (LOCAL_CACHE 수축) ----------------
  val lc      = d.localCache
  val midEff   = Mux(lc && (prof.collapse == Collapse.Mid).B,   1.U, d.midCnt)
  val outerEff = Mux(lc && (prof.collapse == Collapse.Outer).B, 1.U, d.outerCnt)

  // ---------------- next candidate (cursor 의 조합 함수 = preview) ----------------
  val ctW       = (p.chunkBytes.U(p.streamW.W) >> sh)                // window 당 tile 수
  val leftChunk = ctW - wpos
  val leftPass  = d.innerCnt - inner
  val len       = Mux(leftPass < leftChunk, leftPass, leftChunk)      // 이번 piece 길이 (tile)

  val isLoopEnd  = len === leftPass
  val isMidLast  = (mid   +& 1.U) >= midEff
  val isOuterLast= (outer +& 1.U) >= outerEff
  val isFinal    = isLoopEnd && isMidLast && isOuterLast
  // LC: 재사용 단위 끝에서 window 강제 close (UB: row 끝 = pass 끝 / WB: matrix 끝 = outer 끝)
  val lcUnitEnd  = isLoopEnd && ((prof.collapse == Collapse.Mid).B || isMidLast)
  val isChunkEnd = (len === leftChunk) || isFinal || (lc && lcUnitEnd)
  val newGroupFirst = (mid === 0.U) && (inner === 0.U) && (outer =/= 0.U)

  val addr = d.base +
    (if (prof.addrOuter) outer * d.strideOuter else 0.U) +
    (if (prof.addrMid)   mid   * d.strideMid   else 0.U) +
    inner * d.strideInner

  // dst: SPILL = 누적 circular, LC = 재사용 단위 내 offset
  val unitPos = (if (prof.collapse == Collapse.Mid) inner else mid * d.innerCnt + inner).pad(p.streamW)
  val dst     = Mux(lc, unitPos << sh, (stream << sh) & (p.localBytes - 1).U)

  io.req.bits.ctxId            := d.ctxId
  io.req.bits.tensorId         := d.tensorId
  io.req.bits.target           := prof.target.U
  io.req.bits.refillId         := win
  io.req.bits.sequenceId       := seq
  io.req.bits.dramAddr         := addr
  io.req.bits.dramBackingValid := d.backingValid
  io.req.bits.bytes            := (len << sh)(15, 0)
  io.req.bits.dstOffset        := dst(15, 0)
  io.req.bits.groupId          := outer
  io.req.bits.phaseId          := mid
  io.req.bits.epoch            := outer(p.idW - 1, 0)
  io.req.bits.newEpoch         := mid === 0.U
  io.req.bits.loopEnd          := isLoopEnd
  io.req.bits.chunkEnd         := isChunkEnd
  io.req.bits.finalSupply      := isFinal
  io.req.bits.cacheRegionId    := d.cacheRegionId
  io.req.bits.cacheable        := d.cacheable
  io.req.bits.fillOnMiss       := d.fillOnMiss

  // ---------------- 제시 조건 ----------------
  val groupOk = outer.pad(p.cntW + 1) < allowedGroups
  val gateOk  = groupOk && (!newGroupFirst || io.oldEpochDrained)
  val valid   = RegInit(false.B)
  io.req.valid := valid
  val fire     = io.req.fire
  val nxtNewGroup = isLoopEnd && isMidLast

  io.busy   := run
  io.done   := doneR
  io.cfgErr := errR

  // ---------------- 순차 ----------------
  when(io.cfgAbort) {
    run := false.B; doneR := false.B; valid := false.B
  }.otherwise {
    when(io.newGroup && run) { allowedGroups := allowedGroups + 1.U }

    when(io.cfgStart && !run) {
      d  := io.cfg
      sh := Mux(io.cfg.compact, 4.U, 8.U)
      outer := 0.U; mid := 0.U; inner := 0.U; seq := 0.U; stream := 0.U; wpos := 0.U; win := 0.U
      allowedGroups := 1.U; valid := false.B; errR := false.B
      // 재사용 단위가 local 에 들어갈 때만 LOCAL_CACHE 허용 (UB v2 §3)
      val unitTiles: UInt = (if (prof.collapse == Collapse.Mid) io.cfg.innerCnt.pad(p.streamW)
                             else (io.cfg.midCnt * io.cfg.innerCnt).pad(p.streamW))
      val unitBytes = unitTiles << Mux(io.cfg.compact, 4.U, 8.U)
      when(io.cfg.localCache && unitBytes > p.localBytes.U) {
        errR := true.B; run := false.B; doneR := true.B
      }.elsewhen(io.cfg.outerCnt === 0.U || io.cfg.midCnt === 0.U || io.cfg.innerCnt === 0.U) {
        run := false.B; doneR := true.B
      }.otherwise {
        run := true.B; doneR := false.B
      }
    }.elsewhen(run) {
      when(fire) {
        seq    := seq + 1.U
        stream := stream + len
        wpos   := Mux(isChunkEnd, 0.U, wpos + len)
        win    := Mux(isChunkEnd, win + 1.U, win)
        when(isLoopEnd) {
          inner := 0.U
          when(isMidLast) {
            mid := 0.U
            when(isOuterLast) { run := false.B; doneR := true.B }
              .otherwise      { outer := outer + 1.U }
          }.otherwise { mid := mid + 1.U }
        }.otherwise { inner := inner + len(p.cntW - 1, 0) }
        // lookahead: 다음 piece 가 같은 group 이면 valid 유지 (bubble 없음)
        valid := !isFinal && !nxtNewGroup
      }.elsewhen(!valid && gateOk) {
        valid := true.B
      }
    }
  }
}

object GenAG extends App {
  val dir = if (args.nonEmpty) args(0) else "gen"
  _root_.circt.stage.ChiselStage.emitSystemVerilogFile(new PatternAG(AGProfile.UB), Array("--target-dir", dir),
    Array("-disable-all-randomization", "-strip-debug-info", "--lowering-options=disallowLocalVariables,disallowPackedArrays,locationInfoStyle=none"))
  _root_.circt.stage.ChiselStage.emitSystemVerilogFile(new PatternAG(AGProfile.WB), Array("--target-dir", dir),
    Array("-disable-all-randomization", "-strip-debug-info", "--lowering-options=disallowLocalVariables,disallowPackedArrays,locationInfoStyle=none"))
}