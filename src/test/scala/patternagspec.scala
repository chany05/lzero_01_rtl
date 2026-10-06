package ag

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.util.Random

/** PatternAG 검증 - reference model 과 piece 단위 비교 + 계약 검사
  *  - 주소·길이·dst·refillId·group/phase·epoch·마커 4종 전부 비교
  *  - stall 중 valid 유지, payload 불변
  *  - 새 group 첫 piece 는 newGroup 허가 뒤·drained 일 때만 제시
  *  - 같은 group 안에서는 accept 다음 cycle 에 bubble 없음 (ready=100% 케이스)
  *  - finalSupply 정확히 1회, LC 는 group 경계마다 새 refillId
  *  - zero length / abort / 불법 LOCAL_CACHE
  */
class PatternAGSpec extends AnyFlatSpec with ChiselScalatestTester {

  case class Piece(addr: BigInt, bytes: Int, dst: Int, rid: Int, outer: Int, mid: Int,
                   loopEnd: Boolean, chunkEnd: Boolean, fin: Boolean, newEpoch: Boolean,
                   groupFirst: Boolean)

  /** 중첩 loop 를 그대로 풀어 기대 trace 생성 (RTL 과 독립된 계산) */
  def reference(prof: AGProfile, mt: Int, nt: Int, kt: Int, lc: Boolean, sh: Int,
                base: BigInt, sO: BigInt, sM: BigInt, sI: BigInt, localBytes: Int = 16384): Seq[Piece] = {
    val ct = 4096 >> sh
    val outerEff = if (lc && prof.collapse == Collapse.Outer) 1 else mt
    val midEff   = if (lc && prof.collapse == Collapse.Mid)   1 else nt
    val out = Seq.newBuilder[Piece]
    var st = 0; var wp = 0; var win = 0
    for (o <- 0 until outerEff; m <- 0 until midEff) {
      var k = 0
      while (k < kt) {
        val lp = kt - k; val lcnk = ct - wp; val len = math.min(lp, lcnk)
        val loopEnd = k + len == kt
        val midLast = m + 1 >= midEff; val outerLast = o + 1 >= outerEff
        val fin = loopEnd && midLast && outerLast
        val lcUnitEnd = loopEnd && (prof.collapse == Collapse.Mid || midLast)
        val ce = len == lcnk || fin || (lc && lcUnitEnd)
        val addr = base + (if (prof.addrOuter) BigInt(o) * sO else 0) + (if (prof.addrMid) BigInt(m) * sM else 0) + BigInt(k) * sI
        val unitPos = if (prof.collapse == Collapse.Mid) k else m * kt + k
        val dst = if (lc) unitPos << sh else (st << sh) % localBytes
        out += Piece(addr, len << sh, dst, win, o, m, loopEnd, ce, fin, m == 0, m == 0 && k == 0 && o > 0)
        st += len; k += len
        if (ce) { wp = 0; win += 1 } else wp += len
      }
    }
    out.result()
  }

  case class Cfg(mt: Int, nt: Int, kt: Int, lc: Boolean = false, compact: Boolean = false,
                 pReady: Int = 100, ngDelayMax: Int = 0, seed: Int = 1)

  def runCase(prof: AGProfile, c: Cfg)(dut: PatternAG): Unit = {
    val sh = if (c.compact) 4 else 8
    val base = BigInt("8000000000", 16) & ((BigInt(1) << 40) - 1)
    val sI = BigInt(1) << sh; val sM = BigInt(c.kt) << sh; val sO = sM * c.nt
    val ref = reference(prof, c.mt, c.nt, c.kt, c.lc, sh, base, sO, sM, sI)
    val rnd = new Random(c.seed)

    dut.io.cfg.outerCnt.poke(c.mt.U); dut.io.cfg.midCnt.poke(c.nt.U); dut.io.cfg.innerCnt.poke(c.kt.U)
    dut.io.cfg.base.poke(base.U); dut.io.cfg.strideOuter.poke(sO.U); dut.io.cfg.strideMid.poke(sM.U); dut.io.cfg.strideInner.poke(sI.U)
    dut.io.cfg.compact.poke(c.compact.B); dut.io.cfg.localCache.poke(c.lc.B); dut.io.cfg.backingValid.poke(true.B)
    dut.io.cfg.ctxId.poke(3.U); dut.io.cfg.tensorId.poke(7.U); dut.io.cfg.cacheRegionId.poke(1.U)
    dut.io.cfg.cacheable.poke(true.B); dut.io.cfg.fillOnMiss.poke(true.B)
    dut.io.req.ready.poke(false.B); dut.io.newGroup.poke(false.B); dut.io.oldEpochDrained.poke(false.B); dut.io.cfgAbort.poke(false.B)
    dut.io.cfgStart.poke(true.B); dut.clock.step(); dut.io.cfgStart.poke(false.B)

    var idx = 0; var fsSeen = 0; var cycles = 0
    var granted = 1; var ngPending = false; var ngTimer = 0
    var prevValid = false; var prevStalled = false; var prevFireSameGroup = false; var drainedPrev = false
    var prevPayload: Option[(BigInt, Int, Boolean, Boolean, Boolean)] = None

    val peek = () => (dut.io.req.bits.dramAddr.peek().litValue, dut.io.req.bits.bytes.peek().litValue.toInt,
                      dut.io.req.bits.loopEnd.peek().litToBoolean, dut.io.req.bits.chunkEnd.peek().litToBoolean,
                      dut.io.req.bits.finalSupply.peek().litToBoolean)

    while (!dut.io.done.peek().litToBoolean && cycles < 200000) {
      // ---- 이번 cycle 입력 ----
      val valid = dut.io.req.valid.peek().litToBoolean
      val ready = valid && rnd.nextInt(100) < c.pReady
      val drained = rnd.nextInt(100) < 60
      var ng = false
      if (ngPending) { if (ngTimer == 0) { ng = true; ngPending = false; granted += 1 } else ngTimer -= 1 }
      dut.io.req.ready.poke(ready.B); dut.io.oldEpochDrained.poke(drained.B); dut.io.newGroup.poke(ng.B)

      // ---- 계약 검사 ----
      if (prevStalled) {
        assert(valid, s"valid dropped before accept @${cycles}")
        assert(prevPayload.contains(peek()), s"payload changed during stall @${cycles}")
      }
      if (valid && !prevValid && idx < ref.size && ref(idx).groupFirst) {
        assert(granted > ref(idx).outer, s"idx=$idx: group ${ref(idx).outer} issued before newGroup (granted=$granted)")
        assert(drainedPrev, s"idx=$idx: group ${ref(idx).outer} issued before old epoch drained")
      }
      if (c.pReady == 100 && prevFireSameGroup) assert(valid, s"bubble after accept within group @${cycles}")

      // ---- fire: reference 비교 ----
      var fireSameGroup = false
      if (valid && ready) {
        assert(idx < ref.size, s"extra request idx=$idx")
        val e = ref(idx); val b = dut.io.req.bits
        assert(b.dramAddr.peek().litValue == e.addr, s"idx=$idx addr ${b.dramAddr.peek().litValue.toString(16)} != ${e.addr.toString(16)}")
        assert(b.bytes.peek().litValue.toInt == e.bytes, s"idx=$idx bytes")
        assert(b.dstOffset.peek().litValue.toInt == e.dst, s"idx=$idx dst ${b.dstOffset.peek().litValue} != ${e.dst}")
        assert(b.refillId.peek().litValue.toInt == e.rid, s"idx=$idx refillId ${b.refillId.peek().litValue} != ${e.rid}")
        assert(b.sequenceId.peek().litValue.toInt == idx, s"idx=$idx sequenceId")
        assert(b.groupId.peek().litValue.toInt == e.outer && b.phaseId.peek().litValue.toInt == e.mid, s"idx=$idx group/phase")
        assert(b.epoch.peek().litValue.toInt == (e.outer & 0xff), s"idx=$idx epoch")
        assert(b.loopEnd.peek().litToBoolean == e.loopEnd, s"idx=$idx loopEnd")
        assert(b.chunkEnd.peek().litToBoolean == e.chunkEnd, s"idx=$idx chunkEnd")
        assert(b.finalSupply.peek().litToBoolean == e.fin, s"idx=$idx finalSupply")
        assert(b.newEpoch.peek().litToBoolean == e.newEpoch, s"idx=$idx newEpoch")
        assert(b.ctxId.peek().litValue == 3 && b.tensorId.peek().litValue == 7 && b.cacheRegionId.peek().litValue == 1 &&
               b.target.peek().litValue == prof.target, s"idx=$idx identity")
        if (e.fin) fsSeen += 1
        val groupEnd = e.loopEnd && (idx + 1 < ref.size) && ref(idx + 1).outer != e.outer
        if (groupEnd) { ngPending = true; ngTimer = if (c.ngDelayMax == 0) 0 else rnd.nextInt(c.ngDelayMax) }
        fireSameGroup = !e.fin && !groupEnd
        idx += 1
      }

      prevStalled = valid && !ready; prevPayload = if (valid) Some(peek()) else None
      prevValid = valid; prevFireSameGroup = fireSameGroup; drainedPrev = drained
      dut.clock.step(); cycles += 1
    }
    dut.io.req.ready.poke(false.B)
    assert(dut.io.done.peek().litToBoolean, "timeout")
    assert(!dut.io.cfgErr.peek().litToBoolean, "unexpected cfgErr")
    assert(idx == ref.size, s"piece count $idx != ${ref.size}")
    assert(ref.isEmpty || fsSeen == 1, s"finalSupply seen $fsSeen")
    if (c.lc) {
      ref.sliding(2).foreach { case Seq(a, b) => if (a.outer != b.outer) assert(a.rid != b.rid, "LC window straddles group") }
    }
    info(f"${prof.name} mt=${c.mt} nt=${c.nt} kt=${c.kt} lc=${c.lc} compact=${c.compact} ready=${c.pReady}%% -> ${idx} pieces, ${cycles} cycles")
  }

  behavior of "PatternAG UB"
  Seq(
    "proposal_spill (bubble-free)" -> Cfg(8, 64, 128),
    "proposal backpressure"        -> Cfg(2, 16, 128, pReady = 30, ngDelayMax = 50),
    "UB v2 §9: Kt=6 Nt=3 -> 6+6+4"  -> Cfg(2, 3, 6),
    "tail Kt=20"                   -> Cfg(2, 3, 20, pReady = 50, ngDelayMax = 20),
    "Kt=17 / Kt=16 boundary"       -> Cfg(3, 2, 17, pReady = 60),
    "min 1x1x1"                    -> Cfg(1, 1, 1),
    "LC Kt=64 (row = 16KiB)"       -> Cfg(4, 8, 64, lc = true, pReady = 70, ngDelayMax = 30),
    "LC Kt=40 late newGroup"       -> Cfg(5, 4, 40, lc = true, pReady = 70, ngDelayMax = 200),
    "compact spill"                -> Cfg(2, 3, 600, compact = true, pReady = 60),
    "compact LC"                   -> Cfg(3, 4, 500, lc = true, compact = true, pReady = 60),
  ).foreach { case (name, cfg) =>
    it should name in { test(new PatternAG(AGProfile.UB))(runCase(AGProfile.UB, cfg)) }
  }

  it should "UB v2 §9 first chunk shape" in {
    val r = reference(AGProfile.UB, 2, 3, 6, lc = false, 8, 0, 0, 0, 256)
    assert(r.take(3).map(_.bytes) == Seq(6 * 256, 6 * 256, 4 * 256))
    assert(r.take(3).map(_.chunkEnd) == Seq(false, false, true))
    assert(r.take(4).map(_.rid) == Seq(0, 0, 0, 1))
  }

  it should "issue nothing on zero length" in {
    test(new PatternAG(AGProfile.UB)) { dut => runCase(AGProfile.UB, Cfg(0, 4, 4))(dut) }
  }

  it should "stop presenting after abort" in {
    test(new PatternAG(AGProfile.UB)) { dut =>
      dut.io.cfg.outerCnt.poke(4.U); dut.io.cfg.midCnt.poke(8.U); dut.io.cfg.innerCnt.poke(128.U)
      dut.io.cfg.base.poke(0.U); dut.io.cfg.strideOuter.poke(32768.U); dut.io.cfg.strideMid.poke(0.U); dut.io.cfg.strideInner.poke(256.U)
      dut.io.cfg.compact.poke(false.B); dut.io.cfg.localCache.poke(false.B); dut.io.cfg.backingValid.poke(true.B)
      dut.io.oldEpochDrained.poke(true.B); dut.io.req.ready.poke(true.B)
      dut.io.cfgStart.poke(true.B); dut.clock.step(); dut.io.cfgStart.poke(false.B)
      dut.clock.step(40)
      dut.io.cfgAbort.poke(true.B); dut.clock.step(); dut.io.cfgAbort.poke(false.B)
      for (_ <- 0 until 30) { dut.clock.step(); assert(!dut.io.busy.peek().litToBoolean && !dut.io.req.valid.peek().litToBoolean, "AG kept going after abort") }
    }
  }

  it should "reject LOCAL_CACHE when row exceeds local buffer" in {
    test(new PatternAG(AGProfile.UB)) { dut =>
      dut.io.cfg.outerCnt.poke(2.U); dut.io.cfg.midCnt.poke(2.U); dut.io.cfg.innerCnt.poke(128.U)   // 32KiB row
      dut.io.cfg.compact.poke(false.B); dut.io.cfg.localCache.poke(true.B)
      dut.io.cfgStart.poke(true.B); dut.clock.step(); dut.io.cfgStart.poke(false.B); dut.clock.step()
      assert(dut.io.cfgErr.peek().litToBoolean && dut.io.done.peek().litToBoolean && !dut.io.req.valid.peek().litToBoolean)
    }
  }

  behavior of "PatternAG WB"
  Seq(
    "proposal spill (weight x Mt)" -> Cfg(8, 64, 128),
    "Kt=6 Nt=3"                    -> Cfg(2, 3, 6, pReady = 70),
    "Kt=20 backpressure"           -> Cfg(3, 5, 20, pReady = 50, ngDelayMax = 20),
  ).foreach { case (name, cfg) =>
    it should name in { test(new PatternAG(AGProfile.WB))(runCase(AGProfile.WB, cfg)) }
  }

  it should "omit strideOuter from the address (m replay)" in {
    val r = reference(AGProfile.WB, 2, 3, 6, lc = false, 8, 0, 99999, 1536, 256)
    // piece 경계는 window 위치에 따라 group 마다 달라지므로, 덮는 byte 범위의 합집합으로 비교
    def covered(o: Int) = r.filter(_.outer == o).flatMap(p => (p.addr until p.addr + p.bytes)).toSet
    assert(covered(0) == covered(1), "WB must re-supply the same weight bytes for every m")
    assert(covered(0).size == 3 * 6 * 256, "each m covers the whole weight exactly once")
  }
}