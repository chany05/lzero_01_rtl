import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import ocm._

// ===========================================================================
// Reference model. Written from the spec, not from the DUT: if the two
// disagree, the hardware is what is wrong.
//
// Everything is expressed in storage units. Since angle moved to 64B the unit
// IS the DMA beat: subWord == 1, and both a quant and an angle request return
// exactly one unit. Nothing below hardcodes that -- every loop is driven by
// p.quantUnits / p.angleUnits / p.subWord, so this model still holds if the
// granules diverge again.
// ===========================================================================
object PBRef {
  /** One storage unit, distinct for every (unit, seed) pair. */
  def unitVal(u: Int, p: PBParams, seed: Int): BigInt = {
    var acc = BigInt(0)
    for (i <- 0 until p.unitBytes) {
      val b = (u * 31 + i * 7 + seed) & 0xFF
      acc = acc | (BigInt(b) << (8 * i))
    }
    acc
  }

  /** Expected quant output at pointer `u`: units u .. u+quantUnits-1. */
  def quantVal(u: Int, p: PBParams, seed: Int): BigInt = {
    var acc = BigInt(0)
    for (j <- 0 until p.quantUnits) {
      acc = acc | (unitVal(u + j, p, seed) << (p.unitBits * j))
    }
    acc
  }

  /** Expected angle output at pointer `u`: units u .. u+angleUnits-1. */
  def angleVal(u: Int, p: PBParams, seed: Int): BigInt = {
    var acc = BigInt(0)
    for (j <- 0 until p.angleUnits) {
      acc = acc | (unitVal(u + j, p, seed) << (p.unitBits * j))
    }
    acc
  }

  /** Beat b carries units subWord*b .. subWord*b+subWord-1, low one first. */
  def beat(b: Int, p: PBParams, seed: Int): BigInt = {
    var acc = BigInt(0)
    for (j <- 0 until p.subWord) {
      acc = acc | (unitVal(p.subWord * b + j, p, seed) << (p.unitBits * j))
    }
    acc
  }

  def bankBeats(p: PBParams, seed: Int): Seq[BigInt] =
    (0 until p.beats).map(b => beat(b, p, seed))

  def check(tag: String, got: BigInt, exp: BigInt): Unit = {
    if (got != exp) {
      println(f"  MISMATCH $tag")
      println(s"    got 0x${got.toString(16)}")
      println(s"    exp 0x${exp.toString(16)}")
    }
    assert(got == exp, s"$tag mismatch")
  }
}

// ===========================================================================
// 1. Storage. Run this first -- everything else assumes it works.
// ===========================================================================
class PBMemTest extends AnyFlatSpec with ChiselScalatestTester {
  import PBRef._
  val p = PBParams()

  def fillBank(dut: PBMem, bank: Int, seed: Int): Unit = {
    dut.io.wen.poke(true.B)
    dut.io.wrBank.poke(bank.U)
    for (b <- 0 until p.beats) {
      dut.io.wrBeat.poke(b.U)
      dut.io.wdata.poke(beat(b, p, seed).U(p.writeBits.W))
      dut.clock.step(1)
    }
    dut.io.wen.poke(false.B)
  }

  def quiesce(dut: PBMem): Unit = {
    dut.io.wen.poke(false.B)
    dut.io.wrBank.poke(0.U)
    dut.io.wrBeat.poke(0.U)
    dut.io.rdBank.poke(0.U)
    dut.io.rdUnit.poke(0.U)
    dut.clock.step(1)
  }

  behavior of "PBMem"

  it should "return one storage unit on both outputs from a single read" in {
    test(new PBMem(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBMem : both outputs ===============")
      println(f"  unit = ${p.unitBytes}%dB, subWord = ${p.subWord}%d, " +
              f"quant = ${p.quantUnits}%d unit, angle = ${p.angleUnits}%d unit")

      quiesce(dut)
      fillBank(dut, 0, 0)

      for (u <- Seq(0, 1, 2, p.units - 1)) {
        dut.io.rdUnit.poke(u.U)
        val q = dut.io.quantData.peek().litValue
        val a = dut.io.angleData.peek().litValue
        println(f"  [unit $u%2d] quant ok = ${q == quantVal(u, p, 0)}, " +
                f"angle ok = ${a == angleVal(u, p, 0)}")
        check(s"quant @ $u", q, quantVal(u, p, 0))
        check(s"angle @ $u", a, angleVal(u, p, 0))
      }

      // With subWord == 1 the two outputs are driven by the same read, so a
      // divergence here means the qWord/aWord wiring has been changed.
      println("  (a quant/angle divergence at subWord==1 means qWord != aWord)")
      println("====================================================\n")
    }
  }

  it should "round-trip a full bank on both outputs" in {
    test(new PBMem(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBMem : full bank ===============")

      quiesce(dut)
      fillBank(dut, 0, 0)
      println(f"  wrote ${p.beats}%d beats (${p.bankBytes}%d byte, ${p.units}%d units)")

      for (u <- 0 until p.units) {
        dut.io.rdUnit.poke(u.U)
        if (u % p.angleUnits == 0) {
          check(s"angle @ $u", dut.io.angleData.peek().litValue, angleVal(u, p, 0))
        }
        if (u % p.quantUnits == 0) {
          check(s"quant @ $u", dut.io.quantData.peek().litValue, quantVal(u, p, 0))
        }
      }
      println(f"  all ${p.units}%d units match on both outputs")
      println("=================================================\n")
    }
  }

  it should "keep the two banks independent" in {
    test(new PBMem(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBMem : bank isolation ===============")

      quiesce(dut)
      for (bank <- 0 until 2) {
        fillBank(dut, bank, bank * 5 + 1)
        println(f"  filled bank $bank")
      }

      for (bank <- 0 until 2) {
        dut.io.rdBank.poke(bank.U)
        for (u <- Seq(0, 1, 2, p.units / 2, p.units - 1)) {
          dut.io.rdUnit.poke(u.U)
          check(s"bank $bank angle @ $u",
                dut.io.angleData.peek().litValue, angleVal(u, p, bank * 5 + 1))
        }
        println(f"  bank $bank verified")
      }
      println("======================================================\n")
    }
  }
}

// ===========================================================================
// 2. Read pointer.
//
// Both PB streams now instantiate with step == 1, so the first test collapses
// to one case. The `>=` end-of-region logic only shows up at step > 1, so that
// case is pinned explicitly below rather than derived from the params -- it is
// part of PtrCtrl's contract even though no PB stream currently uses it.
// ===========================================================================
class PtrCtrlTest extends AnyFlatSpec with ChiselScalatestTester {
  val p = PBParams()

  behavior of "PtrCtrl"

  it should "advance by its step for every stream in use" in {
    for (step <- Seq(p.quantUnits, p.angleUnits).distinct) {
      test(new PtrCtrl(p, step)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
        println(f"\n=============== PtrCtrl : step = $step ===============")

        val base = 8
        val reqs = 4
        val len  = reqs * step
        dut.io.base.poke(base.U)
        dut.io.len.poke(len.U)
        dut.io.req.poke(false.B)
        dut.io.clear.poke(true.B)
        dut.clock.step(1)
        dut.io.clear.poke(false.B)

        dut.io.req.poke(true.B)
        for (i <- 0 until reqs) {
          val unit = dut.io.unit.peek().litValue
          val left = dut.io.left.peek().litValue
          println(f"  [req $i] unit = $unit%2d  left = $left%2d")
          assert(unit == base + i * step, s"req $i: unit = $unit")
          assert(left == len - i * step, s"req $i: left = $left")
          assert(dut.io.fire.peek().litToBoolean, s"req $i: fire low")
          assert(!dut.io.done.peek().litToBoolean, s"req $i: done early")
          dut.clock.step(1)
        }
        assert(dut.io.done.peek().litToBoolean, "done not set after the region")
        assert(dut.io.left.peek().litValue == 0, "left not zero at done")
        println("=================================================\n")
      }
    }
  }

  it should "not advance past its region when requests keep coming" in {
    test(new PtrCtrl(p, p.quantUnits)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PtrCtrl : overrun guard ===============")

      val len  = 6
      val reqs = len / p.quantUnits           // NOT hardcoded: step is 1 now
      dut.io.base.poke(0.U)
      dut.io.len.poke(len.U)
      dut.io.req.poke(false.B)
      dut.io.clear.poke(true.B)
      dut.clock.step(1)
      dut.io.clear.poke(false.B)

      dut.io.req.poke(true.B)
      dut.clock.step(reqs)
      val atDone = dut.io.unit.peek().litValue
      assert(dut.io.done.peek().litToBoolean,
             s"done not set after $reqs requests -- check len/step arithmetic")

      dut.clock.step(10)
      val later = dut.io.unit.peek().litValue
      println(f"  len=$len, step=${p.quantUnits} -> $reqs requests")
      println(f"  unit at done = $atDone, after 10 more requests = $later")
      assert(later == atDone,
             "pointer advanced past its region -- it would read the other stream")
      println("======================================================\n")
    }
  }

  it should "stop at a region whose size is not a multiple of the step" in {
    // step is pinned to 2 on purpose. At the current params both PB streams
    // step by 1, where no region can be ragged -- but PtrCtrl is parameterised
    // and the `>=` comparison is still its contract, so it stays under test.
    val step = 2
    test(new PtrCtrl(p, step)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PtrCtrl : ragged region ===============")
      println(f"  (step=$step pinned by the test; no PB stream uses it today)")

      dut.io.base.poke(0.U)
      dut.io.len.poke(5.U)                    // step 2 cannot land exactly on 5
      dut.io.req.poke(false.B)
      dut.io.clear.poke(true.B)
      dut.clock.step(1)
      dut.io.clear.poke(false.B)

      dut.io.req.poke(true.B)
      var fires = 0
      for (_ <- 0 until 10) {
        if (dut.io.fire.peek().litToBoolean) fires += 1
        dut.clock.step(1)
      }
      println(f"  len=5, step=$step -> $fires fires (>= comparison, not ===)")
      assert(fires == 3, s"expected 3 fires, got $fires -- done never tripped?")
      assert(dut.io.done.peek().litToBoolean, "done not set on a ragged region")
      println("======================================================\n")
    }
  }

  it should "report done immediately for an unallocated stream" in {
    test(new PtrCtrl(p, p.angleUnits)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PtrCtrl : len = 0 ===============")

      dut.io.base.poke(0.U)
      dut.io.len.poke(0.U)
      dut.io.req.poke(true.B)
      dut.io.clear.poke(true.B)
      dut.clock.step(1)
      dut.io.clear.poke(false.B)

      for (t <- 0 until 5) {
        assert(dut.io.done.peek().litToBoolean,
               s"cycle $t: len=0 must report done, or the bank never flips")
        assert(!dut.io.fire.peek().litToBoolean, s"cycle $t: len=0 must never fire")
        dut.clock.step(1)
      }
      println("  done from cycle 0, never fires")
      println("=================================================\n")
    }
  }

  it should "restart at the base on clear" in {
    val step = p.angleUnits
    test(new PtrCtrl(p, step)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PtrCtrl : clear ===============")

      val base = 16
      val reqs = 5
      dut.io.base.poke(base.U)
      dut.io.len.poke(8.U)
      dut.io.req.poke(false.B)
      dut.io.clear.poke(true.B)
      dut.clock.step(1)
      dut.io.clear.poke(false.B)

      dut.io.req.poke(true.B)
      dut.clock.step(reqs)
      assert(dut.io.unit.peek().litValue == base + reqs * step)

      dut.io.clear.poke(true.B)
      dut.clock.step(1)
      dut.io.clear.poke(false.B)
      assert(dut.io.unit.peek().litValue == base, "clear did not reset to base")
      assert(!dut.io.done.peek().litToBoolean, "clear did not drop done")
      println("  reset to base, done cleared")
      println("===============================================\n")
    }
  }
}

// ===========================================================================
// 3. Controller.
//
// Layout used throughout: quant in the low part of the bank, angle above it.
// Requests are issued one stream at a time, which is the loader's stated
// behaviour and what the single read port requires.
//
// All region sizes are in storage units, which are now 64B -- half the count
// they were at 32B units for the same byte footprint. A bank holds
// p.units == 24 of them.
// ===========================================================================
class PBControllerTest extends AnyFlatSpec with ChiselScalatestTester {
  import PBRef._

  def init(dut: PBController, qBase: Int, qLen: Int, aBase: Int, aLen: Int): Unit = {
    dut.io.enable.poke(true.B)
    dut.io.shoot.poke(false.B)
    dut.io.reqQuant.poke(false.B)
    dut.io.reqAngle.poke(false.B)
    dut.io.dmaDataValid.poke(false.B)
    dut.io.dmaDataLast.poke(false.B)
    dut.io.dmaData.poke(0.U)
    dut.io.quantBase.poke(qBase.U)
    dut.io.quantLen.poke(qLen.U)
    dut.io.angleBase.poke(aBase.U)
    dut.io.angleLen.poke(aLen.U)
    dut.io.softReset.poke(true.B)
    dut.clock.step(1)
    dut.io.softReset.poke(false.B)
  }

  /** Wait for `hungry`, then push one bank of beats with `last` on the final one. */
  def serviceFill(dut: PBController, p: PBParams, seed: Int): Unit = {
    var guard = 0
    while (!dut.io.hungry.peek().litToBoolean) {
      dut.clock.step(1)
      guard += 1
      assert(guard < 500, "hungry never asserted")
    }
    val beats = bankBeats(p, seed)
    for ((b, i) <- beats.zipWithIndex) {
      dut.io.dmaDataValid.poke(true.B)
      dut.io.dmaDataLast.poke((i == beats.length - 1).B)
      dut.io.dmaData.poke(b.U(p.writeBits.W))
      dut.clock.step(1)
    }
    dut.io.dmaDataValid.poke(false.B)
    dut.io.dmaDataLast.poke(false.B)
    println(f"  filled a bank (seed $seed)")
  }

  def shootToWorking(dut: PBController): Unit = {
    var guard = 0
    dut.io.shoot.poke(true.B)
    while (!dut.io.working.peek().litToBoolean) {
      dut.clock.step(1)
      guard += 1
      assert(guard < 500, "never reached S_WORKING")
    }
    dut.io.shoot.poke(false.B)
  }

  behavior of "PBController"

  it should "walk IDLE -> PRELOAD -> READY -> WORKING" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : FSM bring-up ===============")
      println(f"  bank = ${p.units}%d units of ${p.unitBytes}%dB, " +
              f"${p.beats}%d DMA beats")

      init(dut, 0, 8, 8, 8)
      assert(dut.io.hungry.peek().litToBoolean, "PRELOAD did not request a fill")
      assert(!dut.io.ready.peek().litToBoolean, "ready high before the bank is filled")
      assert(!dut.io.working.peek().litToBoolean, "working high before shoot")
      println("  PRELOAD: hungry=1, ready=0")

      serviceFill(dut, p, 1)
      assert(dut.io.ready.peek().litToBoolean, "ready low after the ping bank filled")
      assert(!dut.io.working.peek().litToBoolean, "entered WORKING without shoot")
      println("  READY: ready=1, waiting on the barrier")

      shootToWorking(dut)
      println("  WORKING")
      println("==========================================================\n")
    }
  }

  it should "hold ready high while disabled so the barrier is not blocked" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : enable = 0 ===============")

      init(dut, 0, 8, 8, 8)
      dut.io.enable.poke(false.B)
      dut.clock.step(2)

      assert(dut.io.ready.peek().litToBoolean, "a disabled OCM must report ready")
      assert(!dut.io.hungry.peek().litToBoolean, "a disabled OCM must not fetch")
      assert(!dut.io.working.peek().litToBoolean)
      println("  ready=1, hungry=0, working=0")
      println("========================================================\n")
    }
  }

  it should "keep the two pointers independent when the streams interleave" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : interleaving ===============")

      val qBase = 0;  val qLen = 8      // 8 quant requests at step 1
      val aBase = 8;  val aLen = 8      // 8 angle requests
      init(dut, qBase, qLen, aBase, aLen)
      serviceFill(dut, p, 3)
      shootToWorking(dut)

      // quant alone -- pointer advances by quantUnits
      dut.io.reqQuant.poke(true.B)
      for (i <- 0 until 2) {
        val u = qBase + i * p.quantUnits
        check(s"quant $i", dut.io.quantData.peek().litValue, quantVal(u, p, 3))
        assert(dut.io.quantValid.peek().litToBoolean)
        assert(!dut.io.angleValid.peek().litToBoolean, "angle fired without a request")
        dut.clock.step(1)
      }
      dut.io.reqQuant.poke(false.B)
      println(f"  quant advanced 2 requests (${2 * p.quantUnits}%d units)")

      // angle alone -- quant must not have moved
      dut.io.reqAngle.poke(true.B)
      for (i <- 0 until 3) {
        val u = aBase + i * p.angleUnits
        check(s"angle $i", dut.io.angleData.peek().litValue, angleVal(u, p, 3))
        assert(dut.io.angleValid.peek().litToBoolean)
        assert(!dut.io.quantValid.peek().litToBoolean, "quant fired without a request")
        dut.clock.step(1)
      }
      dut.io.reqAngle.poke(false.B)
      println(f"  angle advanced 3 requests (${3 * p.angleUnits}%d units)")

      // back to quant -- it resumes where it stopped
      dut.io.reqQuant.poke(true.B)
      val resume = qBase + 2 * p.quantUnits
      check("quant resume", dut.io.quantData.peek().litValue, quantVal(resume, p, 3))
      dut.clock.step(1)
      dut.io.reqQuant.poke(false.B)
      println(f"  quant resumed at unit $resume, no drift from the angle traffic")
      println("==========================================================\n")
    }
  }

  it should "flip only once both streams are spent, with no idle cycle" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : joint exhaustion ===============")

      val qBase = 0; val qLen = 4       // 4 quant requests at step 1
      val aBase = 4; val aLen = 5       // 5 angle requests
      val qReqs = qLen / p.quantUnits
      val aReqs = aLen / p.angleUnits
      init(dut, qBase, qLen, aBase, aLen)
      serviceFill(dut, p, 7)            // ping
      shootToWorking(dut)
      serviceFill(dut, p, 9)            // pong, prefetched during WORKING

      // spend quant first
      dut.io.reqQuant.poke(true.B)
      for (i <- 0 until qReqs) {
        check(s"quant $i", dut.io.quantData.peek().litValue,
              quantVal(qBase + i * p.quantUnits, p, 7))
        dut.clock.step(1)
      }
      dut.io.reqQuant.poke(false.B)
      assert(dut.io.quantDone.peek().litToBoolean, "quant not done after its region")
      assert(!dut.io.angleDone.peek().litToBoolean, "angle done too early")
      println(f"  quant spent after $qReqs requests, angle still running -- no flip yet")

      // still on bank 0: had the flip fired on quant alone, this would be seed 9
      dut.io.reqAngle.poke(true.B)
      check("angle first", dut.io.angleData.peek().litValue, angleVal(aBase, p, 7))

      for (i <- 0 until aReqs) {
        check(s"angle $i", dut.io.angleData.peek().litValue,
              angleVal(aBase + i * p.angleUnits, p, 7))
        assert(!dut.io.quantValid.peek().litToBoolean, "quant fired after done")
        dut.clock.step(1)
      }
      println(f"  angle spent -- both spent, flip fires that cycle")

      // The flip lands on the cycle the final granule is delivered, so bank 1
      // is readable immediately. An idle cycle here would be a bubble.
      assert(!dut.io.quantDone.peek().litToBoolean, "done flags not cleared by the flip")
      assert(!dut.io.angleDone.peek().litToBoolean)
      assert(dut.io.angleValid.peek().litToBoolean, "bubble: no angle data after the flip")
      check("angle after flip", dut.io.angleData.peek().litValue, angleVal(aBase, p, 9))
      dut.io.reqAngle.poke(false.B)

      dut.io.reqQuant.poke(true.B)
      check("quant after flip", dut.io.quantData.peek().litValue, quantVal(qBase, p, 9))
      dut.io.reqQuant.poke(false.B)
      println("  pointers reset, other bank readable with no idle cycle")
      println("==============================================================\n")
    }
  }

  it should "request the pong bank as soon as it frees, not after the ping drains" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : hungry timing ===============")

      // 8 + 16 == 24 == p.units: the whole bank is allocated
      init(dut, 0, 8, 8, 16)
      serviceFill(dut, p, 2)
      shootToWorking(dut)

      // spec 5.1: with the ping bank full and the pong bank empty, the request
      // must already be out -- not deferred until quant_done & angle_done.
      assert(dut.io.hungry.peek().litToBoolean,
             "hungry low with an empty pong bank -- prefetch margin thrown away")
      println("  hungry=1 immediately in WORKING, before any granule is consumed")

      serviceFill(dut, p, 4)
      assert(!dut.io.hungry.peek().litToBoolean, "hungry still high with both banks full")
      println("  hungry=0 once both banks hold data")
      println("===========================================================\n")
    }
  }

  it should "raise impending only near exhaustion with no bank behind it" in {
    val p = PBParams(lowWater = 4)
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : impending ===============")
      println(f"  bank = ${p.units}%d units, lowWater = ${p.lowWater}%d units, " +
              f"angle step = ${p.angleUnits}%d")

      // quantLen is 0 here: this test drives angle only, and an allocated but
      // untouched quant region would trip the starvation assert.
      val aBase = 8; val aLen = 10
      val aReqs = aLen / p.angleUnits
      init(dut, 0, 0, aBase, aLen)
      assert(!dut.io.impending.peek().litToBoolean,
             "impending during PRELOAD -- the S_WORKING gate is missing")
      println("  PRELOAD: impending=0")

      serviceFill(dut, p, 5)
      shootToWorking(dut)

      dut.io.reqAngle.poke(true.B)
      var firstAt = -1
      for (i <- 0 until aReqs) {
        val imp  = dut.io.impending.peek().litToBoolean
        val left = dut.io.angleLeft.peek().litValue
        println(f"  [req $i%2d] angleLeft = $left%2d  impending = $imp")
        if (imp && firstAt < 0) firstAt = i
        dut.clock.step(1)
      }
      dut.io.reqAngle.poke(false.B)

      // `left` falls by angleUnits per request, so the first crossing is the
      // first i with (aLen - i*angleUnits) <= lowWater.
      val expectAt = (0 until aReqs).indexWhere(i => aLen - i * p.angleUnits <= p.lowWater)
      assert(firstAt == expectAt,
             s"impending first fired at request $firstAt, expected $expectAt")
      println(f"  first fired at request $firstAt (left <= ${p.lowWater})")
      println("=======================================================\n")
    }
  }

  it should "clear impending once the pong bank is filled" in {
    val p = PBParams(lowWater = 4)
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : impending clears ===============")

      val aLen = 10
      init(dut, 0, 0, 8, aLen)
      serviceFill(dut, p, 5)
      shootToWorking(dut)

      // step just past the low-water mark without exhausting the region
      val reqs = (0 until aLen).indexWhere(i => aLen - i * p.angleUnits <= p.lowWater) + 1
      dut.io.reqAngle.poke(true.B)
      dut.clock.step(reqs)
      assert(dut.io.impending.peek().litToBoolean, "impending low near exhaustion")
      println(f"  impending=1 after $reqs requests, pong bank empty")

      dut.io.reqAngle.poke(false.B)
      serviceFill(dut, p, 6)
      assert(!dut.io.impending.peek().litToBoolean,
             "impending still high after the pong bank filled")
      println("  impending=0 once the refill lands")
      println("==============================================================\n")
    }
  }

  it should "stream three banks back to back" in {
    val p = PBParams()
    test(new PBController(p)).withAnnotations(Seq(WriteVcdAnnotation)) { dut =>
      println("\n=============== PBController : streaming ===============")

      val qBase = 0; val qLen = 4
      val aBase = 4; val aLen = 6
      val qReqs = qLen / p.quantUnits
      val aReqs = aLen / p.angleUnits
      init(dut, qBase, qLen, aBase, aLen)

      val seeds = Seq(11, 22, 33)
      serviceFill(dut, p, seeds(0))
      shootToWorking(dut)
      serviceFill(dut, p, seeds(1))

      for ((seed, n) <- seeds.zipWithIndex) {
        // quant first, one stream at a time -- there is only one read port
        dut.io.reqQuant.poke(true.B)
        for (i <- 0 until qReqs) {
          assert(dut.io.quantValid.peek().litToBoolean, s"bank $n quant $i: no data")
          check(s"bank $n quant $i", dut.io.quantData.peek().litValue,
                quantVal(qBase + i * p.quantUnits, p, seed))
          dut.clock.step(1)
        }
        dut.io.reqQuant.poke(false.B)

        dut.io.reqAngle.poke(true.B)
        for (i <- 0 until aReqs) {
          assert(dut.io.angleValid.peek().litToBoolean, s"bank $n angle $i: no data")
          check(s"bank $n angle $i", dut.io.angleData.peek().litValue,
                angleVal(aBase + i * p.angleUnits, p, seed))
          dut.clock.step(1)
        }
        dut.io.reqAngle.poke(false.B)
        println(f"  bank $n (seed $seed) drained")

        // The last angle granule triggered the flip, so a bank has just freed
        // and `hungry` is up. Refilling any earlier would hang: with both
        // banks full there is nothing to request.
        if (n + 2 < seeds.length) serviceFill(dut, p, seeds(n + 2))
      }

      println("  three banks streamed, data intact across both flips")
      println("=======================================================\n")
    }
  }
}