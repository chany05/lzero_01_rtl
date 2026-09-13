package ocm

import chisel3._
import chiseltest._
import chiseltest.simulator.WriteVcdAnnotation
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable

// ===========================================================================
// Small parameters: a bank is 8 beats / 32 columns, a chunk is 4 beats.
// The structure under test is identical to the real 256-beat / 64-beat case.
// ===========================================================================
class UBSpec extends AnyFlatSpec with ChiselScalatestTester {

  val P = UBParams(
    rows         = 16,
    beatBytes    = 64,
    bankBytes    = 512,   // -> beats = 8, cols = 32
    banks        = 2,
    gemvRatio    = 4,
    impendCycles = 2,
    useXpm       = false
  )
  val CH   = 4                 // beats per DMA chunk
  val CCOL = CH * P.subWord    // columns per chunk = 16

  // ---- golden model ------------------------------------------------------
  // Data is a function of the GLOBAL column index, so bank alternation is
  // checked for free: the drain must emit columns 0,1,2,... in order.
  def cell(col: Int, row: Int): Int = (col * 7 + row * 13 + 1) & 0xff

  def mkBeat(g: Int): BigInt = {
    var v = BigInt(0)
    for (k <- 0 until P.subWord; lane <- 0 until P.rows) {
      val byteIdx = k * P.rows + lane
      v |= BigInt(cell(g * P.subWord + k, lane)) << (byteIdx * 8)
    }
    v
  }

  def expCol(col: Int): BigInt =
    (0 until P.rows).foldLeft(BigInt(0)) { (a, l) =>
      a | (BigInt(cell(col, l)) << (l * 8))
    }

  case class Beat(g: Int, chunkLast: Boolean = false, dramLast: Boolean = false)
  case class Sample(idx: Int, cyc: Int, data: BigInt, bankEnd: Boolean,
                    lastOut: Boolean, frag: Boolean, hungry: Boolean,
                    impending: Boolean)

  class H(c: UBController) {
    val q   = mutable.Queue[Beat]()
    val log = mutable.ArrayBuffer[Sample]()

    var enable    = false
    var shoot     = false
    var compact   = false
    var stallIn   = false
    var softReset = false
    var feed      = true
    var cyc       = 0

    def init(): Unit = {
      c.io.softReset.poke(false.B); c.io.enable.poke(false.B)
      c.io.shoot.poke(false.B);     c.io.compactMode.poke(false.B)
      c.io.stallIn.poke(false.B);   c.io.dmaDataValid.poke(false.B)
      c.io.dmaData.poke(0.U);       c.io.chunkLast.poke(false.B)
      c.io.dramDataLast.poke(false.B)
    }

    def tick(n: Int = 1): Unit = for (_ <- 0 until n) {
      c.io.softReset.poke(softReset.B); c.io.enable.poke(enable.B)
      c.io.shoot.poke(shoot.B);         c.io.compactMode.poke(compact.B)
      c.io.stallIn.poke(stallIn.B)

      val have = q.nonEmpty && feed
      c.io.dmaDataValid.poke(have.B)
      if (have) {
        val b = q.head
        c.io.dmaData.poke(mkBeat(b.g).U(P.writeBits.W))
        c.io.chunkLast.poke(b.chunkLast.B)
        c.io.dramDataLast.poke(b.dramLast.B)
      } else {
        c.io.chunkLast.poke(false.B); c.io.dramDataLast.poke(false.B)
      }
      val fired = have && c.io.wrReady.peek().litToBoolean

      c.clock.step(); cyc += 1
      if (fired) q.dequeue()

      if (c.io.dataValid.peek().litToBoolean) {
        log += Sample(log.size, cyc,
          c.io.ubDataOut.peek().litValue,
          c.io.bankEnd.peek().litToBoolean,
          c.io.lastDataOut.peek().litToBoolean,
          c.io.frag.peek().litToBoolean,
          c.io.hungry.peek().litToBoolean,
          c.io.impending.peek().litToBoolean)
      }
    }

    /** one DMA chunk: `n` beats with RLAST on the last */
    def chunk(gStart: Int, n: Int = CH, dramLast: Boolean = false): Seq[Beat] =
      (0 until n).map { i =>
        Beat(gStart + i, chunkLast = i == n - 1,
             dramLast = dramLast && i == n - 1)
      }

    def push(bs: Seq[Beat]): Unit = q ++= bs

    def waitReady(max: Int = 500): Unit = {
      var i = 0
      while (!c.io.ready.peek().litToBoolean && i < max) { tick(); i += 1 }
      assert(c.io.ready.peek().litToBoolean, "never reached S_READY")
    }

    def fire(): Unit = { shoot = true; tick(); shoot = false }

    def drain(nCols: Int, max: Int = 6000): Unit = {
      var i = 0
      while (log.size < nCols && i < max) { tick(); i += 1 }
      assert(log.size >= nCols, s"got ${log.size} of $nCols columns")
    }

    def checkCols(n: Int): Unit =
      for (i <- 0 until n)
        assert(log(i).data == expCol(i),
               f"col $i: got ${log(i).data}%x want ${expCol(i)}%x")
  }

  def run(body: (UBController, H) => Unit): Unit =
    test(new UBController(P)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      val h = new H(c); h.init(); body(c, h)
    }

  behavior of "UBController"

  // =====================================================================
  it should "map byte lanes and stream a long sequence in order" in {
    run { (c, h) =>
      h.enable = true
      h.push((0 until 12).flatMap(k => h.chunk(k * CH)))
      h.waitReady()
      h.fire()
      h.drain(40 * P.subWord)
      h.checkCols(40 * P.subWord)
    }
  }

  // =====================================================================
  // The flip is what cuts a chunk. Bank 0 runs out while an 8-beat chunk is
  // still streaming into bank 1, so the write pointer is dragged onto bank 0
  // and the remainder lands there. Bank 0 is then non-empty but incomplete,
  // and hungry must NOT drop.
  it should "tag the bank that catches a chunk cut by the flip" in {
    run { (c, h) =>
      h.enable = true
      h.push(h.chunk(0))                 // bank 0: one 4-beat chunk
      h.waitReady()
      h.fire()

      h.drain(CCOL - 2)                  // nearly finish bank 0
      h.push(h.chunk(CH, 8))             // long chunk starts into bank 1
      h.drain(CCOL)                      // flip happens a couple of beats in
      h.tick(1)

      assert(c.io.hungry.peek().litToBoolean,
             "hungry dropped on a bank holding only a cut tail")
      assert(c.io.wrReady.peek().litToBoolean, "should still accept data")
    }
  }

  // =====================================================================
  // The cut tail's own chunk_last does NOT satisfy the buffer: its front half
  // is already being read out of the other bank. hungry must stay up until a
  // NEW chunk starts arriving.
  it should "keep hungry raised past the cut tail's own chunk_last" in {
    run { (c, h) =>
      h.enable = true
      h.push(h.chunk(0))
      h.waitReady()
      h.fire()

      h.drain(CCOL - 2)
      h.push(h.chunk(CH, 8))
      h.drain(CCOL)
      h.tick(1)

      // freeze the read so the write bank stays put while we watch the fill
      h.stallIn = true
      h.tick(30)                          // the tail and its chunk_last arrive
      assert(c.io.hungry.peek().litToBoolean,
             "hungry dropped on the tail's own chunk_last")

      h.push(h.chunk(50, 2))              // a new chunk starts arriving
      h.tick(4)
      assert(!c.io.hungry.peek().litToBoolean,
             "still hungry after a new chunk started")
    }
  }

  // =====================================================================
  // Flipping into an empty bank would drag the write pointer ahead of the
  // stream and reorder it, so the drain parks at the frontier instead.
  it should "park at the frontier when the next bank is empty" in {
    run { (c, h) =>
      h.enable = true
      h.push(h.chunk(0))
      h.waitReady()
      h.fire()
      h.drain(CCOL)
      h.tick(60)
      assert(h.log.size == CCOL, s"read past the frontier: ${h.log.size}")
      assert(!h.log.exists(_.bankEnd), "flipped into an empty bank")
      assert(c.io.stall.peek().litToBoolean, "should be stalled")
    }
  }

  // =====================================================================
  it should "assert ubFull when the write bank fills up" in {
    run { (c, h) =>
      h.enable = true
      h.push(h.chunk(0))
      h.waitReady()                       // bank 0 loaded, write moves to bank 1
      h.push((1 until 4).flatMap(k => h.chunk(k * CH)))
      h.tick(40)
      assert(c.io.ubFull.peek().litToBoolean, "ubFull not raised")
      assert(!c.io.wrReady.peek().litToBoolean, "still accepting writes")
      assert(h.q.nonEmpty, "swallowed beats with no room")
    }
  }

  // =====================================================================
  it should "raise lastDataOut and go quiet at the tensor tail" in {
    run { (c, h) =>
      h.enable = true
      h.push(h.chunk(0))
      h.waitReady()
      h.fire()
      h.push(h.chunk(CH, 3, dramLast = true))

      val total = CCOL + 3 * P.subWord
      h.drain(total)
      h.tick(80)

      assert(h.log.size == total, s"read past the tail: ${h.log.size}")
      assert(h.log(total - 1).lastOut, "lastDataOut missing")
      assert(!h.log(CCOL - 1).lastOut, "lastDataOut on the wrong bank")
      assert(!c.io.hungry.peek().litToBoolean, "hungry must be suppressed")
      assert(c.io.streamEnd.peek().litToBoolean)
      h.checkCols(total)
    }
  }

  // =====================================================================
  it should "consume one column every gemvRatio cycles in compact mode" in {
    run { (c, h) =>
      h.enable = true; h.compact = true
      h.push((0 until 8).flatMap(k => h.chunk(k * CH)))
      h.waitReady(); h.fire()
      h.drain(P.cols)
      h.checkCols(P.cols)
      val deltas = h.log.map(_.cyc).sliding(2).map(w => w(1) - w(0)).toSeq
      assert(deltas.forall(_ == P.gemvRatio),
             s"compact cadence broken: ${deltas.distinct}")
    }
  }

  // =====================================================================
  // A stall must freeze the prescaler too, or compact-mode phase shifts and
  // the array gets a column on the wrong cycle.
  it should "keep column order across a stall in compact mode" in {
    run { (c, h) =>
      h.enable = true; h.compact = true
      h.push((0 until 8).flatMap(k => h.chunk(k * CH)))
      h.waitReady(); h.fire()

      h.drain(6)
      h.stallIn = true
      val frozen = h.log.size
      h.tick(11)
      assert(h.log.size == frozen, "read fired while stalled")
      h.stallIn = false

      h.drain(P.cols)
      h.checkCols(P.cols)
    }
  }

  // =====================================================================
  it should "not raise impending when the next bank already holds data" in {
    run { (c, h) =>
      h.enable = true
      h.push((0 until 8).flatMap(k => h.chunk(k * CH)))
      h.waitReady()
      h.tick(40)
      h.fire()
      h.drain(CCOL)
      assert(!h.log.take(CCOL).exists(_.impending),
             "false impending with relief loaded")
    }
  }

  // =====================================================================
  it should "clear all pointers on softReset" in {
    run { (c, h) =>
      h.enable = true
      h.push((0 until 4).flatMap(k => h.chunk(k * CH)))
      h.waitReady(); h.fire()
      h.drain(8)

      h.softReset = true; h.tick(2); h.softReset = false
      h.log.clear(); h.q.clear()
      h.tick(10)
      assert(!c.io.ready.peek().litToBoolean, "still ready after reset")
      assert(h.log.isEmpty, "data streamed after reset")

      h.push((0 until 4).flatMap(k => h.chunk(k * CH)))
      h.waitReady(); h.fire()
      h.drain(P.cols)
      h.checkCols(P.cols)
    }
  }
}