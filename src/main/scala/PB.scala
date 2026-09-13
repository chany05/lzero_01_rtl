package ocm

import chisel3._
import chisel3.util._

// ===========================================================================
// Parameters
//
// The two streams read at DIFFERENT widths: quant takes 64B per request,
// angle takes 32B. Storage is therefore organised in units of the greatest
// common divisor -- 32B -- and quant simply consumes two of them per request.
//
// Bank size is 1.5KB, not the 4KB the common OCM spec assumes. That means the
// DMA burst for PB is 24 beats rather than 64, so `dma_data_last` must arrive
// on beat 24. Worth confirming with the DMA owner: PB is the one OCM whose
// transfer is not a 4KB AXI burst.
//
// NOTE ON STORAGE: `Mem` infers distributed RAM (LUTRAM). 3KB with a single
// read port is roughly 750 LUTs. The single port is only possible because the
// loader never requests quant and angle on the same cycle; a second port
// would make Vivado replicate the whole array.
// ===========================================================================
case class PBParams(
  bankBytes:  Int     = 1536,
  beatBytes:  Int     = 64,
  unitBytes:  Int     = 64,   // was 32 -- gcd(64,64)
  quantBytes: Int     = 64,
  angleBytes: Int     = 64,   // was 32
  lowWater:   Int     = 8,
  regOut:     Boolean = false
) {
  require(beatBytes  % unitBytes == 0, "the DMA beat must be a whole number of units")
  require(quantBytes % unitBytes == 0, "quant granule must be a whole number of units")
  require(angleBytes % unitBytes == 0, "angle granule must be a whole number of units")
  require(bankBytes  % beatBytes == 0, "a bank must be a whole number of DMA beats")

  val units      = bankBytes / unitBytes      // 48 storage units per bank
  val beats      = bankBytes / beatBytes      // 24 DMA beats per bank
  val subWord    = beatBytes / unitBytes      // 2 units per beat
  val quantUnits = quantBytes / unitBytes     // 2 units per quant request
  val angleUnits = angleBytes / unitBytes     // 1 unit per angle request

  require(lowWater < units,
          s"lowWater=$lowWater must be under $units, or `impending` never clears")

  val unitW      = log2Ceil(units)            // 6
  val beatW      = log2Ceil(beats)            // 5
  val subSel     = log2Ceil(subWord)          // 1
  val cntW       = unitW + 1                  // 7, must hold `units` itself

  // The bank bit sits above the beat index, so the address space is
  // 2^beatW * 2 -- not beats * 2. With `beats` a power of two those are the
  // same; at 24 beats they are not, and sizing to 48 would put bank 1's upper
  // entries out of range.
  val memAddrW   = beatW + 1                  // 6
  val memDepth   = 1 << memAddrW              // 64 entries, 48 of them used
  val unitBits   = unitBytes  * 8             // 256
  val quantBits  = quantBytes * 8             // 512
  val angleBits  = angleBytes * 8             // 256
  val writeBits  = beatBytes  * 8             // 512
}

// ===========================================================================
// Storage.
//
// Split into `subWord` sub-memories of one storage unit each. A DMA beat
// writes all of them on the same cycle, so the write path is pure wiring.
//
// ONE read port serves both streams. The port always reads a full beat's
// worth -- every sub-memory at one entry -- and the two widths fall out of
// what is taken from it:
//
//   quant (64B) : the whole read
//   angle (32B) : one sub-memory, picked by the pointer's low bits
//
// This works because the loader never requests both on the same cycle. A
// second port would mean Vivado replicating the LUTRAM -- roughly double the
// LUT cost for a concurrency that does not exist.
// ===========================================================================
class PBMem(p: PBParams) extends Module {
  val io = IO(new Bundle {
    val wen    = Input(Bool())
    val wrBank = Input(UInt(1.W))
    val wrBeat = Input(UInt(p.beatW.W))
    val wdata  = Input(UInt(p.writeBits.W))

    val rdBank    = Input(UInt(1.W))
    val rdUnit    = Input(UInt(p.unitW.W))    // whichever stream is firing
    val quantData = Output(UInt(p.quantBits.W))
    val angleData = Output(UInt(p.angleBits.W))
  })

  val waddr = Cat(io.wrBank, io.wrBeat)

  val mems = Seq.tabulate(p.subWord) { j =>
    val m = Mem(p.memDepth, UInt(p.unitBits.W))
    when(io.wen) {
      m.write(waddr, io.wdata(p.unitBits * (j + 1) - 1, p.unitBits * j))
    }
    m
  }

  // rdUnit[unitW-1:subSel] is the beat index. Sliced explicitly to beatW bits
  // so the read address is the same width as the write address.
  val entry = Cat(io.rdBank, io.rdUnit(p.subSel + p.beatW - 1, p.subSel))
  val words = mems.map(_.read(entry))

  val qWord = Cat(words.reverse)              // head = MSB, so word 0 is low
  val aWord =
    if (p.subWord == 1) words.head
    else VecInit(words)(io.rdUnit(p.subSel - 1, 0))

  io.quantData := (if (p.regOut) RegNext(qWord) else qWord)
  io.angleData := (if (p.regOut) RegNext(aWord) else aWord)

  require(p.quantUnits == p.subWord,
          "quant granule must equal the DMA beat, or qWord needs a wider read")
}

// ===========================================================================
// One read stream.
//
// `step` is how many storage units one request consumes: 2 for quant, 1 for
// angle. Everything downstream counts in units, so the two streams share this
// module despite reading at different widths.
// ===========================================================================
class PtrCtrl(p: PBParams, step: Int) extends Module {
  val io = IO(new Bundle {
    val base  = Input(UInt(p.unitW.W))    // start of this region, in units
    val len   = Input(UInt(p.cntW.W))     // size of this region, in units
    val req   = Input(Bool())
    val clear = Input(Bool())             // soft_reset or bank flip

    val unit  = Output(UInt(p.unitW.W))
    val fire  = Output(Bool())
    val done  = Output(Bool())
    val left  = Output(UInt(p.cntW.W))    // units remaining
  })

  val cnt  = RegInit(0.U(p.cntW.W))
  val done = RegInit(false.B)

  // A stream the compiler did not allocate (len == 0) must report done from
  // the start; otherwise `cnt + step >= len` never trips, the joint flip
  // never fires, and PB sits on one bank forever.
  val unused = io.len === 0.U

  io.fire := io.req && !done && !unused

  when(io.clear) {
    cnt  := 0.U
    done := false.B
  }.elsewhen(io.fire) {
    cnt := cnt + step.U
    // >= rather than ===: a region whose size is not a multiple of `step`
    // would otherwise step straight over the end.
    when(cnt + step.U >= io.len) { done := true.B }
  }

  io.unit := io.base + cnt
  io.done := done || unused
  io.left := Mux(unused, 0.U, io.len - cnt)
}

// ===========================================================================
// PB Controller.
//
// Common OCM interface (spec section 2) plus the PB-specific dual-pointer
// read path (section 3.4).
//
// Two places where this follows section 5 over section 3.4:
//
//   `hungry` -- 3.4 gives (quant_done & angle_done) && pong_empty, which only
//   requests AFTER the bank is spent, throwing away the whole drain as
//   prefetch margin. Section 5.1 requests the moment the pong bank frees.
//
//   `impending` -- 5.2 gates on S_WORKING; 3.4 omits it. Without the gate the
//   signal fires during preload, when the bank is empty by definition.
// ===========================================================================
class PBController(p: PBParams = PBParams()) extends Module {
  val io = IO(new Bundle {
    // ---- common control (from Compute Initializer Unit) ----
    val enable    = Input(Bool())
    val softReset = Input(Bool())
    val shoot     = Input(Bool())

    // ---- static config (from npu_struct), in storage units ----
    val quantBase = Input(UInt(p.unitW.W))
    val quantLen  = Input(UInt(p.cntW.W))
    val angleBase = Input(UInt(p.unitW.W))
    val angleLen  = Input(UInt(p.cntW.W))

    // ---- DMA read ----
    val dmaDataValid = Input(Bool())
    val dmaDataLast  = Input(Bool())
    val dmaData      = Input(UInt(p.writeBits.W))

    // ---- parameter loader ----
    val reqQuant   = Input(Bool())
    val reqAngle   = Input(Bool())
    val quantData  = Output(UInt(p.quantBits.W))   // 64B
    val angleData  = Output(UInt(p.angleBits.W))   // 32B
    val quantValid = Output(Bool())
    val angleValid = Output(Bool())
    val quantDone  = Output(Bool())
    val angleDone  = Output(Bool())
    val quantLeft  = Output(UInt(p.cntW.W))        // units left, for STATUS
    val angleLeft  = Output(UInt(p.cntW.W))

    // ---- common status ----
    val ready     = Output(Bool())
    val hungry    = Output(Bool())
    val impending = Output(Bool())
    val working   = Output(Bool())
  })

  val mem   = Module(new PBMem(p))
  val quant = Module(new PtrCtrl(p, p.quantUnits))
  val angle = Module(new PtrCtrl(p, p.angleUnits))

  // ---- bank state --------------------------------------------------------
  val valid  = RegInit(VecInit(Seq.fill(2)(false.B)))
  val rdBank = RegInit(0.U(1.W))

  // Fill the read bank while it is still empty (preload), the other one after
  // that. Latched at the start of a burst so a mid-transfer flip cannot move
  // the destination.
  val fillTarget = Mux(!valid(rdBank), rdBank, ~rdBank)
  val fillBank   = RegInit(0.U(1.W))
  val fillBusy   = RegInit(false.B)
  val pongEmpty  = !valid(fillTarget)

  // ---- FSM ---------------------------------------------------------------
  val sIdle :: sPreload :: sReady :: sWorking :: sStall :: Nil = Enum(5)
  val state = RegInit(sIdle)

  val rdValid = valid(rdBank)

  // Exhaustion is detected on the cycle the final granule is DELIVERED, not
  // one cycle later when `done` has registered. Waiting for `done` costs an
  // idle cycle at every flip: the pointer is spent, so `fire` is low, but the
  // bank has not swapped yet.
  //
  // A stream that finished earlier, or was never allocated, reports `done`
  // and satisfies its half of the AND on its own.
  val quantSpent = quant.io.done ||
                   (quant.io.fire && quant.io.left <= p.quantUnits.U)
  val angleSpent = angle.io.done ||
                   (angle.io.fire && angle.io.left <= p.angleUnits.U)
  val exhausted  = quantSpent && angleSpent

  switch(state) {
    is(sIdle) {
      when(io.enable) { state := sPreload }
    }
    is(sPreload) {
      when(!io.enable)     { state := sIdle }
        .elsewhen(rdValid) { state := sReady }
    }
    is(sReady) {
      when(!io.enable)      { state := sIdle }
        .elsewhen(io.shoot) { state := sWorking }
    }
    is(sWorking) {
      when(!io.enable)      { state := sIdle }
        .elsewhen(!rdValid) { state := sStall }   // underflow
    }
    is(sStall) {
      when(!io.enable)     { state := sIdle }
        .elsewhen(rdValid) { state := sWorking }
    }
  }
  when(io.softReset) { state := Mux(io.enable, sPreload, sIdle) }

  // ---- write path (data-driven / push) ------------------------------------
  val wrBeat = RegInit(0.U(p.beatW.W))

  when(io.dmaDataValid && !fillBusy) {
    fillBank := fillTarget          // latch destination on the first beat
    fillBusy := true.B
  }
  when(io.dmaDataValid) {
    wrBeat := Mux(io.dmaDataLast, 0.U, wrBeat + 1.U)
  }

  val fillDone = io.dmaDataValid && io.dmaDataLast
  when(fillDone) {
    fillBusy := false.B
    valid(Mux(fillBusy, fillBank, fillTarget)) := true.B
  }

  // `last` replaces the completion counter, but wrBeat still supplies the
  // write address -- so the two must agree. PB's burst is 24 beats, not the
  // 64 the other OCMs use.
  assert(!fillDone || wrBeat === (p.beats - 1).U,
         "PB: dma_data_last arrived at the wrong beat")

  mem.io.wen    := io.dmaDataValid
  mem.io.wrBank := Mux(fillBusy, fillBank, fillTarget)
  mem.io.wrBeat := wrBeat
  mem.io.wdata  := io.dmaData

  // ---- two independent read streams ---------------------------------------
  // Requests only count in S_WORKING with a filled bank; otherwise a pointer
  // would advance past parameters that were never written.
  val canRead = (state === sWorking) && rdValid
  val flip    = (state === sWorking) && exhausted

  quant.io.base  := io.quantBase
  quant.io.len   := io.quantLen
  quant.io.req   := io.reqQuant && canRead
  quant.io.clear := io.softReset || flip

  angle.io.base  := io.angleBase
  angle.io.len   := io.angleLen
  angle.io.req   := io.reqAngle && canRead
  angle.io.clear := io.softReset || flip

  // A single read port, driven by whichever stream is firing. quant wins the
  // tie only to keep the address deterministic -- the assert below is the
  // real contract, since the loader is specified never to request both.
  mem.io.rdBank := rdBank
  mem.io.rdUnit := Mux(quant.io.fire, quant.io.unit, angle.io.unit)

  assert(!(quant.io.fire && angle.io.fire),
         "PB: quant and angle requested on the same cycle -- there is only one read port")

  io.quantData  := mem.io.quantData
  io.angleData  := mem.io.angleData
  io.quantValid := (if (p.regOut) RegNext(quant.io.fire, false.B) else quant.io.fire)
  io.angleValid := (if (p.regOut) RegNext(angle.io.fire, false.B) else angle.io.fire)
  io.quantDone  := quant.io.done
  io.angleDone  := angle.io.done
  io.quantLeft  := quant.io.left
  io.angleLeft  := angle.io.left

  // ---- bank flip (spec 3.4: both streams spent) ---------------------------
  when(flip) {
    valid(rdBank) := false.B
    rdBank        := rdBank + 1.U
  }
  when(io.softReset) {
    rdBank   := 0.U
    valid    := VecInit(Seq.fill(2)(false.B))
    wrBeat   := 0.U
    fillBusy := false.B
  }

  assert(!(fillDone && flip && fillBank === rdBank),
         "PB: fill and flip collided on the same bank")

  // ---- common outputs -----------------------------------------------------
  io.ready   := rdValid || !io.enable                      // spec 2.2
  io.working := state === sWorking                         // spec 5.3

  // Spec 5.1. `fillBusy` suppresses a repeat once beats are flowing; the
  // arbiter is expected to latch the request during the DRAM latency window,
  // since `hungry` stays high until data actually arrives.
  io.hungry := ((state === sWorking) || (state === sPreload)) &&
               pongEmpty && !fillBusy

  // Spec 5.2 with the PB-specific OR of the two streams (spec 3.4). An
  // unallocated stream reports left == 0, so it is excluded here or
  // `impending` would sit high for the whole drain.
  val quantLow = (io.quantLen =/= 0.U) && (quant.io.left <= p.lowWater.U)
  val angleLow = (io.angleLen =/= 0.U) && (angle.io.left <= p.lowWater.U)
  io.impending := (state === sWorking) && (quantLow || angleLow) && pongEmpty

  assert(!(io.enable && io.quantLen === 0.U && io.angleLen === 0.U),
         "PB: both regions are zero-length -- the bank flips every cycle")

  // The quant read spans every sub-memory at one entry, which is only the
  // right data when the pointer sits on a beat boundary.
  assert(!quant.io.fire || (quant.io.unit % p.quantUnits.U) === 0.U,
         "PB: quant pointer is not aligned to its granule -- check quant_base")

  // Starvation check. If one region is spent while the other still has work,
  // its VPU gets nothing. Should that VPU raise a stall, the pipeline freezes,
  // the other stream stops requesting, its `done` never arrives, the flip
  // never fires, and the spent region is never refilled -- a deadlock.
  //
  // The cure is a correct quant_len : angle_len ratio, which is the
  // compiler's job. This turns a silent hang into a simulation failure.
  //
  // An unallocated region (len == 0) is excluded: it reports `done` from the
  // start by design and is never requested.
  val bothAllocated = (io.quantLen =/= 0.U) && (io.angleLen =/= 0.U)
  assert(!(bothAllocated && (state === sWorking) &&
           io.reqQuant && quant.io.done && !angle.io.done),
         "PB: quant region spent while angle still has work -- " +
         "quant_len : angle_len does not match the consumption ratio")
  assert(!(bothAllocated && (state === sWorking) &&
           io.reqAngle && angle.io.done && !quant.io.done),
         "PB: angle region spent while quant still has work -- " +
         "quant_len : angle_len does not match the consumption ratio")
}