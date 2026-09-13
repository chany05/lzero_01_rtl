package ocm

import chisel3._
import chisel3.util._

// ===========================================================================
// UB (Unified Buffer) Controller
//
// Spec mapping:
//   32 KB total = 16 KB Ping + 16 KB Pong, 16 x BRAM18
//   Write 64 B/cycle from DMA, read 16 B/cycle to the systolic array
//   Read-driven ping-pong flip, compact-mode 16x slow-down
//
// The 16 BRAM count comes from the READ port, not from capacity: the array
// needs 16 bytes in the same cycle and each BRAM lane owns one matrix row,
// so 16 lanes x 8b read ports. Depth is free -- 512 x 32b per lane fills a
// RAMB18E2 exactly.
//
// STRICT PING-PONG, READ-DRIVEN. The write pointer is always on the bank the
// read pointer is not on. RLAST marks where a chunk ends but does not close a
// bank, so the DMA can keep appending chunks into the leftover room on any
// spare cycle and the full 16 KB gets used.
//
// The flip is what cuts a chunk in half: finishing bank A drags the write
// pointer from B onto A, truncating whatever was streaming into B, and the
// remainder lands in A. `frag` marks that bank so `hungry` is not fooled into
// dropping by a bank that merely looks non-empty.
// ===========================================================================
case class UBParams(
  rows:         Int     = 16,     // systolic rows == BRAM count == byte lanes
  beatBytes:    Int     = 64,     // DMA write granule (512b)
  bankBytes:    Int     = 16384,  // one bank (Ping or Pong)
  banks:        Int     = 2,      // 2 = Ping/Pong
  gemvRatio:    Int     = 16,     // compact-mode slow-down factor
  impendCycles: Int     = 8,      // underflow warning lookahead
  useXpm:       Boolean = false   // true: XPM blackbox, false: sim model
) {
  require(beatBytes % rows == 0)
  require(bankBytes % beatBytes == 0)
  require(banks >= 2 && isPow2(banks))

  val vecBytes  = rows                     // 16 B read granule == one column
  val subWord   = beatBytes / vecBytes     // 4 columns per beat
  val wordBits  = subWord * 8              // 32b BRAM write word
  val cols      = bankBytes / vecBytes     // 1024 columns per bank
  val beats     = bankBytes / beatBytes    // 256 beats per bank

  val colW      = log2Ceil(cols)           // 10
  val beatW     = log2Ceil(beats)          // 8
  val subSel    = log2Ceil(subWord)        // 2
  val prescW    = log2Ceil(gemvRatio)      // 4
  val bankW     = log2Ceil(banks)          // 1
  val endW      = beatW + 1                // 9, must hold `beats` itself

  val wrAddrW   = bankW + beatW            // 9  {bank, beat}
  val rdAddrW   = bankW + colW             // 11 {bank, col}
  // The bank index sits in the address MSBs, so depth is 2^(bankW+beatW).
  val wordDepth = 1 << (bankW + beatW)     // 512
  val readBits  = vecBytes * 8             // 128
  val writeBits = beatBytes * 8            // 512
}

// ===========================================================================
// Byte-lane extraction.
//
// A 64 B beat holds 4 consecutive columns; lane L owns matrix row L, so it
// takes bytes L, L+16, L+32, L+48 and packs them into one 32b word.
//
// Column 4b+0 MUST land in word bits [7:0]: Xilinx asymmetric SDP maps the
// narrow port's low address bits starting from the LSB. Pure wiring, 0 LUTs.
// ===========================================================================
object LaneWord {
  def apply(beat: UInt, lane: Int, p: UBParams): UInt = {
    val bytes = Seq.tabulate(p.subWord) { k =>
      val b = k * p.rows + lane
      beat(b * 8 + 7, b * 8)
    }
    Cat(bytes.reverse)   // Cat head = MSB, so k=0 ends up at [7:0]
  }
}

// ===========================================================================
// xpm_memory_sdpram, asymmetric: write 32b x 512, read 8b x 2048.
// CASCADE_HEIGHT 1 keeps READ_LATENCY_B at 1. If this ever spills past one
// primitive the latency becomes 2 and the CU schedule must change with it.
// ===========================================================================
class XpmSdpRam(p: UBParams) extends BlackBox(Map(
  "MEMORY_SIZE"        -> p.wordDepth * p.wordBits,
  "MEMORY_PRIMITIVE"   -> "block",
  "CASCADE_HEIGHT"     -> 1,
  "CLOCKING_MODE"      -> "common_clock",
  "ECC_MODE"           -> "no_ecc",
  "USE_MEM_INIT"       -> 0,
  "WRITE_DATA_WIDTH_A" -> p.wordBits,
  "BYTE_WRITE_WIDTH_A" -> p.wordBits,
  "ADDR_WIDTH_A"       -> p.wrAddrW,
  "READ_DATA_WIDTH_B"  -> 8,
  "ADDR_WIDTH_B"       -> p.rdAddrW,
  "READ_LATENCY_B"     -> 1,
  "WRITE_MODE_B"       -> "read_first"
)) {
  override def desiredName = "xpm_memory_sdpram"

  val io = IO(new Bundle {
    val sleep          = Input(Bool())
    val clka           = Input(Clock())
    val ena            = Input(Bool())
    val wea            = Input(UInt(1.W))
    val addra          = Input(UInt(p.wrAddrW.W))
    val dina           = Input(UInt(p.wordBits.W))
    val injectsbiterra = Input(Bool())
    val injectdbiterra = Input(Bool())
    val clkb           = Input(Clock())
    val rstb           = Input(Bool())
    val enb            = Input(Bool())
    val regceb         = Input(Bool())
    val addrb          = Input(UInt(p.rdAddrW.W))
    val doutb          = Output(UInt(8.W))
    val sbiterrb       = Output(Bool())
    val dbiterrb       = Output(Bool())
  })
}

// ===========================================================================
// One lane = one BRAM = one matrix row. Read latency is 1 in both paths.
// ===========================================================================
class UBLane(p: UBParams) extends Module {
  val io = IO(new Bundle {
    val wen   = Input(Bool())
    val waddr = Input(UInt(p.wrAddrW.W))
    val wdata = Input(UInt(p.wordBits.W))
    val ren   = Input(Bool())
    val raddr = Input(UInt(p.rdAddrW.W))
    val rdata = Output(UInt(8.W))
  })

  if (p.useXpm) {
    val ram = Module(new XpmSdpRam(p))
    ram.io.sleep          := false.B
    ram.io.injectsbiterra := false.B
    ram.io.injectdbiterra := false.B
    ram.io.regceb         := true.B
    ram.io.clka           := clock
    ram.io.clkb           := clock
    ram.io.rstb           := reset
    ram.io.ena            := io.wen
    ram.io.wea            := io.wen
    ram.io.addra          := io.waddr
    ram.io.dina           := io.wdata
    ram.io.enb            := io.ren
    ram.io.addrb          := io.raddr
    io.rdata              := ram.io.doutb
  } else {
    // Sub-word select MUST match the hardware: index 0 -> bits [7:0].
    val mem  = SyncReadMem(p.wordDepth, UInt(p.wordBits.W))
    when(io.wen) { mem.write(io.waddr, io.wdata) }
    val word = mem.read(io.raddr >> p.subSel, io.ren)
    val sel  = RegNext(io.raddr(p.subSel - 1, 0))
    io.rdata := VecInit(Seq.tabulate(p.subWord)(i => word(i * 8 + 7, i * 8)))(sel)
  }
}

// ===========================================================================
// Memory array. The bank index is folded into the address MSB: the banks
// share all 16 BRAMs by depth, not by splitting them 8 + 8.
// ===========================================================================
class UBMem(p: UBParams) extends Module {
  val io = IO(new Bundle {
    val wen    = Input(Bool())
    val wrBank = Input(UInt(p.bankW.W))
    val wrBeat = Input(UInt(p.beatW.W))
    val wdata  = Input(UInt(p.writeBits.W))

    val ren    = Input(Bool())
    val rdBank = Input(UInt(p.bankW.W))
    val rdCol  = Input(UInt(p.colW.W))
    val rdata  = Output(UInt(p.readBits.W))   // valid 1 cycle after ren
  })

  val outs = Seq.tabulate(p.rows) { lane =>
    val l = Module(new UBLane(p))
    l.io.wen   := io.wen
    l.io.waddr := Cat(io.wrBank, io.wrBeat)
    l.io.wdata := LaneWord(io.wdata, lane, p)
    l.io.ren   := io.ren
    l.io.raddr := Cat(io.rdBank, io.rdCol)
    l.io.rdata
  }

  // Cat head = MSB, so reverse puts lane 0 at rdata[7:0] == systolic row 0.
  io.rdata := Cat(outs.reverse)
}

// ===========================================================================
// Address generator: column counter + compact-mode prescaler.
//
// `en` gates the prescaler as well as the counter. If a stall froze the
// column but let the prescaler run, compact-mode phase would shift and the
// array would receive a column on the wrong cycle.
//
// `lastCol` is an input because a fractional tail bank ends before cols-1.
// ===========================================================================
class AddrGen(p: UBParams) extends Module {
  val io = IO(new Bundle {
    val compact = Input(Bool())
    val en      = Input(Bool())
    val clear   = Input(Bool())
    val lastCol = Input(UInt(p.colW.W))
    val col     = Output(UInt(p.colW.W))
    val fire    = Output(Bool())   // issue a BRAM read this cycle
    val last    = Output(Bool())   // last column of this bank
  })

  val col   = RegInit(0.U(p.colW.W))
  val presc = RegInit(0.U(p.prescW.W))
  val tick  = Mux(io.compact, presc === (p.gemvRatio - 1).U, true.B)
  val atEnd = col === io.lastCol

  when(io.clear) {
    col   := 0.U
    presc := 0.U
  }.elsewhen(io.en) {
    presc := Mux(io.compact, Mux(tick, 0.U, presc + 1.U), 0.U)
    when(tick) { col := Mux(atEnd, 0.U, col + 1.U) }
  }

  io.col  := col
  io.fire := io.en && tick
  io.last := io.en && tick && atEnd
}

// ===========================================================================
// Bank bookkeeping.
//
// STRICT PING-PONG. The write pointer is always the bank the read pointer is
// NOT on, so it is derived, not stored:
//
//     wr_bank = (state == PRELOAD) ? rd_bank : ~rd_bank
//
// Per bank:
//   wrCount(b)  write frontier, in beats. Also the write address, so nothing
//               has to be saved when a transfer pauses.
//   fragR(b)    this bank was handed the tail of a chunk that a flip cut in
//               half, and no NEW chunk has started arriving since. The tail's
//               own chunk_last does NOT clear it: that beat only completes
//               the fragment, whose front half is already being read out of
//               the other bank, so the buffer really does still need data.
//               It clears on the first beat of the next chunk instead, which
//               `midChunk` already identifies -- no extra state needed.
//   lastTagR(b) dram_data_last landed here: this bank holds the tensor tail.
//
// A flip is what creates a fragment. When the drain finishes bank A, the
// write pointer is dragged from B onto A, so whatever chunk was streaming
// into B is truncated and its remainder lands in A. `midChunk` remembers
// whether a chunk was in flight at that instant.
//
// A flip is only allowed when the next bank already holds data, otherwise
// the write pointer would jump ahead of the stream and reorder it. With an
// empty next bank the drain parks at the frontier and waits.
// ===========================================================================
class BankCtrl(p: UBParams) extends Module {
  val n = p.banks

  val io = IO(new Bundle {
    val clear   = Input(Bool())
    val enable  = Input(Bool())
    val preload = Input(Bool())        // S_IDLE or S_PRELOAD

    // ---- DMA side ----
    val dmaFire      = Input(Bool())
    val chunkLast    = Input(Bool())   // RLAST: end of a chunk, NOT of a bank
    val dramDataLast = Input(Bool())
    val wrReady      = Output(Bool())
    val ubFull       = Output(Bool())  // no room at all right now
    val wrBank       = Output(UInt(p.bankW.W))
    val wrBeat       = Output(UInt(p.beatW.W))

    // ---- drain side ----
    val col       = Input(UInt(p.colW.W))
    val drainLast = Input(Bool())

    // ---- status ----
    val rdBank     = Output(UInt(p.bankW.W))
    val dataReady  = Output(Bool())
    val lastCol    = Output(UInt(p.colW.W))
    val preloadOk  = Output(Bool())
    val needData   = Output(Bool())    // write bank empty, or holds a fragment
    val relief     = Output(Bool())    // next bank already holds data
    val frag       = Output(Bool())    // read bank is a cut piece  [debug]
    val atTail     = Output(Bool())
    val remainCols = Output(UInt((p.colW + 1).W))
    val streamEnd  = Output(Bool())
  })

  val wrCount  = RegInit(VecInit(Seq.fill(n)(0.U(p.endW.W))))
  val fragR    = RegInit(VecInit(Seq.fill(n)(false.B)))
  val lastTagR = RegInit(VecInit(Seq.fill(n)(false.B)))
  val rdBank   = RegInit(0.U(p.bankW.W))
  val midChunk = RegInit(false.B)      // a chunk is currently in flight
  val done     = RegInit(false.B)

  val nextBank = (rdBank + 1.U)(p.bankW - 1, 0)
  val wrBank   = Mux(io.preload, rdBank, nextBank)

  // ---- fill -------------------------------------------------------------
  val room = wrCount(wrBank) =/= p.beats.U
  io.wrReady := io.enable && room
  io.ubFull  := !room
  io.wrBank  := wrBank
  io.wrBeat  := wrCount(wrBank)(p.beatW - 1, 0)

  // Value of midChunk after this cycle's beat, needed by the flip below so a
  // chunk that ends on the very same cycle is not counted as being in flight.
  val midNext = Mux(io.dmaFire, !io.chunkLast, midChunk)

  when(io.dmaFire) {
    wrCount(wrBank) := wrCount(wrBank) + 1.U
    midChunk        := !io.chunkLast
    // The previous beat ended a chunk, so this one starts a NEW chunk: fresh
    // data is flowing into this bank and the fragment no longer defines it.
    when(!midChunk) { fragR(wrBank) := false.B }
    when(io.dramDataLast) {
      lastTagR(wrBank) := true.B
      done             := true.B
    }
  }

  // ---- drain ------------------------------------------------------------
  val head = io.col >> p.subSel
  io.dataReady := wrCount(rdBank) > head

  // Flipping with an empty next bank would drag the write pointer ahead of
  // the stream, so park at the frontier instead. The tail bank is the one
  // exception: no further data exists, so it must be allowed to finish.
  val mayFlip = (wrCount(nextBank) =/= 0.U) || lastTagR(rdBank)
  val tailCol = ((wrCount(rdBank) << p.subSel).asUInt - 1.U)(p.colW - 1, 0)
  io.lastCol := Mux(mayFlip, tailCol, (p.cols - 1).U)

  when(io.drainLast) {
    // The drained bank becomes the write target. If a chunk was in flight it
    // is cut here, and the remainder will land in this bank.
    wrCount(rdBank)  := 0.U
    fragR(rdBank)    := midNext
    lastTagR(rdBank) := false.B
    rdBank           := nextBank
  }

  io.rdBank     := rdBank
  io.preloadOk  := (wrCount(rdBank) =/= 0.U) && !midChunk
  io.needData   := (wrCount(wrBank) === 0.U) || fragR(wrBank)
  io.relief     := wrCount(nextBank) =/= 0.U
  io.frag       := fragR(rdBank)
  io.atTail     := lastTagR(rdBank)
  io.remainCols := (wrCount(rdBank) << p.subSel).asUInt - io.col
  io.streamEnd  := done

  // ---- clear ------------------------------------------------------------
  when(io.clear) {
    wrCount  := VecInit(Seq.fill(n)(0.U(p.endW.W)))
    fragR    := VecInit(Seq.fill(n)(false.B))
    lastTagR := VecInit(Seq.fill(n)(false.B))
    rdBank   := 0.U
    midChunk := false.B
    done     := false.B
  }

  // ---- invariants -------------------------------------------------------
  assert(!(io.dmaFire && !io.wrReady),
         "UB: write accepted with wrReady low")
  assert(!(io.dmaFire && !room),
         "UB: write into a full bank")
  assert(!(io.dmaFire && io.dramDataLast && !io.chunkLast),
         "UB: dram_data_last without chunk_last")
  assert(!(io.drainLast && !io.preload && wrBank === rdBank),
         "UB: write and read pointers landed on the same bank")
}

// ===========================================================================
// Top level.
//
// FSM:  S_IDLE -> S_PRELOAD -> S_READY -(shoot)-> S_WORKING <-> S_STALL
//
// Datapath contract:
//   a read fires at T, ubDataOut is valid at T+1 alongside dataValid
//   stallIn freezes the column counter AND the compact prescaler
//   lastDataOut pulses with the final column of the tensor
//   frag says the bank being read is a cut piece
//
// stallIn is not in the spec but the loop has to close: impending goes to
// the Stall Generator, and whatever the Stall Generator decides must come
// back here, or the UB keeps streaming into a frozen pipeline.
// ===========================================================================
class UBController(p: UBParams = UBParams()) extends Module {
  val io = IO(new Bundle {
    // ---- Initializer ----
    val softReset    = Input(Bool())
    val enable       = Input(Bool())
    val shoot        = Input(Bool())
    val compactMode  = Input(Bool())   // 1 = GEMV, 16x slow-down

    // ---- Stall Generator ----
    val stallIn      = Input(Bool())

    // ---- DMA ----
    val dmaDataValid = Input(Bool())
    val dmaData      = Input(UInt(p.writeBits.W))
    val chunkLast    = Input(Bool())   // RLAST
    val dramDataLast = Input(Bool())
    val wrReady      = Output(Bool())
    val ubFull       = Output(Bool())

    // ---- Datapath ----
    val ubDataOut    = Output(UInt(p.readBits.W))
    val dataValid    = Output(Bool())
    val lastDataOut  = Output(Bool())
    val bankEnd      = Output(Bool())
    val frag         = Output(Bool())

    // ---- status / control ----
    val ready        = Output(Bool())
    val hungry       = Output(Bool())
    val impending    = Output(Bool())
    val stall        = Output(Bool())
    val streamEnd    = Output(Bool())
  })

  val sIdle :: sPreload :: sReady :: sWorking :: sStall :: Nil = Enum(5)
  val state = RegInit(sIdle)

  val mem  = Module(new UBMem(p))
  val addr = Module(new AddrGen(p))
  val bank = Module(new BankCtrl(p))

  val clear   = io.softReset || !io.enable
  val running = (state === sWorking) || (state === sStall)

  // ---- FSM --------------------------------------------------------------
  switch(state) {
    is(sIdle)    { when(io.enable)          { state := sPreload } }
    is(sPreload) { when(bank.io.preloadOk)  { state := sReady   } }
    is(sReady)   { when(io.shoot)           { state := sWorking } }
    is(sWorking) { when(!bank.io.dataReady) { state := sStall   } }
    is(sStall)   { when(bank.io.dataReady)  { state := sWorking } }
  }
  when(clear) { state := sIdle }

  // ---- write path (push) -------------------------------------------------
  val dmaFire = io.dmaDataValid && bank.io.wrReady

  bank.io.clear        := clear
  bank.io.enable       := io.enable
  bank.io.preload      := (state === sIdle) || (state === sPreload)
  bank.io.dmaFire      := dmaFire
  bank.io.chunkLast    := io.chunkLast
  bank.io.dramDataLast := io.dramDataLast

  mem.io.wen    := dmaFire
  mem.io.wrBank := bank.io.wrBank
  mem.io.wrBeat := bank.io.wrBeat
  mem.io.wdata  := io.dmaData

  io.wrReady := bank.io.wrReady
  io.ubFull  := bank.io.ubFull

  // ---- read path (pull) --------------------------------------------------
  // S_STALL is a status state, not a bubble: `en` covers WORKING and STALL
  // so the pipeline restarts on the very cycle the next bank goes full.
  addr.io.compact := io.compactMode
  addr.io.clear   := clear
  addr.io.lastCol := bank.io.lastCol
  addr.io.en      := running && bank.io.dataReady && !io.stallIn

  mem.io.ren    := addr.io.fire
  mem.io.rdBank := bank.io.rdBank     // register output: pre-flip value
  mem.io.rdCol  := addr.io.col

  bank.io.col       := addr.io.col
  bank.io.drainLast := addr.io.last

  // ---- outputs -----------------------------------------------------------
  io.ubDataOut   := mem.io.rdata
  io.dataValid   := RegNext(addr.io.fire, false.B)
  io.bankEnd     := RegNext(addr.io.last, false.B)
  io.lastDataOut := RegNext(addr.io.last && bank.io.atTail, false.B)
  io.frag        := RegNext(bank.io.frag, false.B)

  // hungry: the write bank is empty, or holds only a chunk tail that a flip
  // cut in half. `frag` is what stops a bank that merely looks non-empty from
  // dropping the request. S_STALL is included -- being starved is the most
  // hungry state there is, and the original spec left it out.
  io.hungry := ((state === sWorking) || (state === sStall) ||
                (state === sPreload)) &&
               bank.io.wrReady &&
               bank.io.needData &&
               !bank.io.atTail &&
               !bank.io.streamEnd

  // impending: the read bank runs dry within impendCycles and no relief is
  // loaded behind it. The watermark is dynamic because one column feeds the
  // array for 16 cycles in compact mode, so 8 cycles of lookahead is 8
  // columns in GEMM but only 1 column here.
  val watermark = Mux(io.compactMode, 1.U, p.impendCycles.U)
  io.impending := (state === sWorking) &&
                  !bank.io.relief &&
                  !bank.io.atTail &&
                  (bank.io.remainCols <= watermark)

  io.ready     := (state === sReady) || running
  io.stall     := state === sStall
  io.streamEnd := bank.io.streamEnd
}