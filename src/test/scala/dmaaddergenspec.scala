package dma

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer
import dma.DmaConst._

/**
 * FreqAddrGen 테스트 - 명세서 3.3 Smart Circular Fetch
 */
class FreqAddrGenSpec extends AnyFlatSpec with ChiselScalatestTester with TraceReporting {
  val BASE = BigInt("90000000", 16)

  def doLoad(dut: FreqAddrGen, base: BigInt, total: Int): Unit = {
    dut.io.base.poke(base.U)
    dut.io.totalFreqSize.poke(total.U)
    dut.io.load.poke(true.B)
    dut.clock.step()
    dut.io.load.poke(false.B)
  }

  case class ChunkResult(addrs: Seq[BigInt], tensorLast: Boolean)

  def runChunk(dut: FreqAddrGen): ChunkResult = {
    dut.io.start.poke(true.B)
    dut.clock.step()
    dut.io.start.poke(false.B)
    dut.io.req.ready.poke(true.B)

    val buf = ArrayBuffer[BigInt]()
    var done = false
    var tensor = false
    var guard = 0
    while (!done && guard < 4000) {
      if (dut.io.req.valid.peek().litToBoolean) {
        buf += dut.io.req.bits.addr.peek().litValue
        done   = dut.io.last.chunkLast.peek().litToBoolean
        tensor = dut.io.last.tensorLast.peek().litToBoolean
      }
      dut.clock.step()
      guard += 1
    }
    dut.io.req.ready.poke(false.B)
    assert(guard < 4000, "runChunk timed out")
    Trace.chunk("FB", 0, buf.toSeq, done, tensor)
    ChunkResult(buf.toSeq, tensor)
  }

  it should "Cache Mode 에서 선형으로 한 번만 읽는다" in {
    val total = 16 * 1024                      // 256 msg = 4 chunk
    test(new FreqAddrGen(32)) { dut =>
      doLoad(dut, BASE, total)
      val chunks = ArrayBuffer[ChunkResult]()
      var guard = 0
      do { chunks += runChunk(dut); guard += 1 }
      while (!chunks.last.tensorLast && guard < 16)

      val all = chunks.flatMap(_.addrs)
      assert(all.length == total / MSG_BYTES, s"got ${all.length} msgs")
      assert(all == (0 until total / MSG_BYTES).map(i => BASE + i * MSG_BYTES),
        "Cache Mode must be a single linear sweep")
      assert(chunks.last.tensorLast, "tensorLast must close Cache Mode")
    }
  }

  it should "정확히 32KB 는 Cache Mode 로 판정한다 - 경계값" in {
    val total = FB_CAPACITY_BYTES              // 정확히 경계값
    test(new FreqAddrGen(32)) { dut =>
      doLoad(dut, BASE, total)
      var chunks = 0
      var tensor = false
      while (!tensor && chunks < 16) { tensor = runChunk(dut).tensorLast; chunks += 1 }
      assert(tensor, "32KB must terminate, i.e. be Cache Mode not Spill")
    }
  }

  it should "Spill Mode 에서 끝없이 되감는다" in {
    val total = 40 * 1024                      // 640 msg > 512
    test(new FreqAddrGen(32)) { dut =>
      doLoad(dut, BASE, total)
      val all = ArrayBuffer[BigInt]()
      for (_ <- 0 until 12) {                  // 12 chunk = 768 msg > 640
        val c = runChunk(dut)
        assert(!c.tensorLast, "Spill Mode must never raise tensorLast")
        all ++= c.addrs
      }
      val wraps = (1 until all.length).count(i => all(i) < all(i - 1))
      assert(wraps >= 1, "Spill Mode must wrap at least once")
      assert(all.forall(a => a >= BASE && a < BASE + total),
        "addresses must stay inside the frequency region")
    }
  }
}


/**
 * DmaAddrGen 통합 테스트 - 라우팅과 Fusion
 */
class DmaAddrGenSpec extends AnyFlatSpec with ChiselScalatestTester with TraceReporting {
  val IN   = BigInt("80000000", 16)
  val W1   = BigInt("81000000", 16)
  val W2   = BigInt("82000000", 16)
  val P1   = BigInt("83000000", 16)
  val P2   = BigInt("84000000", 16)
  val NORM = BigInt("85000000", 16)
  val FREQ = BigInt("86000000", 16)

  def setCfg(dut: DmaAddrGen, rows: Int, cols: Int, interm: Int,
             fusion: Boolean, freqSize: Int = 16 * 1024): Unit = {
    dut.io.cfg.inputAddr.poke(IN.U)
    dut.io.cfg.weightAddr(0).poke(W1.U)
    dut.io.cfg.weightAddr(1).poke(W2.U)
    dut.io.cfg.paramAddr(0).poke(P1.U)
    dut.io.cfg.paramAddr(1).poke(P2.U)
    dut.io.cfg.normAddr.poke(NORM.U)
    dut.io.cfg.freqAddr.poke(FREQ.U)
    dut.io.cfg.rowNum.poke(rows.U)
    dut.io.cfg.colNum.poke(cols.U)
    dut.io.cfg.intermNum.poke(interm.U)
    dut.io.cfg.totalFreqSize.poke(freqSize.U)
    dut.io.cfg.fusionMode.poke(fusion.B)
    Trace.params(IN, interm, rows, interm)
  }

  def doLoad(dut: DmaAddrGen): Unit = {
    dut.io.load.poke(true.B)
    dut.clock.step()
    dut.io.load.poke(false.B)
  }

  case class ChunkResult(ocm: Int, addrs: Seq[BigInt],
                         chunkLast: Boolean, tensorLast: Boolean)

  def runChunk(dut: DmaAddrGen, ocm: Int): ChunkResult = {
    dut.io.targetOcm.poke(ocm.U)
    dut.io.trigger.poke(true.B)
    dut.clock.step()
    dut.io.trigger.poke(false.B)
    dut.io.req.ready.poke(true.B)

    val buf = ArrayBuffer[BigInt]()
    var done = false
    var chunkLast = false
    var tensorLast = false
    var guard = 0
    while (!done && guard < 4000) {
      if (dut.io.req.valid.peek().litToBoolean) {
        buf += dut.io.req.bits.addr.peek().litValue
        if (dut.io.burstDone.peek().litToBoolean) {
          done       = true
          chunkLast  = dut.io.chunkLast.peek().litToBoolean
          tensorLast = dut.io.tensorLast.peek().litToBoolean
          assert(dut.io.lastOcm.peek().litValue == ocm,
            "lastOcm must name the triggered target")
        }
      }
      dut.clock.step()
      guard += 1
    }
    dut.io.req.ready.poke(false.B)
    assert(guard < 4000, s"runChunk(ocm=$ocm) timed out")
    val name = Seq("UB", "WB", "NB", "PB", "FB")(ocm)
    val ph = if (dut.io.phase(ocm).peek().litToBoolean) 1 else 0
    Trace.chunk(name, ph, buf.toSeq, chunkLast, tensorLast)
    ChunkResult(ocm, buf.toSeq, chunkLast, tensorLast)
  }

  def refMsgs(base: BigInt, stride: Int, rows: Int, cols: Int): Seq[BigInt] =
    for {
      col <- 0 until cols
      row <- 0 until rows
      m   <- 0 until MSG_PER_TILE
    } yield base + (BigInt(row) * stride + col) * TILE_BYTES + m * MSG_BYTES

  // ------------------------------------------------------------
  it should "타겟마다 자기 base 와 stride 로 라우팅한다" in {
    val (rows, cols, interm) = (3, 4, 4)
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = false)
      doLoad(dut)

      val ub = runChunk(dut, Ocm.UB)
      val wb = runChunk(dut, Ocm.WB)
      val pb = runChunk(dut, Ocm.PB)

      // UB : stride = intermNum, rows = rowNum,    cols = intermNum
      assert(ub.addrs == refMsgs(IN, interm, rows, interm), "UB mapping")
      // WB : stride = colNum,    rows = intermNum, cols = colNum
      assert(wb.addrs == refMsgs(W1, cols, interm, cols), "WB mapping")
      assert(pb.addrs == refMsgs(P1, cols, interm, cols), "PB mapping")
    }
  }

  it should "타겟을 교차해도 포인터가 오염되지 않는다" in {
    val (rows, cols, interm) = (4, 8, 8)   // 각자 2 chunk
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = false)
      doLoad(dut)

      // UB, WB 를 번갈아 돌린다 - Arbiter 의 비대칭 스케줄링 흉내
      val u0 = runChunk(dut, Ocm.UB)
      val w0 = runChunk(dut, Ocm.WB)
      val u1 = runChunk(dut, Ocm.UB)
      val w1 = runChunk(dut, Ocm.WB)

      val ubRef = refMsgs(IN, interm, rows, interm).grouped(MSG_PER_CHUNK).toSeq
      val wbRef = refMsgs(W1, cols, interm, cols).grouped(MSG_PER_CHUNK).toSeq

      assert(u0.addrs == ubRef(0) && u1.addrs == ubRef(1), "UB pointer drifted")
      assert(w0.addrs == wbRef(0) && w1.addrs == wbRef(1), "WB pointer drifted")
    }
  }

  it should "fusion off 면 phase 를 0 으로 두고 W1/P1 만 쓴다" in {
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, 4, 8, 8, fusion = false)
      doLoad(dut)
      for (_ <- 0 until 3) {
        val c = runChunk(dut, Ocm.WB)
        assert(c.addrs.forall(a => a >= W1 && a < W2),
          "standard mode must never touch W2")
        assert(!dut.io.phase(Ocm.WB).peek().litToBoolean, "phase must stay 0")
      }
    }
  }

  /**
   * colNum 을 크게 잡아 WB 텐서가 여러 chunk 에 걸치게 한다.
   * WB 타일 수 = intermNum * colNum 이므로, 작게 잡으면 첫 chunk 에서 텐서가
   * 소진되어 W1/W2 양쪽이 halt 되고 이후 trigger 가 무시된다 (설계 의도대로).
   *   WB = 1 x 128 tile = 128 tile = 8 chunk
   */
  it should "fusion on 이면 intermNum chunk 마다 W1 과 W2 를 교차한다" in {
    val (rows, cols, interm) = (4, 128, 1)   // interm = 1 -> 매 chunk 전환
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = true)
      doLoad(dut)

      val regions = (0 until 4).map { _ =>
        val c = runChunk(dut, Ocm.WB)
        if (c.addrs.head >= W2) 1 else 0
      }
      assert(regions == Seq(0, 1, 0, 1),
        s"expected W1/W2 alternation, got $regions")
    }
  }

  /**
   * UB 타일 수 = rowNum * intermNum 이고 colTiles = intermNum 이다.
   * rowNum 을 크게 잡아야 텐서가 여러 chunk 에 걸쳐 재읽기를 관찰할 수 있다.
   *   UB = 128 x 1 tile = 128 tile = 8 chunk, intermNum = 1 이므로 매 chunk 전환
   */
  it should "두 Phase 가 같은 입력 영역을 읽는다 - UB" in {
    val (rows, cols, interm) = (128, 8, 1)
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = true)
      doLoad(dut)

      val c0 = runChunk(dut, Ocm.UB)   // Phase 0
      val c1 = runChunk(dut, Ocm.UB)   // Phase 1 - 같은 영역 재읽기
      val c2 = runChunk(dut, Ocm.UB)   // Phase 0 - 다음 영역
      val c3 = runChunk(dut, Ocm.UB)   // Phase 1 - c2 재읽기

      assert(c0.addrs == c1.addrs, "phase 1 must replay the phase 0 region")
      assert(c2.addrs == c3.addrs, "phase 1 must replay the phase 0 region")
      assert(c0.addrs != c2.addrs, "the region must advance between rounds")

      // 회귀 - save 가 전진 전 위치를 캡처하면 c2 와 c3 가 한 타일 어긋난다.
      // 두 번째 라운드는 첫 라운드 바로 다음 영역에서 이어져야 한다.
      assert(c2.addrs.head == c0.addrs.last + MSG_BYTES,
        "round 2 must start right after round 1, with no tile gap or overlap")
    }
  }

  /**
   * 회귀 테스트 - halted 인 타겟에 trigger 가 왔을 때의 deadlock.
   *
   * 인터락이 없으면 소유권만 잡히고 요청이 나가지 않아 burstDone 이 영원히
   * 오지 않는다. 시뮬레이터는 타임아웃으로 알려주지만 하드웨어는 그냥 멈춘다.
   */
  it should "halted 타겟에 trigger 가 오면 즉시 ack 한다" in {
    val (rows, cols, interm) = (3, 4, 4)   // 1 chunk 로 소진
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = false)
      doLoad(dut)

      val c0 = runChunk(dut, Ocm.UB)
      assert(c0.tensorLast, "tensor should be exhausted")
      assert(dut.io.halted(Ocm.UB).peek().litToBoolean, "UB should be halted")

      // 소진된 타겟에 다시 trigger - 요청 없이 burstDone 만 돌아와야 한다
      dut.io.targetOcm.poke(Ocm.UB.U)
      dut.io.trigger.poke(true.B)
      dut.clock.step()
      dut.io.trigger.poke(false.B)
      dut.io.req.ready.poke(true.B)

      assert(dut.io.burstDone.peek().litToBoolean,
        "a trigger on a halted target must be acked within one cycle")
      assert(!dut.io.req.valid.peek().litToBoolean,
        "no request may be issued for a halted target")
      assert(!dut.io.chunkLast.peek().litToBoolean,
        "an empty ack must not raise chunkLast - no data arrived")

      dut.clock.step()
      assert(!dut.io.burstDone.peek().litToBoolean, "ack must be a single pulse")

      // 다른 타겟은 정상 동작해야 한다
      dut.io.req.ready.poke(false.B)
      val wb = runChunk(dut, Ocm.WB)
      assert(wb.addrs.nonEmpty, "WB must still work after the halted ack")
    }
  }

  it should "Phase 0 의 tensorLast 를 억제하고 Phase 1 에서 낸다" in {
    // 텐서가 1 chunk 로 끝나므로 Phase 0 에서 경계에 먼저 닿는다 (명세서 4.3)
    val (rows, cols, interm) = (3, 4, 4)
    test(new DmaAddrGen(32, 16)) { dut =>
      setCfg(dut, rows, cols, interm, fusion = true)
      doLoad(dut)

      val c0 = runChunk(dut, Ocm.UB)
      assert(c0.chunkLast, "chunkLast must still fire at the boundary")
      assert(!c0.tensorLast,
        "phase 0 boundary is a forced transition, not a termination")

      val c1 = runChunk(dut, Ocm.UB)
      assert(c1.addrs == c0.addrs, "forced transition must replay the region")
      assert(c1.tensorLast, "phase 1 boundary is the real end")
    }
  }
}