package dma

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer
import dma.DmaConst._

/**
 * TargetAddrGen 단위 테스트
 *
 * 레퍼런스는 절대식으로 계산한다.
 *     addr = base + (row * stride + col) * 256 + msg * 64
 * RTL 은 누적 가산으로 만들므로, 둘이 일치하면 누적 로직에 드리프트가 없다.
 * test/ref_model.py 가 같은 계산을 파이썬으로 하며 교차 검증용이다.
 */
class TargetAddrGenSpec extends AnyFlatSpec with ChiselScalatestTester with TraceReporting {

  // ------------------------------------------------------------
  // 레퍼런스
  // ------------------------------------------------------------
  def refMsgs(base: BigInt, stride: Int, rows: Int, cols: Int): Seq[BigInt] =
    for {
      col <- 0 until cols
      row <- 0 until rows
      m   <- 0 until MSG_PER_TILE
    } yield base + (BigInt(row) * stride + col) * TILE_BYTES + m * MSG_BYTES

  def refChunks(base: BigInt, stride: Int, rows: Int, cols: Int): Seq[Seq[BigInt]] =
    refMsgs(base, stride, rows, cols).grouped(MSG_PER_CHUNK).toSeq

  // ------------------------------------------------------------
  // 드라이버
  // ------------------------------------------------------------
  case class ChunkResult(addrs: Seq[BigInt], chunkLast: Boolean, tensorLast: Boolean)

  /** load 펄스. 인가 규약상 다음 cycle 까지 start 를 주면 안 된다 */
  def doLoad(dut: TargetAddrGen, bases: Seq[BigInt],
             stride: Int, rows: Int, cols: Int): Unit = {
    bases.zipWithIndex.foreach { case (b, i) => dut.io.base(i).poke(b.U) }
    dut.io.strideTiles.poke(stride.U)
    dut.io.rowTiles.poke(rows.U)
    dut.io.colTiles.poke(cols.U)
    dut.io.load.poke(true.B)
    dut.clock.step()
    dut.io.load.poke(false.B)
    Trace.params(bases.head, stride, rows, cols)
  }

  /**
   * chunk 하나를 끝까지 돌린다.
   * readyPattern 이 주어지면 그 패턴대로 backpressure 를 건다.
   */
  def runChunk(dut: TargetAddrGen, ptr: Int = 0,
               readyPattern: Seq[Boolean] = Seq(true)): ChunkResult = {
    dut.io.ptrSel.poke(ptr.U)
    dut.io.start.poke(true.B)
    dut.clock.step()
    dut.io.start.poke(false.B)

    val buf = ArrayBuffer[BigInt]()
    var chunkLast = false
    var tensorLast = false
    var i = 0
    var guard = 0

    while (!chunkLast && guard < 4000) {
      val rdy = readyPattern(i % readyPattern.length)
      dut.io.req.ready.poke(rdy.B)

      if (rdy && dut.io.req.valid.peek().litToBoolean) {
        buf += dut.io.req.bits.addr.peek().litValue
        chunkLast  = dut.io.last.chunkLast.peek().litToBoolean
        tensorLast = dut.io.last.tensorLast.peek().litToBoolean
      }
      dut.clock.step()
      i += 1
      guard += 1
    }
    dut.io.req.ready.poke(false.B)
    assert(guard < 4000, "runChunk timed out - chunkLast never asserted")
    Trace.chunk(s"ptr$ptr", ptr, buf.toSeq, chunkLast, tensorLast)
    ChunkResult(buf.toSeq, chunkLast, tensorLast)
  }

  /** tensorLast 가 뜰 때까지 chunk 를 반복한다 */
  def runTensor(dut: TargetAddrGen, ptr: Int = 0,
                readyPattern: Seq[Boolean] = Seq(true)): Seq[ChunkResult] = {
    val out = ArrayBuffer[ChunkResult]()
    var guard = 0
    do {
      out += runChunk(dut, ptr, readyPattern)
      guard += 1
    } while (!out.last.tensorLast && guard < 64)
    assert(guard < 64, "runTensor timed out")
    out.toSeq
  }

  // ------------------------------------------------------------
  // 케이스
  // ------------------------------------------------------------
  val BASE = BigInt("80000000", 16)

  /** (이름, stride, rows, cols) - ref_model.py 의 CASES 와 동일 */
  val cases = Seq(
    ("tail_only",  4, 3, 4),   // 12 tile = 48 msg  < 1 chunk
    ("two_chunks", 8, 4, 8),   // 32 tile = 128 msg = 2 chunk
    ("row_is_one", 6, 1, 6),   // rowNum = 1 - decode 단계
    ("interm_one", 1, 5, 1),   // stride = 1
    ("ragged",     5, 3, 5))   // 15 tile = 60 msg - tail

  for ((name, stride, rows, cols) <- cases) {
    it should s"레퍼런스 주소 시퀀스를 생성한다 - $name" in {
      test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
        dut.io.save.poke(false.B)
        dut.io.restore.poke(false.B)
        doLoad(dut, Seq(BASE), stride, rows, cols)

        val got = runTensor(dut).flatMap(_.addrs)
        val exp = refMsgs(BASE, stride, rows, cols)
        assert(got.length == exp.length,
          s"$name: got ${got.length} msgs, expected ${exp.length}")
        assert(got == exp, s"$name: address sequence mismatch")
      }
    }
  }

  it should "4KB chunk 로 분할하고 경계마다 chunkLast 를 올린다" in {
    val (stride, rows, cols) = (8, 4, 8)   // 정확히 2 chunk
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), stride, rows, cols)

      val chunks = runTensor(dut)
      val exp = refChunks(BASE, stride, rows, cols)

      assert(chunks.length == exp.length, "chunk count mismatch")
      chunks.zip(exp).foreach { case (c, e) =>
        assert(c.addrs == e)
        assert(c.chunkLast, "chunkLast must assert on the final message")
      }
      assert(!chunks.init.exists(_.tensorLast), "tensorLast raised too early")
      assert(chunks.last.tensorLast, "tensorLast missing on the last chunk")
    }
  }

  it should "텐서가 4KB 전에 끝나면 짧은 tail chunk 를 낸다" in {
    val (stride, rows, cols) = (5, 3, 5)   // 60 msg < 64
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), stride, rows, cols)

      val c = runChunk(dut)
      assert(c.addrs.length == 60, s"expected 60 msgs, got ${c.addrs.length}")
      assert(c.chunkLast && c.tensorLast,
        "a boundary-truncated chunk must raise both flags")
    }
  }

  it should "ready 가 내려간 동안 주소를 유지한다" in {
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), 4, 3, 4)

      // ready 를 3 cycle 중 1 cycle 만 올린다
      val c = runChunk(dut, readyPattern = Seq(true, false, false))
      assert(c.addrs == refMsgs(BASE, 4, 3, 4),
        "backpressure must not perturb the sequence")
    }
  }

  /**
   * 회귀 테스트 - tileCnt 가 chunk 경계에서 리셋되지 않던 버그를 잡은 케이스다.
   * 텐서가 tile 15 전에 끝나는 tail chunk 뒤에는 tileCnt 가 남아 있어서,
   * 다음 chunk 가 중간부터 시작해 짧게 끊겼다.
   */
  it should "두 포인터를 독립적으로 유지한다 - nPtr = 2" in {
    val (stride, rows, cols) = (4, 3, 4)
    val b0 = BASE
    val b1 = BASE + 0x100000
    test(new TargetAddrGen(32, 16, nPtr = 2)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(b0, b1), stride, rows, cols)

      // ptr0 로 한 chunk, ptr1 로 한 chunk - 서로 간섭하지 않아야 한다
      val c0 = runChunk(dut, ptr = 0)
      val c1 = runChunk(dut, ptr = 1)

      assert(c0.addrs == refMsgs(b0, stride, rows, cols))
      assert(c1.addrs == refMsgs(b1, stride, rows, cols))
      assert(c0.tensorLast && c1.tensorLast,
        "each pointer reaches its own tensor end")
    }
  }

  it should "백업 위치로 복원한다 - Fusion 재읽기" in {
    val (stride, rows, cols) = (8, 4, 8)   // 2 chunk
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), stride, rows, cols)

      val first = runChunk(dut)            // chunk 0

      // Phase 1 진입을 흉내낸다 - 백업 위치(= load 직후)로 복원
      dut.io.restore.poke(true.B)
      dut.clock.step()
      dut.io.restore.poke(false.B)

      val again = runChunk(dut)            // 같은 chunk 0 이 다시 나와야 한다
      assert(again.addrs == first.addrs,
        "restore must replay the identical address sequence")
    }
  }

  it should "조기 텐서 종료 후 restore 로 halt 를 푼다" in {
    val (stride, rows, cols) = (4, 3, 4)   // 1 chunk 로 끝남
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), stride, rows, cols)

      val c = runChunk(dut)
      assert(c.tensorLast)
      assert(dut.io.halted.peek().litToBoolean, "should be halted")

      // 명세서 4.3 의 강제 전환 - restore 로 halt 를 푼다
      dut.io.restore.poke(true.B)
      dut.clock.step()
      dut.io.restore.poke(false.B)
      assert(!dut.io.halted.peek().litToBoolean, "restore must clear halt")

      val again = runChunk(dut)
      assert(again.addrs == c.addrs)
    }
  }

  it should "발행 도중 load 가 오면 chunk 를 중단한다" in {
    test(new TargetAddrGen(32, 16, nPtr = 1)) { dut =>
      dut.io.save.poke(false.B); dut.io.restore.poke(false.B)
      doLoad(dut, Seq(BASE), 8, 4, 8)

      dut.io.ptrSel.poke(0.U)
      dut.io.start.poke(true.B)
      dut.clock.step()
      dut.io.start.poke(false.B)
      dut.io.req.ready.poke(true.B)

      dut.clock.step(10)                   // chunk 중간
      assert(dut.io.busy.peek().litToBoolean, "should be running")

      dut.io.load.poke(true.B)
      dut.clock.step()
      dut.io.load.poke(false.B)

      assert(!dut.io.busy.peek().litToBoolean,
        "load must stop the in-flight chunk")
      assert(!dut.io.req.valid.peek().litToBoolean,
        "no requests may be issued without a new trigger")

      // 재시작하면 처음부터 나와야 한다
      val c = runChunk(dut)
      assert(c.addrs.head == BASE, "must restart from base")
    }
  }
}