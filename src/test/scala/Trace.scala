package dma

import java.nio.file.{Files, Paths, StandardOpenOption}
import java.nio.charset.StandardCharsets
import org.scalatest.{Outcome, TestSuite, TestSuiteMixin}

/**
 * 테스트 트레이스 기록기
 *
 * DUT 가 실제로 낸 주소 시퀀스와 플래그를 JSONL 로 남긴다.
 * 레퍼런스 기대값이 아니라 RTL 출력이라는 점이 중요하다 - 시각화를 보고
 * "기대대로 나왔나"가 아니라 "하드웨어가 뭘 했나"를 판단할 수 있어야 한다.
 *
 *   target/dma-trace/trace.jsonl
 *
 * 렌더링:
 *   python3 test/viz_trace.py
 *   -> target/dma-trace/report.html
 */
object Trace {
  private val dir  = Paths.get("target", "dma-trace")
  private val file = dir.resolve("trace.jsonl")

  private var current: String = "unknown"
  private var suite:   String = "unknown"
  private var seq:     Int    = 0

  /** 환경변수 DMA_TRACE=0 이면 기록을 끈다 */
  private val enabled: Boolean = sys.env.getOrElse("DMA_TRACE", "1") != "0"

  def reset(): Unit = if (enabled) synchronized {
    Files.createDirectories(dir)
    Files.deleteIfExists(file)
  }

  private def write(json: String): Unit = if (enabled) synchronized {
    Files.createDirectories(dir)
    Files.write(file, (json + "\n").getBytes(StandardCharsets.UTF_8),
      StandardOpenOption.CREATE, StandardOpenOption.APPEND)
  }

  private def esc(s: String): String =
    s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ")

  def start(suiteName: String, testName: String): Unit = {
    suite = suiteName
    current = testName
    seq = 0
    write(s"""{"kind":"start","suite":"${esc(suiteName)}","test":"${esc(testName)}"}""")
  }

  /** 테스트가 쓰는 파라미터. 격자 렌더링에 필요하다 */
  def params(base: BigInt, stride: Int, rows: Int, cols: Int): Unit =
    write(s"""{"kind":"params","suite":"${esc(suite)}","test":"${esc(current)}",""" +
          s""""base":$base,"stride":$stride,"rows":$rows,"cols":$cols}""")

  /** chunk 하나의 결과 */
  def chunk(target: String, ptr: Int, addrs: Seq[BigInt],
            chunkLast: Boolean, tensorLast: Boolean): Unit = {
    val a = addrs.mkString(",")
    write(s"""{"kind":"chunk","suite":"${esc(suite)}","test":"${esc(current)}",""" +
          s""""seq":$seq,"target":"$target","ptr":$ptr,"n":${addrs.length},""" +
          s""""chunkLast":$chunkLast,"tensorLast":$tensorLast,"addrs":[$a]}""")
    seq += 1
  }

  def result(ok: Boolean, detail: String): Unit =
    write(s"""{"kind":"result","suite":"${esc(suite)}","test":"${esc(current)}",""" +
          s""""ok":$ok,"detail":"${esc(detail)}"}""")
}


/**
 * 스펙에 섞으면 테스트 이름과 통과 여부가 자동으로 기록된다.
 *
 *   class MySpec extends AnyFlatSpec with ChiselScalatestTester with TraceReporting
 */
trait TraceReporting extends TestSuiteMixin { this: TestSuite =>
  abstract override def withFixture(test: NoArgTest): Outcome = {
    Trace.start(suiteName, test.name)
    val outcome = super.withFixture(test)
    val detail = outcome match {
      case f: org.scalatest.Failed => f.exception.getMessage
      case _                       => ""
    }
    Trace.result(outcome.isSucceeded, if (detail == null) "" else detail)
    outcome
  }
}