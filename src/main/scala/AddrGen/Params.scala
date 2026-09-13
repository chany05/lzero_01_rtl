package dma

import chisel3._
import chisel3.util._

/**
 * DMA Address Generator - 공통 상수 및 Bundle 정의
 *
 * ============================================================
 * 전송 계층 (반드시 이 4계층으로 이해할 것)
 * ============================================================
 *
 *   chunk (4KB)  = Arbiter trigger 1회분. 논리적 단위이며 버스에 나가지 않음
 *     └ tile (256B)  = 주소 stride 점프의 단위. 16x16 INT8 tile 1개
 *         └ message (64B) = 실제 TileLink Get 1개 (lgSize = 6)
 *             └ beat (8B) = 물리 전송 단위. D channel에서만 관측됨
 *
 *   1 chunk = 16 tile = 64 message = 512 beat (beatBytes = 8 기준)
 *
 * TileLink는 4KB 단일 메시지를 지원하지 않는다.
 *   - a_size 는 2의 거듭제곱 바이트만 표현 가능
 *   - 주소는 2^a_size 에 natural-aligned 여야 함
 *   - rocket-chip manager 는 통상 supportsGet = TransferSizes(1, 64) 를 광고
 * 따라서 4KB chunk 는 64B Get 64개의 시퀀스로 분해된다.
 *
 * ============================================================
 * 이 모듈이 전제하는 가정 (명세서 미확정 항목)
 * ============================================================
 *   A1. 바깥 루프(N 방향 블록 반복)는 host 가 RF 를 재설정해 돌린다.
 *       HW 는 블록 1개를 처리하고 dram_data_last 후 정지한다.
 *       HW 가 Phase 교차만 담당하는 이유는, 그 경계에서만 on-chip 상태(중간
 *       텐서 Y1)를 유지해야 하기 때문이다. 블록 경계는 완전한 단절점이라
 *       host 가 개입해도 잃을 상태가 없다.
 *   A2. NB 의 base 와 주소 규칙은 명세서에 없다. WB 와 동일하다고 가정한다.
 *       PB 는 PARAM1/PARAM2 가 명시되어 있으므로 WB 와 같이 2-phase 로 둔다.
 *   A3. 행렬 크기는 tile 의 정수배이다. 즉 256B 미만 잔여가 없다.
 *   A4. tile 내부 4개 message 는 오름차순 연속 주소이다.
 *   A5. Fusion 의 Phase 전환은 chunk 경계에서만 일어난다. chunk 도중에는
 *       전환되지 않으므로 msgCnt / tileCnt 는 전환 시점에 항상 0 이다.
 */
object DmaConst {
  val TILE_BYTES  = 256
  val CHUNK_BYTES = 4096
  val MSG_BYTES   = 64

  val MSG_PER_TILE   = TILE_BYTES / MSG_BYTES    // 4
  val TILE_PER_CHUNK = CHUNK_BYTES / TILE_BYTES  // 16
  val MSG_PER_CHUNK  = CHUNK_BYTES / MSG_BYTES   // 64

  /** TileLink a_size 값. 64B = 2^6 */
  val LG_MSG_BYTES = log2Ceil(MSG_BYTES)         // 6

  /** FB 물리 용량. 이 값 이하면 Cache Mode, 초과하면 Spill Mode */
  val FB_CAPACITY_BYTES = 32 * 1024
}

/** Arbiter 가 지정하는 타겟 OCM */
object Ocm {
  val UB = 0  // Input  matrix
  val WB = 1  // Weight matrix
  val NB = 2  // (가정 A2) Weight 와 동일 규칙
  val PB = 3  // (가정 A2) Weight 와 동일 규칙
  val FB = 4  // Frequency buffer
  val NUM = 5
}

/**
 * RF 로부터 정적으로 배선되는 설정값.
 * 모든 차원값의 단위는 **tile 개수**이지 element 개수가 아니다.
 *   intermNum = ceil(K / 16),  colNum = ceil(N / 16),  rowNum = ceil(M / 16)
 * driver 가 K 를 그대로 넣으면 stride 가 16배로 튄다.
 */
class DmaConfig(val addrW: Int = 32, val dimW: Int = 16) extends Bundle {
  val inputAddr   = UInt(addrW.W)   // INPUT_ADDR

  /**
   * Weight / Param 은 phase 별로 독립 base 를 가진다 (명세서 2.1).
   *   index 0 = WEIGHT1_ADDR / PARAM1_ADDR  (Phase 0)
   *   index 1 = WEIGHT2_ADDR / PARAM2_ADDR  (Phase 1)
   * Standard Mode 에서는 index 0 만 사용된다.
   */
  val weightAddr  = Vec(2, UInt(addrW.W))
  val paramAddr   = Vec(2, UInt(addrW.W))

  val normAddr    = UInt(addrW.W)   // NB base. 명세서에 규정 없음 (가정 A2)
  val freqAddr    = UInt(addrW.W)   // FREQ_ADDR

  val rowNum      = UInt(dimW.W)    // M 방향 tile 수
  val colNum      = UInt(dimW.W)    // N 방향 tile 수
  val intermNum   = UInt(dimW.W)    // K 방향 tile 수 (= contraction dim)

  val totalFreqSize = UInt(addrW.W) // byte 단위

  /** 1 이면 명세서 4장의 Phase 교차가 동작한다 */
  val fusionMode = Bool()
}

/** 주소 생성기가 뱉는 TileLink 요청 1건 */
class MemReq(val addrW: Int = 32) extends Bundle {
  val addr   = UInt(addrW.W)
  val lgSize = UInt(4.W)
}

/** chunk / tensor 경계 표시 */
class LastFlags extends Bundle {
  /** 이번 chunk(4KB 또는 꼬리)의 마지막 요청. OCM 의 Logical Full 을 유발 */
  val chunkLast = Bool()
  /** 이 텐서의 진짜 끝. OCM 의 [LAST_TAG] 를 유발하고 이후 요청을 영구 차단 */
  val tensorLast = Bool()
}
