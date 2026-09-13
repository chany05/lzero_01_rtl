#!/usr/bin/env python3
"""
DMA Address Generator 레퍼런스 모델

RTL 과 독립적으로 기대 주소 시퀀스를 계산한다. RTL 은 누적 가산(incremental)
으로 주소를 만들고 이 모델은 절대식(absolute)으로 만들기 때문에, 둘이 일치하면
누적 로직에 드리프트가 없다는 뜻이 된다.

    absolute : addr = base + (row * stride + col) * 256 + msg * 64
    RTL      : tile 경계마다 += stride*256, wrap 시 base + col*256

사용:
    python3 ref_model.py            # 시퀀스 요약 출력
    python3 ref_model.py --dump     # 전체 주소를 hex 로 출력
"""

import argparse

TILE_BYTES = 256
CHUNK_BYTES = 4096
MSG_BYTES = 64

MSG_PER_TILE = TILE_BYTES // MSG_BYTES      # 4
TILE_PER_CHUNK = CHUNK_BYTES // TILE_BYTES  # 16
MSG_PER_CHUNK = CHUNK_BYTES // MSG_BYTES    # 64

FB_CAPACITY = 32 * 1024


# ----------------------------------------------------------------------
# 3.1 / 3.2  Tiled Transpose Read
# ----------------------------------------------------------------------
def target_tiles(base, stride, rows, cols):
    """방문 순서대로 (row, col, tile_addr) 를 낸다.

    바깥 루프가 col, 안쪽 루프가 row 다. 세로로 훑어 내려가므로 row-major 로
    저장된 행렬을 column 단위로 뽑게 된다 = transpose.
    """
    for col in range(cols):
        for row in range(rows):
            yield row, col, base + (row * stride + col) * TILE_BYTES


def target_msgs(base, stride, rows, cols):
    """64B message 단위 주소 시퀀스."""
    out = []
    for _, _, tile_addr in target_tiles(base, stride, rows, cols):
        for m in range(MSG_PER_TILE):
            out.append(tile_addr + m * MSG_BYTES)
    return out


def chunkify(msgs):
    """message 리스트를 chunk 로 자른다. 마지막은 4KB 미만일 수 있다 (tail)."""
    return [msgs[i:i + MSG_PER_CHUNK] for i in range(0, len(msgs), MSG_PER_CHUNK)]


# ----------------------------------------------------------------------
# 3.3  FB Smart Circular Fetch
# ----------------------------------------------------------------------
def freq_msgs(base, total_size, n_msgs):
    """n_msgs 개를 뽑는다. Cache Mode 면 total_size 에서 끊기고,
    Spill Mode 면 base 로 되감아 계속된다."""
    cache = total_size <= FB_CAPACITY
    out = []
    sent = 0
    addr = base
    for _ in range(n_msgs):
        out.append(addr)
        sent += MSG_BYTES
        if sent >= total_size:
            if cache:
                break
            addr, sent = base, 0
        else:
            addr += MSG_BYTES
    return out


# ----------------------------------------------------------------------
# 4장  Fusion - UB 되감기
# ----------------------------------------------------------------------
def fusion_ub_chunks(base, stride, rows, cols, interm_num):
    """UB 의 chunk 시퀀스. 각 영역을 Phase 0 / Phase 1 로 두 번 읽는다.

    전환 규칙 (4.1, 4.3)
      - chunk 를 interm_num 개 보내면 전환
      - 못 채워도 텐서 경계를 만나면 강제 전환
    포인터 규칙 (4.2)
      - Phase 1 진입: 백업 위치로 복원
      - Phase 0 복귀: 현재 위치를 백업
    """
    all_chunks = chunkify(target_msgs(base, stride, rows, cols))
    out = []          # (phase, chunk) 리스트
    i = 0             # 다음에 읽을 chunk 인덱스
    saved = 0         # 백업된 chunk 인덱스
    phase = 0

    while saved < len(all_chunks):
        cnt = 0
        while True:
            if i >= len(all_chunks):
                break
            out.append((phase, all_chunks[i]))
            i += 1
            cnt += 1
            boundary = (i == len(all_chunks))
            if cnt == interm_num or boundary:
                break
        if phase == 0:
            i = saved          # restore - 같은 영역을 다시 읽는다
            phase = 1
        else:
            saved = i          # save - 현재 위치가 다음 영역의 시작
            phase = 0
    return out


# ----------------------------------------------------------------------
# 검증 - 절대식과 누적식이 같은지
# ----------------------------------------------------------------------
def incremental_msgs(base, stride, rows, cols):
    """RTL 과 동일한 누적 방식. 절대식과 일치해야 한다."""
    out = []
    tile_addr = base
    row = col = 0
    while True:
        for m in range(MSG_PER_TILE):
            out.append(tile_addr + m * MSG_BYTES)
        if row == rows - 1 and col == cols - 1:
            break
        if row == rows - 1:
            row, col = 0, col + 1
            tile_addr = base + col * TILE_BYTES      # wrap - 절대 재계산
        else:
            row += 1
            tile_addr += stride * TILE_BYTES          # stride - 누적
    return out


# ----------------------------------------------------------------------
# 시각화
# ----------------------------------------------------------------------
def viz_grid(base, stride, rows, cols):
    """타일 격자에 방문 순서를 찍는다.

    각 칸은  #방문순서 / DRAM 타일 인덱스  두 줄이다.
    세로로 훑어 내려가다 열 끝에서 다음 열 맨 위로 wrap 하는 것이 보여야 한다.
    """
    order = {}
    for k, (r, c, addr) in enumerate(target_tiles(base, stride, rows, cols)):
        order[(r, c)] = (k, (addr - base) // TILE_BYTES)

    w = 9
    print("       " + "".join(f"{'col '+str(c):^{w}}" for c in range(cols)))
    print("      +" + ("-" * (w - 1) + "+") * cols)
    for r in range(rows):
        top = f"row {r} |"
        bot = "      |"
        for c in range(cols):
            k, tile = order[(r, c)]
            top += f"{'#'+str(k):^{w-1}}|"
            bot += f"{'t'+str(tile):^{w-1}}|"
        print(top)
        print(bot)
        print("      +" + ("-" * (w - 1) + "+") * cols)
    print("      # = 방문 순서,  t = DRAM 타일 위치")


def viz_chunks(base, stride, rows, cols):
    """chunk 분할을 막대로 그린다. 마지막이 짧으면 tail 이다."""
    chunks = chunkify(target_msgs(base, stride, rows, cols))
    width = 48
    for i, ch in enumerate(chunks):
        filled = round(len(ch) / MSG_PER_CHUNK * width)
        bar = "#" * filled + "." * (width - filled)
        tag = "tail" if len(ch) < MSG_PER_CHUNK else "full"
        print(f"  chunk {i}  [{bar}] {len(ch):3d}/{MSG_PER_CHUNK} msg  {tag}")


def viz_fusion(base, stride, rows, cols, interm_num):
    """Phase 교차 타임라인. 같은 영역이 두 번 나오는지 본다."""
    seq = fusion_ub_chunks(base, stride, rows, cols, interm_num)
    prev = None
    for k, (ph, ch) in enumerate(seq):
        lane = "P0 |####|    " if ph == 0 else "P1 |    |####"
        mark = " <- 재읽기" if prev is not None and ch == prev else ""
        print(f"  #{k:2d}  {lane}  0x{ch[0]:08x}..0x{ch[-1]:08x} "
              f"({len(ch)} msg){mark}")
        prev = ch


def viz_freq(base, total, n):
    """FB 주소를 막대 위치로 찍는다. 되감기가 보인다."""
    msgs = freq_msgs(base, total, n)
    width = 56
    step = max(1, len(msgs) // 24)
    for i in range(0, len(msgs), step):
        pos = round((msgs[i] - base) / total * (width - 1))
        line = "." * pos + "#" + "." * (width - 1 - pos)
        print(f"  msg {i:4d}  |{line}|")


# ----------------------------------------------------------------------
CASES = {
    # 이름: (base, stride, rows, cols)
    "tail_only":   (0x8000_0000, 4, 3, 4),    # 12 tile = 48 msg  < 1 chunk
    "two_chunks":  (0x8000_0000, 8, 4, 8),    # 32 tile = 128 msg = 2 chunk
    "row_is_one":  (0x8000_0000, 6, 1, 6),    # rowNum = 1 - decode 단계
    "interm_one":  (0x8000_0000, 1, 5, 1),    # intermNum = 1
    "ragged":      (0x8000_0000, 5, 3, 5),    # 15 tile = 60 msg - tail
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", action="store_true")
    ap.add_argument("--viz", action="store_true", help="격자/chunk/Phase 시각화")
    args = ap.parse_args()

    print("=" * 68)
    print("절대식 vs 누적식 일치 검사")
    print("=" * 68)
    for name, (base, stride, rows, cols) in CASES.items():
        a = target_msgs(base, stride, rows, cols)
        b = incremental_msgs(base, stride, rows, cols)
        status = "OK" if a == b else "MISMATCH"
        print(f"  {name:12s} rows={rows} cols={cols} stride={stride} "
              f"msgs={len(a):4d}  {status}")
        assert a == b, name

    print()
    print("=" * 68)
    print("chunk 분할 (마지막이 4KB 미만이면 tail)")
    print("=" * 68)
    for name, (base, stride, rows, cols) in CASES.items():
        chunks = chunkify(target_msgs(base, stride, rows, cols))
        sizes = [len(c) for c in chunks]
        print(f"  {name:12s} chunks={len(chunks)} sizes={sizes}")

    print()
    print("=" * 68)
    print("tail_only 방문 순서 - transpose 확인")
    print("=" * 68)
    base, stride, rows, cols = CASES["tail_only"]
    print(f"  {rows}x{cols} tile 행렬, 가로폭 {stride} tile")
    print("  (row,col) -> tile index")
    order = [(r, c, (a - base) // TILE_BYTES)
             for r, c, a in target_tiles(base, stride, rows, cols)]
    print("   ", " ".join(f"({r},{c})->{t}" for r, c, t in order))

    print()
    print("=" * 68)
    print("FB Circular Fetch")
    print("=" * 68)
    for label, total in [("cache (16KB)", 16 * 1024),
                         ("cache (=32KB)", 32 * 1024),
                         ("spill (40KB)", 40 * 1024)]:
        m = freq_msgs(0x9000_0000, total, 700)
        wrapped = sum(1 for i in range(1, len(m)) if m[i] < m[i - 1])
        print(f"  {label:14s} msgs={len(m):4d} wraps={wrapped}")

    print()
    print("=" * 68)
    print("Fusion - UB chunk 시퀀스 (two_chunks, intermNum=1)")
    print("=" * 68)
    base, stride, rows, cols = CASES["two_chunks"]
    seq = fusion_ub_chunks(base, stride, rows, cols, interm_num=1)
    for k, (ph, ch) in enumerate(seq):
        print(f"  #{k} phase={ph} first=0x{ch[0]:08x} "
              f"last=0x{ch[-1]:08x} n={len(ch)}")
    print("  -> 같은 영역이 phase 0 / 1 로 두 번 나와야 한다")

    if args.viz:
        print()
        print("=" * 68)
        print("타일 방문 순서 - tail_only (stride=4, rows=3, cols=4)")
        print("=" * 68)
        viz_grid(*CASES["tail_only"])

        print()
        print("=" * 68)
        print("타일 방문 순서 - row_is_one (rowNum=1, decode 단계)")
        print("=" * 68)
        viz_grid(*CASES["row_is_one"])
        print("      rowNum=1 이면 매 타일이 wrap - 사실상 linear 이다")

        print()
        print("=" * 68)
        print("chunk 분할")
        print("=" * 68)
        for name in ("two_chunks", "ragged"):
            print(f"  [{name}]")
            viz_chunks(*CASES[name])

        print()
        print("=" * 68)
        print("Fusion Phase 타임라인 (two_chunks, intermNum=1)")
        print("=" * 68)
        viz_fusion(*CASES["two_chunks"], interm_num=1)

        print()
        print("=" * 68)
        print("FB Spill Mode 주소 궤적 (40KB)")
        print("=" * 68)
        viz_freq(0x9000_0000, 40 * 1024, 700)

    if args.dump:
        print()
        print("=" * 68)
        print("전체 주소 덤프 (tail_only)")
        print("=" * 68)
        for i, a in enumerate(target_msgs(*CASES["tail_only"])):
            print(f"  {i:4d} 0x{a:08x}")


if __name__ == "__main__":
    main()