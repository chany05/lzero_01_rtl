#!/usr/bin/env python3
"""
DMA 테스트 트레이스 시각화

ChiselTest 가 남긴 target/dma-trace/trace.jsonl 을 읽어 HTML 리포트를 만든다.
기대값이 아니라 **DUT 가 실제로 낸 주소**를 그린다.

사용:
    sbt test                       # 트레이스 생성
    python3 test/viz_trace.py      # -> target/dma-trace/report.html

옵션:
    --ascii    HTML 대신 터미널에 요약을 찍는다
"""

import argparse
import html
import json
import os
import sys
from collections import OrderedDict

TILE_BYTES = 256
MSG_BYTES = 64
MSG_PER_CHUNK = 64

TRACE = os.path.join("target", "dma-trace", "trace.jsonl")
REPORT = os.path.join("target", "dma-trace", "report.html")


# ----------------------------------------------------------------------
def load(path):
    """JSONL 을 테스트 단위로 묶는다."""
    if not os.path.exists(path):
        sys.exit(f"트레이스가 없습니다: {path}\n먼저 sbt test 를 실행하세요.")

    tests = OrderedDict()
    cur = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            key = (r["suite"], r["test"])
            if r["kind"] == "start":
                tests[key] = {"suite": r["suite"], "test": r["test"],
                              "params": None, "chunks": [],
                              "ok": None, "detail": ""}
                cur = tests[key]
            elif cur is None:
                continue
            elif r["kind"] == "params":
                cur["params"] = r
            elif r["kind"] == "chunk":
                cur["chunks"].append(r)
            elif r["kind"] == "result":
                cur["ok"] = r["ok"]
                cur["detail"] = r.get("detail", "")
    return list(tests.values())


# ----------------------------------------------------------------------
def tile_grid(t):
    """방문 순서 격자. 주소를 (row, col) 로 역산한다.

    tile index = (addr - base) / 256,  row = idx // stride,  col = idx % stride
    격자가 너무 크면 생략한다.
    """
    p = t["params"]
    if not p:
        return None
    base, stride, rows, cols = p["base"], p["stride"], p["rows"], p["cols"]
    if rows * cols > 96 or rows == 0 or cols == 0:
        return None

    # 방문 순서대로 tile 시작 주소만 추린다
    visits = []
    for ch in t["chunks"]:
        for i, a in enumerate(ch["addrs"]):
            if (a - base) % TILE_BYTES == 0:
                visits.append((a, ch["seq"]))

    cell = {}
    for k, (a, chunk_seq) in enumerate(visits):
        idx = (a - base) // TILE_BYTES
        r, c = idx // stride, idx % stride
        if 0 <= r < rows and 0 <= c < stride:
            cell.setdefault((r, c), []).append((k, idx, chunk_seq))
    return base, stride, rows, cols, cell


def grid_html(t):
    g = tile_grid(t)
    if not g:
        return ""
    base, stride, rows, cols, cell = g
    ncol = min(stride, 12)
    out = ['<table class="grid"><tr><th></th>']
    out += [f"<th>col {c}</th>" for c in range(ncol)]
    out.append("</tr>")
    for r in range(rows):
        out.append(f"<tr><th>row {r}</th>")
        for c in range(ncol):
            hits = cell.get((r, c))
            if not hits:
                out.append('<td class="empty"></td>')
            else:
                k, idx, cseq = hits[0]
                rep = f'<span class="rep">x{len(hits)}</span>' if len(hits) > 1 else ""
                out.append(f'<td class="hit c{cseq % 6}">'
                           f'<b>#{k}</b>{rep}<br><span class="sub">t{idx}</span></td>')
        out.append("</tr>")
    out.append("</table>")
    return "".join(out)


def chunk_bars(t):
    rows = []
    for ch in t["chunks"]:
        n = ch["n"]
        pct = min(100, round(n / MSG_PER_CHUNK * 100))
        tail = n < MSG_PER_CHUNK
        flags = []
        if ch["chunkLast"]:
            flags.append('<span class="flag">chunkLast</span>')
        if ch["tensorLast"]:
            flags.append('<span class="flag last">tensorLast</span>')
        first = ch["addrs"][0] if ch["addrs"] else 0
        last = ch["addrs"][-1] if ch["addrs"] else 0
        rows.append(
            f'<div class="bar-row">'
            f'<span class="tag">{html.escape(ch["target"])}'
            f'<em>p{ch["ptr"]}</em></span>'
            f'<div class="bar"><div class="fill{" tail" if tail else ""}" '
            f'style="width:{pct}%"></div></div>'
            f'<span class="num">{n}/{MSG_PER_CHUNK}</span>'
            f'<span class="addr">0x{first:08x} → 0x{last:08x}</span>'
            f'{"".join(flags)}</div>')
    return "".join(rows)


def phase_lane(t):
    """Phase 교차와 재읽기를 레인으로 그린다."""
    chunks = t["chunks"]
    if not chunks or all(c["ptr"] == 0 for c in chunks):
        return ""
    seen = {}
    out = ['<div class="lanes">']
    for ch in chunks:
        key = (ch["addrs"][0] if ch["addrs"] else 0, ch["n"])
        repeat = key in seen
        seen[key] = True
        p = ch["ptr"]
        cls = "p1" if p else "p0"
        mark = '<span class="repeat">재읽기</span>' if repeat else ""
        out.append(f'<div class="lane"><span class="lane-id">#{ch["seq"]}</span>'
                   f'<span class="cell {cls}">P{p}</span>{mark}</div>')
    out.append("</div>")
    return "".join(out)


# ----------------------------------------------------------------------
CSS = """
:root{--bg:#faf9f5;--card:#fff;--bd:#e3e1d8;--fg:#26241f;--mut:#73726c;
--ok:#3B6D11;--okbg:#EAF3DE;--ng:#A32D2D;--ngbg:#FCEBEB;--ac:#534AB7}
@media(prefers-color-scheme:dark){:root{--bg:#1a1915;--card:#24231f;--bd:#3b3a34;
--fg:#e8e6dd;--mut:#9c9a92;--okbg:#27500A;--ok:#C0DD97;--ngbg:#791F1F;--ng:#F7C1C1;--ac:#AFA9EC}}
*{box-sizing:border-box}
body{margin:0;padding:32px;background:var(--bg);color:var(--fg);
font:14px/1.6 ui-sans-serif,system-ui,sans-serif}
h1{font-size:22px;font-weight:500;margin:0 0 4px}
h2{font-size:18px;font-weight:500;margin:32px 0 12px}
h3{font-size:15px;font-weight:500;margin:0}
.sum{color:var(--mut);margin-bottom:24px}
.pill{display:inline-block;padding:2px 10px;border-radius:999px;font-size:12px;margin-right:6px}
.pass{background:var(--okbg);color:var(--ok)}
.fail{background:var(--ngbg);color:var(--ng)}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;
padding:16px 18px;margin-bottom:14px}
.card.bad{border-color:var(--ng)}
.hd{display:flex;align-items:center;gap:10px;margin-bottom:12px;flex-wrap:wrap}
.meta{color:var(--mut);font-size:12px;font-family:ui-monospace,monospace}
.err{background:var(--ngbg);color:var(--ng);padding:8px 10px;border-radius:8px;
font-family:ui-monospace,monospace;font-size:12px;margin-top:10px;
white-space:pre-wrap;word-break:break-all;max-height:160px;overflow:auto}
.bar-row{display:flex;align-items:center;gap:10px;margin:5px 0;font-size:12px}
.tag{min-width:64px;font-weight:500}
.tag em{color:var(--mut);font-style:normal;margin-left:4px}
.bar{flex:0 0 220px;height:10px;background:var(--bd);border-radius:5px;overflow:hidden}
.fill{height:100%;background:var(--ac)}
.fill.tail{background:#EF9F27}
.num{font-family:ui-monospace,monospace;color:var(--mut);min-width:52px}
.addr{font-family:ui-monospace,monospace;color:var(--mut)}
.flag{background:var(--bd);padding:1px 7px;border-radius:999px;font-size:11px}
.flag.last{background:var(--okbg);color:var(--ok)}
.grid{border-collapse:separate;border-spacing:3px;margin-top:12px;font-size:11px}
.grid th{color:var(--mut);font-weight:400;padding:0 4px}
.grid td{width:54px;height:38px;text-align:center;border-radius:6px;
background:var(--bd);color:var(--mut)}
.grid td.hit{background:#EEEDFE;color:#3C3489}
.grid td.c1{background:#E1F5EE;color:#085041}.grid td.c2{background:#FAECE7;color:#712B13}
.grid td.c3{background:#E6F1FB;color:#0C447C}.grid td.c4{background:#FBEAF0;color:#72243E}
.grid td.c5{background:#FAEEDA;color:#633806}
.grid td.empty{background:transparent;border:1px dashed var(--bd)}
.grid .sub{opacity:.7}
.grid .rep{color:#993C1D;margin-left:3px}
.lanes{margin-top:10px}
.lane{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:12px}
.lane-id{font-family:ui-monospace,monospace;color:var(--mut);min-width:28px}
.cell{padding:2px 12px;border-radius:6px;font-size:11px}
.p0{background:#EEEDFE;color:#3C3489}.p1{background:#E1F5EE;color:#085041}
.repeat{color:#993C1D;font-size:11px}
.legend{color:var(--mut);font-size:12px;margin-top:8px}
"""


def render(tests):
    total = len(tests)
    passed = sum(1 for t in tests if t["ok"])
    parts = [f"<style>{CSS}</style>",
             "<h1>DMA address generator - 테스트 트레이스</h1>",
             f'<div class="sum"><span class="pill pass">{passed} passed</span>'
             f'<span class="pill fail">{total - passed} failed</span>'
             f"DUT 가 실제로 발행한 주소를 그린 것이며 기대값이 아니다</div>"]

    by_suite = OrderedDict()
    for t in tests:
        by_suite.setdefault(t["suite"], []).append(t)

    for suite, group in by_suite.items():
        parts.append(f"<h2>{html.escape(suite)}</h2>")
        for t in group:
            bad = "" if t["ok"] else " bad"
            badge = '<span class="pill pass">pass</span>' if t["ok"] \
                else '<span class="pill fail">fail</span>'
            p = t["params"]
            meta = ""
            if p:
                meta = (f'<span class="meta">stride={p["stride"]} '
                        f'rows={p["rows"]} cols={p["cols"]} '
                        f'base=0x{p["base"]:08x}</span>')
            nmsg = sum(c["n"] for c in t["chunks"])
            parts.append(
                f'<div class="card{bad}"><div class="hd">{badge}'
                f'<h3>{html.escape(t["test"])}</h3>{meta}'
                f'<span class="meta">{len(t["chunks"])} chunk / {nmsg} msg</span></div>')
            parts.append(chunk_bars(t))
            parts.append(phase_lane(t))
            parts.append(grid_html(t))
            if grid_html(t):
                parts.append('<div class="legend">#n = 방문 순서, '
                             'tn = DRAM 타일 위치, 색 = chunk 번호</div>')
            if not t["ok"] and t["detail"]:
                parts.append(f'<div class="err">{html.escape(t["detail"][:1200])}</div>')
            parts.append("</div>")
    return "\n".join(parts)


def ascii_report(tests):
    for t in tests:
        mark = "PASS" if t["ok"] else "FAIL"
        print(f"[{mark}] {t['suite']} :: {t['test']}")
        for ch in t["chunks"]:
            w = 40
            filled = round(ch["n"] / MSG_PER_CHUNK * w)
            bar = "#" * filled + "." * (w - filled)
            fl = []
            if ch["chunkLast"]:
                fl.append("chunkLast")
            if ch["tensorLast"]:
                fl.append("tensorLast")
            a0 = ch["addrs"][0] if ch["addrs"] else 0
            print(f"    {ch['target']:>4s} p{ch['ptr']} [{bar}] "
                  f"{ch['n']:3d}/64  0x{a0:08x}  {' '.join(fl)}")
        if not t["ok"]:
            print(f"    ! {t['detail'][:160]}")
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", default=TRACE)
    ap.add_argument("--out", default=REPORT)
    ap.add_argument("--ascii", action="store_true")
    args = ap.parse_args()

    tests = load(args.trace)
    if args.ascii:
        ascii_report(tests)
        return

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(render(tests))
    passed = sum(1 for t in tests if t["ok"])
    print(f"{args.out} 생성 완료 - {passed}/{len(tests)} passed")


if __name__ == "__main__":
    main()