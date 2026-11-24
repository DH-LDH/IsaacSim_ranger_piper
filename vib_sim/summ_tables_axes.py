"""원고의 결과표 [Table 3]‒[Table 5]를 두 가지 판으로: (가) 3차원 거리(지금 원고), (나) 월드 좌표계 {W}의 축별(x 진행 방향, y 팔이 뻗은 방향, z 위아래)
→ figs/tables_two_versions.md, 그리고 --docx <파일>을 주면 박사님께 보여 드릴 문서(docx)도 만든다.
자료와 조건은 summ_final.py·summ_matrix.py와 같다(턱 10 mm·0.3 m/s의 보정 없음과 (b)+(c)는 세 번의 평균, 그 밖은 한 번).
실행: ~/nonmoon/.venv_pptx/bin/python summ_tables_axes.py [--docx 출력.docx]"""
import os, sys
import numpy as np
import summ_line as S
H = os.path.dirname(os.path.abspath(__file__)); PR = "kinprev+prevzE10+ff10"; PB = "kinmap"
GRP = [("A", "Vertical", "rl", "bzc", "z"), ("A", "Fore-aft", "rl", "bzc", "x"), ("B", "Vertical", "rlH", "bzcH", "z"), ("B", "Fore-aft", "rlH", "bzcH", "x")]
AX = [("d3", "‖e‖"), ("x", "x"), ("y", "y"), ("z", "z")]
def avg(tags, case):
    R = [r for r in (S.one(t, case) for t in tags) if r]; return ({k: float(np.mean([r[k] for r in R])) for k in R[0]}, len(R)) if R else (None, 0)
pr = lambda r, a: f"{r[a + '_pk']:.1f} / {r[a + '_rms']:.2f}"
T3, T4, T5 = {}, {}, {}
for p, ln, pre, bz, k in GRP:
    seeds = [f"{pre}_{k}_j23_{t}" for t in ("base", "base_s1", "base_s2", "base_s3")]; no, n0 = avg(seeds, "none"); ar, n1 = avg(seeds, PR); st = S.one(f"{pre}_{k}_j23_still", "none"); sh = S.one(f"{bz}_{k}_b10", "bz")
    for a, _ in AX: T3[(p, ln, a)] = [pr(st, a), pr(no, a), pr(sh, a), pr(ar, a)]
    sp = [S.one(f"{pre}_{k}_j23_{t}", c) for c in ("none", PR) for t in ("v15", "base", "v50")]
    for a, _ in AX: T4[(p, ln, a)] = [f"{r[a + '_rms']:.2f}" for r in sp]
    six = [(S.one(f"{pre}_{k}_j23_{t}", "none"), S.one(f"{pre}_{k}_j23_{t}", PB), S.one(f"{pre}_{k}_j23_{t}", PR)) for t in ("h5", "base", "h15", "v15", "v50", "pot")]
    for a, _ in AX:
        f = lambda j, key: (lambda v: f"{np.median(v):.2f} ({v.min():.2f}‒{v.max():.2f})")(np.array([s[j][a + key] / s[0][a + key] for s in six]))
        T5[(p, ln, a)] = [f(1, "_pk"), f(1, "_rms"), f(2, "_pk"), f(2, "_rms")]
    print(p, ln, "세 번 평균에 쓴 실행 수", n0, n1)
H3 = ["Base at rest", "No compensation", "Base speed shaping only", "(b) + (c)"]; H4 = ["No comp.: 0.15", "0.3", "0.5 m/s", "(b) + (c): 0.15", "0.3", "0.5 m/s"]; H5 = ["(b) only: peak", "RMS", "(b) + (c): peak", "RMS"]
TAB = [("Table 3", "Tip position error [mm], peak / RMS, 10 mm bump at 0.3 m/s (mean of three runs; base speed shaping: one run)", H3, T3),
       ("Table 4", "RMS tip position error [mm] for a 10 mm bump at three base speeds (one run each)", H4, T4),
       ("Table 5", "Ratio to no compensation over six conditions, median (range)", H5, T5)]
def rows(T, axes):
    out = []
    for p, ln, *_ in GRP:
        for a, an in AX:
            if a in axes: out.append([p, ln] + ([an] if len(axes) > 1 else []) + T[(p, ln, a)])
    return out
md = ["# 결과표의 두 가지 판 — 3차원 거리 / 축별", "", "축은 월드 좌표계 {W}: **x = AMR 진행 방향, y = 팔이 뻗은 방향(AMR의 왼쪽), z = 위아래**. 베이스가 진행한 수평 거리는 뺀 값이다(원고 식 (13)).",
      "(가)는 지금 원고의 표와 같은 값(3차원 거리 ‖e‖), (나)는 같은 실행을 축별 성분으로 나눈 것이다. 최댓값은 축마다 다른 순간에 나오므로 축별 최댓값을 제곱해 더해도 ‖e‖의 최댓값이 되지 않는다.", ""]
OUT = []
for name, cap, hd, T in TAB:
    for ver, axes, vn in (("가", ["d3"], "3차원 거리(지금 원고)"), ("나", ["x", "y", "z"], "축별")):
        h = ["Posture", "Line"] + (["Axis"] if len(axes) > 1 else []) + hd; R = rows(T, axes); OUT.append((f"[{name}] ({ver}) {vn}", cap, h, R))
        md += [f"## [{name}] ({ver}) {vn}", "", cap, "", "| " + " | ".join(h) + " |", "|" + "---|" * len(h)] + ["| " + " | ".join(r) + " |" for r in R] + [""]
open(H + "/figs/tables_two_versions.md", "w").write("\n".join(md)); print("\n".join(md[:14]))
if "--docx" in sys.argv:
    from docx import Document
    from docx.enum.section import WD_ORIENT
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.oxml.ns import qn
    from docx.shared import Mm, Pt
    doc = Document(); sec = doc.sections[0]; sec.page_width, sec.page_height = Mm(210), Mm(297); sec.left_margin = sec.right_margin = Mm(18); sec.top_margin = sec.bottom_margin = Mm(16)
    st = doc.styles["Normal"]; st.font.name = "Times New Roman"; st.font.size = Pt(10); st.element.rPr.rFonts.set(qn("w:eastAsia"), "바탕")
    def P(t, size=10, bold=False, after=3):
        p = doc.add_paragraph(); r = p.add_run(t); r.font.size = Pt(size); r.bold = bold; r._element.get_or_add_rPr().get_or_add_rFonts().set(qn("w:eastAsia"), "바탕"); p.paragraph_format.space_after = Pt(after); p.paragraph_format.space_before = Pt(0); return p
    P("KROS 원고의 결과표 두 가지 판 — 3차원 거리 / 축별", 14, True, 6)
    P("원고 4절의 [Table 3]‒[Table 5]를 지금처럼 3차원 거리로 낸 판(가)과, 같은 실행을 월드 좌표계 {W}의 축별로 나눈 판(나)이다. 어느 쪽을 원고에 실을지 고르시도록 나란히 두었다.")
    P("축의 방향: x = AMR 진행 방향, y = 팔이 뻗은 방향(AMR의 왼쪽), z = 위아래. 베이스가 진행한 수평 거리는 뺀 값이다(원고 식 (13), [Fig. 1]의 {W}).")
    P("읽을 때: 최댓값은 축마다 다른 순간에 나오므로 축별 최댓값을 제곱해 더해도 3차원 거리의 최댓값이 되지 않는다. (b) = 베이스 자세 보상, (c) = 관성력 보정(뒷바퀴 예측). 턱 10 mm·0.3 m/s의 보정 없음과 (b) + (c)는 세 번의 평균, 그 밖은 한 번 돌린 값이다.", after=8)
    for title, cap, h, R in OUT:
        p = P(title, 11, True, 1); p.paragraph_format.keep_with_next = True; p = P(cap, 9.5, False, 2); p.paragraph_format.keep_with_next = True
        t = doc.add_table(rows=len(R) + 1, cols=len(h)); t.style = "Table Grid"
        for i, r in enumerate([h] + R):
            for j, c in enumerate(r):
                cp = t.cell(i, j).paragraphs[0]; cp.alignment = WD_ALIGN_PARAGRAPH.CENTER; cp.paragraph_format.space_after = Pt(0); cp.paragraph_format.keep_with_next = i < len(R); rn = cp.add_run(c); rn.font.size = Pt(8.5); rn.bold = (i == 0); rn.italic = (i > 0 and h[j] == "Axis" and c in "xyz")
        P("", 4, False, 6)
    P("전 조건(128칸)의 축별 값은 비교_경우_전체표_축별_1124.md에 있다.", 9)
    out = sys.argv[sys.argv.index("--docx") + 1]; doc.save(out); print("saved", out)
