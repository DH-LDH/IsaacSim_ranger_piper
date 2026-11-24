"""비교한 경우 전체(128칸)를 월드 좌표계 {W}의 축별로 → figs/matrix_axes_1124.md
칸의 값은 끝단 위치 오차의 x / y / z 성분 [mm] (x 진행 방향, y 팔이 뻗은 방향 = AMR의 왼쪽, z 위아래). 실효값 표와 최댓값 표를 따로 낸다.
3차원 거리로 낸 표는 summ_matrix.py(figs/matrix_1124.md). 실행: ~/nonmoon/.venv_pptx/bin/python summ_matrix_axes.py"""
import os
import numpy as np
import summ_line as S
import summ_tk as K
H = os.path.dirname(os.path.abspath(__file__))
def cell(r, suf): return " / ".join(f"{r[a + suf]:.2f}" if suf == "_rms" else f"{r[a + suf]:.1f}" for a in "xyz") if r else "—"
md = ["# 비교한 경우 전체 — 축별 (2026-11-24, 시뮬레이션)", "", "칸의 값은 끝단 위치 오차의 **x / y / z 성분 [mm]**. x = AMR 진행 방향, y = 팔이 뻗은 방향(AMR의 왼쪽), z = 위아래. 월드 좌표계 {W} 기준이고 베이스가 진행한 수평 거리는 뺐다.",
      "조건은 3차원 거리 표(`matrix_1124.md`)와 같다: 토크 제어 + 추종 제어기(2·3번 관절), 바퀴 접촉 스프링 1.5×10⁵ N/m, 한 번씩 돌린 값.", ""]
ALL = []; n = 0
for suf, nm in (("_rms", "실효값"), ("_pk", "최댓값")):
    md += [f"# {nm}", ""]
    for p, pre, bz, pn in K.POSE:
        for k, kn in K.LINE:
            st = S.one(f"{pre}_{k}_j23_still", "none"); md += [f"## {pn} — {kn} ({nm})", "", f"AMR 정지(팔만 움직임): **{cell(st, suf)}**", "", "| 외란 | 보정 없음 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 |", "|---|---|---|---|---|"]
            for lab, *rs in K.rows("", p, pre, bz, k):
                md.append(f"| {lab} | " + " | ".join(cell(r, suf) for r in rs) + " |")
                if suf == "_rms": ALL.append((lab, rs)); n += sum(1 for r in rs if r)
            md.append("")
md += ["# 보정 없음 대비 비 — 축별, 가운데 값(범위), 네 묶음(자세 2 × 직선 2)을 합쳐서", "", "| 외란 | 지표 | 축 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 |", "|---|---|---|---|---|---|"]
for grp, sel in (("턱·패인 바닥(24건)", lambda l: "평지" not in l), ("평지 가감속(8건)", lambda l: "평지" in l)):
    for suf, nm in (("_rms", "실효값"), ("_pk", "최댓값")):
        for a in "xyz":
            A = np.array([[r[a + suf] / rs[0][a + suf] for r in rs[1:]] for lab, rs in ALL if sel(lab) and all(rs)])
            md.append(f"| {grp} | {nm} | {a} | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(3)) + " |")
md += ["", f"채워진 칸: {n} / 128", ""]
open(H + "/figs/matrix_axes_1124.md", "w").write("\n".join(md)); print("\n".join(md[-22:]))
