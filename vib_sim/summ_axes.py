"""끝단 위치 오차를 월드 좌표계 {W}의 축별로 → figs/axes_1124.md
x = AMR 진행 방향, y = 왼쪽(팔이 뻗은 방향 x_a), z = 위. 베이스가 진행한 수평 거리는 뺀 값(원고 식 (13)의 e의 성분).
턱 10 mm·0.3 m/s는 세 번 돌린 평균, 그 밖의 조건은 한 번. 실행: ~/nonmoon/.venv_pptx/bin/python summ_axes.py"""
import os
import numpy as np
import summ_line as S
H = os.path.dirname(os.path.abspath(__file__)); PR = "kinprev+prevzE10+ff10"
GRP = [("A", "Vertical", "rl", "z"), ("A", "Fore-aft", "rl", "x"), ("B", "Vertical", "rlH", "z"), ("B", "Fore-aft", "rlH", "x")]
def avg(tags, case):
    R = [r for r in (S.one(t, case) for t in tags) if r]; return {k: float(np.mean([r[k] for r in R])) for k in R[0]} if R else None
md = ["# 끝단 위치 오차의 축별 값 (월드 좌표계 {W}: x 진행 방향, y 왼쪽 = 팔이 뻗은 방향, z 위)", "", "## 턱 10 mm, 0.3 m/s — 실효값 (최댓값) [mm], 세 번의 평균", "",
      "| 자세 | 직선 | 정지: x | y | z | 보정 없음: x | y | z | 로봇팔 보정: x | y | z |", "|---|---|---|---|---|---|---|---|---|---|---|"]
c = lambda r, a: f"{r[a + '_rms']:.2f} ({r[a + '_pk']:.1f})"; RAT = {a: [] for a in "xyz"}; RATP = {a: [] for a in "xyz"}
for p, ln, pre, k in GRP:
    st = S.one(f"{pre}_{k}_j23_still", "none"); tg = [f"{pre}_{k}_j23_{t}" for t in ("base", "base_s2", "base_s3")]; no, ar = avg(tg, "none"), avg(tg, PR)
    md.append(f"| {p} | {ln} | " + " | ".join(c(st, a) for a in "xyz") + " | " + " | ".join(c(no, a) for a in "xyz") + " | " + " | ".join(c(ar, a) for a in "xyz") + " |")
    for a in "xyz": RAT[a].append(ar[a + "_rms"] / no[a + "_rms"]); RATP[a].append(ar[a + "_pk"] / no[a + "_pk"])
md += ["", "로봇팔 보정 / 보정 없음(네 묶음의 범위): " + ", ".join(f"{a} 실효값 {min(RAT[a]):.2f}–{max(RAT[a]):.2f}·최댓값 {min(RATP[a]):.2f}–{max(RATP[a]):.2f}" for a in "xyz"), "",
       "## 여섯 조건(턱 5·10·15 mm, 0.15·0.3·0.5 m/s, 패인 바닥) × 네 묶음 — 로봇팔 보정 / 보정 없음, 가운데 값(범위)", "", "| 축 | 실효값 | 최댓값 | 보정 없음의 실효값 범위 [mm] | 보정 뒤 |", "|---|---|---|---|---|"]
ALL = []
for p, ln, pre, k in GRP:
    for t in ("h5", "base", "h15", "v15", "v50", "pot"):
        no, ar = S.one(f"{pre}_{k}_j23_{t}", "none"), S.one(f"{pre}_{k}_j23_{t}", PR)
        if no and ar: ALL.append((no, ar))
for a in "xyz":
    r = np.array([ar[a + "_rms"] / no[a + "_rms"] for no, ar in ALL]); rp = np.array([ar[a + "_pk"] / no[a + "_pk"] for no, ar in ALL]); n0 = [no[a + "_rms"] for no, _ in ALL]; n1 = [ar[a + "_rms"] for _, ar in ALL]
    md.append(f"| {a} | {np.median(r):.2f} ({r.min():.2f}–{r.max():.2f}) | {np.median(rp):.2f} ({rp.min():.2f}–{rp.max():.2f}) | {min(n0):.2f}–{max(n0):.2f} | {min(n1):.2f}–{max(n1):.2f} |")
md += ["", f"건수 {len(ALL)}", ""]
open(H + "/figs/axes_1124.md", "w").write("\n".join(md)); print("\n".join(md))
