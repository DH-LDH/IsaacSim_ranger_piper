"""비교한 경우 전체(자세 2 × 직선 2 × 외란 8 × 제어 4 = 128칸 + AMR 정지 기준) → figs/matrix_1124.md
값은 끝단이 있어야 할 자리에서 떨어진 거리(월드 기준, 3차원 전체), 최대 / 실효값[mm]. 토크 제어 + 추종 제어기 J2·J3, 관절 강성 1100·500·800."""
import os
import numpy as np
import summ_line as S
H = os.path.dirname(os.path.abspath(__file__)); PR = "kinprev+prevzE10+ff10"; FL = "kin10+ff10"
POSE = [("A", "rl", "bzc", "기존 자세(그리퍼 아래)"), ("H", "rlH", "bzcH", "횡 방향 자세(그리퍼 앞)")]; LINE = [("z", "위아래 직선"), ("x", "팔 앞뒤 직선")]
def cell(r): return f"{r['d3_pk']:.1f} / {r['d3_rms']:.2f}" if r else "—"
def rows(p, pre, bz, k):
    out = []
    for tg, lab in (("h5", "턱 5 mm, 0.3 m/s"), ("base", "턱 10 mm, 0.3 m/s"), ("h15", "턱 15 mm, 0.3 m/s"), ("v15", "턱 10 mm, 0.15 m/s"), ("v50", "턱 10 mm, 0.5 m/s"), ("pot", "패인 바닥 10 mm, 0.3 m/s")):
        no, arm = S.one(f"{pre}_{k}_j23_{tg}", "none"), S.one(f"{pre}_{k}_j23_{tg}", PR)
        t2 = {"base": f"{bz}_{k}_b10", "pot": f"{bz}_{k}_pot"}.get(tg, f"fill_{p}_{k}_{tg}")
        out.append((lab, no, S.one(t2, "bz"), arm, S.one(t2, "bz+" + PR)))
    for v, lab in ((30, "평지 가감속, 0.3 m/s"), (50, "평지 가감속, 0.5 m/s")):
        tg = f"fl2_{k}_j23_v{v}" if p == "A" else (f"fill_H_{k}_flat30" if v == 30 else f"rlH_{k}_j23_flat")
        out.append((lab, S.one(tg, "none"), S.one(tg, "bz"), S.one(tg, FL), S.one(tg, "bz+" + FL)))
    return out
md = ["# 비교한 경우 전체 (2026-11-24, 시뮬레이션)", "",
      "값은 끝단이 있어야 할 자리에서 떨어진 거리(월드 기준, 3차원 전체)의 **최대 / 실효값 [mm]**. 베이스의 수평 이동은 빼고, 베이스가 들리고 기우는 몫은 넣는다.",
      "토크 제어 + 추종 제어기(2·3번 관절), 관절 강성 1100·500·800 N·m/rad, 끝단은 실측 직선 기록(0.25 Hz 왕복)을 따라간다. 턱·패인 바닥 장면은 출발 → 넘기 → 정지, 가감속 1 m/s².",
      "AMR 보정 = 속도 명령 ZVD 성형. 로봇팔 보정 = 자세 보상 + 뒷바퀴 예측(평지에서는 자세 보상 + 베이스 IMU로 잰 값의 보정). 턱 10 mm·0.3 m/s의 보정 없음과 로봇팔 보정만 세 번 돌렸고(여기는 첫 회의 값), 나머지는 한 번씩이다.", ""]
ALL = []
for p, pre, bz, pn in POSE:
    for k, kn in LINE:
        st = S.one(f"{pre}_{k}_j23_still", "none")
        md += [f"## {pn} — {kn}", "", f"AMR 정지(팔만 움직임): **{cell(st)}**", "", "| 외란 | 보정 없음 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 |", "|---|---|---|---|---|"]
        for lab, *rs in rows(p, pre, bz, k):
            md.append(f"| {lab} | " + " | ".join(cell(r) for r in rs) + " |"); ALL.append((p, k, lab, rs))
        md.append("")
n = sum(1 for *_, rs in ALL for r in rs if r); md += [f"채워진 칸: {n} / {4 * len(ALL)}", ""]
# 보정 없음 대비 비(턱·패인 바닥 여섯 조건 / 평지 둘)
md += ["## 보정 없음 대비 비 — 가운데 값(범위)", "", "| 자세·직선 | 외란 | 지표 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 |", "|---|---|---|---|---|---|"]
for p, pre, bz, pn in POSE:
    for k, kn in LINE:
        R = [x for x in ALL if x[0] == p and x[1] == k]
        for grp, sel in (("턱·패인 바닥(6)", lambda l: "평지" not in l), ("평지 가감속(2)", lambda l: "평지" in l)):
            for key, kl in (("d3_pk", "최대"), ("d3_rms", "실효값")):
                A = np.array([[r[key] / rs[0][key] for r in rs[1:]] for _, _, lab, rs in R if sel(lab) and all(rs)])
                md.append(f"| {pn.split('(')[0]}, {kn} | {grp} | {kl} | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(3)) + " |")
open(H + "/figs/matrix_1124.md", "w").write("\n".join(md) + "\n"); print(f"채워진 칸 {n} / {4 * len(ALL)} →", H + "/figs/matrix_1124.md")
