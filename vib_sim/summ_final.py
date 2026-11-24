"""두 자세의 직선 결과를 한 파일로(새 강성 1100·500·800, 2026-11-24) → figs/final_1124.md
입력: jobs_rl.txt(자세 A), jobs_rlH.txt·jobs_rlH2.txt(그리퍼가 앞을 보는 자세), jobs_flat2.txt. 월드 기준으로 직선에서 벗어난 거리[mm]."""
import io, contextlib, os
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import summ_line as S
H = os.path.dirname(os.path.abspath(__file__))
C = "none,kinmap,kinmap+map,kinprev+prevzE10+ff10".split(","); CN = ["대책 없음", "자세 보상만", "자세 + 지형 예견", "자세 + 뒷바퀴 예측"]
T = [("base", "턱 10 mm, 0.3 m/s"), ("h5", "턱 5 mm"), ("h15", "턱 15 mm"), ("v15", "0.15 m/s"), ("v50", "0.5 m/s"), ("pot", "패인 바닥 10 mm")]
MN = {"pos": "위치 제어(엔코더 재생)", "a3": "토크 제어 + 추종 제어기 J1–3", "j23": "토크 제어 + 추종 제어기 J2·J3"}
def seeds(pre, k, m, c):
    x = [S.one(f"{pre}_{k}_{m}_base", c), S.one(f"{pre}_{k}_{m}_base_s1", c)] + [S.one(f"{pre}_{k}_{m}_base_s{sd}", c) for sd in (2, 3)]
    return [r for r in x if r]
md = ["# 끝단 직선 중 출발 → 턱·패인 바닥 → 정지 — 두 자세 (2026-11-24, 시뮬레이션, 관절 강성 1100·500·800)", "",
      "월드 기준으로 직선에서 벗어난 거리[mm]. 실측 직선 기록을 재생(위치 제어 = 위치 모드 기록, 토크 제어 = 추종 제어기 기록의 관절 4–6과 기준). 베이스 자세는 참값. 기본 조건 밖은 한 번씩.", ""]
for pre, pn in (("rl", "자세 A"), ("rlH", "그리퍼가 앞을 보는 자세")):
    for k, kn in (("z", "위아래 직선 ±30 mm"), ("x", "팔 앞뒤 직선 ±20 mm")):
        md += [f"## {pn} — {kn}", "", "| 구동 | 베이스 정지: 실효값 | 흔들림 | 최대 |", "|---|---|---|---|"]
        for m in ("pos", "a3", "j23"):
            st = S.one(f"{pre}_{k}_{m}_still", "none")
            if st: md.append(f"| {MN[m]} | {st['w_rms']:.2f} | {st['w_vib']:.2f} | {st['w_pk']:.2f} |")
        md += ["", "턱 10 mm·0.3 m/s (씨앗마다), 최대 / 실효값:", "", "| 구동 | " + " | ".join(CN) + " |", "|" + "---|" * (len(CN) + 1)]
        for m in ("pos", "a3", "j23"):
            cells = []
            for c in C:
                x = seeds(pre, k, m, c); cells.append(("·".join(f"{r['w_pk']:.1f}" for r in x) + " / " + "·".join(f"{r['w_rms']:.2f}" for r in x)) if x else "—")
            md.append(f"| {MN[m]} | " + " | ".join(cells) + " |")
        md.append("")
        for key, lab in (("w_pk", "최대"), ("w_rms", "실효값"), ("w_vib", "흔들림(2–20 Hz)")):
            md += [f"### {lab} [mm] — 여섯 조건", "", "| 구동 | 조건 | " + " | ".join(CN) + " |", "|" + "---|" * (len(CN) + 2)]
            for m in ("pos", "j23"):
                A = []
                for tg, lb in T:
                    v = [S.one(f"{pre}_{k}_{m}_{tg}", c) for c in C]
                    if any(x is None for x in v): continue
                    v = [x[key] for x in v]; A.append([x / v[0] for x in v]); md.append(f"| {MN[m]} | {lb} | " + " | ".join(f"{x:.2f}" for x in v) + " |")
                if A:
                    A = np.array(A); md.append(f"| {MN[m]} | **비: 가운데 값(범위)** | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(len(C))) + " |")
            md.append("")
open(H + "/figs/final_1124.md", "w").write("\n".join(md) + "\n"); print(len(md), "줄 →", H + "/figs/final_1124.md")
