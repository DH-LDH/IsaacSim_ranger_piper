"""베이스 자세 보상(kin*) 조건 격자 요약 → figs/kin_tables.md  (입력: jobs_udk2.txt 의 실행, out/exp6_ts_udk2_*.npz)
위아래 동작(10/29 J2 기록) + 출발 → 턱·패인 바닥 → 정지. 값은 월드 기준으로 직선에서 벗어난 거리[mm]."""
import io, contextlib, os
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import summ_line as S
H = os.path.dirname(os.path.abspath(__file__))
C = "none,prevzE10+ff10+dampRJ,map+dampRJ,kinprev,kinmap,kinprev+prevzE10+ff10+dampRJ,kinmap+map+dampRJ,kinmap+map".split(",")
CN = ["대책 없음", "기존: 뒷바퀴 예측 + 피드백", "기존: 지형 예견 + 피드백", "자세 보상(뒷바퀴)", "자세 보상(지형)", "자세 + 뒷바퀴 예측 + 피드백", "자세 + 지형 예견 + 피드백", "자세 + 지형 예견"]
TAGS = [("base_a3_s1", "턱 10 mm, 0.3 m/s (씨앗 1)"), ("base_a3_s2", "같은 조건 (씨앗 2)"), ("base_a3_s3", "같은 조건 (씨앗 3)"), ("h5_a3", "턱 5 mm"), ("h15_a3", "턱 15 mm"), ("v15_a3", "0.15 m/s"), ("v50_a3", "0.5 m/s"), ("pot_a3", "패인 바닥 10 mm"),
        ("base_pos", "위치 제어: 턱 10 mm"), ("h5_pos", "위치 제어: 턱 5 mm"), ("h15_pos", "위치 제어: 턱 15 mm"), ("v15_pos", "위치 제어: 0.15 m/s"), ("v50_pos", "위치 제어: 0.5 m/s"), ("pot_pos", "위치 제어: 패인 바닥")]
md = ["# 베이스 자세 보상 — 조건 격자 (2026-11-24, 시뮬레이션)", "", "위아래 동작(2번 관절 ±5°, 실측 기록) 중 출발 → 턱·패인 바닥 → 정지. 월드 기준으로 직선에서 벗어난 거리[mm]. 위쪽 여덟 줄은 토크 제어 + 추종 제어기(J1–3), 아래 여섯 줄은 위치 제어(엔코더 재생, 모터가 보정을 그대로 따라감).",
      "자세 보상 = 베이스의 들림·기울기를 관절 1–3으로 되돌림(베이스 자세는 참값). 비는 대책 없음 대비, 씨앗 2·3을 뺀 여섯 조건의 가운데 값(범위).", ""]
for key, lab in (("w_pk", "최대"), ("w_rms", "실효값"), ("w_vib", "흔들림(2–20 Hz)")):
    md += [f"## {lab} [mm]", "", "| 조건 | " + " | ".join(CN) + " |", "|" + "---|" * (len(CN) + 1)]; acc = {"a3": [], "pos": []}
    for tg, lb in TAGS:
        row = [S.one("udk2_" + tg, c) for c in C]
        if row[0] is None: continue
        v = [r[key] if r else float("nan") for r in row]; md.append("| " + lb + " | " + " | ".join(f"{x:.2f}" for x in v) + " |")
        if "s2" not in tg and "s3" not in tg: acc["pos" if tg.endswith("pos") else "a3"].append([x / v[0] for x in v])
    for k, nm in (("a3", "**토크 제어: 비**"), ("pos", "**위치 제어: 비**")):
        A = np.array(acc[k]); md.append("| " + nm + " | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(len(C))) + " |")
    md.append("")
open(H + "/figs/kin_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md[:6]))

# ---- J1 추종 제어기를 끈 구성(추종 제어기는 J2·J3만), jobs_j23.txt → figs/j23_tables.md ----
C2 = "none,kinmap,kinmap+map,kinprev+prevzE10+ff10,kinmap+map+dampRJ,kinprev+prevzE10+ff10+dampRJ".split(",")
CN2 = ["대책 없음", "자세 보상만(지형)", "자세 + 지형 예견", "자세 + 뒷바퀴 예측", "자세 + 지형 예견 + 피드백", "자세 + 뒷바퀴 예측 + 피드백"]
T2 = [("base", "턱 10 mm, 0.3 m/s"), ("h5", "턱 5 mm"), ("h15", "턱 15 mm"), ("v15", "0.15 m/s"), ("v50", "0.5 m/s"), ("pot", "패인 바닥 10 mm")]
md = ["# J1 추종 제어기를 끈 구성 — 조건 격자 (2026-11-24, 시뮬레이션)", "", "토크 제어, 추종 제어기는 J2·J3만(J1은 내장 PD). 위아래 동작 중 출발 → 턱·패인 바닥 → 정지, 월드 기준으로 직선에서 벗어난 거리[mm]. 조건마다 한 번(기본 조건의 씨앗 셋은 ab_j23_s1–3).", ""]
for key, lab in (("w_pk", "최대"), ("w_rms", "실효값"), ("w_vib", "흔들림(2–20 Hz)")):
    md += [f"## {lab} [mm]", "", "| 조건 | " + " | ".join(CN2) + " |", "|" + "---|" * (len(CN2) + 1)]; A = []
    for tg, lb in T2:
        row = [S.one("j23_" + tg, c) for c in C2]
        if row[0] is None: continue
        v = [r[key] for r in row]; md.append("| " + lb + " | " + " | ".join(f"{x:.2f}" for x in v) + " |"); A.append([x / v[0] for x in v])
    if A:
        A = np.array(A); md.append("| **비: 가운데 값(범위)** | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(len(C2))) + " |")
    md.append("")
open(H + "/figs/j23_tables.md", "w").write("\n".join(md) + "\n")
