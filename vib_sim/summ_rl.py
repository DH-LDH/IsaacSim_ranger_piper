"""직선 실측 기록(11/24, piper_ws docs/data/raw/line)으로 돌린 조건 격자 요약 → figs/rl_tables.md   (실행 목록 jobs_rl.txt)
위치 제어 = 위치 모드 기록을 모터 쪽에 재생. 토크 제어 = 추종 제어기 기록(asmc3)의 관절 4–6과 기준을 재생하고 관절 1–3은 모형(a3: 추종 제어기 J1–3, j23: J2·J3만)."""
import io, contextlib, os
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import summ_line as S
H = os.path.dirname(os.path.abspath(__file__))
C = "none,kinmap,kinmap+map,kinprev+prevzE10+ff10".split(","); CN = ["대책 없음", "자세 보상만(지형)", "자세 + 지형 예견", "자세 + 뒷바퀴 예측"]
T = [("base", "턱 10 mm, 0.3 m/s"), ("h5", "턱 5 mm"), ("h15", "턱 15 mm"), ("v15", "0.15 m/s"), ("v50", "0.5 m/s"), ("pot", "패인 바닥 10 mm")]
MN = {"pos": "위치 제어(엔코더 재생)", "a3": "토크 제어 + 추종 제어기 J1–3", "j23": "토크 제어 + 추종 제어기 J2·J3"}
md = ["# 끝단 직선(실측 기록) 중 출발 → 턱·패인 바닥 → 정지 — 조건 격자 (2026-11-24, 시뮬레이션)", "", "월드 기준으로 직선에서 벗어난 거리[mm]. 베이스 자세는 참값. 기본 조건 밖은 한 번씩 돌린 값.", ""]
for k, kn in (("z", "위아래 직선 ±30 mm"), ("x", "팔 앞뒤 직선 ±20 mm")):
    md += [f"## {kn}", "", "| 구동 | 베이스 정지: 실효값 | 흔들림 | 최대 |", "|---|---|---|---|"]
    for m in ("pos", "a3", "j23"):
        st = S.one(f"rl_{k}_{m}_still", "none")
        if st: md.append(f"| {MN[m]} | {st['w_rms']:.2f} | {st['w_vib']:.2f} | {st['w_pk']:.2f} |")
    md.append("")
    for key, lab in (("w_pk", "최대"), ("w_rms", "실효값"), ("w_vib", "흔들림(2–20 Hz)")):
        md += [f"### {lab} [mm]", "", "| 구동 | 조건 | " + " | ".join(CN) + " |", "|" + "---|" * (len(CN) + 2)]
        for m in ("pos", "a3", "j23"):
            A = []
            for tg, lb in T:
                v = [S.one(f"rl_{k}_{m}_{tg}", c) for c in C]
                if v[0] is None: continue
                v = [x[key] for x in v]; A.append([x / v[0] for x in v]); md.append(f"| {MN[m]} | {lb} | " + " | ".join(f"{x:.2f}" for x in v) + " |")
            if A:
                A = np.array(A); md.append(f"| {MN[m]} | **비: 가운데 값(범위)** | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(len(C))) + " |")
        md.append("")
    md += ["### 씨앗 셋(턱 10 mm), 최대 / 실효값 [mm]", "", "| 구동 | " + " | ".join(CN) + " |", "|" + "---|" * (len(CN) + 1)]
    for m in ("a3", "j23"):
        cells = []
        for c in C:
            x = [S.one(f"rl_{k}_{m}_base", c)] + [S.one(f"rl_{k}_{m}_base_s{sd}", c) for sd in (2, 3)]; x = [r for r in x if r]
            cells.append("·".join(f"{r['w_pk']:.1f}" for r in x) + " / " + "·".join(f"{r['w_rms']:.2f}" for r in x))
        md.append(f"| {MN[m]} | " + " | ".join(cells) + " |")
    md.append("")
open(H + "/figs/rl_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md[:5]))

# ---- 평지 출발·정지(턱 없음, 가감속 1 m/s^2), jobs_flat2.txt → figs/flat2_tables.md ----
CF = "none,bz,kin10+ff10,bz+kin10+ff10".split(","); CFN = ["대책 없음", "AMR 속도 명령 성형", "팔 쪽: 자세 보상 + IMU 보정(10 ms)", "성형 + 팔 쪽"]
md = ["# 팔이 움직이는 동안 평지 출발 → 정지 (2026-11-24, 시뮬레이션)", "", "월드 기준으로 직선에서 벗어난 거리[mm]. 가감속 1 m/s². 토크 제어는 추종 제어기 J2·J3. 한 번씩 돌린 값. 지형 예견은 평지에 해당 없음(턱 둘레의 지도라서).", ""]
for key, lab in (("w_pk", "최대"), ("w_rms", "실효값"), ("w_vib", "흔들림(2–20 Hz)")):
    md += [f"## {lab} [mm]", "", "| 동작 | 구동 | 속도 | " + " | ".join(CFN) + " |", "|" + "---|" * (len(CFN) + 3)]
    for k, kn in (("ud", "위아래 동작"), ("z", "위아래 직선"), ("x", "팔 앞뒤 직선")):
        for m in ("pos", "j23"):
            for v in (30, 50):
                r = [S.one(f"fl2_{k}_{m}_v{v}", c) for c in CF]
                if r[0] is None: continue
                md.append(f"| {kn} | {MN[m]} | 0.{v // 10} m/s | " + " | ".join(f"{x[key]:.2f}" if x else "—" for x in r) + " |")
    md.append("")
open(H + "/figs/flat2_tables.md", "w").write("\n".join(md) + "\n")
