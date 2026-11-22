"""밤사이 조건 격자(out/exp6_*.csv) 정리 → figs/night_tables.md.
표본(n_*): 조건마다 '대책 없음 대비 비'를 내고 방식별로 중앙값·범위·몇 조건에서 0.8 아래인지."""
import csv, glob, os, statistics as S
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out/"
def rd(tag):
    p = O + f"exp6_{tag}.csv"
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []
def rat(rows, key="vec"):
    b = float(rows[0][key]); return {r["case"]: float(r[key]) / b for r in rows} if b > 0 else {}
NAME = {"ff10": "베이스 IMU 피드포워드(10 ms)", "ff10+damp10p": "③ 피드포워드 + 위상 예측 되먹임", "map": "① 지도 예견(정확)", "mapE3": "① 위치 3 mm 오차",
        "mapservo": "① 위치 모드 경로", "prev": "② 뒷바퀴 예측 그대로", "prevzE10+ff10": "②′ 닮은 성분만 예측 + 잰 값", "prevzE10+ff10+damp10p": "②′ + 위상 예측 되먹임"}
md = ["# 밤사이 조건 격자 (팔 뒷바퀴 축 위, 타이어 150 kN/m 기본, 끝단 진동의 대책 없음 대비 비)\n"]
tags = sorted(os.path.basename(p)[5:-4] for p in glob.glob(O + "exp6_n_*.csv"))
allr = {t: rd(t) for t in tags}; cases = list(NAME)
md.append(f"## 1. 표본 {len(tags)}조건에서의 방식별 효과\n\n| 방식 | 중앙값 | 최소–최대 | 0.8 아래인 조건 | 1.0을 넘는 조건 | 뒷바퀴 구간 중앙값 |\n|---|---|---|---|---|---|")
for c in cases:
    v = [rat(r)[c] for r in allr.values() if c in rat(r)]; vr = [rat(r, "rear")[c] for r in allr.values() if c in rat(r, "rear")]
    if v: md.append(f"| {NAME[c]} | {S.median(v):.2f} | {min(v):.2f}–{max(v):.2f} | {sum(x < 0.8 for x in v)} / {len(v)} | {sum(x > 1.0 for x in v)} / {len(v)} | {S.median(vr):.2f} |")
md.append("\n## 2. 조건별\n\n| 조건 | 대책 없음 [rad/s] (끄덕임 / 좌우 / 비틀림) | 베이스 az 최대 | " + " | ".join(c for c in cases) + " |\n|---|---|---|" + "---|" * len(cases))
for t in tags:
    r = allr[t]; b = r[0]; ra = rat(r)
    md.append(f"| {t[2:]} | {float(b['vec']):.3f} ({float(b['pitch']):.3f} / {float(b['yaw']):.3f} / {float(b['roll']):.3f}) | {float(b['base_az_peak']):.1f} | " + " | ".join(f"{ra[c]:.2f}" if c in ra else "—" for c in cases) + " |")
# 장착 방향
md.append("\n## 3. 팔 장착 방향 (턱 10 mm·0.3 m/s, 패인 바닥 10 mm)\n\n| 방향 | 장애물 | 대책 없음 [rad/s] (끄덕임 / 좌우 / 비틀림) | " + " | ".join(cases) + " |\n|---|---|---|" + "---|" * len(cases))
for lab, tag in (("옆 90°", "n_b_h10_v30"), ("앞 0°", "y_yaw0_bump"), ("뒤 180°", "y_yaw180_bump"), ("옆 90°", "n_p_d10_l6_v15"), ("앞 0°", "y_yaw0_pot"), ("뒤 180°", "y_yaw180_pot")):
    r = rd(tag)
    if r:
        b = r[0]; ra = rat(r); md.append(f"| {lab} | {'패인 바닥' if 'pot' in tag or '_p_' in tag else '턱'}{'(0.15 m/s)' if 'v15' in tag else ''} | {float(b['vec']):.3f} ({float(b['pitch']):.3f} / {float(b['yaw']):.3f} / {float(b['roll']):.3f}) | " + " | ".join(f"{ra[c]:.2f}" if c in ra else "—" for c in cases) + " |")
for y in (0, 180):
    p = O + f"exp2_yaw{y}.csv"
    if os.path.exists(p):
        r = list(csv.DictReader(open(p))); b = float(r[0]["stop_vec"])
        md.append(f"\n출발·정지(1 m/s², 강체 섀시) 장착 {y}°: 대책 없음 {b:.5f} rad/s (비틀림 {float(r[0]['stop_roll']):.4f}, 끄덕임 {float(r[0]['stop_pitch']):.4f}, 좌우 {float(r[0]['stop_yaw']):.4f}), 주로 {r[0]['axis']} {float(r[0]['f']):.1f} Hz, " + ", ".join(f"{x['case']} {float(x['stop_vec']) / b:.2f}" for x in r[1:]))
# 보정 분해능·불감대
md.append("\n## 4. 보정각의 불감대·분해능 (턱 10 mm·0.3 m/s — 모터 쪽이 작은 보정에는 안 움직인다고 볼 때)\n\n| 조건 | " + " | ".join(cases) + " |\n|---|" + "---|" * len(cases))
for lab, tag in (("제약 없음", "n_b_h10_v30"), ("불감대 0.02°", "q_dead002"), ("불감대 0.05°", "q_dead005"), ("불감대 0.10°", "q_dead010"), ("불감대 0.20°", "q_dead020"), ("분해능 0.01°", "q_step001"), ("분해능 0.05°", "q_step005"), ("분해능 0.10°", "q_step010"), ("작은 턱(5 mm·0.15 m/s) 제약 없음", "n_b_h5_v15"), ("작은 턱 불감대 0.05°", "q_dead005_small")):
    r = rd(tag)
    if r: ra = rat(r); md.append(f"| {lab} | " + " | ".join(f"{ra[c]:.2f}" if c in ra else "—" for c in cases) + " |")
# 속도
md.append("\n## 5. 넘는 속도에 따른 끝단 진동 (대책 없음, rad/s) — 턱 앞에서 속도를 줄이는 것과 견주기\n\n| 턱 높이 | 0.1 m/s | 0.15 | 0.3 | 0.5 | 0.3 m/s에서 ③ 적용 |\n|---|---|---|---|---|---|")
for h in (5, 10):
    vals = []
    for tag in (f"v_h{h}_v10", f"n_b_h{h}_v15", f"n_b_h{h}_v30", f"n_b_h{h}_v50"):
        r = rd(tag); vals.append(f"{float(r[0]['vec']):.3f}" if r else "—")
    r = rd(f"n_b_h{h}_v30"); a3 = f"{float(r[0]['vec']) * rat(r)['ff10+damp10p']:.3f}" if r else "—"
    md.append(f"| {h} mm | " + " | ".join(vals) + f" | {a3} |")
open(H + "/figs/night_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
