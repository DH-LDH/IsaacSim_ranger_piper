"""턱·패인 바닥 실험 표. 입력 out/exp5_r_*.csv → figs/bump_tables.md"""
import csv, os
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out/"
def rd(tag):
    p = O + f"exp5_{tag}.csv"
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []
def ratio(rows, key="vec"):
    b = float(rows[0][key]); return {r["case"]: float(r[key]) / b for r in rows}
md = ["# 턱·패인 바닥 넘기 (팔은 뒷바퀴 축 위 −0.23 m, 타이어 150 kN/m, 끝단 진동의 대책 없음 대비 비)\n"]
m, p, d = rd("r_methods"), rd("r_pot"), rd("r_delay")
if m:
    b = m[0]; md.append(f"대책 없음(턱 10 mm·0.3 m/s): 끝단 {float(b['vec']):.3f} rad/s (끄덕임 {float(b['pitch']):.3f}, 좌우 {float(b['yaw']):.3f}, 비틀림 {float(b['roll']):.3f}), 베이스 상하 가속도 최대 {float(b['base_az_peak']):.1f} m/s², 앞·뒤 상하 가속도 닮음 {float(b['az_corr']):.2f}\n")
    ra, rr, rf = ratio(m), ratio(m, "rear"), ratio(m, "front"); pa = ratio(p) if p else {}; pr = ratio(p, "rear") if p else {}
    md.append("## 세 방식\n\n| 조건 | 턱: 전체 | 앞바퀴 구간 | 뒷바퀴 구간 | 패인 바닥: 전체 | 뒷바퀴 구간 |\n|---|---|---|---|---|---|")
    for c in [x["case"] for x in m][1:]:
        md.append(f"| {c} | {ra[c]:.2f} | {rf[c]:.2f} | {rr[c]:.2f} | " + (f"{pa[c]:.2f} | {pr[c]:.2f} |" if c in pa else "— | — |"))
    if p: md.append(f"\n패인 바닥 대책 없음: 끝단 {float(p[0]['vec']):.3f} rad/s, 앞·뒤 닮음 {float(p[0]['az_corr']):.2f}")
if d:
    ra = ratio(d); md.append("\n## 지연별\n\n| 지연 | 베이스 IMU 피드포워드 | 끝단 되먹임(예측 없음) | 끝단 되먹임 + 위상 예측 |\n|---|---|---|---|")
    for n in ("0", "5", "10", "20", "36", "40"):
        md.append(f"| {n} ms | " + " | ".join(f"{ra[k]:.2f}" if k in ra else "—" for k in (f"ff{n}", f"damp{n}", f"damp{n}p")) + " |")
    md.append("\n" + ", ".join(f"{k} {ra[k]:.2f}" for k in ("ffservo", "ff10+damp20p", "ff20+damp36p") if k in ra))
md.append("\n## 턱 크기·속도별\n\n| 턱 높이 | 속도 | 베이스 상하 가속도 최대 | 끝단 진동 [rad/s] (끄덕임 / 좌우 / 비틀림) | ③ ff10+damp10p | ① map | ②′ prevzE10+ff10 (뒷바퀴 구간) | 앞·뒤 닮음 |\n|---|---|---|---|---|---|---|---|")
for h in ("0.003", "0.005", "0.01"):
    for v in ("0.15", "0.3"):
        g = rd(f"r_g_h{h}_v{v}")
        if not g: continue
        b = g[0]; ra, rr = ratio(g), ratio(g, "rear")
        md.append(f"| {float(h) * 1000:g} mm | {v} m/s | {float(b['base_az_peak']):.1f} | {float(b['vec']):.3f} ({float(b['pitch']):.3f} / {float(b['yaw']):.3f} / {float(b['roll']):.3f}) | {ra['ff10+damp10p']:.2f} | {ra['map']:.2f} | {ra['prevzE10+ff10']:.2f} ({rr['prevzE10+ff10']:.2f}) | {float(b['az_corr']):.2f} |")
open(H + "/figs/bump_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
