"""MIT 모터 쪽 모형을 넣은 이동 베이스 결과 요약 → figs/task5_tables.md
입력: out/exp6_tk5_s{1,2,3}_{pos,pd,tde,tde3,asmc3}.csv, tk5r_*(AMR 정지), fl5_v{30,50}_*(평지), tq5_{base,h5,h15,v15,v50,pot}_*(턱·패인 바닥)"""
import csv, os, statistics as st
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"
MODE = [("pos", "위치 제어(엔코더 재생)"), ("pd", "MIT 펌웨어 PD"), ("tde", "MIT + 추종 제어(J2)"), ("tde3", "MIT + 추종 제어(J1–3)"), ("asmc3", "MIT + 적응 슬라이딩(J1–3)")]
CASE = [("none", "대책 없음"), ("bz", "속도 성형"), ("bz+dampRJ", "성형 + 끝단 되먹임"), ("bz+prevzE10+ff10+dampRJ", "성형 + 뒷바퀴 예측 + 되먹임"), ("bz+map+dampRJ", "성형 + 지형 예견 + 되먹임")]
def load(tag):
    p = f"{O}/exp6_{tag}.csv"
    return {r["case"]: r for r in csv.DictReader(open(p))} if os.path.exists(p) else None
md = ["# MIT 모터 쪽 모형을 넣은 이동 베이스 결과 (exp6_bump.py --mit, 2026-11-23)", "",
      "끝단 각속도는 2–20 Hz 세 축 크기 rms[rad/s], 흔들림은 끝단 궤적 이탈의 2–20 Hz 성분[mm], 오차는 궤적 이탈 전체 rms[mm]. 비는 같은 동작의 위치 제어·대책 없음 대비.", ""]
for s, nm in ((1, "한 관절(J2)"), (2, "두 관절(J1·J2)"), (3, "세 관절(J1–3)")):
    base = load(f"tk5_s{s}_pos"); b = base["none"] if base else None
    md += [f"## 종합 장면 — 팔 동작 {nm}, 출발 → 턱 → 정지", "", "| 구동 방식 | 대책 | 끝단 각속도 | 비 | 끄덕임 / 좌우 | 흔들림 [mm] | 비 | 궤적 오차 [mm] | 비 | 최대 [mm] | 관절 추종 오차 J1·J2·J3 [°] |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for m, mn in MODE:
        R = load(f"tk5_s{s}_{m}"); ref = load(f"tk5r_s{s}_{m}")
        if ref:
            r = ref["none"]; f = lambda k: float(r[k])
            md.append(f"| {mn} | (AMR 정지, 팔만) | {f('full_vec'):.4f} | {f('full_vec') / float(b['full_vec']):.2f} | {f('full_pitch'):.4f} / {f('full_yaw'):.4f} | {f('tip_vib_mm'):.2f} | {f('tip_vib_mm') / float(b['tip_vib_mm']):.2f} | {f('tip_err_mm'):.2f} | {f('tip_err_mm') / float(b['tip_err_mm']):.2f} | {f('tip_peak_mm'):.2f} | {f('err_j1'):.3f} · {f('err_j2'):.3f} · {f('err_j3'):.3f} |")
        if not R: continue
        for c, cn in CASE:
            if c not in R: continue
            r = R[c]; f = lambda k: float(r[k])
            md.append(f"| {mn} | {cn} | {f('full_vec'):.4f} | {f('full_vec') / float(b['full_vec']):.2f} | {f('full_pitch'):.4f} / {f('full_yaw'):.4f} | {f('tip_vib_mm'):.2f} | {f('tip_vib_mm') / float(b['tip_vib_mm']):.2f} | {f('tip_err_mm'):.2f} | {f('tip_err_mm') / float(b['tip_err_mm']):.2f} | {f('tip_peak_mm'):.2f} | {f('err_j1'):.3f} · {f('err_j2'):.3f} · {f('err_j3'):.3f} |")
    md.append("")
md += ["## 평지 출발·정지 (팔은 자세 유지)", "", "| 속도 | 구동 방식 | " + " | ".join(x for x in ("대책 없음 [rad/s]", "속도 성형", "끝단 되먹임", "성형 + 되먹임", "베이스 IMU 보정", "성형 + IMU 보정")) + " | 대책 없음: 위치 제어 대비 | J1이 미끄러진 시간 비율 |", "|---|---|---|---|---|---|---|---|---|---|"]
for v in (30, 50):
    bp = load(f"fl5_v{v}_pos")
    for m, mn in MODE[:4]:
        R = load(f"fl5_v{v}_{m}")
        if not R: continue
        b = R["none"]; g = lambda c: f"{float(R[c]['full_vec']) / float(b['full_vec']):.2f}" if c in R else "—"
        md.append(f"| 0.{v // 10} m/s | {mn} | {float(b['full_vec']):.4f} | {g('bz')} | {g('dampRJ')} | {g('bz+dampRJ')} | {g('ff10')} | {g('bz+ff10')} | {float(b['full_vec']) / float(bp['none']['full_vec']):.2f} | {b.get('slip_j1', '') if m != 'pos' else '—'} |")
md += ["", "## 턱·패인 바닥 여섯 조건 (팔은 자세 유지) — 대책 없음 대비 끝단 각속도 비, 가운데 값(범위)", "", "| 구동 방식 | 대책 없음: 위치 제어 대비 | 끝단 되먹임 | 베이스 IMU 보정 | 뒷바퀴 예측 + 되먹임 | 지형 예견 + 되먹임 | J1·J2·J3이 미끄러진 시간 비율(가운데) |", "|---|---|---|---|---|---|---|"]
CN = ("base", "h5", "h15", "v15", "v50", "pot")
for m, mn in MODE[:4]:
    Rs = [(load(f"tq5_{c}_{m}"), load(f"tq5_{c}_pos")) for c in CN]; Rs = [(a_, b_) for a_, b_ in Rs if a_ and b_]
    if not Rs: continue
    def col(c):
        x = [float(a_[c]["vec"]) / float(a_["none"]["vec"]) for a_, _ in Rs if c in a_]
        return f"{st.median(x):.2f} ({min(x):.2f}–{max(x):.2f})" if x else "—"
    x0 = [float(a_["none"]["vec"]) / float(b_["none"]["vec"]) for a_, b_ in Rs]
    sl = " · ".join((f"{st.median([float(a_['none'][f'slip_j{j}']) for a_, _ in Rs]):.2f}" if m != "pos" else "—") for j in (1, 2, 3))
    md.append(f"| {mn} ({len(Rs)}조건) | {st.median(x0):.2f} ({min(x0):.2f}–{max(x0):.2f}) | {col('dampRJ')} | {col('ff10')} | {col('prevzE10+ff10+dampRJ')} | {col('map+dampRJ')} | {sl} |")
open(H + "/figs/task5_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
