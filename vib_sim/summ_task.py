"""종합 시나리오(출발 → 턱 → 정지 + 팔 동작) 표: out/exp6_tk2*.csv, exp6_tk_s3_*.csv, exp6_tkr_hold*.csv → figs/task_tables.md"""
import csv, os
H = os.path.dirname(os.path.abspath(__file__))
def rd(tag):
    p = f"{H}/out/exp6_{tag}.csv"
    return {r["case"]: {k: (float(v) if k not in ("case", "replay", "bshape", "pose") and v not in ("", "nan") else v) for k, v in r.items()} for r in csv.DictReader(open(p))} if os.path.exists(p) else None
NM = {"none": "대책 없음", "bz": "속도 명령 성형(ZVD)만", "bz+dampRJ": "성형 + 감쇠만 되먹임", "bz+dampJ": "성형 + 위상 예측 되먹임", "bz+ff10": "성형 + 베이스 IMU 피드포워드",
      "bz+map": "성형 + 지형 예견", "bz+map+dampRJ": "성형 + 지형 예견 + 감쇠만", "bz+prevzE10+ff10+dampRJ": "성형 + 뒷바퀴 예측 + 감쇠만"}
out = ["# 종합 시나리오: AMR 출발 → 턱 → 정지 + 팔 동작 (exp6_bump.py --stop)", "",
       "전체 구간(0.1 s부터 정지 뒤까지) 값. 각속도 = 끝단 자이로 세 축 2–20 Hz 벡터 rms[rad/s], 흔들림 = 끝단 위치가 기준 궤적에서 벗어난 양의 2–20 Hz 성분[mm], 오차 = 벗어난 양 전체 rms[mm]. 괄호는 대책 없음 대비 비.", ""]
for ctl, cn in (("pos", "위치 모드 기록"), ("tde", "하이브리드 기록(참고)"), ("asmc", "ASMC16 기록(참고)")):
    out += [f"## 팔 = {cn}, 턱 10 mm, 0.3 m/s", "", "| 팔 동작 | 조건 | 각속도 | 그중 끄덕임 / 좌우 | 가속도 2–10 Hz | 10–20 Hz | 흔들림 [mm] | 오차 [mm] | 최대 [mm] |", "|---|---|---|---|---|---|---|---|---|"]
    for n in "123":
        ref, R = rd(f"tk2r_s{n}_{ctl}"), rd(f"tk2_s{n}_{ctl}")
        if not R: continue
        if ref:
            r = ref["none"]; out.append(f"| {n}축 | 팔만 (AMR 정지) | {r['full_vec']:.4f} | {r['full_pitch']:.3f} / {r['full_yaw']:.3f} | {r['full_acc_lo']:.2f} | {r['full_acc_hi']:.2f} | {r['tip_vib_mm']:.2f} | {r['tip_err_mm']:.2f} | {r['tip_peak_mm']:.2f} |")
        b = R["none"]
        for c, r in R.items():
            q = lambda k: f"{r[k]:.2f} ({r[k] / b[k]:.2f})"
            out.append(f"| {n}축 | {NM.get(c, c)} | {r['full_vec']:.4f} ({r['full_vec'] / b['full_vec']:.2f}) | {r['full_pitch']:.3f} / {r['full_yaw']:.3f} | {q('full_acc_lo')} | {q('full_acc_hi')} | {q('tip_vib_mm')} | {r['tip_err_mm']:.2f} | {r['tip_peak_mm']:.2f} |")
    out.append("")
out += ["## 조건을 바꾼 것 (3축, 위치 모드 기록)", "", "| 조건 | 제어 | 각속도 | 가속도 2–10 Hz | 10–20 Hz | 흔들림 [mm] | 오차 [mm] |", "|---|---|---|---|---|---|---|"]
for tag, nm in (("tk2_s3_h5", "턱 5 mm"), ("tk2_s3_pot", "패인 바닥 10 mm × 60 mm"), ("tk2_s3_v15", "턱 10 mm, 0.15 m/s"), ("tk2_s3_v50", "턱 10 mm, 0.5 m/s")):
    R = rd(tag)
    if not R: continue
    b = R["none"]
    for c in ("none", "bz+dampRJ", "bz+prevzE10+ff10+dampRJ", "bz+map+dampRJ"):
        r = R[c]; q = lambda k: f"{r[k]:.2f} ({r[k] / b[k]:.2f})"
        out.append(f"| {nm} | {NM[c]} | {r['full_vec']:.4f} ({r['full_vec'] / b['full_vec']:.2f}) | {q('full_acc_lo')} | {q('full_acc_hi')} | {q('tip_vib_mm')} | {r['tip_err_mm']:.2f} |")
open(H + "/figs/task_tables.md", "w").write("\n".join(out) + "\n"); print("\n".join(out))
