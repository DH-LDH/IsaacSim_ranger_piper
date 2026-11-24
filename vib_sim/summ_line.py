"""끝단 직선 왕복 장면(run_line.sh) 요약 → figs/line_tables.md

두 가지 기준으로 "직선에서 벗어난 거리"를 낸다.
  팔 몫   : J_v (q_링크 − q_기준) 가운데 직선에 수직인 성분. 팔의 탄성 처짐 + 관절 추종 오차 — 지금까지의 궤적 오차와 같은 정의(베이스에 실린 좌표).
  월드 기준: 끝단의 실제 월드 위치에서 베이스의 수평 이동만 빼고 기준 직선과 견준 것. 베이스가 턱에서 들리고 기우는 몫(강체 운동)까지 들어간다.
입력: out/exp6_ts_ln_<방향>_<구동>[r]_<조건>.npz (exp6_bump.py --keep)
"""
import glob, math, os
import numpy as np
from gen_line import fk, load_chain

H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; CH = load_chain()
RM = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])      # 팔 장착 90도: 팔 베이스 좌표 → 월드


def band(x, fs, lo, hi):
    X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), 1.0 / fs); X[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(X, len(x))


def perp(V, d):
    return V - np.outer(V @ d, d)


def one(tag, case):
    p = f"{O}/exp6_ts_{tag}_{case}.npz"
    if not os.path.exists(p): return None
    z = np.load(p); t = z["t"]; T = t[-1] + 0.001; m = (t >= 0.1) & (t < T - 0.05); fs = 1000.0
    if "mf" in z.files: m = z["mf"].astype(bool)                       # MIT 기록의 순간 튐 구간을 뺀 창(exp6_bump.py가 저장)
    pr = np.array([fk(CH, q) for q in z["qd"][::5]]); dref = np.repeat((pr - pr[0]) @ RM.T, 5, axis=0)[:len(t)]
    d = dref[np.argmax(np.linalg.norm(dref, axis=1))]; d = d / np.linalg.norm(d)                       # 직선 방향(월드)
    te = z["te"]; cp = perp(te, d)                                                                       # 팔 몫
    w = (z["ep"] - z["ep"][0]) - dref; w[:, :2] -= (z["bp"] - z["bp"][0])[:, :2]; cw = perp(w, d)        # 월드 기준
    rms = lambda V: float(np.sqrt(np.mean(np.sum(V[m] ** 2, axis=1)))) * 1e3
    pk = lambda V: float(np.max(np.linalg.norm(V[m], axis=1))) * 1e3
    vib = lambda V: float(np.sqrt(sum(np.mean(band(V[:, c], fs, 2.0, 20.0)[m] ** 2) for c in range(3)))) * 1e3
    tf, tr = float(z["tf"]), float(z["tr"])
    ax = {f"{c}_rms": float(np.sqrt(np.mean(w[m, i] ** 2))) * 1e3 for i, c in enumerate("xyz")}; ax.update({f"{c}_pk": float(np.max(np.abs(w[m, i]))) * 1e3 for i, c in enumerate("xyz")})
    # d3_* = 3차원 전체 거리(끝단이 있어야 할 자리에서 떨어진 거리) — 11/24부터 원고의 주 지표. w_* = 직선에 수직인 성분만(앞의 지표). x·y·z = 월드 축별(x 진행 방향, y 옆, z 위아래)
    return dict(arm_rms=rms(cp), arm_pk=pk(cp), arm_vib=vib(cp), arm_along=float(np.sqrt(np.mean((te @ d)[m] ** 2))) * 1e3, w_rms=rms(cw), w_pk=pk(cw), w_vib=vib(cw), d3_rms=rms(w), d3_pk=pk(w), d3_vib=vib(w), along_rms=float(np.sqrt(np.mean((w @ d)[m] ** 2))) * 1e3, **ax,
                heave=float(np.max(np.abs(z["bp"][m, 2] - z["bp"][0, 2]))) * 1e3, tf=tf, tr=tr)


CASE = [("none", "대책 없음"), ("dampRJ", "끝단 피드백"), ("ff10", "베이스 IMU 보정"), ("prevzE10+ff10+dampRJ", "뒷바퀴 예측 + 피드백"), ("map+dampRJ", "지형 예견 + 피드백"),
        ("bz", "(베이스 속도 성형만)"), ("bz+prevzE10+ff10+dampRJ", "속도 성형 + 뒷바퀴 예측 + 피드백")]
DIRN = [("y", "진행 방향 직선(±30 mm, 주로 J1)"), ("x", "팔 앞뒤 직선(±20 mm, J2·J3)"), ("z", "위아래 직선(±30 mm, 주로 J2)")]
def main():
    md = ["# 끝단이 직선을 왕복하는 동안 출발 → 10 mm 턱 → 정지 (2026-11-24, 시뮬레이션)", "",
          "기준 궤적은 만든 것이다(gen_line.py, 0.25 Hz 왕복, 관절 ±5° 안). 구동은 토크 제어(MIT) 모드 모형, 베이스 0.3 m/s. 값은 직선에서 벗어난 거리[mm](직선에 수직인 성분).",
          "팔 몫 = 탄성 처짐 + 추종 오차(베이스에 실린 좌표), 월드 기준 = 베이스가 들리고 기우는 몫까지. 흔들림은 2–20 Hz 성분.", ""]
    ROWS = {}
    for k, nm in DIRN:
        md += [f"## {nm}", "", "| 구동 | 팔 쪽 대책 | 팔 몫: 실효값 | 흔들림 | 최대 | 월드 기준: 실효값 | 흔들림 | 최대 |", "|---|---|---|---|---|---|---|---|"]
        for tag, mn, cases in ((f"ln_{k}_pdr", "내장 PD만", [("none", "(베이스 정지)")]), (f"ln_{k}_pd", "내장 PD만", [("none", "대책 없음")]),
                               (f"ln_{k}_a3r", "추종 제어기(J1–3)", [("none", "(베이스 정지)")]), (f"ln_{k}_a3", "추종 제어기(J1–3)", CASE)):
            for c, cn in cases:
                r = one(tag, c)
                if r is None: continue
                ROWS[(tag, c)] = r
                md.append(f"| {mn} | {cn} | {r['arm_rms']:.2f} | {r['arm_vib']:.2f} | {r['arm_pk']:.2f} | {r['w_rms']:.2f} | {r['w_vib']:.2f} | {r['w_pk']:.2f} |")
        md.append("")
    # 실측 기록이 있는 위아래 동작(2번 관절 ±5°, 10/29 레인저 미니 위 위치 제어 기록) — 호이지만 ±32 mm 구간에서 직선과 1.4 mm 차이. 위치 제어(엔코더 재생)와 견줄 수 있다
    md += ["## 위아래 동작 — 실측 기록(2번 관절 ±5°, 0.25 Hz)", "", "위치 제어는 실측 엔코더를 모터 쪽에 재생한 것이라, 팔 쪽 보정은 모터가 그대로 따라간다고 본 이상적인 경우다.", "",
           "| 구동 | 팔 쪽 대책 | 팔 몫: 실효값 | 흔들림 | 최대 | 월드 기준: 실효값 | 흔들림 | 최대 |", "|---|---|---|---|---|---|---|---|"]
    for tag, mn, cases in (("ud_posr", "위치 제어(엔코더 재생)", [("none", "(베이스 정지)")]), ("ud_pos", "위치 제어(엔코더 재생)", CASE), ("ud_pdr", "내장 PD만", [("none", "(베이스 정지)")]), ("ud_pd", "내장 PD만", [("none", "대책 없음")]),
                           ("ud_a3r", "추종 제어기(J1–3)", [("none", "(베이스 정지)")]), ("ud_a3", "추종 제어기(J1–3)", CASE)):
        for c, cn in cases:
            r = one(tag, c)
            if r is None: continue
            ROWS[(tag, c)] = r
            md.append(f"| {mn} | {cn} | {r['arm_rms']:.2f} | {r['arm_vib']:.2f} | {r['arm_pk']:.2f} | {r['w_rms']:.2f} | {r['w_vib']:.2f} | {r['w_pk']:.2f} |")
    md.append("")
    os.makedirs(H + "/figs", exist_ok=True); open(H + "/figs/line_tables.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
    h = [r["heave"] for (tg, c), r in ROWS.items() if not tg.endswith("r")]
    if h: print(f"\n베이스 원점이 들린 최대 높이: {min(h):.1f}–{max(h):.1f} mm")


if __name__ == "__main__":
    main()
