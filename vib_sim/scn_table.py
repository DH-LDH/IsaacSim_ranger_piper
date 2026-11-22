"""시나리오 기록(piper_ws docs/data/raw/scn)의 실측 |gyro| 5–10 Hz와 엔코더 재생 결과를 나란히. 출력: figs/scn_table.md"""
import csv, glob, gzip, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.expanduser("~/piper_ws/docs/data/raw")
def band(x, fs, lo, hi):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)
def real_gyro(stem):
    t0 = float(open(f"{R}/scn/{stem}.t0").read())
    with gzip.open(f"{R}/scn/{stem}.csv.gz", "rt") as f:
        r = list(csv.DictReader(f))
    ta = np.array([float(x["t"]) for x in r]); T = ta[-1] - ta[0]
    with gzip.open(f"{R}/scn/{stem}.imu.csv.gz", "rt") as f:
        g = [(float(x["t"]) - t0 - ta[0], float(x["x"]), float(x["y"]), float(x["z"])) for x in csv.DictReader(f) if x["stream"] == "g"]
    g = np.array(g); tu = np.arange(0.5, T - 0.5, 1 / 400.0)
    out = {}
    for lo, hi, k in ((5, 10, "5_10"), (2, 20, "2_20")):
        B = np.stack([band(np.interp(tu, g[:, 0], g[:, i]), 400.0, lo, hi) for i in (1, 2, 3)], axis=1)
        out[k] = float(np.sqrt(np.sum(np.mean(B ** 2, axis=0)))); out[k + "_ax"] = np.sqrt(np.mean(B ** 2, axis=0))
    return out
meta = {r["stem"]: r for r in csv.DictReader(open(R + "/index.csv")) if r["set"] == "scn"}
sim = {}
for p in glob.glob(H + "/out/exp3_replay_scn_*.csv"):
    for r in csv.DictReader(open(p)):
        sim[(r["arm"], r["base"])] = r
order = ["110051_slow_j2_s1", "110813_sine", "111226_sine", "204150_slow_j12_sxy", "203040_sine", "112534_sine", "204306_slow_j123_s3", "205503_sine", "205725_sine"]
name = {"pos": "위치 모드", "tde": "하이브리드(TDE J2)", "asmc16": "ASMC16(J2)"}
md = ["| 시나리오 | 제어 | 실측 5–10 Hz (카메라) | 재생 고정 베이스 ×1.27 | 비 | AMR 정지 2–20 Hz | AMR 이동 | 늘어남 | 이동 + ZVD | 늘어남 |", "|---|---|---|---|---|---|---|---|---|---|"]
rows = []
for o in order:
    stem = "20261029_" + o; m = meta[stem]; rg = real_gyro(stem)
    scn = m["cond"].split(" ")[0]; f = sim[(stem, "fixed")]; s, mv, z = (sim[(stem, b)] for b in ("still", "move", "zvd"))
    fx = float(f["move_vec_5_10"]) * 1.27; a, b, c = float(s["move_vec_2_20"]), float(mv["move_vec_2_20"]), float(z["move_vec_2_20"])
    md.append(f"| {scn} | {name[m['ctrl']]} | {rg['5_10']:.4f} | {fx:.4f} | {fx / rg['5_10']:.2f} | {a:.4f} | {b:.4f} | {b / a - 1:+.0%} | {c:.4f} | {c / a - 1:+.0%} |")
    rows.append((scn, m["ctrl"], rg["5_10"], fx, a, b, c, float(mv["yaw"]), float(s["yaw"]), float(s["pitch"])))
open(H + "/figs/scn_table.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
print("\n좌우 축(2–20 Hz) AMR 정지 → 이동:", "; ".join(f"{r[0]} {r[1]} {r[8]:.4f}→{r[7]:.4f}" for r in rows))
pos = {r[0]: r for r in rows if r[1] == "pos"}
print("위치 모드 대비 비 (실측 5–10 / 재생 고정 5–10 / 재생 AMR 이동 2–20):")
for r in rows:
    if r[1] != "pos": p = pos[r[0]]; print(f"  {r[0]} {r[1]}: {r[2] / p[2]:.2f} / {r[3] / p[3]:.2f} / {r[5] / p[5]:.2f}")
