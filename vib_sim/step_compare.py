"""계단 기록(J1·J2·J3 2° 계단, 자세 A·B) 재생 대 실측: 계단 뒤 0.35–3 s의 끄덕임·좌우 각속도(2–20 Hz) rms와 봉우리 주파수.
실측 좌우 = 자이로를 정지 구간의 중력 방향에 투영(카메라가 기울어 있음), 끄덕임 = 카메라 x. 출력 figs/step_compare.md"""
import csv, glob, gzip, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.expanduser("~/piper_ws/docs/data/raw")
def band(x, fs, lo=2.0, hi=20.0):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)
def peak(x, fs):
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)), 8 * len(x))); f = np.fft.rfftfreq(8 * len(x), 1 / fs); m = (f > 2) & (f < 20)
    return float(f[m][np.argmax(X[m])])
md = ["| 기록 | 계단 | 축 | 실측 rms [rad/s] | 재생 rms | 재생 / 실측 | 실측 봉우리 [Hz] | 재생 봉우리 [Hz] |", "|---|---|---|---|---|---|---|---|"]
for sub, stem in (("lat", "20261023_143906_step_j1_poseA"), ("lat", "20261023_145736_step_j1_poseB"), ("id", "20261023_143950_step_j3_poseA"), ("id", "20261023_145815_step_j3_poseB"), ("id", "20261023_143931_step_j2_poseA"), ("id", "20261023_145755_step_j2_poseB")):
    with gzip.open(f"{R}/{sub}/{stem}.csv.gz", "rt") as f: r = list(csv.DictReader(f))
    ta = np.array([float(x["t"]) for x in r]); j = int(stem.split("step_j")[1][0]); qr = np.array([float(x[f"qref{j}"]) for x in r])
    ev = [ta[i] - ta[0] for i in range(1, len(qr)) if abs(qr[i] - qr[i - 1]) > 0.5]
    t0 = float(open(f"{R}/{sub}/{stem}.t0").read())
    with gzip.open(f"{R}/{sub}/{stem}.imu.csv.gz", "rt") as f: im = list(csv.DictReader(f))
    g = np.array([(float(x["t"]) - t0 - ta[0], float(x["x"]), float(x["y"]), float(x["z"])) for x in im if x["stream"] == "g"])
    ac = np.array([(float(x["t"]) - t0 - ta[0], float(x["x"]), float(x["y"]), float(x["z"])) for x in im if x["stream"] == "a"])
    up = ac[(ac[:, 0] > 0.5) & (ac[:, 0] < ev[0] - 0.5), 1:].mean(axis=0); up /= np.linalg.norm(up)
    d = np.load(glob.glob(H + f"/out/exp3_ts_steps_{stem[:40]}*_fixed.npz")[0]); ts, G = d["t"], d["gyro"]
    for k, e in enumerate(ev[:2]):
        tu = np.arange(e + 0.35, e + 3.0, 1 / 400.0)
        real = {"끄덕임": np.interp(tu, g[:, 0], g[:, 1]), "좌우": sum(np.interp(tu, g[:, 0], g[:, 1 + c]) * up[c] for c in range(3))}
        ms = (ts >= e + 0.35) & (ts < e + 3.0); sim = {"끄덕임": G[ms, 1], "좌우": G[ms, 2]}
        for axn in (("좌우", "끄덕임") if j == 1 else ("끄덕임", "좌우")):
            rb, sb = band(real[axn], 400.0), band(sim[axn], 1000.0); rr, sr = rb.std(), sb.std() * (1.27 if axn == "끄덕임" else 1.0)
            md.append(f"| J{j} {'자세 A' if 'poseA' in stem else '자세 B'} | {k + 1} | {axn} | {rr:.4f} | {sr:.4f} | {sr / rr:.2f} | {peak(rb, 400.0):.1f} | {peak(sb, 1000.0):.1f} |")
open(H + "/figs/step_compare.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
