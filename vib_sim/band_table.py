"""대역별 표: out/exp6_ts_<tag>_<case>.npz에서 끝단 가속도(2–10, 10–20, 2–20 Hz)와 끝단 각속도(2–10, 2–20 Hz)를 낸다.
사용: python3 band_table.py <tag> [rigid_tag]"""
import glob, os, sys
import numpy as np
O = os.path.dirname(os.path.abspath(__file__)) + "/out/"
def band(x, fs, lo, hi):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt); X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)
def rms3(V, m, lo, hi, deriv=False):
    out = 0
    for c in range(3):
        b = band(V[m, c], 1000.0, lo, hi)
        if deriv: b = np.gradient(b, 0.001)
        out += np.mean(b ** 2)
    return float(np.sqrt(out))
def row(path):
    d = np.load(path); t = d["t"]; m = (t >= float(d["tf"]) - 0.1) & (t < float(d["tr"]) + 1.5)
    return [rms3(d["ev"], m, 2, 10, True), rms3(d["ev"], m, 10, 20, True), rms3(d["ev"], m, 2, 20, True), rms3(d["gyro"], m, 2, 10), rms3(d["gyro"], m, 2, 20)]
tag = sys.argv[1]; files = sorted(glob.glob(O + f"exp6_ts_{tag}_*.npz"), key=os.path.getmtime)
base = None
print("%-22s | 가속도 2–10 Hz | 10–20 Hz | 2–20 Hz | 각속도 2–10 Hz | 2–20 Hz   (괄호는 대책 없음 대비)" % "조건")
if len(sys.argv) > 2:
    r = row(O + f"exp6_ts_{sys.argv[2]}_none.npz"); print("%-22s | %.2f | %.2f | %.2f | %.3f | %.3f" % ("단단한 팔", *r))
for p in files:
    c = os.path.basename(p)[len(f"exp6_ts_{tag}_"):-4]; r = row(p)
    if c == "none": base = r
    print("%-22s | %.2f (%.2f) | %.2f (%.2f) | %.2f (%.2f) | %.3f (%.2f) | %.3f (%.2f)" % (c, r[0], r[0] / base[0], r[1], r[1] / base[1], r[2], r[2] / base[2], r[3], r[3] / base[3], r[4], r[4] / base[4]))
