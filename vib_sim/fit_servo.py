"""위치 모드 서보 모형을 실측에 맞춘다: 명령(qref2) → 엔코더(q2). 입력: piper_ws docs/data/raw/c1/*.csv.gz (Isaac 필요 없음)
모형: 지연 Td + 2차 응답(fs, zs). 등속 구간과 정지 구간의 엔코더 위치 오차 rms를 최소로 하는 값을 찾는다.
주의: 엔코더에는 끝단 모드가 비친 4.5–6.5 Hz 오르내림이 섞여 있다. 그 성분은 서보가 아니라 구조 쪽 것이므로 1.5 Hz 아래로 거른 값에 맞춘다(--lp 0이면 거르지 않음).
"""
import argparse, csv, glob, gzip, os
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("--lp", type=float, default=0.0); a = ap.parse_args()
R = os.path.expanduser("~/piper_ws/docs/data/raw/c1")
files = [f for f in sorted(glob.glob(R + "/*.csv.gz")) if ".imu." not in f]
DT = 0.001
def load(f):
    with gzip.open(f, "rt") as fh:
        r = list(csv.DictReader(fh))
    t = np.array([float(x["t"]) for x in r]); q = np.array([float(x["q2"]) for x in r]); qr = np.array([float(x["qref2"]) for x in r]); ph = [x["phase"] for x in r]
    i0 = next(i for i, p in enumerate(ph) if p == "ramp"); i1 = next(i for i, p in enumerate(ph) if p == "after")
    tt = np.arange(t[i0] - 0.5, t[i1], DT)
    return tt - tt[0], np.interp(tt, t, qr), np.interp(tt, t, q)
data = [load(f) for f in files]
def sim(qr, Td, fs, zs):
    n = len(qr); nd = int(round(Td / DT)); w = 2 * np.pi * fs; q = qr[0]; dq = 0.0; out = np.empty(n)
    hold = qr[::10].repeat(10)[:n]                      # 명령 100 Hz
    for i in range(n):
        r = hold[max(0, i - nd)]
        dq += (w * w * (r - q) - 2 * zs * w * dq) * DT; q += dq * DT; out[i] = q
    return out
def lp(x, fc):
    if fc <= 0: return x
    X = np.fft.rfft(x - x[0]); f = np.fft.rfftfreq(len(x), DT); X[f > fc] = 0
    return np.fft.irfft(X, len(x)) + x[0]
best = []
for Td in (0.02, 0.03, 0.04, 0.05, 0.06, 0.07):
    for fs in (3, 4, 5, 6, 8, 10, 14, 20):
        for zs in (0.35, 0.5, 0.7, 1.0):
            e = [np.sqrt(np.mean((lp(sim(qr, Td, fs, zs), a.lp) - lp(q, a.lp)) ** 2)) for _, qr, q in data]
            best.append((float(np.mean(e)), Td, fs, zs))
best.sort()
print("오차 rms[deg]  Td     fs    zs   (작은 순 8개), 거름", a.lp)
for b in best[:8]: print("  %.4f      %.3f  %4.1f  %.2f" % b)
cur = [b for b in best if b[1:] == (0.06, 10, 0.35)]
print("참고 — 지금 값에 가까운 (0.06, 10, 0.35):", cur)
# 전체 지연: 등속 구간에서 엔코더가 명령보다 늦는 시간
lag = []
for t, qr, q in data:
    v = np.gradient(qr, DT); m = np.abs(v) > 1.0
    if m.sum() > 300:
        sl = np.where(m)[0][150:-50]; lag.append(float(np.mean((qr[sl] - q[sl]) / v[sl])))
print("등속 구간 추종 지연[ms] 중앙 %.0f (%.0f–%.0f)" % (np.median(lag) * 1000, min(lag) * 1000, max(lag) * 1000))
