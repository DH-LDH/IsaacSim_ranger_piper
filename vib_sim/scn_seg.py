"""시나리오 기록의 실측과 엔코더 재생을 "튐에 걸치지 않는 1 s 조각" 방식으로 다시 낸다(젯슨 FW191_D_LOG 16.1절과 같은 방식).
  python3 scn_seg.py        (재생 시계열: exp3_scn.py --src replay --bases fixed --keep --tag segfix)
조각: 1 s, Hann, 절반 겹침, 조각마다 선형 추세 제거, 평균 스펙트럼을 대역에서 더함. 창은 젯슨 표(scn_band2.csv)의 t0·dur(기록 시작 기준)."""
import csv, glob, gzip, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.expanduser("~/piper_ws/docs/data/raw"); D = os.path.expanduser("~/piper_ws/docs/data")
BANDS = (("5_10", 5, 10), ("2_10", 2, 10), ("10_20", 10, 20))

def seg_rms(t, X, fs, t0, t1, excl, seg=1.0):
    """t: 기록 시작 기준 시각, X: n x 3. 반환 {대역: 3축 크기 rms}, 쓴 조각 수"""
    N = int(round(seg * fs)); w = np.hanning(N); f = np.fft.rfftfreq(N, 1 / fs); P = np.zeros(len(f)); n = 0; k = np.arange(N)
    i = int(np.searchsorted(t, t0))
    while i + N <= len(t) and t[i + N - 1] <= t1:
        a, b = t[i], t[i + N - 1]
        if not any(a < e1 and b > e0 for e0, e1 in excl):
            for c in range(3):
                x = X[i:i + N, c]; x = x - np.polyval(np.polyfit(k, x, 1), k)
                P += 2 * np.abs(np.fft.rfft(x * w)) ** 2 / (N * np.sum(w ** 2))
            n += 1
        i += N // 2
    return ({kk: float(np.sqrt(np.sum(P[(f >= lo) & (f <= hi)]) / n)) for kk, lo, hi in BANDS} if n else {}), n

def brick_rms(t, X, fs, t0, t1, excl, lo, hi):
    m = (t >= t0) & (t < t1); keep = np.ones(int(m.sum()), bool)
    for e0, e1 in excl: keep &= ~((t[m] >= e0) & (t[m] < e1))
    s = 0.0
    for c in range(3):
        x = X[m, c]; n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
        Xf = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); Xf[(f < lo) | (f > hi)] = 0; s += np.mean(np.fft.irfft(Xf, n)[keep] ** 2)
    return float(np.sqrt(s))

if __name__ == "__main__":
    jet = {r["id"]: r for r in csv.DictReader(open(D + "/scn_band2.csv"))}
    order = [("1축", "pos", "110051_slow_j2_s1"), ("1축", "tde", "110813_sine"), ("1축", "asmc16", "111226_sine"), ("2축", "pos", "204150_slow_j12_sxy"), ("2축", "tde", "203040_sine"),
             ("2축", "asmc16", "112534_sine"), ("3축", "pos", "204306_slow_j123_s3"), ("3축", "tde", "205503_sine"), ("3축", "asmc16", "205725_sine")]
    name = {"pos": "위치 모드", "tde": "하이브리드(TDE J2)", "asmc16": "ASMC16(J2)"}; rows = []
    for scn, ctl, o in order:
        stem = "20261029_" + o; j = jet[o.split("_")[0]]; t0 = float(j["t0"]); t1 = t0 + float(j["dur"])
        with gzip.open(f"{R}/scn/{stem}.csv.gz", "rt") as f_:
            ta0 = float(next(csv.DictReader(f_))["t"])
        excl = [(float(r["exclude_from"]) - ta0, float(r["exclude_to"]) - ta0) for r in csv.DictReader(open(R + "/mit_jumps.csv")) if r["stem"] == stem]
        tz = float(open(f"{R}/scn/{stem}.t0").read())
        with gzip.open(f"{R}/scn/{stem}.imu.csv.gz", "rt") as f_:
            g = np.array([(float(x["t"]) - tz - ta0, float(x["x"]), float(x["y"]), float(x["z"])) for x in csv.DictReader(f_) if x["stream"] == "g"])
        tu = np.arange(0.2, g[-1, 0] - 0.2, 1 / 400.0); G = np.stack([np.interp(tu, g[:, 0], g[:, c]) for c in (1, 2, 3)], axis=1)
        rs, ns = seg_rms(tu, G, 400.0, t0, t1, excl); rb = brick_rms(tu, G, 400.0, t0, t1, excl, 5, 10)
        p = glob.glob(H + f"/out/exp3_ts_segfix_{stem[:40]}*_fixed.npz")
        if p:
            d = np.load(p[0]); ss, _ = seg_rms(d["t"], d["gyro"], 1000.0, t0, t1, excl); sb = brick_rms(d["t"], d["gyro"], 1000.0, t0, t1, excl, 5, 10)
        else:
            ss, sb = {}, float("nan")
        rows.append(dict(scn=scn, ctl=ctl, stem=stem, n_jump=len(excl), n_seg=ns, jet_5_10=float(j["gyro_5_10"]), real_brick=rb, sim_brick=sb * 1.27,
                         **{"real_" + k: v for k, v in rs.items()}, **{"sim_" + k: v * 1.27 for k, v in ss.items()}))
    pos = {r["scn"]: r for r in rows if r["ctl"] == "pos"}
    md = ["튐에 걸치지 않는 1 s 조각(Hann, 절반 겹침) 방식. 창은 사인 시작 + 2 s부터 22 s(젯슨 `scn_band2.csv`의 t0·dur). 끝단 자이로 세 축 크기 rms[rad/s], 재생 값 × 1.27.", "",
          "| 시나리오 | 제어 | 튐 | 조각 수 | 실측 5–10 Hz | (젯슨 값) | 재생 5–10 Hz | 재생 / 실측 | 위치 모드 대비: 실측 | 재생 | 벽돌 방식의 재생 / 실측 |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        p_ = pos[r["scn"]]; has = "sim_5_10" in r
        md.append(f"| {r['scn']} | {name[r['ctl']]} | {r['n_jump']} | {r['n_seg']} | {r['real_5_10']:.4f} | {r['jet_5_10']:.4f} | " + (f"{r['sim_5_10']:.4f} | {r['sim_5_10'] / r['real_5_10']:.2f} | " if has else "— | — | ")
                  + ("— | — | " if r["ctl"] == "pos" else f"{r['real_5_10'] / p_['real_5_10']:.2f} | " + (f"{r['sim_5_10'] / p_['sim_5_10']:.2f} | " if has else "— | ")) + f"{r['sim_brick'] / r['real_brick']:.2f} |")
    md += ["", "두 대역(조각 방식): 실측 / 재생, 그리고 위치 모드 대비 비(실측 · 재생)", "", "| 시나리오 | 제어 | 2–10 Hz 실측 / 재생 | 위치 모드 대비 | 10–20 Hz 실측 / 재생 | 위치 모드 대비 |", "|---|---|---|---|---|---|"]
    for r in rows:
        if "sim_5_10" not in r: continue
        p_ = pos[r["scn"]]; q = lambda k: "—" if r["ctl"] == "pos" else f"{r['real_' + k] / p_['real_' + k]:.2f} · {r['sim_' + k] / p_['sim_' + k]:.2f}"
        md.append(f"| {r['scn']} | {name[r['ctl']]} | {r['real_2_10']:.4f} / {r['sim_2_10']:.4f} ({r['sim_2_10'] / r['real_2_10']:.2f}) | {q('2_10')} | {r['real_10_20']:.4f} / {r['sim_10_20']:.4f} ({r['sim_10_20'] / r['real_10_20']:.2f}) | {q('10_20')} |")
    open(H + "/figs/scn_seg_table.md", "w").write("\n".join(md) + "\n"); print("\n".join(md))
