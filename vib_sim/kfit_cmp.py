"""강성 후보별 재생 대조: exp3_scn.py --src replay --bases fixed --keep --tag <태그> --K ... 의 시계열을 실측 손목 IMU와 조각 방식으로 견준다.
  python3 kfit_cmp.py kf0 kf1 ...    (창은 기록 첫 줄 + 1 s부터 끝 − 1 s, 재생 값 × 1.27)"""
import csv, gzip, glob, os, sys, numpy as np
from scn_seg import seg_rms
R = os.path.expanduser("~/piper_ws/docs/data/raw")
def real(stem):
    p = (glob.glob(f"{R}/c1/{stem}*.imu.csv.gz") + glob.glob(f"{R}/line/{stem}*.imu.csv.gz"))[0][:-len(".imu.csv.gz")]
    with gzip.open(p + ".csv.gz", "rt") as f_: rows = list(csv.DictReader(f_))
    ta0 = float(rows[0]["t"]); tend = float(rows[-1]["t"]) - ta0; tz = float(open(p + ".t0").read())
    with gzip.open(p + ".imu.csv.gz", "rt") as f_:
        g = np.array([(float(x["t"]) - tz - ta0, float(x["x"]), float(x["y"]), float(x["z"])) for x in csv.DictReader(f_) if x["stream"] == "g"])
    tu = np.arange(0.2, g[-1, 0] - 0.2, 1 / 400.0); return tu, np.stack([np.interp(tu, g[:, 0], g[:, c]) for c in (1, 2, 3)], axis=1), tend, os.path.basename(p)
if __name__ == "__main__":
    tags = sys.argv[1:]; stems = sorted(set(os.path.basename(p)[len(f"exp3_ts_{tags[0]}_"):-len("_fixed.npz")] for p in glob.glob(f"out/exp3_ts_{tags[0]}_*_fixed.npz")))
    res = {k: [] for k in tags}; print("| 기록 | 실측 2–10 | " + " | ".join(tags) + " |")
    for st in stems:
        tu, G, tend, full = real(st); t0, t1 = 1.0, tend - 1.0; rs, _ = seg_rms(tu, G, 400.0, t0, t1, []); row = []
        for k in tags:
            p = f"out/exp3_ts_{k}_{st}_fixed.npz"
            if not os.path.exists(p): row.append(float("nan")); continue
            d = np.load(p); ss, _ = seg_rms(d["t"], d["gyro"], 1000.0, t0, t1, []); r = ss["2_10"] * 1.27 / rs["2_10"]; row.append(r); res[k].append((full, r, ss["10_20"] * 1.27 / rs["10_20"]))
        print(f"| {full[9:]} | {rs['2_10']:.4f} | " + " | ".join(f"{x:.2f}" for x in row) + " |")
    isH = lambda s_: "poseH" in s_ or "_H_" in s_
    for k in tags:
        a = np.array([r for s_, r, _ in res[k] if not isH(s_)]); h = np.array([r for s_, r, _ in res[k] if isH(s_)]); a2 = np.array([r for s_, _, r in res[k] if not isH(s_)]); h2 = np.array([r for s_, _, r in res[k] if isH(s_)])
        print(f"{k}: 자세 A({len(a)}) {np.median(a):.2f} ({a.min():.2f}–{a.max():.2f}) | 그 자세({len(h)}) {np.median(h):.2f} ({h.min():.2f}–{h.max():.2f}) | 10–20 Hz A {np.median(a2):.2f} 그 자세 {np.median(h2):.2f} | 로그 오차 합 {np.sum(np.abs(np.log(np.r_[a, h]))):.2f}")
