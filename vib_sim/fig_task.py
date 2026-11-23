"""논문 그림 3: 종합 장면(팔 동작 + AMR 출발 → 턱 → 정지). 실행: ~/nonmoon/.venv_pptx/bin/python fig_task.py
입력: out/exp6_ts_figtask_*.npz (exp6_bump.py --stop --keep --tag figtask), out/exp6_tk4_s{1,2,3}.csv, out/exp6_tk2r_s{1,2,3}_pos.csv"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"
plt.rcParams.update({"font.size": 8, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 200, "savefig.bbox": "tight"})
CS = [("none", "No control", "#555555"), ("bz+prevzE10+ff10+dampRJ", "Shaping + rear-wheel prediction + damping FB", "#1f77b4"), ("bz+map+dampRJ", "Shaping + terrain preview + damping FB", "#d62728")]

def band(x, fs, lo, hi):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)

fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.5), gridspec_kw={"width_ratios": [1.25, 1.25, 1.0]})
for case, lab, c in CS:
    d = np.load(O + f"/exp6_ts_figtask_{case}.npz"); t = d["t"]
    ax[0].plot(t, band(d["gyro"][:, 2], 1000.0, 2, 20), color=c, lw=0.6, label=lab)
    dev = np.sqrt(sum(band(d["te"][:, k], 1000.0, 2, 20) ** 2 for k in range(3))) * 1000.0
    ax[1].plot(t, dev, color=c, lw=0.6)
tf, tr = float(d["tf"]), float(d["tr"])
for a_ in ax[:2]:
    for x_, s_ in ((tf, "front"), (tr, "rear")):
        a_.axvline(x_, color="k", lw=0.4, ls=":")
    a_.set_xlabel("Time [s]"); a_.set_xlim(0.3, t[-1] - 0.3)
    a_.text(tf, 1.01, "front", fontsize=6, ha="center", transform=a_.get_xaxis_transform()); a_.text(tr, 1.01, "rear wheel", fontsize=6, ha="center", transform=a_.get_xaxis_transform())
ax[0].set_ylabel("Lateral angular velocity,\n2–20 Hz [rad/s]"); ax[0].set_title("(a) Tip angular velocity (joint-2 motion)", fontsize=8, pad=9)
ax[1].set_ylabel("Tip deviation, 2–20 Hz [mm]"); ax[1].set_title("(b) Deviation from reference path", fontsize=8, pad=9)
w = 0.2; names = ["1 joint", "2 joints", "3 joints"]
for i, n in enumerate("123"):
    R = {r["case"]: r for r in csv.DictReader(open(O + f"/exp6_tk4_s{n}.csv"))}; ref = next(csv.DictReader(open(O + f"/exp6_tk2r_s{n}_pos.csv")))
    vals = [(float(ref["tip_vib_mm"]), "#bbbbbb", "Arm motion only (base at rest)")] + [(float(R[c_]["tip_vib_mm"]), col, lab) for c_, lab, col in CS]
    for k, (v, col, lab) in enumerate(vals):
        ax[2].bar(i + (k - 1.5) * w, v, w, color=col, label=lab if i == 0 else None)
ax[2].set_xticks(range(3)); ax[2].set_xticklabels(names); ax[2].set_ylabel("Tip deviation rms, 2–20 Hz [mm]"); ax[2].set_title("(c) Whole run", fontsize=8, pad=9)
ax[2].grid(axis="x", visible=False); hh, ll = ax[2].get_legend_handles_labels()
fig.legend(hh, ll, loc="lower center", ncol=2, fontsize=6.5, frameon=False, bbox_to_anchor=(0.5, -0.12))
fig.tight_layout(); fig.savefig(H + "/figs/fig_task.png"); plt.close(fig); print("ok")
