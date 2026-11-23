"""KRoC 양식용 그림(1단 폭 80 mm, 300 dpi 이상·1000 픽셀 이상, 글자 9 pt 안팎, 캡션은 영어).
실행: ~/nonmoon/.venv_pptx/bin/python fig_kroc.py  →  figs/kroc_fig1.png, figs/kroc_fig2.png
입력: out/exp3_replay_c1.csv, out/exp6_ts_figtask_*.npz, out/exp6_tk4_s{1,2,3}.csv, out/exp6_tk2r_s{1,2,3}_pos.csv"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; CAM = 1.27
plt.rcParams.update({"font.size": 8.5, "font.family": "DejaVu Sans", "axes.grid": True, "grid.alpha": 0.3, "savefig.dpi": 400, "savefig.bbox": "tight", "axes.linewidth": 0.6})

def band(x, fs, lo, hi):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)

# ---- Fig. 1: 실측 대 모형(엔코더 재생), 18조건 ----
rows = list(csv.DictReader(open(O + "/exp3_replay_c1.csv")))
fig, ax = plt.subplots(figsize=(3.15, 2.25))
for v, c, m in ((5, "#1f77b4", "o"), (10, "#ff7f0e", "s"), (15, "#d62728", "^")):
    r = [x for x in rows if float(x["cond"].split("_v")[1].split("_")[0]) == v]
    ax.scatter([float(x["real_gyro_rms"]) for x in r], [float(x["stop_pitch_5_10"]) * CAM for x in r], s=18, color=c, marker=m, label=f"{v} deg/s", zorder=3)
lim = 0.12; ax.plot([0, lim], [0, lim], "k--", lw=0.6); ax.set_xlim(0, lim); ax.set_ylim(0, lim); ax.set_aspect("equal")
ax.set_xlabel("Measured residual vibration [rad/s]"); ax.set_ylabel("Model [rad/s]"); ax.legend(frameon=False, fontsize=7.5, loc="lower right")
fig.savefig(H + "/figs/kroc_fig1.png"); plt.close(fig)

# ---- Fig. 2: 종합 장면(팔 동작 + 베이스 출발 → 턱 → 정지) ----
CS = [("none", "No control", "#555555"), ("bz+prevzE10+ff10+dampRJ", "Shaping + rear-wheel pred. + FB", "#1f77b4"), ("bz+map+dampRJ", "Shaping + terrain preview + FB", "#d62728")]
fig, ax = plt.subplots(2, 1, figsize=(3.15, 3.5), gridspec_kw={"height_ratios": [1, 1], "hspace": 0.62})
for case, lab, c in CS:
    d = np.load(O + f"/exp6_ts_figtask_{case}.npz"); t = d["t"]
    dev = np.sqrt(sum(band(d["te"][:, k], 1000.0, 2, 20) ** 2 for k in range(3))) * 1000.0
    ax[0].plot(t, dev, color=c, lw=0.6)
tf, tr = float(d["tf"]), float(d["tr"])
for x_, s_ in ((tf, "front wheel"), (tr, "rear")):
    ax[0].axvline(x_, color="k", lw=0.5, ls=":"); ax[0].text(x_, 1.02, s_, fontsize=7, ha="center", transform=ax[0].get_xaxis_transform())
ax[0].set_xlim(0.3, t[-1] - 0.3); ax[0].set_xlabel("Time [s]"); ax[0].set_ylabel("Tip deviation [mm]"); ax[0].text(0.01, 0.9, "(a)", transform=ax[0].transAxes, fontsize=8.5)
w = 0.2
for i, n in enumerate("123"):
    R = {r["case"]: r for r in csv.DictReader(open(O + f"/exp6_tk4_s{n}.csv"))}; ref = next(csv.DictReader(open(O + f"/exp6_tk2r_s{n}_pos.csv")))
    vals = [(float(ref["tip_vib_mm"]), "#bbbbbb", "Arm motion only (base at rest)")] + [(float(R[c_]["tip_vib_mm"]), col, lab) for c_, lab, col in CS]
    for k, (v, col, lab) in enumerate(vals):
        ax[1].bar(i + (k - 1.5) * w, v, w, color=col, label=lab if i == 0 else None, zorder=3)
ax[1].set_xticks(range(3)); ax[1].set_xticklabels(["1 joint", "2 joints", "3 joints"]); ax[1].set_ylabel("Deviation rms [mm]"); ax[1].grid(axis="x", visible=False)
ax[1].text(0.01, 0.9, "(b)", transform=ax[1].transAxes, fontsize=8.5)
hh, ll = ax[1].get_legend_handles_labels(); fig.legend(hh, ll, loc="lower center", ncol=1, fontsize=7.2, frameon=False, bbox_to_anchor=(0.5, -0.17))
fig.savefig(H + "/figs/kroc_fig2.png"); plt.close(fig); print("ok")
