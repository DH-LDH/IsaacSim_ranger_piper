"""KRoC 양식용 그림(1단 폭 80 mm, 300 dpi 이상·1000 픽셀 이상, 글자 9 pt 안팎, 캡션은 영어).
실행: ~/nonmoon/.venv_pptx/bin/python fig_kroc.py  →  figs/kroc_fig1.png, figs/kroc_fig2.png
입력: out/exp3_replay_c1.csv, out/exp6_tk5_s{1,2,3}_{pos,pd,asmc3}.csv, out/exp6_tk5r_s{1,2,3}_asmc3.csv"""
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

# ---- Fig. 2 (v4, MIT 기준): 종합 장면(팔 동작 + 베이스 출발 → 턱 → 정지) — (a) 궤적 오차, (b) 흔들림 ----
# 앞선 판(위치 모드 기록 재생, 시간 파형 + 막대)은 figs/kroc_fig2_v3.png로 남겼다.
def L(tag): return {r["case"]: r for r in csv.DictReader(open(O + f"/exp6_{tag}.csv"))}
LAY = "bz+prevzE10+ff10+dampRJ"; w = 0.2
fig, ax = plt.subplots(2, 1, figsize=(3.15, 3.25), gridspec_kw={"hspace": 1.12})
A = [("pos", "none", "Position control", "#999999"), ("pd", "none", "Torque mode, built-in PD only", "#f0b27a"), ("asmc3", "none", "Torque mode + tracking control", "#5b9bd5"), ("asmc3", LAY, "  + shaping + rear-wheel pred. + FB", "#1f4e79")]
for i, n in enumerate("123"):
    for k, (m, c, lab, col) in enumerate(A):
        ax[0].bar(i + (k - 1.5) * w, float(L(f"tk5_s{n}_{m}")[c]["tip_err_mm"]), w, color=col, label=lab if i == 0 else None, zorder=3)
ax[0].set_ylabel("Path error rms [mm]"); ax[0].text(0.01, 0.88, "(a)", transform=ax[0].transAxes, fontsize=8.5)
B = [("r", "none", "Arm motion only (base at rest)", "#bbbbbb"), ("", "none", "Base moving, no base-side control", "#555555"), ("", LAY, "Shaping + rear-wheel pred. + FB", "#1f4e79")]
for i, n in enumerate("123"):
    for k, (r_, c, lab, col) in enumerate(B):
        ax[1].bar(i + (k - 1) * w * 1.2, float(L(f"tk5{r_}_s{n}_asmc3")[c]["tip_vib_mm"]), w * 1.2, color=col, label=lab if i == 0 else None, zorder=3)
ax[1].set_ylabel("Deviation 2-20 Hz [mm]"); ax[1].text(0.01, 0.88, "(b)", transform=ax[1].transAxes, fontsize=8.5)
for a_ in ax:
    a_.set_xticks(range(3)); a_.set_xticklabels(["1 joint", "2 joints", "3 joints"]); a_.grid(axis="x", visible=False)
    a_.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=1, fontsize=6.8, frameon=False, handlelength=1.2, labelspacing=0.2)
fig.savefig(H + "/figs/kroc_fig2.png"); plt.close(fig); print("ok")
