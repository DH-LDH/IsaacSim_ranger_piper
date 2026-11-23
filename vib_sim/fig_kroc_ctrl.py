"""KRoC 양식용 제어 구조도(1단 폭 80 mm, 글자 9.5 pt Times New Roman, 800 dpi).
실행: ~/nonmoon/.venv_pptx/bin/python fig_kroc_ctrl.py  →  figs/kroc_fig_ctrl.png"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4; W, HT = 80.0, 46.0
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix"})
fig = plt.figure(figsize=(W * MM, HT * MM)); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(4.0, 50.0); ax.axis("off")
BL, GY, OR = "#dbe9f6", "#eeeeee", "#fbe5d0"

def box(x, y, w, h, s, fc=GY, fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.8", fc=fc, ec="k", lw=0.6)); ax.text(x + w / 2, y + h / 2, s, ha="center", va="center", fontsize=fs, linespacing=0.95)

def arr(p, q, ls="-", c="k"):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="-|>", lw=0.7, color=c, ls=ls, shrinkA=0, shrinkB=0, mutation_scale=7))

def line(pts, ls="-", c="k"):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], ls=ls, color=c, lw=0.7, solid_capstyle="butt")

def summ(x, y):
    ax.add_patch(Circle((x, y), 1.5, fc="w", ec="k", lw=0.6)); ax.text(x, y, "+", ha="center", va="center", fontsize=8)

# ---- 윗줄: 베이스 ----
yb = 43.0
ax.text(0.5, yb + 3.0, "Base speed", ha="left", va="center"); ax.text(0.5, yb - 0.6, "command", ha="left", va="center")
arr((16.5, yb + 1.2), (19.5, yb + 1.2)); box(19.5, yb - 2.8, 25.5, 8.0, "(a) ZVD shaper", BL)
arr((45.0, yb + 1.2), (49.5, yb + 1.2)); box(49.5, yb - 2.8, 29.5, 8.0, "Mobile base", GY)
# ---- 가운데 줄: 팔 ----
ya = 22.0
ax.text(0.5, ya + 2.6, "Arm", ha="left", va="center"); ax.text(0.5, ya - 1.0, "ref.", ha="left", va="center")
arr((7.5, ya + 0.8), (10.5, ya + 0.8)); summ(12.0, ya + 0.8)
arr((13.5, ya + 0.8), (16.5, ya + 0.8)); box(16.5, ya - 3.4, 15.5, 8.4, "Tracking\ncontrol", GY)
arr((32.0, ya + 0.8), (35.0, ya + 0.8)); summ(36.5, ya + 0.8)
arr((38.0, ya + 0.8), (41.0, ya + 0.8)); box(41.0, ya - 3.4, 15.5, 8.4, "Built-in\nPD, motor", GY)
arr((56.5, ya + 0.8), (60.0, ya + 0.8)); box(60.0, ya - 3.4, 19.0, 8.4, "Elastic\njoints, links", GY)
# 베이스 → 팔: 관성력(점선)
arr((72.0, yb - 2.8), (72.0, ya + 5.0), ls=(0, (3, 2))); ax.text(71.0, 30.5, "inertial\nforce", ha="right", va="center", fontsize=8.5, linespacing=0.9)
# ---- (c) 피드포워드: 베이스 가속도 → 위치 기준 ----
line([(52.0, yb - 2.8), (52.0, 34.5)]); ax.text(53.0, 37.2, r"$\boldsymbol{a}_b$", ha="left", va="center")
arr((52.0, 34.5), (34.0, 34.5)); box(12.0, 31.0, 22.0, 7.0, r"(c) $\boldsymbol{K}^{-1}\boldsymbol{C}\boldsymbol{a}_b$", OR)
ax.text(35.0, 32.0, "base IMU or", ha="left", va="center", fontsize=8.5); ax.text(35.0, 29.2, "rear-wheel pred.", ha="left", va="center", fontsize=8.5)
line([(12.0, 34.5), (8.5, 34.5), (8.5, 27.5), (12.0, 27.5)]); arr((12.0, 27.5), (12.0, ya + 2.3))
# ---- (b) 끝단 피드백: 끝단 자이로 → 토크 ----
yf = 6.0
line([(72.0, ya - 3.4), (72.0, yf + 3.5)]); arr((72.0, yf + 3.5), (64.5, yf + 3.5)); ax.text(71.0, 15.6, "tip gyro", ha="right", va="center", fontsize=8.5)
box(38.5, yf, 26.0, 7.0, "(b) Damping FB", BL)
line([(38.5, yf + 3.5), (36.5, yf + 3.5)]); arr((36.5, yf + 3.5), (36.5, ya - 0.7))
ax.text(35.5, 13.0, "torque", ha="right", va="center", fontsize=8.5)
fig.savefig(H + "/figs/kroc_fig_ctrl.png", dpi=800); plt.close(fig)

# ---- 시간 파형: 종합 장면(한 관절 동작), 끝단 흔들림의 크기 ----
import numpy as np
O = H + "/out"
def band(x, fs, lo, hi):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)
plt.rcParams.update({"axes.labelsize": 9.5, "xtick.labelsize": 9.5, "ytick.labelsize": 9.5, "legend.fontsize": 9.5, "axes.linewidth": 0.6, "axes.grid": True, "grid.alpha": 0.3, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})
fig, ax = plt.subplots(figsize=(80 * MM, 36 * MM), layout="constrained")
for case, lab, c in (("none", "+ tracking", "#5b9bd5"), ("bz+prevzE10+ff10+dampRJ", "+ base layers", "#1f4e79")):
    d = np.load(O + f"/exp6_ts_figtask5_{case}.npz"); t = d["t"]
    dev = np.sqrt(sum(band(d["te"][:, k], 1000.0, 2, 20) ** 2 for k in range(3))) * 1000.0
    ax.plot(t, dev, color=c, lw=0.6, label=lab)
tf, tr = float(d["tf"]), float(d["tr"])
for x_, s_ in ((tf, "front"), (tr, "rear")):
    ax.axvline(x_, color="k", lw=0.5, ls=":"); ax.text(x_, 1.02, s_, fontsize=8.5, ha="center", transform=ax.get_xaxis_transform())
ax.set_xlim(0.3, t[-1] - 0.3); ax.set_ylim(0, None); ax.set_xlabel("Time [s]"); ax.set_ylabel("Deviation [mm]")
ax.legend(frameon=False, loc="upper right", handlelength=1.0, labelspacing=0.1, borderaxespad=0.1)
fig.savefig(H + "/figs/kroc_fig_time.png", dpi=800); print("ok")
