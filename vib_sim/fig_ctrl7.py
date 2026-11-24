"""KROS 초안 v7의 제어 구조도(식 번호를 v7에 맞춤) → figs/kroc7_ctrl.png (80 × 46 mm, 800 dpi). fig_kroc6.py의 Fig. 2 블록에서 문구만 바꾼 것.
실행: ~/nonmoon/.venv_pptx/bin/python fig_ctrl7.py"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix"})
W, HT = 80.0, 46.0
fig = plt.figure(figsize=(W * MM, HT * MM)); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, HT); ax.axis("off")
BL, GY, OR = "#dbe9f6", "#eeeeee", "#fbe5d0"
def box(x, y, w, h, s, fc=GY, fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.8", fc=fc, ec="k", lw=0.6)); ax.text(x + w / 2, y + h / 2, s, ha="center", va="center", fontsize=fs, linespacing=0.95)
def arr(p, q, ls="-"):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="-|>", lw=0.7, color="k", ls=ls, shrinkA=0, shrinkB=0, mutation_scale=7))
def line(pts, ls="-"):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], ls=ls, color="k", lw=0.7, solid_capstyle="butt")
def summ(x, y):
    ax.add_patch(Circle((x, y), 1.5, fc="w", ec="k", lw=0.6)); ax.text(x, y, "+", ha="center", va="center", fontsize=8)
yt, ym, ya = 37.0, 22.0, 5.5
box(57.0, yt, 22.0, 7.5, "Mobile base", GY); box(3.0, yt, 35.0, 7.5, "Rear-wheel prediction", BL)
arr((57.0, yt + 3.75), (38.0, yt + 3.75)); ax.text(47.5, yt + 4.6, r"$\delta z,\,\delta\varphi,\,\mathbf{a}_b$", ha="center", va="bottom", fontsize=9.5)
box(3.0, ym, 37.0, 8.0, r"(b) $-\mathbf{J}^{-1}(\delta z\,\hat{\mathbf{z}}+\delta\varphi\times\mathbf{r}_e)$", OR)
box(43.0, ym, 21.0, 8.0, r"(c) $\mathbf{K}^{-1}\mathbf{N}\,\mathbf{a}_P$", OR)
arr((16.0, yt), (16.0, ym + 8.0)); line([(30.0, yt), (30.0, ym + 11.5), (53.5, ym + 11.5)]); arr((53.5, ym + 11.5), (53.5, ym + 8.0))
ys = ya + 3.75
line([(53.5, ym), (53.5, 18.0), (28.0, 18.0)]); summ(26.5, 18.0); arr((26.5, ym), (26.5, 19.5))
line([(25.0, 18.0), (6.0, 18.0)]); arr((6.0, 18.0), (6.0, ys + 1.5)); summ(6.0, ys)
ax.text(7.2, 14.8, r"$\Delta\mathbf{q}$", ha="left", va="center", fontsize=9.5)
ax.text(0.2, ys - 1.2, "Path", ha="left", va="top", fontsize=8.5); arr((0.3, ys), (4.5, ys))
arr((7.5, ys), (11.0, ys)); box(11.0, ya, 21.0, 7.5, "(a) Tracking\ncontrol, J2-3", GY)
arr((32.0, ys), (35.5, ys)); box(35.5, ya, 16.5, 7.5, "Built-in\nPD, motor", GY)
arr((52.0, ys), (55.5, ys)); box(55.5, ya, 23.5, 7.5, "Elastic joints,\nlinks", GY)
arr((68.0, yt), (68.0, ya + 7.5), ls=(0, (3, 2))); ax.text(69.3, 26.0, "base\nmotion", ha="left", va="center", fontsize=8.5, linespacing=0.9)
fig.savefig(H + "/figs/kroc7_ctrl.png", dpi=800); print("→ figs/kroc7_ctrl.png")
