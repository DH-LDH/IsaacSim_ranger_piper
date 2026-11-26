"""KROS 원고의 조건별 그래프: 보상 없음 대 제안한 보상 (b) + (c), 끝단 위치 오차의 최댓값과 RMS [mm]
값: vib_sim/figs/matrix_1124.md의 "보정 없음"과 "로봇팔 보정만" 열(한 번씩 돌린 값). 턱 10 mm, 0.3 m/s만 원고 [Table 3]의 세 번 평균.
막대 = 네 조합(자세 A·B × 위아래·팔 앞뒤 직선)의 평균, 세로 선 = 네 조합의 최소‒최대, 숫자 = 평균. 띠 = 베이스가 정지해 있을 때(네 조합의 최소‒최대).
실행: ~/nonmoon/.venv_pptx/bin/python fig_cond.py  → fig_v7/kroc_cond_1col.png (80 × 78 mm), fig_v7/kroc_cond_2col.png (170 × 52 mm), 600 dpi"""
import os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
SRC = H + "/figs/matrix_1124.md"
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.labelsize": 9.5, "xtick.labelsize": 8.5, "ytick.labelsize": 9.5, "legend.fontsize": 8.5,
                     "axes.linewidth": 0.6, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})
ROWS = ["턱 5 mm, 0.3 m/s", "턱 10 mm, 0.3 m/s", "턱 15 mm, 0.3 m/s", "턱 10 mm, 0.15 m/s", "턱 10 mm, 0.5 m/s", "패인 바닥 10 mm, 0.3 m/s", "평지 가감속, 0.5 m/s"]
TICK = ["5", "10", "15", "0.15", "0.5", "Pothole\n10 mm", "Flat\nground"]
# 원고 [Table 3] (세 번 평균): 자세 A 위아래, A 팔 앞뒤, B 위아래, B 팔 앞뒤
T3 = {"none": [(12.0, 2.59), (13.4, 2.76), (14.0, 3.18), (16.1, 3.40)], "prop": [(5.7, 1.27), (5.9, 1.21), (5.7, 1.54), (6.1, 1.43)]}


def load():
    txt = open(SRC, encoding="utf-8").read().split("## 보정 없음 대비 비")[0]; secs = txt.split("\n## ")[1:]; assert len(secs) == 4
    none = {r: [] for r in ROWS}; prop = {r: [] for r in ROWS}; rest = []
    for s in secs:
        rest.append(tuple(float(v) for v in re.search(r"AMR 정지.*?\*\*([\d.]+) / ([\d.]+)\*\*", s).groups()))
        for ln in s.splitlines():
            c = [x.strip() for x in ln.strip().strip("|").split("|")]
            if c and c[0] in none:
                none[c[0]].append(tuple(float(v) for v in c[1].split("/"))); prop[c[0]].append(tuple(float(v) for v in c[3].split("/")))
    none["턱 10 mm, 0.3 m/s"] = T3["none"]; prop["턱 10 mm, 0.3 m/s"] = T3["prop"]
    return {k: np.array([v[r] for r in ROWS]) for k, v in (("none", none), ("prop", prop))}, np.array(rest)   # (7 조건, 4 조합, 2 지표)


D, REST = load()
for k in ("none", "prop"):
    assert D[k].shape == (7, 4, 2), D[k].shape
print("조건별 평균 (최소‒최대) [mm]")
for i, r in enumerate(ROWS):
    for m, nm in ((0, "최댓값"), (1, "RMS")):
        a, b = D["none"][i, :, m], D["prop"][i, :, m]
        print(f"  {r:22s} {nm:4s} 보상 없음 {a.mean():5.2f} ({a.min():.2f}‒{a.max():.2f}) → (b)+(c) {b.mean():5.2f} ({b.min():.2f}‒{b.max():.2f})")
print("  베이스 정지: 최댓값", REST[:, 0].min(), "‒", REST[:, 0].max(), " RMS", REST[:, 1].min(), "‒", REST[:, 1].max())
CN, CP, CR = "#a6a6a6", "#1f4e79", "#f3d9b1"


def panel(ax, m, ylab, fmt, top, legend):
    x = np.arange(7); w = 0.38
    ax.axhspan(REST[:, m].min(), REST[:, m].max(), color=CR, lw=0, zorder=1, label="Base at rest")
    for k, c, off, lab, wt in (("none", CN, -w / 2, "No compensation", "normal"), ("prop", CP, w / 2, "Proposed", "bold")):
        v = D[k][:, :, m]; mu = v.mean(axis=1)
        ax.bar(x + off, mu, w, color=c, zorder=3, label=lab)
        ax.errorbar(x + off, mu, yerr=[mu - v.min(axis=1), v.max(axis=1) - mu], fmt="none", ecolor="k", elinewidth=0.5, capsize=1.3, capthick=0.5, zorder=4)
        for xi, vi, hi in zip(x + off, mu, v.max(axis=1)):
            ax.text(xi, hi + top * 0.012, fmt.format(vi), ha="center", va="bottom", fontsize=7.2, fontweight=wt, color="k" if k == "none" else CP, zorder=5)
    ax.set_xticks(x); ax.set_xticklabels(TICK, linespacing=0.9); ax.set_xlim(-0.6, 6.6); ax.set_ylim(0, top); ax.set_ylabel(ylab); ax.tick_params(axis="x", length=0)
    ax.yaxis.grid(True, alpha=0.3, lw=0.5); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for x0, x1, s in ((-0.45, 2.45, "Bump height [mm]\nat 0.3 m/s"), (2.55, 4.45, "Speed [m/s]\n10 mm bump")):
        ax.annotate("", xy=(x0, -0.17), xytext=(x1, -0.17), xycoords=("data", "axes fraction"), textcoords=("data", "axes fraction"), arrowprops=dict(arrowstyle="-", lw=0.5, color="k"), annotation_clip=False)
        ax.text((x0 + x1) / 2, -0.20, s, ha="center", va="top", fontsize=8.5, linespacing=0.9, transform=ax.get_xaxis_transform())
    if legend:
        ax.legend(frameon=False, loc="lower center", ncol=3, handlelength=1.0, handleheight=0.7, columnspacing=1.0, handletextpad=0.4, borderaxespad=0.0, bbox_to_anchor=(0.5, 1.02))


for name, size, grid in (("1col", (80, 88), (2, 1)), ("1col_short", (80, 64), (2, 1)), ("2col", (170, 52), (1, 2))):
    fig, ax = plt.subplots(*grid, figsize=(size[0] * MM, size[1] * MM), layout="constrained")
    panel(ax[0], 0, "Peak error [mm]", "{:.1f}", 26.0, True); panel(ax[1], 1, "RMS error [mm]", "{:.2f}", 5.6, name == "2col")
    fig.get_layout_engine().set(h_pad=0.02, w_pad=0.02, hspace=0.06, wspace=0.04)
    out = f"{H}/figs/kroc_cond_{name}.png"; fig.savefig(out, dpi=600); plt.close(fig); print("→", out)
