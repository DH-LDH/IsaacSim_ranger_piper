"""KROS 초안 v6의 그림 넷(KRoC 양식: 그림 안 글자 9.5 pt Times New Roman, 넣는 크기 그대로, 800 dpi).
실행: ~/nonmoon/.venv_pptx/bin/python fig_kroc6.py  →  figs/kroc6_fig1.png(40 × 33 mm) · fig2(80 × 46) · fig3(80 × 36) · fig4(80 × 44)
입력: out/exp3_ts_kf6_*(재생 대조), out/exp6_ts_rl*_j23_*(이동 베이스), piper_ws docs/data/raw(실측 IMU). 원고의 "자세 B" = 그리퍼가 앞을 보는 자세(vib_sim의 H)."""
import glob, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
import summ_line as S
from kfit_cmp import real
from scn_seg import seg_rms
from gen_line import fk
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; MM = 1 / 25.4; DPI = 800
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.labelsize": 9.5, "xtick.labelsize": 9.5, "ytick.labelsize": 9.5, "legend.fontsize": 9.5,
                     "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})
CA, CB = "#1f77b4", "#d62728"          # 자세 A, 자세 B
PROP = "kinprev+prevzE10+ff10"

# ---- Fig. 1: 끝단 각속도(2–10 Hz) 모형 대 실측, 두 자세의 기록 14개 ----
fig, ax = plt.subplots(figsize=(40 * MM, 33 * MM), layout="constrained")
for p in sorted(glob.glob(O + "/exp3_ts_kf6_*_fixed.npz")):
    st = os.path.basename(p)[len("exp3_ts_kf6_"):-len("_fixed.npz")]; tu, G, tend, full = real(st); rs, _ = seg_rms(tu, G, 400.0, 1.0, tend - 1.0, [])
    d = np.load(p); ss, _ = seg_rms(d["t"], d["gyro"], 1000.0, 1.0, tend - 1.0, []); isB = "poseH" in full or "_H_" in full
    ax.scatter(rs["2_10"], ss["2_10"] * 1.27, s=10, color=CB if isB else CA, marker="s" if isB else "o", zorder=3)
lim = 0.08; ax.plot([0, lim], [0, lim], "k--", lw=0.6); ax.set_xlim(0, lim); ax.set_ylim(0, lim); ax.set_aspect("equal"); ax.set_xticks([0, 0.04, 0.08]); ax.set_yticks([0, 0.04, 0.08])
ax.set_xticklabels(["0", "0.04", "0.08"]); ax.set_yticklabels(["0", "0.04", "0.08"]); ax.set_xlabel("Measured [rad/s]"); ax.set_ylabel("Model [rad/s]")
fig.savefig(H + "/figs/kroc6_fig1.png", dpi=DPI); plt.close(fig)

# ---- Fig. 2: 제어 구조 ----
W, HT = 80.0, 46.0
fig = plt.figure(figsize=(W * MM, HT * MM)); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, HT); ax.axis("off"); ax.grid(False)
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
box(57.0, yt, 22.0, 7.5, "Mobile base", GY)
box(3.0, yt, 35.0, 7.5, "Rear-wheel prediction", BL)
arr((57.0, yt + 3.75), (38.0, yt + 3.75)); ax.text(47.5, yt + 4.6, r"$\delta z,\,\delta\boldsymbol{\theta},\,\boldsymbol{a}_b$", ha="center", va="bottom", fontsize=9.5)
box(3.0, ym, 35.0, 8.0, r"(b) $-\boldsymbol{J}^{-1}(\delta z+\delta\boldsymbol{\theta}\times\boldsymbol{r})$", OR)
box(42.0, ym, 22.0, 8.0, r"(c) $\boldsymbol{K}^{-1}\boldsymbol{C}\boldsymbol{a}_b$", OR)
arr((16.0, yt), (16.0, ym + 8.0))
line([(30.0, yt), (30.0, ym + 11.5), (53.0, ym + 11.5)]); arr((53.0, ym + 11.5), (53.0, ym + 8.0))
ys = ya + 3.75
line([(53.0, ym), (53.0, 18.0), (28.0, 18.0)]); summ(26.5, 18.0); arr((26.5, ym), (26.5, 19.5))
line([(25.0, 18.0), (6.0, 18.0)]); arr((6.0, 18.0), (6.0, ys + 1.5)); summ(6.0, ys)
ax.text(7.2, 14.8, r"$\Delta\boldsymbol{q}$", ha="left", va="center", fontsize=9.5)
ax.text(0.2, ys - 1.2, "Path", ha="left", va="top", fontsize=8.5); arr((0.3, ys), (4.5, ys))
arr((7.5, ys), (11.0, ys)); box(11.0, ya, 21.0, 7.5, "(a) Tracking\ncontrol, J2-3", GY)
arr((32.0, ys), (35.5, ys)); box(35.5, ya, 16.5, 7.5, "Built-in\nPD, motor", GY)
arr((52.0, ys), (55.5, ys)); box(55.5, ya, 23.5, 7.5, "Elastic joints,\nlinks", GY)
arr((68.0, yt), (68.0, ya + 7.5), ls=(0, (3, 2))); ax.text(69.3, 26.0, "base\nmotion", ha="left", va="center", fontsize=8.5, linespacing=0.9)
fig.savefig(H + "/figs/kroc6_fig2.png", dpi=DPI); plt.close(fig)

# ---- 월드 기준으로 직선에서 벗어난 거리의 시계열 ----
def series(tag, case):
    z = np.load(f"{O}/exp6_ts_{tag}_{case}.npz"); t = z["t"]
    pr = np.array([fk(S.CH, q) for q in z["qd"][::5]]); dref = np.repeat((pr - pr[0]) @ S.RM.T, 5, axis=0)[:len(t)]
    d = dref[np.argmax(np.linalg.norm(dref, axis=1))]; d = d / np.linalg.norm(d)
    w = (z["ep"] - z["ep"][0]) - dref; w[:, :2] -= (z["bp"] - z["bp"][0])[:, :2]
    return t, np.linalg.norm(S.perp(w, d), axis=1) * 1e3, float(z["tf"]), float(z["tr"])

# ---- Fig. 3: 자세 B, 위아래 직선, 턱 10 mm ----
fig, ax = plt.subplots(figsize=(80 * MM, 36 * MM), layout="constrained")
for case, lab, c, lw in (("none", "No comp.", "#999999", 0.7), (PROP, "(b) + (c)", "#1f4e79", 0.7)):
    t, dev, tf, tr = series("rlH_z_j23_base", case); ax.plot(t, dev, color=c, lw=lw, label=lab)
for x_, s_ in ((tf, "front"), (tr, "rear")):
    ax.axvline(x_, color="k", lw=0.5, ls=":"); ax.text(x_, 1.02, s_, fontsize=8.5, ha="center", transform=ax.get_xaxis_transform())
ax.set_xlim(0.3, t[-1] - 0.3); ax.set_ylim(0, None); ax.set_xlabel("Time [s]"); ax.set_ylabel("Deviation [mm]")
ax.legend(frameon=False, loc="upper right", handlelength=1.0, labelspacing=0.1, borderaxespad=0.1)
fig.savefig(H + "/figs/kroc6_fig3.png", dpi=DPI); plt.close(fig)

# ---- Fig. 4: 네 경우의 최댓값과 실효값 ----
def mean(pre, k, case, key, still=False):
    if still: return S.one(f"{pre}_{k}_j23_still", "none")[key]
    x = [S.one(f"{pre}_{k}_j23_base", case)] + [S.one(f"{pre}_{k}_j23_base_s{sd}", case) for sd in (2, 3)]; return float(np.mean([r[key] for r in x if r]))
G = [("rl", "z", "A, V"), ("rl", "x", "A, F"), ("rlH", "z", "B, V"), ("rlH", "x", "B, F")]
B = [("rest", "At rest", "#d9d9d9"), ("none", "No comp.", "#999999"), ("kinmap", "(b)", "#5b9bd5"), (PROP, "(b) + (c)", "#1f4e79")]
fig, ax = plt.subplots(1, 2, figsize=(80 * MM, 44 * MM), layout="constrained")
for a_, key, yl in ((ax[0], "w_pk", "Peak [mm]"), (ax[1], "w_rms", "RMS [mm]")):
    for i, (pre, k, _) in enumerate(G):
        for j, (case, lab, c) in enumerate(B):
            a_.bar(i + (j - 1.5) * 0.2, mean(pre, k, case, key, still=(case == "rest")), 0.2, color=c, zorder=3, label=lab if i == 0 else None)
    a_.set_xticks(range(4)); a_.set_xticklabels([g[2] for g in G], fontsize=8.5); a_.set_ylabel(yl); a_.grid(axis="x", visible=False); a_.tick_params(axis="x", length=0)
hd, lb = ax[0].get_legend_handles_labels(); fig.legend(hd, lb, frameon=False, loc="outside upper center", ncol=4, handlelength=0.8, columnspacing=0.8, handletextpad=0.3, borderaxespad=0.0, fontsize=8.5)
fig.savefig(H + "/figs/kroc6_fig4.png", dpi=DPI); plt.close(fig); print("ok")
