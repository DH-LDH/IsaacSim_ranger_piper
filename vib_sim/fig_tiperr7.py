"""KROS 초안 v7의 그림 6: 턱 10 mm를 넘을 때의 끝단 위치 오차(3차원 거리) 시간 파형 → figs/kroc7_tiperr.png (80 × 36 mm, 800 dpi)
fig_kroc6.py의 Fig. 3과 같은 자료·같은 그림이고 세로축 이름만 본문의 용어(tip position error)로 맞췄다. 자세 B, 위아래 직선.
실행: ~/nonmoon/.venv_pptx/bin/python fig_tiperr7.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import summ_line as S
from gen_line import fk
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; MM = 1 / 25.4; PROP = "kinprev+prevzE10+ff10"
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.labelsize": 9.5, "xtick.labelsize": 9.5, "ytick.labelsize": 9.5, "legend.fontsize": 9.5,
                     "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})


def series(tag, case):
    """원고 식 (13)의 e의 크기[mm]: 월드 기준, 베이스가 진행한 수평 거리만 뺌"""
    z = np.load(f"{O}/exp6_ts_{tag}_{case}.npz"); t = z["t"]; pr = np.array([fk(S.CH, q) for q in z["qd"][::5]]); dref = np.repeat((pr - pr[0]) @ S.RM.T, 5, axis=0)[:len(t)]
    w = (z["ep"] - z["ep"][0]) - dref; w[:, :2] -= (z["bp"] - z["bp"][0])[:, :2]; return t, np.linalg.norm(w, axis=1) * 1e3, float(z["tf"]), float(z["tr"])


fig, ax = plt.subplots(figsize=(80 * MM, 36 * MM), layout="constrained")
for case, lab, c, lw in (("none", "No comp.", "#999999", 0.7), (PROP, "(b) + (c)", "#1f4e79", 0.7)):
    t, dev, tf, tr = series("rlH_z_j23_base", case); ax.plot(t, dev, color=c, lw=lw, label=lab); print(case, "최대", round(float(dev[(t > 0.3) & (t < t[-1] - 0.3)].max()), 2), "mm")
for x_, s_ in ((tf, "front"), (tr, "rear")):
    ax.axvline(x_, color="k", lw=0.5, ls=":"); ax.text(x_, 1.02, s_, fontsize=8.5, ha="center", transform=ax.get_xaxis_transform())
ax.set_xlim(0.3, t[-1] - 0.3); ax.set_ylim(0, None); ax.set_xlabel("Time [s]"); ax.set_ylabel("Tip position\nerror [mm]", linespacing=0.95)
ax.legend(frameon=False, loc="upper right", handlelength=1.0, labelspacing=0.1, borderaxespad=0.1)
fig.savefig(H + "/figs/kroc7_tiperr.png", dpi=800); print("→ figs/kroc7_tiperr.png")
