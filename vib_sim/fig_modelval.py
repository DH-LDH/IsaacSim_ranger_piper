"""model_val.py가 저장한 결과(out/modelval_eq_*.npz)로 검증 그림만 다시 그린다(식을 다시 풀지 않음) → figs/kroc_modelval.png
실행: ~/nonmoon/.venv_pptx/bin/python fig_modelval.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6})
fig, axs = plt.subplots(2, 2, figsize=(170 * MM, 78 * MM), sharex="col"); fig.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.12, hspace=0.12, wspace=0.17)
for c, (tag, lab) in enumerate((("A_tk15", "pose A"), ("H_tk15", "pose B"))):
    z = np.load(f"{H}/out/modelval_eq_{tag}.npz"); t, sim, e = z["t"], z["sim"], z["eq"]; t0 = t[int(z["i_f"])]
    for r, (j, jn) in enumerate(((1, "Joint 2"), (2, "Joint 3"))):
        ax = axs[r, c]; ax.plot(t - t0, np.degrees(sim[:, j]), color="k", lw=0.9, label="Simulation (physics engine)"); ax.plot(t - t0, np.degrees(e[:, j]), color="#d62728", lw=0.7, ls=(0, (4, 2)), label="Eqs. (1)–(4), (7)")
        ax.set_xlim(-0.2, 2.6); ax.tick_params(pad=2)
        if c == 0: ax.set_ylabel(f"{jn} deflection [deg]")
        if r == 0: ax.set_title(f"({'ab'[c]}) {lab}", fontsize=9.5, pad=3)
        if r == 1: ax.set_xlabel("Time from front-wheel contact [s]")
        if r == 0 and c == 0: ax.legend(loc="upper left", fontsize=8.5, frameon=False, handlelength=2.2, borderaxespad=0.2)
fig.savefig(H + "/figs/kroc_modelval.png", dpi=600); print("→ figs/kroc_modelval.png")
