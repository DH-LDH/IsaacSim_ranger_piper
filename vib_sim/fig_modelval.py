"""model_val.py가 저장한 결과(out/modelval_eq_*.npz)로 검증 그림만 다시 그린다(식을 다시 풀지 않음). 600 dpi, 9.5 pt Times New Roman. 배치 셋:
  2col    → figs/kroc_modelval.png (170 × 78 mm, 두 단: 자세 A·B × 2·3번 관절)
  1col    → figs/kroc_modelval_1col.png (80 × 62 mm, 한 단: 자세 A의 2·3번 관절)
  1col_j2 → figs/kroc_modelval_1col_j2.png (80 × 38 mm, 한 단: 자세 A의 2번 관절만)
실행: ~/nonmoon/.venv_pptx/bin/python fig_modelval.py [2col|1col|1col_j2|all]   (기본 2col)"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6})
LAY = {"2col": (170.0, 78.0, "kroc_modelval", (("A_tk15", "pose A"), ("H_tk15", "pose B")), ((1, "Joint 2"), (2, "Joint 3")), dict(left=0.08, right=0.97, top=0.93, bottom=0.12, hspace=0.12, wspace=0.17)),
       "1col": (80.0, 62.0, "kroc_modelval_1col", (("A_tk15", "pose A"),), ((1, "Joint 2"), (2, "Joint 3")), dict(left=0.235, right=0.975, top=0.975, bottom=0.155, hspace=0.12)),
       "1col_j2": (80.0, 38.0, "kroc_modelval_1col_j2", (("A_tk15", "pose A"),), ((1, "Joint 2"),), dict(left=0.235, right=0.975, top=0.96, bottom=0.25))}


def build(layout):
    FW, FH, name, poses, joints, adj = LAY[layout]; fig, axs = plt.subplots(len(joints), len(poses), figsize=(FW * MM, FH * MM), sharex="col", squeeze=False); fig.subplots_adjust(**adj)
    for c, (tag, lab) in enumerate(poses):
        z = np.load(f"{H}/out/modelval_eq_{tag}.npz"); t, sim, e = z["t"], z["sim"], z["eq"]; t0 = t[int(z["i_f"])]
        for r, (j, jn) in enumerate(joints):
            ax = axs[r, c]; ax.plot(t - t0, np.degrees(sim[:, j]), color="k", lw=0.9, label="Simulation" if FW < 100 else "Simulation (physics engine)"); ax.plot(t - t0, np.degrees(e[:, j]), color="#d62728", lw=0.7, ls=(0, (4, 2)), label="Eqs. (1)–(4), (7)")
            ax.set_xlim(-0.2, 2.6); ax.tick_params(pad=2)
            if c == 0: ax.set_ylabel(f"{jn}\ndeflection [deg]" if FW < 100 else f"{jn} deflection [deg]", linespacing=0.95)
            if r == 0 and len(poses) > 1: ax.set_title(f"({'ab'[c]}) {lab}", fontsize=9.5, pad=3)
            if r == len(joints) - 1: ax.set_xlabel("Time from front-wheel contact [s]")
            if r == 0 and c == 0: ax.legend(loc="upper left", fontsize=8.5, frameon=False, handlelength=2.0, borderaxespad=0.2, labelspacing=0.15)
    fig.savefig(f"{H}/figs/{name}.png", dpi=600); plt.close(fig); print(f"→ figs/{name}.png")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "2col"
    for lay in (tuple(LAY) if which == "all" else (which,)): build(lay)
