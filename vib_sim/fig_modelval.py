"""model_val.py가 저장한 결과(out/modelval_eq_*.npz)로 검증 그림만 다시 그린다(식을 다시 풀지 않음). 600 dpi, 9.5 pt Times New Roman. 배치 셋:
  2col    → figs/kroc_modelval.png (170 × 78 mm, 두 단: 자세 A·B × 2·3번 관절)
  1col    → figs/kroc_modelval_1col.png (80 × 62 mm, 한 단: 자세 A의 2·3번 관절)
  1col_j2 → figs/kroc_modelval_1col_j2.png (80 × 38 mm, 한 단: 자세 A의 2번 관절만)
  1col_ab → figs/kroc_modelval_1col_ab.png (80 × 66 mm, 한 단: 자세 A·B × 2·3번 관절, 범례는 위에 한 줄, 세로축은 줄마다 같이 씀)
실행: ~/nonmoon/.venv_pptx/bin/python fig_modelval.py [2col|1col|1col_j2|1col_ab|all]   (기본 2col)
범례의 식 번호와 파일 이름 꼬리는 환경 변수로: EQ_LABEL="Eqs. (1)–(4)" OUT_SUFFIX=_eq1-4 (11/26, 원고에서 식을 줄여 번호가 바뀜)"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
EQ_LABEL = os.environ.get("EQ_LABEL", "Eqs. (1)–(4), (7)"); SUF = os.environ.get("OUT_SUFFIX", "")   # 원고의 식 번호가 바뀌면: EQ_LABEL="Eqs. (1)–(4)" OUT_SUFFIX=_eq1-4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6})
LAY = {"2col": (170.0, 78.0, "kroc_modelval", (("A_tk15", "posture A"), ("H_tk15", "posture B")), ((1, "Joint 2"), (2, "Joint 3")), dict(left=0.08, right=0.97, top=0.93, bottom=0.12, hspace=0.12, wspace=0.17)),
       "1col": (80.0, 62.0, "kroc_modelval_1col", (("A_tk15", "pose A"),), ((1, "Joint 2"), (2, "Joint 3")), dict(left=0.235, right=0.975, top=0.975, bottom=0.155, hspace=0.12)),
       "1col_j2": (80.0, 38.0, "kroc_modelval_1col_j2", (("A_tk15", "pose A"),), ((1, "Joint 2"),), dict(left=0.235, right=0.975, top=0.96, bottom=0.25)),
       "1col_ab": (80.0, 66.0, "kroc_modelval_1col_ab", (("A_tk15", "posture A"), ("H_tk15", "posture B")), ((1, "Joint 2"), (2, "Joint 3")), dict(left=0.235, right=0.975, top=0.865, bottom=0.165, hspace=0.12, wspace=0.10))}


def build(layout):
    FW, FH, name, poses, joints, adj = LAY[layout]; narrow = FW < 100 and len(poses) > 1                # 한 단에 두 자세: 범례와 가로축 이름을 그림 전체에 하나씩
    fig, axs = plt.subplots(len(joints), len(poses), figsize=(FW * MM, FH * MM), sharex="col", sharey="row" if narrow else False, squeeze=False); fig.subplots_adjust(**adj)
    for c, (tag, lab) in enumerate(poses):
        z = np.load(f"{H}/out/modelval_eq_{tag}.npz"); t, sim, e = z["t"], z["sim"], z["eq"]; t0 = t[int(z["i_f"])]
        for r, (j, jn) in enumerate(joints):
            ax = axs[r, c]; ax.plot(t - t0, np.degrees(sim[:, j]), color="k", lw=0.9, label="Simulation" if FW < 100 else "Simulation (physics engine)"); ax.plot(t - t0, np.degrees(e[:, j]), color="#d62728", lw=0.7, ls=(0, (4, 2)), label=EQ_LABEL)
            ax.set_xlim(-0.2, 2.6); ax.tick_params(pad=2)
            if narrow: ax.set_xticks([0, 1, 2])
            if c == 0: ax.set_ylabel(f"{jn}\ndeflection [deg]" if FW < 100 else f"{jn} deflection [deg]", linespacing=0.95)
            if r == 0 and len(poses) > 1: ax.set_title(f"({'ab'[c]}) {lab}", fontsize=9.5, pad=3)
            if r == len(joints) - 1 and not narrow: ax.set_xlabel("Time from front-wheel contact [s]")
            if r == 0 and c == 0 and not narrow: ax.legend(loc="upper left", fontsize=8.5, frameon=False, handlelength=2.0, borderaxespad=0.2, labelspacing=0.15)
    if narrow:
        xc = (adj["left"] + adj["right"]) / 2; fig.text(xc, 0.012, "Time from front-wheel contact [s]", ha="center", va="bottom")
        fig.legend(*axs[0, 0].get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(0.5, 1.005), ncol=2, frameon=False, handlelength=2.0, columnspacing=1.2, borderaxespad=0.1)
    fig.savefig(f"{H}/figs/{name}{SUF}.png", dpi=600); plt.close(fig); print(f"→ figs/{name}{SUF}.png")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "2col"
    for lay in (tuple(LAY) if which == "all" else (which,)): build(lay)
