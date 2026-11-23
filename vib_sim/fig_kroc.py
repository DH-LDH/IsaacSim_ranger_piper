"""KRoC 양식용 그림(300 dpi 이상·1000 픽셀 이상, 그림 안 글자는 본문과 같은 9.5 pt, 캡션은 영어). 원고에 넣는 크기 그대로 만든다: Fig. 1은 40 × 33 mm, Fig. 2는 80 × 40 mm.
실행: ~/nonmoon/.venv_pptx/bin/python fig_kroc.py  →  figs/kroc_fig1.png, figs/kroc_fig2.png
입력: out/exp3_replay_c1.csv, out/exp6_tk5_s{1,2,3}_{pos,pd,asmc3}.csv, out/exp6_tk5r_s{1,2,3}_asmc3.csv"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; CAM = 1.27
# KRoC 양식: 그림 안 글자는 본문과 같은 9.5 pt, 300 dpi 이상·1,000 픽셀 이상.
# 그림을 "원고에 넣는 크기 그대로" 만든다(W1·W2 mm) — 넣을 때 줄이거나 키우면 글자 크기가 달라진다. 글꼴은 본문의 영문 글꼴인 Times New Roman(없으면 폭이 같은 Liberation Serif로 바꿔 쓴다).
W1, H1, W2, H2, DPI = 40.0, 33.0, 80.0, 40.0, 800      # 11/23 저녁: 2쪽에 맞추려고 납작하게(앞선 값 48 × 39, 80 × 50 mm). 800 dpi면 40 mm 폭도 1,000픽셀을 넘는다
MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.labelsize": 9.5, "xtick.labelsize": 9.5, "ytick.labelsize": 9.5, "legend.fontsize": 9.5,
                     "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})

# ---- Fig. 1: 실측 대 모형(엔코더 재생), 18조건. 표식의 뜻은 캡션에 적는다(5: 원, 10: 네모, 15 deg/s: 세모) ----
rows = list(csv.DictReader(open(O + "/exp3_replay_c1.csv")))
fig, ax = plt.subplots(figsize=(W1 * MM, H1 * MM), layout="constrained")
for v, c, m in ((5, "#1f77b4", "o"), (10, "#ff7f0e", "s"), (15, "#d62728", "^")):
    r = [x for x in rows if float(x["cond"].split("_v")[1].split("_")[0]) == v]
    ax.scatter([float(x["real_gyro_rms"]) for x in r], [float(x["stop_pitch_5_10"]) * CAM for x in r], s=9, color=c, marker=m, zorder=3)
lim = 0.12; ax.plot([0, lim], [0, lim], "k--", lw=0.6); ax.set_xlim(0, lim); ax.set_ylim(0, lim); ax.set_aspect("equal"); ax.set_xticks([0, 0.05, 0.1]); ax.set_yticks([0, 0.05, 0.1]); ax.set_xticklabels(["0", "0.05", "0.1"]); ax.set_yticklabels(["0", "0.05", "0.1"])
ax.set_xlabel("Measured [rad/s]"); ax.set_ylabel("Model [rad/s]")
fig.savefig(H + "/figs/kroc_fig1.png", dpi=DPI); plt.close(fig)

# ---- Fig. 2 (토크 제어 모드 기준): 종합 장면(팔 동작 + 베이스 출발 → 턱 → 정지) — (a) 궤적 오차, (b) 흔들림 ----
# 앞선 판(위치 모드 기록 재생, 시간 파형 + 막대)은 figs/kroc_fig2_v3.png로 남겼다.
def L(tag): return {r["case"]: r for r in csv.DictReader(open(O + f"/exp6_{tag}.csv"))}
LAY = "bz+prevzE10+ff10+dampRJ"
COL = {"Position": "#999999", "Torque, PD": "#f0b27a", "+ tracking": "#5b9bd5", "+ base layers": "#1f4e79", "Arm only": "#d9d9d9"}
fig, ax = plt.subplots(1, 2, figsize=(W2 * MM, H2 * MM), layout="constrained")
A = [("pos", "none", "Position"), ("pd", "none", "Torque, PD"), ("asmc3", "none", "+ tracking"), ("asmc3", LAY, "+ base layers")]
for i, n in enumerate("123"):
    for k, (m, c, lab) in enumerate(A):
        ax[0].bar(i + (k - 1.5) * 0.2, float(L(f"tk5_s{n}_{m}")[c]["tip_err_mm"]), 0.2, color=COL[lab], zorder=3)
B = [("r", "none", "Arm only"), ("", "none", "+ tracking"), ("", LAY, "+ base layers")]
for i, n in enumerate("123"):
    for k, (r_, c, lab) in enumerate(B):
        ax[1].bar(i + (k - 1) * 0.26, float(L(f"tk5{r_}_s{n}_asmc3")[c]["tip_vib_mm"]), 0.26, color=COL[lab], zorder=3)
ax[0].set_ylabel("Path error [mm]"); ax[1].set_ylabel("Deviation [mm]")
for a_, t_ in zip(ax, ("(a)", "(b)")):
    a_.set_xticks(range(3)); a_.set_xticklabels(["1", "2", "3"]); a_.set_xlabel("Moving joints"); a_.grid(axis="x", visible=False)
    a_.set_title(t_, loc="left", fontsize=9.5, pad=2)
hd = [plt.Rectangle((0, 0), 1, 1, color=c) for c in COL.values()]
fig.legend(hd, list(COL.keys()), loc="outside lower center", ncol=3, frameon=False, handlelength=1.0, columnspacing=1.0, labelspacing=0.05, handletextpad=0.4, borderaxespad=0.0)
fig.savefig(H + "/figs/kroc_fig2.png", dpi=DPI); plt.close(fig); print("ok")
