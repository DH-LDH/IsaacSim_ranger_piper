"""논문용 그림·표. 실행: ~/nonmoon/.venv_pptx/bin/python make_figs.py   (Isaac 필요 없음)
입력: out/*.csv, out/*_ts_*.npz, 실측 ~/piper_ws/docs/data/c1_stops.csv·d1a_stops.csv
출력: figs/fig_valid.png, figs/fig_amr.png, figs/fig_susp.png, figs/summary.md
"""
import csv, glob, os, statistics as S
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

H = os.path.dirname(os.path.abspath(__file__)); O = H + "/out"; F = H + "/figs"; os.makedirs(F, exist_ok=True)
REAL = os.path.expanduser("~/piper_ws/docs/data")
CAM = 1.27   # 카메라 자리 / 그리퍼 몸통 끄덕임 비 (FW191_B_LOG) — 실측과 절댓값을 댈 때만 곱함
plt.rcParams.update({"font.size": 8, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 200, "savefig.bbox": "tight"})
C = {"none": "#555555", "scurve": "#1f77b4", "zv": "#ff7f0e", "zvd": "#d62728", "armff": "#9467bd", "scurve+armff": "#2ca02c", "zvd+armff": "#8c564b"}
LAB = {"none": "Trapezoid (no shaping)", "scurve": "Cosine accel.", "zv": "ZV shaper", "zvd": "ZVD shaper", "armff": "Arm feedforward",
       "scurve+armff": "Cosine + arm FF", "zvd+armff": "ZVD + arm FF"}


def rd(p):
    with open(p) as f:
        return [{k: (float(v) if v.replace(".", "", 1).replace("-", "", 1).replace("e", "", 1).replace("+", "", 1).isdigit() else v) for k, v in r.items()} for r in csv.DictReader(f)]


def band(x, fs, lo=5.0, hi=10.0):
    n = len(x); tt = np.arange(n); x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)


md = ["# 시뮬레이션 결과 요약 (make_figs.py가 생성)\n"]

# ---------------- 그림 1: 고정 베이스 검증 ----------------
sim = rd(O + "/exp1_fixed.csv")
real = rd(REAL + "/c1_stops.csv")
fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.1), gridspec_kw={"width_ratios": [1.35, 1, 1]})
d = np.load(O + "/exp1_ts_v10_d+10_none.npz"); t = d["t"] - float(d["t0"])
ax[0].plot(t, np.degrees(d["vm"]), color="#1f77b4", lw=0.9, label="motor-side joint vel. [deg/s]")
ax[0].plot(t, np.degrees(d["gyro"]) * CAM, color="#d62728", lw=0.9, label="EE pitch rate [deg/s]")
ax[0].set_xlim(-1.3, 1.5); ax[0].set_xlabel("time from encoder stop [s]"); ax[0].set_title("(a) Sim: J2 10 deg/s x 10 deg, stop", fontsize=8); ax[0].legend(fontsize=6, loc="lower left")
sp = [5, 10, 15]
rs = {v: [abs(r["gyro_rms"]) for r in real if r["speed_dps"] == v] for v in sp}
ss = {v: [r["rms"] * CAM for r in sim if r["speed"] == v and r["shape"] == "none"] for v in sp}
bp = ax[1].boxplot([rs[v] for v in sp], positions=np.array(sp) - 0.9, widths=1.4, patch_artist=True, showfliers=False, medianprops={"color": "k"})
for b in bp["boxes"]: b.set_facecolor("#bbbbbb")
for v in sp:
    ax[1].scatter([v + 0.9] * len(ss[v]), ss[v], s=10, color="#d62728", zorder=3)
ax[1].set_xticks(sp); ax[1].set_xticklabels(sp); ax[1].set_xlabel("joint speed [deg/s]"); ax[1].set_ylabel("residual 5-10 Hz rms [rad/s]")
ax[1].set_title("(b) Real (box, n=30 each) vs sim (dots)", fontsize=8)
# 입력 성형: 실측(11/18) 대 시뮬레이션(모드 일치 / 6.3 Hz / 6.06 Hz)
shp = {}
for tag, lab in (("", "7.0"), ("_ks0.81", "6.3"), ("_ks0.735", "6.06")):
    rows = rd(O + f"/exp1_fixed{tag}.csv")
    for v in (10, 15):
        base = [r["rms"] for r in rows if r["speed"] == v and r["dist"] == 10 and r["shape"] == "none"][0]
        for s_ in ("zv", "zvd"):
            shp[(lab, v, s_)] = [r["rms"] for r in rows if r["speed"] == v and r["shape"] == s_][0] / base
realshape = {(10, "zv"): 0.42, (15, "zv"): 0.72, (10, "zvd"): 0.28, (15, "zvd"): 0.29}   # FW191_D_LOG 5.2
x = np.arange(2); w = 0.2
ax[2].bar(x - 1.5 * w, [np.mean([realshape[(v, s_)] for v in (10, 15)]) for s_ in ("zv", "zvd")], w, color="#bbbbbb", label="real (mode 6.0-6.6 Hz)")
for i, lab in enumerate(("7.0", "6.3", "6.06")):
    ax[2].bar(x + (i - 0.5) * w, [np.mean([shp[(lab, v, s_)] for v in (10, 15)]) for s_ in ("zv", "zvd")], w, color=["#f4a3a3", "#e05a5a", "#a01010"][i], label=f"sim, mode {lab} Hz")
ax[2].set_xticks(x); ax[2].set_xticklabels(["ZV", "ZVD"]); ax[2].set_ylabel("ratio to unshaped"); ax[2].set_title("(c) Input shaping at 7.0 Hz", fontsize=8); ax[2].legend(fontsize=5.5)
fig.tight_layout(); fig.savefig(F + "/fig_valid.png"); plt.close(fig)

md.append("## 1. 고정 베이스 검증 (J2 등속 후 정지, 자세 A)\n")
md.append("시뮬레이션 값은 그리퍼 몸통 기준 값에 카메라 자리 비 1.27을 곱한 것. 실측은 c1_stops.csv(11/17, 90회).\n")
md.append("| 속도 | 실측 rms 중앙 (조건 중앙의 최소–최대) | 시뮬레이션 rms (거리 6조건 최소–최대) | 실측 창 최대 | 시뮬레이션 창 최대 |\n|---|---|---|---|---|")
for v in sp:
    cm = [S.median([abs(r["gyro_rms"]) for r in real if r["speed_dps"] == v and r["dist_deg"] == dd]) for dd in (-10, -5, -2.5, 2.5, 5, 10)]
    pm = [S.median([abs(r["max_rads"]) for r in real if r["speed_dps"] == v and r["dist_deg"] == dd]) for dd in (-10, -5, -2.5, 2.5, 5, 10)]
    sp_ = [r["peak"] * CAM for r in sim if r["speed"] == v and r["shape"] == "none"]
    md.append(f"| {v} °/s | {S.median(rs[v]):.4f} ({min(cm):.4f}–{max(cm):.4f}) | {min(ss[v]):.4f}–{max(ss[v]):.4f} | {min(pm):.3f}–{max(pm):.3f} | {min(sp_):.3f}–{max(sp_):.3f} |")
md.append("\n입력 성형(성형기 7.0 Hz·ζ 0.10, 성형 없음 대비 rms 비):\n\n| | 실측 11/18 (그날 모드 6.0–6.6 Hz) | 시뮬레이션 모드 7.0 Hz | 6.3 Hz | 6.06 Hz |\n|---|---|---|---|---|")
for s_ in ("zv", "zvd"):
    md.append(f"| {s_.upper()} 10 / 15 °/s | {realshape[(10, s_)]} / {realshape[(15, s_)]} | " + " | ".join(f"{shp[(lab, 10, s_)]:.2f} / {shp[(lab, 15, s_)]:.2f}" for lab in ("7.0", "6.3", "6.06")) + " |")

# ---------------- 그림 2: AMR 출발–정지 (실물 장착: 팔 앞이 AMR 옆) ----------------
# 지표: 감속 명령이 끝난 뒤 2 s 창, 끝단 자이로 세 축의 2–20 Hz 벡터 rms (stop_vec). 직진 가감속은 좌우 모드를 흔든다.
main = rd(O + "/exp2_main.csv")
b0 = [r for r in main if r["case"] == "none"][0]
fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.2), gridspec_kw={"width_ratios": [1.5, 1, 1.1]})
for c in ("none", "scurve", "zvd"):
    d = np.load(O + f"/exp2_ts_main_{c}_a1_T0.5.npz"); t = d["t"]
    g = band(d["gyro"][:, 2], 1000.0, 0.0, 20.0)
    ax[0].plot(t - float(d["t_stop_end"]), g, color=C[c], lw=0.7, label=LAB[c])
ax[0].set_xlim(-2.7, 1.5); ax[0].set_xlabel("time from end of AMR decel. command [s]"); ax[0].set_ylabel("EE yaw rate [rad/s]")
ax[0].set_title("(a) AMR 0 -> 0.5 m/s -> 0 at 1 m/s$^2$, arm sideways", fontsize=8); ax[0].legend(fontsize=6, loc="upper left")
sw = rd(O + "/exp2_sweepTa.csv")
ax[1].plot([r["Ta"] for r in sw], [r["stop_vec"] * 1e3 for r in sw], "o-", ms=2.5, lw=0.8, color=C["none"], label=LAB["none"])
t4 = rd(O + "/exp2_Ta4.csv") + main
for c in ("scurve", "zv", "zvd"):
    rr = sorted([r for r in t4 if r["case"] == c], key=lambda r: r["Ta"])
    ax[1].plot([r["Ta"] for r in rr], [r["stop_vec"] * 1e3 for r in rr], "s-", ms=2.5, lw=0.8, color=C[c], label=LAB[c])
ax[1].set_xlabel("acceleration time $T_a$ [s]"); ax[1].set_ylabel("residual rms [mrad/s]"); ax[1].set_title("(b) vs. accel. time (1 m/s$^2$)", fontsize=8); ax[1].legend(fontsize=5.5)
rob = {200.0: main}
for k1 in (140, 170, 260):
    p = O + f"/exp2_k1_{k1}.csv"
    if os.path.exists(p): rob[float(k1)] = rd(p)
kss = sorted(rob); fl = {k: [r["f"] for r in rob[k] if r["case"] == "none"][0] for k in kss}
for c in ("scurve", "zv", "zvd", "scurve+armff"):
    y = [[r["stop_vec"] for r in rob[k] if r["case"] == c][0] / [r["stop_vec"] for r in rob[k] if r["case"] == "none"][0] for k in kss]
    ax[2].plot([fl[k] for k in kss], y, "o-", ms=3, lw=0.9, color=C[c], label=LAB[c])
ax[2].axvline(fl[200.0], color="k", lw=0.5, ls=":"); ax[2].set_xlabel(f"actual lateral mode [Hz] (design: {fl[200.0]:.1f})"); ax[2].set_ylabel("ratio to trapezoid")
ax[2].set_title("(c) Robustness to mode shift", fontsize=8); ax[2].legend(fontsize=5.5)
fig.tight_layout(); fig.savefig(F + "/fig_amr.png"); plt.close(fig)

arm10 = [r["rms"] for r in sim if r["speed"] == 10 and r["dist"] == 10 and r["shape"] == "none"][0]
md.append("\n## 2. AMR 직선 출발–정지 — 실물 장착 방향(팔 앞이 AMR 옆, 90°), 강체 섀시, 자세 A 유지\n")
md.append("0 → 0.5 m/s → 0, 1 m/s², 가속 시간 0.5 s. 지표는 감속 명령이 끝난 뒤 2 s 창의 끝단 자이로 세 축 2–20 Hz 벡터 rms(그리퍼 몸통 기준).\n")
md.append(f"대책 없음: {b0['stop_vec']:.5f} rad/s (비틀림 {b0['stop_roll']:.5f}, 끄덕임 {b0['stop_pitch']:.5f}, 좌우 {b0['stop_yaw']:.5f}), 벡터 최대 {b0['stop_peak']:.4f} rad/s, 주로 {b0['axis']} {b0['f']:.1f} Hz. "
          f"끄덕임 5–10 Hz rms는 {b0['stop_pitch_5_10']:.5f}. 같은 모형의 팔 J2 10 °/s 정지(끄덕임 {arm10:.4f})와 견주면 {b0['stop_vec'] / arm10:.2f}배.\n")
md.append("| 조건 | 출발 뒤 비 | 정지 뒤 비 | 정지 뒤 최대 비 | 이동 시간 [s] |\n|---|---|---|---|---|")
for r in main:
    md.append(f"| {r['case']} | {r['go_vec'] / b0['go_vec']:.2f} | {r['stop_vec'] / b0['stop_vec']:.3f} | {r['stop_peak'] / b0['stop_peak']:.2f} | {r['move_time']:.2f} |")
md.append(f"\n성형기 주파수 {b0['sf']:.2f} Hz(대책 없음에서 가장 크게 울린 주파수).\n")
if os.path.exists(O + "/exp2_acc.csv"):
    md.append("가속도 크기: " + ", ".join(f"{r['accel']:g} m/s² {r['case']} {r['stop_vec']:.5f}" for r in rd(O + "/exp2_acc.csv")) + " rad/s\n")
md.append(f"가속 시간 스윕(대책 없음, 1 m/s², Ta 0.05–0.7 s {len(sw)}점): {min(r['stop_vec'] for r in sw):.5f}–{max(r['stop_vec'] for r in sw):.5f} rad/s\n")
md.append("가속 시간별 비(대책 없음 대비):\n\n| Ta [s] | scurve | zv | zvd |\n|---|---|---|---|")
for T in sorted({r["Ta"] for r in t4}):
    b = [r["stop_vec"] for r in t4 if r["case"] == "none" and r["Ta"] == T][0]
    md.append(f"| {T:g} | " + " | ".join(f"{[r['stop_vec'] for r in t4 if r['case'] == c and r['Ta'] == T][0] / b:.2f}" for c in ("scurve", "zv", "zvd")) + " |")
md.append(f"\n좌우 모드가 어긋날 때(성형기·피드포워드는 {b0['sf']:.1f} Hz·J1 200 N·m/rad 기준 그대로, J1 강성만 바꿈):\n\n| J1 강성 | 실제 좌우 모드 [Hz] | 대책 없음 [rad/s] | scurve | zv | zvd | scurve+armff |\n|---|---|---|---|---|---|---|")
for k in kss:
    b = [r["stop_vec"] for r in rob[k] if r["case"] == "none"][0]
    md.append(f"| {k:g} | {fl[k]:.1f} | {b:.5f} | " + " | ".join(f"{[r['stop_vec'] for r in rob[k] if r['case'] == c][0] / b:.2f}" for c in ("scurve", "zv", "zvd", "scurve+armff")) + " |")

# ---------------- 그림 3: 서스펜션 민감도 (실물 장착) ----------------
ss_ = []
for p in sorted(glob.glob(O + "/exp2_susp*.csv")):
    ss_ += rd(p)
ss_ = [r for r in ss_ if not 30000 <= r["susp_k"] <= 45000]   # 이 구간은 등속 중에도 차체가 스스로 흔들리는 불안정이 나와 뺀다(아래 주)
if ss_:
    ks = sorted({r["susp_k"] for r in ss_})
    fig, ax = plt.subplots(1, 2, figsize=(5.2, 2.1))
    for c in ("none", "scurve", "zv", "zvd"):
        pts = sorted([(r["susp_k"] / 1e3, r["stop_vec"] * 1e3, r["stop_peak"]) for r in ss_ if r["case"] == c])
        ax[0].semilogx([p[0] for p in pts], [p[1] for p in pts], "o-", ms=3, lw=0.9, color=C[c], label=LAB[c])
        ax[1].semilogx([p[0] for p in pts], [p[2] for p in pts], "o-", ms=3, lw=0.9, color=C[c])
    for a_, yl, rigid in ((ax[0], "residual rms [mrad/s]", b0["stop_vec"] * 1e3), (ax[1], "residual peak [rad/s]", b0["stop_peak"])):
        a_.axhline(rigid, color="k", lw=0.6, ls=":"); a_.set_xlabel("suspension stiffness per wheel [kN/m]"); a_.set_ylabel(yl)
    ax[0].legend(fontsize=5.5); ax[0].set_title("(a) 3-axis rms (2-20 Hz) after stop", fontsize=8); ax[1].set_title("(b) peak after stop", fontsize=8)
    fig.tight_layout(); fig.savefig(F + "/fig_susp.png"); plt.close(fig)
    md.append("\n## 3. 서스펜션 민감도 — 실물 장착 방향 (바퀴마다 상하 스프링·댐퍼, 감쇠는 강성의 제곱근에 비례)\n")
    md.append("실물 Ranger Mini의 서스펜션·타이어 값은 모른다. '강체 대비'는 2절 값 대비.\n")
    md.append("바퀴당 30–45 kN/m에서는 등속 중에도 차체가 스스로 흔들리고(끝단 비틀림 약 1 rad/s) 이동 거리가 1.0 m가 아니라 0.87 m로 나온다. 바퀴 속도 구동 모형과 서스펜션이 맞물린 불안정으로 보이며 실물의 성질인지는 모른다. 표와 그림에서 뺐다.\n")
    md.append("| 강성 [kN/m] | 섀시 끄덕임 봉우리 [Hz] | 대책 없음 rms (강체 대비) | 축별(비틀림 / 끄덕임 / 좌우) | 최대 (강체 대비) | scurve 비 | zv 비 | zvd 비 |\n|---|---|---|---|---|---|---|---|")
    for k in ks:
        g = {r["case"]: r for r in ss_ if r["susp_k"] == k}
        if "none" not in g: continue
        n = g["none"]
        md.append(f"| {k / 1e3:g} | {n['base_pitch_f']:.1f} | {n['stop_vec']:.5f} ({n['stop_vec'] / b0['stop_vec']:.1f}배) | {n['stop_roll']:.4f} / {n['stop_pitch']:.4f} / {n['stop_yaw']:.4f} | {n['stop_peak']:.3f} ({n['stop_peak'] / b0['stop_peak']:.1f}배) | "
                  + " | ".join(f"{g[c]['stop_vec'] / n['stop_vec']:.2f}" if c in g else "–" for c in ("scurve", "zv", "zvd")) + " |")

# ---------------- 참고: 팔 앞이 진행 방향일 때(장착 0°) — 11/22 오후 첫 실행분, out/yaw0 ----------------
p0 = O + "/yaw0/exp2_main.csv"
if os.path.exists(p0):
    m0 = rd(p0); n0 = [r for r in m0 if r["case"] == "none"][0]
    md.append("\n## 4. 참고 — 가감속이 팔의 끄덕임 평면으로 들어올 때 (장착 0°, 실물에서는 AMR이 옆으로 움직이는 경우에 해당)\n")
    md.append(f"지표는 끄덕임 5–10 Hz rms. 대책 없음 {n0['stop_rms']:.5f} rad/s(팔 10 °/s 정지의 {n0['stop_rms'] / arm10:.2f}배), 끄덕임 모드 7.25 Hz.\n")
    md.append("| 조건 | 정지 뒤 비 |\n|---|---|")
    for r in m0:
        md.append(f"| {r['case']} | {r['stop_rms'] / n0['stop_rms']:.3f} |")
    md.append("\n그림과 나머지 표는 `figs/yaw0/`.")

open(F + "/summary.md", "w").write("\n".join(md) + "\n")
print("\n".join(md))
