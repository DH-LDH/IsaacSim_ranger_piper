"""끝단 IMU의 좌표계와 신호 처리(원고의 그림), 800 dpi, 9.5 pt Times New Roman. 배치 셋:
  2col    → figs/kroc_imu.png (170 × 56 mm, 두 단: (a)(b)(c)(d))
  1col    → figs/kroc_imu_1col.png (80 × 100 mm, 한 단: 위에 (a)(b), 아래에 (c)(d))
  1col_ad → figs/kroc_imu_1col_ad.png (80 × 50 mm, 한 단: 좌표계와 스펙트럼만)
(a) 손목을 앞에서 본 확대(팔의 평면): 손목 카메라(Intel RealSense D455)의 IMU 좌표계 {I} — x_I 영상의 오른쪽(평면 밖), y_I 아래, z_I 광축
(b) 처리 순서   (c) 등속으로 움직이다 멈출 때의 x_I 각속도: 잰 값과 2–10 Hz만 남긴 값   (d) 1 s 조각 평균 스펙트럼(세 축 합): 멈춘 뒤 / 움직이기 전
자료: piper_ws docs/data/raw/c1/20261117_210158_ramp_j2_v10_d-10_c1_059 (자세 A, 위치 제어, 2번 관절 10°/s × 10°)
실행: ~/nonmoon/.venv_pptx/bin/python fig_imu.py [2col|1col|1col_ad|all]   (기본 2col)"""
import csv, glob, gzip, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
import model_check as mc
import robot_render as rr
from kfit_cmp import real, R

H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.linewidth": 0.6, "xtick.major.pad": 2, "ytick.major.pad": 2, "axes.labelpad": 2})
CI, CE, CR = "k", "k", "#d62728"; HALO = [pe.withStroke(linewidth=1.6, foreground="white")]
STEM = "20261117_210158"


def band(x, fs, lo, hi):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs); X[(f < lo) | (f > hi)] = 0; return np.fft.irfft(X, len(x))


def seg_spec(t, X, fs, t0, t1, seg=1.0):
    """scn_seg.seg_rms와 같은 조각 처리(1 s, Hann, 절반 겹침, 선형 추세 제거)의 평균 스펙트럼 — 세 축 합. 반환 f, 1 Hz 칸마다의 실효값"""
    N = int(round(seg * fs)); w = np.hanning(N); f = np.fft.rfftfreq(N, 1 / fs); P = np.zeros(len(f)); n = 0; k = np.arange(N); i = int(np.searchsorted(t, t0))
    while i + N <= len(t) and t[i + N - 1] <= t1:
        for c in range(3):
            x = X[i:i + N, c]; x = x - np.polyval(np.polyfit(k, x, 1), k); P += 2 * np.abs(np.fft.rfft(x * w)) ** 2 / (N * np.sum(w ** 2))
        n += 1; i += N // 2
    return f, np.sqrt(P / n), n


tu, G, tend, full = real(STEM); p = glob.glob(f"{R}/c1/{STEM}*.csv.gz"); p = [x for x in p if not x.endswith(".imu.csv.gz")][0]
with gzip.open(p, "rt") as f_: rows = list(csv.DictReader(f_))
te = np.array([float(r["t"]) for r in rows]); te -= te[0]; q2 = np.array([float(r["q2"]) for r in rows]); mv = np.where(np.abs(np.gradient(q2, te)) > 1.0)[0]; ts = te[mv[-1]]   # 멈춘 시각

LAY = {"2col": (170.0, 56.0, "kroc_imu", {"a": [0.002, 0.02, 0.215, 0.88], "b": [0.222, 0.02, 0.225, 0.88], "c": [0.515, 0.215, 0.20, 0.685], "d": [0.79, 0.215, 0.20, 0.685]}, {"a": "(a) Tip IMU frame {I}", "c": "(c) Gyro about $x_I$", "d": "(d) Spectrum, 3 axes"}),
       "1col": (80.0, 100.0, "kroc_imu_1col", {"a": [0.005, 0.475, 0.44, 0.49], "b": [0.465, 0.49, 0.53, 0.46], "c": [0.165, 0.10, 0.30, 0.30], "d": [0.66, 0.10, 0.30, 0.30]}, {"a": "(a) Tip IMU frame {I}", "c": "(c) Gyro, $x_I$", "d": "(d) Spectrum"}),
       "1col_ad": (80.0, 50.0, "kroc_imu_1col_ad", {"a": [0.005, 0.02, 0.40, 0.89], "d": [0.565, 0.235, 0.395, 0.655]}, {"a": "(a) Tip IMU frame {I}", "d": "(b) Spectrum, 3 axes"})}


def build(layout):
    FW, FH, name, R, TT = LAY[layout]; fig = plt.figure(figsize=(FW * MM, FH * MM))
    if "a" in R:
        # ---- (a) 손목 확대와 {I} ----
        ax = fig.add_axes(R["a"]); ax.set_title(TT["a"], fontsize=9.5, pad=2); ax.set_aspect("equal"); ax.axis("off")
        parts, T = rr.robot_parts(mc.POSE_A, base=False); XL, YL = (0.30, 0.53), (-0.035, 0.285); ax.set_xlim(*XL); ax.set_ylim(*YL); fig.canvas.draw()
        mmpm = (ax.transData.transform((1.0, 0.0))[0] - ax.transData.transform((0.0, 0.0))[0]) / fig.dpi * 25.4        # 도면 꼴 선 그림(그림 1과 같은 방식)
        ax.imshow(rr.lineart(parts, "front", XL, YL, dpi=800, width_mm=(XL[1] - XL[0]) * mmpm, lw_mm=0.15), extent=[*XL, *YL], interpolation="none", zorder=3, aspect="equal")
        pr = lambda v: rr.proj("front", v); oI, RI = rr.cam_frame(T); o = pr(oI); E = pr(T["gripper_base"][:3, 3])
        def arrow(p, q, c=CI, lw=1.4):
            a = FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=8, lw=lw, color=c, shrinkA=0, shrinkB=0, zorder=6); a.set_path_effects([pe.withStroke(linewidth=lw + 1.4, foreground="white")]); ax.add_patch(a)
        def txt(x, y, s, c="k", ha="center", va="center"): ax.text(x, y, s, color=c, ha=ha, va=va, zorder=8, path_effects=HALO)
        dy, dz = pr(RI[:, 1]) * 0.075, pr(RI[:, 2]) * 0.075; arrow(o, o + dy); arrow(o, o + dz)
        ax.add_patch(Circle(o, 0.0085, fc="white", ec=CI, lw=1.1, zorder=8)); ax.add_patch(Circle(o, 0.0028, fc=CI, ec=CI, zorder=9))
        txt(o[0] + dy[0] - 0.012, o[1] + dy[1] + 0.016, "$y_I$", CI); txt(o[0] + dz[0] - 0.022, o[1] + dz[1] - 0.006, "$z_I$", CI); txt(o[0] + 0.022, o[1] + 0.016, "$x_I$", CI); txt(o[0] + 0.012, o[1] - 0.022, "{I}", CI, ha="left")
        ax.add_patch(Circle(E, 0.0045, fc=CE, ec="white", lw=0.5, zorder=7)); ax.plot([E[0], E[0] - 0.04], [E[1], E[1] - 0.03], color="k", lw=0.4, zorder=7); txt(E[0] - 0.045, E[1] - 0.04, "{E}", CE, ha="right")
        txt(0.335, 0.262, "IMU", ha="left"); ax.plot([0.385, o[0] + 0.012], [0.252, o[1] + 0.03], color="k", lw=0.5, zorder=7)
        ax.set_xlim(*XL); ax.set_ylim(*YL)
    if "b" in R:
        # ---- (b) 처리 순서 ----
        ax = fig.add_axes(R["b"]); ax.set_title("(b) Processing", fontsize=9.5, pad=2); ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.axis("off")
        labs = ["Tip gyro, 3 axes, 400 Hz", "1 s Hann segments,\n50% overlap, detrend", "Averaged spectrum", "Keep 2–10 Hz", "RMS over 3 axes"]; hs = [13, 20, 13, 13, 13]; y = 99.0
        for i, (s_, h) in enumerate(zip(labs, hs)):
            ax.add_patch(FancyBboxPatch((3, y - h), 94, h, boxstyle="round,pad=0,rounding_size=2", fc="#fbe5d0" if i == 3 else "#eeeeee", ec="k", lw=0.6)); ax.text(50, y - h / 2, s_, ha="center", va="center", fontsize=9.5, linespacing=0.95)
            if i < len(labs) - 1: ax.annotate("", xy=(50, y - h - 6.0), xytext=(50, y - h), arrowprops=dict(arrowstyle="-|>", lw=0.7, color="k", shrinkA=0, shrinkB=0, mutation_scale=7))
            y -= h + 6.0
    if "c" in R:
        # ---- (c) 시간 파형 ----
        ax = fig.add_axes(R["c"]); ax.set_title(TT["c"], fontsize=9.5, pad=2); m = (tu > ts - 1.6) & (tu < ts + 2.3)
        ax.axvspan(-1.0, 0.0, color="#dddddd", lw=0); ax.plot(tu[m] - ts, G[m, 0], color="#888888", lw=0.6, label="measured"); ax.plot(tu[m] - ts, band(G[:, 0], 400.0, 2.0, 10.0)[m], color=CR, lw=0.8, label="2–10 Hz")
        ax.set_xlim(-1.6, 2.3); ax.set_ylim(-0.34, 0.34); ax.set_yticks([-0.3, 0, 0.3]); ax.set_xticks([-1, 0, 1, 2]); ax.set_xlabel("Time from stop [s]"); ax.set_ylabel("[rad/s]"); ax.grid(alpha=0.3)
        ax.text(-0.5, -0.29, "moving", ha="center", va="center", fontsize=8.5); ax.legend(loc="upper right", fontsize=8.5, frameon=False, handlelength=1.2, borderaxespad=0.1, labelspacing=0.15)
    if "d" in R:
        # ---- (d) 스펙트럼: 멈춘 뒤는 울림이 창 가운데에 오게 잡은 조각 하나, 움직이기 전은 조각 여섯의 평균 ----
        ax = fig.add_axes(R["d"]); ax.set_title(TT["d"], fontsize=9.5, pad=2)
        f1, a1, n1 = seg_spec(tu, G, 400.0, ts - 0.1, ts + 0.905); f0, a0, n0 = seg_spec(tu, G, 400.0, 0.3, te[mv[0]] - 0.1)
        ax.axvspan(2, 10, color="#fbe5d0", lw=0); ax.semilogy(f1[1:31], a1[1:31], color=CR, lw=0.9, marker="o", ms=1.8, label="after stop"); ax.semilogy(f0[1:31], a0[1:31], color="#555555", lw=0.8, marker="s", ms=1.6, label="at rest")
        ax.set_xlim(0, 30); ax.set_ylim(3e-4, 0.1); ax.set_xticks([0, 10, 20, 30]); ax.set_xlabel("Frequency [Hz]"); ax.set_ylabel("[rad/s]"); ax.grid(alpha=0.3, which="both", lw=0.3)
        ax.legend(loc="upper right", fontsize=8.5, frameon=False, handlelength=1.2, borderaxespad=0.1, labelspacing=0.15)
    fig.savefig(f"{H}/figs/{name}.png", dpi=800); plt.close(fig); print(f"→ figs/{name}.png")
    b = lambda f, a, lo, hi: float(np.sqrt(np.sum(a[(f >= lo) & (f <= hi)] ** 2)))
    print(f"멈춘 시각 {ts:.2f} s | 조각 수: 멈춘 뒤 {n1}, 움직이기 전 {n0} | 세 축 합 2–10 Hz 실효값: 멈춘 뒤 {b(f1, a1, 2, 10):.4f}, 움직이기 전 {b(f0, a0, 2, 10):.4f} rad/s (비 {b(f1, a1, 2, 10) / b(f0, a0, 2, 10):.1f}) | 봉우리 {f1[1:31][np.argmax(a1[1:31])]:.0f} Hz")
    gx = band(G[:, 0], 400.0, 2.0, 10.0); mm = (tu > ts) & (tu < ts + 1.5); tot = np.sqrt(sum(np.mean(band(G[:, c], 400.0, 2.0, 10.0)[mm] ** 2) for c in range(3))); print(f"멈춘 뒤 1.5 s, 2–10 Hz: x_I 축 {np.sqrt(np.mean(gx[mm] ** 2)):.4f}, 세 축 합 {tot:.4f} rad/s (x_I가 {np.mean(gx[mm] ** 2) / tot ** 2 * 100:.0f}%의 에너지)")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "2col"
    for lay_ in (tuple(LAY) if which == "all" else (which,)): build(lay_)
