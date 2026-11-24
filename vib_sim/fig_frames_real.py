"""좌표계 그림(실제 형상판) → figs/kroc_frames_real.png (170 × 66 mm, 800 dpi, 9.5 pt Times New Roman)
AgileX Ranger Mini 3.0과 PiPER의 공개 형상 파일을 정사영으로 그리고(robot_render.py) 그 위에 좌표계와 식의 기호를 얹는다.
(a) 옆(x–z): {W}, {B}, 축간 거리, 턱, 베이스의 들림·기울기, 베이스 IMU   (b) 위(x–y): {A}가 {B}에서 +90°, 끝단 {E}, r
(c) 앞에서 본 팔의 평면(y_b–z_b): 어깨 점 P, θ2, δ, m2·m3, K2·K3, a_x·a_z·α, 끝단 IMU 좌표계 {I}
팔 베이스의 위치는 시뮬레이션과 같은 앞뒤·좌우 값을 3.0 상판 위에 둔 것이다(실물 장착 위치는 실물이 와야 정해진다).
실행: ~/nonmoon/.venv_pptx/bin/python fig_frames_real.py"""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Rectangle, Circle, Arc, FancyArrowPatch
import model_check as mc
import robot_render as rr

H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4; DPI = 800
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix"})
CW, CBF, CAF, CE, CI = "#333333", "#0b5cad", "#1a7f1a", "#c41e1e", "#d95f02"; HALO = [pe.withStroke(linewidth=1.6, foreground="white")]
GZ = -rr.H_B; L = 0.494


def arrow(ax, p, q, c, lw=1.1, ls="-", ms=7):
    a = FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=ms, lw=lw, ls=ls, color=c, shrinkA=0, shrinkB=0, zorder=6); a.set_path_effects([pe.withStroke(linewidth=lw + 1.4, foreground="white")]); ax.add_patch(a)


def txt(ax, x, y, s, c="k", ha="center", va="center"):
    ax.text(x, y, s, color=c, ha=ha, va=va, zorder=8, path_effects=HALO)


def dot(ax, p, c, r=0.011):
    ax.add_patch(Circle(p, r, fc=c, ec="white", lw=0.5, zorder=7))


def spring(ax, p):
    ax.add_patch(Circle(p, 0.022, fc="white", ec="k", lw=0.9, zorder=6))
    th = np.linspace(0, 3.2 * math.pi, 60); r = 0.004 + 0.013 * th / th[-1]; ax.plot(p[0] + r * np.cos(th), p[1] + r * np.sin(th), color="k", lw=0.5, zorder=7)


q = mc.POSE_A; parts, T = rr.robot_parts(q); mc.P_BA = rr.P_BA3; B, _ = mc.bodies(*mc.load(), q)
def lump(names):
    m = sum(b[1] for b in B if b[0] in names); return sum(b[1] * b[2] for b in B if b[0] in names) / m
c2, c3 = lump(["link2"]), lump(["link3", "link4", "link5", "link6", "gripper_base", "camera", "link7", "link8"])
P3, J3, E3 = T["link2"][:3, 3], T["link3"][:3, 3], T["gripper_base"][:3, 3]; oI, RI = rr.cam_frame(T); A3 = rr.P_BA3

fig, axs = plt.subplots(1, 3, figsize=(170 * MM, 66 * MM), gridspec_kw=dict(width_ratios=[1.30, 1.02, 1.12])); fig.subplots_adjust(left=0.003, right=0.997, top=0.915, bottom=0.005, wspace=0.02)
for ax in axs: ax.set_aspect("equal"); ax.axis("off")

# ---- (a) 옆에서 본 모습: x–z ----
ax = axs[0]; ax.set_title("(a) Side view ($x$–$z$)", fontsize=9.5, pad=2); rr.draw(ax, parts, "side")
ax.plot([-0.70, 0.76], [GZ, GZ], color="k", lw=0.8, zorder=0); ax.add_patch(Rectangle((0.56, GZ), 0.05, 0.022, fc="#888888", ec="k", lw=0.5)); txt(ax, 0.585, GZ + 0.075, "bump")
arrow(ax, (-0.62, GZ), (-0.49, GZ), CW); arrow(ax, (-0.62, GZ), (-0.62, GZ + 0.13), CW); dot(ax, (-0.62, GZ), CW); txt(ax, -0.455, GZ, "$x$", CW); txt(ax, -0.62, GZ + 0.17, "$z$", CW); txt(ax, -0.62, GZ - 0.07, "{W}", CW)
arrow(ax, (0, 0), (0.14, 0), CBF, lw=1.3); arrow(ax, (0, 0), (0, 0.15), CBF, lw=1.3); dot(ax, (0, 0), CBF); txt(ax, 0.195, -0.005, "$x_b$", CBF); txt(ax, 0.0, 0.20, "$z_b$", CBF); txt(ax, -0.075, -0.05, "{B}", CBF)
ax.plot([0.0], [-0.085], marker="s", ms=3.5, color=CI, mec="white", mew=0.4, zorder=7); txt(ax, 0.03, -0.09, "IMU", CI, ha="left")
ax.annotate("", (L / 2, GZ - 0.055), (-L / 2, GZ - 0.055), arrowprops=dict(arrowstyle="<->", lw=0.6)); txt(ax, 0, GZ - 0.105, "$L$")
arrow(ax, (0.12, 0.44), (0.32, 0.44), "k"); txt(ax, 0.36, 0.44, "$v$")
ax.annotate("", (0.46, 0.13), (0.46, -0.02), arrowprops=dict(arrowstyle="<->", lw=0.9, color=CBF)); txt(ax, 0.545, 0.06, r"$\delta z$", CBF)
ax.add_patch(Arc((0.0, 0.0), 0.56, 0.56, theta1=28, theta2=62, color=CBF, lw=0.9, zorder=6)); arrow(ax, (0.240, 0.147), (0.250, 0.128), CBF, ms=6); txt(ax, 0.325, 0.245, r"$\delta\varphi$", CBF)
e = rr.proj("side", E3); dot(ax, e, CE, 0.013); txt(ax, e[0] - 0.04, e[1] + 0.005, "{E}", CE, ha="right"); txt(ax, -0.31, 0.44, "arm", ha="right")
ax.set_xlim(-0.72, 0.78); ax.set_ylim(GZ - 0.15, 0.52)

# ---- (b) 위에서 본 모습: x–y ----
ax = axs[1]; ax.set_title("(b) Top view ($x$–$y$)", fontsize=9.5, pad=2); rr.draw(ax, parts, "top")
e = rr.proj("top", E3); a = rr.proj("top", A3)
arrow(ax, (0, 0), (0.14, 0), CBF, lw=1.3); arrow(ax, (0, 0), (0, 0.14), CBF, lw=1.3); dot(ax, (0, 0), CBF); txt(ax, 0.195, -0.005, "$x_b$", CBF); txt(ax, 0.06, 0.145, "$y_b$", CBF); txt(ax, 0.0, -0.065, "{B}", CBF)
arrow(ax, a, (a[0], a[1] + 0.13), CAF, lw=1.3); arrow(ax, a, (a[0] - 0.13, a[1]), CAF, lw=1.3); dot(ax, a, CAF); txt(ax, a[0] + 0.06, a[1] + 0.105, "$x_a$", CAF); txt(ax, a[0] - 0.185, a[1], "$y_a$", CAF); txt(ax, a[0], a[1] - 0.065, "{A}", CAF)
arrow(ax, (0, 0), tuple(e * 0.93), CE, lw=0.7, ls="--", ms=6); txt(ax, -0.07, 0.275, "$r$", CE); dot(ax, e, CE, 0.013); txt(ax, e[0] - 0.05, e[1] + 0.02, "{E}", CE, ha="right")
arrow(ax, (0.10, -0.36), (0.30, -0.36), "k"); txt(ax, 0.34, -0.36, "$v$")
ax.set_xlim(-0.60, 0.54); ax.set_ylim(-0.43, 0.56)

# ---- (c) 앞에서 본 팔의 평면: y_b–z_b ----
ax = axs[2]; ax.set_title("(c) Arm plane, front view ($y_b$–$z_b$)", fontsize=9.5, pad=2); rr.draw(ax, parts, "front"); pr = lambda p: rr.proj("front", p)
Pp, J3p, Ep, c2p, c3p, oIp = pr(P3), pr(J3), pr(E3), pr(c2), pr(c3), pr(oI); t2 = math.atan2(*(J3p - Pp)[::-1]); t3 = math.atan2(*(c3p - J3p)[::-1])
ax.plot([-0.36, 0.78], [GZ, GZ], color="k", lw=0.8, zorder=0)
ax.plot([Pp[0], J3p[0]], [Pp[1], J3p[1]], color="k", lw=0.7, zorder=5); ax.plot([J3p[0], c3p[0]], [J3p[1], c3p[1]], color="k", lw=0.7, zorder=5)
spring(ax, Pp); spring(ax, J3p); txt(ax, Pp[0] - 0.075, Pp[1] + 0.045, "$K_2$"); txt(ax, J3p[0] - 0.075, J3p[1] + 0.04, "$K_3$"); txt(ax, Pp[0] - 0.055, Pp[1] - 0.045, "$P$")
ax.plot([Pp[0], Pp[0] + 0.17], [Pp[1], Pp[1]], color="k", lw=0.5, ls=(0, (3, 2)), zorder=5); ax.add_patch(Arc(Pp, 0.21, 0.21, theta1=0, theta2=math.degrees(t2), color="k", lw=0.6, zorder=6)); txt(ax, Pp[0] + 0.155, Pp[1] + 0.045, r"$\theta_2$")
ax.plot([J3p[0], J3p[0] + 0.11 * math.cos(t2)], [J3p[1], J3p[1] + 0.11 * math.sin(t2)], color="k", lw=0.5, ls=(0, (3, 2)), zorder=5); ax.add_patch(Arc(J3p, 0.15, 0.15, theta1=math.degrees(t3), theta2=math.degrees(t2), color="k", lw=0.6, zorder=6)); txt(ax, J3p[0] + 0.12, J3p[1] + 0.005, r"$\delta$")
dot(ax, c2p, "k"); txt(ax, c2p[0] - 0.06, c2p[1] + 0.025, "$m_2$"); dot(ax, c3p, "k"); txt(ax, c3p[0] - 0.075, c3p[1] - 0.02, "$m_3$")
dot(ax, Ep, CE, 0.012); txt(ax, Ep[0] - 0.075, Ep[1] - 0.045, "{E}", CE)
for k in (1, 2):                                                                                   # 끝단 IMU 좌표계 {I}: 축의 이름과 처리 과정은 fig_imu.py의 그림에 따로 그린다
    arrow(ax, oIp, oIp + pr(RI[:, k]) * 0.075, CI, lw=1.2, ms=6)
ax.add_patch(Circle(oIp, 0.015, fc="white", ec=CI, lw=1.0, zorder=8)); ax.add_patch(Circle(oIp, 0.0045, fc=CI, ec=CI, zorder=9)); txt(ax, oIp[0] + 0.03, oIp[1] + 0.005, "{I}", CI, ha="left")
txt(ax, oIp[0] + 0.03, oIp[1] - 0.06, "tip IMU", CI, ha="left")
ab = pr(A3); arrow(ax, (ab[0] - 0.12, ab[1] + 0.0), (ab[0] - 0.12, ab[1] + 0.12), CBF, lw=1.3); txt(ax, ab[0] - 0.175, ab[1] + 0.075, "$a_z$", CBF)
arrow(ax, (ab[0] + 0.07, ab[1] + 0.035), (ab[0] + 0.19, ab[1] + 0.035), CBF, lw=1.3); txt(ax, ab[0] + 0.235, ab[1] + 0.035, "$a_x$", CBF)
ax.add_patch(Arc((ab[0], ab[1] - 0.02), 0.50, 0.50, theta1=168, theta2=200, color=CBF, lw=0.9, zorder=6)); arrow(ax, (ab[0] - 0.238, ab[1] - 0.085), (ab[0] - 0.232, ab[1] - 0.105), CBF, ms=6); txt(ax, ab[0] - 0.295, ab[1] - 0.03, r"$\alpha$", CBF)
arrow(ax, (0.50, GZ + 0.13), (0.60, GZ + 0.13), CAF, lw=1.3); arrow(ax, (0.50, GZ + 0.13), (0.50, GZ + 0.23), CAF, lw=1.3); dot(ax, (0.50, GZ + 0.13), CAF); txt(ax, 0.60, GZ + 0.07, r"$x_a\ (y_b)$", CAF); txt(ax, 0.545, GZ + 0.235, "$z$", CAF)
ax.set_xlim(-0.36, 0.78); ax.set_ylim(GZ - 0.04, 0.50)

os.makedirs(H + "/figs", exist_ok=True); fig.savefig(H + "/figs/kroc_frames_real.png", dpi=DPI); print("→", H + "/figs/kroc_frames_real.png")
print("끝단 IMU 축({B} 성분) x_I", np.round(RI[:, 0], 3), "y_I", np.round(RI[:, 1], 3), "z_I", np.round(RI[:, 2], 3), "| θ2", round(math.degrees(t2), 1), "θ3", round(math.degrees(t3), 1))
