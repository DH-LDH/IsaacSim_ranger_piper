"""좌표계 그림(박사님 11/24 저녁: 팔·베이스·센서의 축을 그림으로) → figs/kroc_frames.png (170 × 54 mm, 800 dpi, 9.5 pt Times New Roman)
(a) 옆에서 본 모습(x–z): 월드 {W}, 베이스 {B}, 축간 거리, 턱, 베이스의 들림·끄덕임   (b) 위에서 본 모습(x–y): 팔 베이스 {A}가 {B}에서 +90° 돌아 있음, 끝단 {E}, r
(c) 팔의 평면(x_a–z_a): 모터 쪽 각 q_m·링크 쪽 각 q_l 사이의 스프링·댐퍼, 끝단 IMU.  치수는 vibsim.py 값(장착 위치, 축간 거리, 자세 A·B의 끝단 위치).
실행: ~/nonmoon/.venv_pptx/bin/python fig_frames.py"""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc, FancyArrowPatch, Polygon

H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4; DPI = 800
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix"})
LWB, RW, ZB = 0.494, 0.100, 0.280                      # 축간 거리, 바퀴 반지름, base_link 높이
PA = np.array([-0.249, 0.022, 0.045])                  # {B}에서 본 팔 베이스
ARM = np.array([[0, 0], [0, 0.123], [0.179, 0.345], [0.378, 0.190]])        # {A}의 x_a–z_a: 베이스, J2, J3, J5 (자세 A)
EA, EB = np.array([0.370, 0.100]), np.array([0.466, 0.170])                 # 끝단: 자세 A(그리퍼 아래), 자세 B(그리퍼 앞)
CW, CBF, CAF, CE, CI = "#444444", "#1f77b4", "#2ca02c", "#d62728", "#ff7f0e"


def arrow(ax, p, q, c, lw=1.0, ls="-", ms=7):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=ms, lw=lw, ls=ls, color=c, shrinkA=0, shrinkB=0, zorder=6))


def txt(ax, x, y, s, c="k", ha="center", va="center"):
    ax.text(x, y, s, color=c, ha=ha, va=va, zorder=8)


def dot(ax, p, c, r=0.010):
    ax.add_patch(Circle(p, r, color=c, zorder=7))


def spring(ax, p, c="k"):
    ax.add_patch(Circle(p, 0.020, fc="white", ec=c, lw=0.9, zorder=5))
    th = np.linspace(0, 3.2 * math.pi, 60); r = 0.004 + 0.012 * th / th[-1]; ax.plot(p[0] + r * np.cos(th), p[1] + r * np.sin(th), color=c, lw=0.5, zorder=6)


fig, axs = plt.subplots(1, 3, figsize=(170 * MM, 54 * MM), gridspec_kw=dict(width_ratios=[1.25, 1.0, 1.0]))
fig.subplots_adjust(left=0.005, right=0.995, top=0.90, bottom=0.01, wspace=0.03)
for ax in axs: ax.set_aspect("equal"); ax.axis("off")

# ---- (a) 옆에서 본 모습: x–z (AMR의 오른쪽에서 +y 쪽을 봄. 팔은 화면 안쪽으로 뻗어 세로로 겹쳐 보인다) ----
ax = axs[0]; ax.set_title("(a) Side view ($x$–$z$)", fontsize=9.5, pad=2)
ax.plot([-0.70, 0.80], [0, 0], color="k", lw=0.8); ax.add_patch(Rectangle((0.56, 0), 0.05, 0.022, fc="#999999", ec="k", lw=0.5)); txt(ax, 0.585, 0.075, "bump")
ax.add_patch(Rectangle((-0.37, 0.14), 0.74, 0.185, fc="#e8e8e8", ec="k", lw=0.7))
for xw in (-LWB / 2, LWB / 2): ax.add_patch(Circle((xw, RW), RW, fc="#bbbbbb", ec="k", lw=0.7, zorder=3))
xa = PA[0]; za = ZB + PA[2]
ax.plot([xa, xa], [za, za + 0.345], color="k", lw=2.2, solid_capstyle="round", zorder=4); dot(ax, (xa, za + 0.100), CE, 0.014)
txt(ax, xa - 0.035, za + 0.33, "arm", ha="right"); txt(ax, xa + 0.035, za + 0.100, "{E}", CE, ha="left")
arrow(ax, (-0.62, 0), (-0.49, 0), CW); arrow(ax, (-0.62, 0), (-0.62, 0.13), CW); dot(ax, (-0.62, 0), CW); txt(ax, -0.455, 0.0, "$x$", CW); txt(ax, -0.62, 0.17, "$z$", CW); txt(ax, -0.62, -0.07, "{W}", CW)
arrow(ax, (0, ZB), (0.13, ZB), CBF); arrow(ax, (0, ZB), (0, ZB + 0.15), CBF); dot(ax, (0, ZB), CBF); txt(ax, 0.185, ZB - 0.005, "$x_b$", CBF); txt(ax, 0.0, ZB + 0.195, "$z_b$", CBF); txt(ax, -0.075, ZB - 0.045, "{B}", CBF)
ax.plot([0.0], [0.195], marker="s", ms=3.5, color=CI, zorder=6); txt(ax, 0.03, 0.19, "IMU", CI, ha="left")
ax.annotate("", (LWB / 2, -0.055), (-LWB / 2, -0.055), arrowprops=dict(arrowstyle="<->", lw=0.6)); txt(ax, 0, -0.105, "$L$")
arrow(ax, (0.10, 0.66), (0.30, 0.66), "k"); txt(ax, 0.34, 0.66, "$v$")
ax.annotate("", (0.44, 0.40), (0.44, 0.25), arrowprops=dict(arrowstyle="<->", lw=0.9, color=CBF)); txt(ax, 0.52, 0.335, r"$\delta z$", CBF)
ax.add_patch(Arc((0.0, ZB), 0.52, 0.52, theta1=28, theta2=62, color=CBF, lw=0.9)); arrow(ax, (0.222, ZB + 0.137), (0.232, ZB + 0.118), CBF, ms=6); txt(ax, 0.30, ZB + 0.225, r"$\delta\varphi$", CBF)   # +y_b 둘레의 양의 회전(앞이 숙여짐) = 이 시점에서 시계 방향
ax.set_xlim(-0.72, 0.82); ax.set_ylim(-0.15, 0.74)

# ---- (b) 위에서 본 모습: x–y ----
ax = axs[1]; ax.set_title("(b) Top view ($x$–$y$)", fontsize=9.5, pad=2)
ax.add_patch(Rectangle((-0.37, -0.25), 0.74, 0.50, fc="#e8e8e8", ec="k", lw=0.7))
for xw in (-LWB / 2, LWB / 2):
    for yw in (-0.21, 0.21): ax.add_patch(Rectangle((xw - 0.09, yw - 0.035), 0.18, 0.07, fc="#bbbbbb", ec="k", lw=0.6, zorder=3))
ea = np.array([PA[0], PA[1] + EA[0]])
ax.plot([PA[0], ea[0]], [PA[1], ea[1]], color="k", lw=2.2, solid_capstyle="round", zorder=4); dot(ax, ea, CE, 0.014); txt(ax, ea[0] - 0.035, ea[1] + 0.015, "{E}", CE, ha="right")
arrow(ax, (0, 0), (0.13, 0), CBF); arrow(ax, (0, 0), (0, 0.13), CBF); dot(ax, (0, 0), CBF); txt(ax, 0.185, -0.005, "$x_b$", CBF); txt(ax, 0.055, 0.135, "$y_b$", CBF); txt(ax, 0.0, -0.065, "{B}", CBF)
arrow(ax, (PA[0], PA[1]), (PA[0], PA[1] + 0.12), CAF, lw=1.3); arrow(ax, (PA[0], PA[1]), (PA[0] - 0.12, PA[1]), CAF, lw=1.3); dot(ax, (PA[0], PA[1]), CAF)
txt(ax, PA[0] + 0.055, PA[1] + 0.10, "$x_a$", CAF); txt(ax, PA[0] - 0.175, PA[1], "$y_a$", CAF); txt(ax, PA[0], PA[1] - 0.065, "{A}", CAF)
arrow(ax, (0, 0), tuple(ea * 0.93), CE, lw=0.6, ls="--", ms=6); txt(ax, -0.075, 0.275, "$r$", CE)
arrow(ax, (0.10, -0.36), (0.30, -0.36), "k"); txt(ax, 0.34, -0.36, "$v$")
ax.set_xlim(-0.56, 0.52); ax.set_ylim(-0.44, 0.50)

# ---- (c) 팔의 평면: x_a–z (= y_b–z_b). 식 (3)의 기호: 어깨 점 P, 링크 각 θ2, 팔꿈치 각 δ, 질량 m2·m3, 관절 스프링 K2·K3, 어깨 점의 가속도 a_x·a_z, 평면 안 각가속도 α ----
ax = axs[2]; ax.set_title("(c) Arm plane ($y_b$–$z_b$)", fontsize=9.5, pad=2)
Pp = ARM[1]; J3 = ARM[2]; t2 = math.radians(50.7); tc2 = math.radians(53.85); t3 = math.radians(-48.2)
c2 = Pp + 0.199 * np.array([math.cos(tc2), math.sin(tc2)]); c3 = J3 + 0.254 * np.array([math.cos(t3), math.sin(t3)])
ax.add_patch(Rectangle((-0.07, -0.03), 0.14, 0.03, fc="#e8e8e8", ec="k", lw=0.7)); ax.plot([0, 0], [0, Pp[1]], color="k", lw=2.2, zorder=3)
ax.plot(ARM[1:, 0], ARM[1:, 1], color="k", lw=2.2, solid_capstyle="round", zorder=3); ax.plot([ARM[3, 0], EA[0]], [ARM[3, 1], EA[1]], color="k", lw=2.2, zorder=3)
spring(ax, Pp); spring(ax, J3); txt(ax, -0.075, 0.175, "$K_2$"); txt(ax, 0.105, 0.385, "$K_3$"); txt(ax, -0.045, 0.085, "$P$")
ax.plot([Pp[0], Pp[0] + 0.16], [Pp[1], Pp[1]], color="k", lw=0.5, ls=(0, (3, 2))); ax.add_patch(Arc(Pp, 0.20, 0.20, theta1=0, theta2=50.7, color="k", lw=0.6)); txt(ax, 0.15, 0.165, r"$\theta_2$")
ax.plot([J3[0], J3[0] + 0.11 * math.cos(t2)], [J3[1], J3[1] + 0.11 * math.sin(t2)], color="k", lw=0.5, ls=(0, (3, 2))); ax.add_patch(Arc(J3, 0.15, 0.15, theta1=-48.2, theta2=50.7, color="k", lw=0.6)); txt(ax, 0.295, 0.355, r"$\delta$")
dot(ax, c2, "k", 0.011); txt(ax, c2[0] - 0.055, c2[1] + 0.02, "$m_2$"); dot(ax, c3, "k", 0.011); txt(ax, c3[0] - 0.065, c3[1] - 0.025, "$m_3$"); txt(ax, 0.185, 0.265, "$l_2$")
dot(ax, EA, CE, 0.012); txt(ax, EA[0] + 0.07, EA[1] - 0.04, "{E}", CE); ax.plot([0.405], [0.125], marker="s", ms=3.5, color=CI, zorder=7); txt(ax, 0.43, 0.125, "tip IMU", CI, ha="left")
arrow(ax, (0.08, -0.015), (0.20, -0.015), CBF, lw=1.3); txt(ax, 0.245, -0.02, "$a_x$", CBF); arrow(ax, (-0.11, -0.03), (-0.11, 0.08), CBF, lw=1.3); txt(ax, -0.165, 0.04, "$a_z$", CBF)
ax.add_patch(Arc((0.0, -0.015), 0.30, 0.30, theta1=215, theta2=262, color=CBF, lw=0.9)); arrow(ax, (-0.035, -0.161), (-0.012, -0.165), CBF, ms=6); txt(ax, -0.16, -0.125, r"$\alpha$", CBF)
arrow(ax, (0.50, -0.13), (0.59, -0.13), CAF, lw=1.3); arrow(ax, (0.50, -0.13), (0.50, -0.04), CAF, lw=1.3); dot(ax, (0.50, -0.13), CAF); txt(ax, 0.60, -0.175, r"$x_a\ (y_b)$", CAF); txt(ax, 0.545, -0.04, "$z$", CAF)
ax.set_xlim(-0.22, 0.72); ax.set_ylim(-0.19, 0.46)

os.makedirs(H + "/figs", exist_ok=True); fig.savefig(H + "/figs/kroc_frames.png", dpi=DPI); print("→", H + "/figs/kroc_frames.png")
