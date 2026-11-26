"""좌표계 그림(도면 꼴 선 그림판), 800 dpi, 9.5 pt Times New Roman. 배치 셋:
  2col    → figs/kroc_frames_schema.png (170 × 106 mm, 두 단: (a)(b)(c))
  1col_ac → figs/kroc_frames_schema_1col_ac.png (80 × 118 mm, 한 단: 옆 + 팔의 평면)
  1col_c  → figs/kroc_frames_schema_1col_c.png (80 × 56 mm, 한 단: 팔의 평면만)
  1col_abc → figs/kroc_frames_schema_1col_abc.png (80 × 170 mm, 한 단: 세 칸을 위아래로)
  1col_ab → figs/kroc_frames_schema_1col_ab.png (80 × 111 mm, 한 단: 옆 + 위)
박사님이 예로 주신 논문(Pham, Weon, Lee, IJCAS 2020, Fig. 1)처럼 3D 모델에서 뽑은 흑백 선 그림 위에 좌표축·중심선·치수선·질량 중심 기호를 얹는다.
형상: AgileX Ranger Mini 3.0, PiPER의 공개 3D 형상(robot_render.lineart가 윤곽·모서리 선을 뽑는다), 손목 카메라는 상자.
(a) x–z 평면(옆)   (b) x–y 평면(위)   (c) y–z 평면(앞에서 본 팔의 평면)
팔 베이스의 위치는 시뮬레이션과 같은 앞뒤·좌우 값을 3.0 상판 위에 둔 것이다(실물 장착 위치는 실물이 와야 정해진다).
실행: ~/nonmoon/.venv_pptx/bin/python fig_frames_schema.py [2col|1col_ac|1col_c|1col_abc|1col_ab|all]   (기본 2col)"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc, FancyArrowPatch, Wedge
import model_check as mc
import robot_render as rr

H = os.path.dirname(os.path.abspath(__file__)); MM = 1 / 25.4; DPI = 800
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix"})
GZ = -rr.H_B; L = 0.494; WB = dict(boxstyle="square,pad=0.08", fc="white", ec="none")


def arrow(ax, p, q, lw=0.9, ms=7, ls="-", style="-|>"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms, lw=lw, ls=ls, color="k", shrinkA=0, shrinkB=0, zorder=6))


def txt(ax, x, y, s, ha="center", va="center", box=True):
    ax.text(x, y, s, color="k", ha=ha, va=va, zorder=8, bbox=WB if box else None)


def cline(ax, p, q, lw=0.45):
    ax.plot([p[0], q[0]], [p[1], q[1]], color="k", lw=lw, ls=(0, (9, 2.5, 1.5, 2.5)), zorder=5)          # 중심선(일점쇄선)


def dline(ax, p, q, lw=0.45):
    ax.plot([p[0], q[0]], [p[1], q[1]], color="k", lw=lw, ls=(0, (4, 2.5)), zorder=5)                   # 기준선(파선)


def tline(ax, p, q, lw=0.4):
    ax.plot([p[0], q[0]], [p[1], q[1]], color="k", lw=lw, zorder=5)                                      # 가는 실선(치수 보조선·지시선)


def dim(ax, p, q, lw=0.5):
    arrow(ax, p, q, lw=lw, ms=6, style="<|-|>")                                                          # 치수선


def com(ax, p, r=0.016):
    ax.add_patch(Circle(p, r, fc="white", ec="k", lw=0.7, zorder=7)); ax.add_patch(Wedge(p, r, 0, 90, fc="k", ec="k", lw=0, zorder=8)); ax.add_patch(Wedge(p, r, 180, 270, fc="k", ec="k", lw=0, zorder=8))


def ground(ax, x0, x1):
    ax.plot([x0, x1], [GZ, GZ], color="k", lw=0.9, zorder=2)
    for x in np.arange(x0 + 0.03, x1, 0.035): ax.plot([x, x - 0.025], [GZ, GZ - 0.025], color="k", lw=0.4, zorder=2)


def origin(ax, p, r=0.008):
    ax.add_patch(Circle(p, r, fc="k", ec="k", zorder=9))


def robot(ax, parts, view, lw_mm=0.13):
    fig.canvas.draw(); s = (ax.transData.transform((1.0, 0.0))[0] - ax.transData.transform((0.0, 0.0))[0]) / fig.dpi * 25.4; xl, yl = ax.get_xlim(), ax.get_ylim()      # 자료 1 m가 지면에서 몇 mm인가
    ax.imshow(rr.lineart(parts, view, xl, yl, dpi=DPI, width_mm=(xl[1] - xl[0]) * s, lw_mm=lw_mm), extent=[*xl, *yl], interpolation="none", zorder=3, aspect="equal"); ax.set_xlim(*xl); ax.set_ylim(*yl)


# 보기 쉽게: 자세 A에서 1·4·6번 관절을 0°로, 카메라의 광축 둘레 장착각을 −90°로 두고 그린다(실제 자세 A는 −0.3°, 1.6°, 13.7°, 장착각 −88.25°라 손목과 카메라가 팔의 평면에서 조금 틀어져 보인다)
q = mc.POSE_A.copy(); q[[0, 3, 5]] = 0.0; rr.CAM_ROLL = -90.0; parts, T = rr.robot_parts(q); mc.P_BA = rr.P_BA3; B, _ = mc.bodies(*mc.load(), q)
def lump(names):
    m = sum(b[1] for b in B if b[0] in names); return sum(b[1] * b[2] for b in B if b[0] in names) / m
c2, c3 = lump(["link2"]), lump(["link3", "link4", "link5", "link6", "gripper_base", "camera", "link7", "link8"])
P3, J3, E3 = T["link2"][:3, 3], T["link3"][:3, 3], T["gripper_base"][:3, 3]; oI, RI = rr.cam_frame(T); A3 = rr.P_BA3


def draw_a(ax):
    """x–z 평면(옆): AMR의 오른쪽에서 봄. 팔은 화면 안쪽으로 뻗어 겹쳐 보인다"""
    robot(ax, parts, "side"); ground(ax, -0.72, 0.80)
    ax.add_patch(Rectangle((0.56, GZ), 0.045, 0.022, fc="k", ec="k", zorder=4)); tline(ax, (0.60, GZ + 0.03), (0.66, GZ + 0.085)); txt(ax, 0.705, GZ + 0.105, "bump")
    arrow(ax, (-0.65, GZ + 0.075), (-0.52, GZ + 0.075)); arrow(ax, (-0.65, GZ + 0.075), (-0.65, GZ + 0.205)); origin(ax, (-0.65, GZ + 0.075), 0.006); txt(ax, -0.485, GZ + 0.075, "$x$"); txt(ax, -0.65, GZ + 0.245, "$z$"); txt(ax, -0.685, GZ + 0.03, "{W}", box=False)
    for xw in (-L / 2, L / 2): cline(ax, (xw, GZ - 0.115), (xw, rr.WHEEL_Z + 0.14)); cline(ax, (xw - 0.13, rr.WHEEL_Z), (xw + 0.13, rr.WHEEL_Z))
    dim(ax, (-L / 2, GZ - 0.085), (L / 2, GZ - 0.085)); txt(ax, 0.0, GZ - 0.135, "$L$", box=False)
    arrow(ax, (0, 0), (0.17, 0), lw=1.2); arrow(ax, (0, 0), (0, 0.17), lw=1.2); origin(ax, (0, 0)); txt(ax, 0.215, -0.002, "$x_b$"); txt(ax, 0.045, 0.185, "$z_b$"); txt(ax, 0.055, -0.045, "{B}")
    tline(ax, (-0.012, -0.012), (-0.10, -0.10)); txt(ax, -0.135, -0.125, "base IMU")
    cline(ax, (A3[0], 0.02), (A3[0], 0.47)); tline(ax, (0.0, 0.19), (0.0, 0.45)); dim(ax, (A3[0], 0.42), (0.0, 0.42)); txt(ax, A3[0] / 2, 0.455, "$r_x$", box=False)
    e = rr.proj("side", E3); origin(ax, e); tline(ax, e, (e[0] - 0.10, e[1] + 0.035)); txt(ax, e[0] - 0.145, e[1] + 0.04, "{E}")
    arrow(ax, (0.30, 0.42), (0.50, 0.42), lw=1.0); txt(ax, 0.54, 0.42, "$v$", box=False)
    dim(ax, (0.47, -0.02), (0.47, 0.13)); txt(ax, 0.535, 0.055, r"$\delta z$", box=False)
    ax.add_patch(Arc((0.0, 0.0), 0.60, 0.60, theta1=24, theta2=58, color="k", lw=0.6, zorder=6)); arrow(ax, (0.268, 0.134), (0.276, 0.117), lw=0.6, ms=6); txt(ax, 0.335, 0.215, r"$\delta\varphi$", box=False)
    


def draw_b(ax):
    """x–y 평면(위)"""
    robot(ax, parts, "top"); e = rr.proj("top", E3); a = rr.proj("top", A3)
    cline(ax, (-0.47, 0), (0.47, 0)); cline(ax, (0, -0.32), (0, 0.33))
    arrow(ax, (0, 0), (0.17, 0), lw=1.2); arrow(ax, (0, 0), (0, 0.17), lw=1.2); origin(ax, (0, 0)); txt(ax, 0.215, -0.045, "$x_b$"); txt(ax, 0.05, 0.19, "$y_b$"); txt(ax, 0.055, -0.05, "{B}")
    arrow(ax, a, (a[0], a[1] + 0.15), lw=1.2); arrow(ax, a, (a[0] - 0.15, a[1]), lw=1.2); origin(ax, a); txt(ax, a[0] + 0.055, a[1] + 0.135, "$x_a$"); txt(ax, a[0] - 0.19, a[1] + 0.045, "$y_a$"); txt(ax, a[0] - 0.06, a[1] - 0.05, "{A}")
    arrow(ax, (0, 0), tuple(e * 0.96), lw=0.6, ms=6, ls=(0, (4, 2.5))); txt(ax, -0.075, 0.265, "$r_e$")
    origin(ax, e); tline(ax, e, (e[0] + 0.11, e[1] + 0.07)); txt(ax, e[0] + 0.15, e[1] + 0.085, "{E}")
    arrow(ax, (0.10, -0.38), (0.30, -0.38), lw=1.0); txt(ax, 0.34, -0.38, "$v$", box=False)
    


def draw_c(ax, full=True, legend=None):
    """y–z 평면(앞에서 본 팔의 평면): 식 (1)의 기호. full이면 바퀴와 바닥까지, 아니면 차체 윗부분까지만. legend = 축 범례의 원점"""
    robot(ax, parts, "front", lw_mm=0.15); pr = lambda p: rr.proj("front", p)
    Pp, J3p, Ep, c2p, c3p, oIp, ab = pr(P3), pr(J3), pr(E3), pr(c2), pr(c3), pr(oI), pr(A3); t2 = math.atan2(*(J3p - Pp)[::-1]); t3 = math.atan2(*(c3p - J3p)[::-1]); u2 = np.array([math.cos(t2), math.sin(t2)]); u3 = np.array([math.cos(t3), math.sin(t3)]); n2 = np.array([-u2[1], u2[0]])
    cline(ax, Pp - 0.04 * u2, J3p + 0.10 * u2); cline(ax, J3p - 0.03 * u3, c3p + 0.05 * u3)
    dline(ax, Pp, (Pp[0] + 0.17, Pp[1])); ax.add_patch(Arc(Pp, 0.22, 0.22, theta1=0, theta2=math.degrees(t2), color="k", lw=0.6, zorder=6)); txt(ax, Pp[0] + 0.135, Pp[1] + 0.045, r"$\theta_2$")
    ax.add_patch(Arc(J3p, 0.14, 0.14, theta1=math.degrees(t3), theta2=math.degrees(t2), color="k", lw=0.6, zorder=6)); txt(ax, J3p[0] + 0.095, J3p[1] + 0.005, r"$\delta$")
    origin(ax, Pp, 0.007); origin(ax, J3p, 0.007); tline(ax, Pp, (Pp[0] - 0.06, Pp[1] + 0.035)); txt(ax, Pp[0] - 0.105, Pp[1] + 0.045, "$P, K_2$"); tline(ax, J3p, (J3p[0] - 0.02, J3p[1] + 0.06)); txt(ax, J3p[0] - 0.03, J3p[1] + 0.08, "$K_3$")
    d0, d1 = Pp + 0.075 * n2, J3p + 0.075 * n2; tline(ax, Pp + 0.035 * n2, Pp + 0.09 * n2); tline(ax, J3p + 0.035 * n2, J3p + 0.09 * n2); dim(ax, d0, d1); txt(ax, *((d0 + d1) / 2 + 0.028 * n2), "$l_2$", box=False)
    com(ax, c2p, 0.012); tline(ax, c2p, (c2p[0] + 0.055, c2p[1] - 0.04)); txt(ax, c2p[0] + 0.08, c2p[1] - 0.052, "$m_2$"); com(ax, c3p, 0.012); tline(ax, c3p, (c3p[0] - 0.06, c3p[1] - 0.035)); txt(ax, c3p[0] - 0.085, c3p[1] - 0.045, "$m_3$")
    origin(ax, Ep, 0.007); tline(ax, Ep, (Ep[0] - 0.075, Ep[1] - 0.065)); txt(ax, Ep[0] - 0.10, Ep[1] - 0.08, "{E}")
    for k, lab, off in ((1, "$y_I$", (0.045, 0.012)), (2, "$z_I$", (0.0, -0.03))):                      # 끝단 IMU 좌표계 {I}: x_I는 지면 밖으로(⊙)
        d = pr(RI[:, k]) * 0.085; arrow(ax, oIp, oIp + d, lw=1.1, ms=6); txt(ax, oIp[0] + d[0] + off[0], oIp[1] + d[1] + off[1], lab)
    ax.add_patch(Circle(oIp, 0.011, fc="white", ec="k", lw=0.9, zorder=8)); ax.add_patch(Circle(oIp, 0.0035, fc="k", ec="k", zorder=9)); txt(ax, oIp[0] + 0.045, oIp[1] - 0.012, "$x_I$"); tline(ax, oIp + np.array([0.010, 0.012]), (oIp[0] + 0.06, oIp[1] + 0.075)); txt(ax, oIp[0] + 0.10, oIp[1] + 0.095, "{I}: tip IMU")
    arrow(ax, (ab[0] - 0.10, ab[1] + 0.0), (ab[0] - 0.10, ab[1] + 0.11), lw=1.1); txt(ax, ab[0] - 0.135, ab[1] + 0.075, "$a_z$", box=False)
    arrow(ax, (ab[0] + 0.065, ab[1] + 0.04), (ab[0] + 0.175, ab[1] + 0.04), lw=1.1); txt(ax, ab[0] + 0.205, ab[1] + 0.042, "$a_x$", box=False)
    ax.add_patch(Arc((ab[0], ab[1] - 0.02), 0.44, 0.44, theta1=160, theta2=200, color="k", lw=0.7, zorder=6)); arrow(ax, (ab[0] - 0.210, ab[1] - 0.085), (ab[0] - 0.204, ab[1] - 0.105), lw=0.7, ms=6); txt(ax, ab[0] - 0.25, ab[1] + 0.055, r"$\alpha$", box=False)
    
    if full: ground(ax, -0.31, 0.69)
    lx, ly = legend if legend else (0.50, GZ + 0.06)
    arrow(ax, (lx, ly), (lx + 0.11, ly)); arrow(ax, (lx, ly), (lx, ly + 0.11)); origin(ax, (lx, ly), 0.005); txt(ax, lx + 0.145, ly, "$y_b$", box=False); txt(ax, lx, ly + 0.14, "$z_b$", box=False)


def build(layout):
    global fig
    FW, FH, name = {"2col": (170.0, 106.0, "kroc_frames_schema"), "1col_ac": (80.0, 118.0, "kroc_frames_schema_1col_ac"), "1col_c": (80.0, 56.0, "kroc_frames_schema_1col_c"),
                    "1col_abc": (80.0, 170.0, "kroc_frames_schema_1col_abc"), "1col_ab": (80.0, 111.0, "kroc_frames_schema_1col_ab")}[layout]
    fig = plt.figure(figsize=(FW * MM, FH * MM))
    def panel(rect, x0, x1, y0, cap):
        """rect = (왼쪽, 아래, 폭, 높이)[그림 비율]. 가로 범위와 아래 끝을 주면 세로 범위는 칸의 모양에 맞춘다(축척이 가로세로 같게)"""
        ax = fig.add_axes(rect); ax.axis("off"); ax.set_xlim(x0, x1); ax.set_ylim(y0, y0 + (x1 - x0) * (rect[3] * FH) / (rect[2] * FW))
        if cap: ax.text(0.5, -0.02, cap, transform=ax.transAxes, ha="center", va="top")
        return ax
    if layout == "2col":
        draw_a(panel((0.003, 0.555, 0.435, 0.435), -0.74, 0.82, GZ - 0.19, "(a) For the $x$–$z$ plane")); draw_b(panel((0.058, 0.060, 0.325, 0.435), -0.58, 0.56, -0.44, "(b) For the $x$–$y$ plane"))
        draw_c(panel((0.448, 0.060, 0.549, 0.930), -0.32, 0.70, GZ - 0.10, "(c) For the $y$–$z$ plane (arm plane, seen from the front)"))
    elif layout == "1col_ac":
        draw_a(panel((0.005, 0.565, 0.99, 0.430), -0.74, 0.82, GZ - 0.19, "(a) For the $x$–$z$ plane")); draw_c(panel((0.005, 0.070, 0.99, 0.425), -0.32, 0.70, -0.16, "(b) For the $y$–$z$ plane (arm plane, from the front)"), full=False, legend=(0.54, -0.125))
    elif layout in ("1col_abc", "1col_ab"):
        mm = lambda l, b, w, h: (l / FW, b / FH, w / FW, h / FH)                                         # 칸을 mm로 준다
        ya = FH - 0.6 - 50.7; yb = ya - 8.0 - 45.0                                                       # (a) 79.2 × 50.7 mm, (b) 55.25 × 45.0 mm (두 단 판과 같은 축척)
        draw_a(panel(mm(0.4, ya, 79.2, 50.7), -0.74, 0.82, GZ - 0.19, "(a) For the $x$–$z$ plane")); draw_b(panel(mm(12.4, yb, 55.25, 45.0), -0.58, 0.56, -0.42, "(b) For the $x$–$y$ plane"))
        if layout == "1col_abc": draw_c(panel(mm(0.4, yb - 8.0 - 50.2, 79.2, 50.2), -0.32, 0.70, -0.16, "(c) For the $y$–$z$ plane (arm plane, from the front)"), full=False, legend=(0.54, -0.125))
    else:
        draw_c(panel((0.005, 0.005, 0.99, 0.99), -0.32, 0.70, -0.19, ""), full=False, legend=(0.54, -0.155))
    os.makedirs(H + "/figs", exist_ok=True); fig.savefig(f"{H}/figs/{name}.png", dpi=DPI); plt.close(fig); print("→", f"{H}/figs/{name}.png")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "2col"
    for lay in (("2col", "1col_ac", "1col_c", "1col_abc", "1col_ab") if which == "all" else (which,)): build(lay)
