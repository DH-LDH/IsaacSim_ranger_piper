"""실제 형상 파일(AgileX Ranger Mini 3.0, PiPER, 손목 카메라 D455 상자)을 정사영으로 그리는 도구 — 좌표계 그림용.
좌표는 모두 베이스 좌표계 {B}(x 앞, y 왼쪽, z 위, 원점 = 3.0 모델의 base_link). 바닥은 z = −H_B.
형상: 3.0 차체·바퀴는 AgileX 공개 저장소(ugv_gazebo_sim, ranger_mini_v3)의 것, 팔은 piper_description의 STL.
그리는 법: 보는 쪽을 향한 삼각형만 남겨 먼 것부터 칠한다(면마다 한 가지 밝기)."""
import math, os
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from matplotlib.collections import PolyCollection
import model_check as mc

RP = os.path.expanduser("~/projects/ranger_piper"); PM = os.path.expanduser("~/piper_ws/src/piper_description/meshes")
WHEEL_XY = [(0.247, 0.182), (0.247, -0.182), (-0.247, 0.182), (-0.247, -0.182)]   # 설명서: 축간 거리 494, 좌우 바퀴 간격 364 mm
WHEEL_Z, WHEEL_R = -0.22, 0.10; H_B = -WHEEL_Z + WHEEL_R                          # base_link 원점의 높이(바닥에서)
P_BA3 = np.array([-0.249, 0.022, 0.016])                                           # 팔 베이스: 시뮬레이션과 같은 앞뒤·좌우 위치, 3.0 상판 위(실물 장착 위치는 미정)
CAM_P, CAM_PITCH, CAM_ROLL = np.array([-0.085, -0.01, 0.026]), 30.75, -88.25        # 손목 카메라(link6 좌표: 위치는 piper_eih_camera_node의 실기 값, 장착각은 common_pkg/imu_frame.py의 정지 중력 맞춤 값)
CAM_BOX = (0.124, 0.029, 0.026)                                                    # D455 겉 치수: 폭(영상의 x) × 높이(y) × 깊이(z)
VIEWS = {"side": np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]]),                  # (가로, 세로, 보는 사람 쪽) — AMR의 오른쪽에서 봄
         "top": np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]),                     # 위에서 봄
         "front": np.array([[0.0, 1, 0], [0, 0, 1], [1, 0, 0]])}                   # 앞에서 뒤를 봄: 가로 = y_b(= 팔의 x_a)


def Tm(R=np.eye(3), p=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T


def r_link6_cam():
    a, b = math.radians(CAM_PITCH), math.radians(CAM_ROLL)
    Ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]); Rz = np.array([[math.cos(b), -math.sin(b), 0], [math.sin(b), math.cos(b), 0], [0, 0, 1]])
    return Ry @ Rz


def link_frames(q, T0=None):
    """팔 링크의 좌표계({B}에서 본 4×4)"""
    L, J = mc.load(); T = {"base_link": Tm(mc.R_BA, P_BA3) if T0 is None else T0}
    for j in J:
        Tp = T[j["parent"]] @ j["T"]
        if j["type"] == "revolute": Tp = Tp @ Tm(mc.axang(j["ax"], q[int(j["name"][5:]) - 1]))
        T[j["child"]] = Tp
    return T


def _tris(path, T, scale=1.0):
    m = trimesh.load(path, force="mesh"); V = np.asarray(m.vertices) * scale; V = V @ T[:3, :3].T + T[:3, 3]; return V[np.asarray(m.faces)]


def robot_parts(q, base=True, arm=True, cam=True):
    """[(삼각형 N×3×3, 색)] — {B} 좌표"""
    P = []
    if base:
        P.append((_tris(RP + "/ranger_mini/urdf/meshes/dae/v3_ranger_base.dae", np.eye(4)), (0.80, 0.81, 0.83)))
        for x, y in WHEEL_XY:
            R = mc.rpy(math.pi if y < 0 else 0.0, 0, 0); P.append((_tris(RP + "/ranger_mini/urdf/meshes/dae/v3_wheel.dae", Tm(R, (x, y, WHEEL_Z)), 10.0), (0.33, 0.33, 0.35)))
    T = link_frames(q)
    if arm:
        vis = {}
        for l in ET.parse(mc.H + "/assets/piper_abs.urdf").getroot().findall("link"):
            o = l.find("visual/origin"); vis[l.get("name")] = Tm(mc.rpy(*[float(v) for v in o.get("rpy").split()]), [float(v) for v in o.get("xyz").split()]) if o is not None else np.eye(4)
        for n in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_base", "link7", "link8"):
            P.append((_tris(f"{PM}/{n}.STL", T[n] @ vis.get(n, np.eye(4))), (0.62, 0.66, 0.72) if n not in ("link7", "link8") else (0.45, 0.47, 0.50)))
    if cam:
        box = trimesh.creation.box(extents=CAM_BOX); Tc = T["link6"] @ Tm(r_link6_cam(), CAM_P); V = np.asarray(box.vertices) @ Tc[:3, :3].T + Tc[:3, 3]; P.append((V[np.asarray(box.faces)], (0.20, 0.20, 0.22)))
    return P, T


def cam_frame(T):
    """손목 카메라 IMU의 원점과 축({B} 좌표): 열이 x_I(영상의 오른쪽), y_I(아래), z_I(광축)"""
    Tc = T["link6"] @ Tm(r_link6_cam(), CAM_P); return Tc[:3, 3], Tc[:3, :3]


def draw(ax, parts, view, light=(-0.35, 0.45, 0.82), zorder=1):
    A = VIEWS[view]; l = np.array(light) / np.linalg.norm(light); V2, C, D = [], [], []
    for tris, col in parts:
        Q = tris @ A.T; n = np.cross(Q[:, 1] - Q[:, 0], Q[:, 2] - Q[:, 0]); ln = np.linalg.norm(n, axis=1); ok = (ln > 1e-12) & (n[:, 2] > 0); Q, n = Q[ok], n[ok] / ln[ok, None]
        sh = 0.50 + 0.50 * np.clip(n @ l, 0, 1); V2.append(Q[:, :, :2]); C.append(np.clip(np.outer(sh, col), 0, 1)); D.append(Q[:, :, 2].mean(axis=1))
    V2, C, D = np.concatenate(V2), np.concatenate(C), np.concatenate(D); o = np.argsort(D)
    pc = PolyCollection(V2[o], facecolors=C[o], edgecolors=C[o], linewidths=0.15, antialiased=True, rasterized=True, zorder=zorder); ax.add_collection(pc)
    return A


def proj(view, p):
    """{B}의 점(…×3) → 그림 좌표(…×2)"""
    return np.asarray(p) @ VIEWS[view][:2].T


def lineart(parts, view, xlim, ylim, dpi=800, mm_per_m=None, width_mm=None, lw_mm=0.16, crease_deg=32.0, depth_jump=0.02, min_px=40):
    """형상을 도면 같은 선 그림으로: 보이는 면의 (부품 번호, 법선, 깊이)를 그림으로 칠한 뒤 이웃 화소와 달라지는 곳(윤곽·가림·모서리)을 선으로 뽑는다.
    반환: RGBA 배열(안쪽 흰색, 선 검정, 바깥 투명) — ax.imshow(img, extent=[*xlim, *ylim]). width_mm = 이 범위가 지면에서 차지할 폭"""
    import matplotlib; from matplotlib.figure import Figure; from matplotlib.backends.backend_agg import FigureCanvasAgg; from scipy import ndimage
    W = int(round(width_mm / 25.4 * dpi)); Hh = int(round(W * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0]))); A = VIEWS[view]; Q, N, ID = [], [], []
    for k, (tris, _) in enumerate(parts):
        q = tris @ A.T; n = np.cross(q[:, 1] - q[:, 0], q[:, 2] - q[:, 0]); ln = np.linalg.norm(n, axis=1); ok = (ln > 1e-12) & (n[:, 2] > 0); Q.append(q[ok]); N.append(n[ok] / ln[ok, None]); ID.append(np.full(int(ok.sum()), k + 1))
    Q, N, ID = np.concatenate(Q), np.concatenate(N), np.concatenate(ID); D = Q[:, :, 2].mean(axis=1); o = np.argsort(D); Q, N, ID, D = Q[o], N[o], ID[o], D[o]
    dn = (D - D.min()) / max(D.max() - D.min(), 1e-9); d24 = np.round(dn * (2 ** 24 - 1)).astype(np.int64)

    def render(cols):
        fig = Figure(figsize=(W / dpi, Hh / dpi), dpi=dpi); cv = FigureCanvasAgg(fig); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.axis("off"); fig.patch.set_facecolor((1, 1, 1, 0))
        ax.add_collection(PolyCollection(Q[:, :, :2], facecolors=cols, edgecolors=cols, linewidths=0.4, antialiased=False)); cv.draw(); return np.asarray(cv.buffer_rgba()).copy()
    c_id = np.c_[(ID * 37 % 256) / 255.0, (ID * 91 % 256) / 255.0, (ID * 149 % 256) / 255.0]; I_ = render(c_id); inside = I_[:, :, 3] > 127; idimg = I_[:, :, 0].astype(int) * 65536 + I_[:, :, 1].astype(int) * 256 + I_[:, :, 2].astype(int); idimg[~inside] = -1
    Nn = render(N * 0.5 + 0.5)[:, :, :3].astype(float) / 255.0 * 2 - 1
    Dd = render(np.c_[(d24 >> 16) & 255, (d24 >> 8) & 255, d24 & 255] / 255.0); dep = (Dd[:, :, 0].astype(np.int64) * 65536 + Dd[:, :, 1].astype(np.int64) * 256 + Dd[:, :, 2]) / (2 ** 24 - 1) * (D.max() - D.min())
    E = np.zeros(inside.shape, bool); cth = math.cos(math.radians(crease_deg))
    for ax_ in (0, 1):
        s0 = [slice(None)] * 2; s1 = [slice(None)] * 2; s0[ax_] = slice(0, -1); s1[ax_] = slice(1, None); s0, s1 = tuple(s0), tuple(s1)
        e = (idimg[s0] != idimg[s1]) | (inside[s0] & inside[s1] & ((np.sum(Nn[s0] * Nn[s1], axis=2) < cth) | (np.abs(dep[s0] - dep[s1]) > depth_jump)))
        E[s0] |= e
    lab, n = ndimage.label(E, structure=np.ones((3, 3))); sz = ndimage.sum(E, lab, index=np.arange(1, n + 1)); E = np.isin(lab, np.where(sz >= min_px)[0] + 1)       # 자잘한 점 지우기
    r = max(1, int(round(lw_mm / 25.4 * dpi / 2))); yy, xx = np.ogrid[-r:r + 1, -r:r + 1]; E = ndimage.binary_dilation(E, structure=(xx * xx + yy * yy) <= r * r)
    body = ndimage.binary_fill_holes(inside | E); out = np.zeros(inside.shape + (4,), np.uint8); out[body] = (255, 255, 255, 255); out[E] = (0, 0, 0, 255)
    return out
