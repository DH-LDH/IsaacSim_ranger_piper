"""손으로 세운 식과 물리 엔진(PhysX)의 질량 행렬을 대조한다(박사님 11/24 저녁: 시스템 모델을 수식으로).

1) URDF의 링크 질량·관성으로 팔의 질량 행렬 M_qq(6×6)와 베이스 결합 행렬 M_qb(6×6: 베이스 병진 가속도 3 + 각가속도 3)를 직접 계산
2) 2·3번 관절만 본 평면 2링크 식(기호로 쓴 닫힌 식)의 값
3) dump_M.py가 뽑아 둔 PhysX 값과 견줌
4) 턱을 넘는 기록에서 관성력 보정 Δq = K⁻¹ M_qb a_b 가운데 병진 가속도 몫과 각가속도 몫
실행: ~/nonmoon/.venv_pptx/bin/python model_check.py   (dump_M.py를 먼저 한 번)"""
import math, os, sys
import xml.etree.ElementTree as ET
import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); np.set_printoptions(precision=4, suppress=True, linewidth=200)
POSE_A = np.radians([-0.3, 121.1, -73.2, 1.6, 52.0, 13.7]); POSE_H = POSE_A.copy(); POSE_H[4] = math.radians(-30.0)
P_BA = np.array([-0.01858 - 0.23, 0.02232, 0.04462])                    # {B}에서 본 팔 베이스 원점
R_BA = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # 팔 베이스 좌표 → 베이스 좌표(+90° yaw)
USE_USD = False                                                         # True면 URDF 대신 모델 파일(physics.usda)의 관성을 쓴다
CAM_M, CAM_P = 0.11, np.array([-0.0115, 0.0731, 0.05])                  # 손목 카메라(질점, gripper_base 좌표)


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def axang(ax, q):
    x, y, z = ax; c, s = math.cos(q), math.sin(q); C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0.0]])


def load(path=H + "/assets/piper_abs.urdf"):
    r = ET.parse(path).getroot(); L = {}; J = []
    for l in r.findall("link"):
        i = l.find("inertial"); I = i.find("inertia"); g = lambda k: float(I.get(k))
        L[l.get("name")] = dict(m=float(i.find("mass").get("value")), c=np.array([float(v) for v in i.find("origin").get("xyz").split()]),
                                I=np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]]))
    for j in r.findall("joint"):
        o = j.find("origin"); T = np.eye(4); T[:3, :3] = rpy(*[float(v) for v in o.get("rpy").split()]); T[:3, 3] = [float(v) for v in o.get("xyz").split()]
        J.append(dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"), T=T,
                      ax=np.array([float(v) for v in j.find("axis").get("xyz").split()]) if j.find("axis") is not None else None))
    L["gripper_base"]["cam"] = True
    if USE_USD:                                                     # 시뮬레이션에 실제로 들어간 관성(모델 파일의 주관성·주축). 3번·5번 링크의 주축이 URDF와 다르다
        import re
        for n, d, q in re.findall(r'over "(\w+)" \(.*?diagonalInertia = \(([^)]*)\).*?principalAxes = \(([^)]*)\)', open(H + "/assets/piper_abs/payloads/Physics/physics.usda").read(), re.S):
            w, x, y, z = (float(v) for v in q.split(",")); R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            L[n]["I"] = R @ np.diag([float(v) for v in d.split(",")]) @ R.T
    return L, J


def bodies(L, J, q):
    """각 링크의 (질량, 질량 중심, 관성[베이스 축], 이 링크를 움직이는 팔 관절들의 (축, 축 위의 점)) — 모두 {B} 좌표"""
    T = {"base_link": np.eye(4)}; T["base_link"][:3, :3] = R_BA; T["base_link"][:3, 3] = P_BA; anc = {"base_link": []}; out = []
    for j in J:
        Tp = T[j["parent"]] @ j["T"]; a = list(anc[j["parent"]])
        if j["type"] == "revolute":
            k = int(j["name"][5:]) - 1; a.append((k, Tp[:3, :3] @ j["ax"], Tp[:3, 3].copy())); Tj = np.eye(4); Tj[:3, :3] = axang(j["ax"], q[k]); Tp = Tp @ Tj
        T[j["child"]] = Tp; anc[j["child"]] = a
    for n, l in L.items():
        R, p = T[n][:3, :3], T[n][:3, 3]; out.append((n, l["m"], p + R @ l["c"], R @ l["I"] @ R.T, anc[n]))
        if l.get("cam"): out.append(("camera", CAM_M, p + R @ CAM_P, np.zeros((3, 3)), anc[n]))
    return out, T


def matrices(q):
    L, J = load(); B, T = bodies(L, J, q); Mqq = np.zeros((6, 6)); Mqb = np.zeros((6, 6))
    for n, m, c, I, a in B:
        Jv = np.zeros((3, 6)); Jw = np.zeros((3, 6))
        for k, ax, p in a: Jv[:, k] = np.cross(ax, c - p); Jw[:, k] = ax
        Bv = np.hstack([np.eye(3), -skew(c)]); Bw = np.hstack([np.zeros((3, 3)), np.eye(3)])        # 베이스 원점의 병진 속도·각속도 → 이 링크의 속도
        Mqq += m * Jv.T @ Jv + Jw.T @ I @ Jw; Mqb += m * Jv.T @ Bv + Jw.T @ I @ Bw
    return Mqq, Mqb, B, T


def planar(q):
    """2·3번 관절만 본 평면 2링크(손목은 굳혀 3번 링크에 붙임). 팔의 평면 = {A}의 x_a–z_a. 기호: m2, l2, lc2, I2 / m3, lc3, I3, 각 φ"""
    L, J = load(); B, T = bodies(L, J, q); n = T["link2"][:3, :3] @ np.array([0, 0, 1.0])                 # 2번 관절 축(평면의 법선)
    p2, p3 = T["link2"][:3, 3], T["link3"][:3, 3]
    def lump(names):
        m = sum(b[1] for b in B if b[0] in names); c = sum(b[1] * b[2] for b in B if b[0] in names) / m
        Ic = sum(n @ b[3] @ n + b[1] * (np.dot(b[2] - c, b[2] - c) - np.dot(n, b[2] - c) ** 2) for b in B if b[0] in names)
        return m, c, Ic
    m2, c2, I2c = lump(["link2"]); m3, c3, I3c = lump(["link3", "link4", "link5", "link6", "gripper_base", "camera", "link7", "link8"])
    e1 = R_BA @ np.array([1.0, 0, 0]); e3 = np.array([0, 0, 1.0]); P = lambda v: np.array([v @ e1, v @ e3])     # 평면 좌표 (x_a, z)
    l2v, lc2v, lc3v = P(p3 - p2), P(c2 - p2), P(c3 - p3); l2, lc2, lc3 = (np.linalg.norm(v) for v in (l2v, lc2v, lc3v))
    ph2, phc2, ph3 = (math.atan2(v[1], v[0]) for v in (l2v, lc2v, lc3v))
    sg = float(np.sign(np.cross(e3, e1) @ n))                                 # q가 늘 때 평면 안에서 도는 방향(+1 = x_a에서 z 쪽으로)
    I2 = I2c + m2 * lc2 ** 2; I3 = I3c + m3 * lc3 ** 2
    M33 = I3; M23 = I3 + m3 * l2 * lc3 * math.cos(ph3 - ph2); M22 = I2 + m3 * l2 ** 2 + I3 + 2 * m3 * l2 * lc3 * math.cos(ph3 - ph2)
    ep = lambda a: np.array([-math.sin(a), math.cos(a)])                      # 각 a인 막대에 수직인 단위 벡터
    r2 = sg * (m2 * lc2 * ep(phc2) + m3 * (l2 * ep(ph2) + lc3 * ep(ph3))); r3 = sg * m3 * lc3 * ep(ph3)   # 팔 베이스의 (x_a, z) 가속도에 대한 결합
    return dict(m2=m2, m3=m3, l2=l2, lc2=lc2, lc3=lc3, I2=I2, I3=I3, ph2=math.degrees(ph2), phc2=math.degrees(phc2), ph3=math.degrees(ph3), sg=sg), np.array([[M22, M23], [M23, M33]]), np.array([r2, r3])


if __name__ == "__main__":
    Z = np.load(H + "/out/massmat.npz", allow_pickle=True); ia, off = Z["ia"], int(Z["off"])
    for nm in ("A", "H"):
        q = Z["q_" + nm]; Mqq, Mqb, B, T = matrices(q); Mp = Z["M_" + nm]; Pqq = Mp[np.ix_(off + ia, off + ia)]; Pqb = Mp[off + ia][:, :6]
        print(f"\n===== 자세 {nm} =====  팔 질량 합 {sum(b[1] for b in B):.3f} kg")
        print("M_qq 대각 (URDF 식 / PhysX):", np.diag(Mqq), "/", np.diag(Pqq)); print("  M_qq 전체의 최대 차이:", f"{np.abs(Mqq - Pqq).max():.5f}", "(최대 원소", f"{np.abs(Pqq).max():.4f})")
        print("M_qb (URDF 식, 행 = 관절 1–6, 열 = 베이스 ax ay az | αx αy αz):\n", Mqb); print("M_qb (PhysX):\n", Pqb)
        print("  병진 열 최대 차이", f"{np.abs(Mqb[:, :3] - Pqb[:, :3]).max():.5f}", "| 회전 열 최대 차이", f"{np.abs(Mqb[:, 3:] - Pqb[:, 3:]).max():.5f}")
        par, M2, C2 = planar(q); print("평면 2링크 기호 값:", {k: round(v, 4) for k, v in par.items()})
        print("  M(q2,q3) 평면 식:\n", M2, "\n  PhysX:\n", Pqq[1:3, 1:3]); e1 = R_BA @ np.array([1.0, 0, 0])
        print("  결합(팔 앞 방향 x_a, 위 z) 평면 식:\n", C2, "\n  PhysX(열 = x_a 방향, z):\n", np.c_[Pqb[1:3, :3] @ e1, Pqb[1:3, 2]])
        # 원고 2절의 꼴: 링크 각 θ = (θ2, δ)로 쓴 식. 기준점 P = 2번 관절 축 위의 점(어깨). N = [[−S1, C1, M11], [−S2, C2, M12]],  dθ/dq = −1
        t2, tc2, t3 = (math.radians(par[k]) for k in ("ph2", "phc2", "ph3")); m2, m3, l2, lc2, lc3 = (par[k] for k in ("m2", "m3", "l2", "lc2", "lc3"))
        S1 = m2 * lc2 * math.sin(tc2) + m3 * (l2 * math.sin(t2) + lc3 * math.sin(t3)); C1 = m2 * lc2 * math.cos(tc2) + m3 * (l2 * math.cos(t2) + lc3 * math.cos(t3)); S2 = m3 * lc3 * math.sin(t3); C2_ = m3 * lc3 * math.cos(t3)
        rP = P_BA + np.array([0, 0, 0.123]); PangP = Pqb[:, 3:] + Pqb[:, :3] @ skew(rP)          # 회전 열을 P 기준으로 옮김
        print(f"  θ2 {par['ph2']:.2f}°, θ3 {par['ph3']:.2f}°, δ {par['ph3'] - par['ph2']:.2f}°, ε {par['phc2'] - par['ph2']:.2f}°  |  S1 {S1:.4f} C1 {C1:.4f} S2 {S2:.4f} C2 {C2_:.4f}")
        print("  N(θ) 평면 식 [a_x, a_z, α]:\n", np.array([[-S1, C1, M2[0, 0]], [-S2, C2_, M2[0, 1]]]), "\n  PhysX에서 옮긴 값(−행, P 기준):\n", -np.c_[Pqb[1:3, 1], Pqb[1:3, 2], PangP[1:3, 0]])
        Kd = np.diag(Z["K"][1:3]); fr = lambda M: np.sort(np.sqrt(np.linalg.eigvals(np.linalg.solve(M, Kd)).real)) / (2 * math.pi)
        print("  평면 2자유도 고유 진동수[Hz]  식(URDF 관성):", fr(M2), " PhysX 블록:", fr(Pqq[1:3, 1:3]))
        mr = sum(b[1] * (R_BA.T @ (b[2] - P_BA))[0] for b in B if b[0] != "base_link"); print(f"  1번 관절: I1 식 {Mqq[0, 0]:.4f} / PhysX {Pqq[0, 0]:.4f},  m·ρ 식 {mr:.4f} / PhysX {-Pqb[0, 0]:.4f},  요 열(P 기준) PhysX {PangP[0, 2]:.4f}")
    # ---- 턱을 넘는 기록에서 보정량의 병진 몫과 회전 몫 ----
    K = Z["K"]
    for tag, nm in (("rl_z_j23_base", "A"), ("rlH_z_j23_base", "H")):
        p = f"{H}/out/exp6_ts_{tag}_none.npz"
        if not os.path.exists(p): print("없음:", p); continue
        d = np.load(p); A6 = d["a6"]; FF = Z["M_" + nm][off + ia][:, :6] / K[:, None]; t = d["t"]; tf, tr = float(d["tf"]), float(d["tr"])
        m = (t > tf - 0.05) & (t < tr + 0.6)                                  # 앞바퀴가 닿기 직전 ~ 뒷바퀴가 넘은 뒤
        rms = lambda x: np.sqrt(np.mean(x[m] ** 2, axis=0))
        print(f"\n--- 자세 {nm}, 턱 10 mm·0.3 m/s, 보정 없는 실행의 베이스 가속도(40 Hz 저역 통과) ---")
        print("베이스 가속도 실효값: 병진 xyz [m/s²]", rms(A6[:, :3]), "| 각가속도 xyz(롤·피치·요) [rad/s²]", rms(A6[:, 3:]))
        tot = rms(A6 @ FF.T); lin = rms(A6[:, :3] @ FF[:, :3].T); ang = rms(A6[:, 3:] @ FF[:, 3:].T)
        print("관성력 보정각 실효값[mdeg] 관절 1–6  전체:", np.degrees(tot) * 1e3); print("   병진 가속도 몫:", np.degrees(lin) * 1e3); print("   각가속도 몫  :", np.degrees(ang) * 1e3)
        for c, cn in enumerate(["ax", "ay", "az", "롤", "피치", "요"]): print(f"   {cn:>3} 성분만: J2 {np.degrees(rms(A6[:, c] * FF[1, c])) * 1e3:7.2f}  J3 {np.degrees(rms(A6[:, c] * FF[2, c])) * 1e3:7.2f}  J1 {np.degrees(rms(A6[:, c] * FF[0, c])) * 1e3:7.2f} mdeg")
