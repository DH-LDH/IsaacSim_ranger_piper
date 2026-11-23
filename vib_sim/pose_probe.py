"""자세 후보를 고정 베이스에 올려 끝단 위치·그리퍼가 향하는 방향·고유 진동수·받치는 토크를 찍는다.
  ~/isaacsim/python.sh pose_probe.py "q1,..,q6[deg]" ["q1,..,q6" ...]"""
import math, sys
import numpy as np
from vibsim import VibSim, POSE_A, POSE_B
sim = VibSim(base="fixed", verbose=False)
cands = [("A", POSE_A), ("B", POSE_B)] + [(f"c{i}", np.radians([float(x) for x in s.split(",")])) for i, s in enumerate(sys.argv[1:])]
def quat_rot(q):   # (w, x, y, z) → 회전 행렬
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
AX = None
for nm, q in cands:
    sim.set_state(q)
    for _ in range(1500): sim.step(q, np.zeros(6))
    p, o = sim.ee.get_world_poses(); p = np.asarray(p)[0]; R = quat_rot(np.asarray(o)[0])
    if AX is None: AX = int(np.argmax(np.abs(R[2, :]))); SG = -np.sign(R[2, AX])     # 자세 A에서 아래를 향하는 끝단 축 = 그리퍼가 향하는 축
    d = SG * R[:, AX]; print("AXIS", AX, SG)
    M = sim.mass_matrix(); off = M.shape[0] - sim.n; Ma = M[np.ix_(off + sim.ia, off + sim.ia)]
    f = np.sort(np.sqrt(np.abs(np.linalg.eigvals(np.linalg.solve(Ma, np.diag(sim.K))))) / (2 * math.pi))[:3]
    hold = sim.K * (q - sim.q_link())
    print(f"POSE {nm} q[deg] {np.round(np.degrees(q), 1)} | 끝단 위치 x {p[0]:+.3f} y {p[1]:+.3f} z {p[2]:.3f} m | 그리퍼 방향 (x {d[0]:+.2f}, y {d[1]:+.2f}, z {d[2]:+.2f}) → 수평에서 {math.degrees(math.asin(-d[2])):+.0f}° 아래 | 모드 {np.round(f, 2)} Hz | 받치는 토크 J2 {hold[1]:+.1f} J3 {hold[2]:+.1f} J5 {hold[4]:+.2f} N·m | 처짐 J5 {math.degrees(q[4] - sim.q_link()[4]):+.2f}°", flush=True)
sim.close()
