"""진단: 시작할 때 차체가 얼마나 튀고 방향이 얼마나 틀어지는지, 직진 중에 얼마나 더 틀어지는지."""
import math, sys
import numpy as np
from vibsim import POSE_A, WHEEL_R, VibSim
susp = float(sys.argv[1]) if len(sys.argv) > 1 else 120000.0
tire = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
sim = VibSim(base="ranger", susp_k=susp, susp_c=623.0 if susp > 0 else 0.0, tire_k=tire, tire_c=float(sys.argv[3]) if len(sys.argv) > 3 else 0.0, verbose=False)
def pose():
    p, q = sim.bb.get_world_poses(); p = np.asarray(p)[0]; w, x, y, z = np.asarray(q)[0]
    return p, math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
p0, y0, _ = pose(); zmax = p0[2]; print("서스펜션", susp, "타이어", tire, "| 시작 직후 base z %.4f yaw %.3f" % (p0[2], y0))
for i in range(2500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4)); p, y, _ = pose(); zmax = max(zmax, p[2])
    if i in (50, 200, 600, 2499): print("  %4d ms: z %.4f (최고 %.4f) x %.4f y %.4f yaw %.3f°" % (i + 1, p[2], zmax, p[0], p[1], y))
ps = p.copy()
for i in range(5000):
    vc = min(0.3, 1.0 * max(0.0, i * 0.001 - 0.1))
    sim.step(POSE_A, np.zeros(6), wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))
    if i in (1499, 2999, 4999): p, y, _ = pose(); print("  직진 %.1f s: x %.3f y %.4f yaw %.3f°  (턱 길이 방향으로 좌우 바퀴 어긋남 %.1f mm)" % ((i + 1) / 1000, p[0] - ps[0], p[1] - ps[1], y, 364 * math.tan(math.radians(abs(y)))))
sim.close()
