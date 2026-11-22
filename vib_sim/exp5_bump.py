"""5단계(예비): 턱 넘기 — 미리 모르는 흔들림에 베이스 IMU 피드포워드가 얼마나 듣는가, 지연에 따라.

AMR이 등속으로 가다가 바닥의 턱(높이 --h, 길이 3 cm, 바퀴 네 개가 모두 넘음)을 넘는다. 팔은 자세 A 유지(실물 장착 방향).
베이스 IMU 피드포워드: 베이스의 가속도(병진 3 + 회전 3)를 재서, 그 관성력이 만드는 관절 처짐만큼 모터 쪽을 반대로 민다.
    dq_m = K^-1 C a_base(t - 지연),   C = 질량 행렬의 (관절, 베이스 6자유도) 결합 항
조건(--cases): none / ff0 ff5 ff10 ff20 ff40 (숫자 = 지연[ms], 모터 쪽이 그대로 따라간다고 본 이상적인 경우) /
              ffservo (위치 모드 서보 모형을 거침: 센서 10 ms + 지연 20 ms + 10 Hz 2차)
출력: out/exp5_<tag>.csv.  바퀴와 턱이 강체라 충격이 실물보다 날카롭다 — 절댓값이 아니라 조건끼리의 비로만 본다.
"""
import argparse, csv, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, WHEEL_R, PosServo, VibSim, band   # vibsim이 먼저 Isaac을 띄운다
from pxr import Gf, UsdGeom, UsdPhysics

ap = argparse.ArgumentParser()
ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--h", type=float, default=0.01)
ap.add_argument("--cases", default="none,ff0,ff5,ff10,ff20,ff40,ffservo")
ap.add_argument("--susp-k", type=float, default=120000.0); ap.add_argument("--susp-c", type=float, default=623.0)
ap.add_argument("--lp", type=float, default=40.0, help="IMU 가속도 저역 통과[Hz]"); ap.add_argument("--gain", type=float, default=1.0)
ap.add_argument("--tag", default="bump"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
sim = VibSim(base="ranger", susp_k=a.susp_k, susp_c=a.susp_c)
os.makedirs(HERE + "/out", exist_ok=True)
XB = 0.75                                    # 턱 위치[m] — 앞바퀴(+0.247)가 0.5 m쯤 가서 만난다
cube = UsdGeom.Cube.Define(sim.stage, "/World/bump"); cube.CreateSizeAttr(1.0)
x = UsdGeom.Xformable(cube); x.AddTranslateOp().Set(Gf.Vec3d(XB, 0.0, 0.0)); x.AddScaleOp().Set(Gf.Vec3f(0.03, 2.0, 2 * a.h))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
for _ in range(2500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
M = sim.mass_matrix(); off = M.shape[0] - sim.n
C = M[off + sim.ia][:, :6]                    # 관절 x 베이스(병진 xyz, 회전 xyz)
FF = C / sim.K[:, None]
print("결합 항 C (J1,J2,J3,J5 x [ax ay az wx wy wz]):\n", np.round(C[[0, 1, 2, 4]], 3), flush=True)


def run(case):
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
    sv = PosServo(POSE_A, sim.dt)
    for _ in range(2500):
        sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
    delay = {"none": None, "ffservo": 10}.get(case, None)
    if case.startswith("ff") and case != "ffservo":
        delay = int(case[2:])
    T = 0.3 + (XB + 0.3 + 0.5) / a.v + 2.0; n = int(T * sim.hz); t = np.arange(n) * sim.dt
    G = np.zeros((n, 3)); XBASE = np.zeros(n); A6 = np.zeros((n, 6)); DQ = np.zeros((n, 6))
    v_prev = np.zeros(6); af = np.zeros(6); al = 1 - math.exp(-2 * math.pi * a.lp * sim.dt); buf = [np.zeros(6)] * 80
    x0 = None
    for i in range(n):
        vc = min(a.v, 1.0 * max(0.0, t[i] - 0.1))             # 1 m/s^2로 속도 올린 뒤 등속
        lv, av = sim.base_vel(); v6 = np.r_[lv, av]
        a6 = (v6 - v_prev) / sim.dt if i > 0 else np.zeros(6); v_prev = v6
        af = af + al * (a6 - af); buf.append(af.copy()); buf.pop(0)
        dq = np.zeros(6)
        if delay is not None:
            dq = a.gain * FF @ buf[-1 - delay]
        if case == "ffservo":
            q, w = sv.step(POSE_A + dq)
        else:
            q, w = POSE_A + dq, np.zeros(6)
        sim.step(q, w, wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))
        G[i] = sim.ee_gyro(); A6[i] = af; DQ[i] = dq
        p = np.asarray(sim.bb.get_world_poses()[0])[0]; XBASE[i] = p[0]
        if x0 is None: x0 = p[0]
    k = np.where(XBASE - x0 + 0.247 >= XB - 0.02)[0]
    t0 = t[k[0]] - 0.1 if len(k) else 0.0
    m = (t >= t0) & (t < t0 + 3.2)
    B = np.stack([band(G[m, c], sim.hz, 2.0, 20.0) for c in range(3)], axis=1); ax = np.sqrt(np.mean(B ** 2, axis=0))
    row = dict(case=case, v=a.v, h=a.h, susp_k=a.susp_k, vec=float(np.sqrt(np.sum(ax ** 2))), roll=float(ax[0]), pitch=float(ax[1]), yaw=float(ax[2]),
               peak=float(np.max(np.linalg.norm(B, axis=1))), base_az_peak=float(np.max(np.abs(A6[m, 2]))), base_wy_acc_peak=float(np.max(np.abs(A6[m, 4]))),
               dq_peak_deg=float(np.degrees(np.max(np.abs(DQ)))), crossed=bool(len(k)), x_end=float(XBASE[-1] - x0))
    if a.keep:
        np.savez(HERE + f"/out/exp5_ts_{a.tag}_{case}.npz", t=t, gyro=G, a6=A6, dq=DQ, t0=t0)
    return row


tw = time.time(); rows = []
for case in a.cases.split(","):
    rows.append(run(case)); print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in rows[-1].items()}, flush=True)
with open(HERE + f"/out/exp5_{a.tag}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print("wall %.1f s" % (time.time() - tw))
sim.close()
