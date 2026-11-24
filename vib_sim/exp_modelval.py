"""손으로 쓴 운동 방정식(원고 2절)을 시간 응답으로 검증하기 위한 기록을 만든다 → out/modelval_<자세>.npz  (model_val.py가 읽는다)
모터 쪽 각은 자세에 고정(q_m = 상수), AMR이 출발 → 턱 → 정지. 매 스텝(1 kHz) 링크 쪽 관절각, 팔 베이스 링크와 차체의 속도·자세를 적는다.
실행: ~/isaacsim/python.sh exp_modelval.py --pose A [--h 0.01 --v 0.3]"""
import argparse
import numpy as np
from vibsim import HERE, POSE_A, POSE_H, WHEEL_R, VibSim, app, _find
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.prims import RigidPrim

ap = argparse.ArgumentParser(); ap.add_argument("--pose", default="A"); ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--h", type=float, default=0.01); ap.add_argument("--tag", default=""); ap.add_argument("--hz", type=int, default=1000); ap.add_argument("--tire-k", type=float, default=0.0, help="바퀴 접촉 스프링[N/m] (0 = 강체 접촉)"); ap.add_argument("--tire-c", type=float, default=0.0); ap.add_argument("--nocol", type=int, default=0, help="1이면 팔 링크의 충돌을 끈다(진단용)")
a = ap.parse_args(); P = {"A": POSE_A, "H": POSE_H}[a.pose.upper()]; XB = 0.75
sim = VibSim(base="ranger", verbose=False, hz=a.hz, tire_k=a.tire_k, tire_c=a.tire_c)
if a.nocol:
    from pxr import Usd
    nc = 0
    for p in list(Usd.PrimRange(sim.stage.GetPrimAtPath("/World/piper"))):
        if p.IsInstance(): p.SetInstanceable(False)                    # 인스턴스 안의 충돌 프림에 쓸 수 있게 푼다
    for p in Usd.PrimRange(sim.stage.GetPrimAtPath("/World/piper")):
        if p.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False); nc += 1
    print("팔 충돌 끔:", nc, "개", flush=True)
c = UsdGeom.Cube.Define(sim.stage, "/World/bump"); c.CreateSizeAttr(1.0); xf = UsdGeom.Xformable(c)
xf.AddTranslateOp().Set(Gf.Vec3d(XB, 0.0, a.h - 0.05)); xf.AddScaleOp().Set(Gf.Vec3f(0.03, 2.0, 0.1)); UsdPhysics.CollisionAPI.Apply(c.GetPrim())
abp = _find(sim.stage, "/World/piper", "base_link", UsdPhysics.RigidBodyAPI); ab = RigidPrim(prim_paths_expr=str(abp.GetPath()), name="armbase"); sim.world.scene.add(ab)
sim.world.reset(); sim._apply_gains(); sim.set_state(P); Z6 = np.zeros(6); Z4 = np.zeros(4)
for _ in range(3 * a.hz): sim.step(P, Z6, wheel_w=Z4, steer=Z4)
Ta = a.v / 1.0; Tc = (XB + 0.9 - a.v * Ta) / a.v; T = 0.3 + 2 * Ta + Tc + 1.5; n = int(T / sim.dt)
def vprof(t):
    t = t - 0.3
    return 0.0 if t < 0 else (t * 1.0 if t < Ta else (a.v if t < Ta + Tc else max(0.0, a.v - (t - Ta - Tc) * 1.0)))
QL = np.zeros((n, 6)); DQL = np.zeros((n, 6)); VA = np.zeros((n, 6)); VB = np.zeros((n, 6)); PA = np.zeros((n, 7)); PB = np.zeros((n, 7)); TS = np.zeros((n, 6)); t = np.arange(n) * sim.dt
for i in range(n):
    QL[i] = sim.q_link(); DQL[i] = sim.dq_link(); VA[i] = np.asarray(ab.get_velocities())[0]; VB[i] = np.r_[sim.base_vel()]; TS[i] = sim.spring_torque()
    p, q = ab.get_world_poses(); PA[i] = np.r_[np.asarray(p)[0], np.asarray(q)[0]]; p, q = sim.bb.get_world_poses(); PB[i] = np.r_[np.asarray(p)[0], np.asarray(q)[0]]
    sim.step(P, Z6, wheel_w=np.full(4, vprof(t[i]) / WHEEL_R), steer=Z4)
np.savez(HERE + f"/out/modelval_{a.pose.upper()}{a.tag}.npz", t=t, ql=QL, dql=DQL, va=VA, vb=VB, pa=PA, pb=PB, ts=TS, qm=P, K=sim.K, D=sim.D, v=a.v, h=a.h)
print("→ out/modelval_%s%s.npz" % (a.pose.upper(), a.tag), "| 처짐 J2·J3 최대[mdeg]", np.round(np.degrees(np.abs(QL - QL[0]).max(0))[1:3] * 1e3, 1), flush=True)
sim.close(); app.close()
