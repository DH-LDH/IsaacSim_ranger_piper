"""5단계(예비): 턱 넘기 — 미리 모르는 흔들림을 얼마나 잡을 수 있는가.

AMR이 등속으로 가다가 바닥의 턱(높이 --h, 길이 3 cm, 앞바퀴 둘 → 뒷바퀴 둘 순서로 넘음)을 넘는다. 팔은 자세 A 유지(실물 장착 방향).
모터 쪽이 보정 명령을 그대로 따라간다고 본 이상적인 경우다(ffservo만 위치 모드 서보 모형을 거침).

조건은 + 로 이어 붙인다(--cases, 쉼표로 여러 개):
  none
  ffN       베이스 IMU 피드포워드, 지연 N ms.  dq_m = K^-1 C a_base(t - N)   (C = 질량 행렬의 관절 x 베이스 6자유도 결합 항)
  ffservo   같은 피드포워드를 위치 모드 서보(센서 10 ms + 지연 20 ms + 10 Hz 2차)로
  prev      뒷바퀴 예측: 앞바퀴가 넘을 때 잰 베이스 가속도를 기억해 두었다가, 뒷바퀴가 넘을 시각(축간 거리 / 속도 뒤)에 지연 없이 미리 넣는다.
            뒷바퀴가 넘을 때는 차체 끄덕임의 부호가 반대라 그 성분만 뒤집는다.  prevE10 = 예측 시각이 10 ms 늦음(E-10 = 10 ms 이름)
  dampN     끝단 자이로 되먹임 감쇠, 지연 N ms.  끄덕임 → J5, 좌우 → J1 모터 쪽을 각속도 반대 방향으로 민다(감쇠비 +0.3 상당)
  dampNp    같은 되먹임에 위상 예측: 울림이 한 주파수(끄덕임 7.5 Hz, 좌우 4.1 Hz)라고 보고 N ms 뒤의 값을 계산해 쓴다
출력: out/exp5_<tag>.csv — 끝단 자이로 세 축 2–20 Hz rms: 전체 / 앞바퀴 구간 / 뒷바퀴 구간.
바퀴와 턱이 강체라 충격이 실물보다 날카롭다 — 절댓값이 아니라 조건끼리의 비로만 본다.
"""
import argparse, csv, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, WHEEL_R, PosServo, VibSim, band   # vibsim이 먼저 Isaac을 띄운다
from pxr import Gf, UsdGeom, UsdPhysics

ap = argparse.ArgumentParser()
ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--h", type=float, default=0.01)
ap.add_argument("--cases", default="none,ff0,ff5,ff10,ff20,ff40,ffservo")
ap.add_argument("--susp-k", type=float, default=120000.0); ap.add_argument("--susp-c", type=float, default=623.0)
ap.add_argument("--lp", type=float, default=40.0, help="IMU 저역 통과[Hz]"); ap.add_argument("--zadd", type=float, default=0.3, help="되먹임으로 더하려는 감쇠비")
ap.add_argument("--fp", type=float, default=7.5); ap.add_argument("--fy", type=float, default=4.1)
ap.add_argument("--tag", default="bump"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
sim = VibSim(base="ranger", susp_k=a.susp_k, susp_c=a.susp_c)
os.makedirs(HERE + "/out", exist_ok=True)
XB, LWB = 0.75, 0.494                        # 턱 위치[m], 축간 거리[m]
cube = UsdGeom.Cube.Define(sim.stage, "/World/bump"); cube.CreateSizeAttr(1.0)
x = UsdGeom.Xformable(cube); x.AddTranslateOp().Set(Gf.Vec3d(XB, 0.0, 0.0)); x.AddScaleOp().Set(Gf.Vec3f(0.03, 2.0, 2 * a.h))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
for _ in range(2500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
M = sim.mass_matrix(); off = M.shape[0] - sim.n
FF = M[off + sim.ia][:, :6] / sim.K[:, None]  # 베이스 가속도(병진 xyz, 회전 xyz) → 모터 쪽 보정각
FLIP = np.array([1, 1, 1, 1, -1, 1.0])        # 뒷바퀴가 넘을 때: 차체 끄덕임(wy)만 부호 반대
DT = sim.dt; NB = 200


def run(case):
    tok = case.split("+")
    ffd = next((int(t[2:]) for t in tok if t.startswith("ff") and t != "ffservo"), None)
    servo = "ffservo" in tok
    prev = next((t for t in tok if t.startswith("prev")), None); perr = int(prev[5:]) if prev and len(prev) > 4 else 0
    dmp = next((t for t in tok if t.startswith("damp")), None)
    dd = int(dmp[4:].rstrip("p")) if dmp else None; dpred = bool(dmp and dmp.endswith("p"))
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A); sv = PosServo(POSE_A, DT)
    for _ in range(2500):
        sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
    T = 0.3 + (XB + 0.3 + 0.5) / a.v + 2.0; n = int(T * sim.hz); t = np.arange(n) * DT
    G = np.zeros((n, 3)); XBASE = np.zeros(n); DQ = np.zeros((n, 6)); A6 = np.zeros((n, 6))
    v_prev = np.zeros(6); af = np.zeros(6); gf = np.zeros(3); al = 1 - math.exp(-2 * math.pi * a.lp * DT)
    abuf = [np.zeros(6)] * NB; gbuf = [np.zeros(3)] * NB
    t_f = None; rec = []; i_rear = None; x0 = None
    lp_lag = 1.0 / (2 * math.pi * a.lp)       # 저역 통과가 더하는 지연(대략)
    for i in range(n):
        vc = min(a.v, 1.0 * max(0.0, t[i] - 0.1))
        lv, av = sim.base_vel(); v6 = np.r_[lv, av]
        a6 = (v6 - v_prev) / DT if i > 0 else np.zeros(6); v_prev = v6
        af = af + al * (a6 - af); abuf.append(af.copy()); abuf.pop(0)
        g_now = sim.ee_gyro(); gf = gf + al * (g_now - gf); gbuf.append(gf.copy()); gbuf.pop(0)
        dq = np.zeros(6)
        # --- 앞바퀴가 턱에 닿은 때를 베이스 상하 가속도로 잡는다 ---
        if t_f is None and t[i] > 0.6 and abs(af[2]) > 1.5:
            t_f = t[i]; rec = [b.copy() for b in abuf[-30:]]          # 30 ms 앞부터 기억
            i_rear = i + int(round((LWB / a.v + perr / 1000.0) / DT)) - 30
        elif t_f is not None and len(rec) < 700:
            rec.append(af.copy())
        in_rear = prev is not None and i_rear is not None and 0 <= i - i_rear < len(rec)
        if in_rear:
            dq += FF @ (FLIP * rec[i - i_rear])                        # 뒷바퀴: 기억해 둔 것을 지연 없이
        elif ffd is not None:
            dq += FF @ abuf[-1 - ffd]
        elif servo:
            dq += FF @ abuf[-1 - 10]
        if dd is not None:                                             # 끝단 자이로 되먹임 감쇠
            s0 = gbuf[-1 - dd]; s1 = gbuf[-1 - dd - 4]; ds = (s0 - s1) / (4 * DT)
            for ax_, j, f in ((1, 4, a.fp), (2, 0, a.fy)):             # 끄덕임 → J5, 좌우 → J1
                w = 2 * math.pi * f; val = s0[ax_]
                if dpred:
                    ph = w * (dd / 1000.0 + lp_lag)
                    val = s0[ax_] * math.cos(ph) + ds[ax_] / w * math.sin(ph)   # 한 주파수 울림이라고 보고 앞당김
                dq[j] += -(2 * a.zadd / w) * val
        if servo:
            q, w_ = sv.step(POSE_A + dq)
        else:
            q, w_ = POSE_A + dq, np.zeros(6)
        sim.step(q, w_, wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))
        G[i] = g_now; DQ[i] = dq; A6[i] = af
        p = np.asarray(sim.bb.get_world_poses()[0])[0]; XBASE[i] = p[0]
        if x0 is None: x0 = p[0]
    tf = t_f if t_f is not None else 0.0; tr = tf + LWB / a.v

    def rms(t0, t1):
        m = (t >= t0) & (t < t1)
        B = np.stack([band(G[m, c], sim.hz, 2.0, 20.0) for c in range(3)], axis=1); ax = np.sqrt(np.mean(B ** 2, axis=0))
        return float(np.sqrt(np.sum(ax ** 2))), ax
    allv, ax = rms(tf - 0.1, tr + 1.5); fr, _ = rms(tf - 0.1, tr - 0.1); re, _ = rms(tr - 0.1, tr + 1.5)
    row = dict(case=case, v=a.v, h=a.h, vec=allv, front=fr, rear=re, roll=float(ax[0]), pitch=float(ax[1]), yaw=float(ax[2]),
               dq_peak_deg=float(np.degrees(np.max(np.abs(DQ)))), t_front=tf, x_end=float(XBASE[-1] - x0))
    if a.keep:
        np.savez(HERE + f"/out/exp5_ts_{a.tag}_{case}.npz", t=t, gyro=G, dq=DQ, a6=A6, tf=tf, tr=tr)
    return row


tw = time.time(); rows = []
for case in a.cases.split(","):
    rows.append(run(case)); print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in rows[-1].items()}, flush=True)
with open(HERE + f"/out/exp5_{a.tag}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
b = rows[0]
print("\n조건            전체   앞바퀴  뒷바퀴   (대책 없음 대비 비)   보정 최대[deg]")
for r in rows:
    print("%-16s %.2f   %.2f   %.2f      %.1f" % (r["case"], r["vec"] / b["vec"], r["front"] / b["front"], r["rear"] / b["rear"], r["dq_peak_deg"]))
print("wall %.1f s" % (time.time() - tw))
sim.close()
