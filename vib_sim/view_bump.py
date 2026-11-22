"""눈으로 보기 — 턱 넘기: AMR이 등속으로 가다 바닥의 턱을 넘는다. 대책 없음과 대책 적용을 번갈아 틀고 끝단 진동을 견준다.

  ~/isaacsim/python.sh view_bump.py                           # 기본: 베이스 IMU 피드포워드(10 ms) + 위상 예측 되먹임(10 ms)
  ~/isaacsim/python.sh view_bump.py --case damp20p            # 위상 예측 되먹임만, 지연 20 ms
  ~/isaacsim/python.sh view_bump.py --case damp36             # 예측 없는 되먹임, 지연 36 ms (발산하는 경우)
  ~/isaacsim/python.sh view_bump.py --case none               # 대책 없음만

  --case   + 로 이어 붙인다:  ffN(베이스 IMU 피드포워드, 지연 N ms) / dampN(끝단 자이로 되먹임) / dampNp(+ 위상 예측)
           prev(뒷바퀴 예측: 앞바퀴 때 잰 것을 뒷바퀴 시각에 그대로) / prevzE10(상하·끄덕임 성분만 예측, 10 ms 늦게 — ffN과 같이 쓴다)
  --pothole 0.06  턱 대신 패인 바닥(길이 m, 깊이는 --h)
  --mount-x -0.23 팔을 뒷바퀴 쪽으로 옮김(m). 0 = 가운데, -0.23 = 뒷바퀴 축 바로 위, -0.30 = 차체 뒤 끝 가까이
  --h 0.01 턱 높이[m]   --v 0.3 속도[m/s]   --slow 2 두 배 느리게   --only 번갈아 틀지 않고 그 조건만
대책 없음 → 대책 적용 순으로 번갈아 튼다. 터미널에 그때마다 끝단 진동(세 축 2–20 Hz)과 대책 없음 대비 비가 찍힌다.
바퀴와 턱이 강체라 충격이 실물보다 날카롭고, 모터 쪽이 보정 명령을 그대로 따라간다고 본 이상적인 경우다.
"""
import argparse, math, os, time

os.environ.setdefault("VIBSIM_GUI", "1")
import numpy as np
from vibsim import POSE_A, WHEEL_R, VibSim, app, band
from pxr import Gf, UsdGeom, UsdPhysics, UsdLux

ap = argparse.ArgumentParser()
ap.add_argument("--case", default="ff10+damp10p"); ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--h", type=float, default=0.01)
ap.add_argument("--susp-k", type=float, default=0.0); ap.add_argument("--susp-c", type=float, default=0.0)
ap.add_argument("--tire-k", type=float, default=150000.0); ap.add_argument("--tire-c", type=float, default=1000.0)
ap.add_argument("--slow", type=float, default=1.0); ap.add_argument("--only", action="store_true"); ap.add_argument("--pothole", type=float, default=0.0)
ap.add_argument("--mount-x", type=float, default=-0.23, help="팔 장착 위치를 앞뒤로 옮김[m]. 음수 = 뒷바퀴 쪽. -0.23이면 뒷바퀴 축 바로 위, -0.30이면 차체 뒤 끝 가까이")
ap.add_argument("--loops", type=int, default=0); ap.add_argument("--shot", default="")
a = ap.parse_args()
ZP = 0.04 if a.pothole > 0 else 0.0
sim = VibSim(base="ranger", susp_k=a.susp_k, susp_c=a.susp_c, tire_k=a.tire_k, tire_c=a.tire_c, z0=ZP, mount_x=a.mount_x, verbose=False)
XB, LWB, DT, NB = 0.75, 0.494, sim.dt, 200
def box(name, x0, x1, top, color):
    c = UsdGeom.Cube.Define(sim.stage, "/World/" + name); c.CreateSizeAttr(1.0); c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xf = UsdGeom.Xformable(c); xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, 0.0, top - 0.05)); xf.AddScaleOp().Set(Gf.Vec3f(x1 - x0, 2.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())


if a.pothole > 0:
    box("deck_a", -2.0, XB, ZP, (0.55, 0.6, 0.65)); box("deck_hole", XB, XB + a.pothole, ZP - a.h, (0.9, 0.25, 0.1)); box("deck_b", XB + a.pothole, XB + a.pothole + 4.0, ZP, (0.55, 0.6, 0.65))
else:
    box("bump", XB - 0.015, XB + 0.015, a.h, (0.9, 0.25, 0.1))
try:
    UsdLux.DomeLight.Define(sim.stage, "/World/dome").CreateIntensityAttr(800.0)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[XB + 0.9, -1.7, 0.75], target=[XB + 0.1, 0.0, 0.3])
except Exception as e:  # noqa: BLE001
    print("카메라·조명 설정 건너뜀:", e)
sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
for _ in range(2500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
M = sim.mass_matrix(); off = M.shape[0] - sim.n
FF = M[off + sim.ia][:, :6] / sim.K[:, None]; FLIP = np.array([1, 1, 1, 1, -1, 1.0])
ZADD, FP, FY, LP = 0.3, 7.5, 4.1, 40.0


def run(case, first):
    tok = case.split("+")
    ffd = next((int(t[2:]) for t in tok if t.startswith("ff")), None)
    pv = next((t for t in tok if t.startswith("prev")), None); prev = pv is not None; pz = bool(pv and pv.startswith("prevz"))
    perr = int(pv[6 if pz else 5:]) if pv and len(pv) > (5 if pz else 4) else 0
    dmp = next((t for t in tok if t.startswith("damp")), None)
    dd = int(dmp[4:].rstrip("p")) if dmp else None; dpred = bool(dmp and dmp.endswith("p"))
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
    T = 0.3 + (XB + 0.3 + 0.5) / a.v + 1.5; n = int(T * sim.hz); t = np.arange(n) * DT; G = np.zeros((n, 3))
    v_prev = np.zeros(6); af = np.zeros(6); gf = np.zeros(3); al = 1 - math.exp(-2 * math.pi * LP * DT)
    abuf = [np.zeros(6)] * NB; gbuf = [np.zeros(3)] * NB; t_f = None; rec = []; i_rear = None; tw = time.time()
    for i in range(-2000, n):
        if i < 0:
            sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
        else:
            vc = min(a.v, 1.0 * max(0.0, t[i] - 0.1))
            lv, av = sim.base_vel(); v6 = np.r_[lv, av]
            a6 = (v6 - v_prev) / DT if i > 0 else np.zeros(6); v_prev = v6
            af = af + al * (a6 - af); abuf.append(af.copy()); abuf.pop(0)
            g_now = sim.ee_gyro(); gf = gf + al * (g_now - gf); gbuf.append(gf.copy()); gbuf.pop(0)
            dq = np.zeros(6)
            pb_ = np.asarray(sim.bb.get_world_poses()[0])[0][0]
            if t_f is None and pb_ + LWB / 2 >= XB - 0.06:
                t_f = t[i]; rec = [b.copy() for b in abuf[-30:]]; i_rear = i + int(round((LWB / a.v + perr / 1000.0) / DT)) - 30
                print(f"      앞바퀴가 턱에 닿음 {t_f:.2f} s → 뒷바퀴는 {t_f + LWB / a.v:.2f} s쯤", flush=True)
            elif t_f is not None and len(rec) < 700:
                rec.append(af.copy())
            if prev and pz and i_rear is not None and 0 <= i - i_rear < len(rec):
                am = abuf[-1 - (ffd if ffd is not None else 0)].copy(); am[[2, 4]] = (FLIP * rec[i - i_rear])[[2, 4]]; dq += FF @ am
            elif prev and i_rear is not None and 0 <= i - i_rear < len(rec):
                dq += FF @ (FLIP * rec[i - i_rear])
            elif ffd is not None:
                dq += FF @ abuf[-1 - ffd]
            if dd is not None:
                s0 = gbuf[-1 - dd]; ds = (s0 - gbuf[-1 - dd - 4]) / (4 * DT)
                for ax_, j, f in ((1, 4, FP), (2, 0, FY)):
                    w = 2 * math.pi * f; val = s0[ax_]
                    if dpred:
                        ph = w * (dd / 1000.0 + 1.0 / (2 * math.pi * LP)); val = s0[ax_] * math.cos(ph) + ds[ax_] / w * math.sin(ph)
                    dq[j] += -(2 * ZADD / w) * val
            sim.step(POSE_A + dq, np.zeros(6), wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4)); G[i] = g_now
        if i % 20 == 0:
            sim.world.render()
            lag = (i + 2000) * DT * a.slow - (time.time() - tw)
            if lag > 0:
                time.sleep(lag)
            if not app.is_running():
                raise KeyboardInterrupt
        if first and a.shot and t_f is not None and abs(t[max(i, 0)] - (t_f + 0.05)) < DT / 2:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture_viewport_to_file(get_active_viewport(), a.shot)
            for _ in range(10):
                app.update()
            print("화면 저장:", a.shot)
    tf = t_f if t_f is not None else 0.0; tr_ = tf + LWB / a.v

    def rms(t0, t1):
        m = (t >= t0) & (t < t1)
        ax = np.sqrt(np.mean(np.stack([band(G[m, c], sim.hz, 2.0, 20.0) for c in range(3)], axis=1) ** 2, axis=0))
        return float(np.sqrt(np.sum(ax ** 2))), ax
    v_all, ax = rms(tf - 0.1, tr_ + 1.5)
    return v_all, ax, rms(tf - 0.1, tr_ - 0.1)[0], rms(tr_ - 0.1, tr_ + 1.5)[0]


print(f"\n턱 높이 {a.h * 1000:g} mm, 속도 {a.v:g} m/s, 팔 위치 {a.mount_x:+.2f} m, 조건 {a.case}" + ("" if a.only or a.case == "none" else " (대책 없음과 번갈아)"), flush=True)
k = 0; base = None
try:
    while app.is_running() and (a.loops == 0 or k < a.loops):
        cases = [a.case] if (a.only or a.case == "none") else ["none", a.case]
        for c in cases:
            v, ax, vf, vr = run(c, k == 0 and c == cases[-1])
            if c == "none":
                base = (v, vf, vr)
            print(f"[{k + 1}] {'대책 없음' if c == 'none' else c:14s} 끝단 진동 {v:.4f} rad/s (앞바퀴 구간 {vf:.4f}, 뒷바퀴 구간 {vr:.4f})"
                  + (f" → 대책 없음의 {v / base[0]:.2f}배 (앞 {vf / base[1]:.2f}, 뒤 {vr / base[2]:.2f})" if c != "none" and base else ""), flush=True)
        k += 1
except KeyboardInterrupt:
    pass
sim.close()
