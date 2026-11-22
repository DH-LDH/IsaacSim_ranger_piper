"""눈으로 보기: Isaac Sim 창을 띄우고 동작을 실시간으로 되풀이한다. 창을 닫거나 Ctrl+C로 끝낸다.

  VIBSIM_GUI=1 ~/isaacsim/python.sh view.py --scene amr   --case none      # AMR 출발–정지, 대책 없음
  VIBSIM_GUI=1 ~/isaacsim/python.sh view.py --scene amr   --case zvd       # AMR 속도 명령 ZVD 성형
  VIBSIM_GUI=1 ~/isaacsim/python.sh view.py --scene fixed --speed 10       # 고정 베이스, J2 10 °/s로 10° 내려와 정지

실제 진동은 끝단에서 1–2 mm라 눈으로는 거의 안 보인다. 보이게 하려면:
  --soft 0.1   관절 강성을 1/10로(모드 약 2.3 Hz, 처짐 10배). 보여 주기용이지 실측에 맞춘 값이 아니다.
  --slow 4     4배 느리게 재생
  --accel 3    AMR 가속도[m/s^2]
터미널에는 되풀이마다 정지 후 끝단 진동(끄덕임 5–10 Hz rms, 세 축 3–20 Hz rms)이 찍힌다. 팔 장착은 기본이 실물 방향(--mount-yaw 90).
"""
import argparse, math, os, time

os.environ.setdefault("VIBSIM_GUI", "1")
import numpy as np
from vibsim import POSE_A, WHEEL_R, PosServo, VibSim, app, shaper, stop_metrics3

ap = argparse.ArgumentParser()
ap.add_argument("--scene", choices=["amr", "fixed"], default="amr")
ap.add_argument("--case", default="none", help="amr: none / zv / zvd / scurve")
ap.add_argument("--accel", type=float, default=1.0); ap.add_argument("--Ta", type=float, default=0.5); ap.add_argument("--cruise", type=float, default=1.5)
ap.add_argument("--speed", type=float, default=10.0, help="fixed: J2 속도[deg/s]"); ap.add_argument("--dist", type=float, default=10.0)
ap.add_argument("--shape", default="none", help="fixed: none / zv / zvd")
ap.add_argument("--soft", type=float, default=1.0, help="관절 강성 배율(보여 주기용)")
ap.add_argument("--slow", type=float, default=1.0); ap.add_argument("--hold", type=float, default=2.5)
ap.add_argument("--susp-k", type=float, default=0.0); ap.add_argument("--susp-c", type=float, default=0.0)
ap.add_argument("--mount-yaw", type=float, default=90.0, help="팔 장착 각[deg]. 90 = 실물, 0 = 팔 앞이 진행 방향"); ap.add_argument("--sf", type=float, default=0.0, help="성형기 주파수[Hz], 0이면 자동(실물 장착 AMR 4.1, 그 밖 7.25 / 7.0)")
ap.add_argument("--loops", type=int, default=0, help="되풀이 횟수(0 = 창을 닫을 때까지)"); ap.add_argument("--shot", default="", help="첫 되풀이 중간에 화면을 이 파일로 저장(그 되풀이의 진동 값은 저장 때문에 어긋나므로 버린다)")
a = ap.parse_args()

sim = VibSim(base="fixed" if a.scene == "fixed" else "ranger", susp_k=a.susp_k, susp_c=a.susp_c, mount_yaw=a.mount_yaw)
if a.soft != 1.0:
    sim.set_KD(sim.K * a.soft, sim.D * math.sqrt(a.soft))   # 감쇠비 유지
try:
    from pxr import UsdLux
    UsdLux.DomeLight.Define(sim.stage, "/World/dome").CreateIntensityAttr(800.0)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.9, -1.3, 1.25], target=[0.15, 0.0, 0.95]) if a.scene == "fixed" else set_camera_view(eye=[0.5, -1.9, 0.9], target=[0.5, 0.0, 0.42])
except Exception as e:  # noqa: BLE001
    print("카메라·조명 설정 건너뜀:", e)

RENDER_EVERY = 20           # 1 kHz 물리, 50 Hz 화면
f_mode = a.sf if a.sf > 0 else ((4.1 if abs(a.mount_yaw) > 45 else 7.25) if a.scene == "amr" else 7.0) * math.sqrt(a.soft)


def steps(n, fn):
    """n 스텝을 실시간(또는 --slow배 느리게)으로 진행. fn(i)가 한 스텝을 처리한다."""
    t_wall = time.time()
    for i in range(n):
        fn(i)
        if i % RENDER_EVERY == 0:
            sim.world.render()
            lag = (i + 1) * sim.dt * a.slow - (time.time() - t_wall)
            if lag > 0:
                time.sleep(lag)
            if not app.is_running():
                raise KeyboardInterrupt


def shot():
    if a.shot:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), a.shot)
        for _ in range(10):
            app.update()
        print("화면 저장:", a.shot)


def run_amr():
    bcase = a.case
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A); sv = PosServo(POSE_A, sim.dt)
    imp = shaper(bcase, f_mode, 0.10) if bcase in ("zv", "zvd") else [(1.0, 0.0)]
    v = a.accel * a.Ta; Dsh = imp[-1][1]

    def base(tt):
        if tt < 0: return 0.0
        if bcase == "scurve":
            if tt < a.Ta: return v * (1 - math.cos(math.pi * tt / a.Ta)) / 2
            if tt < a.Ta + a.cruise: return v
            if tt < 2 * a.Ta + a.cruise: return v * (1 + math.cos(math.pi * (tt - a.Ta - a.cruise) / a.Ta)) / 2
            return 0.0
        if tt < a.Ta: return a.accel * tt
        if tt < a.Ta + a.cruise: return v
        if tt < 2 * a.Ta + a.cruise: return v - a.accel * (tt - a.Ta - a.cruise)
        return 0.0

    def idle(i):
        q, dq = sv.step(POSE_A); sim.step(q, dq, wheel_w=np.zeros(4), steer=np.zeros(4))
    steps(1500, idle)
    t_end = 2 * a.Ta + a.cruise + Dsh; n = int((t_end + a.hold) * sim.hz); gy = np.zeros((n, 3))

    def go(i):
        tt = i * sim.dt; vc = sum(ai * base(tt - ti) for ai, ti in imp)
        q, dq = sv.step(POSE_A); sim.step(q, dq, wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4)); gy[i] = sim.ee_gyro()
        if i == int(0.5 * n) and run_amr.first:
            run_amr.first = False; shot()
    steps(n, go)
    return stop_metrics3(np.arange(n) * sim.dt, gy, t_end, win=min(2.0, a.hold - 0.1), fs=sim.hz)


run_amr.first = True


def run_fixed():
    A = POSE_A.copy(); S = A.copy(); S[1] = A[1] - math.radians(a.dist)
    sim.set_state(S); sv = PosServo(S, sim.dt)
    imp = shaper(a.shape, f_mode, 0.10); Tl = abs(a.dist) / a.speed; Dsh = imp[-1][1]

    def idle(i):
        q, dq = sv.step(S); sim.step(q, dq)
    steps(1500, idle)
    n = int((Tl + Dsh + a.hold) * sim.hz); gy = np.zeros((n, 3))

    def go(i):
        tt = i * sim.dt; ref = S.copy()
        ref[1] = S[1] + sum(ai * (A[1] - S[1]) * min(1.0, max(0.0, (tt - ti) / Tl)) for ai, ti in imp)
        q, dq = sv.step(ref); sim.step(q, dq); gy[i] = sim.ee_gyro()
        if i == int(0.5 * n) and run_amr.first:
            run_amr.first = False; shot()
    steps(n, go)
    return stop_metrics3(np.arange(n) * sim.dt, gy, Tl + Dsh + 0.09, win=min(2.0, a.hold - 0.2), fs=sim.hz)


k = 0
try:
    while app.is_running() and (a.loops == 0 or k < a.loops):
        m = run_amr() if a.scene == "amr" else run_fixed(); k += 1
        print(f"[{k}] 정지 후 끝단 진동[rad/s]: 끄덕임 5–10 Hz rms {m['pitch_rms']:.5f} | 세 축 3–20 Hz rms {m['vec_rms']:.5f}, 최대 {m['vec_peak']:.4f}, 주로 {m['axis']} {m['f_dom']:.1f} Hz", flush=True)
except KeyboardInterrupt:
    pass
sim.close()
