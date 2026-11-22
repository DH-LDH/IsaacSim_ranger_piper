"""직접 명령 넣기: Isaac Sim 창을 띄워 두고, 이 터미널에 명령을 치면 그대로 움직인다.

  ~/isaacsim/python.sh drive.py                 # Ranger + PiPER
  ~/isaacsim/python.sh drive.py --scene fixed   # 고정 베이스 팔만
  (서스펜션: --susp-k 35700 --susp-c 340, 팔을 진행 방향으로 장착: --mount-yaw 0)

명령 (한 줄에 여러 개는 ; 로 구분)
  v <m/s> [가속도=1.0]        AMR 전진 속도(음수면 후진). 가속도[m/s^2] 한계로 올리고 내린다
  stop [가속도=1.0]           AMR 정지 (= v 0)
  j <관절 1-6> <각도°> [속도=10]   관절을 그 각도까지 등속[°/s]으로
  jr <관절> <변화량°> [속도=10]    지금 각도에서 그만큼
  pose a|b [속도=10]          자세 A / 자세 B로
  shape none|zv|zvd [Hz]      이후 명령(AMR 속도, 관절 모두)에 입력 성형. Hz를 안 주면 끄덕임 모드 주파수(약 7 Hz).
                              실물 장착 방향에서 AMR 직진은 좌우 모드(모형에서 약 4.1 Hz)를 흔들므로 shape zvd 4.1 처럼 준다
  soft <배율>                 관절 강성 배율(0.1이면 처짐 10배 — 보여 주기용, 실측에 맞춘 값 아님)
  slow <배율>                 재생 속도(4면 4배 느리게)
  wait <초>                   다음 명령까지 기다림(여러 명령을 한 줄에 쓸 때)
  stat                        지금 관절 각도·처짐·AMR 속도
  reset                       처음 상태로
  help / quit

움직임이 끝나고 2초가 지나면 정지 후 끝단 진동(끄덕임 5–10 Hz rms, 세 축 3–20 Hz rms, 주로 흔들린 축과 주파수)을 찍는다. 그 2초 안에 새 명령을 넣으면 그 측정은 취소된다.
"""
import argparse, math, os, queue, sys, threading, time
from collections import deque

os.environ.setdefault("VIBSIM_GUI", "1")
import numpy as np
from vibsim import POSE_A, POSE_B, WHEEL_R, PosServo, VibSim, app, shaper, stop_metrics3

ap = argparse.ArgumentParser()
ap.add_argument("--scene", choices=["amr", "fixed"], default="amr")
ap.add_argument("--susp-k", type=float, default=0.0); ap.add_argument("--susp-c", type=float, default=0.0)
ap.add_argument("--mount-yaw", type=float, default=90.0, help="팔 장착 각[deg]. 90 = 실물(팔 앞이 AMR 옆), 0 = 팔 앞이 진행 방향")
a = ap.parse_args()
AMR = a.scene == "amr"
sim = VibSim(base="ranger" if AMR else "fixed", susp_k=a.susp_k, susp_c=a.susp_c, mount_yaw=a.mount_yaw, verbose=False)
K0, D0 = sim.K.copy(), sim.D.copy()
try:
    from pxr import UsdLux
    UsdLux.DomeLight.Define(sim.stage, "/World/dome").CreateIntensityAttr(800.0)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.5, -1.9, 0.9], target=[0.5, 0.0, 0.42]) if AMR else set_camera_view(eye=[0.9, -1.3, 1.25], target=[0.15, 0.0, 0.95])
except Exception as e:  # noqa: BLE001
    print("카메라·조명 설정 건너뜀:", e)

DT, HZ = sim.dt, sim.hz
NH = int(1.0 * HZ)            # 성형기용 과거 1초
state = dict(soft=1.0, slow=1.0, shape="none", sf=None, v_t=0.0, acc=1.0, wait_until=0.0, label="", quit=False)


class Shaped:
    """기준 신호의 과거 값을 쥐고 있다가 입력 성형(지연된 값의 가중합)을 낸다."""

    def __init__(self, x0):
        self.h = deque([np.array(x0, float)] * NH, maxlen=NH); self.imp = [(1.0, 0.0)]

    def push(self, x):
        self.h.appendleft(np.array(x, float))
        return sum(ai * self.h[min(NH - 1, int(round(ti / DT)))] for ai, ti in self.imp)


def mode_f():
    return (7.25 if AMR else 7.0) * math.sqrt(state["soft"])


def init():
    global q_ref, q_t, spd, v_ref, js, bs, sv, gbuf, tbuf, prev_out, moving, t_end, t
    if AMR:
        sim.world.reset(); sim._apply_gains()
    sim.set_state(POSE_A)
    q_ref = POSE_A.copy(); q_t = POSE_A.copy(); spd = np.full(6, math.radians(10.0)); v_ref = 0.0
    state["v_t"] = 0.0
    js, bs, sv = Shaped(q_ref), Shaped([0.0]), PosServo(POSE_A, DT)
    js.imp = bs.imp = shaper(state["shape"], state["sf"] or mode_f(), 0.10)
    gbuf, tbuf = deque(maxlen=8 * HZ), deque(maxlen=8 * HZ)
    prev_out = None; moving = False; t_end = None; t = 0.0


def busy():
    return moving or abs(state["v_t"] - v_ref) > 1e-9 or np.any(np.abs(q_t - q_ref) > 1e-9)


def do(cmd):
    global q_t, spd
    p = cmd.split()
    if not p:
        return
    c, x = p[0].lower(), p[1:]
    try:
        if c in ("quit", "exit", "q"):
            state["quit"] = True
        elif c == "help":
            print(__doc__)
        elif c == "wait":
            state["wait_until"] = t + float(x[0])
        elif c in ("v", "stop"):
            if not AMR:
                print("  고정 베이스 장면에는 AMR이 없다 (--scene amr로 실행)"); return
            state["v_t"] = 0.0 if c == "stop" else float(x[0])
            rest = x if c == "stop" else x[1:]
            state["acc"] = abs(float(rest[0])) if rest else 1.0
            state["label"] = f"AMR {state['v_t']:g} m/s (가속도 {state['acc']:g} m/s², 성형 {state['shape']})"
        elif c in ("j", "jr"):
            n = int(x[0]) - 1; val = math.radians(float(x[1])); s = math.radians(abs(float(x[2]))) if len(x) > 2 else math.radians(10.0)
            q_t = q_t.copy(); q_t[n] = val if c == "j" else q_t[n] + val; spd[n] = s
            state["label"] = f"J{n + 1} → {math.degrees(q_t[n]):.1f}° ({math.degrees(s):g} °/s, 성형 {state['shape']})"
        elif c == "pose":
            q_t = (POSE_A if x[0].lower() == "a" else POSE_B).copy(); spd[:] = math.radians(abs(float(x[1])) if len(x) > 1 else 10.0)
            state["label"] = f"자세 {x[0].upper()} (성형 {state['shape']})"
        elif c == "shape":
            if busy():
                print("  움직이는 중에는 성형기를 바꿀 수 없다. 멈춘 뒤에 다시."); return
            state["shape"] = x[0].lower(); state["sf"] = float(x[1]) if len(x) > 1 else None
            js.imp = bs.imp = shaper(state["shape"], state["sf"] or mode_f(), 0.10)
            print(f"  입력 성형 {state['shape']}" + ("" if state["shape"] == "none" else f" ({state['sf'] or mode_f():.2f} Hz, 명령이 {js.imp[-1][1] * 1000:.0f} ms 길어짐)"))
        elif c == "soft":
            state["soft"] = float(x[0]); sim.set_KD(K0 * state["soft"], D0 * math.sqrt(state["soft"]))
            if state["sf"] is None:
                js.imp = bs.imp = shaper(state["shape"], mode_f(), 0.10)
            print(f"  관절 강성 {state['soft']:g}배 → 팔 모드 약 {mode_f():.1f} Hz" + ("" if state["soft"] == 1.0 else " (보여 주기용)"))
        elif c == "slow":
            state["slow"] = max(0.1, float(x[0])); print(f"  재생 {state['slow']:g}배 느리게")
        elif c == "stat":
            d = np.degrees(sim.q_link() - sv.q)
            print("  관절(모터 쪽)[°] " + " ".join(f"{v:.2f}" for v in np.degrees(sv.q)) + f"\n  처짐 J2·J3·J5[°] {d[1]:.3f} {d[2]:.3f} {d[4]:.3f}"
                  + (f"\n  AMR 속도 {sim.base_vel()[0][0]:.3f} m/s (명령 {state['v_t']:g})" if AMR else ""))
        elif c == "reset":
            init(); print("  처음 상태로")
        else:
            print(f"  모르는 명령: {cmd}  (help)")
    except (IndexError, ValueError):
        print(f"  형식이 맞지 않음: {cmd}  (help)")


cmds = queue.Queue()


def reader():
    for line in sys.stdin:
        for c in line.strip().split(";"):
            cmds.put(c.strip())
    cmds.put("quit")


init()
for _ in range(1500):
    q, dq = sv.step(POSE_A); sim.step(q, dq, wheel_w=np.zeros(4) if AMR else None, steer=np.zeros(4) if AMR else None)
t = 0.0
print("\n" + "=" * 60 + f"\n준비됨 ({'Ranger + PiPER' if AMR else '고정 베이스'}). 명령을 치세요 — 예: " + ("v 0.5 ; wait 4 ; stop" if AMR else "jr 2 -10 ; wait 4 ; jr 2 10") + "   (help, quit)\n" + "=" * 60, flush=True)
threading.Thread(target=reader, daemon=True).start()

i = 0; t_wall = time.time(); t_sim0 = 0.0
try:
    while app.is_running() and not state["quit"]:
        while t >= state["wait_until"] and not cmds.empty() and not state["quit"]:
            do(cmds.get())
        # 기준 만들기: AMR 속도는 가속도 한계로, 관절은 등속으로 목표에 다가간다
        v_ref += float(np.clip(state["v_t"] - v_ref, -state["acc"] * DT, state["acc"] * DT))
        q_ref = q_ref + np.clip(q_t - q_ref, -spd * DT, spd * DT)
        vc = float(bs.push([v_ref])[0]); qc = js.push(q_ref)
        q, dq = sv.step(qc)
        sim.step(q, dq, wheel_w=np.full(4, vc / WHEEL_R) if AMR else None, steer=np.zeros(4) if AMR else None)
        t += DT; i += 1
        gbuf.append(sim.ee_gyro()); tbuf.append(t)
        out = np.r_[vc, qc]
        was = moving
        # 속도 명령이 바뀌는 중이거나 관절 명령이 바뀌는 중이면 "움직이는 중"
        moving = prev_out is not None and (abs(out[0] - prev_out[0]) > 1e-9 or np.any(np.abs(out[1:] - prev_out[1:]) > 1e-10))
        prev_out = out
        if moving and t_end is not None:
            print("  (새 움직임 — 앞 측정 취소)", flush=True); t_end = None
        if was and not moving:
            t_end = t
        if t_end is not None and t >= t_end + 2.05:
            m = stop_metrics3(np.array(tbuf), np.array(gbuf), t_end, fs=HZ)
            print(f"  ▶ {state['label']} 뒤 끝단 진동[rad/s]: 끄덕임 5–10 Hz rms {m['pitch_rms']:.5f} | 세 축 3–20 Hz rms {m['vec_rms']:.5f} "
                  f"(비틀림 {m['roll_rms']:.5f}, 끄덕임 {m['pitch_w_rms']:.5f}, 좌우 {m['yaw_rms']:.5f}), 최대 {m['vec_peak']:.4f}, 주로 {m['axis']} {m['f_dom']:.1f} Hz", flush=True)
            t_end = None
        if i % 20 == 0:
            sim.world.render()
            lag = (t - t_sim0) * state["slow"] - (time.time() - t_wall)
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.5:      # 밀렸으면 기준 시각을 다시 잡는다
                t_wall = time.time(); t_sim0 = t
except KeyboardInterrupt:
    pass
sim.close()
