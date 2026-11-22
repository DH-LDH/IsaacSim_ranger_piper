"""눈으로 보기 — 실측 엔코더 기록 재생: 실기 외란 시나리오 기록을 모터 쪽에 그대로 넣고, 베이스를 같이 움직인다.

  ~/isaacsim/python.sh view_replay.py --scn 1 --ctrl pos --base move     # 1축, 위치 모드 기록, 베이스 앞뒤 이동
  ~/isaacsim/python.sh view_replay.py --scn 3 --ctrl asmc --base zvd     # 3축, ASMC16 기록, 베이스 이동 + ZVD 성형
  ~/isaacsim/python.sh view_replay.py --scn 2 --ctrl tde --base fixed    # 2축, 하이브리드 기록, 고정 베이스(팔만)

  --scn  1(J2) / 2(J1+J2) / 3(J1+J2+J3)          --ctrl pos(위치 모드) / tde(하이브리드) / asmc(ASMC16)
  --base fixed(팔만) / still(AMR 정지) / move(AMR 앞뒤 이동, 1 m/s^2) / zvd(이동 + ZVD 성형)
  --t0 --t1  기록의 어느 구간을 틀지[s] (기본: 처음부터 20 s)      --slow 2  두 배 느리게
  --soft 0.2 관절 강성 배율(흔들림을 크게 보이게 — 보여 주기용, 실측에 맞춘 값 아님)
  --file     기록 파일을 직접 지정(piper_ws docs/data/raw 아래 *.csv.gz)

MIT 기록(tde, asmc)에는 순간 튐이 들어 있다. 화면에서 팔이 툭 튀는 순간이 그것이고, 터미널에 튐 시각을 미리 찍어 준다.
터미널에는 되풀이마다 끝단 진동(튐 구간 제외)이 찍힌다. 창을 닫거나 Ctrl+C로 끝낸다.
"""
import argparse, csv, gzip, math, os, time

os.environ.setdefault("VIBSIM_GUI", "1")
import numpy as np
from vibsim import WHEEL_R, VibSim, app, band, shaper

RAW = os.path.expanduser("~/piper_ws/docs/data/raw")
FILES = {("1", "pos"): "scn/20261029_110051_slow_j2_s1", ("1", "tde"): "scn/20261029_110813_sine", ("1", "asmc"): "scn/20261029_111226_sine",
         ("2", "pos"): "scn/20261029_204150_slow_j12_sxy", ("2", "tde"): "scn/20261029_203040_sine", ("2", "asmc"): "scn/20261029_112534_sine",
         ("3", "pos"): "scn/20261029_204306_slow_j123_s3", ("3", "tde"): "scn/20261029_205503_sine", ("3", "asmc"): "scn/20261029_205725_sine"}
ap = argparse.ArgumentParser()
ap.add_argument("--scn", choices=["1", "2", "3"], default="1"); ap.add_argument("--ctrl", choices=["pos", "tde", "asmc"], default="pos")
ap.add_argument("--base", choices=["fixed", "still", "move", "zvd"], default="move"); ap.add_argument("--file", default="")
ap.add_argument("--t0", type=float, default=0.0); ap.add_argument("--t1", type=float, default=20.0)
ap.add_argument("--slow", type=float, default=1.0); ap.add_argument("--soft", type=float, default=1.0); ap.add_argument("--sf", type=float, default=4.125)
ap.add_argument("--loops", type=int, default=0); ap.add_argument("--shot", default="")
a = ap.parse_args()
path = a.file or f"{RAW}/{FILES[(a.scn, a.ctrl)]}.csv.gz"
stem = os.path.basename(path).replace(".csv.gz", "").replace(".csv", "")
FIXED = a.base == "fixed"

with (gzip.open if path.endswith(".gz") else open)(path, "rt") as f:
    rec = list(csv.DictReader(f))
tr = np.array([float(x["t"]) for x in rec]); tcsv0 = float(tr[0]); tr = tr - tcsv0
qr = np.radians(np.array([[float(x[f"q{j}"]) for j in range(1, 7)] for x in rec]))
t1 = min(a.t1, tr[-1]); DT = 0.001
tt = np.arange(a.t0, t1, DT); Q = np.stack([np.interp(tt, tr, qr[:, j]) for j in range(6)], axis=1)
V = np.gradient(Q, DT, axis=0); k = np.ones(25) / 25.0; V = np.stack([np.convolve(V[:, j], k, "same") for j in range(6)], axis=1)
jp = os.path.join(RAW, "mit_jumps.csv")
excl = [(float(r["exclude_from"]) - tcsv0, float(r["exclude_to"]) - tcsv0, float(r["t_csv"]) - tcsv0, r["joint"]) for r in csv.DictReader(open(jp)) if r["stem"] == stem] if os.path.exists(jp) else []
print(f"\n기록 {stem}  구간 {a.t0:g}–{t1:g} s  베이스 {a.base}")
if excl:
    print("  순간 튐: " + ", ".join(f"{e[2] - a.t0:.1f} s(J{e[3]})" for e in excl if a.t0 <= e[2] < t1) + "  ← 화면에서 팔이 툭 튀는 때(구간 시작 기준)")

sim = VibSim(base="fixed" if FIXED else "ranger", verbose=False)
if a.soft != 1.0:
    sim.set_KD(sim.K * a.soft, sim.D * math.sqrt(a.soft))
try:
    from pxr import UsdLux
    UsdLux.DomeLight.Define(sim.stage, "/World/dome").CreateIntensityAttr(800.0)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.9, -1.3, 1.25], target=[0.15, 0.0, 0.95]) if FIXED else set_camera_view(eye=[2.1, -1.5, 1.0], target=[0.4, 0.2, 0.45])
except Exception as e:  # noqa: BLE001
    print("카메라·조명 설정 건너뜀:", e)


def base_v(t):
    if a.base in ("fixed", "still"):
        return 0.0

    def trip(u):
        if u < 0: return 0.0
        if u < 0.5: return u
        if u < 2.0: return 0.5
        if u < 2.5: return 0.5 - (u - 2.0)
        return 0.0

    def raw(u):
        if u < 0: return 0.0
        u = u % 6.0
        return trip(u) - trip(u - 3.0)

    if a.base == "zvd":
        return sum(ai * raw(t - ti) for ai, ti in shaper("zvd", a.sf * math.sqrt(a.soft), 0.10))
    return raw(t)


def run(first):
    if not FIXED:
        sim.world.reset(); sim._apply_gains()
        if a.soft != 1.0:
            sim.set_KD(sim.K, sim.D)
    kw = (lambda vc: dict(wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))) if not FIXED else (lambda vc: {})
    sim.set_state(Q[0])
    n = len(tt); G = np.zeros((n, 3)); tw = time.time()
    for i in range(-1500, n):
        j = max(i, 0)
        sim.step(Q[j], V[j] if i >= 0 else np.zeros(6), **kw(base_v(i * DT - 0.3) if i >= 0 else 0.0))
        if i >= 0:
            G[i] = sim.ee_gyro()
        if i % 20 == 0:
            sim.world.render()
            lag = (i + 1500) * DT * a.slow - (time.time() - tw)
            if lag > 0:
                time.sleep(lag)
            if not app.is_running():
                raise KeyboardInterrupt
        if first and a.shot and i == n // 2:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture_viewport_to_file(get_active_viewport(), a.shot)
            for _ in range(10):
                app.update()
            print("화면 저장:", a.shot)
    keep = np.ones(n, bool)
    for e in excl:
        keep &= ~((tt >= e[0]) & (tt < e[1]))
    r5 = np.sqrt(np.mean(np.stack([band(G[:, c], sim.hz, 5.0, 10.0) for c in range(3)], axis=1)[keep] ** 2, axis=0))
    r2 = np.sqrt(np.mean(np.stack([band(G[:, c], sim.hz, 2.0, 20.0) for c in range(3)], axis=1)[keep] ** 2, axis=0))
    return r5, r2, (time.time() - tw) / ((n + 1500) * DT)


kk = 0
try:
    while app.is_running() and (a.loops == 0 or kk < a.loops):
        r5, r2, rate = run(kk == 0); kk += 1
        print(f"[{kk}] 끝단 진동[rad/s] (튐 구간 제외): 세 축 5–10 Hz {np.sqrt(np.sum(r5 ** 2)):.4f} | 세 축 2–20 Hz {np.sqrt(np.sum(r2 ** 2)):.4f} "
              f"(비틀림 {r2[0]:.4f}, 끄덕임 {r2[1]:.4f}, 좌우 {r2[2]:.4f}) | 재생 속도 실제의 1/{rate:.1f}", flush=True)
except KeyboardInterrupt:
    pass
sim.close()
