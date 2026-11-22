"""3단계: 팔이 움직이는 동안 베이스도 움직일 때 — 실기 외란 시나리오(1축·2축·3축) × 베이스 조건.

팔 동작(--src)
  ideal  : 실기 시나리오 정의 그대로의 명령을 위치 모드 서보 모형에 넣는다.
           --scn hold(자세 유지) / 1(J2) / 2(J1+J2) / 3(J1+J2+J3), ±5°, 0.25 Hz 사인, 진폭 2 s 램프 (vib_probe --stage slow와 같은 식)
  replay : 실측 엔코더 기록(piper_ws docs/data/raw/*.csv.gz, q1..q6)을 모터 쪽 각도로 그대로 재생한다.
           실물의 모터 쪽 떨림(동작 중 4.5–6.5 Hz 오르내림)이 그대로 들어간다. --files 로 여러 개.
베이스(--bases, 쉼표로 여러 개)
  fixed  : 고정 베이스 장면(이때는 다른 베이스 조건과 섞지 않는다)
  still  : AMR 위, 정지
  move   : AMR이 0 → 0.5 m/s → 0 (1 m/s^2)으로 앞으로 갔다 뒤로 오기를 되풀이(한 번 6 s)
  zvd    : move에 AMR 속도 명령 ZVD 성형(--sf Hz)
출력: out/exp3_<tag>.csv — 동작 구간의 끝단 자이로 세 축 rms(5–10 Hz 벡터 = 실기 |gyro| 지표, 2–20 Hz 벡터와 축별),
      replay에서 기록에 hold 구간이 있으면 정지 창(엔코더 정지부터 2 s)의 끄덕임 5–10 Hz rms도.
"""
import argparse, csv, gzip, io, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, WHEEL_R, PosServo, VibSim, band, shaper, stop_metrics3

ap = argparse.ArgumentParser()
ap.add_argument("--src", choices=["ideal", "replay"], default="ideal")
ap.add_argument("--scns", default="hold,1,2,3"); ap.add_argument("--files", nargs="*", default=[])
ap.add_argument("--bases", default="still,move,zvd")
ap.add_argument("--amp", type=float, default=5.0); ap.add_argument("--hz", type=float, default=0.25); ap.add_argument("--cycles", type=float, default=3.0)
ap.add_argument("--sf", type=float, default=4.125); ap.add_argument("--accel", type=float, default=1.0); ap.add_argument("--Ta", type=float, default=0.5)
ap.add_argument("--K", default="", help="관절 강성 6개(쉼표) — 재식별용. 감쇠는 강성 비례로 1차 모드 감쇠비 0.10")
ap.add_argument("--tag", default="scn"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
bases = a.bases.split(",")
FIXED = bases == ["fixed"]
assert FIXED or "fixed" not in bases, "fixed는 따로 돌린다"
sim = VibSim(base="fixed" if FIXED else "ranger")
if a.K:
    K_ = np.array([float(x) for x in a.K.split(",")]); D_ = K_ * 2 * 0.10 / (2 * math.pi * 7.0); D_[0] = sim.D[0] * K_[0] / sim.K[0]
    sim.set_KD(K_, D_)
os.makedirs(HERE + "/out", exist_ok=True)
SCN = {"hold": [], "1": [1], "2": [0, 1], "3": [0, 1, 2]}
LEAD, TAIL = 1.0, 2.5


def base_v(t, case):
    """앞으로 갔다(2.5 s) 0.5 s 쉬고 뒤로 왔다(2.5 s) 0.5 s 쉬기를 되풀이."""
    if case in ("still", "fixed"):
        return 0.0
    v = a.accel * a.Ta; cr = 1.5

    def trip(tt):
        if tt < 0: return 0.0
        if tt < a.Ta: return a.accel * tt
        if tt < a.Ta + cr: return v
        if tt < 2 * a.Ta + cr: return v - a.accel * (tt - a.Ta - cr)
        return 0.0

    def raw(tt):
        if tt < 0: return 0.0
        k, u = divmod(tt, 6.0)
        return trip(u) - trip(u - 3.0)

    if case == "zvd":
        return sum(ai * raw(t - ti) for ai, ti in shaper("zvd", a.sf, 0.10))
    return raw(t)


def load(path):
    """실측 기록 → (t[s], q[rad] n x 6, hold 시작 시각 목록)"""
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        r = list(csv.DictReader(f))
    t = np.array([float(x["t"]) for x in r]); q = np.radians(np.array([[float(x[f"q{j}"]) for j in range(1, 7)] for x in r]))
    ph = [x["phase"] for x in r]
    holds = [t[i] for i in range(1, len(r)) if ph[i] == "hold" and ph[i - 1] != "hold"]
    return t - t[0], q, [h - t[0] for h in holds]


def run(arm, case, tstop_ms=None):
    """arm: ("ideal", scn) 또는 ("replay", path)"""
    if not FIXED:
        sim.world.reset(); sim._apply_gains()
    kw = (lambda vc: dict(wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))) if not FIXED else (lambda vc: {})
    if arm[0] == "ideal":
        q0 = POSE_A.copy(); sv = PosServo(q0, sim.dt); T = LEAD + a.cycles / a.hz + TAIL; w = 2 * math.pi * a.hz; joints = SCN[arm[1]]
        t0w, t1w = LEAD, LEAD + a.cycles / a.hz; holds = []
    else:
        tr, qr, holds = load(arm[1]); q0 = qr[0].copy(); T = tr[-1]
        tt = np.arange(0, T, sim.dt); Q = np.stack([np.interp(tt, tr, qr[:, j]) for j in range(6)], axis=1)
        V = np.gradient(Q, sim.dt, axis=0); k = np.ones(25) / 25.0; V = np.stack([np.convolve(V[:, j], k, "same") for j in range(6)], axis=1)
        t0w, t1w = 0.5, T - 0.5
    sim.set_state(q0)
    for _ in range(2000 if not FIXED else 1500):
        sim.step(q0, np.zeros(6), **kw(0.0))
    n = int(T * sim.hz); t = np.arange(n) * sim.dt; G = np.zeros((n, 3)); QM = np.zeros((n, 6))
    for i in range(n):
        if arm[0] == "ideal":
            ref = q0.copy(); ts = t[i] - LEAD
            if 0 < ts < a.cycles / a.hz:
                A = math.radians(a.amp) * min(1.0, ts / 2.0)
                for j in joints: ref[j] = q0[j] + A * math.sin(w * ts)
            q, dq = sv.step(ref)
        else:
            q, dq = Q[min(i, len(Q) - 1)], V[min(i, len(V) - 1)]
        sim.step(q, dq, **kw(base_v(t[i] - 0.3, case)))
        G[i] = sim.ee_gyro(); QM[i] = q
    m = (t >= t0w) & (t < t1w)
    B5 = np.stack([band(G[m, k], sim.hz, 5.0, 10.0) for k in range(3)], axis=1)
    mm = stop_metrics3(t, G, t0w, win=t1w - t0w, fs=sim.hz)
    row = dict(src=arm[0], arm=os.path.basename(arm[1]).split(".")[0] if arm[0] == "replay" else arm[1], base=case,
               move_vec_5_10=float(np.sqrt(np.sum(np.mean(B5 ** 2, axis=0)))), move_pitch_5_10=float(np.sqrt(np.mean(B5[:, 1] ** 2))),
               move_vec_2_20=mm["vec_rms"], roll=mm["roll_rms"], pitch=mm["pitch_w_rms"], yaw=mm["yaw_rms"], f=mm["f_dom"], axis=mm["axis"])
    if holds and tstop_ms is not None:   # 정지 창: 엔코더 정지(hold 시작 + 정지 지연)부터 2 s, 끄덕임 5–10 Hz
        vals = [stop_metrics3(t, G, h + tstop_ms / 1000.0, fs=sim.hz)["pitch_rms"] for h in holds if h + tstop_ms / 1000.0 + 2.0 < T]
        row["stop_pitch_5_10"] = float(np.median(vals)) if vals else float("nan"); row["n_stop"] = len(vals)
    if a.keep:
        np.savez(HERE + f"/out/exp3_ts_{a.tag}_{row['arm'][:40]}_{case}.npz", t=t, gyro=G, qm=QM)
    return row


tw = time.time(); rows = []
if a.src == "ideal":
    for scn in a.scns.split(","):
        for case in bases:
            rows.append(run(("ideal", scn), case)); print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in rows[-1].items()}, flush=True)
else:
    idx = {}
    ip = os.path.expanduser("~/piper_ws/docs/data/raw/index.csv")
    if os.path.exists(ip):
        for r in csv.DictReader(open(ip)):
            idx[r["stem"]] = r
    for path in a.files:
        stem = os.path.basename(path).replace(".csv.gz", "").replace(".csv", "")
        meta = idx.get(stem, {})
        ts_ms = float(meta["t_stop_ms"]) if meta.get("t_stop_ms") not in (None, "") else 65.0
        for case in bases:
            row = run(("replay", path), case, ts_ms)
            row.update(real_gyro_rms=meta.get("gyro_rms", ""), cond=meta.get("cond", ""), set=meta.get("set", ""))
            rows.append(row); print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()}, flush=True)
keys = []
for r in rows:
    keys += [k for k in r if k not in keys]
with open(HERE + f"/out/exp3_{a.tag}.csv", "w", newline="") as f:
    w_ = csv.DictWriter(f, fieldnames=keys); w_.writeheader(); w_.writerows(rows)
print("wall %.1f s" % (time.time() - tw))
sim.close()
