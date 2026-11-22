"""1단계: 고정 베이스 — J2 등속 후 정지(실기 C-1·입력 성형과 같은 동작)를 돌려 실측과 대조한다.
사용: python.sh exp1_fixed.py [--tune] [--Td 0.045 --fs 6 --zs 0.45]
출력: out/exp1_fixed.csv (조건별 값), out/exp1_ts_*.npz (대표 시계열)
"""
import argparse, csv, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, PosServo, VibSim, shaper, stop_metrics

ap = argparse.ArgumentParser()
ap.add_argument("--Td", type=float, default=0.020); ap.add_argument("--fs", type=float, default=10.0); ap.add_argument("--zs", type=float, default=0.5)
ap.add_argument("--tune", action="store_true"); ap.add_argument("--tag", default="")
ap.add_argument("--K-scale", type=float, default=1.0, help="관절 강성 배율 — 성형기(7.0 Hz)와 실제 모드가 어긋난 날을 흉내")
ap.add_argument("--shape-only", action="store_true"); ap.add_argument("--hz", type=int, default=1000)
a = ap.parse_args()
sim = VibSim(base="fixed", hz=a.hz)
if a.K_scale != 1.0:
    sim.set_KD(sim.K * a.K_scale, sim.D * a.K_scale)
PITCH = np.array([0, 1, 1, 0, 1, 0.0])
J = 1  # J2


def trial(speed, dist, shape="none", Td=a.Td, fs=a.fs, zs=a.zs, hold=2.6, keep=False, sf=7.0, sz=0.10):
    A = POSE_A.copy(); S = A.copy(); S[J] = A[J] - math.radians(dist)
    sim.set_state(S); sv = PosServo(S, sim.dt, Td, fs, zs)
    for _ in range(int(1.5 * sim.hz)):          # 출발 자세에서 가라앉힘
        q, dq = sv.step(S); sim.step(q, dq)
    imp = shaper(shape, sf, sz); Tl = abs(dist) / speed; Dsh = imp[-1][1]; T = Tl + Dsh + hold
    n = int(T * sim.hz); t = np.arange(n) * sim.dt
    gy = np.zeros(n); qm = np.zeros(n); vm = np.zeros(n); qr = np.zeros(n)
    for i in range(n):
        ref = S.copy()
        ref[J] = S[J] + sum(ai * (A[J] - S[J]) * min(1.0, max(0.0, (t[i] - ti) / Tl)) for ai, ti in imp)
        q, dq = sv.step(ref); sim.step(q, dq)
        gy[i] = sim.ee_vel()[1][1]; qm[i] = q[J]; vm[i] = dq[J]; qr[i] = ref[J]
    tref = Tl + Dsh
    # 엔코더 정지 시각: 기준이 멈춘 뒤 모터 쪽 속도가 처음 1 °/s 아래로 내려간 때
    k = np.where((t >= tref) & (np.abs(vm) < math.radians(1.0)))[0]
    t0 = t[k[0]] if len(k) else tref
    m = stop_metrics(t, gy, t0, fs=sim.hz)
    m.update(v_enc=float(np.degrees(np.max(np.abs(vm)))), t_stop_ms=float((t0 - tref) * 1000))
    if keep:
        np.savez(HERE + f"/out/exp1_ts_v{speed:g}_d{dist:+g}_{shape}.npz", t=t, gyro=gy, qm=qm, vm=vm, qref=qr, t0=t0)
    return m


os.makedirs(HERE + "/out", exist_ok=True)
t_wall = time.time()
if a.tune:
    for Td in (0.03, 0.045, 0.06):
        for fs in (4.0, 6.0, 9.0, 14.0):
            for zs in (0.35, 0.5, 0.7):
                r = [trial(10, 10, Td=Td, fs=fs, zs=zs), trial(5, 10, Td=Td, fs=fs, zs=zs), trial(15, 10, Td=Td, fs=fs, zs=zs)]
                print("TUNE Td %.3f fs %4.1f zs %.2f | v10: rms %.4f peak %.3f venc %.2f tstop %3.0f f %.2f | v5 rms %.4f venc %.2f | v15 rms %.4f venc %.2f"
                      % (Td, fs, zs, r[0]["rms"], r[0]["peak"], r[0]["v_enc"], r[0]["t_stop_ms"], r[0]["f"], r[1]["rms"], r[1]["v_enc"], r[2]["rms"], r[2]["v_enc"]), flush=True)
else:
    rows = []
    for speed in (() if a.shape_only else (5, 10, 15)):
        for dist in (-10, -5, -2.5, 2.5, 5, 10):
            m = trial(speed, dist, keep=(dist == 10)); rows.append(dict(speed=speed, dist=dist, shape="none", **m))
            print("ROW", rows[-1], flush=True)
    for speed in (10, 15):
        for shape in (("none", "zv", "zvd") if a.shape_only else ("zv", "zvd")):
            m = trial(speed, 10, shape, keep=not a.shape_only); rows.append(dict(speed=speed, dist=10, shape=shape, **m))
            print("ROW", rows[-1], flush=True)
    with open(HERE + f"/out/exp1_fixed{a.tag}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print("wall %.1f s" % (time.time() - t_wall))
sim.close()
