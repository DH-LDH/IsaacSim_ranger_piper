"""4단계(1차): 모터 쪽 동역학 — MIT 모드 펌웨어 PD로 서 있는 관절.

모터 쪽(관절마다):  Jm * ddq_m = Kp (q_d - q_m) - B * dq_m + tau_ff - tau_spring
  tau_spring = K (q_m - q_l) + D (dq_m - dq_l)   ← 링크가 모터 쪽을 미는 힘(중력, 관성력, 베이스 가감속)이 여기로 들어온다
값(J2, 11/19 MIT 펌웨어 PD 기록 d1b5 pd에서):
  Kp = 159 N·m/rad   처짐으로 낸 위치 강성 51–80(토크 명령 단위)/rad × 2.44. 범위 124–195
  B  = 5.1 N·m·s/rad 등속 10 °/s에서 오차가 ±0.32° 달라지고 멈추면 0.05 s 안에 풀림 → 속도에 비례하는 항
  2.44 N·m = 토크 명령 1: 오차 0으로 서 있을 때 명령 3.20 = 모형 중력 토크 7.8 N·m. (SDK eff x 4와도 맞음: 위고 계수 0.25)
  Jm = 0.05 kg·m^2   가정(기록의 풀림이 1차 꼴이라 값이 잘 안 잡힌다)
J1·J3에도 같은 값을 쓴다(같은 급 모터라는 가정, 실측 없음).

실험
  --exp fit  : 고정 베이스, J2 10 °/s x 10° 왕복 — 오차의 모양과 정지 후 끝단 진동을 실측 pd 기록과 댄다
  --exp base : AMR 위에서 자세 A 유지, 베이스 0 → 0.5 m/s → 0 (1 m/s^2). 위치 모드(모터 쪽 고정) 대 MIT 펌웨어 PD
출력: out/exp4_<tag>.csv
"""
import argparse, csv, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, WHEEL_R, VibSim, band, shaper, stop_metrics3

ap = argparse.ArgumentParser()
ap.add_argument("--exp", choices=["fit", "base"], default="fit")
ap.add_argument("--Kp", type=float, default=159.0); ap.add_argument("--B", type=float, default=12.7); ap.add_argument("--Jm", type=float, default=0.05)
ap.add_argument("--Fs", type=float, default=0.5, help="정지 마찰(버티는 토크)[N·m]. 0이면 마찰 없음"); ap.add_argument("--Fc-ratio", type=float, default=0.8, help="운동 마찰 / 정지 마찰")
ap.add_argument("--accel", type=float, default=1.0)
ap.add_argument("--joints", default="1,2,3", help="MIT 펌웨어 PD로 두는 관절(나머지는 모터 쪽 고정)")
ap.add_argument("--K1", type=float, default=0.0, help="J1 구조 강성 바꿔 보기"); ap.add_argument("--sag", default="0,-0.88,-0.18", help="관절별 서 있을 때 오차 qref-q [deg] (J1,J2,J3)")
ap.add_argument("--cases", default="pos,mit,mit+zvd"); ap.add_argument("--sf", type=float, default=0.0)
ap.add_argument("--tag", default="mit"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
FIXED = a.exp == "fit"
sim = VibSim(base="fixed" if FIXED else "ranger")
if a.K1 > 0:
    K_, D_ = sim.K.copy(), sim.D.copy(); D_[0] *= a.K1 / K_[0]; K_[0] = a.K1; sim.set_KD(K_, D_)
os.makedirs(HERE + "/out", exist_ok=True)
JM = [int(x) - 1 for x in a.joints.split(",")]
SAG = np.zeros(6); SAG[:3] = np.radians([float(x) for x in a.sag.split(",")])
kw = (lambda vc: dict(wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))) if not FIXED else (lambda vc: {})


class MitPD:
    """MIT 펌웨어 PD 관절의 모터 쪽. 그 밖의 관절은 모터 쪽을 기준 각도에 고정."""

    def __init__(self, q_ref0, mit):
        self.mit = mit
        self.qm = q_ref0.copy(); self.wm = np.zeros(6); self.tff = np.zeros(6)

    def settle(self, q_ref0, vc=0.0):
        # 모터 쪽을 고정한 채 가라앉혀 받치는 토크를 재고, 서 있을 때 오차(SAG)가 나오도록 tau_ff를 정한다
        sim.set_state(q_ref0)
        for _ in range(2000 if not FIXED else 1500):
            sim.step(q_ref0, np.zeros(6), **kw(0.0))
        hold = sim.K * (q_ref0 - sim.q_link())                  # 스프링이 링크에 주는 토크 = 모터가 받쳐야 하는 토크
        if self.mit:
            self.tff = hold - a.Kp * SAG                        # Kp*SAG + tff = hold
            self.qm = q_ref0.copy(); self.qm[JM] = (q_ref0 - SAG)[JM]
            for _ in range(1500):                               # 처진 자리에서 다시 가라앉힘
                self.step(q_ref0); sim.step(self.qm, self.wm, **kw(0.0))

    def step(self, q_ref, w_ref=None):
        if not self.mit:
            self.wm = (q_ref - self.qm) / sim.dt; self.qm = q_ref.copy(); return
        ql, wl = sim.q_link(), sim.dq_link()
        tsp = sim.K * (self.qm - ql) + sim.D * (self.wm - wl)
        drive = a.Kp * (q_ref - self.qm) - a.B * self.wm + self.tff - tsp
        wm = self.wm.copy()
        for j in JM:                                            # 정지 마찰: 멈춰 있고 미는 힘이 Fs 아래면 그대로 붙어 있는다
            stuck = abs(wm[j]) < 1e-4 and abs(drive[j]) <= a.Fs
            if stuck:
                wm[j] = 0.0; continue
            fr = a.Fs * a.Fc_ratio * (np.sign(wm[j]) if abs(wm[j]) >= 1e-4 else np.sign(drive[j]))
            w_new = wm[j] + (drive[j] - fr) / a.Jm * sim.dt
            if wm[j] != 0.0 and np.sign(w_new) != np.sign(wm[j]) and abs(drive[j]) <= a.Fs:
                w_new = 0.0                                     # 속도가 0을 지나갈 때 미는 힘이 약하면 붙는다
            wm[j] = w_new
        qm = self.qm + wm * sim.dt
        new_q = q_ref.copy(); new_w = np.zeros(6)
        new_q[JM] = qm[JM]; new_w[JM] = wm[JM]
        self.qm, self.wm = new_q, new_w


def exp_fit():
    A = POSE_A.copy(); S = A.copy(); S[1] = A[1] - math.radians(10.0); v = math.radians(10.0)
    m = MitPD(S, True); m.settle(S)
    seq = [("ramp", 1.0, S, A), ("hold", 3.0, A, A), ("back", 1.0, A, S), ("bhold", 3.0, S, S)] * 3
    T = sum(x[1] for x in seq); n = int(T * sim.hz); t = np.arange(n) * sim.dt
    G = np.zeros((n, 3)); E = np.zeros(n); seg = []; k = 0; tk = 0.0
    for i in range(n):
        while t[i] - tk >= seq[k][1] - 1e-9 and k < len(seq) - 1:
            tk += seq[k][1]; k += 1
        nm, d, qa, qb = seq[k]; u = min(1.0, (t[i] - tk) / d)
        ref = qa + (qb - qa) * u
        m.step(ref); sim.step(m.qm, m.wm)
        G[i] = sim.ee_gyro(); E[i] = math.degrees(ref[1] - m.qm[1]); seg.append(nm)
    seg = np.array(seg); b = [0] + [i for i in range(1, n) if seg[i] != seg[i - 1]] + [n]
    out = {}
    for i0, i1 in zip(b[:-1], b[1:]):
        s = seg[i0]
        if s in ("ramp", "back"): out.setdefault(s + "_mid", []).append(np.median(E[i0 + (i1 - i0) // 2:i1 - 50]))
        else:
            end = np.median(E[i1 - 800:i1]); out.setdefault(s + "_end", []).append(end)
            d_ = np.abs(E[i0:i1] - end); kk = np.where(d_ > 0.05)[0]; out.setdefault(s + "_settle", []).append(t[i0 + kk[-1]] - t[i0] if len(kk) else 0.0)
            if s == "hold":
                out.setdefault("stop_pitch", []).append(stop_metrics3(t, G, t[i0] + 0.05, fs=sim.hz)["pitch_rms"])
    row = {k_: float(np.median(v_)) for k_, v_ in out.items()}
    row.update(Kp=a.Kp, B=a.B, Jm=a.Jm, Fs=a.Fs, stop_pitch_cam=row["stop_pitch"] * 1.27)
    X = np.abs(np.fft.rfft(band(G[:, 1], sim.hz, 2.0, 20.0))); row["f_peak"] = float(np.fft.rfftfreq(n, sim.dt)[np.argmax(X)])
    print("실측(d1b5 pd): ramp_mid -0.563, hold_end -0.879, back_mid -1.283, bhold_end -0.953, 풀림 0.05 s 안, 정지 후 끝단 0.52 x 위치 모드(약 0.031 rad/s, 카메라)")
    return [row]


def exp_base():
    rows = []
    for case in a.cases.split(","):
        mit = case.startswith("mit"); zvd = case.endswith("zvd")
        sim.world.reset(); sim._apply_gains()
        m = MitPD(POSE_A, mit); m.settle(POSE_A)
        sf = a.sf if a.sf > 0 else exp_base.f.get(mit, 4.125)
        imp = shaper("zvd", sf, 0.10) if zvd else [(1.0, 0.0)]
        Ta, cr = 0.5, 1.5; Dsh = imp[-1][1]; t_end = 2 * Ta + cr + Dsh; T = t_end + 3.0; ACC = a.accel

        def trap(tt):
            if tt < 0: return 0.0
            if tt < Ta: return ACC * tt
            if tt < Ta + cr: return ACC * Ta
            if tt < 2 * Ta + cr: return ACC * (Ta - (tt - Ta - cr))
            return 0.0
        n = int(T * sim.hz); t = np.arange(n) * sim.dt; G = np.zeros((n, 3)); DJ = np.zeros((n, 3))
        for i in range(n):
            vc = sum(ai * trap(t[i] - ti) for ai, ti in imp)
            m.step(POSE_A); sim.step(m.qm, m.wm, **kw(vc))
            G[i] = sim.ee_gyro(); DJ[i] = np.degrees((m.qm - POSE_A)[:3])
        st = stop_metrics3(t, G, t_end, win=2.5, fs=sim.hz)
        if not zvd:
            exp_base.f[mit] = st["f_dom"]
        acc_win = (t > 0.1) & (t < Ta)
        rows.append(dict(case=case, K1=float(sim.K[0]), Kp=a.Kp, Fs=a.Fs, accel=a.accel, j1_final_offset_deg=float(DJ[-1, 0] - DJ[0, 0]), stop_vec=st["vec_rms"], stop_roll=st["roll_rms"], stop_pitch=st["pitch_w_rms"], stop_yaw=st["yaw_rms"],
                         stop_peak=st["vec_peak"], f=st["f_dom"], axis=st["axis"], sf=sf if zvd else 0.0,
                         j1_motor_dev_deg=float(np.max(np.abs(DJ[:, 0] - DJ[0, 0]))), j1_motor_dev_accel_deg=float(np.median(DJ[acc_win, 0]) - DJ[0, 0])))
        if a.keep:
            np.savez(HERE + f"/out/exp4_ts_{a.tag}_{case}.npz", t=t, gyro=G, dj=DJ, t_end=t_end)
    return rows


exp_base.f = {}
tw = time.time()
rows = exp_fit() if a.exp == "fit" else exp_base()
for r in rows:
    print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
with open(HERE + f"/out/exp4_{a.tag}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print("wall %.1f s" % (time.time() - tw))
sim.close()
