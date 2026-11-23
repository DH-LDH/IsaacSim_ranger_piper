"""MIT 모드 팔의 모터 쪽 동역학 + 실기 추종층(piper_ws src/common_pkg/joint_ctrl.py 그대로) — 기록 대조용 틀.

실기 MIT 기록의 기준 궤적(qref)을 그대로 주고, 모터 쪽을 "펌웨어 PD + τ_ff(고정 또는 TDE·ASMC) + 마찰"로 굴린다.
나온 엔코더 오차(qref − q)와 끝단 각속도를 그 기록의 실측과 댄다. 고정 베이스.

모터 쪽(관절 1–3):  Jm·q̈m = Kp(qr − qm) + B(vr − q̇m) + G·τ_ff − τ_스프링 − 마찰      (qr, vr, τ_ff는 Td만큼 늦게 닿는다)
  Kp 159 N·m/rad(124–195), B 12.7 N·m·s/rad, G 2.44 N·m per 명령, Td 32 ms — SIM_VIB_KROS 4.4절의 기록 값
  마찰: 정지 마찰 Fs[N·m](--Fs "J1,J2,J3"), 운동 마찰 = Fs × --Fc-ratio.  ← 버티는 토크 실측이 오면 이 값을 넣는다
추종층: 100 Hz, 엔코더 0.001° 분해능, 토크 한 칸 0.063 명령, 주기당 변화 ±0.3 명령, 상한 J2 ±5·그 밖 ±4 명령 (mit_probe와 같음)
관절 4–6의 모터 쪽은 기록의 엔코더를 그대로 재생한다.

  ~/isaacsim/python.sh exp7_mit.py --file <기록.csv.gz> --ctrl pd|tde|asmc16 [--tdc-joints 2] [--Fs 0.5,0.5,0.5]
출력: ROW 한 줄 + out/exp7_<tag>.csv (+ --keep 이면 시계열 npz)
"""
import argparse, csv, gzip, json, math, os, sys, time
import numpy as np
from vibsim import HERE, VibSim
sys.path.insert(0, os.path.expanduser("~/piper_ws/src/common_pkg"))
from joint_ctrl import JointCtrl, JointCtrlCfg
from scn_seg import seg_rms

ap = argparse.ArgumentParser()
ap.add_argument("--file", required=True); ap.add_argument("--ctrl", default="", help="pd / tde / asmc16 (비우면 meta.json·파일 이름에서)")
ap.add_argument("--tdc-joints", default="2"); ap.add_argument("--mbar", type=float, default=0.01); ap.add_argument("--eps", type=float, default=0.08)
ap.add_argument("--Kp", type=float, default=159.0); ap.add_argument("--B", type=float, default=12.7); ap.add_argument("--G", type=float, default=2.44); ap.add_argument("--Jm", type=float, default=0.05)
ap.add_argument("--Td", type=float, default=32.0, help="명령이 모터에 닿는 지연[ms]")
ap.add_argument("--Fs", default="0.5,0.5,0.5"); ap.add_argument("--Fc-ratio", type=float, default=0.8)
ap.add_argument("--dur", type=float, default=0.0, help="MIT 시작부터 몇 초(0 = 기록 끝까지)")
ap.add_argument("--tag", default="mit7"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
RAW = os.path.expanduser("~/piper_ws/docs/data/raw"); stem = os.path.basename(a.file).replace(".csv.gz", "")
meta = {}
mp = a.file.replace(".csv.gz", ".meta.json")
if os.path.exists(mp): meta = json.load(open(mp))
ctrl = a.ctrl or ("pd" if stem.endswith("_pd") else meta.get("ctrl", "") or ("tde" if "tde" in stem else "asmc16"))
ctrl = {"mit": "pd", "asmc": "asmc16"}.get(ctrl, ctrl)
TJ = [] if ctrl == "pd" else [int(x) - 1 for x in a.tdc_joints.split(",")]
JM = [0, 1, 2]; FS = np.zeros(6); FS[:3] = [float(x) for x in a.Fs.split(",")]

with gzip.open(a.file, "rt") as f: R = list(csv.DictReader(f))
tc = np.array([float(x["t"]) for x in R]); Q = np.radians(np.array([[float(x[f"q{j}"]) for j in range(1, 7)] for x in R])); QR = np.radians(np.array([[float(x[f"qref{j}"]) for j in range(1, 7)] for x in R]))
TF = np.array([[float(x[f"tff{j}"]) for j in range(1, 7)] for x in R])
i_m = int(np.argmax(np.abs(TF[:, 1]) > 1e-6)); t_m = tc[i_m]                 # MIT 시작(τ_ff가 처음 0이 아닌 때)
ph = [x["phase"] for x in R]; t_end = tc[max(i for i, p_ in enumerate(ph) if p_ == "mit")] - 0.3 if "mit" in ph else tc[-1] - 0.5   # 끝의 estop(내려앉음)은 뺀다
T = (t_end - t_m) if a.dur <= 0 else min(a.dur, t_end - t_m)
sim = VibSim(base="fixed"); DT = sim.dt; n = int(T / DT); t = np.arange(n) * DT; ts = t + t_m
qr = np.stack([np.interp(ts, tc, QR[:, j]) for j in range(6)], axis=1); qe = np.stack([np.interp(ts, tc, Q[:, j]) for j in range(6)], axis=1)
k_ = np.ones(21) / 21.0; vr = np.stack([np.convolve(np.gradient(qr[:, j], DT), k_, "same") for j in range(6)], axis=1); ar = np.stack([np.convolve(np.gradient(vr[:, j], DT), k_, "same") for j in range(6)], axis=1)
ve = np.stack([np.convolve(np.gradient(qe[:, j], DT), k_, "same") for j in range(6)], axis=1)
excl = [(float(x["exclude_from"]) - t_m, float(x["exclude_to"]) - t_m) for x in csv.DictReader(open(RAW + "/mit_jumps.csv")) if x["stem"] == stem]

# ---- 가라앉히기: 모터 쪽을 기록의 첫 각도에 두고 받치는 토크를 잰다 ----
q0 = qe[0].copy(); sim.set_state(q0)
for _ in range(1500): sim.step(q0, np.zeros(6))
hold = sim.K * (q0 - sim.q_link())                                             # 스프링이 링크를 받치는 토크[N·m]
sag0 = (qr[0] - qe[0])                                                         # 기록의 시작 오차(처짐)
tff = np.zeros(6); tff[JM] = TF[i_m, JM]                                       # 기록의 τ_ff(명령 단위) 그대로. 처짐은 (받치는 토크 − G·τ_ff) / Kp로 모형에서 나온다
print('받치는 토크[N·m] J1–3', np.round(hold[:3], 2), '| 기록 τ_ff[명령]', np.round(tff[:3], 3), '| G·τ_ff', np.round(a.G * tff[:3], 2), flush=True)
tau_lim = np.array([4.0, 5.0, 4.0, 4.0, 4.0, 4.0]); STEP = 0.063; RATE = 0.3
C = {j: JointCtrl(JointCtrlCfg(mbar=a.mbar, ks=20.0, kg=20.0, acc_fc=15.0, hybrid=True, kp_fw=10.0, kd_fw=0.8, switch="asmc16" if ctrl == "asmc16" else "none",
                               phi=200.0, alpha=1.0, eps=a.eps, k_max=20.0, sw_phi=0.2, dist_mode="torque"), tff[j]) for j in TJ}
nd = max(1, int(round(a.Td / 1000.0 / DT))); buf = [(qr[0].copy(), np.zeros(6), tff.copy())] * nd
qm = q0.copy(); wm = np.zeros(6); cmd = (qr[0].copy(), np.zeros(6), tff.copy()); tau_last = tff.copy()
G_ = np.zeros((n, 3)); QM = np.zeros((n, 6)); TFs = np.zeros((n, 6)); tw = time.time(); nlim = 0
for i in range(n):
    if i % 10 == 0:                                                            # 100 Hz 제어 주기
        qenc = np.round(np.degrees(qm), 3) * math.pi / 180.0; tnew = tau_last.copy()
        for j in TJ:
            o = C[j].step(float(qenc[j]), float(wm[j]), float(qr[i, j]), float(vr[i, j]), float(ar[i, j]), 0.01 if i else 0.0)
            raw = float(np.clip(o["tau"], -tau_lim[j], tau_lim[j]))
            if abs(raw - tau_last[j]) > RATE: raw = tau_last[j] + math.copysign(RATE, raw - tau_last[j]); nlim += 1
            raw = round(raw / STEP) * STEP; C[j].commit(raw); tnew[j] = raw
        tau_last = tnew; cmd = (qr[i].copy(), vr[i].copy(), tnew.copy())
    buf.append(cmd); c_qr, c_vr, c_tf = buf.pop(0)                             # Td 뒤에 모터에 닿는다
    ql, wl = sim.q_link(), sim.dq_link(); tsp = sim.K * (qm - ql) + sim.D * (wm - wl)
    drive = a.Kp * (c_qr - qm) + a.B * (c_vr - wm) + a.G * c_tf - tsp
    for j in JM:
        if abs(wm[j]) < 1e-4 and abs(drive[j]) <= FS[j]: wm[j] = 0.0; continue
        fr = FS[j] * a.Fc_ratio * (np.sign(wm[j]) if abs(wm[j]) >= 1e-4 else np.sign(drive[j]))
        w_new = wm[j] + (drive[j] - fr) / a.Jm * DT
        if wm[j] != 0.0 and np.sign(w_new) != np.sign(wm[j]) and abs(drive[j]) <= FS[j]: w_new = 0.0
        wm[j] = w_new
    qm[JM] += wm[JM] * DT; qm[3:] = qe[i, 3:]; wm[3:] = ve[i, 3:]
    sim.step(qm, wm); G_[i] = sim.ee_gyro(); QM[i] = qm; TFs[i] = c_tf
keep = np.ones(n, bool)
for e0, e1 in excl: keep &= ~((t >= e0) & (t < e1))
row = dict(stem=stem[-44:], ctrl=ctrl, tdc="".join(str(j + 1) for j in TJ), Fs=a.Fs, Kp=a.Kp, B=a.B, Jm=a.Jm, Td=a.Td, T=round(T, 1), n_jump=len(excl), n_ratelim=nlim)
for j in JM:
    es = np.degrees(qr[:, j] - QM[:, j])[keep]; er = np.degrees(qr[:, j] - qe[:, j])[keep]
    row[f"J{j + 1}_err_sim"] = float(np.sqrt(np.mean((es - es.mean()) ** 2))); row[f"J{j + 1}_err_real"] = float(np.sqrt(np.mean((er - er.mean()) ** 2)))
    row[f"J{j + 1}_sag_sim"] = float(es.mean()); row[f"J{j + 1}_sag_real"] = float(er.mean())
    row[f"J{j + 1}_corr"] = float(np.corrcoef(es, er)[0, 1]) if es.std() > 0 and er.std() > 0 else float("nan")
# 끝단 각속도: 조각 방식(튐 제외), 재생 값 × 1.27 대 실측(카메라)
ss, ns = seg_rms(t, G_, 1000.0, 2.0, T - 0.5, excl)
tz = float(open(a.file.replace(".csv.gz", ".t0")).read())
with gzip.open(a.file.replace(".csv.gz", ".imu.csv.gz"), "rt") as f:
    g = np.array([(float(x["t"]) - tz - t_m, float(x["x"]), float(x["y"]), float(x["z"])) for x in csv.DictReader(f) if x["stream"] == "g"])
tu = np.arange(max(0.0, g[0, 0]) + 0.05, min(T, g[-1, 0]) - 0.05, 1 / 400.0); Gr = np.stack([np.interp(tu, g[:, 0], g[:, c]) for c in (1, 2, 3)], axis=1)
rs, _ = seg_rms(tu, Gr, 400.0, 2.0, T - 0.5, excl)
for k in ("2_10", "5_10", "10_20"):
    row[f"gyro_{k}_sim"] = ss.get(k, float("nan")) * 1.27; row[f"gyro_{k}_real"] = rs.get(k, float("nan")); row[f"gyro_{k}_ratio"] = row[f"gyro_{k}_sim"] / row[f"gyro_{k}_real"] if rs.get(k) else float("nan")
print("ROW", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()}, flush=True)
os.makedirs(HERE + "/out", exist_ok=True); p = HERE + f"/out/exp7_{a.tag}.csv"; new = not os.path.exists(p)
with open(p, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(row.keys()))
    if new: w.writeheader()
    w.writerow(row)
if a.keep: np.savez(HERE + f"/out/exp7_ts_{a.tag}_{ctrl}.npz", t=t, gyro=G_, qm=QM, qr=qr, qe=qe, tff=TFs)
print("wall %.1f s" % (time.time() - tw)); sim.close()
