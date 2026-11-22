"""2·3단계: Ranger Mini 위 PiPER — 직선 출발–등속–정지에서 끝단 진동, 명령 단계 대책 비교.

팔은 위치 모드로 자세 A를 유지한다. AMR은 속도 명령을 따라간다(가속도 acc, 가속 시간 Ta → 등속 v = acc·Ta).
조건(--cases):
  none      : 대책 없음(사다리꼴 속도 명령 그대로)
  zv / zvd  : AMR 속도 명령에 입력 성형(팔 모드 주파수 --sf 기준)
  scurve    : 가감속 구간을 코사인으로 다듬음(구간 시간은 Ta 그대로)
  armff     : AMR 명령은 그대로. 팔 관절 명령에 베이스 가속도 피드포워드 — 관성력이 만드는 처짐만큼 모터 쪽을 미리 반대로 민다
              dq_m = K^-1 M_jb a_cmd(t + 서보 지연).  M_jb는 질량 행렬의 (관절, 베이스 x 병진) 결합 항.
출력: out/exp2_<tag>.csv, out/exp2_ts_<tag>_<case>_a<acc>_T<Ta>.npz
"""
import argparse, csv, math, os, time
import numpy as np
from vibsim import HERE, POSE_A, POSE_B, WHEEL_R, PosServo, VibSim, band, shaper, stop_metrics3

ap = argparse.ArgumentParser()
ap.add_argument("--accels", default="1.0"); ap.add_argument("--Tas", default="0.5", help="가속 시간[s] 목록")
ap.add_argument("--cruise", type=float, default=1.5); ap.add_argument("--hold", type=float, default=2.3)
ap.add_argument("--cases", default="none,zv,zvd,scurve,armff"); ap.add_argument("--sf", type=float, default=0.0, help="성형기 주파수[Hz]. 0이면 같은 조건의 대책 없음(none)에서 가장 크게 울린 주파수를 쓴다(none을 먼저 돌려야 함)"); ap.add_argument("--sz", type=float, default=0.10)
ap.add_argument("--K-scale", type=float, default=1.0, help="실제 관절 강성 배율 — 성형기·피드포워드는 원래 값 그대로 써서 어긋남 시험")
ap.add_argument("--ff-lead", type=float, default=0.067, help="armff 미리보기 시간[s] = 위치 서보 지연")
ap.add_argument("--ff-gain", type=float, default=1.0)
ap.add_argument("--tire-k", type=float, default=0.0, help="타이어 접촉 강성[N/m] (0 = 강체)"); ap.add_argument("--tire-c", type=float, default=0.0)
ap.add_argument("--susp-k", type=float, default=0.0, help="바퀴당 서스펜션 강성[N/m] (0 = 없음)"); ap.add_argument("--susp-c", type=float, default=0.0)
ap.add_argument("--mount-yaw", type=float, default=90.0, help="팔 장착 각[deg]. 90 = 실물(팔 앞이 AMR 옆), 0 = 팔 앞이 진행 방향")
ap.add_argument("--K1", type=float, default=0.0, help="J1(좌우) 강성을 바꿔 보기[N·m/rad]")
ap.add_argument("--sf2", type=float, default=0.0, help="zvd2(두 모드 성형)의 둘째 주파수[Hz]")
ap.add_argument("--motion", choices=["straight", "crab", "spin"], default="straight", help="straight 직진 / crab 게걸음(바퀴를 90° 돌려 옆으로) / spin 제자리 회전(--accels가 각가속도 rad/s^2)")
ap.add_argument("--pose", default="A", help="팔 자세 A / B / home(접음) / up(세움)")
ap.add_argument("--tag", default="base"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()

POSES = {"A": POSE_A, "B": POSE_B, "home": np.radians([0.0, 20.0, -20.0, 0.0, 30.0, 0.0]), "up": np.radians([0.0, 90.0, -90.0, 0.0, 0.0, 0.0])}
POSE_A = POSES[a.pose]                          # 이 파일에서 쓰는 유지 자세
RS = math.hypot(0.494 / 2, 0.364 / 2)           # 바퀴까지의 반지름(제자리 회전)
SPIN_STEER = np.radians([-53.6, 53.6, 53.6, -53.6]); SPIN_SIGN = np.array([-1.0, 1.0, -1.0, 1.0])   # fl, fr, rl, rr
def wheel_cmd(v):
    """속도 명령 v를 바퀴 속도·조향각으로. 직진: v [m/s], 게걸음: 옆 방향 v, 제자리 회전: v = 각속도 [rad/s]"""
    if a.motion == "crab":
        return np.full(4, v / WHEEL_R), np.full(4, math.pi / 2)
    if a.motion == "spin":
        return SPIN_SIGN * v * RS / WHEEL_R, SPIN_STEER
    return np.full(4, v / WHEEL_R), np.zeros(4)
sim = VibSim(base="ranger", tire_k=a.tire_k, tire_c=a.tire_c, susp_k=a.susp_k, susp_c=a.susp_c, mount_yaw=a.mount_yaw)
if a.K1 > 0:
    K_, D_ = sim.K.copy(), sim.D.copy(); D_[0] *= a.K1 / K_[0]; K_[0] = a.K1; sim.set_KD(K_, D_)
SF = {"f": a.sf}
K_nom = sim.K.copy()
if a.K_scale != 1.0:
    sim.set_KD(sim.K * a.K_scale, sim.D * a.K_scale)
os.makedirs(HERE + "/out", exist_ok=True)

# 결합 항 M_jb: 떠 있는 베이스 질량 행렬 (6 + n) x (6 + n), 앞 6개가 베이스(병진 3, 회전 3)
sim.set_state(POSE_A)
for _ in range(1500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=wheel_cmd(0.0)[1])
M = sim.mass_matrix()
off = M.shape[0] - sim.n
Mjb = M[off + sim.ia, 0] if off == 6 else np.zeros(6)
FF = Mjb / K_nom           # 베이스 가속도 1 m/s^2당 모터 쪽 보정각[rad]
print("M shape", M.shape, "M_jb(x)", np.round(Mjb, 4), "FF[deg per m/s2]", np.round(np.degrees(FF), 4), flush=True)


def vel_profile(t, acc, Ta, cruise, case):
    v = acc * Ta

    def trap(tt):
        if tt < 0: return 0.0
        if tt < Ta: return acc * tt
        if tt < Ta + cruise: return v
        if tt < 2 * Ta + cruise: return v - acc * (tt - Ta - cruise)
        return 0.0

    case = case.split("+")[0]
    if case == "zvd2":                                   # 두 모드를 함께 겨냥: 두 성형기를 이어 붙임
        return sum(ai * bj * trap(t - ti - tj) for ai, ti in shaper("zvd", SF['f'], a.sz) for bj, tj in shaper("zvd", a.sf2, a.sz))
    if case in ("zv", "zvd"):
        return sum(ai * trap(t - ti) for ai, ti in shaper(case, SF['f'], a.sz))
    if case == "scurve":
        if t < 0: return 0.0
        if t < Ta: return v * (1 - math.cos(math.pi * t / Ta)) / 2
        if t < Ta + cruise: return v
        if t < 2 * Ta + cruise: return v * (1 + math.cos(math.pi * (t - Ta - cruise) / Ta)) / 2
        return 0.0
    return trap(t)


def run(case, acc, Ta):
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
    sv = PosServo(POSE_A, sim.dt)
    for _ in range(4000 if a.susp_k > 0 else 2000):   # 내려앉아 가라앉을 때까지
        q, dq = sv.step(POSE_A); sim.step(q, dq, wheel_w=np.zeros(4), steer=wheel_cmd(0.0)[1])
    bcase = case.split("+")[0] if "+" in case else ("none" if case == "armff" else case)   # AMR 속도 명령의 모양
    ff = case.endswith("armff")
    Dsh = shaper(bcase, SF['f'], a.sz)[-1][1] if bcase in ("zv", "zvd") else 0.0
    if bcase == "zvd2":
        Dsh = shaper("zvd", SF['f'], a.sz)[-1][1] + shaper("zvd", a.sf2, a.sz)[-1][1]
    t_go_end = Ta + Dsh; t_stop_end = 2 * Ta + a.cruise + Dsh; T = t_stop_end + a.hold
    n = int(T * sim.hz); t = np.arange(n) * sim.dt; h = 1e-3
    gy = np.zeros((n, 3)); vb = np.zeros(n); pb = np.zeros(n); vc = np.zeros(n); defl = np.zeros((n, 3)); qcmd = np.zeros((n, 3))
    for i in range(n):
        vcmd = vel_profile(t[i], acc, Ta, a.cruise, bcase)
        ref = POSE_A.copy()
        if ff:
            tl = t[i] + a.ff_lead
            a_cmd = (vel_profile(tl + h, acc, Ta, a.cruise, bcase) - vel_profile(tl - h, acc, Ta, a.cruise, bcase)) / (2 * h)
            ref = POSE_A + a.ff_gain * FF * a_cmd
        q, dq = sv.step(ref)
        ww_, st_ = wheel_cmd(vcmd); sim.step(q, dq, wheel_w=ww_, steer=st_)
        gy[i] = sim.ee_gyro(); lv, av = sim.base_vel(); vb[i] = lv[0]; pb[i] = av[1]; vc[i] = vcmd
        defl[i] = (sim.q_link() - POSE_A)[[0, 1, 2]]; qcmd[i] = (q - POSE_A)[[0, 1, 2]]
    go = stop_metrics3(t, gy, t_go_end, win=min(2.0, a.cruise), fs=sim.hz); st = stop_metrics3(t, gy, t_stop_end, fs=sim.hz)
    if case == "none" and a.sf <= 0:
        SF["f"] = st["f_dom"]
    ab = np.gradient(vb, sim.dt)
    pw = pb[(t >= t_stop_end - Ta) & (t < t_stop_end + 2)]; pw = pw - pw.mean()
    PX = np.abs(np.fft.rfft(pw * np.hanning(len(pw)), 8 * len(pw))); pf = np.fft.rfftfreq(8 * len(pw), sim.dt); sel = (pf > 1.5) & (pf < 30)
    row = dict(case=case, motion=a.motion, pose=a.pose, accel=acc, Ta=Ta, v=acc * Ta, K_scale=a.K_scale, go_vec=go["vec_rms"], stop_vec=st["vec_rms"], stop_peak=st["vec_peak"],
               stop_roll=st["roll_rms"], stop_pitch=st["pitch_w_rms"], stop_yaw=st["yaw_rms"], stop_pitch_5_10=st["pitch_rms"], f=st["f_dom"], axis=st["axis"], sf=SF["f"], mount_yaw=a.mount_yaw,
               base_acc_peak=float(np.max(np.abs(band(ab, sim.hz, 0, 20)))),
               base_pitch_rms=float(np.sqrt(np.mean(band(pb[(t >= t_stop_end) & (t < t_stop_end + 2)], sim.hz) ** 2))),
               base_pitch_peak=float(np.max(np.abs(pb[t >= t_stop_end - Ta]))), base_pitch_f=float(pf[sel][np.argmax(PX[sel])]),
               susp_k=a.susp_k, move_time=t_stop_end, dist=float(np.trapezoid(vb, t)))
    if a.keep:
        np.savez(HERE + f"/out/exp2_ts_{a.tag}_{case}_a{acc:g}_T{Ta:g}.npz", t=t, gyro=gy, vb=vb, ab=ab, pb=pb, vcmd=vc, defl=defl, qcmd=qcmd,
                 t_go_end=t_go_end, t_stop_end=t_stop_end)
    return row


tw = time.time(); rows = []
for acc in [float(x) for x in a.accels.split(",")]:
    for Ta in [float(x) for x in a.Tas.split(",")]:
        for case in a.cases.split(","):
            rows.append(run(case, acc, Ta))
            print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in rows[-1].items()}, flush=True)
with open(HERE + f"/out/exp2_{a.tag}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print("wall %.1f s" % (time.time() - tw))
sim.close()
