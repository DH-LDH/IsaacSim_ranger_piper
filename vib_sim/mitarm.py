"""MIT 모드 팔(관절 1–3)의 모터 쪽 모형 — exp7_mit.py(기록 대조)와 exp6_bump.py(이동 베이스)가 같이 쓴다.

  Jm·q̈m = Kp(qr − qm) + B(vr − q̇m) + G·τ_ff + c0 − τ_스프링 − 마찰        (qr, vr, τ_ff는 Td만큼 늦게 닿는다)

기본값은 11/23 버티는 토크 실측(piper_ws FW191_D_LOG 17절)과 11/19 기록의 회귀(SIM_VIB_KROS 4.4절 둘째 덧붙임):
  Kp 219·257·218 N·m/rad (전류 토크 59.9·67.6·62.4 eff/rad × 배율 3.65·3.8·3.5), B = 0.08·Kp(펌웨어 Kd/Kp, 가정),
  G 3.18·3.23·3.0 N·m per 명령(0.87·0.85·0.85 eff per 명령), c0 0·2.7·0.88 N·m(중력이 걸리는 관절의 절편), Td 10 ms,
  마찰 Fs 1.17·1.67·0.88 N·m(정지), Fc 0.58·1.10·0.60 N·m(운동), Jm 0.05 kg·m²(가정 — 잰 값 없음).
추종층: 실기 코드(piper_ws src/common_pkg/joint_ctrl.py)를 그대로 부른다. 100 Hz, 엔코더 0.001°, 속도 0.001 rad/s,
  토크 한 칸 0.063 명령, 주기당 변화 ±0.3 명령, 상한 J2 ±5·그 밖 ±4 명령 (mit_probe와 같음).
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/piper_ws/src/common_pkg"))
from joint_ctrl import JointCtrl, JointCtrlCfg  # noqa: E402

JM = [0, 1, 2]
RATIO = np.array([3.65, 3.8, 3.5])            # 전류 토크 eff → N·m (J1은 가정)
TAU_LIM = np.array([4.0, 5.0, 4.0, 4.0, 4.0, 4.0]); STEP = 0.063; RATE = 0.3


def add_args(ap):
    ap.add_argument("--Kp", default="219,257,218", help="펌웨어 PD 위치 강성[N·m/rad], J1,J2,J3 (값 하나면 셋 다)"); ap.add_argument("--Bratio", type=float, default=0.08, help="B = Bratio·Kp")
    ap.add_argument("--G", default="3.18,3.23,3.0", help="τ_ff 한 단위[N·m]"); ap.add_argument("--c0", default="0,2.7,0.88", help="τ_ff 눈금의 절편[N·m]"); ap.add_argument("--Jm", type=float, default=0.05)
    ap.add_argument("--Td", type=float, default=10.0, help="명령이 모터에 닿는 지연[ms]")
    ap.add_argument("--Fs", default="1.17,1.67,0.88", help="정지 마찰[N·m]"); ap.add_argument("--Fc", default="0.58,1.10,0.60", help="운동 마찰[N·m]")
    ap.add_argument("--mbar", type=float, default=0.01); ap.add_argument("--eps", type=float, default=0.08)
    ap.add_argument("--qd-noise", type=float, default=0.0, help="추종층에 넘기는 관절 속도의 잡음 표준편차[rad/s] (실기 motor_speed: 정지 0.004–0.014)")


def v3(s_):
    x = [float(v) for v in str(s_).split(",")]; o = np.zeros(6); o[:3] = x * 3 if len(x) == 1 else x; return o


def start_tff(a, hold, ctrl):
    """MIT 진입 때의 τ_ff[명령](6). hold: 스프링이 링크를 받치는 토크[N·m].
    pd: 실기처럼 진입 직전 위치 모드의 전류 토크 평균을 그대로 쓴다(= 받치는 토크 / 배율) — 처짐이 남는다.
    추종층: 오차 0에서 받치는 값 (받치는 토크 − c0) / G"""
    o = np.zeros(6); h = np.asarray(hold, float)[JM]
    o[JM] = h / RATIO if ctrl == "pd" else (h - v3(a.c0)[JM]) / v3(a.G)[JM]
    return o


class MitArm:
    def __init__(self, a, sim, q0, qr0, tff0, ctrl="pd", tdc=(), seed=0):
        """a: add_args의 값. q0: 모터 쪽 시작 각(6). qr0: 시작 기준(6). tff0: 시작 τ_ff[명령](6). ctrl: pd / tde / asmc16. tdc: 추종층을 거는 관절 번호(0부터)"""
        self.a, self.sim, self.dt = a, sim, sim.dt
        self.KP, self.GG, self.C0, self.FS, self.FC = v3(a.Kp), v3(a.G), v3(a.c0), v3(a.Fs), v3(a.Fc); self.BB = a.Bratio * self.KP
        self.rng = np.random.default_rng(seed); self.TJ = [] if ctrl == "pd" else list(tdc)
        self.C = {j: JointCtrl(JointCtrlCfg(mbar=a.mbar, ks=20.0, kg=20.0, acc_fc=15.0, hybrid=True, kp_fw=10.0, kd_fw=0.8, switch="asmc16" if ctrl == "asmc16" else "none",
                                            phi=200.0, alpha=1.0, eps=a.eps, k_max=20.0, sw_phi=0.2, dist_mode="torque"), float(tff0[j])) for j in self.TJ}
        tff0 = np.asarray(tff0, float).copy(); nd = max(1, int(round(a.Td / 1000.0 / self.dt)))
        self.buf = [(np.asarray(qr0, float).copy(), np.zeros(6), tff0.copy())] * nd
        self.qm = np.asarray(q0, float).copy(); self.wm = np.zeros(6); self.cmd = (np.asarray(qr0, float).copy(), np.zeros(6), tff0.copy())
        self.tau_last = tff0.copy(); self.c_tf = tff0.copy(); self.nlim = 0; self.k = 0; self.nslip = np.zeros(3, int)

    def step(self, qr, vr, ar, q_rest, w_rest, tau_add=None):
        """물리 한 스텝(1 ms). qr·vr·ar: 기준 각·속도·가속도(6). q_rest·w_rest: 관절 4–6의 모터 쪽(뒤 셋만 쓴다).
        tau_add: 추종층 출력에 더해 보내는 토크[명령](6) — 변화율 제한·한 칸 끊기 전에 더한다. 반환: 모터 쪽 각, 속도(6)"""
        a, sim, DT = self.a, self.sim, self.dt; qm, wm = self.qm, self.wm
        if self.k % 10 == 0:                                                       # 100 Hz 제어 주기
            qenc = np.round(np.degrees(qm), 3) * math.pi / 180.0; tnew = self.tau_last.copy()
            for j in self.TJ:
                wj = float(round(wm[j] + a.qd_noise * self.rng.standard_normal(), 3))
                o = self.C[j].step(float(qenc[j]), wj, float(qr[j]), float(vr[j]), float(ar[j]), 0.01 if self.k else 0.0)
                raw = float(np.clip(o["tau"], -TAU_LIM[j], TAU_LIM[j]))
                if abs(raw - self.tau_last[j]) > RATE: raw = self.tau_last[j] + math.copysign(RATE, raw - self.tau_last[j]); self.nlim += 1
                raw = round(raw / STEP) * STEP; self.C[j].commit(raw); tnew[j] = raw
            self.tau_last = tnew; send = tnew.copy()
            if tau_add is not None: send = send + np.round(np.asarray(tau_add) / STEP) * STEP
            self.cmd = (np.asarray(qr, float).copy(), np.asarray(vr, float).copy(), send)
        self.buf.append(self.cmd); c_qr, c_vr, c_tf = self.buf.pop(0)              # Td 뒤에 모터에 닿는다
        ql, wl = sim.q_link(), sim.dq_link(); tsp = sim.K * (qm - ql) + sim.D * (wm - wl)
        drive = self.KP * (c_qr - qm) + self.BB * (c_vr - wm) + self.GG * c_tf + self.C0 - tsp
        for j in JM:
            if abs(wm[j]) < 1e-4 and abs(drive[j]) <= self.FS[j]: wm[j] = 0.0; continue
            fr = self.FC[j] * (np.sign(wm[j]) if abs(wm[j]) >= 1e-4 else np.sign(drive[j]))
            w_new = wm[j] + (drive[j] - fr) / a.Jm * DT
            if wm[j] != 0.0 and np.sign(w_new) != np.sign(wm[j]) and abs(drive[j]) <= self.FS[j]: w_new = 0.0
            wm[j] = w_new; self.nslip[j] += 1
        qm[JM] += wm[JM] * DT; qm[3:] = np.asarray(q_rest)[3:]; wm[3:] = np.asarray(w_rest)[3:]
        self.c_tf = c_tf; self.k += 1
        return qm, wm
