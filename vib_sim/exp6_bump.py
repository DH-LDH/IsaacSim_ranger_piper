"""6단계: exp5_bump.py에 조건을 넓히는 옵션을 더한 것(밤사이 조건 격자용). 옵션 추가: --yaw(팔 장착 방향), --blen(턱 길이), --side(한쪽 바퀴만),
--pose(자세 A/B), --dq-step·--dq-dead(보정각의 분해능·불감대 — 모터 쪽이 작은 보정에는 안 움직이는 경우). 나머지는 exp5와 같다.

5단계(예비): 턱 넘기 — 미리 모르는 흔들림을 얼마나 잡을 수 있는가.

AMR이 등속으로 가다가 바닥의 턱(높이 --h, 길이 3 cm, 앞바퀴 둘 → 뒷바퀴 둘 순서로 넘음)을 넘는다. 팔은 자세 A 유지(실물 장착 방향).
모터 쪽이 보정 명령을 그대로 따라간다고 본 이상적인 경우다(ffservo만 위치 모드 서보 모형을 거침).

조건은 + 로 이어 붙인다(--cases, 쉼표로 여러 개):
  none
  ffN       베이스 IMU 피드포워드, 지연 N ms.  dq_m = K^-1 C a_base(t - N)   (C = 질량 행렬의 관절 x 베이스 6자유도 결합 항)
  ffservo   같은 피드포워드를 위치 모드 서보(센서 10 ms + 지연 20 ms + 10 Hz 2차)로
  prev      뒷바퀴 예측: 앞바퀴가 넘을 때 잰 베이스 가속도를 기억해 두었다가, 뒷바퀴가 넘을 시각(축간 거리 / 속도 뒤)에 지연 없이 미리 넣는다.
            뒷바퀴가 넘을 때는 차체 끄덕임의 부호가 반대라 그 성분만 뒤집는다.  prevE10 = 예측 시각이 10 ms 늦음(E-10 = 10 ms 이름)
  map       지도 기반 예견(앞을 보는 센서): 같은 턱을 미리 한 번 넘어 잰 베이스 가속도를 "위치의 함수"로 기억해 두고, 그 위치에서 지연 없이 넣는다.
            mapE3 = 턱 위치를 3 mm 뒤로 잘못 앎(E-3은 앞으로), mapS70 = 턱 크기를 잘못 알아 파형을 0.7배로 넣음, mapservo = 위치 모드 서보로(서보 지연만큼 앞당겨 넣음)
            prevz = 상하·끄덕임 성분만 예측하고 앞뒤 방향 가속도 등 나머지는 잰 값(ffN과 같이 씀). prevzE10 = 10 ms 늦게
  dampN     끝단 자이로 되먹임 감쇠, 지연 N ms.  끄덕임 → J5, 좌우 → J1 모터 쪽을 각속도 반대 방향으로 민다(감쇠비 +0.3 상당)
  dampJ     실기처럼: 100 Hz로 계산, 표본 나이가 매번 다름(중앙 22.5 ms, 95% 48.7 ms), 나이를 알고 그만큼 위상을 앞당김.
            dampJn = 앞당기지 않음, dampJs = 나이가 40 ms를 넘는 주기는 되먹임을 끔, dampJg = 나이에 반비례해 세기를 줄임
  dampRN    감쇠만 더하기: 끝단 각속도를 모드 주파수의 공진기(대역 통과, --Q)에 넣고 그 상태로 위상을 당김(지연 N ms). 모드보다 빠른 성분을 건드리지 않는다.
            dampRJ = 실기처럼 100 Hz·들쭉날쭉한 지연
  dampNp    같은 되먹임에 위상 예측: 울림이 한 주파수(끄덕임 7.5 Hz, 좌우 4.1 Hz)라고 보고 N ms 뒤의 값을 계산해 쓴다
--mit pd|tde|asmc16: 관절 1–3의 모터 쪽을 엔코더 재생·유지 대신 MIT 모드 모형(mitarm.py: 펌웨어 PD + τ_ff + 마찰, 추종층은 실기 joint_ctrl)으로 굴린다.
            기준은 --replay 기록의 qref(없으면 자세 유지), 관절 4–6은 그대로 재생. 이때 미리 넣는 보정(ff·prev·map)은 위치 기준에 더해 보내고,
            끝단 되먹임(damp)은 링크에 바로 닿는 토크로 넣는다(스프링 목표를 τ/K만큼 옮김 — 실기 처프에서 엔코더는 안 움직이는데 끝단이 응답한 것에 맞춘 꼴).
            --pre 초만큼은 AMR을 세워 두고 기준을 따라가며 자리를 잡는다(지표에서 뺌).
출력: out/exp5_<tag>.csv — 끝단 자이로 세 축 2–20 Hz rms: 전체 / 앞바퀴 구간 / 뒷바퀴 구간.
타이어 탄성은 접촉 강성(--tire-k, 기본 150 kN/m)으로 넣는다. 실물 타이어 값은 모른다 — 절댓값이 아니라 조건끼리의 비로 본다.
"""
import argparse, csv, math, os, sys, time
if "--gui" in sys.argv:
    os.environ.setdefault("VIBSIM_GUI", "1")   # 화면으로 보기: vibsim을 들이기 전에 켠다
import numpy as np
from vibsim import HERE, POSE_A, POSE_B, WHEEL_R, PosServo, VibSim, app, band, shaper   # vibsim이 먼저 Isaac을 띄운다
from pxr import Gf, UsdGeom, UsdPhysics
import mitarm
from mitarm import MitArm

ap = argparse.ArgumentParser()
ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--h", type=float, default=0.01)
ap.add_argument("--cases", default="none,ff0,ff5,ff10,ff20,ff40,ffservo")
ap.add_argument("--susp-k", type=float, default=0.0); ap.add_argument("--susp-c", type=float, default=0.0)
ap.add_argument("--tire-k", type=float, default=150000.0, help="타이어(접촉) 강성[N/m], 0이면 강체 바퀴"); ap.add_argument("--tire-c", type=float, default=1000.0)
ap.add_argument("--lp", type=float, default=40.0, help="IMU 저역 통과[Hz]"); ap.add_argument("--zadd", type=float, default=0.3, help="되먹임으로 더하려는 감쇠비")
ap.add_argument("--fp", type=float, default=7.5); ap.add_argument("--fy", type=float, default=4.1)
ap.add_argument("--mount-x", type=float, default=-0.23, help="팔 장착 위치를 앞뒤로 옮김[m]. -0.20이면 뒷바퀴 축(-0.247) 가까이")
ap.add_argument("--pothole", type=float, default=0.0, help="0보다 크면 턱 대신 패인 곳: 길이[m] (깊이는 --h)")
ap.add_argument("--yaw", type=float, default=90.0, help="팔 장착 방향[deg]: 90 = 옆(실물), 0 = 앞, 180 = 뒤")
ap.add_argument("--blen", type=float, default=0.03, help="턱 길이[m]"); ap.add_argument("--side", type=int, default=0, help="1이면 왼쪽 바퀴만 넘는 턱")
ap.add_argument("--pose", default="A"); ap.add_argument("--dq-step", type=float, default=0.0, help="보정각 분해능[deg]"); ap.add_argument("--dq-dead", type=float, default=0.0, help="보정각 불감대[deg]")
ap.add_argument("--lib-from", default="", help="지도 예견에 쓸 베이스 가속도 지도를 다른 실행(태그)에서 가져온다 — 다른 크기·속도의 턱에서 만든 지도를 쓰는 경우")
ap.add_argument("--gui", action="store_true", help="Isaac Sim 창을 띄워 실시간으로 본다"); ap.add_argument("--loops", type=int, default=1, help="조건 목록을 몇 번 되풀이할지(0 = 창을 닫을 때까지)"); ap.add_argument("--slow", type=float, default=1.0)
ap.add_argument("--stop", action="store_true", help="종합 시나리오: 출발 → 턱 → 0.9 m 더 가서 정지(1 m/s^2). 지표는 전체 구간")
ap.add_argument("--fbp", type=int, default=5, help="끄덕임 되먹임을 넣는 관절: 5(기본) 또는 2(실기 감쇠층의 통로). 부호를 뒤집으려면 음수")
ap.add_argument("--tau-clip", type=float, default=0.0, help="되먹임 보정의 토크 한계[N·m] — 관절 스프링을 거치는 토크 K_j·|Δq| 를 이 값으로 자른다. 0 = 없음")
ap.add_argument("--tau-step", type=float, default=0.0, help="모터 토크 한 칸[N·m] (실기 0.15). 0 = 없음"); ap.add_argument("--tau-share", type=float, default=1.0, help="모터 토크 가운데 관절 스프링으로 가는 몫 K_j/(K_j+K_PD) (실기 J2 0.82–0.88)")
ap.add_argument("--kick-old", action="store_true", help="충격 꼴을 11/23 오전 것(20 ms·80 ms, 다 돌아옴)으로")
ap.add_argument("--kick", default="", help="충격성 외란(MIT 순간 튐 흉내): \"관절,도,시각;...\" — 모터 쪽이 20 ms에 튀고 80 ms 시정수로 돌아온다")
ap.add_argument("--tail", type=float, default=2.0, help="종합 시나리오에서 정지한 뒤 더 재는 시간[s]")
ap.add_argument("--still", action="store_true", help="AMR을 세워 둔다(팔만 움직이는 기준)")
ap.add_argument("--bshape", default="none", help="AMR 속도 명령 성형을 모든 조건에: none / zvd (조건 토큰 bz로 조건마다 켤 수도 있다)"); ap.add_argument("--bsf", type=float, default=4.125)
ap.add_argument("--rigid", action="store_true", help="팔을 강체로(관절 강성 100배) — 증폭이 없을 때의 전달률 기준")
ap.add_argument("--Q", type=float, default=1.5, help="dampR 공진기의 Q(클수록 좁은 대역)")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--replay", default="", help="팔을 유지하는 대신 실측 엔코더 기록(csv.gz)을 재생하면서 넘는다"); ap.add_argument("--replay-t0", type=float, default=8.0)
ap.add_argument("--lib-scale", type=float, default=1.0, help="가져온 지도에 곱할 배율(턱 높이 비 등)")
ap.add_argument("--mit", default="", help="관절 1–3을 MIT 모드 모형으로: pd / tde / asmc16 (비우면 엔코더 재생·유지)"); ap.add_argument("--mit-tdc", default="2", help="추종층을 거는 관절")
ap.add_argument("--pre", type=float, default=0.0, help="재기 전에 AMR을 세워 둔 채 기준을 따라가는 시간[s] (--replay면 기록의 그만큼 앞에서 시작)"); mitarm.add_args(ap)
ap.add_argument("--tag", default="bump"); ap.add_argument("--keep", action="store_true")
a = ap.parse_args()
ZP = 0.04 if a.pothole > 0 else 0.0           # 패인 곳은 바닥을 뚫을 수 없어 4 cm 높이의 단 위를 달리게 하고 단에 홈을 낸다
sim = VibSim(base="ranger", susp_k=a.susp_k, susp_c=a.susp_c, tire_k=a.tire_k, tire_c=a.tire_c, mount_x=a.mount_x, z0=ZP, mount_yaw=a.yaw)
POSE_A = POSE_A if a.pose.upper() == "A" else POSE_B     # 이 파일 안에서 쓰는 유지 자세
if a.rigid:
    sim.set_KD(sim.K * 100.0, sim.D * 100.0)
os.makedirs(HERE + "/out", exist_ok=True)
XB, LWB = 0.75, 0.494                        # 턱 위치[m], 축간 거리[m]
def box(name, x0, x1, top, y0=-1.0, y1=1.0):
    c = UsdGeom.Cube.Define(sim.stage, "/World/" + name); c.CreateSizeAttr(1.0)
    c.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.25, 0.1) if name in ("bump", "deck_hole") else Gf.Vec3f(0.55, 0.6, 0.65)])
    xf = UsdGeom.Xformable(c); xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, top - 0.05)); xf.AddScaleOp().Set(Gf.Vec3f(x1 - x0, y1 - y0, 0.1))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())


if a.pothole > 0:
    box("deck_a", -2.0, XB, ZP); box("deck_hole", XB, XB + a.pothole, ZP - a.h); box("deck_b", XB + a.pothole, XB + a.pothole + 4.0, ZP)
else:
    box("bump", XB - a.blen / 2, XB + a.blen / 2, a.h, 0.0 if a.side else -1.0, 1.0)

if a.gui:
    try:
        from pxr import UsdLux
        UsdLux.DomeLight.Define(sim.stage, "/World/dome").CreateIntensityAttr(800.0)
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[XB + 0.9, -1.7, 0.75 + ZP], target=[XB + 0.1, 0.0, 0.3 + ZP])
    except Exception as e:  # noqa: BLE001
        print("카메라·조명 설정 건너뜀:", e)
sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
for _ in range(2500):
    sim.step(POSE_A, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
M = sim.mass_matrix(); off = M.shape[0] - sim.n
FF = M[off + sim.ia][:, :6] / sim.K[:, None]  # 베이스 가속도(병진 xyz, 회전 xyz) → 모터 쪽 보정각
FLIP = np.array([1, 1, 1, 1, -1, 1.0])        # 뒷바퀴가 넘을 때: 차체 끄덕임(wy)만 부호 반대
DT = sim.dt; NB = 200; LIB = None; CORR = float("nan")
RQ = RV = RQREF = RVREF = RAREF = None; JMP = []; NPRE = int(round(a.pre * sim.hz))
FBMAP = ((1, abs(a.fbp) - 1, a.fp, 1.0 if a.fbp > 0 else -1.0), (2, 0, a.fy, 1.0))
KICKS = [(int(k_.split(",")[0]) - 1, math.radians(float(k_.split(",")[1])), float(k_.split(",")[2])) for k_ in a.kick.split(";") if k_]
SAT = [0, 0]


def clipj(j, v):
    """되먹임 보정 한 관절의 토크 한계: K_j·|Δq| <= tau_clip"""
    SAT[1] += 1
    if a.tau_clip <= 0: return v
    tm = sim.K[j] * v / a.tau_share; lim = a.tau_clip                    # 모터 토크로 바꿔 한 칸 단위로 끊고 자른다
    if a.tau_step > 0:
        tm = round(tm / a.tau_step) * a.tau_step; lim = math.floor(a.tau_clip / a.tau_step + 1e-9) * a.tau_step
    if abs(tm) >= lim - 1e-12 and abs(sim.K[j] * v / a.tau_share) > lim: SAT[0] += 1
    return float(a.tau_share * np.clip(tm, -lim, lim) / sim.K[j])


def kick_at(tt):
    k_ = np.zeros(6)
    for j, A_, t0_ in KICKS:
        u = tt - t0_
        if u <= 0: continue
        if a.kick_old:
            k_[j] += A_ * (min(1.0, u / 0.02) if u < 0.02 else math.exp(-(u - 0.02) / 0.08)); continue
        # 관절별 꼴(FW191_D_LOG 16.8절 (라), 659건): 오르는 시간, 돌아오는 시정수, 1 s 뒤 남는 각(펌웨어 PD만인 관절)
        ri, ta, rs = {1: (0.015, 0.18, 0.12), 2: (0.018, 0.15, 0.09), 4: (0.052, 0.09, 0.04), 5: (0.020, 0.10, 0.13)}.get(j, (0.02, 0.10, 0.0))
        rs = math.copysign(math.radians(rs), A_)
        k_[j] += A_ * u / ri if u < ri else rs + (A_ - rs) * math.exp(-(u - ri) / ta)
    return k_
EEB = list(sim.art.body_names).index("gripper_base")      # 끝단 위치 오차용 야코비안의 몸체 번호
if a.replay:
    import csv as _csv, gzip as _gz
    with _gz.open(a.replay, "rt") as _f: _r = list(_csv.DictReader(_f))
    _t = np.array([float(x["t"]) for x in _r]); _t = _t - _t[0]; _q = np.radians(np.array([[float(x[f"q{j}"]) for j in range(1, 7)] for x in _r]))
    _tt = np.arange(a.replay_t0 - a.pre, _t[-1], DT); RQ = np.stack([np.interp(_tt, _t, _q[:, j]) for j in range(6)], axis=1)
    RV = np.gradient(RQ, DT, axis=0); _k = np.ones(25) / 25.0; RV = np.stack([np.convolve(RV[:, j], _k, "same") for j in range(6)], axis=1)
    _jp = os.path.expanduser("~/piper_ws/docs/data/raw/mit_jumps.csv"); _st = os.path.basename(a.replay).replace(".csv.gz", "")
    JMP = [(float(x["exclude_from"]) - a.replay_t0, float(x["exclude_to"]) - a.replay_t0) for x in _csv.DictReader(open(_jp)) if x["stem"] == _st] if os.path.exists(_jp) else []
    _qr = np.radians(np.array([[float(x[f"qref{j}"]) for j in range(1, 7)] for x in _r])); RQREF = np.stack([np.interp(_tt, _t, _qr[:, j]) for j in range(6)], axis=1)
    _k2 = np.ones(41) / 41.0; RVREF = np.stack([np.convolve(np.gradient(RQREF[:, j], DT), _k2, "same") for j in range(6)], axis=1); RAREF = np.stack([np.convolve(np.gradient(RVREF[:, j], DT), _k2, "same") for j in range(6)], axis=1)


def run(case):
    tok = case.split("+")
    ffd = next((int(t[2:]) for t in tok if t.startswith("ff") and t != "ffservo"), None)
    servo = "ffservo" in tok
    prev = next((t for t in tok if t.startswith("prev")), None)
    pz = bool(prev and prev.startswith("prevz"))                      # prevz: 상하·끄덕임 성분만 예측하고 나머지는 잰 값(ffN)을 쓴다
    perr = int(prev[6 if pz else 5:]) if prev and len(prev) > (5 if pz else 4) else 0
    mp = next((t for t in tok if t.startswith("map")), None)
    mp_e = float(mp[4:]) / 1000.0 if mp and mp.startswith("mapE") else 0.0; mp_s = float(mp[4:]) / 100.0 if mp and mp.startswith("mapS") else 1.0
    mp_servo = mp == "mapservo"; servo = servo or mp_servo
    dmp = next((t for t in tok if t.startswith("damp")), None)
    dr = dmp[5:] if dmp and dmp.startswith("dampR") else None        # 감쇠만 더하기: 모드 주파수의 공진기(대역 통과)를 거친 되먹임
    if dr is not None:
        dmp = None
    RZ = np.zeros((2, 2))                                            # 축마다 공진기 상태 [z, dz] (끄덕임, 좌우)
    dj = dmp[5:] if dmp and dmp.startswith("dampJ") else None        # 실기처럼: 100 Hz 주기, 표본 나이가 매번 다름
    if dj is not None:
        dmp = None
    dd = int(dmp[4:].rstrip("p")) if dmp else None; dpred = bool(dmp and dmp.endswith("p"))
    rng = np.random.default_rng(a.seed); dq_fb = np.zeros(6); ages = []; SAT[0] = SAT[1] = 0
    P0_ = RQ[0] if RQ is not None else POSE_A
    sim.world.reset(); sim._apply_gains(); sim.set_state(P0_); sv = PosServo(POSE_A, DT)
    for _ in range(2500):
        sim.step(P0_, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
    arm = None; Z6 = np.zeros(6); W40 = dict(wheel_w=np.zeros(4), steer=np.zeros(4))
    if a.mit:
        hold = sim.K * (P0_ - sim.q_link()); qr0 = (RQREF[0] if RQREF is not None else POSE_A).copy()
        arm = MitArm(a, sim, P0_, qr0, mitarm.start_tff(a, hold, a.mit), a.mit, [int(x) - 1 for x in a.mit_tdc.split(",")], seed=a.seed)
    for k_ in range(NPRE):                                               # 자리 잡기: AMR은 서 있고 팔은 기준(기록)을 따라간다
        if arm is not None:
            if RQ is not None: qm_, wm_ = arm.step(RQREF[k_], RVREF[k_], RAREF[k_], RQ[k_], RV[k_])
            else: qm_, wm_ = arm.step(POSE_A, Z6, Z6, POSE_A, Z6)
            sim.step(qm_, wm_, **W40)
        elif RQ is not None: sim.step(RQ[k_], RV[k_], **W40)
        else: sim.step(POSE_A, Z6, **W40)
    dqff_last = np.zeros(6); vff = np.zeros(6); ERR = None
    Ta_ = a.v / 1.0; Tc_ = (XB + 0.9 - a.v * Ta_) / a.v; bz_ = a.bshape == "zvd" or "bz" in tok; BI = shaper("zvd", a.bsf, 0.10) if bz_ else [(1.0, 0.0)]

    def vprof(tt):
        u = tt - 0.1
        if a.still: return 0.0
        if not a.stop: return min(a.v, 1.0 * max(0.0, u))
        if u < 0: return 0.0
        if u < Ta_: return u
        if u < Ta_ + Tc_: return a.v
        if u < 2 * Ta_ + Tc_: return a.v - (u - Ta_ - Tc_)
        return 0.0
    T = (0.1 + 2 * Ta_ + Tc_ + 0.3 + a.tail) if a.stop else (0.3 + (XB + 0.3 + 0.5) / a.v + 2.0)
    n = int(T * sim.hz); t = np.arange(n) * DT; TE = np.zeros((n, 3)); te_ = np.zeros(3); OFF_ = None
    ERR = np.zeros((n, 3))
    G = np.zeros((n, 3)); XBASE = np.zeros(n); DQ = np.zeros((n, 6)); A6 = np.zeros((n, 6)); EV = np.zeros((n, 3)); BV = np.zeros((n, 6))
    v_prev = np.zeros(6); af = np.zeros(6); gf = np.zeros(3); al = 1 - math.exp(-2 * math.pi * a.lp * DT)
    abuf = [np.zeros(6)] * NB; gbuf = [np.zeros(3)] * NB
    t_f = None; rec = []; i_rear = None; x0 = None; tw0 = time.time()
    lp_lag = 1.0 / (2 * math.pi * a.lp)       # 저역 통과가 더하는 지연(대략)
    for i in range(n):
        vc = sum(ai * vprof(t[i] - ti) for ai, ti in BI)
        lv, av = sim.base_vel(); v6 = np.r_[lv, av]
        a6 = (v6 - v_prev) / DT if i > 0 else np.zeros(6); v_prev = v6
        af = af + al * (a6 - af); abuf.append(af.copy()); abuf.pop(0)
        g_now = sim.ee_gyro(); gf = gf + al * (g_now - gf); gbuf.append(gf.copy()); gbuf.pop(0)
        dq = np.zeros(6)
        # --- 앞바퀴가 턱에 닿은 때를 베이스 상하 가속도로 잡는다 ---
        xrel = (XBASE[i - 1] - x0) if i > 0 else 0.0
        if t_f is None and xrel + LWB / 2 >= XB - 0.06:                # 앞바퀴가 턱(홈)에 닿기 직전 — 위치로 잡는다
            t_f = t[i]; rec = [b.copy() for b in abuf[-30:]]          # 30 ms 앞부터 기억
            i_rear = i + int(round((LWB / a.v + perr / 1000.0) / DT)) - 30
        elif t_f is not None and len(rec) < 700:
            rec.append(af.copy())
        in_rear = prev is not None and i_rear is not None and 0 <= i - i_rear < len(rec)
        if mp is not None and LIB is not None:                         # 지도 기반 예견: 위치로 찾아 넣는다
            xq = (XBASE[i - 1] - x0 if i > 0 else 0.0) - mp_e + (a.v * 0.036 if mp_servo else 0.0)
            dq += mp_s * (FF @ np.array([np.interp(xq, LIB[0], LIB[1][:, c]) for c in range(6)]))
        elif in_rear and pz:
            am = abuf[-1 - (ffd if ffd is not None else 0)].copy(); pr = FLIP * rec[i - i_rear]; am[[2, 4]] = pr[[2, 4]]
            dq += FF @ am
        elif in_rear:
            dq += FF @ (FLIP * rec[i - i_rear])                        # 뒷바퀴: 기억해 둔 것을 지연 없이
        elif ffd is not None:
            dq += FF @ abuf[-1 - ffd]
        elif servo and not mp_servo:
            dq += FF @ abuf[-1 - 10]
        dq_ff = dq.copy()                                              # 여기까지가 미리 넣는 보정, 아래는 끝단 되먹임
        if dd is not None:                                             # 끝단 자이로 되먹임 감쇠
            s0 = gbuf[-1 - dd]; s1 = gbuf[-1 - dd - 4]; ds = (s0 - s1) / (4 * DT)
            for ax_, j, f, sg_ in FBMAP:                                # 끄덕임 → J5(또는 J2), 좌우 → J1
                w = 2 * math.pi * f; val = s0[ax_]
                if dpred:
                    ph = w * (dd / 1000.0 + lp_lag)
                    val = s0[ax_] * math.cos(ph) + ds[ax_] / w * math.sin(ph)   # 한 주파수 울림이라고 보고 앞당김
                dq[j] += clipj(j, -sg_ * (2 * a.zadd / w) * val)
        if dr is not None:
            # 끝단 각속도 → 모드 주파수의 대역 통과(공진기) → 지연만큼 위상을 당김 → 모터 쪽을 반대로.
            # 미분을 쓰지 않고 공진기의 두 상태(대역 통과 출력과 그 90도 늦은 값)로 당기므로 모드보다 빠른 성분은 줄어든 채 나간다.
            jit = dr.startswith("J")
            if (not jit) or i % 10 == 0:
                age = int(np.clip(rng.lognormal(math.log(22.5), 0.469), 8, 150)) if jit else int(dr)
                nsub = 10 if jit else 1
                for kk, (ax_, j, f, sg_) in enumerate(FBMAP):
                    w = 2 * math.pi * f; u = gbuf[-1 - age][ax_]
                    for _s in range(nsub):
                        acc_ = (w / a.Q) * (u - RZ[kk, 1]) - w * w * RZ[kk, 0]
                        RZ[kk, 1] += acc_ * DT; RZ[kk, 0] += RZ[kk, 1] * DT
                    ph = w * (age / 1000.0 + lp_lag + (0.005 if jit else 0.0))
                    y_lead = RZ[kk, 1] * math.cos(ph) - w * RZ[kk, 0] * math.sin(ph)
                    dq_fb[j] = clipj(j, -sg_ * (2 * a.zadd / w) * y_lead)
            dq = dq + dq_fb
        if dj is not None:
            if i % 10 == 0:                                            # 10 ms마다 한 번 계산하고 그 사이에는 값을 유지
                age = int(np.clip(rng.lognormal(math.log(22.5), 0.469), 8, 150)); ages.append(age)   # 중앙 22.5 ms, 95% 48.7 ms
                s0 = gbuf[-1 - age]; ds = (s0 - gbuf[-1 - age - 4]) / (4 * DT)
                gfac = 0.0 if ("s" in dj and age > 40) else (min(1.0, 25.0 / age) if "g" in dj else 1.0)
                for ax_, j, f, sg_ in FBMAP:
                    w = 2 * math.pi * f; val = s0[ax_]
                    if "n" not in dj:                                  # 표본 나이를 알고 그만큼 앞당김
                        ph = w * (age / 1000.0 + lp_lag + 0.005); val = s0[ax_] * math.cos(ph) + ds[ax_] / w * math.sin(ph)
                    dq_fb[j] = clipj(j, -sg_ * (2 * a.zadd / w) * val * gfac)
            dq = dq + dq_fb
        if a.dq_dead > 0:
            dq = np.where(np.abs(dq) < math.radians(a.dq_dead), 0.0, dq)
        if a.dq_step > 0:
            st_ = math.radians(a.dq_step); dq = np.round(dq / st_) * st_
        DQ[i] = dq
        kq = kick_at(t[i]) if KICKS else Z6; dq_all = dq; dq = dq + kq
        ii = min(NPRE + i, len(RQ) - 1) if RQ is not None else 0
        if arm is not None:                                            # MIT 모드: 미리 넣는 보정은 위치 기준에, 되먹임은 링크에 닿는 토크로
            if i % 10 == 0: vff = (dq_ff - dqff_last) / 0.01 if i else Z6; dqff_last = dq_ff.copy()
            qb_, wb_ = (RQ[ii], RV[ii]) if RQ is not None else (POSE_A, Z6)
            qf_, vf_, af_ = (RQREF[ii], RVREF[ii], RAREF[ii]) if RQ is not None else (POSE_A, Z6, Z6)
            qm_, wm_ = arm.step(qf_ + dq_ff, vf_ + vff, af_, qb_ + dq, wb_)
            ERR[i] = (qf_ - qm_)[:3]
            q = qm_.copy(); q[:3] += (dq_all - dq_ff)[:3] + kq[:3]; w_ = wm_.copy()
        elif RQ is not None:                                           # 팔이 움직이는 중: 재생 각도 위에 보정을 더한다
            q, w_ = RQ[ii] + dq, RV[ii]; ERR[i] = (RQREF[ii] - RQ[ii])[:3]
        elif servo:
            q, w_ = sv.step(POSE_A + dq)
        else:
            q, w_ = POSE_A + dq, np.zeros(6)
        sim.step(q, w_, wheel_w=np.full(4, vc / WHEEL_R), steer=np.zeros(4))
        if a.gui and i % 20 == 0:
            sim.world.render(); lag_ = i * DT * a.slow - (time.time() - tw0)
            if lag_ > 0: time.sleep(lag_)
        G[i] = g_now; A6[i] = af; EV[i] = sim.ee_vel()[0]; BV[i] = v6
        if i % 5 == 0:                                                 # 끝단 위치가 기준 궤적에서 벗어난 양: J_v (q_링크 - q_기준)
            Jv = np.asarray(sim.art.get_jacobians())[0, EEB, :3, :]; Jv = Jv[:, Jv.shape[1] - sim.n + sim.ia]
            qd_ = RQREF[ii] if RQREF is not None else POSE_A
            te_ = Jv @ (sim.q_link() - qd_)
            if OFF_ is None: OFF_ = te_.copy()                         # 시작할 때의 정적 처짐은 뺀다
        TE[i] = te_ - (OFF_ if OFF_ is not None else 0.0)
        p = np.asarray(sim.bb.get_world_poses()[0])[0]; XBASE[i] = p[0]
        if x0 is None: x0 = p[0]
    tf = t_f if t_f is not None else (XB - LWB / 2) / a.v; tr = tf + LWB / a.v

    def rms(t0, t1):
        m = (t >= t0) & (t < t1)
        B = np.stack([band(G[m, c], sim.hz, 2.0, 20.0) for c in range(3)], axis=1); ax = np.sqrt(np.mean(B ** 2, axis=0))
        return float(np.sqrt(np.sum(ax ** 2))), ax
    allv, ax = rms(tf - 0.1, tr + 1.5); fr, _ = rms(tf - 0.1, tr - 0.1); re, _ = rms(tr - 0.1, tr + 1.5)
    if case == "none":                                                 # 지도(위치 → 베이스 가속도)와 앞·뒤 충격의 닮은 정도
        np.savez(HERE + f"/out/exp6_lib_{a.tag}.npz", x=XBASE - x0, a6=A6)
        if a.lib_from:
            L_ = np.load(HERE + f"/out/exp6_lib_{a.lib_from}.npz"); globals()["LIB"] = (L_["x"], L_["a6"] * a.lib_scale)
        else:
            globals()["LIB"] = (XBASE - x0, A6.copy())
        if a.stop:
            lx, la = LIB; x1_, x2_ = XB - LWB / 2 - 0.10, XB + LWB / 2 + 0.35
            wgt = np.clip((lx - x1_) / 0.03, 0, 1) * np.clip((x2_ - lx) / 0.03, 0, 1); kp = np.r_[True, np.diff(np.maximum.accumulate(lx)) > 0]
            globals()["LIB"] = (lx[kp], (la * wgt[:, None])[kp])
        i0 = np.searchsorted(t, tf - 0.03); j0 = np.searchsorted(t, tr - 0.03); best = 0.0
        for lag in range(-150, 151, 2):
            f_, r_ = A6[i0:i0 + 600, 2], A6[j0 + lag:j0 + lag + 600, 2]
            if len(r_) == 600: best = max(best, float(np.corrcoef(f_, r_)[0, 1]))
        globals()["CORR"] = best
    mb = (t >= tf - 0.1) & (t < tr + 1.5)
    bb_ = [float(np.sqrt(np.mean(band(A6[mb, c], sim.hz, 2.0, 20.0) ** 2))) for c in (0, 2, 4)]
    # 전달률: 끝단 가속도 / 베이스 가속도 (세 축 벡터, 2–20 Hz). 속도를 대역 통과한 뒤 미분한다
    def acc_rms(Vel):
        return float(np.sqrt(sum(np.mean(np.gradient(band(Vel[mb, c], sim.hz, 2.0, 20.0), DT) ** 2) for c in range(3))))
    ea_, ba_ = acc_rms(EV), acc_rms(BV[:, :3])
    def acc_band(lo, hi):
        return float(np.sqrt(sum(np.mean(np.gradient(band(EV[mb, c], sim.hz, lo, hi), DT) ** 2) for c in range(3))))
    ea_lo, ea_hi = acc_band(2.0, 10.0), acc_band(10.0, 20.0)
    bg_ = float(np.sqrt(sum(np.mean(band(BV[mb, 3 + c], sim.hz, 2.0, 20.0) ** 2) for c in range(3))))
    mf = (t >= 0.1) & (t < T - 0.05)                                    # 전체 구간 지표(종합 시나리오). MIT 기록의 순간 튐 구간은 뺀다
    for j0_, j1_ in JMP: mf &= ~((t >= j0_) & (t < j1_))
    fax_ = [float(np.sqrt(np.mean(band(G[:, c], sim.hz, 2.0, 20.0)[mf] ** 2))) for c in range(3)]; fg_ = float(np.sqrt(sum(x_ ** 2 for x_ in fax_)))
    fa_ = [float(np.sqrt(sum(np.mean(np.gradient(band(EV[:, c], sim.hz, lo_, hi_), DT)[mf] ** 2) for c in range(3)))) for lo_, hi_ in ((2.0, 10.0), (10.0, 20.0))]
    fa2_ = float(np.sqrt(sum(np.mean(np.gradient(band(EV[:, c], sim.hz, 20.0, 40.0), DT)[mf] ** 2) for c in range(3))))
    tv_ = float(np.sqrt(sum(np.mean(band(TE[:, c], sim.hz, 2.0, 20.0)[mf] ** 2) for c in range(3)))) * 1000.0
    terr_ = float(np.sqrt(np.mean(np.sum(TE[mf] ** 2, axis=1)))) * 1000.0; tpk_ = float(np.max(np.linalg.norm(TE[mf], axis=1))) * 1000.0
    row = dict(case=case, bz=int(bz_), still=int(a.still), replay=os.path.basename(a.replay)[:40] if a.replay else "", full_vec=fg_, full_roll=fax_[0], full_pitch=fax_[1], full_yaw=fax_[2], kept_s=float(np.sum(mf) * DT), t0=a.replay_t0, full_acc_lo=fa_[0], full_acc_hi=fa_[1], tip_vib_mm=tv_, tip_err_mm=terr_, tip_peak_mm=tpk_, bshape=a.bshape, ee_acc_lo=ea_lo, ee_acc_hi=ea_hi, ee_acc_rms=ea_, base_acc_rms=ba_, trans=ea_ / ba_ if ba_ > 0 else float("nan"), base_gyro_rms=bg_, base_ax_rms=bb_[0], base_az_rms=bb_[1], base_pitch_acc_rms=bb_[2], mount_x=a.mount_x, mount_yaw=a.yaw, blen=a.blen, side=a.side, pose=a.pose, dq_step=a.dq_step, dq_dead=a.dq_dead, pothole=a.pothole, susp_k=a.susp_k, tire_k=a.tire_k, az_corr=CORR, base_az_peak=float(np.max(np.abs(A6[(t >= tf - 0.1) & (t < tr + 1.5), 2]))), v=a.v, h=a.h, vec=allv, front=fr, rear=re, roll=float(ax[0]), pitch=float(ax[1]), yaw=float(ax[2]),
               dq_peak_deg=float(np.degrees(np.max(np.abs(DQ)))), dq_pk_j1=float(np.degrees(np.max(np.abs(DQ[:, 0])))), dq_pk_j2=float(np.degrees(np.max(np.abs(DQ[:, 1])))), dq_pk_j5=float(np.degrees(np.max(np.abs(DQ[:, 4])))),
               tau_pk_j1=float(sim.K[0] * np.max(np.abs(DQ[:, 0]))), tau_pk_j2=float(sim.K[1] * np.max(np.abs(DQ[:, 1]))), tau_pk_j5=float(sim.K[4] * np.max(np.abs(DQ[:, 4]))), sat=SAT[0] / max(1, SAT[1]), zadd=a.zadd, fbp=a.fbp, tau_clip=a.tau_clip, tau_step=a.tau_step, tau_share=a.tau_share, kick=a.kick,
               full_acc_20_40=fa2_, t_front=tf, x_end=float(XBASE[-1] - x0), mit=a.mit, mit_tdc=a.mit_tdc if a.mit and a.mit != "pd" else "", pre=a.pre,
               **{f"err_j{j + 1}": float(np.degrees(np.std(ERR[mf, j]))) for j in range(3)}, **{f"errm_j{j + 1}": float(np.degrees(np.mean(ERR[mf, j]))) for j in range(3)},
               n_ratelim=arm.nlim if arm is not None else 0, **{f"slip_j{j + 1}": (float(arm.nslip[j]) / max(1, arm.k) if arm is not None else float("nan")) for j in range(3)})
    if a.keep:
        np.savez(HERE + f"/out/exp6_ts_{a.tag}_{case}.npz", t=t, gyro=G, dq=DQ, a6=A6, ev=EV, bv=BV, te=TE, tf=tf, tr=tr)
    return row


tw = time.time(); rows = []; lp_ = 0
while True:
    rows = []
    for case in a.cases.split(","):
        rows.append(run(case)); r_ = rows[-1]; b_ = rows[0]
        if a.gui:
            if a.stop:
                print(f"[{lp_ + 1}] {case:20s} 전체 구간: 끝단 각속도 {r_['full_vec']:.4f} rad/s, 가속도 {r_['full_acc_lo']:.2f} / {r_['full_acc_hi']:.2f} m/s² (2–10 / 10–20 Hz), 궤적 주위 흔들림 {r_['tip_vib_mm']:.2f} mm, 궤적 오차 {r_['tip_err_mm']:.2f} mm(최대 {r_['tip_peak_mm']:.2f})", flush=True)
            print(f"[{lp_ + 1}] {case:24s} 끝단 진동 {r_['vec']:.4f} rad/s (대책 없음의 {r_['vec'] / b_['vec']:.2f}배; 앞바퀴 구간 {r_['front'] / b_['front']:.2f}, 뒷바퀴 구간 {r_['rear'] / b_['rear']:.2f})"
                  f" | 베이스 상하 가속도 {r_['base_az_rms']:.2f} m/s², 앞뒤 {r_['base_ax_rms']:.2f} m/s²", flush=True)
        else:
            print("ROW", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in r_.items()}, flush=True)
        if a.gui and not app.is_running(): break
    lp_ += 1
    if (a.loops and lp_ >= a.loops) or (a.gui and not app.is_running()) or (not a.gui and not a.loops): break
with open(HERE + f"/out/exp6_{a.tag}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
b = rows[0]
print("\n조건            전체   앞바퀴  뒷바퀴   (대책 없음 대비 비)   보정 최대[deg]")
for r in rows:
    print("%-16s %.2f   %.2f   %.2f      %.1f" % (r["case"], r["vec"] / b["vec"], r["front"] / b["front"], r["rear"] / b["rear"], r["dq_peak_deg"]))
print("wall %.1f s" % (time.time() - tw))
sim.close()
