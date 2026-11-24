"""진동 검증용 Isaac Sim 장면 (PiPER 단독 고정 베이스 / Ranger Mini 탑재).

기존 plant_node(벨트·비전·pick&place)와 별개다. 여기서는 끝단 진동만 본다.

모형
- 팔: 공식 URDF에서 가져온 USD(질량 4.66 kg). 기존 piper_v2.usd는 링크 질량이 1/4 수준이라 쓰지 않는다.
- 관절 탄성: PhysX 관절 각도 = 링크 쪽 각도 q_l. 모터 쪽 각도 q_m은 이 파일에서 따로 적분한다.
  관절 구동(stiffness K, damping D)이 둘 사이의 스프링·댐퍼다:  tau = K (q_m - q_l) + D (dq_m - dq_l)
  엔코더가 읽는 값은 q_m, 끝단 IMU가 보는 것은 q_l 쪽이다.
- 모터 쪽: J_m ddq_m = Kp (q_d - q_m) + Kd (dq_d - dq_m) + tau_ff - tau_spring
  위치 모드는 (q_d = 지연된 명령, 펌웨어 게인 Kp_pos·Kd_pos), MIT 모드는 (우리가 준 Kp·Kd·tau_ff, 100 Hz, 8 bit 토크).
- 물리 스텝 1 kHz.

사용: 다른 스크립트에서  from vibsim import VibSim
GUI로 보려면 환경 변수 VIBSIM_GUI=1
"""
import math
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("VIBSIM_GUI", "0") != "1"})

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade, Sdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402

import carb  # noqa: E402

# 상태는 텐서 API로 직접 읽으므로 매 스텝 USD에 되쓰지 않는다(스텝 속도 2배 이상)
if os.environ.get("VIBSIM_GUI", "0") != "1" and os.environ.get("VIBSIM_USD", "0") != "1":
    _s = carb.settings.get_settings()
    for _k in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateForceSensorsToUsd", "/physics/updateParticlesToUsd"):
        _s.set_bool(_k, False)

HERE = os.path.dirname(os.path.abspath(__file__))
PIPER_USD = HERE + "/assets/piper_abs/piper_abs.usda"
RANGER_USD = os.path.expanduser("~/projects/ranger_piper/ranger_mini/usd/ranger_mini_v2/ranger_mini_v2.usda")

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIP_JOINTS = ["joint7", "joint8"]
STEER_JOINTS = ["fl_steering_joint", "fr_steering_joint", "rl_steering_joint", "rr_steering_joint"]
DRIVE_JOINTS = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]
WHEEL_R = 0.100
ARM_MOUNT_X = -0.23      # 팔 장착 위치(앞뒤). 뒷바퀴 축 바로 위 — 다한이 실물도 이리 옮기기로 함(2026-11-22). 0이면 차체 가운데(그 결과는 out/center)
RANGER_REST_Z = 0.280   # 바퀴가 바닥에 닿을 때의 base_link 높이(가라앉은 값 0.277) + 3 mm
ARM_BASE_LOCAL = (-0.01858, 0.02232, 0.04462)  # Ranger 상판 기준 팔 베이스 위치(step22_common과 같은 값)
ARM_BASE_YAW_DEG = 90.0  # 실물 장착 방향(step22_common과 같음): 팔 앞이 AMR의 옆(+y). 0이면 팔 앞 = AMR 진행 방향
CAM_POS_L6 = (-0.0115, 0.0731, 0.05)  # 손목 카메라 위치(link6 기준, step22_common EIH_MOUNT_POS)

POSE_A = np.radians([-0.3, 121.1, -73.2, 1.6, 52.0, 13.7])
POSE_B = np.radians([0.0, 125.0, -100.0, 0.0, 50.0, 13.7])
POSE_H = np.radians([-0.3, 121.1, -73.2, 1.6, -30.0, 13.7])   # 자세 A에서 손목(J5)만 돌려 그리퍼가 옆(수평)을 향하게 — 박사님 11/23 제안. 실측으로 맞춘 자세가 아니다

# 기본 파라미터 — identify.py 결과로 채운다(값의 근거는 README)
# 관절 강성 [N·m/rad]: fit_modal.py — 자세 A 7.01 Hz·자세 B 6.25 Hz(실측 6.6–7.3 / 6.2–6.3), 둘째 모드 12.6 Hz
# J1(좌우): 10/23 J1 계단·옆 툭을 좌우 축으로 다시 낸 값(FW191_D_LOG 14절) — 자세 A 4.0–5.1 Hz, 자세 B 3.1–3.4 Hz, 감쇠비 0.07–0.19.
# K1 = 200이면 자세 A 4.15 Hz, 자세 B 3.5 Hz. 이 성분에는 J1 관절이 서보에 대해 돌아가는 몫이 섞여 있는데(엔코더에도 보임) 여기서는 스프링 하나로 묶었다.
# (11/22 저녁 첫 실행은 VIB_E1의 "수평 9.4–9.6 Hz"에 맞춘 1500이었다 — 그 값은 좌우 축 것이 아니었다. 결과는 out/lat11hz)
# J2·J3·J5(끄덕임): 11/22 저녁 재식별 — 실측 엔코더 기록(C-1)을 모터 쪽에 재생했을 때 끝단 진동 크기가 실측과 맞는 조합.
# 자세 A 7.08·7.89 Hz(가까운 모드 둘), 자세 B 6.24 Hz. 손목(J5)이 무르다. 처음 값 (1355, 361, 342)은 주파수는 같지만 진동 크기가 실측의 절반이었다(out/prefit).
# 2026-11-24 재식별: 그리퍼가 앞을 보는 자세(H)의 실측(끄덕임 6.3–6.45 Hz, 젯슨 FW191_D_LOG 23·24절)까지 넣어 다시 골랐다.
# 앞의 값 (881, 5704, 27)은 자세 A·B만 보고 손목(J5)을 무르게 잡은 조합이라 자세 H에서 5.5 Hz·실측의 3배로 흔들렸다.
# 새 값은 자세 A 7.2–7.4·B 6.2–6.3·H 6.3–6.45 Hz를 같이 맞추는 조합(K2 806–1320, K3 386–1500, K5 134–3000) 가운데 하나다.
# 두 자세의 기록 열넷을 재생한 끝단 각속도 / 실측(2–10 Hz): 자세 A 0.70(0.59–0.79), 자세 H 1.02(0.88–1.10). 앞의 값은 0.97(0.72–1.42)·2.96(1.42–3.48).
# 앞의 값으로 돌리려면 환경 변수 VIBSIM_K=old. (그때의 결과는 out/Kold_1124, figs/Kold_1124)
K_OLD_1122 = np.array([200.0, 881.0, 5704.0, 3000.0, 27.0, 3000.0])
DEFAULT_K = K_OLD_1122.copy() if os.environ.get("VIBSIM_K", "") == "old" else np.array([200.0, 1100.0, 500.0, 3000.0, 800.0, 3000.0])
if "," in os.environ.get("VIBSIM_K", ""): DEFAULT_K = np.array([float(v) for v in os.environ["VIBSIM_K"].split(",")])   # 민감도 시험용: VIBSIM_K=200,1100,500,3000,60,3000
DEFAULT_ZETA = 0.10          # 자세 A 실측 감쇠비 0.09–0.13
DEFAULT_F1 = 7.0
DEFAULT_D = DEFAULT_K * 2 * DEFAULT_ZETA / (2 * math.pi * DEFAULT_F1)   # 강성 비례 감쇠: 1차 모드에서 zeta
DEFAULT_D[0] = 2 * 0.12 * DEFAULT_K[0] / (2 * math.pi * 4.15)            # 좌우 모드 감쇠비 0.12
CAM_GAIN = 1.27              # 카메라 자리 끄덕임 / 그리퍼 몸통 끄덕임 (FW191_B_LOG) — 절댓값 비교 때만 곱한다


def _find(stage, root, name, api=None):
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if p.GetName() == name and (api is None or p.HasAPI(api)):
            return p
    return None


class VibSim:
    def __init__(self, base="fixed", hz=1000, cam_mass=0.11, payload=0.0, K=None, D=None, pose=POSE_A,
                 wheel_mu=1.0, tire_k=0.0, tire_c=0.0, susp_k=0.0, susp_c=0.0, mount_yaw=ARM_BASE_YAW_DEG, mount_x=ARM_MOUNT_X, z0=0.0, verbose=True):
        self.mount_x, self.z0 = mount_x, z0   # 팔 장착 위치를 앞뒤로 옮김[m](음수 = 뒷바퀴 쪽), 로봇을 놓는 높이[m]
        self.mount_yaw = mount_yaw if base != "fixed" else 0.0
        self.susp_k, self.susp_c = susp_k, susp_c   # 바퀴마다 상하 서스펜션 [N/m, N·s/m] — 0이면 없음(강체 섀시)
        self.tire_k, self.tire_c = tire_k, tire_c   # 타이어(접촉) 탄성 [N/m, N·s/m] — 0이면 강체 접촉
        self.base, self.hz, self.dt = base, hz, 1.0 / hz
        self.world = World(physics_dt=self.dt, rendering_dt=self.dt * 20, stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        st = self.stage = get_current_stage()
        scene = None
        for p in st.Traverse():
            if p.IsA(UsdPhysics.Scene):
                scene = p
        if scene is not None:
            px = PhysxSchema.PhysxSceneAPI.Apply(scene)
            px.CreateSolverTypeAttr("TGS")
            px.CreateEnableStabilizationAttr(False)

        add_reference_to_stage(usd_path=PIPER_USD, prim_path="/World/piper")
        arm_base = _find(st, "/World/piper", "base_link", UsdPhysics.RigidBodyAPI)
        self.l6 = _find(st, "/World/piper", "link6", UsdPhysics.RigidBodyAPI)
        self.gb = _find(st, "/World/piper", "gripper_base", UsdPhysics.RigidBodyAPI) or self.l6
        assert arm_base and self.l6, "PiPER 링크를 찾지 못함"

        # 손목 카메라(+ 브래킷) 질량과 끝단 부하: 측정 링크에 질점으로 더한다
        self._add_point_mass(self.gb, cam_mass + payload, CAM_POS_L6)

        if base == "fixed":
            root = "/World/piper"
            UsdGeom.XformCommonAPI(st.GetPrimAtPath("/World/piper")).SetTranslate(Gf.Vec3d(0, 0, 0.8))
            fj = UsdPhysics.FixedJoint.Define(st, "/World/piper/world_fix")
            fj.CreateBody1Rel().SetTargets([arm_base.GetPath()])
            fj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0.8))
            fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0)); fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            # 고정 베이스 articulation: 루트를 월드 고정 관절로 옮긴다(안 그러면 떠 있는 베이스 + 구속으로 풀린다)
            for p in Usd.PrimRange(st.GetPrimAtPath("/World/piper")):
                if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                    p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                        p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            UsdPhysics.ArticulationRootAPI.Apply(fj.GetPrim())
        else:
            root = "/World/Ranger"
            add_reference_to_stage(usd_path=RANGER_USD, prim_path=root)
            # 바퀴가 바닥에 닿는 높이에서 시작한다. 예전에는 0에 놓아 바퀴가 바닥에 묻힌 채 시작했고,
            # 그 반발로 차체가 0.3–0.6 m 튀어 오른 뒤(서스펜션이 있으면 18.7° 틀어져) 내려앉았다(2026-11-22 다한이 화면에서 발견).
            z0 = z0 + RANGER_REST_Z
            UsdGeom.XformCommonAPI(st.GetPrimAtPath(root)).SetTranslate(Gf.Vec3d(0, 0, z0))
            ABL = (ARM_BASE_LOCAL[0] + mount_x, ARM_BASE_LOCAL[1], ARM_BASE_LOCAL[2])
            rb = _find(st, root, "base_link", UsdPhysics.RigidBodyAPI)
            for p in Usd.PrimRange(st.GetPrimAtPath("/World/piper")):
                if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                    p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                        p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            UsdGeom.XformCommonAPI(st.GetPrimAtPath("/World/piper")).SetTranslate(Gf.Vec3d(ABL[0], ABL[1], ABL[2] + z0))
            UsdGeom.XformCommonAPI(st.GetPrimAtPath("/World/piper")).SetRotate(Gf.Vec3f(0, 0, self.mount_yaw))
            fj = UsdPhysics.FixedJoint.Define(st, "/World/piper/mount_joint")
            fj.CreateBody0Rel().SetTargets([rb.GetPath()])
            fj.CreateBody1Rel().SetTargets([arm_base.GetPath()])
            fj.CreateLocalPos0Attr(Gf.Vec3f(*ABL)); fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            cy, sy = math.cos(math.radians(self.mount_yaw / 2)), math.sin(math.radians(self.mount_yaw / 2))
            fj.CreateLocalRot0Attr(Gf.Quatf(cy, 0, 0, sy)); fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            self.ranger_base = rb
            if susp_k > 0:
                self._add_suspension(rb)
        self.root = root

        ar = self.stage.GetPrimAtPath(self._art_root(root))
        pa = PhysxSchema.PhysxArticulationAPI.Apply(ar)
        pa.CreateSolverPositionIterationCountAttr(32); pa.CreateSolverVelocityIterationCountAttr(4)
        pa.CreateEnabledSelfCollisionsAttr(False); pa.CreateSleepThresholdAttr(0.0)
        self.art = Articulation(prim_paths_expr=str(ar.GetPath()), name="robot")
        self.world.scene.add(self.art)
        self.ee = RigidPrim(prim_paths_expr=str(self.gb.GetPath()), name="ee")
        self.world.scene.add(self.ee)
        if base != "fixed":
            self.bb = RigidPrim(prim_paths_expr=str(self.ranger_base.GetPath()), name="bb")
            self.world.scene.add(self.bb)
        if base != "fixed":
            nw = self._ranger_materials(wheel_mu)
        self.world.reset()
        if base != "fixed" and nw == 0:      # 참조 직후 충돌 프림이 아직 안 보이는 경우
            nw = self._ranger_materials(wheel_mu)
        if base != "fixed" and verbose:
            print(f"[vibsim] 바퀴 충돌 프림 {nw}개에 재질 적용 (mu {wheel_mu}, tire_k {tire_k}, tire_c {tire_c})")

        names = list(self.art.dof_names)
        self.names = names
        self.n = len(names)
        self.ia = np.array([names.index(j) for j in ARM_JOINTS])
        self.ig = np.array([names.index(j) for j in GRIP_JOINTS])
        if base != "fixed":
            self.isteer = np.array([names.index(j) for j in STEER_JOINTS])
            self.iw = np.array([names.index(j) for j in DRIVE_JOINTS])

        self.K = np.array(DEFAULT_K if K is None else K, float)
        self.D = np.array(DEFAULT_D if D is None else D, float)
        self._apply_gains()

        # 모터 쪽 상태
        self.qm = np.array(pose, float).copy()
        self.dqm = np.zeros(6)
        self.set_state(pose)
        self.t = 0.0
        if verbose:
            print(f"[vibsim] base={base} dof={self.n} names={names}")

    # ---- 장면 구성 보조 ----
    def _art_root(self, root):
        for p in Usd.PrimRange(self.stage.GetPrimAtPath(root)):
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                return p.GetPath()
        raise RuntimeError("articulation root 없음: " + root)

    def _add_suspension(self, rb):
        """바퀴 네 곳에 상하 직동 관절(스프링·댐퍼)을 넣는다: base_link → susp_X → (조향) → 바퀴.
        원래 모델에는 서스펜션이 없다. 실물 값은 모르므로 섀시 끄덕임 모드를 바꿔 보는 민감도 시험용이다."""
        st = self.stage
        Tb = UsdGeom.Xformable(rb).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.susp_names = []
        for jn in STEER_JOINTS:
            jp = _find(st, "/World/Ranger", jn)
            j = UsdPhysics.Joint(jp)
            lp0 = j.GetLocalPos0Attr().Get(); lr0 = j.GetLocalRot0Attr().Get()
            M = Gf.Matrix4d(); M.SetTransform(Gf.Rotation(Gf.Quatd(lr0.GetReal(), Gf.Vec3d(*lr0.GetImaginary()))), Gf.Vec3d(*lp0))
            Tw = M * Tb
            name = "susp_" + jn.split("_")[0]
            path = "/World/Ranger/" + name
            x = UsdGeom.Xform.Define(st, path)
            x.AddTranslateOp().Set(Tw.ExtractTranslation())
            q = Tw.ExtractRotationQuat()
            x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
            UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
            ma = UsdPhysics.MassAPI.Apply(x.GetPrim()); ma.CreateMassAttr(0.5); ma.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-3, 1e-3, 1e-3))
            pj = UsdPhysics.PrismaticJoint.Define(st, "/World/Ranger/" + name + "_joint")
            pj.CreateAxisAttr("Z"); pj.CreateLowerLimitAttr(-0.05); pj.CreateUpperLimitAttr(0.05)
            pj.CreateBody0Rel().SetTargets([rb.GetPath()]); pj.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            pj.CreateLocalPos0Attr(lp0); pj.CreateLocalRot0Attr(lr0)
            pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear"); d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(self.susp_k); d.CreateDampingAttr(self.susp_c); d.CreateMaxForceAttr(1.0e5)
            j.GetBody0Rel().SetTargets([Sdf.Path(path)])
            j.GetLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0)); j.GetLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            self.susp_names.append(name + "_joint")

    def _add_point_mass(self, prim, m, pos):
        """링크의 질량·질량 중심·관성에 질점 m(pos)을 더한다."""
        if m <= 0:
            return
        api = UsdPhysics.MassAPI(prim)
        m0 = float(api.GetMassAttr().Get())
        c0 = np.array(api.GetCenterOfMassAttr().Get(), float)
        I0 = np.array(api.GetDiagonalInertiaAttr().Get(), float)
        p = np.array(pos, float)
        m1 = m0 + m
        c1 = (m0 * c0 + m * p) / m1
        # 주축 방향은 그대로 두고 평행축 항만 대각에 더한다(끝단 작은 질량이라 주축 회전은 무시)
        def par(mm, d):
            return mm * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
        I1 = I0 + par(m0, c0 - c1) + par(m, p - c1)
        api.GetMassAttr().Set(m1)
        api.GetCenterOfMassAttr().Set(Gf.Vec3f(*c1.tolist()))
        api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*I1.tolist()))

    def _ranger_materials(self, mu):
        st = self.stage
        mpath = "/World/PM_drive"
        UsdShade.Material.Define(st, mpath)
        m = UsdPhysics.MaterialAPI.Apply(st.GetPrimAtPath(mpath))
        m.CreateStaticFrictionAttr(mu); m.CreateDynamicFrictionAttr(mu); m.CreateRestitutionAttr(0.0)
        if self.tire_k > 0:   # 접촉점마다 스프링·댐퍼(compliant contact) — 섀시의 상하·끄덕임 모드를 만든다
            pm = PhysxSchema.PhysxMaterialAPI.Apply(st.GetPrimAtPath(mpath))
            pm.CreateCompliantContactStiffnessAttr(self.tire_k); pm.CreateCompliantContactDampingAttr(self.tire_c)
        mat = UsdShade.Material(st.GetPrimAtPath(mpath))
        nw = 0
        for p in Usd.PrimRange(st.GetPrimAtPath(self.root), Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.CollisionAPI) and "wheel_link" in str(p.GetPath()).lower() and not p.IsInstanceProxy():
                UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics"); nw += 1
        return nw

    def _apply_gains(self):
        kp = np.zeros(self.n); kd = np.zeros(self.n); mf = np.full(self.n, 1.0e4)
        kp[self.ia], kd[self.ia] = self.K, self.D
        kp[self.ig], kd[self.ig] = 1.0e4, 1.0e2
        if self.base != "fixed":
            kp[self.isteer], kd[self.isteer] = 3.0e5, 3.0e3
            kd[self.iw] = 1.0e4
            mf[self.iw] = 1.0e5
            if self.susp_k > 0:
                isu = np.array([self.names.index(x) for x in self.susp_names])
                kp[isu], kd[isu], mf[isu] = self.susp_k, self.susp_c, 1.0e5
        self.art.set_gains(kps=kp[None, :], kds=kd[None, :])
        self.art.set_max_efforts(mf[None, :])

    def set_KD(self, K, D):
        self.K, self.D = np.array(K, float), np.array(D, float)
        self._apply_gains()

    # ---- 상태 ----
    def set_state(self, q_arm):
        q = np.zeros(self.n); q[self.ia] = q_arm
        self.art.set_joint_positions(q[None, :])
        self.art.set_joint_velocities(np.zeros((1, self.n)))
        self.qm = np.array(q_arm, float).copy(); self.dqm[:] = 0
        self._targets()

    def _targets(self, wheel_w=None, steer=None):
        pt = np.zeros(self.n); vt = np.zeros(self.n)
        pt[self.ia], vt[self.ia] = self.qm, self.dqm
        self.art.set_joint_position_targets(pt[None, :])
        if self.base != "fixed":
            if steer is not None:
                pt[self.isteer] = steer
                self.art.set_joint_position_targets(pt[None, :])
            if wheel_w is not None:
                vt[self.iw] = wheel_w
        self.art.set_joint_velocity_targets(vt[None, :])

    def q_link(self):
        return np.asarray(self.art.get_joint_positions())[0][self.ia]

    def dq_link(self):
        return np.asarray(self.art.get_joint_velocities())[0][self.ia]

    def spring_torque(self):
        """스프링이 링크에 주는 토크(= 모터 쪽이 받는 반력의 반대 부호)."""
        return self.K * (self.qm - self.q_link()) + self.D * (self.dqm - self.dq_link())

    def ee_vel(self):
        v = np.asarray(self.ee.get_velocities())[0]
        return v[:3], v[3:]  # 선속도, 각속도(월드)

    def ee_gyro(self):
        """끝단 각속도를 팔 기준 축으로: [비틀림(팔 앞 방향 축), 끄덕임, 좌우(수직축)]. 베이스의 회전은 작다고 보고 장착 각만 되돌린다."""
        w = np.asarray(self.ee.get_velocities())[0][3:]
        c, s = math.cos(math.radians(self.mount_yaw)), math.sin(math.radians(self.mount_yaw))
        return np.array([c * w[0] + s * w[1], -s * w[0] + c * w[1], w[2]])

    def base_vel(self):
        v = np.asarray(self.bb.get_velocities())[0]
        return v[:3], v[3:]

    def mass_matrix(self):
        M = np.asarray(self.art.get_mass_matrices())[0]
        return M

    def gravity(self):
        return np.asarray(self.art.get_generalized_gravity_forces())[0]

    def step(self, qm, dqm, wheel_w=None, steer=None):
        """모터 쪽 각도·속도(6)를 주고 한 스텝 진행."""
        self.qm, self.dqm = np.asarray(qm, float), np.asarray(dqm, float)
        self._targets(wheel_w, steer)
        self.world.step(render=False)
        self.t += self.dt

    def settle(self, sec=2.0):
        for _ in range(int(sec * self.hz)):
            self.step(self.qm, np.zeros(6), wheel_w=np.zeros(4) if self.base != "fixed" else None)

    def close(self):
        app.close()


class PosServo:
    """위치 모드 펌웨어 서보(모터 쪽): 명령을 100 Hz로 받아 지연 Td 뒤 2차 응답으로 따라간다.
    값은 fit_servo.py — C-1 원본 18시행의 명령(qref2) → 엔코더(q2) 응답에 맞춤(위치 오차 rms 0.029°, 등속 중 추종 지연 36 ms).
    처음 값(55 ms, 9 Hz, 0.35)은 엔코더 속도 봉우리를 서보 넘침으로 잘못 읽고 맞춘 것이었다."""

    def __init__(self, q0, dt, Td=0.020, fs=10.0, zs=0.5, cmd_hz=100.0):
        self.dt, self.w, self.z = dt, 2 * math.pi * fs, zs
        self.nd = max(1, int(round(Td / dt)))
        self.buf = [np.array(q0, float)] * self.nd
        self.q = np.array(q0, float).copy(); self.dq = np.zeros(len(q0))
        self.hold = np.array(q0, float).copy(); self.k = 0; self.every = max(1, int(round(1.0 / cmd_hz / dt)))

    def step(self, q_cmd):
        if self.k % self.every == 0:
            self.hold = np.array(q_cmd, float)
        self.k += 1
        self.buf.append(self.hold); r = self.buf.pop(0)
        ddq = self.w * self.w * (r - self.q) - 2 * self.z * self.w * self.dq
        self.dq = self.dq + ddq * self.dt
        self.q = self.q + self.dq * self.dt
        return self.q, self.dq


def shaper(kind, f, zeta):
    """입력 성형기 임펄스열 [(크기, 시각)] — piper_ws tools/vib_probe.py와 같은 식."""
    if kind == "none":
        return [(1.0, 0.0)]
    r = math.sqrt(1 - zeta * zeta); K = math.exp(-zeta * math.pi / r); half = 0.5 / (f * r)
    amp = [1.0, K] if kind == "zv" else [1.0, 2 * K, K * K]
    return [(x / sum(amp), i * half) for i, x in enumerate(amp)]


def band(x, fs, lo=5.0, hi=10.0):
    """실기 분석(vib_stop_report)과 같은 FFT 벽돌 필터."""
    x = np.asarray(x, float)
    n = len(x); tt = np.arange(n)
    x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n)


def stop_metrics(t, gyro, t0, win=2.0, fs=1000.0):
    """정지 시각 t0부터 win초 창: 5–10 Hz rms, 창 최대(20 Hz 아래), 감쇠 사인 맞춤 없이 FFT 봉우리 주파수."""
    m = (t >= t0) & (t < t0 + win)
    g = np.asarray(gyro)[m]
    b = band(g, fs)
    X = np.abs(np.fft.rfft(b * np.hanning(len(b)), 8 * len(b))); f = np.fft.rfftfreq(8 * len(b), 1 / fs)
    lp = band(g, fs, 0.0, 20.0)
    return dict(rms=float(np.sqrt(np.mean(b ** 2))), peak=float(np.max(np.abs(lp))), f=float(f[np.argmax(X)]))


def stop_metrics3(t, G, t0, win=2.0, fs=1000.0):
    """세 축 자이로 G(n x 3: 비틀림, 끄덕임, 좌우). 끄덕임 5–10 Hz rms(실기 지표)와, 세 축 2–20 Hz 벡터 rms·축별 rms·벡터 최대·가장 큰 축의 봉우리 주파수."""
    m = (t >= t0) & (t < t0 + win)
    g = np.asarray(G)[m]
    B = np.stack([band(g[:, k], fs, 2.0, 20.0) for k in range(3)], axis=1)
    ax_rms = np.sqrt(np.mean(B ** 2, axis=0)); k = int(np.argmax(ax_rms))
    X = np.abs(np.fft.rfft(B[:, k] * np.hanning(len(B)), 8 * len(B))); f = np.fft.rfftfreq(8 * len(B), 1 / fs)
    return dict(pitch_rms=float(np.sqrt(np.mean(band(g[:, 1], fs) ** 2))), vec_rms=float(np.sqrt(np.sum(ax_rms ** 2))),
                roll_rms=float(ax_rms[0]), pitch_w_rms=float(ax_rms[1]), yaw_rms=float(ax_rms[2]),
                vec_peak=float(np.max(np.linalg.norm(B, axis=1))), f_dom=float(f[np.argmax(X)]), axis="비틀림 끄덕임 좌우".split()[k])
