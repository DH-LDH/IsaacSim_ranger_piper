# PiPER 실물 이식 — Jetson Orin Nano 인수인계

> 이 문서는 Jetson에서 작업할 세션(사람이든 Claude Code든)에게 그대로 프롬프트로
> 붙여넣어도 되도록 작성했다. 이 저장소(`IsaacSim_ranger_piper`)는 지금까지 **Isaac
> Sim 시뮬레이션 PC에서** 개발됐고, 실물 PiPER 구동은 여기(Jetson Orin Nano)에서
> 처음 진행한다. 아래 내용을 순서대로 따라가면 된다.

## 0. 지금 상태 요약

- 5개 sim 노드(`plant_pkg`/`vision_pkg`/`control_pkg`/`hardware_pkg`/`common_pkg`)는
  전부 Isaac Sim 전용이며 **무수정** — 시뮬레이션은 이 저장소가 원래 하던 그대로 돌아간다.
- 새 패키지 `piper_hw_pkg`가 실물 브릿지 역할을 한다: CAN으로 팔을 직접 구동하는
  `piper_driver_node`, 그리퍼 폐루프 제어 `piper_gripper_node`, 손목캠 소스
  `piper_eih_camera_node`, AMR이 없어 대신 lock 신호를 쏴주는 `piper_fake_amr_node`.
- `control_pkg/arm_node.py`(파지 시퀀스 상태머신)와 `vision_pkg/vision_node.py`(ArUco
  검출)는 **거의 무수정** — 순수 ROS2 토픽 인터페이스로 짜여 있어서 그대로 재사용된다.
  단 `arm_node.py`에 파라미터 하나(`use_marker_place`, 기본값 False=sim 그대로)를
  추가해 실물에서는 이 값을 `true`로 켜 "두 번째 아르코 마커로 place 위치 인식" 경로를
  쓰도록 했다.
- **2026-09-15, Jetson에서 `really_enable:=false`로 첫 연결 테스트 완료.** `/joint_states`
  60Hz, `/arm/ee_pose_body`, `/piper/gripper_feedback` 전부 실측값 정상 발행 확인.
  이 과정에서 나온 버그 2건 수정함(둘 다 이 저장소에 반영돼 커밋 대상):
  `piper_driver_node.py`의 `GetArmStatus().arm_status`는 상태값이 아니라 한 겹 더 감싼
  객체라 `.arm_status.arm_status`로 접근해야 함(고침). `vision_node.py`의 `_make_detector()`가
  구버전 OpenCV(Jetson 기본 4.5.4)에서 무조건 죽던 것도 고침(신/구 API 분기 안으로 이동).
  `really_enable:=true` 실제 모션 검증은 아직 안 함.
- **2026-09-17, `really_enable:=true`로 그리퍼만 고립 테스트하려다 팔이 실제로 움직임.**
  원인: `arm_phase` 기본값 `"wait"`가 `JOINT_HOLD_PHASES`에 포함돼 있어서, `arm_node` 없이
  `piper_driver_node`만 띄워도 매 틱마다 `SEARCH_Q`로 `JointCtrl`이 나감. `enable_arm_motion`
  파라미터(기본값 True) 추가해서 고립 테스트 시 `-p enable_arm_motion:=false`로 끄면 팔은
  안 움직이고 그리퍼 명령(`GripperCtrl`)만 통과되게 고침.
- **2026-09-17, 그리퍼 실물 검증 중 발견한 필수 초기화 2건(`piper_driver_node.py`의
  `EnablePiper()` 직후에 이미 반영해둠) — 이거 없이 순정 `piper_sdk`만으로 테스트해도 똑같이
  막힘, 우리 코드 버그 아니었음:**
  1. `EnablePiper()`로 축별 enable만 해서는 실제 모션이 하나도 안 나감 — `MotionCtrl_2(0x01,
     move_mode, speed, 0)`로 `ctrl_mode`를 "CAN 명령 제어 모드(0x01)"로 올려야
     `JointCtrl`/`EndPoseCtrl`/`GripperCtrl`이 전부 실행됨(안 올리면 enable 상태 피드백은
     정상으로 보이는데 명령은 전부 씹힘 — status_code로는 구분 안 됨).
  2. 그리퍼는 추가로 `GripperTeachingPendantParamConfig(100, 70, 1)`(최대 행정 70mm 설정)를
     한 번 안 해주면 실측 최대 개구부가 약 49mm로 줄어든 채 동작함(70mm 명령해도 거기서
     멈춤, 토크는 한계치 안 걸림 — 손으로는 더 벌어지는 것도 확인함, 즉 기계적 한계가
     아니라 펌웨어 설정 문제였음). 영점(`set_zero=0xAE`)은 이 문제와 무관— 없어도
     동작 자체는 됨.
- **2026-09-18, 실물 파지 성공.** 원인은 손목캠 장착 각도(pitch) 12.5° 오차 + 좌표계
  기준점 상수 14mm 불일치 두 가지였다. 캘리브레이션 값은 이미 기본값에 반영돼 있어
  `ros2 launch piper_hw_pkg piper_real.launch.py really_enable:=true move_spd_rate_ctrl:=5
  step_confirm:=true`만으로 동작한다. 자세한 내용은 4절 마지막, 재발 방지 지침은 6절.
- **2026-09-20, pick→grip→lift 반복 성공 상태를 커밋/태그로 고정**(`real-pick-lift-ok`).
  되돌릴 때 `git checkout real-pick-lift-ok -- .`. 이어서 (1) 파지 판정을 그리퍼
  스트로크 기반으로 교체, (2) 손목캠 재투영 게이트 추가, (3) place 경로를 마커 기준
  오프셋 방식으로 정리했다 — 5절 참고. place는 아직 **실기 미검증**이다.
- **2026-09-17, `grippers_effort`가 닫는 방향(저항) 쪽으로는 음수로 나옴**(여는 쪽은 양수로
  확인함) — `piper_gripper_node.py`의 `_on_feedback`이 부호 신경 안 쓰고 크기(`abs`)만
  쓰도록 고침. 안 고쳤을 땐 `CONTACT_F_MIN` 비교(`F_con>=0.3`)가 음수라 항상 실패해서
  실제로 세게 잡고 있어도 "접촉 미감지"로 뜨고, SMC도 계속 더 닫으라고만 명령해서
  펌웨어 토크 한계(2.0N·m)까지 밀어붙였음.

## 1. 사전 준비물 (Jetson 쪽)

```bash
pip3 install piper_sdk python-can
sudo apt install can-utils ethtool
```

- ROS2가 이미 깔려 있어야 한다(이 저장소는 Jazzy 기준으로 만들어졌지만, 코드 자체는
  ROS2 배포판에 크게 의존하지 않는다 — Jetson에 깔린 배포판 그대로 써도 될 것).
- `robot_state_publisher`, `xacro` 패키지: `sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro`
- **로봇 설명(URDF) 패키지가 하나 더 필요하다** — 이 저장소엔 안 들어있음(용량/라이선스
  때문에 벤더 원본을 그대로 참조하는 쪽을 택함):
  ```bash
  git clone -b humble https://github.com/agilexrobotics/piper_ros.git /tmp/piper_ros_src
  cp -r /tmp/piper_ros_src/src/piper_description <워크스페이스>/src/
  ```
  (`humble` 브랜치지만 URDF/mesh만 쓰는 순수 description 패키지라 다른 ROS2 배포판에서도
  그대로 빌드된다.)

## 2. 클론 & 빌드

```bash
git clone https://github.com/DH-LDH/IsaacSim_ranger_piper.git ~/ros2_ws_ranger_piper
cd ~/ros2_ws_ranger_piper
# 위 1번에서 받은 piper_description을 src/ 밑에 넣기
colcon build --symlink-install
source install/setup.bash
```

`piper_description/urdf/piper_description.xacro`의 `<gazebo>...</gazebo>` 블록(gazebo_ros2_control 플러그인)은 `piper_gazebo` 패키지가 없으면 xacro 처리가 실패하니 통째로 지울 것 — 실물 launch엔 필요 없는 블록.

## 3. CAN 활성화

PiPER는 USB-CAN 동글(gs_usb/candleLight 계열, lsusb에 `1d50:606f`)로 연결한다 — Jetson 보드 내장 CAN(mttcan, 보통 can0)이 아니다. NVIDIA L4T 커널엔 `gs_usb.ko`가 기본으로 없어서 직접 빌드해야 한다(한 번 하면 재부팅 전까지 유지):

```bash
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -sL -o gs_usb.c https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c
printf 'obj-m += gs_usb.o\nKDIR := /lib/modules/$(shell uname -r)/build\nall:\n\t$(MAKE) -C $(KDIR) M=$(PWD) modules\n' > Makefile
make
sudo insmod gs_usb.ko   # 재부팅마다 다시 해줘야 함(영구화 안 했으면)
```

**주의**: `can0`/`can1` 이름은 Jetson 내장 mttcan과 USB 동글 중 어느 쪽 드라이버가 먼저
로드되냐에 따라 **부팅마다 뒤바뀔 수 있다**(실제로 한 번 바뀌는 걸 확인함 — mttcan이
`can1`을 차지하고 USB 동글이 `can0`가 된 적 있음). 그래서 udev 규칙으로 USB 동글을
`can_piper`라는 고정 이름에 묶어뒀다:

```bash
# /etc/udev/rules.d/80-can-piper.rules (시리얼 넘버로 이 동글만 특정)
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="004900345443570F20393433", NAME="can_piper"
```

다른 동글로 교체하면 시리얼 값을 `udevadm info -a -p /sys/class/net/<현재이름> | grep ATTRS{serial}`로
다시 확인해서 갱신할 것. 규칙 적용 후:

```bash
sudo udevadm control --reload-rules
# 동글을 뽑았다 다시 꽂거나 재부팅해야 새 이름이 적용됨
ip -br link show type can   # can_piper가 보여야 함
sudo ip link set can_piper down
sudo ip link set can_piper type can bitrate 1000000
sudo ip link set can_piper up
```

`piper_real.launch.py`/`piper_driver_node.py` 기본 `can_name`은 `can_piper`로 맞춰뒀다.

## 4. 첫 실행 — 절대 처음부터 `really_enable:=true`로 켜지 말 것

**1단계: 연결·피드백만 확인 (모션 명령 없음)**

```bash
ros2 launch piper_hw_pkg piper_real.launch.py really_enable:=false
```

- `piper_driver_node`가 CAN에 붙어 `ConnectPort()`까지는 하되 `EnableArm`/모션 명령은
  전혀 안 보낸다(기본 안전장치, 코드 파라미터 `really_enable` 기본값 False).
- 이 상태에서 확인할 것:
  ```bash
  ros2 topic echo /arm/ee_pose_body      # 팔의 실제 현재 위치가 나오는지
  ros2 topic echo /joint_states          # 실제 관절각이 나오는지
  ros2 topic echo /piper/gripper_feedback
  ```
  값이 전혀 안 나오거나 이상하면(0 고정 등) CAN 연결/펌웨어 버전 문제일 가능성이 큼 —
  이 단계에서 반드시 잡고 넘어갈 것.

**2단계: `piper_driver_node.py`의 `_quat_to_rpy_deg()` 변환 검증 — ✅ 2026-09-17 완료**

- `piper_driver_node`를 끈 상태(동시에 CAN에 명령 보내는 프로세스가 둘이면 서로
  덮어써서 헛갈림 — 실측으로 확인함)에서 `piper_sdk`로 직접 `EndPoseCtrl`을 보내
  pitch만 +10° 바꿔봄. 피드백이 `(RX+180, 180-RY, RZ+180)` 형태(오일러각 180도
  대칭 이중해)로 나왔는데, 이게 정확히 `R=Rz(yaw)@Ry(pitch)@Rx(roll)` 가정에서
  나오는 이중해 패턴과 오차 0.02° 이내로 일치 — 변환식 맞는 걸로 확인. 코드의
  "★검증 필요" 표시 제거함.

**3단계: 그리퍼 물리 스펙 확정**

- `piper_gripper_node.py` 상단에 `GRIPPER_STROKE_MAX_M`(자리표시자 0.07m),
  `SMC_F_TARGET`/`SMC_PHI`/`SMC_K_A`(전부 N·m 단위 자리표시자) — 실제 그리퍼로
  빈손 개폐/약한 물체 파지를 몇 번 해보면서 `grippers_effort` 피드백 값의 실제
  범위를 관찰한 뒤 재설정할 것. sim의 N 단위 값을 그대로 가져올 근거가 없다.

**4단계: 카메라·마커 세팅**

- `piper_eih_camera_node.py`의 `K_PLACEHOLDER`(초점거리 500px 가정) — 실제 카메라로
  체스보드 캘리브레이션(OpenCV `cv2.calibrateCamera`) 해서 갱신할 것. 지금 값 그대로면
  ArUco pose(특히 z, 거리)가 부정확하다.
- 손목캠 TCP 오프셋은 `piper_eih_camera_node`의 ROS 파라미터(`cam_tcp_offset_x/y/z`,
  `cam_tcp_offset_pitch_deg`)로 관리한다(예전엔 xacro에 박아뒀었는데 튜닝 편의상 옮김,
  `link6→eih_cam` static TF를 코드에서 직접 발행). 기본값은 2026-09-15 실측+마커
  교차검증값(x=-0.085, y=0, z=0.026) + 2026-09-17 전자각도계 실측 pitch=40deg(그리퍼
  끝단이 시야 중앙 근처에 오는 것도 마커로 재검증함). `link1..link6` 체인 자체는
  여전히 `piper_description.xacro`+`robot_state_publisher`가 joint_states로 갱신한다.
- `common_pkg/step22_common.py`의 `PLACE_CENTER_BODY_GUESS`(자리표시자 `(0.20, KEEP_DIST,
  OBJ_CENTER_BODY[2])`) — 두 번째 마커(place 목표)를 대략 어디에 둘지 실제 배치에 맞게
  갱신할 것. `place_hover`가 이 값으로 먼저 접근한 뒤 마커 재검출로 정밀 보정한다.
- **2026-09-17 발견: `arm_node.py`의 `_grasp_point()`/`_place_point()`/detect 게이트가
  `BODY_LINK_WORLD_Z`(=0.277, sim에서 body_link가 world보다 높은 만큼)를 더하는데, 실물은
  `body_link=world=팔 베이스`라 이 오프셋이 없어야 함 — 그대로 두면 Z가 27.7cm 어긋난
  목표로 EndPoseCtrl이 나감.** `arm_node.py`에 `body_link_world_z` 파라미터 추가(기본값은
  sim과 동일하게 0.277 유지, `piper_real.launch.py`에서 0.0으로 오버라이드)로 고침.
- **2026-09-17 발견(더 큰 원인): `_on_lock()`의 hover 목표 계산(`OBJ_CENTER_BODY - HOVER_APPROACH_DIR×...`)은
  위 변환조차 안 하고 `OBJ_CENTER_BODY`(world 절대높이 상수)를 그대로 썼음** — 실측(팔 베이스
  지면 38cm, 물체 지면 25cm → body_link 기준 -0.13m)으로 역산하면 원래 목표(z=0.508m, 팔
  베이스보다 훨씬 위)가 올바른 목표(z=0.143m)보다 36cm나 높았던 것 — 첫 hover 시도가
  `TARGET_POS_EXCEEDS_LIMIT`로 반복 실패한 주 원인으로 추정됨. `obj_expected_x/y/z_body`
  파라미터 3개 추가해서 hover 목표/detect 게이트가 이 값을 직접 쓰도록 고침(sim 기본값은
  기존 계산과 수치 동일, 실물은 `piper_real.launch.py`에서 실측값(-0.13, 0.40)으로
  오버라이드). **아직 실물에서 재검증은 안 함** — 다음 hover 재시도 때 이 좌표로 확인할 것.
- **2026-09-17 확인 완료(가장 근본 원인): body_link의 X/Y축이 통째로 sim 관례랑 어긋나
  있었음.** sim은 `ARM_BASE_YAW_DEG=90`(팔이 AMR 몸체 기준 90도 돌아서 장착)이라
  body_link의 Y축이 전진방향인데, `piper_fake_amr_node.py`가 `body_link↔world`를
  identity로 발행하고 있어서 실물에선 이 90도가 반영이 안 되고 있었음. 추가로
  `vision_node.py`의 `_on_eih_image`가 solvePnP 결과에 `diag(1,-1,-1)`(sim의 USD
  카메라 y/z 부호 보정용)을 곱하는데, 실물 `eih_cam`(OpenCV 관례로 직접 정의)엔 이게
  불필요해서 이중으로 틀어져 있었음. 두 가지 다 고치고 HOVER_Q_VERIFIED 실측 EndPose와
  정방향 기구학 역산으로 교차검증(오차 <1cm)해서 확인함:
  - `piper_fake_amr_node.py`: `body_link→world` TF 회전을 identity 대신 `ARM_BASE_YAW_DEG`
    반영하도록 수정.
  - `vision_node.py`: `eih_axis_flip` 파라미터 추가(기본 True=sim 그대로, 실물은
    `piper_real.launch.py`에서 `false`).
  - `piper_driver_node.py`: `arm_node`가 body_link 관례로 계산한 `cartesian_target`
    XY를 `EndPoseCtrl`에 넣기 전에 native로, `GetArmEndPoseMsgs()` 피드백을
    `/arm/ee_pose_body`에 싣기 전에 body_link로 변환하는 `_body_to_native_xy`/
    `_native_to_body_xy` 추가(`_tick()`/`_publish_feedback()`에 적용).
  **이 세 가지가 다 맞아야 hover/detect/grasp 전체가 앞뒤가 맞는다** — 하나라도 빠지면
  이번처럼 목표가 90도 돌아간 방향으로 나가서 fault/낙하로 이어질 수 있음.
- **2026-09-17 추가 발견: yaw(RZ)도 같은 회전보정이 빠져있었음.** 위 세 가지를 고치고
  `step_confirm` 모드로 hover까지 실행했더니 위치는 HOVER_Q_VERIFIED 실측값과 거의
  일치했는데(native X=0.40 근처) 그 직후 `TARGET_POS_EXCEEDS_LIMIT`가 뜸 — hover의
  계산된 yaw(-90°)가 실측 hover 자세의 RZ(180°)랑 90도(=`ARM_BASE_YAW_DEG`) 차이나서,
  위치는 맞는 곳으로 가면서 손목이 무리하게 yaw를 맞추려다 한계 초과한 것으로 추정.
  `piper_driver_node.py`의 `_tick()`에서 `rz = rz_body - ARM_BASE_YAW_DEG`로 보정
  추가함. **아직 재검증 전** — 다음 hover 재시도 때 fault 없이 도달하는지 확인할 것.

- **2026-09-17 저녁 — 카메라 광축 롤(roll) 누락 발견/수정.** eih_cam TF가 link6 축과 나란하다고
  가정했는데 실제 카메라는 광축 기준 90° 돌아가 장착돼 있었음. 그래서 이미지 좌우가 로봇
  전후로 해석돼 좌우 8cm씩 엉뚱하게 움직였음. `cam_tcp_offset_roll_deg`(기본 -90) 추가로
  해결 — 파지 좌우오차 81.7mm → **15.3mm**, detect mk x는 +0.089 → **-0.006**(거의 정중앙).
- **2026-09-17 저녁 — 그리퍼가 반쯤 닫힌 채로 파지를 시작하던 문제.** `SMC_ALPHA_0=0.5`면
  개구부 30mm에서 시작해 물체를 감싸지 못함 → **0.0(완전 개방)으로 변경**. 추가로
  `piper_gripper_node`가 기동 3초 뒤 자동으로 한 번 열도록 함(이전 파지의 닫힌 상태로 시작 방지).
- **2026-09-17 저녁 — 남은 블로커: 하강 도달 한계.** 전방 0.40m + 그리퍼 수직 자세에서
  **z≈0.15 아래로는 팔이 안 내려감**(4회 재현: 0.148 / 0.151 / 0.159에서 정지, 0.137 명령 시
  `TARGET_POS_EXCEEDS_LIMIT`). 파지 목표가 그보다 낮게 잡히면 도달 실패 → 헛집음.
  다음 시도: **물체를 7~10cm 높이거나 베이스 쪽으로 10cm 당겨서** 파지점이 z≈0.15 이상이
  되게 할 것. `ee_grip_offset` 파라미터(기본 0.135, link6→파지점 거리)도 이 높이에 직접
  영향을 주니 같이 조정할 것 — 0.25로 올리면 목표가 13cm 위로 올라감.
- 2026-09-17 저녁 기준 각 축 오차: 좌우 15mm / 전후 23mm / **높이 126mm(위 한계 때문)**.


- **2026-09-18 — 파지 실패의 근본 원인 2건 확정, 실물 파지 성공.**
  - **원인 ① 손목캠 외부파라미터 pitch 오차 (주원인).** `cam_tcp_offset_pitch_deg`가
    40.0(전자각도계 실측)이었는데 실제는 **27.5°**. 회전 오차는 *거리에 비례하는* 위치
    오차(거리×sin12.5°≈22%)를 만든다 — 50cm에서 11cm. 더 나쁜 건 오차가 카메라 자세에
    의존한다는 점으로, 하강 중 재검출마다 목표가 움직이고 팔이 그걸 쫓으면서 **양의
    피드백**이 걸려 목표가 150mm 발산했다(팔은 74mm 뒤처진 채 영원히 수렴 못 함).
  - **원인 ② 좌표계 기준점 불일치 (상수 14mm).** 비전은 URDF `link6`(FK)를, 모션 명령은
    펌웨어 `EndPoseCtrl`의 기준점을 쓰는데 두 점이 툴축 방향으로 어긋나 있다. `ee_grip_offset`
    **0.135 → 0.121**로 흡수. 물체 높이를 바꿔 반복 측정해 거리·자세 무관한 상수임을 확인했다.
  - **진단 방법(핵심).** *팔을 정지시킨 상태의 관측만* 썼다 — 움직이는 데이터는 추종
    오차와 캘리브레이션 오차가 섞여 구분이 안 된다. ①정지 16샘플로 카메라→몸체 변환을
    강체정합(잔차 0.5mm) → 소프트웨어 버그 배제, ②기대치와의 오차 벡터가 광축과 **87°**
    (수직)임을 확인 → 거리/스케일 오차가 아닌 **지향 오차**로 범위 축소, ③마커 고정 +
    팔 자세만 9회 변경해 자세 간 편차를 최소화하는 외부파라미터 최적화(`eih_cam_calib.py`)
    → 편차 10.4mm → 3.8mm. **마커의 실제 위치를 몰라도 되는 방법**이라 별도 계측 장비가 필요 없다.
  - **같이 고친 소프트웨어 결함 4건:**
    1. `arm_node.py` PRE 재검출 수용 판정이 *직전값* 대비(`|g2-ml_grasp|<=60mm`)라
       2mm짜리 보정 787회가 150mm로 무한 누적됐다 → **anchor(최초검출) 대비**로 변경.
       이것 때문에 grasp 단계에서는 반대로 448/448 전부 기각됐다.
    2. 물리도달 대기 상한 90/120스텝(1.5/2초)은 `move_spd_rate_ctrl:=5`에 너무 짧아
       팔이 도착하기 전에 "그냥 진행"했다 → **300스텝(5초)**.
    3. 그리퍼 닫기 명령이 grasp 단계 *끝*에서 나가 `step_confirm`의 grip 게이트보다
       먼저 닫혔다 → **grip 단계 첫 틱으로 이동**. 이제 Enter 전엔 안 닫힌다.
    4. `set_arm_joint.py`가 publish 직후 바로 종료해서 디스커버리가 늦으면 명령이 통째로
       사라졌다 → 구독자와 매칭될 때까지 대기 후 publish.
  - **추가된 도구/인자:**
    - `eih_cam_calib.py` — 손목캠 외부파라미터 캘리브레이션. 샘플은 `eih_calib_samples.npz`로
      저장되고 `--load`로 재분석된다. **거리를 150/250/400mm로 흩어야** pitch·병진·마커크기가
      분리된다(거리가 한 곳에 몰리면 scale이 퇴화해로 빠진다 — 경고가 뜬다).
    - `vision_node`의 `eih_debug_view` → `/vision/eih_debug_image`. **파이프라인이 실제로
      쓰는 검출/포즈를 그대로 그린다**(별도 `eih_marker_debug_node`는 자체 검출이라 값이 다를 수 있음).
      런치가 `rqt_image_view`까지 같이 띄운다(`debug_view:=false`로 끔).
    - `[진단-eih]` 로그 2초마다 — `d=`(렌즈~마커 거리)와 발행되는 `body=`가 한 줄에 나온다.
    - `step_confirm.py`가 `/arm/step_wait`로 **어느 단계를 기다리는지** 표시.
    - 런치 인자: `arm`(false면 arm_node 제외 → `set_arm_joint.py`로 관절 직접 지령 가능),
      `cam_tcp_offset_*` 5종, `ee_grip_offset`, `approach_dist`, `grasp_depth_extra`,
      `grasp_z_below_anchor_max`, `pre_redetect`, `grasp_eih_track`, `debug_view`.
  - **★ 위 2026-09-17 저녁 항목의 "`ee_grip_offset`을 0.25로 올려라"는 조언은 폐기한다.**
    그 값은 목표 높이를 밀어 올리는 임시방편이었고, 실제 원인은 pitch 오차였다. 현재
    확정값은 **0.121**이다. 같은 항목의 "z≈0.15 아래로 안 내려감" 블로커는 현재 설정에서
    파지가 성공하므로 실질적으로 해소됐으나, 도달 한계 자체를 따로 규명하지는 않았다.

### 2026-09-20 실기: 전체 pick&place 완주 + 그 과정에서 잡은 것들

**① 팔이 명령을 못 따라가던 문제 (가장 컸던 것).** `piper_driver_node`가 목표가 안
바뀌어도 `EndPoseCtrl`을 **매 틱(60Hz) 재전송**하고 있었다. 5% 속도에서는 가속 램프가
16ms마다 리셋돼 팔이 사실상 안 움직인다. 실측: `grasp` 28초 동안 손목캠 거리가
285→280mm(5mm)만 줄었고, 명령이 끊기는 `grip` 단계에 들어가자 8초에 133mm 이동했다.
→ **값이 바뀐 경우에만 전송**하도록 고침. 5초마다 `모션명령 전송 N회 / 중복생략 M회`
로그가 나온다(place_descend에서 474 / 4695).

- 이것 때문에 `pre`/`grasp`가 매번 `★ 물리도달 실패`로 빠졌고, 그게 "덜 문다"의 정체였다.
  `step_confirm`으로 사람이 기다리는 동안 팔이 뒤늦게 따라가서 "될 때도 있던" 것이다.
- 도달 대기 상한도 5초로는 부족하다 → `arrive_max_wait`(기본 300, 실물 런치 900=15초).
  실측에서 스트림 종료 후 목표 도달까지 300스텝이 더 걸렸다.
- `hover`에는 물리 도달 확인이 아예 없었다(시간 기준으로만 전이) → 추가. 여기서 뒤처지면
  `detect`/`pre`/`grasp`가 통째로 밀린다(실측: grasp 진입 시 이미 134mm 뒤처짐).
- `lift`, `place_retreat`도 같은 이유로 확인 추가(`_wait_physical_arrive()`).

**② 중복생략 캐시는 단계가 바뀌면 반드시 비울 것.** 안 비웠더니 기동 시 `wait`에서 보낸
`SEARCH_Q`가 캐시에 남아, `place_home`의 같은 `SEARCH_Q`가 "중복"으로 걸러져 **명령이 한
번도 안 나갔다**(전송 0회, 팔이 복귀 안 함). `_on_arm_status()`의 단계 전환에서 비운다.

**③ 그리퍼 여는 명령이 `place_release` 게이트보다 먼저 나갔다.** 2026-09-18에 고친
"닫기 명령이 `grip` 게이트보다 먼저" 와 같은 버그의 릴리즈 판이다 — `place_release`
첫 틱으로 옮겼다. 이제 `step_confirm`이 켜져 있으면 **Enter 전엔 물체를 놓지 않는다.**

**④ 손목캠 재투영 게이트를 조일 때 주의.** `rel`은 거리에 따라 크게 변한다 — 실측:

| 손목캠~마커 거리 | rep | rel |
|---|---|---|
| 280mm (grasp 초반) | 0.05~0.35px | 0.1~0.7% |
| 137mm (파지 직전) | 6.0px | 7.4% |
| 121mm (물체를 쥔 채) | 8.3px | 9.1% |

**근접에서 7~9%까지 올라가므로 `eih_reproj_max_rel`을 3~5%로 조이면 파지 직전 구간의
추종이 통째로 끊긴다.** 현재 기본값 0.20은 그래서 그대로 두는 게 맞다. 조일 거면
0.12~0.15가 상한이고, 그 전에 왜 근접에서 rel이 커지는지(마커 판 평면도, 모션 블러,
코너 정밀화 윈도 5px가 큰 마커에 부족한 것)부터 볼 것.

**⑤ place 중에는 쥔 물체의 픽 마커(ID0)가 손목캠에 계속 잡힌다**(d≈121mm 고정).
`arm_node`는 `detect/pre/grasp`에서만 이 토픽을 쓰므로 지금은 무해하지만, 물체를 쥔
채로 다시 `hover`/`detect`로 들어가는 흐름을 만들면 **자기가 든 물체를 타겟으로 잡는다.**

### place(선반에 놓기) — 2026-09-20 정리, **실기 완주 확인**

물체를 쥐고 있으면 그 물체가 손목캠 시야를 가리므로, 마커를 계속 보면서 내려가는
방식은 근접에서 무너진다. 그래서 다음 세 가지로 바꿨다.

1. **놓는 위치를 마커에서 비켜 잡는다.** 선반 place 마커 중심에서 **35mm 비킨 지점**에
   놓는다(`place_point_mode:=marker_inset`, `place_inset_m:=0.035`). 마커 위에 그대로
   올리면 마커가 덮여 재검출이 끊긴다. 방향은 `place_inset_mode`로 고른다:

   | 모드 | 방향 | 쓸 때 |
   |---|---|---|
   | `marker_y` (기본) | 마커 자신의 **−Y** | 마커를 비스듬히 붙여도 "마커 기준 앞쪽"이 따라간다 |
   | `body_y` | body −Y (팔 베이스 정면) | 마커 방향과 무관하게 항상 같은 쪽 |
   | `radial` | 원점→마커의 반대 | 종전 동작 |

   **마커의 +Y는 인쇄된 마커 이미지의 "위쪽" 모서리를 향한다**(vision의 코너 모델이
   좌상-우상-우하-좌하 순서라서). 즉 `marker_y`는 **마커 아래쪽 모서리 방향**으로 비킨다 —
   반대로 나가면 마커를 180° 돌려 붙이거나 `body_y`를 쓰면 된다.
   `place_detect` 로그에 세 후보 방향이 같이 찍힌다:
   `방향후보 marker_y=(...) Δ0.0°  body_y=(...) Δ13.1°  radial=(...) Δ2.2°` —
   **`Δ0.0°`인 것이 실제 적용된 방향**이다. 주의: 마커를 로봇 정면에 반듯이 붙이면
   `marker_y`와 `radial`이 2°(35mm에서 1.4mm) 차이로 겹쳐서 **눈으로는 구분이 안 된다**
   (2026-09-20 실측: 베이스→마커 방위각 100.9° vs 마커 yaw 103.1°). 모드가 제대로
   먹는지 확인하려면 마커를 물리적으로 90° 돌려서 오프셋이 같이 도는지 볼 것. 이 방향은 vision이 새로 발행하는 마커 yaw에서 나온다
   (`pack_eih_marker`의 5번째 필드 — 옛 4개짜리 메시지도 그대로 받는다).
2. **릴리즈 높이는 물체 바닥 기준으로 계산한다.** 손끝은 물체 윗면보다
   `grasp_depth_extra`만큼 아래를 쥐고 있으므로(파지점 정의), 손끝~물체 바닥 거리는
   `obj_height_m - grasp_depth_extra`다. 거기에 `place_release_gap`(기본 5mm)을 더해
   "물체 바닥이 선반면에서 5mm 뜬" 높이에서 연다. 현재 값(높이 16.36mm, 깊이 8mm)이면
   **손끝은 마커면 +13.4mm**.
3. **최초 검출 후에는 맹목 하강한다**(`place_freeze_after_detect:=true`). 부분 오클루전
   상태의 PnP는 코너 일부만 보고 엉뚱한 자세를 내므로, 가려지기 전에 잡은 놓는점을
   고정하는 편이 낫다. 릴리즈 뒤에는 수직으로 50mm 빠진 다음(`place_retreat`) 초기
   자세로 복귀한다 — 바로 `SEARCH_Q`로 튀면 손가락이 방금 놓은 물체를 친다.

**실행 전 반드시 맞춰야 하는 것:**

- `place_expected_{x,y,z}_body` — 선반 마커의 대략 위치(팔 베이스 기준). 선반이 픽
  타겟과 같은 선반이라 **기본값은 픽 hover 위치(`obj_expected_*`)를 그대로 물려받는다.**
  hover가 일단 여기로 가서 마커를 찾는 출발점일 뿐이라 화각 안에 들어올 정도면 된다.
- `eih_place_marker_id`(기본 **3**) / `eih_place_marker_size_m`(기본 0.034) — 선반에 붙인
  실제 마커와 일치시킬 것. 크기가 틀리면 solvePnP 거리가 그 비율만큼 통째로 틀어진다
  (예전 코드는 sim 기본 20mm로 풀어서 0.59배로 가깝게 나왔다 — 이번에 고침).
- `place_pitch_deg`(기본 0.0) — sim의 30°는 sim 선반 높이 전용이었다. 실물은 픽과 같은
  수직 접근이 기본이고, **`arm_node`와 `piper_driver_node`에 같은 값이 들어가야 한다**
  (런치가 둘 다에 넘겨준다). IK가 안 풀리면 여기부터 만질 것.
- 첫 시도는 `step_confirm:=true`로 단계마다 끊어서 볼 것.

## 5. 알려진 한계 (지금 범위에서 일부러 안 건드린 것)

- ~~파지 성공 판정이 차체캠 전제~~ → **2026-09-20 해결.** `verify_mode` 파라미터로
  갈라둔다(실물 기본 `gripper`). `_verify_by_gripper()`가 그리퍼 개구부(스트로크)와
  접촉으로 판정한다 — 물체를 물고 있으면 손가락이 물체 폭에서 멈추고, 헛집으면 0 근처로
  닫힌다. 접촉력만으로는 이 둘이 구분이 안 된다(빈손이어도 손가락끼리 눌리면 접촉으로
  잡힘)는 게 핵심이라 스트로크가 주 근거다. 차체캠 로직(`_verify_by_chassis_cam`)은
  지우지 않고 그대로 뒀다 — 차체캠을 달면 `verify_mode:=both`로 두 판정을 나란히
  찍어보고(판정은 그리퍼가 함) 일치하면 `chassis`로 돌리거나 AND로 묶으면 된다.
  - 판정 입력은 새 토픽 `/gripper/grip_state`([스트로크mm, effort, 접촉, 유지, 활성]),
    실물/sim 그리퍼 노드가 같은 계약으로 발행한다.
  - 임계값은 `obj_width_m`(런치 인자, 현재 0.01636)에서 파생 — 하한 폭×0.5, 상한 폭+10mm,
    슬립 3mm. **물체를 바꾸면 이 인자를 반드시 같이 바꿀 것.**
  - **★ 보고되는 개구부는 실제 간격이 아니다.** 2026-09-20 실기에서 16.36mm 물체를
    정상 파지했는데 `grippers_angle`이 **25.9mm**로 보고됐다(패드 두께 + 영점 미설정
    추정, 상수 오프셋 ≈9.5mm). 이걸 모르고 물체폭만으로 임계값을 잡으면 정상 파지가
    상한(26.4mm)을 0.5mm 차이로 겨우 통과한다 — 다음 런에 27mm면 **정상 파지를
    실패로 판정**한다. `grip_stroke_offset_mm`(런치 기본 9.5)로 판정선을 보고값
    스케일로 옮긴다(허용 17.7~35.9mm).
    - **재측정법: 아무것도 없이 그리퍼를 닫고 `[실접촉토크 요약]`의 개구부 값**이 곧
      이 오프셋이다. 9.5와 다르면 그 값으로 갱신할 것.
    - 판정선까지 3mm 미만으로 붙으면 `★ [주의]` 로그가 뜬다 — 뜨면 재측정 신호.
- ~~손목캠 픽 마커 경로에 재투영 게이트 없음~~ → **2026-09-20 게이트 추가.** 단, 차체캠의
  `REPROJ_MAX=3.0px`를 그대로 쓰면 안 된다 — 손목캠은 0.1~0.2m 근접이라 같은 3D 정확도라도
  마커가 화면에서 크고 px 오차가 비례해 커진다. 그래서 **마커 한 변 픽셀길이 대비 비율**
  (`eih_reproj_max_rel`)이 주 게이트이고 px 절대값(`eih_reproj_max_px`)은 명백한 쓰레기만
  거르는 상한이다. 기각은 "그 프레임 미검출"이라 팔은 직전 목표를 유지한다(안전측).
  **기본값(0.20 / 30px)은 일부러 거의 안 거른다** — `detect` 단계엔 타임아웃이 없어서
  근거 없이 조인 값이 모든 프레임을 기각하면 픽이 그 자리에서 멈춘다. 그리고 실측상
  근접(120~140mm)에서 rel이 7~9%까지 올라가므로 **3~5%로 조이면 파지 직전 추종이
  끊긴다** — 위 4절 ④의 표를 먼저 볼 것.
- AMR(Ranger Mini) 통합, 벨트 추종은 이번 범위 밖(팔 단독 pick&place).
- `piper_driver_node.py`는 종료 시 `DisableArm()`을 자동 호출하지 않는다(브레이크
  미보유 축이 있으면 무동력 낙하 위험 — 문서상 근거 불충분해 마지막 자세 유지가 기본).
  비상정지가 필요하면 `piper_sdk`의 `EmergencyStop(0x01)`을 별도로 호출할 것.
- **★안전 절차: `EnablePiper()`/`EnableArm()`을 부르는 모든 시점(최초 기동, fault 복구
  후 재인에이블, 프로세스 재시작 등) 직전에 팔을 손으로 받치고 시작할 것.** 2026-09-17
  실기에서 최소 2회 재현 확인 — 인에이블 직후 짧은 순간 토크가 안 걸려 처지는 현상이
  있음(브레이크 없는 축 추정). fault(`TARGET_POS_EXCEEDS_LIMIT` 등) 복구 시에도 동일하게
  적용됨 — `MotionCtrl_1(0x02, 복구)` 직후 반드시 손으로 받치고 진행.

## 6. 코드 수정 시 참고 (2026-09-18 디버깅에서 얻은 것)

이 절은 "다음에 또 파지가 빗나갈 때" 같은 길을 두 번 헤매지 않으려고 적어둔다.

**1. 파라미터의 성격을 구분할 것 — 오차를 설계값에 흡수시키지 말 것.**
`grasp_depth_extra`는 "물체를 얼마나 깊이 물지"라는 **설계값**(기준: 물체 높이/2)이고,
`ee_grip_offset`은 체인 전체의 **상수 편향**이다. 파지가 얕다고 `grasp_depth_extra`를
키우면 당장은 잡히지만 물체 높이가 바뀌는 순간 다시 틀어진다. 상수 편향은 반드시
`ee_grip_offset`에 넣을 것. **판별법: 물체 높이를 바꿔 두 번 재본다. 같은 오차면 상수
편향(=`ee_grip_offset`), 달라지면 비전 캘리브레이션이 아직 안 끝난 것.**

**2. `ee_grip_offset`은 물리적 손가락 길이가 아니다.** "EndPoseCtrl 명령 기준점 → 손끝"
거리이고, 펌웨어 EndPose 기준점과 URDF `link6` 원점의 불일치까지 합쳐져 있다. 그래서
URDF의 joint7 장착점(135.8mm)보다 짧은 게 정상이다 — **"값이 이상하다"고 되돌리지 말 것.**

**3. 진단은 팔을 세워두고 한다.** 움직이는 로그는 추종 오차·캘리브레이션 오차·재검출
드리프트가 전부 섞여 있어 원인 분리가 불가능하다. `really_enable:=false`로 띄우면
변환이 상수가 되어 강체정합으로 파이프라인 자체의 무결성을 먼저 검증할 수 있다.

**4. 오차 벡터와 광축의 각도를 먼저 본다.** 이 한 번의 계산이 원인 범위를 절반으로 줄인다.
- **광축과 나란함** → 거리(스케일) 오차 = 마커 크기, `fx`
- **광축과 수직** → 지향(회전) 오차 = `cam_tcp_offset_pitch_deg` / `_roll_deg`
- **거리와 무관하게 일정** → 병진 오차 = `cam_tcp_offset_x/y/z`, 또는 프레임 불일치

**5. 재검출 수용 판정은 항상 anchor(최초 검출) 대비로 짤 것.** 직전값 대비로 하면 각
보정이 임계값 아래라도 무한히 누적된다(실제로 2mm×787회 = 150mm 발산). 목표를 실시간
추종시킬 때는 "한 번에 얼마나 뛰는가"가 아니라 "출발점에서 얼마나 멀어졌는가"를 막아야 한다.

**6. `/arm/status`는 driver와의 계약이다.** `piper_driver_node`가 이 토픽의 phase로
JointCtrl/EndPoseCtrl을 고르므로, `step_confirm` 게이트 중에는 일부러 발행을 멈춰
이전 phase를 유지시킨다(안 그러면 둘 다 못 보내는 공백이 생겨 팔이 멈춤). 단계 정보를
외부에 알릴 일이 생기면 **별도 토픽**을 쓸 것(`/arm/step_wait`가 그렇게 추가됐다).

**7. 좌표계가 세 개다. 새 계산을 넣을 때 어느 것인지 명시할 것.**
- `body_link` — `vision_node`/`arm_node`가 쓰는 기준(실물에서는 팔 베이스=world)
- native — `EndPoseCtrl`의 고유 좌표. `_body_to_native_xy`로 `ARM_BASE_YAW_DEG`만큼 회전. yaw(RZ)도 같은 보정 필요
- URDF `link6` — TF/FK 기준. 손목캠이 여기 붙어 있어 비전은 이걸 탄다. **EndPose 기준점과 다르다**(위 2번)

**8. 실기 검증은 `step_confirm:=true`로.** 특히 `grip` 게이트에서 멈춘 채로 손끝과 물체
윗면의 간격을 자로 재면, 그 숫자 하나로 원인이 갈린다(1번 판별법). 그리퍼는 Enter 전엔
닫히지 않으므로 안전하게 잴 수 있다.

**9. 측정값은 화면이 아니라 로그로 남길 것.** rqt 오버레이는 눈으로 옮겨 적어야 해서
기록이 안 남는다. `[진단-eih]`처럼 주기적으로 콘솔에 찍으면 런치 로그를 그대로 붙여
나중에 역산할 수 있다 — 이번 원인 규명도 저장된 로그 16줄에서 나왔다.

**10. 캘리브레이션 샘플은 반드시 저장할 것.** 캡처는 비싸고(팔 자세 바꿔가며 수 분),
분석은 싸다. `eih_cam_calib.py`는 `eih_calib_samples.npz`로 저장하고 `--load`로 재분석한다.
한 번은 저장을 안 해서 9자세를 통째로 다시 잡아야 했다.

## 7. 연락할 파일

- 설계/좌표계/제어식 전체 설명: [PiPER 엔드이펙터 접근·제어 알고리즘](이 vault 노트,
  시뮬레이션 PC의 `/home/da/Documents/tetra/`에 있음 — 필요하면 내용만 복사해서 가져올 것)
- 코드 리뷰로 발견된 기존 sim 버그 5건: 같은 vault의 `PiPER 워크스페이스 코드 리뷰.md`
  (실물 이식과 무관, sim에만 해당하니 급하지 않음)
