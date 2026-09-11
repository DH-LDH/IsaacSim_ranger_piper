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
- **아직 실제 CAN 연결로 테스트한 적이 없다.** 시뮬레이션 PC에는 CAN 하드웨어도
  `piper_sdk`도 없어서, API 자체는 agilexrobotics의 공식 `piper_sdk`(pip) 소스코드를
  직접 pip install해 함수 시그니처/반환 객체 구조까지 확인하고 작성했지만(추측 아님),
  실제 팔에 물려서 돌려본 적은 없다. **여기서 처음 연결 테스트를 한다.**

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

## 3. CAN 활성화

```bash
cd src/piper_hw_pkg   # 또는 piper_sdk 저장소의 스크립트를 따로 받아도 됨
bash find_all_can_port.sh   # 없으면 pip show piper_sdk로 설치 경로 확인 후 그 안의 스크립트 사용
bash can_activate.sh can0 1000000
ifconfig   # can0가 보이는지 확인
```

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

**2단계: `piper_driver_node.py`의 `_quat_to_rpy_deg()` 변환이 맞는지 검증**

- 코드에 "★검증 필요"로 표시해둔 부분 — 쿼터니언을 `(RX,RY,RZ)` Euler로 바꾸는 합성
  순서(`R=Rz(yaw)@Ry(pitch)@Rx(roll)` 가정)가 실제 PiPER 펌웨어의 관례와 같은지
  문서만으로 확인 못 했다.
- 검증법: `really_enable:=true`로 딱 한 번, **팔 주변에 장애물/사람 없는 상태**에서
  낮은 속도(`move_spd_rate_ctrl` 기본 20)로 `arm_node`를 통하지 않고 `piper_sdk`를
  직접 열어 알고 있는 자세(예: 완전 수직 하강 자세) 하나를 `EndPoseCtrl`로 보내보고,
  `GetArmEndPoseMsgs()` 피드백이 기대한 자세와 맞는지 눈으로/로그로 대조.
- 안 맞으면 `piper_driver_node.py`의 `_quat_to_rpy_deg()`만 고치면 된다(호출부는
  건드릴 필요 없음).

**3단계: 그리퍼 물리 스펙 확정**

- `piper_gripper_node.py` 상단에 `GRIPPER_STROKE_MAX_M`(자리표시자 0.07m),
  `SMC_F_TARGET`/`SMC_PHI`/`SMC_K_A`(전부 N·m 단위 자리표시자) — 실제 그리퍼로
  빈손 개폐/약한 물체 파지를 몇 번 해보면서 `grippers_effort` 피드백 값의 실제
  범위를 관찰한 뒤 재설정할 것. sim의 N 단위 값을 그대로 가져올 근거가 없다.

**4단계: 카메라·마커 세팅**

- `piper_eih_camera_node.py`의 `K_PLACEHOLDER`(초점거리 500px 가정) — 실제 카메라로
  체스보드 캘리브레이션(OpenCV `cv2.calibrateCamera`) 해서 갱신할 것. 지금 값 그대로면
  ArUco pose(특히 z, 거리)가 부정확하다.
- `piper_hw_pkg/urdf/eih_cam_mount.xacro`의 `<origin xyz="0 0 0" rpy="0 0 0"/>` —
  손목캠을 link6에 실제로 장착한 뒤 위치/자세를 실측(또는 hand-eye calibration)해서
  채울 것. 지금은 카메라가 link6 원점에 있다고 거짓 가정하고 있다.
- `common_pkg/step22_common.py`의 `PLACE_CENTER_BODY_GUESS`(자리표시자 `(0.20, KEEP_DIST,
  OBJ_CENTER_BODY[2])`) — 두 번째 마커(place 목표)를 대략 어디에 둘지 실제 배치에 맞게
  갱신할 것. `place_hover`가 이 값으로 먼저 접근한 뒤 마커 재검출로 정밀 보정한다.

## 5. 알려진 한계 (지금 범위에서 일부러 안 건드린 것)

- **파지 성공 판정(`arm_node.py`의 `_verify_by_chassis_cam`)은 차체캠(AMR) 전제라
  v1(AMR 없음)에서는 사실상 무력화된다** — `chassis_pose`가 안 들어오니
  `verify_low_hits`가 절대 안 늘어나 관찰 창이 끝나면 그냥 "성공"으로 흐른다. 지금은
  운용자가 눈으로 확인하는 수밖에 없다. 대체 판정(그리퍼 접촉여부 기반 등)은 다음 과제.
  - `EndPoseCtrl(...)` 예시는 `piper_sdk` 데모(`piper_ctrl_moveL.py` 등)에 있음 — 참고할 것.
- AMR(Ranger Mini) 통합, 벨트 추종은 이번 범위 밖(팔 단독 pick&place).
- `piper_driver_node.py`는 종료 시 `DisableArm()`을 자동 호출하지 않는다(브레이크
  미보유 축이 있으면 무동력 낙하 위험 — 문서상 근거 불충분해 마지막 자세 유지가 기본).
  비상정지가 필요하면 `piper_sdk`의 `EmergencyStop(0x01)`을 별도로 호출할 것.

## 6. 연락할 파일

- 설계/좌표계/제어식 전체 설명: [PiPER 엔드이펙터 접근·제어 알고리즘](이 vault 노트,
  시뮬레이션 PC의 `/home/da/Documents/tetra/`에 있음 — 필요하면 내용만 복사해서 가져올 것)
- 코드 리뷰로 발견된 기존 sim 버그 5건: 같은 vault의 `PiPER 워크스페이스 코드 리뷰.md`
  (실물 이식과 무관, sim에만 해당하니 급하지 않음)
