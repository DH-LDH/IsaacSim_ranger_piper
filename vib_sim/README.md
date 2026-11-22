# vib_sim — 끝단 진동 검증용 Isaac Sim 장면

Ranger Mini + PiPER에서 팔 끝단의 약 7 Hz 진동을 보는 전용 장면이다. 벨트·비전·pick&place가 들어 있는 `plant_node`와 별개이고 ROS 없이 돈다. 2026-11-22 작성(KROS 2쪽 논문용).

## 실행

```bash
cd ~/isaacsim/ros2_ws_ranger_piper/vib_sim
~/isaacsim/python.sh import_piper.py            # 한 번만: 공식 URDF → assets/piper_abs (질량을 URDF 값으로)
~/isaacsim/python.sh t0_modal.py 0              # 자세 A·B 질량 행렬 → out/modal_*.npz
python3 fit_modal.py                            # 관절 강성 후보 훑기(모드 해석)
~/isaacsim/python.sh exp1_fixed.py              # 고정 베이스: J2 등속 후 정지 18조건 + 입력 성형
~/isaacsim/python.sh exp2_base.py --cases none,scurve,zv,zvd,armff,scurve+armff --tag main --keep
./run_all_yaw90.sh                              # AMR 실험 전부(가속 시간·가속도·좌우 강성·서스펜션), 실물 장착 방향
~/nonmoon/.venv_pptx/bin/python make_figs.py    # figs/*.png, figs/summary.md
```

- 눈으로 보기: `VIBSIM_GUI=1 ~/isaacsim/python.sh view.py --scene amr --case none` (옵션은 `view.py` 맨 위 설명). 실험 스크립트(exp1·exp2)는 화면을 그리지 않는다.
- 직접 명령 넣기: `~/isaacsim/python.sh drive.py` (또는 `--scene fixed`). 창이 뜬 뒤 터미널에 `v 0.5`, `stop`, `jr 2 -10`, `shape zvd` 같은 명령을 친다. 명령 목록은 `help`.
- Isaac을 한꺼번에 여러 개 띄우면 시작 단계에서 멈추는 일이 있었다(8개 동시 실행 때 4개). 3개까지는 문제없었다.
- 속도: 물리 1 kHz, 초당 약 1,400 스텝(실시간의 1.4배). USD 되쓰기를 꺼서 얻은 값이다(`vibsim.py` 위쪽 설정).

## 모형

| 항목 | 값 | 근거 |
|---|---|---|
| 팔 질량 | 공식 URDF 4.66 kg + 손목 카메라 0.11 kg | 기존 `piper_v2.usd`는 링크 합이 약 1.2 kg이라 쓰지 않음. 카메라 쪽 유효 질량은 FW191_B_LOG |
| 관절 탄성 | 관절마다 스프링·댐퍼. K = [200, 1355, 361, 3000, 342, 3000] N·m/rad | J2·J3·J5는 `fit_modal.py`: 자세 A 7.01 Hz, 자세 B 6.25 Hz(실측 6.6–7.3 / 6.2–6.3 Hz), 둘째 모드 12.6 Hz. J1(좌우)은 10/23 J1 계단·옆 툭을 좌우 축으로 다시 낸 값(FW191_D_LOG 14절: 자세 A 4.0–5.1 Hz, 자세 B 3.1–3.4 Hz, 감쇠비 0.07–0.19)에 맞춰 자세 A 4.15 Hz·감쇠비 0.12. 예비 값이다. 이 성분에는 J1 관절이 서보에 대해 돌아가는 몫이 섞여 있는데 스프링 하나로 묶었다 |
| 감쇠 | 강성 비례, 1차 모드에서 감쇠비 0.10 | 자세 A 실측 0.09–0.13 |
| 엔코더 / IMU | 엔코더 = 모터 쪽 각도, 끝단 자이로 = 그리퍼 몸통 각속도의 끄덕임 성분 | 실측은 카메라 자리. 절댓값을 댈 때만 1.27을 곱한다(FW191_B_LOG) |
| 위치 모드 서보 | 명령 100 Hz, 지연 55 ms + 2차 응답(9 Hz, 감쇠비 0.35) | 실측 지연 67–83 ms, 정지 지연 50–80 ms, 등속 중 엔코더 속도 봉우리 +20–25%, 5·10 °/s 정지 후 rms |
| AMR | 기존 Ranger USD(64.5 kg + 바퀴), 바퀴 속도 구동, 마찰 계수 1.0 | 서스펜션 없음. 민감도 시험에서만 바퀴마다 상하 스프링·댐퍼를 넣는다 |
| 팔 장착 | 팔 앞이 AMR의 옆(90°, 기본값) | 실물과 같다(`step22_common.ARM_BASE_YAW_DEG`). 직진 가감속은 팔의 좌우 모드를 흔든다. `--mount-yaw 0`이면 팔 앞이 진행 방향(끄덕임 모드를 흔듦) — 그 결과는 `out/yaw0`, `figs/yaw0` |

구현: PhysX 관절 각도를 링크 쪽 각도로 보고, 모터 쪽 각도는 스크립트에서 따로 적분해 관절 구동의 목표로 넣는다. 관절 구동의 stiffness·damping이 곧 관절 탄성이다.

## 결과

`figs/summary.md`(표), `figs/fig_valid.png`, `figs/fig_amr.png`, `figs/fig_susp.png`. 정리와 "쓸 수 있는 말"은 볼트 노트 `PiPER 진동 Isaac Sim 모형과 결과.md`.

## 모형에 없는 것

- 엔코더가 끝단 진동의 약 2할을 보는 것(모터 쪽이 반력에 밀리지 않는다), 백래시, 마찰, 토크 분해능, MIT 모드와 순간 튐.
- 자세 B의 낮은 감쇠비(0.04–0.05), 처짐 상태에 따라 모드가 6.0–7.5 Hz로 움직이는 것(강성 배율 `--K-scale`로 흉내만 낸다).
- 15 °/s에서 실측 진동이 더 커지지 않는 것(시뮬레이션은 속도에 비례).
- 실물 Ranger Mini의 서스펜션·타이어 특성, 바닥 요철.
- 요 자이로에 보이는 자세 A 7.7–7.9 Hz 성분, J1 서보 강성과 구조 탄성의 구분(지금은 한 스프링).
- 서스펜션 바퀴당 30–45 kN/m에서 차체가 스스로 흔들리는 불안정이 나온다(바퀴 속도 구동 모형 문제로 추정). 그 구간 결과는 쓰지 않는다.
- 이전 실행: `out/lat11hz`·`figs/lat11hz`는 좌우 모드를 11.4 Hz로 잘못 둔 결과, `out/yaw0`·`figs/yaw0`는 팔을 진행 방향으로 장착한 결과.
- 타이어 탄성을 접촉 강성(compliant contact)으로 넣는 방식은 차체가 튕겨 나가 버렸다. `--tire-k`는 쓰지 말 것.
