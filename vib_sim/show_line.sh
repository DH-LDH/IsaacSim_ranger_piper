#!/bin/bash
# 눈으로 보기: 팔이 선을 그리는 동안 AMR이 출발 → (턱 / 패인 바닥 / 평지) → 정지. 자세 A, 토크 제어 + 추종 제어기(J2·J3).
#   ./show_line.sh <동작> <바닥> [조건들] [그 밖의 exp6_bump.py 옵션]
#   동작: z = 위아래 직선(실측 기록)   x = 팔 앞뒤 직선(실측 기록)   ud = 위아래 동작(2번 관절 ±5°, 10/29 기록)
#   바닥: bump = 턱 10 mm   bump15 = 턱 15 mm   pot = 패인 바닥 10 mm   flat = 평지 출발·정지(0.5 m/s)
#   조건(쉼표, 차례로 되풀이해 보여 준다). 비우면 바닥에 맞는 기본값:
#     none = 대책 없음   kinprev+prevzE10+ff10 = 자세 보상 + 뒷바퀴 예측   kinmap+map = 자세 보상 + 지형 예견(none 뒤에 와야 함)
#     kinmap = 자세 보상만   bz = AMR 속도 명령 성형   kin10+ff10 = 베이스 IMU로 잰 값으로 보정
#   화면: 초록 공 = 끝단이 있어야 할 자리(월드 기준), 빨강 공 = 거기서 벗어난 양을 20배로 키운 자리. 터미널에 조건마다 수치가 찍힌다.
#   카메라를 옆에서 보려면 끝에 --cam side, 실제 속도보다 느리게는 --slow 3
cd "$(dirname "$0")"; L=$HOME/piper_ws/docs/data/raw/line
case "$1" in
  z)  REP="--replay $L/20261124_110709_traj_line_z30_asmc3.csv.gz --replay-t0 10.02" ;;
  x)  REP="--replay $L/20261124_110952_traj_line_x20_asmc3.csv.gz --replay-t0 10.09" ;;
  ud) REP="--replay $HOME/piper_ws/docs/data/raw/scn/20261029_110051_slow_j2_s1.csv.gz" ;;
  *)  sed -n 2,11p "$0"; exit 1 ;;
esac
case "$2" in
  bump)   FL="";                              DEF="none,kinprev+prevzE10+ff10,kinmap+map" ;;
  bump15) FL="--h 0.015";                     DEF="none,kinprev+prevzE10+ff10,kinmap+map" ;;
  pot)    FL="--pothole 0.06";                DEF="none,kinprev+prevzE10+ff10,kinmap+map" ;;
  flat)   FL="--pothole 0.06 --h 0 --v 0.5";  DEF="none,bz,kin10+ff10" ;;
  *)      sed -n 2,11p "$0"; exit 1 ;;
esac
C="${3:-$DEF}"; shift 3 2>/dev/null || shift $#
exec $HOME/isaacsim/python.sh exp6_bump.py --gui --loops 0 --tag show --cam rear --mag 20 --pre 1.5 --stop --cases "$C" \
  --fbp 2 --tau-clip 0.48 --tau-step 0.20 --tau-share 0.44 --mit asmc16 --mit-tdc 2,3 --qd-noise 0.01 $REP $FL "$@"
