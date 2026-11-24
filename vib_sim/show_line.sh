#!/bin/bash
# 눈으로 보기(녹화용): 팔 끝단이 선을 그리는 동안 AMR이 출발 → (턱 / 패인 바닥 / 평지) → 정지. 토크 제어 + 추종 제어기(J2·J3).
#   ./show_line.sh <자세> <동작> <바닥> [조건들] [그 밖의 exp6_bump.py 옵션]
#   자세: A = 그리퍼가 아래를 봄   H = 그리퍼가 앞을 봄(원고의 자세 B)      (자세를 빼면 A)
#   동작: z = 위아래 직선   x = 팔 앞뒤 직선   ud = 위아래 동작(2번 관절 ±5°)      — 모두 실측 기록
#   바닥: bump = 턱 10 mm   bump15 = 턱 15 mm   pot = 패인 바닥 10 mm   flat = 평지 출발·정지(0.5 m/s)
#   조건(쉼표, 차례로 되풀이해 보여 준다). 비우면 바닥에 맞는 기본값(대책 없음 → 원고의 구성):
#     none = 대책 없음   kinprev+prevzE10+ff10 = 자세 보상 + 뒷바퀴 예측(원고의 (b) + (c))   kinmap = 자세 보상만(none 뒤에 와야 함)
#     kinmap+map = 자세 보상 + 지형 예견(none 뒤)   bz = AMR 속도 명령 성형   kin10+ff10 = 베이스 IMU로 잰 값으로 보정(평지)
#   화면: 초록 공 = 끝단이 있어야 할 자리(월드 기준), 다른 공 = 거기서 벗어난 양을 20배로 키운 자리(대책 없음일 때 빨강, 대책 있음일 때 파랑).
#         조건이 시작할 때 터미널에 ">>> 지금 조건"이, 끝날 때 수치가 찍힌다.
#   옵션 예: --cam side(옆에서), --slow 3(3배 느리게), --mag 50(표식을 50배로), --loops 1(한 바퀴만 돌고 끝)
cd "$(dirname "$0")"; L=$HOME/piper_ws/docs/data/raw/line; PS=A
case "$1" in A|a) shift ;; H|h|B|b) PS=H; shift ;; esac
if [ $PS = A ]; then
  case "$1" in
    z)  REP="--replay $L/20261124_110709_traj_line_z30_asmc3.csv.gz --replay-t0 10.02" ;;
    x)  REP="--replay $L/20261124_110952_traj_line_x20_asmc3.csv.gz --replay-t0 10.09" ;;
    ud) REP="--replay $HOME/piper_ws/docs/data/raw/scn/20261029_110051_slow_j2_s1.csv.gz" ;;
    *)  sed -n 2,13p "$0"; exit 1 ;;
  esac
else
  case "$1" in
    z)  REP="--pose H --replay $L/20261124_133048_traj_line_z30_H_asmc23.csv.gz --replay-t0 13.98" ;;
    x)  REP="--pose H --replay $L/20261124_133334_traj_line_x20_H_asmc23.csv.gz --replay-t0 10.09" ;;
    ud) REP="--pose H --replay $L/20261124_112823_slow_j2_poseH_s1.csv.gz" ;;
    *)  sed -n 2,13p "$0"; exit 1 ;;
  esac
fi
case "$2" in
  bump)   FL="";                              DEF="none,kinprev+prevzE10+ff10" ;;
  bump15) FL="--h 0.015";                     DEF="none,kinprev+prevzE10+ff10" ;;
  pot)    FL="--pothole 0.06";                DEF="none,kinprev+prevzE10+ff10" ;;
  flat)   FL="--pothole 0.06 --h 0 --v 0.5";  DEF="none,bz,kin10+ff10" ;;
  *)      sed -n 2,13p "$0"; exit 1 ;;
esac
C="${3:-$DEF}"; shift 3 2>/dev/null || shift $#
exec $HOME/isaacsim/python.sh exp6_bump.py --gui --loops 0 --tag show --cam rear --mag 20 --pre 1.5 --stop --cases "$C" \
  --fbp 2 --tau-clip 0.48 --tau-step 0.20 --tau-share 0.44 --mit asmc16 --mit-tdc 2,3 --qd-noise 0.01 $REP $FL "$@"
