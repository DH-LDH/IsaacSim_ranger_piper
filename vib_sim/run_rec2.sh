#!/bin/bash
# 박사님 보고용 비교 영상(그리퍼가 옆을 향한 자세, 뒤에서 본 시점, 확대 표식 20배): 평지·턱 10 mm·패인 바닥. 턱 15 mm는 이미 있음
cd "$(dirname "$0")"; P=/home/da/isaacsim/python.sh; export VIBSIM_USD=1
L="--fbp 2 --tau-clip 0.48 --tau-step 0.20 --tau-share 0.44 --mit asmc16 --mit-tdc 1,2,3 --qd-noise 0.01 --pose H --fp 5.5 --fy 3.6 --cam rear --mag 20 --pre 1.0"
R="prevzE10+ff10+dampRJ"
( timeout 900 $P exp6_bump.py --tag rec2_flat --rec out/rec2/flatH --cases none,bz $L --bsf 3.6 --stop --pothole 0.06 --h 0 --v 0.5 > out/exp6_rec2_flat.log 2>&1 ) &
( timeout 900 $P exp6_bump.py --tag rec2_b10 --rec out/rec2/b10H --cases none,$R $L > out/exp6_rec2_b10.log 2>&1 ) &
( timeout 900 $P exp6_bump.py --tag rec2_pot --rec out/rec2/potH --cases none,$R $L --pothole 0.06 > out/exp6_rec2_pot.log 2>&1 ) &
wait
T="— 초록: 기준 자리, 빨강: 벗어난 양을 20배로 키운 자리"
$P rec_combine.py out/rec2/flatH none bz "평지 출발·정지 (0.5 m/s), 그리퍼가 옆을 향한 자세 $T" "제어 없음(추종층만)" "AMR 속도 명령 성형(ZVD)" out/rec2/kros_flatH.mp4 2>/dev/null | tail -1
$P rec_combine.py out/rec2/b10H none $R "턱 넘기 (10 mm, 0.3 m/s), 그리퍼가 옆을 향한 자세 $T" "제어 없음(추종층만)" "뒷바퀴 예측 피드포워드 + 끝단 피드백" out/rec2/kros_bumpH_h10.mp4 2>/dev/null | tail -1
$P rec_combine.py out/rec2/potH none $R "패인 바닥 (깊이 10 mm, 0.3 m/s), 그리퍼가 옆을 향한 자세 $T" "제어 없음(추종층만)" "뒷바퀴 예측 피드포워드 + 끝단 피드백" out/rec2/kros_potholeH.mp4 2>/dev/null | tail -1
cp out/rec2/kros_flatH.mp4 out/rec2/kros_bumpH_h10.mp4 out/rec2/kros_potholeH.mp4 /home/da/nonmoon/KROS/video/
echo REC2_DONE
