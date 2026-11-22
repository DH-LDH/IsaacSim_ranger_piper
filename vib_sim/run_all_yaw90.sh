#!/bin/bash
# 실물 장착 방향(팔 앞이 AMR 옆, 90도)으로 AMR 실험 전부. 두 줄로 나눠 돌린다(동시에 2개).
cd "$(dirname "$0")"
P=/home/da/isaacsim/python.sh
# 본 비교를 먼저 돌려 대책 없음에서 가장 크게 울린 주파수를 성형기 주파수로 쓴다
timeout 600 $P exp2_base.py --accels 1.0 --Tas 0.5 --cases none,scurve,zv,zvd,armff,scurve+armff --tag main --keep > out/exp2_main.log 2>&1
SF=$(python3 -c "import csv;print(next(csv.DictReader(open('out/exp2_main.csv')))['sf'])")
echo "SF=$SF"
lane1() {
  timeout 900 $P exp2_base.py --accels 1.0 --Tas 0.05,0.075,0.1,0.125,0.15,0.175,0.2,0.225,0.25,0.275,0.3,0.325,0.35,0.4,0.45,0.5,0.6,0.7 --cases none --tag sweepTa > out/exp2_sweepTa.log 2>&1
  timeout 900 $P exp2_base.py --accels 1.0 --Tas 0.1,0.2,0.35,0.7 --cases none,scurve,zv,zvd --sf $SF --tag Ta4 > out/exp2_Ta4.log 2>&1
  timeout 600 $P exp2_base.py --accels 0.5,2.0 --Tas 0.5 --cases none,zvd --sf $SF --tag acc > out/exp2_acc.log 2>&1
  for k1 in 140 170 260; do
    timeout 600 $P exp2_base.py --accels 1.0 --Tas 0.5 --cases none,scurve,zv,zvd,scurve+armff --sf $SF --K1 $k1 --tag k1_$k1 > out/exp2_k1_$k1.log 2>&1
  done
}
lane2() {
  sleep 20
  for k in 10000 20000 35700 60000 85000 120000 200000 400000; do
    c=$(python3 -c "print(round(340*($k/35700)**0.5))")
    timeout 300 $P exp2_base.py --accels 1.0 --Tas 0.5 --cases none,scurve,zv,zvd --sf $SF --susp-k $k --susp-c $c --tag susp$k --keep > out/exp2_susp$k.log 2>&1
  done
}
lane1 & lane2 & wait
echo ALL_DONE
