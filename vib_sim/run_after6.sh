#!/bin/bash
# run_tk6 묶음이 끝난 뒤 이어서: 토크에도 넣는 보정, 빠진 기준 하나, 좌우 모드 여유 확인
cd "$(dirname "$0")"
while [ "$(grep -c NIGHT_DONE out/run_tk6.log)" -lt 4 ]; do sleep 15; done
CASES=none,bz+prevzE10+ff10+dampRJ,bz+map+dampRJ,ff10 ./run_night.sh jobs_task6t.txt
CASES=none ./run_night.sh jobs_task6r2.txt
CASES=none,bz,ff10,bz+ff10 ./run_night.sh jobs_task6k.txt
echo AFTER6_DONE
