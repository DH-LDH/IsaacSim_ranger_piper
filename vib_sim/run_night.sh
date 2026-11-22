#!/bin/bash
# 밤사이 조건 격자: jobs_night.txt의 줄마다 "태그 옵션들". 동시에 3개.
cd "$(dirname "$0")"; P=/home/da/isaacsim/python.sh
C="${CASES:-none,ff10,ff10+damp10p,map,mapE3,mapservo,prev,prevzE10+ff10,prevzE10+ff10+damp10p}"
one() { tag=$1; shift; [ -f out/exp6_$tag.csv ] && return; sleep $((RANDOM % 12)); timeout 1500 $P exp6_bump.py --cases $C --tag $tag "$@" > out/exp6_$tag.log 2>&1; }
export -f one; export P C
cat ${1:-jobs_night.txt} | xargs -P 3 -L 1 bash -c 'one "$@"' _
# 팔 장착 방향별 출발·정지(기본 목록일 때만)
[ -z "$1" ] && for y in 0 180; do timeout 600 $P exp2_base.py --accels 1.0 --Tas 0.5 --cases none,scurve,zv,zvd --mount-yaw $y --tag yaw$y > out/exp2_yaw$y.log 2>&1; done
echo NIGHT_DONE
