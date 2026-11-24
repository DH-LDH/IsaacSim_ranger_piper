#!/bin/bash
# 끝단 직선 왕복(gen_line.py) + 출발 → 10 mm 턱 → 정지. 줄마다 "태그 조건목록 옵션들". 동시에 3개. (박사님 11/24 의견)
cd "$(dirname "$0")"; P=/home/da/isaacsim/python.sh
one() { tag=$1; cases=$2; shift 2; [ -f out/exp6_$tag.csv ] && return; sleep $((RANDOM % 8)); timeout 1500 $P exp6_bump.py --cases $cases --tag $tag --keep --replay-t0 2.0 --pre 1.5 --stop "$@" > out/exp6_$tag.log 2>&1; }
export -f one; export P
sed 's/[[:space:]]*$//' ${1:-jobs_line.txt} | grep -v '^$' | xargs -P 3 -L 1 bash -c 'one "$@"' _
echo LINE_DONE
