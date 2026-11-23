#!/bin/bash
# exp7_mit.py 조건 격자: 줄마다 "태그 로그이름 옵션들". 동시에 3개.
cd "$(dirname "$0")"; P=/home/da/isaacsim/python.sh
one() { tag=$1; name=$2; shift 2; [ -f out/exp7_${tag}_$name.log ] && grep -q "^ROW" out/exp7_${tag}_$name.log && return; sleep $((RANDOM % 8)); timeout 900 $P exp7_mit.py --tag $tag "$@" > out/exp7_${tag}_$name.log 2>&1; }
export -f one; export P
cat $1 | xargs -P 3 -L 1 bash -c 'one "$@"' _
echo MIT7_DONE
