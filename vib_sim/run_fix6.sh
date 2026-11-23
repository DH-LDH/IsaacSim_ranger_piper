#!/bin/bash
# xargs -L 1이 줄 끝 공백 때문에 줄을 이어 붙여 빠진 실행을 다시 돌린다(run_after6 뒤에)
cd "$(dirname "$0")"
while ! grep -q AFTER6_DONE out/run_after6.log; do sleep 15; done
CASES=none,bz,dampRJ,bz+dampRJ,ff10,bz+ff10 ./run_night.sh jobs_fix_f.txt
CASES=none,dampRJ,ff10,prevzE10+ff10+dampRJ,map+dampRJ ./run_night.sh jobs_fix_b.txt
echo FIX6_DONE
