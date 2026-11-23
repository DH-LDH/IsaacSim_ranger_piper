#!/bin/bash
cd "$(dirname "$0")"
CASES=none,bz,dampRJ,bz+dampRJ,ff10,bz+ff10 ./run_night.sh jobs_task7f.txt
CASES=none,dampRJ,ff10,prevzE10+ff10+dampRJ,map+dampRJ ./run_night.sh jobs_task7b.txt
echo TASK7_DONE
