#!/bin/bash
cd "$(dirname "$0")"
CASES=none,bz,ff10,bz+ff10 ./run_night.sh jobs_poseh_f.txt
CASES=none,dampRJ,ff10,prevzE10+ff10+dampRJ,map+dampRJ ./run_night.sh jobs_poseh_b.txt
echo POSEH_DONE
