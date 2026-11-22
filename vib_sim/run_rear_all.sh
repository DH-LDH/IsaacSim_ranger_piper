#!/bin/bash
# 팔을 뒷바퀴 축 위(-0.23 m, vibsim 기본값)에 둔 상태로 AMR이 들어가는 실험 전부. 동시에 3개까지.
cd "$(dirname "$0")"; P=/home/da/isaacsim/python.sh; R=/home/da/piper_ws/docs/data/raw
bump() {
  timeout 1200 $P exp5_bump.py --cases none,ff10,ff10+damp10p,prev,prevzE10+ff10,prevzE10+ff10+damp10p,map,mapE3,mapE-3,mapE9,mapS70,mapS130,mapservo --tag r_methods > out/exp5_r_methods.log 2>&1
  timeout 1200 $P exp5_bump.py --cases none,ff0,ff5,ff10,ff20,ff40,ffservo,damp10,damp10p,damp20,damp20p,damp36,damp36p,ff10+damp20p,ff20+damp36p --tag r_delay > out/exp5_r_delay.log 2>&1
  timeout 1200 $P exp5_bump.py --pothole 0.06 --h 0.01 --cases none,ff10,ff10+damp10p,prev,prevzE10+ff10,map,mapE3,mapservo --tag r_pot > out/exp5_r_pot.log 2>&1
  for hv in "0.003 0.15" "0.005 0.15" "0.01 0.15" "0.003 0.3" "0.005 0.3" "0.01 0.3"; do set -- $hv
    timeout 600 $P exp5_bump.py --h $1 --v $2 --cases none,ff10+damp10p,map,prevzE10+ff10 --tag r_g_h$1_v$2 > out/exp5_r_g_h$1_v$2.log 2>&1; done
}
bump &
sleep 20; ./run_all_yaw90.sh > out/run_all.log 2>&1
F=$(ls $R/scn/*.csv.gz | grep -v -E "imu|tdc" | grep slow)      # 위치 모드 시나리오 셋
timeout 1500 $P exp3_scn.py --src replay --bases still,move,zvd --files $F --tag replay_scn_amr > out/exp3_replay_scn_amr.log 2>&1 &
sleep 20; timeout 900 $P exp3_scn.py --src ideal --bases still,move,zvd --tag ideal_amr > out/exp3_ideal_amr.log 2>&1
wait; echo REAR_ALL_DONE
