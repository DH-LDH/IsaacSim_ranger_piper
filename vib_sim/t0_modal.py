"""0단계: 장면이 서는지 확인하고, 자세 A·B의 질량 행렬·중력 토크·끝단 끄덕임 행을 저장한다.

끝단 끄덕임 행 p: 관절 속도 dq(6) → 끝단 각속도의 끄덕임 성분. 관절마다 단위 속도를 줘서 직접 잰다.
결과: out/modal_<pose>_<payload>.npz  (M, g, P(3x6 각속도 야코비안))
"""
import os
import sys

import numpy as np
from vibsim import POSE_A, POSE_B, VibSim, HERE

payload = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
os.makedirs(HERE + "/out", exist_ok=True)
sim = VibSim(base="fixed", K=np.full(6, 1.0e5), D=np.full(6, 1.0e3), payload=payload)
for name, pose in (("A", POSE_A), ("B", POSE_B)):
    sim.set_state(pose)
    sim.settle(0.3)
    q = sim.q_link()
    M = sim.mass_matrix()
    g = sim.gravity()
    ia = sim.ia
    P = np.zeros((3, 6))
    for j in range(6):  # 관절 j만 0.2 rad/s로 움직여 끝단 각속도를 읽는다
        sim.set_state(pose)
        dq = np.zeros(6); dq[j] = 0.2
        qm = pose.copy()
        for _ in range(100):
            qm = qm + dq * sim.dt
            sim.step(qm, dq)
        P[:, j] = sim.ee_vel()[1] / 0.2
    np.savez(HERE + f"/out/modal_{name}_{payload:g}.npz", M=M, g=g, P=P, ia=ia, q=q, names=np.array(sim.names))
    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    print(f"=== pose {name} payload {payload} q_link(deg) {np.degrees(q)}")
    print("M(arm)=\n", M[np.ix_(ia, ia)])
    print("g(arm)=", g[ia])
    print("P=\n", P)
sim.close()
