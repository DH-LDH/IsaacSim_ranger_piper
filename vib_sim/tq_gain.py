"""J2 통로의 공진 이득: 모터 쪽 J2에 사인 Δq를 넣고 끝단 끄덕임 각속도의 진폭을 잰다(고정 베이스, 자세 A).
스프링 토크 K_J2·Δq 1 N·m당 rad/s. 실기 처프의 v/τ 피크(0.21–0.39 rad/s per N·m, 카메라 자리, FW191_D_LOG 16.8절 (다))와 견준다."""
import math, sys
import numpy as np
from vibsim import POSE_A, VibSim, HERE
sim = VibSim(base="fixed"); DT = sim.dt; TAU = 0.3; A = TAU / sim.K[1]; out = []
for f in np.arange(5.0, 10.01, 0.25):
    sim.world.reset(); sim._apply_gains(); sim.set_state(POSE_A)
    for _ in range(1000): sim.step(POSE_A, np.zeros(6))
    n = int(3.5 / DT); g = np.zeros(n)
    for i in range(n):
        q = POSE_A.copy(); q[1] += A * math.sin(2 * math.pi * f * i * DT) * min(1.0, i * DT / 0.5)
        sim.step(q, np.zeros(6)); g[i] = sim.ee_gyro()[1]
    x = g[int(2.0 / DT):]; x = x - x.mean(); amp = math.sqrt(2) * float(np.sqrt(np.mean(x ** 2)))
    out.append((f, amp / TAU)); print("GAIN f %.2f Hz  %.3f rad/s per N·m(스프링 토크, 그리퍼 몸통)" % (f, amp / TAU), flush=True)
fp, gp = max(out, key=lambda r: r[1])
print("PEAK %.2f Hz  스프링 토크 기준 %.3f, 카메라 자리(x1.27) %.3f, 모터 토크 기준(x0.82–0.88) %.3f–%.3f rad/s per N·m" % (fp, gp, gp * 1.27, gp * 1.27 * 0.82, gp * 1.27 * 0.88))
np.savetxt(HERE + "/out/tq_gain.csv", np.array(out), delimiter=",", header="f_hz,gain_rad_s_per_Nm", comments="")
sim.close()
