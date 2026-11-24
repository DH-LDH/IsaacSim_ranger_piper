"""물리 엔진의 질량 행렬을 자세 A·H에서 뽑아 둔다 → out/massmat.npz  (손으로 유도한 식과 대조하는 용도, model_check.py가 읽는다)
저장: M_A, M_H(전체: 베이스 6 + 관절), ia(팔 관절 열 번호), off(관절 열이 시작하는 곳), q_A, q_H(가라앉은 링크 쪽 각), names
실행: ~/isaacsim/python.sh dump_M.py"""
import numpy as np
from vibsim import HERE, POSE_A, POSE_H, VibSim, app

sim = VibSim(base="ranger", verbose=False)
out = {}
for nm, P in (("A", POSE_A), ("H", POSE_H)):
    sim.world.reset(); sim._apply_gains(); sim.set_state(P)
    for _ in range(2500):
        sim.step(P, np.zeros(6), wheel_w=np.zeros(4), steer=np.zeros(4))
    M = sim.mass_matrix(); out["M_" + nm] = M; out["q_" + nm] = sim.q_link(); out["g_" + nm] = sim.gravity()
    bp, bq = sim.bb.get_world_poses(); ep, eq = sim.ee.get_world_poses()
    out["bp_" + nm] = np.asarray(bp)[0]; out["bq_" + nm] = np.asarray(bq)[0]; out["ep_" + nm] = np.asarray(ep)[0]
    print(nm, "M", M.shape, "q_link[deg]", np.round(np.degrees(sim.q_link()), 2), flush=True)
out["ia"] = sim.ia; out["off"] = out["M_A"].shape[0] - sim.n; out["K"] = sim.K
try: out["names"] = np.array(list(sim.art.dof_names))
except Exception: pass  # noqa: BLE001
np.savez(HERE + "/out/massmat.npz", **out); print("→ out/massmat.npz", flush=True)
sim.close(); app.close()
