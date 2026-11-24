"""끝단이 직선을 왕복하는 기준 궤적을 만든다(박사님 11/24 의견: 끝단을 세워 두지 말고 직선으로 움직이면서 진동이 주는 것을 보일 것).

exp6_bump.py --replay 가 읽는 기록과 같은 꼴(t, q1..q6, qref1..qref6 [deg])로 쓴다. 실측 기록이 아니라 만든 기준이다 —
관절 1–3은 --mit 모형이 이 기준을 따라가고, 관절 4–6은 기준 그대로 둔다(자세 A의 손목 각 유지). 그러므로 위치 제어(엔코더 재생) 비교에는 쓸 수 없다.

직선은 팔 베이스 좌표에서 gripper_base 원점이 자세 A의 자리를 가운데로 ±amp 를 사인(0.25 Hz)으로 왕복한다. 관절 1–3만으로 위치 셋을 맞춘다(끝단 방향은 조금 변함).
방향: x = 팔 앞(실물 장착에서는 AMR의 옆), y = 팔의 좌우(AMR 진행 방향), z = 위아래.
왕복 크기는 가장 많이 움직이는 관절이 ±5°쯤이 되게 고른다(모형을 실측과 대조한 동작이 ±5°, 0.25 Hz라서).

사용: python3 gen_line.py --dir y --amp 0.03   →  lines/line_y30.csv.gz
"""
import argparse, gzip, math, os
import xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
POSE_A = np.radians([-0.3, 121.1, -73.2, 1.6, 52.0, 13.7])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def load_chain(path=HERE + "/assets/piper_abs.urdf"):
    J = {j.get("name"): j for j in ET.parse(path).getroot().findall("joint")}; ch = []
    for n in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
        o = J[n].find("origin"); T = np.eye(4)
        T[:3, :3] = _rpy(*[float(v) for v in o.get("rpy").split()]); T[:3, 3] = [float(v) for v in o.get("xyz").split()]
        ch.append((T, np.array([float(v) for v in J[n].find("axis").get("xyz").split()])))
    return ch


def fk(ch, q):
    """gripper_base 원점의 위치(팔 베이스 좌표)[m]"""
    T = np.eye(4)
    for (T0, ax), qi in zip(ch, q):
        c, s = math.cos(qi), math.sin(qi); x, y, z = ax; C = 1 - c
        R = np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
        Tj = np.eye(4); Tj[:3, :3] = R; T = T @ T0 @ Tj
    return T[:3, 3]


def ik3(ch, q0, p):
    """관절 1–3만 움직여 위치 p를 맞춘다(뉴턴법, 수치 야코비안)"""
    q = q0.copy()
    for _ in range(50):
        e = p - fk(ch, q)
        if np.linalg.norm(e) < 1e-9: break
        Jm = np.zeros((3, 3))
        for j in range(3):
            d = np.zeros(6); d[j] = 1e-6; Jm[:, j] = (fk(ch, q + d) - fk(ch, q - d)) / 2e-6
        q[:3] += np.linalg.solve(Jm, e)
    return q


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="y"); ap.add_argument("--amp", type=float, default=0.03, help="왕복 반폭[m]")
    ap.add_argument("--f", type=float, default=0.25); ap.add_argument("--T", type=float, default=24.0); ap.add_argument("--t-start", type=float, default=1.0)
    ap.add_argument("--ramp", type=float, default=1.0, help="진폭을 0에서 올리는 시간[s]"); ap.add_argument("--cycles", type=float, default=0.0, help="0보다 크면 이 주기 수만 왕복하고 같은 시간에 걸쳐 진폭을 내린 뒤 멈춘다(실기용)")
    ap.add_argument("--rate", type=float, default=200.0, help="표본율[Hz]"); ap.add_argument("--ref-only", action="store_true", help="t, qref1..6 만 쓴다(실기에 넘기는 기준 궤적)")
    ap.add_argument("--pose", default="A", help="가운데 자세: A 또는 H(자세 A에서 5번 관절만 −30°로 — 그리퍼가 수평으로 팔 앞을 향함)")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    if a.pose.upper() == "H": POSE_A = POSE_A.copy(); POSE_A[4] = math.radians(-30.0)
    ch = load_chain(); p0 = fk(ch, POSE_A); d = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}[a.dir]
    sm = lambda u: 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    if a.cycles > 0: a.T = a.t_start + a.cycles / a.f + a.t_start
    t = np.arange(0.0, a.T, 1.0 / a.rate); env = sm(np.clip((t - a.t_start) / a.ramp, 0.0, 1.0))                                  # 부드럽게 시작
    if a.cycles > 0: env = env * sm(np.clip((a.t_start + a.cycles / a.f - t) / a.ramp, 0.0, 1.0))                               # 부드럽게 끝
    s = a.amp * env * np.sin(2 * math.pi * a.f * (t - a.t_start))
    Q = np.zeros((len(t), 6)); q = POSE_A.copy()
    for i, si in enumerate(s):
        q = ik3(ch, q, p0 + si * d); Q[i] = q
    P = np.array([fk(ch, qi) for qi in Q]); dev = P - (p0 + np.outer(s, d))
    out = a.out or f"{HERE}/lines/line_{a.dir}{int(round(a.amp * 1000))}{'' if a.pose.upper() == 'A' else '_' + a.pose.upper()}.csv.gz"; os.makedirs(os.path.dirname(out), exist_ok=True)
    with gzip.open(out, "wt") as f:
        f.write("t," + ("" if a.ref_only else ",".join(f"q{j}" for j in range(1, 7)) + ",") + ",".join(f"qref{j}" for j in range(1, 7)) + ",s_mm\n")
        for ti, qi, si in zip(t, np.degrees(Q), s):
            row = ",".join(f"{v:.5f}" for v in qi); f.write(f"{ti:.4f},{row},{si * 1000:.4f}\n" if a.ref_only else f"{ti:.4f},{row},{row},{si * 1000:.4f}\n")
    ex = np.degrees(Q.max(0) - Q.min(0)) / 2
    print(f"자세 A 끝단 위치(팔 베이스 좌표) {np.round(p0, 4)} m, 방향 {a.dir}, 반폭 {a.amp * 1000:.0f} mm, 최대 속도 {a.amp * 2 * math.pi * a.f * 1000:.0f} mm/s")
    print("관절 최고 속도[deg/s] J1–3:", np.round(np.abs(np.gradient(np.degrees(Q), 1.0 / a.rate, axis=0)).max(0)[:3], 2), f"| 길이 {t[-1]:.1f} s, 시작·끝 자세 차 {np.abs(np.degrees(Q[-1] - Q[0])).max():.4f} deg")
    print("관절 왕복 반폭[deg] J1–3:", np.round(ex[:3], 2), "| 직선에서 벗어남(역기구학 오차) 최대", f"{np.abs(dev).max() * 1e6:.3f} um")
    print("→", out)
