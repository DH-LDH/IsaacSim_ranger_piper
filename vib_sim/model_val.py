"""손으로 쓴 운동 방정식(원고 2절 식 (1)–(4), (7))을 직접 풀어 시뮬레이션과 겹쳐 본다 → figs/kroc_modelval.png, 표는 화면에.
입력: out/modelval_<자세>.npz (exp_modelval.py: 모터 쪽 각 고정, AMR이 출발 → 턱 10 mm → 감속)
식에 넣는 것은 시뮬레이션에서 읽은 팔 베이스 링크의 속도·자세뿐이다(→ 어깨 점 P의 가속도, 각가속도). 관절이 휘는 각은 식이 낸다.
실행: ~/nonmoon/.venv_pptx/bin/python model_val.py"""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import model_check as mc

H = mc.H; G0 = 9.81; MM = 1 / 25.4
plt.rcParams.update({"font.size": 9.5, "font.family": "Times New Roman", "mathtext.fontset": "stix", "axes.grid": True, "grid.alpha": 0.3, "axes.linewidth": 0.6})


def quatR(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def inputs(d, com=True):
    """어깨 점 P의 '가속도 + 중력'(팔 베이스 좌표 성분)과 팔 베이스의 각속도·각가속도(팔 베이스 좌표)"""
    t = d["t"]; dt = t[1] - t[0]; n = len(t); va = d["va"]; pa = d["pa"]; R = np.array([quatR(q) for q in pa[:, 3:]])
    cB = np.array([-0.00473641, 0.0000256829, 0.0414515]); rP = np.array([0.0, 0.0, 0.123])
    w = va[:, 3:]; vo = va[:, :3] - (np.cross(w, np.einsum("nij,j->ni", R, cB)) if com else 0.0)      # 물리 엔진의 병진 속도는 질량 중심의 것 → 링크 원점으로
    vP = vo + np.cross(w, np.einsum("nij,j->ni", R, rP))
    aP = np.zeros((n, 3)); aP[:-1] = (vP[1:] - vP[:-1]) / dt; dw = np.zeros((n, 3)); dw[:-1] = (w[1:] - w[:-1]) / dt
    f = np.einsum("nji,nj->ni", R, aP + np.array([0, 0, G0])); wA = np.einsum("nji,nj->ni", R, w); dwA = np.einsum("nji,nj->ni", R, dw)
    return f, wA, dwA, vo, R


def solve(d, usd=True, use_ang=True, center=None):
    mc.USE_USD = usd; qm = d["qm"]; K = d["K"]; D = d["D"]; par, M2, _ = mc.planar(qm); Mqq, Mqb, B, T = mc.matrices(qm)
    m2, m3, l2, lc2, lc3, I2, I3 = (par[k] for k in ("m2", "m3", "l2", "lc2", "lc3", "I2", "I3")); t2m, eps, t3m = math.radians(par["ph2"]), math.radians(par["phc2"] - par["ph2"]), math.radians(par["ph3"]); dm = t3m - t2m
    I1 = Mqq[0, 0]; mrho = sum(b[1] * (mc.R_BA.T @ (b[2] - mc.P_BA))[0] for b in B if b[0] != "base_link")
    f, wA, dwA, vo, R = inputs(d); t = d["t"]; dt = t[1] - t[0]; n = len(t)
    if center is not None: f = center                                           # 비교용: 각가속도 몫을 뺀 입력(차체 가운데의 가속도만)
    fx, fy, fz = f[:, 0], f[:, 1], f[:, 2]; phd = -wA[:, 1]; al = -dwA[:, 1]; alz = dwA[:, 2]     # 평면 안 회전(x_a → z)은 −y_a 둘레
    if not use_ang: phd = phd * 0; al = al * 0; alz = alz * 0

    def acc(x, i):
        th2, de, w2, wd = x; th3 = th2 + de; h = m3 * l2 * lc3 * math.sin(de); cd = math.cos(de)
        M = np.array([[I2 + I3 + m3 * l2 ** 2 + 2 * m3 * l2 * lc3 * cd, I3 + m3 * l2 * lc3 * cd], [I3 + m3 * l2 * lc3 * cd, I3]])
        W2 = w2 + phd[i]; c = h * np.array([-(2 * W2 * wd + wd ** 2), W2 ** 2])
        S = np.array([m2 * lc2 * math.sin(th2 + eps) + m3 * (l2 * math.sin(th2) + lc3 * math.sin(th3)), m3 * lc3 * math.sin(th3)])
        C = np.array([m2 * lc2 * math.cos(th2 + eps) + m3 * (l2 * math.cos(th2) + lc3 * math.cos(th3)), m3 * lc3 * math.cos(th3)])
        tau = np.array([K[1] * (t2m - th2) - D[1] * w2, K[2] * (dm - de) - D[2] * wd])
        return np.linalg.solve(M, tau - c - C * fz[i] + S * fx[i] - M[:, 0] * al[i])

    # 정지 평형에서 시작
    x = np.array([t2m, dm, 0.0, 0.0]); f0x, f0z = float(np.mean(fx[:100])), float(np.mean(fz[:100]))
    for _ in range(100):                                                       # 정지 평형: K(θ_m − θ) = C(θ) f_z − S(θ) f_x 를 되풀이해 푼다
        th2, de = x[0], x[1]; th3 = th2 + de
        S = np.array([m2 * lc2 * math.sin(th2 + eps) + m3 * (l2 * math.sin(th2) + lc3 * math.sin(th3)), m3 * lc3 * math.sin(th3)]); C = np.array([m2 * lc2 * math.cos(th2 + eps) + m3 * (l2 * math.cos(th2) + lc3 * math.cos(th3)), m3 * lc3 * math.cos(th3)])
        x[:2] = np.array([t2m, dm]) - (C * f0z - S * f0x) / K[1:3]
    TH = np.zeros((n, 2)); q1 = np.zeros(n); x1 = np.array([0.0, 0.0])
    for i in range(n):                                                         # 반암시적 오일러(물리 엔진과 같은 순서), 1 kHz
        TH[i] = x[:2]; q1[i] = x1[0]
        x[2:] += acc(x, i) * dt; x[:2] += x[2:] * dt
        a1 = (-K[0] * x1[0] - D[0] * x1[1] - mrho * fy[i] - I1 * alz[i]) / I1; x1[1] += a1 * dt; x1[0] += x1[1] * dt
    dq2 = -(TH[:, 0] - TH[0, 0]); dq3 = -(TH[:, 1] - TH[0, 1])                 # 관절각 부호로(dθ/dq = −1)
    return np.c_[q1 - q1[0], dq2, dq3]


def compare(tag):
    d = dict(np.load(f"{H}/out/modelval_{tag}.npz")); t = d["t"]; dt = t[1] - t[0]; sim = d["ql"][:, :3] - d["ql"][0, :3]
    x = d["pb"][:, 0]; i_f = int(np.argmax(x + 0.247 >= 0.72)); i_r = int(np.argmax(x - 0.247 >= 0.72)); m = np.zeros(len(t), bool); m[i_f - int(0.1 / dt):i_r + int(1.0 / dt)] = True
    return d, t, sim, m, i_f, i_r


if __name__ == "__main__":
    rows = []
    for tag, lab in (("A_tk15", "자세 A, 스프링 접촉 1.5e5(논문 결과와 같은 설정), 1 kHz"), ("H_tk15", "자세 B, 스프링 접촉 1.5e5(논문 결과와 같은 설정), 1 kHz"), ("A_tk2e5", "자세 A, 스프링 접촉 2e5, 1 kHz"), ("A_tk2e5_2k", "자세 A, 스프링 접촉 2e5, 2 kHz"), ("H_tk2e5", "자세 B, 스프링 접촉 2e5, 1 kHz"), ("A", "자세 A, 강체 접촉, 1 kHz"), ("A_2k", "자세 A, 강체 접촉, 2 kHz"), ("H", "자세 B, 강체 접촉, 1 kHz")):
        if not os.path.exists(f"{H}/out/modelval_{tag}.npz"): continue
        d, t, sim, m, i_f, i_r = compare(tag); rms = lambda v: float(np.sqrt(np.mean(v[m] ** 2))); e = solve(d, usd=True)
        print(f"\n--- {lab} ---  베이스 들림 최대 {1e3 * (d['pb'][:, 2] - d['pb'][0, 2]).max():.2f} mm, 한 스텝의 2번 관절 속도 변화 최대 {np.abs(np.diff(d['dql'][:, 1])).max():.3f} rad/s")
        for j, jn in ((1, "J2"), (2, "J3"), (0, "J1")):
            ac = lambda v: v[m] - np.mean(v[m])
            print(f"  {jn}: 시뮬레이션 실효값 {np.degrees(rms(sim[:, j])) * 1e3:6.1f}, 최대 {np.degrees(np.abs(sim[m, j]).max()) * 1e3:6.1f} mdeg | 식/시뮬레이션 실효값 {rms(e[:, j]) / rms(sim[:, j]):.2f}, 최대 {np.abs(e[m, j]).max() / np.abs(sim[m, j]).max():.2f}, 상관 {np.corrcoef(e[m, j], sim[m, j])[0, 1]:.3f}, 차이의 실효값/신호 {rms(e[:, j] - sim[:, j]) / rms(sim[:, j]):.2f}")
        np.savez(f"{H}/out/modelval_eq_{tag}.npz", t=t, sim=sim, eq=e, m=m, i_f=i_f, i_r=i_r)
    fig, axs = plt.subplots(2, 2, figsize=(170 * MM, 78 * MM), sharex="col"); fig.subplots_adjust(left=0.08, right=0.982, top=0.93, bottom=0.12, hspace=0.12, wspace=0.17)
    for c, (tag, lab) in enumerate((("A_tk15", "pose A"), ("H_tk15", "pose B"))):
        z = np.load(f"{H}/out/modelval_eq_{tag}.npz"); t, sim, e = z["t"], z["sim"], z["eq"]; t0 = t[int(z["i_f"])]
        for r, (j, jn) in enumerate(((1, "Joint 2"), (2, "Joint 3"))):
            ax = axs[r, c]; ax.plot(t - t0, np.degrees(sim[:, j]), color="k", lw=0.9, label="Simulation (physics engine)"); ax.plot(t - t0, np.degrees(e[:, j]), color="#d62728", lw=0.7, ls=(0, (4, 2)), label="Eqs. (1), (4), (7)")
            ax.set_xlim(-0.2, t[int(z["i_r"])] - t0 + 0.9); ax.tick_params(pad=2)
            if c == 0: ax.set_ylabel(f"{jn} deflection [deg]")
            if r == 0: ax.set_title(f"({'ab'[c]}) {lab}", fontsize=9.5, pad=3)
            if r == 1: ax.set_xlabel("Time from front-wheel contact [s]")
            if r == 0 and c == 0: ax.legend(loc="upper left", fontsize=8.5, frameon=False, handlelength=2.2, borderaxespad=0.2)
    fig.savefig(H + "/figs/kroc_modelval.png", dpi=600); print("\n→ figs/kroc_modelval.png")
