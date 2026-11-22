"""관절 강성(K2, K3, K5) 후보를 모드 해석으로 훑는다. 입력: out/modal_*.npz (t0_modal.py)
목표: 자세 A 7.0 Hz(6.6-7.3), 자세 B 6.25 Hz(6.2-6.3), 가진 계수(1자유도 모형 대비 배율) 2.9-3.8, 80 g 얹으면 약 -1 Hz
"""
import itertools, sys
import numpy as np
d = {k: np.load(f"out/modal_{k}.npz") for k in ("A_0", "B_0", "A_0.08", "B_0.08")}
p = np.array([0, 1, 1, 0, 1, 0.0])   # 끝단 끄덕임 행
KBIG = 1e5
def modes(M, K):
    L = np.linalg.cholesky(M); Li = np.linalg.inv(L)
    w2, Y = np.linalg.eigh(Li @ np.diag(K) @ Li.T); V = Li.T @ Y   # V^T M V = I
    return np.sqrt(w2) / 2 / np.pi, V
def first(key, K, src=1):
    M = d[key]["M"][:6, :6]
    f, V = modes(M, K)
    best = None
    for i in range(6):                   # 끄덕임 성분이 가장 큰 저차 모드
        gam = (p @ V[:, i]) * (V[:, i] @ M[:, src])
        if best is None or (abs(p @ V[:, i]) > 0.5 * abs(p @ best[2]) and f[i] < best[0] and abs(gam) > 0.3):
            best = (f[i], gam, V[:, i])
    i0 = int(np.argmin(f))
    gam0 = (p @ V[:, i0]) * (V[:, i0] @ M[:, src])
    return f[i0], gam0, V[:, i0], f
rows = []
for K2, K3, K5 in itertools.product(np.geomspace(300, 20000, 40), np.geomspace(150, 20000, 40), np.geomspace(15, 5000, 40)):
    K = np.array([KBIG, K2, K3, KBIG, K5, KBIG])
    fA, gA, vA, _ = first("A_0", K)
    if not 6.8 < fA < 7.2: continue
    fB, gB, vB, _ = first("B_0", K)
    fAp = first("A_0.08", K)[0]
    rows.append((K2, K3, K5, fA, fB, fAp - fA, gA, gB))
rows = np.array(rows)
print("후보 수", len(rows))
err = ((rows[:, 4] - 6.25) / 0.2) ** 2
for name, sel in (("fB 우선", np.argsort(err)[:12]), ("배율 큰 순(fB 5.9-6.6)", None)):
    if sel is None:
        m = (rows[:, 4] > 5.9) & (rows[:, 4] < 6.6)
        sub = rows[m]; sel2 = np.argsort(-np.abs(sub[:, 6]))[:12]; show = sub[sel2]
    else:
        show = rows[sel]
    print("==", name); print("     K2      K3      K5    fA    fB   dF80g  gamA  gamB")
    for r in show: print("%7.0f %7.0f %7.0f  %.2f  %.2f  %+.2f  %+.2f %+.2f" % tuple(r))
