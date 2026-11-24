"""타이어(바퀴 접촉) 강성 민감도: 기존 128칸(1.5e5 N/m·1000 N·s/m, exp6_bump.py 기본값)과 2e5 N/m·1200 N·s/m로 다시 낸 값(태그 tk_)을 나란히 → figs/matrix_tire_sens.md
값은 끝단 위치 오차(월드 기준 3차원 거리)의 최대 / 실효값[mm]. 실행: ~/nonmoon/.venv_pptx/bin/python summ_tk.py
두 값 모두 가정(실측 없음). 기존 값이 강체 접촉이라고 본 것은 잘못이었다(11/24 밤 바로잡음). 근거: 볼트 "PiPER + Ranger Mini 시스템 모델" 9절"""
import os
import numpy as np
import summ_line as S
H = os.path.dirname(os.path.abspath(__file__)); PR = "kinprev+prevzE10+ff10"; FL = "kin10+ff10"
POSE = [("A", "rl", "bzc", "기존 자세(그리퍼 아래)"), ("H", "rlH", "bzcH", "횡 방향 자세(그리퍼 앞)")]; LINE = [("z", "위아래 직선"), ("x", "팔 앞뒤 직선")]
DIST = [("h5", "턱 5 mm, 0.3 m/s"), ("base", "턱 10 mm, 0.3 m/s"), ("h15", "턱 15 mm, 0.3 m/s"), ("v15", "턱 10 mm, 0.15 m/s"), ("v50", "턱 10 mm, 0.5 m/s"), ("pot", "패인 바닥 10 mm, 0.3 m/s")]


def cell(r): return f"{r['d3_pk']:.1f} / {r['d3_rms']:.2f}" if r else "—"


def rows(pf, p, pre, bz, k):
    out = []
    for tg, lab in DIST:
        no, arm = S.one(f"{pf}{pre}_{k}_j23_{tg}", "none"), S.one(f"{pf}{pre}_{k}_j23_{tg}", PR)
        t2 = {"base": f"{bz}_{k}_b10", "pot": f"{bz}_{k}_pot"}.get(tg, f"fill_{p}_{k}_{tg}")
        out.append((lab, no, S.one(pf + t2, "bz"), arm, S.one(pf + t2, "bz+" + PR)))
    for v, lab in ((30, "평지 가감속, 0.3 m/s"), (50, "평지 가감속, 0.5 m/s")):
        tg = f"fl2_{k}_j23_v{v}" if p == "A" else (f"fill_H_{k}_flat30" if v == 30 else f"rlH_{k}_j23_flat")
        out.append((lab, S.one(pf + tg, "none"), S.one(pf + tg, "bz"), S.one(pf + tg, FL), S.one(pf + tg, "bz+" + FL)))
    return out


def main():
    md = ["# 타이어(바퀴 접촉) 강성 민감도 — 기존 값(1.5×10⁵ N/m)과 2×10⁵ N/m로 다시 낸 값", "",
          "값은 끝단 위치 오차(월드 기준 3차원 거리)의 **최대 / 실효값 [mm]**. 그 밖의 조건은 기존 128칸 표와 같다(토크 제어 + 추종 제어기 2·3번 관절, 실측 직선 기록).",
          "두 강성 모두 가정이다(레인저 미니의 타이어를 잰 적이 없다). 감쇠는 1000 / 1200 N·s/m.", ""]
    ALL = {"": [], "tk_": []}; n_new = 0
    for p, pre, bz, pn in POSE:
        for k, kn in LINE:
            so, sn = S.one(f"{pre}_{k}_j23_still", "none"), S.one(f"tk_{pre}_{k}_j23_still", "none")
            md += [f"## {pn} — {kn}", "", f"AMR 정지(팔만 움직임): 기존 **{cell(so)}**, 다시 **{cell(sn)}**", "",
                   "| 외란 | 접촉 | 보정 없음 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 | 로봇팔 보정/없음 (최대, 실효값) |", "|---|---|---|---|---|---|---|"]
            ro, rn = rows("", p, pre, bz, k), rows("tk_", p, pre, bz, k)
            for (lab, *a), (_, *b) in zip(ro, rn):
                for nm, rs, key in (("기존(1.5e5)", a, ""), ("다시(2e5)", b, "tk_")):
                    rt = f"{rs[2]['d3_pk'] / rs[0]['d3_pk']:.2f}, {rs[2]['d3_rms'] / rs[0]['d3_rms']:.2f}" if rs[0] and rs[2] else "—"
                    md.append(f"| {lab if nm == '기존(1.5e5)' else ''} | {nm} | " + " | ".join(cell(r) for r in rs) + f" | {rt} |"); ALL[key].append((p, k, lab, rs, so if key == "" else sn))
                n_new += sum(1 for r in b if r)
            md.append("")
    md += [f"2e5로 채워진 칸: {n_new} / 128", "", "## 보정 없음 대비 비 — 가운데 값(범위), 네 묶음(자세 2 × 직선 2)을 합쳐서", "",
           "| 외란 | 지표 | 접촉 | AMR 보정만 | 로봇팔 보정만 | AMR + 로봇팔 | 로봇팔 보정 / AMR 보정 |", "|---|---|---|---|---|---|---|"]
    for grp, sel in (("턱·패인 바닥", lambda l: "평지" not in l), ("평지 가감속", lambda l: "평지" in l)):
        for key, kl in (("d3_pk", "최대"), ("d3_rms", "실효값")):
            for pf, nm in (("", "기존(1.5e5)"), ("tk_", "다시(2e5)")):
                A = np.array([[r[key] / rs[0][key] for r in rs[1:]] + [rs[2][key] / rs[1][key]] for _, _, lab, rs, _ in ALL[pf] if sel(lab) and all(rs)])
                if len(A) == 0: md.append(f"| {grp} | {kl} | {nm} | — | — | — | — |"); continue
                md.append(f"| {grp} | {kl} | {nm} ({len(A)}건) | " + " | ".join(f"{np.median(A[:, j]):.2f} ({A[:, j].min():.2f}–{A[:, j].max():.2f})" for j in range(4)) + " |")
    md += ["", "## 절대값의 범위 [mm] — 턱·패인 바닥, 네 묶음 합쳐서", "", "| 접촉 | 보정 없음: 최대 | 실효값 | 로봇팔 보정만: 최대 | 실효값 | AMR 정지: 최대 | 실효값 |", "|---|---|---|---|---|---|---|"]
    for pf, nm in (("", "기존(1.5e5)"), ("tk_", "다시(2e5)")):
        R = [x for x in ALL[pf] if "평지" not in x[2] and x[3][0] and x[3][2]]
        if not R: continue
        f = lambda v: f"{min(v):.1f}–{max(v):.1f}"; st = [x[4] for x in R if x[4]]
        md.append(f"| {nm} | {f([x[3][0]['d3_pk'] for x in R])} | {f([x[3][0]['d3_rms'] for x in R])} | {f([x[3][2]['d3_pk'] for x in R])} | {f([x[3][2]['d3_rms'] for x in R])} | " + (f"{f([s['d3_pk'] for s in st])} | {f([s['d3_rms'] for s in st])} |" if st else "— | — |"))
    # 세 번 돌린 조건(턱 10 mm, 0.3 m/s)
    md += ["", "## 턱 10 mm·0.3 m/s를 세 번 돌린 값(2e5) — 최대 / 실효값 [mm]", "", "| 자세·직선 | 보정 없음 | 로봇팔 보정만 |", "|---|---|---|"]
    for p, pre, bz, pn in POSE:
        for k, kn in LINE:
            c = lambda cs: ", ".join(cell(S.one(f"tk_{pre}_{k}_j23_{t}", cs)) for t in ("base", "base_s2", "base_s3"))
            md.append(f"| {pn.split('(')[0]}, {kn} | {c('none')} | {c(PR)} |")
    out = H + "/figs/matrix_tire_sens.md"; open(out, "w").write("\n".join(md) + "\n"); print("\n".join(md)); print("→", out)


if __name__ == "__main__":
    main()
