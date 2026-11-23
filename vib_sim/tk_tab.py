"""exp6 종합 장면 CSV를 한 줄씩:  python3 tk_tab.py tk5_s1_pos tk5_s1_pd ..."""
import csv, sys
for tag in sys.argv[1:]:
    try: R = list(csv.DictReader(open(f"out/exp6_{tag}.csv")))
    except FileNotFoundError: print(tag, "없음"); continue
    b = R[0]
    for r in R:
        f = lambda k: float(r[k])
        print(tag.ljust(14), r["case"].ljust(26), "각속도 %.4f(%.2f) 끄덕임 %.4f 좌우 %.4f | 가속 %.2f/%.2f | 흔들림 %.2f mm(%.2f) 오차 %.2f 최대 %.2f | 관절 오차 %.3f %.3f %.3f° | 미끄러진 비율 %s | 포화 %.2f"
              % (f("full_vec"), f("full_vec") / float(b["full_vec"]), f("full_pitch"), f("full_yaw"), f("full_acc_lo"), f("full_acc_hi"), f("tip_vib_mm"), f("tip_vib_mm") / float(b["tip_vib_mm"]), f("tip_err_mm"), f("tip_peak_mm"),
                 f("err_j1"), f("err_j2"), f("err_j3"), " ".join("%.2f" % float(r[f"slip_j{j}"]) if r.get(f"slip_j{j}", "nan") not in ("nan", "") else "—" for j in (1, 2, 3)), f("sat")))
