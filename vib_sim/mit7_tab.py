"""exp7 로그의 ROW 줄을 모아 한 줄씩 보여 준다:  python3 mit7_tab.py m2 [m3 ...]"""
import glob, sys
nan = float("nan")
class np:  # 로그의 np.float64(...) 표기를 읽기 위한 것
    float64 = float; str_ = str
for tag in sys.argv[1:]:
    for f in sorted(glob.glob(f"out/exp7_{tag}_*.log")):
        for l in open(f):
            if l.startswith("ROW"):
                r = eval(l[4:])
                print(f[len(f"out/exp7_{tag}_"):-4].ljust(16), "처짐 %.2f/%.2f" % (r["J2_sag_sim"], r["J2_sag_real"]), "| J2 오차 %.3f/%.3f(%.2f배) 상관 %.2f" % (r["J2_err_sim"], r["J2_err_real"], r["J2_err_sim"] / r["J2_err_real"], r["J2_corr"]),
                      "| J1 %.3f/%.3f J3 %.3f/%.3f" % (r["J1_err_sim"], r["J1_err_real"], r["J3_err_sim"], r["J3_err_real"]),
                      "| 끝단 2–10 %.2f 5–10 %.2f 10–20 %.2f" % (r["gyro_2_10_ratio"], r["gyro_5_10_ratio"], r["gyro_10_20_ratio"]), "(모형 %.4f 실측 %.4f)" % (r["gyro_2_10_sim"], r["gyro_2_10_real"]), "변화율 제한", r["n_ratelim"])
