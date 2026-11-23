"""한 줄로 보기: 케이스와 제어기를 고르면 맞는 장면을 띄운다.

  ~/isaacsim/python.sh show.py <케이스> <제어기> [추가 옵션]

케이스       제어기
  start      none | zv | zvd | scurve                 AMR 출발·정지. 속도 명령 성형
  bump       none | ff | phase | real | ffb | rear | rearz | rearzfb | preview | previewpos | previewfb
  pothole    (bump와 같음)                            패인 바닥
  arm        pos | tde | asmc                         실측 팔 동작 기록 재생 + 베이스 앞뒤 이동 (--scn 1/2/3 으로 1·2·3축)

bump·pothole의 제어기
  ff         베이스 IMU 피드포워드(지연 10 ms)            phase      끝단 되먹임 + 위상 예측(지연 20 ms)
  real       끝단 되먹임, 실기 같은 지연(100 Hz, 들쭉날쭉)     ffb        피드포워드 + 위상 예측 되먹임(10 ms)
  rear       뒷바퀴 예측 그대로                            rearz      닮은 성분만 예측 + 잰 값
  rearzfb    rearz + 위상 예측 되먹임                      preview    지형을 미리 앎(지도 기반 예견)
  previewpos 예견을 위치 모드로                            previewfb  예견 + 실기 같은 되먹임
bump·pothole은 "대책 없음 → 고른 제어기"를 번갈아 틀고, 그때마다 끝단 진동과 베이스 진동을 터미널에 찍는다.
추가 옵션 예: --h 0.005(턱 높이·홈 깊이 m)  --v 0.15(속도)  --slow 2(느리게)  --mount-x 0(팔을 가운데로)
"""
import os, sys

H = os.path.dirname(os.path.abspath(__file__))
TOK = {"none": "none", "ff": "ff10", "phase": "damp20p", "real": "dampJ", "ffb": "ff10+damp10p", "rear": "prev", "rearz": "prevzE10+ff10",
       "rearzfb": "prevzE10+ff10+damp10p", "preview": "map", "previewpos": "mapservo", "previewfb": "map+dampJg"}
if len(sys.argv) < 3 or sys.argv[1] not in ("start", "bump", "pothole", "arm"):
    print(__doc__); sys.exit(0)
case, ctrl, extra = sys.argv[1], sys.argv[2], sys.argv[3:]
if case == "start":
    cmd = [H + "/view.py", "--scene", "amr", "--case", ctrl]
elif case == "arm":
    cmd = [H + "/view_replay.py", "--ctrl", ctrl, "--base", "move"] + (["--scn", "1"] if "--scn" not in extra else [])
else:
    tok = TOK.get(ctrl, ctrl)
    cmd = [H + "/exp6_bump.py", "--gui", "--loops", "0", "--tag", "show", "--cases", "none" if tok == "none" else "none," + tok]
    if case == "pothole":
        cmd += ["--pothole", "0.06"]
    if "--h" not in extra: cmd += ["--h", "0.01"]
    if "--v" not in extra: cmd += ["--v", "0.3"]
os.environ.setdefault("VIBSIM_GUI", "1")
print("실행:", " ".join(os.path.basename(c) if c.endswith(".py") else c for c in cmd + extra), flush=True)
os.execv(sys.executable, [sys.executable] + cmd + extra)
