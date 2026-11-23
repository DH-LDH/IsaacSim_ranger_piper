"""녹화한 두 조건(대책 없음 / 대책 적용)을 나란히 붙이고 아래에 끝단 흔들림 시계열을 그린다.
  ~/isaacsim/python.sh rec_combine.py <prefix> <조건 A> <조건 B> "<제목>" "<A 이름>" "<B 이름>" <out.mp4>
입력: <prefix>_<조건>.mp4·.npz (exp6_bump.py --rec). 글자는 PIL(한글 글꼴), 영상은 OpenCV."""
import sys
import cv2, numpy as np
from PIL import Image, ImageDraw, ImageFont
pre, ca, cb, title, la, lb, out = sys.argv[1:8]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
f_t, f_l, f_s = ImageFont.truetype(FONT, 34, index=1), ImageFont.truetype(FONT, 26, index=1), ImageFont.truetype(FONT, 20, index=1)   # index 1 = KR
W, H, PH, TH = 960, 540, 230, 64
D = [np.load(f"{pre}_{c}.npz") for c in (ca, cb)]; V = [cv2.VideoCapture(f"{pre}_{c}.mp4") for c in (ca, cb)]
dev = [d["dev"] for d in D]; n = min(len(dev[0]), len(dev[1])); fps = 25.0
ymax = max(1.0, float(np.ceil(max(dev[0][:n].max(), dev[1][:n].max()))))
skip = int(0.4 * fps)                                    # 시작 직후는 표시용 필터가 자리 잡는 구간이라 실효값에서 뺀다
rms = [float(np.sqrt(np.mean(d[skip:n] ** 2))) for d in dev]
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (2 * W, TH + H + PH))
COL = [(90, 90, 90), (31, 78, 121)]
def text(img, items):
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); dr = ImageDraw.Draw(im)
    for xy, s, f, c in items: dr.text(xy, s, font=f, fill=c)
    return cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
x0, x1, y0, y1 = 70, W - 20, TH + H + 40, TH + H + PH - 40
def px(k, v, off): return (int(off + x0 + (x1 - x0) * k / max(1, n - 1)), int(y1 - (y1 - y0) * min(v, ymax) / ymax))
for k in range(n):
    fr = []
    for v in V:
        ok, f = v.read(); fr.append(f if ok else np.zeros((H, W, 3), np.uint8))
    img = np.full((TH + H + PH, 2 * W, 3), 255, np.uint8); img[TH:TH + H, :W] = fr[0]; img[TH:TH + H, W:] = fr[1]
    for s_ in (0, 1):
        off = s_ * W
        cv2.rectangle(img, (off + x0, y0), (off + x1, y1), (200, 200, 200), 1)
        for g in range(1, int(ymax)): cv2.line(img, (off + x0, px(0, g, off)[1]), (off + x1, px(0, g, off)[1]), (230, 230, 230), 1)
        pts = np.array([px(j, dev[s_][j], off) for j in range(k + 1)], np.int32)
        if len(pts) > 1: cv2.polylines(img, [pts], False, COL[s_][::-1], 2, cv2.LINE_AA)
        cv2.line(img, (px(k, 0, off)[0], y0), (px(k, 0, off)[0], y1), (0, 0, 220), 1)
    items = [((24, 12), title, f_t, (0, 0, 0)), ((16, TH + 10), la, f_l, (255, 255, 255)), ((W + 16, TH + 10), lb, f_l, (255, 255, 255))]
    for s_ in (0, 1):
        off = s_ * W
        items += [((off + x0, TH + H + 8), f"끝단 흔들림(궤적 이탈의 빠른 성분) [mm]   전체 구간 실효값 {rms[s_]:.2f} mm", f_s, COL[s_]),
                  ((off + 8, y0 - 12), f"{ymax:.0f}", f_s, (90, 90, 90)), ((off + 22, y1 - 12), "0", f_s, (90, 90, 90)), ((off + x1 - 150, y1 + 6), f"시간 {k / fps:4.1f} s", f_s, (90, 90, 90))]
    vw.write(text(img, items))
vw.release(); print(out, "frames", n, "rms", [round(r, 2) for r in rms], "ratio %.2f" % (rms[1] / rms[0]))
