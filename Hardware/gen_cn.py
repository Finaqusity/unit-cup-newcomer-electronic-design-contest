# -*- coding: utf-8 -*-
"""
gen_cn.py  ——  OLED 汉字字库 & 转义宏 生成工具
用法:
  1. 在下方 CN_WORDS 里增删词条 (宏名 -> 中文字符串)
  2. 命令行运行:  python gen_cn.py
  3. 它会自动重写同目录下的:
       OLED_Font_CN.h  (16x16 点阵, 按 Unicode 码点索引, 供 OLED.c 用)
       CN_Str.h        (每个词条的 UTF-8 十六进制转义 #define, 供 main.c 用)
  4. 在 .c 里: #include "CN_Str.h"  然后 OLED_ShowChinese(x, y, CN_XXX);
说明: 中文字面量必须用转义宏, 不能直接写汉字, 否则 ARMCC v5(GBK locale)编译报错.
"""
import os
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "C:/Windows/Fonts/simhei.ttf"
HERE = os.path.dirname(os.path.abspath(__file__))

# ==== 在这里维护需要的中文词 (宏名, 中文) ====
CN_WORDS = [
    ("CN_MODE",     "模式"),
    ("CN_TEXT",     "字符测试"),
    ("CN_GRAPH",    "图形测试"),
    ("CN_WAVE",     "波形显示"),
    ("CN_CNTEST",   "汉字测试"),
    ("CN_SINE",     "正弦波"),
    ("CN_SQUARE",   "方波"),
    ("CN_TRIANGLE", "三角波"),
    ("CN_FREQ",     "频率"),
    ("CN_VPP",      "峰峰值"),
    ("CN_DCMODE",   "直流模式"),
    ("CN_AC",       "交流"),
    ("CN_COUNT",    "计数"),
    ("CN_SHORTKEY", "短按切换"),
    ("CN_LONGKEY",  "长按复位"),
    ("CN_VOLT",     "电压"),
    ("CN_STATE",    "状态"),
    ("CN_CALIB",    "校准"),
    ("CN_SAMPLE",   "采样"),
]
CN_WORDS = [(n, s) for (n, s) in CN_WORDS if s is not None and n.replace("_", "").isalnum()]

SS = 8  # 超采样倍数


def render(ch):
    size = 16 * SS
    big = Image.new("L", (size, size), 0)
    d = ImageDraw.Draw(big)
    f = ImageFont.truetype(FONT_PATH, int(15 * SS))
    bb = d.textbbox((0, 0), ch, font=f)
    w, h = bb[2] - bb[0], bb[3] - bb[1]
    d.text(((size - w) // 2 - bb[0], (size - h) // 2 - bb[1]), ch, font=f, fill=255)
    return big.resize((16, 16), Image.LANCZOS)


def to_bytes(img, thr=110):
    px = img.load()
    out = [0] * 32
    for x in range(16):
        t = b = 0
        for y in range(8):
            if px[x, y] > thr:
                t |= (1 << y)
        for y in range(8, 16):
            if px[x, y] > thr:
                b |= (1 << (y - 8))
        out[x] = t
        out[16 + x] = b
    return out


def esc(s):
    r = []
    for ch in s:
        if ord(ch) < 0x80:
            r.append(ch)
        else:
            for bb in ch.encode("utf-8"):
                r.append("\\" + "x%02X" % bb)
    return "".join(r)


def main():
    words = CN_WORDS
    # 收集全部唯一汉字(保序)
    chars = []
    for _, s in words:
        for ch in s:
            if ord(ch) > 0x7F and ch not in chars:
                chars.append(ch)

    # ---- OLED_Font_CN.h ----
    L = []
    L.append("#ifndef __OLED_FONT_CN_H")
    L.append("#define __OLED_FONT_CN_H")
    L.append("")
    L.append("/* 16x16 汉字点阵, 阴码/列行式, 存储格式与 OLED_F8x16 一致:")
    L.append("   每字32字节 = 前16字节(上半page,每列1字节,bit0=最上行) + 后16字节(下半page).")
    L.append("   由 gen_cn.py + SimHei 自动生成; 按 Unicode 码点索引. 请勿手改. */")
    L.append("")
    L.append("const uint16_t OLED_CN16_Code[] = { %s };" %
             ", ".join("0x%04X" % ord(c) for c in chars))
    L.append("")
    L.append("const uint8_t OLED_CN16[][32] =")
    L.append("{")
    for c in chars:
        g = to_bytes(render(c))
        L.append("\t%s,  /* %s U+%04X */" % (",".join("0x%02X" % v for v in g[:16]), c, ord(c)))
        L.append("\t%s," % (",".join("0x%02X" % v for v in g[16:])))
    L.append("};")
    L.append("")
    L.append("#define OLED_CN16_COUNT (sizeof(OLED_CN16_Code)/sizeof(OLED_CN16_Code[0]))")
    L.append("")
    L.append("#endif")
    L.append("")
    with open(os.path.join(HERE, "OLED_Font_CN.h"), "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(L))

    # ---- CN_Str.h ----
    S = []
    S.append("#ifndef __CN_STR_H")
    S.append("#define __CN_STR_H")
    S.append("/* 中文字符串的 UTF-8 十六进制转义宏, 由 gen_cn.py 自动生成. 请勿手改.")
    S.append("   用法: OLED_ShowChinese(x, y, CN_XXX);  (右侧注释为实际文字) */")
    S.append("")
    for n, s in words:
        e = esc(s)
        pad = " " * max(1, 48 - len(e))
        S.append('#define %-14s "%s"%s/* %s */' % (n, e, pad, s))
    S.append("")
    S.append("#endif")
    S.append("")
    with open(os.path.join(HERE, "CN_Str.h"), "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(S))

    print("生成完成: %d 个词条, %d 个汉字" % (len(words), len(chars)))
    print("汉字集:", "".join(chars))
    print("文件: OLED_Font_CN.h, CN_Str.h (位于 %s)" % HERE)


if __name__ == "__main__":
    main()
