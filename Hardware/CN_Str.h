#ifndef __CN_STR_H
#define __CN_STR_H
/* 中文字符串的 UTF-8 十六进制转义宏, 由 gen_cn.py 自动生成. 请勿手改.
   用法: OLED_ShowChinese(x, y, CN_XXX);  (右侧注释为实际文字) */

#define CN_MODE        "\xE6\xA8\xA1\xE5\xBC\x8F"                        /* 模式 */
#define CN_TEXT        "\xE5\xAD\x97\xE7\xAC\xA6\xE6\xB5\x8B\xE8\xAF\x95" /* 字符测试 */
#define CN_GRAPH       "\xE5\x9B\xBE\xE5\xBD\xA2\xE6\xB5\x8B\xE8\xAF\x95" /* 图形测试 */
#define CN_WAVE        "\xE6\xB3\xA2\xE5\xBD\xA2\xE6\x98\xBE\xE7\xA4\xBA" /* 波形显示 */
#define CN_CNTEST      "\xE6\xB1\x89\xE5\xAD\x97\xE6\xB5\x8B\xE8\xAF\x95" /* 汉字测试 */
#define CN_SINE        "\xE6\xAD\xA3\xE5\xBC\xA6\xE6\xB3\xA2"            /* 正弦波 */
#define CN_SQUARE      "\xE6\x96\xB9\xE6\xB3\xA2"                        /* 方波 */
#define CN_TRIANGLE    "\xE4\xB8\x89\xE8\xA7\x92\xE6\xB3\xA2"            /* 三角波 */
#define CN_FREQ        "\xE9\xA2\x91\xE7\x8E\x87"                        /* 频率 */
#define CN_VPP         "\xE5\xB3\xB0\xE5\xB3\xB0\xE5\x80\xBC"            /* 峰峰值 */
#define CN_DCMODE      "\xE7\x9B\xB4\xE6\xB5\x81\xE6\xA8\xA1\xE5\xBC\x8F" /* 直流模式 */
#define CN_AC          "\xE4\xBA\xA4\xE6\xB5\x81"                        /* 交流 */
#define CN_COUNT       "\xE8\xAE\xA1\xE6\x95\xB0"                        /* 计数 */
#define CN_SHORTKEY    "\xE7\x9F\xAD\xE6\x8C\x89\xE5\x88\x87\xE6\x8D\xA2" /* 短按切换 */
#define CN_LONGKEY     "\xE9\x95\xBF\xE6\x8C\x89\xE5\xA4\x8D\xE4\xBD\x8D" /* 长按复位 */
#define CN_VOLT        "\xE7\x94\xB5\xE5\x8E\x8B"                        /* 电压 */
#define CN_STATE       "\xE7\x8A\xB6\xE6\x80\x81"                        /* 状态 */
#define CN_CALIB       "\xE6\xA0\xA1\xE5\x87\x86"                        /* 校准 */
#define CN_SAMPLE      "\xE9\x87\x87\xE6\xA0\xB7"                        /* 采样 */

#endif
