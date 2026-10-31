#ifndef __OLED_H
#define __OLED_H

/* 屏幕分辨率：128 × 64 像素 */
#define OLED_W  128
#define OLED_H  64

/*==================== 基础 ====================*/
void OLED_Init(void);
void OLED_Clear(void);      /* 清空帧缓冲（仅内存），不写入显示屏 */
void OLED_Flush(void);      /* 将帧缓冲一次性写入显示屏 */
uint8_t *OLED_FB_Get(void); /* 帧缓冲首地址（1024 字节，page-major）；便于在 PC 端编译时读取整屏内容 */

/*==================== 像素 / 图形（X:0~127，Y:0~63） ====================*/
void    OLED_SetPixel(uint8_t X, uint8_t Y, uint8_t Color);   /* Color：1=点亮，0=熄灭 */
uint8_t OLED_GetPixel(uint8_t X, uint8_t Y);
void    OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, uint8_t Color);
void    OLED_DrawFrame(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color);   /* 空心矩形 */
void    OLED_FillRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color);    /* 实心矩形 */

/*==================== 文本（沿用常见 8×16 OLED 例程接口：Line 1~4，Column 1~16） ====================*/
/* 说明：这些函数按整字节写入帧缓冲，属覆盖式绘制，会替换同一区域内已有的像素 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/*==================== 5×7 半高文本（按像素定位，仅 Wave 页使用） ====================*/
/* 8×16 字库每个字符行高 16 像素；改用 5×7 后两行小字合计只占 y=0..15，y=16..63 全部留给波形。未收录字符按空格处理。 */
uint8_t OLED_Text5x7(uint8_t x, uint8_t y, char *s);                    /* 返回文本结束后的下一个可写 x 坐标 */
uint8_t OLED_Num5x7(uint8_t x, uint8_t y, uint32_t v, uint8_t len);     /* len=0 时按实际位数输出，否则右对齐补足到 len 位 */

/* Wave 页整页布局（两行小字 + 坐标系 + 迹线）。迹线的行号由 Capture_WaveRows 预先计算，
   本函数只负责绘制；box_y/box_h 为波形框的起始行与高度，取值与 Capture.h 中的 WAVE_BOX_* 一致。 */
void OLED_WavePage(uint32_t vpp_mv, uint32_t freq_hz, int32_t dc_mv, uint8_t have_num, uint8_t have_wave,
                   const uint8_t *ylo, const uint8_t *yhi, uint16_t n,
                   uint16_t vdiv_mv, float tdiv_ms, int16_t y0v,
                   uint8_t box_y, uint8_t box_h);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/*==================== 汉字 / 混排字符串（像素坐标 X:0~127，Y:0~63） ====================*/
/* 逐像素绘制，Y 可为任意行值（不要求 page 对齐）；字符串可混合 UTF-8 汉字（16×16）与 ASCII（8×16）。
   源文件须以 UTF-8 保存（与本工程其余文件一致），中文字面量才能与字模表对应。 */
void OLED_ShowChinese(uint8_t X, uint8_t Y, const char *Str);

/*==================== 波形 ====================*/
/* 把一段采样点连成折线绘制在指定矩形区域内，纵向自动缩放。
   buf：n 个采样值；(x0,y0,w,h)：显示区域（像素）；不绘制坐标轴。 */
void OLED_DrawWave(const uint16_t *buf, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
void OLED_DrawWaveEnv(const uint16_t *lo, const uint16_t *hi, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
void OLED_DrawGrid(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint8_t nx, uint8_t ny, int16_t y0v);
void OLED_DrawWaveScaled(const uint8_t *ylo, const uint8_t *yhi, uint16_t n, uint8_t x0, uint8_t w);

#endif
