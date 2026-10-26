#ifndef __OLED_H
#define __OLED_H

/* 屏幕分辨率: 128 x 64 */
#define OLED_W  128
#define OLED_H  64

/*==================== 基础 ====================*/
void OLED_Init(void);
void OLED_Clear(void);      /* 清空帧缓冲(内存), 不上屏 */
void OLED_Flush(void);      /* 把帧缓冲一次性刷到屏幕 */

/*==================== 像素 / 图形 (X:0~127, Y:0~63) ====================*/
void    OLED_SetPixel(uint8_t X, uint8_t Y, uint8_t Color);   /* Color: 1亮 0灭 */
uint8_t OLED_GetPixel(uint8_t X, uint8_t Y);
void    OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, uint8_t Color);
void    OLED_DrawFrame(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color);   /* 空心矩形 */
void    OLED_FillRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color);    /* 实心矩形 */

/*==================== 文本 (按行/列定位: Line 1~4, Column 1~16) ====================*/
/* 这类文本接口按整字节写入帧缓冲，覆盖同一区域的已有像素（不做叠加） */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/*==================== 汉字 / 混排字符串 (像素坐标 X:0~127, Y:0~63) ====================*/
/* 逐像素绘制, Y 可为任意值(不必 page 对齐); 字符串可混合 UTF-8 汉字(16x16)与 ASCII(8x16).
   源文件需以 UTF-8 保存(与本工程一致), 中文字面量才能与字模表对应. */
void OLED_ShowChinese(uint8_t X, uint8_t Y, const char *Str);

/*==================== 波形 ====================*/
/* 把一段采样点连成折线画进指定矩形区域, 自动纵向缩放.
   buf: n 个采样值; (x0,y0,w,h): 显示区域; 不画坐标轴. */
void OLED_DrawWave(const uint16_t *buf, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
void OLED_DrawWaveEnv(const uint16_t *lo, const uint16_t *hi, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);

#endif
