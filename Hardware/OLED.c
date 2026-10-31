#include "stm32f10x.h"
#include "OLED_Font.h"
#include "OLED_Font_CN.h"

/*==================== 端口配置（软件 I2C 引脚选择） ====================*/
/* OLED_USE_ONBOARD 决定 SCL/SDA 使用哪一组引脚：
   0 = 外接显示屏模块（SCL=PB10，SDA=PB11）；1 = 板载显示屏模块（SCL=PB8，SDA=PB9）
   只修改该宏并重新编译即可切换，两组引脚共用同一套初始化与读写代码。 */
#define OLED_USE_ONBOARD   1

#if OLED_USE_ONBOARD
  #define OLED_SCL_PIN   GPIO_Pin_8
  #define OLED_SDA_PIN   GPIO_Pin_9
#else
  #define OLED_SCL_PIN   GPIO_Pin_10
  #define OLED_SDA_PIN   GPIO_Pin_11
#endif

#define OLED_GPIO        GPIOB
#define OLED_GPIO_RCC    RCC_APB2Periph_GPIOB

/* 引脚电平写入宏 */
#define OLED_W_SCL(x)    GPIO_WriteBit(OLED_GPIO, OLED_SCL_PIN, (BitAction)(x))
#define OLED_W_SDA(x)    GPIO_WriteBit(OLED_GPIO, OLED_SDA_PIN, (BitAction)(x))

/*==================== 帧缓冲（128x64 = 1024 字节，page-major） ====================*/
/* OLED_FB[page][col]，page=0~7，col=0~127；每字节对应一列的 8 个竖直像素，bit0 为最上一行。
   该组织方式与 SSD1306 显存布局一致，刷屏时按页直接写入，无需二次转换。 */
static uint8_t OLED_FB[8][128];

/*==================== 软件 I2C ====================*/
void OLED_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(OLED_GPIO_RCC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;   /* 开漏输出 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = OLED_SCL_PIN;
	GPIO_Init(OLED_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = OLED_SDA_PIN;
	GPIO_Init(OLED_GPIO, &GPIO_InitStructure);

	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/**
  * @brief  OLED写命令
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据 (单字节)
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/**
  * @brief  OLED 连续写数据（单个 I2C 事务内连写多字节，减少起始与停止条件的开销，用于提高刷屏速率）
  */
void OLED_WriteDataBurst(const uint8_t *Data, uint16_t Length)
{
	uint16_t i;
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	for (i = 0; i < Length; i++)
	{
		OLED_I2C_SendByte(Data[i]);
	}
	OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y page, 0~7 ; X 列, 0~127
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置page
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置列高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置列低4位
}

/*==================== 帧缓冲操作 ====================*/

/**
  * @brief  清空帧缓冲 (仅清内存, 需再调 OLED_Flush 才生效)
  */
uint8_t *OLED_FB_Get(void) { return &OLED_FB[0][0]; }   /* 帧缓冲首地址；便于在 PC 端编译时读取整屏内容以验证布局，设备端显示流程不使用该接口 */

void OLED_Clear(void)
{
	uint8_t p; uint16_t x;
	for (p = 0; p < 8; p++)
		for (x = 0; x < 128; x++)
			OLED_FB[p][x] = 0x00;
}

/**
  * @brief  将帧缓冲一次性写入显示屏
  */
void OLED_Flush(void)
{
	uint8_t page;
	for (page = 0; page < 8; page++)
	{
		OLED_SetCursor(page, 0);
		OLED_WriteDataBurst(OLED_FB[page], 128);
	}
}

/**
  * @brief  绘制单个像素（写入帧缓冲）
  * @param  X 0~127, Y 0~63, Color 1亮/0灭
  */
void OLED_SetPixel(uint8_t X, uint8_t Y, uint8_t Color)
{
	if (X >= 128 || Y >= 64) return;              //越界保护
	if (Color) OLED_FB[Y >> 3][X] |=  (1 << (Y & 7));
	else       OLED_FB[Y >> 3][X] &= ~(1 << (Y & 7));
}

/**
  * @brief  读取单个像素
  */
uint8_t OLED_GetPixel(uint8_t X, uint8_t Y)
{
	if (X >= 128 || Y >= 64) return 0;
	return (OLED_FB[Y >> 3][X] >> (Y & 7)) & 1;
}

/**
  * @brief  绘制直线（Bresenham 算法）
  */
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, uint8_t Color)
{
	int16_t dx = (X1 > X0) ? (X1 - X0) : (X0 - X1);
	int16_t dy = (Y1 > Y0) ? (Y1 - Y0) : (Y0 - Y1);
	int16_t sx = (X0 < X1) ? 1 : -1;
	int16_t sy = (Y0 < Y1) ? 1 : -1;
	int16_t err = dx - dy, e2;
	while (1)
	{
		OLED_SetPixel((uint8_t)X0, (uint8_t)Y0, Color);
		if (X0 == X1 && Y0 == Y1) break;
		e2 = 2 * err;
		if (e2 > -dy) { err -= dy; X0 += sx; }
		if (e2 <  dx) { err += dx; Y0 += sy; }
	}
}

/**
  * @brief  绘制空心矩形
  */
void OLED_DrawFrame(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color)
{
	if (W == 0 || H == 0) return;
	OLED_DrawLine(X, Y, X + W - 1, Y, Color);
	OLED_DrawLine(X, Y + H - 1, X + W - 1, Y + H - 1, Color);
	OLED_DrawLine(X, Y, X, Y + H - 1, Color);
	OLED_DrawLine(X + W - 1, Y, X + W - 1, Y + H - 1, Color);
}

/**
  * @brief  绘制实心矩形
  */
void OLED_FillRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color)
{
	uint8_t i, j;
	for (j = 0; j < H; j++)
		for (i = 0; i < W; i++)
			OLED_SetPixel(X + i, Y + j, Color);
}

/*==================== 文本（写入帧缓冲，按 page 对齐，覆盖式绘制） ====================*/

/**
  * @brief  显示一个字符 (8x16)
  * @param  Line 1~4, Column 1~16
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	uint8_t base = (Column - 1) * 8;    //起始列
	uint8_t tp   = (Line - 1) * 2;      //起始page
	if (Line < 1 || Line > 4 || Column < 1 || Column > 16) return;
	if (Char < ' ' || Char > '~') Char = ' ';
	for (i = 0; i < 8; i++)
	{
		OLED_FB[tp    ][base + i] = OLED_F8x16[Char - ' '][i];       //上半page
		OLED_FB[tp + 1][base + i] = OLED_F8x16[Char - ' '][i + 8];   //下半page
	}
}

/**
  * @brief  显示字符串 (8x16 ASCII)
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  幂运算（X 的 Y 次方）
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--) Result *= X;
	return Result;
}

/**
  * @brief  显示数字 (十进制正数)
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  显示数字 (十进制带符号)
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0) { OLED_ShowChar(Line, Column, '+'); Number1 = Number; }
	else             { OLED_ShowChar(Line, Column, '-'); Number1 = -Number; }
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  显示数字 (十六进制)
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10) OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		else                   OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
	}
}

/**
  * @brief  显示数字 (二进制)
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/*==================== 汉字 / 混排（逐像素绘制，Y 可为任意行） ====================*/

/* 将一个 8x16 ASCII 字模绘制到像素坐标 (X,Y) */
static void OLED_DrawAscii16(uint8_t X, uint8_t Y, char Char)
{
	uint8_t i, j, col;
	if (Char < ' ' || Char > '~') Char = ' ';
	for (i = 0; i < 8; i++)                       //8列
	{
		col = OLED_F8x16[Char - ' '][i];          //上半page字节
		for (j = 0; j < 8; j++)
			if (col & (1 << j)) OLED_SetPixel(X + i, Y + j, 1);
		col = OLED_F8x16[Char - ' '][i + 8];      //下半page字节
		for (j = 0; j < 8; j++)
			if (col & (1 << j)) OLED_SetPixel(X + i, Y + 8 + j, 1);
	}
}

/* 将一个 16x16 汉字字模绘制到像素坐标 (X,Y) */
static void OLED_DrawCN16(uint8_t X, uint8_t Y, const uint8_t *glyph)
{
	uint8_t i, j, col;
	for (i = 0; i < 16; i++)                      //16列
	{
		col = glyph[i];                           //上半page
		for (j = 0; j < 8; j++)
			if (col & (1 << j)) OLED_SetPixel(X + i, Y + j, 1);
		col = glyph[16 + i];                      //下半page
		for (j = 0; j < 8; j++)
			if (col & (1 << j)) OLED_SetPixel(X + i, Y + 8 + j, 1);
	}
}

/**
  * @brief  显示汉字/ASCII混排字符串 (UTF-8 编码)
  * @param  X 0~127, Y 0~63 (16高, 建议 Y 取 0/16/32/48)
  * @param  Str 以 UTF-8 保存的字符串; 汉字占16宽, ASCII占8宽
  * @note   源文件必须以 UTF-8 编码保存(与本工程其余文件一致)
  */
void OLED_ShowChinese(uint8_t X, uint8_t Y, const char *Str)
{
	uint16_t i = 0, k;
	uint8_t b1, found;
	uint16_t cp;
	while (Str[i] != '\0' && X < 128)
	{
		b1 = (uint8_t)Str[i];
		if (b1 < 0x80)                            //ASCII (1字节)
		{
			OLED_DrawAscii16(X, Y, (char)b1);
			X += 8; i += 1;
		}
		else
		{
			if ((b1 & 0xE0) == 0xC0)              //2字节 UTF-8
			{
				cp = ((uint16_t)(b1 & 0x1F) << 6) | ((uint8_t)Str[i + 1] & 0x3F);
				i += 2;
			}
			else if ((b1 & 0xF0) == 0xE0)         //3字节 UTF-8 (常用汉字)
			{
				cp = ((uint16_t)(b1 & 0x0F) << 12) |
				     (((uint16_t)((uint8_t)Str[i + 1] & 0x3F)) << 6) |
				      ((uint8_t)Str[i + 2] & 0x3F);
				i += 3;
			}
			else                                  //非法/4字节, 跳过
			{
				i += 1;
				continue;
			}
			found = 0;
			for (k = 0; k < OLED_CN16_COUNT; k++)
			{
				if (OLED_CN16_Code[k] == cp)
				{
					OLED_DrawCN16(X, Y, OLED_CN16[k]);
					found = 1;
					break;
				}
			}
			if (!found) OLED_DrawFrame(X, Y, 16, 16, 1);  //字模表未收录该汉字：绘制 16x16 空心矩形占位，保持排版宽度不变
			X += 16;
		}
	}
}

/*==================== 波形 ====================*/

/**
  * @brief  绘制一段波形（自动纵向缩放，折线连接）
  * @param  buf n个采样值; n 点数
  * @param  x0,y0,w,h 显示区域 (像素)
  */
void OLED_DrawWave(const uint16_t *buf, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h)
{
	uint16_t i;
	uint16_t vmin, vmax;
	int16_t px, py, lx = 0, ly = 0;
	if (n == 0 || w == 0 || h == 0) return;

	vmin = 0xFFFF; vmax = 0;
	for (i = 0; i < n; i++)
	{
		if (buf[i] < vmin) vmin = buf[i];
		if (buf[i] > vmax) vmax = buf[i];
	}
	if (vmax - vmin < 4) vmax = vmin + 4;         //限制最小跨度：峰谷差小于 4 时按 4 处理，避免除零并保证迹线可见

	for (i = 0; i < n; i++)
	{
		px = x0 + (int16_t)((uint32_t)i * (w - 1) / (n - 1 ? n - 1 : 1));
		py = y0 + h - 1 - (int16_t)((uint32_t)(buf[i] - vmin) * (h - 1) / (vmax - vmin));
		if (i) OLED_DrawLine(lx, ly, px, py, 1);
		else   OLED_SetPixel((uint8_t)px, (uint8_t)py, 1);
		lx = px; ly = py;
	}
}

/* 抽样包络绘制（每周期像素数较少时的做法）：每个采样列画一条 lo[i]~hi[i] 之间的竖线。
   逐点连线在每周期只有几个像素时会因抽稀遗漏峰谷，使显示幅度偏小、方波边沿出现倾斜；
   采用 min/max 包络可保证峰值与谷值都被绘制：稳定段呈实心带，跳变沿呈陡竖线。 */
void OLED_DrawWaveEnv(const uint16_t *lo, const uint16_t *hi, uint16_t n, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h)
{
	uint16_t i;
	uint16_t vmin, vmax;
	int16_t px, ya, yb;
	if (n == 0 || w == 0 || h == 0) return;

	vmin = 0xFFFF; vmax = 0;
	for (i = 0; i < n; i++)
	{
		if (lo[i] < vmin) vmin = lo[i];
		if (hi[i] > vmax) vmax = hi[i];
	}
	if (vmax - vmin < 4) vmax = vmin + 4;         //限制最小跨度：峰谷差小于 4 时按 4 处理，避免除零并保证迹线可见

	for (i = 0; i < n; i++)
	{
		px = x0 + (int16_t)((uint32_t)i * (w - 1) / (n - 1 ? n - 1 : 1));
		ya = y0 + h - 1 - (int16_t)((uint32_t)(hi[i] - vmin) * (h - 1) / (vmax - vmin));
		yb = y0 + h - 1 - (int16_t)((uint32_t)(lo[i] - vmin) * (h - 1) / (vmax - vmin));
		if (ya == yb) OLED_SetPixel((uint8_t)px, (uint8_t)ya, 1);
		else          OLED_DrawLine(px, ya, px, yb, 1);
	}
}

/* 坐标系网格：外框由调用者绘制，本函数绘制内部分格（点线）与 0V 参考线（实线）。
   nx/ny = 水平与垂直方向的格数；格线坐标按四舍五入取整，避免不能整除时偏差逐格累积。
   y0v = 0V 所在行；该行超出绘制区时不画参考线，改在上边缘或下边缘画箭头指示 0V 方位。 */
void OLED_DrawGrid(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint8_t nx, uint8_t ny, int16_t y0v)
{
	uint8_t k, i, j;
	int16_t xi, yi;
	if (w < 2 || h < 2 || nx < 2 || ny < 2) return;

	for (k = 1; k < nx; k++) {                       /* 竖直分格线：稀疏点线，每 4 像素一个点 */
		xi = (int16_t)(x0 + (k * (w - 1) + (int16_t)(nx / 2)) / nx);
		for (j = 0; j < h; j += 4) OLED_SetPixel((uint8_t)xi, (uint8_t)(y0 + j), 1);
	}
	for (k = 1; k < ny; k++) {                       /* 水平分格线：稀疏点线 */
		yi = (int16_t)(y0 + (k * (h - 1) + (int16_t)(ny / 2)) / ny);
		for (i = 0; i < w; i += 4) OLED_SetPixel((uint8_t)(x0 + i), (uint8_t)yi, 1);
	}

	if (y0v >= (int16_t)y0 && y0v <= (int16_t)(y0 + h - 1)) {
		for (i = 0; i < w; i++) OLED_SetPixel((uint8_t)(x0 + i), (uint8_t)y0v, 1);   /* 0V 参考线（实线） */
	} else if (y0v < (int16_t)y0) {                  /* 0V 参考行位于绘制区上方：绘制向上箭头 */
		int16_t ay = (int16_t)(y0 + 1);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)ay, 1);
		OLED_SetPixel((uint8_t)(x0 + 1), (uint8_t)(ay + 1), 1);
		OLED_SetPixel((uint8_t)(x0 + 3), (uint8_t)(ay + 1), 1);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)(ay + 2), 1);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)(ay + 3), 1);
	} else {                                         /* 0V 参考行位于绘制区下方：绘制向下箭头 */
		int16_t ay = (int16_t)(y0 + h - 4);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)(ay + 3), 1);
		OLED_SetPixel((uint8_t)(x0 + 1), (uint8_t)(ay + 2), 1);
		OLED_SetPixel((uint8_t)(x0 + 3), (uint8_t)(ay + 2), 1);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)ay, 1);
		OLED_SetPixel((uint8_t)(x0 + 2), (uint8_t)(ay - 1), 1);
	}
}

/* 按调用者预先算好的像素行号绘制波形：每列在 yhi[i] 与 ylo[i] 之间画竖线（包络），单点列则与上一列连线（折线）。
   电压到像素行的映射由调用者完成：标定链属于 Algorithm 模块，显示层不承担单位换算。 */
void OLED_DrawWaveScaled(const uint8_t *ylo, const uint8_t *yhi, uint16_t n, uint8_t x0, uint8_t w)
{
	uint16_t i;
	int16_t px, ya, yb, lpx = -1, lpy = 0;
	if (n == 0 || w == 0) return;

	for (i = 0; i < n; i++)
	{
		px = x0 + (int16_t)((uint32_t)i * (w - 1) / (n - 1 ? n - 1 : 1));
		yb = yhi[i]; ya = ylo[i];                      /* yhi 对应屏幕上方（行号较小），ylo 对应下方 */
		if (yb > ya) { int16_t t = yb; yb = ya; ya = t; }
		if (yb == ya) {
			if (lpx >= 0) OLED_DrawLine(lpx, lpy, px, (int16_t)ya, 1);
			else          OLED_SetPixel((uint8_t)px, (uint8_t)ya, 1);
			lpy = ya;
		} else {
			OLED_DrawLine(px, yb, px, ya, 1);
			lpy = ya;
		}
		lpx = px;
	}
}

/*==================== 5×7 半高字库（仅 Wave 页使用） ====================
   8×16 字库每个字符行高 16 像素，两行即占用 32 像素（半屏高度）；波形页需要幅值、频率与两个坐标刻度共 4 项信息，
   因此另设一套 5 宽 7 高的字模：两行小字只占 y=0..15，y=16..63 的 48 像素全部留给波形。
   存储方式：每个字符 7 字节对应 7 行，每字节的 bit4..bit0 自左向右对应 5 列，按行式存放便于核对。
   字库仅收录 Wave 页用到的字符；未收录字符按空格处理，只留白而不绘制错误字形。 */
typedef struct { char ch; uint8_t rows[7]; } Font5x7;
static const Font5x7 F5x7[] = {
	{' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{'0', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
	{'1', {0x04,0x0C,0x14,0x04,0x04,0x04,0x0E}},   /* 字形说明：起笔短划位于左上，主竖居中（第 3 列），底部衬线跨第 2~4 列 */
	{'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
	{'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
	{'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
	{'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
	{'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},
	{'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
	{'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
	{'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x1C}},
	{'=', {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00}},
	{'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
	{'/', {0x01,0x01,0x02,0x04,0x08,0x10,0x10}},
	{':', {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}},
	{'V', {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},
	{'F', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},
	{'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
	{'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
	{'M', {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}},
	{'z', {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},
	{'d', {0x01,0x01,0x01,0x0F,0x11,0x11,0x0F}},
	{'m', {0x00,0x15,0x15,0x15,0x15,0x15,0x15}},
	{'s', {0x0E,0x10,0x10,0x0E,0x01,0x01,0x0E}},
	{'u', {0x11,0x11,0x11,0x11,0x11,0x0F,0x01}},
	{'e', {0x00,0x0E,0x11,0x1F,0x11,0x0E,0x00}},
	{'a', {0x00,0x0E,0x01,0x0F,0x11,0x11,0x0F}},
	{'r', {0x00,0x1E,0x11,0x10,0x10,0x10,0x10}},
	{'i', {0x0C,0x00,0x04,0x04,0x04,0x04,0x0E}},
	{'n', {0x00,0x1E,0x11,0x11,0x11,0x11,0x11}},
	{'g', {0x00,0x0F,0x11,0x11,0x0F,0x01,0x1E}},
	{'l', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
	{'o', {0x00,0x0E,0x11,0x11,0x11,0x11,0x0E}},
	{'p', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
	{'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
	{'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
	{'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
	{'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
};
#define F5x7_N ((uint8_t)(sizeof(F5x7)/sizeof(F5x7[0])))

static const uint8_t *font5x7_of(char c)
{
	uint8_t i;
	for (i = 0; i < F5x7_N; i++) if (F5x7[i].ch == c) return F5x7[i].rows;
	return F5x7[0].rows;                       /* 未收录字符退回空格：保留占位宽度，不绘制不确定的字形 */
}

/* 在像素坐标 (x,y) 处画 5×7 文本，字宽 5 + 间隔 1 = 6px；返回下一个可写的 x。 */
uint8_t OLED_Text5x7(uint8_t x, uint8_t y, char *s)
{
	uint8_t r, c;
	while (*s) {
		const uint8_t *g = font5x7_of(*s++);
		for (r = 0; r < 7; r++)
			for (c = 0; c < 5; c++)
				if (g[r] & (1u << (4 - c))) OLED_SetPixel((uint8_t)(x + c), (uint8_t)(y + r), 1);
		x = (uint8_t)(x + 6);
	}
	return x;
}

/* 5×7 数字：len=0 时按实际位数输出，否则右对齐补足到 len 位；返回文本结束后的下一个可写 x */
uint8_t OLED_Num5x7(uint8_t x, uint8_t y, uint32_t v, uint8_t len)
{
	char buf[11];
	uint8_t i = 0, n = 1, d = len;
	uint32_t t = v;
	while (t >= 10u) { t /= 10u; n++; }
	if (d == 0 || d < n) d = n;
	buf[d] = 0;
	for (i = d; i > 0; i--) { buf[i - 1] = (char)('0' + (v % 10u)); v /= 10u; }
	return OLED_Text5x7(x, y, buf);
}

/* 0V 参考行超出绘制区（直流偏置大到框内无法容纳）时，在没有迹线的一侧角落标注直流电平：
   绘制前迹线已被整体平移到刚好不被裁剪，因此 0V 缺失的一侧必定没有波形像素，文本不会与迹线重叠。 */
static void show_dc(uint8_t x, uint8_t y, int32_t dc_mv)
{
	uint32_t a = (uint32_t)((dc_mv < 0) ? -dc_mv : dc_mv);
	x = OLED_Text5x7(x, y, "DC=");
	x = OLED_Text5x7(x, y, (dc_mv < 0) ? "-" : "+");
	x = OLED_Num5x7(x, y, a / 1000u, 1);
	x = OLED_Text5x7(x, y, ".");
	x = OLED_Num5x7(x, y, (a % 1000u) / 10u, 2);
	OLED_Text5x7(x, y, "V");
}

/* Wave 页整页布局：两行 5×7 小字（y=0..15）与波形框、网格、迹线（y=16..63）。
   布局放在显示层是为了只保留一份坐标定义：同一份代码既可编入固件，也便于在 PC 端编译并渲染整屏预览，避免预览与固件布局不一致。
   数据侧（码值→电压→行号、V/div 档位）由 Capture_WaveRows 计算后传入，本函数只负责绘制。 */
void OLED_WavePage(uint32_t vpp_mv, uint32_t freq_hz, int32_t dc_mv, uint8_t have_num, uint8_t have_wave,
                   const uint8_t *ylo, const uint8_t *yhi, uint16_t n,
                   uint16_t vdiv_mv, float tdiv_ms, int16_t y0v,
                   uint8_t box_y, uint8_t box_h)
{
	uint8_t x;

	if (have_num) {
		x = OLED_Text5x7(0,  0, "Vpp=");
		x = OLED_Num5x7(x,   0, vpp_mv, 4);
		x = OLED_Text5x7(x,  0, "mV");
		x = OLED_Text5x7(66, 0, "F=");
		x = OLED_Num5x7(x,   0, freq_hz, 5);
		OLED_Text5x7(x, 0, "Hz");
	} else {
		OLED_Text5x7(37, 0, "Measuring");            /* 幅值与频率尚未稳定：在该行居中显示提示文本，代替分隔符占位 */
	}

	if (have_wave) {
		x = OLED_Text5x7(0, 8, "V/d=");
		if (vdiv_mv >= 1000u) { x = OLED_Num5x7(x, 8, vdiv_mv / 1000u, 0); x = OLED_Text5x7(x, 8, "V"); }
		else                  { x = OLED_Num5x7(x, 8, vdiv_mv, 0);        x = OLED_Text5x7(x, 8, "mV"); }
		x = OLED_Text5x7(x, 8, "  T/d=");
		if (tdiv_ms >= 1.0f) {
			uint32_t v = (uint32_t)(tdiv_ms * 10.0f + 0.5f);              /* 换算为十分之一毫秒的整数并四舍五入，供后续拆分整数位与小数位 */
			x = OLED_Num5x7(x, 8, v / 10u, 0);
			if (v % 10u) { x = OLED_Text5x7(x, 8, "."); x = OLED_Num5x7(x, 8, v % 10u, 1); }
			OLED_Text5x7(x, 8, "ms");
		} else {
			x = OLED_Num5x7(x, 8, (uint32_t)(tdiv_ms * 1000.0f + 0.5f), 0);
			OLED_Text5x7(x, 8, "us");
		}
	}

	OLED_DrawFrame(0, box_y, 128, box_h, 1);
	if (have_wave) {
		OLED_DrawGrid(0, box_y, 128, box_h, 8, 8, y0v);
		OLED_DrawWaveScaled(ylo, yhi, n, 1, 126);
		/* 0V 参考行超出绘制区时，该侧没有迹线像素，直流电平标注在此处 */
		if (y0v < (int16_t)(box_y + 1))              show_dc(8, (uint8_t)(box_y + 1), dc_mv);
		else if (y0v > (int16_t)(box_y + box_h - 2)) show_dc(8, (uint8_t)(box_y + box_h - 8), dc_mv);
	} else {
		OLED_DrawGrid(0, box_y, 128, box_h, 8, 8, y0v);      /* 无信号时仍绘制坐标系与 0V 参考线 */
		OLED_Text5x7(37, (uint8_t)(box_y + 4), "no signal"); /* 文本置于绘制区上部，避免与 0V 参考线重叠 */
	}
}

/*==================== 初始化 ====================*/

/**
  * @brief  OLED初始化
  */
void OLED_Init(void)
{
	uint32_t i, j;

	for (i = 0; i < 1000; i++)			//上电延时
		for (j = 0; j < 1000; j++);

	OLED_I2C_Init();					//端口初始化

	OLED_WriteCommand(0xAE);	//关闭显示

	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);

	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);

	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);

	OLED_WriteCommand(0x40);	//设置显示开始行

	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置

	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示

	OLED_Clear();				//清空帧缓冲
	OLED_Flush();				//将帧缓冲写入显示屏
}
