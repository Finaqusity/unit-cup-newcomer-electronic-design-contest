#include "stm32f10x.h"
#include "OLED_Font.h"
#include "OLED_Font_CN.h"

/*==================== 屏幕接口选择 ====================*/
/* OLED_USE_ONBOARD = 0：外接显示屏模块，SCL=PB10、SDA=PB11
   OLED_USE_ONBOARD = 1：板载显示屏，SCL=PB8、SDA=PB9
   更换显示硬件后修改该宏并重新编译。 */
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

/*引脚写*/
#define OLED_W_SCL(x)    GPIO_WriteBit(OLED_GPIO, OLED_SCL_PIN, (BitAction)(x))
#define OLED_W_SDA(x)    GPIO_WriteBit(OLED_GPIO, OLED_SDA_PIN, (BitAction)(x))

/*==================== 帧缓冲 (128x64 = 1024字节, page-major) ====================*/
/* OLED_FB[page][col], page=0~7, col=0~127; 每字节 = 该列8个竖直像素, bit0=最上行.
   直接对应 SSD1306 显存布局, 刷屏时无需转换. */
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
  * @brief  OLED连续写数据 (一次事务连写多字节, 刷屏提速关键)
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
void OLED_Clear(void)
{
	uint8_t p; uint16_t x;
	for (p = 0; p < 8; p++)
		for (x = 0; x < 128; x++)
			OLED_FB[p][x] = 0x00;
}

/**
  * @brief  把帧缓冲一次性刷到屏幕
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
  * @brief  画一个像素 (写入帧缓冲)
  * @param  X 0~127, Y 0~63, Color 1亮/0灭
  */
void OLED_SetPixel(uint8_t X, uint8_t Y, uint8_t Color)
{
	if (X >= 128 || Y >= 64) return;              //越界保护
	if (Color) OLED_FB[Y >> 3][X] |=  (1 << (Y & 7));
	else       OLED_FB[Y >> 3][X] &= ~(1 << (Y & 7));
}

/**
  * @brief  读一个像素
  */
uint8_t OLED_GetPixel(uint8_t X, uint8_t Y)
{
	if (X >= 128 || Y >= 64) return 0;
	return (OLED_FB[Y >> 3][X] >> (Y & 7)) & 1;
}

/**
  * @brief  画直线 (Bresenham)
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
  * @brief  画空心矩形
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
  * @brief  画实心矩形
  */
void OLED_FillRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, uint8_t Color)
{
	uint8_t i, j;
	for (j = 0; j < H; j++)
		for (i = 0; i < W; i++)
			OLED_SetPixel(X + i, Y + j, Color);
}

/*==================== 文本 (写帧缓冲, page对齐, 不透明) ====================*/

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
  * @brief  次方函数
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

/*==================== 汉字 / 混排 (逐像素, Y 任意) ====================*/

/* 把一个 8x16 ASCII 字模画到像素坐标 (X,Y) */
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

/* 把一个 16x16 汉字字模画到像素坐标 (X,Y) */
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
			if (!found) OLED_DrawFrame(X, Y, 16, 16, 1);  //字库里没有: 空框占位
			X += 16;
		}
	}
}

/*==================== 波形 ====================*/

/**
  * @brief  画一段波形 (自动纵向缩放, 折线连接)
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
	if (vmax - vmin < 4) vmax = vmin + 4;         //防除零/平线

	for (i = 0; i < n; i++)
	{
		px = x0 + (int16_t)((uint32_t)i * (w - 1) / (n - 1 ? n - 1 : 1));
		py = y0 + h - 1 - (int16_t)((uint32_t)(buf[i] - vmin) * (h - 1) / (vmax - vmin));
		if (i) OLED_DrawLine(lx, ly, px, py, 1);
		else   OLED_SetPixel((uint8_t)px, (uint8_t)py, 1);
		lx = px; ly = py;
	}
}

/* 抽样包络绘制：每个显示列画一条 lo[i]~hi[i] 之间的竖线。
   当信号周期仅占数个显示列时，逐点连线会漏抽峰谷点，表现为幅度偏小、方波边沿倾斜；
   采用 min/max 包络可保证峰与谷都呈现在屏上：平顶为实心带，边沿为陡立的竖线。 */
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
	if (vmax - vmin < 4) vmax = vmin + 4;         //防除零/平线

	for (i = 0; i < n; i++)
	{
		px = x0 + (int16_t)((uint32_t)i * (w - 1) / (n - 1 ? n - 1 : 1));
		ya = y0 + h - 1 - (int16_t)((uint32_t)(hi[i] - vmin) * (h - 1) / (vmax - vmin));
		yb = y0 + h - 1 - (int16_t)((uint32_t)(lo[i] - vmin) * (h - 1) / (vmax - vmin));
		if (ya == yb) OLED_SetPixel((uint8_t)px, (uint8_t)ya, 1);
		else          OLED_DrawLine(px, ya, px, yb, 1);
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

	OLED_Clear();				//清帧缓冲
	OLED_Flush();				//上屏
}
