#include "stm32f10x.h"

// =========================================================
// 1. FSMC 显存映射地址定义
//    精英板使用 FSMC Bank1 区域4 (NE4 -> LCD_CS 接 FSMC_NE4 / PG12)
//    RS (寄存器/数据选择) 接 FSMC_A10 (PG0)
// =========================================================
typedef struct {
    volatile unsigned short LCD_REG;
    volatile unsigned short LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE        ((unsigned int)(0x6C000000 | 0x000007FE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

// 常用 16位 565 RGB 颜色定义
#define WHITE           0xFFFF
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x7FFF
#define YELLOW          0xFFE0

volatile unsigned short lcd_id = 0;


// =========================================================
// 2. 延时函数
// =========================================================
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

void Delay_us(unsigned int us) {
    unsigned int i;
    for (i = 0; i < us * 8; i++);
}


// =========================================================
// 3. LCD 底层写指令与写数据
// =========================================================
void LCD_WR_REG(unsigned short regval) {
    LCD->LCD_REG = regval; // 写入寄存器地址
}

void LCD_WR_DATA(unsigned short data) {
    LCD->LCD_RAM = data;   // 写入数据
}

void LCD_WriteReg(unsigned short LCD_Reg, unsigned short LCD_RegValue) {
    LCD->LCD_REG = LCD_Reg;
    LCD->LCD_RAM = LCD_RegValue;
}


// =========================================================
// 4. FSMC 与 GPIO 初始化
// =========================================================
void LCD_FSMC_Init(void) {
    // 开启 GPIOB(背光), GPIOD, GPIOE, GPIOG(FSMC) 与 FSMC 时钟
    RCC->APB2ENR |= (1 << 3) | (1 << 5) | (1 << 6) | (1 << 8);
    RCC->AHBENR  |= (1 << 8); // 使能 FSMC 时钟

    // 1. PB0(LCD 背光): 推挽输出，拉高点亮背光
    GPIOB->CRL &= 0xFFFFFFF0;
    GPIOB->CRL |= 0x00000003;
    GPIOB->BSRR = (1 << 0); // PB0 = 1 点亮背光

    // 2. PD0,1,4,5,8,9,10,14,15 复用推挽输出 50MHz (FSMC D2,D3,NOE,NWE,D13,D14,D15,D0,D1)
    GPIOD->CRL &= 0xFF00FF00;
    GPIOD->CRL |= 0x00BB00BB;
    GPIOD->CRH &= 0x00FFF000;
    GPIOD->CRH |= 0xBB000BBB;

    // 3. PE7~15 复用推挽 (FSMC D4~D12)
    GPIOE->CRH &= 0x00000000;
    GPIOE->CRH |= 0xBBBBBBBB;
    GPIOE->CRL &= 0x0FFFFFFF;
    GPIOE->CRL |= 0xB0000000;

    // 4. PG0(RS->A10), PG12(CS->NE4) 复用推挽
    GPIOG->CRL &= 0xFFFFFFF0;
    GPIOG->CRL |= 0x0000000B;
    GPIOG->CRH &= 0xFFF0FFFF;
    GPIOG->CRH |= 0x000B0000;

    // 5. 配置 FSMC Bank1_NORSRAM4
    FSMC_Bank1->BTCR[6] = 0x00000000; // BCR4 寄存器清零
    FSMC_Bank1->BTCR[7] = 0x00000000; // BTR4 寄存器清零

    // 存储器类型 SRAM，数据宽度 16bit，写使能
    FSMC_Bank1->BTCR[6] |= (1 << 12) | (1 << 4) | (1 << 0); // WREN=1, MWID=16bit, MBKEN=1

    // 地址建立 2 HCLK，数据保存 5 HCLK
    FSMC_Bank1->BTCR[7] |= (2 << 0) | (5 << 8);
}


// =========================================================
// 5. 屏幕初始化序列与窗口设置
// =========================================================
void LCD_SetCursor(unsigned short Xpos, unsigned short Ypos) {
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(Xpos >> 8);
    LCD_WR_DATA(Xpos & 0xFF);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(Ypos >> 8);
    LCD_WR_DATA(Ypos & 0xFF);
}

void LCD_Init(void) {
    LCD_FSMC_Init();
    Delay_ms(50);

    // 软复位与上电初始化序列 (标准 ILI9341/ST7789 通用寄存器配置)
    LCD_WR_REG(0x11); // 退出睡眠模式
    Delay_ms(120);

    // 显存访问控制 (横屏/竖屏方向)
    LCD_WR_REG(0x36);
    LCD_WR_DATA(0x08); // 竖屏模式

    // 像素格式：16-bit 565
    LCD_WR_REG(0x3A);
    LCD_WR_DATA(0x55);

    // 开启显示
    LCD_WR_REG(0x29);
    Delay_ms(50);
}

// 全屏刷指定纯色
void LCD_Clear(unsigned short color) {
    unsigned int index = 0;
    unsigned int totalpoint = 240 * 320; // 标准 2.8 寸屏分辨率 240x320

    LCD_WR_REG(0x2A);
    LCD_WR_DATA(0x00); LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00); LCD_WR_DATA(239);

    LCD_WR_REG(0x2B);
    LCD_WR_DATA(0x00); LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x01); LCD_WR_DATA(319 - 256);

    LCD_WR_REG(0x2C); // 准备写入显存
    for (index = 0; index < totalpoint; index++) {
        LCD->LCD_RAM = color;
    }
}

// 在屏幕指定矩形区域填充颜色
void LCD_Fill(unsigned short sx, unsigned short sy, unsigned short ex, unsigned short ey, unsigned short color) {
    unsigned short i, j;
    for (i = sy; i <= ey; i++) {
        LCD_SetCursor(sx, i);
        LCD_WR_REG(0x2C);
        for (j = sx; j <= ex; j++) {
            LCD->LCD_RAM = color;
        }
    }
}


// =========================================================
// 6. 主函数：亮屏与彩色界面展示
// =========================================================
int main(void) {
    Delay_ms(300);
    LCD_Init(); // 初始化 FSMC 并点亮 LCD

    while (1) {
        // 1. 刷全屏黑色背景
        LCD_Clear(BLACK);
        Delay_ms(800);

        // 2. 刷全屏蓝色背景
        LCD_Clear(BLUE);
        Delay_ms(800);

        // 3. 在中间画几个彩色几何仪表块
        LCD_Clear(BLACK);
        LCD_Fill(20,  30, 220,  70, RED);    // 红色标题栏
        LCD_Fill(20,  90, 220, 160, GREEN);  // 绿色状态框
        LCD_Fill(20, 180, 220, 280, YELLOW); // 黄色雷达区

        Delay_ms(2000);
    }
}
