/*
 * RZ/G2UL 272P Display Parameters
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * 该文件定义了272P显示所需的时序参数
 */
#ifndef __POI_DEFINE_272P__
#define __POI_DEFINE_272P__

// PLL2.FOUT3 = 533.333 MHz, lpcalk = 533.333 / 2 / 16 = 16.666 MHz
#define CPG_LPCLK_DIV       0 // CPG_PL2_DDIV

// 为480x272@60Hz配置PLL5
// 目标像素时钟约为9MHz (480+2+41+2) * (272+2+10+2) * 60 = 9.0MHz
// PLL5.FOUTPOSTDIV = 24 MHz / 2 * (75 + (8808038 / (2^24))) / 8 / 1 = 113.3 MHz
#define CPG_PL5_REFDIV      2 // CLK1
#define CPG_PL5_INTIN       75 // CPG_SIPLL5_CLK4  
#define CPG_PL5_FRACIN      8808038 // CPG_SIPLL5_CLK3
#define CPG_PL5_POSTDIV1    8 // CLK1
#define CPG_PL5_POSTDIV2    1 // CLK1
#define CPG_PL5_DIVVAL      0 // CPG_SIPLL5_CLK3
#define CPG_PL5_SPREAD      0x16 // CPG_SIPLL5_CLK5

// vclk1 = 113.3 MHz / 12 = 9.44 MHz
#define CPG_DSI_DIV_A       2  // 4分频
#define CPG_DSI_DIV_B       1  // 3分频

// DU timing parameters for 480x272@60Hz
#define LCD_HACTIVE         480  // 水平有效像素
#define LCD_VACTIVE         272  // 垂直有效行数
#define LCD_HFRONT          2    // 水平前肩
#define LCD_HSYNC           41   // 水平同步
#define LCD_HBACK          2    // 水平后肩
#define LCD_VFRONT          2    // 垂直前肩  
#define LCD_VSYNC           10   // 垂直同步
#define LCD_VBACK           2    // 垂直后肩
#define LCD_VSPOL           1    // 垂直同步极性
#define LCD_HSPOL           1    // 水平同步极性
#define LCD_DEMD            0x3  // 数据使能模式

// VSPD parameters
#define LCD_BPP             24   // 像素位深
#define LCD_VIR             0    // 虚拟输入使能
#define LCD_RDFMT           0x14 // 读格式
#define LCD_RDCSC           0    // 色彩空间转换使能
#define LCD_SRCM_ADDR       0x58000000  // 源内存地址
#define LCD_WRFMT           0x00 // 写格式
#define LCD_WRCSC           0    // 写色彩空间转换
#define LCD_ODE             0    // 输出数据使能
#define LCD_CFMT            0    // 色彩格式

#endif // !__POI_DEFINE_272P__