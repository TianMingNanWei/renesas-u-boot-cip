/*
 * RZ/G2UL 272P Display Parameters (Fire Version)
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * 该文件定义了272P显示所需的时序参数（Fire版本）
 */
#ifndef __POI_DEFINE_272P_FIRE__
#define __POI_DEFINE_272P_FIRE__

// PLL2.FOUT3 = 533.333 MHz, lpcalk = 533.333 / 2 / 16 = 16.666 MHz
#define CPG_LPCLK_DIV       0 // CPG_PL2_DDIV

// 为480x272@60Hz配置PLL5
// 目标像素时钟为9MHz (根据数据表)
#define CPG_PL5_REFDIV      2
#define CPG_PL5_INTIN       75
#define CPG_PL5_FRACIN      8808038
#define CPG_PL5_POSTDIV1    8
#define CPG_PL5_POSTDIV2    1
#define CPG_PL5_DIVVAL      0
#define CPG_PL5_SPREAD      0x16

// vclk1 = 113.3 MHz / 12 = 9.44 MHz
#define CPG_DSI_DIV_A       2
#define CPG_DSI_DIV_B       1

// DU timing parameters for 480x272@60Hz (根据数据表调整)
#define LCD_HACTIVE         480  // 水平有效像素
#define LCD_VACTIVE         272  // 垂直有效行数
#define LCD_HFRONT          5    // 水平前肩 (Thfp)
#define LCD_HSYNC           40   // 水平同步 (Thbp)
#define LCD_HBACK           40   // 水平后肩 (Thbp)
#define LCD_VFRONT          8    // 垂直前肩 (Tvfp)
#define LCD_VSYNC           8    // 垂直同步 (Tvb)
#define LCD_VBACK           8    // 垂直后肩 (Tvb)
#define LCD_VSPOL           0    // 垂直同步极性
#define LCD_HSPOL           0    // 水平同步极性
#define LCD_DEMD            0x3  // 数据使能模式

// VSPD parameters (保持不变)
#define LCD_BPP             24   // 像素位深
#define LCD_VIR             0    // 虚拟输入使能
#define LCD_RDFMT           0x18 // 读格式
#define LCD_RDCSC           0    // 色彩空间转换使能
#define LCD_SRCM_ADDR       0x58000000  // 源内存地址
#define LCD_WRFMT           0x18 // 写格式
#define LCD_WRCSC           0    // 写色彩空间转换
#define LCD_ODE             0    // 输出数据使能
#define LCD_CFMT            0    // 色彩格式

#endif // !__POI_DEFINE_272P_FIRE__ 