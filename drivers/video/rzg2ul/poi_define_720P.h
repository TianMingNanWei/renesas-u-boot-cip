/*
 * RZ/G2UL 720P Display Parameters
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * 该文件定义了720P显示所需的时序参数
 */

#ifndef __POI_DEFINE_720P__
#define __POI_DEFINE_720P__

// PLL2.FOUT3 = 533.333 MHz, lpcalk = 533.333 / 2 / 16 = 16.666 MHz
#define CPG_LPCLK_DIV       0 // CPG_PL2_DDIV

// PLL5.FOUTPOSTDIV = 24 MHz / 2 * (75 + (8808038 / (2^24))) / 3 / 1 = 302.1 MHz, SSC = 31.25 kHz / 0.5%
// hsclk = 302.1 MHz / 16 = 18.88125 MHz
#define CPG_PL5_REFDIV      2 // CLK1
#define CPG_PL5_INTIN       74 // CPG_SIPLL5_CLK4
#define CPG_PL5_FRACIN      4194304 // CPG_SIPLL5_CLK3
#define CPG_PL5_POSTDIV1    1 // CLK1
#define CPG_PL5_POSTDIV2    1 // CLK1
#define CPG_PL5_SSC_EN      1
#define CPG_PL5_DOWNSPREAD  1
#define CPG_PL5_DIVVAL      0 // CPG_SIPLL5_CLK3
#define CPG_PL5_SPREAD      0x16 // CPG_SIPLL5_CLK5

// vclk1 = 302.1 MHz / 4 / 3 = 25.175 MHz
// vclk1 = 302.1 MHz / 2 / 2 = 75 MHz
#define CPG_DSI_DIV_A       1
#define CPG_DSI_DIV_B       2 

// DU
#define LCD_HACTIVE         1280
#define LCD_VACTIVE         720 // The number of lines in the Vactive period
#define LCD_HFRONT          110
#define LCD_HSYNC           40
#define LCD_HBACK           220
#define LCD_VFRONT          5
#define LCD_VSYNC           5 // The number of lines in the Vsync period
#define LCD_VBACK           20
#define LCD_VSPOL           1 // vsync polarity
#define LCD_HSPOL           1 // hsync polarity
#define LCD_DEMD            0x3

// VSPD
#define LCD_BPP             24
#define LCD_VIR             0 // Virtual Input Enable
#define LCD_RDFMT           0x18
#define LCD_RDCSC           0 // Color Space Conversion Enable
#define LCD_SRCM_ADDR       0x58000000
#define LCD_WRFMT           0x18
#define LCD_WRCSC           0
#define LCD_ODE             0
#define LCD_CFMT            0

#endif // !__POI_DEFINE_720P__