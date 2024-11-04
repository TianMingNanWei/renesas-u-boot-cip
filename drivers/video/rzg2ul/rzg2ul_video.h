/*
 * RZ/G2UL Video Driver Header
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * 该头文件定义了RZ/G2UL视频驱动所需的各种宏和数据结构
 */
#ifndef _RZG2UL_VIDEO_H_
#define _RZG2UL_VIDEO_H_

/* U-boot Header*/
#include <dm/uclass.h>
#include <dm/device_compat.h>
#include <cpu_func.h>
#include <common.h>
#include <asm/io.h>
#include <i2c.h>
#include <linux/delay.h>
#include <string.h>

/* Register Define */
#include "rzg2ul_def.h"

/* Screen Header */
// #include "poi_define_480P.h"
#include "poi_define_272P.h"

/* Du Define */

#define DU_DITR1		0x14
#define DU_DITR1_VSA(x)		((x) << 0)
#define DU_DITR1_VACTIVE(x)	((x) << 16)

#define DU_DITR2		0x18
#define DU_DITR2_VBP(x)		((x) << 0)
#define DU_DITR2_VFP(x)		((x) << 16)

#define DU_DITR3		0x1C
#define DU_DITR3_HSA(x)		((x) << 0)
#define DU_DITR3_HACTIVE(x)	((x) << 16)

#define DU_DITR4		0x20
#define DU_DITR4_HBP(x)		((x) << 0)
#define DU_DITR4_HFP(x)		((x) << 16)

#define DU_DITR5		0x24
#define DU_DITR5_VSFT(x)	((x) << 0)
#define DU_DITR5_HSFT(x)	((x) << 16)

#define DU_PBCR0		0x4C
#define DU_PBCR0_PB_DEP(x)	((x) << 0)

/* Write Read Register */
#define reg_write(addr,val)     writel((val),((uint64_t)addr))
#define reg_read(addr)          readl(addr)

int rzg2ul_video_init(void);

#endif /*_RZG2UL_VIDEO_H_*/