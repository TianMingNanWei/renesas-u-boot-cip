#include "rzg2ul_video.h"
#include <common.h>
#include <command.h>

// Function declarations
static void rzg2ul_cpg_init(void);
// static void rzg2ul_fpvcg_init(void);
static void rzg2ul_du_init(void);
static void rzg2ul_vcpd_init(void);
static void rzg2ul_lcdc_start(void);

// Video initialization function
int rzg2ul_video_init(void) {
    printf("Build at: 20230424 1916\r\n");

    rzg2ul_cpg_init();
    run_command("rzg2ul_video_load_img", 0);
    rzg2ul_du_init();
    rzg2ul_vcpd_init();
    rzg2ul_lcdc_start();

    return 0;
}

// Command functions
int do_rzg2ul_video_init(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[]) {
    return rzg2ul_video_init();
}

int do_rzg2ul_video_load_img(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[]) {
    dcache_disable();
    char buf[64] = { 0 };
    snprintf(buf, sizeof(buf), "fatload mmc 1:1 0x%x boot_img.bin", LCD_SRCM_ADDR); // 使用snprintf防止溢出
    return run_command(buf, 0);
}

// U-Boot command definitions
U_BOOT_CMD(
    rzg2ul_video_init, 1, 0, do_rzg2ul_video_init,
    "Init Rzg2ul",
    ""
);

U_BOOT_CMD(
    rzg2ul_video_load_img, 1, 0, do_rzg2ul_video_load_img,
    "Load Rzg2ul",
    ""
);

// CPG register values
static const uint32_t cpg_register_values[][2] = {
    {0x11010204, 0x10000000 | (CPG_LPCLK_DIV << 12)}, // CPG_PL2_DDIV
    {0x11010420, 0x01010000 | (CPG_DSI_DIV_A << 0) | (CPG_DSI_DIV_B << 8)}, // CPG_PL5_SDIV
    {0x11010144, 0x01110000 | (CPG_PL5_POSTDIV1 << 0) | (CPG_PL5_POSTDIV2 << 4) | (CPG_PL5_REFDIV << 8)}, // CPG_SIPLL5_CLK1
    {0x1101014c, (CPG_PL5_DIVVAL << 0) | (CPG_PL5_FRACIN << 8)}, // CPG_SIPLL5_CLK3
    {0x11010150, 0x000000ff | (CPG_PL5_INTIN << 16)}, // CPG_SIPLL5_CLK4
    {0x11010154, (CPG_PL5_SPREAD << 0)}, // CPG_SIPLL5_CLK5
    {0x11010140, 0x00150011}, // CPG_SIPLL5_STBY
    {0x1101056c, 0x00030003}, // CPG_CLKON_LCDC
    {0x11010580, 0x000f000f}, // CPG_CLKON_I2C
};

static const uint32_t cpg_register_values1[][2] = {
    {0x11010868, 0x00060006}, // CPG_RST_MIPI_DSI
    {0x1101086c, 0x00010001}, // CPG_RST_LCDC
    {0x11010880, 0x000f000f}, // CPG_RST_I2C
    {0x11010be8, 0x00010001}, // CPG_OTHERFUNC1_REG
};

// static const uint32_t fcpvd_register_values[][2] = {//0x10880000
// 	{0x10880000,0x00000109},
// 	{0x10880004,0x00000000},
// 	{0x10880010,0x00000000},
// 	{0x10880018,0x00000000},
// };

static const uint32_t du_register_values[][2] = {//0x10890000 //use linux value
//	{0x10890000,0x00010100},//DU_MCR0
//	{0x10890004,0x00010100},//DU_MSR0
//	{0x10890008,0x00000000},//DU_MSR1
	{0x1089000c,0x00000000},//DU_IMR0
	{0x10890010,(1 << 8 ) | (1 << 9) | (LCD_VSPOL << 16) | (LCD_HSPOL << 17)},//DU_DITR0
	{0x10890014,DU_DITR1_VSA(LCD_VSYNC) | DU_DITR1_VACTIVE(LCD_VACTIVE)},//DU_DITR1
	{0x10890018,DU_DITR2_VBP(LCD_VBACK) | DU_DITR2_VFP(LCD_VFRONT)},//DU_DITR2
	{0x1089001c,DU_DITR3_HSA(LCD_HSYNC) | DU_DITR3_HACTIVE(LCD_HACTIVE)},//DU_DITR3
	{0x10890020,DU_DITR4_HBP(LCD_HBACK) | DU_DITR4_HFP(LCD_HFRONT)},//DU_DITR4
	{0x10890024,0x00000000},//DU_DITR5
//	{0x10890040,0x00010000},//DU_MCR1 //patch for underflow
	{0x1089004c,0x0000001F},//DU_PBCR0
	{0x10890050,0x00000001},//DU_PBCR1 
	{0x10890054,0x00ff00ff},//DU_PBCR2
};

static const uint32_t vcpd_register_values[][2] = {//0x10870000  //still picture output
	{0x10870000, 0x00000000}, // VI6_CMD0
//	{0x10870000, 0x10010F1F}, // VI6_CLK_CTRL0
//	{0x10870000, 0xFF10FFFF}, // VI6_CLK_CTRL1
	{0x10870018, 0x00000808}, // VI6_CLK_DCSWT
	{0x10870028, 0x00000001}, // VI6_SRESET
//	{0x10870038, 0x00000000}, // VI6_STATUS: cmp32 = 0x00000000
	{0x10870048, 0x00000000}, // VI6_WPF0_IRQ_ENB
	// {0x10870048, 0x00011003}, // VI6_WPF0_IRQ_ENB
//	{0x1087004C, 0x00000000}, // VI6_WPF0_IRQ_STA: cmp32 = 0x00000000
	{0x10870100, 0x00000007}, // VI6_DL_CTRL
	{0x10870114, 0x00000000}, // VI6_DL_SWAP0
	{0x10870120, 0x00000002}, // VI6_DL_BODY_SIZE0
//	{0x10870130, 0x00000000}, 				// VI6_DL_HDR_REF_ADDR0: cmp32 = 0x00000000
	{0x10870158, 0x00000000}, // VI6_DL_WUPCNT0
	{0x10870300, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)}, // VI6_RPF_SRC_BSIZE: BVSIZE, BHSIZE
	{0x10870304, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)}, // VI6_RPF_SRC_DSIZE: EVSIZE, EHSIZE
	{0x10870308, (LCD_RDFMT << 0) | (LCD_RDCSC << 8) | (LCD_VIR<<28)}, // VI6_RPF_INFMT: RDFMT, CSC
//	{0x10870308, 0x00000018}, // VI6_RPF_INFMT: RDFMT, CSC
	{0x10870310, 0x00000000}, // VI6_RPF_LOC
	{0x1087030C, 0x00000F0F}, // VI6_RPF_DSWAP
	{0x10870318, 0xFF0000FF}, // VI6_RPFn_VRTCOL_SET
	{0x10870334, ((LCD_HACTIVE * 1) << 0) | ((LCD_HACTIVE * 3) << 16)}, // VI6_RPF_SRCM_STRIDE: PICT_STRD_C, PICT_STRD_Y
	{0x1087033C, LCD_SRCM_ADDR}, // VI6_RPF_SRCM_ADDR_Y: SRCM_ADDR_Y=DDR Address
	{0x10871000, 0x00000002}, // VI6_WPF0_SRCRPF: RPF0 is master
//	{0x1087100C, 0xFF000018},// VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
	{0x1087100C, (LCD_WRFMT << 0) | (LCD_WRCSC << 8) | (LCD_ODE << 22) | (0xFF << 24)},// VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
	{0x10871010, 0x0000000F}, // VI6_WPF_DSWAP
	{0x10872000, (56 << 0) }, // VI6_DPR_RPF0_ROUTE: RT_RPF0=WPF0
	{0x10872004, (0x3F << 0)},// VI6_DPR_RPF1_ROUTE: RT_RPF1=UNUSED
	{0x10872014, 0x00000500}, // VI6_DPR_WPF0_FPORCH
	{0x10872050, (0x3F << 0) | (0 << 8) | (1 << 28)}, // VI6_DPR_ILV_BRS_ROUTE: RT=UNUSED, FP=0, IIFSEL=BRS
	{0x10873B00, (1 << 0) | (1 << 1) | (LCD_CFMT << 4) | (1500 << 16)}, // VI6_LIF0_CTRL: LIF_EN=1, REQSEL=1, CFMT, OBTH=1500
	{0x10873B04, 0x00000000 | (1500 << 16)}, // VI6_LIF0_CSBTH
	{0x10873B0C, (1536 << 16) | (1 << 31)}, // VI6_LIF0_LBA: LBA1=1536, LBA0=1
	{0x10873900, 0x00000000}, // VI6_BRS_INCTRL
	{0x10873910, (0x0 << 0) | (0x0 << 4) | (0x1 << 16) | (0x0 << 20) | (1 << 31)},  // VI6_BRSA_CTRL
	{0x10873914, (0x00 << 0) | (0x00 << 8) | (0x4 << 16) | (0x4 << 20) | (0 << 23) | (0x3 << 24) | (0x2 << 28) | (0 << 31)}, // VI6_BRSA_BLD
	{0x10873918, (0x0 << 0) | (0x0 << 4) | (0 << 31)}, // VI6_BRSB_CTRL
	{0x10870104, 0x00000000}, // VI6_DL_HDR_ADDR0
};

// Register setting function
static void rzg2ul_registers_set(const uint32_t (*arr)[2], uint32_t len) {
    for (int i = 0; i < len; i++) {
        if (arr[i][0] == 0) {
            udelay(arr[i][1]);
        } else {
            writel(arr[i][1], ((uint64_t)(arr[i][0])));
        }
    }
}

// CPG initialization
static void rzg2ul_cpg_init(void) {
    rzg2ul_registers_set(cpg_register_values, ARRAY_SIZE(cpg_register_values));
    // Wait clock on
    while ((reg_read(CPG_base_addr + 0x06EC) & 0x00000003) != 0x00000003); // CPG_CLKMON_LCDC

    rzg2ul_registers_set(cpg_register_values1, ARRAY_SIZE(cpg_register_values1));
    // Wait reset on
    while ((reg_read(CPG_base_addr + 0x09EC) & 0x00000001) != 0x00000000); // CPG_RSTMON_LCDC
}

// // FCPVD initialization
// static void rzg2ul_fpvcg_init(void) {
//     printf("FCPVD Init\r\n");
//     rzg2ul_registers_set(fcpvd_register_values, ARRAY_SIZE(fcpvd_register_values));
// }

// DU initialization
static void rzg2ul_du_init(void) {
    printf("DU Init\r\n");
    rzg2ul_registers_set(du_register_values, ARRAY_SIZE(du_register_values));
}

// VPCD initialization
static void rzg2ul_vcpd_init(void) {
    printf("VCPD Init\r\n");
    rzg2ul_registers_set(vcpd_register_values, ARRAY_SIZE(vcpd_register_values));
}

// Start LCDC
static void rzg2ul_lcdc_start(void) {
    // Start Video Output
    reg_write(VSPD_base_addr + 0x0000, 0x00000001); // VI6_CMD0: STRCMD=1
    while ((reg_read(VSPD_base_addr + 0x007C) & 0x00000100) != 0x00000100); // Wait until VI6_DISP0_IRQ_STA.DST=1

    reg_write(DU_base_addr + 0x0000, 0x00000100); // DU_MCR0 : DI_EN=1
    // while ((reg_read(DSI_LINK_base_addr + ISR) & 0x00000100) != 0x00000100); // ISR.VIN1
}
