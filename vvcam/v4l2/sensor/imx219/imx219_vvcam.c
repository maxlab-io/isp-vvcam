// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX219 cameras.
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx258 camera driver
 * Copyright (C) 2018 Intel Corporation
 *
 * DT / fwnode changes, and regulator / GPIO control taken from imx214 driver
 * Copyright 2018 Qtechnology A/S
 *
 * Flip handling taken from the Sony IMX319 driver.
 * Copyright (C) 2018 Intel Corporation
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

#include "vvsensor.h"

#define IMX219_REG_VALUE_08BIT		1
#define IMX219_REG_VALUE_16BIT		2

#define IMX219_REG_MODE_SELECT		0x0100
#define IMX219_MODE_STANDBY		0x00
#define IMX219_MODE_STREAMING		0x01

/* Chip ID */
#define IMX219_REG_CHIP_ID		0x0000
#define IMX219_CHIP_ID			0x0219

/* External clock frequency is 24.0M */
#define IMX219_XCLK_FREQ		24000000

/* Pixel rate is fixed at 182.4M for all the modes */
#define IMX219_PIXEL_RATE		182400000

#define IMX219_DEFAULT_LINK_FREQ	456000000

/* V_TIMING internal */
#define IMX219_REG_VTS			0x0160
#define IMX219_VTS_15FPS		0x0dc6
#define IMX219_VTS_30FPS_1080P		0x06e3
#define IMX219_VTS_30FPS_BINNED		0x06e3
#define IMX219_VTS_30FPS_640x480	0x06e3
#define IMX219_VTS_MAX			0xffff

#define IMX219_VBLANK_MIN		4

/*Frame Length Line*/
#define IMX219_FLL_MIN			0x08a6
#define IMX219_FLL_MAX			0xffff
#define IMX219_FLL_STEP			1
#define IMX219_FLL_DEFAULT		0x0c98

/* HBLANK control - read only */
#define IMX219_PPL_DEFAULT		3448

/* Exposure control */
#define IMX219_REG_EXPOSURE		0x015a
#define IMX219_EXPOSURE_MIN		4
#define IMX219_EXPOSURE_STEP		1
#define IMX219_EXPOSURE_DEFAULT		0x640
#define IMX219_EXPOSURE_MAX		65535

/* Analog gain control */
#define IMX219_REG_ANALOG_GAIN		0x0157
#define IMX219_ANA_GAIN_MIN		0
#define IMX219_ANA_GAIN_MAX		232
#define IMX219_ANA_GAIN_STEP		1
#define IMX219_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX219_REG_DIGITAL_GAIN		0x0158
#define IMX219_DGTL_GAIN_MIN		0x0100
#define IMX219_DGTL_GAIN_MAX		0x0fff
#define IMX219_DGTL_GAIN_DEFAULT	0x0100
#define IMX219_DGTL_GAIN_STEP		1

#define IMX219_REG_ORIENTATION		0x0172

/* Test Pattern Control */
#define IMX219_REG_TEST_PATTERN		0x0600
#define IMX219_TEST_PATTERN_DISABLE	0
#define IMX219_TEST_PATTERN_SOLID_COLOR	1
#define IMX219_TEST_PATTERN_COLOR_BARS	2
#define IMX219_TEST_PATTERN_GREY_COLOR	3
#define IMX219_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX219_REG_TESTP_RED		0x0602
#define IMX219_REG_TESTP_GREENR		0x0604
#define IMX219_REG_TESTP_BLUE		0x0606
#define IMX219_REG_TESTP_GREENB		0x0608
#define IMX219_TESTP_COLOUR_MIN		0
#define IMX219_TESTP_COLOUR_MAX		0x03ff
#define IMX219_TESTP_COLOUR_STEP	1
#define IMX219_TESTP_RED_DEFAULT	IMX219_TESTP_COLOUR_MAX
#define IMX219_TESTP_GREENR_DEFAULT	0
#define IMX219_TESTP_BLUE_DEFAULT	0
#define IMX219_TESTP_GREENB_DEFAULT	0

/* IMX219 native and active pixel array size. */
#define IMX219_NATIVE_WIDTH		3296U
#define IMX219_NATIVE_HEIGHT		2480U
#define IMX219_PIXEL_ARRAY_LEFT		8U
#define IMX219_PIXEL_ARRAY_TOP		8U
#define IMX219_PIXEL_ARRAY_WIDTH	3280U
#define IMX219_PIXEL_ARRAY_HEIGHT	2464U

struct imx219_reg {
	u16 address;
	u8 val;
};

struct imx219_reg_list {
	unsigned int num_of_regs;
	const struct imx219_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx219_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* V-timing */
	unsigned int vts_def;

	/* Default register values */
	struct imx219_reg_list reg_list;
};

/*
 * Register sets lifted off the i2C interface from the Raspberry Pi firmware
 * driver.
 * 3280x2464 = mode 2, 1920x1080 = mode 1, 1640x1232 = mode 4, 640x480 = mode 7.
 */
static const struct imx219_reg mode_3280x2464_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x0c},
	{0x30eb, 0x05},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0c},
	{0x0167, 0xcf},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016a, 0x09},
	{0x016b, 0x9f},
	{0x016c, 0x0c},
	{0x016d, 0xd0},
	{0x016e, 0x09},
	{0x016f, 0xa0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x0c},
	{0x0625, 0xd0},
	{0x0626, 0x09},
	{0x0627, 0xa0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};

static const struct imx219_reg mode_1920_1080_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0162, 0x0d},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xa8},
	{0x0166, 0x0a},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xb4},
	{0x016a, 0x06},
	{0x016b, 0xeb},
	{0x016c, 0x07},
	{0x016d, 0x80},
	{0x016e, 0x04},
	{0x016f, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x07},
	{0x0625, 0x80},
	{0x0626, 0x04},
	{0x0627, 0x38},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
};

static const struct imx219_reg mode_1640_1232_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x0c},
	{0x30eb, 0x05},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0c},
	{0x0167, 0xcf},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016a, 0x09},
	{0x016b, 0x9f},
	{0x016c, 0x06},
	{0x016d, 0x68},
	{0x016e, 0x04},
	{0x016f, 0xd0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x01},
	{0x0175, 0x01},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x06},
	{0x0625, 0x68},
	{0x0626, 0x04},
	{0x0627, 0xd0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};

static const struct imx219_reg mode_640_480_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0162, 0x0d},
	{0x0163, 0x78},
	{0x0164, 0x03},
	{0x0165, 0xe8},
	{0x0166, 0x08},
	{0x0167, 0xe7},
	{0x0168, 0x02},
	{0x0169, 0xf0},
	{0x016a, 0x06},
	{0x016b, 0xaf},
	{0x016c, 0x02},
	{0x016d, 0x80},
	{0x016e, 0x01},
	{0x016f, 0xe0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x03},
	{0x0175, 0x03},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x06},
	{0x0625, 0x68},
	{0x0626, 0x04},
	{0x0627, 0xd0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
};

static const struct imx219_reg raw8_framefmt_regs[] = {
	{0x018c, 0x08},
	{0x018d, 0x08},
	{0x0309, 0x08},
};

static const struct imx219_reg raw10_framefmt_regs[] = {
	{0x018c, 0x0a},
	{0x018d, 0x0a},
	{0x0309, 0x0a},
};

static const s64 imx219_link_freq_menu[] = {
	IMX219_DEFAULT_LINK_FREQ,
};

static const char * const imx219_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx219_test_pattern_val[] = {
	IMX219_TEST_PATTERN_DISABLE,
	IMX219_TEST_PATTERN_COLOR_BARS,
	IMX219_TEST_PATTERN_SOLID_COLOR,
	IMX219_TEST_PATTERN_GREY_COLOR,
	IMX219_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx219_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define IMX219_NUM_SUPPLIES ARRAY_SIZE(imx219_supply_name)

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,

	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software stanby) must be not less than:
 *   t4 + max(t5, t6 + <time to initialize the sensor register over I2C>)
 * where
 *   t4 is fixed, and is max 200uS,
 *   t5 is fixed, and is 6000uS,
 *   t6 depends on the sensor external clock, and is max 32000 clock periods.
 * As per sensor datasheet, the external clock must be from 6MHz to 27MHz.
 * So for any acceptable external clock t6 is always within the range of
 * 1185 to 5333 uS, and is always less than t5.
 * For this reason this is always safe to wait (t4 + t5) = 6200 uS, then
 * initialize the sensor over I2C, and then exit the software standby.
 *
 * This start-up time can be optimized a bit more, if we start the writes
 * over I2C after (t4+t6), but before (t4+t5) expires. But then sensor
 * initialization over I2C may complete before (t4+t5) expires, and we must
 * ensure that capture is not started before (t4+t5).
 *
 * This delay doesn't account for the power supply startup time. If needed,
 * this should be taken care of via the regulator framework. E.g. in the
 * case of DT for regulator-fixed one should define the startup-delay-us
 * property.
 */
#define IMX219_XCLR_MIN_DELAY_US	6200
#define IMX219_XCLR_DELAY_RANGE_US	1000

/* Mode configs */
static const struct imx219_mode supported_modes[] = {
	{
		/* 8MPix 15fps mode */
		.width = 3280,
		.height = 2464,
		.crop = {
			.left = IMX219_PIXEL_ARRAY_LEFT,
			.top = IMX219_PIXEL_ARRAY_TOP,
			.width = 3280,
			.height = 2464
		},
		.vts_def = IMX219_VTS_15FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3280x2464_regs),
			.regs = mode_3280x2464_regs,
		},
	},
	{
		/* 1080P 30fps cropped */
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = 688,
			.top = 700,
			.width = 1920,
			.height = 1080
		},
		.vts_def = IMX219_VTS_30FPS_1080P,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_regs),
			.regs = mode_1920_1080_regs,
		},
	},
	{
		/* 2x2 binned 30fps mode */
		.width = 1640,
		.height = 1232,
		.crop = {
			.left = IMX219_PIXEL_ARRAY_LEFT,
			.top = IMX219_PIXEL_ARRAY_TOP,
			.width = 3280,
			.height = 2464
		},
		.vts_def = IMX219_VTS_30FPS_BINNED,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1640_1232_regs),
			.regs = mode_1640_1232_regs,
		},
	},
	{
		/* 640x480 30fps mode */
		.width = 640,
		.height = 480,
		.crop = {
			.left = 1008,
			.top = 760,
			.width = 1280,
			.height = 960
		},
		.vts_def = IMX219_VTS_30FPS_640x480,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640_480_regs),
			.regs = mode_640_480_regs,
		},
	},
};


static struct vvcam_mode_info_s pov2775_mode_info[] = {
	{
		.index          = 0,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines     = 0x466,
			.curr_frm_len_lines    = 0x466,
			.one_line_exp_time_ns  = 29625,

			.max_integration_line  = 0x466 - 4,
			.min_integration_line  = 1,

			.max_again             = 8 * 1024,
			.min_again             = 2 * 1024,
			.max_dgain             = 4 * 1024,
			.min_dgain             = 1.5 * 1024,
			.gain_step             = 4,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data      = ov2775_init_setting_1080p,
		.reg_data_count = ARRAY_SIZE(ov2775_init_setting_1080p),
	},
	{
		.index          = 1,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_DUAL_DCG,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern  = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines        = 0x466,
			.curr_frm_len_lines       = 0x466,
			.one_line_exp_time_ns     = 59167,

			.max_integration_line     = 0x400,
			.min_integration_line     = 16,

			.max_vsintegration_line   = 44,
			.min_vsintegration_line   = 1,

			.max_long_again = 8 * 1024 * DCG_CONVERSION_GAIN,
			.min_long_again = 1 * 1024 * DCG_CONVERSION_GAIN,
			.max_long_dgain = 3 * 1024,
			.min_long_dgain = 2 * 1024,

			.max_again      = 8 * 1024,
			.min_again      = 2 * 1024,
			.max_dgain      = 4 * 1024,
			.min_dgain      = 1.5 * 1024,

			.max_short_again = 8 * 1024,
			.min_short_again = 2 * 1024,
			.max_short_dgain = 4 * 1024,
			.min_short_dgain = 1.5 * 1024,
			.start_exposure  = 2 * 300 * 1024,

			.gain_step       = 4,
			.cur_fps         = 30 * 1024,
			.max_fps         = 30 * 1024,
			.min_fps         = 5 * 1024,
			.min_afps        = 5 * 1024,
			.hdr_ratio       = {
				.ratio_l_s = 8 * 1024,
				.ratio_s_vs = 8 * 1024,
				.accuracy = 1024,
			},
			.int_update_delay_frm = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data = ov2775_init_setting_1080p_hdr,
		.reg_data_count = ARRAY_SIZE(ov2775_init_setting_1080p_hdr),
	},
	{
		.index          = 2,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_HDR_NATIVE,
		.stitching_mode = SENSOR_STITCHING_DUAL_DCG_NOWAIT,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 1,
			.x_bit  = 16,
			.y_bit  = 12,
		},
		.bayer_pattern  = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines        = 0x466,
			.curr_frm_len_lines       = 0x466,
			.one_line_exp_time_ns     = 59167,

			.max_integration_line     = 0x466 - 4,
			.min_integration_line     = 1,

			.max_long_again = 8 * 1024 * DCG_CONVERSION_GAIN,
			.min_long_again = 1 * 1024 * DCG_CONVERSION_GAIN,
			.max_long_dgain = 3 * 1024,
			.min_long_dgain = 2 * 1024,

			.max_again      = 8 * 1024 ,
			.min_again      = 2 * 1024,
			.max_dgain      = 4 * 1024,
			.min_dgain      = 1.5 * 1024,

			.start_exposure = 3 * 400 * 1024,

			.gain_step       = 4,
			.cur_fps         = 30 * 1024,
			.max_fps         = 30 * 1024,
			.min_fps         = 5 * 1024,
			.min_afps        = 5 * 1024,
			.hdr_ratio       = {
				.ratio_l_s = 8 * 1024,
				.ratio_s_vs = 8 * 1024,
				.accuracy = 1024,
			},
			.int_update_delay_frm = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data = ov2775_1080p_native_hdr_regs,
		.reg_data_count = ARRAY_SIZE(ov2775_1080p_native_hdr_regs),
	},
	{
		.index          = 3,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 16,
			.left          = 16,
			.width         = 1280,
			.height        = 720,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines     = 0x466,
			.curr_frm_len_lines    = 0x466,
			.one_line_exp_time_ns  = 29625,

			.max_integration_line  = 0x466 - 4,
			.min_integration_line  = 1,

			.max_again             = 8 * 1024,
			.min_again             = 2 * 1024,
			.max_dgain             = 4 * 1024,
			.min_dgain             = 1.5 * 1024,
			.gain_step             = 4,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data      = ov2775_init_setting_1080p,
		.reg_data_count = ARRAY_SIZE(ov2775_init_setting_1080p),
	},
	{
		.index          = 4,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_DUAL_DCG_NOWAIT,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern  = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines        = 0x466,
			.curr_frm_len_lines       = 0x466,
			.one_line_exp_time_ns     = 59167,

			.max_integration_line     = 0x466 - 4,
			.min_integration_line     = 1,

			.max_long_again = 8 * 1024 * DCG_CONVERSION_GAIN,
			.min_long_again = 1 * 1024 * DCG_CONVERSION_GAIN,
			.max_long_dgain = 4 * 1024,
			.min_long_dgain = 2 * 1024,

			.max_again      = 8 * 1024,
			.min_again      = 2 * 1024,
			.max_dgain      = 4 * 1024,
			.min_dgain      = 1.5 * 1024,

			.start_exposure  = 3 * 400 * 1024,

			.gain_step       = 4,
			.cur_fps         = 30 * 1024,
			.max_fps         = 30 * 1024,
			.min_fps         = 5 * 1024,
			.min_afps        = 5 * 1024,
			.hdr_ratio       = {
				.ratio_l_s = 8 * 1024,
				.ratio_s_vs = 8 * 1024,
				.accuracy = 1024,
			},
			.int_update_delay_frm = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 2,
		},
		.preg_data = ov2775_init_setting_1080p_hdr_2dol,
		.reg_data_count = ARRAY_SIZE(ov2775_init_setting_1080p_hdr_2dol),
	},
};

struct imx219 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to IMX219 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX219_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx219_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct imx219 *to_imx219(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx219, sd);
}

/* Read registers up to 2 at a time */
static int imx219_read_reg(struct imx219 *imx219, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "failed to read reg %x %d %d\n", reg, len, ret);
		return -EIO;
    }

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx219_write_reg(struct imx219 *imx219, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx219_write_regs(struct imx219 *imx219,
			     const struct imx219_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx219_write_reg(imx219, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx219_get_format_code(struct imx219 *imx219, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx219->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (imx219->vflip->val ? 2 : 0) |
	    (imx219->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx219_set_default_format(struct imx219 *imx219)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &imx219->fmt;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx219_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx219 *imx219 = to_imx219(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx219->mutex);

	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = imx219_get_format_code(imx219,
					       MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = IMX219_PIXEL_ARRAY_TOP;
	try_crop->left = IMX219_PIXEL_ARRAY_LEFT;
	try_crop->width = IMX219_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX219_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx219->mutex);

	return 0;
}

static int imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *imx219 =
		container_of(ctrl->handler, struct imx219, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = imx219->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
			exposure_max : IMX219_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx219->exposure,
					 imx219->exposure->minimum,
					 exposure_max, imx219->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx219_write_reg(imx219, IMX219_REG_ANALOG_GAIN,
				       IMX219_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx219_write_reg(imx219, IMX219_REG_EXPOSURE,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx219_write_reg(imx219, IMX219_REG_DIGITAL_GAIN,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx219_write_reg(imx219, IMX219_REG_TEST_PATTERN,
				       IMX219_REG_VALUE_16BIT,
				       imx219_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx219_write_reg(imx219, IMX219_REG_ORIENTATION, 1,
				       imx219->hflip->val |
				       imx219->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx219_write_reg(imx219, IMX219_REG_VTS,
				       IMX219_REG_VALUE_16BIT,
				       imx219->mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx219_write_reg(imx219, IMX219_REG_TESTP_RED,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx219_write_reg(imx219, IMX219_REG_TESTP_GREENR,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx219_write_reg(imx219, IMX219_REG_TESTP_BLUE,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx219_write_reg(imx219, IMX219_REG_TESTP_GREENB,
				       IMX219_REG_VALUE_16BIT, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_set_ctrl,
};

static int imx219_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx219 *imx219 = to_imx219(sd);

	if (code->index >= (ARRAY_SIZE(codes) / 4))
		return -EINVAL;

	mutex_lock(&imx219->mutex);
	code->code = imx219_get_format_code(imx219, codes[code->index * 4]);
	mutex_unlock(&imx219->mutex);

	return 0;
}

static int imx219_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx219 *imx219 = to_imx219(sd);
	u32 code;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&imx219->mutex);
	code = imx219_get_format_code(imx219, fse->code);
	mutex_unlock(&imx219->mutex);
	if (fse->code != code)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx219_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx219_update_pad_format(struct imx219 *imx219,
				     const struct imx219_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx219_reset_colorspace(&fmt->format);
}

static int __imx219_get_pad_format(struct imx219 *imx219,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx219->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = imx219_get_format_code(imx219, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		imx219_update_pad_format(imx219, imx219->mode, fmt);
		fmt->format.code = imx219_get_format_code(imx219,
							  imx219->fmt.code);
	}

	return 0;
}

static int imx219_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx219 *imx219 = to_imx219(sd);
	int ret;

	mutex_lock(&imx219->mutex);
	ret = __imx219_get_pad_format(imx219, sd_state, fmt);
	mutex_unlock(&imx219->mutex);

	return ret;
}

static int imx219_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx219 *imx219 = to_imx219(sd);
	const struct imx219_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	int exposure_max, exposure_def, hblank;
	unsigned int i;

	mutex_lock(&imx219->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == fmt->format.code)
			break;
	if (i >= ARRAY_SIZE(codes))
		i = 0;

	/* Bayer order varies with flips */
	fmt->format.code = imx219_get_format_code(imx219, codes[i]);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx219_update_pad_format(imx219, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else if (imx219->mode != mode ||
		   imx219->fmt.code != fmt->format.code) {
		imx219->fmt = fmt->format;
		imx219->mode = mode;
		/* Update limits and set FPS to default */
		__v4l2_ctrl_modify_range(imx219->vblank, IMX219_VBLANK_MIN,
					 IMX219_VTS_MAX - mode->height, 1,
					 mode->vts_def - mode->height);
		__v4l2_ctrl_s_ctrl(imx219->vblank,
				   mode->vts_def - mode->height);
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->vts_def - 4;
		exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
			exposure_max : IMX219_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx219->exposure,
					 imx219->exposure->minimum,
					 exposure_max, imx219->exposure->step,
					 exposure_def);
		/*
		 * Currently PPL is fixed to IMX219_PPL_DEFAULT, so hblank
		 * depends on mode->width only, and is not changeble in any
		 * way other than changing the mode.
		 */
		hblank = IMX219_PPL_DEFAULT - mode->width;
		__v4l2_ctrl_modify_range(imx219->hblank, hblank, hblank, 1,
					 hblank);
	}

	mutex_unlock(&imx219->mutex);

	return 0;
}

static int imx219_set_framefmt(struct imx219 *imx219)
{
	switch (imx219->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return imx219_write_regs(imx219, raw8_framefmt_regs,
					ARRAY_SIZE(raw8_framefmt_regs));

	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return imx219_write_regs(imx219, raw10_framefmt_regs,
					ARRAY_SIZE(raw10_framefmt_regs));
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__imx219_get_pad_crop(struct imx219 *imx219,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx219->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx219->mode->crop;
	}

	return NULL;
}

static int imx219_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx219 *imx219 = to_imx219(sd);

		mutex_lock(&imx219->mutex);
		sel->r = *__imx219_get_pad_crop(imx219, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx219->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX219_NATIVE_WIDTH;
		sel->r.height = IMX219_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX219_PIXEL_ARRAY_TOP;
		sel->r.left = IMX219_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX219_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX219_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx219_start_streaming(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	const struct imx219_reg_list *reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	/* Apply default values of current mode */
	reg_list = &imx219->mode->reg_list;
	ret = imx219_write_regs(imx219, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = imx219_set_framefmt(imx219);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx219->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

    ret = imx219_write_reg(imx219, IMX219_REG_TEST_PATTERN,
                        IMX219_REG_VALUE_16BIT, 1);
	if (ret)
		goto err_rpm_put;

    dev_info(&client->dev, "%s started\n", __func__);

	/* set stream on register */
	ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
			       IMX219_REG_VALUE_08BIT, IMX219_MODE_STREAMING);
	if (ret)
		goto err_rpm_put;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx219->vflip, true);
	__v4l2_ctrl_grab(imx219->hflip, true);

	return 0;

err_rpm_put:
    dev_info(&client->dev, "%s failed %d\n", __func__, ret);
	pm_runtime_put(&client->dev);
	return ret;
}

static void imx219_stop_streaming(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	int ret;

	/* set stream off register */
	ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
			       IMX219_REG_VALUE_08BIT, IMX219_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	__v4l2_ctrl_grab(imx219->vflip, false);
	__v4l2_ctrl_grab(imx219->hflip, false);

	pm_runtime_put(&client->dev);
}

static int imx219_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx219 *imx219 = to_imx219(sd);
	int ret = 0;

	mutex_lock(&imx219->mutex);
	if (imx219->streaming == enable) {
		mutex_unlock(&imx219->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx219_start_streaming(imx219);
        v4l2_info(sd, "start streaming %d\n", ret);
		if (ret)
			goto err_unlock;
	} else {
		imx219_stop_streaming(imx219);
	}

	imx219->streaming = enable;

	mutex_unlock(&imx219->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx219->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx219_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);
	int ret;

	ret = regulator_bulk_enable(IMX219_NUM_SUPPLIES,
				    imx219->supplies);
	if (ret) {
		dev_err(dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx219->xclk);
	if (ret) {
		dev_err(dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx219->reset_gpio, 1);
	usleep_range(IMX219_XCLR_MIN_DELAY_US,
		     IMX219_XCLR_MIN_DELAY_US + IMX219_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX219_NUM_SUPPLIES, imx219->supplies);

	return ret;
}

static int imx219_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);

	gpiod_set_value_cansleep(imx219->reset_gpio, 0);
	regulator_bulk_disable(IMX219_NUM_SUPPLIES, imx219->supplies);
	clk_disable_unprepare(imx219->xclk);

	return 0;
}

static int __maybe_unused imx219_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);

	if (imx219->streaming)
		imx219_stop_streaming(imx219);

	return 0;
}

static int __maybe_unused imx219_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);
	int ret;

	if (imx219->streaming) {
		ret = imx219_start_streaming(imx219);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx219_stop_streaming(imx219);
	imx219->streaming = false;

	return ret;
}

static int imx219_get_regulators(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	unsigned int i;

	for (i = 0; i < IMX219_NUM_SUPPLIES; i++)
		imx219->supplies[i].supply = imx219_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX219_NUM_SUPPLIES,
				       imx219->supplies);
}

/* Verify chip ID */
static int imx219_identify_module(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	int ret;
	u32 val;

	ret = imx219_read_reg(imx219, IMX219_REG_CHIP_ID,
			      IMX219_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x %d\n",
                IMX219_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX219_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX219_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

static int imx219_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int imx219_get_clk(struct imx219 *sensor, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret = 0;
	vvcam_clk.sensor_mclk = clk_get_rate(sensor->sensor_clk);
	vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (ret != 0)
		ret = -EINVAL;
	return ret;
}

static int imx219_query_capability(struct imx219 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "imx219");
	sprintf((char *)pcap->bus_info, "csi%d",sensor->csi_id);
	if(sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int imx219_query_supports(struct imx219 *sensor, void* parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(pimx219_mode_info);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, pimx219_mode_info,
			   sizeof(pimx219_mode_info));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;

}

static int imx219_get_sensor_id(struct imx219 *sensor, void* pchip_id)
{
	int ret = 0;
	u16 chip_id;
	u8 chip_id_high = 0;
	u8 chip_id_low = 0;
	ret = imx219_read_reg(sensor, 0x300a, &chip_id_high);
	ret |= imx219_read_reg(sensor, 0x300b, &chip_id_low);

	chip_id = ((chip_id_high & 0xff) << 8) | (chip_id_low & 0xff);

	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx219_get_reserve_id(struct imx219 *sensor, void* preserve_id)
{
	int ret = 0;
	u16 reserve_id = 0x2770;
	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx219_get_sensor_mode(struct imx219 *sensor, void* pmode)
{
	int ret = 0;
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx219_set_sensor_mode(struct imx219 *sensor, void* pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;
	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(pimx219_mode_info); i++) {
		if (pimx219_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pimx219_mode_info[i],
				sizeof(struct vvcam_mode_info_s));
			if ((pimx219_mode_info[i].index == 1) &&
			    (sensor->ocp.max_pixel_frequency == 266000000)) {
				sensor->cur_mode.preg_data =
					imx219_init_setting_1080p_hdr_low_freq;
				sensor->cur_mode.reg_data_count =
					ARRAY_SIZE(imx219_init_setting_1080p_hdr_low_freq);
				sensor->cur_mode.ae_info.one_line_exp_time_ns = 60784;
			}
			return 0;
		}
	}

	return -ENXIO;
}

static int imx219_set_lexp(struct imx219 *sensor, u32 exp)
{
	return 0;
}

static int imx219_set_exp(struct imx219 *sensor, u32 exp)
{
	int ret = 0;
	ret |= imx219_write_reg(sensor, 0x3467, 0x00);
	ret |= imx219_write_reg(sensor, 0x3464, 0x04);

	ret |= imx219_write_reg(sensor, 0x30b6, (exp >> 8) & 0xff);
	ret |= imx219_write_reg(sensor, 0x30b7, exp & 0xff);

	ret |= imx219_write_reg(sensor, 0x3464, 0x14);
	ret |= imx219_write_reg(sensor, 0x3467, 0x01);
	return ret;
}

static int imx219_set_vsexp(struct imx219 *sensor, u32 exp)
{
	int ret = 0;
	if (exp == 0x16)
		exp = 0x17;
	ret |= imx219_write_reg(sensor, 0x3467, 0x00);
	ret |= imx219_write_reg(sensor, 0x3464, 0x04);

	ret |= imx219_write_reg(sensor, 0x30b8, (exp >> 8) & 0xff);
	ret |= imx219_write_reg(sensor, 0x30b9, exp & 0xff);

	ret |= imx219_write_reg(sensor, 0x3464, 0x14);
	ret |= imx219_write_reg(sensor, 0x3467, 0x01);
	return ret;
}

static int imx219_set_lgain(struct imx219 *sensor, u32 gain)
{
	u32 again = 0;
	u32 dgain = 0;

	if (gain < ((2 * DCG_CONVERSION_GAIN) << 10)){
		gain = (2 * DCG_CONVERSION_GAIN) << 10;
	}

	if (gain < ((2 * DCG_CONVERSION_GAIN) << 10)) {
		again = 0;
	} else if (gain < ((4 * DCG_CONVERSION_GAIN) << 10)) {
		again = 1;
	} else if (gain < ((8 * DCG_CONVERSION_GAIN) << 10)) {
		again = 2;
	} else {
		again = 3;
	}
	dgain = (gain * 0x100) / ((( 1<< again) * DCG_CONVERSION_GAIN) << 10);

	sensor->hcg_again = again;
	sensor->hcg_dgain = dgain;
	return 0;
}

static int imx219_set_gain(struct imx219 *sensor, u32 gain)
{
	int ret = 0;
	u32 again = 0;
	u32 dgain = 0;
	u8 reg_val;

	if (gain < (3 << 10)){
		gain = 3  << 10;
	}

	if (gain < (3 << 10)) {
		again = 0;
	} else if (gain < (6 << 10)) {
		again = 1;
	} else if (gain < (12 << 10)) {
		again = 2;
	} else {
		again = 3;
	}
	dgain = (gain * 0x100) / (( 1<< again) << 10);

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		ret = imx219_read_reg(sensor, 0x30bb, &reg_val);
		reg_val &= ~0x03;
		reg_val |= again  & 0x03;

		ret  = imx219_write_reg(sensor, 0x3467, 0x00);
		ret |= imx219_write_reg(sensor, 0x3464, 0x04);

		ret |= imx219_write_reg(sensor, 0x30bb, reg_val);
		ret |= imx219_write_reg(sensor, 0x315a, (dgain>>8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x315b, dgain & 0xff);

		ret |= imx219_write_reg(sensor, 0x3464, 0x14);
		ret |= imx219_write_reg(sensor, 0x3467, 0x01);
	} else {
		ret = imx219_read_reg(sensor, 0x30bb, &reg_val);

		reg_val &= ~0x03;
		reg_val |= sensor->hcg_again  & 0x03;

		reg_val &= ~(0x03 << 2);
		reg_val |= (again & 0x03) << 2;

		ret  = imx219_write_reg(sensor, 0x3467, 0x00);
		ret |= imx219_write_reg(sensor, 0x3464, 0x04);

		ret |= imx219_write_reg(sensor, 0x30bb, reg_val);

		ret |= imx219_write_reg(sensor, 0x315a, (sensor->hcg_dgain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x315b, sensor->hcg_dgain & 0xff);

		ret |= imx219_write_reg(sensor, 0x315c, (dgain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x315d, dgain & 0xff);

		ret |= imx219_write_reg(sensor, 0x3464, 0x14);
		ret |= imx219_write_reg(sensor, 0x3467, 0x01);
	}

	return ret;
}

static int imx219_set_vsgain(struct imx219 *sensor, u32 gain)
{
	int ret = 0;
	u32 again = 0;
	u32 dgain = 0;
	u8 reg_val;

	if (gain < (3 << 10)) {
		gain = (3 << 10);
	}

	if (gain < (3 << 10)) {
		again = 0;
	} else if (gain < (6 << 10)) {
		again = 1;
	} else if (gain < (12 << 10)) {
		again = 2;
	} else {
		again = 3;
	}
	dgain = (gain * 0x100) / ((1 << again) << 10);

	ret = imx219_read_reg(sensor, 0x30bb, &reg_val);
	reg_val &= ~0x30;
	reg_val |= (again & 0x03) << 4;

	ret = imx219_write_reg(sensor, 0x3467, 0x00);
	ret |= imx219_write_reg(sensor, 0x3464, 0x04);

	ret |= imx219_write_reg(sensor, 0x30bb, reg_val);
	ret |= imx219_write_reg(sensor, 0x315e, (dgain >> 8) & 0xff);
	ret |= imx219_write_reg(sensor, 0x315f, dgain & 0xff);

	ret |= imx219_write_reg(sensor, 0x3464, 0x14);
	ret |= imx219_write_reg(sensor, 0x3467, 0x01);

	return ret;
}

static int imx219_set_fps(struct imx219 *sensor, u32 fps)
{
	u32 vts;
	int ret = 0;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		fps = sensor->cur_mode.ae_info.max_fps;
	}
	else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}
	vts = sensor->cur_mode.ae_info.max_fps *
	      sensor->cur_mode.ae_info.def_frm_len_lines / fps;

	ret = imx219_write_reg(sensor, 0x30B2, (u8)(vts >> 8) & 0xff);
	ret |= imx219_write_reg(sensor, 0x30B3, (u8)(vts & 0xff));

	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		sensor->cur_mode.ae_info.max_integration_line = vts - 4;
	} else {
		if (sensor->cur_mode.stitching_mode ==
		    SENSOR_STITCHING_DUAL_DCG){
			sensor->cur_mode.ae_info.max_vsintegration_line = 44;
			sensor->cur_mode.ae_info.max_integration_line = vts -
				4 - sensor->cur_mode.ae_info.max_vsintegration_line;
		} else {
			sensor->cur_mode.ae_info.max_integration_line = vts - 4;
		}
	}
	sensor->cur_mode.ae_info.curr_frm_len_lines = vts;
	return ret;
}

static int imx219_get_fps(struct imx219 *sensor, u32 *pfps)
{
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int imx219_set_test_pattern(struct imx219 *sensor, void * arg)
{
	int ret;
	struct sensor_test_pattern_s test_pattern;

	ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
	if (ret != 0)
		return -ENOMEM;
	if (test_pattern.enable) {
		switch (test_pattern.pattern) {
		case 0:
			ret = imx219_write_reg(sensor, 0x303a, 0x04);
			ret |= imx219_write_reg(sensor, 0x3253, 0x80);
			break;
		case 1:
			ret = imx219_write_reg(sensor, 0x303a, 0x04);
			ret |= imx219_write_reg(sensor, 0x3253, 0x82);
			break;
		case 2:
			ret = imx219_write_reg(sensor, 0x303a, 0x04);
			ret |= imx219_write_reg(sensor, 0x3253, 0x83);
			break;
		case 3:
			ret = imx219_write_reg(sensor, 0x303a, 0x04);
			ret |= imx219_write_reg(sensor, 0x3253, 0x92);
			break;
		default:
			ret = -1;
			break;
		}
	} else {
		ret = imx219_write_reg(sensor, 0x303a, 0x04);
		ret |= imx219_write_reg(sensor, 0x3253, 0x00);
	}
	return ret;
}

static int imx219_set_ratio(struct imx219 *sensor, void* pratio)
{
	int ret = 0;
	struct sensor_hdr_artio_s hdr_ratio;
	struct vvcam_ae_info_s *pae_info = &sensor->cur_mode.ae_info;

	ret = copy_from_user(&hdr_ratio, pratio, sizeof(hdr_ratio));

	if ((hdr_ratio.ratio_l_s != pae_info->hdr_ratio.ratio_l_s) ||
	    (hdr_ratio.ratio_s_vs != pae_info->hdr_ratio.ratio_s_vs) ||
	    (hdr_ratio.accuracy != pae_info->hdr_ratio.accuracy)) {
		pae_info->hdr_ratio.ratio_l_s = hdr_ratio.ratio_l_s;
		pae_info->hdr_ratio.ratio_s_vs = hdr_ratio.ratio_s_vs;
		pae_info->hdr_ratio.accuracy = hdr_ratio.accuracy;
		/*imx219 vs exp is limited for isp,so no need update max exp*/
	}

	return 0;
}

static int imx219_set_blc(struct imx219 *sensor, sensor_blc_t *pblc)
{
	int ret = 0;
	u32 r_offset, gr_offset, gb_offset, b_offset;
	u32 r_gain, gr_gain, gb_gain, b_gain;

	r_gain  = sensor->wb.r_gain;
	gr_gain = sensor->wb.gr_gain;
	gb_gain = sensor->wb.gb_gain;
	b_gain  = sensor->wb.b_gain;

	if (r_gain < 0x100)
		r_gain = 0x100;
	if (gr_gain < 0x100)
		gr_gain = 0x100;
	if (gb_gain < 0x100)
		gb_gain = 0x100;
	if (b_gain < 0x100)
		b_gain = 0x100;

	r_offset  = (r_gain  - 0x100) * pblc->red;
	gr_offset = (gr_gain - 0x100) * pblc->gr;
	gb_offset = (gb_gain - 0X100) * pblc->gb;
	b_offset  = (b_gain  - 0X100) * pblc->blue;
	//R,Gr,Gb,B HCG Offset
	ret |= imx219_write_reg(sensor, 0x3378, (r_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3379, (r_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x337a,  r_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x337b, (gr_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x337c, (gr_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x337d,  gr_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x337e, (gb_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x337f, (gb_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3380,  gb_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x3381, (b_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3382, (b_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3383,  b_offset        & 0xff);

	//R,Gr,Gb,B LCG Offset
	ret |= imx219_write_reg(sensor, 0x3384, (r_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3385, (r_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3386,  r_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x3387, (gr_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3388, (gr_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3389,  gr_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x338a, (gb_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x338b, (gb_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x338c,  gb_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x338d, (b_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x338e, (b_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x338f,  b_offset        & 0xff);

	//R,Gr,Gb,B VS Offset
	ret |= imx219_write_reg(sensor, 0x3390, (r_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3391, (r_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3392,  r_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x3393, (gr_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3394, (gr_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3395,  gr_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x3396, (gb_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x3397, (gb_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x3398,  gb_offset        & 0xff);

	ret |= imx219_write_reg(sensor, 0x3399, (b_offset >> 16) & 0xff);
	ret |= imx219_write_reg(sensor, 0x339a, (b_offset >> 8)  & 0xff);
	ret |= imx219_write_reg(sensor, 0x339b,  b_offset        & 0xff);

	memcpy(&sensor->blc, pblc, sizeof(sensor_blc_t));
	return ret;
}

static int imx219_set_wb(struct imx219 *sensor, void *pwb_cfg)
{
	int ret = 0;
	bool update_flag = false;
	struct sensor_white_balance_s wb;
	ret = copy_from_user(&wb, pwb_cfg, sizeof(wb));
	if (ret != 0)
		return -ENOMEM;
	if (wb.r_gain != sensor->wb.r_gain) {
		ret |= imx219_write_reg(sensor, 0x3360, (wb.r_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3361,  wb.r_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x3368, (wb.r_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3369,  wb.r_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x3370, (wb.r_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3371,  wb.r_gain & 0xff);
		update_flag = true;
	}

	if (wb.gr_gain != sensor->wb.gr_gain) {
		ret |= imx219_write_reg(sensor, 0x3362, (wb.gr_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3363,  wb.gr_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x336a, (wb.gr_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x336b,  wb.gr_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x3372, (wb.gr_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3373,  wb.gr_gain & 0xff);
		update_flag = true;
	}

	if (wb.gb_gain != sensor->wb.gb_gain) {
		ret |= imx219_write_reg(sensor, 0x3364, (wb.gb_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3365,  wb.gb_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x336c, (wb.gb_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x336d,  wb.gb_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x3374, (wb.gb_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3375,  wb.gb_gain & 0xff);
		update_flag = true;
	}

	if (wb.b_gain != sensor->wb.b_gain) {
		ret |= imx219_write_reg(sensor, 0x3366, (wb.b_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3367,  wb.b_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x336e, (wb.b_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x336f,  wb.b_gain & 0xff);
		ret |= imx219_write_reg(sensor, 0x3376, (wb.b_gain >> 8) & 0xff);
		ret |= imx219_write_reg(sensor, 0x3377,  wb.b_gain & 0xff);
		update_flag = true;
	}
	if (ret != 0)
		return ret;

	memcpy (&sensor->wb, &wb, sizeof(struct sensor_white_balance_s));

	if (update_flag) {
		ret = imx219_set_blc(sensor, &sensor->blc);
	}
	return ret;
}

static int imx219_get_expand_curve(struct imx219 *sensor,
				   sensor_expand_curve_t* pexpand_curve)
{
	int i;
	if ((pexpand_curve->x_bit == 12) && (pexpand_curve->y_bit == 16)) {
		uint8_t expand_px[64] = {6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
					6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
					6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
					6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};

		memcpy(pexpand_curve->expand_px,expand_px,sizeof(expand_px));

		pexpand_curve->expand_x_data[0] = 0;
		pexpand_curve->expand_y_data[0] = 0;
		for(i = 1; i < 65; i++) {
			pexpand_curve->expand_x_data[i] =
				(1 << pexpand_curve->expand_px[i-1]) +
				pexpand_curve->expand_x_data[i-1];

			if (pexpand_curve->expand_x_data[i] < 512) {
				pexpand_curve->expand_y_data[i] =
					pexpand_curve->expand_x_data[i] << 1;

			} else if (pexpand_curve->expand_x_data[i] < 768)
			{
				pexpand_curve->expand_y_data[i] =
					(pexpand_curve->expand_x_data[i] - 256) << 2 ;

			} else if (pexpand_curve->expand_x_data[i] < 2560) {
				pexpand_curve->expand_y_data[i] =
					(pexpand_curve->expand_x_data[i] - 512) << 3 ;

			} else {
				pexpand_curve->expand_y_data[i] =
					(pexpand_curve->expand_x_data[i] - 2048) << 5;
			}
		}
		return 0;
	}
	return -1;
}

static long imx219_priv_ioctl(struct v4l2_subdev *sd,
                              unsigned int cmd,
                              void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2775 *sensor = client_to_ov2775(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;
	uint32_t value = 0;
	sensor_blc_t blc;
	sensor_expand_curve_t expand_curve;

	mutex_lock(&sensor->lock);
	switch (cmd){
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = imx219_get_clk(sensor,arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = imx219_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = imx219_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = imx219_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = imx219_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx219_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx219_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_s_stream(&sensor->subdev, value);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx219_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx219_read_reg(sensor, sensor_reg.addr,
			(u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_LONG_EXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_lexp(sensor, value);
		break;
	case VVSENSORIOC_S_EXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_exp(sensor, value);
		break;
	case VVSENSORIOC_S_VSEXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_vsexp(sensor, value);
		break;
	case VVSENSORIOC_S_LONG_GAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_lgain(sensor, value);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_gain(sensor, value);
		break;
	case VVSENSORIOC_S_VSGAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_vsgain(sensor, value);
		break;
	case VVSENSORIOC_S_FPS:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx219_set_fps(sensor, value);
		break;
	case VVSENSORIOC_G_FPS:
		ret = imx219_get_fps(sensor, &value);
		ret |= copy_to_user(arg, &value, sizeof(value));
		break;
	case VVSENSORIOC_S_HDR_RADIO:
		ret = imx219_set_ratio(sensor, arg);
		break;
	case VVSENSORIOC_S_BLC:
		ret = copy_from_user(&blc, arg, sizeof(blc));
		ret |= imx219_set_blc(sensor, &blc);
		break;
	case VVSENSORIOC_S_WB:
		ret = imx219_set_wb(sensor, arg);
		break;
	case VVSENSORIOC_G_EXPAND_CURVE:
		ret = copy_from_user(&expand_curve, arg, sizeof(expand_curve));
		ret |= imx219_get_expand_curve(sensor, &expand_curve);
		ret |= copy_to_user(arg, &expand_curve, sizeof(expand_curve));
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret= imx219_set_test_pattern(sensor, arg);
		break;
	default:
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops imx219_core_ops = {
	.s_power = imx219_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
    .ioctl = imx219_priv_ioctl,
};

static const struct v4l2_subdev_video_ops imx219_video_ops = {
	.s_stream = imx219_set_stream,
};

static const struct v4l2_subdev_pad_ops imx219_pad_ops = {
	.enum_mbus_code = imx219_enum_mbus_code,
	.get_fmt = imx219_get_pad_format,
	.set_fmt = imx219_set_pad_format,
	.get_selection = imx219_get_selection,
	.enum_frame_size = imx219_enum_frame_size,
};

static const struct v4l2_subdev_ops imx219_subdev_ops = {
	.core = &imx219_core_ops,
	.video = &imx219_video_ops,
	.pad = &imx219_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx219_internal_ops = {
	.open = imx219_open,
};

/* Initialize control handlers */
static int imx219_init_controls(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int height = imx219->mode->height;
	struct v4l2_fwnode_device_properties props;
	int exposure_max, exposure_def, hblank;
	int i, ret;

	ctrl_hdlr = &imx219->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	mutex_init(&imx219->mutex);
	ctrl_hdlr->lock = &imx219->mutex;

	/* By default, PIXEL_RATE is read only */
	imx219->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX219_PIXEL_RATE,
					       IMX219_PIXEL_RATE, 1,
					       IMX219_PIXEL_RATE);

	imx219->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx219_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(imx219_link_freq_menu) - 1, 0,
				       imx219_link_freq_menu);
	if (imx219->link_freq)
		imx219->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure parameters based on current mode */
	imx219->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					   V4L2_CID_VBLANK, IMX219_VBLANK_MIN,
					   IMX219_VTS_MAX - height, 1,
					   imx219->mode->vts_def - height);
	hblank = IMX219_PPL_DEFAULT - imx219->mode->width;
	imx219->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx219->hblank)
		imx219->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = imx219->mode->vts_def - 4;
	exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
		exposure_max : IMX219_EXPOSURE_DEFAULT;
	imx219->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX219_EXPOSURE_MIN, exposure_max,
					     IMX219_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANA_GAIN_MIN, IMX219_ANA_GAIN_MAX,
			  IMX219_ANA_GAIN_STEP, IMX219_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX219_DGTL_GAIN_MIN, IMX219_DGTL_GAIN_MAX,
			  IMX219_DGTL_GAIN_STEP, IMX219_DGTL_GAIN_DEFAULT);

	imx219->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx219->hflip)
		imx219->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx219->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx219->vflip)
		imx219->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx219_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx219_test_pattern_menu) - 1,
				     0, 0, imx219_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX219_TESTP_COLOUR_MIN,
				  IMX219_TESTP_COLOUR_MAX,
				  IMX219_TESTP_COLOUR_STEP,
				  IMX219_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx219_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx219->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx219->mutex);

	return ret;
}

static void imx219_free_controls(struct imx219 *imx219)
{
	v4l2_ctrl_handler_free(imx219->sd.ctrl_handler);
	mutex_destroy(&imx219->mutex);
}

static int imx219_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX219_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx219_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx219_sd_media_ops = {
	.link_setup = imx219_link_setup,
};

static int imx219_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx219 *imx219;
	int ret;

	imx219 = devm_kzalloc(&client->dev, sizeof(*imx219), GFP_KERNEL);
	if (!imx219)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (imx219_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx219->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx219->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx219->xclk);
	}

	imx219->xclk_freq = clk_get_rate(imx219->xclk);
	if (imx219->xclk_freq != IMX219_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx219->xclk_freq);
		return -EINVAL;
	}

	ret = imx219_get_regulators(imx219);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx219->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx219_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx219_power_on(dev);
	if (ret)
		return ret;

	ret = imx219_identify_module(imx219);
	if (ret)
		goto error_power_off;

	/* Set default mode to max resolution */
	imx219->mode = &supported_modes[0];

	/* sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
			       IMX219_REG_VALUE_08BIT, IMX219_MODE_STREAMING);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
			       IMX219_REG_VALUE_08BIT, IMX219_MODE_STANDBY);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	ret = imx219_init_controls(imx219);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx219->sd.internal_ops = &imx219_internal_ops;
	imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx219->sd.entity.ops = &imx219_sd_media_ops;

	/* Initialize source pad */
	imx219->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	imx219_set_default_format(imx219);

	ret = media_entity_pads_init(&imx219->sd.entity, 1, &imx219->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx219->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

    dev_info(dev, "successfully probed\n");

	return 0;

error_media_entity:
	media_entity_cleanup(&imx219->sd.entity);

error_handler_free:
	imx219_free_controls(imx219);

error_power_off:
	imx219_power_off(dev);

	return ret;
}

static int imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx219 *imx219 = to_imx219(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx219_free_controls(imx219);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx219_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id imx219_dt_ids[] = {
	{ .compatible = "sony,imx219" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx219_dt_ids);

static const struct dev_pm_ops imx219_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx219_suspend, imx219_resume)
	SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
};

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.of_match_table	= imx219_dt_ids,
		.pm = &imx219_pm_ops,
	},
	.probe_new = imx219_probe,
	.remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_AUTHOR("Dave Stevenson <dave.stevenson@raspberrypi.com");
MODULE_AUTHOR("Rost Kurylo <rost@maxlab.io");
MODULE_DESCRIPTION("Sony IMX219 sensor driver for NXP/Verisilicon VVCAM");
MODULE_LICENSE("GPL v2");
