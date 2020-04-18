/*
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "os08a20_mipi_v3.h"
#include "os08a20_regs_1080p.h"
#include "os08a20_regs_4k.h"
#include "os08a20_ioctl.h"

static int os08a20_framerates[] = {
	[os08a20_15_fps] = 15,
	[os08a20_30_fps] = 30,
};

static struct os08a20_mode_info os08a20_mode_info_data[2][os08a20_mode_MAX +
							  1] = {
	{
	 {os08a20_mode_4K_3840_2160, OS08A20_SCALING, 3840, 2160,
	  os08a20_init_setting_4k,
	  ARRAY_SIZE(os08a20_init_setting_4k)},

	 /*{os08a20_mode_720P_1280_720, -1, 0, 0, NULL, 0},
	    {os08a20_mode_NTSC_720_480, -1, 0, 0, NULL, 0},
	    {os08a20_mode_VGA_640_480, -1, 0, 0, NULL, 0},
	    {os08a20_mode_QVGA_320_240, -1, 0, 0, NULL, 0},
	    {os08a20_mode_QSXGA_2592_1944, SCALING, 2592, 1944,
	    os08a20_setting_15fps_QSXGA_2592_1944,
	    ARRAY_SIZE(os08a20_setting_15fps_QSXGA_2592_1944)}, */
	 },
	{
	 {os08a20_mode_1080P_1920_1080, OS08A20_SCALING, 1920, 1080,
	  os08a20_init_setting_1080p,
	  ARRAY_SIZE(os08a20_init_setting_1080p)},

	 /*{os08a20_mode_720P_1280_720, SUBSAMPLING, 1280, 720,
	    os08a20_setting_30fps_720P_1280_720,
	    ARRAY_SIZE(os08a20_setting_30fps_720P_1280_720)},
	    {os08a20_mode_NTSC_720_480, SUBSAMPLING, 720, 480,
	    os08a20_setting_30fps_NTSC_720_480,
	    ARRAY_SIZE(os08a20_setting_30fps_NTSC_720_480)},
	    {os08a20_mode_VGA_640_480, SUBSAMPLING, 640,  480,
	    os08a20_setting_30fps_VGA_640_480,
	    ARRAY_SIZE(os08a20_setting_30fps_VGA_640_480)},
	    {os08a20_mode_QVGA_320_240, SUBSAMPLING, 320,  240,
	    os08a20_setting_30fps_QVGA_320_240,
	    ARRAY_SIZE(os08a20_setting_30fps_QVGA_320_240)},
	    {os08a20_mode_QSXGA_2592_1944, -1, 0, 0, NULL, 0}, */
	 },
};

static struct os08a20_hs_info hs_setting[] = {
	/*{2592, 1944, 30, 0x0B},
	   {2592, 1944, 15, 0x10}, */

	{1920, 1080, 60, 0x0B},
	{1920, 1080, 15, 0x10},
	{3840, 2160, 30, 0x0B},
	{3840, 2160, 15, 0x10},

	/*{1280, 720,  30, 0x11},
	   {1280, 720,  15, 0x16},

	   {720,  480,  30, 0x1E},
	   {720,  480,  15, 0x23},

	   {640,  480,  30, 0x1E},
	   {640,  480,  15, 0x23},

	   {320,  240,  30, 0x1E},
	   {320,  240,  15, 0x23}, */
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;

static int os08a20_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *device_id);
static int os08a20_remove(struct i2c_client *client);

static void os08a20_stop(struct os08a20 *sensor);

static const struct i2c_device_id os08a20_id[] = {
	{"ov2775", 0},		/* FIXME: change with dts file */
	{},
};

MODULE_DEVICE_TABLE(i2c, os08a20_id);

static struct i2c_driver os08a20_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ov2775",	/* FIXME: change with dts file */
		   },
	.probe = os08a20_probe,
	.remove = os08a20_remove,
	.id_table = os08a20_id,
};

#if 0
static const struct os08a20_datafmt os08a20_colour_fmts[] = {
	{MEDIA_BUS_FMT_YVYU8_2X8, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};
#else
static const struct os08a20_datafmt os08a20_colour_fmts[] = {
	{MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_COLORSPACE_JPEG},
};
#endif

static enum os08a20_frame_rate to_os08a20_frame_rate(struct v4l2_fract
						     *timeperframe)
{
	enum os08a20_frame_rate rate;
	u32 tgt_fps;		/* target frames per secound */

	pr_info("enter %s\n", __func__);
	tgt_fps = timeperframe->denominator / timeperframe->numerator;

	if (tgt_fps == 30)
		rate = os08a20_30_fps;
	else if (tgt_fps == 15)
		rate = os08a20_15_fps;
	else
		rate = -EINVAL;

	return rate;
}

static uint16_t find_hs_configure(struct os08a20 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &sensor->streamcap.timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;
	u32 frame_rate = timeperframe->denominator / timeperframe->numerator;
	int i;

	pr_info("enter %s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(hs_setting); i++) {
		if (hs_setting[i].width == pix->width &&
		    hs_setting[i].height == pix->height &&
		    hs_setting[i].frame_rate == frame_rate)
			return hs_setting[i].val;
	}

	if (i == ARRAY_SIZE(hs_setting))
		dev_err(dev, "%s can not find hs configure\n", __func__);

	return -EINVAL;
}

/* Find a data format by a pixel code in an array */
static const struct os08a20_datafmt
*os08a20_find_datafmt(u32 code)
{
	int i;

	pr_info("enter %s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(os08a20_colour_fmts); i++)
		if (os08a20_colour_fmts[i].code == code)
			return os08a20_colour_fmts + i;

	return NULL;
}

static inline void os08a20_power_down(struct os08a20 *sensor, int enable)
{
	pr_info("enter %s\n", __func__);
	gpio_set_value_cansleep(sensor->pwn_gpio, enable);
	udelay(2000);
}

static inline void os08a20_reset(struct os08a20 *sensor)
{
	pr_info("enter %s\n", __func__);
	if (sensor->pwn_gpio < 0 || sensor->rst_gpio < 0)
		return;

	gpio_set_value_cansleep(sensor->pwn_gpio, 1);
	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	udelay(5000);

	gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	udelay(1000);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(20);
}

static int os08a20_regulator_enable(struct device *dev)
{
	int ret = 0;

	pr_info("enter %s\n", __func__);
	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OS08a20_VOLTAGE_DIGITAL_IO,
				      OS08a20_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		io_regulator = NULL;
		dev_warn(dev, "cannot get io voltage\n");
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OS08a20_VOLTAGE_DIGITAL_CORE,
				      OS08a20_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		core_regulator = NULL;
		dev_warn(dev, "cannot get core voltage\n");
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				      OS08a20_VOLTAGE_ANALOG,
				      OS08a20_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret)
			dev_err(dev, "set analog voltage failed\n");
		else
			dev_dbg(dev, "set analog voltage ok\n");
	} else {
		analog_regulator = NULL;
		dev_warn(dev, "cannot get analog voltage\n");
	}

	return ret;
}

s32 os08a20_write_reg(struct os08a20 * sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(sensor->i2c_client, au8Buf, 3) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}

	return 0;
}

s32 os08a20_read_reg(struct os08a20 * sensor, u16 reg, u8 * val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int os08a20_set_clk_rate(struct os08a20 *sensor)
{
	u32 tgt_xclk;		/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min_t(u32, tgt_xclk, (u32) OS08a20_XCLK_MAX);
	tgt_xclk = max_t(u32, tgt_xclk, (u32) OS08a20_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", sensor->mclk);
	return ret;
}

/* download os08a20 settings to sensor through i2c */
static int os08a20_download_firmware(struct os08a20 *sensor,
				     struct os08a20_reg_value *pModeSetting,
				     s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	pr_info("enter %s\n", __func__);
	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = os08a20_read_reg(sensor, RegAddr, &RegVal);
			if (retval < 0)
				break;

			RegVal &= ~(u8) Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = os08a20_write_reg(sensor, RegAddr, Val);
		if (retval < 0)
			break;

		if (Delay_ms)
			msleep(Delay_ms);
	}

	return retval;
}

static int os08a20_config_init(struct os08a20 *sensor)
{
	struct os08a20_reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	/* Configure os08a20 initial parm */
	pr_info("enter %s\n", __func__);
	pModeSetting = os08a20_init_setting_4k;
	ArySize = ARRAY_SIZE(os08a20_init_setting_4k);

	retval = os08a20_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		return retval;

	return 0;
}

static void os08a20_start(struct os08a20 *sensor)
{
	pr_info("enter %s\n", __func__);
#if 1
	os08a20_write_reg(sensor, 0x3012, 0x01);
#else
	os08a20_write_reg(sensor, 0x3008, 0x02);
	os08a20_write_reg(sensor, 0x4202, 0x00);
#endif
	/* Color bar control */
	/* os08a20_write_reg(sensor, 0x503d, 0x80); */

	/* skip the first three frame for 30fps */
	msleep(100);
}

static int os08a20_change_mode(struct os08a20 *sensor)
{
	struct os08a20_reg_value *pModeSetting = NULL;
	enum os08a20_mode mode = sensor->streamcap.capturemode;
	enum os08a20_frame_rate frame_rate =
	    to_os08a20_frame_rate(&sensor->streamcap.timeperframe);
	int ArySize = 0, retval = 0;

	pr_info("enter %s\n", __func__);
	if (mode > os08a20_mode_MAX || mode < os08a20_mode_MIN) {
		pr_err("Wrong os08a20 mode detected!\n");
		return -1;
	}

	pModeSetting = os08a20_mode_info_data[frame_rate][mode].init_data_ptr;
	ArySize = os08a20_mode_info_data[frame_rate][mode].init_data_size;

	sensor->pix.width = os08a20_mode_info_data[frame_rate][mode].width;
	sensor->pix.height = os08a20_mode_info_data[frame_rate][mode].height;

	if (sensor->pix.width == 0 || sensor->pix.height == 0 ||
	    pModeSetting == NULL || ArySize == 0) {
		pr_err("Not support mode=%d %s\n", mode,
		       (frame_rate == 0) ? "15(fps)" : "30(fps)");
		return -EINVAL;
	}

	retval = os08a20_download_firmware(sensor, pModeSetting, ArySize);

	return retval;
}

static void os08a20_stop(struct os08a20 *sensor)
{
	pr_info("enter %s\n", __func__);
#if 1
	os08a20_write_reg(sensor, 0x3012, 0x00);
#else
	os08a20_write_reg(sensor, 0x4202, 0x0f);
	os08a20_write_reg(sensor, 0x3008, 0x42);
	os08a20_write_reg(sensor, 0x4800, 0x24);
#endif
}

static int init_device(struct os08a20 *sensor)
{
	int retval;

	pr_info("enter %s\n", __func__);
	retval = os08a20_config_init(sensor);
	if (retval < 0)
		return retval;

	return 0;
}

/*!
 * os08a20_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int os08a20_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);

	pr_info("enter %s\n", __func__);
	if (on)
		clk_prepare_enable(sensor->sensor_clk);
	else
		clk_disable_unprepare(sensor->sensor_clk);

	sensor->on = on;
	return 0;
}

/*!
 * os08a20_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int os08a20_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	pr_info("enter %s\n", __func__);
	switch (a->type) {
		/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

		/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int os08a20_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;		/* target frames per secound */
	enum os08a20_mode mode = a->parm.capture.capturemode;
	int ret = 0;

	pr_info("enter %s\n", __func__);
	switch (a->type) {
		/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		if (mode > os08a20_mode_MAX || mode < os08a20_mode_MIN) {
			pr_err("The camera mode[%d] is not supported!\n", mode);
			return -EINVAL;
		}

		sensor->streamcap.capturemode = mode;
		sensor->streamcap.timeperframe = *timeperframe;
		break;

		/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			 a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int os08a20_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);

	pr_info("enter %s\n", __func__);
	if (enable)
		os08a20_start(sensor);
	else
		os08a20_stop(sensor);

	sensor->on = enable;
	return 0;
}

static struct os08a20_mode_info *get_max_resolution(enum os08a20_frame_rate
						    rate)
{
	u32 max_width;
	enum os08a20_mode mode;
	int i;

	pr_info("enter %s\n", __func__);
	mode = 0;
	max_width = os08a20_mode_info_data[rate][0].width;

	for (i = 0; i < (os08a20_mode_MAX + 1); i++) {
		if (os08a20_mode_info_data[rate][i].width > max_width) {
			max_width = os08a20_mode_info_data[rate][i].width;
			mode = i;
		}
	}
	return &os08a20_mode_info_data[rate][mode];
}

static struct os08a20_mode_info *match(struct v4l2_mbus_framefmt *fmt,
				       enum os08a20_frame_rate rate)
{
	struct os08a20_mode_info *info;
	int i;

	pr_info("enter %s\n", __func__);
	for (i = 0; i < (os08a20_mode_MAX + 1); i++) {
		if (fmt->width == os08a20_mode_info_data[rate][i].width &&
		    fmt->height == os08a20_mode_info_data[rate][i].height) {
			info = &os08a20_mode_info_data[rate][i];
			break;
		}
	}
	if (i == os08a20_mode_MAX + 1)
		info = NULL;

	return info;
}

static void try_to_find_resolution(struct os08a20 *sensor,
				   struct v4l2_mbus_framefmt *mf)
{
	enum os08a20_mode mode = sensor->streamcap.capturemode;
	struct v4l2_fract *timeperframe = &sensor->streamcap.timeperframe;
	enum os08a20_frame_rate frame_rate =
	    to_os08a20_frame_rate(timeperframe);
	struct device *dev = &sensor->i2c_client->dev;
	struct os08a20_mode_info *info;
	bool found = false;

	pr_info("enter %s\n", __func__);
	if ((mf->width == os08a20_mode_info_data[frame_rate][mode].width) &&
	    (mf->height == os08a20_mode_info_data[frame_rate][mode].height)) {
		info = &os08a20_mode_info_data[frame_rate][mode];
		found = true;
	} else {
		/* get mode info according to frame user's width and height */
		info = match(mf, frame_rate);
		if (info == NULL) {
			frame_rate ^= 0x1;
			info = match(mf, frame_rate);
			if (info) {
				sensor->streamcap.capturemode = -1;
				dev_err(dev, "%s %dx%d only support %s(fps)\n",
					__func__, info->width, info->height,
					(frame_rate == 0) ? "15fps" : "30fps");
				return;
			}
			goto max_resolution;
		}
		found = true;
	}

	/* get max resolution to resize */
max_resolution:
	if (!found) {
		frame_rate ^= 0x1;
		info = get_max_resolution(frame_rate);
	}

	sensor->streamcap.capturemode = info->mode;
	sensor->streamcap.timeperframe.denominator = (frame_rate) ? 30 : 15;
	sensor->pix.width = info->width;
	sensor->pix.height = info->height;
}

static int os08a20_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct os08a20_datafmt *fmt = os08a20_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);
	int ret;

	pr_info("enter %s\n", __func__);
	if (format->pad) {
		return -EINVAL;
	}

	if (!fmt) {
		mf->code = os08a20_colour_fmts[0].code;
		mf->colorspace = os08a20_colour_fmts[0].colorspace;
	}

	mf->field = V4L2_FIELD_NONE;
	try_to_find_resolution(sensor, mf);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ret = os08a20_change_mode(sensor);
	sensor->fmt = fmt;
	return ret;
}

static int os08a20_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct os08a20 *sensor = client_to_os08a20(client);

	pr_info("enter %s\n", __func__);
	if (format->pad)
		return -EINVAL;

	memset(mf, 0, sizeof(struct v4l2_mbus_framefmt));

	mf->code = os08a20_colour_fmts[0].code;
	mf->colorspace = os08a20_colour_fmts[0].colorspace;
	mf->width = sensor->pix.width;
	mf->height = sensor->pix.height;
	mf->field = V4L2_FIELD_NONE;
	mf->reserved[1] = find_hs_configure(sensor);

	dev_dbg(&client->dev,
		"%s code=0x%x, w/h=(%d,%d), colorspace=%d, field=%d\n",
		__func__, mf->code, mf->width, mf->height, mf->colorspace,
		mf->field);

	return 0;
}

static int os08a20_enum_code(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info("enter %s\n", __func__);
	if (code->pad || code->index >= ARRAY_SIZE(os08a20_colour_fmts))
		return -EINVAL;

	code->code = os08a20_colour_fmts[code->index].code;
	return 0;
}

/*!
 * os08a20_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int os08a20_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	pr_info("enter %s\n", __func__);
	if (fse->index > os08a20_mode_MAX)
		return -EINVAL;

	fse->max_width =
	    max(os08a20_mode_info_data[0][fse->index].width,
		os08a20_mode_info_data[1][fse->index].width);
	fse->min_width = fse->max_width;

	fse->max_height =
	    max(os08a20_mode_info_data[0][fse->index].height,
		os08a20_mode_info_data[1][fse->index].height);
	fse->min_height = fse->max_height;

	return 0;
}

/*!
 * os08a20_enum_frameintervals - V4L2 sensor interface handler for
 *				   VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int os08a20_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum
				       *fie)
{
	int i, j, count;

	pr_info("enter %s\n", __func__);
	if (fie->index < 0 || fie->index > os08a20_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(os08a20_framerates); i++) {
		for (j = 0; j < (os08a20_mode_MAX + 1); j++) {
			if (fie->width == os08a20_mode_info_data[i][j].width
			    && fie->height ==
			    os08a20_mode_info_data[i][j].height
			    && os08a20_mode_info_data[i][j].init_data_ptr !=
			    NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator =
				    os08a20_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int os08a20_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	return 0;
}

static struct v4l2_subdev_video_ops os08a20_subdev_video_ops = {
	.g_parm = os08a20_g_parm,
	.s_parm = os08a20_s_parm,
	.s_stream = os08a20_s_stream,
};

static const struct v4l2_subdev_pad_ops os08a20_subdev_pad_ops = {
	.enum_frame_size = os08a20_enum_framesizes,
	.enum_frame_interval = os08a20_enum_frameintervals,
	.enum_mbus_code = os08a20_enum_code,
	.set_fmt = os08a20_set_fmt,
	.get_fmt = os08a20_get_fmt,
};

static struct v4l2_subdev_core_ops os08a20_subdev_core_ops = {
	.s_power = os08a20_s_power,
	.ioctl = os08a20_priv_ioctl,
};

static struct v4l2_subdev_ops os08a20_subdev_ops = {
	.core = &os08a20_subdev_core_ops,
	.video = &os08a20_subdev_video_ops,
	.pad = &os08a20_subdev_pad_ops,
};

static const struct media_entity_operations os08a20_sd_media_ops = {
	.link_setup = os08a20_link_setup,
};

/*!
 * os08a20 I2C probe function
 *
 * @param adapter struct i2c_adapter *
 * @return  Error code indicating success or failure
 */

struct v4l2_device *os08a20_v4l2_dev;

static int os08a20_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	int retval;

	/* u8 chip_id_high, chip_id_low; */
	struct os08a20 *sensor;

	pr_info("enter %s\n", __func__);
	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	/* Set initial values for the sensor struct. */
	memset(sensor, 0, sizeof(*sensor));

	/* os08a20 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
					       GPIOF_OUT_INIT_HIGH,
					       "os08a20_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	/* request reset pin */
	sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->rst_gpio,
					       GPIOF_OUT_INIT_HIGH,
					       "os08a20_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	/* Set initial values for the sensor struct. */
	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
				      (u32 *) & (sensor->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}
	mdelay(2);

	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	mdelay(2);

	/* Set mclk rate before clk on */
	os08a20_set_clk_rate(sensor);

	retval = clk_prepare_enable(sensor->sensor_clk);
	if (retval < 0) {
		dev_err(dev, "%s: enable sensor clk fail\n", __func__);
		return -EINVAL;
	}
	mdelay(2);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(20);

	sensor->io_init = os08a20_reset;
	sensor->i2c_client = client;

	sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	sensor->pix.width = os08a20_mode_info_data[1][0].width;
	sensor->pix.height = os08a20_mode_info_data[1][0].height;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
	    V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	os08a20_regulator_enable(&client->dev);

	os08a20_power_down(sensor, 1);

	retval = init_device(sensor);
	if (retval < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		pr_warn("camera os08a20 init fail\n");
		return -ENODEV;
	}
	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &os08a20_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[OS08a20_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	retval = media_entity_pads_init(&sd->entity, OS08a20_SENS_PADS_NUM,
					sensor->pads);
	sd->entity.ops = &os08a20_sd_media_ops;
	if (retval < 0)
		return retval;
	retval = v4l2_device_register_subdev(os08a20_v4l2_dev, sd);
	if (retval < 0) {
		dev_err(&client->dev,
			"%s--Async register failed, ret=%d\n", __func__,
			retval);
		media_entity_cleanup(&sd->entity);
	}

	/* clk_disable_unprepare(sensor->sensor_clk); */

	pr_info("%s camera mipi os08a20, is found\n", __func__);
	return retval;
}

/*!
 * os08a20 I2C detach function
 *
 * @param client struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int os08a20_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os08a20 *sensor = client_to_os08a20(client);

	pr_info("enter %s\n", __func__);
	v4l2_device_unregister_subdev(sd);

	/* clk_unprepare(sensor->sensor_clk); */

	os08a20_power_down(sensor, 1);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

int os08a20_hw_register(struct v4l2_device *vdev)
{
	os08a20_v4l2_dev = vdev;
	return i2c_add_driver(&os08a20_i2c_driver);
}

void os08a20_hw_unregister(void)
{
	i2c_del_driver(&os08a20_i2c_driver);
}
