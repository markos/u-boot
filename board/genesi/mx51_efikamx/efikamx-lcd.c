#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx5x_pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <errno.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <i2c.h>

#include <../drivers/video/ipu.h>

#define	EFIKASB_LVDS_I2C	0x3a
#define	EFIKAMX_HDMI_I2C	0x39

uint8_t lcd_init_code[] = {
	0x00, 0x20, 0xAF, 0x59, 0x2B, 0xDE, 0x51, 0x00,
	0x00, 0x04, 0x17, 0x00, 0x58, 0x02, 0x00, 0x00,
	0x00, 0x3B, 0x01, 0x08, 0x00, 0x1E, 0x01, 0x05,
	0x00, 0x01, 0x72, 0x05, 0x32, 0x00, 0x00, 0x04,
	0x00, 0x00, 0x20, 0xA8, 0x02, 0x12, 0x00, 0x58,
	0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x02, 0x10, 0x01, 0x68, 0x03, 0xC2, 0x01,
	0x4A, 0x03, 0x46, 0x00, 0xF1, 0x01, 0x5C, 0x04,
	0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3A,
	0x18, 0x4B, 0x29, 0x5C, 0xDE, 0xF6, 0xE0, 0x1C,
	0x03, 0xFC, 0xE3, 0x1F, 0xF3, 0x75, 0x26, 0x45,
	0x4A, 0x91, 0x8A, 0xFF, 0x3F, 0x83, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x48,
	0x00, 0x01, 0x10, 0x01, 0x00, 0x00, 0x10, 0x04,
	0x02, 0x1F, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00,
	0x32, 0x00, 0x00, 0x04, 0x12, 0x00, 0x58, 0x02,
	0x02, 0x7C, 0x04, 0x98, 0x02, 0x11, 0x78, 0x18,
	0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

struct fb_videomode mx51_efika_mode = {
	.name		= "1024x600",
	.refresh	= 60,
	.xres		= 1024,
	.yres		= 600,
	.pixclock	= 22800,
	.left_margin	= 80,
	.right_margin	= 40,
	.upper_margin	= 21,
	.lower_margin	= 21,
	.hsync_len	= 4,
	.vsync_len	= 4,
	.sync		= 0x100,	/* Active Low */
	.vmode		= FB_VMODE_NONINTERLACED,
	0,
};

void setup_iomux_lcd(void)
{
	if (machine_is_efikasb()) {
		/* DISP RST */
		mxc_request_iomux(MX51_PIN_DISPB2_SER_DIN, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DISPB2_SER_DIN, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);

		/* DISP2 EN# */
		mxc_request_iomux(MX51_PIN_DISPB2_SER_CLK, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DISPB2_SER_CLK, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);
		/* LCD_PWRON */
		mxc_request_iomux(MX51_PIN_CSI1_D9, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D9, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);
		/* LCD_PWRON_R */
		mxc_request_iomux(MX51_PIN_CSI1_D8, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D8, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);
		/* LCD_BL_PWRON# */
		mxc_request_iomux(MX51_PIN_CSI2_D19, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_CSI2_D19, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);
		/* DISP1_EN# */
		mxc_request_iomux(MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DI1_D1_CS, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);

		mxc_iomux_set_pad(MX51_PIN_DI2_DISP_CLK, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_LOW | PAD_CTL_PKE_ENABLE);
	} else {
		/* HDMI Enable */
		mxc_request_iomux(MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DI1_D1_CS, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);

		/* VGA Enable */
		mxc_request_iomux(MX51_PIN_DISPB2_SER_CLK, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DISPB2_SER_CLK, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);

		/* HDMI Reset */
		mxc_request_iomux(MX51_PIN_DISPB2_SER_DIN, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DISPB2_SER_DIN, PAD_CTL_SRE_FAST |
					PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE);
	}
}

void setup_efikasb_lcd(void)
{
	int ret = 10;
	const int size = sizeof(lcd_init_code);

	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI1_D8), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 1);
	udelay(10000);

	/* Reset the LCD */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), 1);
	udelay(10000);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_D8), 1);
	udelay(5000);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 1);
	udelay(5000);

	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19), 0);
	udelay(10000);

	/* Program the LCD init code */
	while (--ret && i2c_write(EFIKASB_LVDS_I2C, 0, 1, lcd_init_code, size))
		;

	if (!ret)
		printf("EfikaSB LCD: I2C TX error\n");
}

#define	SIIHDMI_RETRIES	10

#define	SIIHDMI_TPI_REG_PIXEL_CLOCK_LSB				0x00
#define	SIIHDMI_TPI_REG_AVI_INPUT_FORMAT			0x09
#define	SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT			0x0a
#define	SIIHDMI_TPI_REG_AVI_DBYTE0				0x0c
#define	SIIHDMI_TPI_REG_SYS_CTRL				0x1a
#define	SIIHDMI_TPI_REG_DEVICE_ID				0x1b
#define	SIIHDMI_TPI_REG_PWR_STATE				0x1e
#define	SIIHDMI_TPI_REG_IER					0x3c
#define	SIIHDMI_TPI_REG_RQB					0xc7

#define	SIIHDMI_TPI_REG_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI	(1 << 0)
#define	SIIHDMI_TPI_REG_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN		(1 << 4)

#define	SIIHDMI_TPI_REG_DEVICE_ID_B0				0xb0

int siihdmi_read(uint8_t reg, uint8_t *val)
{
	int ret = SIIHDMI_RETRIES;

	while (--ret && i2c_read(EFIKAMX_HDMI_I2C, reg, 1, val, 1))
		udelay(1);

	if (!ret)
		printf("SIIHDMI: I2C Error\n");

	return !ret;
}

int siihdmi_write(uint8_t reg, uint8_t val)
{
	int ret = SIIHDMI_RETRIES;

	while (--ret && i2c_write(EFIKAMX_HDMI_I2C, reg, 1, &val, 1))
		udelay(1);

	if (!ret)
		printf("SIIHDMI: I2C Error\n");

	return !ret;
}

void siihdmi_reset(void)
{
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 1);

	/* Reset SII9022, 1 mS for reset sequence, 10 mS to leave reset */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 1);
	udelay(1000);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	udelay(10000);
}

int siihdmi_detect_rev(void)
{
	int ret = 0, try;
	uint8_t val;

	ret = siihdmi_write(SIIHDMI_TPI_REG_RQB, 0x0);
	if (ret)
		return ret;

	try = SIIHDMI_RETRIES;
	while (--try) {
		ret = siihdmi_read(SIIHDMI_TPI_REG_DEVICE_ID, &val);
		if (ret)
			goto exit;

		if (val == SIIHDMI_TPI_REG_DEVICE_ID_B0)
			break;
	}

	ret = !try;

exit:
	return ret;
}

int siihdmi_init(void)
{
	int ret = 0;

	siihdmi_reset();

	ret = siihdmi_detect_rev();
	if (ret)
		goto exit;

	/* Power up the transceiver, state D0 */
	ret = siihdmi_write(SIIHDMI_TPI_REG_PWR_STATE, 0x0);
	if (ret)
		goto exit;

	ret = siihdmi_write(SIIHDMI_TPI_REG_IER, 0x3);
	if (ret)
		goto exit;

	return 0;

exit:
	return ret;
}

int siihdmi_sink_present(void)
{
	int ret;
	uint8_t val;

	ret = siihdmi_read(SIIHDMI_TPI_REG_IER, &val);
	if (ret)
		return -1;

	/* IRQ Happened */
	if (val & 0x3)
		return 1;

	/* No IRQ */
	return 0;
}

int siihdmi_set_res(struct fb_videomode *mode)
{
	int ret, try;
	uint8_t ctrl;
	uint32_t pixclk = 1000000000UL / (10 * mode->pixclock);
	uint32_t htotal = mode->xres + mode->left_margin +
				mode->hsync_len + mode->right_margin;
	uint32_t vtotal = mode->yres + mode->upper_margin +
				mode->vsync_len + mode->lower_margin;
	uint32_t refresh = (pixclk * 100000UL) / (htotal * vtotal);
	uint16_t mode_packet[4] = { pixclk, htotal, vtotal, refresh };
	uint8_t empty[0xe] = {0};

	ret = siihdmi_read(SIIHDMI_TPI_REG_SYS_CTRL, &ctrl);
	if (ret)
		return ret;

	/* Set output to DVI and power down TMDS */
	ctrl &= ~SIIHDMI_TPI_REG_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;
	ctrl |= SIIHDMI_TPI_REG_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;

	ret = siihdmi_write(SIIHDMI_TPI_REG_SYS_CTRL, ctrl);
	if (ret)
		return ret;

	/* Wait frame drain time */
	udelay(1000);

	/* Send the mode data */
	try = SIIHDMI_RETRIES;
	while (--try) {
		ret = i2c_write(EFIKAMX_HDMI_I2C,
				SIIHDMI_TPI_REG_PIXEL_CLOCK_LSB, 1,
				(uint8_t *)mode_packet, sizeof(mode_packet));
		if (!ret)
			break;

		udelay(1000);
	}

	if (!try)
		return 1;

	ret = siihdmi_write(SIIHDMI_TPI_REG_AVI_INPUT_FORMAT, 0);
	if (ret)
		return ret;

	ret = siihdmi_write(SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT, 0);
	if (ret)
		return ret;

	try = SIIHDMI_RETRIES;
	while (--try) {
		ret =  i2c_write(EFIKAMX_HDMI_I2C, SIIHDMI_TPI_REG_AVI_DBYTE0,
					1, empty, sizeof(empty));
		if (!ret)
			break;

		udelay(1000);
	}

	if (!try)
		return 1;

	ctrl &= ~SIIHDMI_TPI_REG_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;

	ret = siihdmi_write(SIIHDMI_TPI_REG_SYS_CTRL, ctrl);
	if (ret)
		return ret;

	return 0;
}

void setup_efikamx_lcd(void)
{
	int ret;

	ret = siihdmi_init();
	if (ret)
		goto exit;

	if (siihdmi_sink_present() != 1) {
		printf("SIIHDMI: No sink present, output not enabled\n");
		return;
	}

	if (siihdmi_set_res(&mx51_efika_mode))
		goto exit;

	return;

exit:
	printf("SIIHDMI: Controller setup error, powering down\n");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 1);
}

void setup_efika_lcd(void)
{
	if (machine_is_efikasb())
		setup_efikasb_lcd();
	else
		setup_efikamx_lcd();
}

void setup_efika_lcd_early(void)
{
	int ret = 0;

	if (machine_is_efikasb())
		ret = ipuv3_fb_init(&mx51_efika_mode, 1, IPU_PIX_FMT_RGB565);
	else
		ret = ipuv3_fb_init(&mx51_efika_mode, 0, IPU_PIX_FMT_RGB24);

	if (ret)
		puts("LCD cannot be configured\n");
}
