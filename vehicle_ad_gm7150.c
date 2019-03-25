#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <media/v4l2-chip-ident.h>
#include <linux/videodev2.h>
#include "vehicle_cfg.h"
#include "vehicle_main.h"
#include "vehicle_ad.h"
#include "vehicle_ad_gm7150.h"

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif


enum {
	FORCE_PAL_WIDTH = 720,
	FORCE_PAL_HEIGHT = 576,
	FORCE_NTSC_WIDTH = 720,
	FORCE_NTSC_HEIGHT = 480,
	FORCE_CIF_OUTPUT_FORMAT = CIF_OUTPUT_FORMAT_420,
};

static v4l2_std_id std_old = V4L2_STD_NTSC;

#define SENSOR_REGISTER_LEN	1	/* sensor register address bytes*/
#define SENSOR_VALUE_LEN	1	/* sensor register value bytes*/

#define GM7150_SENSOR_BUS_PARAM	(SOCAM_MASTER |	\
					SOCAM_PCLK_SAMPLE_RISING |	\
					SOCAM_HSYNC_ACTIVE_HIGH |	\
					SOCAM_VSYNC_ACTIVE_HIGH |	\
					SOCAM_DATA_ACTIVE_HIGH |	\
					SOCAM_DATAWIDTH_8 |	\
					SOCAM_MCLK_24MHZ)

struct rk_sensor_reg {
	unsigned int reg;
	unsigned int val;
};

#define GM7150_STATUS_REG_1				0x88
#define GM7150_STATUS5_REG		        0x8c
#define GM7150_STATUS5_NO_SIGNAL		0x01
#define GM7150_STATUS5_AUTOD_MASK	    0x0f
#define GM7150_STATUS5_AUTOD_SWITCH	0x80
#define GM7150_STATUS5_AUTOD_STD		0x00
#define GM7150_STATUS5_AUTOD_NTSC_M_J	0x01
#define GM7150_STATUS5_AUTOD_PAL_B_D_N 0X03
#define GM7150_STATUS5_AUTOD_PAL_M		0X05
#define GM7150_STATUS5_AUTOD_PAL_NC	0X07
#define GM7150_STATUS5_AUTOD_NTSC_4_43 0x09
#define GM7150_STATUS5_AUTOD_SECAM		0x0b

#define SEQCMD_WAIT_MS	 0xFD000000
#define SEQCMD_END  0xFF000000

#define SensorWaitMs(a) 	   {SEQCMD_WAIT_MS, a}
#define SensorEnd   {SEQCMD_END, 0x00}

#define SENSOR_DG DBG
#define SENSOR_TR DBG
static struct ad_dev *pAd;

/* Preview resolution setting*/
static struct rk_sensor_reg sensor_preview_data_cvbs[] = {
	{0x03, 0x0d},
	SensorEnd
};

static v4l2_std_id gm7150_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!(status1 & GM7150_STATUS5_NO_SIGNAL))
		return V4L2_STD_UNKNOWN;

	switch (status1 & GM7150_STATUS5_AUTOD_MASK) {
	    case GM7150_STATUS5_AUTOD_NTSC_M_J:
		return V4L2_STD_NTSC;
	    case GM7150_STATUS5_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	    case GM7150_STATUS5_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	    case GM7150_STATUS5_AUTOD_PAL_B_D_N:
		return V4L2_STD_PAL;
	    case GM7150_STATUS5_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	    default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 gm7150_status_to_v4l2(u8 status)
{
	if (!status)
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

int gm7150_ad_check_siganl(struct ad_dev *pAd)
{
    unsigned int pValue = -1,nRet = -1;

    pValue = vehicle_generic_sensor_read(pAd, 0x88);
    if(IS_ERR_VALUE(pValue)){
        printk("\n status0 failed = 0x%x\n",pValue);
    } 
 
    if((pValue >>4) == 0x7){
       printk("======No signal\n");
       nRet = 0 ;     
    } else if(pValue==0x10){
       printk("======No  signal \n");  
       nRet = 0 ; 
    }else{
       printk("have signal \n");
       nRet = 1 ; 
    }

    return nRet ;
}

static int gm7150_vehicle_status(struct ad_dev *ad,
				  u32 *status,
				  v4l2_std_id *std)
{
	char status1 = 0,status2 = -1;
	static int laststate = -1;
	int state;

	status1 = vehicle_generic_sensor_read(ad, GM7150_STATUS5_REG);
	if (status1 < 0)
		return status1;

	if (std)
		*std = gm7150_std_to_v4l2(status1);
    status2 = vehicle_generic_sensor_read(ad, GM7150_STATUS_REG_1);

	state = status2 & 0x06;
	if (status)
		*status = state == 0x06 ? 0 : 1;

	if (state != laststate){
		vehicle_ad_stat_change_notify();
		laststate = state;
	}
    
	return 0;
}

static void gm7150_reinit_parameter(struct ad_dev *ad, v4l2_std_id std)
{
	int i;

	ad->cfg.bus_param = GM7150_SENSOR_BUS_PARAM;
    printk("===CVSTD_720P60= std%d\n",(int)std);

	switch (std) {
	case V4L2_STD_PAL:
		ad->cfg.width = FORCE_PAL_WIDTH;
		ad->cfg.height = FORCE_PAL_HEIGHT;
		ad->cfg.start_x = 4;
		ad->cfg.start_y = 8;
		ad->cfg.input_format = CIF_INPUT_FORMAT_PAL;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;
		ad->cfg.frame_rate = 25;
		ad->cfg.skip_frames = 4;
		break;
	case V4L2_STD_NTSC:
		ad->cfg.width = 720;
		ad->cfg.height = 462;
		ad->cfg.start_x = 4;
		ad->cfg.start_y = 8;
		ad->cfg.input_format = CIF_INPUT_FORMAT_NTSC;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;
		ad->cfg.frame_rate = 30;
		ad->cfg.skip_frames = 4;
		break;
	default:
		ad->cfg.width = 720;
		ad->cfg.height = 480;
		ad->cfg.start_x = 8;
		ad->cfg.start_y = 20;
		ad->cfg.input_format = CIF_INPUT_FORMAT_NTSC;//CIF_INPUT_FORMAT_YUV;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;/*00 - UYVY*/
		/*href:0,high;1,low*/
        if (ad->cfg.bus_param | SOCAM_HSYNC_ACTIVE_HIGH)
            ad->cfg.href = 0;
        else
            ad->cfg.href = 1;
		/*vsync:0,low;1,high*/
		if (ad->cfg.bus_param | SOCAM_VSYNC_ACTIVE_HIGH)
			ad->cfg.vsync = 1;
		else
			ad->cfg.vsync = 0;
		ad->cfg.frame_rate = 25;
		ad->cfg.skip_frames = 4;
		break;		
	}

	/*href:0,high;1,low*/
	if (ad->cfg.bus_param | SOCAM_HSYNC_ACTIVE_HIGH)
		ad->cfg.href = 0;
	else
		ad->cfg.href = 1;
	/*vsync:0,low;1,high*/
	if (ad->cfg.bus_param | SOCAM_VSYNC_ACTIVE_HIGH)
		ad->cfg.vsync = 1;
	else
		ad->cfg.vsync = 0;

	/* fix crop info from dts config */
	for (i = 0; i < 4; i++) {
		if ((ad->defrects[i].width == ad->cfg.width) &&
		    (ad->defrects[i].height == ad->cfg.height)) {
			ad->cfg.start_x = ad->defrects[i].crop_x;
			ad->cfg.start_y = ad->defrects[i].crop_y;
			ad->cfg.width  =  ad->defrects[i].crop_width;
			ad->cfg.height  = ad->defrects[i].crop_height;
		}
	}

	DBG("gm7150 size %dx%d, crop(%d,%d)\n",
	    ad->cfg.width, ad->cfg.height,
	    ad->cfg.start_x, ad->cfg.start_y);
}

static void gm7150_reg_init(struct ad_dev *ad, unsigned char cvstd)
{
	struct rk_sensor_reg *sensor;
	int i = 0;
	unsigned char val[2];

	sensor = sensor_preview_data_cvbs;
    pAd = ad;
	while ((sensor[i].reg != SEQCMD_END) && (sensor[i].reg != 0xFC000000)) {
		val[0] = sensor[i].val;
		vehicle_generic_sensor_write(ad, sensor[i].reg, val);
		//DBG("%s write:=0x%x ,read:=0x%x\n", ad->ad_name,sensor[i].val, vehicle_generic_sensor_read(ad, sensor[i].reg));
		i++;
	}
}

int gm7150_ad_get_cfg(struct vehicle_cfg **cfg)
{
	u32 status = 0;
  
	if (!g_addev)
		return -1;

	gm7150_vehicle_status(g_addev, &status, NULL);
	if (status) /* No signal */
		g_addev->cfg.ad_ready = false;
	else
		g_addev->cfg.ad_ready = true;

	*cfg = &g_addev->cfg;

	return 0;
}

void gm7150_ad_check_cif_error(struct ad_dev *ad, int last_line)
{
	DBG("%s, last_line %d\n\n\n", __func__, last_line);
	if (last_line < 1)
		return;

	ad->cif_error_last_line = last_line;
	if (V4L2_STD_PAL == std_old) {
		if (last_line == FORCE_NTSC_HEIGHT) {
			if (ad->state_check_work.state_check_wq)
				queue_delayed_work(
						   ad->state_check_work.
						   state_check_wq,
						   &ad->state_check_work.work,
						   msecs_to_jiffies(0));
		}
	} else if (V4L2_STD_NTSC == std_old) {
		if (last_line == FORCE_PAL_HEIGHT) {
			if (ad->state_check_work.state_check_wq)
				queue_delayed_work(
						   ad->state_check_work.
						   state_check_wq,
						   &ad->state_check_work.work,
						   msecs_to_jiffies(0));
		}
	}
}
static void power_on(struct ad_dev *ad);
static void power_off(struct ad_dev *ad);
int gm7150_check_id(struct ad_dev *ad)
{
	int i = 0;
	int check_id_ret = -1;
	int val,valTemp;

	DBG("%s\n", __func__);

	/*  1. i2c init */
	ad->adapter = i2c_get_adapter(ad->i2c_chl);
	if (ad->adapter == NULL)
		return -1;

	if (!i2c_check_functionality(ad->adapter, I2C_FUNC_I2C)) {
		i2c_put_adapter(ad->adapter);
		ad->adapter = NULL;
		DBG("====%s i2c_check_functionality failed\n", __func__);
		return -1;
	}

	/*  2. ad power on*/
	power_on(ad);
	msleep(20);

	while (i++ < 5) {
		msleep(5);
		val = vehicle_generic_sensor_read(ad, 0x80);
		valTemp = vehicle_generic_sensor_read(ad, 0x81);
		DBG("%s read 0x80 --> 0x%02x ,0x81--->0x%02x \n", ad->ad_name, val,valTemp);
		if ((val == 0x71) && (valTemp == 0x50)){
			pr_info("gm7150 check id succeed\n");
			check_id_ret = 0;
			break;
		}
	}

	/*if sensor match, set raw params here*/
	if (check_id_ret == 0) {
		ad->fix_format = 0;
		ad->defrects[0].width = 720;
		ad->defrects[0].height = 462;//byfreehua 480;
		ad->defrects[0].crop_x = 4;
		ad->defrects[0].crop_y = 8;
		ad->defrects[0].crop_width = 712; 
		ad->defrects[0].crop_height = 462;
		ad->defrects[0].interface = "cvbs_ntsc";

		ad->defrects[1].width = 720;
		ad->defrects[1].height = 576;
		ad->defrects[1].crop_x = 8;
		ad->defrects[1].crop_y = 0;
		ad->defrects[1].crop_width = 712;
		ad->defrects[1].crop_height = 576;
		ad->defrects[1].interface = "cvbs_pal";

		DBG("%s: ad_chl=%d,,ad_addr=%x,fix_for=%d\n", ad->ad_name,
		    ad->ad_chl, ad->i2c_add, ad->fix_format);
		DBG("gpio power:%d, active:%d\n", ad->power, ad->pwr_active);
		DBG("gpio powerdown:%d, active:%d\n",
		    ad->powerdown, ad->pwdn_active);
	}

	/* if check failed, release resources*/
	if (check_id_ret == -1) {
		power_off(ad);
		i2c_put_adapter(ad->adapter);
		ad->adapter = NULL;
	}
	return check_id_ret;
}

static int gm7150_check_std(struct ad_dev *ad, v4l2_std_id *std)
{
	u32 status;
	static bool is_first = true;

	gm7150_vehicle_status(ad, &status, std);

	if (status && is_first) { /* No signal */
		mdelay(30);
		gm7150_vehicle_status(ad, &status, std);
		DBG("status 0x%x\n", status);
	}

	if (status)
		*std = V4L2_STD_NTSC;
    
	return 0;
}

static void power_on(struct ad_dev *ad)
{
	/* gpio_direction_output(ad->power, ad->pwr_active); */

	if (gpio_is_valid(ad->powerdown)) {
		gpio_request(ad->powerdown, "ad_powerdown");
		gpio_direction_output(ad->powerdown, !ad->pwdn_active);
		/* gpio_set_value(ad->powerdown, !ad->pwdn_active); */
	}

	if (gpio_is_valid(ad->power)) {
		gpio_request(ad->power, "ad_power");
		gpio_direction_output(ad->power, ad->pwr_active);
		/* gpio_set_value(ad->power, ad->pwr_active); */
	}
}

static void power_off(struct ad_dev *ad)
{
	if (gpio_is_valid(ad->powerdown))
		gpio_free(ad->powerdown);

	if (gpio_is_valid(ad->power))
		gpio_free(ad->power);
}

void gm7150_check_state_work(struct work_struct *work)
{
	struct ad_dev *ad;
	v4l2_std_id std;

	ad = g_addev;

	if (ad->cif_error_last_line > 0)
		ad->cif_error_last_line = 0;

	gm7150_check_std(ad, &std);

	if (std != std_old) {
		std_old = std;
		gm7150_reinit_parameter(ad, std);
		vehicle_ad_stat_change_notify();
	}    

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));
}

int gm7150_ad_deinit(void)
{
	struct ad_dev *ad;

	ad = g_addev;

	if (!ad)
		return -1;

	if (ad->state_check_work.state_check_wq) {
		cancel_delayed_work_sync(&ad->state_check_work.work);
		flush_delayed_work(&ad->state_check_work.work);
		flush_workqueue(ad->state_check_work.state_check_wq);
		destroy_workqueue(ad->state_check_work.state_check_wq);
	}
	if (ad->irq)
		free_irq(ad->irq, ad);
	power_off(ad);
	return 0;
}

int gm7150_ad_init(struct ad_dev *ad)
{
	int ret;
	v4l2_std_id std;

	if (!ad)
		return -1;

	g_addev = ad;

	/*  1. i2c init */
	if (ad->adapter == NULL) {
		ad->adapter = i2c_get_adapter(ad->i2c_chl);
        DBG("===gm7150_ad_init i2c channel=%d\n",ad->i2c_chl);
		if (ad->adapter == NULL)
			return -1;

		if (!i2c_check_functionality(ad->adapter, I2C_FUNC_I2C)){
			return -1;
		}
	}

	/*  2. ad power on, already power on when checkid*/
	/* fix mode */
	msleep(500);
	gm7150_check_std(ad, &std);
	std_old = std;
	DBG("std: %s\n", (std == V4L2_STD_NTSC) ? "ntsc" : "pal");

	/*  3 .init default format params */
	gm7150_reg_init(ad, std);
	gm7150_reinit_parameter(ad, std);
	vehicle_ad_stat_change_notify();

	/*  4. ad register signal detect irq */
	if(0){
		ad->irq = gpio_to_irq(ad->cvstd);
		ret = request_irq(ad->irq, NULL, IRQF_TRIGGER_FALLING,
				  "vehicle ad_gm7150", ad);
	}

	/*  5. create workqueue to detect signal change */
	INIT_DELAYED_WORK(&ad->state_check_work.work, gm7150_check_state_work);
	ad->state_check_work.state_check_wq =
		create_singlethread_workqueue("vehicle-ad-gm7150");

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));

	return 0;
}


