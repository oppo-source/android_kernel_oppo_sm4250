#define pr_fmt(fmt)	"[hl7138] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
//#include <linux/wakelock.h>
#include <linux/proc_fs.h>

#include <trace/events/sched.h>
#include<linux/ktime.h>
#include <linux/pm_qos.h>
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
//#include "oplus_sc8547.h"
#include "oplus_hl7138.h"
#include "vooc_base.h"
#include "oplus_vooc_20031.h"


extern int voocphy_log_level;

#define vphy_alert(fmt, ...)	\
do {					\
	if (voocphy_log_level <= 4)	\
		printk(KERN_ERR "[hl7138:]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define vphy_err(fmt, ...)	\
do {					\
	if (voocphy_log_level <= 3)	\
		printk(KERN_ERR "[hl7138:]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define vphy_info(fmt, ...)	\
do {						\
	if (voocphy_log_level <= 2)	\
		printk(KERN_INFO "[hl7138:]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define vphy_dbg(fmt, ...)	\
do {					\
	if (voocphy_log_level <= 1)	\
		printk(KERN_DEBUG "[hl7138:]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

struct chip_hl7138 {
	struct device *dev;
	struct i2c_client *client;

	int irq_gpio;
	int irq;

	//add for uart and cp
	int switch1_gpio;		//gpio 47
	struct pinctrl *pinctrl;
	struct pinctrl_state *charging_inter_active;
	struct pinctrl_state *charging_inter_sleep;

	struct pinctrl_state *charger_gpio_sw_ctrl2_high;
	struct pinctrl_state *charger_gpio_sw_ctrl2_low;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	int  vbus_error;
	int charger_mode;
	int direct_charge;

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	//struct chip_hl7138_cfg *cfg;

	int skip_writes;
	int skip_reads;

	//struct chip_hl7138_platform_data *platform_data;

	struct delayed_work irq_work;

	//struct dentry *debug_root;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *fc2_psy;


	//adpter request info
	u8 vooc_rxdata;
	u8 vooc_flag;
	struct wakeup_source ws;
};

static struct chip_hl7138 *gchip;

extern BattMngr_err_code_e oplus_vooc_init_res(UINT8 pmic_index);
extern bool oplus_chg_get_flash_led_status(void);
int hl7138_clear_int(void);
extern void opchg_voocphy_set_switch_normal_charger(void);
struct oplus_chg_chip* oplus_get_oplus_chip(void);

static int __hl7138_read_byte(struct chip_hl7138 *hl7138, u8 reg, u8 *data)
{
	s32 ret;
	//hl7138_dbg(" rm mutex enter!\n");
	ret = i2c_smbus_read_byte_data(hl7138->client, reg);
	if (ret < 0) {
		if(oplus_vooc_mg) {
			oplus_vooc_mg->voocphy_iic_err = 1;
		}
		vphy_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;
	//hl7138_dbg(" exit!\n");
	return 0;
}

static int __hl7138_write_byte(struct chip_hl7138 *hl7138, int reg, u8 val)
{
	s32 ret;

	//hl7138_dbg(" enter!\n");
	ret = i2c_smbus_write_byte_data(hl7138->client, reg, val);
	if (ret < 0) {
		if(oplus_vooc_mg) {
			oplus_vooc_mg->voocphy_iic_err = 1;
		}
		vphy_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	//hl7138_dbg(" exit!\n");
	return 0;
}

static int hl7138_read_byte(struct chip_hl7138 *hl7138, u8 reg, u8 *data)
{
	int ret;

	if (hl7138->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&hl7138->i2c_rw_lock);
	ret = __hl7138_read_byte(hl7138, reg, data);
	mutex_unlock(&hl7138->i2c_rw_lock);

	return ret;
}

static int hl7138_write_byte(struct chip_hl7138 *hl7138, u8 reg, u8 data)
{
	int ret;

	if (hl7138->skip_writes)
		return 0;

	mutex_lock(&hl7138->i2c_rw_lock);
	ret = __hl7138_write_byte(hl7138, reg, data);
	mutex_unlock(&hl7138->i2c_rw_lock);

	return ret;
}

static s32 hl7138_read_word(struct chip_hl7138 *hl7138, u8 reg)
{
	s32 ret;

	mutex_lock(&hl7138->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(hl7138->client, reg);
	if (ret < 0) {
		if(oplus_vooc_mg) {
			oplus_vooc_mg->voocphy_iic_err = 1;
		}
		vphy_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&hl7138->i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&hl7138->i2c_rw_lock);
	return ret;
}


static s32 hl7138_write_word(struct chip_hl7138 *hl7138, u8 reg, u16 val)
{
	s32 ret;

	mutex_lock(&hl7138->i2c_rw_lock);
	ret = i2c_smbus_write_word_data(hl7138->client, reg, val);
	if (ret < 0) {
		if(oplus_vooc_mg) {
			oplus_vooc_mg->voocphy_iic_err = 1;
		}
		vphy_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&hl7138->i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&hl7138->i2c_rw_lock);
	return 0;
}


static ssize_t hl7138_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct chip_hl7138 *hl7138 = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "hl7138");
	for (addr = 0x0; addr <= 0x4F; addr++) {
		if((addr < 0x18) || (addr > 0x35 && addr < 0x4F)) {
			ret = hl7138_read_byte(hl7138, addr, &val);
			if (ret == 0) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}

	return idx;
}

static ssize_t hl7138_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct chip_hl7138 *hl7138 = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x4F)
		hl7138_write_byte(hl7138, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, hl7138_show_registers, hl7138_store_register);


static int hl7138_reg_reset(struct chip_hl7138 *chip, bool enable)
{
	unsigned char value;
	int ret = 0;
	ret = hl7138_read_byte(gchip, HL7138_REG_01, &value);	//clear INTb

	hl7138_write_byte(gchip, HL7138_REG_09, 0x00);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_0A, 0x2E);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_0C, 0x03);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_0F, 0x00);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_13, 0x00);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_15, 0x00);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_17, 0x00);	//set default mode;
	hl7138_write_byte(gchip, HL7138_REG_37, 0x02);	//reset VOOC PHY;
	hl7138_write_byte(gchip, HL7138_REG_3F, 0xD1);	//bit7 T6:170us

	return ret;
}

static int hl7138_init_device(struct chip_hl7138 *chip)
{
	hl7138_write_byte(gchip, HL7138_REG_40, 0x00);	//ADC_CTRL:disable,JL:11-40;
	hl7138_write_byte(gchip, HL7138_REG_0B, 0x82);	//VAC_OVP=6V,JL:02->0B;
	//hl7138_write_byte(gchip, HL7138_REG_0C, 0x0F);		//VBUS_OVP:10.2 2:1 or 1:1V,JL:04-0C;
	hl7138_write_byte(gchip, HL7138_REG_11, 0xDC);	//ovp:90mV
	hl7138_write_byte(gchip, HL7138_REG_08, 0x38);	//VBAT_OVP:4.56	4.56+0.09
	hl7138_write_byte(gchip, HL7138_REG_0E, 0x32);	//IBUS_OCP:3.5A      ocp:100mA
	//hl7138_write_byte(gchip, HL7138_REG_0A, 0x2E);		//IBAT_OCP:max;JL:01-0A;0X2E=6.6A,MAX;
	hl7138_write_byte(gchip, HL7138_REG_37, 0x00);	//VOOC_CTRL:disable;JL:2B->37;

	hl7138_write_byte(gchip, HL7138_REG_3C, 0x85);	//diff mask inter;
	hl7138_write_byte(gchip, HL7138_REG_02, 0xE0);	//JL:mask all INT_FLAG
	hl7138_write_byte(gchip, HL7138_REG_10, 0xEC);	//JL:Dis IIN_REG;
	hl7138_write_byte(gchip, HL7138_REG_12, 0x05);	//JL:Fsw=500KHz;07->12;
	hl7138_write_byte(gchip, HL7138_REG_14, 0x08);	//JL:dis WDG;
	hl7138_write_byte(gchip, HL7138_REG_16, 0x3B);	//JL:OV=600, UV=200

	return 0;
}

static int hl7138_work_mode_lockcheck(struct chip_hl7138 *chip)
{
	unsigned char reg;

	if (!chip) {
		return -1;
	}

	if ((!hl7138_read_byte(chip, HL7138_REG_A7, &reg) && reg != 0x4)
		|| (!hl7138_read_byte(chip, HL7138_REG_D7, &reg) && reg != 0x80)) {
		/*test mode unlock & lock avoid burnning out the chip*/
		hl7138_write_byte(chip, HL7138_REG_A0, 0xF9);
		hl7138_write_byte(chip, HL7138_REG_A0, 0x9F);	//Unlock test register
		hl7138_write_byte(chip, HL7138_REG_A7, 0x04);
		hl7138_write_byte(chip, HL7138_REG_D7, 0x80);
		hl7138_write_byte(chip, HL7138_REG_A0, 0x00);	//Lock test register
		vphy_err("hl7138_work_mode_lockcheck done\n");
	}
	return 0;
}

static int hl7138_set_predata(vooc_manager *chip, u16 val)
{
	int ret;
	if (!gchip) {
		vphy_err("failed: gchip is null\n");
		return -1;
	}

	//predata, pre_wdata,JL: REG_31 change to REG_3D
	ret = hl7138_write_word(gchip, HL7138_REG_3D, val);
	if (ret < 0) {
		vphy_err("failed: write predata\n");
		return -1;
	}
	vphy_info("write predata 0x%0x\n", val);
	return ret;
}

int hl7138_init_vooc(vooc_manager *chip)
{
	vphy_err(">>>> start init vooc\n");

	oplus_vooc_reset_variables();
	vooc_monitor_stop(0);
	vooc_monitor_timer_stop(VOOC_THREAD_TIMER_COMMU, VOOC_COMMU_EVENT_TIME);

	hl7138_reg_reset(gchip, true);

	hl7138_work_mode_lockcheck(gchip);

	hl7138_init_device(gchip);

	//to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us
	hl7138_set_predata(oplus_vooc_mg, 0);

	return 0;
}

void hl7138_send_handshake_seq(vooc_manager *chip)
{
	hl7138_write_byte(gchip, HL7138_REG_37, 0x81);	//JL:2B->37,EN & Handshake;
}

static int hl7138_svooc_hw_setting(struct chip_hl7138 *chip)
{
	//hl7138_write_byte(gchip, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08;
	hl7138_write_byte(gchip, HL7138_REG_40, 0x05);	//ADC_CTRL:ADC_EN,JL:11-40;

	hl7138_write_byte(gchip, HL7138_REG_0B, 0x88);	//VAC_OVP:12v,JL:02->0B;
	hl7138_write_byte(gchip, HL7138_REG_0C, 0x00);	//VBUS_OVP:10.2v,,JL:04-0C;

	hl7138_write_byte(gchip, HL7138_REG_0E, 0x32);	//IBUS_OCP:3.6A,UCP_DEB=5ms;JL:05-0E;

	hl7138_write_byte(gchip, HL7138_REG_14, 0x02);	//WD:1000ms,JL:09-14;
	hl7138_write_byte(gchip, HL7138_REG_15, 0x00);	//enter cp mode
	hl7138_write_byte(gchip, HL7138_REG_16, 0x3B);	//JL:OV=600, UV=200

	/*avoid PBS01 & PBV01 chg break*/
	if (oplus_vooc_mg->err_trans_flg) {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0x91);	//Loose_det=1,JL:33-3F;
	} else {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0xD1);	//Loose_det=1,JL:33-3F;
	}
	return 0;
}

static int hl7138_vooc_hw_setting(struct chip_hl7138 *chip)
{
	//hl7138_write_byte(gchip, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08;
	hl7138_write_byte(gchip, HL7138_REG_40, 0x05);	//ADC_CTRL:ADC_EN,JL:11-40;

	hl7138_write_byte(gchip, HL7138_REG_0B, 0x82);	//VAC_OVP=6V,JL:02->0B;
	hl7138_write_byte(gchip, HL7138_REG_0C, 0x0F);	//VBUS_OVP:5.85v,,JL:04-0C;

	hl7138_write_byte(gchip, HL7138_REG_0E, 0x1A);	//IBUS_OCP:4.6A,(16+9)*0.1+0.1+2=4.6A;

	hl7138_write_byte(gchip, HL7138_REG_14, 0x02);	//WD:1000ms,JL:09-14;
	hl7138_write_byte(gchip, HL7138_REG_15, 0x80);	//JL:bp mode;
	hl7138_write_byte(gchip, HL7138_REG_16, 0x3B);	//JL:OV=600, UV=200

	/*avoid PBS01 & PBV01 chg break*/
	if (oplus_vooc_mg->err_trans_flg) {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0x91);	//Loose_det=1,JL:33-3F;
	} else {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0xD1);	//Loose_det=1,JL:33-3F;
	}
	return 0;
}


static int hl7138_5v2a_hw_setting(struct chip_hl7138 *chip)
{
	//hl7138_write_byte(gchip, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08;
	hl7138_write_byte(gchip, HL7138_REG_0B, 0x82);	//VAC_OVP=6V,JL:02->0B;
	hl7138_write_byte(gchip, HL7138_REG_0C, 0x0F);	//VBUS_OVP:10.5v,,JL:04-0C;
	hl7138_write_byte(gchip, HL7138_REG_0E, 0xAF);	//IBUS_OCP:3.6A,UCP_DEB=5ms;JL:05-0E;
	hl7138_write_byte(gchip, HL7138_REG_14, 0x08);	//WD:DIS,JL:09-14;
	hl7138_write_byte(gchip, HL7138_REG_15, 0x80);	//JL:bp mode;

	hl7138_write_byte(gchip, HL7138_REG_40, 0x00);	//ADC_CTRL:disable,JL:11-40;
	hl7138_write_byte(gchip, HL7138_REG_37, 0x00);	//VOOC_CTRL:disable;JL:2B->37;
	//hl7138_write_byte(gchip, HL7138_REG_3D, 0x01);	//pre_wdata,JL:31-3D;
	//hl7138_write_byte(gchip, HL7138_REG_3E, 0x80);	//pre_wdata,JL:32-3E;
	//hl7138_write_byte(gchip, HL7138_REG_3F, 0xd1);	//Loose_det=1,JL:33-3F;
	return 0;
}

int hl7138_enable_t5t6_check(bool enable)
{
	if (!enable) {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0x91);
	} else {
		hl7138_write_byte(gchip, HL7138_REG_3F, 0xD1);
	}
	return 0;
}

static int hl7138_hw_reset(void)
{
	hl7138_write_byte(gchip, HL7138_REG_14, 0xc8);
	mdelay(10);

	return 0;
}

static int hl7138_get_int_flag(vooc_manager *chip)
{
	int data;
	if (!gchip || !oplus_vooc_mg) {
		vphy_err("gchip or oplus_vooc_mg is null\n");
		return -1;
	}

	data = hl7138_read_word(gchip, HL7138_REG_05);
	if (data < 0) {
		vphy_err("hl7138_read_word faile\n");
		return -1;
	}
	oplus_vooc_mg->int_flag = data;

	return 0;
}

#define VIN_OVP_STS_MASK	BIT(7)
#define VIN_UVLO_STS_MASK	BIT(6)
#define TRACK_OV_STS_MASK	BIT(5)
#define TRACK_UV_STS_MASK	BIT(4)
#define VBAT_OVP_STS_MASK	BIT(3)
#define VOUT_OVP_STS_MASK	BIT(2)
#define PMID_QUAL_STS_MASK	BIT(1)
#define VBUS_UV_STS_MASK	BIT(0)

#define IIN_OCP_STS_MASK	BIT(7)
#define IBAT_OCP_STS_MASK	BIT(6)
#define IIN_UCP_STS_MASK	BIT(5)
#define FET_SHORT_STS_MASK	BIT(4)
#define CFLY_SHORT_STS_MASK	BIT(3)
#define DEV_MODE_STS_MASK	(BIT(2)| BIT(1))
#define THSD_STS_MASK		BIT(0)

int hl7138_print_cp_int_exception(vooc_manager * chip)
{
	int chg_exception_a = 0;
	int chg_exception_b = 0;
	unsigned char status_a, status_b;
	int i = 0;
	struct irqinfo status_a_bits[IRQ_EVNET_NUM] = {
		{VIN_OVP_STS_MASK, "VIN_OVP_STS", 1},
		{VIN_UVLO_STS_MASK, "VIN_UVLO_STS", 1},
		{TRACK_OV_STS_MASK, "TRACK_OV_STS", 0},
		{TRACK_UV_STS_MASK, "TRACK_UV_STS", 0},
		{VBAT_OVP_STS_MASK, "VBAT_OVP_STS", 1},
		{VOUT_OVP_STS_MASK, "VOUT_OVP_STS", 1},
		{PMID_QUAL_STS_MASK, "PMID_QUAL_STS", 0},
		{VBUS_UV_STS_MASK,   "VBUS_UV_STS", 1},
	};

	struct irqinfo status_b_bits[IRQ_EVNET_NUM] = {
		{IIN_OCP_STS_MASK, "IIN_OCP_STS", 1},
		{IBAT_OCP_STS_MASK, "IBAT_OCP_STS", 1},
		{IIN_UCP_STS_MASK, "IIN_UCP_STS", 1},
		{FET_SHORT_STS_MASK, "FET_SHORT_STS", 0},
		{CFLY_SHORT_STS_MASK, "CFLY_SHORT_STS", 0},
		{DEV_MODE_STS_MASK, "DEV_MODE_STS", 0},
		{THSD_STS_MASK, "THSD_STS", 0},
	};

	VOOCPHY_DATA16_SPLIT(chip->int_flag, status_a , status_b);
	for (i = 0; i < IRQ_EVNET_NUM; i++) {
		if ((status_a_bits[i].mask & status_a) && status_a_bits[i].mark_except) {
			printk("cp int status_a happened %s\n", status_a_bits[i].except_info);
			chg_exception_a = 1;
		}
	}

	for (i = 0; i < IRQ_EVNET_NUM; i++) {
		if ((status_b_bits[i].mask & status_b) && status_b_bits[i].mark_except) {
			printk("cp int status_b happened %s\n", status_b_bits[i].except_info);
			chg_exception_b = 1;
		}
	}

	vphy_info("int_flag, status_a, status_b:0x%0x 0x%0x 0x%0x\n", chip->int_flag, status_a, status_b);

	if (chg_exception_a || chg_exception_b) {
		printk("status_a:[0x%0x] status_b:[0x%0x]\n", status_a , status_b);
		opchg_voocphy_dump_reg();
		return true;
	} else {
		return false;
	}
}

static int hl7138_hw_setting(vooc_manager *chip, int reason)
{
	if (!gchip) {
		vphy_err("gchip is null exit\n");
		return -1;
	}
	switch (reason) {
		case SETTING_REASON_PROBE:
		case SETTING_REASON_RESET:
			hl7138_init_device(gchip);
			/*reset for avoiding PBS01 & PBV01 chg break*/
			hl7138_enable_t5t6_check(true);
			hl7138_hw_reset();
			vphy_info("SETTING_REASON_RESET OR PROBE\n");
			break;
		case SETTING_REASON_SVOOC:
			hl7138_svooc_hw_setting(gchip);
			vphy_info("SETTING_REASON_SVOOC\n");
			break;
		case SETTING_REASON_VOOC:
			hl7138_vooc_hw_setting(gchip);
			vphy_info("SETTING_REASON_VOOC\n");
			break;
		case SETTING_REASON_5V2A:
			hl7138_5v2a_hw_setting(gchip);
			vphy_info("SETTING_REASON_5V2A\n");
			break;
		default:
			vphy_err("do nothing\n");
			break;
	}
	return 0;
}

static int hl7138_set_txbuff(vooc_manager *chip, u16 val)
{
	int ret;
	if (!gchip) {
		vphy_err("failed: gchip is null\n");
		return -1;
	}

	//txbuff, tx_wdata, JL: REG_2C change to REG_38
	ret = hl7138_write_word(gchip, HL7138_REG_38, val);
	if (ret < 0) {
		vphy_err("failed: write txbuff\n");
		return -1;
	}
	//vphy_dbg("write txbuff 0x%0x\n", val);
	return ret;
}

static int hl7138_get_adapter_info(vooc_manager *chip)
{
	int data;
	if (!gchip || !oplus_vooc_mg) {
		vphy_err("gchip or oplus_vooc_mg is null\n");
		return -1;
	}

	data = hl7138_read_word(gchip, HL7138_REG_3A);		//JL: 2E=RX_Rdata,change to 0x3A

	if (data < 0) {
		vphy_err("hl7138_read_word faile\n");
		return -1;
	}

	VOOCPHY_DATA16_SPLIT(data, gchip->vooc_rxdata , gchip->vooc_flag);
	vphy_info("data, vooc_flag, vooc_rxdata: 0x%0x 0x%0x 0x%0x\n", data, gchip->vooc_flag, gchip->vooc_rxdata);

	return 0;
}

#define HL7138_SVOOC_IBUS_FACTOR	110/100
#define HL7138_VOOC_IBUS_FACTOR		215/100
extern int hl7138_clear_interrupts(vooc_manager *chip);
static void hl7138_update_data(vooc_manager *chip)
{
	u8 data_block[12] = {0};
	int i = 0;
	int ret = 0;

	hl7138_get_int_flag(oplus_vooc_mg);

	/*parse data_block for improving time of interrupt*/
	//JL:VIN,IIN,VBAT,IBAT,VTS,VOUT,VDIE,;
	ret = i2c_smbus_read_i2c_block_data(gchip->client, HL7138_REG_42, 12, data_block);		//JL:first Reg is 13,need to change to 42;
	if (ret < 0) {
		if(oplus_vooc_mg) {
			oplus_vooc_mg->voocphy_iic_err = 1;
		}
		vphy_err("hl7138_update_data error \n");
	}
	hl7138_clear_interrupts(oplus_vooc_mg);
	for (i=0; i<12; i++) {
		vphy_info("data_block[%d] = %u\n", i, data_block[i])
	}

	if (oplus_vooc_mg->adapter_type == ADAPTER_SVOOC) {
		oplus_vooc_mg->cp_ichg = ((data_block[2] << 4) | data_block[3])*HL7138_SVOOC_IBUS_FACTOR;	//Iin_lbs=1.10mA@CP;
	} else {
		oplus_vooc_mg->cp_ichg = ((data_block[2] << 4) | data_block[3])*HL7138_VOOC_IBUS_FACTOR;	//Iin_lbs=2.15mA@BP;
	}
	oplus_vooc_mg->cp_vbus = ((data_block[0] << 4) | data_block[1])*400 / 100;		//vbus_lsb=4mV;
	oplus_vooc_mg->cp_vsys = ((data_block[10] << 4) | data_block[11])*125 / 100;	//vout_lsb=1.25mV;
	vphy_info("cp_ichg = %d cp_vbus = %d, cp_vsys = %d int_flag = %d",
		oplus_vooc_mg->cp_ichg, oplus_vooc_mg->cp_vbus, oplus_vooc_mg->cp_vsys, oplus_vooc_mg->int_flag);
	oplus_vooc_mg->bq27541_vbatt = oplus_gauge_get_batt_mvolts();
}

static int hl7138_get_chg_enable(vooc_manager *chip, u8 *data)
{
	int ret = 0;
	if (!gchip) {
		vphy_err("Failed\n");
		return -1;
	}
	ret = hl7138_read_byte(gchip, HL7138_REG_12, data);	//JL:RST_REG 12 change to 14;
	if (ret < 0) {
		vphy_err("HL7138_REG_12\n");
		return -1;
	}
	*data = *data >> 7;

	return ret;
}


static int hl7138_chg_enable(vooc_manager *chip)
{
	if (oplus_chg_check_pd_svooc_adapater()) {
		return hl7138_write_byte(gchip, HL7138_REG_12, 0x81);		//is pdsvooc adapter: disable ucp
	} else {
		return hl7138_write_byte(gchip, HL7138_REG_12, 0x85);		//is not pdsvooc adapter
	}
}


int hl7132_reset_voocphy(vooc_manager *chip)
{
	//stop hardware monitor	//lkl need modify
	//status = oplus_vooc_hardware_monitor_stop();

	/*aviod exit fastchg vbus ovp drop out*/
	hl7138_write_byte(gchip, HL7138_REG_14, 0x08);

	//hwic config with plugout
	hl7138_write_byte(gchip, HL7138_REG_11, 0xDC);	//JL:Dis VBAT,IBAT reg;
	hl7138_write_byte(gchip, HL7138_REG_08, 0x38);	//JL:vbat_ovp=4.65V;00->08;(4.65-0.09)/10=54;
	hl7138_write_byte(gchip, HL7138_REG_0B, 0x82);	//JL:vac_ovp=6V;02->0B;4+val*lsb;
	//hl7138_write_byte(gchip, HL7138_REG_0C, 0x0F);		//JL:vbus_ovp=10V;04->0c;10.5/5.25V;
	hl7138_write_byte(gchip, HL7138_REG_0E, 0x32);	//JL:UCP_deb=5ms;IBUS_OCP=3.6A;05->0e;3.5A_max;
	hl7138_write_byte(gchip, HL7138_REG_40, 0x00);	//JL:Dis_ADC;11->40;
	hl7138_write_byte(gchip, HL7138_REG_02, 0xE0);	//JL:mask all INT_FLAG
	hl7138_write_byte(gchip, HL7138_REG_10, 0xEC);	//JL:Dis IIN_REG;

	//turn off mos
	hl7138_write_byte(gchip, HL7138_REG_12, 0x05);	//JL:Fsw=500KHz;07->12;

	//clear tx data
	hl7138_write_byte(gchip, HL7138_REG_38, 0x00);	//JL:2C->38;
	hl7138_write_byte(gchip, HL7138_REG_39, 0x00);	//JL:2D->39;

	//disable vooc phy irq
	hl7138_write_byte(gchip, HL7138_REG_3C, 0xff);	//JL:30->3C,VOOC_PHY FLAG ALL MASK;

	//set D+ HiZ
	//hl7138_write_byte(gchip, HL7138_REG_21, 0xc0);	//JL:No need in HL7138;

	//select big bang mode

	//disable vooc
	hl7138_write_byte(gchip, HL7138_REG_37, 0x00);	//JL:2B->37,Dis all;
	hl7138_set_predata(oplus_vooc_mg, 0);

	vphy_info ("oplus_vooc_reset_voocphy done");

	return 0;
}


int hl7138_reactive_voocphy(vooc_manager *chip)
{
	//u8 value;

	//to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us
	hl7138_set_predata(oplus_vooc_mg, 0);
	//hl7138_read_byte(gchip, HL7138_REG_3A, &value);	//JL:How to explain;
	//value = value | (3 << 5);
	//hl7138_write_byte(gchip, HL7138_REG_3A, value);

	//dpdm
	//hl7138_write_byte(gchip, HL7138_REG_21, 0x21);
	//hl7138_write_byte(gchip, HL7138_REG_22, 0x00);
	//hl7138_write_byte(gchip, HL7138_REG_33, 0xD1);

	//clear tx data
	hl7138_write_byte(gchip, HL7138_REG_38, 0x00);	//JL:2C->38,Dis all;
	hl7138_write_byte(gchip, HL7138_REG_39, 0x00);	//JL:2D->39,Dis all;

	//vooc
	hl7138_write_byte(gchip, HL7138_REG_3C, 0x85);	//JL:30->3C,JUST enable RX_START & TX_DONE;
	oplus_vooc_send_handshake_seq();

	vphy_info ("oplus_vooc_reactive_voocphy done");

	return 0;
}

int hl7138_clear_interrupts(vooc_manager *chip)
{
	int ret = 0;
	u8 val = 0;

	if (!gchip) {
		vphy_err("gchip is null\n");
		return -1;
	}

	ret = hl7138_read_byte(gchip, HL7138_REG_01, &val);
	if (ret) {
		vphy_err("clear int fail %d", ret);
		return ret;
	}
	ret = hl7138_read_byte(gchip, HL7138_REG_3B, &val);
	if (ret) {
		vphy_err("clear int fail %d", ret);
		return ret;
	}
	return 0;
}

int hl7138_get_voocphy_enable(vooc_manager * chip, unsigned char *value)
{
	return hl7138_read_byte(gchip, HL7138_REG_37, value);
}

static void hl7138_set_pd_svooc_config(vooc_manager *chip, bool enable)
{
	int ret = 0;
	u8 reg = 0;
	if (!chip) {
		vphy_err("Failed\n");
		return;
	}

	if (enable)
		hl7138_write_byte(gchip, HL7138_REG_12, 0x01);		//disable ucp
	else
		hl7138_write_byte(gchip, HL7138_REG_12, 0x85);		//enable ucp

	ret = hl7138_read_byte(gchip, HL7138_REG_12, &reg);
	if (ret < 0) {
		vphy_err("HL7138_REG_12\n");
		return;
	}
	vphy_dbg("pd_svooc config HL7138_REG_12 = %d\n", reg);
}

static bool hl7138_get_pd_svooc_config(vooc_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		vphy_err("Failed\n");
		return false;
	}

	ret = hl7138_read_byte(gchip, HL7138_REG_12, &data);
	if (ret < 0) {
		vphy_err("HL7138_REG_12\n");
		return false;
	}

	vphy_dbg("HL7138_REG_12 = 0x%0x\n", data);

	return !(data & (1 << 2));
}

#define REGS_INFO_MAX	64
static int regsinfo[REGS_INFO_MAX] = {
	0X01,
	0X03,
	0X04,
	0X05,
	0X06,
	0X07,
	0x12,
	0X15,
	0X37,
	0X3B,
	0X3C,
	0X3D,
	0X3E,
	0X3F,
	0X42,
	0X43,
	0x44,
	0x45,
	0X46,
	0X47,
	0X48,
	0X49,
	0X4C,
	0X4D,
	0xff
};

static void hl7138_dump_regs(void)
{
	int i = 0, p = 0;
	struct oplus_chg_chip *chip = oplus_get_oplus_chip();

	if(!chip || !gchip) {
		vphy_err( "!!!!! oplus_chg_chip or chip_hl7138 chip is NULL");
		return;
	}

	//printk(KERN_ERR "\n[%s] ", __func__);
	for( i = 0; i < REGS_INFO_MAX; i++) {
		if (regsinfo[i] == 0xff) {
			vphy_err( " p[%d] regsinfo is get done\n", p);
			break;
		}
		hl7138_read_byte(gchip, regsinfo[i], &chip->voocphy.reg_dump[p]);
		//printk(KERN_ERR "0x%0x:[0x%0x], ", regsinfo[i], chip->voocphy.reg_dump[p]);
		p = p + 1;
	}
	return;
}

struct oplus_voocphy_operations hl7138_ops = {
	.hw_setting = hl7138_hw_setting,
	.init_vooc = hl7138_init_vooc,
	.set_predata = hl7138_set_predata,
	.set_txbuff = hl7138_set_txbuff,
	.get_adapter_info = hl7138_get_adapter_info,
	.update_data = hl7138_update_data,
	.get_chg_enable = hl7138_get_chg_enable,
	.set_chg_enable = hl7138_chg_enable,
	.reset_voocphy = hl7132_reset_voocphy,
	.reactive_voocphy = hl7138_reactive_voocphy,
	//void (*set_switch_mode)(vooc_manager *chip, int mode);
	.send_handshake = hl7138_send_handshake_seq,
	.clear_intertupts = hl7138_clear_interrupts,
	.get_voocphy_enable = hl7138_get_voocphy_enable,
	.print_cp_int_exception = hl7138_print_cp_int_exception,
	.set_pd_svooc_config = hl7138_set_pd_svooc_config,
	.get_pd_svooc_config = hl7138_get_pd_svooc_config,
	.dump_regs = hl7138_dump_regs,
};

extern ktime_t calltime, rettime;
static irqreturn_t hl7138_charger_interrupt(int irq, void *dev_id)
	{
		struct chip_hl7138 *chip = dev_id;
		BattMngr_err_code_e status = BATTMNGR_SUCCESS;

		//start ti);me
		calltime = ktime_get();

		if (!chip)
			return IRQ_HANDLED;

		oplus_vooc_mg->irq_total_num++;

		//for flash led
		if (oplus_vooc_mg->fastchg_need_reset) {
			vphy_info("fastchg_need_reset and clr interruts\n");
			oplus_vooc_mg->fastchg_need_reset = 0;
			oplus_vooc_set_status_and_notify_ap(FAST_NOTIFY_USER_EXIT_FASTCHG);
			hl7138_clear_interrupts(oplus_vooc_mg);
			goto handle_done;
		}

		oplus_voocphy_get_adapter_request_info();
		if (VOOC_CMD_ASK_CURRENT_LEVEL == oplus_vooc_mg->pre_adapter_ask_cmd && ADJ_CUR_STEP_SET_CURR_DONE == oplus_vooc_mg->adjust_curr) {
			if (oplus_write_txbuff_error()) {
				oplus_vooc_mg->ap_need_change_current = 5;
				oplus_vooc_mg->adjust_fail_cnt++;
				vphy_err("adjust curr wtxbuff fail %d times\n", oplus_vooc_mg->adjust_fail_cnt);
			} else {
				cpuboost_charge_event(CPU_CHG_FREQ_STAT_AUTO);
				//trace_oplus_tp_sched_change_ux(1, task_cpu(current));
				//trace_oplus_tp_sched_change_ux(0, task_cpu(current));
				vphy_err("adjust cpu to default");
				oplus_vooc_mg->adjust_fail_cnt = 0;
				oplus_vooc_mg->adjust_curr = ADJ_CUR_STEP_DEFAULT;
			}
		}

		/*avoiding PBS01 & PBV01 chg break*/
		if ((chip->vooc_flag & ERR_TRANS_DET_FLAG_MASK) && (chip->vooc_flag & RX_START_FLAG_MASK)) {
			oplus_vooc_mg->err_trans_cnt++;
			if (oplus_vooc_mg->err_trans_cnt == 10) {
				vphy_err("err_trans_cnt 10 times hl7138_enable_t5t6_check false\n");
				hl7138_enable_t5t6_check(false);
				oplus_vooc_mg->err_trans_flg = true;
			}
		} else {
			oplus_vooc_mg->err_trans_cnt = 0;
		}

		if (chip->vooc_flag == 0xF) {
			//do nothing
			hl7138_clear_interrupts(oplus_vooc_mg);
			vphy_err("!RX_START_FLAG & TXDATA_WR_FAIL_FLAG occured do nothing");
		} else if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {	//rxdata recv done
			//feed soft monitor watchdog
			if (!oplus_vooc_mg->fastchg_commu_stop) {
				status = vooc_monitor_timer_stop(VOOC_THREAD_TIMER_COMMU, VOOC_COMMU_EVENT_TIME);
				if (status != BATTMNGR_SUCCESS) {
					vphy_err("stop commu timer fail");
				}
				status = vooc_monitor_timer_start(VOOC_THREAD_TIMER_COMMU,VOOC_COMMU_EVENT_TIME);
				if (status != BATTMNGR_SUCCESS) {
					vphy_err("restart commu timer fail");
				}
			}

			oplus_vooc_mg->irq_rcvok_num++;
			oplus_vooc_mg->voocphy_rx_buff = chip->vooc_rxdata;
			oplus_vooc_adapter_commu_with_voocphy(0);
			if (oplus_vooc_ap_allow_fastchg()) {
				if (oplus_vooc_mg->fastchg_stage == OPLUS_FASTCHG_STAGE_2 && oplus_chg_get_flash_led_status()) {
					//oplus_vooc_set_status_and_notify_ap(FAST_NOTIFY_USER_EXIT_FASTCHG);
					oplus_vooc_mg->fastchg_need_reset = 1;
					hl7138_clear_interrupts(oplus_vooc_mg);
					vphy_info("OPLUS_FASTCHG_STAGE_2 and open torch exit fastchg\n");
					goto handle_done;
				} else {
					oplus_chg_disable_charge();
					vphy_err("allow fastchg adapter type %d\n", oplus_vooc_mg->adapter_type);
				}

				//handle timeout of adapter ask cmd 0x4
				oplus_is_vbus_ok_predata();

				if (oplus_vooc_mg->adapter_type == ADAPTER_SVOOC) {
					oplus_voocphy_hw_setting(SETTING_REASON_SVOOC);
					if(oplus_chg_check_pd_svooc_adapater()) {
						vphy_err("pd_svooc adapter, set config!\n");
						oplus_voocphy_set_pdsvooc_adapter_config(oplus_vooc_mg, true);
					}
				} else if (oplus_vooc_mg->adapter_type == ADAPTER_VOOC20 || oplus_vooc_mg->adapter_type == ADAPTER_VOOC30){
					oplus_voocphy_hw_setting(SETTING_REASON_VOOC);
				} else {
					vphy_err("SETTING_REASON_DEFAULT\n");
				}
			}
			oplus_vooc_mg->pre_adapter_ask_cmd = oplus_vooc_mg->fastchg_adapter_ask_cmd;
			oplus_voocphy_update_chg_data();
		}else {
			hl7138_clear_interrupts(oplus_vooc_mg);
		}

		if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {
			oplus_vooc_mg->irq_hw_timeout_num++;
		}

		if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {
			/*move to hl7138_update_chg_data for improving time of interrupt*/
		} else {
			hl7138_get_int_flag(oplus_vooc_mg);
		}
		vphy_info("adapter_ask = 0x%0x, 0x%0x int_flag 0x%0x 0x%0x\n",
			oplus_vooc_mg->pre_adapter_ask_cmd, oplus_vooc_mg->fastchg_adapter_ask_cmd,
			oplus_vooc_mg->int_flag, oplus_vooc_mg->vooc_flag);

		oplus_vooc_wake_voocphy_service_work(VOOCPHY_REQUEST_UPDATE_DATA);

handle_done:
		return IRQ_HANDLED;
	}

static int hl7138_irq_register(struct chip_hl7138 *chip)
{
	int ret;

	oplus_voocphy_irq_gpio_init();
	if (oplus_vooc_mg->irq) {
        ret = request_threaded_irq(oplus_vooc_mg->irq, NULL,
                                   hl7138_charger_interrupt,
                                   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                   "hl7138_charger_irq", chip);
		if (ret < 0) {
			vphy_dbg("request irq for irq=%d failed, ret =%d\n",
							oplus_vooc_mg->irq, ret);
			return ret;
		}
		enable_irq_wake(oplus_vooc_mg->irq);
	}
	vphy_dbg("request irq ok\n");

	return ret;
}

static void hl7138_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}

static struct of_device_id hl7138_charger_match_table[] = {
	{
		.compatible = "chip,hl7138-standalone",
	},
	{},
};

static int hl7138_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct chip_hl7138 *chip;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret;

	vphy_err("hl7138_charger_probe enter!\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct chip_hl7138), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	gchip = chip;

	chip->dev = &client->dev;
	chip->client = client;

	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->data_lock);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->irq_complete);
	//wakeup_source_init(&chip->ws, "voocphy_wlock");	//lkl porting

	chip->resume_completed = true;
	chip->irq_waiting = false;

	i2c_set_clientdata(client, chip);
	hl7138_create_device_node(&(client->dev));

	match = of_match_node(hl7138_charger_match_table, node);
	if (match == NULL) {
		vphy_dbg("device tree match not found!\n");
		//return -ENODEV;
	}

	//check commu of iic to identificate chip id
	ret = hl7138_reg_reset(chip, true);
	if (ret) {
		goto hl7138_probe_err;
	}
	hl7138_work_mode_lockcheck(chip);
	hl7138_init_device(chip);

	//request oplus_vooc_mg memery
	oplus_vooc_init_res(0);
	if (!oplus_vooc_mg) {
		ret = -ENOMEM;
		goto hl7138_probe_err;
	}
	oplus_vooc_mg->chrpmp_id = CHGPMP_ID_HL7138;
	oplus_vooc_mg->vops = &hl7138_ops;
	oplus_vooc_mg->dev = &client->dev;
	oplus_vooc_mg->client = client;
	oplus_vooc_chg_sw_ctrl2_parse_dt();

	ret = hl7138_irq_register(chip);
	if (ret < 0)
		goto hl7138_probe_err;

	init_proc_voocphy_debug();

	vphy_err("hl7138_charger_probe succesfull\n");
	return 0;

hl7138_probe_err:
	vphy_err("hl7138_charger_probe failed\n");
	return ret;
}

static void hl7138_charger_shutdown(struct i2c_client *client)
{
	struct oplus_chg_chip* chip = oplus_get_oplus_chip();

	if (chip) {
		chip->chg_ops->set_chargerid_switch_val(0);
		opchg_voocphy_set_switch_normal_charger();
		vphy_err("hl7138_charger_shutdown\n");
	}

	if (gchip) {
		hl7138_write_byte(gchip, HL7138_REG_40, 0x00);	//disable
		hl7138_reg_reset(gchip, true);
		hl7138_hw_reset();
	}
	vphy_err("hl7138_charger_shutdown end\n");

	return;
}

static const struct i2c_device_id hl7138_charger_id[] = {
	{"hl7138-standalone", 0},
	{},
};

static struct i2c_driver hl7138_charger_driver = {
	.driver		= {
		.name	= "hl7138-charger",
		.owner	= THIS_MODULE,
		.of_match_table = hl7138_charger_match_table,
	},
	.id_table	= hl7138_charger_id,

	.probe		= hl7138_charger_probe,
	.shutdown	= hl7138_charger_shutdown,
};

module_i2c_driver(hl7138_charger_driver);

MODULE_DESCRIPTION("Hl7138 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("luakili@oppo.com");


