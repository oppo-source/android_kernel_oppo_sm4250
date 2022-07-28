/***************************************************
 * File:touch.c
 * OPLUS_FEATURE_TP_BASIC
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/regulator/consumer.h>
#include "oplus_touchscreen/tp_devices.h"
#include "oplus_touchscreen/touchpanel_common.h"
#include <soc/oplus/oplus_project.h>
#include <soc/oplus/device_info.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100
extern char *saved_command_line;
/*if can not compile success, please update vendor/oppo_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
     {TP_OFILM, "OFILM"},
     {TP_BIEL, "BIEL"},
     {TP_TRULY, "TRULY"},
     {TP_BOE, "BOE"},
     {TP_G2Y, "G2Y"},
     {TP_TPK, "TPK"},
     {TP_JDI, "JDI"},
     {TP_TIANMA, "TIANMA"},
     {TP_SAMSUNG, "SAMSUNG"},
     {TP_DSJM, "DSJM"},
     {TP_BOE_B8, "BOEB8"},
     {TP_UNKNOWN, "UNKNOWN"},
};
int g_tp_prj_id = 0;
int g_tp_dev_vendor = TP_UNKNOWN;
char *g_tp_ext_prj_name = NULL;

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

bool __init tp_judge_ic_match(char *tp_ic_name)
{
    return true;
}

bool  tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
    int prj_id = 0;
    int i = 0;
    prj_id = get_project();
    pr_err("[TP] boot_command_line = %s \n", saved_command_line);
    for(i = 0; i<panel_data->project_num; i++){
        if(prj_id == panel_data->platform_support_project[i]){
            g_tp_prj_id = panel_data->platform_support_project_dir[i];
            g_tp_ext_prj_name = panel_data->platform_support_external_name[i];
            if(strstr(saved_command_line, panel_data->platform_support_commandline[i])||strstr("default_commandline", panel_data->platform_support_commandline[i]) ){
                pr_err("[TP] Driver match the project\n");
                return true;
            }
        }
    }
    pr_err("[TP] Driver does not match the project\n");
    pr_err("Lcd module not found\n");
    return false;
}


int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    char* vendor;
    int prj_id = 0;

    panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
    if (panel_data->test_limit_name == NULL) {
        pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
    }

    prj_id = g_tp_prj_id;

    memcpy(panel_data->manufacture_info.version, "0x", 2);

    if (g_tp_ext_prj_name) {
        strncpy(panel_data->manufacture_info.version + strlen(panel_data->manufacture_info.version),
                g_tp_ext_prj_name, 7);
    }
    if (panel_data->tp_type == TP_UNKNOWN) {
        pr_err("[TP]%s type is unknown\n", __func__);
        return 0;
    }

    vendor = GET_TP_DEV_NAME(panel_data->tp_type);

    strcpy(panel_data->manufacture_info.manufacture, vendor);
    snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s.img",
            prj_id, panel_data->chip_name, vendor);

    if (panel_data->test_limit_name) {
        snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
            "tp/%d/LIMIT_%s_%s.img",
            prj_id, panel_data->chip_name, vendor);
    }

        if ((prj_id == 20221) || (prj_id == 20222) || (prj_id == 20223) || (prj_id == 20224) || (prj_id == 20225) || (prj_id == 20226)
	|| (prj_id == 20227) || (prj_id == 20228) || (prj_id == 20229) || (prj_id == 20021) || (prj_id == 20202) || (prj_id == 20203)
	|| (prj_id == 20204) || (prj_id == 20207) || (prj_id == 20208) || (prj_id == 20025)) {
                pr_err("[TP]project is soda\n");
                pr_err("[TP] saved_command_line = [%s] \n", saved_command_line);
                if (strstr(saved_command_line, "mdss_dsi_ili9881h_90hz_boe_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_ILI9881H_90HZ_BOE.bin");
                        pr_err("[TP]This is ILI9881H_90HZ_BOE\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_ILI9881H_90HZ_BOE.ini");
                        }
                        panel_data->vid_len = 8;
                        memcpy(panel_data->manufacture_info.version, "FA218BI9", 8);
                        memcpy(panel_data->manufacture_info.manufacture, "90HZ_BOE", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20221_ILI9881H_90HZ_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_ILI9881H_90HZ_BOE);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_90hz_hlt_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_ILI9881H_90HZ_HLT.bin");
                        pr_err("[TP]This is ILI9881H_90HZ_HLT\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_ILI9881H_90HZ_HLT.ini");
                        }
                        panel_data->vid_len = 8;
                        memcpy(panel_data->manufacture_info.version, "FA218HI9", 8);
                        memcpy(panel_data->manufacture_info.manufacture, "90HZ_HLT", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20221_ILI9881H_90HZ_HLT;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_ILI9881H_90HZ_HLT);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9882n_90hz_txd_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_ILI9882N_90HZ_TXD.bin");
                        pr_err("[TP]This is ILI9882N_TXD\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_ILI9882N_90HZ_TXD.ini");
                        }
                        panel_data->vid_len = 9;
                        memcpy(panel_data->manufacture_info.version, "FA218TI9N", 9);
                        memcpy(panel_data->manufacture_info.manufacture, "90HZ_TXD", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20221_ILI9882N_TXD;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_ILI9882N_TXD);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9882n_90hz_boe_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_ILI9882N_90HZ_BOE.bin");
                        pr_err("[TP]This is ILI9882N_BOE\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_ILI9882N_90HZ_BOE.ini");
                        }
                        panel_data->vid_len = 9;
                        memcpy(panel_data->manufacture_info.version, "FA218BI9N", 9);
                        memcpy(panel_data->manufacture_info.manufacture, "90HZ_BOE", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20221_ILI9882N_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_ILI9882N_BOE);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9882n_90hz_inx_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_ILI9882N_90HZ_INX.bin");
                        pr_err("[TP]This is ILI9882N_INX\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_ILI9882N_90HZ_INX.ini");
                        }
                        panel_data->vid_len = 9;
                        memcpy(panel_data->manufacture_info.version, "FA218II9N", 9);
                        memcpy(panel_data->manufacture_info.manufacture, "90HZ_INX", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20221_ILI9882N_INX;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_ILI9882N_INX);
                } else if (strstr(saved_command_line, "mdss_dsi_nt36525b_90hz_inx_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20221/FW_NF_NT36525B_90HZ_INNOLUX.bin");
                        pr_err("[TP]This is NT36525B_90HZ_INNOLUX\n");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20221/LIMIT_NF_NT36525B_90HZ_INNOLUX.img");
                        }
                        panel_data->vid_len = 8;
                        memcpy(panel_data->manufacture_info.version, "FA218IN9", 8);
                        memcpy(panel_data->manufacture_info.manufacture, "INNOLUX", 7);
                        panel_data->firmware_headfile.firmware_data = FW_20221_NT36525B_90HZ_INNOLUX;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20221_NT36525B_90HZ_INNOLUX);
                }
        }

	if ((prj_id == 20211) || (prj_id == 20212) || (prj_id == 20213) || (prj_id == 20214) || (prj_id == 20215)) {
		pr_err("[TP]project is rum\n");
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20211/FW_GT9886_SAMSUNG.bin");
		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20211/LIMIT_GT9886_SAMSUNG.img");
		}
		panel_data->vid_len = 7;
		memcpy(panel_data->manufacture_info.version, "FA218ON", 7);
	}

        if ((prj_id == 20673) || (prj_id == 20674) || (prj_id == 20675) || (prj_id == 20677) || (prj_id == 0x2067D) || (prj_id == 0x2067E)) {
                printk("project is coco\n");
                if (strstr(saved_command_line, "mdss_dsi_hx83112a_tm_90hz_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20673/FW_NF_HX83112A_TIANMA.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20673/LIMIT_NF_HX83112A_TIANMA.img");
                        }
                        panel_data->vid_len = 15;
                        memcpy(panel_data->manufacture_info.version, "RA378_TM_Himax_", 15);
                        panel_data->firmware_headfile.firmware_data = FW_20671_Hx83221A_TIANMA;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20671_Hx83221A_TIANMA);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_boe_90hz_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20673/FW_NF_ILI9881H_90HZ_BOE.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20673/LIMIT_NF_ILI9881H_90HZ_BOE.ini");
                        }
                        panel_data->vid_len = 14;
                        memcpy(panel_data->manufacture_info.version, "RA378_BOE_ILI_", 14);
                        panel_data->firmware_headfile.firmware_data = FW_20673_ILI9881H_90HZ_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20673_ILI9881H_90HZ_BOE);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_hlt_90hz_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20673/FW_NF_ILI9881H_90HZ_HLT.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20673/LIMIT_NF_ILI9881H_90HZ_HLT.ini");
                        }
                        panel_data->vid_len = 14;
                        memcpy(panel_data->manufacture_info.version, "RA378_HLT_ILI_", 14);
                        panel_data->firmware_headfile.firmware_data = FW_20673_ILI9881H_90HZ_HLT;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20673_ILI9881H_90HZ_HLT);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_boe_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20673/FW_NF_ILI9881H_BOE.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20673/LIMIT_NF_ILI9881H_BOE.ini");
                        }
                        memcpy(panel_data->manufacture_info.version, "RA378BI_", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20673_ILI9881H_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20673_ILI9881H_BOE);
                }
        }
        if ((prj_id == 20670) || (prj_id == 20671) || (prj_id == 20672) || (prj_id == 20676) || (prj_id == 20679) || (prj_id == 0x2067C)) {
                printk("project is coco-b\n");
                if (strstr(saved_command_line, "mdss_dsi_ili9881h_boe_90hz_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20671/FW_B_NF_ILI9881H_90HZ_BOE.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20671/LIMIT_B_NF_ILI9881H_90HZ_BOE.ini");
                        }
                        panel_data->vid_len = 16;
                        memcpy(panel_data->manufacture_info.version, "RA378_B_BOE_ILI_", 16);
                        panel_data->firmware_headfile.firmware_data = FW_B_20671_ILI9881H_90HZ_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_B_20671_ILI9881H_90HZ_BOE);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_hlt_90hz_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20671/FW_B_NF_ILI9881H_90HZ_HLT.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20671/LIMIT_B_NF_ILI9881H_90HZ_HLT.ini");
                        }
                        panel_data->vid_len = 16;
                        memcpy(panel_data->manufacture_info.version, "RA378_B_HLT_ILI_", 16);
                        panel_data->firmware_headfile.firmware_data = FW_B_20671_ILI9881H_90HZ_HLT;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_B_20671_ILI9881H_90HZ_HLT);
                } else if (strstr(saved_command_line, "mdss_dsi_ili9881h_boe_video")) {
                        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH, "tp/20671/FW_B_NF_ILI9881H_BOE.bin");
                        if (panel_data->test_limit_name) {
                                snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH, "tp/20671/LIMIT_B_NF_ILI9881H_BOE.ini");
                        }
                        memcpy(panel_data->manufacture_info.version, "RA378BI_", 8);
                        panel_data->firmware_headfile.firmware_data = FW_20673_ILI9881H_BOE;
                        panel_data->firmware_headfile.firmware_size = sizeof(FW_20673_ILI9881H_BOE);
                }
        }

    panel_data->manufacture_info.fw_path = panel_data->fw_name;

    pr_info("[TP]vendor:%s fw:%s limit:%s\n",
        vendor,
        panel_data->fw_name,
        panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
    return 0;
}

int preconfig_power_control(struct touchpanel_data *ts)
{/*
    int pcb_verison = -1;
  
    pcb_verison = get_PCB_Version();
    if (is_project(OPPO_17107) || is_project(OPPO_17127)) {
        if ((is_project(OPPO_17107) && ((pcb_verison < 4) || (pcb_verison == 5))) || (is_project(OPPO_17127) && (pcb_verison == 1))) {
        } else {
            pr_err("[TP]set voltage to 3.0v\n");
            ts->hw_res.vdd_volt = 3000000;
        }
    }
   */
    return 0;
}
EXPORT_SYMBOL(preconfig_power_control);

int reconfig_power_control(struct touchpanel_data *ts)
{
/*
    int pcb_verison = -1;

    pcb_verison = get_PCB_Version();
    if (is_project(OPPO_17107) || is_project(OPPO_17127)) {
        if ((is_project(OPPO_17107) && ((pcb_verison < 4) || (pcb_verison == 5))) || (is_project(OPPO_17127) && (pcb_verison == 1))) {
            pr_err("[TP]2v8 gpio free\n");
            if (gpio_is_valid(ts->hw_res.enable2v8_gpio)) {
                gpio_free(ts->hw_res.enable2v8_gpio);
                ts->hw_res.enable2v8_gpio = -1;
            }
        } else {
            pr_err("[TP]2v8 regulator put\n");
            if (!IS_ERR_OR_NULL(ts->hw_res.vdd_2v8)) {
                regulator_put(ts->hw_res.vdd_2v8);
                ts->hw_res.vdd_2v8 = NULL;
            }
        }
    }
*/
    return 0;
}
EXPORT_SYMBOL(reconfig_power_control);

