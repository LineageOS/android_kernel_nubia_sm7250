/*
 * Touchkey driver for CYPRESS4000 controller
 *
 * Copyright (C) 2018 nubia
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
//#include <linux/i2c/mcs.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include "cy8c4014l_touchkey.h"
//#include "cy8c4014l_touchkey_StringImage.h" //nubia, disable hard code
#include "cy8c4014l_touchkey_firmware_update.h"

#define NUBIA_FIRM_VERSION "nubia_tk_ver.cyacd"
#define NUBIA_FIRM_NAME "nubia_tk_fw.cyacd"
#define NUBIA_FIRM_LINE_LEN 143    //char and \r,\n
#define NUBIA_TIMER_SIGNAL_BASE 10000    //signal print base int number

static int cypress_get_mode(struct i2c_client *client) {
	return i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_MODE);
}

static int cypress_set_mode(struct i2c_client *client, int mode) {
	int ret = -1;
	int cur_mode = 0;
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;

	cur_mode = cypress_get_mode(client);
	if(cur_mode == mode){
        pr_err("[%s] mode already is 0x%2x now!!\n", __func__, mode);
		return mode;
	}
	if(mode == CYPRESS4000_SLEEP_MODE) {
		if(pdata->bLeftDown) {
			if(timer_pending(&pdata->LTimer)) {
				del_timer(&pdata->LTimer);
			}
			input_report_key(pdata->input_dev, KEY_F7, 0);
			input_sync(pdata->input_dev);
			pdata->bLeftDown = false;
			pr_err("[%s] Send left up\n", __func__);
		}
		if(pdata->bRightDown) {
			if(timer_pending(&pdata->RTimer)) {
				del_timer(&pdata->RTimer);
			}
			input_report_key(pdata->input_dev, KEY_F8, 0);
			input_sync(pdata->input_dev);
			pdata->bRightDown = false;
			pr_err("[%s] Send right up\n", __func__);
		}
	}
	ret = i2c_smbus_write_byte_data(client,CYPRESS4000_TOUCHKEY_MODE, mode);
	if(ret<0) {
		pr_err("[%s] Set mode=%d error!ret=0x%x\n", __func__, mode, ret);
		return mode;
	}
	pr_err("[%s] set touchkey to %s mode!!\n", __func__, (mode == CYPRESS4000_SLEEP_MODE)?"sleep":"wake");
	return mode;
}

static ssize_t touchkey_command_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    if (sscanf(buf, "%u", &input) != 1)
    {
        dev_info(dev, "Failed to get input message!\n");
        return -EINVAL;
    }
    /* 0x1 enter read cp mode
        0x2 soft reset IC ,other command are not supported for now*/
    if(input == 0x1 || input == 0x2)
    {
        ret = i2c_smbus_write_byte_data(client,CYPRESS4000_TOUCHKEY_CMD, input);
        if(ret<0)
        {
            dev_err(&client->dev, "Failed to set command 0x%x!\n",input);
        }
    }
    else
    {
        dev_info(dev, "Command 0x%x not support!\n",input);
    }
    return count;
}
static ssize_t touchkey_command_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int cmd = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    cmd = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_CMD);
    if(cmd < 0)
    {
        dev_info(dev, "Failed to get command state(0x%x)!\n",cmd);
        return scnprintf(buf,CYPRESS_BUFF_LENGTH, "cmd: error\n");
    }
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "cmd: 0x%x\n", cmd);
}

static ssize_t touchkey_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int mode = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
	if(!pdata->bUpdateOver) {
		mode = pdata->new_mode;
        dev_err(dev, "firmware updating, get touchkey mode=%s!\n", (mode == CYPRESS4000_SLEEP_MODE)?"sleep":"wake");
	} else {
	    mode = cypress_get_mode(client);
	}
    //dev_info(dev, "touchkey_mode_show [0x%x]\n", mode);
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "mode: %s(0x%x)\n",
        (mode == CYPRESS4000_WAKEUP_MODE)?"wakeup":((mode == CYPRESS4000_SLEEP_MODE)?"sleep":"unknown"),
        mode);
}

static ssize_t touchkey_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int mode = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;

    if(buf[0] == '0'){
        mode = CYPRESS4000_SLEEP_MODE;
    }else if(buf[0] == '1'){
        mode = CYPRESS4000_WAKEUP_MODE;
    }else{
        dev_err(dev, "mode set failed, 0: sleep, 1: wakeup, %c unknown\n", buf[0]);
        return count;
    }
	pdata->new_mode = mode;
	if(!pdata->bUpdateOver) {
        dev_err(dev, "firmware updating, set touchkey to %s mode delayed!\n", (mode == CYPRESS4000_SLEEP_MODE)?"sleep":"wake");
        return count;
	}
    ret = cypress_set_mode(client, mode);
    return count;
}

static ssize_t touchkey_firmversion_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    int version = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    version = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_FW);
	//dev_info(dev, "touchkey_firmversion_show [0x%x]\n", version);
	return scnprintf(buf,CYPRESS_BUFF_LENGTH, "firmware version: 0x%x\n", version);
}

static ssize_t touchkey_firmversion_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int version = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    dev_err(&client->dev, "buf: %s \n", buf);

	version = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_FW);
/*
	if ((buf[0] == 'f') || (version != CURRENT_NEW_FIRMWARE_VER))
	{
		int ret; //0:success
		disable_irq(client->irq);
		dev_info(&client->dev, "Ready to update firmware\n");
		ret = cypress_firmware_update(client,stringImage_0, LINE_CNT_0);

		if (ret < 0)
			dev_err(&client->dev, "cypress Firmware update fail,cur ver is :0x%x,ret=%x\n", version,ret);
		else
			dev_err(&client->dev, "cypress Firmware update success, cur ver is :0x%x\n", version);

		enable_irq(client->irq);
	}else{
		dev_err(&client->dev,"cypress Firmware version(0x%x) is newest!!", version);
	}
*/
	return count;
}

static ssize_t touchkey_reglist_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u8 regval = 0;
    u8 regaddr = CYPRESS4000_KEY0_RAWDATA0;
    int size = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    for(regaddr = CYPRESS4000_KEY0_RAWDATA0;regaddr <= CYPRESS4000_KEY1_CP3;regaddr++)
    {
        regval = i2c_smbus_read_byte_data(client, regaddr);
        size += scnprintf(buf + size , CYPRESS_BUFF_LENGTH - size, "Reg[0x%x] :%d\n",regaddr,regval);
        //dev_info(dev, "size=%d,reg[0x%x]=%d\n", size,regaddr,regval);
    }
    return size;
}
static ssize_t touchkey_signal_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u8 raw0 = 0;
    u8 raw1 = 0;
    u8 base0 = 0;
    u8 base1 = 0;
    int size = 0;

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    raw0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_RAWDATA0);
    raw1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_RAWDATA1);
    base0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_BASELINE0);
    base1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_BASELINE1);
    size += scnprintf(buf + size, CYPRESS_BUFF_LENGTH - size,
        "[Key0 signal %d]:%d %d %d %d\n",
        (((raw0 << 8) | raw1) - ((base0 << 8) | base1)),raw0,raw1,base0,base1);

    raw0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_RAWDATA0);
    raw1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_RAWDATA1);
    base0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_BASELINE0);
    base1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_BASELINE1);
    size += scnprintf(buf + size, CYPRESS_BUFF_LENGTH - size,
        "[Key1 signal %d]:%d %d %d %d\n",
        (((raw0 << 8) | raw1) - ((base0 << 8) | base1)),raw0,raw1,base0,base1);

	return size;
}

static ssize_t touchkey_Lsensitivity_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    int level = 0;
    int r_level = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    r_level = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_LSENSITIVITY);
    dev_err(&client->dev, "%s:r_level: %x \n", __func__,r_level);
    level = (r_level > 6)?(r_level - 6):r_level;
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "Left sensitivity level: %d\n", level);
}

static ssize_t touchkey_Rsensitivity_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    int level = 0;
    int r_level = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    r_level = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_RSENSITIVITY);
    dev_err(&client->dev, "%s:r_level: %x \n", __func__,r_level);
    level = (r_level > 6)?(r_level - 6):r_level;
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "Right sensitivity level: %d\n", level);
}

static int touchkey_sensitivity_isvalid(const char *buf)
{
    int level;

    if(buf[0] > '0' && buf[0] <= '3'){
        level = buf[0] - '0';
    }else{
        level = -1;
    }

    return level;
}

static ssize_t touchkey_Lsensitivity_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int level;
    int new_level;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    level = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_LSENSITIVITY);

    dev_err(&client->dev, "buf: %s \n", buf);
    new_level = touchkey_sensitivity_isvalid(buf);

    if(new_level == level || new_level < 0)
    {
        return count;
    }

    ret = i2c_smbus_write_byte_data(client,CYPRESS4000_TOUCHKEY_LSENSITIVITY, new_level);
    if(ret < 0){
        dev_err(&client->dev, "Set %d Lsensitivity error!\n", new_level);
    }

    dev_err(&client->dev, "Set Lsensitivity level: %d OK\n", new_level);
    return count;
}

static ssize_t touchkey_Rsensitivity_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int level;
    int new_level;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    level = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_RSENSITIVITY);

    dev_err(&client->dev, "buf: %s \n", buf);
    new_level = touchkey_sensitivity_isvalid(buf);

    if(new_level == level || new_level < 0)
    {
        return count;
    }

    ret = i2c_smbus_write_byte_data(client,CYPRESS4000_TOUCHKEY_RSENSITIVITY, new_level);
    if(ret < 0){
        dev_err(&client->dev, "Set %d Rsensitivity error!\n", new_level);
    }

    dev_err(&client->dev, "Set Rsensitivity level: %d OK\n", new_level);
    return count;
}

static ssize_t touchkey_LKeyInt_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "%d\n", pdata->LKeyInt);
}

static ssize_t touchkey_RKeyInt_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
    return scnprintf(buf,CYPRESS_BUFF_LENGTH, "%d\n", pdata->RKeyInt);
}

static ssize_t touchkey_LKeyInt_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
	pdata->LKeyInt = simple_strtol(buf, NULL, 10);
	pr_info("%s:LKeyInt=%d\n", __func__, pdata->LKeyInt);
    return count;
}

static ssize_t touchkey_RKeyInt_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
	pdata->RKeyInt = simple_strtol(buf, NULL, 10);
	pr_info("%s:RKeyInt=%d\n", __func__, pdata->RKeyInt);
    return count;
}

static void cypress_LKeyTimer_cb(struct timer_list *t) {
	struct cypress_platform_data *pdata = from_timer(pdata, t, LTimer);
    struct input_dev *input = pdata->input_dev;
	if(pdata->bLeftDown) {
		pr_info("%s: LI=%d\n", __func__, pdata->LKeyInt);
        input_report_key(input, KEY_F7, 1);
        input_sync(input);
        input_report_key(input, KEY_F7, 0);
        input_sync(input);
		mod_timer(&pdata->LTimer, jiffies+msecs_to_jiffies(pdata->LKeyInt));
	}
}

static void cypress_RKeyTimer_cb(struct timer_list *t) {
	struct cypress_platform_data *pdata = from_timer(pdata, t, RTimer);
    struct input_dev *input = pdata->input_dev;
	if(pdata->RKeyInt > NUBIA_TIMER_SIGNAL_BASE) {
		if(pdata->bRightDown || pdata->bLeftDown) {
			queue_delayed_work(pdata->cypress_update_wq, &pdata->cypress_update_work, 0);
			mod_timer(&pdata->RTimer, jiffies+msecs_to_jiffies(pdata->RKeyInt - NUBIA_TIMER_SIGNAL_BASE));
		}
	} else {
		if(pdata->bRightDown) {
			pr_info("%s: RI=%d\n", __func__, pdata->RKeyInt);
    	    input_report_key(input, KEY_F8, 1);
        	input_sync(input);
	        input_report_key(input, KEY_F8, 0);
    	    input_sync(input);
			mod_timer(&pdata->RTimer, jiffies+msecs_to_jiffies(pdata->RKeyInt));
		}
	}
}

static ssize_t touchkey_signal_print(struct device *dev, char *buf) {
    u8 raw0 = 0;
    u8 raw1 = 0;
    u8 base0 = 0;
    u8 base1 = 0;
    int size = 0;

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    raw0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_RAWDATA0);
    raw1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_RAWDATA1);
    base0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_BASELINE0);
    base1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY1_BASELINE1);
    size += scnprintf(buf + size, CYPRESS_BUFF_LENGTH - size,
        "[Key0 signal %d-%d ] ",
        ((raw0 << 8) | raw1), ((base0 << 8) | base1));

    raw0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_RAWDATA0);
    raw1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_RAWDATA1);
    base0 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_BASELINE0);
    base1 = i2c_smbus_read_byte_data(client, CYPRESS4000_KEY0_BASELINE1);
    size += scnprintf(buf + size, CYPRESS_BUFF_LENGTH - size,
        "[Key1 signal %d-%d ]",
        ((raw0 << 8) | raw1), ((base0 << 8) | base1));

	return size;
}

static struct device_attribute attrs[] = {
    __ATTR(mode, S_IRUGO|S_IWUSR|S_IXUGO,touchkey_mode_show, touchkey_mode_store),
    __ATTR(firm_version, S_IRUGO|S_IWUSR|S_IXUGO,touchkey_firmversion_show, touchkey_firmversion_store),
    __ATTR(reg_list, S_IRUGO|S_IXUGO,touchkey_reglist_show, NULL),
    __ATTR(key_signal, S_IRUGO|S_IXUGO,touchkey_signal_show, NULL),
    __ATTR(command, S_IRUGO|S_IXUGO|S_IWUSR|S_IWGRP,touchkey_command_show, touchkey_command_store),
    __ATTR(L_sensitivity, S_IRUGO|S_IXUGO|S_IWUSR|S_IWGRP,touchkey_Lsensitivity_show, touchkey_Lsensitivity_store),
    __ATTR(R_sensitivity, S_IRUGO|S_IXUGO|S_IWUSR|S_IWGRP,touchkey_Rsensitivity_show, touchkey_Rsensitivity_store),
	__ATTR(LKeyInt, S_IRUGO|S_IXUGO|S_IWUSR|S_IWGRP,touchkey_LKeyInt_show, touchkey_LKeyInt_store),
	__ATTR(RKeyInt, S_IRUGO|S_IXUGO|S_IWUSR|S_IWGRP,touchkey_RKeyInt_show, touchkey_RKeyInt_store)
};

#define CYPRESS_AVDD_VOL 2960000
static int cypress_power_on(struct cypress_platform_data *pdata, bool val)
{
    int ret = 0;
    if (val)
    {
        if ( pdata->avdd_ldo && !(IS_ERR(pdata->avdd_ldo))) {
            pr_err("%s: enable avdd_ldo\n", __func__);
            ret = regulator_set_voltage(pdata->avdd_ldo, CYPRESS_AVDD_VOL, CYPRESS_AVDD_VOL);
            if(ret){
               pr_err("%s: set avdd ldo to %d error(%d)\n", __func__, CYPRESS_AVDD_VOL, ret);
                return -1;
            }

            pr_err("%s: avdd ldo state1: %d\n", __func__, regulator_is_enabled(pdata->avdd_ldo));

            ret = regulator_enable(pdata->avdd_ldo);
            if(ret){
                pr_err("%s: enable avdd ldo error(%d)\n", __func__, ret);
                return -1;
            }

           pr_err("%s: avdd ldo state2: %d\n", __func__, regulator_is_enabled(pdata->avdd_ldo));
        }else{
            pr_err("%s: avdd_ldo not exist!\n", __func__);
        }

        //power on 1p8
        if (pdata->power_gpio < 0) {
            pr_err("%s:power_gpio not set!!!\n", __func__);
            return -1;
        }

        pr_info("%s:start power on 1v8\n", __func__);
        pr_info("%s: set gpio %d to %d\n", __func__, pdata->power_gpio, pdata->power_on_flag);
        ret = gpio_request(pdata->power_gpio, "cypress_touch_key");
        if (ret) {
            pr_err("%s: Failed to get gpio %d (ret: %d)\n",__func__, pdata->power_gpio, ret);
            return ret;
        }

        ret = gpio_direction_output(pdata->power_gpio, pdata->power_on_flag);
        if (ret) {
            pr_err("%s: Failed to set gpio %d to %d\n", pdata->power_gpio, pdata->power_on_flag);
            return ret;
        }

        gpio_free(pdata->power_gpio);
        msleep(200);
    }
    else
    {
    //tp use the power,we not need power off it
    //add power off func
    }
    return ret;
}

static irqreturn_t cypress_touchkey_interrupt(int irq, void *dev_id)
{

    struct i2c_client *client = (struct i2c_client *)dev_id;
	struct cypress_platform_data *pdata = (struct cypress_platform_data *)client->dev.platform_data;
    struct input_dev *input = pdata->input_dev;
    u8 val;
    int cur_value;
    int status;
    static int last_value = 0;

    //read key value single keys value are 0x01, 0x02, both pressed keys value is 0x03
    val = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_KEYVAL);
    if (val < 0) {
        dev_err(&client->dev, "cypress key read error [%d]\n", val);
        goto out;
    }
	if(val == 0xff && !pdata->bLeftDown && !pdata->bRightDown) {
        dev_err(&client->dev, "cypress key ignored [%d]\n", val);
		last_value = 0;
		goto out;
	}
//    dev_info(&client->dev, "val = 0x%x\n", val);

    cur_value = (val == 0xff ) ? 0x0 : val;
    status = last_value ^ cur_value;

    if(status & CYPRESS_LEFT_BIT)
    {
        input_report_key(input, KEY_F7, (cur_value & CYPRESS_LEFT_BIT)?1:0);
        input_sync(input);
		pdata->bLeftDown = (cur_value & CYPRESS_LEFT_BIT)?true:false;
		if(pdata->LKeyInt) {
			if(pdata->bLeftDown) {
				mod_timer(&pdata->LTimer, jiffies+msecs_to_jiffies(pdata->LKeyInt));
			}
			else if(timer_pending(&pdata->LTimer)) {
				del_timer(&pdata->LTimer);
			}
		}
    }
    if(status & CYPRESS_RIGHT_BIT)
    {
        input_report_key(input, KEY_F8, (cur_value & CYPRESS_RIGHT_BIT)?1:0);
        input_sync(input);
		pdata->bRightDown = (cur_value & CYPRESS_RIGHT_BIT)?true:false;
		if(pdata->RKeyInt) {
			if(pdata->bRightDown) {
				mod_timer(&pdata->RTimer, jiffies+msecs_to_jiffies(pdata->RKeyInt));
			}
			else if(timer_pending(&pdata->RTimer)) {
				del_timer(&pdata->RTimer);
			}
		}
    }
	if(pdata->RKeyInt > NUBIA_TIMER_SIGNAL_BASE) {
		if(pdata->bLeftDown || pdata->bRightDown) {
			mod_timer(&pdata->RTimer, jiffies+msecs_to_jiffies(pdata->RKeyInt - NUBIA_TIMER_SIGNAL_BASE));
		}
		else if(timer_pending(&pdata->RTimer)) {
			del_timer(&pdata->RTimer);
		}
	}
	pr_info("cypress val = %3d, cur_value = %d, status = %d keyst=[%d,%d]\n", val, cur_value, status, (int)pdata->bLeftDown, (int)pdata->bRightDown);
    last_value = cur_value;
 out:
    return IRQ_HANDLED;
}
static int parse_dt(struct device *dev, struct cypress_platform_data *pdata)
{
    int retval;
    u32 value;
    struct device_node *np = dev->of_node;

    pdata->irq_gpio = of_get_named_gpio_flags(np,
        "touchkey,irq-gpio", 0, NULL);

	pr_err("Cypress irq gpio:%d\n",pdata->irq_gpio);

    retval = of_property_read_u32(np, "touchkey,irq-flags", &value);
    if (retval < 0){
	dev_err(dev, "parse irq-flags error\n");
        return retval;
    }else{
        pdata->irq_flag = value;
	pr_err("Cypress irq flag:0x%x\n",pdata->irq_flag);
    }

    pdata->power_gpio = of_get_named_gpio_flags(np,
        "touchkey,power-gpio", 0, NULL);

	 pr_err("Cypress power_gpio:%d\n",pdata->power_gpio);

    retval = of_property_read_u32(np, "touchkey,power-on-flag", &value);
    if (retval < 0){
	dev_err(dev, "parse power-on-flag error\n");
        return retval;
    }else{
        pdata->power_on_flag = value;
        	pr_err("Cypress power_on_flag:0x%x\n",pdata->power_on_flag);
    }

    pdata->avdd_ldo = regulator_get_optional(dev, "touchkey,avdd");
    pdata->avdd_ldo = regulator_get_optional(dev, "touchkey,avdd");

    return 0;
}

static void cypress_touch_key_update_work_func(struct work_struct *work)
{
//    struct cypress_platform_data *pdata = (struct cypress_platform_data *)work;
	struct delayed_work *pDelayedWork = container_of(work, struct delayed_work, work);
    struct cypress_platform_data *pdata = container_of(pDelayedWork, struct cypress_platform_data, cypress_update_work);
    struct i2c_client *client = pdata->client;
    struct device *dev = &client->dev;
    int fw_ver = -1, fw_f_ver = -1;
    int retry = 5;
    //read fw version and update
    int retval;
    const struct firmware *fw_entry = NULL;
	char *pImageBuf = NULL;
	const char **stringImage = NULL;
	size_t imgLen;
	int i = 0;
	int lineCnt = 0;
	bool bTail = false;
	char buf[CYPRESS_BUFF_LENGTH];

    if(pdata == NULL || dev == NULL || client == NULL) {
        pr_err("%s: pdata/dev/client=[%p, %p, %p]\n", __func__, pdata, dev, client);
		goto exit;
    }

	if(pdata->bUpdateOver) {
		if(pdata->RKeyInt > NUBIA_TIMER_SIGNAL_BASE && (pdata->bLeftDown || pdata->bRightDown)) {
			touchkey_signal_print(&pdata->client->dev, buf);
			pr_info("cypress %s\n", buf);
			return;
		}
	}

    pr_info("%s: start check fw version...[%p, %p, %p]\n", __func__, pdata, dev, client);
    retval = request_firmware(&fw_entry, NUBIA_FIRM_VERSION, &client->dev);
    if (retval != 0) {
        pr_err("%s: Firmware ihex %s not available\n",
                __func__, NUBIA_FIRM_NAME);
		goto exit;
    }
	fw_f_ver = simple_strtol(fw_entry->data, NULL, 0);
	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}
    do{
        fw_ver = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_FW);
        if(fw_ver < 0){
           pr_err("%s: read fw version error(%d)\n",  __func__, fw_ver);
           msleep(100);
        }else{
           break;
        }
    }while(retry--);

    pr_info("%s: current fw version:[0x%x] , new fw version [0x%x], retry=%d\n", __func__, fw_ver, fw_f_ver, retry);

    if (fw_ver == fw_f_ver) {
		goto exit;
    }
    retval = request_firmware(&fw_entry, NUBIA_FIRM_NAME, &client->dev);
    if (retval != 0) {
        pr_err("%s: Firmware ihex %s not available\n",
                __func__, NUBIA_FIRM_NAME);
		goto exit;
    }
	imgLen = fw_entry->size;
	pImageBuf = kzalloc(imgLen + 1, GFP_KERNEL);
	memcpy((void *)pImageBuf, (const void *)fw_entry->data, imgLen+1);
	stringImage = kzalloc((imgLen / NUBIA_FIRM_LINE_LEN + 2) * sizeof(const char *), GFP_KERNEL);
	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}
	stringImage[lineCnt] = pImageBuf;
	lineCnt++;
	for(i = 0; i < imgLen; i++) {
		if(pImageBuf[i] == '\r' || pImageBuf[i] == '\n') {
			pImageBuf[i] = '\0';
			bTail = true;
		}
		else if(bTail) {
			stringImage[lineCnt] = &pImageBuf[i];
			lineCnt++;
			bTail = false;
		}
	}
	pr_info("%s: start check fw version, line_0=%s\n", __func__, stringImage[0]);
	pr_info("%s: start check fw version, size=%d, pn=%d, lineCnt=%d, i=%d, bTail=%d\n", __func__, imgLen, (imgLen / NUBIA_FIRM_LINE_LEN + 2), lineCnt, i, (int)bTail);
	pr_info("Ready to update firmware\n");
	disable_irq_nosync(pdata->irq);
//        ret = cypress_firmware_update(client,stringImage_0, LINE_CNT_0);
	retval = cypress_firmware_update(client,stringImage, lineCnt);
	if (retval) {
		pr_err("cypress Firmware update fail,cur ver is :%x,ret=%x\n", fw_ver,retval);
	}
	else {
		msleep(100);
		fw_ver = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_FW);
		pr_err("cypress Firmware update success,new version: 0x%x\n", fw_ver);
	}

	enable_irq(pdata->irq);

	if(pImageBuf)
		kfree(pImageBuf);
	if(stringImage)
		kfree(stringImage);
exit:
	pdata->bUpdateOver = true;
	cypress_set_mode(client, pdata->new_mode);
}

static int cypress_touchkey_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct cypress_platform_data *pdata = NULL;
    struct input_dev *input_dev = NULL;
    int ret = 0;
//    int fw_ver;
    int attr_count = 0;

    dev_info(&client->dev, " now Cypress probe start!\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
    {
        dev_err(&client->dev, "i2c_check_functionality error!\n");
        ret = -ENODEV;
        goto info_err;
    }

    if (client->dev.of_node)
    {
        pdata = devm_kzalloc(&client->dev,
            sizeof(struct cypress_platform_data),
            GFP_KERNEL);
        if (!pdata)
        {
            dev_err(&client->dev, "Failed to allocate memory\n");
            ret = -ENOMEM;
            goto info_err;
        }

        client->dev.platform_data = pdata;
		pdata->client = client;
        ret = parse_dt(&client->dev,pdata);
        if (ret)
        {
            dev_err(&client->dev, "Cypress parse device tree error\n");
            goto data_err;
        }
        else
        {
            dev_info(&client->dev, "Cypress irq gpio:%d,irg flag:0x%x\n",pdata->irq_gpio,pdata->irq_flag);
        }
    }
    else
    {
        pdata = client->dev.platform_data;
        if (!pdata)
        {
            dev_err(&client->dev, "No platform data\n");
            ret = -ENODEV;
            goto info_err;
        }
    }

    cypress_power_on(pdata, true);
    input_dev = input_allocate_device();
    if(input_dev == NULL)
    {
       dev_info(&client->dev, "Failed to allocate input device !\n");
       ret= -ENOMEM;
       goto info_err;
    }
    input_dev->name = "cypress_touchkey";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(KEY_F7, input_dev->keybit);
    __set_bit(KEY_F8, input_dev->keybit);

    ret = input_register_device(input_dev);
    if (ret)
    {
        dev_info(&client->dev, "Failed to register input device !\n");
        goto input_err;
    }

    pdata->input_dev = input_dev;
    pdata->irq = gpio_to_irq(pdata->irq_gpio);
    ret = gpio_direction_input(pdata->irq_gpio);
    if(ret)
    {
        dev_err(&client->dev, "Failed to set gpio\n");
        goto data_err;
    }

    pdata->cypress_update_wq = create_singlethread_workqueue("cypress_update_work");
    INIT_DELAYED_WORK(&pdata->cypress_update_work, cypress_touch_key_update_work_func);
    i2c_set_clientdata(client, pdata);

    ret = request_threaded_irq(pdata->irq, NULL, cypress_touchkey_interrupt,
        pdata->irq_flag,client->dev.driver->name, client);
    if (ret)
    {
        dev_err(&client->dev, "Failed to register interrupt\n");
        goto irq_err;
    }

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
    {
        ret = sysfs_create_file(&client->dev.kobj,&attrs[attr_count].attr);
        if (ret < 0)
        {
            dev_err(&client->dev,"%s: Failed to create sysfs attributes\n",__func__);
        }
    }

	pdata->new_mode = CYPRESS4000_WAKEUP_MODE;
	pdata->bUpdateOver = false;
	pdata->bLeftDown = false;
	pdata->bRightDown = false;
	pdata->LKeyInt = 0;
	pdata->RKeyInt = 0;
	timer_setup(&pdata->LTimer, cypress_LKeyTimer_cb, 0);
	timer_setup(&pdata->RTimer, cypress_RKeyTimer_cb, 0);
	dev_err(&client->dev,"%s: [%p, %p, %p]\n",__func__, pdata, &client->dev, client);

    queue_delayed_work(pdata->cypress_update_wq, &pdata->cypress_update_work, 3 / 10 * HZ);
    return 0;

irq_err:
    free_irq(pdata->irq, pdata);
data_err:
    //devm_kfree(pdata);
input_err:
    input_free_device(input_dev);
info_err:
	if(pdata) {
		kfree(pdata);
	}
    return ret;
}


static int cypress_touchkey_remove(struct i2c_client *client)
{
#if 0
	struct cypress_touchkey_data *data = i2c_get_clientdata(client);
    int attr_count = 0;
	free_irq(client->irq, data);
	cypress_power_on(false);
	input_unregister_device(data->input_dev);
	kfree(data);
    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
    {
        sysfs_remove_file(&client->dev.kobj,&attrs[attr_count].attr);
    }
#endif
	return 0;
}

static void cypress_touchkey_shutdown(struct i2c_client *client)
{

 //   cypress_power_on(false);
}

#ifdef CONFIG_PM_SLEEP
static int cypress_touchkey_suspend(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    int ret;
    ret = cypress_set_mode(client, CYPRESS4000_SLEEP_MODE);
    dev_err(&client->dev, "touchkey suspend success, suspend mode[0x%x]\n",ret);
    return 0;
}

static int cypress_touchkey_resume(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    int ret;
    ret = cypress_set_mode(client, CYPRESS4000_WAKEUP_MODE);
    dev_err(&client->dev, "touchkey resume success, resume mode[0x%x]\n",ret);
    return 0;
}
#endif


static SIMPLE_DEV_PM_OPS(cypress_touchkey_pm_ops,
    cypress_touchkey_suspend, cypress_touchkey_resume);

static const struct i2c_device_id cypress_touchkey_id[] = {
    { "cypress,touchkey", 0 },
    {},
};
MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);


static struct of_device_id cypress_touchkey_match_table[] = {
        { .compatible = "cypress,touchkey-i2c",},
        {},
};

static struct i2c_driver cypress_touchkey_driver = {
    .driver = {
        .name = "cypress_touchkey",
        .owner = THIS_MODULE,
        .of_match_table = cypress_touchkey_match_table,
        .pm = &cypress_touchkey_pm_ops,
    },
    .probe = cypress_touchkey_probe,
    .remove = cypress_touchkey_remove,
    .shutdown = cypress_touchkey_shutdown,
    .id_table = cypress_touchkey_id,
};

module_i2c_driver(cypress_touchkey_driver);

/* Module information */
MODULE_AUTHOR("nubia, Inc.");
MODULE_DESCRIPTION("Touchkey driver for cy8c4014lqi-421");
MODULE_LICENSE("GPL");
