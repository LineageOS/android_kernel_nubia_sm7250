/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include "cam_nubia_dev.h"
#include "../cam_eeprom/cam_eeprom_soc.h"
#include "../cam_eeprom/cam_eeprom_core.h"



#define CAL_DATA_ADDR    0x158F  //nubia caipengbo  add for CalibrationData in Hi846

static struct kobject *nubia_camera_kobj;
static int bokeh_write;

static struct camera_io_master eeprom_master_info;  //nubia caipengbo add for CalibrationData
bool is_eeprom_poweron;  //nubia caipengbo add for CalibrationData

/*ZTEMT: caipengbo add for write calibration--------Start*/
int32_t cam_nubia_eeprom_io_init(struct camera_io_master io_master)
{
	int rc = 0;
	eeprom_master_info = io_master;
	CAM_ERR(CAM_EEPROM, "mater_type:%d,sid=%x",eeprom_master_info.master_type,eeprom_master_info.cci_client->sid);
	return rc;
}
EXPORT_SYMBOL(cam_nubia_eeprom_io_init);
int32_t cam_nubia_eeprom_unlock_write(void)
{   
	//CAM_ERR(CAM_SENSOR, "cam_nubia_eeprom_unlock_write");
	int rc = 0;
	uint32_t readdata = 0; 
	uint16_t sid_tmp = 0;
	
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array unlock_reg[]=
		{
			{0x06CA, 0x1D, 0x00, 0x00},
		};

	if(!(eeprom_master_info.cci_client)){
		CAM_ERR(CAM_EEPROM, "eeprom_master_info.cci_client is NULL");
		return -1;
	}
	
	sid_tmp = eeprom_master_info.cci_client->sid;
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = unlock_reg;
	write_setting.size = 1; 
	
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting, false);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_write  [unlock] error");
		return -1;
	}
	usleep_range(10000, 11000);
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, unlock_reg->reg_addr,
				&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	eeprom_master_info.cci_client->sid = sid_tmp;
	CAM_ERR(CAM_EEPROM, " read CSP =%x ,rc=%d", readdata,rc);
    return rc;
}

int32_t cam_nubia_eeprom_lock_write(void)
{
	int rc = 0;
	uint32_t readdata = 0;
	uint16_t sid_tmp = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array lock_reg[] =
		{
			{0x06CA, 0x1F, 0x00, 0x00},
		};
	sid_tmp = eeprom_master_info.cci_client->sid;
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;//0xBC

	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = lock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting, false);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [lock] error");
		return -1;
	}
		usleep_range(10000, 11000);

	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, lock_reg->reg_addr,
				&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	eeprom_master_info.cci_client->sid = sid_tmp;
	CAM_ERR(CAM_EEPROM, " read CSP =0x%x ", readdata);
	return rc;

}

int32_t cam_nubia_i2c_write_seq(uint32_t addr, uint8_t *data, uint32_t num_byte)
{
	int32_t rc = -EFAULT;
	int write_num = 0;
	int write_width = 32;
	int i = 0;
	int j = 0;

	struct cam_sensor_i2c_reg_array write_buf[32];
	struct cam_sensor_i2c_reg_setting write_setting;

	CAM_ERR(CAM_EEPROM, "%s  ---E\n", __func__);
	CAM_ERR(CAM_EEPROM, "num_byte = %d\n", num_byte);

	if (num_byte < 32)
	{
		CAM_ERR(CAM_EEPROM, "%s  num_byte is smaller than 32 ,error\n", __func__);
		goto END;
	}

	//frist_write_width
	write_width = 32 - (addr % 32);

	for (i = 0; i < num_byte;)
	{
		//update write_width
		if (num_byte - write_num < 32)
		{
			write_width = num_byte - write_num;
		}
		CAM_ERR(CAM_EEPROM, "caipengbo remain num = %d addr= %x write_width = %d \n",
				num_byte - write_num, addr + i, write_width);

		//update write_buf
		for (j = 0; j < write_width; j++)
		{
			write_buf[j].reg_data = data[write_num + j];
			write_buf[j].delay = 0;
			write_buf[j].data_mask = 0;
			
		}

		write_buf[0].reg_addr = addr + i;
		write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		write_setting.delay = 0;
		write_setting.reg_setting = write_buf;
		write_setting.size = write_width;

		rc = cam_cci_i2c_write_continuous_table(&eeprom_master_info, &write_setting, 0, false);
		if ( rc < 0)
		{
			CAM_ERR(CAM_EEPROM, "%s nubia_i2c_write_seq error\n", __func__);
			goto END;
		}

		write_num += write_width;
		i += write_width;
		write_width = 32;

		usleep_range(5000, 6000);
	}

	CAM_ERR(CAM_EEPROM, "%s  total write num = %d\n", __func__, write_num);

END:
	CAM_ERR(CAM_EEPROM, "%s  ---X\n", __func__);
	return rc;
}
/*ZTEMT: caipengbo add for write calibration--------end*/
/*ZTEMT: caipengbo add for write calibration--------Start*/
static ssize_t eeprom_calibration_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    if (bokeh_write){
        return sprintf(buf,"%s\0",  "OK");
    }else {
        return sprintf(buf,"%s\0",  "FAIL");
    }
}

static ssize_t eeprom_calibration_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int ret= 0;

	if (!is_eeprom_poweron){
        return -EINVAL;
	}

	if (eeprom_master_info.master_type == CCI_MASTER)
	{
		ret = camera_io_init(&eeprom_master_info);
		if (ret)
		{
			pr_err( "cci_init failed");
			return -EINVAL;
		}
	}

	cam_nubia_eeprom_unlock_write();

	usleep_range(10000, 11000);	
	ret = cam_nubia_i2c_write_seq(CAL_DATA_ADDR,(uint8_t*)buf, count);
	usleep_range(10000, 11000);

	cam_nubia_eeprom_lock_write();
	camera_io_release(&eeprom_master_info);
    if (ret < 0)
        bokeh_write = 0;
    else
        bokeh_write = 1;


	return count;
}


static struct kobj_attribute eeprom_calibration_attribute =
	__ATTR(eeprom_calibration, 0664, eeprom_calibration_show, eeprom_calibration_store);
/*ZTEMT: caipengbo add for write calibration--------end*/


static struct attribute *attrs[] = {
	&eeprom_calibration_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


static int32_t __init cam_nubia_node_init_module(void)
{
	int retval;

	pr_err("[CAM_NODE] cam_nubia_node_init_module \n");

	nubia_camera_kobj = kobject_create_and_add("camera", kernel_kobj);
	if (!nubia_camera_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(nubia_camera_kobj, &attr_group);
	if (retval)
		kobject_put(nubia_camera_kobj);

	return retval;

}

static void __exit cam_nubia_node_exit_module(void)
{
	pr_err("[CAM_NODE] cam_nubia_node_exit_module \n");
	kobject_put(nubia_camera_kobj);
}

module_init(cam_nubia_node_init_module);
module_exit(cam_nubia_node_exit_module);
MODULE_DESCRIPTION("CAM NUBIA");
MODULE_LICENSE("GPL v2");

