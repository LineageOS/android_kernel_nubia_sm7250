// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/bitops.h>
#include <linux/of_address.h>


#define CC_LOG(fmt, args...) printk(KERN_DEBUG "[CC_CTRL] [%s: %d] "fmt,  __func__, __LINE__, ##args)

#define CC_UFP_MODE         0x02
#define CC_DRP_MODE         0x10
#define CC_ROLE_MASK        0x12
#define CC_TYPEC_MASK       0x003f

#define OPT_WRITE           0x01
#define OPT_READ            0x02
#define OPT_DEBUG           0x03

enum ROLE{
    ROLE_UFP = 1,
	ROLE_DRP = 2,
	ROLE_UNKNOW = 0xff
};

typedef struct{
    struct device *dev;
	u8 role;
	int irq_gpio;
	int irq_id;
	u16 reg_base;
	struct regmap *regmap;
	struct kobject *kobj;
	struct mutex lock;
}ST_CC_CTRL;


int ctype_reg_operate(ST_CC_CTRL *cc_control, u8 opt_type, u32 mask, u8 val)
{
	int ret = 0;
	int tmp = 0;

	mutex_lock(&cc_control->lock);
	if(opt_type==OPT_WRITE || opt_type==OPT_DEBUG){
		ret = regmap_update_bits(cc_control->regmap, cc_control->reg_base, mask, val);
		if (ret < 0){
			CC_LOG("Set reg=%x bit=%x to %02x failed!\n", cc_control->reg_base, mask, val);
		}else{
			if(opt_type != OPT_DEBUG){
				cc_control->role = val;
			}
		}
	}

	regmap_read(cc_control->regmap, cc_control->reg_base, &tmp);

	mutex_unlock(&cc_control->lock);

	return tmp;
}

static ssize_t role_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int tmp = 0;
	ST_CC_CTRL *cc_control = dev_get_drvdata(dev);

	if (!buf || count <= 0) {
		CC_LOG("argument err.\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		CC_LOG("fail to covert digit\n");
		return -EINVAL;
	}

	if(val == ROLE_DRP){
		tmp = ctype_reg_operate(cc_control, OPT_WRITE, CC_ROLE_MASK, CC_DRP_MODE);
		CC_LOG("Force role to DRP, Read back reg=%02x.\n", tmp);
	}else if(val == ROLE_UFP){
		tmp = ctype_reg_operate(cc_control, OPT_WRITE, CC_ROLE_MASK, CC_UFP_MODE);
		CC_LOG("Force role to UFP, Read back reg=%02x.\n", tmp);
	}else{
		CC_LOG("unknow role=%d\n", val);
	}

	return count;
}
static ssize_t role_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ST_CC_CTRL *cc_control = dev_get_drvdata(dev);

	ret =  sprintf(buf, "role:%d.UFP=%d;DRP=%d.\n", cc_control->role==CC_UFP_MODE?ROLE_UFP:ROLE_DRP, ROLE_UFP, ROLE_DRP);

	return ret;
}


static ssize_t typec_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int tmp = 0;
	unsigned int val = 0;
	ST_CC_CTRL *cc_control = dev_get_drvdata(dev);

	if (!buf || count <= 0) {
		CC_LOG("argument err.\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		CC_LOG("fail to covert digit\n");
		return -EINVAL;
	}

	
	tmp = ctype_reg_operate(cc_control, OPT_DEBUG, CC_TYPEC_MASK, val);
	CC_LOG("Force write 0x%02x to ctype reg. Read back reg=%02x.\n", val, tmp);

	return count;
}
static ssize_t typec_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int val = 0;
	ST_CC_CTRL *cc_control = dev_get_drvdata(dev);

	val = ctype_reg_operate(cc_control, OPT_READ, CC_TYPEC_MASK, 0);

	ret =  sprintf(buf,     "TYPEC_MOD_CFG[0:7]:%02x.\n", val);
	ret += sprintf(buf+ret, "              bit0:TYPEC_DISABLE_CMD.\n");
	ret += sprintf(buf+ret, "              bit1:EN_SNK_ONLY.\n");
	ret += sprintf(buf+ret, "              bit2:EN_SRC_ONLY.\n");
	ret += sprintf(buf+ret, "              bit3:EN_TRY_SRC.\n");
	ret += sprintf(buf+ret, "              bit4:EN_TRY_SNK.\n");
	ret += sprintf(buf+ret, "              bit5:EN_VPD_DETECTION.\n");

	return ret;
}

static DEVICE_ATTR(role, S_IRUGO | S_IWUSR, role_show, role_store);
static DEVICE_ATTR(typec_reg, S_IRUGO | S_IWUSR, typec_reg_show, typec_reg_store);


static struct attribute *cc_sys_attrs[] = {
	&dev_attr_role.attr,
	&dev_attr_typec_reg.attr,
	NULL,
};

static const struct attribute_group cc_sys_attr_group = {
	.attrs = cc_sys_attrs,
	.name = "cc_control",
};

int cc_stat_init(ST_CC_CTRL *cc_control)
{
	int val = 0;

	val = gpio_get_value(cc_control->irq_gpio);
	if(val){			//no device insert
		ctype_reg_operate(cc_control, OPT_WRITE, CC_ROLE_MASK, CC_UFP_MODE);
		CC_LOG("C-type port is idle, set role to UFP!\n");
	}else{				//device insert
		CC_LOG("C-type port is busy, keep current cc status.\n");
	}

	return 0;
}
static irqreturn_t cc_control_interrupt_handler(int irq, void *dev_id)
{
	int val = 0;
	unsigned int tmp = 0;
	ST_CC_CTRL *cc_control = (ST_CC_CTRL *)dev_id;

	val = gpio_get_value(cc_control->irq_gpio);
	if(val){			//no device insert
		tmp = ctype_reg_operate(cc_control, OPT_WRITE, CC_ROLE_MASK, CC_UFP_MODE);
		CC_LOG("Set usb role to UFP. Read back reg=%02x.\n", tmp);
	}else{				//device insert
		tmp = ctype_reg_operate(cc_control, OPT_WRITE, CC_ROLE_MASK, CC_DRP_MODE);
		CC_LOG("Set usb role to DRP. Read back reg=%02x.\n", tmp);
	}

	return IRQ_HANDLED;
}

static int cc_control_probe(struct platform_device *pdev)
{
	int ret = 0;
	const __be32 *addr;
	ST_CC_CTRL *cc_control = NULL;
	struct pinctrl *cc_pinctrl=NULL;
	struct pinctrl_state *cc_pinctrl_stat=NULL;

	CC_LOG("Enter.\n");

	cc_control = devm_kzalloc(&pdev->dev, sizeof(*cc_control), GFP_KERNEL);
	if (!cc_control)
		return -ENOMEM;

	cc_control->dev = &pdev->dev;
	CC_LOG("struct device address is %p. cc_control address=%p.\n", &pdev->dev, cc_control);

	cc_control->regmap = dev_get_regmap(cc_control->dev->parent, NULL);
	if (!cc_control->regmap) {
		CC_LOG("get device regmap failed!\n");
		return -EINVAL;
	}
	addr = of_get_address(cc_control->dev->of_node, 0, NULL, NULL);
	if (!addr) {
		CC_LOG("get device reg address failed!\n");
		return -EINVAL;
	}
	cc_control->reg_base = be32_to_cpu(addr[0]);

	mutex_init(&cc_control->lock);

	dev_set_drvdata(cc_control->dev, cc_control);

	cc_pinctrl = devm_pinctrl_get(cc_control->dev);
	if(cc_pinctrl == NULL){
		CC_LOG("get CC pinctrl failed!\n");
	}

	cc_pinctrl_stat = pinctrl_lookup_state(cc_pinctrl, "cc_control_irq");
	if(cc_pinctrl_stat == NULL){
		CC_LOG("lookup pinctrl failed!\n");
	}

	if(cc_pinctrl!=NULL && cc_pinctrl_stat!=NULL)
	{
		ret = pinctrl_select_state(cc_pinctrl, cc_pinctrl_stat);
	}

	CC_LOG("pinctrl config end!\n");

	cc_control->irq_gpio = of_get_named_gpio(cc_control->dev->of_node,"irq-gpio",0);
    if(!gpio_is_valid(cc_control->irq_gpio))
    {
        CC_LOG("Gpio=%d is invalid!!!\n", cc_control->irq_gpio);
		ret = -EINVAL;
		goto destroy;
    }
    ret = gpio_request(cc_control->irq_gpio, "irq-gpio");
    if(ret)
    {
        CC_LOG("Gpio=%d request failed!!!\n", cc_control->irq_gpio);
		ret = -EINVAL;
		goto destroy;
    }

	CC_LOG("irq-gpio=%d get and request success!\n", cc_control->irq_gpio);

	gpio_direction_input(cc_control->irq_gpio);

	cc_control->irq_id = gpio_to_irq(cc_control->irq_gpio);

	ret = request_threaded_irq( cc_control->irq_id,
		                        NULL,
                                (irq_handler_t)cc_control_interrupt_handler,
                                IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_ONESHOT,
                                "USB_CC_CTRL",
                                (void*)cc_control);
	if (ret < 0) {
		CC_LOG("thread_id request failed!!!\n");
		ret = -EINVAL;
		goto destroy;
	}

	CC_LOG("irq-gpio=%d to irq-id=%d interrupt config success!\n", cc_control->irq_gpio, cc_control->irq_id);

	disable_irq_nosync(cc_control->irq_id);

	enable_irq(cc_control->irq_id);		//may creat a invalid irq

	msleep(10);		//delay for invalid interrupt proce end.

	cc_stat_init(cc_control);	//set cc current status

	enable_irq_wake(cc_control->irq_id);

    ret = sysfs_create_group(&cc_control->dev->kobj, &cc_sys_attr_group);
    if(ret){
		CC_LOG("device debug node create failed!\n");
	}else{
		CC_LOG("cc_control/ for debug create success!\n");
	}

	CC_LOG("Exit success!\n");

	return 0;
destroy:
	mutex_destroy(&cc_control->lock);
	dev_set_drvdata(cc_control->dev, NULL);

	return ret;
}
int cc_control_remove(struct platform_device *pdev)
{
	ST_CC_CTRL *cc_control = dev_get_drvdata(&pdev->dev);

	CC_LOG("Enter.\n");

	mutex_destroy(&cc_control->lock);

	disable_irq_nosync(cc_control->irq_id);

	free_irq(cc_control->irq_id, NULL);

	gpio_free(cc_control->irq_gpio);

	sysfs_remove_group(&cc_control->dev->kobj, &cc_sys_attr_group);

	dev_set_drvdata(cc_control->dev, NULL);

	CC_LOG("Exit success!\n");

    return 0;
}
static int cc_control_suspend(struct device *device)
{
	return 0;
}

static int  cc_control_resume(struct device *device)
{
	return 0;
}

static const struct of_device_id cc_control_of_match[] = {
	{ .compatible = "nubia,cc_control", },
	{ },
};
static const struct dev_pm_ops cc_control_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cc_control_suspend, cc_control_resume)
};

static struct platform_driver cc_control_driver = {
	.driver = {
		.name = "usb-cc-control",
		.owner = THIS_MODULE,
		.of_match_table = cc_control_of_match,
		.pm = &cc_control_pm_ops,
	},
	.probe = cc_control_probe,
	.remove = cc_control_remove,
};

static int __init cc_control_init(void)
{
	CC_LOG("CC control driver register.\n");

	return platform_driver_register(&cc_control_driver);
}

static void __exit cc_control_exit(void)
{
	CC_LOG("CC control driver unregister.\n");

	platform_driver_unregister(&cc_control_driver);
}

module_init(cc_control_init);
module_exit(cc_control_exit);

//MODULE_DESCRIPTION("Nubia usb cc control driver");
//MODULE_LICENSE("GPL v2");
