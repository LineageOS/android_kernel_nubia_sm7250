// SPDX-License-Identifier: GPL-2.0
/*
 * mlx90632.c - Melexis MLX90632 contactless IR temperature sensor
 *
 * Copyright (c) 2017 Melexis <cmo@melexis.com>
 *
 * Driver for the Melexis MLX90632 I2C 16-bit IR thermopile sensor
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#include "../sensor_common.h"
#undef LOG_TAG
#define LOG_TAG "MLX90632"


/* Memory sections addresses */
#define MLX90632_ADDR_RAM	0x4000 /* Start address of ram */
#define MLX90632_ADDR_EEPROM	0x2480 /* Start address of user eeprom */

/* EEPROM addresses - used at startup */
#define MLX90632_EE_CTRL	0x24d4 /* Control register initial value */
#define MLX90632_EE_I2C_ADDR	0x24d5 /* I2C address register initial value */
#define MLX90632_EE_VERSION	0x240b /* EEPROM version reg address */
#define MLX90632_EE_P_R		0x240c /* P_R calibration register 32bit */
#define MLX90632_EE_P_G		0x240e /* P_G calibration register 32bit */
#define MLX90632_EE_P_T		0x2410 /* P_T calibration register 32bit */
#define MLX90632_EE_P_O		0x2412 /* P_O calibration register 32bit */
#define MLX90632_EE_Aa		0x2414 /* Aa calibration register 32bit */
#define MLX90632_EE_Ab		0x2416 /* Ab calibration register 32bit */
#define MLX90632_EE_Ba		0x2418 /* Ba calibration register 32bit */
#define MLX90632_EE_Bb		0x241a /* Bb calibration register 32bit */
#define MLX90632_EE_Ca		0x241c /* Ca calibration register 32bit */
#define MLX90632_EE_Cb		0x241e /* Cb calibration register 32bit */
#define MLX90632_EE_Da		0x2420 /* Da calibration register 32bit */
#define MLX90632_EE_Db		0x2422 /* Db calibration register 32bit */
#define MLX90632_EE_Ea		0x2424 /* Ea calibration register 32bit */
#define MLX90632_EE_Eb		0x2426 /* Eb calibration register 32bit */
#define MLX90632_EE_Fa		0x2428 /* Fa calibration register 32bit */
#define MLX90632_EE_Fb		0x242a /* Fb calibration register 32bit */
#define MLX90632_EE_Ga		0x242c /* Ga calibration register 32bit */

#define MLX90632_EE_Gb		0x242e /* Gb calibration register 16bit */
#define MLX90632_EE_Ka		0x242f /* Ka calibration register 16bit */

#define MLX90632_EE_Ha		0x2481 /* Ha customer calib value reg 16bit */
#define MLX90632_EE_Hb		0x2482 /* Hb customer calib value reg 16bit */

#define MLX90632_Default_Ha		0x4000 /* Ha default value */
#define MLX90632_Default_Hb		0x00   /* Hb default value */

/* Register addresses - volatile */
#define MLX90632_REG_I2C_ADDR	0x3000 /* Chip I2C address register */

/* Control register address - volatile */
#define MLX90632_REG_CONTROL	0x3001 /* Control Register address */
#define   MLX90632_CFG_PWR_MASK		GENMASK(2, 1) /* PowerMode Mask */
/* PowerModes statuses */
#define MLX90632_PWR_STATUS(ctrl_val) (ctrl_val << 1)
#define MLX90632_PWR_STATUS_HALT MLX90632_PWR_STATUS(0) /* hold */
#define MLX90632_PWR_STATUS_SLEEP_STEP MLX90632_PWR_STATUS(1) /* sleep step*/
#define MLX90632_PWR_STATUS_STEP MLX90632_PWR_STATUS(2) /* step */
#define MLX90632_PWR_STATUS_CONTINUOUS MLX90632_PWR_STATUS(3) /* continuous*/

/* Device status register - volatile */
#define MLX90632_REG_STATUS	0x3fff /* Device status register */
#define   MLX90632_STAT_BUSY		BIT(10) /* Device busy indicator */
#define   MLX90632_STAT_EE_BUSY		BIT(9) /* EEPROM busy indicator */
#define   MLX90632_STAT_BRST		BIT(8) /* Brown out reset indicator */
#define   MLX90632_STAT_CYCLE_POS	GENMASK(6, 2) /* Data position */
#define   MLX90632_STAT_DATA_RDY	BIT(0) /* Data ready indicator */

/* RAM_MEAS address-es for each channel */
#define MLX90632_RAM_1(meas_num)	(MLX90632_ADDR_RAM + 3 * meas_num)
#define MLX90632_RAM_2(meas_num)	(MLX90632_ADDR_RAM + 3 * meas_num + 1)
#define MLX90632_RAM_3(meas_num)	(MLX90632_ADDR_RAM + 3 * meas_num + 2)

/* Magic constants */
#define MLX90632_ID_MEDICAL	0x0105 /* EEPROM DSPv5 Medical device id */
#define MLX90632_ID_CONSUMER	0x0205 /* EEPROM DSPv5 Consumer device id */
#define MLX90632_DSP_VERSION	5 /* DSP version */
#define MLX90632_DSP_MASK	GENMASK(7, 0) /* DSP version in EE_VERSION */
#define MLX90632_RESET_CMD	0x0006 /* Reset sensor (address or global) */
#define MLX90632_REF_12		12LL /**< ResCtrlRef value of Ch 1 or Ch 2 */
#define MLX90632_REF_3		12LL /**< ResCtrlRef value of Channel 3 */
#define MLX90632_MAX_MEAS_NUM	31 /**< Maximum measurements in list */
#define MLX90632_SLEEP_DELAY_MS 3000 /**< Autosleep delay */

#define TEMP_INTERNAL_MOD 1
#define TEMP_OBJECT_MOD  2


struct mlx90632_data {
	struct i2c_client *client;
	struct work_struct	mlx_work; /* for mlx temperature polling */
	struct workqueue_struct *mlx_wq;
	struct hrtimer mlx_timer;
	ktime_t mlx_poll_delay;	/* needed for temperature sensor polling */
	struct device *mlx90632_data_dev;
	struct input_dev *mlx90632_input_dev;
	struct mutex lock; /* Multiple reads for single measurement */
	struct regmap *regmap;
	u16 emissivity;
	bool mlx_enable; /* mlx enable,it will set hrtimer to report temp data */
	uint8_t enable_body; /* enable_body, for sensor-hal enable body temperature */
	uint8_t enable_object; /* enable_object, for sensor-hal enable object temperature */
	bool fac_enable; /* factory enable */
	bool fac_calibrated;
	bool fac_Ha_set;
	bool fac_Hb_set;
	s32 mlx_Ha; /* Ha customer calib value reg 16bit */
	s32 mlx_Hb; /* Hb customer calib value reg 16bit */
	struct mutex data_lock; /* data lock for set data safe */
	signed pwr_vdd_gpio;
};

static const struct regmap_range mlx90632_volatile_reg_range[] = {
	regmap_reg_range(MLX90632_REG_I2C_ADDR, MLX90632_REG_CONTROL),
	regmap_reg_range(MLX90632_REG_STATUS, MLX90632_REG_STATUS),
	regmap_reg_range(MLX90632_RAM_1(0),
	                 MLX90632_RAM_3(MLX90632_MAX_MEAS_NUM)),
};

static const struct regmap_access_table mlx90632_volatile_regs_tbl = {
	.yes_ranges = mlx90632_volatile_reg_range,
	.n_yes_ranges = ARRAY_SIZE(mlx90632_volatile_reg_range),
};

static const struct regmap_range mlx90632_read_reg_range[] = {
	regmap_reg_range(MLX90632_EE_VERSION, MLX90632_EE_Ka),
	regmap_reg_range(MLX90632_EE_CTRL, MLX90632_EE_I2C_ADDR),
	regmap_reg_range(MLX90632_EE_Ha, MLX90632_EE_Hb),
	regmap_reg_range(MLX90632_REG_I2C_ADDR, MLX90632_REG_CONTROL),
	regmap_reg_range(MLX90632_REG_STATUS, MLX90632_REG_STATUS),
	regmap_reg_range(MLX90632_RAM_1(0),
	                 MLX90632_RAM_3(MLX90632_MAX_MEAS_NUM)),
};

static const struct regmap_access_table mlx90632_readable_regs_tbl = {
	.yes_ranges = mlx90632_read_reg_range,
	.n_yes_ranges = ARRAY_SIZE(mlx90632_read_reg_range),
};

static const struct regmap_range mlx90632_no_write_reg_range[] = {
	regmap_reg_range(MLX90632_EE_VERSION, MLX90632_EE_Ka),
	regmap_reg_range(MLX90632_RAM_1(0),
	                 MLX90632_RAM_3(MLX90632_MAX_MEAS_NUM)),
};

static const struct regmap_access_table mlx90632_writeable_regs_tbl = {
	.no_ranges = mlx90632_no_write_reg_range,
	.n_no_ranges = ARRAY_SIZE(mlx90632_no_write_reg_range),
};

static const struct regmap_config mlx90632_regmap = {
	.reg_bits = 16,
	.val_bits = 16,

	.volatile_table = &mlx90632_volatile_regs_tbl,
	.rd_table = &mlx90632_readable_regs_tbl,
	.wr_table = &mlx90632_writeable_regs_tbl,

	.use_single_rw = true,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_RBTREE,
};

#define MLX90632_DRV_NAME "temperature"
#define INPUT_NAME_MLX90632       "temperature"

#define MLX90632_CAL_HA_FILE_PATH        "/persist/sensors/calibrate_ha"
#define MLX90632_CAL_HB_FILE_PATH        "/persist/sensors/calibrate_hb"


static dev_t  mlx90632_dev_t;
static struct class *mlx90632_class;

static s32 mlx90632_pwr_set_sleep_step(struct regmap *regmap)
{
	return regmap_update_bits(regmap, MLX90632_REG_CONTROL,
	                          MLX90632_CFG_PWR_MASK,
	                          MLX90632_PWR_STATUS_SLEEP_STEP);
}

static s32 mlx90632_pwr_continuous(struct regmap *regmap)
{
	return regmap_update_bits(regmap, MLX90632_REG_CONTROL,
	                          MLX90632_CFG_PWR_MASK,
	                          MLX90632_PWR_STATUS_CONTINUOUS);
}

/**
 * mlx90632_perform_measurement - Trigger and retrieve current measurement cycle
 * @*data: pointer to mlx90632_data object containing regmap information
 *
 * Perform a measurement and return latest measurement cycle position reported
 * by sensor. This is a blocking function for 500ms, as that is default sensor
 * refresh rate.
 */
static int mlx90632_perform_measurement(struct mlx90632_data *data)
{
	int ret, tries = 100;
	unsigned int reg_status;

	ret = regmap_update_bits(data->regmap, MLX90632_REG_STATUS,
	                         MLX90632_STAT_DATA_RDY, 0);
	if (ret < 0)
		return ret;

	while (tries-- > 0) {
		ret = regmap_read(data->regmap, MLX90632_REG_STATUS,
		                  &reg_status);
		if (ret < 0)
			return ret;
		if (reg_status & MLX90632_STAT_DATA_RDY)
			break;
		usleep_range(10000, 11000);
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready");
		return -ETIMEDOUT;
	}

	return (reg_status & MLX90632_STAT_CYCLE_POS) >> 2;
}

static int mlx90632_channel_new_select(int perform_ret, uint8_t *channel_new,
                                       uint8_t *channel_old)
{
	switch (perform_ret) {
	case 1:
		*channel_new = 1;
		*channel_old = 2;
		break;
	case 2:
		*channel_new = 2;
		*channel_old = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mlx90632_read_ambient_raw(struct regmap *regmap,
                                     s16 *ambient_new_raw, s16 *ambient_old_raw)
{
	int ret;
	unsigned int read_tmp;

	ret = regmap_read(regmap, MLX90632_RAM_3(1), &read_tmp);
	if (ret < 0)
		return ret;
	*ambient_new_raw = (s16)read_tmp;

	ret = regmap_read(regmap, MLX90632_RAM_3(2), &read_tmp);
	if (ret < 0)
		return ret;
	*ambient_old_raw = (s16)read_tmp;

	return ret;
}

static int mlx90632_read_object_raw(struct regmap *regmap,
                                    int perform_measurement_ret,
                                    s16 *object_new_raw, s16 *object_old_raw)
{
	int ret;
	unsigned int read_tmp;
	s16 read;
	u8 channel = 0;
	u8 channel_old = 0;

	ret = mlx90632_channel_new_select(perform_measurement_ret, &channel,
	                                  &channel_old);
	if (ret != 0)
		return ret;

	ret = regmap_read(regmap, MLX90632_RAM_2(channel), &read_tmp);
	if (ret < 0)
		return ret;

	read = (s16)read_tmp;

	ret = regmap_read(regmap, MLX90632_RAM_1(channel), &read_tmp);
	if (ret < 0)
		return ret;
	*object_new_raw = (read + (s16)read_tmp) / 2;

	ret = regmap_read(regmap, MLX90632_RAM_2(channel_old), &read_tmp);
	if (ret < 0)
		return ret;
	read = (s16)read_tmp;

	ret = regmap_read(regmap, MLX90632_RAM_1(channel_old), &read_tmp);
	if (ret < 0)
		return ret;
	*object_old_raw = (read + (s16)read_tmp) / 2;

	return ret;
}

static int mlx90632_read_all_channel(struct mlx90632_data *data,
                                     s16 *ambient_new_raw, s16 *ambient_old_raw,
                                     s16 *object_new_raw, s16 *object_old_raw)
{
	s32 ret, measurement;

	mutex_lock(&data->lock);
	measurement = mlx90632_perform_measurement(data);
	if (measurement < 0) {
		ret = measurement;
		goto read_unlock;
	}
	ret = mlx90632_read_ambient_raw(data->regmap, ambient_new_raw,
	                                ambient_old_raw);
	if (ret < 0)
		goto read_unlock;

	ret = mlx90632_read_object_raw(data->regmap, measurement,
	                               object_new_raw, object_old_raw);
read_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int mlx90632_read_ee_register(struct regmap *regmap, u16 reg_lsb,
                                     s32 *reg_value)
{
	s32 ret;
	unsigned int read;
	u32 value;

	ret = regmap_read(regmap, reg_lsb, &read);
	if (ret < 0)
		return ret;

	value = read;

	ret = regmap_read(regmap, reg_lsb + 1, &read);
	if (ret < 0)
		return ret;

	*reg_value = (read << 16) | (value & 0xffff);

	return 0;
}

static s64 mlx90632_preprocess_temp_amb(s16 ambient_new_raw,
                                        s16 ambient_old_raw, s16 Gb)
{
	s64 VR_Ta, kGb, tmp;

	kGb = ((s64)Gb * 1000LL) >> 10ULL;
	VR_Ta = (s64)ambient_old_raw * 1000000LL +
	        kGb * div64_s64(((s64)ambient_new_raw * 1000LL),
	                        (MLX90632_REF_3));
	tmp = div64_s64(
	          div64_s64(((s64)ambient_new_raw * 1000000000000LL),
	                    (MLX90632_REF_3)), VR_Ta);
	return div64_s64(tmp << 19ULL, 1000LL);
}

static s64 mlx90632_preprocess_temp_obj(s16 object_new_raw, s16 object_old_raw,
                                        s16 ambient_new_raw,
                                        s16 ambient_old_raw, s16 Ka)
{
	s64 VR_IR, kKa, tmp;

	kKa = ((s64)Ka * 1000LL) >> 10ULL;
	VR_IR = (s64)ambient_old_raw * 1000000LL +
	        kKa * div64_s64(((s64)ambient_new_raw * 1000LL),
	                        (MLX90632_REF_3));
	tmp = div64_s64(
	          div64_s64(((s64)((object_new_raw + object_old_raw) / 2)
	                     * 1000000000000LL), (MLX90632_REF_12)),
	          VR_IR);
	return div64_s64((tmp << 19ULL), 1000LL);
}

static s32 mlx90632_calc_temp_ambient(s16 ambient_new_raw, s16 ambient_old_raw,
                                      s32 P_T, s32 P_R, s32 P_G, s32 P_O,
                                      s16 Gb)
{
	s64 Asub, Bsub, Ablock, Bblock, Cblock, AMB, sum;

	AMB = mlx90632_preprocess_temp_amb(ambient_new_raw, ambient_old_raw,
	                                   Gb);
	Asub = ((s64)P_T * 10000000000LL) >> 44ULL;
	Bsub = AMB - (((s64)P_R * 1000LL) >> 8ULL);
	Ablock = Asub * (Bsub * Bsub);
	Bblock = (div64_s64(Bsub * 10000000LL, P_G)) << 20ULL;
	Cblock = ((s64)P_O * 10000000000LL) >> 8ULL;

	sum = div64_s64(Ablock, 1000000LL) + Bblock + Cblock;

	return div64_s64(sum, 10000000LL);
}

static s32 mlx90632_calc_temp_object_iteration(s32 prev_object_temp, s64 object,
        s64 TAdut, s32 Fa, s32 Fb,
        s32 Ga, s16 Ha, s16 Hb,
        u16 emissivity)
{
	s64 calcedKsTO, calcedKsTA, ir_Alpha, TAdut4, Alpha_corr;
	s64 Ha_customer, Hb_customer;

	Ha_customer = ((s64)Ha * 1000000LL) >> 14ULL;
	Hb_customer = ((s64)Hb * 100) >> 10ULL;

	calcedKsTO = ((s64)((s64)Ga * (prev_object_temp - 25 * 1000LL)
	                    * 1000LL)) >> 36LL;
	calcedKsTA = ((s64)(Fb * (TAdut - 25 * 1000000LL))) >> 36LL;
	Alpha_corr = div64_s64((((s64)(Fa * 10000000000LL) >> 46LL)
	                        * Ha_customer), 1000LL);
	Alpha_corr *= ((s64)(1 * 1000000LL + calcedKsTO + calcedKsTA));
	Alpha_corr = emissivity * div64_s64(Alpha_corr, 100000LL);
	Alpha_corr = div64_s64(Alpha_corr, 1000LL);
	ir_Alpha = div64_s64((s64)object * 10000000LL, Alpha_corr);
	TAdut4 = (div64_s64(TAdut, 10000LL) + 27315) *
	         (div64_s64(TAdut, 10000LL) + 27315) *
	         (div64_s64(TAdut, 10000LL)  + 27315) *
	         (div64_s64(TAdut, 10000LL) + 27315);

	return (int_sqrt64(int_sqrt64(ir_Alpha * 1000000000000LL + TAdut4))
	        - 27315 - Hb_customer) * 10;
}

static s32 mlx90632_calc_temp_object(s64 object, s64 ambient, s32 Ea, s32 Eb,
                                     s32 Fa, s32 Fb, s32 Ga, s16 Ha, s16 Hb,
                                     u16 tmp_emi)
{
	s64 kTA, kTA0, TAdut;
	s64 temp = 25000;
	s8 i;

	kTA = (Ea * 1000LL) >> 16LL;
	kTA0 = (Eb * 1000LL) >> 8LL;
	TAdut = div64_s64(((ambient - kTA0) * 1000000LL), kTA) + 25 * 1000000LL;

	/* Iterations of calculation as described in datasheet */
	for (i = 0; i < 5; ++i) {
		temp = mlx90632_calc_temp_object_iteration(temp, object, TAdut,
		        Fa, Fb, Ga, Ha, Hb,
		        tmp_emi);
	}
	return temp;
}

static int mlx90632_calc_object_dsp105(struct mlx90632_data *data, int *val)
{
	s32 ret;
	s32 Ea, Eb, Fa, Fb, Ga;
	unsigned int read_tmp;
	s16 Ha, Hb, Gb, Ka;
	s16 ambient_new_raw, ambient_old_raw, object_new_raw, object_old_raw;
	s64 object, ambient;

	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_Ea, &Ea);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_Eb, &Eb);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_Fa, &Fa);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_Fb, &Fb);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_Ga, &Ga);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MLX90632_EE_Gb, &read_tmp);
	if (ret < 0)
		return ret;
	Gb = (s16)read_tmp;
	ret = regmap_read(data->regmap, MLX90632_EE_Ka, &read_tmp);
	if (ret < 0)
		return ret;
	Ka = (s16)read_tmp;

	if(data->fac_calibrated == true) {
		Ha = data->mlx_Ha;
		Hb = data->mlx_Hb;
	} else {
		ret = regmap_read(data->regmap, MLX90632_EE_Ha, &read_tmp);
		if (ret < 0)
			return ret;
		Ha = (s16)read_tmp;
		ret = regmap_read(data->regmap, MLX90632_EE_Hb, &read_tmp);
		if (ret < 0)
			return ret;
		Hb = (s16)read_tmp;
	}

	ret = mlx90632_read_all_channel(data,
	                                &ambient_new_raw, &ambient_old_raw,
	                                &object_new_raw, &object_old_raw);
	if (ret < 0)
		return ret;

	ambient = mlx90632_preprocess_temp_amb(ambient_new_raw,
	                                       ambient_old_raw, Gb);
	object = mlx90632_preprocess_temp_obj(object_new_raw,
	                                      object_old_raw,
	                                      ambient_new_raw,
	                                      ambient_old_raw, Ka);

	*val = mlx90632_calc_temp_object(object, ambient, Ea, Eb, Fa, Fb, Ga,
	                                 Ha, Hb, data->emissivity);
	return 0;
}

static int mlx90632_calc_ambient_dsp105(struct mlx90632_data *data, int *val)
{
	s32 ret;
	unsigned int read_tmp;
	s32 PT, PR, PG, PO;
	s16 Gb;
	s16 ambient_new_raw, ambient_old_raw;

	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_P_R, &PR);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_P_G, &PG);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_P_T, &PT);
	if (ret < 0)
		return ret;
	ret = mlx90632_read_ee_register(data->regmap, MLX90632_EE_P_O, &PO);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MLX90632_EE_Gb, &read_tmp);
	if (ret < 0)
		return ret;
	Gb = (s16)read_tmp;

	ret = mlx90632_read_ambient_raw(data->regmap, &ambient_new_raw,
	                                &ambient_old_raw);
	if (ret < 0)
		return ret;
	*val = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,
	                                  PT, PR, PG, PO, Gb);
	return ret;
}

static int mlx90632_read_raw(struct mlx90632_data *data, int mod, int *val)
{
	int ret;

	switch (mod) {
	case TEMP_INTERNAL_MOD:
		ret = mlx90632_calc_ambient_dsp105(data, val);
		if (ret < 0)
			return ret;
		break;
	case TEMP_OBJECT_MOD:
		ret = mlx90632_calc_object_dsp105(data, val);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}
	return 0;

}
/*
static int mlx90632_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel, int val,
			      int val2, long mask)
{
	struct mlx90632_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBEMISSIVITY:
		// Confirm we are within 0 and 1.0
		if (val < 0 || val2 < 0 || val > 1 ||
		    (val == 1 && val2 != 0))
			return -EINVAL;
		data->emissivity = val * 1000 + val2 / 1000;
		return 0;
	default:
		return -EINVAL;
	}
}*/

static void mlx90632_input_report(struct mlx90632_data *data, int object_temp, int internal_temp)
{

	input_report_rel(data->mlx90632_input_dev, REL_X, object_temp+1);
	input_report_rel(data->mlx90632_input_dev, REL_Y, internal_temp+1);
	input_sync(data->mlx90632_input_dev);
	SENSOR_LOG_INFO("input event object_temp:%d, internal_temp:%d\n", object_temp, internal_temp);
}

/* temperature polling routine */
static void mlx90632_temp_polling_work_handler(struct work_struct *work)
{
	struct mlx90632_data *data = container_of(work, struct mlx90632_data, mlx_work);
	int object_temp,internal_temp;

	mlx90632_read_raw(data, TEMP_OBJECT_MOD, &object_temp);
	mlx90632_read_raw(data, TEMP_INTERNAL_MOD, &internal_temp);
	mlx90632_input_report(data, object_temp, internal_temp);
}

/* hrtimer_start call back function,use to report temperature data */
static enum hrtimer_restart mlx90632_temp_timer_func(struct hrtimer *timer)
{
	struct mlx90632_data* data = container_of(timer, struct mlx90632_data, mlx_timer);

	queue_work(data->mlx_wq, &data->mlx_work);
	hrtimer_forward_now(&data->mlx_timer, data->mlx_poll_delay);
	return HRTIMER_RESTART;
}

static int32_t mlx90632_enable_workqueue(struct mlx90632_data *data, uint8_t enable)
{
	int32_t ret;
	uint8_t curr_mlx_enable = (data->mlx_enable)?1:0;

	if(curr_mlx_enable == enable)
		return 0;

	if (enable) {
		data->mlx_enable = true;
		hrtimer_start(&data->mlx_timer, data->mlx_poll_delay, HRTIMER_MODE_REL);
	} else {
		data->mlx_enable = false;
		hrtimer_cancel(&data->mlx_timer);
		cancel_work_sync(&data->mlx_work);
	}
	return ret;
}

static int32_t mlx_set_ha(struct mlx90632_data *data, s32 Ha)
{
	if(Ha == 0) {
		SENSOR_LOG_INFO("Ha = 0, Ha not set\n");
		return -1;
	}
	mutex_lock(&data->data_lock);
	data->mlx_Ha = Ha;
	data->fac_Ha_set = true;
	mutex_unlock(&data->data_lock);
	sensor_write_file(MLX90632_CAL_HA_FILE_PATH, (char *)(&Ha), sizeof(Ha));
	SENSOR_LOG_INFO("Ha set\n");
	//TODO: set ha cal value to chip

	return 0;
}

static int32_t mlx_set_hb(struct mlx90632_data *data, s32 Hb)
{
	mutex_lock(&data->data_lock);
	data->mlx_Hb = Hb;
	data->fac_Hb_set = true;
	mutex_unlock(&data->data_lock);
	sensor_write_file(MLX90632_CAL_HB_FILE_PATH, (char *)(&Hb), sizeof(Hb));
	SENSOR_LOG_INFO("Hb set\n");
	//TODO: set hb cal value to chip

	return 0;

}

static void mlx90632_set_factory(struct mlx90632_data *data, uint8_t fac_enable)
{
	if(fac_enable == 0)
		data->fac_enable = false;
	else
		data->fac_enable = true;

	if(data->fac_enable == true && data->mlx_enable == true) {
		hrtimer_cancel(&data->mlx_timer);
		cancel_work_sync(&data->mlx_work);
	}
	if(data->fac_enable == false && data->mlx_enable == true) {
		hrtimer_start(&data->mlx_timer, data->mlx_poll_delay, HRTIMER_MODE_REL);
	}
}

static int32_t mlx90632_check_calibration_config(struct mlx90632_data *data)
{

	s32 chip_Ha, chip_Hb, persist_Ha = 0, persist_Hb = 0;
	unsigned int read_tmp;
	int32_t ret = 0;

	ret = regmap_read(data->regmap, MLX90632_EE_Ha, &read_tmp);
	if (ret < 0) {
		SENSOR_LOG_ERROR("read chip_Ha cal file error\n");
		return ret;
	}
	chip_Ha = (s16)read_tmp;
	ret = sensor_read_file(MLX90632_CAL_HA_FILE_PATH, (char *)(&persist_Ha), sizeof(persist_Ha));
	if (ret < 0)
		SENSOR_LOG_ERROR("read persist_Ha cal file error\n");
	/* it maybe replace mlx90632 chip,or erase persist sector*/
	if(chip_Ha != MLX90632_Default_Ha && chip_Ha != persist_Ha) {
		sensor_write_file(MLX90632_CAL_HA_FILE_PATH, (char *)(&chip_Ha), sizeof(chip_Ha));
		SENSOR_LOG_INFO("Persist cal file not exist! load chip Ha to persist Ha\n");
	}

	ret = regmap_read(data->regmap, MLX90632_EE_Hb, &read_tmp);
	if (ret < 0) {
		SENSOR_LOG_ERROR("Read chip_Hb cal file error\n");
		return ret;
	}
	chip_Hb = (s16)read_tmp;
	ret = sensor_read_file(MLX90632_CAL_HB_FILE_PATH, (char *)(&persist_Hb), sizeof(persist_Hb));
	if (ret < 0)
		SENSOR_LOG_ERROR("Read persist_Hb cal file error\n");
	/* it maybe replace mlx90632 chip,or erase persist sector*/
	if(chip_Hb != MLX90632_Default_Hb && chip_Hb != persist_Hb) {
		sensor_write_file(MLX90632_CAL_HB_FILE_PATH, (char *)(&chip_Hb), sizeof(chip_Hb));
		SENSOR_LOG_INFO("Persist cal file not exist! load chip Hb to persist Hb\n");
	}
	SENSOR_LOG_INFO("chip Ha %d, chip Hb %d\n", chip_Ha, chip_Hb);
	SENSOR_LOG_INFO("persist Ha %d, chip Hb %d\n", persist_Ha, persist_Hb);
	/*if chip_Ha == 0,chip_Hb == 0 the chip not calibrate*/
	/*
	if(chip_Ha == MLX90632_Default_Ha && chip_Hb == MLX90632_Default_Hb) {
		persist_Ha = 0;
		persist_Hb = 0;
		sensor_write_file(MLX90632_CAL_HA_FILE_PATH, (char *)(&persist_Ha), sizeof(persist_Ha));
		sensor_write_file(MLX90632_CAL_HB_FILE_PATH, (char *)(&persist_Hb), sizeof(persist_Hb));
		SENSOR_LOG_ERROR("chip Ha Hb not calibrate\n");
		data->fac_calibrated = false;
		return -1;
	}
	*/
	if(persist_Ha == 0 && persist_Hb == 0){
		SENSOR_LOG_ERROR("Ha Hb not calibrate\n");
		data->fac_calibrated = false;
		return -1;
	}
	data->fac_calibrated = true;
	mutex_lock(&data->data_lock);
	//data->mlx_Ha = chip_Ha;
	//data->mlx_Hb = chip_Hb;
	data->mlx_Ha = persist_Ha;
	data->mlx_Hb = persist_Hb;
	mutex_unlock(&data->data_lock);
	SENSOR_LOG_INFO("mlx_Ha %d, mlx_Hb %d\n", data->mlx_Ha, data->mlx_Hb);

	return 0;
}


static ssize_t mlx90632_init_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t mlx90632_init_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t size)
{
	int err;
	struct mlx90632_data *data = dev_get_drvdata(dev);

	err = mlx90632_check_calibration_config(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("read factory cal parameters failed\n");
	}
	return size;
}

static ssize_t mlx90632_enable_body_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data =  dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (data->enable_body)?1:0);
}

static ssize_t mlx90632_enable_body_store(struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		data->enable_body = 1;
	else if (sysfs_streq(buf, "0"))
		data->enable_body = 0;
	else {
		SENSOR_LOG_ERROR("invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("Enable: %d\n", data->enable_body);
	/*
	 * body temperature and object temperature use same input type
	 * so if diable body temperature, need check object enable state
	 */
	if(!data->enable_body && data->enable_object)
		return size;
	mlx90632_enable_workqueue(data, data->enable_body);
	return size;
}

static ssize_t mlx90632_enable_object_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data =  dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (data->enable_object)?1:0);
}

static ssize_t mlx90632_enable_object_store(struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		data->enable_object = 1;
	else if (sysfs_streq(buf, "0"))
		data->enable_object = 0;
	else {
		SENSOR_LOG_ERROR("invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("Enable: %d\n", data->enable_object);
	/*
	 * body temperature and object temperature use same input type
	 * so if diable object temperature, need check body enable state
	 */
	if(!data->enable_object && data->enable_body)
		return size;
	mlx90632_enable_workqueue(data, data->enable_object);
	return size;
}

static ssize_t internal_temp_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);
	int internal_temp;

	mlx90632_read_raw(data, TEMP_INTERNAL_MOD, &internal_temp);

	return sprintf(buf, "%d\n", internal_temp);
}
static ssize_t object_temp_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);
	int object_temp;

	mlx90632_read_raw(data, TEMP_OBJECT_MOD, &object_temp);

	return sprintf(buf, "%d\n", object_temp);

}

static ssize_t factory_test_store(struct device *dev, struct device_attribute *attr,
                                  const char *buf, size_t size)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);
	uint8_t fac_enable;

	if (sysfs_streq(buf, "1"))
		fac_enable = 1;
	else if (sysfs_streq(buf, "0"))
		fac_enable = 0;
	else {
		SENSOR_LOG_ERROR("invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("Enable: %d\n",fac_enable);
	mlx90632_set_factory(data, fac_enable);
	return size;
}

static ssize_t factory_test_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data =  dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (data->fac_enable)?1:0);
}

static ssize_t calibrate_ha_store(struct device *dev, struct device_attribute *attr,
                                  const char *buf, size_t size)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);
	int err;
	s32 value;

	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}
	err = kstrtoint(buf, 0, &value);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}
	err = mlx_set_ha(data, value);
	if (err < 0) {
		SENSOR_LOG_ERROR("stk als calibrate: failed.\n");
		return err;
	}
	if(data->fac_Ha_set == true && data->fac_Hb_set == true) {
		data->fac_calibrated = true;
		SENSOR_LOG_INFO("Hb Ha calibrated,Ha %d, Hb %d\n", data->mlx_Ha, data->mlx_Hb);
	}

	return size;
}

static ssize_t calibrate_ha_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data =  dev_get_drvdata(dev);
	if(data->fac_calibrated == false) {
		SENSOR_LOG_ERROR("mlx not calibrate");
		return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->mlx_Ha);
}

static ssize_t calibrate_hb_store(struct device *dev, struct device_attribute *attr,
                                  const char *buf, size_t size)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);
	int err;
	s32 value;

	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}
	err = kstrtoint(buf, 0, &value);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}
	err = mlx_set_hb(data, value);
	if (err < 0) {
		SENSOR_LOG_ERROR("Hb calibrate: failed.\n");
		return err;
	}
	if(data->fac_Hb_set == true && data->fac_Ha_set == true) {
		data->fac_calibrated = true;
		SENSOR_LOG_INFO("Hb Ha calibrated,Ha %d, Hb %d\n", data->mlx_Ha, data->mlx_Hb);
	}
	return size;
}

static ssize_t calibrate_hb_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct mlx90632_data *data =  dev_get_drvdata(dev);
	if(data->fac_calibrated == false) {
		SENSOR_LOG_ERROR("mlx not calibrate");
		return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->mlx_Hb);
}


static struct device_attribute attrs_mlx90632_device[] = {
	__ATTR(init,0664,mlx90632_init_show, mlx90632_init_store),
	__ATTR(enable_body,0664,mlx90632_enable_body_show, mlx90632_enable_body_store),
	__ATTR(enable_object,0664,mlx90632_enable_object_show, mlx90632_enable_object_store),
	__ATTR(object_temp, 0444, object_temp_show, NULL),
	__ATTR(internal_temp, 0444, internal_temp_show, NULL),
	__ATTR(factory_test, 0664, factory_test_show, factory_test_store),
	__ATTR(calibrate_ha, 0664, calibrate_ha_show, calibrate_ha_store),
	__ATTR(calibrate_hb, 0664, calibrate_hb_show, calibrate_hb_store),
};

static int mlx90632_sleep(struct mlx90632_data *data)
{
	regcache_mark_dirty(data->regmap);

	dev_dbg(&data->client->dev, "Requesting sleep");
	return mlx90632_pwr_set_sleep_step(data->regmap);
}

static int mlx90632_wakeup(struct mlx90632_data *data)
{
	int ret;

	ret = regcache_sync(data->regmap);
	if (ret < 0) {
		dev_err(&data->client->dev,
		        "Failed to sync regmap registers: %d\n", ret);
		return ret;
	}

	dev_dbg(&data->client->dev, "Requesting wake-up\n");
	return mlx90632_pwr_continuous(data->regmap);
}
/*
int mlx90632_parse_dts(struct device *dev, struct mlx90632_data *mlx90632)
{
	int rc = 0;
	struct device_node *np = dev->of_node;

	mlx90632->pwr_vdd_gpio = of_get_named_gpio(np, "melexis,mlx_pwr_vdd", 0);
	dev_err(dev, "pwr_vdd_gpio %d\n", mlx90632->pwr_vdd_gpio);
	if (mlx90632->pwr_vdd_gpio < 0) {
		dev_err(dev, "falied to get mlx_pwr_vdd gpio!\n");
		return mlx90632->pwr_vdd_gpio;
	}
	gpio_free(mlx90632->pwr_vdd_gpio);
	rc = devm_gpio_request(dev, mlx90632->pwr_vdd_gpio, "mlx_pwr_vdd");
	if (rc) {
		dev_err(dev, "failed to request mlx_pwr_vdd gpio, rc = %d\n", rc);
		goto err_avdd;
	}
	gpio_direction_output(mlx90632->pwr_vdd_gpio, 1);

	if (gpio_is_valid(mlx90632->pwr_vdd_gpio)) {
		dev_err(dev, "vdd power gpio_is_valid \n");
		gpio_set_value(mlx90632->pwr_vdd_gpio, 1);
	}
	dev_err(dev, "vdd power %s\n", gpio_get_value(mlx90632->pwr_vdd_gpio)? "on" : "off");
	return 0;

err_avdd:
	return rc;
}
*/
static int mlx90632_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
	struct mlx90632_data *mlx90632;
	struct regmap *regmap;
	int ret;
	unsigned int read;
	dev_err(&client->dev, "MLX90632 Probe\n");

	mlx90632 = kzalloc(sizeof(struct mlx90632_data),GFP_KERNEL);
	if(!mlx90632) {
		dev_err(&client->dev, "failed to allocate mlx90632_data\n");
		return -ENOMEM;
	}

	regmap = devm_regmap_init_i2c(client, &mlx90632_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}
	i2c_set_clientdata(client, mlx90632);
	mlx90632->client = client;
	mlx90632->regmap = regmap;
	mutex_init(&mlx90632->lock);
	mutex_init(&mlx90632->data_lock);

	ret = mlx90632_wakeup(mlx90632);
	if (ret < 0) {
		dev_err(&client->dev, "Wakeup failed: %d\n", ret);
		return ret;
	}

	ret = regmap_read(mlx90632->regmap, MLX90632_EE_VERSION, &read);
	if (ret < 0) {
		dev_err(&client->dev, "read of version failed: %d\n", ret);
		return ret;
	}
	if (read == MLX90632_ID_MEDICAL) {
		dev_dbg(&client->dev,
		        "Detected Medical EEPROM calibration %x\n", read);
	} else if (read == MLX90632_ID_CONSUMER) {
		dev_dbg(&client->dev,
		        "Detected Consumer EEPROM calibration %x\n", read);
	} else if ((read & MLX90632_DSP_MASK) == MLX90632_DSP_VERSION) {
		dev_dbg(&client->dev,
		        "Detected Unknown EEPROM calibration %x\n", read);
	} else {
		dev_err(&client->dev,
		        "Wrong DSP version %x (expected %x)\n",
		        read, MLX90632_DSP_VERSION);
		return -EPROTONOSUPPORT;
	}

	mlx90632->fac_enable = false;
	mlx90632->mlx_enable = false;
	mlx90632->fac_calibrated = false;
	mlx90632->fac_Ha_set = false;
	mlx90632->fac_Hb_set = false;
	mlx90632->emissivity = 1000;
	mlx90632->mlx_Ha = 0;
	mlx90632->mlx_Hb = 0;

	mlx90632->mlx_wq = create_singlethread_workqueue("mlx_work_queue");
	INIT_WORK(&mlx90632->mlx_work, mlx90632_temp_polling_work_handler);
	hrtimer_init(&mlx90632->mlx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mlx90632->mlx_timer.function = mlx90632_temp_timer_func;
	mlx90632->mlx_poll_delay = ns_to_ktime(550 * NSEC_PER_MSEC);

	pm_runtime_disable(&client->dev);
	ret = pm_runtime_set_active(&client->dev);
	if (ret < 0) {
		mlx90632_sleep(mlx90632);
		return ret;
	}
	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, MLX90632_SLEEP_DELAY_MS);
	pm_runtime_use_autosuspend(&client->dev);

	/* allocate temperature sys class node */
	mlx90632_class = class_create(THIS_MODULE, MLX90632_DRV_NAME);
	alloc_chrdev_region(&mlx90632_dev_t, 0, 1, MLX90632_DRV_NAME);
	mlx90632->mlx90632_data_dev = device_create(mlx90632_class, 0,
	                              mlx90632_dev_t, 0, MLX90632_DRV_NAME);
	if (IS_ERR(mlx90632->mlx90632_data_dev)) {
		SENSOR_LOG_ERROR("device_create mlx90632 failed\n");
		goto create_mlx90632_dev_failed;
	}
	dev_set_drvdata(mlx90632->mlx90632_data_dev, mlx90632);

	ret = sensor_create_sysfs_interfaces(mlx90632->mlx90632_data_dev, attrs_mlx90632_device, ARRAY_SIZE(attrs_mlx90632_device));
	if (ret < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_failed;
	}

	/* allocate temperature input device */
	mlx90632->mlx90632_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(mlx90632->mlx90632_input_dev)) {
		ret = -ENOMEM;
		SENSOR_LOG_ERROR("could not allocate virtual input device\n");
		goto allocate_input_device_failed;
	}
	input_set_drvdata(mlx90632->mlx90632_input_dev, mlx90632);
	mlx90632->mlx90632_input_dev->name = INPUT_NAME_MLX90632;
	mlx90632->mlx90632_input_dev->id.bustype = BUS_I2C;

	set_bit(EV_REL, mlx90632->mlx90632_input_dev->evbit);
	set_bit(REL_X, mlx90632->mlx90632_input_dev->relbit);
	set_bit(REL_Y, mlx90632->mlx90632_input_dev->relbit);

	ret = input_register_device(mlx90632->mlx90632_input_dev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("could not register virtual_als input device\n");
		ret = -ENOMEM;
		goto register_input_device_failed;
	}

	return 0;
register_input_device_failed:
	input_unregister_device(mlx90632->mlx90632_input_dev);
allocate_input_device_failed:
	input_free_device(mlx90632->mlx90632_input_dev);
create_sysfs_interface_failed:
	sensor_remove_sysfs_interfaces(mlx90632->mlx90632_data_dev, attrs_mlx90632_device, ARRAY_SIZE(attrs_mlx90632_device));
create_mlx90632_dev_failed:
	mlx90632->mlx90632_data_dev = NULL;
	device_destroy(mlx90632_class, mlx90632_dev_t);
	class_destroy(mlx90632_class);
	return ret;

}

static int mlx90632_remove(struct i2c_client *client)
{
	struct mlx90632_data *mlx90632 = i2c_get_clientdata(client);

	input_unregister_device(mlx90632->mlx90632_input_dev);
	input_free_device(mlx90632->mlx90632_input_dev);
	sensor_remove_sysfs_interfaces(mlx90632->mlx90632_data_dev, attrs_mlx90632_device, ARRAY_SIZE(attrs_mlx90632_device));
	mlx90632->mlx90632_data_dev = NULL;
	device_destroy(mlx90632_class, mlx90632_dev_t);
	class_destroy(mlx90632_class);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	mlx90632_sleep(mlx90632);

	return 0;
}

static const struct i2c_device_id mlx90632_id[] = {
	{ "mlx90632", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mlx90632_id);

static const struct of_device_id mlx90632_of_match[] = {
	{ .compatible = "melexis,mlx90632" },
	{ }
};
MODULE_DEVICE_TABLE(of, mlx90632_of_match);

static int __maybe_unused mlx90632_pm_suspend(struct device *dev)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);

	return mlx90632_sleep(data);
}

static int __maybe_unused mlx90632_pm_resume(struct device *dev)
{
	struct mlx90632_data *data = dev_get_drvdata(dev);

	return mlx90632_wakeup(data);
}

static UNIVERSAL_DEV_PM_OPS(mlx90632_pm_ops, mlx90632_pm_suspend,
                            mlx90632_pm_resume, NULL);

static struct i2c_driver mlx90632_driver = {
	.driver = {
		.name	= "mlx90632",
		.of_match_table = mlx90632_of_match,
		.pm	= &mlx90632_pm_ops,
	},
	.probe = mlx90632_probe,
	.remove = mlx90632_remove,
	.id_table = mlx90632_id,
};
//module_i2c_driver(mlx90632_driver);

static int __init mlx90632_init(void)
{
	int ret;
	ret = i2c_add_driver(&mlx90632_driver);
	if (ret) {
		i2c_del_driver(&mlx90632_driver);
		return ret;
	}
	return 0;
}

static void __exit mlx90632_exit(void)
{
	i2c_del_driver(&mlx90632_driver);
}

module_init(mlx90632_init);
module_exit(mlx90632_exit);
MODULE_AUTHOR("Crt Mori <cmo@melexis.com>");
MODULE_DESCRIPTION("Melexis MLX90632 contactless Infra Red temperature sensor driver");
MODULE_LICENSE("GPL v2");
