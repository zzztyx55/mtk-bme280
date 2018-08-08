/* BOSCH Pressure,Temperature,Humitidy Sensor bme280 Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/math64.h>

#include "bme280.h"

#include "barometer.h"
#include "humidity.h"
#include "temperature.h"

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("BME280 I2C Driver for MTK");
MODULE_AUTHOR("tianyx < zzztyx55@sina.com >");

//#define EN_DEBUG

#define DBG_TAG                "bme280_3in1"

#ifdef EN_DEBUG
    #define dbg_mesg(format, args...)										\
		    do{																\
			    printk(KERN_ERR DBG_TAG": %s() _%d_: " format	            \
					    , __FUNCTION__										\
					    , __LINE__											\
					    , ## args);											\
		    }while(0)
#else
    #define dbg_mesg(format, args...)     ((void)0)
#endif

#define alert_mesg(format, args...)										\
		do{																\
			printk(KERN_ERR DBG_TAG": %s() _%d_: " format	            \
					, __FUNCTION__										\
					, __LINE__											\
					, ## args);											\
		}while(0)



static bool is_i2c_registed = false;

/*
 * Structure that holds bme280 calibration data - this data is loaded into
 * the device at production and it is different for each device, this structure
 * is populated in the probe function.
 *
 * */
struct bme280_calibration_t{
	unsigned short dig_T1;
      	signed short dig_T2;
	signed short dig_T3;
	unsigned short dig_P1;
	signed short dig_P2;
	signed short dig_P3;
	signed short dig_P4;
	signed short dig_P5;
	signed short dig_P6;
	signed short dig_P7;
	signed short dig_P8;
	signed short dig_P9;
	unsigned char dig_H1;
	signed short dig_H2;
	unsigned char dig_H3;
	signed short dig_H4;
	signed short dig_H5;
	signed char dig_H6;
};

/* Structure that holds bme280 configuration */
struct bme280_configuration_t{
	unsigned char ctrl_hum;
	unsigned char ctrl_meas;
	unsigned char config;
};

/* Structure that holds configuration and client data for the device */
struct bme280_data_t{
	struct i2c_client *client;
	struct bme280_calibration_t *cal_data;
	struct bme280_configuration_t *cfg_data;

    /* calculated temperature correction coefficient */
    s32 t_fine;
};

/* Global variables */
static struct bme280_calibration_t bme280_calibration;
static struct bme280_configuration_t bme280_configuration =
					{
					.ctrl_hum = CTRL_HUM_OSRS_H(1),
					.ctrl_meas = CTRL_MEAS_OSRS_T(1) | CTRL_MEAS_OSRS_P(1),
					.config = 0
					};
static struct bme280_data_t bme280_data =
					{
					.client = NULL,
					.cal_data = &bme280_calibration,
					.cfg_data = &bme280_configuration,
					.t_fine = 0
					};



extern int baro_register_control_path(struct baro_control_path *ctl);
extern int baro_register_data_path(struct baro_data_path *data);
extern int hmdy_register_control_path(struct hmdy_control_path *ctl);
extern int hmdy_register_data_path(struct hmdy_data_path *data);
extern int temp_register_control_path(struct temp_control_path *ctl);
extern int temp_register_data_path(struct temp_data_path *data);


/* Helper functions definitions */
/*
 * This helper function is used to obtain the calibration
 * parameters and populate the calibration structure
 *
 * parameter | Register address |   bit
 * ----------|------------------|----------------
 * dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 * dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 * dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 * dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 * dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 * dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 * dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 * dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 * dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 * dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 * dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 * dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 * dig_H1    |         0xA1     | from 0 to 7
 * dig_H2    |  0xE1 and 0xE2   | from 0 : 7 to 8: 15
 * dig_H3    |         0xE3     | from 0 to 7
 * dig_H4    |0xE4 and 0xE5[3:0]| from 4 : 11 to 0: 3
 * dig_H5    |0xE5[7:4] and 0xE6| from 0 : 3 to 4: 11
 * dig_H6    |         0xE7     | from 0 to 7
 *
 * */
static int bme280_get_calibration(struct bme280_calibration_t *calibration){
	u8 calib_data_0_25[CALIBRATION_REG_SIZE_0];
	u8 calib_data_26_41[CALIBRATION_REG_SIZE_26];
	s32 tmp;
	u8 i = 0;

	/*
	 * Read calibration data, the data is arranged in two chunks
	 * from 0x88 (calib0) to 0xA1 (calib25) and
	 * from 0xE1 (calib26) to 0xF0 (calib41)
	 * Use read byte instead of emulated block data read because 4.1
	 * does not support emulated block read
	 * */
	while(i < CALIBRATION_REG_SIZE_0){
		tmp = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_CALIB00 + i);
		if(tmp < 0){
			goto out;
		}
		calib_data_0_25[i] = (u8) (tmp & 0xFF);
		i++;
	}

	i = 0;
	while(i < CALIBRATION_REG_SIZE_26){
		tmp = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_CALIB26 + i);
		if(tmp < 0){
			goto out;
		}
		calib_data_26_41[i] = (u8) (tmp & 0xFF);
		i++;
	}

	/* Fill calibration structure from obtained parameters */
	bme280_data.cal_data->dig_T1 = (unsigned short)((calib_data_0_25[1] << 8) | calib_data_0_25[0]);
	bme280_data.cal_data->dig_T2 = (short) ((calib_data_0_25[3] << 8) | calib_data_0_25[2]);
	bme280_data.cal_data->dig_T3 = (short) ((calib_data_0_25[5] << 8) | calib_data_0_25[4]);
	bme280_data.cal_data->dig_P1 = (unsigned short) ((calib_data_0_25[7] << 8) | calib_data_0_25[6]);
	bme280_data.cal_data->dig_P2 = (short) ((calib_data_0_25[9] << 8) | calib_data_0_25[8]);
	bme280_data.cal_data->dig_P3 = (short) ((calib_data_0_25[11] << 8) | calib_data_0_25[10]);
	bme280_data.cal_data->dig_P4 = (short) ((calib_data_0_25[13] << 8) | calib_data_0_25[12]);
	bme280_data.cal_data->dig_P5 = (short) ((calib_data_0_25[15] << 8) | calib_data_0_25[14]);
	bme280_data.cal_data->dig_P6 = (short) ((calib_data_0_25[17] << 8) | calib_data_0_25[16]);
	bme280_data.cal_data->dig_P7 = (short) ((calib_data_0_25[19] << 8) | calib_data_0_25[18]);
	bme280_data.cal_data->dig_P8 = (short) ((calib_data_0_25[21] << 8) | calib_data_0_25[20]);
	bme280_data.cal_data->dig_P9 = (short) ((calib_data_0_25[23] << 8) | calib_data_0_25[22]);
	bme280_data.cal_data->dig_H1 = calib_data_0_25[24];
	bme280_data.cal_data->dig_H2 = (short) ((calib_data_26_41[1] << 8) | calib_data_26_41[0]);
	bme280_data.cal_data->dig_H3 = calib_data_26_41[2];
	bme280_data.cal_data->dig_H4 = (short) ((calib_data_26_41[4] & 0x0F ) |
			(calib_data_26_41[3] << 4));
	bme280_data.cal_data->dig_H5 = (short) ((calib_data_26_41[5] << 4) |
			( (calib_data_26_41[4] & 0xF0) >> 4 ) );
	bme280_data.cal_data->dig_H6 = (s8) calib_data_26_41[6];

	return 0;
out:
	return tmp;
}

static int bme280_set_configuration(struct bme280_configuration_t *configuration)
{
	int tmp;

	/* Configure CTRL_HUM  */
	tmp = i2c_smbus_write_byte_data(bme280_data.client, R_BME280_CTRL_HUM,
			bme280_data.cfg_data->ctrl_hum);
	if(tmp < 0){
		dbg_mesg("%s: Unable to write CTRL_HUM.\n", BME_DEV_NAME);
		goto out;
	}

	/*
	 * Configure CTRL_MEAS
	 * changes to CTRL_HUM become effective only after writing to CTRL_MEAS
	 * */
	tmp = i2c_smbus_write_byte_data(bme280_data.client, R_BME280_CTRL_MEAS,
			bme280_data.cfg_data->ctrl_meas);
	if(tmp < 0){
		dbg_mesg("%s: Unable to write CTRL_MEAS.\n", BME_DEV_NAME);
		goto out;
	}

	/* Set CONFIG register */
	tmp = i2c_smbus_write_byte_data(bme280_data.client, R_BME280_CONFIG,
			bme280_data.cfg_data->config);
	if(tmp < 0){
		dbg_mesg("%s: Unable to write CONFIG.\n", BME_DEV_NAME);
		goto out;
	}

out:
	return tmp;
}

#if 0
/*******************************************************************************
 *	Description: *//**\brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 *mode : Pointer holding the mode value.
 *	0x00			->	BME280_SLEEP_MODE
 *	0x01 and 0x02	->	BME280_FORCED_MODE
 *	0x03			->	BME280_NORMAL_MODE
 *
 *
 *  \return : results of bus communication function
 *
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_mode(u8 mode)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	u8 v_mode_u8r = BME280_Zero_U8X;
	u8 prev_pow_mode = BME280_Zero_U8X;
	u8 pre_ctrl_hum_value = BME280_Zero_U8X;
	u8 pre_config_value = BME280_Zero_U8X;
	u8 v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			if (mode < BME280_Four_U8X) {
				v_mode_u8r = p_bme280->ctrl_meas_reg;
				v_mode_u8r =
				BME280_SET_BITSLICE(v_mode_u8r,
				BME280_CTRLMEAS_REG_MODE, mode);
				comres += bme280_get_mode(&prev_pow_mode);
				if (prev_pow_mode != BME280_SLEEP_MODE) {
					comres += bme280_set_softreset();
					p_bme280->delay_msec(BME280_3MS_DELAY);
					/* write previous value of
					configuration register*/
					pre_config_value = p_bme280->config_reg;
					comres += bme280_write_register(
						BME280_CONFIG_REG,
					&pre_config_value, 1);
					/* write previous value of
					humidity oversampling*/
					pre_ctrl_hum_value =
					p_bme280->ctrl_hum_reg;
					comres += bme280_write_register(
					BME280_CTRLHUM_REG,
					&pre_ctrl_hum_value, 1);
					/* write previous and updated value of
					control measurement register*/
					comres += bme280_write_register(
					BME280_CTRLMEAS_REG,
					&v_mode_u8r, 1);
				} else {
					comres +=
					p_bme280->BME280_BUS_WRITE_FUNC(
					p_bme280->dev_addr,
					BME280_CTRLMEAS_REG_MODE__REG,
					&v_mode_u8r, 1);
				}
				/* read the control measurement register value*/
				comres += bme280_read_register(
					BME280_CTRLMEAS_REG,
				&v_data_u8r, 1);
				p_bme280->ctrl_meas_reg = v_data_u8r;
				/* read the control humidity register value*/
				comres += bme280_read_register(
					BME280_CTRLHUM_REG,
				&v_data_u8r, 1);
				p_bme280->ctrl_hum_reg = v_data_u8r;
				/* read the config register value*/
				comres += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8r, 1);
				p_bme280->config_reg = v_data_u8r;
			} else {
			comres = E_BME280_OUT_OF_RANGE;
			}
		}
	return comres;
}
#endif

static int bme280_compensate_T_int32(int adc_t)
{
	int v_x1_u32r = 0;
	int v_x2_u32r = 0;
	int temperature = 0;

	v_x1_u32r  = ((((adc_t >> 3) - ((int)
	bme280_data.cal_data->dig_T1 << 1))) *
	((int)bme280_data.cal_data->dig_T2)) >> 11;
	v_x2_u32r  = (((((adc_t >> 4) -
	((int)bme280_data.cal_data->dig_T1)) * ((adc_t >> 4) -
	((int)bme280_data.cal_data->dig_T1))) >> 12) *
	((int)bme280_data.cal_data->dig_T3)) >> 14;
	bme280_data.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (bme280_data.t_fine * 5 + 128) >> 8;
	return temperature;
}

static int bme280_compensate_P_int32(int adc_p)
{
	int v_x1_u32r = 0;
	int v_x2_u32r = 0;
	int pressure = 0;

	v_x1_u32r = (((s32)bme280_data.t_fine) >> 1) -
	(s32)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
	((s32)bme280_data.cal_data->dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
	((s32)bme280_data.cal_data->dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) +
	(((s32)bme280_data.cal_data->dig_P4) << 16);
	v_x1_u32r = (((bme280_data.cal_data->dig_P3 * (((v_x1_u32r >> 2) *
	(v_x1_u32r >> 2)) >> 13)) >> 3) +
	((((s32)bme280_data.cal_data->dig_P2) *
	v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) *
	((s32)bme280_data.cal_data->dig_P1))	>> 15);
	pressure = (((u32)(((s32)1048576) - adc_p) -
	(v_x2_u32r >> 12))) * 3125;
	if (pressure < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != 0)
			pressure = (pressure << 1) / ((u32)v_x1_u32r);
		else
			return 0;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != 0)
			pressure = (pressure / (u32)v_x1_u32r) * 2;
		else
			return 0;

	v_x1_u32r = (((s32)bme280_data.cal_data->dig_P9) *
		((s32)(((pressure >> 3) * (pressure >> 3)) >> 13)))
		>> 12;
	v_x2_u32r = (((s32)(pressure >> 2)) *
		((s32)bme280_data.cal_data->dig_P8)) >> 13;
	pressure = (u32)((s32)pressure +
		((v_x1_u32r + v_x2_u32r + bme280_data.cal_data->dig_P7) >> 4));

	return pressure;
}

static int bme280_compensate_H_int32(int adc_h)
{
	int v_x1_u32r;
	v_x1_u32r = (bme280_data.t_fine - ((s32)76800));
	v_x1_u32r = (((((adc_h << 14) -
	(((s32)bme280_data.cal_data->dig_H4) << 20) -
	(((s32)bme280_data.cal_data->dig_H5) * v_x1_u32r)) +
	((s32)16384)) >> 15) *
	(((((((v_x1_u32r *
	((s32)bme280_data.cal_data->dig_H6)) >> 10) *
	(((v_x1_u32r * ((s32)bme280_data.cal_data->dig_H3)) >> 11) +
	((s32)32768))) >> 10) +
	((s32)2097152)) *
	((s32)bme280_data.cal_data->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) *
	(v_x1_u32r >> 15)) >> 7) *
	((s32)bme280_data.cal_data->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (u32)(v_x1_u32r>>12);
}

static int bme280_read_uputuh(int * data_ut, int * data_up, int * data_uh)
{
	int tmp, working_mode;
	u32 i = 0;
	//u8 press_st[MAX_STRING_SIZE];
	//u8 temp_st[MAX_STRING_SIZE];
	//u8 hum_st[MAX_STRING_SIZE];
	//u8 final_st[MAX_STRING_SIZE * 3];
	u32 press, temp;
	u16 hum;
	u8 data_readout[DATA_READOUT_SIZE];

	 dbg_mesg("++\n");

	/* Check working mode */
	working_mode = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_CTRL_MEAS);
	if(working_mode < 0){
		tmp = working_mode;
		return -1;
	}

	/* If in sleep mode we need to do a force read */
	if((working_mode & 0b11) == 0){
		tmp = i2c_smbus_write_byte_data(bme280_data.client, R_BME280_CTRL_MEAS,
				(working_mode | 0b10));
		if(tmp < 0){
			return -1;
		}
	}

	/* Check status flag */
	do{
		tmp = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_STATUS);
		if(tmp < 0){
			return -1;
		}
	}while(tmp & STATUS_MEASURING_RUNNING);

	/*
	 * Perform raw read, it is adviced to perform a burst read
	 * from 0xF7 (PRESS_MSB) to 0xFE (HUM_LSB)
	 * Use read byte instead of emulated block data read because 4.1
	 * does not support emulated block read
	 * */
	while(i < DATA_READOUT_SIZE){
		tmp = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_PRESS_MSB + i);
		if(tmp < 0){
			return -1;
		}
		data_readout[i] =(u8) (tmp & 0xFF);
		i++;
	}

	/* Process raw data */
	press = (data_readout[0] << 12) | (data_readout[1] << 4) | (data_readout[2] >> 4);
	temp = (data_readout[3] << 12) | (data_readout[4] << 4) | (data_readout[5] >> 4);
	hum = (data_readout[6] << 8) |  data_readout[7];

	*data_ut = temp;
    *data_up = press;
    *data_uh = hum;

    dbg_mesg("raw data: temp=%d, press=%d, hum=%d\n", temp, press, hum);

    return 0;
}



static int bme280_get_pressure(int* value, int * status)
{
	int press, temp;
	int hum;
    int ret;

    dbg_mesg("++\n");

    ret = bme280_read_uputuh(&temp, &press, &hum);
    if(ret < 0){
        dbg_mesg("bme280_read_uputuh fail\n");
        return ret;
    }

    /* Compensate obtained results */
    /* temperature first*/
    temp = bme280_compensate_T_int32(temp);
    /* need temperature data*/
    press = bme280_compensate_P_int32(press);
    //hum = bme280_compensate_H_int32(hum);

    *value = press;
    *status = 0;

    dbg_mesg("real data: temp=%d, press=%d\n", temp, press);

	return 0;
}

static int bme280_get_temperature(int* value, int * status)
{
	int press, temp;
	int hum;
    int ret;

    dbg_mesg("++\n");

    ret = bme280_read_uputuh(&temp, &press, &hum);
    if(ret < 0){
        dbg_mesg("bme280_read_uputuh fail\n");
        return ret;
    }

    /* Compensate obtained results */
    /* temperature first*/
    temp = bme280_compensate_T_int32(temp);
    /* need temperature data*/
    //press = bme280_compensate_P_int32(press);
    //hum = bme280_compensate_H_int32(hum);

    *value = temp;
    *status = 0;

    dbg_mesg("real data: temp=%d\n", temp);

	return 0;
}

static int bme280_get_humidity(int* value, int * status)
{
	int press, temp;
	int hum;
    int ret;

    dbg_mesg("++\n");

    ret = bme280_read_uputuh(&temp, &press, &hum);
    if(ret < 0){
        dbg_mesg("bme280_read_uputuh fail\n");
        return ret;
    }

    dbg_mesg("raw data: hum=0x%x\n",  hum);

    /* Compensate obtained results */
    /* temperature first*/
    temp = bme280_compensate_T_int32(temp);
    /* need temperature data*/
    //press = bme280_compensate_P_int32(press);
    hum = bme280_compensate_H_int32(hum);

    *value = hum;
    *status = 0;

	return 0;
}

static int bme280_enable_nodata(int en)
{
    dbg_mesg("++, en=%d\n", en);
    return 0;
}

static int bme280_set_delay(u64 ns)
{
    dbg_mesg("++\n");
 	return 0;
}

static int bme280_open_report_data(int open)
{
    dbg_mesg("++\n");
 	return 0;
}


static int bme_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	int tmp, chip_id;

    dbg_mesg("++\n");

	/* Check if adapter supports the functionality we need */
	adapter = client->adapter;
	tmp = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA);
	if(!tmp)
		goto err_out;

	dbg_mesg("i2c_check_functionality ok\n");

	/* Get chip_id */
	chip_id = i2c_smbus_read_byte_data(client, R_BME280_CHIP_ID);
	if(BME280_CHIP_ID == chip_id)
	{
		alert_mesg("Client ID (0x%x) match chip bme280: 3 in 1.\n", chip_id);
	}
	else if(BMP280_CHIP_ID == chip_id)
	{
		alert_mesg("Client ID (0x%x) match chip bmp280: 2 in 1.\n", chip_id);
	}
	else
	{
		alert_mesg("Client ID (0x%x), does not match chip bmp280 or bme280.\n", chip_id);
		tmp = -1;
		goto err_out;
	}

	dbg_mesg("chip_id ok\n");

	bme280_data.client = client;

	/* Get calibration parameters */
	bme280_get_calibration(bme280_data.cal_data);

	dbg_mesg("bme280_get_calibration ok\n");

	/* Set configuration */
	tmp = bme280_set_configuration(bme280_data.cfg_data);
	if(tmp < 0){
		goto err_out;
	}

	dbg_mesg("probe success!\n");

	is_i2c_registed = true;

	return 0;

err_out:
	return tmp;
}

static int bme_i2c_remove(struct i2c_client *client) {

    return 0;
}

static int bme_i2c_detect(struct i2c_client *client,
                          struct i2c_board_info *info) {
    strcpy(info->type, BME_DEV_NAME);
    return 0;
}

static const struct i2c_device_id bme_i2c_id[] = {
    { BME_DEV_NAME, 0 },
    {}
};

#ifdef CONFIG_OF
static const struct of_device_id bme280_3in1_of_match[] = {
	{.compatible = "mediatek,bme280_3in1"},
	{},
};
#endif

static struct i2c_driver bme_i2c_driver = {
    .driver = {
        .owner  =   THIS_MODULE,
        .name   =   BME_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = bme280_3in1_of_match,
#endif
    },
    .probe  =   bme_i2c_probe,
    .remove =   bme_i2c_remove,
    .detect =   bme_i2c_detect,
    //.suspend    =   bme_suspend,
    //.resume =   bme_resume,
    .id_table   =   bme_i2c_id,
};



static int  bme280_pressure_init(void)
{
	struct baro_control_path pressure_ctl = { 0 };
    struct baro_data_path pressure_data = { 0 };
    int ret = 0;

    dbg_mesg("++, is_i2c_registed = %s\n", is_i2c_registed ? "yes" : "no");

 	if(!is_i2c_registed)
 	{
		if (i2c_add_driver(&bme_i2c_driver)) {
			dbg_mesg("add driver error\n");
			return -1;  // why cannot get here
		}

		//is_i2c_registed = true;
	}

	if(!is_i2c_registed)
 	{
		alert_mesg("cannot get i2c client device: bme280\n");
		return -1;
	}

    pressure_ctl.open_report_data = bme280_open_report_data;
    pressure_ctl.enable_nodata = bme280_enable_nodata;
    pressure_ctl.set_delay = bme280_set_delay;
    pressure_ctl.is_report_input_direct = false;
    pressure_ctl.is_support_batch = false;
    pressure_ctl.is_use_common_factory = false;
    ret = baro_register_control_path(&pressure_ctl);
    if (ret) {
		return ret;
    }

    pressure_data.get_data = bme280_get_pressure;
    pressure_data.vender_div = 100;
    ret = baro_register_data_path(&pressure_data);
    if (ret) {
		return ret;
    }

    return 0;
}

static int  bme280_hmdy_init(void)
{
    struct hmdy_control_path hmdy_ctl = { 0 };
    struct hmdy_data_path hmdy_data = { 0 };
    int ret = 0;
	int chip_id;

    dbg_mesg("++, is_i2c_registed = %s\n", is_i2c_registed ? "yes" : "no");

 	if(!is_i2c_registed)
 	{
		if (i2c_add_driver(&bme_i2c_driver)) {
			dbg_mesg("add driver error\n");
			return -1; // why cannot get here
		}

		//is_i2c_registed = true;
	}

	if(!is_i2c_registed)
 	{
		alert_mesg("cannot get i2c client device: bme280\n");
		return -1;
	}

	chip_id = i2c_smbus_read_byte_data(bme280_data.client, R_BME280_CHIP_ID);
	if(BME280_CHIP_ID == chip_id)
	{
        hmdy_ctl.open_report_data = bme280_open_report_data;
        hmdy_ctl.enable_nodata = bme280_enable_nodata;
        hmdy_ctl.set_delay = bme280_set_delay;
        hmdy_ctl.is_report_input_direct = false;
        hmdy_ctl.is_support_batch = false;
        hmdy_ctl.is_use_common_factory = false;
        ret = hmdy_register_control_path(&hmdy_ctl);
        if (ret) {
			return ret;
        }

        hmdy_data.get_data = bme280_get_humidity;
        hmdy_data.vender_div = 1000;
        ret = hmdy_register_data_path(&hmdy_data);
        if (ret) {
			return ret;
        }
	}
	else if(BMP280_CHIP_ID == chip_id)
	{
		dbg_mesg("warmming: bmp280 only support temperature/pressure, donot support humidity yet.");
	}

	return 0;
}

static int  bme280_temp_init(void)
{
    struct temp_control_path temp_ctl = { 0 };
    struct temp_data_path temp_data = { 0 };
    int ret = 0;

    dbg_mesg("++, is_i2c_registed = %s\n", is_i2c_registed ? "yes" : "no");

 	if(!is_i2c_registed)
 	{
		if (i2c_add_driver(&bme_i2c_driver)) {
			dbg_mesg("add driver error\n");
			return -1;  // why cannot get here
		}

		//is_i2c_registed = true;
	}

	if(!is_i2c_registed)
 	{
		alert_mesg("cannot get i2c client device: bme280\n");
		return -1;
	}

	temp_ctl.open_report_data = bme280_open_report_data;
    temp_ctl.enable_nodata = bme280_enable_nodata;
    temp_ctl.set_delay = bme280_set_delay;
    temp_ctl.is_report_input_direct = false;
    temp_ctl.is_support_batch = false;
    temp_ctl.is_use_common_factory = false;
    ret = temp_register_control_path(&temp_ctl);
    if (ret) {
		return ret;
    }

    temp_data.get_data = bme280_get_temperature;
    temp_data.vender_div = 100;
    ret = temp_register_data_path(&temp_data);
    if (ret) {
		return ret;
    }

    return 0;
}

static int bme280_remove(void)
{
    dbg_mesg("++, is_i2c_registed = %s\n", is_i2c_registed ? "yes" : "no");

 	if(!is_i2c_registed)
		return 0;

    i2c_del_driver(&bme_i2c_driver);
	is_i2c_registed = false;

    return 0;
}

static struct baro_init_info bme280_p_init_info =
{
    .name = "bme280_p",
    .init = bme280_pressure_init,
    .uninit = bme280_remove,
};

static struct hmdy_init_info bme280_h_init_info =
{
    .name = "bme280_h",
    .init = bme280_hmdy_init,
    .uninit = bme280_remove,
};

static struct temp_init_info bme280_t_init_info =
{
    .name = "bme280_t",
    .init = bme280_temp_init,
    .uninit = bme280_remove,
};

static int __init bme_init(void)
{
    dbg_mesg("++\n");

	is_i2c_registed = false;

    baro_driver_add(&bme280_p_init_info);
	hmdy_driver_add(&bme280_h_init_info);
	temp_driver_add(&bme280_t_init_info);

    return 0;
}

static void __exit bme_exit(void)
{
    dbg_mesg("++\n");
}

module_init(bme_init);
module_exit(bme_exit);



