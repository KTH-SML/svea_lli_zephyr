/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * This is a copy/rewrite from the ISM330DHCX driver that exist in Zephyr tree.
 * 
 * Path: 
 * zephyr/drivers/sensor/st/ism330dhcx
 * 
 * Author:
 * Kaj Munhoz Arfvidsson
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* ST Microelectronics ISM330DLC 6-axis IMU sensor driver
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ism330dlc.pdf
 */

#define DT_DRV_COMPAT st_ism330dlc

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/logging/log.h>

#include "ism330dlc.h"

LOG_MODULE_REGISTER(ISM330DLC, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t ism330dlc_odr_map[] = {0, 12, 26, 52, 104, 208, 416, 833,
					1660, 3330, 6660};

static int ism330dlc_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ism330dlc_odr_map); i++) {
		if (freq <= ism330dlc_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int ism330dlc_odr_to_freq_val(uint16_t odr)
{
	/* for valid index, return value from map */
	if (odr < ARRAY_SIZE(ism330dlc_odr_map)) {
		return ism330dlc_odr_map[odr];
	}

	/* invalid index, return last entry */
	return ism330dlc_odr_map[ARRAY_SIZE(ism330dlc_odr_map) - 1];
}

static const uint16_t ism330dlc_accel_fs_map[] = {2, 16, 4, 8};
static const uint16_t ism330dlc_accel_fs_sens[] = {1, 8, 2, 4};

static int ism330dlc_accel_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ism330dlc_accel_fs_map); i++) {
		if (range == ism330dlc_accel_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

/*
 * Following arrays are initialized in order to mimic
 * the ism330dlc_fs_g_t enum:
 *
 * typedef enum
 * {
 *   ISM330DLC_125dps = 2,
 *   ISM330DLC_250dps = 0,
 *   ISM330DLC_500dps = 4,
 *   ISM330DLC_1000dps = 8,
 *   ISM330DLC_2000dps = 12,
 *   ISM330DLC_4000dps = 1,
 * } ism330dlc_fs_g_t;
 */
static const uint16_t ism330dlc_gyro_fs_map[] = {
				250, 4000, 125, 0, 500,
				0, 0, 0, 1000,
				0, 0, 0, 2000
				};
static const uint16_t ism330dlc_gyro_fs_sens[] = {
				2, 32, 1, 0, 4,
				0, 0, 0, 8,
				0, 0, 0, 16
				};

static int ism330dlc_gyro_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ism330dlc_gyro_fs_map); i++) {
		if (range == ism330dlc_gyro_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static inline int ism330dlc_reboot(const struct device *dev)
{
	struct ism330dlc_data *data = dev->data;

	if (ism330dlc_boot_set(data->ctx, 1) < 0) {
		return -EIO;
	}

	/* Wait sensor turn-on time as per datasheet */
	k_busy_wait(35 * USEC_PER_MSEC);

	return 0;
}

static int ism330dlc_accel_set_fs_raw(const struct device *dev, uint8_t fs)
{
	struct ism330dlc_data *data = dev->data;

	if (ism330dlc_xl_full_scale_set(data->ctx, fs) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

static int ism330dlc_accel_set_odr_raw(const struct device *dev, uint8_t odr)
{
	struct ism330dlc_data *data = dev->data;

	if (ism330dlc_xl_data_rate_set(data->ctx, odr) < 0) {
		return -EIO;
	}

	data->accel_freq = ism330dlc_odr_to_freq_val(odr);

	return 0;
}

static int ism330dlc_gyro_set_fs_raw(const struct device *dev, uint8_t fs)
{
	struct ism330dlc_data *data = dev->data;

	if (ism330dlc_gy_full_scale_set(data->ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

static int ism330dlc_gyro_set_odr_raw(const struct device *dev, uint8_t odr)
{
	struct ism330dlc_data *data = dev->data;

	if (ism330dlc_gy_data_rate_set(data->ctx, odr) < 0) {
		return -EIO;
	}

	return 0;
}

static int ism330dlc_accel_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = ism330dlc_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (ism330dlc_accel_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}

static int ism330dlc_accel_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct ism330dlc_data *data = dev->data;

	fs = ism330dlc_accel_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (ism330dlc_accel_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->acc_gain = (ism330dlc_accel_fs_sens[fs] * GAIN_UNIT_XL);
	return 0;
}

static int ism330dlc_accel_config(const struct device *dev,
				   enum sensor_channel chan,
				   enum sensor_attribute attr,
				   const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330dlc_accel_range_set(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330dlc_accel_odr_set(dev, val->val1);
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330dlc_gyro_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = ism330dlc_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (ism330dlc_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set gyroscope sampling rate");
		return -EIO;
	}

	return 0;
}

static int ism330dlc_gyro_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct ism330dlc_data *data = dev->data;

	fs = ism330dlc_gyro_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (ism330dlc_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	data->gyro_gain = (ism330dlc_gyro_fs_sens[fs] * GAIN_UNIT_G);
	return 0;
}

static int ism330dlc_gyro_config(const struct device *dev,
				  enum sensor_channel chan,
				  enum sensor_attribute attr,
				  const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330dlc_gyro_range_set(dev, sensor_rad_to_degrees(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330dlc_gyro_odr_set(dev, val->val1);
	default:
		LOG_DBG("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330dlc_attr_set(const struct device *dev,
			       enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return ism330dlc_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return ism330dlc_gyro_config(dev, chan, attr, val);
#if defined(CONFIG_ISM330DLC_SENSORHUB)
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_PRESS:
	case SENSOR_CHAN_HUMIDITY:
		return ism330dlc_shub_config(dev, chan, attr, val);
#endif /* CONFIG_ISM330DLC_SENSORHUB */
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330dlc_sample_fetch_accel(const struct device *dev)
{
	struct ism330dlc_data *data = dev->data;
	int16_t buf[3];

	if (ism330dlc_acceleration_raw_get(data->ctx, buf) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->acc[0] = sys_le16_to_cpu(buf[0]);
	data->acc[1] = sys_le16_to_cpu(buf[1]);
	data->acc[2] = sys_le16_to_cpu(buf[2]);

	return 0;
}

static int ism330dlc_sample_fetch_gyro(const struct device *dev)
{
	struct ism330dlc_data *data = dev->data;
	int16_t buf[3];

	if (ism330dlc_angular_rate_raw_get(data->ctx, buf) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->gyro[0] = sys_le16_to_cpu(buf[0]);
	data->gyro[1] = sys_le16_to_cpu(buf[1]);
	data->gyro[2] = sys_le16_to_cpu(buf[2]);

	return 0;
}

#if defined(CONFIG_ISM330DLC_ENABLE_TEMP)
static int ism330dlc_sample_fetch_temp(const struct device *dev)
{
	struct ism330dlc_data *data = dev->data;
	int16_t buf;

	if (ism330dlc_temperature_raw_get(data->ctx, &buf) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->temp_sample = sys_le16_to_cpu(buf);

	return 0;
}
#endif

#if defined(CONFIG_ISM330DLC_SENSORHUB)
static int ism330dlc_sample_fetch_shub(const struct device *dev)
{
	if (ism330dlc_shub_fetch_external_devs(dev) < 0) {
		LOG_DBG("failed to read ext shub devices");
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_ISM330DLC_SENSORHUB */

static int ism330dlc_sample_fetch(const struct device *dev,
				   enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330dlc_sample_fetch_accel(dev);
#if defined(CONFIG_ISM330DLC_SENSORHUB)
		ism330dlc_sample_fetch_shub(dev);
#endif
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		ism330dlc_sample_fetch_gyro(dev);
		break;
#if defined(CONFIG_ISM330DLC_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		ism330dlc_sample_fetch_temp(dev);
		break;
#endif
	case SENSOR_CHAN_ALL:
		ism330dlc_sample_fetch_accel(dev);
		ism330dlc_sample_fetch_gyro(dev);
#if defined(CONFIG_ISM330DLC_ENABLE_TEMP)
		ism330dlc_sample_fetch_temp(dev);
#endif
#if defined(CONFIG_ISM330DLC_SENSORHUB)
		ism330dlc_sample_fetch_shub(dev);
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void ism330dlc_accel_convert(struct sensor_value *val, int raw_val,
					    uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in ug/LSB */
	/* Convert to m/s^2 */
	dval = (int64_t)(raw_val) * sensitivity;
	sensor_ug_to_ms2(dval, val);

}

static inline int ism330dlc_accel_get_channel(const struct device *dev, enum sensor_channel chan,
					       struct sensor_value *val, uint32_t sensitivity)
{
	struct ism330dlc_data *data = dev->data;
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ism330dlc_accel_convert(val, data->acc[0], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ism330dlc_accel_convert(val, data->acc[1], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ism330dlc_accel_convert(val, data->acc[2], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (i = 0; i < 3; i++) {
			ism330dlc_accel_convert(val++, data->acc[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ism330dlc_accel_channel_get(const struct device *dev, enum sensor_channel chan,
					struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;

	return ism330dlc_accel_get_channel(dev, chan, val, data->acc_gain);
}

static inline void ism330dlc_gyro_convert(struct sensor_value *val, int raw_val,
					   uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in udps/LSB */
	/* So, calculate value in 10 udps unit and then to rad/s */
	dval = (int64_t)(raw_val) * sensitivity / 10;
	sensor_10udegrees_to_rad(dval, val);
}

static inline int ism330dlc_gyro_get_channel(const struct device *dev, enum sensor_channel chan,
					      struct sensor_value *val, uint32_t sensitivity)
{
	struct ism330dlc_data *data = dev->data;
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		ism330dlc_gyro_convert(val, data->gyro[0], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		ism330dlc_gyro_convert(val, data->gyro[1], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		ism330dlc_gyro_convert(val, data->gyro[2], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (i = 0; i < 3; i++) {
			ism330dlc_gyro_convert(val++, data->gyro[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ism330dlc_gyro_channel_get(const struct device *dev, enum sensor_channel chan,
				       struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;

	return ism330dlc_gyro_get_channel(dev, chan, val, data->gyro_gain);
}

#if defined(CONFIG_ISM330DLC_ENABLE_TEMP)
static void ism330dlc_gyro_channel_get_temp(const struct device *dev,
					     struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;

	/* val = temp_sample / 256 + 25 */
	val->val1 = data->temp_sample / 256 + 25;
	val->val2 = (data->temp_sample % 256) * (1000000 / 256);
}
#endif

#if defined(CONFIG_ISM330DLC_SENSORHUB)
static inline void ism330dlc_magn_convert(struct sensor_value *val, int raw_val,
					   uint16_t sensitivity)
{
	double dval;

	/* Sensitivity is exposed in ugauss/LSB */
	dval = (double)(raw_val * sensitivity);
	val->val1 = (int32_t)dval / 1000000;
	val->val2 = (int32_t)dval % 1000000;
}

static inline int ism330dlc_magn_get_channel(const struct device *dev, enum sensor_channel chan,
					      struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;
	int16_t sample[3];
	int idx;

	idx = ism330dlc_shub_get_idx(SENSOR_CHAN_MAGN_XYZ);
	if (idx < 0) {
		LOG_DBG("external magn not supported");
		return -ENOTSUP;
	}


	sample[0] = (int16_t)(data->ext_data[idx][0] |
			    (data->ext_data[idx][1] << 8));
	sample[1] = (int16_t)(data->ext_data[idx][2] |
			    (data->ext_data[idx][3] << 8));
	sample[2] = (int16_t)(data->ext_data[idx][4] |
			    (data->ext_data[idx][5] << 8));

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		ism330dlc_magn_convert(val, sample[0], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Y:
		ism330dlc_magn_convert(val, sample[1], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Z:
		ism330dlc_magn_convert(val, sample[2], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		ism330dlc_magn_convert(val, sample[0], data->magn_gain);
		ism330dlc_magn_convert(val + 1, sample[1], data->magn_gain);
		ism330dlc_magn_convert(val + 2, sample[2], data->magn_gain);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void ism330dlc_hum_convert(const struct device *dev, struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;
	float rh;
	int16_t raw_val;
	struct hts221_data *ht = &data->hts221;
	int idx;

	idx = ism330dlc_shub_get_idx(SENSOR_CHAN_HUMIDITY);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = ((int16_t)(data->ext_data[idx][0] |
			   (data->ext_data[idx][1] << 8)));

	/* find relative humidty by linear interpolation */
	rh = (ht->y1 - ht->y0) * raw_val + ht->x1 * ht->y0 - ht->x0 * ht->y1;
	rh /= (ht->x1 - ht->x0);

	/* convert humidity to integer and fractional part */
	val->val1 = rh;
	val->val2 = rh * 1000000;
}

static inline void ism330dlc_press_convert(const struct device *dev, struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;
	int32_t raw_val;
	int idx;

	idx = ism330dlc_shub_get_idx(SENSOR_CHAN_PRESS);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = (int32_t)(data->ext_data[idx][0] |
			  (data->ext_data[idx][1] << 8) |
			  (data->ext_data[idx][2] << 16));

	/* Pressure sensitivity is 4096 LSB/hPa */
	/* Convert raw_val to val in kPa */
	val->val1 = (raw_val >> 12) / 10;
	val->val2 = (raw_val >> 12) % 10 * 100000 +
		(((int32_t)((raw_val) & 0x0FFF) * 100000L) >> 12);
}

static inline void ism330dlc_temp_convert(const struct device *dev, struct sensor_value *val)
{
	struct ism330dlc_data *data = dev->data;
	int16_t raw_val;
	int idx;

	idx = ism330dlc_shub_get_idx(SENSOR_CHAN_PRESS);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = (int16_t)(data->ext_data[idx][3] |
			  (data->ext_data[idx][4] << 8));

	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = (int32_t)raw_val % 100 * (10000);
}
#endif

static int ism330dlc_channel_get(const struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330dlc_accel_channel_get(dev, chan, val);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ism330dlc_gyro_channel_get(dev, chan, val);
		break;
#if defined(CONFIG_ISM330DLC_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		ism330dlc_gyro_channel_get_temp(dev, val);
		break;
#endif
#if defined(CONFIG_ISM330DLC_SENSORHUB)
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		ism330dlc_magn_get_channel(dev, chan, val);
		break;

	case SENSOR_CHAN_HUMIDITY:
		ism330dlc_hum_convert(dev, val);
		break;

	case SENSOR_CHAN_PRESS:
		ism330dlc_press_convert(dev, val);
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
		ism330dlc_temp_convert(dev, val);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api ism330dlc_api_funcs = {
	.attr_set = ism330dlc_attr_set,
#if CONFIG_ISM330DLC_TRIGGER
	.trigger_set = ism330dlc_trigger_set,
#endif
	.sample_fetch = ism330dlc_sample_fetch,
	.channel_get = ism330dlc_channel_get,
};

static int ism330dlc_init_chip(const struct device *dev)
{
	const struct ism330dlc_config * const cfg = dev->config;
	struct ism330dlc_data *ism330dlc = dev->data;
	uint8_t chip_id;

	ism330dlc->dev = dev;

	if (ism330dlc_device_id_get(ism330dlc->ctx, &chip_id) < 0) {
		LOG_DBG("Failed reading chip id");
		return -EIO;
	}

	LOG_INF("chip id 0x%x", chip_id);

	if (chip_id != ISM330DLC_ID) {
		LOG_DBG("Invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	/* reset device */
	if (ism330dlc_reset_set(ism330dlc->ctx, 1) < 0) {
		return -EIO;
	}

	k_busy_wait(100);

	LOG_DBG("accel range is %d", cfg->accel_range);
	if (ism330dlc_accel_range_set(dev, cfg->accel_range) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	LOG_DBG("accel odr is %d", cfg->accel_odr);
	if (ism330dlc_accel_set_odr_raw(dev, cfg->accel_odr) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	LOG_DBG("gyro range is %d", cfg->gyro_range);
	if (ism330dlc_gyro_range_set(dev, cfg->gyro_range) < 0) {
		LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	LOG_DBG("gyro odr is %d", cfg->gyro_odr);
	ism330dlc->gyro_freq = ism330dlc_odr_to_freq_val(cfg->gyro_odr);
	if (ism330dlc_gyro_set_odr_raw(dev, cfg->gyro_odr) < 0) {
		LOG_DBG("failed to set gyroscope sampling rate");
		return -EIO;
	}

	/* Set FIFO bypass mode */
	if (ism330dlc_fifo_mode_set(ism330dlc->ctx, ISM330DLC_BYPASS_MODE) < 0) {
		LOG_DBG("failed to set FIFO mode");
		return -EIO;
	}

	if (ism330dlc_block_data_update_set(ism330dlc->ctx, 1) < 0) {
		LOG_DBG("failed to set BDU mode");
		return -EIO;
	}

	return 0;
}

static int ism330dlc_init(const struct device *dev)
{
	const struct ism330dlc_config * const config = dev->config;

	config->bus_init(dev);

	if (ism330dlc_init_chip(dev) < 0) {
		LOG_DBG("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_ISM330DLC_TRIGGER
	if (config->drdy_gpio.port) {
		if (ism330dlc_init_interrupt(dev) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

#ifdef CONFIG_ISM330DLC_SENSORHUB
	if (ism330dlc_shub_init(dev) < 0) {
		LOG_DBG("failed to initialize external chip");
		return -EIO;
	}
#endif

	return 0;
}

#define ISM330DLC_DEFINE(inst)									\
	static struct ism330dlc_data ism330dlc_data_##inst;					\
												\
	static const struct ism330dlc_config ism330dlc_config_##inst = {			\
		.accel_odr = DT_INST_PROP(inst, accel_odr),					\
		.accel_range = DT_INST_PROP(inst, accel_range),					\
		.gyro_odr = DT_INST_PROP(inst, gyro_odr),					\
		.gyro_range = DT_INST_PROP(inst, gyro_range),					\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),						\
			    (.bus_init = ism330dlc_spi_init,					\
			     .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER |		\
							 SPI_MODE_CPOL | SPI_MODE_CPHA |	\
							 SPI_WORD_SET(8), 0),),			\
			    ())									\
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c),						\
			    (.bus_init = ism330dlc_i2c_init,					\
			     .i2c = I2C_DT_SPEC_INST_GET(inst),),				\
			    ())									\
		IF_ENABLED(CONFIG_ISM330DLC_TRIGGER,						\
			   (.drdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, { 0 }),	\
			    .int_pin = DT_INST_PROP_OR(inst, int_pin, 0),))			\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ism330dlc_init, NULL,				\
			      &ism330dlc_data_##inst, &ism330dlc_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &ism330dlc_api_funcs);		\

DT_INST_FOREACH_STATUS_OKAY(ISM330DLC_DEFINE)
