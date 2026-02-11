#include "LSM6DSV16X.h"

#include <Arduino.h>
#include <common2.h>

#include <bit>
#include <cstring>

#define MIN_ST_LIMIT_mg 50.0f
#define MAX_ST_LIMIT_mg 1700.0f
#define MIN_ST_LIMIT_mdps 150000.0f
#define MAX_ST_LIMIT_mdps 700000.0f
#define ST_PASS 1U
#define ST_FAIL 0U

std::uint8_t self_test(const stmdev_ctx_t* dev_ctx) {
	lsm6dsv16x_data_ready_t drdy;
	std::int16_t data_raw[3];
	float val_st_off[3];
	float val_st_on[3];
	float test_val[3];
	std::uint8_t st_result;
	std::uint8_t i;
	std::uint8_t j;

	/*
	 * Accelerometer Self Test
	 */
	/* Set Output Data Rate */
	common2::println("Set Output Data Rate");
	lsm6dsv16x_xl_data_rate_set(dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
	dev_ctx->mdelay(100);
	/* Set full scale */
	common2::println("Set full scale");
	lsm6dsv16x_xl_full_scale_set(dev_ctx, LSM6DSV16X_4g);
	/* Wait stable output */
	dev_ctx->mdelay(100);

	/* Check if new value available */
	do {
		common2::println("Check if new value available");
		lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
		dev_ctx->mdelay(100);
	} while (!drdy.drdy_xl);

	dev_ctx->mdelay(100);

	/* Read dummy data and discard it */
	common2::println("Read dummy data and discard it");
	lsm6dsv16x_acceleration_raw_get(dev_ctx, data_raw);
	dev_ctx->mdelay(100);
	/* Read 5 sample and get the average vale for each axis */
	memset(val_st_off, 0x00, 3 * sizeof(float));

	for (i = 0; i < 5; i++) {
		/* Check if new value available */
		do {
			common2::println("Check if new value available");
			lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
			dev_ctx->mdelay(100);
		} while (!drdy.drdy_xl);

		/* Read data and accumulate the mg value */
		common2::println("Read data and accumulate the mg value");
		lsm6dsv16x_acceleration_raw_get(dev_ctx, data_raw);
		dev_ctx->mdelay(100);

		for (j = 0; j < 3; j++) {
			val_st_off[j] += lsm6dsv16x_from_fs4_to_mg(data_raw[j]);
		}
	}

	/* Calculate the mg average values */
	for (i = 0; i < 3; i++) {
		val_st_off[i] /= 5.0f;
	}

	/* Enable Self Test positive (or negative) */
	common2::println("Enable Self Test positive (or negative)");
	lsm6dsv16x_xl_self_test_set(dev_ctx, LSM6DSV16X_XL_ST_NEGATIVE);
	// lsm6dsv16x_xl_self_test_set(dev_ctx, LSM6DSV16X_XL_ST_POSITIVE);
	/* Wait stable output */
	dev_ctx->mdelay(100);

	/* Check if new value available */
	do {
		common2::println("Check if new value available");
		lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
		dev_ctx->mdelay(100);
	} while (!drdy.drdy_xl);

	/* Read dummy data and discard it */
	common2::println("Read dummy data and discard it");
	lsm6dsv16x_acceleration_raw_get(dev_ctx, data_raw);
	dev_ctx->mdelay(100);
	/* Read 5 sample and get the average vale for each axis */
	memset(val_st_on, 0x00, 3 * sizeof(float));

	for (i = 0; i < 5; i++) {
		/* Check if new value available */
		do {
			common2::println("Check if new value available");
			lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
			dev_ctx->mdelay(100);
		} while (!drdy.drdy_xl);

		/* Read data and accumulate the mg value */
		common2::println("Read data and accumulate the mg value");
		lsm6dsv16x_acceleration_raw_get(dev_ctx, data_raw);
		dev_ctx->mdelay(100);

		for (j = 0; j < 3; j++) {
			val_st_on[j] += lsm6dsv16x_from_fs4_to_mg(data_raw[j]);
		}
	}

	/* Calculate the mg average values */
	for (i = 0; i < 3; i++) {
		val_st_on[i] /= 5.0f;
	}

	/* Calculate the mg values for self test */
	for (i = 0; i < 3; i++) {
		test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
	}

	/* Check self test limit */
	st_result = ST_PASS;

	for (i = 0; i < 3; i++) {
		if ((MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mg)) {
			common2::println(MIN_ST_LIMIT_mg, " > ", test_val[i], " > ", MAX_ST_LIMIT_mg);
			st_result = ST_FAIL;
		}
	}

	/* Disable Self Test */
	common2::println("Disable Self Test");
	lsm6dsv16x_xl_self_test_set(dev_ctx, LSM6DSV16X_XL_ST_DISABLE);
	dev_ctx->mdelay(100);
	/* Disable sensor. */
	common2::println("Disable sensor");
	lsm6dsv16x_xl_data_rate_set(dev_ctx, LSM6DSV16X_ODR_OFF);
	dev_ctx->mdelay(100);

	if (st_result == ST_PASS) common2::println("Accelerometer OK!");

	/*
	 * Gyroscope Self Test
	 */
	/* Set Output Data Rate */
	common2::println("Set Output Data Rate");
	lsm6dsv16x_gy_data_rate_set(dev_ctx, LSM6DSV16X_ODR_AT_240Hz);
	dev_ctx->mdelay(100);
	/* Set full scale */
	common2::println("Set full scale");
	lsm6dsv16x_gy_full_scale_set(dev_ctx, LSM6DSV16X_2000dps);
	/* Wait stable output */
	dev_ctx->mdelay(100);

	/* Check if new value available */
	do {
		common2::println("Check if new value available");
		lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
		dev_ctx->mdelay(100);
	} while (!drdy.drdy_gy);

	/* Read dummy data and discard it */
	common2::println("Read dummy data and discard it");
	lsm6dsv16x_angular_rate_raw_get(dev_ctx, data_raw);
	dev_ctx->mdelay(100);
	/* Read 5 sample and get the average vale for each axis */
	memset(val_st_off, 0x00, 3 * sizeof(float));

	for (i = 0; i < 5; i++) {
		/* Check if new value available */
		do {
			common2::println("Check if new value available");
			lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
			dev_ctx->mdelay(100);
		} while (!drdy.drdy_gy);

		/* Read data and accumulate the mg value */
		common2::println("Read data and accumulate the mg value");
		lsm6dsv16x_angular_rate_raw_get(dev_ctx, data_raw);
		dev_ctx->mdelay(100);

		for (j = 0; j < 3; j++) {
			val_st_off[j] += lsm6dsv16x_from_fs2000_to_mdps(data_raw[j]);
		}
	}

	/* Calculate the mg average values */
	for (i = 0; i < 3; i++) {
		val_st_off[i] /= 5.0f;
	}

	/* Enable Self Test positive (or negative) */
	common2::println("Enable Self Test positive (or negative)");
	lsm6dsv16x_gy_self_test_set(dev_ctx, LSM6DSV16X_GY_ST_POSITIVE);
	// lsm6dsv16x_gy_self_test_set(dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
	/* Wait stable output */
	dev_ctx->mdelay(100);
	/* Read 5 sample and get the average vale for each axis */
	memset(val_st_on, 0x00, 3 * sizeof(float));

	for (i = 0; i < 5; i++) {
		/* Check if new value available */
		do {
			common2::println("Check if new value available");
			lsm6dsv16x_flag_data_ready_get(dev_ctx, &drdy);
			dev_ctx->mdelay(100);
		} while (!drdy.drdy_gy);

		/* Read data and accumulate the mg value */
		common2::println("Read data and accumulate the mg value");
		lsm6dsv16x_angular_rate_raw_get(dev_ctx, data_raw);
		dev_ctx->mdelay(100);

		for (j = 0; j < 3; j++) {
			val_st_on[j] += lsm6dsv16x_from_fs2000_to_mdps(data_raw[j]);
		}
	}

	/* Calculate the mg average values */
	for (i = 0; i < 3; i++) {
		val_st_on[i] /= 5.0f;
	}

	/* Calculate the mg values for self test */
	for (i = 0; i < 3; i++) {
		test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
	}

	/* Check self test limit */
	for (i = 0; i < 3; i++) {
		if ((MIN_ST_LIMIT_mdps > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mdps)) {
			common2::println(MIN_ST_LIMIT_mdps, " > ", test_val[i], " > ", MAX_ST_LIMIT_mdps);
			st_result = ST_FAIL;
		}
	}

	/* Disable Self Test */
	common2::println("Disable Self Test");
	lsm6dsv16x_gy_self_test_set(dev_ctx, LSM6DSV16X_GY_ST_DISABLE);
	dev_ctx->mdelay(100);
	/* Disable sensor. */
	common2::println("Disable sensor");
	lsm6dsv16x_gy_data_rate_set(dev_ctx, LSM6DSV16X_ODR_OFF);
	dev_ctx->mdelay(100);

	if (st_result == ST_PASS) common2::println("Gyro OK!");

	return st_result;
}

void LSM6DSV16X::init() const {
	platform_delay(100 * BOOT_TIME);

	common2::print("Reboot device...");
	lsm6dsv16x_reboot(this);

	platform_delay(100 * BOOT_TIME);

	common2::print("Check reboot and check reset...");
	for (lsm6dsv16x_ctrl3_t ctrl3 = {0}; lsm6dsv16x_read_reg(this, LSM6DSV16X_CTRL3, reinterpret_cast<uint8_t*>(&ctrl3), 1) || ctrl3.boot != 0u || ctrl3.sw_reset != 0u;) {
		common2::print("Error! BOOT: ", static_cast<std::uint8_t>(ctrl3.boot), "/0 SW_RESET: ", static_cast<std::uint8_t>(ctrl3.sw_reset), "/0! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Software reset...");
	lsm6dsv16x_sw_reset(this);

	common2::print("Check reboot and check reset...");
	for (lsm6dsv16x_ctrl3_t ctrl3 = {0}; lsm6dsv16x_read_reg(this, LSM6DSV16X_CTRL3, reinterpret_cast<uint8_t*>(&ctrl3), 1) || ctrl3.boot != 0u || ctrl3.sw_reset != 0u;) {
		common2::print("Error! BOOT: ", static_cast<std::uint8_t>(ctrl3.boot), "/0 SW_RESET: ", static_cast<std::uint8_t>(ctrl3.sw_reset), "/0! Retry...");
		delay(100);
	}
	common2::println("Done!");

	// Check device ID
	common2::print("Check device ID...");
	for (std::uint8_t id; lsm6dsv16x_device_id_get(this, &id) && id != LSM6DSV16X_ID;) {
		common2::print("Error! ID: ", id, "/", " Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Enable Auto Increment...");
	for (std::uint8_t val; lsm6dsv16x_auto_increment_get(this, &val) || val != PROPERTY_ENABLE;) {
		common2::print(val, "/", PROPERTY_ENABLE, "...");
		for (; lsm6dsv16x_auto_increment_set(this, PROPERTY_ENABLE);) {
			common2::print("...Error! Retry");
			delay(100);
		}
		delay(100);
	}
	common2::println("Done!");

	// Restore default configuration
	common2::print("Restore default configuration...");
	for (; lsm6dsv16x_sw_por(this);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Enable Block Data Update...");
	for (std::uint8_t val; lsm6dsv16x_block_data_update_get(this, &val) || val != PROPERTY_ENABLE;) {
		common2::print(val, "/", PROPERTY_ENABLE, "...");
		for (; lsm6dsv16x_block_data_update_set(this, PROPERTY_ENABLE);) {
			common2::print("...Error! Retry");
			delay(100);
		}
		delay(100);
	}
	common2::println("Done!");

	// Disable Sensor Hub
	// common2::print("Disable Sensor hub I2C master...");
	// for (; lsm6dsv16x_sh_master_set(this, PROPERTY_DISABLE);) {
	// 	common2::print("Error! Retry...");
	// 	delay(100);
	// }
	// common2::println("Done!");
}

LSM6DSV16X::LSM6DSV16X(TwoWire* i2c, std::uint8_t const device_address) : stmdev_ctx_t{.write_reg = platform_write, .read_reg = platform_read, .mdelay = platform_delay, .handle = this}, i2c(i2c), device_address(device_address) {}
void LSM6DSV16X::begin_self_test(lsm6dsv16x_xl_full_scale_t const accel_scale_value, lsm6dsv16x_gy_full_scale_t const gyro_scale_value) {
	accel_scale = accel_scale_value;
	gyro_scale = gyro_scale_value;

	init();

	self_test(this);
}

void LSM6DSV16X::begin(lsm6dsv16x_xl_full_scale_t const accel_scale_value, lsm6dsv16x_gy_full_scale_t const gyro_scale_value) {
	accel_scale = accel_scale_value;
	gyro_scale = gyro_scale_value;

	init();

	// Set full scale
	common2::print("Set Accelerometer full scale...");
	for (; lsm6dsv16x_xl_full_scale_set(this, accel_scale);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Set Gyro full scale...");
	for (; lsm6dsv16x_gy_full_scale_set(this, gyro_scale);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	// Set Output Data Rate.
	// Selected data rate have to be equal or greater with respect with MLC data rate.
	common2::print("Set Accelerometer Output Data Rate...");
	for (; lsm6dsv16x_xl_data_rate_set(this, LSM6DSV16X_ODR_AT_7Hz5);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	delay(100);

	common2::print("Set Gyro Output Data Rate...");
	for (; lsm6dsv16x_gy_data_rate_set(this, LSM6DSV16X_ODR_AT_15Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	delay(100);

	// Configure filtering chain
	common2::print("Set Settling Mask...");
	filt_settling_mask.drdy = PROPERTY_ENABLE;
	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	filt_settling_mask.irq_g = PROPERTY_ENABLE;
	for (; lsm6dsv16x_filt_settling_mask_set(this, filt_settling_mask);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Enable Gyro LP1...");
	for (; lsm6dsv16x_filt_gy_lp1_set(this, PROPERTY_ENABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Set Gyro LP1 bandwidth...");
	for (; lsm6dsv16x_filt_gy_lp1_bandwidth_set(this, LSM6DSV16X_GY_ULTRA_LIGHT);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Enable Accelerometer LP2...");
	for (; lsm6dsv16x_filt_xl_lp2_set(this, PROPERTY_ENABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Set Accelerometer LP2 bandwidth...");
	for (; lsm6dsv16x_filt_xl_lp2_bandwidth_set(this, LSM6DSV16X_XL_STRONG);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
}
void LSM6DSV16X::begin_fifo(lsm6dsv16x_xl_full_scale_t const accel_scale_value, lsm6dsv16x_gy_full_scale_t const gyro_scale_value) {
	accel_scale = accel_scale_value;
	gyro_scale = gyro_scale_value;

	init();

	common2::print("Set Accelerometer full scale...");
	for (; lsm6dsv16x_xl_full_scale_set(this, accel_scale_value);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Gyro full scale...");
	for (; lsm6dsv16x_gy_full_scale_set(this, gyro_scale_value);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO watermark...");
	for (; lsm6dsv16x_fifo_watermark_set(this, 64);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO batch Accelerometer ODR to 60Hz...");
	for (; lsm6dsv16x_fifo_xl_batch_set(this, LSM6DSV16X_XL_BATCHED_AT_60Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO batch Gyro ODR to 12.5Hz...");
	for (; lsm6dsv16x_fifo_gy_batch_set(this, LSM6DSV16X_GY_BATCHED_AT_15Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO mode to Stream mode (aka Continuous Mode)...");
	for (; lsm6dsv16x_fifo_mode_set(this, LSM6DSV16X_STREAM_MODE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Accelerometer Output Data Rate...");
	for (; lsm6dsv16x_xl_data_rate_set(this, LSM6DSV16X_ODR_AT_60Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Gyro Output Data Rate...");
	for (; lsm6dsv16x_gy_data_rate_set(this, LSM6DSV16X_ODR_AT_15Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Timestamp Rate...");
	for (; lsm6dsv16x_fifo_timestamp_batch_set(this, LSM6DSV16X_TMSTMP_DEC_8);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Enable Timestamp...");
	for (; lsm6dsv16x_timestamp_set(this, PROPERTY_ENABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
}
void LSM6DSV16X::begin_sensor_fusion(lsm6dsv16x_xl_full_scale_t const accel_scale_value, lsm6dsv16x_gy_full_scale_t const gyro_scale_value) {
	accel_scale = accel_scale_value;
	gyro_scale = gyro_scale_value;

	init();

	// Set full scale
	common2::print("Set Accelerometer full scale...");
	for (; lsm6dsv16x_xl_full_scale_set(this, accel_scale);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Set Gyro full scale...");
	for (; lsm6dsv16x_gy_full_scale_set(this, gyro_scale);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO watermark...");
	for (; lsm6dsv16x_fifo_watermark_set(this, 32);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Set FIFO batch of sflp data...");
	lsm6dsv16x_fifo_sflp_raw_t fifo_sflp{.game_rotation = 1, .gravity = 1, .gbias = 1};
	for (; lsm6dsv16x_fifo_sflp_batch_set(this, fifo_sflp);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set FIFO mode to Stream mode (aka Continuous Mode)...");
	for (; lsm6dsv16x_fifo_mode_set(this, LSM6DSV16X_STREAM_MODE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Accelerometer Output Data Rate...");
	for (; lsm6dsv16x_xl_data_rate_set(this, LSM6DSV16X_ODR_AT_30Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Gyro Output Data Rate...");
	for (; lsm6dsv16x_gy_data_rate_set(this, LSM6DSV16X_ODR_AT_30Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set sflp Output Data Rate...");
	for (; lsm6dsv16x_sflp_data_rate_set(this, LSM6DSV16X_SFLP_30Hz);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
	common2::print("Enable Game Rotation...");
	for (; lsm6dsv16x_sflp_game_rotation_set(this, PROPERTY_ENABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Initialize Gyro Offset...");
	lsm6dsv16x_sflp_gbias_t gbias{.gbias_x = 0.0f, .gbias_y = 0.0f, .gbias_z = 0.0f};
	for (; lsm6dsv16x_sflp_game_gbias_set(this, &gbias);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");
}
void LSM6DSV16X::begin_minimal() const {
	init();

	common2::print("Enable Accel High-Performance Mode...");
	for (lsm6dsv16x_xl_mode_t val; lsm6dsv16x_xl_mode_get(this, &val) || val != LSM6DSV16X_XL_HIGH_PERFORMANCE_MD;) {
		switch (val) {
			case LSM6DSV16X_XL_HIGH_PERFORMANCE_MD: common2::print("LSM6DSV16X_XL_HIGH_PERFORMANCE_MD"); break;
			case LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD: common2::print("LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD"); break;
			case LSM6DSV16X_XL_ODR_TRIGGERED_MD: common2::print("LSM6DSV16X_XL_ODR_TRIGGERED_MD"); break;
			case LSM6DSV16X_XL_LOW_POWER_2_AVG_MD: common2::print("LSM6DSV16X_XL_LOW_POWER_2_AVG_MD"); break;
			case LSM6DSV16X_XL_LOW_POWER_4_AVG_MD: common2::print("LSM6DSV16X_XL_LOW_POWER_4_AVG_MD"); break;
			case LSM6DSV16X_XL_LOW_POWER_8_AVG_MD: common2::print("LSM6DSV16X_XL_LOW_POWER_8_AVG_MD"); break;
			case LSM6DSV16X_XL_NORMAL_MD: common2::print("LSM6DSV16X_XL_NORMAL_MD"); break;
		}
		common2::print("/", "LSM6DSV16X_XL_HIGH_PERFORMANCE_MD", "...");
		for (; lsm6dsv16x_xl_mode_set(this, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);) {
			common2::print("...Error! Retry");
			delay(100);
		}
		delay(100);
	}
	common2::println("Done!");

	common2::print("Select Accel ODR to 60Hz...");
	for (lsm6dsv16x_data_rate_t odr_xl; lsm6dsv16x_xl_data_rate_get(this, &odr_xl) || odr_xl != LSM6DSV16X_ODR_AT_60Hz;) {
		switch (odr_xl) {
			case LSM6DSV16X_ODR_OFF: common2::print("LSM6DSV16X_ODR_OFF"); break;
			case LSM6DSV16X_ODR_AT_1Hz875: common2::print("LSM6DSV16X_ODR_AT_1Hz875"); break;
			case LSM6DSV16X_ODR_AT_7Hz5: common2::print("LSM6DSV16X_ODR_AT_7Hz5"); break;
			case LSM6DSV16X_ODR_AT_15Hz: common2::print("LSM6DSV16X_ODR_AT_15Hz"); break;
			case LSM6DSV16X_ODR_AT_30Hz: common2::print("LSM6DSV16X_ODR_AT_30Hz"); break;
			case LSM6DSV16X_ODR_AT_60Hz: common2::print("LSM6DSV16X_ODR_AT_60Hz"); break;
			case LSM6DSV16X_ODR_AT_120Hz: common2::print("LSM6DSV16X_ODR_AT_120Hz"); break;
			case LSM6DSV16X_ODR_AT_240Hz: common2::print("LSM6DSV16X_ODR_AT_240Hz"); break;
			case LSM6DSV16X_ODR_AT_480Hz: common2::print("LSM6DSV16X_ODR_AT_480Hz"); break;
			case LSM6DSV16X_ODR_AT_960Hz: common2::print("LSM6DSV16X_ODR_AT_960Hz"); break;
			case LSM6DSV16X_ODR_AT_1920Hz: common2::print("LSM6DSV16X_ODR_AT_1920Hz"); break;
			case LSM6DSV16X_ODR_AT_3840Hz: common2::print("LSM6DSV16X_ODR_AT_3840Hz"); break;
			case LSM6DSV16X_ODR_AT_7680Hz: common2::print("LSM6DSV16X_ODR_AT_7680Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_15Hz625: common2::print("LSM6DSV16X_ODR_HA01_AT_15Hz625"); break;
			case LSM6DSV16X_ODR_HA01_AT_31Hz25: common2::print("LSM6DSV16X_ODR_HA01_AT_31Hz25"); break;
			case LSM6DSV16X_ODR_HA01_AT_62Hz5: common2::print("LSM6DSV16X_ODR_HA01_AT_62Hz5"); break;
			case LSM6DSV16X_ODR_HA01_AT_125Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_125Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_250Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_250Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_500Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_500Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_1000Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_1000Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_2000Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_2000Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_4000Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_4000Hz"); break;
			case LSM6DSV16X_ODR_HA01_AT_8000Hz: common2::print("LSM6DSV16X_ODR_HA01_AT_8000Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_12Hz5: common2::print("LSM6DSV16X_ODR_HA02_AT_12Hz5"); break;
			case LSM6DSV16X_ODR_HA02_AT_25Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_25Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_50Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_50Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_100Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_100Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_200Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_200Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_400Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_400Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_800Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_800Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_1600Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_1600Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_3200Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_3200Hz"); break;
			case LSM6DSV16X_ODR_HA02_AT_6400Hz: common2::print("LSM6DSV16X_ODR_HA02_AT_6400Hz"); break;
		}
		common2::print("/", "LSM6DSV16X_ODR_AT_60Hz", "...");
		for (; lsm6dsv16x_xl_data_rate_set(this, LSM6DSV16X_ODR_AT_60Hz);) {
			common2::print("...Error! Retry");
			delay(100);
		}
		delay(100);
	}
	common2::println("Done!");
}
std::int32_t LSM6DSV16X::platform_write(void* handle, std::uint8_t const reg, std::uint8_t const* bufp, std::uint16_t const len) {
	auto* obj = static_cast<LSM6DSV16X*>(handle);

	obj->i2c->beginTransmission(obj->device_address);
	if (auto const ret = obj->i2c->write(reg); ret == 0) return 1;
	if (auto const ret = obj->i2c->write(bufp, len); ret != len) return 1;
	if (auto const ret = obj->i2c->endTransmission(); !ret) return ret;

	return 0;
}
std::int32_t LSM6DSV16X::platform_read(void* handle, std::uint8_t const reg, std::uint8_t* bufp, std::uint16_t const len) {
	auto* obj = static_cast<LSM6DSV16X*>(handle);

	obj->i2c->beginTransmission(obj->device_address);
	if (auto const ret = obj->i2c->write(reg); ret == 0) return 1;
	if (auto const ret = obj->i2c->endTransmission(true); ret != 0) return ret;  // STM32 chip produces some Errors with repeated START -> so send STOP signal to be safe.

	if (auto const ret = obj->i2c->requestFrom(obj->device_address, static_cast<std::uint8_t>(len)); ret < len) return 1;
	if (auto const ret = obj->i2c->readBytes(bufp, len); ret < len) return 1;

	return 0;
}
void LSM6DSV16X::platform_delay(std::uint32_t const ms) { delay(ms); }
void LSM6DSV16X::start_measurement() {
	// common2::print("Get Data Ready...");
	for (; lsm6dsv16x_flag_data_ready_get(this, &drdy);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	// common2::println("Done!");
}
void LSM6DSV16X::start_measurement_fifo() {
	// common2::print("Read watermark flag...");
	for (; lsm6dsv16x_fifo_status_get(this, &fifo_status);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	// common2::println("Done!");

	// num = 0;
	if (fifo_status.fifo_th) {
		num = fifo_status.fifo_level;
	}
}

std::optional<AccelerationDataRaw> LSM6DSV16X::get_measurement_accelerometer() const {
	if (drdy.drdy_xl) {
		AccelerationDataRaw accel_data_raw{};

		for (; lsm6dsv16x_acceleration_raw_get(this, accel_data_raw.arr.data());) {
			common2::print("Error! Retry...");
			delay(100);
		}

		return accel_data_raw;
	}

	return {};
}

std::optional<GyroDataRaw> LSM6DSV16X::get_measurement_gyro() const {
	if (drdy.drdy_gy) {
		GyroDataRaw gyro_data_raw{};

		for (; lsm6dsv16x_angular_rate_raw_get(this, gyro_data_raw.arr.data());) {
			common2::print("Error! Retry...");
			delay(100);
		}

		return gyro_data_raw;
	}

	return {};
}

std::variant<LSM6DSV16X::NoData, AccelerationDataRaw, GyroDataRaw, std::uint64_t> LSM6DSV16X::get_measurement_fifo() {
	for (; num; --num) {
		lsm6dsv16x_fifo_out_raw_t f_data;

		for (; lsm6dsv16x_fifo_out_raw_get(this, &f_data);) {
			common2::print("Error! Retry...");
			delay(100);
		}

		++messages[f_data.tag];

		switch (f_data.tag) {
			case LSM6DSV16X_GY_NC_TAG: return GyroDataRaw{.bytes = std::to_array(f_data.data)};
			case LSM6DSV16X_XL_NC_TAG: return AccelerationDataRaw{.bytes = std::to_array(f_data.data)};
			case LSM6DSV16X_TIMESTAMP_TAG: return std::uint64_t{std::bit_cast<std::uint32_t>(std::array{f_data.data[0], f_data.data[1], f_data.data[2], f_data.data[3]})};
			default: return NoData{};
		}
	}

	common2::println(messages);
	return NoData{};
}
std::variant<LSM6DSV16X::NoData, RotationQuaternion, LSM6DSV16X::GBiasVectorRaw, LSM6DSV16X::GravityVectorRaw> LSM6DSV16X::get_measurement_sensor_fusion() {
	for (; num; --num) {
		lsm6dsv16x_fifo_out_raw_t f_data;

		for (; lsm6dsv16x_fifo_out_raw_get(this, &f_data);) {
			common2::print("Error! Retry...");
			delay(100);
		}

		++messages[f_data.tag];

		switch (f_data.tag) {
			case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: return RotationQuaternion{.arr = sflp2q(std::bit_cast<std::array<std::uint16_t, 3>>(std::to_array(f_data.data)))};
			case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG: return GBiasVectorRaw{.bytes = std::to_array(f_data.data)};
			case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: return GravityVectorRaw{.bytes = std::to_array(f_data.data)};
			default: return NoData{};
		}
	}

	common2::println(messages);
	return NoData{};
}

float LSM6DSV16X::npy_half_to_float(std::uint16_t const h) {
	union {
		float_t ret;
		uint32_t retbits;
	} conv;
	conv.retbits = lsm6dsv16x_from_f16_to_f32(h);
	return conv.ret;
}

std::array<float, 4> LSM6DSV16X::sflp2q(std::array<std::uint16_t, 3> sflp) {
	std::array<float, 4> quat{npy_half_to_float(sflp[0]), npy_half_to_float(sflp[1]), npy_half_to_float(sflp[2])};

	float sumsq = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2];

	if (sumsq > 1.0f) {
		float const n = sqrtf(sumsq);
		quat[0] /= n;
		quat[1] /= n;
		quat[2] /= n;
		sumsq = 1.0f;
	}

	quat[3] = sqrtf(1.0f - sumsq);

	return quat;
}