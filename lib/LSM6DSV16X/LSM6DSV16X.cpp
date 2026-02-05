#include "LSM6DSV16X.h"

#include <Arduino.h>
#include <common2.h>
LSM6DSV16X::LSM6DSV16X(TwoWire* i2c, std::uint8_t const device_address) : stmdev_ctx_t{.write_reg = platform_write, .read_reg = platform_read, .mdelay = platform_delay, .handle = this}, i2c(i2c), device_address(device_address) {}
void LSM6DSV16X::begin() {
	platform_delay(BOOT_TIME);

	common2::print("Reboot device...");
	for (; lsm6dsv16x_reboot(this);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	platform_delay(BOOT_TIME);

	// common2::print("Software reset...");
	// for (; lsm6dsv16x_sw_reset(this);) {
	// 	common2::print("Error! Retry...");
	// 	delay(100);
	// }
	// common2::println("Done!");

	// common2::print("Sensor Hub reset...");
	// for (; lsm6dsv16x_sh_reset_set(this, PROPERTY_ENABLE);) {
	// 	common2::print("Error! Retry...");
	// 	delay(100);
	// }
	// common2::println("Done!");

	common2::print("Check device ID...");
	for (std::uint8_t id; lsm6dsv16x_device_id_get(this, &id) && id != LSM6DSV16X_ID;) {
		common2::print("Error! ID: ", id, "/", " Retry...");
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

	common2::print("Disable Sensor hub I2C master...");
	for (; lsm6dsv16x_sh_master_set(this, PROPERTY_DISABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Enable Block Data Update...");
	for (; lsm6dsv16x_block_data_update_set(this, PROPERTY_ENABLE);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Accelerometer full scale...");
	for (; lsm6dsv16x_xl_full_scale_set(this, LSM6DSV16X_2g);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	common2::print("Set Gyro full scale...");
	for (; lsm6dsv16x_gy_full_scale_set(this, LSM6DSV16X_2000dps);) {
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

	//common2::print("Set FIFO batch XL ODR to 12.5Hz...");
	//for (; lsm6dsv16x_fifo_xl_batch_set(this, LSM6DSV16X_XL_BATCHED_AT_60Hz);) {
	//	common2::print("Error! Retry...");
	//	delay(100);
	//}
	//common2::println("Done!");

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

	// common2::print("Set Accelerometer Output Data Rate...");
	// for (; lsm6dsv16x_xl_data_rate_set(this, LSM6DSV16X_ODR_AT_60Hz);) {
	// 	common2::print("Error! Retry...");
	// 	delay(100);
	// }
	// common2::println("Done!");

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

	// common2::print("Set FIFO batch of sflp data...");
	// static lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
	// fifo_sflp.game_rotation = 1;
	// fifo_sflp.gravity = 1;
	// fifo_sflp.gbias = 1;
	// for (; lsm6dsv16x_fifo_sflp_batch_set(this, fifo_sflp);) {
	// 	common2::print("Error! Retry...");
	// 	delay(100);
	// }
	// common2::println("Done!");

	// common2::print("Set sflp Output Data Rate...");
	// for (; lsm6dsv16x_sflp_data_rate_set(this, LSM6DSV16X_SFLP_30Hz);) {
	//	common2::print("Error! Retry...");
	//	delay(100);
	// }
	// common2::println("Done!");
	//
	// common2::print("Enable Game Rotation...");
	// for (; lsm6dsv16x_sflp_game_rotation_set(this, PROPERTY_ENABLE);) {
	//	common2::print("Error! Retry...");
	//	delay(100);
	//}
	// common2::println("Done!");
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
	if (auto const ret = obj->i2c->endTransmission(false); !ret) return ret;

	if (auto const ret = obj->i2c->requestFrom(obj->device_address, static_cast<std::uint8_t>(len)); ret < len) return 1;
	if (auto const ret = obj->i2c->readBytes(bufp, len); ret < len) return 1;

	return 0;
}
void LSM6DSV16X::platform_delay(std::uint32_t const ms) { delay(ms); }
void LSM6DSV16X::start_measurement() {
	common2::print("Read watermark flag...");
	for (; lsm6dsv16x_fifo_status_get(this, &fifo_status);) {
		common2::print("Error! Retry...");
		delay(100);
	}
	common2::println("Done!");

	// num = 0;
	if (fifo_status.fifo_th) {
		num = fifo_status.fifo_level;
		common2::println("FIFO num: ", static_cast<std::uint16_t>(fifo_status.fifo_level));
		common2::println("FIFO full: ", static_cast<std::uint8_t>(fifo_status.fifo_full));
		common2::println(messages);
	}
}

bool LSM6DSV16X::get_measurement() {
	for (; num; --num) {
		lsm6dsv16x_fifo_out_raw_t f_data;
		std::uint8_t* axis;
		float quat[4];
		float gravity_mg[3];
		float gbias_mdps[3];

		// common2::print("Read FIFO sensor value...");
		for (; lsm6dsv16x_fifo_out_raw_get(this, &f_data);) {
			common2::print("Error! Retry...");
			delay(100);
		}
		// common2::println("Done!");

		++messages[f_data.tag];

		switch (f_data.tag) {
			case LSM6DSV16X_XL_NC_TAG: common2::println("num: ", num, ", LSM6DSV16X_XL_NC_TAG tag"); return true;
			case LSM6DSV16X_GY_NC_TAG: common2::println("num: ", num, ", LSM6DSV16X_GY_NC_TAG tag"); return true;
			case LSM6DSV16X_TIMESTAMP_TAG: common2::println("num: ", num, ", LSM6DSV16X_TIMESTAMP_TAG tag"); return true;
		}
	}

	// case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG: common2::println("num: ", num, ", LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG tag"); return true;
	// case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: common2::println("num: ", num, ", LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG tag: "); return true;
	// case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: common2::println("num: ", num, ", LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG tag: "); return true;
	// default: common2::print("num: ", num, ", Unknown tag: ", static_cast<std::uint8_t>(f_data.tag));

	return false;
}