#pragma once

#include <AccelerationDataRaw.h>
#include <GravityVector.h>
#include <GyroBiasVector.h>
#include <GyroDataRaw.h>
#include <RotationQuaternion.h>
#include <Wire.h>
#include <common2.h>
#include <lsm6dsv16x_reg.h>

#include <cstdint>
#include <optional>
#include <variant>

class LSM6DSV16X : public stmdev_ctx_t {
	constexpr static auto BOOT_TIME = 10;  // ms
	std::uint16_t num = 0;
	lsm6dsv16x_fifo_status_t fifo_status;
	lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
	lsm6dsv16x_data_ready_t drdy;
	std::array<std::uint64_t, 0x1E> messages = {};

	lsm6dsv16x_xl_full_scale_t accel_scale = LSM6DSV16X_16g;
	lsm6dsv16x_gy_full_scale_t gyro_scale = LSM6DSV16X_4000dps;

	void init() const;

   public:
	TwoWire *i2c;
	std::uint8_t const device_address;

	LSM6DSV16X(TwoWire *i2c, std::uint8_t device_address);
	void begin_self_test(lsm6dsv16x_xl_full_scale_t accel_scale_value = LSM6DSV16X_2g, lsm6dsv16x_gy_full_scale_t gyro_scale_value = LSM6DSV16X_1000dps);
	void begin(lsm6dsv16x_xl_full_scale_t accel_scale_value = LSM6DSV16X_2g, lsm6dsv16x_gy_full_scale_t gyro_scale_value = LSM6DSV16X_1000dps);
	void begin_fifo(lsm6dsv16x_xl_full_scale_t accel_scale_value = LSM6DSV16X_2g, lsm6dsv16x_gy_full_scale_t gyro_scale_value = LSM6DSV16X_1000dps);
	void begin_sensor_fusion(lsm6dsv16x_xl_full_scale_t accel_scale_value = LSM6DSV16X_2g, lsm6dsv16x_gy_full_scale_t gyro_scale_value = LSM6DSV16X_1000dps);
	void begin_both(lsm6dsv16x_xl_full_scale_t accel_scale_value = LSM6DSV16X_2g, lsm6dsv16x_gy_full_scale_t gyro_scale_value = LSM6DSV16X_1000dps);
	void begin_minimal() const;

	static std::int32_t platform_write(void *handle, std::uint8_t reg, const std::uint8_t *bufp, std::uint16_t len);
	static std::int32_t platform_read(void *handle, std::uint8_t reg, std::uint8_t *bufp, std::uint16_t len);
	static void platform_delay(std::uint32_t ms);

	void start_measurement();
	void start_measurement_fifo();

	[[nodiscard]] std::optional<AccelerationDataRaw> get_measurement_accelerometer() const;
	[[nodiscard]] float get_scale_factor_accelerometer() const {
		switch (accel_scale) {
			case LSM6DSV16X_2g: return 0.061f;
			case LSM6DSV16X_4g: return 0.122f;
			case LSM6DSV16X_8g: return 0.244f;
			case LSM6DSV16X_16g: return 0.488f;
		}

		return NAN;
	}
	[[nodiscard]] std::optional<GyroDataRaw> get_measurement_gyro() const;
	[[nodiscard]] float get_scale_factor_gyro() const {
		switch (gyro_scale) {
			case LSM6DSV16X_125dps: return 4.375f;
			case LSM6DSV16X_250dps: return 8.750f;
			case LSM6DSV16X_500dps: return 17.50f;
			case LSM6DSV16X_1000dps: return 35.0f;
			case LSM6DSV16X_2000dps: return 70.0f;
			case LSM6DSV16X_4000dps: return 140.0f;
		}

		return NAN;
	}
	struct NoData {};
	std::variant<NoData, AccelerationDataRaw, GyroDataRaw, std::uint64_t> get_measurement_fifo();
	std::variant<NoData, RotationQuaternion, GyroBiasVector, GravityVector> get_measurement_sensor_fusion();

	static float npy_half_to_float(std::uint16_t h);
	static std::array<float, 4> sflp2q(std::array<std::uint16_t, 3> sflp);
	static std::array<float, 3> sflp2gbv(std::array<std::int16_t, 3> sflp);
	static std::array<float, 3> sflp2gv(std::array<std::int16_t, 3> sflp);
};