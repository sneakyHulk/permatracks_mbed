#pragma once

#include <AccelerationDataRaw.h>
#include <GyroDataRaw.h>
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
	void begin_minimal() const;

	static std::int32_t platform_write(void *handle, std::uint8_t reg, const std::uint8_t *bufp, std::uint16_t len);
	static std::int32_t platform_read(void *handle, std::uint8_t reg, std::uint8_t *bufp, std::uint16_t len);
	static void platform_delay(std::uint32_t ms);

	void start_measurement();
	void start_measurement_fifo();

	struct GravityVector {
		float x;
		float y;
		float z;
	};
	struct GBiasVector {
		float x;
		float y;
		float z;
	};
	struct RotationQuaternion {
		float x;
		float y;
		float z;
		float w;
	};

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
};

/*
 *#include <variant>
class LSM6DSV16X {
   public:
    LSM6DSV16X() = default;

    ///////////////////////////////////////////////////////////////////////
    // This method is called to initialize the LSM6DSV16X library and connect to
    // the LSM6DSV16X device. This method must be called before calling any other method
    // that interacts with the device.
    //
    //  Parameter   Description
    //  ---------   ----------------------------
    //  wirePort    The Wire port.
    //  address     optional. I2C Address. If not provided, the default address is used.
    //  retval      true on success, false on startup failure

    bool begin(TwoWire &wirePort, std::uint8_t const deviceAddress = LSM6DSV16X_ADDRESS_LOW) {
        // Setup  I2C object and pass into the superclass
        setCommunicationBus(_i2cBus, deviceAddress);

        // Give the I2C port provided by the user to the I2C bus class.
        _i2cBus.init(wirePort, true);

        // Initialize the system - return results
        return this->QwDevLSM6DSV16X::init();
    }

    auto enableBlockDataUpdate(bool const enable = true) {
        if (enable) return lsm6dsv16x_block_data_update_set(&sfe_dev, PROPERTY_ENABLE);

        return lsm6dsv16x_block_data_update_set(&sfe_dev, PROPERTY_DISABLE);
    }

    auto setXLFullScale(lsm6dsv16x_xl_full_scale_t val) {
        lsm6dsv16x_ctrl8_t ctrl8;
        int32_t ret;

        ret = lsm6dsv16x_read_reg(&sfe_dev, LSM6DSV16X_CTRL8, (uint8_t *)&ctrl8, 1);

        if (ret == 0) {
            ctrl8.fs_xl = (uint8_t)val & 0x3U;
            ret = lsm6dsv16x_write_reg(&sfe_dev, LSM6DSV16X_CTRL8, (uint8_t *)&ctrl8, 1);
        }

        return ret;
    }

    auto setGYFullScale(lsm6dsv16x_gy_full_scale_t val) {
        lsm6dsv16x_ctrl6_t ctrl6;
        int32_t ret;

        ret = lsm6dsv16x_read_reg(&sfe_dev, LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);

        if (ret == 0) {
            ctrl6.fs_g = (uint8_t)val & 0xfu;
            ret = lsm6dsv16x_write_reg(&sfe_dev, LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);
        }

        return ret;
    }

    auto setFIFOWatermark(std::uint8_t const val) {
        lsm6dsv16x_fifo_ctrl1_t fifo_ctrl1;
        int32_t ret;

        ret = lsm6dsv16x_read_reg(&sfe_dev, LSM6DSV16X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);

        if (ret == 0) {
            fifo_ctrl1.wtm = val;
            ret = lsm6dsv16x_write_reg(&sfe_dev, LSM6DSV16X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);
        }

        return ret;
    }

    auto setFifoMode(lsm6dsv16x_fifo_mode_t const mode) { return lsm6dsv16x_fifo_mode_set(&sfe_dev, mode); }
    auto setFIFObatchOfSFLP(lsm6dsv16x_fifo_sflp_raw_t fifo_sflp = {.game_rotation = 1, .gravity = 1, .gbias = 1}) { return lsm6dsv16x_fifo_sflp_batch_set(&sfe_dev, fifo_sflp); }
    auto setSFLPDataRate(lsm6dsv16x_sflp_data_rate_t const rate) { return lsm6dsv16x_sflp_data_rate_set(&sfe_dev, rate); }
    auto setSFLPGameRotation(bool const enable = true) {
        if (enable) return lsm6dsv16x_sflp_game_rotation_set(&sfe_dev, PROPERTY_ENABLE);

        return lsm6dsv16x_sflp_game_rotation_set(&sfe_dev, PROPERTY_DISABLE);
    }
    lsm6dsv16x_sflp_gbias_t gbias2;
    auto setSFLPGameGbias(lsm6dsv16x_sflp_gbias_t gbias = {.gbias_x = 0.0f, .gbias_y = 0.0f, .gbias_z = 0.0f}) {
        gbias2 = gbias;
        return lsm6dsv16x_sflp_game_gbias_set(&sfe_dev, &gbias2);
    }
    auto getFIFOStatus(lsm6dsv16x_fifo_status_t &fifo_status) { return lsm6dsv16x_fifo_status_get(&sfe_dev, &fifo_status); }

    struct GravityVector {
        float x;
        float y;
        float z;
    };
    struct GBiasVector {
        float x;
        float y;
        float z;
    };
    struct RotationQuaternion {
        float x;
        float y;
        float z;
        float w;
    };

    std::variant<GBiasVector, GravityVector, RotationQuaternion> getFIFOData() {
        lsm6dsv16x_fifo_out_raw_t f_data;
        lsm6dsv16x_fifo_out_raw_get(&sfe_dev, &f_data);

        switch (f_data.tag) {
            case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
                return GBiasVector{
                    lsm6dsv16x_from_fs125_to_mdps(f_data.data[0] | f_data.data[1] << 8), lsm6dsv16x_from_fs125_to_mdps(f_data.data[2] | f_data.data[3] << 8), lsm6dsv16x_from_fs125_to_mdps(f_data.data[4] | f_data.data[5] << 8)};
            case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
                return GravityVector{lsm6dsv16x_from_sflp_to_mg(f_data.data[0] | f_data.data[1] << 8), lsm6dsv16x_from_sflp_to_mg(f_data.data[2] | f_data.data[3] << 8), lsm6dsv16x_from_sflp_to_mg(f_data.data[4] | f_data.data[5] << 8)};
            case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: return RotationQuaternion{};
        }

        return {};
    }

   private:
    // I2C bus class
    sfe_LSM6DSV16X::QwI2C _i2cBus;

    static float npy_half_to_float(uint16_t h) {
        union {
            float ret;
            std::uint32_t retbits;
        } conv;
        conv.retbits = ToFloatBits(h);
        return conv.ret;
    }

    static void sflp2q(float_t quat[4], uint16_t sflp[3]) {
        float_t sumsq = 0;

        quat[0] = npy_half_to_float(sflp[0]);
        quat[1] = npy_half_to_float(sflp[1]);
        quat[2] = npy_half_to_float(sflp[2]);

        for (uint8_t i = 0; i < 3; i++) sumsq += quat[i] * quat[i];

        if (sumsq > 1.0f) {
            float_t n = sqrtf(sumsq);
            quat[0] /= n;
            quat[1] /= n;
            quat[2] /= n;
            sumsq = 1.0f;
        }

        quat[3] = sqrtf(1.0f - sumsq);
    }

    static uint32_t ToFloatBits(uint16_t h) {
        uint16_t h_exp = (h & 0x7c00u);
        uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16;
        switch (h_exp) {
            case 0x0000u:  // 0 or subnormal
            {
                uint16_t h_sig = (h & 0x03ffu);
                // Signed zero
                if (h_sig == 0) {
                    return f_sgn;
                }
                // Subnormal
                h_sig <<= 1;
                while ((h_sig & 0x0400u) == 0) {
                    h_sig <<= 1;
                    h_exp++;
                }
                uint32_t f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
                uint32_t f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
                return f_sgn + f_exp + f_sig;
            }
            case 0x7c00u:  // inf or NaN
                // All-ones exponent and a copy of the significand
                return f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
            default:  // normalized
                // Just need to adjust the exponent and shift
                return f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
        }
    }
};*/
