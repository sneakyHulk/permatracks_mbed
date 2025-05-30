#pragma once

#include <SPI.h>
#include <common.h>

#define private protected
#include <SparkFun_MMC5983MA_Arduino_Library.h>

#define MMC5983MA_MODE_BITS 18
#define MMC5983MA_FULL_SCALE_RANGE_UTESLA 800

class MMC5983MA final : public SFE_MMC5983MA {
	uint8_t cs_pin;
	SPIClass* spi;

   public:
	const char* sensor_name;

	explicit MMC5983MA(const char* sensor_name, uint8_t cs_pin, SPIClass* spi) : sensor_name(sensor_name), cs_pin(cs_pin), spi(spi) {}

	void begin(uint16_t filter_bandwidth = 100, uint16_t continuous_mode_frequency = 100, uint16_t periodic_set_samples = 1, bool enable_automatic_set_reset = true, bool enable_continuous_mode = true) {
		if (!SFE_MMC5983MA::begin(cs_pin, SPISettings(10000000, MSBFIRST, SPI_MODE0), *spi)) {  // 10MHz max
			common::println_critical_time_loc(millis(), '\'', sensor_name, '\'', " failed to initialize SPI!");
			return;
		}

		setFilterBandwidth(filter_bandwidth);
		common::print_time(millis(), '\'', sensor_name, '\'', " filter bandwidth set to: ");
		common::println(getFilterBandwidth());

		setContinuousModeFrequency(continuous_mode_frequency);
		common::print_time(millis(), '\'', sensor_name, '\'', " continuous mode frequency set to: ");
		common::println(getContinuousModeFrequency());

		setPeriodicSetSamples(periodic_set_samples);
		common::print_time(millis(), '\'', sensor_name, '\'', " periodic set samples set to: ");
		common::println(getPeriodicSetSamples());

		if (enable_automatic_set_reset) {
			enableAutomaticSetReset();
		} else {
			disableAutomaticSetReset();
		}
		common::print_time(millis(), '\'', sensor_name, '\'', " automatic set/reset set to: ");
		common::println(isAutomaticSetResetEnabled() ? "enabled" : "disabled");

		if (enable_continuous_mode) {
			enableContinuousMode();
		} else {
			disableContinuousMode();
		}
		common::print_time(millis(), '\'', sensor_name, '\'', " continuous mode set to: ");
		common::println(isContinuousModeEnabled() ? "enabled" : "disabled");
	}

	void start_measurement() {
		bool success = setShadowBit(INT_CTRL_0_REG, TM_M);

		if (!success) {
			clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only
			SAFE_CALLBACK(errorCallback, SF_MMC5983MA_ERROR::BUS_ERROR);

			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");
		}
	}

	void stop_measurement_float() { start_measurement(); }

	common::MagneticFluxDensityData get_measurement_float() {
		// Wait until measurement is completed or times out
		uint16_t timeOut = getTimeout();

		while ((!mmc_io.isBitSet(STATUS_REG, MEAS_M_DONE)) && (timeOut > 0)) {
			// Wait a little so we won't flood MMC with requests
			delay(1);
			timeOut--;
			common::println_warn_time_loc(millis(), "too slow");
		}

		clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only

		// Read the fields even if a timeout occurred - old data vs no data
		// Return false if a timeout or a read error occurred

		uint32_t x_value = 0, y_value = 0, z_value = 0;
		if (!readFieldsXYZ(&x_value, &y_value, &z_value) || timeOut == 0) {
			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");

			return {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
		}

		return {static_cast<float>(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<float>(static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<float>(static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA};
	}

	struct MagneticFluxDensityDataRaw {
		union {
			struct {
				int32_t x : 18;
				int32_t y : 18;
				int32_t z : 18;
				uint16_t reserved : 10;
			};
			struct {
				uint8_t bytes[7];
				uint8_t reserved2[1];
			};
		};
	};

	MagneticFluxDensityDataRaw get_measurement() {
		// Wait until measurement is completed or times out
		uint16_t timeOut = getTimeout();

		while ((!mmc_io.isBitSet(STATUS_REG, MEAS_M_DONE)) && (timeOut > 0)) {
			// Wait a little so we won't flood MMC with requests
			delay(1);
			timeOut--;
			common::println_warn_time_loc(millis(), sensor_name, " too slow");
		}

		clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only

		// Read the fields even if a timeout occurred - old data vs no data
		// Return false if a timeout or a read error occurred

		uint32_t x_value = 0, y_value = 0, z_value = 0;
		if (!readFieldsXYZ(&x_value, &y_value, &z_value) || timeOut == 0) {
			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");

			return {0, 0, 0};
		}

		return {static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1), static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1), static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)};
	}

#undef private
   private:
	using SFE_MMC5983MA::begin;
};
