#pragma once

#define private protected
#include <SPI.h>
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
		if (!SFE_MMC5983MA::begin(cs_pin, *spi)) {
			Serial1.printf("[%s, %010d]: ", sensor_name, millis());
			Serial1.println("Failed to initialize SPI!");
			return;
		}

		setFilterBandwidth(filter_bandwidth);
		Serial1.printf("[%s, %010d]: ", sensor_name, millis());
		Serial1.print("Filter Bandwidth set to: ");
		Serial1.println(getFilterBandwidth());

		setContinuousModeFrequency(continuous_mode_frequency);
		Serial1.printf("[%s, %010d]: ", sensor_name, millis());
		Serial1.print("Continuous Mode Frequency set to: ");
		Serial1.println(getContinuousModeFrequency());

		setPeriodicSetSamples(periodic_set_samples);
		Serial1.printf("[%s, %010d]: ", sensor_name, millis());
		Serial1.print("PeriodicSetSamples set to: ");
		Serial1.println(getPeriodicSetSamples());

		if (enable_automatic_set_reset) {
			enableAutomaticSetReset();
		} else {
			disableAutomaticSetReset();
		}
		Serial1.printf("[%s, %010d]: ", sensor_name, millis());
		Serial1.print("Automatic Set Reset set to: ");
		Serial1.println(isAutomaticSetResetEnabled() ? "enabled" : "disabled");

		if (enable_continuous_mode) {
			enableContinuousMode();
		} else {
			disableContinuousMode();
		}
		Serial1.print("Continuous Mode set to: ");
		Serial1.println(isContinuousModeEnabled() ? "enabled" : "disabled");
	}

	void start_measurement() {
		bool success = setShadowBit(INT_CTRL_0_REG, TM_M);

		if (!success) {
			clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only
			SAFE_CALLBACK(errorCallback, SF_MMC5983MA_ERROR::BUS_ERROR);

			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");
		}
	}

	std::tuple<decltype(millis()), float, float, float> get_measurement() {
		// Wait until measurement is completed or times out
		uint16_t timeOut = getTimeout();

		while ((!mmc_io.isBitSet(STATUS_REG, MEAS_M_DONE)) && (timeOut > 0)) {
			// Wait a little so we won't flood MMC with requests
			delay(1);
			timeOut--;
		}

		clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only

		// Read the fields even if a timeout occurred - old data vs no data
		// Return false if a timeout or a read error occurred

		uint32_t x_value = 0, y_value = 0, z_value = 0;
		if (!readFieldsXYZ(&x_value, &y_value, &z_value) || timeOut == 0) {
			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");

			return std::make_tuple(millis(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		}

		return {millis(), static_cast<float>(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<float>(static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<float>(static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA};
	}

#undef private
   private:
	using SFE_MMC5983MA::begin;
};
