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

	void begin(bool const enable_automatic_set_reset = true, bool const enable_continuous_mode = true, uint16_t const filter_bandwidth = 100, uint16_t const continuous_mode_frequency = 100, uint16_t const periodic_set_samples = 1) {
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

	bool performSetOperation() {
		// Set the SET bit to perform a set operation.
		// Do this using the shadow register. If we do it with setRegisterBit
		// (read-modify-write) we end up setting the Auto_SR_en bit too as that
		// always seems to read as 1...? I don't know why.
		bool success = setShadowBit(INT_CTRL_0_REG, SET_OPERATION);

		clearShadowBit(INT_CTRL_0_REG, SET_OPERATION, false);  // Clear the bit - in shadow memory only

		// Wait for the set operation to complete (500ns).
		// delay(1);

		return success;
	}

	bool performResetOperation() {
		// Set the RESET bit to perform a reset operation.
		// Do this using the shadow register. If we do it with setRegisterBit
		// (read-modify-write) we end up setting the Auto_SR_en bit too as that
		// always seems to read as 1...? I don't know why.
		bool success = setShadowBit(INT_CTRL_0_REG, RESET_OPERATION);

		clearShadowBit(INT_CTRL_0_REG, RESET_OPERATION, false);  // Clear the bit - in shadow memory only

		// Wait for the reset operation to complete (500ns).
		// delay(1);

		return success;
	}

	void start_measurement() {
		bool success = setShadowBit(INT_CTRL_0_REG, TM_M);

		if (!success) {
			clearShadowBit(INT_CTRL_0_REG, TM_M, false);  // Clear the bit - in shadow memory only
			SAFE_CALLBACK(errorCallback, SF_MMC5983MA_ERROR::BUS_ERROR);

			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");
		}
	}

	void start_measurement_float() { start_measurement(); }

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

		// z-value is down (see datasheet)
		return {static_cast<float>(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<float>(static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    -static_cast<float>(static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA};
	}
#pragma pack(push, 1)
	struct MagneticFluxDensityDataRaw {
		union {
			struct {
				std::int32_t x : 19;
				std::int32_t y : 19;
				std::int32_t z : 18;
			};
			std::uint8_t bytes[7];
		};
	};
#pragma pack(pop)

	std::uint32_t get_scale_factor() const {
		constexpr std::uint32_t value = (1U << (MMC5983MA_MODE_BITS - 1)) * (1000000 / MMC5983MA_FULL_SCALE_RANGE_UTESLA);  // in Tesla
		return value;
	}

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

		std::array<double, 3> test1 = {-static_cast<double>(static_cast<std::int32_t>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    -static_cast<double>(static_cast<std::int32_t>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA,
		    static_cast<double>(static_cast<std::int32_t>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA};

		MagneticFluxDensityDataRaw value = {-(static_cast<std::int32_t>(x_value) - (1 << (MMC5983MA_MODE_BITS - 1))), -(static_cast<std::int32_t>(y_value) - (1 << (MMC5983MA_MODE_BITS - 1))),
		    (static_cast<std::int32_t>(z_value) - (1 << (MMC5983MA_MODE_BITS - 1)))};  // because of overflow z must be more than 18 bits
		std::array<double, 3> test2 = {static_cast<double>(static_cast<std::int32_t>(value.x)) / static_cast<double>(get_scale_factor()) * 1e6,
		    static_cast<double>(static_cast<std::int32_t>(value.y)) / static_cast<double>(get_scale_factor()) * 1e6, static_cast<double>(static_cast<std::int32_t>(value.z)) / static_cast<double>(get_scale_factor()) * 1e6};

		if (std::abs(test1[0] - test2[0]) > 0.001 || std::abs(test1[1] - test2[1]) > 0.001 || std::abs(test1[2] - test2[2]) > 0.001) {
			common::print_error("Difference between measurements!", test1[0], " vs. ", test2[0], ", ", test1[1], " vs. ", test2[1], ", ", test1[2], " vs. ", test2[2], ", ", static_cast<std::int32_t>(value.z), " vs. ", z_value);
		}

		// z-value is down (see datasheet) (because of overflow z must be more than 18 bits)
		return value;
	}

#undef private
   private:
	using SFE_MMC5983MA::begin;
};

namespace common {
	void print_low_level(MMC5983MA::MagneticFluxDensityDataRaw const& d) {
		common::print_low_level(d.x);
		common::print_low_level(',');
		common::print_low_level(d.y);
		common::print_low_level(',');
		common::print_low_level(d.z);
	}
}  // namespace common