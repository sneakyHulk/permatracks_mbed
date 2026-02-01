#pragma once

#include <SPI.h>
#include <common2.h>
#include <common2_output.h>

#define private protected
#include <Adafruit_LIS3MDL.h>

class LIS3MDL final : public Adafruit_LIS3MDL {
	uint8_t cs_pin;
	SPIClass* spi;
	sensors_event_t event;

   public:
	const char* sensor_name;

	explicit LIS3MDL(const char* sensor_name, uint8_t cs_pin, SPIClass* spi) : sensor_name(sensor_name), cs_pin(cs_pin), spi(spi) {}

	void begin(lis3mdl_performancemode_t performance_mode = LIS3MDL_ULTRAHIGHMODE, lis3mdl_operationmode_t operation_mode = LIS3MDL_CONTINUOUSMODE, lis3mdl_dataRate_t data_rate = LIS3MDL_DATARATE_80_HZ,
	    lis3mdl_range_t range = LIS3MDL_RANGE_4_GAUSS) {
		if (!begin_SPI(cs_pin, spi, 10000000)) {
			common2::println_time_loc(millis(), '\'', sensor_name, '\'', " failed to initialize SPI!");
			return;
		}

		setPerformanceMode(performance_mode);
		common2::print_time(millis(), '\'', sensor_name, '\'', " performance mode set to: ");
		switch (getPerformanceMode()) {
			case LIS3MDL_LOWPOWERMODE: common2::println("Low"); break;
			case LIS3MDL_MEDIUMMODE: common2::println("Medium"); break;
			case LIS3MDL_HIGHMODE: common2::println("High"); break;
			case LIS3MDL_ULTRAHIGHMODE: common2::println("Ultra-High"); break;
		}

		setOperationMode(operation_mode);
		common2::print_time(millis(), '\'', sensor_name, '\'', " operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (getOperationMode()) {
			case LIS3MDL_CONTINUOUSMODE: common2::println("Continuous"); break;
			case LIS3MDL_SINGLEMODE: common2::println("Single mode"); break;
			case LIS3MDL_POWERDOWNMODE: common2::println("Power-down"); break;
		}

		setDataRate(data_rate);
		// You can check the datarate by looking at the frequency of the DRDY pin
		common2::print_time(millis(), '\'', sensor_name, '\'', " data rate set to: ");

		switch (getDataRate()) {
			case LIS3MDL_DATARATE_0_625_HZ: common2::println("0.625 Hz"); break;
			case LIS3MDL_DATARATE_1_25_HZ: common2::println("1.25 Hz"); break;
			case LIS3MDL_DATARATE_2_5_HZ: common2::println("2.5 Hz"); break;
			case LIS3MDL_DATARATE_5_HZ: common2::println("5 Hz"); break;
			case LIS3MDL_DATARATE_10_HZ: common2::println("10 Hz"); break;
			case LIS3MDL_DATARATE_20_HZ: common2::println("20 Hz"); break;
			case LIS3MDL_DATARATE_40_HZ: common2::println("40 Hz"); break;
			case LIS3MDL_DATARATE_80_HZ: common2::println("80 Hz"); break;
			case LIS3MDL_DATARATE_155_HZ: common2::println("155 Hz"); break;
			case LIS3MDL_DATARATE_300_HZ: common2::println("300 Hz"); break;
			case LIS3MDL_DATARATE_560_HZ: common2::println("560 Hz"); break;
			case LIS3MDL_DATARATE_1000_HZ: common2::println("1000 Hz"); break;
		}

		setRange(range);
		common2::print_time(millis(), '\'', sensor_name, '\'', " range set to: ");
		switch (getRange()) {
			case LIS3MDL_RANGE_4_GAUSS: common2::println("+-4 gauss"); break;
			case LIS3MDL_RANGE_8_GAUSS: common2::println("+-8 gauss"); break;
			case LIS3MDL_RANGE_12_GAUSS: common2::println("+-12 gauss"); break;
			case LIS3MDL_RANGE_16_GAUSS: common2::println("+-16 gauss"); break;
		}
	}

	void start_measurement_float() {
		if (!getEvent(&event)) {
			common2::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");
		}
	}

	[[nodiscard]] MagneticFluxDensityData get_measurement_float() const { return {event.magnetic.y, -event.magnetic.x, event.magnetic.z}; }

	void start_measurement() {
		uint8_t buffer[6];

		Adafruit_BusIO_Register XYZDataReg = Adafruit_BusIO_Register(i2c_dev, spi_dev, AD8_HIGH_TOREAD_AD7_HIGH_TOINC, LIS3MDL_REG_OUT_X_L, 6);
		XYZDataReg.read(buffer, 6);
		x = buffer[0];
		x |= buffer[1] << 8;
		y = buffer[2];
		y |= buffer[3] << 8;
		z = buffer[4];
		z |= buffer[5] << 8;
	}

	struct MagneticFluxDensityDataRaw {
		union {
			struct {
				std::int16_t x;
				std::int16_t y;
				std::int16_t z;
			};
			std::uint8_t bytes[6];
		};
	};

	std::uint32_t get_scale_factor() {
		std::uint32_t scale;

		switch (rangeBuffered) {
			case LIS3MDL_RANGE_16_GAUSS: scale = 1711; break;
			case LIS3MDL_RANGE_12_GAUSS: scale = 2281; break;
			case LIS3MDL_RANGE_8_GAUSS: scale = 3421; break;
			case LIS3MDL_RANGE_4_GAUSS: scale = 6842; break;
		}

		return scale * 10000;
	}

	// switch x and y because of datasheet
	[[nodiscard]] MagneticFluxDensityDataRaw get_measurement() const { return {y, static_cast<std::int16_t>(x == std::numeric_limits<std::int16_t>::min() ? std::numeric_limits<std::int16_t>::max() : -x), z}; }

#undef private
   private:
	using Adafruit_LIS3MDL::begin_I2C;
	using Adafruit_LIS3MDL::begin_SPI;
};

namespace common2 {
	void print_low_level(LIS3MDL::MagneticFluxDensityDataRaw const& d) {
		common2::print_low_level(d.x);
		common2::print_low_level(',');
		common2::print_low_level(d.y);
		common2::print_low_level(',');
		common2::print_low_level(d.z);
	}
}  // namespace common