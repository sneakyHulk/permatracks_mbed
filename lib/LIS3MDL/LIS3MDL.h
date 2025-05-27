#pragma once
#include <Adafruit_LIS3MDL.h>
#include <SPI.h>
#include <common_printing.h>

#include "common_output.h"

class LIS3MDL final : public Adafruit_LIS3MDL {
	uint8_t cs_pin;
	SPIClass* spi;
	sensors_event_t event;

   public:
	const char* sensor_name;

	explicit LIS3MDL(const char* sensor_name, uint8_t cs_pin, SPIClass* spi) : sensor_name(sensor_name), cs_pin(cs_pin), spi(spi) {}

	void begin(lis3mdl_performancemode_t performance_mode = LIS3MDL_ULTRAHIGHMODE, lis3mdl_operationmode_t operation_mode = LIS3MDL_CONTINUOUSMODE, lis3mdl_dataRate_t data_rate = LIS3MDL_DATARATE_80_HZ,
	    lis3mdl_range_t range = LIS3MDL_RANGE_4_GAUSS) {
		if (!begin_SPI(cs_pin, spi, 2000000)) {
			common::println_critical_time_loc(millis(), '\'', sensor_name, '\'', " failed to initialize SPI!");
			return;
		}

		setPerformanceMode(performance_mode);
		common::print_time(millis(), '\'', sensor_name, '\'', " performance mode set to: ");
		switch (getPerformanceMode()) {
			case LIS3MDL_LOWPOWERMODE: common::println("Low"); break;
			case LIS3MDL_MEDIUMMODE: common::println("Medium"); break;
			case LIS3MDL_HIGHMODE: common::println("High"); break;
			case LIS3MDL_ULTRAHIGHMODE: common::println("Ultra-High"); break;
		}

		setOperationMode(operation_mode);
		common::print_time(millis(), '\'', sensor_name, '\'', " operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (getOperationMode()) {
			case LIS3MDL_CONTINUOUSMODE: common::println("Continuous"); break;
			case LIS3MDL_SINGLEMODE: common::println("Single mode"); break;
			case LIS3MDL_POWERDOWNMODE: common::println("Power-down"); break;
		}

		setDataRate(data_rate);
		// You can check the datarate by looking at the frequency of the DRDY pin
		common::print_time(millis(), '\'', sensor_name, '\'', " data rate set to: ");

		switch (getDataRate()) {
			case LIS3MDL_DATARATE_0_625_HZ: common::println("0.625 Hz"); break;
			case LIS3MDL_DATARATE_1_25_HZ: common::println("1.25 Hz"); break;
			case LIS3MDL_DATARATE_2_5_HZ: common::println("2.5 Hz"); break;
			case LIS3MDL_DATARATE_5_HZ: common::println("5 Hz"); break;
			case LIS3MDL_DATARATE_10_HZ: common::println("10 Hz"); break;
			case LIS3MDL_DATARATE_20_HZ: common::println("20 Hz"); break;
			case LIS3MDL_DATARATE_40_HZ: common::println("40 Hz"); break;
			case LIS3MDL_DATARATE_80_HZ: common::println("80 Hz"); break;
			case LIS3MDL_DATARATE_155_HZ: common::println("155 Hz"); break;
			case LIS3MDL_DATARATE_300_HZ: common::println("300 Hz"); break;
			case LIS3MDL_DATARATE_560_HZ: common::println("560 Hz"); break;
			case LIS3MDL_DATARATE_1000_HZ: common::println("1000 Hz"); break;
		}

		setRange(range);
		common::print_time(millis(), '\'', sensor_name, '\'', " range set to: ");
		switch (getRange()) {
			case LIS3MDL_RANGE_4_GAUSS: common::println("+-4 gauss"); break;
			case LIS3MDL_RANGE_8_GAUSS: common::println("+-8 gauss"); break;
			case LIS3MDL_RANGE_12_GAUSS: common::println("+-12 gauss"); break;
			case LIS3MDL_RANGE_16_GAUSS: common::println("+-16 gauss"); break;
		}
	}

	void start_measurement() {
		if (!getEvent(&event)) {
			common::println_warn_time(millis(), '\'', sensor_name, '\'', " failed to get data!");
		}
	}

	// switch x and y because of datasheet
	[[nodiscard]] std::tuple<decltype(millis()), float, float, float> get_measurement() const { return std::make_tuple(event.timestamp, event.magnetic.y, event.magnetic.x, event.magnetic.z); }

   private:
	using Adafruit_LIS3MDL::begin_I2C;
	using Adafruit_LIS3MDL::begin_SPI;
};