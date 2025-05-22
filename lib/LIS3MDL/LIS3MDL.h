#pragma once
#include <Adafruit_LIS3MDL.h>
#include <SPI.h>

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
			Serial1.printf("[%s, %8.3lf]: ", sensor_name, millis() / 1000.0);
			Serial1.println("Failed to initialize SPI!");

			return;
		}

		setPerformanceMode(performance_mode);
		Serial1.printf("[%s, %8.3lf]: ", sensor_name, millis() / 1000.0);
		Serial1.print("Performance mode set to: ");
		switch (getPerformanceMode()) {
			case LIS3MDL_LOWPOWERMODE: Serial1.println("Low"); break;
			case LIS3MDL_MEDIUMMODE: Serial1.println("Medium"); break;
			case LIS3MDL_HIGHMODE: Serial1.println("High"); break;
			case LIS3MDL_ULTRAHIGHMODE: Serial1.println("Ultra-High"); break;
		}

		setOperationMode(operation_mode);
		Serial1.printf("[%s, %8.3lf]: ", sensor_name, millis() / 1000.0);
		Serial1.print("Operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (getOperationMode()) {
			case LIS3MDL_CONTINUOUSMODE: Serial1.println("Continuous"); break;
			case LIS3MDL_SINGLEMODE: Serial1.println("Single mode"); break;
			case LIS3MDL_POWERDOWNMODE: Serial1.println("Power-down"); break;
		}

		setDataRate(data_rate);
		// You can check the datarate by looking at the frequency of the DRDY pin
		Serial1.printf("[%s, %8.3lf]: ", sensor_name, millis() / 1000.0);
		Serial1.print("Data rate set to: ");
		switch (getDataRate()) {
			case LIS3MDL_DATARATE_0_625_HZ: Serial1.println("0.625 Hz"); break;
			case LIS3MDL_DATARATE_1_25_HZ: Serial1.println("1.25 Hz"); break;
			case LIS3MDL_DATARATE_2_5_HZ: Serial1.println("2.5 Hz"); break;
			case LIS3MDL_DATARATE_5_HZ: Serial1.println("5 Hz"); break;
			case LIS3MDL_DATARATE_10_HZ: Serial1.println("10 Hz"); break;
			case LIS3MDL_DATARATE_20_HZ: Serial1.println("20 Hz"); break;
			case LIS3MDL_DATARATE_40_HZ: Serial1.println("40 Hz"); break;
			case LIS3MDL_DATARATE_80_HZ: Serial1.println("80 Hz"); break;
			case LIS3MDL_DATARATE_155_HZ: Serial1.println("155 Hz"); break;
			case LIS3MDL_DATARATE_300_HZ: Serial1.println("300 Hz"); break;
			case LIS3MDL_DATARATE_560_HZ: Serial1.println("560 Hz"); break;
			case LIS3MDL_DATARATE_1000_HZ: Serial1.println("1000 Hz"); break;
		}

		setRange(range);
		Serial1.printf("[%s, %8.3lf]: ", sensor_name, millis() / 1000.0);
		Serial1.print("Range set to: ");
		switch (getRange()) {
			case LIS3MDL_RANGE_4_GAUSS: Serial1.println("+-4 gauss"); break;
			case LIS3MDL_RANGE_8_GAUSS: Serial1.println("+-8 gauss"); break;
			case LIS3MDL_RANGE_12_GAUSS: Serial1.println("+-12 gauss"); break;
			case LIS3MDL_RANGE_16_GAUSS: Serial1.println("+-16 gauss"); break;
		}
	}

	void get_data() {
		if (!getEvent(&event)) {
			Serial1.printf("[%s, %8.3lf]: ", sensor_name, event.timestamp / 1000.0);
			Serial1.println("Failed to get data!");
		}
	}

	void output_data() const {
		Serial1.printf("[%s, %8.3lf]: ", sensor_name, event.timestamp / 1000.0);
		Serial1.print("X=");
		Serial1.print(event.magnetic.x, 5);
		Serial1.print(", Y=");
		Serial1.print(event.magnetic.y, 5);
		Serial1.print(", Z=");
		Serial1.print(event.magnetic.z, 5);
		Serial1.println('.');
	}

   private:
	using Adafruit_LIS3MDL::begin_I2C;
	using Adafruit_LIS3MDL::begin_SPI;
};