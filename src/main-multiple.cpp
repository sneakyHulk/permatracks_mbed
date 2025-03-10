#include <Adafruit_LIS3MDL.h>
#include <Arduino.h>
Adafruit_LIS3MDL lis3mdl;
#include <Adafruit_MMC56x3_RAW.h>
Adafruit_MMC5603_RAW mmc5603;
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>
#define MMC5983MA_MODE_BITS 18
#define MMC5983MA_FULL_SCALE_RANGE_UTESLA 800
SFE_MMC5983MA mmc5983ma;

void setup() {
	Serial.begin(115200);
	while (!Serial) delay(10);

	Serial.println("Hello World");

	// connect sensors:
	if (!lis3mdl.begin_I2C()) {
		Serial.println("Failed to find LIS3MDL chip");
		while (true) {
			delay(10);
		}
	}
	Serial.println("LIS3MDL Found!");

	if (!mmc5603.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
		Serial.println("Failed to find MMC5603 chip");
		while (true) {
			delay(10);
		}
	}
	Serial.println("MMC5603 Found!");

	Wire.begin();
	if (!mmc5983ma.begin()) {
		Serial.println("Failed to find MMC5983MA chip");
		while (true) {
			delay(10);
		}
	}
	mmc5983ma.softReset();

	Serial.println("MMC5983MA Found!");

	// configure sensors
	lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
	Serial.print("Performance mode set to: ");
	switch (lis3mdl.getPerformanceMode()) {
		case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
		case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
		case LIS3MDL_HIGHMODE: Serial.println("High"); break;
		case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
	}

	lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
	Serial.print("Operation mode set to: ");
	// Single shot mode will complete conversion and go into power down
	switch (lis3mdl.getOperationMode()) {
		case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
		case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
		case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
	}

	lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
	// You can check the datarate by looking at the frequency of the DRDY pin
	Serial.print("Data rate set to: ");
	switch (lis3mdl.getDataRate()) {
		case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
		case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
		case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
		case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
		case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
		case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
		case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
		case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
		case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
		case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
		case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
		case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
	}

	lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
	Serial.print("Range set to: ");
	switch (lis3mdl.getRange()) {
		case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
		case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
		case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
		case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
	}

	mmc5603.printSensorDetails();

	mmc5603.setDataRate(100);
	Serial.print("Data Rate set to: ");
	Serial.println(mmc5603.getDataRate());

	mmc5603.setContinuousMode(true);
	Serial.print("Continuous Mode set to: ");
	Serial.println("true");

	mmc5983ma.setFilterBandwidth(400);
	Serial.print("Filter Bandwidth set to: ");
	Serial.println(mmc5983ma.getFilterBandwidth());

	mmc5983ma.setContinuousModeFrequency(100);
	Serial.print("Continuous Mode Frequency set to: ");
	Serial.println(mmc5983ma.getContinuousModeFrequency());

	mmc5983ma.enableAutomaticSetReset();
	Serial.print("Automatic Set Reset set to: ");
	Serial.println(mmc5983ma.isAutomaticSetResetEnabled() ? "enabled" : "disabled");

	mmc5983ma.enableContinuousMode();
	Serial.print("Continuous Mode set to: ");
	Serial.println(mmc5983ma.isContinuousModeEnabled() ? "enabled" : "disabled");
}

uint32_t next_heartbeat = 0;

void send_message(float const x, float const y, float const z, const char* sensor_name, uint32_t const timestamp = millis()) {
	Serial.print('[');
	Serial.print(timestamp);
	Serial.print("] ");
	Serial.print(sensor_name);
	Serial.print(": X=");
	Serial.print(x, 5);
	Serial.print(", Y=");
	Serial.print(y, 5);
	Serial.print(", Z=");
	Serial.print(z, 5);
	Serial.println('.');
}

void loop() {
	if (uint32_t const current_time = millis(); next_heartbeat < current_time) {  // sends heartbeat every second
		next_heartbeat = current_time + 1000;

		Serial.println("Heartbeat: 'Nucleo'");
	}

	{
		sensors_event_t event;
		lis3mdl.getEvent(&event);
		send_message(event.magnetic.x, event.magnetic.y, event.magnetic.z, "LIS3MDL");
	}

	{
		sensors_event_t event;
		mmc5603.getEvent(&event);
		send_message(event.magnetic.x, event.magnetic.y, event.magnetic.z, "MMC5603");
	}

	{
		uint32_t x_value = 0, y_value = 0, z_value = 0;
		if (!mmc5983ma.getMeasurementXYZ(&x_value, &y_value, &z_value)) return;

		float x = static_cast<float>(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;
		float y = static_cast<float>(static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;
		float z = static_cast<float>(static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;

		send_message(x, y, z, "MMC5983MA");
	}
}