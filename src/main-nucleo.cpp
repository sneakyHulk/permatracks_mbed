#include <Arduino.h>
#include <SPI.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <numeric>
#include <ranges>

#include "AK09940A.h"
#include "CRC16.h"
#include "RunningAverage.h"

RunningAverage stats_x(100);
RunningAverage stats_y(100);
RunningAverage stats_z(100);

#ifdef USE_LIS3MDL
#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;
#elifdef USE_MLX90393
#include <Adafruit_MLX90393_RAW.h>
Adafruit_MLX90393_RAW mlx90393;
#elifdef USE_MMC5603
#include <Adafruit_MMC56x3_RAW.h>
Adafruit_MMC5603_RAW mmc5603;
#elifdef USE_MMC5983MA
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>
#define MMC5983MA_MODE_BITS 18
#define MMC5983MA_FULL_SCALE_RANGE_UTESLA 800
SFE_MMC5983MA mmc5983ma;
#elifdef USE_AK09940A
constexpr std::uint8_t CS_PIN = D10;
constexpr std::uint8_t TRG_PIN = D7;

SPIClass spi(PA7, PA6, PA1);

constexpr std::uint8_t CNTL4 = 0x33;   // SRST bit (D0)
constexpr std::uint8_t I2CDIS = 0x36;  // lock out I²C
constexpr std::uint8_t CNTL3 = 0x32;   // MODE[4:0]
constexpr std::uint8_t CNTL2 = 0x31;   // TEMP
constexpr std::uint8_t CNTL1 = 0x30;   // MT2
constexpr std::uint8_t ST = 0x0F;      // DRDY flag
constexpr std::uint8_t ST1 = 0x10;     // ST1
constexpr std::uint8_t ST2 = 0x1B;     // ST2
constexpr std::uint8_t HXL = 0x11;     // mag data read start

inline void spiWrite(uint8_t reg, uint8_t data) {
	digitalWrite(CS_PIN, LOW);
	spi.transfer(reg & 0x7F);  // bit-7 = 0 → write
	spi.transfer(data);
	digitalWrite(CS_PIN, HIGH);
}

inline uint8_t spiRead(uint8_t reg) {
	digitalWrite(CS_PIN, LOW);
	spi.transfer(reg | 0x80);  // bit-7 = 1 → read
	std::uint8_t val = spi.transfer(0x00);
	digitalWrite(CS_PIN, HIGH);
	return val;
}

bool check_company_device_id() {
	constexpr std::uint8_t WHO_AM_I1_ADDR = 0x00;
	constexpr std::uint8_t WHO_AM_I2_ADDR = 0x01;

	constexpr std::uint8_t EXPECTED_WIA1 = 0x48;  // Company ID ("AKM")
	constexpr std::uint8_t EXPECTED_WIA2 = 0xA3;  // Device ID (AK09940A)

	std::uint8_t const wia1 = spiRead(WHO_AM_I1_ADDR);
	Serial.print("WIA1: ");
	Serial.println(wia1);

	std::uint8_t const wia2 = spiRead(WHO_AM_I2_ADDR);
	Serial.print("WIA2: ");
	Serial.println(wia2);

	return (wia1 == EXPECTED_WIA1) && (wia2 == EXPECTED_WIA2);
}

void softReset() {
	spiWrite(CNTL4, 0x01);  // SRST = 1 → soft reset
	delay(100);             // > 100 µs Twait
}

void disableI2C() {
	spiWrite(I2CDIS, 0b00011011);
	delay(100);
}

bool waitDRDY(uint16_t timeout_ms = 1000) {
	uint32_t t0 = millis();
	while (millis() - t0 < timeout_ms) {
		auto status = spiRead(ST);
		if (status & 0x01)  // DRDY bit = 1?
			return true;
		delayMicroseconds(10);
	}
	return false;  // timeout
}
#endif

void setup() {
	Serial.begin(230400);
	// while (!Serial) delay(10);

	delay(1000);
	Serial.println("Hello World");
	delay(1000);

	// connect sensor:
#ifdef USE_LIS3MDL
	if (!lis3mdl.begin_I2C()) {
		Serial.println("Failed to find LIS3MDL chip");
		while (true) {
			delay(10);
		}
	}
	Serial.println("LIS3MDL Found!");
#elifdef USE_MLX90393
	if (!mlx90393.begin_I2C(0x18)) {
		Serial.println(": Failed to find MLX90393 chip");
		while (true) {
			delay(10);
		}
	}
	Serial.println("MLX90393 Found!");
#elifdef USE_MMC5603
	if (!mmc5603.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
		Serial.println("Failed to find MMC5603 chip");
		while (true) {
			delay(10);
		}
	}
	Serial.println("MMC5603 Found!");
#elifdef USE_MMC5983MA
	Wire.begin();
	if (!mmc5983ma.begin()) {
		Serial.println("Failed to find MMC5983MA chip");
		while (true) {
			delay(10);
		}
	}
	mmc5983ma.softReset();

	Serial.println("MMC5983MA Found!");
#elifdef USE_AK09940A

	pinMode(CS_PIN, OUTPUT);
	digitalWrite(CS_PIN, HIGH);

#ifndef CONTINUOUS
	pinMode(TRG_PIN, OUTPUT);
	digitalWrite(TRG_PIN, LOW);
#endif

	delay(100);
	spi.beginTransaction(SPISettings(3'000'000, BitOrder::MSBFIRST, SPI_MODE0));  // AK09940A uses SPI Mode 3

#endif
	Serial.println("beginTransaction()");
	delay(1000);
	// configure sensor
#ifdef USE_LIS3MDL
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

		// This is useful for generating an Interrupt when magnet comes close (N = 500) in the direction of the z-Axis.
		// lis3mdl.setIntThreshold(500);
		// lis3mdl.configInterrupt(false, false, true, // enable z axis
		//                         true, // polarity
		//                         false, // don't latch
		//                         true); // enabled!
#elifdef USE_MLX90393
	mlx90393.setGain(MLX90393_GAIN_1X);
	Serial.print("Gain set to: ");
	switch (mlx90393.getGain()) {
		case MLX90393_GAIN_1X: Serial.println("1 x"); break;
		case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
		case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
		case MLX90393_GAIN_2X: Serial.println("2 x"); break;
		case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
		case MLX90393_GAIN_3X: Serial.println("3 x"); break;
		case MLX90393_GAIN_4X: Serial.println("4 x"); break;
		case MLX90393_GAIN_5X: Serial.println("5 x"); break;
	}

	// Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
	mlx90393.setResolution(MLX90393_X, MLX90393_RES_17);
	Serial.print("X Resolution set to: ");
	switch (mlx90393.getResolution(MLX90393_X)) {
		case MLX90393_RES_16: Serial.println("16"); break;
		case MLX90393_RES_17: Serial.println("17"); break;
		case MLX90393_RES_18: Serial.println("18"); break;
		case MLX90393_RES_19: Serial.println("19"); break;
	}

	mlx90393.setResolution(MLX90393_Y, MLX90393_RES_17);
	Serial.print("Y Resolution set to: ");
	switch (mlx90393.getResolution(MLX90393_Y)) {
		case MLX90393_RES_16: Serial.println("16"); break;
		case MLX90393_RES_17: Serial.println("17"); break;
		case MLX90393_RES_18: Serial.println("18"); break;
		case MLX90393_RES_19: Serial.println("19"); break;
	}

	mlx90393.setResolution(MLX90393_Z, MLX90393_RES_16);
	Serial.print("Z Resolution set to: ");
	switch (mlx90393.getResolution(MLX90393_Z)) {
		case MLX90393_RES_16: Serial.println("16"); break;
		case MLX90393_RES_17: Serial.println("17"); break;
		case MLX90393_RES_18: Serial.println("18"); break;
		case MLX90393_RES_19: Serial.println("19"); break;
	}

	// Set oversampling, see OSR at 16.2.5.
	mlx90393.setOversampling(MLX90393_OSR_3);
	Serial.print("Oversampling set to: ");
	switch (mlx90393.getOversampling()) {
		case MLX90393_OSR_3: Serial.println("3");
		case MLX90393_OSR_2: Serial.println("2");
		case MLX90393_OSR_1: Serial.println("1");
		case MLX90393_OSR_0: Serial.println("0");
	}

	// Set digital filtering, see DIG_FILT at 16.2.5.
	mlx90393.setFilter(MLX90393_FILTER_2);
	Serial.print("Filter set to: ");
	switch (mlx90393.getFilter()) {
		case MLX90393_FILTER_7: Serial.println("7");
		case MLX90393_FILTER_6: Serial.println("6");
		case MLX90393_FILTER_5: Serial.println("5");
		case MLX90393_FILTER_4: Serial.println("4");
		case MLX90393_FILTER_3: Serial.println("3");
		case MLX90393_FILTER_2: Serial.println("2");
		case MLX90393_FILTER_1: Serial.println("1");
		case MLX90393_FILTER_0: Serial.println("0");
	}
#elifdef USE_MMC5603
	mmc5603.printSensorDetails();

	mmc5603.setDataRate(100);
	Serial.print("Data Rate set to: ");
	Serial.println(mmc5603.getDataRate());

	mmc5603.setContinuousMode(true);
	Serial.print("Continuous Mode set to: ");
	Serial.println("true");
#elifdef USE_MMC5983MA
	mmc5983ma.setFilterBandwidth(100);
	Serial.print("Filter Bandwidth set to: ");
	Serial.println(mmc5983ma.getFilterBandwidth());

	mmc5983ma.setContinuousModeFrequency(100);
	Serial.print("Continuous Mode Frequency set to: ");
	Serial.println(mmc5983ma.getContinuousModeFrequency());

	mmc5983ma.setPeriodicSetSamples(1);
	Serial.print("PeriodicSetSamples set to: ");
	Serial.println(mmc5983ma.getPeriodicSetSamples());

	mmc5983ma.enableAutomaticSetReset();
	Serial.print("Automatic Set Reset set to: ");
	Serial.println(mmc5983ma.isAutomaticSetResetEnabled() ? "enabled" : "disabled");

	mmc5983ma.enableContinuousMode();
	Serial.print("Continuous Mode set to: ");
	Serial.println(mmc5983ma.isContinuousModeEnabled() ? "enabled" : "disabled");
#endif
		// hard-iron calibrate sensor
#ifdef USE_MMC5983MA
		// do {
		//	uint32_t x_value = 0, y_value = 0, z_value = 0;
		//	do {
		//		delay(50);
		//		mmc5983ma.readFieldsXYZ(&x_value, &y_value, &z_value);
		//	} while (!sixDOF.deviantSpread(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1), static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1), static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)));
		//	Serial.println("MMC5983MA read fields until deviantSpread!");
		// } while (!sixDOF.calOffsets());

#elifdef USE_LIS3MDL
		// do {
		//	do {
		//		delay(50);
		//		lis3mdl.read();
		//	} while (!sixDOF.deviantSpread(lis3mdl.x, lis3mdl.y, lis3mdl.z));
		// } while (!sixDOF.calOffsets());
#elifdef USE_MLX90393
	// do {
	//	int16_t x_value = 0, y_value = 0, z_value = 0;
	//	do {
	//		delay(50);
	//		mlx90393.readRawData(&x_value, &y_value, &z_value);
	//	} while (!sixDOF.deviantSpread(x_value, y_value, z_value));
	// } while (!sixDOF.calOffsets());
#elifdef USE_MMC5603
	// do {
	//	int32_t x_value = 0, y_value = 0, z_value = 0;
	//	do {
	//		delay(50);
	//		mmc5603.readRawData(&x_value, &y_value, &z_value);
	//	} while (!sixDOF.deviantSpread(x_value, y_value, z_value));
	// } while (!sixDOF.calOffsets());
#elifdef USE_AK09940A
	softReset();
	Serial.println("softReset()");
	disableI2C();
	Serial.println("disableI2C()");

	while (!check_company_device_id()) {
		Serial.println("Error in 'check_company_device_id()'");

		delay(1000);
	}

	// Power down mode:
	spiWrite(CNTL3, 0b0000'0000);
	delay(100);
	// disable Ultra low power drive setting

#ifdef CONTINUOUS
	spiWrite(CNTL1, 0b0000'0000);
	delay(100);
	// Temperature Sensor enable
	spiWrite(CNTL2, 0b0100'0000);
	// set continous mode to 100Hz and low noise drive 2
	spiWrite(CNTL3, 0b0110'1000);
#else
	spiWrite(CNTL1, 0b0010'0000);
	delay(100);
	// Temperature Sensor enable
	spiWrite(CNTL2, 0b0100'0000);
	// set external trigger mode and low noise drive 2
	spiWrite(CNTL3, 0b0111'1000);
#endif
#endif
	delay(1000);
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
	// static uint32_t count = 0;
	// count++;
	//
	// if (uint32_t const current_time = millis(); next_heartbeat < current_time) {  // sends heartbeat every second
	//	next_heartbeat = current_time + 1000;
	//
	//	Serial.print("Heartbeat: 'Nucleo', Hz = ");
	//	Serial.println(count);
	//	count = 0;
	//}

#ifdef USE_MMC5983MA
	uint32_t x_value = 0, y_value = 0, z_value = 0;
	if (!mmc5983ma.getMeasurementXYZ(&x_value, &y_value, &z_value)) return;

	float x = static_cast<float>(static_cast<int>(x_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;
	float y = static_cast<float>(static_cast<int>(y_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;
	float z = static_cast<float>(static_cast<int>(z_value) - (1 << MMC5983MA_MODE_BITS - 1)) / (1 << MMC5983MA_MODE_BITS - 1) * MMC5983MA_FULL_SCALE_RANGE_UTESLA;

	send_message(x, y, z, "MMC5983MA");
#elifdef USE_LIS3MDL
	sensors_event_t event;
	if (!lis3mdl.getEvent(&event)) return;

	send_message(event.magnetic.x, event.magnetic.y, event.magnetic.z, "LIS3MDL");
#elifdef USE_MLX90393
	sensors_event_t event;
	mlx90393.getEvent(&event);

	send_message(event.magnetic.x, event.magnetic.y, event.magnetic.z, "MLX90393");
#elifdef USE_MMC5603
	sensors_event_t event;
	mmc5603.getEvent(&event);

	send_message(event.magnetic.x, event.magnetic.y, event.magnetic.z, "MMC5603");
#elifdef USE_AK09940A
	// single measurement mode enable
	// spiWrite(CNTL3, 0b0110'0001);

#ifndef CONTINUOUS
	digitalWrite(TRG_PIN, HIGH);
	delayMicroseconds(100);  // > 3us
	digitalWrite(TRG_PIN, LOW);
	delay(5);  // > 3.1ms
#endif

	delay(13);

	digitalWrite(CS_PIN, LOW);
	for (auto i = 0;; ++i) {
		spi.transfer(ST1 | 0x80);
		if (auto const status = spi.transfer(0x00); (status & 0x01) == 1) break;
		if (i > 100) {
			digitalWrite(CS_PIN, HIGH);
			Serial.println("TIMEOUT DRDY");
			return;
		}
		delayMicroseconds(10);
	}
	std::uint8_t const raw0 = spi.transfer(0x00);
	std::uint8_t const raw1 = spi.transfer(0x00);
	std::uint8_t const raw2 = spi.transfer(0x00);
	std::uint8_t const raw3 = spi.transfer(0x00);
	std::uint8_t const raw4 = spi.transfer(0x00);
	std::uint8_t const raw5 = spi.transfer(0x00);
	std::uint8_t const raw6 = spi.transfer(0x00);
	std::uint8_t const raw7 = spi.transfer(0x00);
	std::uint8_t const raw8 = spi.transfer(0x00);
	std::uint8_t const raw9 = spi.transfer(0x00);
	std::uint8_t const dor = spi.transfer(0x00);

	digitalWrite(CS_PIN, HIGH);

	if ((dor & 0x01) == 1) {
		Serial.println("DATA OVERRUN");
		return;
	}

	// Serial.print("Temp Raw 10: ");
	// Serial.println(raw10);

	double const temp = 30.0 - raw9 / 1.7;

	// Serial.print("Temp: ");
	// Serial.println(temp);

	// Serial.print("Raw 1-9: ");
	// Serial.print(raw1);
	// Serial.print(", ");
	// Serial.print(raw2);
	// Serial.print(", ");
	// Serial.print(raw3);
	// Serial.print(", ");
	// Serial.print(raw4);
	// Serial.print(", ");
	// Serial.print(raw5);
	// Serial.print(", ");
	// Serial.print(raw6);
	// Serial.print(", ");
	// Serial.print(raw7);
	// Serial.print(", ");
	// Serial.print(raw8);
	// Serial.print(", ");
	// Serial.println(raw9);

	std::uint32_t const x_raw = (static_cast<std::uint32_t>(raw2 & 0x03) << 16) | (static_cast<std::uint32_t>(raw1) << 8) | static_cast<std::uint32_t>(raw0);
	std::uint32_t const y_raw = (static_cast<std::uint32_t>(raw5 & 0x03) << 16) | (static_cast<std::uint32_t>(raw4) << 8) | static_cast<std::uint32_t>(raw3);
	std::uint32_t const z_raw = (static_cast<std::uint32_t>(raw8 & 0x03) << 16) | (static_cast<std::uint32_t>(raw7) << 8) | static_cast<std::uint32_t>(raw6);

	std::int32_t const x_raw2 = static_cast<std::int32_t>(x_raw & 0x20000 ? x_raw - 0x40000 : x_raw);
	std::int32_t const y_raw2 = static_cast<std::int32_t>(y_raw & 0x20000 ? y_raw - 0x40000 : y_raw);
	std::int32_t const z_raw2 = static_cast<std::int32_t>(z_raw & 0x20000 ? z_raw - 0x40000 : z_raw);

	AK09940A::MagneticFluxDensityDataRaw ak09940a;

	ak09940a.x = x_raw2;
	ak09940a.y = y_raw2;
	ak09940a.z = z_raw2;

	// static CRC16 crc(0x8005, 0, false, true, true);
	// crc.restart();

	// constexpr std::array<uint8_t, 2> const header = {'H', 'i'};
	// Serial.write(header.data(), 2);

	// auto const scale_ak09940a = std::bit_cast<std::array<std::uint8_t, sizeof(AK09940A::get_scale_factor())> >(AK09940A::get_scale_factor());
	// Serial.write(scale_ak09940a.data(), scale_ak09940a.size());
	// crc.add(scale_ak09940a.data(), scale_ak09940a.size());

	// Serial.write(ak09940a.bytes, sizeof(AK09940A::MagneticFluxDensityDataRaw));
	// crc.add(ak09940a.bytes, sizeof(AK09940A::MagneticFluxDensityDataRaw));

	// auto crc_value = std::bit_cast<std::array<uint8_t, 2> >(crc.calc());
	//  Serial.write(crc_value.data(), 2);

	// Serial.print("Xraw: ");
	// Serial.print(x_raw2);
	// Serial.print(", Yraw: ");
	// Serial.print(y_raw);
	// Serial.print(", Zraw: ");
	// Serial.println(z_raw);

	constexpr double scale = 0.01;

	double const x_uT = x_raw2 * scale;
	double const y_uT = y_raw2 * scale;
	double const z_uT = z_raw2 * scale;

	stats_x.addValue(x_uT);
	stats_y.addValue(y_uT);
	stats_z.addValue(z_uT);

	// Serial.print("X: ");
	// Serial.print(x_uT, 2);
	// Serial.print(", Y: ");
	// Serial.print(y_uT, 2);
	// Serial.print(", Z: ");
	// Serial.println(z_uT, 2);

	// Serial.print("mean X: ");
	// Serial.print(mean_x, 2);
	// Serial.print(", mean Y: ");
	// Serial.print(mean_y, 2);
	// Serial.print(", mean Z: ");
	// Serial.print(mean_z, 2);

	Serial.print("std X: ");
	Serial.print(stats_x.getStandardDeviation(), 2);
	Serial.print(", std Y: ");
	Serial.print(stats_y.getStandardDeviation(), 2);
	Serial.print(", std Z: ");
	Serial.println(stats_z.getStandardDeviation(), 2);

	// Serial.print("rms X: ");
	// Serial.print(rms_x, 2);
	// Serial.print(", rms X Y: ");
	// Serial.print(rms_y, 2);
	// Serial.print(", rms X Z: ");
	// Serial.println(rms_z, 2);

	// Serial.print(", std ges: ");

#endif
}