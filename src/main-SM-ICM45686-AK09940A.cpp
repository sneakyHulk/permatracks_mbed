#include <Arduino.h>
#include <Wire.h>
#include <common2.h>

#include <cstdint>

#include "RunningAverage.h"

RunningAverage stats_x(100);
RunningAverage stats_y(100);
RunningAverage stats_z(100);

constexpr std::uint8_t CNTL4 = 0x33;          // SRST bit (D0)
constexpr std::uint8_t I2CDIS = 0x36;         // lock out I²C
constexpr std::uint8_t CNTL3 = 0x32;          // MODE[4:0]
constexpr std::uint8_t CNTL2 = 0x31;          // TEMP
constexpr std::uint8_t CNTL1 = 0x30;          // MT2
constexpr std::uint8_t ST = 0x0F;             // DRDY flag
constexpr std::uint8_t ST1 = 0x10;            // ST1
constexpr std::uint8_t ST2 = 0x1B;            // ST2
constexpr std::uint8_t HXL = 0x11;            // mag data read start
constexpr std::uint8_t EXPECTED_WIA1 = 0x48;  // Company ID ("AKM")
constexpr std::uint8_t EXPECTED_WIA2 = 0xA3;  // Device ID (AK09940A)
constexpr std::uint8_t WHO_AM_I1_ADDR = 0x00;
constexpr std::uint8_t WHO_AM_I2_ADDR = 0x01;
constexpr std::uint8_t DEVICE_ADDR = 0x0C;

bool check_company_id() {
	for (auto i = 0; i < 3; ++i) {
		Wire.beginTransmission(DEVICE_ADDR);
		Wire.write(WHO_AM_I1_ADDR);
		Wire.endTransmission(false);
		Wire.requestFrom(DEVICE_ADDR, 2U);
		Wire.endTransmission();

		if (Wire.available() >= 2) {
			std::uint8_t const wia1 = Wire.read();
			std::uint8_t const wia2 = Wire.read();

			if (wia1 == EXPECTED_WIA1 && wia2 == EXPECTED_WIA2) {
				common2::println("Done!");
				return true;
			}
			common2::print("Error! WIA1: ", wia1, "/", EXPECTED_WIA1, ", WIA2: ", wia2, "/", EXPECTED_WIA2, "! Retry...");
		}
	}

	return false;
}

bool disable_ultra_low_power_drive() {
	Wire.beginTransmission(DEVICE_ADDR);
	Wire.write(CNTL1);
	Wire.endTransmission(false);

	Wire.requestFrom(DEVICE_ADDR, 1U);

	Wire.endTransmission();

	if (Wire.available() >= 1) {
		std::uint8_t const cntl1 = Wire.read();

		Wire.beginTransmission(DEVICE_ADDR);
		Wire.write(CNTL1);
		Wire.write(cntl1 & 0b01000000);
		Wire.endTransmission();

		return true;
	}

	return false;
}

bool setup_continous() {
	Wire.beginTransmission(DEVICE_ADDR);
	Wire.write(CNTL3);
	Wire.write(0b0110'1000);
	Wire.endTransmission();

	return true;
}

void setup() {
	Serial.begin(230400);

	Wire.begin();

	delay(1000);

	if (!check_company_id())
		while (true);

	delay(1000);

	if (!disable_ultra_low_power_drive())
		while (true);

	delay(1000);

	if (!setup_continous())
		while (true);

	delay(1000);
}

bool ready() {
	Wire.beginTransmission(DEVICE_ADDR);
	Wire.write(ST);
	Wire.endTransmission(false);
	Wire.requestFrom(DEVICE_ADDR, 1U);
	Wire.endTransmission();

	if (Wire.available()) {
		std::uint8_t const st = Wire.read();

		return st & 0b00000001;
	}

	return false;
}

void loop() {
	if (ready()) {
		Wire.beginTransmission(DEVICE_ADDR);
		Wire.write(ST1);
		Wire.endTransmission(false);
		Wire.requestFrom(DEVICE_ADDR, 12U);
		Wire.endTransmission();

		std::uint8_t const st1 = Wire.read();
		std::uint8_t const hxl = Wire.read();
		std::uint8_t const hxm = Wire.read();
		std::uint8_t const hxh = Wire.read();
		std::uint8_t const hyl = Wire.read();
		std::uint8_t const hym = Wire.read();
		std::uint8_t const hyh = Wire.read();
		std::uint8_t const hzl = Wire.read();
		std::uint8_t const hzm = Wire.read();
		std::uint8_t const hzh = Wire.read();
		std::uint8_t const tmps = Wire.read();
		std::uint8_t const dor = Wire.read();

		if ((dor & 0x01) == 1) {
			// common::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " data has been skipped!");
		}

		double const temp = 30.0 - static_cast<std::int8_t>(tmps) / 1.7;

		// common::println('\'', "AK09940A ", n, '\'', " ", temp, "°C");

		std::uint32_t const x_raw = (static_cast<std::uint32_t>(hxh & 0x03) << 16) | (static_cast<std::uint32_t>(hxm) << 8) | static_cast<std::uint32_t>(hxl);
		std::uint32_t const y_raw = (static_cast<std::uint32_t>(hyh & 0x03) << 16) | (static_cast<std::uint32_t>(hym) << 8) | static_cast<std::uint32_t>(hyl);
		std::uint32_t const z_raw = (static_cast<std::uint32_t>(hzh & 0x03) << 16) | (static_cast<std::uint32_t>(hzm) << 8) | static_cast<std::uint32_t>(hzl);

		std::int32_t const hx = static_cast<std::int32_t>(x_raw & 0x20000 ? x_raw - 0x40000 : x_raw);
		std::int32_t const hy = static_cast<std::int32_t>(y_raw & 0x20000 ? y_raw - 0x40000 : y_raw);
		std::int32_t const hz = static_cast<std::int32_t>(z_raw & 0x20000 ? z_raw - 0x40000 : z_raw);

		double const x_T = static_cast<double>(hx) / 100'000'000.;
		double const y_T = static_cast<double>(hy) / 100'000'000.;
		double const z_T = static_cast<double>(hz) / 100'000'000.;

		stats_x.addValue(x_T);
		stats_y.addValue(y_T);
		stats_z.addValue(z_T);

		common2::println(static_cast<std::uint64_t>(micros()) * 1'000U, " ns, ", x_T * 1e6, " uT, ", y_T * 1e6, " uT, ", z_T * 1e6, " uT, std X: ", stats_x.getStandardDeviation() * 1e6, " uT, std Y: ", stats_y.getStandardDeviation() * 1e6,
		    " uT, std Z: ", stats_z.getStandardDeviation() * 1e6, " uT");
	}
}