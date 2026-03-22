#include <Arduino.h>
#include <SPI.h>

#include <cstdint>

#include "RunningAverage.h"

RunningAverage stats_x(100);
RunningAverage stats_y(100);
RunningAverage stats_z(100);

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

auto settings = SPISettings(1'000'000, BitOrder::MSBFIRST, SPI_MODE3);

void spi_write(std::uint8_t const reg, std::uint8_t const data) {
	spi.beginTransaction(settings);
	digitalWrite(CS_PIN, LOW);

	spi.transfer(reg & 0x7F);  // bit-7 = 0 → write
	spi.transfer(data);

	digitalWrite(CS_PIN, HIGH);
	spi.endTransaction();
}

[[nodiscard]] std::uint8_t spi_read(uint8_t const reg) {
	spi.beginTransaction(settings);
	digitalWrite(CS_PIN, LOW);

	spi.transfer(reg | 0x80);  // bit-7 = 1 → read
	std::uint8_t const val = spi.transfer(0x00);

	digitalWrite(CS_PIN, HIGH);
	spi.endTransaction();
	return val;
}

void setup() {
	Serial.begin(230400);

	delay(1000);
	Serial.println("Hello World");
	delay(1000);

	pinMode(CS_PIN, OUTPUT);
	digitalWrite(CS_PIN, HIGH);

	delay(100);

	pinMode(TRG_PIN, OUTPUT);
	digitalWrite(TRG_PIN, LOW);

	delay(100);

	// power down Mode
	spi_write(CNTL3, 0b0000'0000);

	if (auto const mode = spi_read(CNTL3); mode & 0b0001'1111) {
		while (true);
	}

	delayMicroseconds(1000);  // Twait ≥ 100µs
	Serial.println("Power Down!");

	spi_write(I2CDIS, 0b0001'1011);

	delayMicroseconds(1000);

	auto cntl1 = spi_read(CNTL1);
	Serial.println(cntl1, BIN);
	cntl1 &= ~0b0000'0111;  // clear WM
	cntl1 |= 0b0000'0111;
	spi_write(CNTL1, cntl1);

	delayMicroseconds(1000);
	cntl1 = spi_read(CNTL1);
	Serial.println(cntl1, BIN);

	spi_write(CNTL3, 0b1110'0010);

	delayMicroseconds(1000);

	if (auto const mode = spi_read(CNTL3); (mode & 0b0001'1111) != 0b0000'0010) {
		while (true);
	} else {
		Serial.println(mode, BIN);
	}

	Serial.println("Continous Mode 10Hz");
}

bool ready() {
	auto const st = spi_read(ST);

	return st & 0b0000'0001;
}

void loop() {
	if (ready()) {
		for (int i = 0; i < 8; i++) {
			spi.beginTransaction(settings);
			digitalWrite(CS_PIN, LOW);

			spi.transfer(ST1 | 0x80);
			std::uint8_t const st1 = spi.transfer(0x00);

			// if (!(st1 & 0b0000'0001)) {
			// 	Serial.println("Not DRDY");
			// 	digitalWrite(CS_PIN, HIGH);
			// 	spi.endTransaction();
			// 	break;
			// }

			std::uint8_t const hxl = spi.transfer(0x00);
			std::uint8_t const hxm = spi.transfer(0x00);
			std::uint8_t const hxh = spi.transfer(0x00);
			std::uint8_t const hyl = spi.transfer(0x00);
			std::uint8_t const hym = spi.transfer(0x00);
			std::uint8_t const hyh = spi.transfer(0x00);
			std::uint8_t const hzl = spi.transfer(0x00);
			std::uint8_t const hzm = spi.transfer(0x00);
			std::uint8_t const hzh = spi.transfer(0x00);
			std::uint8_t const tmps = spi.transfer(0x00);
			std::uint8_t const dor = spi.transfer(0x00);

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

			Serial.print("X: ");
			Serial.print(x_T * 1e6);
			Serial.print(", Y: ");
			Serial.print(y_T * 1e6);
			Serial.print(", Z: ");
			Serial.print(z_T * 1e6);
			Serial.print(", std X: ");
			Serial.print(stats_x.getStandardDeviation() * 1e6, 2);
			Serial.print(", std Y: ");
			Serial.print(stats_y.getStandardDeviation() * 1e6, 2);
			Serial.print(", std Z: ");
			Serial.println(stats_z.getStandardDeviation() * 1e6, 2);
			Serial.println();

			digitalWrite(CS_PIN, HIGH);
			spi.endTransaction();
		}
	}
}

// #include <Arduino.h>
//
// #undef Serial
// HardwareSerial Serial12(PB7, PB6);
// #define Serial Serial12
//
// #include <common_message.h>
// #include <common_output.h>
// #include <common_time.h>
//
// void setup() {
//	Serial.begin(230400);
//	delay(2000);
//
//	pinMode(PC13, OUTPUT);
//	digitalWrite(PC13, HIGH);
//
//	common::message(0.001, 102020, "test");
//	common::println("Listening: ");
// }
//
// void loop() {
//	auto const offset = common::sync_time();
//	common::message("Offset: ", offset);  // theoretical max = 0.8246 ms
//	delay(1000);
//
//	return;
//
//	common::message(0.001, 102020, "test");
//	if (static bool first = true; first) {
//		first = false;
//
//		// turn led on
//		digitalWrite(PC13, LOW);
//	}
// }