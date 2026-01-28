#pragma once

#include <SPI.h>

#include <cstdint>

#include "../common/common.h"
#include "../common/common_output.h"

class AK09940A final {
	inline static std::uint8_t N = 0;

   public:
	std::uint8_t n = N++;

#pragma pack(push, 1)
	struct MagneticFluxDensityDataRaw {
		union {
			struct {
				std::int32_t x : 18;
				std::int32_t y : 18;
				std::int32_t z : 18;
			};
			std::uint8_t bytes[7];
		};
	};
#pragma pack(pop)

	static std::uint32_t get_scale_factor() { return 100'000'000U; }

   private:
	SPIClass* spi;
	std::uint8_t const cs_pin;
	bool const continuous;

	// registers
	constexpr static std::uint8_t CNTL4 = 0x33;   // SRST bit (D0)
	constexpr static std::uint8_t I2CDIS = 0x36;  // lock out I²C
	constexpr static std::uint8_t CNTL3 = 0x32;   // MODE[4:0]
	constexpr static std::uint8_t CNTL2 = 0x31;   // TEMP
	constexpr static std::uint8_t CNTL1 = 0x30;   // MT2
	constexpr static std::uint8_t ST = 0x0F;      // DRDY flag
	constexpr static std::uint8_t ST1 = 0x10;     // ST1
	constexpr static std::uint8_t ST2 = 0x1B;     // ST2
	constexpr static std::uint8_t HXL = 0x11;     // mag data read start
	constexpr static std::uint8_t WHO_AM_I1_ADDR = 0x00;
	constexpr static std::uint8_t WHO_AM_I2_ADDR = 0x01;

	// defined values
	constexpr static std::uint8_t EXPECTED_WIA1 = 0x48;  // Company ID ("AKM")
	constexpr static std::uint8_t EXPECTED_WIA2 = 0xA3;  // Device ID (AK09940A)

   public:
	AK09940A(SPIClass* spi, std::uint8_t const cs_pin, bool const continuous = true) : spi(spi), cs_pin(cs_pin), continuous(continuous) {}

	void begin() {
		pinMode(cs_pin, OUTPUT);
		digitalWrite(cs_pin, HIGH);

		// do soft reset
		common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " do soft reset...");
		spiWrite(CNTL4, 0x01);  // SRST = 1 → soft reset
		delay(100);             // > 100 µs Twait
		common::println("Done!");

		// disable I2C
		common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable I2C...");
		spiWrite(I2CDIS, 0b0001'1011);
		delay(100);
		common::println("Done!");

		// check sensor
		common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " check company device id...");
		while (!check_company_device_id()) {
			common::print("Error! Retry...");

			delay(1000);
		}
		common::println("Done!");

		// power down mode
		common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " switch to power down mode...");
		spiWrite(CNTL3, 0b0000'0000);
		delay(100);
		common::println("Done!");

		if (continuous) {
			// disable ultra low power drive setting and data ready output
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable Ultra low power drive setting and data ready output...");
			spiWrite(CNTL1, 0b0000'0000);
			delay(100);
			common::println("Done!");

			// enable temperature sensor
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " enable temperature sensor...");
			spiWrite(CNTL2, 0b0100'0000);
			common::println("Done!");

			// set continuous mode to 100Hz and low noise drive 2
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " set continuous mode to 100Hz and low noise drive 2...");
			spiWrite(CNTL3, 0b0110'1000);
			common::println("Done!");
		} else {
			// disable ultra low power drive setting and external trigger pulse input
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable Ultra low power drive setting and external trigger pulse input...");
			spiWrite(CNTL1, 0b0010'0000);
			delay(100);
			common::println("Done!");

			// enable temperature sensor
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " enable temperature sensor...");
			spiWrite(CNTL2, 0b0100'0000);
			common::println("Done!");

			// set external trigger mode and low noise drive 2
			common::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " set external trigger mode and low noise drive 2...");
			spiWrite(CNTL3, 0b0111'1000);
			common::println("Done!");
		}

		common::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " Ready!");
	}

	void start_measurement() const {
		if (continuous) {
			// nothing to do
		}
	}

	[[nodiscard]] MagneticFluxDensityDataRaw get_measurement() const {
		digitalWrite(cs_pin, LOW);
		for (auto i = 0;; ++i) {
			spi->transfer(ST1 | 0x80);
			if (auto const status = spi->transfer(0x00); (status & 0x01) == 1) break;
			if (i > 100) {
				digitalWrite(cs_pin, HIGH);
				common::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " TIMEOUT DRDY!");
				return {};
			}
			delayMicroseconds(10);
		}

		std::uint8_t const raw0 = spi->transfer(0x00);
		std::uint8_t const raw1 = spi->transfer(0x00);
		std::uint8_t const raw2 = spi->transfer(0x00);
		std::uint8_t const raw3 = spi->transfer(0x00);
		std::uint8_t const raw4 = spi->transfer(0x00);
		std::uint8_t const raw5 = spi->transfer(0x00);
		std::uint8_t const raw6 = spi->transfer(0x00);
		std::uint8_t const raw7 = spi->transfer(0x00);
		std::uint8_t const raw8 = spi->transfer(0x00);
		std::uint8_t const raw9 = spi->transfer(0x00);
		std::uint8_t const dor = spi->transfer(0x00);

		digitalWrite(cs_pin, HIGH);

		if ((dor & 0x01) == 1) {
			common::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " data has been skipped!");
		}

		double const temp = 30.0 - static_cast<std::int8_t>(raw9) / 1.7;

		common::println('\'', "AK09940A ", n, '\'', " ", temp, "°C");

		std::uint32_t const x_raw = (static_cast<std::uint32_t>(raw2 & 0x03) << 16) | (static_cast<std::uint32_t>(raw1) << 8) | static_cast<std::uint32_t>(raw0);
		std::uint32_t const y_raw = (static_cast<std::uint32_t>(raw5 & 0x03) << 16) | (static_cast<std::uint32_t>(raw4) << 8) | static_cast<std::uint32_t>(raw3);
		std::uint32_t const z_raw = (static_cast<std::uint32_t>(raw8 & 0x03) << 16) | (static_cast<std::uint32_t>(raw7) << 8) | static_cast<std::uint32_t>(raw6);

		std::int32_t const x_raw2 = static_cast<std::int32_t>(x_raw & 0x20000 ? x_raw - 0x40000 : x_raw);
		std::int32_t const y_raw2 = static_cast<std::int32_t>(y_raw & 0x20000 ? y_raw - 0x40000 : y_raw);
		std::int32_t const z_raw2 = static_cast<std::int32_t>(z_raw & 0x20000 ? z_raw - 0x40000 : z_raw);

		return MagneticFluxDensityDataRaw{.x = x_raw2, .y = y_raw2, .z = z_raw2};
	}

   private:
	void spiWrite(uint8_t const reg, uint8_t const data) const {
		digitalWrite(cs_pin, LOW);
		spi->transfer(reg & 0x7F);  // bit-7 = 0 → write
		spi->transfer(data);
		digitalWrite(cs_pin, HIGH);
	}

	[[nodiscard]] std::uint8_t spiRead(uint8_t const reg) const {
		digitalWrite(cs_pin, LOW);
		spi->transfer(reg | 0x80);  // bit-7 = 1 → read
		std::uint8_t const val = spi->transfer(0x00);
		digitalWrite(cs_pin, HIGH);
		return val;
	}

	[[nodiscard]] bool check_company_device_id() const {
		std::uint8_t const wia1 = spiRead(WHO_AM_I1_ADDR);
		std::uint8_t const wia2 = spiRead(WHO_AM_I2_ADDR);

		if ((wia1 == EXPECTED_WIA1) && (wia2 == EXPECTED_WIA2)) {
			return true;
		}

		common::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " does not match the expected values. WIA1: ", wia1, "/", EXPECTED_WIA1, ", WIA2: ", wia2, "/", EXPECTED_WIA2, "!");
		return false;
	}
};