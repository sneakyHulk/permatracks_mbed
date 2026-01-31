#pragma once

#include <SPI.h>

#include <cstdint>

#include <common2_output.h>

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

		power_down();

		// disable I2C
		common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable I2C...");
		spiWrite(I2CDIS, 0b0001'1011);
		delayMicroseconds(200);
		common2::println("Done!");

		// do soft reset
		common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " do soft reset...");
		spiWrite(CNTL4, 0b0000'0001);  // SRST = 1 → soft reset
		delayMicroseconds(2000);       // > 100 µs Twait
		while (spiRead(CNTL4) & 0b0000'0001) {
			common2::print("Error! Waiting on soft reset to complete...");
			delay(100);
		}
		common2::println("Done!");

		// check sensor
		check_company_device_id();
		check_company_device_id();
		check_company_device_id();

		power_down();

		// enable temperature sensor
		common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " enable temperature sensor...");
		spiWrite(CNTL2, 0b0100'0000);
		delayMicroseconds(200);
		common2::println("Done!");

		if (continuous) {
			// disable ultra low power drive setting and data ready output
			common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable Ultra low power drive setting and data ready output...");
			std::uint8_t const cntl1 = spiRead(CNTL1);
			spiWrite(CNTL1, cntl1 & 0b0100'0000);  // dont change up reserved RSV28 bit.
			delayMicroseconds(200);
			common2::println("Done!");

			// set continuous mode to 100Hz and low noise drive 2
			common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " set continuous mode to 100Hz and low noise drive 2...");
			spiWrite(CNTL3, 0b0110'1000);
			delayMicroseconds(200);
			common2::println("Done!");
		} else {
			// disable ultra low power drive setting and external trigger pulse input
			common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " disable Ultra low power drive setting and external trigger pulse input...");
			std::uint8_t const cntl1 = spiRead(CNTL1);
			spiWrite(CNTL1, (cntl1 & 0b0100'0000) | 0b0010'0000);  // dont change up reserved RSV28 bit.
			delayMicroseconds(200);
			common2::println("Done!");

			// set external trigger mode and low noise drive 2
			common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " set external trigger mode and low noise drive 2...");
			spiWrite(CNTL3, 0b0111'1000);
			delayMicroseconds(200);
			common2::println("Done!");
		}

		common2::println_time_loc(millis(), '\'', "AK09940A ", n, '\'', " Ready!");
	}

	void start_measurement() const {
		if (continuous) {
			// nothing to do
		}
	}

	void power_down() const {
		// power down mode
		common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " switch to power down mode...");
		spiWrite(CNTL3, 0b0000'0000);
		delayMicroseconds(1000);

		while (spiRead(CNTL3) & 0b0001'1111) {
			common2::print("Error! Retry...");
			spiWrite(CNTL3, 0b0000'0000);
			delay(100);
		}

		common2::println("Done!");
	}

	[[nodiscard]] MagneticFluxDensityDataRaw get_measurement() const {
		digitalWrite(cs_pin, LOW);

		// polling does not work! -> don't need for trg, because it is always one in the buffer after wait period!
		// do {
		//	spi->transfer(ST | 0x80);
		//
		//	if (std::uint8_t const status = spi->transfer(0x00); status & 0b0000'0001) break;
		//} while (true);

		do {
			spi->transfer(0x00 | 0x80);
			std::uint8_t const wia1 = spi->transfer(0x00);
			std::uint8_t const wia2 = spi->transfer(0x00);
			std::uint8_t const rsv1 = spi->transfer(0x00);
			std::uint8_t const rsv2 = spi->transfer(0x00);
			std::uint8_t const st1 = spi->transfer(0x00);

			// common::print("WIA1: ", wia1, "/", EXPECTED_WIA1, ", WIA2: ", wia2, "/", EXPECTED_WIA2, "!");

			if (wia1 == EXPECTED_WIA1 && wia2 == EXPECTED_WIA2 && st1 & 0b0000'0001) break;
		} while (true);

		std::uint8_t const hxl = spi->transfer(0x00);
		std::uint8_t const hxm = spi->transfer(0x00);
		std::uint8_t const hxh = spi->transfer(0x00);
		std::uint8_t const hyl = spi->transfer(0x00);
		std::uint8_t const hym = spi->transfer(0x00);
		std::uint8_t const hyh = spi->transfer(0x00);
		std::uint8_t const hzl = spi->transfer(0x00);
		std::uint8_t const hzm = spi->transfer(0x00);
		std::uint8_t const hzh = spi->transfer(0x00);
		std::uint8_t const tmps = spi->transfer(0x00);
		std::uint8_t const dor = spi->transfer(0x00);

		digitalWrite(cs_pin, HIGH);

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

		return MagneticFluxDensityDataRaw{.x = hx, .y = hy, .z = hz};
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

	void check_company_device_id() const {
		common2::print_time_loc(millis(), '\'', "AK09940A ", n, '\'', " check company device id...");

		digitalWrite(cs_pin, LOW);
		delayMicroseconds(200);
		spi->transfer(WHO_AM_I1_ADDR | 0x80);
		std::uint8_t wia1 = spi->transfer(0x00);
		std::uint8_t wia2 = spi->transfer(0x00);
		digitalWrite(cs_pin, HIGH);
		delayMicroseconds(200);

		while ((wia1 != EXPECTED_WIA1) || (wia2 != EXPECTED_WIA2)) {
			common2::print("Error! WIA1: ", wia1, "/", EXPECTED_WIA1, ", WIA2: ", wia2, "/", EXPECTED_WIA2, "! Retry...");
			delay(100);

			digitalWrite(cs_pin, LOW);
			delayMicroseconds(200);
			spi->transfer(WHO_AM_I1_ADDR | 0x80);
			wia1 = spi->transfer(0x00);
			wia2 = spi->transfer(0x00);
			digitalWrite(cs_pin, HIGH);
			delayMicroseconds(200);
		}

		common2::println("Done!");
	}
};