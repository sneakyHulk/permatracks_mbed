#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>

#include <bit>
#include <cstdint>

#include "AK09940A.h"
#include "CRC16.h"

bool led_state = LOW;

auto i2c2 = TwoWire(PB_11, PB_10);
auto gyro = LSM6DSV16XSensor(&i2c2);

auto spi1 = SPIClass(PA_7, PA_6, PA_5);
auto spi2 = SPIClass(PB_15, PB_14, PB_13);
auto spi3 = SPIClass(PC_12, PC_11, PC_10);

constexpr std::uint8_t CS_PIN = PA_9;

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
	spi3.transfer(reg & 0x7F);  // bit-7 = 0 → write
	spi3.transfer(data);
	digitalWrite(CS_PIN, HIGH);
}

inline uint8_t spiRead(uint8_t reg) {
	digitalWrite(CS_PIN, LOW);
	spi3.transfer(reg | 0x80);  // bit-7 = 1 → read
	uint8_t val = spi3.transfer(0x00);
	digitalWrite(CS_PIN, HIGH);
	return val;
}

bool check_company_device_id() {
	constexpr std::uint8_t WHO_AM_I1_ADDR = 0x00;
	constexpr std::uint8_t WHO_AM_I2_ADDR = 0x01;

	constexpr std::uint8_t EXPECTED_WIA1 = 0x48;  // Company ID ("AKM")
	constexpr std::uint8_t EXPECTED_WIA2 = 0xA3;  // Device ID (AK09940A)

	std::uint8_t wia1 = spiRead(WHO_AM_I1_ADDR);
	Serial.print("WIA1: ");
	Serial.println(wia1);

	std::uint8_t wia2 = spiRead(WHO_AM_I2_ADDR);
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

bool waitDRDY(uint16_t timeout_ms = 2000) {
	uint32_t t0 = millis();
	while (millis() - t0 < timeout_ms) {
		delayMicroseconds(10);
		auto status = spiRead(ST);
		if (status & 0x01)  // DRDY bit = 1?
			return true;
	}
	return false;  // timeout
}

void setup() {
	{  // turn led on
		pinMode(PD_10, OUTPUT);
		digitalWrite(PD_10, led_state);
	}

	delay(3000);

	{  // config Serial over USB; USB Speed ca. ~8.1 Mbit/s
		Serial.begin();
		delay(100);
		Serial.println("Hello over USB");
	}

	delay(100);

	{  // config gyro
		i2c2.begin();
		delay(100);

		// Initialize LSM6DSV16X
		gyro.begin();

		if (std::uint8_t id; gyro.ReadID(&id) == LSM6DSV16X_OK && id == 0x70u) {
			Serial.print("Initialized LSM6DSV16X, ID: ");
			Serial.println(id);

			gyro.Enable_X();
			gyro.Enable_G();

			// Enable Sensor Fusion
			int status = LSM6DSV16X_OK;
			status |= gyro.Set_X_FS(4);
			status |= gyro.Set_G_FS(2000);
			status |= gyro.Set_X_ODR(120.0f);
			status |= gyro.Set_G_ODR(120.0f);
			status |= gyro.Set_SFLP_ODR(120.0f);
			status |= gyro.Enable_Rotation_Vector();
			status |= gyro.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

			if (status != LSM6DSV16X_OK) {
				Serial.println("LSM6DSV16X Sensor failed to init/configure");
			}
		} else {
			Serial.print("Initialization of LSM6DSV16X failed, ID: ");
			Serial.println(id);
		}
	}

	delay(100);

	{  // connect AK09940A
		pinMode(CS_PIN, OUTPUT);
		digitalWrite(CS_PIN, HIGH);

		delay(10);
		spi3.beginTransaction(SPISettings(3'000'000, MSBFIRST, SPI_MODE3));  // AK09940A uses SPI Mode 3
	}

	{  // config AK09940A
		softReset();
		disableI2C();

		while (!check_company_device_id()) {
			Serial.println("Error in 'check_company_device_id()'");

			delay(1000);
		}

		// Power down mode:
		spiWrite(CNTL3, 0b0000'0000);
		delay(100);
		// disable Ultra low power drive setting
		spiWrite(CNTL1, 0b0000'0000);
		delay(100);
		// Temperature Sensor enable
		spiWrite(CNTL2, 0b0100'0000);
		// set continous mode to 100Hz and low noise drive 2
		spiWrite(CNTL3, 0b0110'1000);
	}

	Serial.println("Ready.");
}

void loop() {
	{  // blink LED
		digitalWrite(PD_10, led_state = !led_state);
		delay(200);
	}

	{  // poll gyro
		if (std::uint16_t samples = 0; gyro.FIFO_Get_Num_Samples(&samples) == LSM6DSV16X_OK) {
			for (int i = 0; i < samples; i++) {
				std::uint8_t tag = 0;
				gyro.FIFO_Get_Tag(&tag);

				if (tag == 0x13u) {
					float quaternions[4] = {0};
					gyro.FIFO_Get_Rotation_Vector(&quaternions[0]);

					// Print Quaternion data
					Serial.print("Quaternion: ");
					Serial.print(quaternions[3], 4);
					Serial.print(", ");
					Serial.print(quaternions[0], 4);
					Serial.print(", ");
					Serial.print(quaternions[1], 4);
					Serial.print(", ");
					Serial.println(quaternions[2], 4);
				} else {
					Serial.print("Unknown tag: ");
					Serial.println(tag);

					break;
				}
			}
		} else {
			Serial.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
			while (true);
		}
	}

	{  // poll AK09940A
		if (!waitDRDY()) {
			Serial.println("Timeout waiting for DRDY");
		} else {
			// delayMicroseconds(10);
			spiRead(ST1);
			// delayMicroseconds(10);
			std::uint8_t raw1 = spiRead(0x11);
			// delayMicroseconds(10);
			std::uint8_t raw2 = spiRead(0x12);
			// delayMicroseconds(10);
			std::uint8_t raw3 = spiRead(0x13);
			// delayMicroseconds(10);
			std::uint8_t raw4 = spiRead(0x14);
			// delayMicroseconds(10);
			std::uint8_t raw5 = spiRead(0x15);
			// delayMicroseconds(10);
			std::uint8_t raw6 = spiRead(0x16);
			// delayMicroseconds(10);
			std::uint8_t raw7 = spiRead(0x17);
			// delayMicroseconds(10);
			std::uint8_t raw8 = spiRead(0x18);
			// delayMicroseconds(10);
			std::uint8_t raw9 = spiRead(0x19);
			// delayMicroseconds(10);
			std::int8_t raw10 = spiRead(0x1a);
			// delayMicroseconds(10);
			spiRead(ST2);

			Serial.print("Temp Raw 10: ");
			Serial.println(raw10);

			double const temp = 30.0 - raw10 / 1.7;

			Serial.print("Temp: ");
			Serial.println(temp);

			Serial.print("Raw 1-9: ");
			Serial.print(raw1);
			Serial.print(", ");
			Serial.print(raw2);
			Serial.print(", ");
			Serial.print(raw3);
			Serial.print(", ");
			Serial.print(raw4);
			Serial.print(", ");
			Serial.print(raw5);
			Serial.print(", ");
			Serial.print(raw6);
			Serial.print(", ");
			Serial.print(raw7);
			Serial.print(", ");
			Serial.print(raw8);
			Serial.print(", ");
			Serial.println(raw9);

			std::uint32_t const x_raw = (static_cast<std::uint32_t>(raw3 & 0x03) << 16) | (static_cast<std::uint32_t>(raw2) << 8) | static_cast<std::uint32_t>(raw1);
			std::uint32_t const y_raw = (static_cast<std::uint32_t>(raw6 & 0x03) << 16) | (static_cast<std::uint32_t>(raw5) << 8) | static_cast<std::uint32_t>(raw4);
			std::uint32_t const z_raw = (static_cast<std::uint32_t>(raw9 & 0x03) << 16) | (static_cast<std::uint32_t>(raw8) << 8) | static_cast<std::uint32_t>(raw7);

			std::int32_t const x_raw2 = static_cast<std::int32_t>(x_raw & 0x20000 ? x_raw - 0x40000 : x_raw);
			std::int32_t const y_raw2 = static_cast<std::int32_t>(y_raw & 0x20000 ? y_raw - 0x40000 : y_raw);
			std::int32_t const z_raw2 = static_cast<std::int32_t>(z_raw & 0x20000 ? z_raw - 0x40000 : z_raw);

			AK09940A::MagneticFluxDensityDataRaw ak09940a;

			ak09940a.x = x_raw2;
			ak09940a.y = y_raw2;
			ak09940a.z = z_raw2;

			static CRC16 crc(0x8005, 0, false, true, true);
			crc.restart();

			constexpr std::array<uint8_t, 2> const header = {'H', 'i'};
			Serial.write(header.data(), 2);

			auto const scale_ak09940a = std::bit_cast<std::array<std::uint8_t, sizeof(AK09940A::get_scale_factor())> >(AK09940A::get_scale_factor());
			Serial.write(scale_ak09940a.data(), scale_ak09940a.size());
			crc.add(scale_ak09940a.data(), scale_ak09940a.size());

			Serial.write(ak09940a.bytes, sizeof(AK09940A::MagneticFluxDensityDataRaw));
			crc.add(ak09940a.bytes, sizeof(AK09940A::MagneticFluxDensityDataRaw));

			auto crc_value = std::bit_cast<std::array<uint8_t, 2> >(crc.calc());
			Serial.write(crc_value.data(), 2);

			Serial.print("Xraw: ");
			Serial.print(x_raw2);
			Serial.print(", Yraw: ");
			Serial.print(y_raw);
			Serial.print(", Zraw: ");
			Serial.println(z_raw);

			constexpr double scale = 0.01;

			double const x_uT = x_raw2 * scale;
			double const y_uT = y_raw2 * scale;
			double const z_uT = z_raw2 * scale;

			Serial.print("X: ");
			Serial.print(x_uT, 2);
			Serial.print(", Y: ");
			Serial.print(y_uT, 2);
			Serial.print(", Z: ");
			Serial.println(z_uT, 2);
		}
	}
}