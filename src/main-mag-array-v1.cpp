#include <Arduino.h>

// includes must follow after Serial definition

#undef Serial
HardwareSerial Serial1(PB7, PB6);
#define Serial Serial1

#include <CRC16.h>
#include <LIS3MDL.h>
#include <MMC5983MA.h>
#include <SPI.h>
#include <common2_message.h>
#include <common2_output.h>
#include <common2_time.h>

#include <bit>
#include <cstring>
#include <utility>

auto SPIrow1 = SPIClass(PB5, PB4, PB3);
auto SPIrow2 = SPIClass(PB15, PB14, PB13);
auto SPIrow3 = SPIClass(PC12, PC11, PC10);

LIS3MDL lis3mdl01("LIS3MDL 01", PA1, &SPIrow1);
LIS3MDL lis3mdl02("LIS3MDL 02", PB0, &SPIrow2);
LIS3MDL lis3mdl03("LIS3MDL 03", PC7, &SPIrow2);
LIS3MDL lis3mdl04("LIS3MDL 04", PD0, &SPIrow3);
LIS3MDL lis3mdl05("LIS3MDL 05", PA2, &SPIrow1);
LIS3MDL lis3mdl06("LIS3MDL 06", PB1, &SPIrow2);
LIS3MDL lis3mdl07("LIS3MDL 07", PD8, &SPIrow2);
LIS3MDL lis3mdl08("LIS3MDL 08", PD1, &SPIrow3);
LIS3MDL lis3mdl09("LIS3MDL 09", PA3, &SPIrow1);
LIS3MDL lis3mdl10("LIS3MDL 10", PB2, &SPIrow2);
LIS3MDL lis3mdl11("LIS3MDL 11", PD9, &SPIrow2);
LIS3MDL lis3mdl12("LIS3MDL 12", PD2, &SPIrow3);
LIS3MDL lis3mdl13("LIS3MDL 13", PA4, &SPIrow1);
LIS3MDL lis3mdl14("LIS3MDL 14", PB10, &SPIrow2);
LIS3MDL lis3mdl15("LIS3MDL 15", PA10, &SPIrow2);
LIS3MDL lis3mdl16("LIS3MDL 16", PD3, &SPIrow3);

MMC5983MA mmc5983ma01("MMC5983MA 01", PA0, &SPIrow1);
MMC5983MA mmc5983ma02("MMC5983MA 02", PC5, &SPIrow1);
MMC5983MA mmc5983ma03("MMC5983MA 03", PC6, &SPIrow2);
MMC5983MA mmc5983ma04("MMC5983MA 04", PC9, &SPIrow3);
MMC5983MA mmc5983ma05("MMC5983MA 05", PD4, &SPIrow3);
MMC5983MA mmc5983ma06("MMC5983MA 06", PC3, &SPIrow1);
MMC5983MA mmc5983ma07("MMC5983MA 07", PC4, &SPIrow1);
MMC5983MA mmc5983ma08("MMC5983MA 08", PA9, &SPIrow2);
MMC5983MA mmc5983ma09("MMC5983MA 09", PC8, &SPIrow3);
MMC5983MA mmc5983ma10("MMC5983MA 10", PD5, &SPIrow3);
MMC5983MA mmc5983ma11("MMC5983MA 11", PC2, &SPIrow1);
MMC5983MA mmc5983ma12("MMC5983MA 12", PA7, &SPIrow1);
MMC5983MA mmc5983ma13("MMC5983MA 13", PA8, &SPIrow2);
MMC5983MA mmc5983ma14("MMC5983MA 14", PA15, &SPIrow3);
MMC5983MA mmc5983ma15("MMC5983MA 15", PD6, &SPIrow3);
MMC5983MA mmc5983ma16("MMC5983MA 16", PC1, &SPIrow1);
MMC5983MA mmc5983ma17("MMC5983MA 17", PA6, &SPIrow1);
MMC5983MA mmc5983ma18("MMC5983MA 18", PB12, &SPIrow2);
MMC5983MA mmc5983ma19("MMC5983MA 19", PA12, &SPIrow3);
MMC5983MA mmc5983ma20("MMC5983MA 20", PB8, &SPIrow3);
MMC5983MA mmc5983ma21("MMC5983MA 21", PC0, &SPIrow1);
MMC5983MA mmc5983ma22("MMC5983MA 22", PA5, &SPIrow1);
MMC5983MA mmc5983ma23("MMC5983MA 23", PB11, &SPIrow2);
MMC5983MA mmc5983ma24("MMC5983MA 24", PA11, &SPIrow3);
MMC5983MA mmc5983ma25("MMC5983MA 25", PB9, &SPIrow3);

std::uint64_t time_delay = 0;
std::uint64_t time_offset = 0;
std::uint64_t timestamp = 0;

bool led_state = LOW;

void blink_led(decltype(millis()) const interval) {
	decltype(millis()) const timeout = millis() + interval;

	while (millis() < timeout) {
		digitalWrite(PC13, led_state = !led_state);

		delay(100);
	}
	if (led_state) digitalWrite(PC13, LOW);
}

void setup() {
	// turn led on
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, led_state);

	Serial.begin(230400);

	blink_led(2000);

	// obligatory Hello World
	common2::println_time(millis(), "Hello World from Magnetometer Array V1");

	// begin SPI
	// clang-format off
	SPIrow1.begin(); digitalWrite(PC13, led_state = !led_state);
	SPIrow2.begin(); digitalWrite(PC13, led_state = !led_state);
	SPIrow3.begin(); digitalWrite(PC13, led_state = !led_state);
	// clang-format on

	// begin the lis3mdl magnetometer
	// clang-format off
	lis3mdl01.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl02.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl03.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl04.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl05.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl06.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl07.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl08.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl09.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl10.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl11.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl12.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl13.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl14.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl15.begin(); digitalWrite(PC13, led_state = !led_state);
	lis3mdl16.begin(); digitalWrite(PC13, led_state = !led_state);
	// clang-format on

	// clang-format off
	mmc5983ma01.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma02.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma03.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma04.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma05.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma06.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma07.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma08.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma09.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma10.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma11.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma12.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma13.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma14.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma15.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma16.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma17.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma18.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma19.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma20.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma21.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma22.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma23.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma24.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	mmc5983ma25.begin(false, false); digitalWrite(PC13, led_state = !led_state);
	// clang-format on

	mmc5983ma01.performSetOperation();
	mmc5983ma02.performSetOperation();
	mmc5983ma03.performSetOperation();
	mmc5983ma04.performSetOperation();
	mmc5983ma05.performSetOperation();
	mmc5983ma06.performSetOperation();
	mmc5983ma07.performSetOperation();
	mmc5983ma08.performSetOperation();
	mmc5983ma09.performSetOperation();
	mmc5983ma10.performSetOperation();
	mmc5983ma11.performSetOperation();
	mmc5983ma12.performSetOperation();
	mmc5983ma13.performSetOperation();
	mmc5983ma14.performSetOperation();
	mmc5983ma15.performSetOperation();
	mmc5983ma16.performSetOperation();
	mmc5983ma17.performSetOperation();
	mmc5983ma18.performSetOperation();
	mmc5983ma19.performSetOperation();
	mmc5983ma20.performSetOperation();
	mmc5983ma21.performSetOperation();
	mmc5983ma22.performSetOperation();
	mmc5983ma23.performSetOperation();
	mmc5983ma24.performSetOperation();
	mmc5983ma25.performSetOperation();

	delay(1);

	digitalWrite(PC13, led_state = !led_state);

	delay(1);

	mmc5983ma01.performResetOperation();
	mmc5983ma02.performResetOperation();
	mmc5983ma03.performResetOperation();
	mmc5983ma04.performResetOperation();
	mmc5983ma05.performResetOperation();
	mmc5983ma06.performResetOperation();
	mmc5983ma07.performResetOperation();
	mmc5983ma08.performResetOperation();
	mmc5983ma09.performResetOperation();
	mmc5983ma10.performResetOperation();
	mmc5983ma11.performResetOperation();
	mmc5983ma12.performResetOperation();
	mmc5983ma13.performResetOperation();
	mmc5983ma14.performResetOperation();
	mmc5983ma15.performResetOperation();
	mmc5983ma16.performResetOperation();
	mmc5983ma17.performResetOperation();
	mmc5983ma18.performResetOperation();
	mmc5983ma19.performResetOperation();
	mmc5983ma20.performResetOperation();
	mmc5983ma21.performResetOperation();
	mmc5983ma22.performResetOperation();
	mmc5983ma23.performResetOperation();
	mmc5983ma24.performResetOperation();
	mmc5983ma25.performResetOperation();

	delay(1);

	digitalWrite(PC13, led_state = LOW);

	std::tie(time_delay, time_offset) = common2::sync_time();

	delay(1);
}

void loop() {
	CRC16 crc(0x8005, 0, false, true, true);

	digitalWrite(PC13, led_state = !led_state);

	static unsigned long last = 0;
	unsigned long now = millis();
	if (now - last > 20) common2::message("Took ", now - last, "ms!");
	while (now - last < 20) {
		delay(1);
		now = millis();
	}

	last = now;

	mmc5983ma01.start_measurement();
	mmc5983ma02.start_measurement();
	mmc5983ma03.start_measurement();
	mmc5983ma04.start_measurement();
	mmc5983ma05.start_measurement();
	mmc5983ma06.start_measurement();
	mmc5983ma07.start_measurement();
	mmc5983ma08.start_measurement();
	mmc5983ma09.start_measurement();
	mmc5983ma10.start_measurement();
	mmc5983ma11.start_measurement();
	mmc5983ma12.start_measurement();
	mmc5983ma13.start_measurement();
	mmc5983ma14.start_measurement();
	mmc5983ma15.start_measurement();
	mmc5983ma16.start_measurement();
	mmc5983ma17.start_measurement();
	mmc5983ma18.start_measurement();
	mmc5983ma19.start_measurement();
	mmc5983ma20.start_measurement();
	mmc5983ma21.start_measurement();
	mmc5983ma22.start_measurement();
	mmc5983ma23.start_measurement();
	mmc5983ma24.start_measurement();
	mmc5983ma25.start_measurement();

	// timestamp computation is here because the LIS3MDLs already have the measurements, whereas for the MMC5983MA they have to be obtained.
	// std::tie(time_delay, time_offset) = common::sync_time();
	if (std::exchange(timestamp, 1000ULL * micros() + time_offset) >= timestamp) {  // when time overflow is detected:
		common2::message("micros overflow detected!");
		return;
	}

	lis3mdl01.start_measurement();
	lis3mdl02.start_measurement();
	lis3mdl03.start_measurement();
	lis3mdl04.start_measurement();
	lis3mdl05.start_measurement();
	lis3mdl06.start_measurement();
	lis3mdl07.start_measurement();
	lis3mdl08.start_measurement();
	lis3mdl09.start_measurement();
	lis3mdl10.start_measurement();
	lis3mdl11.start_measurement();
	lis3mdl12.start_measurement();
	lis3mdl13.start_measurement();
	lis3mdl14.start_measurement();
	lis3mdl15.start_measurement();
	lis3mdl16.start_measurement();

	Serial.write(static_cast<std::uint8_t>('M'));

	auto const scale_lis3mdl = std::bit_cast<std::array<std::uint8_t, sizeof(lis3mdl01.get_scale_factor())>>(lis3mdl01.get_scale_factor());
	Serial.write(scale_lis3mdl.data(), scale_lis3mdl.size());
	crc.add(scale_lis3mdl.data(), scale_lis3mdl.size());

	// clang-format off
	auto mag_data26 = lis3mdl01.get_measurement(); Serial.write(mag_data26.bytes, 6); crc.add(mag_data26.bytes, 6);
	auto mag_data27 = lis3mdl02.get_measurement(); Serial.write(mag_data27.bytes, 6); crc.add(mag_data27.bytes, 6);
	auto mag_data28 = lis3mdl03.get_measurement(); Serial.write(mag_data28.bytes, 6); crc.add(mag_data28.bytes, 6);
	auto mag_data29 = lis3mdl04.get_measurement(); Serial.write(mag_data29.bytes, 6); crc.add(mag_data29.bytes, 6);
	auto mag_data30 = lis3mdl05.get_measurement(); Serial.write(mag_data30.bytes, 6); crc.add(mag_data30.bytes, 6);
	auto mag_data31 = lis3mdl06.get_measurement(); Serial.write(mag_data31.bytes, 6); crc.add(mag_data31.bytes, 6);
	auto mag_data32 = lis3mdl07.get_measurement(); Serial.write(mag_data32.bytes, 6); crc.add(mag_data32.bytes, 6);
	auto mag_data33 = lis3mdl08.get_measurement(); Serial.write(mag_data33.bytes, 6); crc.add(mag_data33.bytes, 6);
	auto mag_data34 = lis3mdl09.get_measurement(); Serial.write(mag_data34.bytes, 6); crc.add(mag_data34.bytes, 6);
	auto mag_data35 = lis3mdl10.get_measurement(); Serial.write(mag_data35.bytes, 6); crc.add(mag_data35.bytes, 6);
	auto mag_data36 = lis3mdl11.get_measurement(); Serial.write(mag_data36.bytes, 6); crc.add(mag_data36.bytes, 6);
	auto mag_data37 = lis3mdl12.get_measurement(); Serial.write(mag_data37.bytes, 6); crc.add(mag_data37.bytes, 6);
	auto mag_data38 = lis3mdl13.get_measurement(); Serial.write(mag_data38.bytes, 6); crc.add(mag_data38.bytes, 6);
	auto mag_data39 = lis3mdl14.get_measurement(); Serial.write(mag_data39.bytes, 6); crc.add(mag_data39.bytes, 6);
	auto mag_data40 = lis3mdl15.get_measurement(); Serial.write(mag_data40.bytes, 6); crc.add(mag_data40.bytes, 6);
	auto mag_data41 = lis3mdl16.get_measurement(); Serial.write(mag_data41.bytes, 6); crc.add(mag_data41.bytes, 6);
	// clang-format on

	auto const scale_mmc5983ma = std::bit_cast<std::array<std::uint8_t, sizeof(mmc5983ma01.get_scale_factor())>>(mmc5983ma01.get_scale_factor());
	Serial.write(scale_mmc5983ma.data(), scale_mmc5983ma.size());
	crc.add(scale_mmc5983ma.data(), scale_mmc5983ma.size());

	delay(7);

	// clang-format off
	auto mag_data01 = mmc5983ma01.get_measurement(); Serial.write(mag_data01.bytes, 7); crc.add(mag_data01.bytes, 7);
	auto mag_data02 = mmc5983ma02.get_measurement(); Serial.write(mag_data02.bytes, 7); crc.add(mag_data02.bytes, 7);
	auto mag_data03 = mmc5983ma03.get_measurement(); Serial.write(mag_data03.bytes, 7); crc.add(mag_data03.bytes, 7);
	auto mag_data04 = mmc5983ma04.get_measurement(); Serial.write(mag_data04.bytes, 7); crc.add(mag_data04.bytes, 7);
	auto mag_data05 = mmc5983ma05.get_measurement(); Serial.write(mag_data05.bytes, 7); crc.add(mag_data05.bytes, 7);
	auto mag_data06 = mmc5983ma06.get_measurement(); Serial.write(mag_data06.bytes, 7); crc.add(mag_data06.bytes, 7);
	auto mag_data07 = mmc5983ma07.get_measurement(); Serial.write(mag_data07.bytes, 7); crc.add(mag_data07.bytes, 7);
	auto mag_data08 = mmc5983ma08.get_measurement(); Serial.write(mag_data08.bytes, 7); crc.add(mag_data08.bytes, 7);
	auto mag_data09 = mmc5983ma09.get_measurement(); Serial.write(mag_data09.bytes, 7); crc.add(mag_data09.bytes, 7);
	auto mag_data10 = mmc5983ma10.get_measurement(); Serial.write(mag_data10.bytes, 7); crc.add(mag_data10.bytes, 7);
	auto mag_data11 = mmc5983ma11.get_measurement(); Serial.write(mag_data11.bytes, 7); crc.add(mag_data11.bytes, 7);
	auto mag_data12 = mmc5983ma12.get_measurement(); Serial.write(mag_data12.bytes, 7); crc.add(mag_data12.bytes, 7);
	auto mag_data13 = mmc5983ma13.get_measurement(); Serial.write(mag_data13.bytes, 7); crc.add(mag_data13.bytes, 7);
	auto mag_data14 = mmc5983ma14.get_measurement(); Serial.write(mag_data14.bytes, 7); crc.add(mag_data14.bytes, 7);
	auto mag_data15 = mmc5983ma15.get_measurement(); Serial.write(mag_data15.bytes, 7); crc.add(mag_data15.bytes, 7);
	auto mag_data16 = mmc5983ma16.get_measurement(); Serial.write(mag_data16.bytes, 7); crc.add(mag_data16.bytes, 7);
	auto mag_data17 = mmc5983ma17.get_measurement(); Serial.write(mag_data17.bytes, 7); crc.add(mag_data17.bytes, 7);
	auto mag_data18 = mmc5983ma18.get_measurement(); Serial.write(mag_data18.bytes, 7); crc.add(mag_data18.bytes, 7);
	auto mag_data19 = mmc5983ma19.get_measurement(); Serial.write(mag_data19.bytes, 7); crc.add(mag_data19.bytes, 7);
	auto mag_data20 = mmc5983ma20.get_measurement(); Serial.write(mag_data20.bytes, 7); crc.add(mag_data20.bytes, 7);
	auto mag_data21 = mmc5983ma21.get_measurement(); Serial.write(mag_data21.bytes, 7); crc.add(mag_data21.bytes, 7);
	auto mag_data22 = mmc5983ma22.get_measurement(); Serial.write(mag_data22.bytes, 7); crc.add(mag_data22.bytes, 7);
	auto mag_data23 = mmc5983ma23.get_measurement(); Serial.write(mag_data23.bytes, 7); crc.add(mag_data23.bytes, 7);
	auto mag_data24 = mmc5983ma24.get_measurement(); Serial.write(mag_data24.bytes, 7); crc.add(mag_data24.bytes, 7);
	auto mag_data25 = mmc5983ma25.get_measurement(); Serial.write(mag_data25.bytes, 7); crc.add(mag_data25.bytes, 7);
	// clang-format on

	mmc5983ma01.performSetOperation();
	mmc5983ma02.performSetOperation();
	mmc5983ma03.performSetOperation();
	mmc5983ma04.performSetOperation();
	mmc5983ma05.performSetOperation();
	mmc5983ma06.performSetOperation();
	mmc5983ma07.performSetOperation();
	mmc5983ma08.performSetOperation();
	mmc5983ma09.performSetOperation();
	mmc5983ma10.performSetOperation();
	mmc5983ma11.performSetOperation();
	mmc5983ma12.performSetOperation();
	mmc5983ma13.performSetOperation();
	mmc5983ma14.performSetOperation();
	mmc5983ma15.performSetOperation();
	mmc5983ma16.performSetOperation();
	mmc5983ma17.performSetOperation();
	mmc5983ma18.performSetOperation();
	mmc5983ma19.performSetOperation();
	mmc5983ma20.performSetOperation();
	mmc5983ma21.performSetOperation();
	mmc5983ma22.performSetOperation();
	mmc5983ma23.performSetOperation();
	mmc5983ma24.performSetOperation();
	mmc5983ma25.performSetOperation();

	// sent timestamp instead of 1ms delay
	auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
	Serial.write(timestamp_.data(), timestamp_.size());
	crc.add(timestamp_.data(), timestamp_.size());

	mmc5983ma01.performResetOperation();
	mmc5983ma02.performResetOperation();
	mmc5983ma03.performResetOperation();
	mmc5983ma04.performResetOperation();
	mmc5983ma05.performResetOperation();
	mmc5983ma06.performResetOperation();
	mmc5983ma07.performResetOperation();
	mmc5983ma08.performResetOperation();
	mmc5983ma09.performResetOperation();
	mmc5983ma10.performResetOperation();
	mmc5983ma11.performResetOperation();
	mmc5983ma12.performResetOperation();
	mmc5983ma13.performResetOperation();
	mmc5983ma14.performResetOperation();
	mmc5983ma15.performResetOperation();
	mmc5983ma16.performResetOperation();
	mmc5983ma17.performResetOperation();
	mmc5983ma18.performResetOperation();
	mmc5983ma19.performResetOperation();
	mmc5983ma20.performResetOperation();
	mmc5983ma21.performResetOperation();
	mmc5983ma22.performResetOperation();
	mmc5983ma23.performResetOperation();
	mmc5983ma24.performResetOperation();
	mmc5983ma25.performResetOperation();

	auto crc_value = std::bit_cast<std::array<uint8_t, 2>>(crc.calc());
	Serial.write(crc_value.data(), crc_value.size());

	Serial.write(static_cast<std::uint8_t>('M'));

	delay(1);

	// clang-format off
	//common::println_time(now,
	//	'\'', mmc5983ma01.sensor_name,  '\'', mmc5983ma01.get_measurement_float(), ",",
	//	'\'', mmc5983ma02.sensor_name,  '\'', mmc5983ma02.get_measurement_float(), ",",
	//	'\'', mmc5983ma03.sensor_name,  '\'', mmc5983ma03.get_measurement_float(), ",",
	//	'\'', mmc5983ma04.sensor_name,  '\'', mmc5983ma04.get_measurement_float(), ",",
	//	'\'', mmc5983ma05.sensor_name,  '\'', mmc5983ma05.get_measurement_float(), ",",
	//	'\'', mmc5983ma06.sensor_name,  '\'', mmc5983ma06.get_measurement_float(), ",",
	//	'\'', mmc5983ma07.sensor_name,  '\'', mmc5983ma07.get_measurement_float(), ",",
	//	'\'', mmc5983ma08.sensor_name,  '\'', mmc5983ma08.get_measurement_float(), ",",
	//	'\'', mmc5983ma09.sensor_name,  '\'', mmc5983ma09.get_measurement_float(), ",",
	//	'\'', mmc5983ma10.sensor_name,  '\'', mmc5983ma10.get_measurement_float(), ",",
	//	'\'', mmc5983ma11.sensor_name,  '\'', mmc5983ma11.get_measurement_float(), ",",
	//	'\'', mmc5983ma12.sensor_name,  '\'', mmc5983ma12.get_measurement_float(), ",",
	//	'\'', mmc5983ma13.sensor_name,  '\'', mmc5983ma13.get_measurement_float(), ",",
	//	'\'', mmc5983ma14.sensor_name,  '\'', mmc5983ma14.get_measurement_float(), ",",
	//	'\'', mmc5983ma15.sensor_name,  '\'', mmc5983ma15.get_measurement_float(), ",",
	//	'\'', mmc5983ma16.sensor_name,  '\'', mmc5983ma16.get_measurement_float(), ",",
	//	'\'', mmc5983ma17.sensor_name,  '\'', mmc5983ma17.get_measurement_float(), ",",
	//	'\'', mmc5983ma18.sensor_name,  '\'', mmc5983ma18.get_measurement_float(), ",",
	//	'\'', mmc5983ma19.sensor_name,  '\'', mmc5983ma19.get_measurement_float(), ",",
	//	'\'', mmc5983ma20.sensor_name,  '\'', mmc5983ma20.get_measurement_float(), ",",
	//	'\'', mmc5983ma21.sensor_name,  '\'', mmc5983ma21.get_measurement_float(), ",",
	//	'\'', mmc5983ma22.sensor_name,  '\'', mmc5983ma22.get_measurement_float(), ",",
	//	'\'', mmc5983ma23.sensor_name,  '\'', mmc5983ma23.get_measurement_float(), ",",
	//	'\'', mmc5983ma24.sensor_name,  '\'', mmc5983ma24.get_measurement_float(), ",",
	//	'\'', mmc5983ma25.sensor_name,  '\'', mmc5983ma25.get_measurement_float(), ",",
	//	'\'', lis3mdl01.sensor_name,  '\'', lis3mdl01.get_measurement_float(), ",",
	//	'\'', lis3mdl02.sensor_name,  '\'', lis3mdl02.get_measurement_float(), ",",
	//	'\'', lis3mdl03.sensor_name,  '\'', lis3mdl03.get_measurement_float(), ",",
	//	'\'', lis3mdl04.sensor_name,  '\'', lis3mdl04.get_measurement_float(), ",",
	//	'\'', lis3mdl05.sensor_name,  '\'', lis3mdl05.get_measurement_float(), ",",
	//	'\'', lis3mdl06.sensor_name,  '\'', lis3mdl06.get_measurement_float(), ",",
	//	'\'', lis3mdl07.sensor_name,  '\'', lis3mdl07.get_measurement_float(), ",",
	//	'\'', lis3mdl08.sensor_name,  '\'', lis3mdl08.get_measurement_float(), ",",
	//	'\'', lis3mdl09.sensor_name,  '\'', lis3mdl09.get_measurement_float(), ",",
	//	'\'', lis3mdl10.sensor_name,  '\'', lis3mdl10.get_measurement_float(), ",",
	//	'\'', lis3mdl11.sensor_name,  '\'', lis3mdl11.get_measurement_float(), ",",
	//	'\'', lis3mdl12.sensor_name,  '\'', lis3mdl12.get_measurement_float(), ",",
	//	'\'', lis3mdl13.sensor_name,  '\'', lis3mdl13.get_measurement_float(), ",",
	//	'\'', lis3mdl14.sensor_name,  '\'', lis3mdl14.get_measurement_float(), ",",
	//	'\'', lis3mdl15.sensor_name,  '\'', lis3mdl15.get_measurement_float(), ",",
	//	'\'', lis3mdl16.sensor_name,  '\'', lis3mdl16.get_measurement_float(), ";");
	// clang-format on
}