#include <Arduino.h>
#include <CRC16.h>
#include <LIS3MDL.h>
#include <MMC5983MA.h>
#include <SPI.h>
#include <common_output.h>

#include <bit>

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

HardwareSerial Serial1(PB7, PB6);

void setup() {
	Serial1.begin(230400);
	delay(2000);

	// obligatory Hello World
	common::println_time(millis(), "Hello World from Magnetometer Array V1");

	// turn led on
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, LOW);

	// begin SPI
	SPIrow1.begin();
	SPIrow2.begin();
	SPIrow3.begin();

	// begin the lis3mdl magnetometer
	lis3mdl01.begin();
	lis3mdl02.begin();
	lis3mdl03.begin();
	lis3mdl04.begin();
	lis3mdl05.begin();
	lis3mdl06.begin();
	lis3mdl07.begin();
	lis3mdl08.begin();
	lis3mdl09.begin();
	lis3mdl10.begin();
	lis3mdl11.begin();
	lis3mdl12.begin();
	lis3mdl13.begin();
	lis3mdl14.begin();
	lis3mdl15.begin();
	lis3mdl16.begin();

	mmc5983ma01.begin();
	mmc5983ma02.begin();
	mmc5983ma03.begin();
	mmc5983ma04.begin();
	mmc5983ma05.begin();
	mmc5983ma06.begin();
	mmc5983ma07.begin();
	mmc5983ma08.begin();
	mmc5983ma09.begin();
	mmc5983ma10.begin();
	mmc5983ma11.begin();
	mmc5983ma12.begin();
	mmc5983ma13.begin();
	mmc5983ma14.begin();
	mmc5983ma15.begin();
	mmc5983ma16.begin();
	mmc5983ma17.begin();
	mmc5983ma18.begin();
	mmc5983ma19.begin();
	mmc5983ma20.begin();
	mmc5983ma21.begin();
	mmc5983ma22.begin();
	mmc5983ma23.begin();
	mmc5983ma24.begin();
	mmc5983ma25.begin();
}

void loop() {
	static CRC16 crc(0x8005, 0, false, true, true);
	crc.restart();
	static unsigned long last = 0;

	unsigned long now = millis();
	if (now - last > 20) common::print_warn("Took ", now - last, "ms!");
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

	constexpr std::array<uint8_t, 2> header = {0xAA, 0x55};
	Serial1.write(header.data(), 2);

	// clang-format off
	auto mag_data26 = lis3mdl01.get_measurement_float(); Serial1.write(mag_data26.bytes, 12); crc.add(mag_data26.bytes, 12);
	auto mag_data27 = lis3mdl02.get_measurement_float(); Serial1.write(mag_data27.bytes, 12); crc.add(mag_data27.bytes, 12);
	auto mag_data28 = lis3mdl03.get_measurement_float(); Serial1.write(mag_data28.bytes, 12); crc.add(mag_data28.bytes, 12);
	auto mag_data29 = lis3mdl04.get_measurement_float(); Serial1.write(mag_data29.bytes, 12); crc.add(mag_data29.bytes, 12);
	auto mag_data30 = lis3mdl05.get_measurement_float(); Serial1.write(mag_data30.bytes, 12); crc.add(mag_data30.bytes, 12);
	auto mag_data31 = lis3mdl06.get_measurement_float(); Serial1.write(mag_data31.bytes, 12); crc.add(mag_data31.bytes, 12);
	auto mag_data32 = lis3mdl07.get_measurement_float(); Serial1.write(mag_data32.bytes, 12); crc.add(mag_data32.bytes, 12);
	auto mag_data33 = lis3mdl08.get_measurement_float(); Serial1.write(mag_data33.bytes, 12); crc.add(mag_data33.bytes, 12);
	auto mag_data34 = lis3mdl09.get_measurement_float(); Serial1.write(mag_data34.bytes, 12); crc.add(mag_data34.bytes, 12);
	auto mag_data35 = lis3mdl10.get_measurement_float(); Serial1.write(mag_data35.bytes, 12); crc.add(mag_data35.bytes, 12);
	auto mag_data36 = lis3mdl11.get_measurement_float(); Serial1.write(mag_data36.bytes, 12); crc.add(mag_data36.bytes, 12);
	auto mag_data37 = lis3mdl12.get_measurement_float(); Serial1.write(mag_data37.bytes, 12); crc.add(mag_data37.bytes, 12);
	auto mag_data38 = lis3mdl13.get_measurement_float(); Serial1.write(mag_data38.bytes, 12); crc.add(mag_data38.bytes, 12);
	auto mag_data39 = lis3mdl14.get_measurement_float(); Serial1.write(mag_data39.bytes, 12); crc.add(mag_data39.bytes, 12);
	auto mag_data40 = lis3mdl15.get_measurement_float(); Serial1.write(mag_data40.bytes, 12); crc.add(mag_data40.bytes, 12);
	auto mag_data41 = lis3mdl16.get_measurement_float(); Serial1.write(mag_data41.bytes, 12); crc.add(mag_data41.bytes, 12);
	// clang-format on

	// clang-format off
	auto mag_data01 = mmc5983ma01.get_measurement_float(); Serial1.write(mag_data01.bytes, 12); crc.add(mag_data01.bytes, 12);
	auto mag_data02 = mmc5983ma02.get_measurement_float(); Serial1.write(mag_data02.bytes, 12); crc.add(mag_data02.bytes, 12);
	auto mag_data03 = mmc5983ma03.get_measurement_float(); Serial1.write(mag_data03.bytes, 12); crc.add(mag_data03.bytes, 12);
	auto mag_data04 = mmc5983ma04.get_measurement_float(); Serial1.write(mag_data04.bytes, 12); crc.add(mag_data04.bytes, 12);
	auto mag_data05 = mmc5983ma05.get_measurement_float(); Serial1.write(mag_data05.bytes, 12); crc.add(mag_data05.bytes, 12);
	auto mag_data06 = mmc5983ma06.get_measurement_float(); Serial1.write(mag_data06.bytes, 12); crc.add(mag_data06.bytes, 12);
	auto mag_data07 = mmc5983ma07.get_measurement_float(); Serial1.write(mag_data07.bytes, 12); crc.add(mag_data07.bytes, 12);
	auto mag_data08 = mmc5983ma08.get_measurement_float(); Serial1.write(mag_data08.bytes, 12); crc.add(mag_data08.bytes, 12);
	auto mag_data09 = mmc5983ma09.get_measurement_float(); Serial1.write(mag_data09.bytes, 12); crc.add(mag_data09.bytes, 12);
	auto mag_data10 = mmc5983ma10.get_measurement_float(); Serial1.write(mag_data10.bytes, 12); crc.add(mag_data10.bytes, 12);
	auto mag_data11 = mmc5983ma11.get_measurement_float(); Serial1.write(mag_data11.bytes, 12); crc.add(mag_data11.bytes, 12);
	auto mag_data12 = mmc5983ma12.get_measurement_float(); Serial1.write(mag_data12.bytes, 12); crc.add(mag_data12.bytes, 12);
	auto mag_data13 = mmc5983ma13.get_measurement_float(); Serial1.write(mag_data13.bytes, 12); crc.add(mag_data13.bytes, 12);
	auto mag_data14 = mmc5983ma14.get_measurement_float(); Serial1.write(mag_data14.bytes, 12); crc.add(mag_data14.bytes, 12);
	auto mag_data15 = mmc5983ma15.get_measurement_float(); Serial1.write(mag_data15.bytes, 12); crc.add(mag_data15.bytes, 12);
	auto mag_data16 = mmc5983ma16.get_measurement_float(); Serial1.write(mag_data16.bytes, 12); crc.add(mag_data16.bytes, 12);
	auto mag_data17 = mmc5983ma17.get_measurement_float(); Serial1.write(mag_data17.bytes, 12); crc.add(mag_data17.bytes, 12);
	auto mag_data18 = mmc5983ma18.get_measurement_float(); Serial1.write(mag_data18.bytes, 12); crc.add(mag_data18.bytes, 12);
	auto mag_data19 = mmc5983ma19.get_measurement_float(); Serial1.write(mag_data19.bytes, 12); crc.add(mag_data19.bytes, 12);
	auto mag_data20 = mmc5983ma20.get_measurement_float(); Serial1.write(mag_data20.bytes, 12); crc.add(mag_data20.bytes, 12);
	auto mag_data21 = mmc5983ma21.get_measurement_float(); Serial1.write(mag_data21.bytes, 12); crc.add(mag_data21.bytes, 12);
	auto mag_data22 = mmc5983ma22.get_measurement_float(); Serial1.write(mag_data22.bytes, 12); crc.add(mag_data22.bytes, 12);
	auto mag_data23 = mmc5983ma23.get_measurement_float(); Serial1.write(mag_data23.bytes, 12); crc.add(mag_data23.bytes, 12);
	auto mag_data24 = mmc5983ma24.get_measurement_float(); Serial1.write(mag_data24.bytes, 12); crc.add(mag_data24.bytes, 12);
	auto mag_data25 = mmc5983ma25.get_measurement_float(); Serial1.write(mag_data25.bytes, 12); crc.add(mag_data25.bytes, 12);
	// clang-format on

	auto crc_value = std::bit_cast<std::array<uint8_t, 2>>(crc.calc());
	Serial1.write(crc_value.data(), 2);

	// clang-format off
	//common::println_time(now,
	//	'\'', mmc5983ma01.sensor_name,  '\'', mmc5983ma01.get_measurement(), ",",
	//	'\'', mmc5983ma02.sensor_name,  '\'', mmc5983ma02.get_measurement(), ",",
	//	'\'', mmc5983ma03.sensor_name,  '\'', mmc5983ma03.get_measurement(), ",",
	//	'\'', mmc5983ma04.sensor_name,  '\'', mmc5983ma04.get_measurement(), ",",
	//	'\'', mmc5983ma05.sensor_name,  '\'', mmc5983ma05.get_measurement(), ",",
	//	'\'', mmc5983ma06.sensor_name,  '\'', mmc5983ma06.get_measurement(), ",",
	//	'\'', mmc5983ma07.sensor_name,  '\'', mmc5983ma07.get_measurement(), ",",
	//	'\'', mmc5983ma08.sensor_name,  '\'', mmc5983ma08.get_measurement(), ",",
	//	'\'', mmc5983ma09.sensor_name,  '\'', mmc5983ma09.get_measurement(), ",",
	//	'\'', mmc5983ma10.sensor_name,  '\'', mmc5983ma10.get_measurement(), ",",
	//	'\'', mmc5983ma11.sensor_name,  '\'', mmc5983ma11.get_measurement(), ",",
	//	'\'', mmc5983ma12.sensor_name,  '\'', mmc5983ma12.get_measurement(), ",",
	//	'\'', mmc5983ma13.sensor_name,  '\'', mmc5983ma13.get_measurement(), ",",
	//	'\'', mmc5983ma14.sensor_name,  '\'', mmc5983ma14.get_measurement(), ",",
	//	'\'', mmc5983ma15.sensor_name,  '\'', mmc5983ma15.get_measurement(), ",",
	//	'\'', mmc5983ma16.sensor_name,  '\'', mmc5983ma16.get_measurement(), ",",
	//	'\'', mmc5983ma17.sensor_name,  '\'', mmc5983ma17.get_measurement(), ",",
	//	'\'', mmc5983ma18.sensor_name,  '\'', mmc5983ma18.get_measurement(), ",",
	//	'\'', mmc5983ma19.sensor_name,  '\'', mmc5983ma19.get_measurement(), ",",
	//	'\'', mmc5983ma20.sensor_name,  '\'', mmc5983ma20.get_measurement(), ",",
	//	'\'', mmc5983ma21.sensor_name,  '\'', mmc5983ma21.get_measurement(), ",",
	//	'\'', mmc5983ma22.sensor_name,  '\'', mmc5983ma22.get_measurement(), ",",
	//	'\'', mmc5983ma23.sensor_name,  '\'', mmc5983ma23.get_measurement(), ",",
	//	'\'', mmc5983ma24.sensor_name,  '\'', mmc5983ma24.get_measurement(), ",",
	//	'\'', mmc5983ma25.sensor_name,  '\'', mmc5983ma25.get_measurement(), ",",
	//	'\'', lis3mdl01.sensor_name,  '\'', lis3mdl01.get_measurement(), ",",
	//	'\'', lis3mdl02.sensor_name,  '\'', lis3mdl02.get_measurement(), ",",
	//	'\'', lis3mdl03.sensor_name,  '\'', lis3mdl03.get_measurement(), ",",
	//	'\'', lis3mdl04.sensor_name,  '\'', lis3mdl04.get_measurement(), ",",
	//	'\'', lis3mdl05.sensor_name,  '\'', lis3mdl05.get_measurement(), ",",
	//	'\'', lis3mdl06.sensor_name,  '\'', lis3mdl06.get_measurement(), ",",
	//	'\'', lis3mdl07.sensor_name,  '\'', lis3mdl07.get_measurement(), ",",
	//	'\'', lis3mdl08.sensor_name,  '\'', lis3mdl08.get_measurement(), ",",
	//	'\'', lis3mdl09.sensor_name,  '\'', lis3mdl09.get_measurement(), ",",
	//	'\'', lis3mdl10.sensor_name,  '\'', lis3mdl10.get_measurement(), ",",
	//	'\'', lis3mdl11.sensor_name,  '\'', lis3mdl11.get_measurement(), ",",
	//	'\'', lis3mdl12.sensor_name,  '\'', lis3mdl12.get_measurement(), ",",
	//	'\'', lis3mdl13.sensor_name,  '\'', lis3mdl13.get_measurement(), ",",
	//	'\'', lis3mdl14.sensor_name,  '\'', lis3mdl14.get_measurement(), ",",
	//	'\'', lis3mdl15.sensor_name,  '\'', lis3mdl15.get_measurement(), ",",
	//	'\'', lis3mdl16.sensor_name,  '\'', lis3mdl16.get_measurement(), ";");
	// clang-format on
}