#include <Adafruit_LIS3MDL.h>
#include <Arduino.h>
#include <LIS3MDL.h>
#include <MMC5983MA.h>
#include <SPI.h>

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
	Serial1.begin(115200);
	delay(2000);

	// obligatory Hello World
	Serial1.println("Hello World from Magnetometer Array V1");

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

void send_message(float const x, float const y, float const z, const char* sensor_name, uint32_t const timestamp = millis()) {
	Serial1.printf("[%s, %8.3lf]: ", sensor_name, timestamp / 1000.0);
	Serial1.print("X=");
	Serial1.print(x, 5);
	Serial1.print(", Y=");
	Serial1.print(y, 5);
	Serial1.print(", Z=");
	Serial1.print(z, 5);
	Serial1.println('.');
}

void loop() {
	lis3mdl01.get_data();
	lis3mdl02.get_data();
	lis3mdl03.get_data();
	lis3mdl04.get_data();
	lis3mdl05.get_data();
	lis3mdl06.get_data();
	lis3mdl07.get_data();
	lis3mdl08.get_data();
	lis3mdl09.get_data();
	lis3mdl10.get_data();
	lis3mdl11.get_data();
	lis3mdl12.get_data();
	lis3mdl13.get_data();
	lis3mdl14.get_data();
	lis3mdl15.get_data();
	lis3mdl16.get_data();

	lis3mdl01.output_data();
	lis3mdl02.output_data();
	lis3mdl03.output_data();
	lis3mdl04.output_data();
	lis3mdl05.output_data();
	lis3mdl06.output_data();
	lis3mdl07.output_data();
	lis3mdl08.output_data();
	lis3mdl09.output_data();
	lis3mdl10.output_data();
	lis3mdl11.output_data();
	lis3mdl12.output_data();
	lis3mdl13.output_data();
	lis3mdl14.output_data();
	lis3mdl15.output_data();
	lis3mdl16.output_data();

	delay(100);
}