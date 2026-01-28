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

AK09940A ak000(&spi3, PA_9, true);
AK09940A ak001(&spi3, PA_10, true);
AK09940A ak002(&spi3, PH_13, true);
AK09940A ak003(&spi3, PH_14, true);
AK09940A ak004(&spi3, PH_15, true);
AK09940A ak005(&spi3, PI_0, true);
AK09940A ak006(&spi3, PI_3, true);
AK09940A ak007(&spi3, PI_2, true);
AK09940A ak008(&spi3, PI_1, true);
AK09940A ak009(&spi3, PD_7, true);
AK09940A ak010(&spi3, PD_6, true);
AK09940A ak011(&spi3, PD_5, true);
AK09940A ak012(&spi3, PD_4, true);
AK09940A ak013(&spi3, PD_3, true);
AK09940A ak014(&spi3, PD_2, true);
AK09940A ak015(&spi3, PD_1, true);
AK09940A ak016(&spi3, PD_0, true);
AK09940A ak017(&spi3, PB_6, true);
AK09940A ak018(&spi3, PB_5, true);
AK09940A ak019(&spi3, PG_15, true);
AK09940A ak020(&spi3, PG_14, true);
AK09940A ak021(&spi3, PG_13, true);
AK09940A ak022(&spi3, PG_12, true);
AK09940A ak023(&spi3, PG_11, true);
AK09940A ak024(&spi3, PG_10, true);
AK09940A ak025(&spi3, PG_9, true);
AK09940A ak026(&spi3, PI_6, true);
AK09940A ak027(&spi3, PI_5, true);
AK09940A ak028(&spi3, PI_4, true);
AK09940A ak029(&spi3, PE_1, true);
AK09940A ak030(&spi3, PE_0, true);
AK09940A ak031(&spi3, PB_9, true);
AK09940A ak032(&spi3, PB_8, true);
AK09940A ak033(&spi3, PB_7, true);

AK09940A ak034(&spi1, PF_0, true);
AK09940A ak035(&spi1, PI_11, true);
AK09940A ak036(&spi1, PI_10, true);
AK09940A ak037(&spi1, PI_9, true);
AK09940A ak038(&spi1, PE_6, true);
AK09940A ak039(&spi1, PE_5, true);
AK09940A ak040(&spi1, PE_4, true);
AK09940A ak041(&spi1, PE_3, true);
AK09940A ak042(&spi1, PE_2, true);
AK09940A ak043(&spi1, PF_7, true);
AK09940A ak044(&spi1, PF_8, true);
AK09940A ak045(&spi1, PF_6, true);
AK09940A ak046(&spi1, PF_5, true);
AK09940A ak047(&spi1, PF_4, true);
AK09940A ak048(&spi1, PF_3, true);
AK09940A ak049(&spi1, PF_2, true);
AK09940A ak050(&spi1, PF_1, true);
AK09940A ak051(&spi1, PH_2, true);
AK09940A ak052(&spi1, PA_2, true);
AK09940A ak053(&spi1, PA_1, true);
AK09940A ak054(&spi1, PC_3, true);
AK09940A ak055(&spi1, PC_2, true);
AK09940A ak056(&spi1, PC_1, true);
AK09940A ak057(&spi1, PC_0, true);
AK09940A ak058(&spi1, PF_10, true);
AK09940A ak059(&spi1, PF_9, true);
AK09940A ak060(&spi1, PB_0, true);
AK09940A ak061(&spi1, PC_5, true);
AK09940A ak062(&spi1, PC_4, true);
AK09940A ak063(&spi1, PA_4, true);
AK09940A ak064(&spi1, PA_3, true);
AK09940A ak065(&spi1, PH_5, true);
AK09940A ak066(&spi1, PH_4, true);
AK09940A ak067(&spi1, PH_3, true);
AK09940A ak068(&spi1, PE_7, true);
AK09940A ak069(&spi1, PG_1, true);
AK09940A ak070(&spi1, PG_0, true);
AK09940A ak071(&spi1, PF_15, true);
AK09940A ak072(&spi1, PF_14, true);
AK09940A ak073(&spi1, PF_13, true);
AK09940A ak074(&spi1, PF_12, true);
AK09940A ak075(&spi1, PF_11, true);
AK09940A ak076(&spi1, PB_1, true);

AK09940A ak077(&spi2, PE_15, true);
AK09940A ak078(&spi2, PE_14, true);
AK09940A ak079(&spi2, PE_13, true);
AK09940A ak080(&spi2, PE_12, true);
AK09940A ak081(&spi2, PE_11, true);
AK09940A ak082(&spi2, PE_10, true);
AK09940A ak083(&spi2, PE_9, true);
AK09940A ak084(&spi2, PE_8, true);
AK09940A ak085(&spi2, PC_6, true);
AK09940A ak086(&spi2, PC_7, true);
AK09940A ak087(&spi2, PC_8, true);
AK09940A ak088(&spi2, PC_9, true);
AK09940A ak089(&spi2, PA_8, true);
AK09940A ak090(&spi2, PH_11, true);
AK09940A ak091(&spi2, PH_10, true);
AK09940A ak092(&spi2, PH_9, true);
AK09940A ak093(&spi2, PH_6, true);
AK09940A ak094(&spi2, PG_3, true);
AK09940A ak095(&spi2, PG_4, true);
AK09940A ak096(&spi2, PG_5, true);
AK09940A ak097(&spi2, PH_7, true);
AK09940A ak098(&spi2, PH_8, true);
AK09940A ak099(&spi2, PG_6, true);
AK09940A ak100(&spi2, PG_7, true);
AK09940A ak101(&spi2, PG_8, true);
AK09940A ak102(&spi2, PD_9, true);
AK09940A ak103(&spi2, PD_8, true);
AK09940A ak104(&spi2, PB_12, true);
AK09940A ak105(&spi2, PD_11, true);
AK09940A ak106(&spi2, PD_12, true);
AK09940A ak107(&spi2, PD_13, true);
AK09940A ak108(&spi2, PD_14, true);
AK09940A ak109(&spi2, PD_15, true);
AK09940A ak110(&spi2, PG_2, true);

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

	{  // config spis
		spi1.beginTransaction(SPISettings(3'000'000, BitOrder::MSBFIRST, SPI_MODE3));
		spi2.beginTransaction(SPISettings(3'000'000, BitOrder::MSBFIRST, SPI_MODE3));
		spi3.beginTransaction(SPISettings(3'000'000, BitOrder::MSBFIRST, SPI_MODE3));
	}

	delay(100);

	{  // config I2C2
		i2c2.begin();
		i2c2.setClock(200'000);
	}

	delay(100);

	{  // config gyro
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
		ak000.begin();
		ak001.begin();
		ak002.begin();
		ak003.begin();
		ak004.begin();
		ak005.begin();
		ak006.begin();
		ak007.begin();
		ak008.begin();
		ak009.begin();
		ak010.begin();
		ak011.begin();
		ak012.begin();
		ak013.begin();
		ak014.begin();
		ak015.begin();
		ak016.begin();
		ak017.begin();
		ak018.begin();
		ak019.begin();
		ak020.begin();
		ak021.begin();
		ak022.begin();
		ak023.begin();
		ak024.begin();
		ak025.begin();
		ak026.begin();
		ak027.begin();
		ak028.begin();
		ak029.begin();
		ak030.begin();
		ak031.begin();
		ak032.begin();
		ak033.begin();

		ak034.begin();
		ak035.begin();
		ak036.begin();
		ak037.begin();
		ak038.begin();
		ak039.begin();
		ak040.begin();
		ak041.begin();
		ak042.begin();
		ak043.begin();
		ak044.begin();
		ak045.begin();
		ak046.begin();
		ak047.begin();
		ak048.begin();
		ak049.begin();
		ak050.begin();
		ak051.begin();
		ak052.begin();
		ak053.begin();
		ak054.begin();
		ak055.begin();
		ak056.begin();
		ak057.begin();
		ak058.begin();
		ak059.begin();
		ak060.begin();
		ak061.begin();
		ak062.begin();
		ak063.begin();
		ak064.begin();
		ak065.begin();
		ak066.begin();
		ak067.begin();
		ak068.begin();
		ak069.begin();
		ak070.begin();
		ak071.begin();
		ak072.begin();
		ak073.begin();
		ak074.begin();
		ak075.begin();
		ak076.begin();

		ak077.begin();
		ak078.begin();
		ak079.begin();
		ak080.begin();
		ak081.begin();
		ak082.begin();
		ak083.begin();
		ak084.begin();
		ak085.begin();
		ak086.begin();
		ak087.begin();
		ak088.begin();
		ak089.begin();
		ak090.begin();
		ak091.begin();
		ak092.begin();
		ak093.begin();
		ak094.begin();
		ak095.begin();
		ak096.begin();
		ak097.begin();
		ak098.begin();
		ak099.begin();
		ak100.begin();
		ak101.begin();
		ak102.begin();
		ak103.begin();
		ak104.begin();
		ak105.begin();
		ak106.begin();
		ak107.begin();
		ak108.begin();
		ak109.begin();
		ak110.begin();
	}

	Serial.println("Ready.");
}

void print(AK09940A::MagneticFluxDensityDataRaw const data) {
	constexpr double scale = 0.01;
	double const x_uT = data.x * scale;  // AK09940A::get_scale_factor();
	double const y_uT = data.y * scale;  // AK09940A::get_scale_factor();
	double const z_uT = data.z * scale;  // AK09940A::get_scale_factor();

	Serial.print("X: ");
	Serial.print(x_uT, 2);
	Serial.print(", Y: ");
	Serial.print(y_uT, 2);
	Serial.print(", Z: ");
	Serial.println(z_uT, 2);
}

void loop() {
	{  // blink LED
		digitalWrite(PD_10, led_state = !led_state);
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
		ak000.start_measurement();
		ak001.start_measurement();
		ak002.start_measurement();
		ak003.start_measurement();
		ak004.start_measurement();
		ak005.start_measurement();
		ak006.start_measurement();
		ak007.start_measurement();
		ak008.start_measurement();
		ak009.start_measurement();
		ak010.start_measurement();
		ak011.start_measurement();
		ak012.start_measurement();
		ak013.start_measurement();
		ak014.start_measurement();
		ak015.start_measurement();
		ak016.start_measurement();
		ak017.start_measurement();
		ak018.start_measurement();
		ak019.start_measurement();
		ak020.start_measurement();
		ak021.start_measurement();
		ak022.start_measurement();
		ak023.start_measurement();
		ak024.start_measurement();
		ak025.start_measurement();
		ak026.start_measurement();
		ak027.start_measurement();
		ak028.start_measurement();
		ak029.start_measurement();
		ak030.start_measurement();
		ak031.start_measurement();
		ak032.start_measurement();
		ak033.start_measurement();

		ak034.start_measurement();
		ak035.start_measurement();
		ak036.start_measurement();
		ak037.start_measurement();
		ak038.start_measurement();
		ak039.start_measurement();
		ak040.start_measurement();
		ak041.start_measurement();
		ak042.start_measurement();
		ak043.start_measurement();
		ak044.start_measurement();
		ak045.start_measurement();
		ak046.start_measurement();
		ak047.start_measurement();
		ak048.start_measurement();
		ak049.start_measurement();
		ak050.start_measurement();
		ak051.start_measurement();
		ak052.start_measurement();
		ak053.start_measurement();
		ak054.start_measurement();
		ak055.start_measurement();
		ak056.start_measurement();
		ak057.start_measurement();
		ak058.start_measurement();
		ak059.start_measurement();
		ak060.start_measurement();
		ak061.start_measurement();
		ak062.start_measurement();
		ak063.start_measurement();
		ak064.start_measurement();
		ak065.start_measurement();
		ak066.start_measurement();
		ak067.start_measurement();
		ak068.start_measurement();
		ak069.start_measurement();
		ak070.start_measurement();
		ak071.start_measurement();
		ak072.start_measurement();
		ak073.start_measurement();
		ak074.start_measurement();
		ak075.start_measurement();
		ak076.start_measurement();

		ak077.start_measurement();
		ak078.start_measurement();
		ak079.start_measurement();
		ak080.start_measurement();
		ak081.start_measurement();
		ak082.start_measurement();
		ak083.start_measurement();
		ak084.start_measurement();
		ak085.start_measurement();
		ak086.start_measurement();
		ak087.start_measurement();
		ak088.start_measurement();
		ak089.start_measurement();
		ak090.start_measurement();
		ak091.start_measurement();
		ak092.start_measurement();
		ak093.start_measurement();
		ak094.start_measurement();
		ak095.start_measurement();
		ak096.start_measurement();
		ak097.start_measurement();
		ak098.start_measurement();
		ak099.start_measurement();
		ak100.start_measurement();
		ak101.start_measurement();
		ak102.start_measurement();
		ak103.start_measurement();
		ak104.start_measurement();
		ak105.start_measurement();
		ak106.start_measurement();
		ak107.start_measurement();
		ak108.start_measurement();
		ak109.start_measurement();
		ak110.start_measurement();

		// clang-format off
		auto const mag000 = ak000.get_measurement(); print(mag000);
		auto const mag001 = ak001.get_measurement(); print(mag001);
		auto const mag002 = ak002.get_measurement(); print(mag002);
		auto const mag003 = ak003.get_measurement(); print(mag003);
		auto const mag004 = ak004.get_measurement(); print(mag004);
		auto const mag005 = ak005.get_measurement(); print(mag005);
		auto const mag006 = ak006.get_measurement(); print(mag006);
		auto const mag007 = ak007.get_measurement(); print(mag007);
		auto const mag008 = ak008.get_measurement(); print(mag008);
		auto const mag009 = ak009.get_measurement(); print(mag009);
		auto const mag010 = ak010.get_measurement(); print(mag010);
		auto const mag011 = ak011.get_measurement(); print(mag011);
		auto const mag012 = ak012.get_measurement(); print(mag012);
		auto const mag013 = ak013.get_measurement(); print(mag013);
		auto const mag014 = ak014.get_measurement(); print(mag014);
		auto const mag015 = ak015.get_measurement(); print(mag015);
		auto const mag016 = ak016.get_measurement(); print(mag016);
		auto const mag017 = ak017.get_measurement(); print(mag017);
		auto const mag018 = ak018.get_measurement(); print(mag018);
		auto const mag019 = ak019.get_measurement(); print(mag019);
		auto const mag020 = ak020.get_measurement(); print(mag020);
		auto const mag021 = ak021.get_measurement(); print(mag021);
		auto const mag022 = ak022.get_measurement(); print(mag022);
		auto const mag023 = ak023.get_measurement(); print(mag023);
		auto const mag024 = ak024.get_measurement(); print(mag024);
		auto const mag025 = ak025.get_measurement(); print(mag025);
		auto const mag026 = ak026.get_measurement(); print(mag026);
		auto const mag027 = ak027.get_measurement(); print(mag027);
		auto const mag028 = ak028.get_measurement(); print(mag028);
		auto const mag029 = ak029.get_measurement(); print(mag029);
		auto const mag030 = ak030.get_measurement(); print(mag030);
		auto const mag031 = ak031.get_measurement(); print(mag031);
		auto const mag032 = ak032.get_measurement(); print(mag032);
		auto const mag033 = ak033.get_measurement(); print(mag033);

		auto const mag034 = ak034.get_measurement(); print(mag034);
		auto const mag035 = ak035.get_measurement(); print(mag035);
		auto const mag036 = ak036.get_measurement(); print(mag036);
		auto const mag037 = ak037.get_measurement(); print(mag037);
		auto const mag038 = ak038.get_measurement(); print(mag038);
		auto const mag039 = ak039.get_measurement(); print(mag039);
		auto const mag040 = ak040.get_measurement(); print(mag040);
		auto const mag041 = ak041.get_measurement(); print(mag041);
		auto const mag042 = ak042.get_measurement(); print(mag042);
		auto const mag043 = ak043.get_measurement(); print(mag043);
		auto const mag044 = ak044.get_measurement(); print(mag044);
		auto const mag045 = ak045.get_measurement(); print(mag045);
		auto const mag046 = ak046.get_measurement(); print(mag046);
		auto const mag047 = ak047.get_measurement(); print(mag047);
		auto const mag048 = ak048.get_measurement(); print(mag048);
		auto const mag049 = ak049.get_measurement(); print(mag049);
		auto const mag050 = ak050.get_measurement(); print(mag050);
		auto const mag051 = ak051.get_measurement(); print(mag051);
		auto const mag052 = ak052.get_measurement(); print(mag052);
		auto const mag053 = ak053.get_measurement(); print(mag053);
		auto const mag054 = ak054.get_measurement(); print(mag054);
		auto const mag055 = ak055.get_measurement(); print(mag055);
		auto const mag056 = ak056.get_measurement(); print(mag056);
		auto const mag057 = ak057.get_measurement(); print(mag057);
		auto const mag058 = ak058.get_measurement(); print(mag058);
		auto const mag059 = ak059.get_measurement(); print(mag059);
		auto const mag060 = ak060.get_measurement(); print(mag060);
		auto const mag061 = ak061.get_measurement(); print(mag061);
		auto const mag062 = ak062.get_measurement(); print(mag062);
		auto const mag063 = ak063.get_measurement(); print(mag063);
		auto const mag064 = ak064.get_measurement(); print(mag064);
		auto const mag065 = ak065.get_measurement(); print(mag065);
		auto const mag066 = ak066.get_measurement(); print(mag066);
		auto const mag067 = ak067.get_measurement(); print(mag067);
		auto const mag068 = ak068.get_measurement(); print(mag068);
		auto const mag069 = ak069.get_measurement(); print(mag069);
		auto const mag070 = ak070.get_measurement(); print(mag070);
		auto const mag071 = ak071.get_measurement(); print(mag071);
		auto const mag072 = ak072.get_measurement(); print(mag072);
		auto const mag073 = ak073.get_measurement(); print(mag073);
		auto const mag074 = ak074.get_measurement(); print(mag074);
		auto const mag075 = ak075.get_measurement(); print(mag075);
		auto const mag076 = ak076.get_measurement(); print(mag076);

		auto const mag077 = ak077.get_measurement(); print(mag077);
		auto const mag078 = ak078.get_measurement(); print(mag078);
		auto const mag079 = ak079.get_measurement(); print(mag079);
		auto const mag080 = ak080.get_measurement(); print(mag080);
		auto const mag081 = ak081.get_measurement(); print(mag081);
		auto const mag082 = ak082.get_measurement(); print(mag082);
		auto const mag083 = ak083.get_measurement(); print(mag083);
		auto const mag084 = ak084.get_measurement(); print(mag084);
		auto const mag085 = ak085.get_measurement(); print(mag085);
		auto const mag086 = ak086.get_measurement(); print(mag086);
		auto const mag087 = ak087.get_measurement(); print(mag087);
		auto const mag088 = ak088.get_measurement(); print(mag088);
		auto const mag089 = ak089.get_measurement(); print(mag089);
		auto const mag090 = ak090.get_measurement(); print(mag090);
		auto const mag091 = ak091.get_measurement(); print(mag091);
		auto const mag092 = ak092.get_measurement(); print(mag092);
		auto const mag093 = ak093.get_measurement(); print(mag093);
		auto const mag094 = ak094.get_measurement(); print(mag094);
		auto const mag095 = ak095.get_measurement(); print(mag095);
		auto const mag096 = ak096.get_measurement(); print(mag096);
		auto const mag097 = ak097.get_measurement(); print(mag097);
		auto const mag098 = ak098.get_measurement(); print(mag098);
		auto const mag099 = ak099.get_measurement(); print(mag099);
		auto const mag100 = ak100.get_measurement(); print(mag100);
		auto const mag101 = ak101.get_measurement(); print(mag101);
		auto const mag102 = ak102.get_measurement(); print(mag102);
		auto const mag103 = ak103.get_measurement(); print(mag103);
		auto const mag104 = ak104.get_measurement(); print(mag104);
		auto const mag105 = ak105.get_measurement(); print(mag105);
		auto const mag106 = ak106.get_measurement(); print(mag106);
		auto const mag107 = ak107.get_measurement(); print(mag107);
		auto const mag108 = ak108.get_measurement(); print(mag108);
		auto const mag109 = ak109.get_measurement(); print(mag109);
		auto const mag110 = ak110.get_measurement(); print(mag110);
		// clang-format on
	}
}
