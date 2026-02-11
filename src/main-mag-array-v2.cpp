#include <AK09940A.h>
#include <AccelerationDataRaw.h>
#include <Arduino.h>
#include <CRC16.h>
#include <CRC8.h>
#include <GyroDataRaw.h>
#include <RotationQuaternion.h>
#include <Wire.h>

inline void print_low_level(AccelerationDataRaw const& d) {
	common2::print_low_level(d.ax);
	common2::print_low_level(',');
	common2::print_low_level(d.ay);
	common2::print_low_level(',');
	common2::print_low_level(d.az);
}

inline void print_low_level(GyroDataRaw const& d) {
	common2::print_low_level(d.gx);
	common2::print_low_level(',');
	common2::print_low_level(d.gy);
	common2::print_low_level(',');
	common2::print_low_level(d.gz);
}
#include <LSM6DSV16X.h>
#include <common2.h>
#include <common2_time.h>

#include <bit>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <utility>

bool led_state = LOW;

auto i2c2 = TwoWire(PB_11, PB_10);
// auto imu = LSM6DSV16XSensor(&i2c2, 0b110'1010);
auto imu = LSM6DSV16X(&i2c2, 0b110'1010);

auto spi1 = SPIClass(PA_7, PA_6, PA_5);
auto spi2 = SPIClass(PB_15, PB_14, PB_13);
auto spi3 = SPIClass(PC_12, PC_11, PC_10);

// clang-format off
AK09940A ak000(&spi3, PA_9, false);
AK09940A ak001(&spi3, PA_10,false);
AK09940A ak002(&spi3, PH_13,false);
AK09940A ak003(&spi3, PH_14,false);
AK09940A ak004(&spi3, PH_15,false);
AK09940A ak005(&spi3, PI_0, false);
AK09940A ak006(&spi3, PI_3, false);
AK09940A ak007(&spi3, PI_2, false);
AK09940A ak008(&spi3, PI_1, false);
AK09940A ak009(&spi3, PD_7, false);
AK09940A ak010(&spi3, PD_6, false);
AK09940A ak011(&spi3, PD_5, false);
AK09940A ak012(&spi3, PD_4, false);
AK09940A ak013(&spi3, PD_3, false);
AK09940A ak014(&spi3, PD_2, false);
AK09940A ak015(&spi3, PD_1, false);
AK09940A ak016(&spi3, PD_0, false);
AK09940A ak017(&spi3, PB_6, false);
AK09940A ak018(&spi3, PB_5, false);
AK09940A ak019(&spi3, PG_15,false);
AK09940A ak020(&spi3, PG_14,false);
AK09940A ak021(&spi3, PG_13,false);
AK09940A ak022(&spi3, PG_12,false);
AK09940A ak023(&spi3, PG_11,false);
AK09940A ak024(&spi3, PG_10,false);
AK09940A ak025(&spi3, PG_9, false);
AK09940A ak026(&spi3, PI_6, false);
AK09940A ak027(&spi3, PI_5, false);
AK09940A ak028(&spi3, PI_4, false);
AK09940A ak029(&spi3, PE_1, false);
AK09940A ak030(&spi3, PE_0, false);
AK09940A ak031(&spi3, PB_9, false);
AK09940A ak032(&spi3, PB_8, false);
AK09940A ak033(&spi3, PB_7, false);

AK09940A ak034(&spi1, PF_0, false);
AK09940A ak035(&spi1, PI_11,false);
AK09940A ak036(&spi1, PI_10,false);
AK09940A ak037(&spi1, PI_9, false);
AK09940A ak038(&spi1, PE_6, false);
AK09940A ak039(&spi1, PE_5, false);
AK09940A ak040(&spi1, PE_4, false);
AK09940A ak041(&spi1, PE_3, false);
AK09940A ak042(&spi1, PE_2, false);
AK09940A ak043(&spi1, PF_7, false);
AK09940A ak044(&spi1, PF_8, false);
AK09940A ak045(&spi1, PF_6, false);
AK09940A ak046(&spi1, PF_5, false);
AK09940A ak047(&spi1, PF_4, false);
AK09940A ak048(&spi1, PF_3, false);
AK09940A ak049(&spi1, PF_2, false);
AK09940A ak050(&spi1, PF_1, false);
AK09940A ak051(&spi1, PH_2, false);
AK09940A ak052(&spi1, PA_2, false);
AK09940A ak053(&spi1, PA_1, false);
AK09940A ak054(&spi1, PC_3, false);
AK09940A ak055(&spi1, PC_2, false);
AK09940A ak056(&spi1, PC_1, false);
AK09940A ak057(&spi1, PC_0, false);
AK09940A ak058(&spi1, PF_10,false);
AK09940A ak059(&spi1, PF_9, false);
AK09940A ak060(&spi1, PB_0, false);
AK09940A ak061(&spi1, PC_5, false);
AK09940A ak062(&spi1, PC_4, false);
AK09940A ak063(&spi1, PA_4, false);
AK09940A ak064(&spi1, PA_3, false);
AK09940A ak065(&spi1, PH_5, false);
AK09940A ak066(&spi1, PH_4, false);
AK09940A ak067(&spi1, PH_3, false);
AK09940A ak068(&spi1, PE_7, false);
AK09940A ak069(&spi1, PG_1, false);
AK09940A ak070(&spi1, PG_0, false);
AK09940A ak071(&spi1, PF_15,false);
AK09940A ak072(&spi1, PF_14,false);
AK09940A ak073(&spi1, PF_13,false);
AK09940A ak074(&spi1, PF_12,false);
AK09940A ak075(&spi1, PF_11,false);
AK09940A ak076(&spi1, PB_1, false);

AK09940A ak077(&spi2, PE_15,false);
AK09940A ak078(&spi2, PE_14,false);
AK09940A ak079(&spi2, PE_13,false);
AK09940A ak080(&spi2, PE_12,false);
AK09940A ak081(&spi2, PE_11,false);
AK09940A ak082(&spi2, PE_10,false);
AK09940A ak083(&spi2, PE_9, false);
AK09940A ak084(&spi2, PE_8, false);
AK09940A ak085(&spi2, PC_6, false);
AK09940A ak086(&spi2, PC_7, false);
AK09940A ak087(&spi2, PC_8, false);
AK09940A ak088(&spi2, PC_9, false);
AK09940A ak089(&spi2, PA_8, false);
AK09940A ak090(&spi2, PH_11,false);
AK09940A ak091(&spi2, PH_10,false);
AK09940A ak092(&spi2, PH_9, false);
AK09940A ak093(&spi2, PH_6, false);
AK09940A ak094(&spi2, PG_3, false);
AK09940A ak095(&spi2, PG_4, false);
AK09940A ak096(&spi2, PG_5, false);
AK09940A ak097(&spi2, PH_7, false);
AK09940A ak098(&spi2, PH_8, false);
AK09940A ak099(&spi2, PG_6, false);
AK09940A ak100(&spi2, PG_7, false);
AK09940A ak101(&spi2, PG_8, false);
AK09940A ak102(&spi2, PD_9, false);
AK09940A ak103(&spi2, PD_8, false);
AK09940A ak104(&spi2, PB_12,false);
AK09940A ak105(&spi2, PD_11,false);
AK09940A ak106(&spi2, PD_12,false);
AK09940A ak107(&spi2, PD_13,false);
AK09940A ak108(&spi2, PD_14,false);
AK09940A ak109(&spi2, PD_15,false);
AK09940A ak110(&spi2, PG_2, false);
// clang-format on

std::uint64_t time_delay = 0;
std::uint64_t time_offset = 0;
std::uint64_t timestamp = 0;

void setup() {
	{  // set trigger pin
		pinMode(PI_7, OUTPUT);
		digitalWrite(PI_7, LOW);
	}

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

	{  // config I2C2 and imu
		i2c2.begin();
		// i2c2.setClock(10'000);

		delay(100);

		imu.begin_sensor_fusion();
	}

	delay(100);

	return;

	{  // connect AK09940A
		delay(100);

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

	{  // sync time
		std::tie(time_delay, time_offset) = common2::sync_time();
	}
}

void print(MagneticFluxDensityDataRawAK09940A const data) {
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

void poll_imu() {
	static CRC8 crc8;

#if not NDEBUG
	if (auto accel_data = imu.get_measurement_accelerometer(); accel_data) {
		common2::println("Accelerometer: ", accel_data.value());
	}
	if (auto gyro_data = imu.get_measurement_gyro(); gyro_data) {
		common2::println("Gyro: ", gyro_data.value());
	}
#endif

	if (auto const accel_data = imu.get_measurement_accelerometer(); accel_data) {
		Serial.write(static_cast<std::uint8_t>('A'));

		auto const scale_imu_accel = std::bit_cast<std::array<std::uint8_t, sizeof(float)>>(imu.get_scale_factor_accelerometer());
		Serial.write(scale_imu_accel.data(), scale_imu_accel.size());
		crc8.add(scale_imu_accel.data(), scale_imu_accel.size());

		Serial.write(accel_data.value().bytes.data(), accel_data.value().bytes.size());
		crc8.add(accel_data.value().bytes.data(), accel_data.value().bytes.size());

		auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
		Serial.write(timestamp_.data(), timestamp_.size());
		crc8.add(timestamp_.data(), timestamp_.size());

		Serial.write(crc8.calc());

		Serial.write(static_cast<std::uint8_t>('A'));

		crc8.restart();
	}
	if (auto const gyro_data = imu.get_measurement_gyro(); gyro_data) {
		Serial.write(static_cast<std::uint8_t>('G'));

		auto const scale_imu_gyro = std::bit_cast<std::array<std::uint8_t, sizeof(float)>>(imu.get_scale_factor_gyro());
		Serial.write(scale_imu_gyro.data(), scale_imu_gyro.size());
		crc8.add(scale_imu_gyro.data(), scale_imu_gyro.size());

		Serial.write(gyro_data.value().bytes.data(), gyro_data.value().bytes.size());
		crc8.add(gyro_data.value().bytes.data(), gyro_data.value().bytes.size());

		auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
		Serial.write(timestamp_.data(), timestamp_.size());
		crc8.add(timestamp_.data(), timestamp_.size());

		Serial.write(crc8.calc());

		Serial.write(static_cast<std::uint8_t>('G'));

		crc8.restart();
	}
}

void poll_imu_fifo() {
	static CRC8 crc8;
	for (std::variant<LSM6DSV16X::NoData, AccelerationDataRaw, GyroDataRaw, std::uint64_t> val; !std::holds_alternative<LSM6DSV16X::NoData>(val = imu.get_measurement_fifo());) {
		if (std::holds_alternative<AccelerationDataRaw>(val)) {
			common2::println("Accelerometer: ", std::get<AccelerationDataRaw>(val));
		}
		if (std::holds_alternative<GyroDataRaw>(val)) {
			common2::println("Gyro: ", std::get<GyroDataRaw>(val));
		}
		if (std::holds_alternative<std::uint64_t>(val)) {
			common2::println("Timestamp: ", std::get<std::uint64_t>(val));
		}
	}
}

void poll_imu_sensor_fusion() {
	static CRC8 crc8;
	for (std::variant<LSM6DSV16X::NoData, RotationQuaternion, LSM6DSV16X::GBiasVectorRaw, LSM6DSV16X::GravityVectorRaw> val; !std::holds_alternative<LSM6DSV16X::NoData>(val = imu.get_measurement_sensor_fusion());) {
		if (std::holds_alternative<RotationQuaternion>(val)) {
			auto const tmp = std::get<RotationQuaternion>(val);
			common2::println("RotationQuaternion: x = ", tmp.rx, ", y = ", tmp.ry, ", z = ", tmp.rz, ", w = ", tmp.rw);
		}
		if (std::holds_alternative<LSM6DSV16X::GBiasVectorRaw>(val)) {
			auto const tmp = std::get<LSM6DSV16X::GBiasVectorRaw>(val);
			common2::println("GBiasVector: x = ", tmp.x * 4.375f, ", y = ", tmp.y * 4.375f, ", z = ", tmp.z * 4.375f);
		}
		if (std::holds_alternative<LSM6DSV16X::GravityVectorRaw>(val)) {
			auto tmp = std::get<LSM6DSV16X::GravityVectorRaw>(val);
			common2::println("GravityVector: x = ", tmp.x * 0.061f, ", y = ", tmp.y * 0.061f, ", z = ", tmp.z * 0.061f);
		}
	}
}

void loop() {
	static CRC16 crc16(0x8005, 0, false, true, true);
	decltype(micros()) t1;

	imu.start_measurement_fifo();

	poll_imu_sensor_fusion();

	delay(20);

	return;

	{  // trigger sensors
		digitalWrite(PI_7, HIGH);
		delayMicroseconds(30);  // > 3us
		digitalWrite(PI_7, LOW);
		t1 = micros();
		// delayMicroseconds(3100);
	}

	{  // check overflow
		// std::tie(time_delay, time_offset) = common::sync_time();
		if (std::exchange(timestamp, 1000ULL * micros() + time_offset) >= timestamp) {  // when time overflow is detected:
			common2::message("micros overflow detected!");
			return;
		}
	}

	{  // blink LED
		digitalWrite(PD_10, led_state = !led_state);
	}

	{  // poll AK09940A
		if (auto t2 = micros(); t2 - t1 < 3100) {
			delayMicroseconds(3100 - (t2 - t1));  // > 3.1ms
		}

		/*ak000.start_measurement();
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
		ak110.start_measurement();*/

		Serial.write(static_cast<std::uint8_t>('M'));

		auto const scale_ak = std::bit_cast<std::array<std::uint8_t, sizeof(AK09940A::get_scale_factor())>>(AK09940A::get_scale_factor());
		Serial.write(scale_ak.data(), scale_ak.size());
		crc16.add(scale_ak.data(), scale_ak.size());

		// clang-format off
		auto const mag000 = ak000.get_measurement(); Serial.write(mag000.bytes.data(), mag000.bytes.size()); crc16.add(mag000.bytes.data(), mag000.bytes.size());
		auto const mag001 = ak001.get_measurement(); Serial.write(mag001.bytes.data(), mag001.bytes.size()); crc16.add(mag001.bytes.data(), mag001.bytes.size());
		auto const mag002 = ak002.get_measurement(); Serial.write(mag002.bytes.data(), mag002.bytes.size()); crc16.add(mag002.bytes.data(), mag002.bytes.size());
		auto const mag003 = ak003.get_measurement(); Serial.write(mag003.bytes.data(), mag003.bytes.size()); crc16.add(mag003.bytes.data(), mag003.bytes.size());
		auto const mag004 = ak004.get_measurement(); Serial.write(mag004.bytes.data(), mag004.bytes.size()); crc16.add(mag004.bytes.data(), mag004.bytes.size());
		auto const mag005 = ak005.get_measurement(); Serial.write(mag005.bytes.data(), mag005.bytes.size()); crc16.add(mag005.bytes.data(), mag005.bytes.size());
		auto const mag006 = ak006.get_measurement(); Serial.write(mag006.bytes.data(), mag006.bytes.size()); crc16.add(mag006.bytes.data(), mag006.bytes.size());
		auto const mag007 = ak007.get_measurement(); Serial.write(mag007.bytes.data(), mag007.bytes.size()); crc16.add(mag007.bytes.data(), mag007.bytes.size());
		auto const mag008 = ak008.get_measurement(); Serial.write(mag008.bytes.data(), mag008.bytes.size()); crc16.add(mag008.bytes.data(), mag008.bytes.size());
		auto const mag009 = ak009.get_measurement(); Serial.write(mag009.bytes.data(), mag009.bytes.size()); crc16.add(mag009.bytes.data(), mag009.bytes.size());
		auto const mag010 = ak010.get_measurement(); Serial.write(mag010.bytes.data(), mag010.bytes.size()); crc16.add(mag010.bytes.data(), mag010.bytes.size());
		auto const mag011 = ak011.get_measurement(); Serial.write(mag011.bytes.data(), mag011.bytes.size()); crc16.add(mag011.bytes.data(), mag011.bytes.size());
		auto const mag012 = ak012.get_measurement(); Serial.write(mag012.bytes.data(), mag012.bytes.size()); crc16.add(mag012.bytes.data(), mag012.bytes.size());
		auto const mag013 = ak013.get_measurement(); Serial.write(mag013.bytes.data(), mag013.bytes.size()); crc16.add(mag013.bytes.data(), mag013.bytes.size());
		auto const mag014 = ak014.get_measurement(); Serial.write(mag014.bytes.data(), mag014.bytes.size()); crc16.add(mag014.bytes.data(), mag014.bytes.size());
		auto const mag015 = ak015.get_measurement(); Serial.write(mag015.bytes.data(), mag015.bytes.size()); crc16.add(mag015.bytes.data(), mag015.bytes.size());
		auto const mag016 = ak016.get_measurement(); Serial.write(mag016.bytes.data(), mag016.bytes.size()); crc16.add(mag016.bytes.data(), mag016.bytes.size());
		auto const mag017 = ak017.get_measurement(); Serial.write(mag017.bytes.data(), mag017.bytes.size()); crc16.add(mag017.bytes.data(), mag017.bytes.size());
		auto const mag018 = ak018.get_measurement(); Serial.write(mag018.bytes.data(), mag018.bytes.size()); crc16.add(mag018.bytes.data(), mag018.bytes.size());
		auto const mag019 = ak019.get_measurement(); Serial.write(mag019.bytes.data(), mag019.bytes.size()); crc16.add(mag019.bytes.data(), mag019.bytes.size());
		auto const mag020 = ak020.get_measurement(); Serial.write(mag020.bytes.data(), mag020.bytes.size()); crc16.add(mag020.bytes.data(), mag020.bytes.size());
		auto const mag021 = ak021.get_measurement(); Serial.write(mag021.bytes.data(), mag021.bytes.size()); crc16.add(mag021.bytes.data(), mag021.bytes.size());
		auto const mag022 = ak022.get_measurement(); Serial.write(mag022.bytes.data(), mag022.bytes.size()); crc16.add(mag022.bytes.data(), mag022.bytes.size());
		auto const mag023 = ak023.get_measurement(); Serial.write(mag023.bytes.data(), mag023.bytes.size()); crc16.add(mag023.bytes.data(), mag023.bytes.size());
		auto const mag024 = ak024.get_measurement(); Serial.write(mag024.bytes.data(), mag024.bytes.size()); crc16.add(mag024.bytes.data(), mag024.bytes.size());
		auto const mag025 = ak025.get_measurement(); Serial.write(mag025.bytes.data(), mag025.bytes.size()); crc16.add(mag025.bytes.data(), mag025.bytes.size());
		auto const mag026 = ak026.get_measurement(); Serial.write(mag026.bytes.data(), mag026.bytes.size()); crc16.add(mag026.bytes.data(), mag026.bytes.size());
		auto const mag027 = ak027.get_measurement(); Serial.write(mag027.bytes.data(), mag027.bytes.size()); crc16.add(mag027.bytes.data(), mag027.bytes.size());
		auto const mag028 = ak028.get_measurement(); Serial.write(mag028.bytes.data(), mag028.bytes.size()); crc16.add(mag028.bytes.data(), mag028.bytes.size());
		auto const mag029 = ak029.get_measurement(); Serial.write(mag029.bytes.data(), mag029.bytes.size()); crc16.add(mag029.bytes.data(), mag029.bytes.size());
		auto const mag030 = ak030.get_measurement(); Serial.write(mag030.bytes.data(), mag030.bytes.size()); crc16.add(mag030.bytes.data(), mag030.bytes.size());
		auto const mag031 = ak031.get_measurement(); Serial.write(mag031.bytes.data(), mag031.bytes.size()); crc16.add(mag031.bytes.data(), mag031.bytes.size());
		auto const mag032 = ak032.get_measurement(); Serial.write(mag032.bytes.data(), mag032.bytes.size()); crc16.add(mag032.bytes.data(), mag032.bytes.size());
		auto const mag033 = ak033.get_measurement(); Serial.write(mag033.bytes.data(), mag033.bytes.size()); crc16.add(mag033.bytes.data(), mag033.bytes.size());

		auto const mag034 = ak034.get_measurement(); Serial.write(mag034.bytes.data(), mag034.bytes.size()); crc16.add(mag034.bytes.data(), mag034.bytes.size());
		auto const mag035 = ak035.get_measurement(); Serial.write(mag035.bytes.data(), mag035.bytes.size()); crc16.add(mag035.bytes.data(), mag035.bytes.size());
		auto const mag036 = ak036.get_measurement(); Serial.write(mag036.bytes.data(), mag036.bytes.size()); crc16.add(mag036.bytes.data(), mag036.bytes.size());
		auto const mag037 = ak037.get_measurement(); Serial.write(mag037.bytes.data(), mag037.bytes.size()); crc16.add(mag037.bytes.data(), mag037.bytes.size());
		auto const mag038 = ak038.get_measurement(); Serial.write(mag038.bytes.data(), mag038.bytes.size()); crc16.add(mag038.bytes.data(), mag038.bytes.size());
		auto const mag039 = ak039.get_measurement(); Serial.write(mag039.bytes.data(), mag039.bytes.size()); crc16.add(mag039.bytes.data(), mag039.bytes.size());
		auto const mag040 = ak040.get_measurement(); Serial.write(mag040.bytes.data(), mag040.bytes.size()); crc16.add(mag040.bytes.data(), mag040.bytes.size());
		auto const mag041 = ak041.get_measurement(); Serial.write(mag041.bytes.data(), mag041.bytes.size()); crc16.add(mag041.bytes.data(), mag041.bytes.size());
		auto const mag042 = ak042.get_measurement(); Serial.write(mag042.bytes.data(), mag042.bytes.size()); crc16.add(mag042.bytes.data(), mag042.bytes.size());
		auto const mag043 = ak043.get_measurement(); Serial.write(mag043.bytes.data(), mag043.bytes.size()); crc16.add(mag043.bytes.data(), mag043.bytes.size());
		auto const mag044 = ak044.get_measurement(); Serial.write(mag044.bytes.data(), mag044.bytes.size()); crc16.add(mag044.bytes.data(), mag044.bytes.size());
		auto const mag045 = ak045.get_measurement(); Serial.write(mag045.bytes.data(), mag045.bytes.size()); crc16.add(mag045.bytes.data(), mag045.bytes.size());
		auto const mag046 = ak046.get_measurement(); Serial.write(mag046.bytes.data(), mag046.bytes.size()); crc16.add(mag046.bytes.data(), mag046.bytes.size());
		auto const mag047 = ak047.get_measurement(); Serial.write(mag047.bytes.data(), mag047.bytes.size()); crc16.add(mag047.bytes.data(), mag047.bytes.size());
		auto const mag048 = ak048.get_measurement(); Serial.write(mag048.bytes.data(), mag048.bytes.size()); crc16.add(mag048.bytes.data(), mag048.bytes.size());
		auto const mag049 = ak049.get_measurement(); Serial.write(mag049.bytes.data(), mag049.bytes.size()); crc16.add(mag049.bytes.data(), mag049.bytes.size());
		auto const mag050 = ak050.get_measurement(); Serial.write(mag050.bytes.data(), mag050.bytes.size()); crc16.add(mag050.bytes.data(), mag050.bytes.size());
		auto const mag051 = ak051.get_measurement(); Serial.write(mag051.bytes.data(), mag051.bytes.size()); crc16.add(mag051.bytes.data(), mag051.bytes.size());
		auto const mag052 = ak052.get_measurement(); Serial.write(mag052.bytes.data(), mag052.bytes.size()); crc16.add(mag052.bytes.data(), mag052.bytes.size());
		auto const mag053 = ak053.get_measurement(); Serial.write(mag053.bytes.data(), mag053.bytes.size()); crc16.add(mag053.bytes.data(), mag053.bytes.size());
		auto const mag054 = ak054.get_measurement(); Serial.write(mag054.bytes.data(), mag054.bytes.size()); crc16.add(mag054.bytes.data(), mag054.bytes.size());
		auto const mag055 = ak055.get_measurement(); Serial.write(mag055.bytes.data(), mag055.bytes.size()); crc16.add(mag055.bytes.data(), mag055.bytes.size());
		auto const mag056 = ak056.get_measurement(); Serial.write(mag056.bytes.data(), mag056.bytes.size()); crc16.add(mag056.bytes.data(), mag056.bytes.size());
		auto const mag057 = ak057.get_measurement(); Serial.write(mag057.bytes.data(), mag057.bytes.size()); crc16.add(mag057.bytes.data(), mag057.bytes.size());
		auto const mag058 = ak058.get_measurement(); Serial.write(mag058.bytes.data(), mag058.bytes.size()); crc16.add(mag058.bytes.data(), mag058.bytes.size());
		auto const mag059 = ak059.get_measurement(); Serial.write(mag059.bytes.data(), mag059.bytes.size()); crc16.add(mag059.bytes.data(), mag059.bytes.size());
		auto const mag060 = ak060.get_measurement(); Serial.write(mag060.bytes.data(), mag060.bytes.size()); crc16.add(mag060.bytes.data(), mag060.bytes.size());
		auto const mag061 = ak061.get_measurement(); Serial.write(mag061.bytes.data(), mag061.bytes.size()); crc16.add(mag061.bytes.data(), mag061.bytes.size());
		auto const mag062 = ak062.get_measurement(); Serial.write(mag062.bytes.data(), mag062.bytes.size()); crc16.add(mag062.bytes.data(), mag062.bytes.size());
		auto const mag063 = ak063.get_measurement(); Serial.write(mag063.bytes.data(), mag063.bytes.size()); crc16.add(mag063.bytes.data(), mag063.bytes.size());
		auto const mag064 = ak064.get_measurement(); Serial.write(mag064.bytes.data(), mag064.bytes.size()); crc16.add(mag064.bytes.data(), mag064.bytes.size());
		auto const mag065 = ak065.get_measurement(); Serial.write(mag065.bytes.data(), mag065.bytes.size()); crc16.add(mag065.bytes.data(), mag065.bytes.size());
		auto const mag066 = ak066.get_measurement(); Serial.write(mag066.bytes.data(), mag066.bytes.size()); crc16.add(mag066.bytes.data(), mag066.bytes.size());
		auto const mag067 = ak067.get_measurement(); Serial.write(mag067.bytes.data(), mag067.bytes.size()); crc16.add(mag067.bytes.data(), mag067.bytes.size());
		auto const mag068 = ak068.get_measurement(); Serial.write(mag068.bytes.data(), mag068.bytes.size()); crc16.add(mag068.bytes.data(), mag068.bytes.size());
		auto const mag069 = ak069.get_measurement(); Serial.write(mag069.bytes.data(), mag069.bytes.size()); crc16.add(mag069.bytes.data(), mag069.bytes.size());
		auto const mag070 = ak070.get_measurement(); Serial.write(mag070.bytes.data(), mag070.bytes.size()); crc16.add(mag070.bytes.data(), mag070.bytes.size());
		auto const mag071 = ak071.get_measurement(); Serial.write(mag071.bytes.data(), mag071.bytes.size()); crc16.add(mag071.bytes.data(), mag071.bytes.size());
		auto const mag072 = ak072.get_measurement(); Serial.write(mag072.bytes.data(), mag072.bytes.size()); crc16.add(mag072.bytes.data(), mag072.bytes.size());
		auto const mag073 = ak073.get_measurement(); Serial.write(mag073.bytes.data(), mag073.bytes.size()); crc16.add(mag073.bytes.data(), mag073.bytes.size());
		auto const mag074 = ak074.get_measurement(); Serial.write(mag074.bytes.data(), mag074.bytes.size()); crc16.add(mag074.bytes.data(), mag074.bytes.size());
		auto const mag075 = ak075.get_measurement(); Serial.write(mag075.bytes.data(), mag075.bytes.size()); crc16.add(mag075.bytes.data(), mag075.bytes.size());
		auto const mag076 = ak076.get_measurement(); Serial.write(mag076.bytes.data(), mag076.bytes.size()); crc16.add(mag076.bytes.data(), mag076.bytes.size());

		auto const mag077 = ak077.get_measurement(); Serial.write(mag077.bytes.data(), mag077.bytes.size()); crc16.add(mag077.bytes.data(), mag077.bytes.size());
		auto const mag078 = ak078.get_measurement(); Serial.write(mag078.bytes.data(), mag078.bytes.size()); crc16.add(mag078.bytes.data(), mag078.bytes.size());
		auto const mag079 = ak079.get_measurement(); Serial.write(mag079.bytes.data(), mag079.bytes.size()); crc16.add(mag079.bytes.data(), mag079.bytes.size());
		auto const mag080 = ak080.get_measurement(); Serial.write(mag080.bytes.data(), mag080.bytes.size()); crc16.add(mag080.bytes.data(), mag080.bytes.size());
		auto const mag081 = ak081.get_measurement(); Serial.write(mag081.bytes.data(), mag081.bytes.size()); crc16.add(mag081.bytes.data(), mag081.bytes.size());
		auto const mag082 = ak082.get_measurement(); Serial.write(mag082.bytes.data(), mag082.bytes.size()); crc16.add(mag082.bytes.data(), mag082.bytes.size());
		auto const mag083 = ak083.get_measurement(); Serial.write(mag083.bytes.data(), mag083.bytes.size()); crc16.add(mag083.bytes.data(), mag083.bytes.size());
		auto const mag084 = ak084.get_measurement(); Serial.write(mag084.bytes.data(), mag084.bytes.size()); crc16.add(mag084.bytes.data(), mag084.bytes.size());
		auto const mag085 = ak085.get_measurement(); Serial.write(mag085.bytes.data(), mag085.bytes.size()); crc16.add(mag085.bytes.data(), mag085.bytes.size());
		auto const mag086 = ak086.get_measurement(); Serial.write(mag086.bytes.data(), mag086.bytes.size()); crc16.add(mag086.bytes.data(), mag086.bytes.size());
		auto const mag087 = ak087.get_measurement(); Serial.write(mag087.bytes.data(), mag087.bytes.size()); crc16.add(mag087.bytes.data(), mag087.bytes.size());
		auto const mag088 = ak088.get_measurement(); Serial.write(mag088.bytes.data(), mag088.bytes.size()); crc16.add(mag088.bytes.data(), mag088.bytes.size());
		auto const mag089 = ak089.get_measurement(); Serial.write(mag089.bytes.data(), mag089.bytes.size()); crc16.add(mag089.bytes.data(), mag089.bytes.size());
		auto const mag090 = ak090.get_measurement(); Serial.write(mag090.bytes.data(), mag090.bytes.size()); crc16.add(mag090.bytes.data(), mag090.bytes.size());
		auto const mag091 = ak091.get_measurement(); Serial.write(mag091.bytes.data(), mag091.bytes.size()); crc16.add(mag091.bytes.data(), mag091.bytes.size());
		auto const mag092 = ak092.get_measurement(); Serial.write(mag092.bytes.data(), mag092.bytes.size()); crc16.add(mag092.bytes.data(), mag092.bytes.size());
		auto const mag093 = ak093.get_measurement(); Serial.write(mag093.bytes.data(), mag093.bytes.size()); crc16.add(mag093.bytes.data(), mag093.bytes.size());
		auto const mag094 = ak094.get_measurement(); Serial.write(mag094.bytes.data(), mag094.bytes.size()); crc16.add(mag094.bytes.data(), mag094.bytes.size());
		auto const mag095 = ak095.get_measurement(); Serial.write(mag095.bytes.data(), mag095.bytes.size()); crc16.add(mag095.bytes.data(), mag095.bytes.size());
		auto const mag096 = ak096.get_measurement(); Serial.write(mag096.bytes.data(), mag096.bytes.size()); crc16.add(mag096.bytes.data(), mag096.bytes.size());
		auto const mag097 = ak097.get_measurement(); Serial.write(mag097.bytes.data(), mag097.bytes.size()); crc16.add(mag097.bytes.data(), mag097.bytes.size());
		auto const mag098 = ak098.get_measurement(); Serial.write(mag098.bytes.data(), mag098.bytes.size()); crc16.add(mag098.bytes.data(), mag098.bytes.size());
		auto const mag099 = ak099.get_measurement(); Serial.write(mag099.bytes.data(), mag099.bytes.size()); crc16.add(mag099.bytes.data(), mag099.bytes.size());
		auto const mag100 = ak100.get_measurement(); Serial.write(mag100.bytes.data(), mag100.bytes.size()); crc16.add(mag100.bytes.data(), mag100.bytes.size());
		auto const mag101 = ak101.get_measurement(); Serial.write(mag101.bytes.data(), mag101.bytes.size()); crc16.add(mag101.bytes.data(), mag101.bytes.size());
		auto const mag102 = ak102.get_measurement(); Serial.write(mag102.bytes.data(), mag102.bytes.size()); crc16.add(mag102.bytes.data(), mag102.bytes.size());
		auto const mag103 = ak103.get_measurement(); Serial.write(mag103.bytes.data(), mag103.bytes.size()); crc16.add(mag103.bytes.data(), mag103.bytes.size());
		auto const mag104 = ak104.get_measurement(); Serial.write(mag104.bytes.data(), mag104.bytes.size()); crc16.add(mag104.bytes.data(), mag104.bytes.size());
		auto const mag105 = ak105.get_measurement(); Serial.write(mag105.bytes.data(), mag105.bytes.size()); crc16.add(mag105.bytes.data(), mag105.bytes.size());
		auto const mag106 = ak106.get_measurement(); Serial.write(mag106.bytes.data(), mag106.bytes.size()); crc16.add(mag106.bytes.data(), mag106.bytes.size());
		auto const mag107 = ak107.get_measurement(); Serial.write(mag107.bytes.data(), mag107.bytes.size()); crc16.add(mag107.bytes.data(), mag107.bytes.size());
		auto const mag108 = ak108.get_measurement(); Serial.write(mag108.bytes.data(), mag108.bytes.size()); crc16.add(mag108.bytes.data(), mag108.bytes.size());
		auto const mag109 = ak109.get_measurement(); Serial.write(mag109.bytes.data(), mag109.bytes.size()); crc16.add(mag109.bytes.data(), mag109.bytes.size());
		auto const mag110 = ak110.get_measurement(); Serial.write(mag110.bytes.data(), mag110.bytes.size()); crc16.add(mag110.bytes.data(), mag110.bytes.size());
		// clang-format on

		auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
		Serial.write(timestamp_.data(), timestamp_.size());
		crc16.add(timestamp_.data(), timestamp_.size());

		auto const crc_value = std::bit_cast<std::array<uint8_t, 2>>(crc16.calc());
		Serial.write(crc_value.data(), crc_value.size());

		Serial.write(static_cast<std::uint8_t>('M'));

		crc16.restart();
	}
}
