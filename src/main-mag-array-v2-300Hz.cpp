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

decltype(micros()) t0 = 0;

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
		spi1.beginTransaction(SPISettings(2'000'000, BitOrder::MSBFIRST, SPI_MODE3));
		spi2.beginTransaction(SPISettings(2'000'000, BitOrder::MSBFIRST, SPI_MODE3));
		spi3.beginTransaction(SPISettings(2'000'000, BitOrder::MSBFIRST, SPI_MODE3));
	}

	delay(100);

	{  // config I2C2 and imu
		i2c2.begin();
		i2c2.setClock(400'000);

		delay(100);

		imu.begin_sensor_fusion();
	}

	delay(100);

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

	{  // trigger sensors
		digitalWrite(PI_7, HIGH);
		delayMicroseconds(5);  // > 3us
		digitalWrite(PI_7, LOW);
		t0 = micros();
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
#if NDEBUG
	static CRC8 crc8;
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
#else
	if (auto accel_data = imu.get_measurement_accelerometer(); accel_data) {
		common2::println("Accelerometer: ", accel_data.value());
	}
	if (auto gyro_data = imu.get_measurement_gyro(); gyro_data) {
		common2::println("Gyro: ", gyro_data.value());
	}
#endif
}

void poll_imu_fifo() {
#if NDEBUG
	static CRC8 crc8;
	static std::uint64_t fifo_timestamp = 0;
	for (std::variant<LSM6DSV16X::NoData, AccelerationDataRaw, GyroDataRaw, std::uint64_t> val; !std::holds_alternative<LSM6DSV16X::NoData>(val = imu.get_measurement_fifo());) {
		if (std::holds_alternative<AccelerationDataRaw>(val)) {
			auto const accel_data = std::get<AccelerationDataRaw>(val);

			Serial.write(static_cast<std::uint8_t>('A'));

			auto const scale_imu_accel = std::bit_cast<std::array<std::uint8_t, sizeof(float)>>(imu.get_scale_factor_accelerometer());
			Serial.write(scale_imu_accel.data(), scale_imu_accel.size());
			crc8.add(scale_imu_accel.data(), scale_imu_accel.size());

			Serial.write(accel_data.bytes.data(), accel_data.bytes.size());
			crc8.add(accel_data.bytes.data(), accel_data.bytes.size());

			auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(fifo_timestamp)>>(fifo_timestamp);
			Serial.write(timestamp_.data(), timestamp_.size());
			crc8.add(timestamp_.data(), timestamp_.size());

			Serial.write(crc8.calc());

			Serial.write(static_cast<std::uint8_t>('A'));

			crc8.restart();
		}
		if (std::holds_alternative<GyroDataRaw>(val)) {
			auto const gyro_data = std::get<GyroDataRaw>(val);

			Serial.write(static_cast<std::uint8_t>('G'));

			auto const scale_imu_gyro = std::bit_cast<std::array<std::uint8_t, sizeof(float)>>(imu.get_scale_factor_gyro());
			Serial.write(scale_imu_gyro.data(), scale_imu_gyro.size());
			crc8.add(scale_imu_gyro.data(), scale_imu_gyro.size());

			Serial.write(gyro_data.bytes.data(), gyro_data.bytes.size());
			crc8.add(gyro_data.bytes.data(), gyro_data.bytes.size());

			auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(fifo_timestamp)>>(fifo_timestamp);
			Serial.write(timestamp_.data(), timestamp_.size());
			crc8.add(timestamp_.data(), timestamp_.size());

			Serial.write(crc8.calc());

			Serial.write(static_cast<std::uint8_t>('G'));

			crc8.restart();
		}
		if (std::holds_alternative<std::uint64_t>(val)) {
			fifo_timestamp = std::get<std::uint64_t>(val);
		}
	}
#else
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
#endif
}

void poll_imu_sensor_fusion() {
#if NDEBUG
	static CRC8 crc8;
	for (std::variant<LSM6DSV16X::NoData, RotationQuaternion, GyroBiasVector, GravityVector> val; !std::holds_alternative<LSM6DSV16X::NoData>(val = imu.get_measurement_sensor_fusion());) {
		if (std::holds_alternative<RotationQuaternion>(val)) {
			auto const quaternion = std::get<RotationQuaternion>(val);
			Serial.write(static_cast<std::uint8_t>('R'));

			Serial.write(quaternion.bytes.data(), quaternion.bytes.size());
			crc8.add(quaternion.bytes.data(), quaternion.bytes.size());

			Serial.write(crc8.calc());

			Serial.write(static_cast<std::uint8_t>('R'));
			crc8.restart();
		} else if (std::holds_alternative<GyroBiasVector>(val)) {
			auto const gyro_bias = std::get<GyroBiasVector>(val);
			Serial.write(static_cast<std::uint8_t>('B'));

			Serial.write(gyro_bias.bytes.data(), gyro_bias.bytes.size());
			crc8.add(gyro_bias.bytes.data(), gyro_bias.bytes.size());

			Serial.write(crc8.calc());

			Serial.write(static_cast<std::uint8_t>('B'));
			crc8.restart();
		} else if (std::holds_alternative<GravityVector>(val)) {
			auto const vec = std::get<GravityVector>(val);
			Serial.write(static_cast<std::uint8_t>('V'));

			Serial.write(vec.bytes.data(), vec.bytes.size());
			crc8.add(vec.bytes.data(), vec.bytes.size());

			Serial.write(crc8.calc());

			Serial.write(static_cast<std::uint8_t>('V'));
			crc8.restart();
		}
	}
#else
	for (std::variant<LSM6DSV16X::NoData, RotationQuaternion, GyroBiasVector, GravityVector> val; !std::holds_alternative<LSM6DSV16X::NoData>(val = imu.get_measurement_sensor_fusion());) {
		if (std::holds_alternative<RotationQuaternion>(val)) {
			auto const tmp = std::get<RotationQuaternion>(val);
			common2::println("RotationQuaternion: x = ", tmp.rx, ", y = ", tmp.ry, ", z = ", tmp.rz, ", w = ", tmp.rw);
		}
		if (std::holds_alternative<GyroBiasVector>(val)) {
			auto const tmp = std::get<GyroBiasVector>(val);
			common2::println("GyroBiasVector: x = ", tmp.gbx, ", y = ", tmp.gby, ", z = ", tmp.gbz);
		}
		if (std::holds_alternative<GravityVector>(val)) {
			auto const tmp = std::get<GravityVector>(val);
			common2::println("GravityVector: x = ", tmp.gvx, ", y = ", tmp.gvy, ", z = ", tmp.gvz);
		}
	}
#endif
}

void loop() {
	static CRC16 crc16(0x8005, 0, false, true, true);

	{  // get imu data
		imu.start_measurement_fifo();

		poll_imu_sensor_fusion();
	}

	{  // check overflow
		// std::tie(time_delay, time_offset) = common::sync_time();
		if (std::exchange(timestamp, 1000ULL * micros() + time_offset) >= timestamp) {  // when time overflow is detected:
			common2::message("micros overflow detected!");
			delayMicroseconds(3100);
			return;
		}
	}

	{  // blink LED
		digitalWrite(PD_10, led_state = !led_state);
	}

	{  // poll AK09940A
		if (auto t1 = micros(); t1 - t0 < 3100) {
			delayMicroseconds(3100 - (t1 - t0));  // > 3.1ms
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

		// clang-format off
		auto const mag000 = ak000.get_measurement();
		auto const mag003 = ak003.get_measurement();
		auto const mag005 = ak005.get_measurement();
		auto const mag008 = ak008.get_measurement();
		auto const mag027 = ak027.get_measurement();
		auto const mag032 = ak032.get_measurement();
		auto const mag051 = ak051.get_measurement();
		auto const mag055 = ak055.get_measurement();
		auto const mag059 = ak059.get_measurement();
		auto const mag078 = ak078.get_measurement();
		auto const mag083 = ak083.get_measurement();
		auto const mag102 = ak102.get_measurement();
		auto const mag105 = ak105.get_measurement();
		auto const mag107 = ak107.get_measurement();
		auto const mag110 = ak110.get_measurement();
		// clang-format on

		// trigger sensors
		digitalWrite(PI_7, HIGH);
		delayMicroseconds(5);  // > 3us
		digitalWrite(PI_7, LOW);
		t0 = micros();

		Serial.write(static_cast<std::uint8_t>('M'));

		auto const scale_ak = std::bit_cast<std::array<std::uint8_t, sizeof(AK09940A::get_scale_factor())>>(AK09940A::get_scale_factor());
		Serial.write(scale_ak.data(), scale_ak.size());
		crc16.add(scale_ak.data(), scale_ak.size());

		// clang-format off
		Serial.write(mag000.bytes.data(), mag000.bytes.size()); crc16.add(mag000.bytes.data(), mag000.bytes.size());
		Serial.write(mag003.bytes.data(), mag003.bytes.size()); crc16.add(mag003.bytes.data(), mag003.bytes.size());
		Serial.write(mag005.bytes.data(), mag005.bytes.size()); crc16.add(mag005.bytes.data(), mag005.bytes.size());
		Serial.write(mag008.bytes.data(), mag008.bytes.size()); crc16.add(mag008.bytes.data(), mag008.bytes.size());
		Serial.write(mag027.bytes.data(), mag027.bytes.size()); crc16.add(mag027.bytes.data(), mag027.bytes.size());
		Serial.write(mag032.bytes.data(), mag032.bytes.size()); crc16.add(mag032.bytes.data(), mag032.bytes.size());
		Serial.write(mag051.bytes.data(), mag051.bytes.size()); crc16.add(mag051.bytes.data(), mag051.bytes.size());
		Serial.write(mag055.bytes.data(), mag055.bytes.size()); crc16.add(mag055.bytes.data(), mag055.bytes.size());
		Serial.write(mag059.bytes.data(), mag059.bytes.size()); crc16.add(mag059.bytes.data(), mag059.bytes.size());
		Serial.write(mag078.bytes.data(), mag078.bytes.size()); crc16.add(mag078.bytes.data(), mag078.bytes.size());
		Serial.write(mag083.bytes.data(), mag083.bytes.size()); crc16.add(mag083.bytes.data(), mag083.bytes.size());
		Serial.write(mag102.bytes.data(), mag102.bytes.size()); crc16.add(mag102.bytes.data(), mag102.bytes.size());
		Serial.write(mag105.bytes.data(), mag105.bytes.size()); crc16.add(mag105.bytes.data(), mag105.bytes.size());
		Serial.write(mag107.bytes.data(), mag107.bytes.size()); crc16.add(mag107.bytes.data(), mag107.bytes.size());
		Serial.write(mag110.bytes.data(), mag110.bytes.size()); crc16.add(mag110.bytes.data(), mag110.bytes.size());
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
