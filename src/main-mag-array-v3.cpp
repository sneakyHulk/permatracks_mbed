// Measuring firmware of mag-array-v3. The time test (main-mag-array-v3-time.cpp) runs the same clock, sensors, loop_frames()
// and drain_host() (keep both in step). The only clock is sof_clock (usb_sof.h): every time stamp comes from sof_clock.now().
#include <LSM6DSV16X.h>
#include <ADS131E08.h>
#include <Arduino.h>
#include <FLC100.h>
#include <H7Adc.h>
#include <HalSpi4.h>
#include <MCP9700B.h>
#include <MagneticFluxDensityDataRawFLC100.h>
#include <SPI.h>
#include <TemperatureDataRaw.h>
#include <Wire.h>
#include <WireMessages.h>
#include <common_parser.h>
#include <usb_sof.h>

#include <algorithm>
#include <array>
#include <boost/crc.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

using Crc = boost::crc_16_type;
using TemperatureMessage = TemperatureDataRawWireMessage<16>;
using MagMessage = MagneticFluxDensityRawWireMessage<16, MagneticFluxDensityDataRawFLC100>;

// bytes from the host: the USB SOF sync and pings (everything else is dropped)
static common::Parser<Header, Crc, SofSyncWireMessage, PingRequestWireMessage> host_parser;

// the only clock: host time in ns from the USB SOF (TIM5 + USB frame number) and the time since the last SOF (TIM8), see usb_sof.h
static UsbSofClock sof_clock;

// info frame [I][header][text: n][n][crc][I], the text is formatted by Arduino's Print::print, at most 255 characters.
// InfoPrint is a Print: every formatted character arrives in write(), goes on to Serial and into the checksum and length
template <common::Checksum C>
static void send_info(auto const&... args) {
	struct InfoPrint final : Print {
		C crc;
		std::uint8_t size = 0;

		std::size_t write(std::uint8_t const byte) override {
			Serial.write(byte);
			crc.process_byte(byte);
			++size;
			return 1;
		}
	} info;
	Header const header{sof_clock.now()};  // send time, host time (0 before the clock runs)

	Serial.write('I');
	for (auto const byte : std::as_bytes(std::span{&header, 1})) info.write(static_cast<std::uint8_t>(byte));
	info.size = 0;  // the header is in the checksum, but not in the length

	(info.print(args), ...);

	Serial.write(info.size);
	for (std::size_t k = 0; k < common::checksum_size<C>; ++k) Serial.write(static_cast<std::uint8_t>(info.crc.checksum() >> (8 * k)));
	Serial.write('I');
}

// One ADC driver each. tmp_adc1: PA/PC/PB channels. tmp_adc3: the PC2_C/PC3_C pads.
static H7Adc tmp_adc1(ADC1);
static H7Adc tmp_adc3(ADC3);

// One MCP9700B per temperature channel: (its H7Adc, channel). Comments show the pin.
static MCP9700B temp01(tmp_adc1, ADC_CHANNEL_16);  // PA0
static MCP9700B temp02(tmp_adc1, ADC_CHANNEL_10);  // PC0
static MCP9700B temp03(tmp_adc1, ADC_CHANNEL_11);  // PC1
static MCP9700B temp04(tmp_adc3, ADC_CHANNEL_0);   // PC2_C
static MCP9700B temp05(tmp_adc3, ADC_CHANNEL_1);   // PC3_C
static MCP9700B temp06(tmp_adc1, ADC_CHANNEL_15);  // PA3
static MCP9700B temp07(tmp_adc1, ADC_CHANNEL_14);  // PA2
static MCP9700B temp08(tmp_adc1, ADC_CHANNEL_17);  // PA1
static MCP9700B temp09(tmp_adc1, ADC_CHANNEL_18);  // PA4
static MCP9700B temp10(tmp_adc1, ADC_CHANNEL_3);   // PA6
static MCP9700B temp11(tmp_adc1, ADC_CHANNEL_19);  // PA5
static MCP9700B temp12(tmp_adc1, ADC_CHANNEL_7);   // PA7
static MCP9700B temp13(tmp_adc1, ADC_CHANNEL_4);   // PC4
static MCP9700B temp14(tmp_adc1, ADC_CHANNEL_8);   // PC5
static MCP9700B temp15(tmp_adc1, ADC_CHANNEL_9);   // PB0
static MCP9700B temp16(tmp_adc1, ADC_CHANNEL_5);   // PB1

// --- Magnetometers: TWO daisy-chained ADS131E08 (16 channels) on SPI4 ---
// SCLK=PE2, DRDY=PE3, CS=PE4, MISO=PE5, MOSI=PE6. One object drives both chips.
// Arduino pin NUMBERS (PE4, not PE_4): the PinName values are off by 2 on this variant.
static HalSpi4 spi4;                                               // direct-HAL SPI4 master (SCLK=PE2, MISO=PE5, MOSI=PE6, AF5)
static ADS131E08<true> mag_adc(spi4, PE4, PE3, PE14, PB10, PB11);  // (spi, CS, DRDY, START, nRESET A, nRESET B), 1 kSPS, 24-bit, VREF=4.096 V

// One FLC100 per channel: (its ADS131E08, channel 0..15). Ch 0..7 = chip 0,
// ch 8..15 = chip 1 in the daisy chain. B[µT] = 50 * V_adc.
static FLC100 flc01(mag_adc, 7);
static FLC100 flc02(mag_adc, 6);
static FLC100 flc03(mag_adc, 5);
static FLC100 flc04(mag_adc, 4);
static FLC100 flc05(mag_adc, 3);
static FLC100 flc06(mag_adc, 2);
static FLC100 flc07(mag_adc, 1);
static FLC100 flc08(mag_adc, 0);
static FLC100 flc09(mag_adc, 15);
static FLC100 flc10(mag_adc, 14);
static FLC100 flc11(mag_adc, 13);
static FLC100 flc12(mag_adc, 12);
static FLC100 flc13(mag_adc, 11);
static FLC100 flc14(mag_adc, 10);
static FLC100 flc15(mag_adc, 9);
static FLC100 flc16(mag_adc, 8);

// --- IMU: LSM6DSV16X on I2C1 (SDA1=PB7, SCL1=PB6), address 0x6A (SDO/SA0 low) ---
static TwoWire i2c1(PB7, PB6);
static LSM6DSV16X imu(&i2c1, 0b110'1010);

// USB serial up, waits for the host to open the port (SOFs flow from then on), says hello
static void setup_usb(char const* const hello) {
	Serial.begin();
	while (!Serial) {
	}  // wait for enumeration → USB SOFs are now flowing
	send_info<Crc>(hello);
	send_info<Crc>("=== BUILD " __DATE__ " " __TIME__ " ===");  // confirms a fresh flash is running
}

// clock: begin, then sync with the host (asks for it until the host sends its SOF sync, host side: data_collection UsbSofSync.h)
static void setup_clock() {
	// until it runs: without it every time stamp is 0 (a sync does not start it)
	for (auto begun = sof_clock.begin(); !begun; begun = sof_clock.begin()) {
		send_info<Crc>("UsbSofClock begin failed: ", begun.error().what(), " (", begun.error().code, "), retry");
		delay(100);
	}
	send_info<Crc>("UsbSofClock running, ", sof_clock.ticks_per_sof, " TIM8 ticks per SOF");

	// ask the host for the sync (SofSyncRequestWireMessage) about once per second until the SofSyncWireMessage arrives
	// (the host measures ~10 s before it answers and ignores the requests meanwhile; GUI: MagArrayV3Visitor, data_collection: UsbSofSync.h)
	for (;;) {
		SofSyncRequestWireMessage request{};
		request.timestamp = sof_clock.now();  // send time (not synced yet)
		auto const frame = common::encode<Crc>(request);
		Serial.write(frame.data(), frame.size());

		if (auto const synced = sof_clock.sync(host_parser, 1'000'000); !synced) {
			send_info<Crc>("UsbSofClock: ", synced.error().what());
		} else {
			send_info<Crc>("UsbSofClock synced, ", sof_clock.ps_per_frame, " ps per frame");
			break;
		}
	}
}

// temperature ADCs and both ADS131E08, sampling from the end of this on
static void setup_sensors() {
	tmp_adc1.begin();  // configure/calibrate ADC1 (PA/PC/PB channels) — logs its own steps
	tmp_adc3.begin();  // configure/calibrate ADC3 (PC2_C/PC3_C pads) — logs its own steps

	spi4.begin();         // direct-HAL SPI4 master (kernel clock + GPIO AF5 + master init)
	mag_adc.begin();      // pins, hardware reset, SDATAC, ID check, write + verify registers (logs each step)
	mag_adc.self_test();  // both chips on the internal test signal: all 16 channels ~ -3495 + offset, status words 0xC...
	mag_adc.start();      // RDATAC + START last: both chips sample synchronously from here on
}

// LSM6DSV16X: I2C1 up, then the class does the rest like in mag-array-v2: sensor fusion (game rotation, gravity, gyro bias) at 120 Hz
// into the FIFO (prints its steps as plain text)
static void setup_imu() {
	i2c1.begin();
	i2c1.setClock(400'000);  // 1 MHz does not work on this board (I2C errors)
	imu.begin_sensor_fusion();
	send_info<Crc>("LSM6DSV16X running, sensor fusion 120 Hz");
}

// Binary output: one TemperatureMessage and one MagMessage per mag sample, for common::Parser on the host.
// Paced by the ADS131E08 (mag_adc.read() waits for its DRDY, 1 kSPS on its own oscillator): one call ≈ 1 ms.
// measurement_timestamp: host time in ns (sof_clock) when measured, header timestamp: host time (sof_clock) when sent.
static void loop_frames() {
	{                     // [T][send time][measurement time][offset][scale][16 x TemperatureDataRaw][crc16][T], T[degC] = datapoint / scale + offset
		tmp_adc1.read();  // convert + latch every enabled ADC1 channel (temp1..3, temp6..16)
		tmp_adc3.read();  // convert + latch every enabled ADC3 channel (temp4, temp5)

		TemperatureMessage message{};
		message.measurement_timestamp = sof_clock.now();
		message.offset = MCP9700B::get_offset();
		message.scale = MCP9700B::get_scale_factor();
		message.data = {temp01.get_measurement(), temp02.get_measurement(), temp03.get_measurement(), temp04.get_measurement(), temp05.get_measurement(), temp06.get_measurement(), temp07.get_measurement(), temp08.get_measurement(),
		    temp09.get_measurement(), temp10.get_measurement(), temp11.get_measurement(), temp12.get_measurement(), temp13.get_measurement(), temp14.get_measurement(), temp15.get_measurement(), temp16.get_measurement()};

		message.timestamp = sof_clock.now();  // send time, host time
		auto const frame = common::encode<Crc>(message);
		Serial.write(frame.data(), frame.size());
	}

	{  // [Q] / [V] / [B][send time][measurement time = 0][data][crc16][Q] / [V] / [B]: every sensor fusion value of the LSM6DSV16X FIFO, like mag-array-v2
		// no waiting: every loop the FIFO fill level (one I2C read, ~0.1 ms), then at most one entry (~0.25 ms): more would push the loop over
		// 1 ms and lose a mag sample. The FIFO fills with ~360 entries/s, the loop takes up to 1000/s.
		for (auto n = std::min<std::uint16_t>(imu.start_measurement_fifo(), 2); n > 0; --n) {  // FIFO fill level: 0 = nothing to do this loop
			auto const value = imu.get_measurement_sensor_fusion();
			if (!value) continue;  // an entry of another kind
			if (std::holds_alternative<RotationQuaternion>(*value)) {
				QuaternionWireMessage message{};
				message.measurement_timestamp = 0;  // not known (the FIFO has no time stamps for sensor fusion)
				message.data = std::get<RotationQuaternion>(*value);
				message.timestamp = sof_clock.now();  // send time, host time
				auto const frame = common::encode<Crc>(message);
				Serial.write(frame.data(), frame.size());
			} else if (std::holds_alternative<GravityVector>(*value)) {
				GravityWireMessage message{};
				message.measurement_timestamp = 0;  // not known (the FIFO has no time stamps for sensor fusion)
				message.data = std::get<GravityVector>(*value);
				message.timestamp = sof_clock.now();  // send time, host time
				auto const frame = common::encode<Crc>(message);
				Serial.write(frame.data(), frame.size());
			} else if (std::holds_alternative<GyroBiasVector>(*value)) {
				GyroBiasWireMessage message{};
				message.measurement_timestamp = 0;  // not known (the FIFO has no time stamps for sensor fusion)
				message.data = std::get<GyroBiasVector>(*value);
				message.timestamp = sof_clock.now();  // send time, host time
				auto const frame = common::encode<Crc>(message);
				Serial.write(frame.data(), frame.size());
			}
		}
	}

	{                    // [M][send time][measurement time][scale][16 x MagneticFluxDensityDataRawFLC100][crc16][M], B[uT] = datapoint / scale
		mag_adc.read();  // wait for DRDY, latch one synchronized 55-byte frame from both ADS131E08

		MagMessage message{};
		message.measurement_timestamp = sof_clock.now();
		message.scale = static_cast<std::int32_t>(std::lround(FLC100<ADS131E08<true>>::get_scale_factor() * 1e-6));  // LSB per uT (LSB per tesla does not fit into int32)
		message.data = {flc01.get_measurement(), flc02.get_measurement(), flc03.get_measurement(), flc04.get_measurement(), flc05.get_measurement(), flc06.get_measurement(), flc07.get_measurement(), flc08.get_measurement(),
		    flc09.get_measurement(), flc10.get_measurement(), flc11.get_measurement(), flc12.get_measurement(), flc13.get_measurement(), flc14.get_measurement(), flc15.get_measurement(), flc16.get_measurement()};

		message.timestamp = sof_clock.now();  // send time, host time
		auto const frame = common::encode<Crc>(message);
		Serial.write(frame.data(), frame.size());
	}
}

// reads what the host sends after the start (all of it, so the USB receive buffer never fills up):
// a SOF sync re-syncs the clock (blocks up to ~1 ms, so one mag sample can be late), a ping is answered right away with
// sof_clock.now() (the host checks that it lies between its send and receive time), everything else is dropped
static void drain_host() {
	while (Serial.available() > 0) {
		std::uint8_t const data = static_cast<std::uint8_t>(Serial.read());
		for (auto result = host_parser.parse(std::span{&data, 1}); result; result = host_parser.parse()) {
			if (auto const* const sync = std::get_if<SofSyncWireMessage>(&*result)) {
				sof_clock.sync(*sync);
				send_info<Crc>("UsbSofClock re-synced, ", sof_clock.ps_per_frame, " ps per frame");
			} else if (auto const* const ping = std::get_if<PingRequestWireMessage>(&*result)) {
				PingResponseWireMessage response{};
				response.time_ns = sof_clock.now();
				response.t0 = ping->timestamp;
				response.timestamp = sof_clock.now();  // send time, host time
				auto const frame = common::encode<Crc>(response);
				Serial.write(frame.data(), frame.size());
			}
		}
	}
}

static bool led_state = LOW;

void setup() {
	{  // turn led on (wiring: 3V3 → resistor → LED → pin, i.e. active-low → LOW = on)
		pinMode(PC11, OUTPUT);
		digitalWrite(PC11, !led_state);  // led_state == LOW → LED on
	}

	setup_usb("Hello over USB");
	setup_imu();
	setup_sensors();
	setup_clock();
}

// Human-readable output: temperature in degC and magnetic flux density in uT, one info frame each (time = frame header), once per second.
static void loop_print() {
	tmp_adc1.read();
	tmp_adc3.read();
	mag_adc.read();

	common::println("T01=", temp01.get_celsius(), " T02=", temp02.get_celsius(), " T03=", temp03.get_celsius(), " T04=", temp04.get_celsius(), " T05=", temp05.get_celsius(), " T06=", temp06.get_celsius(), " T07=", temp07.get_celsius(),
	    " T08=", temp08.get_celsius(), " T09=", temp09.get_celsius(), " T10=", temp10.get_celsius(), " T11=", temp11.get_celsius(), " T12=", temp12.get_celsius(), " T13=", temp13.get_celsius(), " T14=", temp14.get_celsius(),
	    " T15=", temp15.get_celsius(), " T16=", temp16.get_celsius(), " degC");

	common::println("B01=", flc01.get_tesla() * 1e6, " B02=", flc02.get_tesla() * 1e6, " B03=", flc03.get_tesla() * 1e6, " B04=", flc04.get_tesla() * 1e6, " B05=", flc05.get_tesla() * 1e6, " B06=", flc06.get_tesla() * 1e6,
	    " B07=", flc07.get_tesla() * 1e6, " B08=", flc08.get_tesla() * 1e6, " B09=", flc09.get_tesla() * 1e6, " B10=", flc10.get_tesla() * 1e6, " B11=", flc11.get_tesla() * 1e6, " B12=", flc12.get_tesla() * 1e6,
	    " B13=", flc13.get_tesla() * 1e6, " B14=", flc14.get_tesla() * 1e6, " B15=", flc15.get_tesla() * 1e6, " B16=", flc16.get_tesla() * 1e6, " uT");

	delay(1000);
}

void loop() {
	loop_frames();  // binary TemperatureMessage + MagMessage per mag sample (loop_print() is plain text for a serial monitor, the receiver skips it)
	drain_host();
}
