#include <ADS131E08.h>
#include <Arduino.h>
#include <FLC100.h>
#include <H7Adc.h>
#include <HalSpi4.h>
#include <MCP9700B.h>
#include <MagneticFluxDensityDataRawFLC100.h>
#include <SPI.h>
#include <TemperatureDataRaw.h>
#include <WireMessages.h>
#include <common_parser.h>
#include <ntp.h>
#include <usb_sof.h>

#include <algorithm>
#include <array>
#include <boost/crc.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>
#include <variant>

using Crc = boost::crc_16_type;
using TemperatureMessage = TemperatureDataRawWireMessage<16>;
using MagMessage = MagneticFluxDensityRawWireMessage<16, MagneticFluxDensityDataRawFLC100>;

// bytes from the host: ntp time sync responses and USB SOF syncs
static common::Parser<Header, Crc, TimeSyncResponseWireMessage, SofSyncWireMessage> host_parser;

// host time (micros() + offset of the ntp time sync in setup())
static NtpClock ntp_clock;

// host-locked ms from the USB SOF (TIM5) and the time since the last SOF (TIM8), see usb_sof.h
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
	Header const header{ntp_clock.now()};

	Serial.write('I');
	for (auto const byte : std::as_bytes(std::span{&header, 1})) info.write(static_cast<std::uint8_t>(byte));
	info.size = 0;  // the header is in the checksum, but not in the length

	(info.print(args), ...);

	Serial.write(info.size);
	for (std::size_t k = 0; k < common::checksum_size<C>; ++k) Serial.write(static_cast<std::uint8_t>(info.crc.checksum() >> (8 * k)));
	Serial.write('I');
}

static bool led_state = LOW;

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

// --- Read the speed the device enumerated at ---
static const char* usb_link_speed() {
	switch (std::uint32_t const enumspd = (reinterpret_cast<USB_OTG_DeviceTypeDef*>(reinterpret_cast<std::uint32_t>(USB_OTG_FS) + USB_OTG_DEVICE_BASE)->DSTS >> 1) & 0x3) {
		case 0b11: return "Full Speed (12 Mbit/s)";  // OTG_FS is always this
		case 0b00: return "High Speed (480 Mbit/s)";
		case 0b10: return "Low Speed (1.5 Mbit/s)";
		default: return "Full Speed";
	}
}

// --- Measure TX throughput: how fast you can send to the PC ---
void usb_throughput_test(std::uint32_t const total_bytes = 262144) {  // default 256 KB
	static uint8_t buf[256];
	for (uint16_t i = 0; i < sizeof(buf); i++) buf[i] = 'A' + (i % 26);

	while (!Serial) {
	}  // wait until the host actually opens the port (DTR)
	delay(300);  // let the monitor settle

	std::uint32_t sent = 0;
	std::uint32_t t0 = micros();
	std::uint32_t guard = t0;
	while (sent < total_bytes) {
		sent += Serial.write(buf, sizeof(buf));    // blocks until the CDC buffer has room
		if (micros() - guard > 10000000UL) break;  // 10 s safety: host not draining
	}
	Serial.flush();  // wait until everything is actually pushed to the host
	std::uint32_t const t1 = micros();

	float const sec = (t1 - t0) / 1e6f;
	float const Bps = sent / sec;

	send_info<Crc>("USB throughput: link ", usb_link_speed(), ", sent ", static_cast<unsigned long>(sent), " bytes in ", sec, " s, ", Bps / 1000.0f, " kB/s (", Bps * 8.0f / 1e6f, " Mbit/s)");
	Serial.flush();
}

void setup() {
	{  // turn led on (wiring: 3V3 → resistor → LED → pin, i.e. active-low → LOW = on)
		pinMode(PC11, OUTPUT);
		digitalWrite(PC11, !led_state);  // led_state == LOW → LED on
	}

	{  // config Serial over USB
		Serial.begin();
		while (!Serial) {
		}  // wait for enumeration → USB SOFs are now flowing
		send_info<Crc>("Hello over USB");
		send_info<Crc>("=== BUILD " __DATE__ " " __TIME__ " ===");  // confirms a fresh flash is running
	}

	{  // clocks: begin both, then sync both with the host (blocking until the host answers)
		// first SOF, then ntp: the host sends its SOF sync once at the start and answers the ntp requests afterwards
		// (host side: data_collection test_permatracks_data_collection_common_parser)
		if (auto const begun = sof_clock.begin(); !begun) send_info<Crc>("UsbSofClock: ", begun.error().what(), " (", begun.error().code, ")");
		if (auto const begun = ntp_clock.begin(); !begun) send_info<Crc>("NtpClock: ", begun.error().what(), " (", begun.error().code, ")");

		if (auto const synced = sof_clock.sync(host_parser); !synced) {
			send_info<Crc>("UsbSofClock: ", synced.error().what());
		} else {
			send_info<Crc>("UsbSofClock synced, ", sof_clock.ps_per_frame, " ps per frame");
		}

		if (auto const synced = ntp_clock.sync(host_parser); !synced) {
			send_info<Crc>("NtpClock: ", synced.error().what());
		} else {
			send_info<Crc>("NtpClock synced, delay ", static_cast<unsigned long>(ntp_clock.delay / 1000), " us");
		}
	}

	tmp_adc1.begin();  // configure/calibrate ADC1 (PA/PC/PB channels) — logs its own steps
	tmp_adc3.begin();  // configure/calibrate ADC3 (PC2_C/PC3_C pads) — logs its own steps

	{  // how long do all temperature conversions take? (must stay well below the 1 ms mag frame period)
		std::uint32_t const t0 = micros();
		tmp_adc1.read();
		tmp_adc3.read();
		send_info<Crc>("'ADC1+ADC3' all 16 channels: ", micros() - t0, " us (budget: 1000 us per mag frame)");
	}

	spi4.begin();         // direct-HAL SPI4 master (kernel clock + GPIO AF5 + master init)
	mag_adc.begin();      // pins, hardware reset, SDATAC, ID check, write + verify registers (logs each step)
	mag_adc.self_test();  // both chips on the internal test signal: all 16 channels ~ -3495 + offset, status words 0xC...
	mag_adc.start();      // RDATAC + START last: both chips sample synchronously from here on
}

// Binary output: one TemperatureMessage and one MagMessage per mag sample, for common::Parser on the host.
// Timestamps are host time in ns (ntp_clock).
static void loop_frames() {
	{                     // [T][timestamp][offset][scale][16 x TemperatureDataRaw][crc16][T], T[degC] = datapoint / scale + offset
		tmp_adc1.read();  // convert + latch every enabled ADC1 channel (temp1..3, temp6..16)
		tmp_adc3.read();  // convert + latch every enabled ADC3 channel (temp4, temp5)

		TemperatureMessage message{};
		message.timestamp = ntp_clock.now();
		message.offset = MCP9700B::get_offset();
		message.scale = MCP9700B::get_scale_factor();
		message.data = {temp01.get_measurement(), temp02.get_measurement(), temp03.get_measurement(), temp04.get_measurement(), temp05.get_measurement(), temp06.get_measurement(), temp07.get_measurement(), temp08.get_measurement(),
		    temp09.get_measurement(), temp10.get_measurement(), temp11.get_measurement(), temp12.get_measurement(), temp13.get_measurement(), temp14.get_measurement(), temp15.get_measurement(), temp16.get_measurement()};

		auto const frame = common::encode<Crc>(message);
		Serial.write(frame.data(), frame.size());
	}

	{                    // [M][timestamp][scale][16 x MagneticFluxDensityDataRawFLC100][crc16][M], B[uT] = datapoint / scale
		mag_adc.read();  // wait for DRDY, latch one synchronized 55-byte frame from both ADS131E08

		MagMessage message{};
		message.timestamp = ntp_clock.now();
		message.scale = static_cast<std::int32_t>(std::lround(FLC100<ADS131E08<true>>::get_scale_factor() * 1e-6));  // LSB per uT (LSB per tesla does not fit into int32)
		message.data = {flc01.get_measurement(), flc02.get_measurement(), flc03.get_measurement(), flc04.get_measurement(), flc05.get_measurement(), flc06.get_measurement(), flc07.get_measurement(), flc08.get_measurement(),
		    flc09.get_measurement(), flc10.get_measurement(), flc11.get_measurement(), flc12.get_measurement(), flc13.get_measurement(), flc14.get_measurement(), flc15.get_measurement(), flc16.get_measurement()};

		auto const frame = common::encode<Crc>(message);
		Serial.write(frame.data(), frame.size());
	}
}

// Human-readable output: temperature in degC and magnetic flux density in uT, one info frame each (time = frame header), once per second.
static void loop_print() {
	tmp_adc1.read();
	tmp_adc3.read();
	mag_adc.read();

	send_info<Crc>("T01=", temp01.get_celsius(), " T02=", temp02.get_celsius(), " T03=", temp03.get_celsius(), " T04=", temp04.get_celsius(), " T05=", temp05.get_celsius(), " T06=", temp06.get_celsius(), " T07=", temp07.get_celsius(),
	    " T08=", temp08.get_celsius(), " T09=", temp09.get_celsius(), " T10=", temp10.get_celsius(), " T11=", temp11.get_celsius(), " T12=", temp12.get_celsius(), " T13=", temp13.get_celsius(), " T14=", temp14.get_celsius(),
	    " T15=", temp15.get_celsius(), " T16=", temp16.get_celsius(), " degC");

	send_info<Crc>("B01=", flc01.get_tesla() * 1e6, " B02=", flc02.get_tesla() * 1e6, " B03=", flc03.get_tesla() * 1e6, " B04=", flc04.get_tesla() * 1e6, " B05=", flc05.get_tesla() * 1e6, " B06=", flc06.get_tesla() * 1e6,
	    " B07=", flc07.get_tesla() * 1e6, " B08=", flc08.get_tesla() * 1e6, " B09=", flc09.get_tesla() * 1e6, " B10=", flc10.get_tesla() * 1e6, " B11=", flc11.get_tesla() * 1e6, " B12=", flc12.get_tesla() * 1e6,
	    " B13=", flc13.get_tesla() * 1e6, " B14=", flc14.get_tesla() * 1e6, " B15=", flc15.get_tesla() * 1e6, " B16=", flc16.get_tesla() * 1e6, " uT");

	delay(1000);
}

// both clocks read right after each other, the host compares them with its arrival time
static void send_time_compare() {
	TimeCompareWireMessage message{};
	message.ntp_ns = ntp_clock.now();
	message.sof_ns = sof_clock.now();

	auto const frame = common::encode<Crc>(message);
	Serial.write(frame.data(), frame.size());
}

void loop() {
	loop_frames();
	send_time_compare();
}
