// Time test of mag-array-v3 under real conditions: the measuring firmware's loop (sensors, loop_frames() paced by the
// ADS131E08, drain_host() with SOF sync and pings, the same as in main-mag-array-v3.cpp, keep both in step) plus
//   - a TimeCompareWireMessage per loop: time1, time2 = two sof_clock.now() right after each other (time2 - time1 >= 0: never backwards)
//   - at the start: how long now() takes, every 10 s: TIM5 against the USB frame number (missed SOFs)
// Host side: data_collection test_permatracks_data_collection_time.
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
static ADS131E08<true> mag_adc(spi4, PE4, PE3, PE14, PB10, PB11);  // (spi, CS, DRDY, START, nRESET A, nRESET B), 2 kSPS, 24-bit, VREF=4.096 V

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

	{  // how long do all temperature conversions take? (must stay well below the 500 us mag frame period)
		std::uint64_t const t0 = sof_clock.now();
		tmp_adc1.read();
		tmp_adc3.read();
		send_info<Crc>("'ADC1+ADC3' all 16 channels: ", static_cast<unsigned long>((sof_clock.now() - t0) / 1000), " us (budget: 500 us per mag frame)");
	}

	spi4.begin();         // direct-HAL SPI4 master (kernel clock + GPIO AF5 + master init)
	mag_adc.begin();      // pins, hardware reset, SDATAC, ID check, write + verify registers (logs each step)
	mag_adc.self_test();  // both chips on the internal test signal: all 16 channels ~ -3495 + offset, status words 0xC...
	mag_adc.start();      // RDATAC + START last: both chips sample synchronously from here on
}

// Binary output: one TemperatureMessage and one MagMessage per mag sample, for common::Parser on the host.
// Paced by the ADS131E08 (mag_adc.read() waits for its DRDY, 2 kSPS on its own oscillator): one call ≈ 500 us.
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

// cost of now(): n calls, each timed with the CPU cycle counter (DWT), min / mean / max in ns.
// The max includes everything a call can meet: the wait of UsbSofClock near a SOF and interrupts (USB) in between.
static void benchmark_now() {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // DWT on (Arduino core enables it already for delayMicroseconds, harmless)
	DWT->LAR = 0xC5ACCE55;                           // unlock (Cortex-M7)
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	constexpr std::uint32_t n = 200'000;
	std::uint32_t min = ~0u, max = 0;
	std::uint64_t sum = 0;
	volatile std::uint64_t sink = 0;  // the result must be used, else the call could be optimized away
	for (std::uint32_t i = 0; i < n; ++i) {
		std::uint32_t const c0 = DWT->CYCCNT;
		sink = sof_clock.now();
		std::uint32_t const cycles = DWT->CYCCNT - c0;
		min = std::min(min, cycles);
		max = std::max(max, cycles);
		sum += cycles;
	}
	static_cast<void>(sink);
	auto const ns = [](std::uint64_t const cycles) { return static_cast<unsigned long>(cycles * 1'000'000'000ULL / SystemCoreClock); };
	send_info<Crc>("UsbSofClock now(), ", static_cast<unsigned long>(n), " calls: min/mean/max ", ns(min), "/", ns(sum / n), "/", ns(max), " ns (", static_cast<unsigned long>(min), "/", static_cast<unsigned long>(sum / n), "/",
	    static_cast<unsigned long>(max), " cycles at ", static_cast<unsigned long>(SystemCoreClock / 1'000'000), " MHz)");
}

// two sof_clock.now() right after each other: the host compares time1 with its arrival time, time2 - time1 must never be negative
static void send_time_compare() {
	TimeCompareWireMessage message{};
	message.time1_ns = sof_clock.now();
	message.time2_ns = sof_clock.now();

	message.timestamp = sof_clock.now();  // send time, host time
	auto const frame = common::encode<Crc>(message);
	Serial.write(frame.data(), frame.size());
}

// every 10 s (host time): how often TIM5 missed or double counted a SOF against the USB frame number (missed SOFs)
static void report_sof_health() {
	static std::uint64_t next_ns = sof_clock.now() + 10'000'000'000ULL;
	if (sof_clock.now() < next_ns) return;
	next_ns += 10'000'000'000ULL;
	send_info<Crc>("UsbSofClock: TIM5 vs USB frame number since the sync: ", sof_clock.frames.corrected, " counts corrected, now ", sof_clock.frames.off, " off");
}

void setup() {
	setup_usb("Hello over USB (time test)");
	setup_clock();
	benchmark_now();  // before the sensors start, so it measures now() alone
	setup_sensors();
}

void loop() {
	loop_frames();  // as in the measuring firmware: ~500 us, paced by the ADS131E08
	send_time_compare();
	drain_host();
	report_sof_health();
}
