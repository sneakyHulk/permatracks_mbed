#pragma once

#include <Arduino.h>
#include <HalSpi4.h>
#include <common2_output.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// ADS131E08 — TI 8-channel 24-bit simultaneous-sampling delta-sigma ADC
// (datasheet SBAS561C). Driven over SPI4 (HalSpi4, mode 1, 7.5 MHz) plus three
// GPIOs: CS (active low), DRDY (input, low pulse = new frame) and START (high =
// convert). Reset via the hardware nRESET pin(s) — never the SPI RESET opcode.
//
//   ADS131E08<true>  mag_adc(spi4, PE4, PE3, PE14, PB10, PB11);  // two chips daisy-chained, 16 channels
//   ADS131E08<false> mag_adc(spi4, PE4, PE3, PE14, PB10);        // one chip, 8 channels
//
// PIN NUMBERS, NOT PinNames: PE4 / PE3 / PE14 (no underscore). On this variant
// PE_x is off by 2 from the Arduino number (PE_14 would drive PH0).
//
// DAISY CHAIN (§10.1.3.3, Fig. 56): shared CS/SCLK/DIN/START/DRDY; chip B's
// DOUT feeds chip A's DAISY_IN; chip A's DOUT is our MISO. Every register write
// reaches both chips, register reads return chip A only. The frame is
// [A: 24 status + 8 x 24 data][1 gap bit][B: 24 status + 8 x 24 data] = 433 bits
// -> 55 bytes; the gap bit misaligns B, so samples are extracted at bit level.
// Single chip: 216 bits -> 27 bytes, DAISY_IN bit set (= multiple-readback mode).
//
// Sequence (all register traffic strictly before RDATAC/START):
//   spi4.begin();  adc.begin();  adc.self_test();  adc.start();
//   loop: adc.read();  adc.volts(ch) / adc.raw(ch)
//
// Timing rules that bit us and are baked in here:
//   tSDECODE = 4 tCLK = 1.96 us between bytes of RREG/WREG, else silently ignored
//   RDATAC frame must be clocked out within one tDR (Eq. 9): 55 B @ 7.5 MHz = 59 us << 1 ms
//   tSETTLE at 1 kSPS = 9224 tCLK = 4.5 ms after START before the first valid DRDY
// -----------------------------------------------------------------------------
template <bool daisy_chain>
class ADS131E08 final {
   public:
	static constexpr std::uint8_t n_chips = daisy_chain ? 2 : 1;
	static constexpr std::uint8_t n_channels = 8 * n_chips;
	static constexpr double vref = 4.096;  // external REF5040 on VREFP (PDB_REFBUF = 0)

	// Pins are Arduino pin numbers. One chip: one nRESET. Daisy chain: nRESET of chip A and of chip B.
	ADS131E08(HalSpi4& spi, std::uint32_t const cs, std::uint32_t const drdy, std::uint32_t const start, std::uint32_t const nreset)
	requires(!daisy_chain) : spi_(spi), cs_(cs), drdy_(drdy), start_(start), nreset_a_(nreset), nreset_b_(no_pin) {}

	ADS131E08(HalSpi4& spi, std::uint32_t const cs, std::uint32_t const drdy, std::uint32_t const start, std::uint32_t const nreset_a, std::uint32_t const nreset_b)
	requires(daisy_chain) : spi_(spi), cs_(cs), drdy_(drdy), start_(start), nreset_a_(nreset_a), nreset_b_(nreset_b) {}

	// Pins, hardware reset, SDATAC, ID check, write + verify all registers.
	// Leaves the chip(s) in SDATAC with START low. spi.begin() must already be done.
	void begin() {
		pinMode(cs_, OUTPUT);
		digitalWrite(cs_, HIGH);  // deselected
		pinMode(start_, OUTPUT);
		digitalWrite(start_, LOW);  // not converting while we talk to registers
		pinMode(drdy_, INPUT);
		pinMode(nreset_a_, OUTPUT);
		digitalWrite(nreset_a_, HIGH);
		if constexpr (daisy_chain) {
			pinMode(nreset_b_, OUTPUT);
			digitalWrite(nreset_b_, HIGH);
		}

		// hardware reset: nRESET low >= 2 tCLK (§9.4.2), then 18 tCLK before the first command
		common2::print_time_loc(millis(), "'ADS131E08' do hardware reset...");
		digitalWrite(nreset_a_, LOW);
		if constexpr (daisy_chain) digitalWrite(nreset_b_, LOW);
		delay(1);
		digitalWrite(nreset_a_, HIGH);
		if constexpr (daisy_chain) digitalWrite(nreset_b_, HIGH);
		delay(1);
		common2::println("Done!");

		// the chip powers up / resets into RDATAC; registers are only reachable after SDATAC
		common2::print_time_loc(millis(), "'ADS131E08' stop continuous read mode...");
		cmd(SDATAC);
		common2::println("Done!");

		// check sensor
		initialized = check_device_id();
		if (!initialized) return;

		// configure: 1 kSPS, 24-bit, external VREF, gain 1, sensor inputs (written to every chip, read back from chip A)
		initialized = configure(config2_normal, chset_normal);
		if (!initialized) return;

		common2::println_time_loc(millis(), "'ADS131E08' Ready!");
	}

	[[nodiscard]] bool is_initialized() const { return initialized; }

	// Active self-test: route every channel to the internal DC test signal
	// (-VREF/2400 -> code -2^23/2400 = -3495 at gain 1, VREF-independent), take one
	// frame from each chip and require every channel inside a window around that.
	// Restores the normal input configuration afterwards. Call between begin() and start().
	bool self_test() {
		if (!initialized) return false;

		// route to internal test signal (all writes before RDATAC / START)
		initialized = configure(config2_test, chset_test);
		if (!initialized) return false;

		start();
		bool const synced = read();
		stop();

		bool ok = synced;
		for (std::uint8_t ch = 0; ch < n_channels; ++ch) ok = ok && raw_[ch] > test_min && raw_[ch] < test_max;
		for (std::uint8_t chip = 0; chip < n_chips; ++chip) ok = ok && status_ok(chip);
		common2::print_time_loc(millis(), "'ADS131E08' self test with internal test signal...");
		if (!ok) {
			common2::println("Abort!");
			common2::println_time_loc(millis(), "'ADS131E08' expected", test_expected, "in (", test_min, ",", test_max, "), synced", synced);
			print_frame();
		} else {
			common2::println("Done!");
		}

		// back to the sensor inputs
		initialized = configure(config2_normal, chset_normal);
		return ok && initialized;
	}

	// RDATAC + START: frames stream on DOUT at every DRDY from here on. No register
	// access until stop().
	void start() {
		common2::print_time_loc(millis(), "'ADS131E08' start conversions (RDATAC, START high)...");
		cmd(RDATAC);
		digitalWrite(start_, HIGH);
		delay(6);  // > tSETTLE (9224 tCLK = 4.5 ms at 1 kSPS)
		common2::println("Done!");
	}

	// START low halts conversions; SDATAC makes registers reachable again.
	void stop() {
		common2::print_time_loc(millis(), "'ADS131E08' stop conversions (START low, SDATAC)...");
		digitalWrite(start_, LOW);
		cmd(SDATAC);
		common2::println("Done!");
	}

	// Wait for the next DRDY falling edge, clock the whole chained frame out and
	// decode it. Returns false if DRDY never came (frame is then whatever DOUT gave).
	bool read() {
		if (!initialized) return false;

		bool const synced = wait_drdy_falling();

		std::uint8_t tx[frame_bytes] = {};
		digitalWrite(cs_, LOW);
		spi_.xfer(tx, frame_, frame_bytes);  // no opcode: RDATAC data is already on DOUT
		digitalWrite(cs_, HIGH);

		status_[0] = bits24(bit_base(0));
		raw_[0] = sample(0, 1);
		raw_[1] = sample(0, 2);
		raw_[2] = sample(0, 3);
		raw_[3] = sample(0, 4);
		raw_[4] = sample(0, 5);
		raw_[5] = sample(0, 6);
		raw_[6] = sample(0, 7);
		raw_[7] = sample(0, 8);
		if constexpr (daisy_chain) {
			status_[1] = bits24(bit_base(1));
			raw_[8] = sample(1, 1);
			raw_[9] = sample(1, 2);
			raw_[10] = sample(1, 3);
			raw_[11] = sample(1, 4);
			raw_[12] = sample(1, 5);
			raw_[13] = sample(1, 6);
			raw_[14] = sample(1, 7);
			raw_[15] = sample(1, 8);
		}
		return synced;
	}

	// Last frame. ch = 0..n_channels-1 (0..7 chip A, 8..15 chip B); chip = 0 (A) / 1 (B).
	[[nodiscard]] std::int32_t raw(std::uint8_t const ch) const { return raw_[ch]; }
	[[nodiscard]] double volts(std::uint8_t const ch) const { return raw_[ch] * vref / 8388608.0; }  // gain 1: LSB = VREF / 2^23
	[[nodiscard]] std::uint32_t status(std::uint8_t const chip) const { return status_[chip]; }
	[[nodiscard]] bool status_ok(std::uint8_t const chip) const { return (status_[chip] >> 20) == 0xC; }  // status word = 1100 | FAULT_STATP | FAULT_STATN | GPIO
	[[nodiscard]] bool data_ready() const { return digitalRead(drdy_) == LOW; }

	void print_frame() const {
		common2::println_time(millis(), "A", status_[0], status_ok(0) ? "OK" : "BAD", raw_[0], raw_[1], raw_[2], raw_[3], raw_[4], raw_[5], raw_[6], raw_[7]);
		if constexpr (daisy_chain) common2::println_time(millis(), "B", status_[1], status_ok(1) ? "OK" : "BAD", raw_[8], raw_[9], raw_[10], raw_[11], raw_[12], raw_[13], raw_[14], raw_[15]);
	}

	// Single register read (chip A). Only valid in SDATAC (between begin()/stop() and start()).
	[[nodiscard]] std::uint8_t rreg(std::uint8_t const addr) {  // [RREG|addr][count-1=0] -> data
		digitalWrite(cs_, LOW);
		byte_gap(static_cast<std::uint8_t>(RREG | addr));
		byte_gap(0x00);
		std::uint8_t const v = byte_gap(0x00);
		digitalWrite(cs_, HIGH);
		return v;
	}

   private:
	static constexpr std::uint32_t no_pin = 0xFFFF'FFFF;
	static constexpr auto retries = 5;

	// opcodes
	static constexpr std::uint8_t RDATAC = 0x10;  // read data continuous
	static constexpr std::uint8_t SDATAC = 0x11;  // stop read data continuous
	static constexpr std::uint8_t RREG = 0x20;    // | addr, then [count-1]
	static constexpr std::uint8_t WREG = 0x40;    // | addr, then [count-1], data

	// registers
	static constexpr std::uint8_t ID = 0x00;
	static constexpr std::uint8_t CONFIG1 = 0x01;
	static constexpr std::uint8_t CONFIG2 = 0x02;
	static constexpr std::uint8_t CONFIG3 = 0x03;
	static constexpr std::uint8_t CH1SET = 0x05, CH2SET = 0x06, CH3SET = 0x07, CH4SET = 0x08;
	static constexpr std::uint8_t CH5SET = 0x09, CH6SET = 0x0A, CH7SET = 0x0B, CH8SET = 0x0C;

	// ID bits 7..0: 110 (ADS131E0x) | 10 | 010 (8 channels)
	static constexpr std::uint8_t expected_id = 0b1101'0010;
	// CONFIG1 bits 7..0: 1 | DAISY_IN (0 = daisy-chain, 1 = multiple readback) | CLK_EN=0 | 1 | 0 | DR=110 (1 kSPS, 24-bit)
	static constexpr std::uint8_t config1 = daisy_chain ? 0b1001'0110 : 0b1101'0110;
	// CONFIG2 bits 7..0: 1 1 1 | INT_TEST | 0 | TEST_AMP (0: x1) | TEST_FREQ=00 / 11 (DC)
	static constexpr std::uint8_t config2_normal = 0b1110'0000;
	static constexpr std::uint8_t config2_test = 0b1111'0011;
	// CONFIG3 bits 7..0: PDB_REFBUF=0 (external VREF) | 1 | VREF_4V=0 | 0 | OPAMP_REF=0 | PDB_OPAMP=0 | 0 0
	static constexpr std::uint8_t config3 = 0b0100'0000;
	static constexpr std::uint8_t config3_mask = 0b1111'1110;  // bit 0 reads back either way
	// CHnSET bits 7..0: PD=0 | GAIN=001 (x1) | 0 | MUX=000 (normal input) / 101 (internal test signal)
	static constexpr std::uint8_t chset_normal = 0b0001'0000;
	static constexpr std::uint8_t chset_test = 0b0001'0101;

	// self-test window: -VREF/2400 -> -3495 codes at gain 1; measured offsets are a few hundred uV (~ -2000 codes)
	static constexpr std::int32_t test_expected = -8388608 / 2400;
	static constexpr std::int32_t test_min = -6500;
	static constexpr std::int32_t test_max = -1000;

	// frame geometry (24-bit mode): per chip 24 status + 8 x 24 data bits, one gap bit between chips
	static constexpr std::uint32_t bits_per_chip = 24 + 8 * 24;  // 216
	static constexpr std::uint32_t gap_bits = 1;
	static constexpr std::uint16_t frame_bytes = (n_chips * bits_per_chip + (n_chips - 1) * gap_bits + 7) / 8;  // 27 / 55

	// One byte, then tSDECODE (4 tCLK = 1.96 us): multi-byte commands need this gap between bytes.
	std::uint8_t byte_gap(std::uint8_t b) {
		std::uint8_t rx = 0;
		spi_.xfer(&b, &rx, 1);
		delayMicroseconds(3);
		return rx;
	}

	void cmd(std::uint8_t const op) {
		digitalWrite(cs_, LOW);
		byte_gap(op);
		digitalWrite(cs_, HIGH);
	}

	void wreg(std::uint8_t const addr, std::uint8_t const val) {  // [WREG|addr][count-1=0][value]
		digitalWrite(cs_, LOW);
		byte_gap(static_cast<std::uint8_t>(WREG | addr));
		byte_gap(0x00);
		byte_gap(val);
		digitalWrite(cs_, HIGH);
	}

	// Writes reach every chip on the shared DIN.
	void write_registers(std::uint8_t const config2, std::uint8_t const chset) {
		wreg(CONFIG1, config1);
		wreg(CONFIG2, config2);
		wreg(CONFIG3, config3);
		wreg(CH1SET, chset);
		wreg(CH2SET, chset);
		wreg(CH3SET, chset);
		wreg(CH4SET, chset);
		wreg(CH5SET, chset);
		wreg(CH6SET, chset);
		wreg(CH7SET, chset);
		wreg(CH8SET, chset);
	}

	// Reads come from chip A only; chip B is proven by its status word in the frame (self_test).
	[[nodiscard]] bool verify(char const* const name, std::uint8_t const addr, std::uint8_t const expected, std::uint8_t const mask = 0xFF) {
		std::uint8_t const got = rreg(addr);
		if ((got & mask) == (expected & mask)) return true;
		common2::print("Error!", name, ":", got, "/", expected, "!");
		return false;
	}

	[[nodiscard]] bool verify_registers(std::uint8_t const config2, std::uint8_t const chset) {
		bool ok = true;
		ok = verify("CONFIG1", CONFIG1, config1) && ok;
		ok = verify("CONFIG2", CONFIG2, config2) && ok;
		ok = verify("CONFIG3", CONFIG3, config3, config3_mask) && ok;
		ok = verify("CH1SET", CH1SET, chset) && ok;
		ok = verify("CH2SET", CH2SET, chset) && ok;
		ok = verify("CH3SET", CH3SET, chset) && ok;
		ok = verify("CH4SET", CH4SET, chset) && ok;
		ok = verify("CH5SET", CH5SET, chset) && ok;
		ok = verify("CH6SET", CH6SET, chset) && ok;
		ok = verify("CH7SET", CH7SET, chset) && ok;
		ok = verify("CH8SET", CH8SET, chset) && ok;
		return ok;
	}

	// Write all registers and read them back; retry like AK09940A::power_down().
	[[nodiscard]] bool configure(std::uint8_t const config2, std::uint8_t const chset) {
		common2::print_time_loc(millis(), "'ADS131E08' write and verify registers...");

		for (auto i = 0; i < retries; ++i) {
			write_registers(config2, chset);

			if (!verify_registers(config2, chset)) {
				common2::print(" Retry...");
				delay(100);
				continue;
			}

			common2::println("Done!");
			return true;
		}

		common2::println("Abort!");
		return false;
	}

	[[nodiscard]] bool check_device_id() {
		common2::print_time_loc(millis(), "'ADS131E08' check device id...");
		for (auto i = 0; i < retries; ++i) {
			std::uint8_t const id = rreg(ID);
			if (id == expected_id) {
				common2::println("Done!");
				return true;
			}
			common2::print("Error! ID:", id, "/", expected_id, "! Retry...");
			delay(100);
		}
		common2::println("Abort!");
		return false;
	}

	// DRDY sits mostly LOW between our reads with a 4 tCLK HIGH blip before each
	// update, so "wait HIGH, then wait LOW" lands right at the start of a fresh frame.
	[[nodiscard]] bool wait_drdy_falling(std::uint32_t const timeout_us = 5000) const {
		std::uint32_t const t0 = micros();
		while (digitalRead(drdy_) == LOW)
			if (micros() - t0 > timeout_us) return false;
		while (digitalRead(drdy_) == HIGH)
			if (micros() - t0 > timeout_us) return false;
		return true;
	}

	// 24 bits MSB first from an arbitrary BIT offset in the frame.
	[[nodiscard]] std::uint32_t bits24(std::uint32_t const bitpos) const {
		std::uint32_t v = 0;
		for (std::uint32_t i = 0; i < 24; ++i) {
			std::uint32_t const k = bitpos + i;
			v = (v << 1) | ((frame_[k >> 3] >> (7 - (k & 7))) & 1u);
		}
		return v;
	}

	static constexpr std::uint32_t bit_base(std::uint8_t const chip) { return chip * (bits_per_chip + gap_bits); }

	// 24-bit two's-complement sample of channel n (1..8) of a chip.
	[[nodiscard]] std::int32_t sample(std::uint8_t const chip, std::uint8_t const n) const {
		std::int32_t v = static_cast<std::int32_t>(bits24(bit_base(chip) + 24 + static_cast<std::uint32_t>(n - 1) * 24));
		if (v & 0x800000) v -= 0x1000000;  // sign-extend 24 -> 32 bit
		return v;
	}

	HalSpi4& spi_;
	std::uint32_t const cs_;
	std::uint32_t const drdy_;
	std::uint32_t const start_;
	std::uint32_t const nreset_a_;
	std::uint32_t const nreset_b_;

	bool initialized = false;  // set by begin() once reset, ID check and register verify succeed

	std::uint8_t frame_[frame_bytes] = {};
	std::uint32_t status_[n_chips] = {};
	std::int32_t raw_[n_channels] = {};
};
