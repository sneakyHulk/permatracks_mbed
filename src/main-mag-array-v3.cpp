#include <ADS131E08.h>
#include <Arduino.h>
#include <CRC16.h>
#include <FLC100.h>
#include <H7Adc.h>
#include <HalSpi4.h>
#include <MCP9700B.h>
#include <MagneticFluxDensityDataRawFLC100.h>
#include <SPI.h>
#include <TemperatureDatapointRaw.h>
#include <common2_output.h>

#include <array>
#include <bit>
#include <cstdint>

static bool led_state = LOW;

// Device-register view of the OTG_FS core (frame number lives in DSTS).
#define OTG_FS_DEV ((USB_OTG_DeviceTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE))

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

	char msg[160];
	snprintf(msg, sizeof(msg),
	    "\n--- USB throughput ---\n"
	    "Link      : %s\n"
	    "Sent      : %lu bytes in %.3f s\n"
	    "Throughput: %.1f kB/s  (%.2f Mbit/s)\n",
	    usb_link_speed(), static_cast<unsigned long>(sent), sec, Bps / 1000.0f, Bps * 8.0f / 1e6f);
	Serial.print(msg);
	Serial.flush();
}

// -----------------------------------------------------------------------------
// STEP 1: a self-running MILLISECOND counter driven by the USB SOF.
//
// TIM5 is put in EXTERNAL CLOCK MODE 1: its counter is clocked by the internal
// trigger ITR8 (= USB2 OTG_FS SOF, per RM0433 Table 338) instead of the CPU
// clock. So the SOF itself fills the timer — TIM5->CNT rises once per SOF with
// no software help. On this core the SOF trigger is a steady DOUBLET — two
// pulses ~1.8µs apart per SOF (measured; an OTG quirk of USB2/OTG_HS2-in-FS) —
// so PSC = 1 (÷2) makes one count = one SOF = one ms. Result: TIM5->CNT is a
// host-locked millisecond count you can read at ANY time, rolling over only
// after 2^32 ms ≈ 49.7 days.
//
// SMCR fields (RM0433 "Bits 21,20,6,5,4 = TS[4:0]"; "Bits 16,2,1,0 = SMS[3:0]").
// NOTE: the TIM2-5 TS map is non-linear (01000=ITR4 ... 01100=ITR8), so:
//   TS  = 0b01100 = ITR8             -> TIM_SMCR_TS_3 | TIM_SMCR_TS_2   (8 + 4)
//   SMS = 0b0111  = ext clock mode 1 -> TIM_SMCR_SMS_2 | SMS_1 | SMS_0
// -----------------------------------------------------------------------------
static void sof_timer_init() {
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->CR1 = 0;
	TIM5->PSC = 1;           // ÷2: confirmed DOUBLET (2 pulses/SOF, ~1.8µs apart) → +1 count per SOF = 1 ms
	TIM5->ARR = 0xFFFFFFFF;  // full 32-bit range
	TIM5->CNT = 0;

	TIM5->SMCR = TIM_SMCR_TS_3 | TIM_SMCR_TS_2                        // TS  = 0b01100 = ITR8 (USB2 OTG_FS SOF)
	             | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // SMS = external clock mode 1

	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_CEN;
}

// Milliseconds since start, straight from the SOF-driven timer. Autonomous:
// correct whenever you read it, no matter how rarely you call it.
static std::uint32_t sof_ms() { return TIM5->CNT; }

// One-shot CHECK: does the SOF-timer advance 1:1 with the DSTS frame counter?
// (DSTS is the raw 11-bit USB frame number = a clean 1 kHz reference.)
static void sof_timer_check() {
	std::uint16_t const f0 = (OTG_FS_DEV->DSTS >> 8) & 0x7FF;
	std::uint32_t const c0 = TIM5->CNT;
	delay(200);  // ~200 SOFs (SysTick delay — this is the CHECK, not the clock)
	std::uint16_t const f1 = (OTG_FS_DEV->DSTS >> 8) & 0x7FF;
	std::uint32_t const c1 = TIM5->CNT;

	std::uint32_t const dframe = (f1 - f0) & 0x7FFu;  // DSTS frames elapsed
	std::uint32_t const dcnt = c1 - c0;               // SOF-timer ms elapsed

	char m[112];
	snprintf(m, sizeof(m), "sof_timer check: %lu timer-ms vs %lu DSTS-frames in ~200ms (want equal)\n", static_cast<unsigned long>(dcnt), static_cast<unsigned long>(dframe));
	Serial.print(m);
}

// DIAGNOSTIC: dump the gaps between consecutive ITR8/SOF trigger events, in
// timer ticks. Reveals the real pattern (clean 1x, doublet, even 2x, or jitter).
// 1 ms ~= 238205 ticks at the CPU clock.
[[maybe_unused]] static void sof_interval_dump() {
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->CR1 = 0;
	TIM5->PSC = 0;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->SMCR = TIM_SMCR_TS_3 | TIM_SMCR_TS_2;  // TS = ITR8, SMS = 0 (input capture, free-run counter)
	TIM5->CCMR1 = (3u << TIM_CCMR1_CC1S_Pos);    // IC1 <- TRC (the trigger)
	TIM5->CCER = TIM_CCER_CC1E;                  // enable capture
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 = TIM_CR1_CEN;

	Serial.println("--- SOF trigger intervals (ticks between consecutive events; 1ms~=238205) ---");
	std::uint32_t prev = TIM5->CCR1;
	for (int i = 0; i < 40; ++i) {
		std::uint32_t g = 0;
		while (TIM5->CCR1 == prev && ++g < 50000000u) {
		}  // wait for the next capture
		std::uint32_t const cur = TIM5->CCR1;
		char m[40];
		snprintf(m, sizeof(m), "%lu\n", static_cast<unsigned long>(cur - prev));
		Serial.print(m);
		prev = cur;
	}
	Serial.println("--- done ---");
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
		Serial.println("Hello over USB");
		Serial.println("=== BUILD " __DATE__ " " __TIME__ " ===");  // confirms a fresh flash is running
	}

	// sof_interval_dump();  // DIAGNOSTIC (confirmed a steady doublet → PSC=1)
	sof_timer_init();   // SOF fills TIM5 (÷2 for the doublet) → CNT = ms
	sof_timer_check();  // verify CNT advances 1:1 with the DSTS frame

	tmp_adc1.begin();  // configure/calibrate ADC1 (PA/PC/PB channels) — logs its own steps
	tmp_adc3.begin();  // configure/calibrate ADC3 (PC2_C/PC3_C pads) — logs its own steps

	{  // how long do all temperature conversions take? (must stay well below the 1 ms mag frame period)
		std::uint32_t const t0 = micros();
		tmp_adc1.read();
		tmp_adc3.read();
		common2::println_time_loc(millis(), "'ADC1+ADC3' all 16 channels:", micros() - t0, "us (budget: 1000 us per mag frame)");
	}

	spi4.begin();         // direct-HAL SPI4 master (kernel clock + GPIO AF5 + master init)
	mag_adc.begin();      // pins, hardware reset, SDATAC, ID check, write + verify registers (logs each step)
	mag_adc.self_test();  // both chips on the internal test signal: all 16 channels ~ -3495 + offset, status words 0xC...
	mag_adc.start();      // RDATAC + START last: both chips sample synchronously from here on
}

// Binary output: one 'C' (temperature) and one 'M' (magnetic) CRC16 frame per mag sample, for the parser.
static void loop_frames() {
	std::uint32_t const ms = sof_ms();  // SOF-driven, host-locked millisecond timestamp

	{  // 'C' frame (Celsius; 'T' is the time sync marker): 'C' | scale (8 B) | offset (8 B) | 16 x TemperatureDatapointRaw (2 B) | timestamp (8 B, ns) | CRC16 (2 B) | 'C'
		static CRC16 crc16(0x8005, 0, false, true, true);

		tmp_adc1.read();  // convert + latch every enabled ADC1 channel (temp1..3, temp6..16)
		tmp_adc3.read();  // convert + latch every enabled ADC3 channel (temp4, temp5)

		auto const tmp01 = temp01.get_measurement();
		auto const tmp02 = temp02.get_measurement();
		auto const tmp03 = temp03.get_measurement();
		auto const tmp04 = temp04.get_measurement();
		auto const tmp05 = temp05.get_measurement();
		auto const tmp06 = temp06.get_measurement();
		auto const tmp07 = temp07.get_measurement();
		auto const tmp08 = temp08.get_measurement();
		auto const tmp09 = temp09.get_measurement();
		auto const tmp10 = temp10.get_measurement();
		auto const tmp11 = temp11.get_measurement();
		auto const tmp12 = temp12.get_measurement();
		auto const tmp13 = temp13.get_measurement();
		auto const tmp14 = temp14.get_measurement();
		auto const tmp15 = temp15.get_measurement();
		auto const tmp16 = temp16.get_measurement();

		Serial.write(static_cast<std::uint8_t>('C'));

		auto const scale_mcp = std::bit_cast<std::array<std::uint8_t, sizeof(double)>>(MCP9700B::get_scale_factor());  // LSB per degC, double
		Serial.write(scale_mcp.data(), scale_mcp.size());
		crc16.add(scale_mcp.data(), scale_mcp.size());

		auto const offset_mcp = std::bit_cast<std::array<std::uint8_t, sizeof(double)>>(MCP9700B::get_offset());  // degC, double
		Serial.write(offset_mcp.data(), offset_mcp.size());
		crc16.add(offset_mcp.data(), offset_mcp.size());

		// clang-format off
		Serial.write(tmp01.bytes.data(), tmp01.bytes.size()); crc16.add(tmp01.bytes.data(), tmp01.bytes.size());
		Serial.write(tmp02.bytes.data(), tmp02.bytes.size()); crc16.add(tmp02.bytes.data(), tmp02.bytes.size());
		Serial.write(tmp03.bytes.data(), tmp03.bytes.size()); crc16.add(tmp03.bytes.data(), tmp03.bytes.size());
		Serial.write(tmp04.bytes.data(), tmp04.bytes.size()); crc16.add(tmp04.bytes.data(), tmp04.bytes.size());
		Serial.write(tmp05.bytes.data(), tmp05.bytes.size()); crc16.add(tmp05.bytes.data(), tmp05.bytes.size());
		Serial.write(tmp06.bytes.data(), tmp06.bytes.size()); crc16.add(tmp06.bytes.data(), tmp06.bytes.size());
		Serial.write(tmp07.bytes.data(), tmp07.bytes.size()); crc16.add(tmp07.bytes.data(), tmp07.bytes.size());
		Serial.write(tmp08.bytes.data(), tmp08.bytes.size()); crc16.add(tmp08.bytes.data(), tmp08.bytes.size());
		Serial.write(tmp09.bytes.data(), tmp09.bytes.size()); crc16.add(tmp09.bytes.data(), tmp09.bytes.size());
		Serial.write(tmp10.bytes.data(), tmp10.bytes.size()); crc16.add(tmp10.bytes.data(), tmp10.bytes.size());
		Serial.write(tmp11.bytes.data(), tmp11.bytes.size()); crc16.add(tmp11.bytes.data(), tmp11.bytes.size());
		Serial.write(tmp12.bytes.data(), tmp12.bytes.size()); crc16.add(tmp12.bytes.data(), tmp12.bytes.size());
		Serial.write(tmp13.bytes.data(), tmp13.bytes.size()); crc16.add(tmp13.bytes.data(), tmp13.bytes.size());
		Serial.write(tmp14.bytes.data(), tmp14.bytes.size()); crc16.add(tmp14.bytes.data(), tmp14.bytes.size());
		Serial.write(tmp15.bytes.data(), tmp15.bytes.size()); crc16.add(tmp15.bytes.data(), tmp15.bytes.size());
		Serial.write(tmp16.bytes.data(), tmp16.bytes.size()); crc16.add(tmp16.bytes.data(), tmp16.bytes.size());
		// clang-format on

		std::uint64_t const timestamp = 1'000'000ULL * ms;  // SOF ms -> ns
		auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
		Serial.write(timestamp_.data(), timestamp_.size());
		crc16.add(timestamp_.data(), timestamp_.size());

		auto const crc_value = std::bit_cast<std::array<std::uint8_t, 2>>(crc16.calc());
		Serial.write(crc_value.data(), crc_value.size());

		Serial.write(static_cast<std::uint8_t>('C'));

		crc16.restart();
	}

	{  // 'M' frame, same layout as mag-array-v2: 'M' | scale | 16 x MagneticFluxDensityDatapointRaw (3 B) | timestamp (8 B, ns) | CRC16 (2 B) | 'M'
		static CRC16 crc16(0x8005, 0, false, true, true);

		mag_adc.read();  // wait for DRDY, latch one synchronized 55-byte frame from both ADS131E08

		auto const mag01 = flc01.get_measurement();
		auto const mag02 = flc02.get_measurement();
		auto const mag03 = flc03.get_measurement();
		auto const mag04 = flc04.get_measurement();
		auto const mag05 = flc05.get_measurement();
		auto const mag06 = flc06.get_measurement();
		auto const mag07 = flc07.get_measurement();
		auto const mag08 = flc08.get_measurement();
		auto const mag09 = flc09.get_measurement();
		auto const mag10 = flc10.get_measurement();
		auto const mag11 = flc11.get_measurement();
		auto const mag12 = flc12.get_measurement();
		auto const mag13 = flc13.get_measurement();
		auto const mag14 = flc14.get_measurement();
		auto const mag15 = flc15.get_measurement();
		auto const mag16 = flc16.get_measurement();

		Serial.write(static_cast<std::uint8_t>('M'));

		auto const scale_flc = std::bit_cast<std::array<std::uint8_t, sizeof(double)>>(FLC100<ADS131E08<true>>::get_scale_factor());  // LSB per tesla, double
		Serial.write(scale_flc.data(), scale_flc.size());
		crc16.add(scale_flc.data(), scale_flc.size());

		// clang-format off
		Serial.write(mag01.bytes.data(), mag01.bytes.size()); crc16.add(mag01.bytes.data(), mag01.bytes.size());
		Serial.write(mag02.bytes.data(), mag02.bytes.size()); crc16.add(mag02.bytes.data(), mag02.bytes.size());
		Serial.write(mag03.bytes.data(), mag03.bytes.size()); crc16.add(mag03.bytes.data(), mag03.bytes.size());
		Serial.write(mag04.bytes.data(), mag04.bytes.size()); crc16.add(mag04.bytes.data(), mag04.bytes.size());
		Serial.write(mag05.bytes.data(), mag05.bytes.size()); crc16.add(mag05.bytes.data(), mag05.bytes.size());
		Serial.write(mag06.bytes.data(), mag06.bytes.size()); crc16.add(mag06.bytes.data(), mag06.bytes.size());
		Serial.write(mag07.bytes.data(), mag07.bytes.size()); crc16.add(mag07.bytes.data(), mag07.bytes.size());
		Serial.write(mag08.bytes.data(), mag08.bytes.size()); crc16.add(mag08.bytes.data(), mag08.bytes.size());
		Serial.write(mag09.bytes.data(), mag09.bytes.size()); crc16.add(mag09.bytes.data(), mag09.bytes.size());
		Serial.write(mag10.bytes.data(), mag10.bytes.size()); crc16.add(mag10.bytes.data(), mag10.bytes.size());
		Serial.write(mag11.bytes.data(), mag11.bytes.size()); crc16.add(mag11.bytes.data(), mag11.bytes.size());
		Serial.write(mag12.bytes.data(), mag12.bytes.size()); crc16.add(mag12.bytes.data(), mag12.bytes.size());
		Serial.write(mag13.bytes.data(), mag13.bytes.size()); crc16.add(mag13.bytes.data(), mag13.bytes.size());
		Serial.write(mag14.bytes.data(), mag14.bytes.size()); crc16.add(mag14.bytes.data(), mag14.bytes.size());
		Serial.write(mag15.bytes.data(), mag15.bytes.size()); crc16.add(mag15.bytes.data(), mag15.bytes.size());
		Serial.write(mag16.bytes.data(), mag16.bytes.size()); crc16.add(mag16.bytes.data(), mag16.bytes.size());
		// clang-format on

		std::uint64_t const timestamp = 1'000'000ULL * ms;  // SOF ms -> ns, same 8-byte field as v2
		auto const timestamp_ = std::bit_cast<std::array<std::uint8_t, sizeof(timestamp)>>(timestamp);
		Serial.write(timestamp_.data(), timestamp_.size());
		crc16.add(timestamp_.data(), timestamp_.size());

		auto const crc_value = std::bit_cast<std::array<std::uint8_t, 2>>(crc16.calc());
		Serial.write(crc_value.data(), crc_value.size());

		Serial.write(static_cast<std::uint8_t>('M'));

		crc16.restart();
	}
}

// Human-readable output: temperature in degC and magnetic flux density in uT, one line each, once per second.
static void loop_print() {
	std::uint32_t const ms = sof_ms();

	tmp_adc1.read();
	tmp_adc3.read();
	mag_adc.read();

	char line[320];
	snprintf(line, sizeof(line),
	    "t=%lu.%03lu T01=%.1f T02=%.1f T03=%.1f T04=%.1f T05=%.1f T06=%.1f T07=%.1f T08=%.1f "
	    "T09=%.1f T10=%.1f T11=%.1f T12=%.1f T13=%.1f T14=%.1f T15=%.1f T16=%.1f degC\n",
	    static_cast<unsigned long>(ms / 1000), static_cast<unsigned long>(ms % 1000), temp01.get_celsius(), temp02.get_celsius(), temp03.get_celsius(), temp04.get_celsius(), temp05.get_celsius(), temp06.get_celsius(), temp07.get_celsius(),
	    temp08.get_celsius(), temp09.get_celsius(), temp10.get_celsius(), temp11.get_celsius(), temp12.get_celsius(), temp13.get_celsius(), temp14.get_celsius(), temp15.get_celsius(), temp16.get_celsius());
	Serial.print(line);

	char mag_line[420];
	snprintf(mag_line, sizeof(mag_line),
	    "t=%lu.%03lu B01=%.3f B02=%.3f B03=%.3f B04=%.3f B05=%.3f B06=%.3f B07=%.3f B08=%.3f "
	    "B09=%.3f B10=%.3f B11=%.3f B12=%.3f B13=%.3f B14=%.3f B15=%.3f B16=%.3f uT\n",
	    static_cast<unsigned long>(ms / 1000), static_cast<unsigned long>(ms % 1000), flc01.get_tesla() * 1e6, flc02.get_tesla() * 1e6, flc03.get_tesla() * 1e6, flc04.get_tesla() * 1e6, flc05.get_tesla() * 1e6, flc06.get_tesla() * 1e6,
	    flc07.get_tesla() * 1e6, flc08.get_tesla() * 1e6, flc09.get_tesla() * 1e6, flc10.get_tesla() * 1e6, flc11.get_tesla() * 1e6, flc12.get_tesla() * 1e6, flc13.get_tesla() * 1e6, flc14.get_tesla() * 1e6, flc15.get_tesla() * 1e6,
	    flc16.get_tesla() * 1e6);
	Serial.print(mag_line);

	delay(1000);
}

void loop() { loop_print(); }
