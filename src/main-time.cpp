// sof_ns(): unix timestamp in ns, driven by the host's USB SOF (1 per ms).
//
// ms     : TIM5 counts SOFs in hardware (+1 per ms, locked to the host clock).
// sub-ms : TIM5 passes every SOF on (TRGO) to TIM8, which is reset by it -> TIM8->CNT = ticks since the last SOF.
// Without sync it counts from 0 (= ns since TIM5 start).
// Frames use the 'C'/'M' protocol: marker | payload | CRC | marker, little endian.
// CRC8 = poly 0x07 (robtillaart CRC8 defaults = boost::crc_optimal<8, 0x07, 0, 0, false, false>).
//
// Host sync (13 bytes):  'S' | frame (uint16, 11-bit USB frame number) | unix_ns (uint64, host time at that frame) | CRC8  | 'S'
// Output   (11 bytes):  'T' | timestamp (uint64, unix ns) | CRC8  | 'T'   every 0.5..1.5 ms (random)
//
// After a sync sof_ns() = unix time of that frame + (frames since that frame) * 1 ms + time since the last SOF.

#include <Arduino.h>
#include <CRC8.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstring>

// Device-register view of the OTG_FS core (frame number lives in DSTS).
#define OTG_FS_DEV ((USB_OTG_DeviceTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE))

// TIM5 clocked by ITR8 (= USB2 OTG_FS SOF); PSC = 1 because the SOF trigger is a doublet -> +1 per ms.
static void sof_timer_init() {
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->CR1 = 0;
	TIM5->PSC = 1;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->CNT = 0;
	TIM5->SMCR = TIM_SMCR_TS_3 | TIM_SMCR_TS_2                        // TS  = 0b01100 = ITR8 (USB2 OTG_FS SOF)
	             | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // SMS = external clock mode 1
	TIM5->CCMR1 = (3u << TIM_CCMR1_CC1S_Pos);                         // IC1 <- TRC (the SOF trigger)
	TIM5->CCER = TIM_CCER_CC1E;                                       // capture on every SOF ...
	TIM5->CR2 = TIM_CR2_MMS_1 | TIM_CR2_MMS_0;                        // ... MMS = 011 compare pulse -> TRGO on every SOF
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_CEN;
}

// TIM8 (16 bit) at the timer clock / 8, reset by TIM5 TRGO (ITR3, RM0433 Table 333) -> CNT = ticks since the last SOF.
static void subms_timer_init() {
	__HAL_RCC_TIM8_CLK_ENABLE();
	TIM8->CR1 = 0;
	TIM8->PSC = 7;  // ÷8 -> ~30k ticks per ms, wraps after ~2.2 ms
	TIM8->ARR = 0xFFFF;
	TIM8->CNT = 0;
	TIM8->SMCR = TIM_SMCR_TS_1 | TIM_SMCR_TS_0  // TS  = 0b00011 = ITR3 (TIM5 TRGO)
	             | TIM_SMCR_SMS_2;              // SMS = 0b0100 = reset mode
	TIM8->EGR = TIM_EGR_UG;
	TIM8->CR1 |= TIM_CR1_CEN;
}

static std::uint32_t ticks_per_ms = 30'000;  // TIM8 ticks per USB frame, measured in subms_timer_calibrate()

// Highest count TIM8 reaches before the SOF resets it, over 20 SOFs (counted by TIM5).
static void subms_timer_calibrate() {
	std::uint32_t max_cnt = 0;
	for (std::uint32_t const start = TIM5->CNT; TIM5->CNT - start < 20;) max_cnt = std::max(max_cnt, static_cast<std::uint32_t>(TIM8->CNT));
	ticks_per_ms = max_cnt;
}

static std::uint64_t base_ns = 0;  // unix ns at the start of TIM5 count base_cnt
static std::uint32_t base_cnt = 0;

static std::uint64_t sof_ns() {
	std::uint32_t const ms = TIM5->CNT;
	std::uint32_t const ticks = TIM8->CNT;
	return base_ns + static_cast<std::uint64_t>(ms - base_cnt) * 1'000'000 + static_cast<std::uint64_t>(ticks) * 1'000'000 / ticks_per_ms;
}

// host frame f11 had host time unix_ns
static void sof_sync(std::uint16_t const f11, std::uint64_t const unix_ns) {
	std::uint32_t const cnt = TIM5->CNT;
	std::uint16_t const now11 = (OTG_FS_DEV->DSTS >> 8) & 0x7FF;
	std::uint32_t const frames_since = (now11 - f11) & 0x7FF;  // how long ago the host's frame was
	base_cnt = cnt - frames_since;
	base_ns = unix_ns;
}

// Receive 'S' frames: shift every byte into a 13-byte window, accept when markers and CRC match.
static void poll_sync() {
	static CRC8 crc8(0x07, 0, 0, false, false);
	static std::uint8_t msg[13];
	while (Serial.available()) {
		std::memmove(msg, msg + 1, sizeof(msg) - 1);
		msg[sizeof(msg) - 1] = static_cast<std::uint8_t>(Serial.read());
		if (msg[0] != 'S' || msg[12] != 'S') continue;

		crc8.restart();
		crc8.add(msg + 1, 10);
		if (crc8.calc() != msg[11]) continue;

		std::uint16_t f11;
		std::uint64_t unix_ns;
		std::memcpy(&f11, msg + 1, 2);
		std::memcpy(&unix_ns, msg + 3, 8);
		sof_sync(f11 & 0x7FF, unix_ns);
		msg[0] = 0;
	}
}

// 'T' | timestamp (uint64, unix ns) | CRC8 | 'T'
static void send_time(std::uint64_t const unix_ns) {
	static CRC8 crc8(0x07, 0, 0, false, false);

	auto const timestamp = std::bit_cast<std::array<std::uint8_t, sizeof(unix_ns)>>(unix_ns);
	crc8.restart();
	crc8.add(timestamp.data(), timestamp.size());

	Serial.write(static_cast<std::uint8_t>('T'));
	Serial.write(timestamp.data(), timestamp.size());
	Serial.write(crc8.calc());
	Serial.write(static_cast<std::uint8_t>('T'));
}

void setup() {
	Serial.begin();
	while (!Serial) {
	}  // wait for enumeration -> SOFs are flowing
	sof_timer_init();
	subms_timer_init();
	subms_timer_calibrate();
}

// xorshift32: cheap pseudo-random numbers for the send jitter
static std::uint32_t random_u32() {
	static std::uint32_t x = 2463534242u;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

void loop() {
	static std::uint64_t next_ns = 0;

	poll_sync();

	// send at a random point inside the ms (every 0.5..1.5 ms), so the sub-ms part (TIM8) shows up
	std::uint64_t const now = sof_ns();
	if (now < next_ns && next_ns - now < 10'000'000) return;  // (a re-sync can jump back: then send right away)
	next_ns = now + 500'000 + random_u32() % 1'000'000;

	send_time(now);
}
