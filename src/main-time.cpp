// sof_ns(): unix timestamp in ns, driven by the host's USB SOF (1 per ms).
//
// TIM5 counts SOFs in hardware (+1 per ms, locked to the host clock).
// Without sync it counts from 0 (= ns since TIM5 start).
// Frames use the 'C'/'M' protocol: marker | payload | CRC | marker, little endian.
// CRC8 = poly 0x07 (robtillaart CRC8 defaults = boost::crc_optimal<8, 0x07, 0, 0, false, false>).
//
// Host sync (13 bytes):  'S' | frame (uint16, 11-bit USB frame number) | unix_ns (uint64, host time at that frame) | CRC8  | 'S'
// Output   (11 bytes):  'T' | timestamp (uint64, unix ns) | CRC8  | 'T'   once per ms
//
// After a sync sof_ns() = unix time of that frame + (frames since that frame) * 1 ms.

#include <Arduino.h>
#include <CRC8.h>

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
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_CEN;
}

static std::uint64_t base_ns = 0;  // unix ns at TIM5 count base_cnt
static std::uint32_t base_cnt = 0;

static std::uint64_t sof_ns() { return base_ns + static_cast<std::uint64_t>(TIM5->CNT - base_cnt) * 1'000'000; }

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
}

void loop() {
	static std::uint32_t last_cnt = 0;

	poll_sync();

	if (TIM5->CNT == last_cnt) return;  // once per ms
	last_cnt = TIM5->CNT;

	send_time(sof_ns());
}
