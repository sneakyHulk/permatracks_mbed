// sof_micros(): unix timestamp in µs, driven by the host's USB SOF (1 per ms).
//
// TIM5 counts SOFs in hardware (+1 per ms, locked to the host clock).
// Without sync it counts from 0 (= µs since TIM5 start).
// Host sync message (binary, little endian, 12 bytes):
//
//   'S' | frame (uint16, 11-bit USB frame number) | unix_us (uint64, host time at that frame) | 'S'
//
// After that sof_micros() = unix_us + (frames since that frame) * 1000.
// Output: sof_micros() once per ms as ISO 8601 UTC text, e.g. 2026-10-01T12:34:56.789000Z

#include <Arduino.h>

#include <cstdint>
#include <cstring>
#include <ctime>

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

static std::uint64_t base_us = 0;   // unix µs at TIM5 count base_cnt
static std::uint32_t base_cnt = 0;

static std::uint64_t sof_millis() { return base_us + static_cast<std::uint64_t>(TIM5->CNT - base_cnt) * 1000; }

// host frame f11 had host time unix_us
static void sof_sync(std::uint16_t const f11, std::uint64_t const unix_us) {
	std::uint32_t const cnt = TIM5->CNT;
	std::uint16_t const now11 = (OTG_FS_DEV->DSTS >> 8) & 0x7FF;
	std::uint32_t const frames_since = (now11 - f11) & 0x7FF;  // how long ago the host's frame was
	base_cnt = cnt - frames_since;
	base_us = unix_us;
}

static void poll_sync() {
	static std::uint8_t msg[12];
	while (Serial.available()) {
		std::memmove(msg, msg + 1, sizeof(msg) - 1);
		msg[sizeof(msg) - 1] = static_cast<std::uint8_t>(Serial.read());
		if (msg[0] == 'S' && msg[11] == 'S') {
			std::uint16_t f11;
			std::uint64_t unix_us;
			std::memcpy(&f11, msg + 1, 2);
			std::memcpy(&unix_us, msg + 3, 8);
			sof_sync(f11 & 0x7FF, unix_us);
			msg[0] = 0;
		}
	}
}

// Print a unix timestamp (µs) as ISO 8601 UTC, e.g. 2026-10-01T12:34:56.789000Z
static void print_iso(std::uint64_t const unix_us) {
	std::time_t const sec = static_cast<std::time_t>(unix_us / 1'000'000);
	std::tm tm{};
	gmtime_r(&sec, &tm);
	char s[32];
	snprintf(s, sizeof(s), "%04d-%02d-%02dT%02d:%02d:%02d.%06luZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<unsigned long>(unix_us % 1'000'000));
	Serial.print(s);
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

	print_iso(sof_millis());
}
