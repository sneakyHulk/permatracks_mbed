#pragma once

#include <Arduino.h>
#include <WireMessages.h>
#include <common_error.h>
#include <common_parser.h>
#include <wait_for.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

// Host time from the USB start of frames (STM32H7, USB2 OTG_FS), synced with SofSyncWireMessage from the host.
//
// ms     : TIM5 counts SOFs in hardware (+1 per ms, locked to the host clock).
// sub-ms : TIM5 passes every SOF on (TRGO) to TIM8, which is reset by it -> TIM8->CNT = ticks since the last SOF.
// After a sync now() = host time of that frame + (frames since that frame + time since the last SOF) * frame length,
// without a sync it counts from 0 (= ns since begin()).
struct UsbSofClock {
	std::uint64_t base_ns = 0;                   // host ns at the start of TIM5 count base_cnt
	std::uint32_t base_cnt = 0;                  // TIM5 count of the synced frame
	std::uint32_t ps_per_frame = 1'000'000'000;  // length of one USB frame in host time (ps), nominal 1 ms until synced
	std::uint32_t ticks_per_sof = 30'000;        // TIM8 ticks per USB frame (SOF to SOF), measured in begin()

	// call after the USB enumeration (SOFs are flowing)
	//
	// TIM5 is put in external clock mode 1: its counter is clocked by the internal trigger ITR8 (= USB2 OTG_FS SOF, RM0433 Table 338)
	// instead of the CPU clock, so TIM5->CNT rises once per SOF with no software help. On this core the SOF trigger is a steady
	// doublet, two pulses ~1.8 µs apart per SOF (an OTG quirk of USB2/OTG_HS2 in FS), so PSC = 1 (÷2) makes one count = one SOF = one ms.
	// TIM5->CNT rolls over after 2^32 ms ≈ 49.7 days.
	// SMCR (RM0433 "Bits 21,20,6,5,4 = TS[4:0]", "Bits 16,2,1,0 = SMS[3:0]"), the TIM2-5 TS map is non-linear (01000 = ITR4 ... 01100 = ITR8):
	//   TS  = 0b01100 = ITR8             -> TIM_SMCR_TS_3 | TIM_SMCR_TS_2
	//   SMS = 0b0111  = ext clock mode 1 -> TIM_SMCR_SMS_2 | SMS_1 | SMS_0
	// Fails without SOFs or when TIM5 does not count 1 per USB frame over 20 frames (~20 ms), e.g. the trigger is not a doublet and PSC = 1 is wrong.
	[[nodiscard]] std::expected<void, common::Error> begin() {
		constexpr std::uint32_t frames = 20;

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

		// TIM8 (16 bit) at the timer clock / 8, reset by TIM5 TRGO (ITR3, RM0433 Table 333) -> CNT = ticks since the last SOF
		__HAL_RCC_TIM8_CLK_ENABLE();
		TIM8->CR1 = 0;
		TIM8->PSC = 7;  // ÷8 -> ~30k ticks per ms, wraps after ~2.2 ms
		TIM8->ARR = 0xFFFF;
		TIM8->CNT = 0;
		TIM8->SMCR = TIM_SMCR_TS_1 | TIM_SMCR_TS_0  // TS  = 0b00011 = ITR3 (TIM5 TRGO)
		             | TIM_SMCR_SMS_2;              // SMS = 0b0100 = reset mode
		TIM8->EGR = TIM_EGR_UG;
		TIM8->CR1 |= TIM_CR1_CEN;

		// over 20 USB frames (DSTS) TIM5 must count 20, meanwhile: highest count TIM8 reaches before the SOF resets it
		std::uint32_t max_cnt = 0;
		std::uint16_t const f0 = usb_frame();
		std::uint32_t const c0 = TIM5->CNT;
		for (std::uint32_t const start = millis(); ((usb_frame() - f0) & 0x7FFu) < frames;) {
			max_cnt = std::max(max_cnt, static_cast<std::uint32_t>(TIM8->CNT));
			if (millis() - start > frames + 100) return std::unexpected(common::Error{"no USB SOF"});
		}
		if (std::uint32_t const timer_ms = TIM5->CNT - c0; timer_ms + 1 < frames || timer_ms > frames + 1) {
			return std::unexpected(common::Error{timer_ms, "TIM5 does not count 1 per USB frame, code = counts in 20 frames"});
		}

		ticks_per_sof = max_cnt;
		return {};
	}

	// host time in ns
	[[nodiscard]] std::uint64_t now() const {
		std::uint32_t const ms = TIM5->CNT;
		std::uint32_t const ticks = TIM8->CNT;
		std::uint64_t const frames_ps = static_cast<std::uint64_t>(ms - base_cnt) * ps_per_frame;
		std::uint64_t const sub_ps = static_cast<std::uint64_t>(ticks) * ps_per_frame / ticks_per_sof;
		return base_ns + (frames_ps + sub_ps) / 1000;
	}

	// waits for a SofSyncWireMessage from the host (other messages are dropped) and syncs with it:
	// host frame message.frame (11 bit) started at host time message.frame_ns, one frame lasts message.ps_per_frame in host time
	// without timeout_us it blocks until the message arrives, with it the error says that none arrived in time
	template <typename Header, common::Checksum Crc, typename... Ms>
	std::expected<void, common::Error> sync(common::Parser<Header, Crc, Ms...>& parser, std::uint32_t const timeout_us = 0) {
		auto const received = wait_for<SofSyncWireMessage>(parser, timeout_us);
		if (!received) return std::unexpected(common::Error{"no USB SOF sync from the host"});
		auto const& message = *received;

		std::uint32_t const cnt = TIM5->CNT;
		std::uint16_t const now11 = usb_frame();
		std::uint32_t const frames_since = (now11 - (message.frame & 0x7FF)) & 0x7FF;  // how long ago the host's frame was
		base_cnt = cnt - frames_since;
		base_ns = message.frame_ns;
		ps_per_frame = message.ps_per_frame;
		return {};
	}

	// 11-bit USB frame number of the last SOF (OTG_FS device status register DSTS)
	[[nodiscard]] static std::uint16_t usb_frame() {
		auto const* const device = reinterpret_cast<USB_OTG_DeviceTypeDef const*>(reinterpret_cast<std::uint32_t>(USB_OTG_FS) + USB_OTG_DEVICE_BASE);
		return (device->DSTS >> 8) & 0x7FF;
	}
};
