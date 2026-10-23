#pragma once

#include <Arduino.h>
#include <WireMessages.h>
#include <common_error.h>
#include <common_parser.h>
#include <usb_sof_frames.h>
#include <wait_for.h>

#include <cstdint>
#include <expected>

// Host time from the USB start of frames (STM32H7, USB2 OTG_FS) with ms resolution: like UsbSofClock, but without the
// sub-ms timer (TIM8). now() is the host time at the start of the current USB frame, so it steps once per ms.
//
// ms : exact USB frames since the synced frame, TIM5 (counts SOFs in hardware) snapped to the USB frame number (DSTS), see usb_sof_frames.h.
// After a sync now() = host time of that frame + frames since that frame * frame length,
// without a sync it counts from 0 (= ns since begin()).
//
// Shares TIM5 with UsbSofClock: begin() only sets the counting part of TIM5 (not its capture / TRGO for TIM8),
// so both can run at once. begin() resets TIM5->CNT, so begin() both first, then sync both.
struct UsbSofMsClock {
	std::uint64_t base_ns = 0;                   // host ns at the start of the anchor frame of `frames`
	UsbSofFrames frames;                         // exact frames since the synced frame (TIM5 + DSTS)
	std::uint32_t ps_per_frame = 1'000'000'000;  // length of one USB frame in host time (ps), nominal 1 ms until synced

	// call after the USB enumeration (SOFs are flowing)
	//
	// TIM5 in external clock mode 1, clocked by ITR8 (= USB2 OTG_FS SOF, RM0433 Table 338): TIM5->CNT rises once per SOF.
	// The SOF trigger is a doublet (two pulses ~1.8 µs apart), so PSC = 1 (÷2) makes one count = one SOF = one ms.
	// TIM5->CNT rolls over after 2^32 ms ≈ 49.7 days.
	//   TS  = 0b01100 = ITR8             -> TIM_SMCR_TS_3 | TIM_SMCR_TS_2
	//   SMS = 0b0111  = ext clock mode 1 -> TIM_SMCR_SMS_2 | SMS_1 | SMS_0
	// Fails without SOFs or when TIM5 does not count 1 per USB frame over 20 frames (~20 ms).
	[[nodiscard]] std::expected<void, common::Error> begin() {
		constexpr std::uint32_t check_frames = 20;

		__HAL_RCC_TIM5_CLK_ENABLE();
		TIM5->CR1 = 0;
		TIM5->PSC = 1;
		TIM5->ARR = 0xFFFFFFFF;
		TIM5->CNT = 0;
		TIM5->SMCR = TIM_SMCR_TS_3 | TIM_SMCR_TS_2                        // TS  = 0b01100 = ITR8 (USB2 OTG_FS SOF)
		             | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // SMS = external clock mode 1
		TIM5->EGR = TIM_EGR_UG;
		TIM5->CR1 |= TIM_CR1_CEN;

		// over 20 USB frames (DSTS) TIM5 must count 20
		std::uint16_t const f0 = usb_frame();
		std::uint32_t const c0 = TIM5->CNT;
		for (std::uint32_t const start = millis(); ((usb_frame() - f0) & 0x7FFu) < check_frames;) {
			if (millis() - start > check_frames + 100) return std::unexpected(common::Error{"no USB SOF"});
		}
		if (std::uint32_t const timer_ms = TIM5->CNT - c0; timer_ms + 1 < check_frames || timer_ms > check_frames + 1) {
			return std::unexpected(common::Error{timer_ms, "TIM5 does not count 1 per USB frame, code = counts in 20 frames"});
		}
		frames.anchor(usb_frame());  // without a sync: count from now
		return {};
	}

	// host time in ns at the start of the current USB frame
	[[nodiscard]] std::uint64_t now() const { return base_ns + static_cast<std::uint64_t>(frames.since_anchor()) * ps_per_frame / 1000; }

	// waits for a SofSyncWireMessage from the host (other messages are dropped) and syncs with it
	// without timeout_us it blocks until the message arrives, with it the error says that none arrived in time
	template <typename Header, common::Checksum Crc, typename... Ms>
	std::expected<void, common::Error> sync(common::Parser<Header, Crc, Ms...>& parser, std::uint32_t const timeout_us = 0) {
		auto const received = wait_for<SofSyncWireMessage>(parser, timeout_us);
		if (!received) return std::unexpected(common::Error{"no USB SOF sync from the host"});
		sync(*received);
		return {};
	}

	// host frame message.frame (11 bit) started at host time message.frame_ns, one frame lasts message.ps_per_frame in host time
	void sync(SofSyncWireMessage const& message) {
		frames.anchor(message.frame);  // blocks up to ~1 ms, see UsbSofFrames::same_frame()
		base_ns = message.frame_ns;
		ps_per_frame = message.ps_per_frame;
	}

	// 11-bit USB frame number of the last SOF (OTG_FS device status register DSTS)
	[[nodiscard]] static std::uint16_t usb_frame() { return UsbSofFrames::usb_frame(); }
};
