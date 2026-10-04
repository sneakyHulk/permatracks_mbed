#pragma once

#include <Arduino.h>
#include <common_parser.h>

#include <cstdint>
#include <optional>
#include <span>
#include <variant>

// Reads Serial into parser until a message of type M arrives, other messages are dropped.
// timeout_us = 0: blocks until the message arrives, otherwise nullopt after timeout_us.
template <typename M, typename Header, common::Checksum Crc, typename... Ms>
std::optional<M> wait_for(common::Parser<Header, Crc, Ms...>& parser, std::uint32_t const timeout_us = 0) {
	for (std::uint32_t const start = micros(); timeout_us == 0 || micros() - start < timeout_us;) {
		auto const byte = Serial.read();  // non-blocking
		if (byte < 0) continue;

		std::uint8_t const data = byte;
		for (auto result = parser.parse(std::span{&data, 1}); result; result = parser.parse()) {
			if (auto const* const message = std::get_if<M>(&*result)) return *message;
		}
	}
	return std::nullopt;
}
