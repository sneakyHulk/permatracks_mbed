#pragma once

#include <Arduino.h>
#include <CRC8.h>

#include <array>
#include <charconv>
#include <cstdint>
#include <utility>

#include "common_output.h"

namespace common {
	template <std::size_t N>
	[[maybe_unused]] void message_low_level(char const (&msg)[N], CRC8& crc) {
		Serial.write(msg, N - 1);  // do not write \0
		crc.add(reinterpret_cast<std::uint8_t const*>(msg), N - 1);
	}
	template <std::size_t N>
	[[maybe_unused]] void message_low_level(char (&msg)[N], CRC8& crc) {
		Serial.write(msg, N - 1);  // do not write \0
		//common::println("HI8");
		crc.add(reinterpret_cast<std::uint8_t const*>(msg), N - 1);
	}

	[[maybe_unused]] void message_low_level(char const c, CRC8& crc) {
		Serial.write(c);
		crc.add(c);
	}

	[[maybe_unused]] void message_low_level(std::integral auto&& n, CRC8& crc) {
		std::array<char, 21> bytes;
		auto [ptr, ec] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n);
		Serial.write(bytes.data(), std::distance(bytes.data(), ptr));
		crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr));
	}
	[[maybe_unused]] void message_low_level(std::integral auto const& n, CRC8& crc) {
		std::array<char, 21> bytes;
		auto [ptr, ec] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n);
		Serial.write(bytes.data(), std::distance(bytes.data(), ptr));
		crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr));
	}

	[[maybe_unused]] void message_low_level(std::floating_point auto&& n, CRC8& crc) {
		std::array<char, 21> bytes;
		auto [ptr, ec] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n, std::chars_format::fixed, 10);
		if (ec != std::errc{}) {
			auto [ptr2, ec2] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n, std::chars_format::scientific, 10);
			Serial.write(bytes.data(), std::distance(bytes.data(), ptr2));
			crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr2));
		} else {
			Serial.write(bytes.data(), std::distance(bytes.data(), ptr));
			crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr));
		}
	}
	[[maybe_unused]] void message_low_level(std::floating_point auto const& n, CRC8& crc) {
		std::array<char, 21> bytes;
		auto [ptr, ec] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n, std::chars_format::fixed, 10);
		if (ec != std::errc{}) {
			auto [ptr2, ec2] = std::to_chars(bytes.data(), bytes.data() + bytes.size(), n, std::chars_format::scientific, 10);
			Serial.write(bytes.data(), std::distance(bytes.data(), ptr2));
			crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr2));
		} else {
			Serial.write(bytes.data(), std::distance(bytes.data(), ptr));
			crc.add(reinterpret_cast<std::uint8_t const*>(bytes.data()), std::distance(bytes.data(), ptr));
		}
	}

	template <typename... T>
	[[maybe_unused]] void message_low_level(std::tuple<T...>&& tup, CRC8& crc) {
		message_low_level('(', crc);

		[]<std::size_t... I>(std::tuple<T...>&& tup, CRC8& crc, std::index_sequence<I...>) { ((I == 0 ? void() : message_low_level(", ", crc), message_low_level(std::forward<T>(std::get<I>(tup)), crc)), ...); }(
		    std::forward<decltype(tup)>(tup), crc, std::make_index_sequence<sizeof...(T)>{});

		message_low_level(')', crc);
	}
	template <typename... T>
	[[maybe_unused]] void message_low_level(std::tuple<T...> const& tup, CRC8& crc) {
		message_low_level('(', crc);

		[]<std::size_t... I>(std::tuple<T...> const& tup, CRC8& crc, std::index_sequence<I...>) { ((I == 0 ? void() : message_low_level(", ", crc), message_low_level(std::get<I>(tup), crc)), ...); }(
		    tup, crc, std::make_index_sequence<sizeof...(T)>{});

		message_low_level(')', crc);
	}

	template <typename T, std::size_t SIZE>
	[[maybe_unused]] void message_low_level(std::array<T, SIZE>&& arr, CRC8& crc) {
		message_low_level('[', crc);

		message_low_level(arr[0], crc);
		for (auto i = 1; i < SIZE; i++) {
			message_low_level(", ", crc);
			message_low_level(arr[i], crc);
		}

		message_low_level(']', crc);
	}
	template <typename T, std::size_t SIZE>
	[[maybe_unused]] void message_low_level(std::array<T, SIZE> const& arr, CRC8& crc) {
		message_low_level('[', crc);

		message_low_level(arr[0], crc);
		for (auto i = 1; i < SIZE; i++) {
			message_low_level(", ", crc);
			message_low_level(arr[i], crc);
		}

		message_low_level(']', crc);
	}

	template <typename... Args>
	[[maybe_unused]] void message(Args&&... args) {
		Serial.write('I');
		CRC8 crc;

		(message_low_level(std::forward<Args>(args), crc), ...);

		Serial.write(crc.calc());
		Serial.write(crc.count());
		Serial.write('I');
	}

}  // namespace common