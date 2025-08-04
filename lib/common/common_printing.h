#pragma once

#include <array>
#include <tuple>

namespace common {
	void print_low_level(std::integral auto const value) { Serial.print(value); }
	inline void print_low_level(char const value) { Serial.print(value); }
	inline void print_low_level(char const* const value) { Serial.print(value); }
	inline void print_low_level(double const value) { Serial.print(value); }
	inline void print_low_level(float const value) { Serial.print(value); }
	inline void println_low_level() { Serial.println(); }

	// make std::array printable
	template <typename T, std::size_t SIZE>
	[[maybe_unused]] void print_low_level(std::array<T, SIZE> const& arr) {
		print_low_level('(');

#if __cplusplus < 202002L
		char space[]{0, 0, 0};
		for (auto const& e : arr) {
#else
		for (char space[]{0, 0, 0}; auto const& e : arr) {
#endif
			print_low_level(space);
			print_low_level(e);
			*space = ',';  //,*(space + 1) = ' '; // dont need spaces
		}

		print_low_level(')');
	}
	template <typename T, std::size_t SIZE>
	[[maybe_unused]] void print_low_level(std::array<T, SIZE>&& arr) {
		print_low_level(arr);
	}

	// make std::tuple printable
	template <typename... T>
	[[maybe_unused]] void print_low_level(std::tuple<T...> const& tup) {
		static auto print_tuple = []<std::size_t... I>(std::tuple<T...> const& tup, std::index_sequence<I...>) {
			char space[]{0, 0, 0};

			// dont need spaces
			((print_low_level(space), print_low_level(std::get<I>(tup)), *space = ',' /*, *(space + 1) = ' '*/), ...);
		};

		print_low_level('(');
		print_tuple(tup, std::make_index_sequence<sizeof...(T)>());
		print_low_level(')');
	}
	template <typename... T>
	[[maybe_unused]] void print_low_level(std::tuple<T...>&& tup) {
		print_low_level(tup);
	}
}  // namespace common