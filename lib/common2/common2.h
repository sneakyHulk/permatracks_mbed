#pragma once
#include <MagneticFluxDensityData.h>
#include <common2_output.h>

#include <algorithm>
#include <cstdint>
#include <functional>

[[maybe_unused]] constexpr std::uint16_t operator""_u16(unsigned long long int num) { return static_cast<std::uint16_t>(num); }
[[maybe_unused]] constexpr std::int16_t operator""_i16(unsigned long long int num) { return static_cast<std::int16_t>(num); }
[[maybe_unused]] constexpr std::uint8_t operator""_u8(unsigned long long int num) { return static_cast<std::uint8_t>(num); }

namespace common2 {
	inline void print_low_level(MagneticFluxDensityData const& d) {
		common2::print_low_level(d.x);
		common2::print_low_level(',');
		common2::print_low_level(d.y);
		common2::print_low_level(',');
		common2::print_low_level(d.z);
	}

	template <typename T, std::size_t N, typename Compare, typename Midpoint>
	T median(std::array<T, N> const& data, Compare comp, Midpoint mid) {
		std::array<T, N / 2 + 1> sorted;
		std::partial_sort_copy(data.begin(), data.end(), sorted.begin(), sorted.end(), comp);

		if constexpr (N % 2) {
			return sorted.back();
		} else {
			return mid(sorted[sorted.size() - 1], sorted[sorted.size() - 2]);
		}
	}

	template <typename T, std::size_t N>
	requires std::totally_ordered<T> && requires(T a) {
		{ a + a } -> std::convertible_to<T>;
		{ a / 2 } -> std::convertible_to<T>;
	} T median(std::array<T, N> const& data) {
		std::array<T, N / 2 + 1> sorted;
		std::partial_sort_copy(data.begin(), data.end(), sorted.begin(), sorted.end());

		if constexpr (N % 2) {
			return sorted.back();
		} else {
			return (sorted[sorted.size() - 1] + sorted[sorted.size() - 2]) / (2);
		}
	}
}  // namespace common2