#pragma once
#include <common_output.h>

#include <cstdint>

[[maybe_unused]] constexpr std::uint16_t operator""_u16(unsigned long long int num) { return static_cast<std::uint16_t>(num); }
[[maybe_unused]] constexpr std::int16_t operator""_i16(unsigned long long int num) { return static_cast<std::int16_t>(num); }
[[maybe_unused]] constexpr std::uint8_t operator""_u8(unsigned long long int num) { return static_cast<std::uint8_t>(num); }

namespace common {
	struct MagneticFluxDensityData {
		union {
			struct {
				float x;
				float y;
				float z;
			};
			uint8_t bytes[12];
		};
	};

	void print_low_level(MagneticFluxDensityData const& d) {
		common::print_low_level(d.x);
		common::print_low_level(',');
		common::print_low_level(d.y);
		common::print_low_level(',');
		common::print_low_level(d.z);
	}

}  // namespace common