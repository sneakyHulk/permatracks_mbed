#pragma once
#include <cstdint>

[[maybe_unused]] constexpr std::uint16_t operator""_u16(unsigned long long int num) { return static_cast<std::uint8_t>(num); }
[[maybe_unused]] constexpr std::int16_t operator""_i16(unsigned long long int num) { return static_cast<std::uint8_t>(num); }

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
}  // namespace common