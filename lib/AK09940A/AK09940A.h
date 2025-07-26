#pragma once

class AK09940A final {
   public:
#pragma pack(push, 1)
	struct MagneticFluxDensityDataRaw {
		union {
			struct {
				std::int32_t x : 18;
				std::int32_t y : 18;
				std::int32_t z : 18;
			};
			std::uint8_t bytes[7];
		};
	};
#pragma pack(pop)

	static std::uint32_t get_scale_factor() { return 100'000'000U; }
};