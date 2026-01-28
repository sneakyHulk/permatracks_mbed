#pragma once
#include <common_printing.h>

#include <cstdio>
#include <source_location>

namespace common {
#ifdef __cpp_concepts
	template <typename T>
	concept printable = requires(const T& msg) {
		{ print_low_level(msg) };
	};

	template <std::size_t N>
	const char* pad0(std::integral auto value, char (&out)[N]) {
		char format[6]{};
		std::snprintf(format, sizeof(format), "%%0%uu", N - 1);
		std::snprintf(out, N, format, value);

		return out;
	}

	[[maybe_unused]] void print(printable auto&&... args) { (print_low_level(args), ...); }
	[[maybe_unused]] void println(printable auto&&... args) {
		(print_low_level(args), ...);

		println_low_level();
	}

	[[maybe_unused]] void print_time(std::integral auto timestamp, printable auto&&... args) {
		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		print(args...);
	}
	[[maybe_unused]] void println_time(std::integral auto timestamp, printable auto&&... args) {
		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		println(args...);
	}

	template <printable... Args>
	struct println_time_loc {
		explicit println_time_loc(std::integral auto timestamp, Args&&... args, std::source_location const location = std::source_location::current()) {
			char buf[11]{};
			print('[', pad0(timestamp, buf), ',', location.file_name(), "]: ");
			println(args...);
		}
	};

	template <printable... Args>
	println_time_loc(std::integral auto timestamp, Args&&... args) -> println_time_loc<Args...>;

	template <printable... Args>
	struct print_time_loc {
		explicit print_time_loc(std::integral auto timestamp, Args&&... args, std::source_location const location = std::source_location::current()) {
			char buf[11]{};
			print('[', pad0(timestamp, buf), ',', location.file_name(), "]: ");
			print(args...);
		}
	};

	template <printable... Args>
	print_time_loc(std::integral auto timestamp, Args&&... args) -> print_time_loc<Args...>;

	inline constexpr auto BLK = "\033[0;30m";
	inline constexpr auto RED = "\033[0;31m";
	inline constexpr auto GRN = "\033e[0;32m";
	inline constexpr auto YEL = "\033[0;33m";
	inline constexpr auto BLU = "\033[0;34m";
	inline constexpr auto MAG = "\033[0;35m";
	inline constexpr auto CYN = "\033[0;36m";
	inline constexpr auto WHT = "\033[0;37m";
	inline constexpr auto RESET = "\033[0m";

	[[maybe_unused]] void print_warn(printable auto&&... args) {
		print(YEL);

		print(args...);

		print(RESET);
	}
	[[maybe_unused]] void println_warn(printable auto&&... args) {
		print(YEL);

		println(args...);

		print(RESET);
	}

	[[maybe_unused]] void print_warn_time(std::integral auto timestamp, printable auto&&... args) {
		print(YEL);

		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		print(args...);

		print(RESET);
	}
	[[maybe_unused]] void println_warn_time(std::integral auto timestamp, printable auto&&... args) {
		print(YEL);

		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		println(args...);

		print(RESET);
	}

	template <printable... Args>
	struct println_warn_time_loc {
		explicit println_warn_time_loc(std::integral auto timestamp, Args&&... args, std::source_location const location = std::source_location::current()) {
			print(YEL);

			char buf[11]{};
			print('[', pad0(millis(), buf), ',', location.file_name(), "]: ");
			println(args...);

			print(RESET);
		}
	};

	template <printable... Args>
	println_warn_time_loc(std::integral auto timestamp, Args&&... args) -> println_warn_time_loc<Args...>;

	[[maybe_unused]] void print_error(printable auto&&... args) {
		print(RED);

		print(args...);

		print(RESET);
	}

	[[maybe_unused]] void print_error_time(std::integral auto timestamp, printable auto&&... args) {
		print(RED);

		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		print(args...);

		print(RESET);
	}
	[[maybe_unused]] void println_error_time(std::integral auto timestamp, printable auto&&... args) {
		print(RED);

		char buf[11]{};
		print('[', pad0(timestamp, buf), "]: ");
		println(args...);

		print(RESET);
	}

	template <printable... Args>
	struct println_error_time_loc {
		explicit println_error_time_loc(std::integral auto timestamp, Args&&... args, std::source_location const location = std::source_location::current()) {
			print(RED);

			char buf[11]{};
			print('[', pad0(millis(), buf), ',', location.file_name(), "]: ");
			println(args...);

			print(RESET);
		}
	};

	template <printable... Args>
	println_error_time_loc(std::integral auto timestamp, Args&&... args) -> println_error_time_loc<Args...>;

	[[maybe_unused]] void println_critical(printable auto&&... args) {
		print(RED);

		while (true) println(args...);
	}

	[[maybe_unused]] void println_critical_time(std::integral auto timestamp, printable auto&&... args) {
		print(RED);

		while (true) {
			char buf[11]{};
			print('[', pad0(timestamp, buf), "]: ");
			println(args...);
		}
	}

	template <printable... Args>
	struct println_critical_time_loc {
		[[noreturn]] explicit println_critical_time_loc(std::integral auto timestamp, Args&&... args, std::source_location const location = std::source_location::current()) {
			print(RED);

			while (true) {
				char buf[11]{};
				print('[', pad0(timestamp, buf), "]: ");
				println(args...);
			}
		}
	};

	template <printable... Args>
	println_critical_time_loc(std::integral auto timestamp, Args&&... args) -> println_critical_time_loc<Args...>;

#endif
}  // namespace common