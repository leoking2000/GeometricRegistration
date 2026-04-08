#pragma once
#include <cstdint>
#include <limits>

namespace geo
{
    // Common fixed-width numeric aliases used across the library for consistency.

    using i8 = std::int8_t;
    using u8 = std::uint8_t;

    using i16 = std::int16_t;
    using u16 = std::uint16_t;

    using i32 = std::int32_t;
    using u32 = std::uint32_t;

    using i64 = std::int64_t;
    using u64 = std::uint64_t;

    using f32 = float;
    using f64 = double;

    // index_t is the canonical index type for point, vertex, and correspondence access.
    using index_t = u32;

    // INVALID_INDEX is a sentinel for APIs that need to report "no valid index".
    inline constexpr index_t INVALID_INDEX = std::numeric_limits<index_t>::max();

    inline constexpr f32 F32_MAX = std::numeric_limits<f32>::max();
    inline constexpr f64 F64_MAX = std::numeric_limits<f64>::max();

}
