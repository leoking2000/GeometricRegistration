#pragma once
#include <cstdint>
#include <limits>

namespace geo
{
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

    using index_t = u32;

    inline constexpr f32 F32_MAX = std::numeric_limits<f32>::max();
    inline constexpr f32 F32_MIN = std::numeric_limits<f32>::lowest();

    inline constexpr f64 F64_MAX = std::numeric_limits<f64>::max();
    inline constexpr f64 F64_MIN = std::numeric_limits<f64>::lowest();

    inline constexpr f32 EPSILON = 1e-6f;
}
