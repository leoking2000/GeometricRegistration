#pragma once
#include <array>
#include "utils/GeoTypes.h"


namespace geo
{
    using Vec6 = std::array<f32, 6>;
    using Mat6 = std::array<std::array<f32, 6>, 6>;

    // solves A*x=b system that is 6x6
    Vec6 Solve6x6(Mat6 A_in, Vec6 b_in);
}

