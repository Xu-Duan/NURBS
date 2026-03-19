#pragma once

#include <cstddef>
#include <vector>

namespace nurbs {

using Scalar = double;
using Index = std::size_t;
using Coordinates = std::vector<Scalar>;

struct ControlPoint {
    Coordinates coordinates;
    Scalar weight {1.0};
};

}  // namespace nurbs
