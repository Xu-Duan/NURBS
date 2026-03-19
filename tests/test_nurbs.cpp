#include "nurbs/Nurbs.hpp"

#include <cassert>
#include <cmath>

namespace {

bool nearlyEqual(double lhs, double rhs, double tolerance = 1.0e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    const nurbs::NurbsCurve curve(
        2,
        nurbs::KnotVector({0.0, 0.0, 0.0, 1.0, 1.0, 1.0}),
        {
            {{0.0, 0.0}, 1.0},
            {{1.0, 2.0}, 1.0},
            {{2.0, 0.0}, 1.0},
        }
    );

    const auto start = curve.evaluate(0.0);
    const auto middle = curve.evaluate(0.5);
    const auto end = curve.evaluate(1.0);

    assert(nearlyEqual(start[0], 0.0));
    assert(nearlyEqual(start[1], 0.0));

    assert(nearlyEqual(middle[0], 1.0));
    assert(nearlyEqual(middle[1], 1.0));

    assert(nearlyEqual(end[0], 2.0));
    assert(nearlyEqual(end[1], 0.0));

    return 0;
}
