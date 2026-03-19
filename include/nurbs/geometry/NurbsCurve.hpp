#pragma once

#include "nurbs/core/KnotVector.hpp"

#include <vector>

namespace nurbs {

class NurbsCurve {
public:
    NurbsCurve(Index degree, KnotVector knotVector, std::vector<ControlPoint> controlPoints);

    Index degree() const noexcept;
    Index dimension() const noexcept;

    const KnotVector& knotVector() const noexcept;
    const std::vector<ControlPoint>& controlPoints() const noexcept;

    Coordinates evaluate(Scalar parameter) const;

private:
    void validate() const;

    Index degree_ {0};
    KnotVector knotVector_;
    std::vector<ControlPoint> controlPoints_;
};

}  // namespace nurbs
