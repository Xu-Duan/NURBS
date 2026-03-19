#include "nurbs/geometry/NurbsCurve.hpp"

#include "nurbs/basis/Basis.hpp"

#include <stdexcept>

namespace nurbs {

NurbsCurve::NurbsCurve(Index degree, KnotVector knotVector, std::vector<ControlPoint> controlPoints)
    : degree_(degree), knotVector_(std::move(knotVector)), controlPoints_(std::move(controlPoints)) {
    validate();
}

Index NurbsCurve::degree() const noexcept {
    return degree_;
}

Index NurbsCurve::dimension() const noexcept {
    return controlPoints_.empty() ? 0 : controlPoints_.front().coordinates.size();
}

const KnotVector& NurbsCurve::knotVector() const noexcept {
    return knotVector_;
}

const std::vector<ControlPoint>& NurbsCurve::controlPoints() const noexcept {
    return controlPoints_;
}

Coordinates NurbsCurve::evaluate(Scalar parameter) const {
    const Index span = Basis::findSpan(degree_, knotVector_, controlPoints_.size(), parameter);
    const std::vector<Scalar> basis = Basis::evaluate(span, parameter, degree_, knotVector_);

    Coordinates point(dimension(), 0.0);
    Scalar weightSum = 0.0;

    for (Index local = 0; local <= degree_; ++local) {
        const Index controlIndex = span - degree_ + local;
        const ControlPoint& controlPoint = controlPoints_.at(controlIndex);
        const Scalar weightedBasis = basis[local] * controlPoint.weight;

        for (Index axis = 0; axis < point.size(); ++axis) {
            point[axis] += weightedBasis * controlPoint.coordinates[axis];
        }

        weightSum += weightedBasis;
    }

    if (weightSum == 0.0) {
        throw std::runtime_error("curve evaluation produced zero homogeneous weight");
    }

    for (Scalar& coordinate : point) {
        coordinate /= weightSum;
    }

    return point;
}

void NurbsCurve::validate() const {
    if (controlPoints_.empty()) {
        throw std::invalid_argument("curve requires at least one control point");
    }

    if (controlPoints_.size() <= degree_) {
        throw std::invalid_argument("control point count must exceed degree");
    }

    if (knotVector_.size() != controlPoints_.size() + degree_ + 1) {
        throw std::invalid_argument("knot vector size does not match curve definition");
    }

    if (!knotVector_.isOpenClamped(degree_)) {
        throw std::invalid_argument("starter implementation expects an open clamped knot vector");
    }

    const Index expectedDimension = controlPoints_.front().coordinates.size();

    if (expectedDimension == 0) {
        throw std::invalid_argument("control point dimension must be positive");
    }

    for (const ControlPoint& controlPoint : controlPoints_) {
        if (controlPoint.coordinates.size() != expectedDimension) {
            throw std::invalid_argument("all control points must share the same dimension");
        }

        if (controlPoint.weight <= 0.0) {
            throw std::invalid_argument("control point weights must be positive");
        }
    }
}

}  // namespace nurbs
