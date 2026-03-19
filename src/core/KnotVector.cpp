#include "nurbs/core/KnotVector.hpp"

#include <stdexcept>

namespace nurbs {

KnotVector::KnotVector(std::vector<Scalar> knots) : knots_(std::move(knots)) {
    if (!isNonDecreasing()) {
        throw std::invalid_argument("knot vector must be non-decreasing");
    }
}

const std::vector<Scalar>& KnotVector::values() const noexcept {
    return knots_;
}

Scalar KnotVector::operator[](Index index) const {
    return knots_.at(index);
}

Index KnotVector::size() const noexcept {
    return knots_.size();
}

bool KnotVector::empty() const noexcept {
    return knots_.empty();
}

bool KnotVector::isNonDecreasing() const noexcept {
    for (Index i = 1; i < knots_.size(); ++i) {
        if (knots_[i] < knots_[i - 1]) {
            return false;
        }
    }

    return true;
}

bool KnotVector::isOpenClamped(Index degree) const noexcept {
    if (knots_.size() < 2 * (degree + 1)) {
        return false;
    }

    const Scalar first = knots_.front();
    const Scalar last = knots_.back();

    for (Index i = 0; i <= degree; ++i) {
        if (knots_[i] != first) {
            return false;
        }

        if (knots_[knots_.size() - 1 - i] != last) {
            return false;
        }
    }

    return true;
}

}  // namespace nurbs
