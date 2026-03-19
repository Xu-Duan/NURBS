#pragma once

#include "nurbs/core/Types.hpp"

#include <vector>

namespace nurbs {

class KnotVector {
public:
    KnotVector() = default;
    explicit KnotVector(std::vector<Scalar> knots);

    const std::vector<Scalar>& values() const noexcept;
    Scalar operator[](Index index) const;

    Index size() const noexcept;
    bool empty() const noexcept;
    bool isNonDecreasing() const noexcept;
    bool isOpenClamped(Index degree) const noexcept;

private:
    std::vector<Scalar> knots_;
};

}  // namespace nurbs
