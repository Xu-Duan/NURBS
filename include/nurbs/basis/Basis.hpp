#pragma once

#include "nurbs/core/KnotVector.hpp"

#include <vector>

namespace nurbs {

class Basis {
public:
    static Index findSpan(Index degree,
                          const KnotVector& knotVector,
                          Index controlPointCount,
                          Scalar parameter);

    static std::vector<Scalar> evaluate(Index span,
                                        Scalar parameter,
                                        Index degree,
                                        const KnotVector& knotVector);
};

}  // namespace nurbs
