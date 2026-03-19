#include "nurbs/basis/Basis.hpp"

#include <stdexcept>

namespace nurbs {

Index Basis::findSpan(Index degree,
                      const KnotVector& knotVector,
                      Index controlPointCount,
                      Scalar parameter) {
    if (controlPointCount == 0) {
        throw std::invalid_argument("control point count must be positive");
    }

    if (controlPointCount <= degree) {
        throw std::invalid_argument("control point count must exceed degree");
    }

    const Index n = controlPointCount - 1;
    const auto& knots = knotVector.values();

    if (knots.size() != controlPointCount + degree + 1) {
        throw std::invalid_argument("invalid knot vector size for given degree and control points");
    }

    const Scalar start = knots[degree];
    const Scalar end = knots[n + 1];

    if (parameter < start || parameter > end) {
        throw std::out_of_range("parameter is outside the knot domain");
    }

    if (parameter == end) {
        return n;
    }

    Index low = degree;
    Index high = n + 1;
    Index mid = (low + high) / 2;

    while (parameter < knots[mid] || parameter >= knots[mid + 1]) {
        if (parameter < knots[mid]) {
            high = mid;
        } else {
            low = mid;
        }

        mid = (low + high) / 2;
    }

    return mid;
}

std::vector<Scalar> Basis::evaluate(Index span,
                                    Scalar parameter,
                                    Index degree,
                                    const KnotVector& knotVector) {
    const auto& knots = knotVector.values();

    if (span >= knots.size() - 1) {
        throw std::out_of_range("basis span is outside the knot vector");
    }

    std::vector<Scalar> basis(degree + 1, 0.0);
    std::vector<Scalar> left(degree + 1, 0.0);
    std::vector<Scalar> right(degree + 1, 0.0);

    basis[0] = 1.0;

    for (Index j = 1; j <= degree; ++j) {
        left[j] = parameter - knots[span + 1 - j];
        right[j] = knots[span + j] - parameter;

        Scalar saved = 0.0;

        for (Index r = 0; r < j; ++r) {
            const Scalar denominator = right[r + 1] + left[j - r];

            if (denominator == 0.0) {
                throw std::runtime_error("encountered zero denominator while evaluating basis");
            }

            const Scalar temp = basis[r] / denominator;
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        basis[j] = saved;
    }

    return basis;
}

}  // namespace nurbs
