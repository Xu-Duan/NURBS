#include "nurbs/Nurbs.hpp"

#include <iostream>

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

    const nurbs::Coordinates point = curve.evaluate(0.5);

    std::cout << "Curve point at u=0.5: (";
    for (nurbs::Index i = 0; i < point.size(); ++i) {
        std::cout << point[i];
        if (i + 1 < point.size()) {
            std::cout << ", ";
        }
    }
    std::cout << ")\n";

    return 0;
}
