#pragma once

#include "Vector3D.hpp"
#include <optional>

constexpr double EPS = 1e-9;

class Segment3D {
private:
    bool containsPoint(const Vector3D& p) const;
    std::optional<Vector3D> handleParallelSegments(const Vector3D& u, const Vector3D& v, const Vector3D& w) const;

public:
    Vector3D start, end;
    Segment3D(const Vector3D& v1, const Vector3D& v2) : start(v1), end(v2) {}

    std::optional<Vector3D> Intersect(const Segment3D& other) const;
    Vector3D directionVector() const { return end - start; }
};