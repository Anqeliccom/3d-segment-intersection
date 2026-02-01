#include "Segment3D.hpp"
#include <algorithm>

bool Segment3D::containsPoint(const Vector3D& p) const {
    Vector3D d = directionVector();
    Vector3D toP = p - start;

    if (d.cross(toP).lengthSquared() > EPS * EPS) return false;
    double t = toP.dot(d) / d.dot(d);

    return t >= -EPS && t <= 1.0 + EPS;
}

std::optional<Vector3D> Segment3D::handleParallelSegments(const Vector3D& u, const Vector3D& v, const Vector3D& w) const {
    if (u.cross(w).lengthSquared() > EPS * EPS) {
        return std::nullopt;
    }

    double t_st2 = w.dot(u) / u.dot(u);
    double t_end2 = (w + v).dot(u) / u.dot(u);

    double overlap_start = std::max(0.0, std::min(t_st2, t_end2));
    double overlap_end = std::min(1.0, std::max(t_st2, t_end2));

    if (overlap_start <= overlap_end + EPS) {
        double t_mid = (overlap_start + overlap_end) / 2.0;
        return start + directionVector() * t_mid;
    }

    return std::nullopt;
}

std::optional<Vector3D> Segment3D::Intersect(const Segment3D& other) const {
    Vector3D u = directionVector();
    Vector3D v = other.directionVector();
    Vector3D w = other.start - start;

    bool thisIsPoint = u.lengthSquared() < EPS*EPS;
    bool otherIsPoint = v.lengthSquared() < EPS*EPS;

    if (thisIsPoint && otherIsPoint)
        return (start - other.start).lengthSquared() < EPS*EPS ? std::optional<Vector3D>(start) : std::nullopt;
    if (thisIsPoint)
        return other.containsPoint(start) ? std::optional<Vector3D>(start) : std::nullopt;
    if (otherIsPoint)
        return containsPoint(other.start) ? std::optional<Vector3D>(other.start) : std::nullopt;

    if (std::abs(u.cross(v).dot(w)) > EPS) {
        return std::nullopt;
    }

    auto solve2DSystem = [&](double det, double u1, double u2,
            double v1, double v2, double w1, double w2) -> std::optional<Vector3D> {

        if ((std::abs(det) < EPS)) return std::nullopt;

        double s = (w1 * v2 - w2 * v1) / det;
        double t = (u1 * w2 - u2 * w1) / det;

        if (s < -EPS || s > 1.0 + EPS || t < -EPS || t > 1.0 + EPS) return std::nullopt;

        return start + u * s;
    };

    double detXY = -(u.x * v.y - u.y * v.x);
    double detXZ = -(u.x * v.z - u.z * v.x);
    double detYZ = -(u.y * v.z - u.z * v.y);

    if (auto r = solve2DSystem(detXY, u.x, u.y, -v.x, -v.y, w.x, w.y)) return r;
    if (auto r = solve2DSystem(detXZ, u.x, u.z, -v.x, -v.z, w.x, w.z)) return r;
    if (auto r = solve2DSystem(detYZ, u.y, u.z, -v.y, -v.z, w.y, w.z)) return r;

    return handleParallelSegments(u, v, w);
}