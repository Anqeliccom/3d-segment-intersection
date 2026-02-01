#pragma once

class Vector3D {
public:
    double x, y, z;

    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3D operator+(const Vector3D& v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
    Vector3D operator-(const Vector3D& v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
    Vector3D operator*(double t) const { return Vector3D(x * t, y * t , z * t); }

    double dot(const Vector3D& v) const { return x * v.x + y * v.y + z * v.z; }

    Vector3D cross(const Vector3D& v) const {
        return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    double lengthSquared() const { return x * x + y * y + z * z; }
};