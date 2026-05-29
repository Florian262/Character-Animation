#include "MathUtils.h"
#include <cmath>
#include <algorithm>

// Vector3D implementation
Vector3D::Vector3D() : x(0.0), y(0.0), z(0.0) {}

Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator*(double scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);
}

double Vector3D::dotProduct(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::crossProduct(const Vector3D& other) const {
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

double Vector3D::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

std::ostream& operator<<(std::ostream& os, const Vector3D& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

// Quaternion implementation
Quaternion::Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}

Quaternion::Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}

Quaternion Quaternion::identity() {
    return Quaternion(0.0, 0.0, 0.0, 1.0);
}

Quaternion Quaternion::fromEuler(double pitch, double yaw, double roll) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    return Quaternion(
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * cp * cr + sy * sp * sr
    );
}

double Quaternion::norm() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

void Quaternion::normalize() {
    double n = norm();
    if (n > 1e-9) {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
    } else {
        *this = identity();
    }
}

Quaternion Quaternion::normalized() const {
    Quaternion q = *this;
    q.normalize();
    return q;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w,
        w * q.w - x * q.x - y * q.y - z * q.z
    );
}

Vector3D Quaternion::rotate(const Vector3D& v) const {
    Quaternion p(v.x, v.y, v.z, 0.0);
    Quaternion result = (*this) * p * conjugate();
    return Vector3D(result.x, result.y, result.z);
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    
    Quaternion b_copy = b;
    if (dot < 0.0f) {
        dot = -dot;
        b_copy = Quaternion(-b.x, -b.y, -b.z, -b.w);
    }

    if (dot > 0.9995) {
        // Linear interpolation for very close orientations
        Quaternion res(
            a.x + t * (b_copy.x - a.x),
            a.y + t * (b_copy.y - a.y),
            a.z + t * (b_copy.z - a.z),
            a.w + t * (b_copy.w - a.w)
        );
        return res.normalized();
    }

    double theta_0 = std::acos(dot);
    double theta = theta_0 * t;
    double sin_theta = std::sin(theta);
    double sin_theta_0 = std::sin(theta_0);

    double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    return Quaternion(
        s0 * a.x + s1 * b_copy.x,
        s0 * a.y + s1 * b_copy.y,
        s0 * a.z + s1 * b_copy.z,
        s0 * a.w + s1 * b_copy.w
    );
}

Vector3D Quaternion::toRotationVector() const {
    Quaternion q = normalized();
    
    // Ensure we take the shortest path
    if (q.w < 0.0) {
        q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
    }
    
    double angle = 2.0 * std::acos(std::clamp(q.w, -1.0, 1.0));
    double s = std::sqrt(std::max(0.0, 1.0 - q.w * q.w));
    
    // If s is close to zero, the angle is zero (no rotation)
    if (s < 0.001) {
        return Vector3D(0.0, 0.0, 0.0);
    }
    
    // Axis * Angle
    return Vector3D(q.x / s, q.y / s, q.z / s) * angle;
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
    return os;
}
