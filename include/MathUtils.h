#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include "arkheon/character/ICharacterController.h"

/**
 * @brief A 3D Vector class for physics calculations using double precision.
 */
class Vector3D {
public:
    double x, y, z;

    // Constructors
    Vector3D();
    Vector3D(double x, double y, double z);

    // Operator Overloading
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator*(double scalar) const;

    // Math Functions
    double dotProduct(const Vector3D& other) const;
    Vector3D crossProduct(const Vector3D& other) const;
    double magnitude() const;

    // SDK conversion
    arkheon_vec3 toSDK() const { return { (float)x, (float)y, (float)z }; }
    static Vector3D fromSDK(const arkheon_vec3& v) { return Vector3D(v.x, v.y, v.z); }

    // Utility for printing
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& v);
};

/**
 * @brief A Quaternion class for representing 3D rotations without gimbal lock.
 * Uses GLTF order (x, y, z, w).
 */
class Quaternion {
public:
    double x, y, z, w;

    // Constructors
    Quaternion(); // Identity: (0,0,0,1)
    Quaternion(double x, double y, double z, double w);

    // Identity
    static Quaternion identity();

    // From Euler (Radians, Order: Y-up standard)
    static Quaternion fromEuler(double pitch, double yaw, double roll);

    // Math Functions
    double norm() const;
    void normalize();
    Quaternion normalized() const;
    Quaternion conjugate() const;

    // Operator Overloading
    Quaternion operator*(const Quaternion& q) const;
    Vector3D rotate(const Vector3D& v) const;

    // Interpolation
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);

    // Conversion
    Vector3D toRotationVector() const;

    // SDK conversion
    arkheon_quat toSDK() const { return { (float)x, (float)y, (float)z, (float)w }; }
    static Quaternion fromSDK(const arkheon_quat& q) { return Quaternion(q.x, q.y, q.z, q.w); }

    // Utility for printing
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
};

#endif // MATH_UTILS_H
