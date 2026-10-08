// math.h - the small linear algebra kit the whole engine is built on.
//
// Everything here is a plain value type with no heap allocation, so the
// compiler can keep vectors in registers and inline every operation.
#pragma once

#include <cmath>

namespace inertia {

constexpr float kPi = 3.14159265358979323846f;

// ---------------------------------------------------------------- Vec3 -----
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }

constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

constexpr float lengthSq(Vec3 v) { return dot(v, v); }
inline float length(Vec3 v) { return std::sqrt(lengthSq(v)); }

// Returns `fallback` for a (near) zero vector instead of dividing by zero.
// Zero-length normals are one of the classic sources of NaNs in physics code.
inline Vec3 normalized(Vec3 v, Vec3 fallback = {0.0f, 1.0f, 0.0f}) {
    const float len = length(v);
    return len > 1e-8f ? v / len : fallback;
}

inline bool isFinite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// ---------------------------------------------------------------- Mat3 -----
// Row-major 3x3 matrix. Used for rotations and inertia tensors.
struct Mat3 {
    float m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    static Mat3 diagonal(Vec3 d) {
        Mat3 r;
        r.m[0][0] = d.x; r.m[1][1] = d.y; r.m[2][2] = d.z;
        return r;
    }

    Vec3 operator*(Vec3 v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
        return r;
    }

    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
};

// ---------------------------------------------------------------- Quat -----
// Unit quaternion representing an orientation. Chosen over Euler angles (no
// gimbal lock) and over matrices (4 floats, cheap to renormalise after drift).
struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromAxisAngle(Vec3 axis, float radians) {
        const Vec3 a = inertia::normalized(axis);
        const float s = std::sin(radians * 0.5f);
        return {std::cos(radians * 0.5f), a.x * s, a.y * s, a.z * s};
    }

    constexpr Quat operator*(Quat o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    Quat normalized() const {
        const float len = std::sqrt(w * w + x * x + y * y + z * z);
        if (len < 1e-8f) return {};
        return {w / len, x / len, y / len, z / len};
    }

    Mat3 toMat3() const {
        Mat3 r;
        r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - w * z);     r.m[0][2] = 2 * (x * z + w * y);
        r.m[1][0] = 2 * (x * y + w * z);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - w * x);
        r.m[2][0] = 2 * (x * z - w * y);     r.m[2][1] = 2 * (y * z + w * x);     r.m[2][2] = 1 - 2 * (x * x + y * y);
        return r;
    }

    Vec3 rotate(Vec3 v) const { return toMat3() * v; }

    // Advance the orientation by a world-space angular velocity for `dt`
    // seconds:  dq/dt = 1/2 * (0, omega) * q.  The result is renormalised
    // because the first-order update slowly stretches the quaternion.
    Quat integrated(Vec3 omega, float dt) const {
        const Quat dq = Quat(0.0f, omega.x, omega.y, omega.z) * (*this);
        return Quat(w + 0.5f * dt * dq.w, x + 0.5f * dt * dq.x,
                    y + 0.5f * dt * dq.y, z + 0.5f * dt * dq.z).normalized();
    }
};

}  // namespace inertia
