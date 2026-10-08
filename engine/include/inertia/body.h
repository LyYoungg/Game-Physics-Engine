// body.h - a rigid body: the state the simulation advances every step.
#pragma once

#include <cstdint>

#include "inertia/math.h"

namespace inertia {

enum class ShapeType : std::uint8_t { Sphere, Plane };

struct Body {
    // --- State (what the integrator advances) ---
    Vec3 position;
    Quat orientation;
    Vec3 velocity;         // m/s
    Vec3 angularVelocity;  // rad/s, world space

    // --- Accumulators (cleared at the end of every step) ---
    Vec3 force;   // N
    Vec3 torque;  // N*m

    // --- Mass properties ---
    // Stored inverted: the solver only ever divides by mass, and an inverse of
    // zero is a clean way to say "infinite mass" (a static body) with no
    // special cases in the maths.
    float invMass = 0.0f;
    Vec3 invInertiaLocal;  // diagonal of the body-space inverse inertia tensor
    Mat3 invInertiaWorld;  // derived: R * invInertiaLocal * R^T

    // --- Material ---
    float restitution = 0.5f;  // 0 = clay, 1 = perfectly elastic
    float friction = 0.5f;     // Coulomb coefficient
    float rollingFriction = 0.0f;  // rolling resistance coefficient; 0 = rolls forever

    // --- Shape ---
    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;               // Sphere
    Vec3 planeNormal{0.0f, 1.0f, 0.0f};  // Plane: free space is dot(n, p) > offset
    float planeOffset = 0.0f;

    std::uint32_t userData = 0;  // free for the game to use (e.g. a colour id)

    bool isStatic() const { return invMass == 0.0f; }
    float mass() const { return invMass > 0.0f ? 1.0f / invMass : 0.0f; }

    // Velocity of the material point at offset `r` from the centre of mass.
    Vec3 velocityAt(Vec3 r) const { return velocity + cross(angularVelocity, r); }

    void applyForce(Vec3 f) { force += f; }

    // A force applied off-centre also produces a torque: tau = r x F.
    void applyForceAtPoint(Vec3 f, Vec3 worldPoint) {
        force += f;
        torque += cross(worldPoint - position, f);
    }

    // An impulse is an instantaneous change in momentum (N*s).
    void applyImpulse(Vec3 impulse, Vec3 r) {
        velocity += impulse * invMass;
        angularVelocity += invInertiaWorld * cross(r, impulse);
    }

    // Recompute the world-space inverse inertia tensor after rotating.
    void updateDerived() {
        const Mat3 rot = orientation.toMat3();
        invInertiaWorld = rot * Mat3::diagonal(invInertiaLocal) * rot.transposed();
    }

    // Solid sphere: m = density * 4/3 pi r^3,  I = 2/5 m r^2.
    static Body makeSphere(Vec3 pos, float radius, float density = 1000.0f) {
        Body b;
        b.shape = ShapeType::Sphere;
        b.position = pos;
        b.radius = radius;
        const float m = density * (4.0f / 3.0f) * kPi * radius * radius * radius;
        const float inertia = 0.4f * m * radius * radius;
        b.invMass = 1.0f / m;
        b.invInertiaLocal = {1.0f / inertia, 1.0f / inertia, 1.0f / inertia};
        b.updateDerived();
        return b;
    }

    // Infinite static half-space. `normal` points into free space.
    static Body makePlane(Vec3 normal, float offset) {
        Body b;
        b.shape = ShapeType::Plane;
        b.planeNormal = normalized(normal);
        b.planeOffset = offset;
        b.invMass = 0.0f;
        b.invInertiaLocal = {};
        b.invInertiaWorld = Mat3::diagonal({});
        return b;
    }
};

}  // namespace inertia
