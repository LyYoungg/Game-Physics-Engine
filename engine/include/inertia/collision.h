// collision.h - narrow phase: exact tests between two specific shapes.
#pragma once

#include <cstdint>

#include "inertia/body.h"

namespace inertia {

// One touching point between two bodies.
struct Contact {
    std::uint32_t a = 0, b = 0;  // body indices
    Vec3 normal;                 // unit vector pointing from body a to body b
    Vec3 point;                  // world-space contact point
    float penetration = 0.0f;    // overlap depth along the normal (>= 0)
};

// Each test fills `out` (except the body indices) and returns true on overlap.
bool collideSphereSphere(const Body& a, const Body& b, Contact& out);
bool collideSpherePlane(const Body& sphere, const Body& plane, Contact& out);

}  // namespace inertia
