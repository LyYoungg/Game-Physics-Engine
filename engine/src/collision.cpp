#include "inertia/collision.h"

namespace inertia {

bool collideSphereSphere(const Body& a, const Body& b, Contact& out) {
    const Vec3 delta = b.position - a.position;
    const float radii = a.radius + b.radius;
    const float distSq = lengthSq(delta);

    // Compare squared distances: rejects the common "not touching" case
    // without paying for a square root.
    if (distSq >= radii * radii) return false;

    const float dist = std::sqrt(distSq);

    // Edge case: centres (almost) coincide, so there is no direction to push
    // along. normalized() falls back to +Y rather than producing NaN.
    out.normal = normalized(delta);
    out.penetration = radii - dist;
    // Midpoint of the overlap region.
    out.point = a.position + out.normal * (a.radius - 0.5f * out.penetration);
    return true;
}

bool collideSpherePlane(const Body& sphere, const Body& plane, Contact& out) {
    // Signed distance from the sphere centre to the plane surface.
    const float dist = dot(plane.planeNormal, sphere.position) - plane.planeOffset;
    if (dist >= sphere.radius) return false;

    out.normal = -plane.planeNormal;  // from the sphere (a) into the plane (b)
    out.penetration = sphere.radius - dist;
    // Lowest point of the sphere. Using the sphere's own surface (not the
    // plane's) keeps the lever arm exactly `radius`, so rolling obeys v = w*r.
    out.point = sphere.position - plane.planeNormal * sphere.radius;
    return true;
}

}  // namespace inertia
