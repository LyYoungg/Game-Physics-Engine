#include "inertia/world.h"

#include <algorithm>

namespace inertia {

namespace {

constexpr float kSphereDragCoefficient = 0.47f;  // smooth sphere, turbulent flow
constexpr float kRestitutionThreshold = 0.5f;    // m/s; slower impacts do not bounce
constexpr float kPenetrationSlop = 0.005f;       // m of overlap we tolerate
constexpr float kCorrectionFraction = 0.4f;      // share of overlap fixed per step

// 1 / (how hard it is to change the relative velocity along `axis` at the
// contact point). Accounts for both bodies' mass and rotational inertia.
float effectiveMass(const Body& a, const Body& b, Vec3 ra, Vec3 rb, Vec3 axis) {
    const Vec3 raXn = cross(ra, axis);
    const Vec3 rbXn = cross(rb, axis);
    const float k = a.invMass + b.invMass +
                    dot(raXn, a.invInertiaWorld * raXn) +
                    dot(rbXn, b.invInertiaWorld * rbXn);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

void applyPair(Body& a, Body& b, Vec3 ra, Vec3 rb, Vec3 impulse) {
    // Newton's third law: equal and opposite.
    a.applyImpulse(-impulse, ra);
    b.applyImpulse(impulse, rb);
}

}  // namespace

BodyId World::addBody(const Body& body) {
    bodies_.push_back(body);
    return static_cast<BodyId>(bodies_.size() - 1);
}

void World::clear() {
    bodies_.clear();
    contacts_.clear();
    constraints_.clear();
    accumulator_ = 0.0f;
}

// Fixed timestep with an accumulator. Physics always advances in identical
// `fixedDt` slices no matter the frame rate, which keeps the simulation stable
// and reproducible. Leftover time carries over to the next frame.
int World::update(float frameDt) {
    // Clamp: after a long hitch (window drag, breakpoint) do not try to
    // simulate the whole gap, or each frame would take longer than the last.
    accumulator_ += std::min(frameDt, 0.25f);
    int steps = 0;
    while (accumulator_ >= fixedDt && steps < maxStepsPerUpdate) {
        step(fixedDt);
        accumulator_ -= fixedDt;
        ++steps;
    }
    if (steps == maxStepsPerUpdate) accumulator_ = 0.0f;  // drop the backlog
    return steps;
}

void World::step(float dt) {
    integrateVelocities(dt);
    detectCollisions();
    buildConstraints();
    for (int i = 0; i < solverIterations; ++i) solveVelocities();
    integratePositions(dt);
    correctPositions();
}

// Forces -> velocities. Together with integratePositions this is semi-implicit
// (symplectic) Euler: the *new* velocity is used to move the body. It costs the
// same as explicit Euler but does not gain energy over time.
void World::integrateVelocities(float dt) {
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;

        // Weight: F = m g
        b.force += gravity * b.mass();

        // Aerodynamic drag: F = -1/2 * rho * Cd * A * |v| * v
        if (airDensity > 0.0f && b.shape == ShapeType::Sphere) {
            const float area = kPi * b.radius * b.radius;
            const float k = 0.5f * airDensity * kSphereDragCoefficient * area;
            b.force -= b.velocity * (k * length(b.velocity));
        }

        // Newton's second law, linear and angular: a = F/m, alpha = I^-1 tau.
        // (The gyroscopic term w x Iw is zero for spheres; it arrives with
        // boxes in stage 2.)
        b.velocity += b.force * (b.invMass * dt);
        b.angularVelocity += (b.invInertiaWorld * b.torque) * dt;

        b.force = {};
        b.torque = {};
    }
}

// Broad phase + narrow phase. Stage 1 tests every pair: O(n^2). Stage 3
// replaces the outer loops with a spatial partition.
void World::detectCollisions() {
    contacts_.clear();
    pairsTested_ = 0;
    const std::size_t n = bodies_.size();

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Body& bi = bodies_[i];
            const Body& bj = bodies_[j];
            if (bi.isStatic() && bj.isStatic()) continue;
            ++pairsTested_;

            Contact c;
            bool hit = false;
            if (bi.shape == ShapeType::Sphere && bj.shape == ShapeType::Sphere) {
                c.a = static_cast<BodyId>(i);
                c.b = static_cast<BodyId>(j);
                hit = collideSphereSphere(bi, bj, c);
            } else if (bi.shape == ShapeType::Sphere && bj.shape == ShapeType::Plane) {
                c.a = static_cast<BodyId>(i);
                c.b = static_cast<BodyId>(j);
                hit = collideSpherePlane(bi, bj, c);
            } else if (bi.shape == ShapeType::Plane && bj.shape == ShapeType::Sphere) {
                c.a = static_cast<BodyId>(j);  // keep the sphere as body a
                c.b = static_cast<BodyId>(i);
                hit = collideSpherePlane(bj, bi, c);
            }
            if (hit) contacts_.push_back(c);
        }
    }
}

void World::buildConstraints() {
    constraints_.clear();
    for (const Contact& c : contacts_) {
        Constraint k;
        k.a = &bodies_[c.a];
        k.b = &bodies_[c.b];
        k.normal = c.normal;
        k.ra = c.point - k.a->position;
        k.rb = c.point - k.b->position;

        // Two axes perpendicular to the normal, for friction.
        const Vec3 helper = std::fabs(c.normal.x) > 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
        k.tangent1 = normalized(cross(c.normal, helper));
        k.tangent2 = cross(c.normal, k.tangent1);

        k.massN = effectiveMass(*k.a, *k.b, k.ra, k.rb, k.normal);
        k.massT1 = effectiveMass(*k.a, *k.b, k.ra, k.rb, k.tangent1);
        k.massT2 = effectiveMass(*k.a, *k.b, k.ra, k.rb, k.tangent2);

        // Common mixing rules: the bouncier material wins; friction uses the
        // geometric mean.
        const float e = std::max(k.a->restitution, k.b->restitution);
        k.friction = std::sqrt(k.a->friction * k.b->friction);
        k.rollingFriction = std::max(k.a->rollingFriction, k.b->rollingFriction);
        // Sphere on plane: the sphere's radius. Sphere on sphere: the reduced
        // radius r1 r2 / (r1 + r2), which tends to the same thing as one
        // sphere grows towards a flat surface.
        k.rollRadius = k.b->shape == ShapeType::Sphere
                           ? k.a->radius * k.b->radius / (k.a->radius + k.b->radius)
                           : k.a->radius;

        // Closing speed along the normal (negative = approaching). Restitution
        // asks for a separating speed of e * closing speed. Below a threshold
        // we ask for none, otherwise resting bodies would jitter forever on
        // the tiny velocity gravity adds each step.
        const float vn = dot(k.b->velocityAt(k.rb) - k.a->velocityAt(k.ra), k.normal);
        k.restitutionBias = vn < -kRestitutionThreshold ? -e * vn : 0.0f;

        constraints_.push_back(k);
    }
}

// Sequential impulses: visit each contact in turn and apply the impulse that
// fixes *its* velocity error. One pass disturbs neighbouring contacts, so we
// repeat; the accumulated impulses converge towards the global solution.
void World::solveVelocities() {
    for (Constraint& k : constraints_) {
        Body& a = *k.a;
        Body& b = *k.b;

        // --- Friction (Coulomb): |tangent impulse| <= mu * normal impulse ---
        const float maxFriction = k.friction * k.jn;
        {
            const float vt = dot(b.velocityAt(k.rb) - a.velocityAt(k.ra), k.tangent1);
            const float old = k.jt1;
            k.jt1 = std::clamp(old - k.massT1 * vt, -maxFriction, maxFriction);
            applyPair(a, b, k.ra, k.rb, k.tangent1 * (k.jt1 - old));
        }
        {
            const float vt = dot(b.velocityAt(k.rb) - a.velocityAt(k.ra), k.tangent2);
            const float old = k.jt2;
            k.jt2 = std::clamp(old - k.massT2 * vt, -maxFriction, maxFriction);
            applyPair(a, b, k.ra, k.rb, k.tangent2 * (k.jt2 - old));
        }

        // --- Non-penetration ---
        // Clamp the *accumulated* impulse, not the per-iteration delta: a
        // contact may push but never pull, yet a later iteration is allowed to
        // take back some of what an earlier one over-applied.
        const float vn = dot(b.velocityAt(k.rb) - a.velocityAt(k.ra), k.normal);
        const float old = k.jn;
        k.jn = std::max(old + k.massN * (k.restitutionBias - vn), 0.0f);
        applyPair(a, b, k.ra, k.rb, k.normal * (k.jn - old));

        // --- Rolling resistance ---
        // Real balls stop because the contact patch deforms, which acts as a
        // torque opposing the relative spin, limited to mu_r * N * r. Solved
        // like friction: aim to cancel the relative spin, then clamp.
        if (k.rollingFriction > 0.0f) {
            const Vec3 w = b.angularVelocity - a.angularVelocity;
            const float kk = dot(w, a.invInertiaWorld * w + b.invInertiaWorld * w);
            if (kk > 1e-12f) {
                Vec3 target = k.jRoll - w * (lengthSq(w) / kk);
                const float maxRoll = k.rollingFriction * k.jn * k.rollRadius;
                const float len = length(target);
                if (len > maxRoll) target *= maxRoll / len;
                const Vec3 delta = target - k.jRoll;
                k.jRoll = target;
                a.angularVelocity -= a.invInertiaWorld * delta;
                b.angularVelocity += b.invInertiaWorld * delta;
            }
        }
    }
}

void World::integratePositions(float dt) {
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;
        b.position += b.velocity * dt;
        b.orientation = b.orientation.integrated(b.angularVelocity, dt);
        b.updateDerived();
    }
}

// The velocity solver stops bodies moving *further* into each other but does
// not undo overlap that already exists. Push them apart directly, split by
// inverse mass. Only part of the overlap is removed per step, and a small
// "slop" is left alone, so resting contacts stay touching instead of popping
// in and out of contact.
void World::correctPositions() {
    for (const Contact& c : contacts_) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];
        const float invMassSum = a.invMass + b.invMass;
        if (invMassSum <= 0.0f) continue;
        const float depth = std::max(c.penetration - kPenetrationSlop, 0.0f);
        const Vec3 correction = c.normal * (kCorrectionFraction * depth / invMassSum);
        a.position -= correction * a.invMass;
        b.position += correction * b.invMass;
    }
}

float World::kineticEnergy() const {
    float e = 0.0f;
    for (const Body& b : bodies_) {
        if (b.isStatic()) continue;
        // 1/2 m v^2  +  1/2 w . I w   (spheres: I is a scalar)
        e += 0.5f * b.mass() * lengthSq(b.velocity);
        if (b.invInertiaLocal.x > 0.0f)
            e += 0.5f * lengthSq(b.angularVelocity) / b.invInertiaLocal.x;
    }
    return e;
}

Vec3 World::linearMomentum() const {
    Vec3 p;
    for (const Body& b : bodies_)
        if (!b.isStatic()) p += b.velocity * b.mass();
    return p;
}

}  // namespace inertia
