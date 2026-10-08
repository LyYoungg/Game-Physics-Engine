// world.h - owns the bodies and runs the simulation pipeline.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "inertia/body.h"
#include "inertia/collision.h"

namespace inertia {

using BodyId = std::uint32_t;

class World {
public:
    // --- Tunables ---
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    float airDensity = 0.0f;         // kg/m^3. 0 = vacuum, 1.2 = air at sea level
    float fixedDt = 1.0f / 120.0f;   // seconds per physics step
    int solverIterations = 8;        // more = stiffer stacks, more CPU
    int maxStepsPerUpdate = 8;       // cap, so a slow frame cannot snowball

    BodyId addBody(const Body& body);
    void clear();

    Body& body(BodyId id) { return bodies_[id]; }
    const Body& body(BodyId id) const { return bodies_[id]; }
    std::vector<Body>& bodies() { return bodies_; }
    const std::vector<Body>& bodies() const { return bodies_; }

    // Advance by exactly `dt` seconds.
    void step(float dt);

    // Feed in real (variable) frame time; runs zero or more fixed steps.
    // Returns how many steps ran.
    int update(float frameDt);

    // --- Diagnostics ---
    const std::vector<Contact>& contacts() const { return contacts_; }
    std::size_t pairsTested() const { return pairsTested_; }
    float kineticEnergy() const;   // translational + rotational, joules
    Vec3 linearMomentum() const;   // kg*m/s

private:
    // A contact plus everything the solver precomputes for it.
    struct Constraint {
        Body* a;
        Body* b;
        Vec3 normal, tangent1, tangent2;
        Vec3 ra, rb;                           // contact point relative to each centre
        float massN, massT1, massT2;           // effective mass along each axis
        float restitutionBias;                 // target separating speed
        float friction;
        float rollingFriction;
        float rollRadius;                      // lever arm for rolling resistance
        float jn = 0, jt1 = 0, jt2 = 0;        // accumulated impulses
        Vec3 jRoll;                            // accumulated angular impulse
    };

    void integrateVelocities(float dt);
    void detectCollisions();
    void buildConstraints();
    void solveVelocities();
    void integratePositions(float dt);
    void correctPositions();

    std::vector<Body> bodies_;
    // Kept as members so their storage is reused: after warm-up the step
    // function performs no heap allocations.
    std::vector<Contact> contacts_;
    std::vector<Constraint> constraints_;
    std::size_t pairsTested_ = 0;
    float accumulator_ = 0.0f;
};

}  // namespace inertia
