// Headless tests: each one checks the engine against a result you can derive
// on paper. No window, no framework - just run the executable.
#include <cmath>
#include <cstdio>

#include "inertia/world.h"

using namespace inertia;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            std::printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                         \
    do {                                                                          \
        ++g_checks;                                                               \
        const double a_ = (actual), e_ = (expected), t_ = (tol);                  \
        if (!(std::fabs(a_ - e_) <= t_)) {                                        \
            ++g_failures;                                                         \
            std::printf("    FAIL %s:%d  %s = %g, expected %g +/- %g\n", __FILE__, \
                        __LINE__, #actual, a_, e_, t_);                           \
        }                                                                         \
    } while (0)

static void run(World& w, float seconds) {
    const int steps = static_cast<int>(std::lround(seconds / w.fixedDt));
    for (int i = 0; i < steps; ++i) w.step(w.fixedDt);
}

static bool allFinite(const World& w) {
    for (const Body& b : w.bodies())
        if (!isFinite(b.position) || !isFinite(b.velocity) || !isFinite(b.angularVelocity))
            return false;
    return true;
}

// ---------------------------------------------------------------------------

static void testMath() {
    const Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    CHECK_NEAR(length(cross(x, y) - z), 0.0, 1e-6);

    // Right-handed: +90 degrees about Y takes +X to -Z.
    const Vec3 r = Quat::fromAxisAngle(y, kPi * 0.5f).rotate(x);
    CHECK_NEAR(length(r - Vec3{0, 0, -1}), 0.0, 1e-5);

    // Spinning at pi rad/s for one second is half a turn.
    Quat q;
    for (int i = 0; i < 1000; ++i) q = q.integrated({0, kPi, 0}, 0.001f);
    CHECK_NEAR(length(q.rotate(x) - Vec3{-1, 0, 0}), 0.0, 1e-2);

    CHECK(isFinite(normalized(Vec3{})));  // zero vector must not produce NaN
}

// v = g t exactly; position lags the textbook 1/2 g t^2 by a known amount.
static void testFreeFall() {
    World w;
    const BodyId id = w.addBody(Body::makeSphere({0, 100, 0}, 0.5f));
    const float t = 2.0f;
    run(w, t);

    const float g = 9.81f;
    CHECK_NEAR(w.body(id).velocity.y, -g * t, 1e-2);

    // Semi-implicit Euler is first-order accurate: it overshoots the analytic
    // drop by exactly 1/2 g t dt. Check for that error, not for zero error.
    const float analytic = 100.0f - 0.5f * g * t * t;
    const float expectedError = 0.5f * g * t * w.fixedDt;
    CHECK_NEAR(w.body(id).position.y, analytic - expectedError, 1e-2);
}

// Drag balances weight at v_t = sqrt(2 m g / (rho Cd A)).
static void testTerminalVelocity() {
    World w;
    w.airDensity = 1.2f;
    const float r = 0.1f;
    const BodyId id = w.addBody(Body::makeSphere({0, 0, 0}, r, 25.0f));
    run(w, 15.0f);

    const float m = w.body(id).mass();
    const float area = kPi * r * r;
    const float vt = std::sqrt(2.0f * m * 9.81f / (1.2f * 0.47f * area));
    CHECK_NEAR(-w.body(id).velocity.y, vt, vt * 0.01);
}

// Dropped from height h with restitution e, a ball rebounds to e^2 h.
static void testBounceHeight() {
    World w;
    Body ground = Body::makePlane({0, 1, 0}, 0);
    ground.restitution = 0.0f;  // mixing takes the max, so the ball's e applies
    w.addBody(ground);
    const float r = 0.5f, h = 5.0f, e = 0.5f;
    Body ball = Body::makeSphere({0, h + r, 0}, r);
    ball.restitution = e;
    const BodyId id = w.addBody(ball);

    bool bounced = false;
    float peak = 0.0f;
    for (int i = 0; i < 600; ++i) {
        w.step(w.fixedDt);
        const Body& b = w.body(id);
        if (!bounced && b.velocity.y > 0.0f) bounced = true;
        if (bounced) {
            if (b.velocity.y < 0.0f) break;
            peak = std::fmax(peak, b.position.y - r);
        }
    }
    CHECK(bounced);
    CHECK_NEAR(peak, e * e * h, e * e * h * 0.08);
}

// 1D elastic collision, unequal masses, compared with the closed form.
static void testElasticCollision() {
    World w;
    w.gravity = {};
    Body a = Body::makeSphere({-2, 0, 0}, 0.5f, 1000.0f);
    Body b = Body::makeSphere({2, 0, 0}, 0.5f, 3000.0f);
    a.restitution = b.restitution = 1.0f;
    a.friction = b.friction = 0.0f;
    a.velocity = {4, 0, 0};
    b.velocity = {-1, 0, 0};
    const BodyId ia = w.addBody(a), ib = w.addBody(b);

    const float m1 = a.mass(), m2 = b.mass(), u1 = 4.0f, u2 = -1.0f;
    const Vec3 p0 = w.linearMomentum();
    const float e0 = w.kineticEnergy();
    run(w, 2.0f);

    const float v1 = ((m1 - m2) * u1 + 2.0f * m2 * u2) / (m1 + m2);
    const float v2 = ((m2 - m1) * u2 + 2.0f * m1 * u1) / (m1 + m2);
    CHECK_NEAR(w.body(ia).velocity.x, v1, 1e-3);
    CHECK_NEAR(w.body(ib).velocity.x, v2, 1e-3);
    CHECK_NEAR(length(w.linearMomentum() - p0), 0.0, length(p0) * 1e-4);
    CHECK_NEAR(w.kineticEnergy(), e0, e0 * 1e-3);
}

// Glancing hit with friction: momentum is still conserved, energy never grows.
static void testObliqueCollision() {
    World w;
    w.gravity = {};
    Body a = Body::makeSphere({-2, 0.4f, 0.1f}, 0.5f);
    Body b = Body::makeSphere({2, 0, 0}, 0.7f);
    a.velocity = {5, 0, 0};
    b.angularVelocity = {0, 0, 6};
    w.addBody(a);
    w.addBody(b);

    const Vec3 p0 = w.linearMomentum();
    const float e0 = w.kineticEnergy();
    run(w, 2.0f);

    CHECK(lengthSq(w.body(1).velocity) > 0.01f);  // they did actually collide
    CHECK_NEAR(length(w.linearMomentum() - p0), 0.0, length(p0) * 1e-3);
    CHECK(w.kineticEnergy() <= e0 * 1.0001f);
}

// A ball placed on the floor stays there: no sinking, no jitter.
static void testRestingContact() {
    World w;
    w.addBody(Body::makePlane({0, 1, 0}, 0));
    const float r = 0.5f;
    const BodyId id = w.addBody(Body::makeSphere({0, r, 0}, r));
    run(w, 5.0f);
    CHECK_NEAR(w.body(id).position.y, r, 0.01);
    CHECK(length(w.body(id).velocity) < 0.02f);
}

// A sliding ball is spun up by friction until it rolls without slipping.
// Angular momentum about the contact point is conserved, which gives the
// classic result v_final = 5/7 v0 for a solid sphere.
static void testSlidingToRolling() {
    World w;
    w.addBody(Body::makePlane({0, 1, 0}, 0));
    const float r = 0.5f, v0 = 7.0f;
    Body ball = Body::makeSphere({0, r, 0}, r);
    ball.velocity = {v0, 0, 0};
    ball.restitution = 0.0f;
    const BodyId id = w.addBody(ball);
    run(w, 4.0f);

    const Body& b = w.body(id);
    CHECK_NEAR(b.velocity.x, v0 * 5.0f / 7.0f, 0.05);
    // Rolling without slipping: v = w r. Moving +X spins about -Z.
    CHECK_NEAR(-b.angularVelocity.z * r, b.velocity.x, 0.02);
}

// Rolling resistance mu_r slows a rolling solid sphere at a = 5/7 mu_r g
// (torque mu_r N r against inertia 7/5 m r^2 about the contact point), and
// then brings it to a complete stop.
static void testRollingResistance() {
    World w;
    w.addBody(Body::makePlane({0, 1, 0}, 0));
    const float r = 0.5f, v0 = 5.0f, mu = 0.05f;
    Body ball = Body::makeSphere({0, r, 0}, r);
    ball.velocity = {v0, 0, 0};
    ball.angularVelocity = {0, 0, -v0 / r};  // already rolling
    ball.restitution = 0.0f;
    ball.rollingFriction = mu;
    const BodyId id = w.addBody(ball);

    run(w, 4.0f);
    CHECK_NEAR(w.body(id).velocity.x, v0 - (5.0f / 7.0f) * mu * 9.81f * 4.0f, 0.05);
    run(w, 16.0f);
    CHECK(length(w.body(id).velocity) < 0.01f);
    CHECK(length(w.body(id).angularVelocity) < 0.02f);
}

// Stress test: a pile of balls in a walled box must settle, not explode.
static void testPileSettles() {
    World w;
    w.addBody(Body::makePlane({0, 1, 0}, 0));
    w.addBody(Body::makePlane({1, 0, 0}, -2));
    w.addBody(Body::makePlane({-1, 0, 0}, -2));
    w.addBody(Body::makePlane({0, 0, 1}, -2));
    w.addBody(Body::makePlane({0, 0, -1}, -2));
    for (int i = 0; i < 64; ++i) {
        const float x = -1.2f + 0.8f * static_cast<float>(i % 4);
        const float z = -1.2f + 0.8f * static_cast<float>((i / 4) % 4);
        const float y = 1.0f + 0.9f * static_cast<float>(i / 16) + 0.01f * static_cast<float>(i);
        Body b = Body::makeSphere({x + 0.03f * static_cast<float>(i % 3), y, z}, 0.35f);
        b.restitution = 0.3f;
        w.addBody(b);
    }
    run(w, 12.0f);

    CHECK(allFinite(w));
    float maxSpeed = 0.0f, lowest = 1e9f;
    for (const Body& b : w.bodies()) {
        if (b.isStatic()) continue;
        maxSpeed = std::fmax(maxSpeed, length(b.velocity));
        lowest = std::fmin(lowest, b.position.y - b.radius);
        CHECK(std::fabs(b.position.x) < 2.0f && std::fabs(b.position.z) < 2.0f);
    }
    CHECK(maxSpeed < 0.3f);
    CHECK(lowest > -0.03f);
}

// Edge case: two spheres spawned at the same point have no separating
// direction. The engine must pick one instead of dividing by zero.
static void testCoincidentSpheres() {
    World w;
    w.gravity = {};
    w.addBody(Body::makeSphere({1, 2, 3}, 0.5f));
    w.addBody(Body::makeSphere({1, 2, 3}, 0.5f));
    run(w, 2.0f);
    CHECK(allFinite(w));
    CHECK(length(w.body(0).position - w.body(1).position) > 0.9f);
}

// The accumulator runs whole fixed steps and survives a huge frame time.
static void testFixedTimestep() {
    World w;
    w.addBody(Body::makeSphere({0, 0, 0}, 0.5f));
    CHECK(w.update(w.fixedDt * 0.5f) == 0);
    CHECK(w.update(w.fixedDt * 0.6f) == 1);
    CHECK(w.update(100.0f) == w.maxStepsPerUpdate);
    CHECK(w.update(w.fixedDt * 0.5f) == 0);  // backlog was dropped
}

int main() {
    struct Test { const char* name; void (*fn)(); };
    const Test tests[] = {
        {"math", testMath},
        {"free fall", testFreeFall},
        {"terminal velocity", testTerminalVelocity},
        {"bounce height", testBounceHeight},
        {"elastic collision", testElasticCollision},
        {"oblique collision", testObliqueCollision},
        {"resting contact", testRestingContact},
        {"sliding to rolling", testSlidingToRolling},
        {"rolling resistance", testRollingResistance},
        {"pile settles", testPileSettles},
        {"coincident spheres", testCoincidentSpheres},
        {"fixed timestep", testFixedTimestep},
    };
    for (const Test& t : tests) {
        const int before = g_failures;
        t.fn();
        std::printf("[%s] %s\n", g_failures == before ? " ok " : "FAIL", t.name);
    }
    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
