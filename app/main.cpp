// Sandbox: a window onto the engine. All physics lives in engine/; this file
// only handles input, the camera and drawing.
#include <cstdlib>
#include <cstring>
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "inertia/world.h"

namespace {

constexpr float kArenaHalf = 8.0f;    // floor is 16 x 16 m
constexpr float kArenaHeight = 10.0f;

const Color kPalette[] = {
    {230, 97, 76, 255}, {242, 177, 52, 255}, {86, 180, 211, 255},
    {120, 200, 120, 255}, {170, 130, 220, 255}, {240, 240, 235, 255},
};
constexpr int kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

Vector3 toRl(inertia::Vec3 v) { return {v.x, v.y, v.z}; }
inertia::Vec3 fromRl(Vector3 v) { return {v.x, v.y, v.z}; }

float frand(float lo, float hi) {
    return lo + (hi - lo) * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX));
}

void addArena(inertia::World& world) {
    using inertia::Body;
    world.addBody(Body::makePlane({0, 1, 0}, 0));
    world.addBody(Body::makePlane({1, 0, 0}, -kArenaHalf));
    world.addBody(Body::makePlane({-1, 0, 0}, -kArenaHalf));
    world.addBody(Body::makePlane({0, 0, 1}, -kArenaHalf));
    world.addBody(Body::makePlane({0, 0, -1}, -kArenaHalf));
}

void spawnBalls(inertia::World& world, int count) {
    for (int i = 0; i < count; ++i) {
        const float r = frand(0.25f, 0.6f);
        inertia::Body b = inertia::Body::makeSphere(
            {frand(-4.0f, 4.0f), frand(4.0f, 9.0f), frand(-4.0f, 4.0f)}, r);
        b.restitution = frand(0.2f, 0.8f);
        b.rollingFriction = 0.03f;
        b.userData = static_cast<std::uint32_t>(std::rand() % kPaletteSize);
        world.addBody(b);
    }
}

void resetScene(inertia::World& world) {
    world.clear();
    addArena(world);
    spawnBalls(world, 60);
}

void drawBall(const inertia::Body& b) {
    const inertia::Quat& o = b.orientation;
    const Matrix rot = QuaternionToMatrix(Quaternion{o.x, o.y, o.z, o.w});
    rlPushMatrix();
    rlTranslatef(b.position.x, b.position.y, b.position.z);
    rlMultMatrixf(MatrixToFloat(rot));
    DrawSphereEx({0, 0, 0}, b.radius, 10, 14, kPalette[b.userData % kPaletteSize]);
    // The wireframe turns with the body, so you can see it spin.
    DrawSphereWires({0, 0, 0}, b.radius * 1.003f, 5, 8, Fade(BLACK, 0.35f));
    rlPopMatrix();
}

}  // namespace

int main(int argc, char** argv) {
    // `sandbox --smoke out.png` renders a few seconds, saves a screenshot and
    // exits. Used to check the build without a person at the keyboard.
    const char* smokeShot = (argc == 3 && std::strcmp(argv[1], "--smoke") == 0) ? argv[2] : nullptr;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Inertia3D - physics sandbox");
    SetTargetFPS(smokeShot ? 0 : 120);

    inertia::World world;
    resetScene(world);

    // Orbit camera state.
    float yaw = 0.7f, pitch = 0.45f, distance = 26.0f;
    const Vector3 target{0, 2.5f, 0};
    Camera3D camera{};
    camera.target = target;
    camera.up = {0, 1, 0};
    camera.fovy = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    bool paused = false, showContacts = false, showVelocity = false, gravityOn = true;
    double stepMs = 0.0;
    int frame = 0;

    while (!WindowShouldClose()) {
        // ---- Input ---------------------------------------------------------
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 d = GetMouseDelta();
            yaw -= d.x * 0.005f;
            pitch = Clamp(pitch + d.y * 0.005f, 0.05f, 1.5f);
        }
        distance = Clamp(distance - GetMouseWheelMove() * 1.5f, 6.0f, 60.0f);
        camera.position = {target.x + distance * cosf(pitch) * sinf(yaw),
                           target.y + distance * sinf(pitch),
                           target.z + distance * cosf(pitch) * cosf(yaw)};

        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_C)) showContacts = !showContacts;
        if (IsKeyPressed(KEY_V)) showVelocity = !showVelocity;
        if (IsKeyPressed(KEY_R)) resetScene(world);
        if (IsKeyPressed(KEY_SPACE)) spawnBalls(world, 40);
        if (IsKeyPressed(KEY_T)) world.airDensity = world.airDensity > 0.0f ? 0.0f : 1.2f;
        if (IsKeyPressed(KEY_G)) {
            gravityOn = !gravityOn;
            world.gravity = gravityOn ? inertia::Vec3{0, -9.81f, 0} : inertia::Vec3{};
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            // Throw a heavy ball from the camera through the cursor.
            const Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            inertia::Body b = inertia::Body::makeSphere(
                fromRl(Vector3Add(ray.position, Vector3Scale(ray.direction, 2.0f))), 0.5f, 4000.0f);
            b.velocity = fromRl(Vector3Scale(ray.direction, 30.0f));
            b.rollingFriction = 0.03f;
            b.userData = 0;
            world.addBody(b);
        }

        // ---- Simulate ------------------------------------------------------
        const double t0 = GetTime();
        int steps = 0;
        if (smokeShot) {
            world.step(world.fixedDt);
            steps = 1;
        } else if (!paused) {
            steps = world.update(GetFrameTime());
        } else if (IsKeyPressed(KEY_N)) {
            world.step(world.fixedDt);  // advance exactly one step
            steps = 1;
        }
        if (steps > 0) stepMs = (GetTime() - t0) * 1000.0 / steps;

        // ---- Draw ----------------------------------------------------------
        BeginDrawing();
        ClearBackground({24, 26, 32, 255});
        BeginMode3D(camera);

        DrawPlane({0, 0, 0}, {kArenaHalf * 2, kArenaHalf * 2}, {52, 56, 66, 255});
        DrawGrid(static_cast<int>(kArenaHalf * 2), 1.0f);
        DrawCubeWires({0, kArenaHeight * 0.5f, 0}, kArenaHalf * 2, kArenaHeight, kArenaHalf * 2,
                      Fade(RAYWHITE, 0.25f));

        // Blob shadows: a dark disc under each ball that fades with height.
        // Cheap, and it makes height above the floor readable at a glance.
        for (const inertia::Body& b : world.bodies()) {
            if (b.shape != inertia::ShapeType::Sphere) continue;
            const float height = b.position.y - b.radius;
            const float alpha = 0.45f * Clamp(1.0f - height / 8.0f, 0.0f, 1.0f);
            DrawCylinder({b.position.x, 0.01f, b.position.z}, b.radius, b.radius, 0.0f, 16,
                         Fade(BLACK, alpha));
        }

        for (const inertia::Body& b : world.bodies()) {
            if (b.shape != inertia::ShapeType::Sphere) continue;
            drawBall(b);
            if (showVelocity)
                DrawLine3D(toRl(b.position), toRl(b.position + b.velocity * 0.15f), YELLOW);
        }
        if (showContacts) {
            for (const inertia::Contact& c : world.contacts()) {
                DrawSphere(toRl(c.point), 0.05f, RED);
                DrawLine3D(toRl(c.point), toRl(c.point + c.normal * 0.4f), RED);
            }
        }
        EndMode3D();

        // Text sizes are multiples of 10 because raylib's built-in font is a
        // 10 px bitmap; other sizes scale unevenly and look ragged.
        DrawRectangle(10, 10, 470, 160, Fade(BLACK, 0.55f));
        DrawText(TextFormat("%d fps   %.3f ms / physics step", GetFPS(), stepMs), 22, 20, 20, RAYWHITE);
        DrawText(TextFormat("bodies %d   contacts %d   pairs %d",
                            static_cast<int>(world.bodies().size()),
                            static_cast<int>(world.contacts().size()),
                            static_cast<int>(world.pairsTested())), 22, 46, 20, RAYWHITE);
        DrawText(TextFormat("kinetic energy %.0f J", world.kineticEnergy()), 22, 72, 20, RAYWHITE);
        DrawText(TextFormat("gravity %s   air drag %s%s", gravityOn ? "on" : "off",
                            world.airDensity > 0.0f ? "on" : "off", paused ? "   PAUSED" : ""),
                 22, 98, 20, paused ? YELLOW : RAYWHITE);
        DrawText("stage 1: spheres, impulses, friction", 22, 132, 20, GRAY);
        DrawRectangle(0, GetScreenHeight() - 64, GetScreenWidth(), 64, Fade(BLACK, 0.55f));
        DrawText("Left click: throw    Right drag: orbit    Wheel: zoom    Space: more balls    R: reset",
                 14, GetScreenHeight() - 56, 20, LIGHTGRAY);
        DrawText("G: gravity    T: air drag    P: pause    N: single step    C: contacts    V: velocities",
                 14, GetScreenHeight() - 30, 20, LIGHTGRAY);
        const bool smokeDone = smokeShot && ++frame == 600;
        if (smokeDone) {
            rlDrawRenderBatchActive();  // flush pending draws so the capture is complete
            TakeScreenshot(smokeShot);
        }
        EndDrawing();
        if (smokeDone) break;
    }

    CloseWindow();
    return 0;
}
