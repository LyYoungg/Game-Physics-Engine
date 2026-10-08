# Inertia3D

A 3D rigid-body physics engine written from scratch in C++17, with a playable
sandbox built on [raylib](https://www.raylib.com/). raylib only opens the
window and draws; every piece of physics (integration, collision detection,
impulse resolution, friction) is implemented in `engine/` with no dependencies.

**Status: stage 1 of 6.** Spheres and static planes, with rotation, restitution
and Coulomb friction. See [docs/ROADMAP.md](docs/ROADMAP.md) for what comes next.

## What is implemented

| Area | Detail |
| --- | --- |
| Dynamics | Newton's second law for linear and angular motion, inertia tensors, quaternion orientation |
| Integration | Semi-implicit (symplectic) Euler on a fixed 120 Hz timestep with an accumulator |
| Forces | Gravity, quadratic aerodynamic drag `F = -1/2 rho Cd A |v| v` |
| Collision | Sphere-sphere and sphere-plane narrow phase, brute-force broad phase |
| Response | Sequential-impulse solver with accumulated-impulse clamping, restitution, Coulomb friction, rolling resistance, positional correction |
| Verification | 12 headless tests that compare the simulation with closed-form physics |

## Build and run

Requirements: CMake 3.16+, Git, and a C++17 compiler (on Windows, Visual Studio
with the "Desktop development with C++" workload).

Windows: double-click `build.bat`. It finds Visual Studio's compiler and bundled
CMake by itself, builds, runs the tests, starts the sandbox, and saves all
output to `build_log.txt`.

Any platform:

```
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The first configure downloads raylib. The sandbox is `build/Release/sandbox.exe`
on Windows and `build/sandbox` elsewhere.

No compiler installed? Push this repository to GitHub: the workflow in
`.github/workflows/build.yml` builds `sandbox.exe` on every push and attaches it to the run under
**Actions > build > Artifacts**. Pushing a tag such as `v0.1.0` also publishes
it as a release.

## Controls

| Input | Action |
| --- | --- |
| Left click | Throw a heavy ball through the cursor |
| Right drag / wheel | Orbit / zoom the camera |
| Space | Drop 40 more balls |
| G / T | Toggle gravity / air drag |
| P / N | Pause / advance one physics step |
| C / V | Draw contact points and normals / velocity vectors |
| R | Reset |

## Tests

`inertia_tests` needs no window. Each test checks a result you can derive by hand:

- Free fall reaches `v = g t`, and position shows exactly the integrator's known first-order error `1/2 g t dt`
- A falling ball in air settles at terminal velocity `sqrt(2 m g / (rho Cd A))`
- A ball with restitution `e` dropped from `h` rebounds to `e^2 h`
- A head-on elastic collision between unequal masses matches the closed-form velocities and conserves momentum and energy
- A sliding ball ends up rolling at `5/7` of its initial speed with `v = w r`
- Rolling resistance `mu_r` decelerates a rolling ball at `5/7 mu_r g` and then stops it completely
- A ball at rest stays at rest; a 64-ball pile settles without sinking or exploding
- Two spheres spawned at the same point separate without producing NaN

## Layout

```
engine/include/inertia/   math.h  body.h  collision.h  world.h
engine/src/               collision.cpp  world.cpp
app/main.cpp              raylib sandbox
tests/test_physics.cpp    headless tests
docs/                     roadmap and per-stage notes
```
