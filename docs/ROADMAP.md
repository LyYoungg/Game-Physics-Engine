# Inertia3D roadmap

Six stages. Each one ends with a build that runs, tests that pass, and a short
note in `docs/` explaining what was built and why.

| Stage | Adds | Skill it demonstrates |
| --- | --- | --- |
| 1. Core (done) | Vectors, quaternions, rigid bodies, forces, fixed timestep, sphere and plane collision, impulse solver with friction, sandbox | Physics fundamentals, systems thinking |
| 2. Boxes | Oriented boxes, separating axis test, contact manifolds by face clipping, warm starting, gyroscopic term, stable stacking | Collision detection edge cases |
| 3. Broad phase | Uniform grid, then a dynamic bounding volume tree; benchmark against the stage 1 brute force | Spatial partitioning, algorithmic optimization |
| 4. Memory and scale | Pool allocator with generation handles, body removal, sleeping islands, data layout for cache efficiency, a frame profiler | Memory management |
| 5. Robustness | Continuous collision detection against tunnelling, full debug-draw overlay (grid cells, tree nodes, manifolds), a written debugging log of real bugs found | Debugging and reasoning |
| 6. Game | A knock-down-the-tower game mode with scoring and levels, alongside the sandbox; tagged release with a downloadable executable | Shipping |

## Known limits of stage 1

- Broad phase tests every pair, so cost grows with the square of the body count. The sandbox HUD shows "pairs tested" so the stage 3 improvement can be measured.
- No continuous collision detection: a small ball moving faster than its own diameter per step (120 steps per second) can pass through another ball. Planes are half-spaces, so nothing escapes the arena.
- Bodies cannot be removed individually yet, only cleared.
- Contacts are solved from scratch each step (no warm starting), which is fine for spheres but will not hold a tall box stack.
