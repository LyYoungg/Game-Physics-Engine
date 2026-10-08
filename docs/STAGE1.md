# Stage 1 notes: the core loop

Read the code in this order: `math.h`, `body.h`, `collision.cpp`, `world.cpp`.
`World::step` is the whole engine in six lines; everything else serves it.

## The step

```
integrateVelocities   forces -> velocity          F = m a
detectCollisions      which bodies overlap, where, how deep
buildConstraints      precompute per-contact solver data
solveVelocities x8    impulses that stop bodies approaching
integratePositions    velocity -> position and orientation
correctPositions      remove leftover overlap
```

## Decisions worth being able to explain

**Fixed timestep with an accumulator.** Physics always advances in 1/120 s
slices regardless of frame rate. A variable step makes the same scene behave
differently on a fast and a slow PC, and large steps make collisions unstable.
The frame time is clamped and the number of steps per frame capped, so one slow
frame cannot cause more work on the next and snowball.

**Semi-implicit Euler.** Velocity is updated first, then the *new* velocity
moves the body. Plain explicit Euler uses the old velocity and steadily gains
energy (orbits spiral outward, springs explode). This variant costs the same and
keeps energy bounded. It is still first-order: the free-fall test checks for its
exact, predictable error of `1/2 g t dt` instead of pretending it is zero.

**Inverse mass.** Bodies store `1/m` and `I^-1`. The solver only ever divides by
mass, and `1/m = 0` describes an immovable body with no special-case branches.

**Quaternions for orientation.** No gimbal lock, four floats, and drift is fixed
by one renormalisation.

**Impulses, not forces, for contact.** A collision changes velocity over a time
far shorter than a step. The solver computes the change in momentum directly:

```
j = (target_vn - vn) / (1/ma + 1/mb + (ra x n) . Ia^-1 (ra x n) + (rb x n) . Ib^-1 (rb x n))
```

The denominator is the "effective mass": how hard the pair is to accelerate at
that point, including the fact that an off-centre hit also spins the body.

**Accumulated impulse clamping.** The solver clamps the running total impulse to
be non-negative, not each iteration's change. A contact can never pull, but a
later iteration may take back part of what an earlier one over-applied. Clamping
each change separately makes piles jitter.

**Friction after the fact.** The tangential impulse is limited to `mu` times the
normal impulse (Coulomb's law). That one rule produces sliding, the transition
to rolling, and rolling without slipping; the `5/7 v0` test proves it.

**Rolling resistance.** Friction alone never stops a rolling ball, because a
ball that rolls without slipping has zero sliding velocity at the contact. Real
balls stop because the contact patch deforms. The engine models that as a
torque opposing the spin, capped at `mu_r * N * r`, solved and clamped the same
way as friction.

**Restitution threshold.** Below 0.5 m/s closing speed, no bounce is requested.
Without it, the small downward velocity gravity adds each step would be bounced
back forever and nothing would come to rest.

**Slop and partial correction.** Overlap under 5 mm is tolerated and only 40% of
the remainder is removed per step. Removing all of it makes resting bodies pop
in and out of contact.

## Edge cases handled

- Coincident sphere centres: no direction to separate along, so `normalized()` returns a fallback axis instead of dividing by zero.
- Two static bodies are never tested against each other.
- Sphere-plane contact uses the sphere's lowest point, keeping the lever arm exactly one radius so rolling satisfies `v = w r`.
- The solver's scratch arrays are members that keep their capacity, so a step allocates no memory once the scene is warm.

## Try this

1. Press `P`, then `N` repeatedly with `C` on, and watch contacts appear a step at a time.
2. In `world.cpp`, set `kRestitutionThreshold` to 0 and watch a resting pile shimmer.
3. Set `solverIterations` to 1 and watch the pile turn spongy.
