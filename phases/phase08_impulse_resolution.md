# Phase 08: Impulse Solver & Resolution 💥

This phase implements constraint-based Sequential Impulse Resolution. It updates velocities at contact points based on mass, inertia, restitution (bounciness), and friction, and applies penetration correction to prevent object sinking.

## 📁 Filenames & Directory Structure

* **Impulse Solver Engine**:
  * `include/core/solver.hpp` & `src/core/solver.cpp` — Solves manifolds. Called inside `World::Step`.

## ⚙️ Core Variables

### `Solver` Configs
* `int VelocityIterations = 8` — Iteration loop passes for resolving velocity constraints.
* `int PositionIterations = 3` — Iteration loop passes for resolving positional drift (sinking).
* `const float PENETRATION_ALLOWANCE = 0.01f` — "Slop" buffer: overlap allowed before position correction starts (prevents jitter).
* `const float POSITION_CORRECTION_PERCENT = 0.2f` — Baumgarte stabilization factor `[0.2, 0.8]` defining how fast penetration is resolved.

### `ContactSolver` Helper State (cached variables per contact point to speed up solver iterations)
* `Vec2 RelativePositionA, RelativePositionB` — Vector from center-of-mass to contact point `(r_A, r_B)`.
* `float NormalMass` — Effective mass projected onto normal axis.
* `float TangentMass` — Effective mass projected onto tangent axis.
* `float Bias` — Baumgarte position correction factor.
* `float AccumulatedNormalImpulse` — Total normal impulse applied (used for clamping).
* `float AccumulatedTangentImpulse` — Total friction impulse applied.

## 🏛️ Engine Component Role

* **Constraint Solver**: Enforces physical boundaries. It modifies body velocities iteratively until the relative velocity along the contact normal is non-negative (non-penetrating) and friction opposes slip.

## 📝 Pseudo-code / Flow of Execution

For each frame, solving passes run over active manifolds:

### 1. Pre-Step (Calculate Cache & Baumgarte Bias)
```cpp
void Solver::PreStep(CollisionManifold& m, float dt) {
    RigidBody* A = m.BodyA;
    RigidBody* B = m.BodyB;

    for (int i = 0; i < m.ContactCount; ++i) {
        Vec2 cp = m.Contacts[i];
        Vec2 rA = cp - A->Position;
        Vec2 rB = cp - B->Position;
        
        // Save relative positions to cache
        m.SolverCache[i].rA = rA;
        m.SolverCache[i].rB = rB;

        // 1. Calculate Effective Mass along normal: 
        // InvMass_eff = InvMass_A + InvMass_B + ((r_A x n)^2 / I_A) + ((r_B x n)^2 / I_B)
        float rnA = Cross(rA, m.Normal);
        float rnB = Cross(rB, m.Normal);
        float invNormalMass = A->InvMass + B->InvMass + (rnA * rnA * A->InvInertia) + (rnB * rnB * B->InvInertia);
        m.SolverCache[i].NormalMass = invNormalMass > 0.0f ? 1.0f / invNormalMass : 0.0f;

        // 2. Calculate Effective Mass along tangent (friction axis)
        Vec2 tangent = Vec2(-m.Normal.y, m.Normal.x); // Orthogonal normal
        float rtA = Cross(rA, tangent);
        float rtB = Cross(rB, tangent);
        float invTangentMass = A->InvMass + B->InvMass + (rtA * rtA * A->InvInertia) + (rtB * rtB * B->InvInertia);
        m.SolverCache[i].TangentMass = invTangentMass > 0.0f ? 1.0f / invTangentMass : 0.0f;

        // 3. Baumgarte Position Correction Bias
        float penetration = std::max(0.0f, m.PenetrationDepth - PENETRATION_ALLOWANCE);
        m.SolverCache[i].Bias = (POSITION_CORRECTION_PERCENT / dt) * penetration;

        // 4. Elastic Restitution Bias
        Vec2 relativeVel = (B->Velocity + Cross(B->AngularVelocity, rB)) - 
                            (A->Velocity + Cross(A->AngularVelocity, rA));
        float normalVel = Dot(relativeVel, m.Normal);
        
        // If relative velocity is fast enough, apply bounciness
        float restitution = std::min(A->Material.Restitution, B->Material.Restitution);
        if (normalVel < -1.0f) {
            m.SolverCache[i].Bias += -restitution * normalVel;
        }
    }
}
```

### 2. Velocity Solver Iterations
This function runs `VelocityIterations` times per frame step:
```cpp
void Solver::ResolveVelocities(CollisionManifold& m) {
    RigidBody* A = m.BodyA;
    RigidBody* B = m.BodyB;
    Vec2 normal = m.Normal;
    Vec2 tangent = Vec2(-normal.y, normal.x);

    for (int i = 0; i < m.ContactCount; ++i) {
        auto& cache = m.SolverCache[i];

        // 1. Calculate Relative Contact Velocity
        Vec2 vA = A->Velocity + Cross(A->AngularVelocity, cache.rA);
        Vec2 vB = B->Velocity + Cross(B->AngularVelocity, cache.rB);
        Vec2 rv = vB - vA;

        // 2. Normal Impulse
        float vn = Dot(rv, normal);
        float lambda = -cache.NormalMass * (vn - cache.Bias);

        // Clamp accumulated impulse so bodies can only push, not pull
        float oldNormalImpulse = cache.AccumulatedNormalImpulse;
        cache.AccumulatedNormalImpulse = std::max(oldNormalImpulse + lambda, 0.0f);
        lambda = cache.AccumulatedNormalImpulse - oldNormalImpulse;

        // Apply impulse to change velocities
        Vec2 impulseVec = normal * lambda;
        A->Velocity = A->Velocity - impulseVec * A->InvMass;
        A->AngularVelocity -= A->InvInertia * Cross(cache.rA, impulseVec);
        
        B->Velocity = B->Velocity + impulseVec * B->InvMass;
        B->AngularVelocity += B->InvInertia * Cross(cache.rB, impulseVec);

        // 3. Friction (Tangent) Impulse
        // Re-read velocities
        vA = A->Velocity + Cross(A->AngularVelocity, cache.rA);
        vB = B->Velocity + Cross(B->AngularVelocity, cache.rB);
        rv = vB - vA;

        float vt = Dot(rv, tangent);
        float lambdaT = -cache.TangentMass * vt;

        // Clamp using Coulomb's friction law: |tangent_impulse| <= normal_impulse * mu
        float mu = std::sqrt(A->Material.Friction * B->Material.Friction);
        float maxFriction = cache.AccumulatedNormalImpulse * mu;

        float oldTangentImpulse = cache.AccumulatedTangentImpulse;
        cache.AccumulatedTangentImpulse = std::clamp(oldTangentImpulse + lambdaT, -maxFriction, maxFriction);
        lambdaT = cache.AccumulatedTangentImpulse - oldTangentImpulse;

        // Apply friction impulse
        Vec2 frictionVec = tangent * lambdaT;
        A->Velocity = A->Velocity - frictionVec * A->InvMass;
        A->AngularVelocity -= A->InvInertia * Cross(cache.rA, frictionVec);

        B->Velocity = B->Velocity + frictionVec * B->InvMass;
        B->AngularVelocity += B->InvInertia * Cross(cache.rB, frictionVec);
    }
}
```

### 3. Position Corrector
An alternative to Baumgarte stabilization is projecting position changes directly without changing velocity, which eliminates restitution bounce issues at low frames:
```cpp
void Solver::ResolvePositions(CollisionManifold& m) {
    RigidBody* A = m.BodyA;
    RigidBody* B = m.BodyB;

    float penetration = std::max(0.0f, m.PenetrationDepth - PENETRATION_ALLOWANCE);
    if (penetration <= 0.0f) return;

    // Direct translation projection factor
    float totalInvMass = A->InvMass + B->InvMass;
    if (totalInvMass <= 0.0f) return;

    Vec2 correction = m.Normal * (penetration / totalInvMass) * POSITION_CORRECTION_PERCENT;

    if (!A->IsStatic) A->Position = A->Position - correction * A->InvMass;
    if (!B->IsStatic) B->Position = B->Position + correction * B->InvMass;
}
```
