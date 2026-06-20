# Phase 06: Integrator ⚙️

This phase implements the numerical integrator (Semi-implicit Euler) that moves bodies under the influence of gravity, forces, torques, and damping. It also implements sleep mechanics to optimize simulation performance.

## 📁 Filenames & Directory Structure

* **Integrator Implementation**:
  * `include/core/integrator.hpp` & `src/core/integrator.cpp` — Math integration utilities.
  * In practice, these methods are driven inside the step loop in `src/core/world.cpp`.

## ⚙️ Core Variables

### Engine Configuration Constants
* `const float SLEEP_LINEAR_THRESHOLD = 0.05f` — Linear velocity limit below which a body may sleep.
* `const float SLEEP_ANGULAR_THRESHOLD = 0.05f` — Angular velocity limit below which a body may sleep.
* `const float SLEEP_TIME_REQUIREMENT = 0.5f` — Duration (in seconds) velocity must remain below threshold before sleeping.

### RigidBody State Additions
* `float SleepTimer` — Counts time spent below velocity thresholds.
* `Vec2 ForceAccumulator` — Accumulated external forces.
* `float TorqueAccumulator` — Accumulated torque forces.
* `float LinearDamping` — Linear air resistance / drag coefficient `[0.0, 1.0]`.
* `float AngularDamping` — Rotational air resistance coefficient `[0.0, 1.0]`.

## 🏛️ Engine Component Role

* **Equations of Motion Solver**: Resolves position and velocity updates over time. Semi-implicit Euler is preferred over Explicit Euler because it preserves energy much better and matches constraint-based impulse solvers.

## 📝 Pseudo-code / Flow of Execution

### 1. The Core Update Loop (`world.cpp`)
At each physics tick, the world processes bodies in two primary phases: integration of forces to velocity, followed by integration of velocity to position.

```cpp
void World::Step(float dt) {
    // Phase A: Integrate Forces (Generate Velocities)
    for (RigidBody* body : Bodies) {
        if (body->IsStatic || body->IsSleeping) continue;

        IntegrateForces(body, dt);
    }

    // Phase B: Resolve Constraints & Collisions (Updates Velocities)
    m_Broadphase.Update(Bodies);
    auto collisionPairs = m_Broadphase.GetPairs();
    m_Solver.ResolveCollisions(collisionPairs, dt);

    // Phase C: Integrate Velocities (Update Positions)
    for (RigidBody* body : Bodies) {
        if (body->IsStatic || body->IsSleeping) continue;

        IntegrateVelocities(body, dt);
        
        // Handle sleeping thresholds
        UpdateSleepState(body, dt);
    }
}
```

### 2. Integration Mechanics (`integrator.cpp`)
```cpp
void IntegrateForces(RigidBody* body, float dt) {
    // 1. Calculate Translational Acceleration (F = ma => a = F/m)
    Vec2 gravityForce = body->GravityScale * WorldGravity * body->Mass;
    Vec2 totalForce = body->ForceAccumulator + gravityForce;
    Vec2 acceleration = totalForce * body->InvMass;
    
    // Update Linear Velocity
    body->Velocity = body->Velocity + acceleration * dt;
    
    // Apply Linear Damping
    body->Velocity = body->Velocity * (1.0f / (1.0f + dt * body->LinearDamping));

    // 2. Calculate Rotational Acceleration (T = I*alpha => alpha = T/I)
    float angularAcceleration = body->TorqueAccumulator * body->InvInertia;
    
    // Update Angular Velocity
    body->AngularVelocity += angularAcceleration * dt;
    
    // Apply Angular Damping
    body->AngularVelocity *= (1.0f / (1.0f + dt * body->AngularDamping));
}

void IntegrateVelocities(RigidBody* body, float dt) {
    // Update position directly from modified velocities
    body->Position = body->Position + body->Velocity * dt;
    
    // Update angle of rotation
    body->Rotation += body->AngularVelocity * dt;

    // Reset force accumulators for the next frame
    body->ForceAccumulator = Vec2(0.0f, 0.0f);
    body->TorqueAccumulator = 0.0f;
}
```

### 3. Sleeping Optimization System
If rigid bodies are idle on the ground, keeping them active wastes CPU cycles.
```cpp
void UpdateSleepState(RigidBody* body, float dt) {
    float velSq = body->Velocity.LengthSq();
    float angVelSq = body->AngularVelocity * body->AngularVelocity;
    
    float linThresholdSq = SLEEP_LINEAR_THRESHOLD * SLEEP_LINEAR_THRESHOLD;
    float angThresholdSq = SLEEP_ANGULAR_THRESHOLD * SLEEP_ANGULAR_THRESHOLD;

    if (velSq < linThresholdSq && angVelSq < angThresholdSq) {
        body->SleepTimer += dt;
        if (body->SleepTimer >= SLEEP_TIME_REQUIREMENT) {
            body->IsSleeping = true;
            body->Velocity = Vec2(0.0f, 0.0f);
            body->AngularVelocity = 0.0f;
        }
    } else {
        body->SleepTimer = 0.0f;
        body->IsSleeping = false;
    }
}
```
*Note: Any collision, user interaction (mouse drag), or external impulse must immediately set `body->IsSleeping = false` and reset `body->SleepTimer = 0.0f` to wake the sleeping body and its neighbors.*
