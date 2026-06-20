# Phase 10: Fluid Simulation (SPH) 🌊

This phase implements a real-time Smoothed Particle Hydrodynamics (SPH) fluid simulation. Fluid is represented as particles that experience density-driven pressure forces, viscosity shear forces, and interact with static/dynamic rigid body colliders.

## 📁 Filenames & Directory Structure

* **SPH Simulator Engine**:
  * `include/core/fluid_sim.hpp` & `src/core/fluid_sim.cpp` — Fluid simulation loop and kernels.
* **SPH Shaders**:
  * `shaders/fluid.vs` & `shaders/fluid.fs` — Displays particles as soft radial drops instead of solid circles.

## ⚙️ Core Variables

### `FluidParticle` Struct
* `Vec2 Position`, `Velocity`, `Force` — Particle dynamics.
* `float Density` — Local density calculated from neighbors.
* `float Pressure` — Pressure calculated from local density vs rest density.

### `FluidConfig` Settings
* `float ParticleRadius = 0.15f` — Collision radius.
* `float H = 0.4f` — Smoothing length (radius of influence for kernel function $W$).
* `float RestDensity = 1000.0f` — Rest density $\rho_0$.
* `float GasConstant = 2000.0f` — Stiffness constant $k$ (determines compressibility).
* `float Viscosity = 250.0f` — Viscosity coefficient $\mu$ (thickness of fluid).
* `Vec2 Gravity = Vec2(0.0f, -9.81f)` — Gravity force.

## 🏛️ Engine Component Role

* **SPH Engine**: Computes fluid particle interactions. Uses a specialized spatial hash to quickly find nearby fluid particles.
* **Fluid-Rigid Coupling**: Checks fluid particles against rigid body boundaries (e.g. circle/box colliders), applying boundary impulse responses to the fluid and equal-and-opposite forces to dynamic rigid bodies.

## 📝 Pseudo-code / Flow of Execution

### 1. SPH Mathematical Kernels
We use Poly6 for density and Spiky kernels for pressure gradients:
```cpp
// Poly6 Kernel for density computation
float KernelPoly6(float rSq, float h) {
    if (rSq < 0.0f || rSq >= h * h) return 0.0f;
    float term = h * h - rSq;
    return (315.0f / (64.0f * M_PI * std::pow(h, 9))) * term * term * term;
}

// Spiky Kernel Gradient factor for pressure forces
Vec2 KernelSpikyGradient(const Vec2& rVec, float r, float h) {
    if (r <= 0.0f || r >= h) return Vec2(0.0f, 0.0f);
    float term = h - r;
    float scalar = -45.0f / (M_PI * std::pow(h, 6)) * term * term;
    return rVec * (scalar / r);
}

// Laplacian of Viscosity Kernel
float KernelViscosityLaplacian(float r, float h) {
    if (r >= h) return 0.0f;
    return (45.0f / (M_PI * std::pow(h, 6))) * (h - r);
}
```

### 2. SPH Simulation Step
```cpp
void FluidSim::Step(float dt, const std::vector<RigidBody*>& rigidBodies) {
    // Step 1: Broadphase - Hash particles into spatial grid
    HashParticles();

    // Step 2: Compute Density & Pressure
    for (size_t i = 0; i < Particles.size(); ++i) {
        float densitySum = 0.0f;
        auto neighbors = GetNeighbors(Particles[i].Position);

        for (int idx : neighbors) {
            Vec2 diff = Particles[i].Position - Particles[idx].Position;
            float rSq = diff.LengthSq();
            densitySum += Mass * KernelPoly6(rSq, H);
        }

        Particles[i].Density = std::max(densitySum, RestDensity); // Clamp to rest density
        
        // Equation of State (Tait Equation variant)
        // P = k * (rho - rho_0)
        Particles[i].Pressure = GasConstant * (Particles[i].Density - RestDensity);
    }

    // Step 3: Compute Forces (Pressure + Viscosity + Gravity)
    for (size_t i = 0; i < Particles.size(); ++i) {
        Vec2 pressureForce(0.0f, 0.0f);
        Vec2 viscosityForce(0.0f, 0.0f);
        auto neighbors = GetNeighbors(Particles[i].Position);

        for (int idx : neighbors) {
            if (idx == (int)i) continue;

            Vec2 diff = Particles[i].Position - Particles[idx].Position;
            float r = diff.Length();
            if (r >= H || r < 0.0001f) continue;

            // Pressure Force: f = -Sum[ m * (P_i + P_j) / (2 * rho_j) * GradW ]
            float pTerm = (Particles[i].Pressure + Particles[idx].Pressure) / (2.0f * Particles[idx].Density);
            pressureForce = pressureForce - KernelSpikyGradient(diff, r, H) * (Mass * pTerm);

            // Viscosity Force: f = mu * Sum[ m * (v_j - v_i) / rho_j * LapW ]
            Vec2 vDiff = Particles[idx].Velocity - Particles[i].Velocity;
            float vTerm = KernelViscosityLaplacian(r, H) / Particles[idx].Density;
            viscosityForce = viscosityForce + vDiff * (Viscosity * Mass * vTerm);
        }

        Vec2 gravityForce = Gravity * Particles[i].Density;
        Particles[i].Force = pressureForce + viscosityForce + gravityForce;
    }

    // Step 4: Integrate Particles & Resolve Rigid Obstacle Collisions
    for (size_t i = 0; i < Particles.size(); ++i) {
        // Semi-implicit Euler integration
        Particles[i].Velocity = Particles[i].Velocity + (Particles[i].Force / Particles[i].Density) * dt;
        Particles[i].Position = Particles[i].Position + Particles[i].Velocity * dt;

        // Resolve boundaries & rigid body obstacles
        ResolveObstacles(Particles[i], rigidBodies);
    }
}
```

### 3. Fluid-Rigid Boundary Penetration
```cpp
void FluidSim::ResolveObstacles(FluidParticle& p, const std::vector<RigidBody*>& rigidBodies) {
    for (RigidBody* body : rigidBodies) {
        // Run a custom simplified collision check between fluid particle and rigid body
        if (body->Type == ShapeType::CIRCLE) {
            Vec2 diff = p.Position - body->Position;
            float dist = diff.Length();
            float rLimit = body->Radius + ParticleRadius;
            
            if (dist < rLimit) {
                Vec2 normal = diff.Normalized();
                float penetration = rLimit - dist;
                
                // Repel particle from surface
                p.Position = p.Position + normal * penetration;
                
                // Mirror velocity along normal (elastic collision)
                float vn = Dot(p.Velocity, normal);
                if (vn < 0.0f) {
                    p.Velocity = p.Velocity - normal * (vn * 1.5f); // 1.5 multiplier for bounce
                }
                
                // Apply equal and opposite reaction force to the rigid body if dynamic
                if (!body->IsStatic) {
                    Vec2 force = normal * (penetration * 50.0f); // Stiffness modifier
                    body->ApplyForceAtPoint(force, p.Position);
                }
            }
        }
        // Additional Box collision checks are handled similarly using Box SDFs
    }
}
```
