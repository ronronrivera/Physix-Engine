# Phase 11: Physics Problem Scenarios 🧪

This phase implements a Scenario/Preset Factory to instantly reset the world state and spawn specific physics configurations (e.g. projectile motion, double pendulums, liquid dam breaks, stack collapses, and friction ramps) for debugging and user interaction.

## 📁 Filenames & Directory Structure

* **Scenario Controllers**:
  * `include/procgen/problem_factory.hpp` & `src/procgen/problem_factory.cpp` — Declares preset presets. Called by ImGui buttons.

## ⚙️ Core Variables

### `ProblemFactory` Settings
* `enum class ScenarioType` — Scenario ID (`PROJECTILE_MOTION`, `PENDULUM_CHAIN`, `FLUID_PRESSURE`, `STACK_COLLAPSE`, `INCLINED_PLANE`).

### Helper structures for Joints (needed for the Pendulum Scenario)
If implementing distance constraints for pendulums:
```cpp
struct DistanceJoint {
    RigidBody* BodyA;
    RigidBody* BodyB;
    Vec2 AnchorLocalA; // Offset from BodyA center
    Vec2 AnchorLocalB; // Offset from BodyB center
    float TargetDistance;
    float Stiffness = 1.0f; // Spring strength
};
```

## 🏛️ Engine Component Role

* **Scene Orquestrator**: Instantiates groups of rigid bodies and constraints, resetting the clock accumulator and emptying old buffers to start a fresh simulation run.

## 📝 Pseudo-code / Flow of Execution

```cpp
void ProblemFactory::LoadScenario(ScenarioType type, World& world, FluidSim& fluidSim) {
    // 1. Reset World and Fluid Sim states
    world.Clear();
    fluidSim.Clear();
    world.Gravity = Vec2(0.0f, -9.81f);

    switch (type) {
        case ScenarioType::PROJECTILE_MOTION:
            SetupProjectileScene(world);
            break;
        case ScenarioType::PENDULUM_CHAIN:
            SetupPendulumScene(world);
            break;
        case ScenarioType::FLUID_PRESSURE:
            SetupFluidDamScene(world, fluidSim);
            break;
        case ScenarioType::STACK_COLLAPSE:
            SetupStackCollapseScene(world);
            break;
        case ScenarioType::INCLINED_PLANE:
            SetupInclinedPlaneScene(world);
            break;
    }
}
```

### 1. Projectile Motion Setup
Tests high velocity integration and basic projectile trajectory curves.
```cpp
void SetupProjectileScene(World& world) {
    // Ground
    RigidBody* ground = new RigidBody();
    ground->Position = Vec2(0.0f, -5.0f);
    ground->Size = Vec2(100.0f, 2.0f);
    ground->IsStatic = true;
    ground->ComputeMassProperties();
    world.AddBody(ground);

    // Launch Cannonball (Circle)
    RigidBody* ball = new RigidBody();
    ball->Type = ShapeType::CIRCLE;
    ball->Radius = 1.0f;
    ball->Position = Vec2(-20.0f, 5.0f);
    ball->Velocity = Vec2(25.0f, 15.0f); // Launch velocity vector
    ball->Material.Density = 2.0f; // Heavy ball
    ball->Material.Restitution = 0.5f;
    ball->ComputeMassProperties();
    world.AddBody(ball);
}
```

### 2. Pendulum Chain (Joint Constraints)
Demonstrates multi-joint constraint loops. A distance joint can be modeled as a soft spring-damper or solved using projection:
```cpp
void SetupPendulumScene(World& world) {
    // Fixed anchor box
    RigidBody* anchor = new RigidBody();
    anchor->Position = Vec2(0.0f, 15.0f);
    anchor->Size = Vec2(1.0f, 1.0f);
    anchor->IsStatic = true;
    anchor->ComputeMassProperties();
    world.AddBody(anchor);

    RigidBody* previous = anchor;
    int linkCount = 5;
    float linkDistance = 2.0f;

    for (int i = 0; i < linkCount; ++i) {
        RigidBody* link = new RigidBody();
        link->Type = ShapeType::CIRCLE;
        link->Radius = 0.5f;
        link->Position = Vec2(previous->Position.x + linkDistance, previous->Position.y);
        link->IsStatic = false;
        link->Material.Density = 1.0f;
        link->ComputeMassProperties();
        world.AddBody(link);

        // Define joint constraint between previous and current link
        DistanceJoint joint;
        joint.BodyA = previous;
        joint.BodyB = link;
        joint.AnchorLocalA = Vec2(0, 0);
        joint.AnchorLocalB = Vec2(0, 0);
        joint.TargetDistance = linkDistance;
        world.AddJoint(joint);

        previous = link;
    }
}
```

### 3. Fluid Pressure (Dam Break)
Tests fluid behavior and fluid-rigid body interactions.
```cpp
void SetupFluidDamScene(World& world, FluidSim& fluidSim) {
    // Create bounding tank
    CreateBoundaryTank(world, 20.0f, 15.0f); // Spawns left, right, bottom static boxes

    // Spawn block of fluid particles to the left (representing a dam)
    int rows = 30;
    int cols = 15;
    float spacing = 0.3f;
    Vec2 startPos(-8.0f, -4.0f);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            FluidParticle p;
            p.Position = startPos + Vec2(c * spacing, r * spacing);
            p.Velocity = Vec2(0.0f, 0.0f);
            fluidSim.AddParticle(p);
        }
    }

    // Spawn dynamic floating box inside tank to demonstrate interaction
    RigidBody* floatBox = new RigidBody();
    floatBox->Position = Vec2(5.0f, -2.0f);
    floatBox->Size = Vec2(4.0f, 4.0f);
    floatBox->Material.Density = 0.5f; // Lightweight, floats easily
    floatBox->ComputeMassProperties();
    world.AddBody(floatBox);
}
```

### 4. Inclined Plane (Friction Ramps)
Tests how different sliding friction values affect movement down a slope.
```cpp
void SetupInclinedPlaneScene(World& world) {
    // Tilted ramp (Box tilted 25 degrees)
    RigidBody* ramp = new RigidBody();
    ramp->Position = Vec2(-5.0f, 2.0f);
    ramp->Size = Vec2(25.0f, 1.0f);
    ramp->Rotation = glm::radians(25.0f);
    ramp->IsStatic = true;
    ramp->ComputeMassProperties();
    world.AddBody(ramp);

    // Sliding Box A (Slick, low friction)
    RigidBody* sliderA = new RigidBody();
    sliderA->Position = Vec2(-10.0f, 7.0f);
    sliderA->Size = Vec2(1.5f, 1.5f);
    sliderA->Rotation = glm::radians(25.0f);
    sliderA->Material.Friction = 0.05f; // Ice-like
    sliderA->ComputeMassProperties();
    world.AddBody(sliderA);

    // Sliding Box B (Rough, high friction)
    RigidBody* sliderB = new RigidBody();
    sliderB->Position = Vec2(-8.0f, 8.0f);
    sliderB->Size = Vec2(1.5f, 1.5f);
    sliderB->Rotation = glm::radians(25.0f);
    sliderB->Material.Friction = 0.8f; // Rubber-like (slides slowly or gets stuck)
    sliderB->ComputeMassProperties();
    world.AddBody(sliderB);
}
```
