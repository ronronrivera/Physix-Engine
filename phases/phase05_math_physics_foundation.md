# Phase 05: Math & Physics Foundation ⚙️

This phase creates the core structures that store physical state (bodies, vectors, shapes) and manages the fixed-timestep simulator clock.

## 📁 Filenames & Directory Structure

* **Math Utilities**:
  * `include/core/math_utils.hpp` — Custom `Vec2` structure and helper operators.
* **Physics Entities**:
  * `include/core/rigid_body.hpp` & `src/core/rigid_body.cpp` — Defines bodies, properties, and configurations.
  * `include/core/shape.hpp` — Subclasses representing Circle and Box bounds.
* **Simulation Management**:
  * `include/core/world.hpp` & `src/core/world.cpp` — Simulation workspace database.
  * `include/support/clock.hpp` & `src/support/clock.cpp` — Semi-implicit tick accumulator.

## ⚙️ Core Variables

### `Vec2` Struct
* `float x, y` — Vector components.

### `Material` Struct
* `float Density` — Used to calculate mass based on area.
* `float Restitution` — Bounciness coefficient `[0.0, 1.0]`.
* `float Friction` — Friction coefficient `[0.0, 1.0]`.

### `RigidBody` Properties
* `Vec2 Position`, `Velocity`, `Force` — Translational properties.
* `float Rotation` (angle in radians), `AngularVelocity`, `Torque` — Rotational properties.
* `float Mass`, `InvMass` — Inertial translation (InvMass = 0 for static bodies).
* `float Inertia`, `InvInertia` — Resistance to angular acceleration.
* `ShapeType Type` — Circle or Box discriminator.
* `bool IsStatic` — Keeps body anchored in space.
* `bool IsSleeping` — Deactivates simulation updates when speed drops below a threshold.
* `float Motion` — Running average of kinetic energy to decide sleeping.

### `World` Context
* `std::vector<RigidBody*> Bodies` — Array of references to active bodies.
* `Vec2 Gravity` — Global gravity vector.

## 🏛️ Engine Component Role

* **Physics Database (World)**: Owns body allocations and handles system execution order.
* **Timekeeper (Clock)**: Decouples rendering framerate from physical tick rates to maintain simulation stability.

## 📝 Pseudo-code / Flow of Execution

### 1. Vector Operations Setup (`math_utils.hpp`)
```cpp
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float _x, float _y) : x(_x), y(_y) {}

    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(float scalar) const { return Vec2(x * scalar, y * scalar); }
    
    float LengthSq() const { return x*x + y*y; }
    float Length() const { return std::sqrt(x*x + y*y); }
    
    Vec2 Normalized() const {
        float len = Length();
        if (len > 0.0001f) return Vec2(x / len, y / len);
        return Vec2(0.0f, 0.0f);
    }
};

inline float Dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline float Cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline Vec2 Cross(const Vec2& v, float s) { return Vec2(s * v.y, -s * v.x); }
inline Vec2 Cross(float s, const Vec2& v) { return Vec2(-s * v.y, s * v.x); }
```

### 2. Timestep Accumulation (`clock.cpp`)
To guarantee reproducible collisions, physics ticks run on a fixed step size (e.g. 1/60th second) rather than raw delta-time:
```cpp
void Clock::Tick(float realDeltaTime, float physicsTimestep, std::function<void(float)> physicsStepCallback) {
    // Avoid "spiral of death" (hanging if framerate is very low)
    if (realDeltaTime > 0.25f) realDeltaTime = 0.25f;

    m_Accumulator += realDeltaTime;
    
    // Step simulation as many times as needed to catch up
    while (m_Accumulator >= physicsTimestep) {
        physicsStepCallback(physicsTimestep);
        m_Accumulator -= physicsTimestep;
    }
}
```

### 3. Mass & Inertia Calculator (`rigid_body.cpp`)
```cpp
void RigidBody::ComputeMassProperties() {
    if (IsStatic) {
        Mass = 0.0f;
        InvMass = 0.0f;
        Inertia = 0.0f;
        InvInertia = 0.0f;
        return;
    }

    if (Type == ShapeType::CIRCLE) {
        float area = M_PI * Radius * Radius;
        Mass = area * Material.Density;
        InvMass = 1.0f / Mass;
        Inertia = 0.5f * Mass * Radius * Radius;
        InvInertia = 1.0f / Inertia;
    } 
    else if (Type == ShapeType::BOX) {
        // Size = (width, height)
        float area = Size.x * Size.y;
        Mass = area * Material.Density;
        InvMass = 1.0f / Mass;
        Inertia = (1.0f / 12.0f) * Mass * (Size.x * Size.x + Size.y * Size.y);
        InvInertia = 1.0f / Inertia;
    }
}
```
