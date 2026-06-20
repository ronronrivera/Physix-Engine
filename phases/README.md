# Physix Implementation Roadmap 🗺️

Welcome to the **Physix** feature implementation plan. This directory contains detailed blueprints for all phases of the project, separating files by phase. Each document includes the **target filenames**, **necessary structures and variables**, **role of the engine component**, and **pseudo-code/math foundations** for implementation.

## 🧱 Architectural Overview

The engine follows a modular, decoupled architecture to ensure that the physics simulation is independent of the rendering and UI layers:

```mermaid
graph TD
    App[Application Loop] --> UI[Dear ImGui Interface]
    App --> World[Physics World]
    App --> Renderer[3D/2D Renderer]
    
    World --> Integrator[Semi-Implicit Euler Integrator]
    World --> Collide[Collision Detection]
    World --> Solver[Impulse Solver]
    
    UI --> Viewport[ImGui Viewport Panel]
    Renderer --> Framebuffer[FBO Texture]
    Framebuffer --> Viewport
```

---

## 🗂️ Phase Directory

Select a phase below to view its specific implementation design:

### 🎮 Phase 1-4: Engine Shell & Rendering
1. **[Phase 01: Window & OpenGL Context](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase01_window_context.md)**  
   *GLFW window initialization, GLAD pointer loader, and core context binding.*
2. **[Phase 02: Dear ImGui Shell](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase02_imgui_shell.md)**  
   *Blender-style layout, Docking workspace, and Framebuffer Object (FBO) setup.*
3. **[Phase 03: 2D Render Abstraction](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase03_render_abstraction.md)**  
   *Shaders, VAO/VBO abstractions, and a batched/instanced shapes renderer.*
4. **[Phase 04: 3D Camera & Raycasting](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase04_camera.md)**  
   *Perspective view, orbit controls, zoom/pan, and NDC-to-world mouse picking.*

### ⚙️ Phase 5-8: Core Rigid Body Physics
5. **[Phase 05: Math & Physics Foundation](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase05_math_physics_foundation.md)**  
   *`Vec2` operators, rigid body state representation, and fixed timestep clock accumulation.*
6. **[Phase 06: Integrator](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase06_integrator.md)**  
   *Force accumulation, semi-implicit Euler integration, and sleep state transitions.*
7. **[Phase 07: Collision Detection](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase07_collision_detection.md)**  
   *Broadphase spatial hash grid, narrowphase SAT (Separating Axis Theorem), and manifold calculation.*
8. **[Phase 08: Impulse Solver & Resolution](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase08_impulse_resolution.md)**  
   *Sequential impulse constraints, normal restitution, tangent friction, and position correction (Baumgarte).*

### 🌊 Phase 9-12: Advanced Features, Proceduralism, & Simulation
9. **[Phase 09: Procedural Terrain & Object Spawner](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase09_procedural_generation.md)**  
   *1D Perlin noise heightmap terrain, static collider chaining, and shape grid generators.*
10. **[Phase 10: Fluid Simulation (SPH)](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase10_fluid_simulation.md)**  
    *Smoothed Particle Hydrodynamics (SPH), density/pressure calculations, and fluid-rigid body coupling.*
11. **[Phase 11: Physics Scenarios](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase11_problem_scenarios.md)**  
    *Preset scene loader (projectile motion, double pendulum, stack collapse, incline planes).*
12. **[Phase 12: Debug Draw](file:///home/ronaspe42/Projects/CPP_PROJECTS/Physix/phases/phase12_debug_draw.md)**  
    *Direct line rendering overlays, contact normal arrows, velocity vectors, and sleep highlights.*
