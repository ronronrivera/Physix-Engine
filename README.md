# Physix 🧪

A 2.5D physics sandbox built from scratch in C++ and OpenGL. 2D rigid body simulation — collision detection, impulse resolution, and fluid sim — viewed through a 3D perspective camera with a Blender-inspired Dear ImGui editor. Built as a hands-on way to learn physics by breaking it in real time.

---

## 🗺️ Track Record

### **Phase 1: Window & Context**
- [x] GLFW window + OpenGL 3.3 core context
- [x] GLAD function pointer loading
- [x] Basic game loop and screen clearing

### **Phase 2: Dear ImGui Shell**
- [x] ImGui GLFW + OpenGL3 backends
- [ ] Blender-inspired layout — left panel, viewport, right panel, bottom bar
- [ ] Framebuffer Object — physics scene renders into ImGui viewport as a texture

### **Phase 3: 2D Render Abstraction**
- [ ] Vertex + fragment shaders (flat, debug, terrain, fluid)
- [ ] VAO/VBO abstractions for circles, boxes, lines
- [ ] Instanced rendering for large body counts
- [ ] Projection + view matrix pipeline via GLM

### **Phase 4: Camera**
- [ ] 3D perspective camera on the XY physics plane
- [ ] Orbit, pan, zoom controls
- [ ] Mouse picking — ray cast from screen to world space

### **Phase 5: Math & Physics Foundation**
- [ ] `Vec2` — position, velocity, force math for the physics layer
- [ ] `Clock` — fixed timestep with accumulator
- [ ] `RigidBody` — pos, vel, mass, shape, material, sleep state
- [ ] `World` — simulation root, owns all bodies and drives the tick

### **Phase 6: Integrator**
- [ ] Semi-implicit Euler integration
- [ ] Gravity, damping, force accumulation
- [ ] Fixed timestep loop with accumulator

### **Phase 7: Collision Detection**
- [ ] Broadphase — spatial hash grid, AABB pairs
- [ ] Narrowphase — SAT for boxes, circle-circle, circle-box
- [ ] Manifold generation — normal, depth, contact points

### **Phase 8: Impulse Resolution**
- [ ] Sequential impulse solver
- [ ] Restitution (elastic + inelastic)
- [ ] Friction
- [ ] Contact point debug draw

### **Phase 9: Procedural Generation**
- [ ] Perlin noise heightmap terrain
- [ ] Static terrain body generation
- [ ] Random object spawner with layout patterns
- [ ] Regenerate on demand from UI

### **Phase 10: Fluid Simulation**
- [ ] SPH (Smoothed Particle Hydrodynamics) particles
- [ ] Pressure + viscosity forces
- [ ] Fluid-rigid body interaction
- [ ] Fluid debug draw

### **Phase 11: Physics Problem Scenarios**
- [ ] Projectile motion
- [ ] Pendulum chain
- [ ] Fluid pressure
- [ ] Stack collapse
- [ ] Inclined plane

### **Phase 12: Debug Draw**
- [ ] AABB overlay
- [ ] Contact normals
- [ ] Velocity vectors
- [ ] Sleep state indicators

---

## 📂 Project Structure

```text
Physix/
├── src/
│   ├── core/              # Window, App, World, RigidBody, physics systems
│   ├── renderer/          # Renderer, Camera3D, Shader, Framebuffer, DebugDraw
│   ├── ui/                # ImGui panels — Viewport, ScenePanel, Inspector, ProblemBar
│   ├── procgen/           # TerrainGen, ObjectSpawner, ProblemFactory
│   ├── support/           # Clock, InputManager
│   └── main.cpp
├── include/               # Headers mirroring src/
├── shaders/               # GLSL source files
├── tools/
│   ├── imgui/             # Dear ImGui + GLFW/OpenGL backends
│   ├── glad/              # OpenGL loader
│   └── stb/               # stb_perlin.h
├── Makefile
└── README.md
```

---

## 🛠️ Build & Run

### Prerequisites

```bash
sudo apt update
sudo apt install build-essential pkg-config libglfw3-dev libgl1-mesa-dev
```

### Build

```bash
make        # compile
./physix    # run
make clean  # clean build files
```

