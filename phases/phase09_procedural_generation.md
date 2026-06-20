# Phase 09: Procedural Terrain & Spawner 🏞️

This phase implements procedural 1D heightmap generation (using Perlin noise) to create dynamic terrain, alongside spawner tools that organize stacks, grids, or circles of rigid bodies on demand.

## 📁 Filenames & Directory Structure

* **Procedural Engines**:
  * `include/procgen/terrain_gen.hpp` & `src/procgen/terrain_gen.cpp` — Noise sampling and terrain meshes.
  * `include/procgen/object_spawner.hpp` & `src/procgen/object_spawner.cpp` — Pattern layout placement.
* **Support Math Headers**:
  * `tools/stb/stb_perlin.h` — External header library providing Perlin noise functions.

## ⚙️ Core Variables

### `Terrain` Configuration State
* `float Width = 100.0f` — Span width of terrain.
* `float MaxHeight = 10.0f` — Max amplitude of terrain hills.
* `int SegmentCount = 200` — Number of linear vertices/colliders connecting ground points.
* `float NoiseScale = 0.05f` — Frequency size for noise iterations.
* `unsigned int Seed` — Current random seed.

### `ObjectSpawner` Configuration
* `int GridRows = 5` — Height count of spawned object stacks.
* `int GridCols = 5` — Width count of spawned object stacks.
* `float ObjectSpacing = 1.2f` — Space cushion multiplier between spawned circles/boxes.
* `float MinRadius = 0.5f`, `MaxRadius = 1.0f` — Dimensions of random objects.

## 🏛️ Engine Component Role

* **Terrain Builder**: Generates static boundary structures representing hills, valleys, or flat planes.
* **Level Populator**: Emplaces collections of dynamic rigid bodies in logical arrangements (e.g. pyramids, neat grid matrices, columns).

## 📝 Pseudo-code / Flow of Execution

### 1. Generating Procedural Terrain
To generate hills, we sample 1D Perlin noise at spaced horizontal intervals and construct a chain of static colliders:
```cpp
#include "stb/stb_perlin.h"

void TerrainGen::Generate(World& world, TerrainConfig config) {
    float step = config.Width / (float)config.SegmentCount;
    std::vector<Vec2> points;

    // 1. Calculate heights using fractal Brownian motion (fBm) or simple noise
    for (int i = 0; i <= config.SegmentCount; ++i) {
        float x = i * step - (config.Width * 0.5f); // Center terrain around x=0
        
        // Sample Perlin noise (we pass seed as the Y input to vary heightmaps)
        float noiseVal = stb_perlin_noise3(x * config.NoiseScale, (float)config.Seed, 0.0f, 0, 0, 0);
        
        // Scale values to engine coordinates
        float y = noiseVal * config.MaxHeight;
        points.push_back(Vec2(x, y));
    }

    // 2. Clear old terrain segments in the World
    world.RemoveTerrainSegments();

    // 3. Create static thin box segments between successive height points
    for (size_t i = 0; i < points.size() - 1; ++i) {
        Vec2 p0 = points[i];
        Vec2 p1 = points[i + 1];
        
        Vec2 midPoint = (p0 + p1) * 0.5f;
        Vec2 diff = p1 - p0;
        float segmentLength = diff.Length();
        float angle = std::atan2(diff.y, diff.x);

        // Create box representing ground thickness
        float thickness = 0.5f; 
        RigidBody* segment = new RigidBody();
        segment->Position = midPoint;
        segment->Size = Vec2(segmentLength, thickness);
        segment->Rotation = angle;
        segment->IsStatic = true;
        segment->ComputeMassProperties();

        world.AddBody(segment);
    }
}
```

### 2. Spawning Object Pyramids/Grids
Grid stack layouts are useful for testing multi-body collisions and solver stability (stack collapse tests).
```cpp
void ObjectSpawner::SpawnStack(World& world, Vec2 bottomCenter, int rows, int cols, float size, bool spawnCircles) {
    float halfColWidth = (cols * size * ObjectSpacing) * 0.5f;
    float startX = bottomCenter.x - halfColWidth + (size * 0.5f);
    float startY = bottomCenter.y + (size * 0.5f);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            // Offset coordinates slightly per row to stagger stacks
            float x = startX + c * size * ObjectSpacing;
            float y = startY + r * size * ObjectSpacing;

            RigidBody* b = new RigidBody();
            b->Position = Vec2(x, y);
            
            if (spawnCircles) {
                b->Type = ShapeType::CIRCLE;
                b->Radius = size * 0.5f;
            } else {
                b->Type = ShapeType::BOX;
                b->Size = Vec2(size, size);
            }
            
            b->IsStatic = false;
            b->Material.Density = 1.0f;
            b->Material.Restitution = 0.2f;
            b->Material.Friction = 0.3f;
            b->ComputeMassProperties();

            world.AddBody(b);
        }
    }
}
```
### 3. Triggering from ImGui
```cpp
ImGui::Begin("Spawner Panel");
static int rows = 5, cols = 5;
ImGui::SliderInt("Rows", &rows, 1, 15);
ImGui::SliderInt("Cols", &cols, 1, 15);

if (ImGui::Button("Spawn Box Grid")) {
    ObjectSpawner::SpawnStack(m_World, Vec2(0, 5), rows, cols, 1.0f, false);
}
ImGui::End();
```
