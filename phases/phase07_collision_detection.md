# Phase 07: Collision Detection 💥

This phase sets up the collision pipeline: a Broadphase Spatial Hash Grid filters out non-overlapping bodies, and a Narrowphase Separating Axis Theorem (SAT) solver tests intersections and computes intersection depths and normals.

## 📁 Filenames & Directory Structure

* **Broadphase Filter**:
  * `include/core/spatial_hash.hpp` & `src/core/spatial_hash.cpp` — Maps rigid body AABBs into cell keys.
* **Narrowphase Solver**:
  * `include/core/collision.hpp` & `src/core/collision.cpp` — Intersection formulas (Circle-Circle, Circle-Box, Box-Box).
  * `include/core/manifold.hpp` — Contains structural descriptions of contact details.

## ⚙️ Core Variables

### `AABB` Struct
* `Vec2 Min`, `Max` — Bounding coordinates.

### `CollisionManifold` Struct
* `RigidBody* BodyA`, `*BodyB` — Intersecting body pointers.
* `float PenetrationDepth` — Distance overlap.
* `Vec2 Normal` — Collision response vector pointing from A to B.
* `std::array<Vec2, 2> Contacts` — Contact point locations.
* `int ContactCount` — Number of contact points (1 or 2 for 2D rigid bodies).

### `SpatialHashGrid` Members
* `float CellSize` — Span size of grid cells (e.g. equal to average object diameter).
* `std::unordered_map<int, std::vector<RigidBody*>> Grid` — Map of grid cell keys to bodies.

## 🏛️ Engine Component Role

* **Broadphase**: Eliminates pairs that are too far apart, converting an $O(N^2)$ brute force search to a near-$O(N)$ lookup.
* **Narrowphase**: Resolves shape overlaps, outputting contact normal, penetration, and contact vertices needed by the impulse solver.

## 📝 Pseudo-code / Flow of Execution

### 1. Broadphase Spatial Hashing
```cpp
// Cell key hash function based on spatial grid cell coordinate
int GetCellKey(const Vec2& position, float cellSize) {
    int ix = (int)std::floor(position.x / cellSize);
    int iy = (int)std::floor(position.y / cellSize);
    return ix * 73856093 ^ iy * 19349663; // Primes blend keys to prevent collisions
}

void SpatialHash::Update(const std::vector<RigidBody*>& bodies) {
    Grid.clear();
    for (RigidBody* b : bodies) {
        AABB aabb = b->GetAABB();
        
        // Find grid bounds covered by AABB
        int startX = (int)std::floor(aabb.Min.x / CellSize);
        int endX   = (int)std::floor(aabb.Max.x / CellSize);
        int startY = (int)std::floor(aabb.Min.y / CellSize);
        int endY   = (int)std::floor(aabb.Max.y / CellSize);

        for (int x = startX; x <= endX; ++x) {
            for (int y = startY; y <= endY; ++y) {
                int key = x * 73856093 ^ y * 19349663;
                Grid[key].push_back(b);
            }
        }
    }
}
```

### 2. Separating Axis Theorem (SAT) for Box vs Box
```cpp
bool BoxToBoxCollision(RigidBody* A, RigidBody* B, CollisionManifold& outManifold) {
    float minOverlap = std::numeric_limits<float>::max();
    Vec2 bestAxis;
    
    // 1. Gather all projection axes (face normals of box A and box B)
    std::vector<Vec2> axes = {
        A->GetLocalAxisX(), A->GetLocalAxisY(),
        B->GetLocalAxisX(), B->GetLocalAxisY()
    };

    for (const Vec2& axis : axes) {
        // Project Box A vertices along axis
        auto [minA, maxA] = ProjectBox(A, axis);
        // Project Box B vertices along axis
        auto [minB, maxB] = ProjectBox(B, axis);

        // Check overlap
        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) {
            return false; // Found separating axis! No collision possible.
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // Ensure normal points from A to B
    Vec2 dir = B->Position - A->Position;
    if (Dot(dir, bestAxis) < 0.0f) {
        bestAxis = bestAxis * -1.0f;
    }

    outManifold.BodyA = A;
    outManifold.BodyB = B;
    outManifold.Normal = bestAxis;
    outManifold.PenetrationDepth = minOverlap;
    
    // Find contact vertices (Clips B's vertices against A's face edges)
    GenerateContactPoints(outManifold);
    return true;
}
```

### 3. Circle-Box Collision
```cpp
bool CircleToBoxCollision(RigidBody* circle, RigidBody* box, CollisionManifold& outManifold) {
    // 1. Transform Circle Center into Box's local coordinate space
    Vec2 relativePos = circle->Position - box->Position;
    float theta = -box->Rotation;
    Vec2 localCircleCenter(
        relativePos.x * std::cos(theta) - relativePos.y * std::sin(theta),
        relativePos.x * std::sin(theta) + relativePos.y * std::cos(theta)
    );

    // 2. Find closest point on Box bounds to the local center
    Vec2 halfSize = box->Size * 0.5f;
    Vec2 localClosestPoint(
        std::clamp(localCircleCenter.x, -halfSize.x, halfSize.x),
        std::clamp(localCircleCenter.y, -halfSize.y, halfSize.y)
    );

    // 3. Distance vector
    Vec2 localDiff = localCircleCenter - localClosestPoint;
    float distSq = localDiff.LengthSq();
    float r = circle->Radius;

    if (distSq > r * r) return false; // Circle outside Box projection

    float dist = std::sqrt(distSq);
    Vec2 normal;
    float depth;

    if (distSq > 0.0001f) {
        normal = localDiff * (1.0f / dist);
        depth = r - dist;
    } else {
        // Circle center is inside the Box. Find closest edge.
        float dx = halfSize.x - std::abs(localCircleCenter.x);
        float dy = halfSize.y - std::abs(localCircleCenter.y);
        
        if (dx < dy) {
            normal = Vec2(localCircleCenter.x > 0 ? 1.0f : -1.0f, 0.0f);
            depth = r + dx;
        } else {
            normal = Vec2(0.0f, localCircleCenter.y > 0 ? 1.0f : -1.0f);
            depth = r + dy;
        }
    }

    // 4. Transform local coordinates back to world space
    outManifold.BodyA = circle;
    outManifold.BodyB = box;
    outManifold.PenetrationDepth = depth;
    
    float cosTheta = std::cos(box->Rotation);
    float sinTheta = std::sin(box->Rotation);
    outManifold.Normal = Vec2(
        normal.x * cosTheta - normal.y * sinTheta,
        normal.x * sinTheta + normal.y * cosTheta
    ) * -1.0f; // Point Normal from A to B

    outManifold.ContactCount = 1;
    outManifold.Contacts[0] = circle->Position + (outManifold.Normal * r);
    return true;
}
```
