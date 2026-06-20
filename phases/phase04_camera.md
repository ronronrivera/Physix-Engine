# Phase 04: Camera 🎥

This phase implements a 3D perspective camera positioned relative to the 2D XY physics simulation plane (Z = 0), allowing orbit, zoom, pan, and mouse picking (converting screen coordinates to world coordinates).

## 📁 Filenames & Directory Structure

* **Camera Engine**:
  * `include/renderer/camera3d.hpp` & `src/renderer/camera3d.cpp` — Stores views, orientation, and mouse projection matrices.
* **Input Hooking**:
  * `include/support/input_manager.hpp` & `src/support/input_manager.cpp` — Filters raw mouse/key events.

## ⚙️ Core Variables

### `Camera3D` State
* `glm::vec3 m_Position` — World position.
* `glm::vec3 m_Target` — Look-at target (anchor point on XY plane).
* `glm::vec3 m_Up`, `m_Right`, `m_Front` — Direction vectors.
* `float m_Yaw` — Orbit angle around the vertical axis.
* `float m_Pitch` — Elevation angle.
* `float m_Distance` — Radius of the orbit sphere around the target.
* `float m_FOV` — Field of view (default: `45.0f` degrees).
* `float m_AspectRatio` — Width / Height ratio matching the ImGui Viewport panel.

## 🏛️ Engine Component Role

* **Perspective Projector**: Generates view and projection matrices to represent 2D physics simulations with a 3D sense of depth.
* **Raycaster**: Translates 2D screen positions from ImGui panels into 3D rays that intersect the physics plane.

## 📝 Pseudo-code / Flow of Execution

### 1. View & Orientation Calculations
```cpp
void Camera3D::UpdateVectors() {
    // Spherical to Cartesian coordinate transformation for Orbit camera
    m_Position.x = m_Target.x + m_Distance * cos(glm::radians(m_Pitch)) * sin(glm::radians(m_Yaw));
    m_Position.y = m_Target.y + m_Distance * sin(glm::radians(m_Pitch));
    m_Position.z = m_Target.z + m_Distance * cos(glm::radians(m_Pitch)) * cos(glm::radians(m_Yaw));

    m_Front = glm::normalize(m_Target - m_Position);
    m_Right = glm::normalize(glm::cross(m_Front, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_Up    = glm::normalize(glm::cross(m_Right, m_Front));
}

glm::mat4 Camera3D::GetViewMatrix() const {
    return glm::lookAt(m_Position, m_Target, m_Up);
}

glm::mat4 Camera3D::GetProjectionMatrix() const {
    return glm::perspective(glm::radians(m_FOV), m_AspectRatio, 0.1f, 1000.0f);
}
```

### 2. Camera Controls (Orbit, Pan, Zoom)
```cpp
void Camera3D::ProcessMouseScroll(float yOffset) {
    // Zoom control
    m_Distance -= yOffset * 2.0f;
    if (m_Distance < 1.0f) m_Distance = 1.0f;
    UpdateVectors();
}

void Camera3D::ProcessPan(float deltaX, float deltaY) {
    // Calculate translate factor based on camera distance to make movement smooth
    float speed = m_Distance * 0.001f;
    
    // Pan right/left
    m_Target -= m_Right * deltaX * speed;
    // Pan up/down
    m_Target += m_Up * deltaY * speed;
    
    UpdateVectors();
}

void Camera3D::ProcessRotate(float deltaX, float deltaY) {
    m_Yaw   += deltaX * 0.25f;
    m_Pitch += deltaY * 0.25f;
    
    // Restrict pitch angle to avoid camera inversion
    m_Pitch = glm::clamp(m_Pitch, -89.0f, 89.0f);
    
    UpdateVectors();
}
```

### 3. Screen-to-World Raycasting (Mouse Picking)
To click and grab rigid bodies, we must project screen clicks onto the physics plane (Z = 0):
```cpp
glm::vec2 Camera3D::ScreenToWorld(const glm::vec2& screenPos, const glm::vec2& viewportSize) {
    // 1. Normalized Device Coordinates (NDC) [-1 to 1]
    float x = (2.0f * screenPos.x) / viewportSize.x - 1.0f;
    float y = 1.0f - (2.0f * screenPos.y) / viewportSize.y; // Flip Y axis
    
    glm::vec4 rayClip = glm::vec4(x, y, -1.0f, 1.0f);
    
    // 2. Homogeneous Eye Coordinates
    glm::mat4 invProj = glm::inverse(GetProjectionMatrix());
    glm::vec4 rayEye = invProj * rayClip;
    rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f); // Make direction ray, not point
    
    // 3. World Coordinates
    glm::mat4 invView = glm::inverse(GetViewMatrix());
    glm::vec3 rayDir = glm::normalize(glm::vec3(invView * rayEye));
    glm::vec3 rayOrigin = m_Position;
    
    // 4. Ray-Plane intersection: Ray = origin + t * dir, Plane: Z = 0
    // Z_origin + t * Z_dir = 0  =>  t = -Z_origin / Z_dir
    if (std::abs(rayDir.z) > 0.0001f) {
        float t = -rayOrigin.z / rayDir.z;
        glm::vec3 intersection = rayOrigin + t * rayDir;
        return glm::vec2(intersection.x, intersection.y);
    }
    
    return glm::vec2(0.0f); // Default fallback
}
```
