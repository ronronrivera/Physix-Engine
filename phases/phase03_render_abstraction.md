# Phase 03: 2D Render Abstraction 🖌️

This phase builds the low-level rendering abstractions to draw 2D primitives (circles, boxes, lines) efficiently, decoupling them from direct OpenGL calls in the main logic.

## 📁 Filenames & Directory Structure

* **OpenGL Wrappers**:
  * `include/renderer/shader.hpp` & `src/renderer/shader.cpp` — Compiles and manages GLSL uniforms.
  * `include/renderer/vertex_array.hpp` & `src/renderer/vertex_array.cpp` — Vertex array layouts.
  * `include/renderer/vertex_buffer.hpp` & `src/renderer/vertex_buffer.cpp` — Raw vertex buffer memory.
  * `include/renderer/index_buffer.hpp` & `src/renderer/index_buffer.cpp` — Index (element) buffers.
* **Primitive Rendering Engine**:
  * `include/renderer/renderer2d.hpp` & `src/renderer/renderer2d.cpp` — Manages batching / instanced draw calls.
* **GLSL Shader Sources**:
  * `shaders/flat.vs` — Vertex shader handling translations, rotations, and projections.
  * `shaders/flat.fs` — Fragment shader applying color/wireframe values.

## ⚙️ Core Variables

### `Vertex` Struct
* `glm::vec2 Position` — Object coordinates.
* `glm::vec4 Color` — RGBA tint.

### `Renderer2D` Members
* `std::shared_ptr<Shader> m_FlatShader` — Standard solid-color primitive shader.
* `unsigned int m_QuadVAO, m_QuadVBO, m_QuadEBO` — Shared quad geometry used for instanced drawing of boxes and fluid particles.
* `unsigned int m_LineVAO, m_LineVBO` — Dynamic buffers for debug lines (re-allocated per frame).
* `std::vector<Vertex> m_LineVertexBuffer` — Local cache containing queued line primitives.

## 🏛️ Engine Component Role

* **Batch/Instance Shader Engine**: Groups shapes by material or geometry configuration to minimize draw calls.
* **Uniform Manager**: Updates transformation matrices (View, Projection) inside GPU memory.

## 📝 Pseudo-code / Flow of Execution

### 1. Matrix Setup (GLM)
To render 2D rigid bodies in a 3D view frustum, we construct matrices using GLM:
```cpp
// View Matrix: Camera positions
glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);

// Projection Matrix: 3D perspective to display 2D XY plane
glm::mat4 projection = glm::perspective(glm::radians(fov), aspect, 0.1f, 100.0f);

glm::mat4 viewProj = projection * view;
```

### 2. Renderer2D Draw Routines (API)
```cpp
void Renderer2D::Init() {
    // 1. Set up static quad VAO/VBO for drawing boxes
    float vertices[] = {
        -0.5f, -0.5f,  // 0: Bottom-left
         0.5f, -0.5f,  // 1: Bottom-right
         0.5f,  0.5f,  // 2: Top-right
        -0.5f,  0.5f   // 3: Top-left
    };
    unsigned int indices[] = { 0, 1, 2, 2, 3, 0 };

    glGenVertexArrays(1, &m_QuadVAO);
    glGenBuffers(1, &m_QuadVBO);
    glGenBuffers(1, &m_QuadEBO);

    glBindVertexArray(m_QuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_QuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_QuadEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0); // Position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    
    // 2. Set up dynamic Line VAO/VBO
    glGenVertexArrays(1, &m_LineVAO);
    glGenBuffers(1, &m_LineVBO);
    glBindVertexArray(m_LineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_LineVBO);
    glBufferData(GL_ARRAY_BUFFER, MAX_LINE_VERTICES * sizeof(Vertex), nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0); // Position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
    glEnableVertexAttribArray(1); // Color
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Color));
}

void Renderer2D::BeginScene(const glm::mat4& viewProjMatrix) {
    m_FlatShader->Bind();
    m_FlatShader->SetMat4("u_ViewProjection", viewProjMatrix);
    m_LineVertexBuffer.clear();
}

void Renderer2D::DrawBox(const glm::vec2& position, const glm::vec2& size, float angle, const glm::vec4& color) {
    // Model Matrix Translation + Rotation + Scale
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(position, 0.0f));
    model = glm::rotate(model, angle, glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::scale(model, glm::vec3(size, 1.0f));

    m_FlatShader->SetMat4("u_Model", model);
    m_FlatShader->SetVec4("u_Color", color);

    glBindVertexArray(m_QuadVAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void Renderer2D::DrawLine(const glm::vec2& p0, const glm::vec2& p1, const glm::vec4& color) {
    m_LineVertexBuffer.push_back({ p0, color });
    m_LineVertexBuffer.push_back({ p1, color });
}

void Renderer2D::EndScene() {
    // Flush dynamic lines
    if (!m_LineVertexBuffer.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, m_LineVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_LineVertexBuffer.size() * sizeof(Vertex), m_LineVertexBuffer.data());
        
        m_LineShader->Bind();
        m_LineShader->SetMat4("u_ViewProjection", m_ViewProjCache);
        
        glBindVertexArray(m_LineVAO);
        glDrawArrays(GL_LINES, 0, m_LineVertexBuffer.size());
    }
}
```

### 3. GLSL Shaders (`shaders/flat.vs` & `shaders/flat.fs`)

**flat.vs**:
```glsl
#version 330 core
layout (location = 0) in vec2 aPos;

uniform mat4 u_ViewProjection;
uniform mat4 u_Model;

void main() {
    gl_Position = u_ViewProjection * u_Model * vec4(aPos, 0.0, 1.0);
}
```

**flat.fs**:
```glsl
#version 330 core
out vec4 FragColor;
uniform vec4 u_Color;

void main() {
    FragColor = u_Color;
}
```
