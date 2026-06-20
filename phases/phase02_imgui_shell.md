# Phase 02: Dear ImGui Shell 🖼️

This phase introduces a multi-panel Editor layout mimicking professional tools like Blender, utilizing a central OpenGL **Framebuffer Object (FBO)** texture to display the rendered physics scene within an ImGui window.

## 📁 Filenames & Directory Structure

* **UI Controllers**:
  * `include/ui/imgui_layer.hpp` & `src/ui/imgui_layer.cpp` — Main layout setup, starts/ends ImGui frames.
  * `include/ui/viewport_panel.hpp` & `src/ui/viewport_panel.cpp` — ImGui wrapper displaying the FBO viewport.
* **OpenGL Render Target**:
  * `include/renderer/framebuffer.hpp` & `src/renderer/framebuffer.cpp` — Allocates, resizes, and binds the FBO.

## ⚙️ Core Variables

### `Framebuffer` Class Members
* `unsigned int m_FboID` — OpenGL Framebuffer Object ID.
* `unsigned int m_TextureID` — Color attachment texture ID.
* `unsigned int m_RboID` — Depth/stencil renderbuffer ID.
* `int m_Width, m_Height` — Dimensions of the off-screen texture.

### `ViewportPanel` Class Members
* `ImVec2 m_PanelSize` — Size of the ImGui viewport window in screen coordinates.
* `bool m_IsFocused` — Tracks whether mouse inputs should target camera/picking or the UI controls.

## 🏛️ Engine Component Role

* **UI Panel Engine**: Draws docking nodes, layouts, inspector fields, and menus.
* **FBO Renderer**: Directs the OpenGL draw calls away from the default monitor window backbuffer and into a texture. This keeps the rendering system separate from user interface buttons.

## 📝 Pseudo-code / Flow of Execution

### 1. Framebuffer Creation & Management
```cpp
void Framebuffer::Create(int width, int height) {
    m_Width = width;
    m_Height = height;

    // Generate FBO
    glGenFramebuffers(1, &m_FboID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_FboID);

    // Create Color Attachment Texture
    glGenTextures(1, &m_TextureID);
    glBindTexture(GL_TEXTURE_2D, m_TextureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_TextureID, 0);

    // Create Depth/Stencil Renderbuffer (for 3D Camera depth testing)
    glGenRenderbuffers(1, &m_RboID);
    glBindRenderbuffer(GL_RENDERBUFFER, m_RboID);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_RboID);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is not complete!" << std::endl;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::Resize(int width, int height) {
    // Clean up old handles and recreate to adjust texture size
    Destroy();
    Create(width, height);
}
```

### 2. Main Render Loop Integration
```cpp
// Within App::Update or Main Loop:

// Step A: Draw physics scene into Framebuffer
m_Framebuffer.Bind();
glViewport(0, 0, m_Framebuffer.Width(), m_Framebuffer.Height());
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

m_Renderer.RenderScene(m_World, m_Camera);

m_Framebuffer.Unbind(); // Directs rendering back to screen backbuffer

// Step B: Setup ImGui layout Dockspace
ImGui_ImplOpenGL3_NewFrame();
ImGui_ImplGlfw_NewFrame();
ImGui::NewFrame();

// DockSpace setup (creating a full-screen background dock space)
ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

// Step C: Left Panel - Hierarchy/Presets
ImGui::Begin("Presets & Problems");
if (ImGui::Button("Spawn Projectile Scenario")) {
    LoadScenario(PROJECTILE);
}
ImGui::End();

// Step D: Right Panel - Inspector
ImGui::Begin("Inspector");
ImGui::DragFloat2("Gravity Offset", &m_World.Gravity.x, 0.1f);
ImGui::End();

// Step E: Viewport Panel (displays the FBO texture)
ImGui::Begin("Physics Viewport");
ImVec2 currentViewportSize = ImGui::GetContentRegionAvail();

// Handle resizing if the panel size changed
if (currentViewportSize.x != m_Framebuffer.Width() || currentViewportSize.y != m_Framebuffer.Height()) {
    m_Framebuffer.Resize((int)currentViewportSize.x, (int)currentViewportSize.y);
}

// Display texture. Note OpenGL texture coordinates are Y-inverted, so we map UVs: (0,1) to (1,0)
ImGui::Image((void*)(intptr_t)m_Framebuffer.TextureID(), currentViewportSize, ImVec2(0, 1), ImVec2(1, 0));

m_ViewportIsFocused = ImGui::IsWindowFocused();
m_ViewportIsHovered = ImGui::IsWindowHovered();

ImGui::End();

// Step F: Draw final UI
ImGui::Render();
ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
```
