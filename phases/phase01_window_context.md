# Phase 01: Window & Context 🎮

This phase initializes the application's base window and configures OpenGL 3.3 Core Profile.

## 📁 Filenames & Directory Structure

* **Main Entry Point**: `src/main.cpp` (already implemented)
* **Build Configuration**: `Makefile` (already implemented)
* **OpenGL Headers**: `tools/glad/glad.h`, `tools/glad.c`

## ⚙️ Core Variables

Within `src/main.cpp` (or a future `App` wrapper class):
* `GLFWwindow* window` — Handles window events, keyboard/mouse capture, and context swap buffers.
* `const int WINDOW_WIDTH = 800` — Window frame width.
* `const int WINDOW_HEIGHT = 600` — Window frame height.
* `GLFWmonitor* primaryMonitor` — Used to get DPI scaling factor for HighDPI displays.
* `float main_scale` — Factor for scaling ImGui font and widget coordinates.

## 🏛️ Engine Component Role

* **Window Manager**: Handles communication with the Operating System window server via GLFW.
* **OpenGL Loader**: GLAD retrieves driver-specific OpenGL function pointers.
* **Application Loop**: Coordinates updating simulation state (physics clock tick) and redrawing the screen.

## 📝 Pseudo-code / Flow of Execution

```cpp
// 1. Initialize GLFW & Configure OpenGL Version Hint
glfwInit();
glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

// 2. Create Window
GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Physix Engine", nullptr, nullptr);
if (!window) {
    std::cerr << "Failed to create GLFW Window" << std::endl;
    glfwTerminate();
    return -1;
}
glfwMakeContextCurrent(window);

// 3. Initialize GLAD
if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to load OpenGL pointers" << std::endl;
    return -1;
}

// 4. Configure Callback for Window Resize
glfwSetFramebufferSizeCallback(window, [](GLFWwindow* w, int width, int height) {
    glViewport(0, 0, width, height);
});

// 5. Main Game Loop
while (!glfwWindowShouldClose(window)) {
    // Poll Events (Input/Window events)
    glfwPollEvents();
    
    // Clear back buffer
    glClearColor(0.1f, 0.1f, 0.14f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Swap back and front buffers
    glfwSwapBuffers(window);
}

// 6. Cleanup
glfwDestroyWindow(window);
glfwTerminate();
```
