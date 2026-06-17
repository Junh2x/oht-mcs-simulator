// STEP 0 bring-up: GLFW window + OpenGL3 + Dear ImGui + ImPlot.
// Run with --selftest to render a few frames and exit (sanity check).

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdio>
#include <cstring>

static void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int argc, char** argv) {
    bool selftest = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--selftest") == 0) selftest = true;
    }

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(
        1280, 720,
        "OHT-MCS Simulator  -  STEP 0 bring-up (ImGui + ImPlot)",
        nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // sample data for the plot
    float xs[200];
    float ys[200];
    for (int i = 0; i < 200; ++i) {
        xs[i] = i * 0.05f;
        ys[i] = std::sin(xs[i]);
    }

    bool show_implot_demo = false;
    const ImVec4 clear_color = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);

    int frame = 0;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("AMHS MCS  -  bring-up check");
        ImGui::Text("Toolchain OK.");
        ImGui::Text("Dear ImGui %s + ImPlot loaded.", IMGUI_VERSION);
        ImGui::Text("%.1f FPS", io.Framerate);
        ImGui::Checkbox("Show ImPlot demo", &show_implot_demo);
        ImGui::Separator();
        if (ImPlot::BeginPlot("sample signal", ImVec2(-1, 260))) {
            ImPlot::PlotLine("sin(x)", xs, ys, 200);
            ImPlot::EndPlot();
        }
        ImGui::End();

        if (show_implot_demo) ImPlot::ShowDemoWindow(&show_implot_demo);

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        if (selftest && ++frame >= 5) break;
    }

    if (selftest) {
        std::printf("selftest OK: rendered %d frames, shutting down cleanly\n", frame);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
