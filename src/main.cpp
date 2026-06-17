// STEP 1: rail-network graph + Dijkstra, rendered in the live ImGui window.
// Run with --selftest to render a few frames and exit (sanity check).

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "rail/RailNetwork.h"
#include "rail/FabTopology.h"
#include "render/RailRenderer.h"

#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

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

    // Scale the whole UI by the monitor DPI so text stays readable on 4K displays.
    float ui_scale = 1.0f;
    if (GLFWmonitor* monitor = glfwGetPrimaryMonitor()) {
        float sx = 1.0f, sy = 1.0f;
        glfwGetMonitorContentScale(monitor, &sx, &sy);
        if (sx > 0.0f) ui_scale = sx;
    }

    GLFWwindow* window = glfwCreateWindow(
        static_cast<int>(1280 * ui_scale), static_cast<int>(760 * ui_scale),
        "OHT-MCS Simulator",
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

    // Rasterize the default font larger and scale widget metrics to the DPI.
    ImFontConfig font_cfg;
    font_cfg.SizePixels = 13.0f * ui_scale;
    io.Fonts->AddFontDefault(&font_cfg);

    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(ui_scale);

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

    // STEP 1: build the synthetic fab, find one shortest path, report it once to stdout.
    rail::FabDemo demo;
    rail::RailNetwork net = rail::buildSyntheticFab(demo);
    rail::PathResult path = rail::dijkstra(net, demo.start, demo.goal);

    std::string joined;
    for (std::size_t i = 0; i < path.nodes.size(); ++i) {
        if (i != 0) joined += ", ";
        joined += net.nodes()[path.nodes[i]].name;
    }
    std::printf("%s -> %s shortest path = [%s] distance %.1f\n",
                net.nodes()[demo.start].name.c_str(),
                net.nodes()[demo.goal].name.c_str(),
                joined.c_str(), path.distance);

    render::RailViewStyle rail_style;
    rail_style.pad *= ui_scale;
    rail_style.node_r *= ui_scale;
    rail_style.port_half *= ui_scale;
    rail_style.seg_thick *= ui_scale;
    rail_style.bottleneck_thick *= ui_scale;
    rail_style.path_thick *= ui_scale;

    int frame = 0;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Status");
        ImGui::Text("%.1f FPS", io.Framerate);
        ImGui::Text("Nodes %d, segments %d. Ports are drawn as squares.",
                    (int)net.nodes().size(), (int)net.segments().size());
        if (path.found) {
            ImGui::Text("Shortest path %s -> %s: distance %.1f (%d segments)",
                        net.nodes()[demo.start].name.c_str(),
                        net.nodes()[demo.goal].name.c_str(),
                        path.distance, (int)path.segments.size());
            ImGui::TextColored(ImVec4(0.95f, 0.67f, 0.24f, 1.0f),
                               "Amber = bottleneck, green = path (amber shows as a border where they overlap).");
        }
        ImGui::Checkbox("Show ImPlot demo", &show_implot_demo);
        ImGui::Separator();
        if (ImPlot::BeginPlot("sample signal", ImVec2(-1, 180))) {
            ImPlot::PlotLine("sin(x)", xs, ys, 200);
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Rail Network");
        ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
        if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
        if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
        render::drawRailNetwork(ImGui::GetWindowDrawList(), canvas_origin, canvas_size, net, path, rail_style);
        ImGui::Dummy(canvas_size);
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
