// OHT-MCS Simulator: rail network plus a time-step OHT simulation in the live ImGui window.
// Run with --selftest for a headless sanity check, or --bench to compare dispatch policies.

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "rail/RailNetwork.h"
#include "rail/FabTopology.h"
#include "render/RailView.h"
#include "render/RailRenderer.h"
#include "render/VehicleRenderer.h"
#include "sim/Simulation.h"
#include "sim/Dispatch.h"

#include <GLFW/glfw3.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Headless dispatch comparison: same job stream and fleet, one policy at a time, swept over load.
// Isolates the dispatcher so the KPI gap is the dispatching effect, and shows where each saturates.
static void runDispatchBench(const rail::RailNetwork& net,
                             const sim::DispatchPolicy* const policies[], int n) {
    const float loads[] = {0.6f, 0.9f, 1.2f};
    const int kSteps = 12000;  // 600 sim seconds at dt 0.05
    const float minutes = kSteps * 0.05f / 60.0f;

    std::printf("\ndispatch benchmark: 12 OHT, seed 7, %d steps/cell, identical job stream per load\n", kSteps);
    std::printf("%6s  %-26s %9s %9s %9s %7s %6s\n",
                "load", "policy", "tput/min", "avg_cyc", "p95_cyc", "empty%", "done");
    for (float load : loads) {
        for (int i = 0; i < n; ++i) {
            sim::SimConfig cfg;
            cfg.seed = 7;
            cfg.oht_count = 12;
            cfg.arrival_per_sec = load;
            sim::Simulation s(net, cfg);
            s.setPolicy(policies[i]);
            for (int k = 0; k < kSteps; ++k) s.step(0.05f);
            sim::SimStats r = s.stats();
            float tput = r.jobs_done / minutes;  // sustained throughput over the whole run
            std::printf("%6.1f  %-26s %9.1f %9.1f %9.1f %6.0f%% %6d\n",
                        load, policies[i]->name(), tput, r.avg_delivery,
                        r.p95_delivery, r.empty_ratio * 100.0f, r.jobs_done);
        }
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    bool selftest = false;
    bool bench = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--selftest") == 0) selftest = true;
        else if (std::strcmp(argv[i], "--bench") == 0) bench = true;
    }

    // Build the synthetic fab once; the headless modes and the live window all share it.
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

    // Dispatch policies (STEP 3): shared by the live selector and the benchmark.
    sim::FirstIdlePolicy p_first;
    sim::NearestVehiclePolicy p_near;
    sim::GreedyMatchPolicy p_greedy;
    const sim::DispatchPolicy* policies[3] = {&p_first, &p_near, &p_greedy};

    if (bench) {
        runDispatchBench(net, policies, 3);
        return 0;
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

    const ImVec4 clear_color = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);

    render::RailViewStyle rail_style;
    rail_style.pad *= ui_scale;
    rail_style.node_r *= ui_scale;
    rail_style.port_half *= ui_scale;
    rail_style.seg_thick *= ui_scale;
    rail_style.bottleneck_thick *= ui_scale;
    rail_style.path_thick *= ui_scale;

    // Simulation: OHTs carry jobs across the fab. Driven by the fixed-dt accumulator in the loop.
    sim::SimConfig sim_cfg;
    sim::Simulation simulation(net, sim_cfg);
    int policy_sel = 1;  // start on nearest vehicle
    simulation.setPolicy(policies[policy_sel]);
    const char* policy_names[3] = {policies[0]->name(), policies[1]->name(), policies[2]->name()};
    int ui_oht_count = sim_cfg.oht_count;
    float ui_arrival = sim_cfg.arrival_per_sec;
    float sim_speed = 4.0f;

    const float kFixedDt = 0.05f;
    float sim_accum = 0.0f;
    float plot_timer = 0.0f;
    const int kPlotCap = 400;  // fixed-size scrolling window for the throughput plot
    std::vector<float> tput_time(kPlotCap, 0.0f);
    std::vector<float> tput_value(kPlotCap, 0.0f);
    int tput_count = 0;
    int tput_head = 0;

    if (selftest) {
        // Headless smoke test: overload the assignment path and verify job accounting holds.
        sim::SimConfig stress_cfg;
        stress_cfg.oht_count = 3;
        stress_cfg.arrival_per_sec = 5.0f;
        sim::Simulation stress(net, stress_cfg);
        for (int s = 0; s < 4000; ++s) stress.step(0.05f);
        sim::SimStats ss = stress.stats();
        bool ok = ss.jobs_pending == stress.pendingCount() && ss.jobs_done > 0;
        std::printf("selftest sim: done %d, active %d, pending %d (queue %d) => %s\n",
                    ss.jobs_done, ss.jobs_active, ss.jobs_pending, stress.pendingCount(),
                    ok ? "OK" : "FAIL");
        if (!ok) return 1;
    }

    int frame = 0;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Advance the sim in fixed steps; clamp substeps so a slow frame cannot spiral.
        float frame_dt = io.DeltaTime > 0.0f ? io.DeltaTime : (1.0f / 60.0f);
        sim_accum += frame_dt * sim_speed;
        int substeps = 0;
        while (sim_accum >= kFixedDt && substeps < 5) {
            simulation.step(kFixedDt);
            sim_accum -= kFixedDt;
            ++substeps;
        }
        if (substeps == 5) sim_accum = 0.0f;

        sim::SimStats st = simulation.stats();
        plot_timer += frame_dt * sim_speed;
        if (plot_timer >= 0.5f) {
            plot_timer = 0.0f;
            tput_time[tput_head] = st.sim_time;
            tput_value[tput_head] = st.throughput_per_min;
            tput_head = (tput_head + 1) % kPlotCap;
            if (tput_count < kPlotCap) ++tput_count;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Simulation");
        ImGui::Text("%.1f FPS   sim time %.0f s", io.Framerate, st.sim_time);
        if (ImGui::SliderInt("OHT count", &ui_oht_count, 1, 60)) simulation.setTargetOhtCount(ui_oht_count);
        if (ImGui::SliderFloat("Job arrival /s", &ui_arrival, 0.05f, 3.0f, "%.2f")) simulation.setArrivalRate(ui_arrival);
        ImGui::SliderFloat("Sim speed x", &sim_speed, 0.0f, 20.0f, "%.1f");
        if (ImGui::Combo("Dispatch policy", &policy_sel, policy_names, 3)) simulation.setPolicy(policies[policy_sel]);
        ImGui::Separator();
        ImGui::Text("OHT live %d / target %d   (busy %d, retiring %d)",
                    st.vehicles_live, st.vehicles_target, st.vehicles_busy, st.vehicles_retiring);
        ImGui::Text("Jobs: pending %d, active %d, done %d", st.jobs_pending, st.jobs_active, st.jobs_done);
        ImGui::Text("Throughput %.1f /min      Utilization %.0f%%", st.throughput_per_min, st.utilization * 100.0f);
        ImGui::Text("Cycle time avg %.1f, p95 %.1f (create to done)", st.avg_delivery, st.p95_delivery);
        ImGui::Text("Empty travel %.0f%% (deadhead share)", st.empty_ratio * 100.0f);
        if (tput_count > 0 && ImPlot::BeginPlot("Throughput", ImVec2(-1, 160))) {
            int offset = (tput_count == kPlotCap) ? tput_head : 0;
            ImPlot::SetupAxes("sim time (s)", "jobs/min");
            ImPlot::PlotLine("throughput", tput_time.data(), tput_value.data(), tput_count, 0, offset, sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::TextColored(ImVec4(0.55f, 0.78f, 0.98f, 1.0f),
                           "Cyan = empty OHT, yellow = carrying, grey = idle.");
        ImGui::End();

        ImGui::Begin("Rail Network");
        ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
        if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
        if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        render::RailView view = render::makeRailView(canvas_origin, canvas_size, net, rail_style.pad);
        render::drawRailNetwork(dl, view, net, path, rail_style);

        // Bridge the sim to the renderer here so neither core depends on the other.
        std::vector<render::VehicleMarker> markers;
        markers.reserve(simulation.vehicles().size());
        for (const sim::Vehicle& v : simulation.vehicles()) {
            if (!v.alive) continue;
            int state = 0;
            if (v.state == sim::VehState::ToPickup) state = 1;
            else if (v.state == sim::VehState::Carrying) state = 2;
            markers.push_back(render::VehicleMarker{simulation.vehicleWorldPos(v), state});
        }
        render::VehicleStyle veh_style;
        veh_style.radius = rail_style.port_half * 1.15f;
        render::drawVehicles(dl, view, markers, veh_style);

        ImGui::Dummy(canvas_size);
        ImGui::End();

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
