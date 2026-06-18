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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
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

// Headless proof of the deadlock toggle: same high-contention scenario, avoidance OFF then ON.
// OFF is expected to deadlock (vehicles trapped in a wait-for cycle); ON must never deadlock.
static void runDeadlockCheck(const rail::RailNetwork& net) {
    const unsigned seeds[] = {1, 2, 3, 4, 5};
    const int nseeds = static_cast<int>(sizeof(seeds) / sizeof(seeds[0]));
    const int kSteps = 6000;
    sim::SimConfig base;
    base.oht_count = 10;
    base.arrival_per_sec = 1.5f;

    std::printf("\ndeadlock check: %d OHT, %.1f job/s, %d steps x %d seeds (high contention)\n",
                base.oht_count, base.arrival_per_sec, kSteps, nseeds);
    std::printf("%-4s %-22s %16s %10s %9s\n", "mode", "verdict", "seedsDeadlocked", "done", "maxDead");
    for (int avoid = 0; avoid <= 1; ++avoid) {
        long total_done = 0;
        int worst_dead = 0;
        int seeds_dead = 0;
        for (int si = 0; si < nseeds; ++si) {
            sim::SimConfig cfg = base;
            cfg.seed = seeds[si];
            sim::Simulation s(net, cfg);
            s.setAvoidance(avoid != 0);
            int maxdead = 0;
            for (int k = 0; k < kSteps; ++k) {
                s.step(0.05f);
                int d = s.stats().vehicles_deadlocked;
                if (d > maxdead) maxdead = d;
            }
            sim::SimStats fin = s.stats();
            total_done += fin.jobs_done;
            if (maxdead > worst_dead) worst_dead = maxdead;
            if (fin.vehicles_deadlocked > 0) ++seeds_dead;  // ended stuck in a wait-for cycle
        }
        const char* mode = avoid ? "ON" : "OFF";
        const char* verdict = avoid ? (worst_dead == 0 ? "deadlock-free (PASS)" : "FAIL: deadlock under ON")
                                    : (seeds_dead > 0 ? "deadlocks (expected)" : "no deadlock seen");
        char seedstr[16];
        std::snprintf(seedstr, sizeof(seedstr), "%d/%d", seeds_dead, nseeds);
        std::printf("%-4s %-22s %16s %10ld %9d\n", mode, verdict, seedstr, total_done, worst_dead);
    }
}

// Batch experiment: two regimes over the same fleet sweep, seeds for confidence intervals.
//  - capacity/deadlock (oversaturated): throughput is the achievable capacity; ON saturates, OFF
//    deadlocks as the fleet grows.
//  - service/sizing (moderate demand, ON): throughput meets demand at some fleet, beyond which extra
//    OHTs only drop utilization, which locates the right fleet size.
static void runSweep(const rail::RailNetwork& net) {
    namespace fs = std::filesystem;
    fs::create_directories("data");

    const float kSpeed = 80.0f;
    const float kDt = 0.05f;
    const int kWarmup = 1500;      // steps to reach steady state before measuring
    const int kMeasure = 4500;     // measured steps (225 sim seconds)
    const float measure_min = kMeasure * kDt / 60.0f;
    const unsigned seeds[] = {1, 2, 3, 4, 5, 6, 7, 8};
    const int nseeds = static_cast<int>(sizeof(seeds) / sizeof(seeds[0]));
    const int ohts[] = {2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28, 32, 36, 40};
    const int noht = static_cast<int>(sizeof(ohts) / sizeof(ohts[0]));

    const float kCapacityLoad = 5.0f;  // oversaturated
    const float kServiceLoad = 0.2f;   // moderate demand, well below capacity
    struct Pass { float arrival; int avoid; };
    const Pass passes[] = {{kCapacityLoad, 1}, {kCapacityLoad, 0}, {kServiceLoad, 1}};
    const int npass = static_cast<int>(sizeof(passes) / sizeof(passes[0]));

    std::ofstream out("data/results.csv");
    out << "arrival,avoidance,oht_count,seed,throughput_per_min,utilization,cycle_avg,cycle_p95,empty_ratio,max_deadlocked,jobs_done\n";

    std::printf("\nsweep: %d fleet points x %d seeds x %d passes, warmup %d + measure %d steps\n",
                noht, nseeds, npass, kWarmup, kMeasure);

    std::vector<std::vector<double>> mean_tput(npass, std::vector<double>(noht, 0.0));
    std::vector<std::vector<double>> mean_util(npass, std::vector<double>(noht, 0.0));
    for (int pi = 0; pi < npass; ++pi) {
        for (int oi = 0; oi < noht; ++oi) {
            for (int si = 0; si < nseeds; ++si) {
                sim::SimConfig cfg;
                cfg.seed = seeds[si];
                cfg.oht_count = ohts[oi];
                cfg.arrival_per_sec = passes[pi].arrival;
                cfg.speed = kSpeed;
                sim::Simulation s(net, cfg);
                s.setAvoidance(passes[pi].avoid != 0);
                for (int k = 0; k < kWarmup; ++k) s.step(kDt);
                int done_w = s.stats().jobs_done;
                int maxdead = 0;
                for (int k = 0; k < kMeasure; ++k) {
                    s.step(kDt);
                    if (k % 50 == 0) {  // deadlock persists once formed, so sampling is enough
                        int d = s.stats().vehicles_deadlocked;
                        if (d > maxdead) maxdead = d;
                    }
                }
                sim::SimStats f = s.stats();
                float tput = (f.jobs_done - done_w) / measure_min;
                out << passes[pi].arrival << "," << passes[pi].avoid << "," << ohts[oi] << "," << seeds[si] << ","
                    << tput << "," << f.utilization << "," << f.avg_delivery << ","
                    << f.p95_delivery << "," << f.empty_ratio << "," << maxdead << ","
                    << f.jobs_done << "\n";
                mean_tput[pi][oi] += tput / nseeds;
                mean_util[pi][oi] += f.utilization / nseeds;
            }
        }
        std::printf("  pass %d (arrival %.1f, avoidance %d) done\n", pi + 1, passes[pi].arrival, passes[pi].avoid);
    }
    out.close();

    std::ofstream p("data/params.csv");
    p << "key,value\n";
    p << "capacity_load_per_sec," << kCapacityLoad << "\n";
    p << "service_load_per_sec," << kServiceLoad << "\n";
    p << "oht_speed," << kSpeed << "\n";
    p << "dt_seconds," << kDt << "\n";
    p << "warmup_steps," << kWarmup << "\n";
    p << "measure_steps," << kMeasure << "\n";
    p << "seeds_per_point," << nseeds << "\n";
    p << "oht_min," << ohts[0] << "\n";
    p << "oht_max," << ohts[noht - 1] << "\n";
    p << "nodes," << net.nodes().size() << "\n";
    p << "segments," << net.segments().size() << "\n";
    p.close();

    // Console summary. Capacity peak (pass 0) and the service knee (pass 2: smallest fleet that reaches
    // 95% of the achievable throughput, i.e. adding OHTs past it only drops utilization).
    int cap_peak = 0;
    for (int oi = 1; oi < noht; ++oi) if (mean_tput[0][oi] > mean_tput[0][cap_peak]) cap_peak = oi;
    double svc_max = 0.0;
    for (int oi = 0; oi < noht; ++oi) if (mean_tput[2][oi] > svc_max) svc_max = mean_tput[2][oi];
    int knee = 0;
    for (int oi = 0; oi < noht; ++oi) if (mean_tput[2][oi] >= 0.95 * svc_max) { knee = oi; break; }
    std::printf("\ncapacity (arrival %.1f, ON): throughput peaks at %.1f /min, %d OHT\n",
                kCapacityLoad, mean_tput[0][cap_peak], ohts[cap_peak]);
    std::printf("service  (arrival %.1f = %.0f/min demand, ON): %d OHT reaches 95%% of achievable %.1f /min (util %.0f%%)\n",
                kServiceLoad, kServiceLoad * 60.0f, ohts[knee], svc_max, mean_util[2][knee] * 100.0);
    std::printf("wrote data/results.csv and data/params.csv\n");
}

// One (arrival, avoidance) curve over fleet size, aggregated across seeds with a 95% CI on throughput.
struct SweepSeries {
    float arrival = 0.0f;
    int avoidance = 0;
    std::vector<float> oht, tput, tput_lo, tput_hi, util, cycle;
};

// Loads data/results.csv (written by --sweep) and aggregates seeds into per-(arrival, avoidance) curves.
static std::vector<SweepSeries> loadSweep(const char* path) {
    std::vector<SweepSeries> out;
    std::ifstream in(path);
    if (!in) return out;

    struct Row { float arrival; int avoid; int oht; float tput, util, cycle; };
    std::vector<Row> rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        float a, av, o, seed, tp, ut, cy, p95, emp, md, done;
        if (std::sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                        &a, &av, &o, &seed, &tp, &ut, &cy, &p95, &emp, &md, &done) == 11) {
            rows.push_back({a, static_cast<int>(av), static_cast<int>(o), tp, ut, cy});
        }
    }

    std::vector<std::pair<float, int>> keys;  // distinct (arrival, avoidance), insertion order
    for (const Row& r : rows) {
        bool seen = false;
        for (auto& k : keys) if (k.first == r.arrival && k.second == r.avoid) { seen = true; break; }
        if (!seen) keys.push_back({r.arrival, r.avoid});
    }
    for (auto& k : keys) {
        SweepSeries s;
        s.arrival = k.first;
        s.avoidance = k.second;
        std::vector<int> ohts;
        for (const Row& r : rows)
            if (r.arrival == k.first && r.avoid == k.second &&
                std::find(ohts.begin(), ohts.end(), r.oht) == ohts.end())
                ohts.push_back(r.oht);
        std::sort(ohts.begin(), ohts.end());
        for (int o : ohts) {
            std::vector<float> tp, ut, cy;
            for (const Row& r : rows)
                if (r.arrival == k.first && r.avoid == k.second && r.oht == o) {
                    tp.push_back(r.tput); ut.push_back(r.util); cy.push_back(r.cycle);
                }
            int n = static_cast<int>(tp.size());
            double m = 0; for (float v : tp) m += v; m /= n;
            double var = 0; for (float v : tp) var += (v - m) * (v - m); var = n > 1 ? var / (n - 1) : 0.0;
            double ci = n > 1 ? 1.96 * std::sqrt(var) / std::sqrt((double)n) : 0.0;
            double um = 0; for (float v : ut) um += v; um /= n;
            double cm = 0; for (float v : cy) cm += v; cm /= n;
            s.oht.push_back(static_cast<float>(o));
            s.tput.push_back(static_cast<float>(m));
            s.tput_lo.push_back(static_cast<float>(m - ci));
            s.tput_hi.push_back(static_cast<float>(m + ci));
            s.util.push_back(static_cast<float>(um * 100.0));
            s.cycle.push_back(static_cast<float>(cm));
        }
        out.push_back(s);
    }
    return out;
}

static const SweepSeries* findSeries(const std::vector<SweepSeries>& v, float arrival, int avoidance) {
    for (const SweepSeries& s : v) if (s.arrival == arrival && s.avoidance == avoidance) return &s;
    return nullptr;
}

int main(int argc, char** argv) {
    bool selftest = false;
    bool bench = false;
    bool deadlock = false;
    bool sweep = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--selftest") == 0) selftest = true;
        else if (std::strcmp(argv[i], "--bench") == 0) bench = true;
        else if (std::strcmp(argv[i], "--deadlock") == 0) deadlock = true;
        else if (std::strcmp(argv[i], "--sweep") == 0) sweep = true;
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
    if (deadlock) {
        runDeadlockCheck(net);
        return 0;
    }
    if (sweep) {
        runSweep(net);
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
    bool ui_avoid = simulation.avoidance();
    int ui_oht_count = sim_cfg.oht_count;
    float ui_arrival = sim_cfg.arrival_per_sec;
    float sim_speed = 4.0f;
    bool paused = false;
    bool do_step = false;  // advance exactly one fixed step while paused

    const float kFixedDt = 0.05f;
    float sim_accum = 0.0f;
    float plot_timer = 0.0f;
    const int kPlotCap = 400;  // fixed-size scrolling window for the throughput plot
    std::vector<float> tput_time(kPlotCap, 0.0f);
    std::vector<float> tput_value(kPlotCap, 0.0f);
    int tput_count = 0;
    int tput_head = 0;

    std::vector<SweepSeries> sweep_series = loadSweep("data/results.csv");  // STEP 6 batch results, if present

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
        // Paused freezes time; Step advances exactly one fixed step for frame-by-frame inspection.
        float frame_dt = io.DeltaTime > 0.0f ? io.DeltaTime : (1.0f / 60.0f);
        if (paused) {
            sim_accum = 0.0f;
            if (do_step) { simulation.step(kFixedDt); do_step = false; }
        } else {
            sim_accum += frame_dt * sim_speed;
            int substeps = 0;
            while (sim_accum >= kFixedDt && substeps < 5) {
                simulation.step(kFixedDt);
                sim_accum -= kFixedDt;
                ++substeps;
            }
            if (substeps == 5) sim_accum = 0.0f;
        }

        sim::SimStats st = simulation.stats();
        if (!paused) {
            plot_timer += frame_dt * sim_speed;
            if (plot_timer >= 0.5f) {
                plot_timer = 0.0f;
                tput_time[tput_head] = st.sim_time;
                tput_value[tput_head] = st.throughput_per_min;
                tput_head = (tput_head + 1) % kPlotCap;
                if (tput_count < kPlotCap) ++tput_count;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Simulation");
        ImGui::Text("%.1f FPS   sim time %.0f s", io.Framerate, st.sim_time);
        if (ImGui::SliderInt("OHT count", &ui_oht_count, 1, 60)) simulation.setTargetOhtCount(ui_oht_count);
        if (ImGui::SliderFloat("Job arrival /s", &ui_arrival, 0.05f, 3.0f, "%.2f")) simulation.setArrivalRate(ui_arrival);
        ImGui::SliderFloat("Sim speed x", &sim_speed, 0.0f, 20.0f, "%.1f");
        if (ImGui::Button(paused ? "Resume" : "Pause")) paused = !paused;
        ImGui::SameLine();
        if (ImGui::Button("Step")) { paused = true; do_step = true; }
        if (paused) { ImGui::SameLine(); ImGui::TextDisabled("(paused)"); }
        if (ImGui::Combo("Dispatch policy", &policy_sel, policy_names, 3)) simulation.setPolicy(policies[policy_sel]);
        if (ImGui::Checkbox("Deadlock avoidance", &ui_avoid)) simulation.setAvoidance(ui_avoid);
        ImGui::SameLine();
        ImGui::TextDisabled(ui_avoid ? "(ON: safe-state gate)" : "(OFF: greedy, may deadlock)");
        ImGui::Separator();
        ImGui::Text("OHT live %d / target %d   (busy %d, retiring %d)",
                    st.vehicles_live, st.vehicles_target, st.vehicles_busy, st.vehicles_retiring);
        ImGui::Text("Jobs: pending %d, active %d, done %d", st.jobs_pending, st.jobs_active, st.jobs_done);
        ImGui::Text("Throughput %.1f /min      Utilization %.0f%%", st.throughput_per_min, st.utilization * 100.0f);
        ImGui::Text("Cycle time avg %.1f, p95 %.1f (create to done)", st.avg_delivery, st.p95_delivery);
        ImGui::Text("Empty travel %.0f%% (deadhead share)", st.empty_ratio * 100.0f);
        ImVec4 dead_col = st.vehicles_deadlocked > 0 ? ImVec4(0.95f, 0.35f, 0.30f, 1.0f)
                                                     : ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
        ImGui::TextColored(dead_col, "Waiting %d, deadlocked %d", st.vehicles_blocked, st.vehicles_deadlocked);
        if (tput_count > 0 && ImPlot::BeginPlot("Throughput", ImVec2(-1, 160))) {
            int offset = (tput_count == kPlotCap) ? tput_head : 0;
            ImPlot::SetupAxes("sim time (s)", "jobs/min");
            ImPlot::PlotLine("throughput", tput_time.data(), tput_value.data(), tput_count, 0, offset, sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::TextColored(ImVec4(0.55f, 0.78f, 0.98f, 1.0f),
                           "Cyan = empty OHT, yellow = carrying, grey = idle.");
        ImGui::TextDisabled("Rail color = live congestion (grey idle -> orange -> red busy).");
        ImGui::End();

        ImGui::Begin("Rail Network");
        ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
        if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
        if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        render::RailView view = render::makeRailView(canvas_origin, canvas_size, net, rail_style.pad);
        render::drawRailNetwork(dl, view, net, path, rail_style, &simulation.segmentCongestion());

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

        ImGui::Begin("Analysis (batch results)");
        if (sweep_series.empty()) {
            ImGui::TextWrapped("No batch data. Run 'oht_mcs_simulator --sweep' to write data/results.csv, then restart.");
        } else {
            const SweepSeries* on = findSeries(sweep_series, 5.0f, 1);
            const SweepSeries* off = findSeries(sweep_series, 5.0f, 0);
            const SweepSeries* svc = findSeries(sweep_series, 0.2f, 1);
            float ph = 200.0f * ui_scale;
            if (on && off && ImPlot::BeginPlot("Throughput vs fleet (saturating load)", ImVec2(-1, ph))) {
                ImPlot::SetupAxes("OHT count", "throughput (jobs/min)");
                ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                ImPlot::PlotShaded("ON 95% CI", on->oht.data(), on->tput_lo.data(), on->tput_hi.data(),
                                   static_cast<int>(on->oht.size()));
                ImPlot::PopStyleVar();
                ImPlot::PlotLine("avoidance ON", on->oht.data(), on->tput.data(), static_cast<int>(on->oht.size()));
                ImPlot::PlotLine("avoidance OFF", off->oht.data(), off->tput.data(), static_cast<int>(off->oht.size()));
                ImPlot::EndPlot();
            }
            if (svc && !svc->oht.empty() && ImPlot::BeginPlot("Utilization vs fleet (service load)", ImVec2(-1, ph))) {
                ImPlot::SetupAxes("OHT count", "utilization (%)");
                ImPlot::PlotLine("utilization", svc->oht.data(), svc->util.data(), static_cast<int>(svc->oht.size()));
                float rx[2] = {svc->oht.front(), svc->oht.back()};
                float ry[2] = {75.0f, 75.0f};
                ImPlot::PlotLine("75% target", rx, ry, 2);
                ImPlot::EndPlot();
            }
            if (svc && ImPlot::BeginPlot("Cycle time vs fleet (service load)", ImVec2(-1, ph))) {
                ImPlot::SetupAxes("OHT count", "cycle time (s)");
                ImPlot::PlotLine("avg cycle", svc->oht.data(), svc->cycle.data(), static_cast<int>(svc->oht.size()));
                ImPlot::EndPlot();
            }
            ImGui::TextDisabled("Capture for the report. Regenerate with --sweep.");
        }
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
