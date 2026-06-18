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

// Routing experiment: fix fleet and load, sweep the congestion weight lambda. lambda 0 is static
// shortest path; lambda > 0 detours around busy segments. Expect cycle time to fall then rise (a
// U-curve: too much detouring hurts), demonstrating an optimal congestion sensitivity.
static void runRouteBench(const rail::RailNetwork& net, const rail::FabDemo& demo) {
    const unsigned seeds[] = {1, 2, 3, 4, 5, 6, 7, 8};
    const int nseeds = static_cast<int>(sizeof(seeds) / sizeof(seeds[0]));
    const float lambdas[] = {0.0f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f};
    const int nlam = static_cast<int>(sizeof(lambdas) / sizeof(lambdas[0]));
    const int kWarmup = 1500, kMeasure = 4500;
    const float dt = 0.05f;
    const float measure_min = kMeasure * dt / 60.0f;
    sim::SimConfig base;
    base.oht_count = 14;
    base.arrival_per_sec = 0.3f;   // concentrated demand congests the bottleneck cross-link
    base.hot_fraction = 0.6f;      // 60% of jobs run the hot lane through the single cross-link
    base.hot_origin = demo.hot_a;
    base.hot_dest = demo.hot_b;

    std::printf("\nrouting benchmark: %d OHT, %.2f job/s (%.0f%% hot lane), %d seeds, lambda sweep (w = len*(1+lambda*cong))\n",
                base.oht_count, base.arrival_per_sec, base.hot_fraction * 100.0f, nseeds);
    std::filesystem::create_directories("data");
    std::ofstream out("data/route.csv");
    out << "lambda,seed,avg_cyc,p95_cyc,throughput_per_min,empty_ratio\n";
    std::printf("%8s %9s %9s %9s %8s\n", "lambda", "avg_cyc", "p95_cyc", "tput/min", "empty%");
    for (int li = 0; li < nlam; ++li) {
        double cyc = 0, p95 = 0, tp = 0, emp = 0;
        for (int si = 0; si < nseeds; ++si) {
            sim::SimConfig cfg = base;
            cfg.seed = seeds[si];
            sim::Simulation s(net, cfg);
            s.setRoutingLambda(lambdas[li]);
            for (int k = 0; k < kWarmup; ++k) s.step(dt);
            int done_w = s.stats().jobs_done;
            for (int k = 0; k < kMeasure; ++k) s.step(dt);
            sim::SimStats f = s.stats();
            float t = (f.jobs_done - done_w) / measure_min;
            out << lambdas[li] << "," << seeds[si] << "," << f.avg_delivery << ","
                << f.p95_delivery << "," << t << "," << f.empty_ratio << "\n";
            cyc += f.avg_delivery / nseeds;
            p95 += f.p95_delivery / nseeds;
            tp += t / nseeds;
            emp += f.empty_ratio * 100.0 / nseeds;
        }
        std::printf("%8.1f %9.1f %9.1f %9.1f %7.0f%%\n", lambdas[li], cyc, p95, tp, emp);
    }
    out.close();
    std::printf("wrote data/route.csv\n");
}

// Little's Law sanity check: on a stable (below-capacity) system in steady state, the time-averaged
// work-in-process L should equal throughput lambda times mean sojourn time W. Close agreement is
// evidence that the simulator's job accounting and timing are internally consistent.
static void runLittle(const rail::RailNetwork& net) {
    const unsigned seeds[] = {1, 2, 3, 4, 5, 6, 7, 8};
    const int nseeds = static_cast<int>(sizeof(seeds) / sizeof(seeds[0]));
    const int kWarmup = 3000;    // 150 s to reach steady state
    const int kMeasure = 12000;  // 600 s of measurement
    const float dt = 0.05f;
    sim::SimConfig base;
    base.oht_count = 12;
    base.arrival_per_sec = 0.2f;  // well below capacity so the queue stays bounded (steady state)

    double L = 0, lam = 0, W = 0;
    for (int si = 0; si < nseeds; ++si) {
        sim::SimConfig cfg = base;
        cfg.seed = seeds[si];
        sim::Simulation s(net, cfg);
        for (int k = 0; k < kWarmup; ++k) s.step(dt);
        sim::SimStats w = s.stats();
        int done_w = w.jobs_done;
        double cyc_w = s.cycleSum();
        double wip_sum = 0.0;
        for (int k = 0; k < kMeasure; ++k) {
            s.step(dt);
            sim::SimStats st = s.stats();
            wip_sum += st.jobs_pending + st.jobs_active;  // jobs in the system
        }
        int completed = s.stats().jobs_done - done_w;
        double Li = wip_sum / kMeasure;
        double lami = completed / (kMeasure * dt);
        double Wi = completed > 0 ? (s.cycleSum() - cyc_w) / completed : 0.0;
        L += Li / nseeds;
        lam += lami / nseeds;
        W += Wi / nseeds;
    }
    double pred = lam * W;
    double err = pred > 0 ? 100.0 * (L - pred) / pred : 0.0;
    std::printf("\nLittle's Law check: %d OHT, %.2f job/s (stable), %d seeds, %d measured steps\n",
                base.oht_count, base.arrival_per_sec, nseeds, kMeasure);
    std::printf("  L  measured avg WIP       = %.3f jobs\n", L);
    std::printf("  lambda  throughput        = %.4f /s\n", lam);
    std::printf("  W  avg sojourn time       = %.2f s\n", W);
    std::printf("  lambda * W  predicted L   = %.3f jobs\n", pred);
    std::printf("  error                     = %+.1f%%  => %s\n",
                err, (err > -5.0 && err < 5.0) ? "consistent (within 5%)" : "outside 5%, check stability");
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

// Routing lambda sweep aggregated across seeds, for the U-curve figure.
struct RouteSeries {
    std::vector<float> lambda, cyc, cyc_lo, cyc_hi, p95;
};

static RouteSeries loadRoute(const char* path) {
    RouteSeries s;
    std::ifstream in(path);
    if (!in) return s;
    struct Row { float lam, cyc, p95; };
    std::vector<Row> rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        float lam, seed, cyc, p95, tp, emp;
        if (std::sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f", &lam, &seed, &cyc, &p95, &tp, &emp) == 6)
            rows.push_back({lam, cyc, p95});
    }
    std::vector<float> lams;
    for (const Row& r : rows)
        if (std::find(lams.begin(), lams.end(), r.lam) == lams.end()) lams.push_back(r.lam);
    std::sort(lams.begin(), lams.end());
    for (float L : lams) {
        std::vector<float> c, p;
        for (const Row& r : rows) if (r.lam == L) { c.push_back(r.cyc); p.push_back(r.p95); }
        int n = static_cast<int>(c.size());
        double m = 0; for (float v : c) m += v; m /= n;
        double var = 0; for (float v : c) var += (v - m) * (v - m); var = n > 1 ? var / (n - 1) : 0.0;
        double ci = n > 1 ? 1.96 * std::sqrt(var) / std::sqrt((double)n) : 0.0;
        double pm = 0; for (float v : p) pm += v; pm /= n;
        s.lambda.push_back(L);
        s.cyc.push_back(static_cast<float>(m));
        s.cyc_lo.push_back(static_cast<float>(m - ci));
        s.cyc_hi.push_back(static_cast<float>(m + ci));
        s.p95.push_back(static_cast<float>(pm));
    }
    return s;
}

// A calm, flat "control room" theme so the console reads as professional software, not a demo.
static void applyConsoleStyle(float scale) {
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 4.0f;
    s.ChildRounding = 4.0f;
    s.FrameRounding = 3.0f;
    s.GrabRounding = 3.0f;
    s.ScrollbarRounding = 3.0f;
    s.TabRounding = 3.0f;
    s.WindowBorderSize = 1.0f;
    s.WindowPadding = ImVec2(12, 10);
    s.FramePadding = ImVec2(8, 5);
    s.ItemSpacing = ImVec2(8, 8);
    s.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]         = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    c[ImGuiCol_TitleBg]          = ImVec4(0.12f, 0.14f, 0.17f, 1.00f);
    c[ImGuiCol_TitleBgActive]    = ImVec4(0.16f, 0.21f, 0.29f, 1.00f);
    c[ImGuiCol_FrameBg]          = ImVec4(0.16f, 0.18f, 0.21f, 1.00f);
    c[ImGuiCol_FrameBgHovered]   = ImVec4(0.22f, 0.25f, 0.31f, 1.00f);
    c[ImGuiCol_FrameBgActive]    = ImVec4(0.26f, 0.31f, 0.39f, 1.00f);
    c[ImGuiCol_Button]           = ImVec4(0.20f, 0.27f, 0.36f, 1.00f);
    c[ImGuiCol_ButtonHovered]    = ImVec4(0.27f, 0.36f, 0.48f, 1.00f);
    c[ImGuiCol_ButtonActive]     = ImVec4(0.32f, 0.43f, 0.58f, 1.00f);
    c[ImGuiCol_Header]           = ImVec4(0.20f, 0.27f, 0.36f, 1.00f);
    c[ImGuiCol_HeaderHovered]    = ImVec4(0.27f, 0.36f, 0.48f, 1.00f);
    c[ImGuiCol_SliderGrab]       = ImVec4(0.38f, 0.58f, 0.82f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.48f, 0.68f, 0.92f, 1.00f);
    c[ImGuiCol_CheckMark]        = ImVec4(0.48f, 0.72f, 0.96f, 1.00f);
    c[ImGuiCol_Separator]        = ImVec4(0.24f, 0.27f, 0.33f, 1.00f);
    c[ImGuiCol_Border]           = ImVec4(0.22f, 0.25f, 0.31f, 1.00f);
    s.ScaleAllSizes(scale);
}

int main(int argc, char** argv) {
    bool selftest = false;
    bool bench = false;
    bool deadlock = false;
    bool sweep = false;
    bool route = false;
    bool little = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--selftest") == 0) selftest = true;
        else if (std::strcmp(argv[i], "--bench") == 0) bench = true;
        else if (std::strcmp(argv[i], "--deadlock") == 0) deadlock = true;
        else if (std::strcmp(argv[i], "--sweep") == 0) sweep = true;
        else if (std::strcmp(argv[i], "--route") == 0) route = true;
        else if (std::strcmp(argv[i], "--little") == 0) little = true;
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
    if (route) {
        runRouteBench(net, demo);
        return 0;
    }
    if (little) {
        runLittle(net);
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
        static_cast<int>(1480 * ui_scale), static_cast<int>(840 * ui_scale),
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

    applyConsoleStyle(ui_scale);

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
    float ui_lambda = simulation.routingLambda();
    bool ui_hot = false;
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
    std::vector<float> util_value(kPlotCap, 0.0f);
    int tput_count = 0;
    int tput_head = 0;

    std::vector<SweepSeries> sweep_series = loadSweep("data/results.csv");  // batch sweep results, if present
    RouteSeries route_series = loadRoute("data/route.csv");                 // routing lambda sweep, if present

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
                util_value[tput_head] = st.utilization * 100.0f;
                tput_head = (tput_head + 1) % kPlotCap;
                if (tput_count < kPlotCap) ++tput_count;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImVec2 disp = io.DisplaySize;
        float lw = 400.0f * ui_scale;
        float rw = 440.0f * ui_scale;
        float mw = disp.x - lw - rw;
        if (mw < 260.0f * ui_scale) mw = 260.0f * ui_scale;

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(lw, disp.y), ImGuiCond_FirstUseEver);
        ImGui::Begin("Control Console");
        ImGui::TextDisabled("OHT-MCS  |  %.0f FPS  |  sim time %.0f s", io.Framerate, st.sim_time);

        ImGui::SeparatorText("Fleet & workload");
        if (ImGui::SliderInt("OHT count", &ui_oht_count, 1, 60)) simulation.setTargetOhtCount(ui_oht_count);
        if (ImGui::SliderFloat("Job arrival (/s)", &ui_arrival, 0.05f, 3.0f, "%.2f")) simulation.setArrivalRate(ui_arrival);

        ImGui::SeparatorText("Playback");
        ImGui::SliderFloat("Sim speed (x)", &sim_speed, 0.0f, 20.0f, "%.1f");
        if (ImGui::Button(paused ? "Resume" : "Pause", ImVec2(96 * ui_scale, 0))) paused = !paused;
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(96 * ui_scale, 0))) { paused = true; do_step = true; }
        if (paused) { ImGui::SameLine(); ImGui::TextDisabled("paused"); }

        ImGui::SeparatorText("Control policy (MCS)");
        if (ImGui::Combo("Dispatch", &policy_sel, policy_names, 3)) simulation.setPolicy(policies[policy_sel]);
        if (ImGui::Checkbox("Deadlock avoidance", &ui_avoid)) simulation.setAvoidance(ui_avoid);
        ImGui::SameLine();
        ImGui::TextDisabled(ui_avoid ? "(safe-state gate)" : "(greedy: may deadlock)");
        if (ImGui::SliderFloat("Routing lambda", &ui_lambda, 0.0f, 8.0f, "%.1f")) simulation.setRoutingLambda(ui_lambda);
        if (ImGui::Checkbox("Hot lane demand", &ui_hot)) simulation.setHotspot(ui_hot ? 0.5f : 0.0f, demo.hot_a, demo.hot_b);

        ImGui::SeparatorText("Live KPIs");
        ImGui::Text("OHT: %d live / %d target   (busy %d)", st.vehicles_live, st.vehicles_target, st.vehicles_busy);
        ImGui::Text("Jobs: %d done  |  %d in-flight  |  %d queued", st.jobs_done, st.jobs_active, st.jobs_pending);
        ImGui::Text("Throughput: %.1f /min      Utilization: %.0f%%", st.throughput_per_min, st.utilization * 100.0f);
        ImGui::Text("Cycle time: %.1f s avg, %.1f s p95", st.avg_delivery, st.p95_delivery);
        ImGui::Text("Empty travel (deadhead): %.0f%%", st.empty_ratio * 100.0f);
        ImVec4 dead_col = st.vehicles_deadlocked > 0 ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                     : ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
        ImGui::TextColored(dead_col, "Waiting: %d      Deadlocked: %d", st.vehicles_blocked, st.vehicles_deadlocked);

        if (tput_count > 0 && ImPlot::BeginPlot("Throughput & utilization", ImVec2(-1, 175 * ui_scale))) {
            int offset = (tput_count == kPlotCap) ? tput_head : 0;
            ImPlot::SetupAxes("sim time (s)", "jobs / min");
            ImPlot::SetupAxis(ImAxis_Y2, "util (%)", ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_Y2, 0.0, 100.0, ImPlotCond_Always);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
            ImPlot::PlotLine("throughput", tput_time.data(), tput_value.data(), tput_count, 0, offset, sizeof(float));
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::PlotLine("utilization", tput_time.data(), util_value.data(), tput_count, 0, offset, sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(lw, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(mw, disp.y), ImGuiCond_FirstUseEver);
        ImGui::Begin("Fab Map (live)");
        ImGui::TextColored(ImVec4(0.55f, 0.78f, 0.98f, 1.0f),
                           "OHT: cyan empty, yellow carrying, grey idle      Rail: grey idle -> orange -> red busy");
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

        ImGui::SetNextWindowPos(ImVec2(lw + mw, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(rw, disp.y), ImGuiCond_FirstUseEver);
        ImGui::Begin("Analysis (batch experiments)");
        float ph = 190.0f * ui_scale;
        if (sweep_series.empty() && route_series.lambda.empty()) {
            ImGui::TextWrapped("No batch data. Run --sweep and/or --route to write data/*.csv, then restart.");
        } else {
            const SweepSeries* on = findSeries(sweep_series, 5.0f, 1);
            const SweepSeries* off = findSeries(sweep_series, 5.0f, 0);
            const SweepSeries* svc = findSeries(sweep_series, 0.2f, 1);
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
            if (!route_series.lambda.empty() &&
                ImPlot::BeginPlot("Cycle time vs routing lambda (hot lane)", ImVec2(-1, ph))) {
                ImPlot::SetupAxes("congestion weight lambda", "cycle time (s)");
                int n = static_cast<int>(route_series.lambda.size());
                ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                ImPlot::PlotShaded("avg 95% CI", route_series.lambda.data(), route_series.cyc_lo.data(),
                                   route_series.cyc_hi.data(), n);
                ImPlot::PopStyleVar();
                ImPlot::PlotLine("avg cycle", route_series.lambda.data(), route_series.cyc.data(), n);
                ImPlot::PlotLine("p95 cycle", route_series.lambda.data(), route_series.p95.data(), n);
                ImPlot::EndPlot();
            }
            ImGui::TextDisabled("Capture for the report. Regenerate with --sweep / --route.");
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
