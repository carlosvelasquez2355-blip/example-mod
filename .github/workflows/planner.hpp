// planner.hpp
// Pure C++ (no Geode / no GD): decision logic for the Review Assist mod.
// Works on plain axis-aligned boxes so it can be unit-tested on a PC.
//
// Units: GD "units" (a block is 30 units). Time: "ticks" = 1/60 s.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace assist {

struct Box {
    float x0, y0, x1, y1;
};

inline Box flipY(const Box& b) { return {b.x0, -b.y1, b.x1, -b.y0}; }

constexpr float kInf = 1e9f;

// ============================================================================
//  CUBE  (analytic jump arc)
// ============================================================================
struct CubeParams {
    float v0 = 11.180032f;    // jump impulse   (units / tick)
    float g = 0.958199f;      // gravity        (units / tick^2)
    float clusterGap = 60.f;  // hazards closer than this are cleared with ONE jump
    float margin = 3.f;       // extra safety margin (units)
    float lookahead = 420.f;  // how far ahead to look (units)
};

struct CubeDecision {
    bool jump = false;      // press now?
    bool hasEvent = false;  // there is an obstacle ahead
    float target = 0.f;     // ideal takeoff x (player centre)
};

inline CubeDecision planCube(const Box& playerIn, float vxTick, float stepDist,
                             const std::vector<Box>& hazardsIn,
                             const std::vector<Box>& solidsIn, bool flipped,
                             const CubeParams& p) {
    CubeDecision out;
    if (vxTick < 0.5f) return out;

    auto mir = [&](const Box& b) { return flipped ? flipY(b) : b; };
    const Box pl = mir(playerIn);

    const float w = pl.x1 - pl.x0;
    const float cx = (pl.x0 + pl.x1) * 0.5f;
    const float feet = pl.y0;
    const float head = pl.y1;
    const float airtime = 2.f * p.v0 / p.g;  // ticks

    auto rise = [&](float h) -> float {
        if (h <= 0.f) return 0.f;
        const float disc = p.v0 * p.v0 - 2.f * p.g * h;
        if (disc < 0.f) return -1.f;
        return (p.v0 - std::sqrt(disc)) / p.g;
    };

    struct Event {
        float start;
        float target;
    };
    std::vector<Event> events;

    // ---- hazards (spikes, saws ...) ---------------------------------------
    std::vector<Box> haz;
    for (const Box& hb : hazardsIn) {
        Box h = mir(hb);
        if (h.x1 < pl.x0) continue;
        if (h.x0 > pl.x1 + p.lookahead) continue;
        if (h.y1 <= feet + 1.f) continue;
        if (h.y0 >= head - 1.f) continue;
        haz.push_back(h);
    }
    std::sort(haz.begin(), haz.end(),
              [](const Box& a, const Box& b) { return a.x0 < b.x0; });

    for (size_t i = 0; i < haz.size();) {
        float xs = haz[i].x0, xe = haz[i].x1;
        size_t j = i + 1;
        while (j < haz.size() && haz[j].x0 - xe <= p.clusterGap) {
            xe = std::max(xe, haz[j].x1);
            ++j;
        }
        const float target = (xs + xe) * 0.5f - vxTick * airtime * 0.5f;
        events.push_back({xs, target});
        i = j;
    }

    // ---- solid walls / steps ----------------------------------------------
    for (const Box& sb : solidsIn) {
        Box s = mir(sb);
        if (s.x1 < pl.x0 - 1.f) continue;
        if (s.x0 > pl.x1 + p.lookahead) continue;
        const float h = s.y1 - feet;
        if (h <= 4.f) continue;
        if (s.y0 >= head - 2.f) continue;
        const float t1 = rise(h + p.margin);
        if (t1 < 0.f) continue;
        const float hi = s.x0 - w * 0.5f - p.margin - vxTick * t1;
        const float lo = s.x0 - vxTick * (airtime - t1);
        const float target = (lo <= hi) ? (lo + hi) * 0.5f : hi;
        events.push_back({s.x0, target});
    }

    if (events.empty()) return out;

    const Event* ev = &events[0];
    for (const Event& e : events)
        if (e.start < ev->start) ev = &e;

    out.hasEvent = true;
    out.target = ev->target;
    out.jump = (cx + 0.5f * stepDist >= ev->target);
    return out;
}

// ============================================================================
//  WAVE  (exact lattice search - the wave only has two moves: up / down)
// ============================================================================
struct WaveParams {
    float half = 5.f;      // hitbox half size
    float margin = 3.f;    // extra safety
    int steps = 22;        // look-ahead in ticks
    float comfort = 16.f;  // preferred clearance to walls / spikes
    float slope = 1.f;     // |dy / dx|  (1 = 45 degrees)
};

struct WaveDecision {
    bool up = false;
    bool found = false;
    int depth = 0;
};

inline WaveDecision planWave(float px, float py, float yRef, float vxTick,
                             const std::vector<Box>& obs, const WaveParams& p,
                             bool currentlyUp) {
    WaveDecision out;
    out.up = currentlyUp;
    if (vxTick < 0.5f) return out;

    const int N = std::max(1, p.steps);
    const float dx = vxTick;
    const float dy = p.slope * vxTick;
    const float R = p.half + p.margin + 0.5f * std::max(dx, dy);

    const float xMax = px + N * dx + R + p.comfort;
    std::vector<Box> near;
    for (const Box& b : obs)
        if (b.x1 + R >= px - R && b.x0 - R <= xMax) near.push_back(b);

    const float INF = 1e30f;
    const int W = N + 1;
    std::vector<float> cost(static_cast<size_t>(W) * W, INF);
    std::vector<signed char> first(static_cast<size_t>(W) * W, -1);
    cost[0] = 0.f;

    int bestDepth = 0;
    for (int d = 1; d <= N; ++d) {
        const float x = px + d * dx;
        bool any = false;
        for (int u = 0; u <= d; ++u) {
            const float y = py + (2 * u - d) * dy;

            float nodeCost = 0.f;
            bool dead = false;
            for (const Box& b : near) {
                if (x < b.x0 - R || x > b.x1 + R) continue;
                const float gap = std::max(b.y0 - y, y - b.y1);
                if (gap < R) {
                    dead = true;
                    break;
                }
                if (gap < R + p.comfort) {
                    const float k = R + p.comfort - gap;
                    nodeCost += k * k;
                }
            }
            if (dead) continue;
            const float drift = y - yRef;
            nodeCost += 0.02f * drift * drift;

            const float cUp = (u >= 1) ? cost[(d - 1) * W + (u - 1)] : INF;
            const float cDn = (u <= d - 1) ? cost[(d - 1) * W + u] : INF;
            if (cUp >= INF && cDn >= INF) continue;

            const bool viaUp = cUp < cDn;
            const float base = viaUp ? cUp : cDn;
            const int pu = viaUp ? u - 1 : u;
            cost[d * W + u] = base + nodeCost;
            first[d * W + u] =
                (d == 1) ? static_cast<signed char>(viaUp ? 1 : 0) : first[(d - 1) * W + pu];
            any = true;
        }
        if (!any) break;
        bestDepth = d;
    }

    if (bestDepth == 0) return out;

    float best = INF;
    int bestU = -1;
    for (int u = 0; u <= bestDepth; ++u) {
        const float c = cost[bestDepth * W + u];
        if (c < best) {
            best = c;
            bestU = u;
        }
    }
    if (bestU < 0) return out;

    bool up = first[bestDepth * W + bestU] == 1;
    float bestUp = INF, bestDn = INF;
    for (int u = 0; u <= bestDepth; ++u) {
        const float c = cost[bestDepth * W + u];
        if (c >= INF) continue;
        if (first[bestDepth * W + u] == 1) bestUp = std::min(bestUp, c);
        else bestDn = std::min(bestDn, c);
    }
    // Near-tie: zig-zag AROUND yRef instead of drifting away in one direction.
    if (std::fabs(bestUp - bestDn) < 1e-3f && bestUp < INF && bestDn < INF) {
        if (py < yRef - 0.5f) up = true;
        else if (py > yRef + 0.5f) up = false;
        else up = !currentlyUp;
    }

    out.up = up;
    out.found = true;
    out.depth = bestDepth;
    return out;
}

// ============================================================================
//  SHIP  (corridor tube + predictive bang-bang controller)
// ============================================================================
struct ShipParams {
    float halfW = 15.f;
    float halfH = 15.f;
    float margin = 4.f;
    float lookTicks = 20.f;
    float predTicks = 7.f;
    float minGap = 4.f;
};

struct Gap {
    float lo, hi;
};

inline Gap freeGapAt(float colX, float halfW, float pad, float yRef,
                     const std::vector<Box>& obs) {
    std::vector<std::pair<float, float>> blocked;
    for (const Box& b : obs) {
        if (b.x1 < colX - halfW || b.x0 > colX + halfW) continue;
        blocked.push_back({b.y0 - pad, b.y1 + pad});
    }
    if (blocked.empty()) return {-kInf, kInf};
    std::sort(blocked.begin(), blocked.end());

    std::vector<std::pair<float, float>> merged;
    for (const auto& iv : blocked) {
        if (!merged.empty() && iv.first <= merged.back().second)
            merged.back().second = std::max(merged.back().second, iv.second);
        else
            merged.push_back(iv);
    }

    std::vector<Gap> free;
    free.push_back({-kInf, merged.front().first});
    for (size_t i = 0; i + 1 < merged.size(); ++i)
        free.push_back({merged[i].second, merged[i + 1].first});
    free.push_back({merged.back().second, kInf});

    Gap best = {-kInf, kInf};
    float bestDist = kInf * 2.f;
    bool have = false;
    for (const Gap& g : free) {
        if (g.hi <= g.lo) continue;
        float d = 0.f;
        if (yRef < g.lo) d = g.lo - yRef;
        else if (yRef > g.hi) d = yRef - g.hi;
        if (!have || d < bestDist) {
            best = g;
            bestDist = d;
            have = true;
        }
    }
    return best;
}

struct ShipDecision {
    bool up = false;
    float target = 0.f;
    bool bounded = false;
};

inline ShipDecision planShip(float px, float py, float vyTick, float vxTick,
                             const std::vector<Box>& obs, const ShipParams& p,
                             bool prevUp) {
    ShipDecision out;
    out.up = prevUp;
    if (vxTick < 0.5f) return out;

    const float pad = p.halfH + p.margin;

    Gap tube = freeGapAt(px, p.halfW, pad, py, obs);
    for (float t = 2.f; t <= p.lookTicks; t += 2.f) {
        const Gap g = freeGapAt(px + vxTick * t, p.halfW, pad, py, obs);
        Gap cand = {std::max(tube.lo, g.lo), std::min(tube.hi, g.hi)};
        if (cand.hi - cand.lo < p.minGap) break;
        tube = cand;
    }

    float target;
    const bool lowBound = tube.lo > -kInf * 0.5f;
    const bool highBound = tube.hi < kInf * 0.5f;
    if (lowBound && highBound) {
        target = (tube.lo + tube.hi) * 0.5f;
        out.bounded = true;
    } else if (lowBound) {
        target = std::max(py, tube.lo + 8.f);
    } else if (highBound) {
        target = std::min(py, tube.hi - 8.f);
    } else {
        target = py;
    }
    out.target = target;

    const float yPred = py + vyTick * p.predTicks;
    if (yPred < target - 1.5f) out.up = true;
    else if (yPred > target + 1.5f) out.up = false;
    return out;
}

}  // namespace assist
