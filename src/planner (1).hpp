// planner.hpp - trajectory search for the Review Assist mod.
//
// Pure C++ (no Geode / cocos), so it can be unit-tested on a PC.
//
// IDEA: the horizontal speed is constant, so "time" and "x" are the same thing.
// For every tick we try "button held" / "button released", simulate the player
// forward with a small physics model of the current game mode and keep a path
// that reaches the end of the look-ahead without touching a hazard. The first
// action of that path is what the bot does now. The search is repeated every
// tick with the real position / speed, so small model errors are corrected.
//
// All numbers are "units per 1/60 s tick". Up is +y. A gravity-flipped player is
// handled with the sign `s` (+1 normal, -1 flipped).
//
// The physics constants below are APPROXIMATIONS of Geometry Dash 2.2. They are
// the first thing to tune if a mode feels off.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace assist {

struct Box {
    float x0, y0, x1, y1;
};

enum class Mode : int { Cube, Ship, Ball, Ufo, Wave, Robot, Spider, Swing };

enum class Kind : int {
    YellowOrb, PinkOrb, RedOrb, BlueOrb, GreenOrb, BlackOrb,
    YellowPad, PinkPad, RedPad, BluePad
};

inline bool isPad(Kind k) { return static_cast<int>(k) >= static_cast<int>(Kind::YellowPad); }

struct Special {
    Box box;
    Kind kind;
};

struct World {
    Mode mode = Mode::Cube;
    float x = 0.f;            // centre x of the player now
    float vx = 5.193f;        // units / tick
    float half = 15.f;        // half size of the player's hitbox
    float slope = 1.f;        // wave: |vy| / vx
    float scale = 1.f;        // physics multiplier (tests use it to break the model)
    float margin = 1.f;       // extra safety around the player vs blocks
    float sideTol = 6.f;      // grazing a block's side by less than this does not kill
    float gscale = 1.f;       // gravity calibration (measured in game)
    float jscale = 1.f;       // jump-speed calibration (measured in game)
    float shipUp = 0.66f;     // ship acceleration while held   (measured in game)
    float shipDown = 0.60f;   // ship acceleration while released (measured in game)
    bool hasFloor = false, hasCeil = false;
    float floorY = 0.f, ceilY = 0.f;
    float horizon = 1e9f;     // look-ahead limit in units (e.g. next portal)
    std::vector<Box> hazards; // kill on touch
    std::vector<Box> solids;  // blocks
    std::vector<Special> specials;
};

// ---------------------------------------------------------------- constants
namespace phys {
constexpr float G = 0.958199f;
constexpr float TERM = 15.f;
constexpr float CUBE_JUMP = 11.180032f;

constexpr float SHIP_DOWN = 0.60f, SHIP_UP = 0.66f, SHIP_MAX_UP = 8.f, SHIP_MAX_DOWN = 6.4f;

constexpr float UFO_G = 0.64f, UFO_JUMP = 7.4f, UFO_TERM = 10.f;
constexpr float BALL_G = 0.76f;
constexpr float ROBOT_V = 7.6f;
constexpr int ROBOT_T = 9;
constexpr float SWING_A = 0.9f, SWING_MAX = 8.f;

constexpr float ORB_YELLOW = 12.f, ORB_PINK = 7.5f, ORB_RED = 15.5f;
constexpr float ORB_GREEN = 8.f, ORB_BLACK = 14.f, ORB_BLUE = 2.f;
constexpr float PAD_YELLOW = 16.f, PAD_PINK = 10.4f, PAD_RED = 20.f;
}  // namespace phys

// -------------------------------------------------------------------- state
struct N {
    float y = 0.f, vy = 0.f;
    int8_t s = 1;        // +1 normal gravity, -1 flipped
    bool ground = false;
    bool held = false;
    bool used = false;   // an orb was used since the button was pressed
    int8_t robotT = 0;
    int8_t pad = -1;     // pad currently touched (so it only fires once)
};

inline bool hit(float cx, float cy, float h, const Box& b) {
    return cx + h > b.x0 && cx - h < b.x1 && cy + h > b.y0 && cy - h < b.y1;
}

inline void applyKind(Kind k, N& n, float sc) {  // sc = jump scale
    switch (k) {
        case Kind::YellowOrb: n.vy = n.s * phys::ORB_YELLOW * sc; break;
        case Kind::PinkOrb: n.vy = n.s * phys::ORB_PINK * sc; break;
        case Kind::RedOrb: n.vy = n.s * phys::ORB_RED * sc; break;
        case Kind::BlueOrb: n.s = -n.s; n.vy = -n.s * phys::ORB_BLUE; break;
        case Kind::GreenOrb: n.s = -n.s; n.vy = -n.s * phys::ORB_GREEN; break;
        case Kind::BlackOrb: n.vy = -n.s * phys::ORB_BLACK; break;
        case Kind::YellowPad: n.vy = n.s * phys::PAD_YELLOW * sc; break;
        case Kind::PinkPad: n.vy = n.s * phys::PAD_PINK * sc; break;
        case Kind::RedPad: n.vy = n.s * phys::PAD_RED * sc; break;
        case Kind::BluePad: n.s = -n.s; n.vy = -n.s * phys::ORB_BLUE; break;
    }
    n.ground = false;
    n.robotT = 0;
}

// Spider: jump to the first surface in the new gravity direction.
inline void spiderTeleport(const World& w, N& n, float x) {
    const float half = w.half;
    const float dir = -static_cast<float>(n.s);  // new "down" direction
    float target = n.y;
    bool found = false;
    auto consider = [&](float cand) {
        if (!found || (dir > 0 ? cand < target : cand > target)) {
            target = cand;
            found = true;
        }
    };
    for (auto const& b : w.solids) {
        if (x + half <= b.x0 || x - half >= b.x1) continue;
        if (dir > 0 && b.y0 >= n.y + half - 1.f) consider(b.y0 - half);
        if (dir < 0 && b.y1 <= n.y - half + 1.f) consider(b.y1 + half);
    }
    if (dir > 0 && w.hasCeil) consider(w.ceilY - half);
    if (dir < 0 && w.hasFloor) consider(w.floorY + half);
    if (found) {
        n.y = target;
        n.ground = true;
    }
    n.vy = 0.f;
}

// Collisions with blocks and the floor / ceiling. false = dead.
inline bool resolve(const World& w, N& n, float x, float prevY) {
    const float half = w.half + w.margin;
    const float tol = 7.f;
    for (auto const& b : w.solids) {
        if (!hit(x, n.y, half, b)) continue;
        if (w.mode == Mode::Wave) return false;
        const float pb = prevY - half, pt = prevY + half;
        if (n.s > 0) {
            if (pb >= b.y1 - tol && n.vy <= 0.f) {
                n.y = b.y1 + half; n.vy = 0.f; n.ground = true;
            } else if (pt <= b.y0 + tol && n.vy > 0.f) {
                n.y = b.y0 - half; n.vy = 0.f;
            } else {
                // side hit: the real game only kills when the smaller inner box touches
                if (std::min(x + half - b.x0, b.x1 - (x - half)) < w.sideTol) continue;
                return false;
            }
        } else {
            if (pt <= b.y0 + tol && n.vy >= 0.f) {
                n.y = b.y0 - half; n.vy = 0.f; n.ground = true;
            } else if (pb >= b.y1 - tol && n.vy < 0.f) {
                n.y = b.y1 + half; n.vy = 0.f;
            } else {
                if (std::min(x + half - b.x0, b.x1 - (x - half)) < w.sideTol) continue;
                return false;
            }
        }
    }
    if (w.hasFloor && n.y - half < w.floorY) {
        n.y = w.floorY + half;
        if (n.vy < 0.f) n.vy = 0.f;
        if (n.s > 0) n.ground = true;
    }
    if (w.hasCeil && n.y + half > w.ceilY) {
        n.y = w.ceilY - half;
        if (n.vy > 0.f) n.vy = 0.f;
        if (n.s < 0) n.ground = true;
    }
    return true;
}

// One 1/60 s tick. `x` = centre x at the start of the tick. false = died.
inline bool step(const World& w, N& n, bool a, float x) {
    using namespace phys;
    const float sc = w.scale;            // overall (tests)
    const float js = w.scale * w.jscale;  // jump speeds
    const float gs = w.scale * w.gscale;  // gravity
    const bool edge = a && !n.held;
    if (!a) n.used = false;

    // ---- input: orbs first, then what the mode does with a click ----------
    bool consumed = false;
    if (a && !n.used && w.mode != Mode::Wave) {
        for (auto const& sp : w.specials) {
            if (isPad(sp.kind)) continue;
            if (hit(x, n.y, w.half, sp.box)) {
                applyKind(sp.kind, n, js);
                n.used = true;
                consumed = true;
                break;
            }
        }
    }
    if (!consumed) {
        switch (w.mode) {
            case Mode::Cube:
                if (a && n.ground) { n.vy = n.s * CUBE_JUMP * js; n.ground = false; }
                break;
            case Mode::Robot:
                if (a && n.ground) { n.vy = n.s * ROBOT_V * js; n.ground = false; n.robotT = ROBOT_T; }
                break;
            case Mode::Ufo:
                if (edge) { n.vy = n.s * UFO_JUMP * js; n.ground = false; }
                break;
            case Mode::Ball:
                if (edge && n.ground) { n.s = -n.s; n.vy = 0.f; n.ground = false; }
                break;
            case Mode::Spider:
                if (edge && n.ground) { n.s = -n.s; spiderTeleport(w, n, x); }
                break;
            case Mode::Swing:
                if (edge) { n.s = -n.s; n.vy *= 0.3f; }
                break;
            default: break;
        }
    }
    if (!a) n.robotT = 0;

    // ---- forces ------------------------------------------------------------
    n.ground = (w.mode == Mode::Spider && n.ground);  // spider keeps the teleport landing
    float vy = n.vy;
    const float s = static_cast<float>(n.s);
    switch (w.mode) {
        case Mode::Cube:
        case Mode::Spider:
        case Mode::Robot:
        case Mode::Ball:
        case Mode::Ufo: {
            const float g = (w.mode == Mode::Ufo ? UFO_G : w.mode == Mode::Ball ? BALL_G : G) * gs;
            const float term = w.mode == Mode::Ufo ? UFO_TERM : TERM;
            if (w.mode == Mode::Robot && a && n.robotT > 0 && s * vy > 0.f) {
                n.robotT--;
                vy = s * ROBOT_V * js;
            } else {
                vy -= s * g;
            }
            if (s * vy < -term) vy = -s * term;
            break;
        }
        case Mode::Ship:
            vy += a ? s * w.shipUp * sc : -s * w.shipDown * sc;
            vy = s * std::clamp(s * vy, -SHIP_MAX_DOWN, SHIP_MAX_UP);
            break;
        case Mode::Swing:
            vy += -s * SWING_A * sc;
            vy = std::clamp(vy, -SWING_MAX, SWING_MAX);
            break;
        case Mode::Wave:
            vy = (a ? 1.f : -1.f) * s * w.slope * w.vx;
            break;
    }
    n.vy = vy;

    // ---- movement in small sub-steps so nothing is skipped ------------------
    const float mv = std::max(std::fabs(vy), w.vx);
    const int sub = std::clamp(static_cast<int>(std::ceil(mv / 4.f)), 1, 20);
    const float dx = w.vx / sub;
    const float hz = w.half * 0.8f + 2.f;  // kill box (a bit smaller than the block box)
    for (int i = 0; i < sub; i++) {
        const float prevY = n.y;
        x += dx;
        n.y += n.vy / sub;
        if (!resolve(w, n, x, prevY)) return false;
        for (auto const& b : w.hazards) {
            if (hit(x, n.y, hz, b)) return false;
        }
        int padIdx = -1;
        for (size_t k = 0; k < w.specials.size(); k++) {
            auto const& sp = w.specials[k];
            if (isPad(sp.kind) && hit(x, n.y, w.half, sp.box)) { padIdx = static_cast<int>(k); break; }
        }
        if (padIdx >= 0 && padIdx != n.pad) {
            applyKind(w.specials[padIdx].kind, n, js);
            n.pad = static_cast<int8_t>(padIdx);
        } else if (padIdx < 0) {
            n.pad = -1;
        }
    }
    n.held = a;
    return true;
}

// ------------------------------------------------------------------- search
struct Decision {
    bool hold = false;
    int depth = 0;    // ticks survived by the best path found
    int horizon = 0;  // ticks we wanted to survive
    bool full() const { return depth >= horizon; }
};

inline int horizonTicks(const World& w) {
    int base = 46;
    if (w.mode == Mode::Ship) base = 40;
    if (w.mode == Mode::Wave) base = 32;
    const int byUnits = static_cast<int>(w.horizon / std::max(w.vx, 0.5f));
    return std::clamp(std::min(base, byUnits), 8, base);
}

class Search {
public:
    Search(const World& world, int budget) : w(world), budget(budget) {}

    // Action tried first: the one that keeps things calm.
    bool prefers(const N& n) const {
        return (w.mode == Mode::Ship || w.mode == Mode::Wave) ? n.held : false;
    }

    // How many ticks (up to H) can the player survive from state n at tick t?
    // `exhausted` becomes true if the node budget ran out (result is then unreliable).
    int run(int t, const N& n, int Hp) {
        H = Hp;
        nodes = 0;
        exhausted = false;
        memo.clear();
        return dfs(t, n);
    }

    bool exhausted = false;

private:
    int dfs(int t, const N& n) {
        if (t >= H) return H;
        if (exhausted) return t;
        if (++nodes > budget) { exhausted = true; return t; }
        const uint64_t k = key(t, n);
        auto it = memo.find(k);
        if (it != memo.end()) return it->second;

        int best = t;
        const bool first = prefers(n);
        const float x = w.x + t * w.vx;
        for (int i = 0; i < 2 && best < H; i++) {
            const bool a = (i == 0) ? first : !first;
            N m = n;
            if (!step(w, m, a, x)) continue;
            best = std::max(best, dfs(t + 1, m));
        }
        if (!exhausted) memo[k] = best;
        return best;
    }

    uint64_t key(int t, const N& n) const {
        // flying modes have a big continuous state space: use a coarser grid
        const bool coarse = (w.mode == Mode::Ship || w.mode == Mode::Swing);
        const uint64_t yq = static_cast<uint16_t>(static_cast<int>(std::lround(coarse ? n.y * 0.5f : n.y)) + 4096);
        const uint64_t vq = static_cast<uint16_t>(static_cast<int>(std::lround(coarse ? n.vy : n.vy * 2.f)) + 4096);
        uint64_t k = static_cast<uint64_t>(t) & 0xFF;
        k |= yq << 8;
        k |= vq << 24;
        k |= static_cast<uint64_t>(n.s > 0) << 40;
        k |= static_cast<uint64_t>(n.ground) << 41;
        k |= static_cast<uint64_t>(n.held) << 42;
        k |= static_cast<uint64_t>(n.used) << 43;
        k |= (static_cast<uint64_t>(n.robotT) & 0xF) << 44;
        k |= (static_cast<uint64_t>(n.pad + 1) & 0xFF) << 48;
        return k;
    }

    const World& w;
    int budget;
    int H = 0;
    int nodes = 0;
    std::unordered_map<uint64_t, int> memo;
};

// What should the button do during the next tick?
//
// The preferred action (usually "button up") is kept as long as it can survive;
// we deepen the look-ahead step by step (12, 24, 36 ... ticks) so a search that
// runs out of time never gives an over-optimistic answer.
inline Decision plan(const World& w, float y, float vy, bool flipped, bool ground, bool held,
                     int robotT = 0, int budget = 5000) {
    const int H = horizonTicks(w);
    Search S(w, budget);
    N n;
    n.y = y;
    n.vy = vy;
    n.s = flipped ? -1 : 1;
    n.ground = ground;
    n.held = held;
    n.robotT = static_cast<int8_t>(robotT);

    const bool pref = S.prefers(n);
    Decision out;
    out.hold = pref;
    out.horizon = H;
    out.depth = 0;

    // depth reached when the first action is `a` and the look-ahead is Hp
    auto eval = [&](bool a, int Hp, bool& unsure) {
        N m = n;
        if (!step(w, m, a, w.x)) { unsure = false; return 0; }
        int d = S.run(1, m, Hp);
        unsure = S.exhausted;
        return d;
    };

    int levels[5] = {12, 24, 36, H, H};
    int last = 0;
    for (int li = 0; li < 4; li++) {
        const int Hp = std::min(levels[li], H);
        if (Hp <= last) continue;
        last = Hp;
        bool unsure = false;
        const int dp = eval(pref, Hp, unsure);
        if (unsure) {
            // The search ran out of time for the preferred action, so we do not know.
            // Only switch if the other action is PROVEN to survive.
            bool unsure2 = false;
            const int dq = eval(!pref, Hp, unsure2);
            if (!unsure2 && dq >= Hp) { out.hold = !pref; out.depth = dq; }
            else { out.depth = std::max(out.depth, dp); }
            return out;
        }
        if (dp >= Hp) { out.depth = Hp; continue; }
        // the preferred action dies within Hp ticks: is the other one better?
        bool unsure2 = false;
        const int dq = eval(!pref, Hp, unsure2);
        if (dq > dp) { out.hold = !pref; out.depth = dq; }
        else { out.depth = dp; }
        // Both options die. If it is about to happen (< ~8 ticks) the model may simply be
        // too strict: for jump modes try the jump anyway instead of walking into it.
        const bool jumpMode = (w.mode == Mode::Cube || w.mode == Mode::Robot || w.mode == Mode::Ufo);
        if (jumpMode && std::max(dp, dq) < Hp && std::max(dp, dq) <= 8 && !pref) {
            out.hold = true;
        }
        return out;
    }
    return out;
}

}  // namespace assist
