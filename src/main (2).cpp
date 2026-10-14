// Review Assist - Geode mod (Android, GD 2.2081)
//
// TAP MODE: the bot plays for you while you keep tapping the screen. Every tap
// (or holding a finger) "wakes" the bot for a short time; if you stop touching
// the screen it stops too. The bot sends its own short CLICKS (not a held
// finger) at the moment its trajectory search says they are needed:
//   cube, ship, ufo, wave, ball, robot, spider, swing.
// A button in the pause menu turns the bot on (green) / off (red) and a second
// small button lowers the opacity of both buttons.

#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "planner.hpp"

using namespace geode::prelude;

// ============================================================================
//  Shared state (saved between sessions)
// ============================================================================
namespace {
    bool g_enabled = true;      // bot on / off
    int g_opacityLevel = 0;     // 0 = 100 %, 1 = 40 %, 2 = 12 %
    int g_pauseSerial = 0;      // bumps every time the pause menu opens

    constexpr float kOpacity[3] = {1.0f, 0.4f, 0.12f};

    constexpr float kTapWindow = 1.00f;   // seconds the bot keeps playing after the last touch
    constexpr float kMinHold = 0.034f;    // a bot click lasts at least ~2 ticks
    constexpr float kMinGap = 0.012f;     // and there is a small gap between two clicks
    constexpr float kPlanEvery = 1.f / 60.f;

    void setEnabled(bool value) {
        g_enabled = value;
        Mod::get()->setSavedValue<bool>("bot-enabled", value);
    }

    void setOpacityLevel(int level) {
        g_opacityLevel = std::clamp(level, 0, 2);
        Mod::get()->setSavedValue<int>("opacity-level", g_opacityLevel);
    }

    // Portals that change mode / gravity / size / speed: the look-ahead stops there.
    bool isPortalId(int id) {
        switch (id) {
            case 10: case 11: case 12: case 13: case 45: case 46: case 47: case 99: case 101:
            case 111: case 200: case 201: case 202: case 203: case 286: case 287: case 660:
            case 745: case 747: case 1331: case 1334: case 1933:
                return true;
            default:
                return false;
        }
    }

    // Orbs and pads by object id. Returns false for everything else.
    bool specialKind(int id, assist::Kind& out) {
        using assist::Kind;
        switch (id) {
            case 36: out = Kind::YellowOrb; return true;
            case 141: out = Kind::PinkOrb; return true;
            case 84: out = Kind::BlueOrb; return true;
            case 1022: out = Kind::GreenOrb; return true;
            case 1330: out = Kind::BlackOrb; return true;
            case 1333: out = Kind::RedOrb; return true;
            case 35: out = Kind::YellowPad; return true;
            case 140: out = Kind::PinkPad; return true;
            case 1332: out = Kind::RedPad; return true;
            case 67: out = Kind::BluePad; return true;
            default: return false;
        }
    }
}

$on_mod(Loaded) {
    g_enabled = Mod::get()->getSavedValue<bool>("bot-enabled", true);
    g_opacityLevel = std::clamp(Mod::get()->getSavedValue<int>("opacity-level", 0), 0, 2);
}

// ============================================================================
//  The bot
// ============================================================================
class $modify(AssistLayer, GJBaseGameLayer) {
    struct Fields {
        // touch / click state
        bool fingerDown = false;   // the real finger is on the screen
        float clock = 0.f;         // seconds since the level started (sum of dt)
        float lastTouch = -100.f;  // clock value of the last real touch event (down or up)
        bool botHolding = false;   // what we are currently feeding to the game
        bool injecting = false;    // true while the bot itself sends a press / release
        float holdTime = 0.f;      // how long the bot has been holding
        float gapTime = 1.f;       // how long since the bot released
        // measuring
        bool havePrev = false;
        int seenPauseSerial = 0;
        float lastX = 0.f, lastY = 0.f, lastDt = 0.f;
        float vx = 5.193f;         // units / tick (tick = 1/60 s)
        float vy = 0.f;
        float prevVyRaw = 0.f;
        bool prevOnGround = false;
        bool prevHeld = false;
        float slope = 1.f;         // |vy / vx| of the wave
        // calibration of the physics model, measured while playing
        int calMode = -1;
        float gscale = 1.f, jscale = 1.f;
        float shipUp = assist::phys::SHIP_UP, shipDown = assist::phys::SHIP_DOWN;
        // planning
        float planAcc = 1.f;       // time since the last plan
        bool want = false;         // result of the last plan
        float groundY = 0.f;       // floor level learned while standing on the real ground
        bool haveGround = false;
        float ceilGroundY = 0.f;   // same for the ceiling (upside-down)
        bool haveCeilGround = false;
    };

    // Only inside a real level: never in the editor, platformer mode or dual mode
    // (in those the game is played normally).
    bool assistAllowed() {
        return typeinfo_cast<PlayLayer*>(this) != nullptr && m_player1 != nullptr &&
               !m_player1->m_isPlatformer && !m_gameState.m_isDualMode;
    }

    // ------------------------------------------------------------------ input
    // Real touches arrive here. While the bot is on we swallow them and only
    // remember WHEN the player touched the screen.
    void handleButton(bool down, int button, bool isPlayer1) {
        auto f = m_fields.self();
        // Presses sent by the bot itself always go through to the game.
        if (!f->injecting && g_enabled && button == 1 && this->assistAllowed()) {
            f->fingerDown = down;
            f->lastTouch = f->clock;
            return;
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    // Sends a press / release to the game. The flag makes our own handleButton
    // hook let the call through.
    void assistSend(bool want) {
        auto f = m_fields.self();
        if (f->botHolding == want) return;
        f->botHolding = want;
        f->holdTime = 0.f;
        f->gapTime = 0.f;
        f->injecting = true;
        GJBaseGameLayer::handleButton(want, 1, true);
        f->injecting = false;
    }

    void assistRelease() {
        auto f = m_fields.self();
        f->want = false;
        this->assistSend(false);
    }

    // ---------------------------------------------------------------- sensing
    void assistGather(assist::World& w, float x, float y, float ahead, float vert, bool slopesAsSolid) {
        CCRect area(x - 60.f, y - vert, ahead + 60.f, vert * 2.f);
        float nextPortal = 1e9f;

        auto addSpecial = [&](GameObject* go) {
            assist::Kind k;
            if (specialKind(go->m_objectID, k)) {
                CCRect const& r = go->getObjectRect();
                w.specials.push_back({{r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()}, k});
            } else if (isPortalId(go->m_objectID)) {
                const float px = go->getPositionX();
                if (px > x + 6.f) nextPortal = std::min(nextPortal, px);
            }
        };

        if (auto arr = this->damagingObjectsInRect(area, true)) {
            for (auto go : CCArrayExt<GameObject*>(arr)) {
                if (!go || go->m_isDisabled) continue;
                CCRect const& r = go->getObjectRect();
                w.hazards.push_back({r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()});
            }
        }
        if (auto arr = this->staticObjectsInRect(area, true)) {
            for (auto go : CCArrayExt<GameObject*>(arr)) {
                if (!go || go->m_isDisabled) continue;
                addSpecial(go);
                if (go->m_isPassable) continue;
                auto t = go->m_objectType;
                if (t != GameObjectType::Solid && !(slopesAsSolid && t == GameObjectType::Slope)) {
                    continue;
                }
                CCRect const& r = go->getObjectRect();
                w.solids.push_back({r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()});
            }
        }
        if (nextPortal < 1e8f) w.horizon = std::max(nextPortal - x, 40.f);
    }

    // Floor / ceiling of the playing area.
    // The floor is LEARNED: every time the player stands on something that is not a
    // block, that height is the real floor. Until then the game's own number is used.
    void assistBounds(assist::World& w, PlayerObject* p, CCPoint pos, bool flipped, Fields* f,
                      bool ceilingAlways) {
        const float lo = this->getMinPortalY();
        const float hi = this->getMaxPortalY();
        const float bottom = pos.y - w.half;
        const float top = pos.y + w.half;

        if (p->m_isOnGround) {
            bool onBlock = false;
            for (auto const& b : w.solids) {
                if (pos.x + w.half <= b.x0 || pos.x - w.half >= b.x1) continue;
                if (!flipped && std::fabs(bottom - b.y1) < 3.f) onBlock = true;
                if (flipped && std::fabs(top - b.y0) < 3.f) onBlock = true;
            }
            if (!onBlock) {
                if (!flipped) { f->groundY = bottom; f->haveGround = true; }
                else { f->ceilGroundY = top; f->haveCeilGround = true; }
            }
        }

        if (f->haveGround) {
            w.hasFloor = true;
            w.floorY = f->groundY;
        } else if (lo < bottom + 8.f && lo > bottom - 700.f) {
            w.hasFloor = true;
            w.floorY = lo;
        }

        if (hi - lo > 100.f && hi - lo < 1000.f && pos.y < hi + 5.f && (ceilingAlways || flipped)) {
            w.hasCeil = true;
            w.ceilY = (flipped && f->haveCeilGround) ? f->ceilGroundY : hi;
        }
    }

    // ------------------------------------------------------------ calibration
    // The planner uses an approximate physics model. These measurements nudge its
    // gravity, jump speed and ship acceleration towards what the game really does.
    void assistCalibrate(assist::Mode mode, PlayerObject* p, float vyRaw, float ticks, bool flipped) {
        using namespace assist::phys;
        auto f = m_fields.self();
        if (f->calMode != static_cast<int>(mode)) {
            f->calMode = static_cast<int>(mode);
            f->gscale = 1.f;
            f->jscale = 1.f;
            f->shipUp = SHIP_UP;
            f->shipDown = SHIP_DOWN;
            return;
        }
        if (ticks < 0.2f || ticks > 3.f) return;
        const float s = flipped ? -1.f : 1.f;
        const float acc = (vyRaw - f->prevVyRaw) / ticks;  // units / tick^2

        if (mode == assist::Mode::Ship) {
            if (std::fabs(vyRaw) < 5.5f && std::fabs(f->prevVyRaw) < 5.5f) {
                if (f->prevHeld && s * acc > 0.2f && s * acc < 1.6f) {
                    f->shipUp += (s * acc - f->shipUp) * 0.05f;
                } else if (!f->prevHeld && -s * acc > 0.2f && -s * acc < 1.6f) {
                    f->shipDown += (-s * acc - f->shipDown) * 0.05f;
                }
                f->shipUp = std::clamp(f->shipUp, 0.35f, 1.1f);
                f->shipDown = std::clamp(f->shipDown, 0.35f, 1.1f);
            }
            return;
        }

        const bool gravityMode = mode == assist::Mode::Cube || mode == assist::Mode::Ufo ||
                                 mode == assist::Mode::Ball || mode == assist::Mode::Spider;
        if (!gravityMode) return;

        const float nominal = mode == assist::Mode::Ufo ? UFO_G : mode == assist::Mode::Ball ? BALL_G : G;
        // free fall only: in the air before and now, not at terminal speed, no orb / pad kick
        if (!p->m_isOnGround && !f->prevOnGround && -s * acc > 0.15f && -s * acc < 2.f &&
            s * vyRaw > -(TERM - 1.f) && s * f->prevVyRaw > -(TERM - 1.f) && !f->prevHeld) {
            const float sample = (-s * acc) / nominal;
            f->gscale = std::clamp(f->gscale + (sample - f->gscale) * 0.03f, 0.7f, 1.4f);
        }
        // the moment of a cube jump: on the ground before, going up now
        if (mode == assist::Mode::Cube && f->prevOnGround && !p->m_isOnGround && f->prevHeld &&
            s * vyRaw > 4.f) {
            const float v0 = s * vyRaw + nominal * ticks * f->gscale;
            const float sample = v0 / CUBE_JUMP;
            if (sample > 0.7f && sample < 1.4f) {
                f->jscale = std::clamp(f->jscale + (sample - f->jscale) * 0.3f, 0.7f, 1.4f);
            }
        }
    }

    // ------------------------------------------------------------------ brain
    void assistStep(float dt) {
        auto f = m_fields.self();
        f->clock += dt;
        f->holdTime += dt;
        f->gapTime += dt;

        if (!g_enabled || !this->assistAllowed()) {
            this->assistRelease();
            f->havePrev = false;
            return;
        }

        // The pause menu was opened: forget the old finger state.
        if (f->seenPauseSerial != g_pauseSerial) {
            f->seenPauseSerial = g_pauseSerial;
            f->fingerDown = false;
            f->lastTouch = -100.f;
            this->assistRelease();
            f->havePrev = false;
        }

        PlayerObject* p = m_player1;
        if (p->m_isDead || dt <= 1e-6f) {
            this->assistRelease();
            f->havePrev = false;
            return;
        }

        const bool flipped = p->m_isUpsideDown;
        const float size = (p->m_vehicleSize > 0.f) ? p->m_vehicleSize : 1.f;

        // which mode are we in?
        assist::Mode mode = assist::Mode::Cube;
        bool supported = true;
        if (p->m_isSideways || p->m_isGoingLeft) supported = false;
        if (p->m_isDart) mode = assist::Mode::Wave;
        else if (p->m_isShip) mode = assist::Mode::Ship;
        else if (p->m_isBird) mode = assist::Mode::Ufo;
        else if (p->m_isBall) mode = assist::Mode::Ball;
        else if (p->m_isSpider) mode = assist::Mode::Spider;
        else if (p->m_isRobot) mode = assist::Mode::Robot;
        else if (p->m_isSwing) mode = assist::Mode::Swing;

        // ---- measure our own speed (units per 1/60 s) ----------------------
        CCPoint pos = p->getPosition();
        if (f->havePrev && f->lastDt > 1e-6f) {
            const float prevTicks = f->lastDt * 60.f;
            const float dx = pos.x - f->lastX;
            const float dy = pos.y - f->lastY;
            if (dx > 0.01f && dx < 40.f * prevTicks) {
                const float vxm = dx / prevTicks;
                const float vym = dy / prevTicks;
                f->vx += (vxm - f->vx) * 0.25f;
                f->vx = std::clamp(f->vx, 2.f, 14.f);
                f->vy += (vym - f->vy) * 0.8f;
                if (mode == assist::Mode::Wave) {
                    const float s = std::clamp(std::fabs(vym) / std::max(vxm, 0.1f), 0.5f, 3.f);
                    f->slope += (s - f->slope) * 0.1f;
                }
                this->assistCalibrate(mode, p, vym, prevTicks, flipped);
                f->prevVyRaw = vym;
            }
        } else {
            f->prevVyRaw = 0.f;
        }
        f->lastX = pos.x;
        f->lastY = pos.y;
        f->lastDt = dt;
        f->havePrev = true;
        f->prevOnGround = p->m_isOnGround;
        f->prevHeld = f->botHolding;

        // ---- no recent touch, no bot ----------------------------------------
        const bool active = f->fingerDown || (f->clock - f->lastTouch) < kTapWindow;
        if (!active) {
            this->assistRelease();
            f->planAcc = 1.f;
            return;
        }

        // ---- unsupported situations: behave like the real finger ------------
        if (!supported) {
            this->assistSend(f->fingerDown);
            return;
        }

        // ---- plan (60 times per second) --------------------------------------
        f->planAcc += dt;
        if (f->planAcc >= kPlanEvery - 1e-4f) {
            f->planAcc = 0.f;

            assist::World w;
            w.mode = mode;
            w.x = pos.x;
            w.vx = f->vx;
            w.half = (mode == assist::Mode::Wave ? 5.f : 15.f) * size;
            w.slope = f->slope;
            w.gscale = f->gscale;
            w.jscale = f->jscale;
            w.shipUp = f->shipUp;
            w.shipDown = f->shipDown;
            w.margin = (mode == assist::Mode::Ship || mode == assist::Mode::Wave ||
                        mode == assist::Mode::Swing) ? 3.f : 1.f;

            const bool flying = mode == assist::Mode::Ship || mode == assist::Mode::Wave ||
                                mode == assist::Mode::Swing || mode == assist::Mode::Ufo;
            this->assistGather(w, pos.x, pos.y, 300.f + f->vx * 10.f, 260.f, flying);
            this->assistBounds(w, p, pos, flipped, f, flying || mode == assist::Mode::Ball ||
                                                      mode == assist::Mode::Spider);

            int robotT = 0;
            if (mode == assist::Mode::Robot && f->botHolding && !p->m_isOnGround) {
                robotT = std::max(0, assist::phys::ROBOT_T - static_cast<int>(f->holdTime * 60.f));
            }
            auto d = assist::plan(w, pos.y, f->vy, flipped, p->m_isOnGround, f->botHolding, robotT);
            f->want = d.hold;
        }

        // ---- turn the decision into clicks -----------------------------------
        const bool levelMode = mode == assist::Mode::Ship || mode == assist::Mode::Wave;
        if (f->want && !f->botHolding) {
            if (levelMode || f->gapTime >= kMinGap) this->assistSend(true);
        } else if (!f->want && f->botHolding) {
            if (levelMode || f->holdTime >= kMinHold) this->assistSend(false);
        }
    }

    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        if (typeinfo_cast<PlayLayer*>(this)) {
            this->assistStep(dt);
        }
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }
};

// ============================================================================
//  Pause menu: ON/OFF button (green / red) + opacity button
// ============================================================================
namespace {
    // Rounded tintable box with a text label; returns the box (used as button sprite).
    CCScale9Sprite* makeBox(float w, float h, char const* text, float textScale,
                            CCLabelBMFont*& label) {
        auto bg = CCScale9Sprite::create("square02_001.png", {0.f, 0.f, 80.f, 80.f});
        bg->setContentSize({w, h});

        label = CCLabelBMFont::create(text, "bigFont.fnt");
        label->setScale(textScale);
        label->setPosition({w / 2.f, h / 2.f});
        bg->addChild(label, 1);
        return bg;
    }
}

class $modify(AssistPause, PauseLayer) {
    struct Fields {
        CCScale9Sprite* bg = nullptr;
        CCLabelBMFont* label = nullptr;
        CCScale9Sprite* eyeBg = nullptr;
        CCLabelBMFont* eyeLabel = nullptr;
    };

    void customSetup() {
        PauseLayer::customSetup();
        g_pauseSerial++;

        auto f = m_fields.self();
        auto win = CCDirector::get()->getWinSize();

        auto menu = CCMenu::create();
        menu->setPosition({0.f, 0.f});

        // Use the same touch priority as the pause menu's own buttons.
        if (auto kids = this->getChildren()) {
            for (auto node : CCArrayExt<CCNode*>(kids)) {
                if (auto other = typeinfo_cast<CCMenu*>(node)) {
                    menu->setTouchPriority(other->getTouchPriority());
                    break;
                }
            }
        }

        const float y = win.height - 30.f;

        f->bg = makeBox(112.f, 34.f, "BOT ON", 0.5f, f->label);
        auto mainNode = f->bg;
        auto mainBtn =
            CCMenuItemSpriteExtra::create(mainNode, this, menu_selector(AssistPause::onAssistToggle));
        mainBtn->setPosition({78.f, y});
        menu->addChild(mainBtn);

        f->eyeBg = makeBox(34.f, 34.f, "OP", 0.4f, f->eyeLabel);
        auto eyeNode = f->eyeBg;
        auto eyeBtn =
            CCMenuItemSpriteExtra::create(eyeNode, this, menu_selector(AssistPause::onAssistFade));
        eyeBtn->setPosition({78.f + 56.f + 26.f, y});
        menu->addChild(eyeBtn);

        this->addChild(menu, 100);
        this->refreshAssist();
    }

    void refreshAssist() {
        auto f = m_fields.self();
        const auto a = static_cast<GLubyte>(255.f * kOpacity[g_opacityLevel]);
        const auto aEye = static_cast<GLubyte>(std::max<int>(a, 90));

        if (f->bg) {
            f->bg->setColor(g_enabled ? ccc3(40, 200, 60) : ccc3(220, 50, 50));
            f->bg->setOpacity(a);
        }
        if (f->label) {
            f->label->setString(g_enabled ? "BOT ON" : "BOT OFF");
            f->label->setOpacity(a);
        }
        if (f->eyeBg) {
            f->eyeBg->setColor(ccc3(60, 60, 60));
            f->eyeBg->setOpacity(aEye);
        }
        if (f->eyeLabel) f->eyeLabel->setOpacity(aEye);
    }

    void onAssistToggle(CCObject*) {
        setEnabled(!g_enabled);
        this->refreshAssist();
    }

    void onAssistFade(CCObject*) {
        setOpacityLevel((g_opacityLevel + 1) % 3);
        this->refreshAssist();
    }
};
