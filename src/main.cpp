// Review Assist - Geode mod (Android, GD 2.2081)
//
// While the player keeps a finger on the screen, the bot plays for them
// (cube / ship / wave). Lift the finger and the bot stops.
// A button in the pause menu turns the bot on (green) / off (red) and a second
// small button lowers the opacity of both buttons so they are almost invisible.

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

    void setEnabled(bool value) {
        g_enabled = value;
        Mod::get()->setSavedValue<bool>("bot-enabled", value);
    }

    void setOpacityLevel(int level) {
        g_opacityLevel = std::clamp(level, 0, 2);
        Mod::get()->setSavedValue<int>("opacity-level", g_opacityLevel);
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
        bool fingerDown = false;   // the real finger is on the screen
        bool botHolding = false;   // what we are currently feeding to the game
        bool injecting = false;    // true while the bot itself sends a press / release
        bool havePrev = false;
        int seenPauseSerial = 0;
        float lastX = 0.f, lastY = 0.f, lastDt = 0.f;
        float vx = 5.193f;         // measured speed (units / tick, tick = 1/60 s)
        float vy = 0.f;
        float slope = 1.f;         // measured |vy / vx| of the wave
        float anchorY = 0.f;       // slow reference altitude for the wave
        bool dirUp = false;        // last direction chosen (ship / wave)
    };

    // Only inside a real level: never in the editor, platformer mode or dual mode
    // (in those the game is played normally).
    bool assistAllowed() {
        return typeinfo_cast<PlayLayer*>(this) != nullptr && m_player1 != nullptr &&
               !m_player1->m_isPlatformer && !m_gameState.m_isDualMode;
    }

    // ------------------------------------------------------------------ input
    // Real touches arrive here. While the bot is on we swallow the jump button
    // and only remember whether the finger is down.
    void handleButton(bool down, int button, bool isPlayer1) {
        auto f = m_fields.self();
        // Presses sent by the bot itself always go through to the game.
        if (!f->injecting && g_enabled && button == 1 && this->assistAllowed()) {
            f->fingerDown = down;
            return;
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    // Presses / releases the jump button for the game. The flag makes our own
    // handleButton hook let the call through, whichever way the call is routed.
    void assistHold(bool want) {
        auto f = m_fields.self();
        if (f->botHolding == want) return;
        f->botHolding = want;
        f->injecting = true;
        GJBaseGameLayer::handleButton(want, 1, true);
        f->injecting = false;
    }

    // ---------------------------------------------------------------- sensing
    void assistGather(std::vector<assist::Box>& haz, std::vector<assist::Box>& sol, float x,
                      float y, float ahead, float vert, bool slopesAsSolid) {
        CCRect area(x - 60.f, y - vert, ahead + 60.f, vert * 2.f);

        if (auto arr = this->damagingObjectsInRect(area, true)) {
            for (auto go : CCArrayExt<GameObject*>(arr)) {
                if (!go || go->m_isDisabled) continue;
                CCRect const& r = go->getObjectRect();
                haz.push_back({r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()});
            }
        }
        if (auto arr = this->staticObjectsInRect(area, true)) {
            for (auto go : CCArrayExt<GameObject*>(arr)) {
                if (!go || go->m_isDisabled || go->m_isPassable) continue;
                auto t = go->m_objectType;
                if (t != GameObjectType::Solid && !(slopesAsSolid && t == GameObjectType::Slope)) {
                    continue;
                }
                CCRect const& r = go->getObjectRect();
                sol.push_back({r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()});
            }
        }
    }

    // ------------------------------------------------------------------ brain
    void assistStep(float dt) {
        auto f = m_fields.self();

        if (!g_enabled || !this->assistAllowed()) {
            this->assistHold(false);
            f->havePrev = false;
            return;
        }

        // The pause menu was opened: forget the old finger state.
        if (f->seenPauseSerial != g_pauseSerial) {
            f->seenPauseSerial = g_pauseSerial;
            f->fingerDown = false;
            this->assistHold(false);
            f->havePrev = false;
        }

        PlayerObject* p = m_player1;
        if (p->m_isDead || dt <= 1e-6f) {
            this->assistHold(false);
            f->havePrev = false;
            return;
        }

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
                f->vy += (vym - f->vy) * 0.5f;
                if (p->m_isDart) {
                    const float s = std::clamp(std::fabs(vym) / std::max(vxm, 0.1f), 0.5f, 3.f);
                    f->slope += (s - f->slope) * 0.1f;
                }
            }
        } else {
            f->anchorY = pos.y;
        }
        f->lastX = pos.x;
        f->lastY = pos.y;
        f->lastDt = dt;
        f->havePrev = true;

        // ---- no finger, no bot --------------------------------------------
        if (!f->fingerDown) {
            this->assistHold(false);
            return;
        }

        const bool flipped = p->m_isUpsideDown;
        const float size = (p->m_vehicleSize > 0.f) ? p->m_vehicleSize : 1.f;
        const float stepTicks = dt * 60.f;
        bool want = true;  // unsupported modes: behave like a normal finger

        if (p->m_isSideways || p->m_isGoingLeft) {
            // Rotated / reversed gameplay: not supported, act like a normal finger.
            want = true;
        } else if (p->m_isDart) {
            // ------------------------------------------------------- WAVE
            std::vector<assist::Box> haz, sol;
            this->assistGather(haz, sol, pos.x, pos.y, 300.f, 260.f, true);
            haz.insert(haz.end(), sol.begin(), sol.end());
            this->assistBounds(haz, pos);

            assist::WaveParams wp;
            wp.half = 5.f * size;
            wp.slope = f->slope;
            f->anchorY += (pos.y - f->anchorY) * std::min(1.f, 0.02f * stepTicks);
            auto d = assist::planWave(pos.x, pos.y, f->anchorY, f->vx, haz, wp, f->dirUp);
            f->dirUp = d.up;
            want = flipped ? !d.up : d.up;
        } else if (p->m_isShip) {
            // ------------------------------------------------------- SHIP
            std::vector<assist::Box> haz, sol;
            this->assistGather(haz, sol, pos.x, pos.y, 300.f, 260.f, true);
            haz.insert(haz.end(), sol.begin(), sol.end());
            this->assistBounds(haz, pos);

            assist::ShipParams sp;
            sp.halfW = 15.f * size;
            sp.halfH = 15.f * size;
            auto d = assist::planShip(pos.x, pos.y, f->vy, f->vx, haz, sp, f->dirUp);
            f->dirUp = d.up;
            want = flipped ? !d.up : d.up;
        } else if (p->m_isBird || p->m_isBall || p->m_isRobot || p->m_isSpider || p->m_isSwing) {
            // ---------------------------------------------- not supported
            want = true;
        } else {
            // ------------------------------------------------------- CUBE
            std::vector<assist::Box> haz, sol;
            assist::CubeParams cp;
            this->assistGather(haz, sol, pos.x, pos.y, cp.lookahead, 200.f, false);

            const float h = 15.f * size;
            assist::Box me{pos.x - h, pos.y - h, pos.x + h, pos.y + h};
            auto d = assist::planCube(me, f->vx, f->vx * stepTicks, haz, sol, flipped, cp);
            want = d.jump && p->m_isOnGround;
        }

        this->assistHold(want);
    }

    // Floor / ceiling of the flying area (ship & wave).
    void assistBounds(std::vector<assist::Box>& obs, CCPoint pos) {
        const float lo = this->getMinPortalY();
        const float hi = this->getMaxPortalY();
        if (!(hi - lo > 100.f && hi - lo < 1000.f)) return;
        if (pos.y < lo - 5.f || pos.y > hi + 5.f) return;
        obs.push_back({pos.x - 1000.f, lo - 1000.f, pos.x + 1000.f, lo});
        obs.push_back({pos.x - 1000.f, hi, pos.x + 1000.f, hi + 1000.f});
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
