#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/CCKeyboardDispatcher.hpp>

using namespace geode::prelude;

// Estado global del bot
bool g_botEnabled = true;
bool g_isTouchingScreen = false;

class $modify(MyAutoBotLayer, PlayLayer) {
    void pushButton(PlayerButton btn, bool isPlayer2) {
        g_isTouchingScreen = true;
        PlayLayer::pushButton(btn, isPlayer2);
    }

    void releaseButton(PlayerButton btn, bool isPlayer2) {
        g_isTouchingScreen = false;
        PlayLayer::releaseButton(btn, isPlayer2);
    }

    void update(float dt) {
        PlayLayer::update(dt);

        // Si está apagado con el volumen o no estás tocando la pantalla, no hace nada
        if (!g_botEnabled || !g_isTouchingScreen) return;

        if (m_player1) {
            // Lógica de asistencia al presionar la pantalla
        }
    }
};

class $modify(CCKeyboardDispatcher) {
    bool dispatchKeyboardMSG(enumKeyCodes key, bool isKeyDown, bool isKeyRepeat) {
        if (isKeyDown && (key == KEY_VolumeUp || key == KEY_VolumeDown)) {
            g_botEnabled = !g_botEnabled;
            
            std::string state = g_botEnabled ? "Bot: ACTIVADO" : "Bot: DESACTIVADO";
            Notification::create(state, NotificationIcon::Info, 1.0f)->show();
            
            return true;
        }
        return CCKeyboardDispatcher::dispatchKeyboardMSG(key, isKeyDown, isKeyRepeat);
    }
};
