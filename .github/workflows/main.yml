#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

using namespace geode::prelude;

// Variables de control global
bool g_botEnabled = true; // Activo o inactivo por el botón de volumen
bool g_isTouchingScreen = false; // Solo funciona si mantienes presionada la pantalla

class $modify(MyAutoBotLayer, PlayLayer) {
    // 1. Detectar cuando presiona y suelta la pantalla
    void pushButton(PlayerButton btn, bool isPlayer2) {
        g_isTouchingScreen = true;
        PlayLayer::pushButton(btn, isPlayer2);
    }

    void releaseButton(PlayerButton btn, bool isPlayer2) {
        g_isTouchingScreen = false;
        PlayLayer::releaseButton(btn, isPlayer2);
    }

    // 2. Bucle principal del nivel (Frame-Perfect Logic)
    void update(float dt) {
        PlayLayer::update(dt);

        // Si el bot está apagado con el volumen O si NO estás tocando la pantalla, no hace nada
        if (!g_botEnabled || !g_isTouchingScreen) return;

        auto player = m_player1;
        if (!player) return;

        // Lógica de salto exacto (Hitbox / Colisión aproximada)
        // Por ejemplo: en Wave/Nave o cuando hay un obstáculo/orbe justo enfrente
        if (this->checkCollision(player)) { 
            // Simula el salto o el ajuste del ángulo perfecto en el frame actual
            this->pushButton(PlayerButton::Jump, false);
        }
    }
};

// 3. Captura del botón de Volumen en Android
#include <Geode/modify/CCKeyboardDispatcher.hpp>

class $modify(CCKeyboardDispatcher) {
    bool dispatchKeyboardMSG(enumKeyCodes key, bool isKeyDown, bool isKeyRepeat) {
        // KEY_VolumeUp o KEY_VolumeDown
        if (isKeyDown && (key == KEY_VolumeUp || key == KEY_VolumeDown)) {
            g_botEnabled = !g_botEnabled; // Alterna entre Encendido y Apagado
            
            // Muestra una notificación visual en pantalla
            std::string state = g_botEnabled ? "Bot: ACTIVADO" : "Bot: DESACTIVADO";
            Notification::create(state, NotificationIcon::Info, 1.0f)->show();
            
            return true; // Consume el evento para no cambiar el volumen del sistema
        }
        return CCKeyboardDispatcher::dispatchKeyboardMSG(key, isKeyDown, isKeyRepeat);
    }
};
