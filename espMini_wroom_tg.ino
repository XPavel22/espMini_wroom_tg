/*
  # Форматируем 
  prettier --print-width 120 --html-whitespace-sensitivity css --write compressed.html
  Далее сжатие gzip -k -9 index.html
  Перевод .gz в .h - % xxd -i index.html.gz > index_html_gz.h

  #ifndef INDEX_HTML_GZ_H
  #define INDEX_HTML_GZ_H

  #include <stdint.h>

  static const uint8_t index_html_gz[] PROGMEM = {
  ... 0x00
  };

  static const unsigned int index_html_gz_len = sizeof(index_html_gz);

  #endif
*/

#include "WiFiManager.h"
#include "WebServer.h"
#include "ConfigSettings.h"
#include "TimeModule.h"
#include "DeviceManager.h"
#include "Info.h"
#include "Ota.h"
#include "AppState.h"
#include "Control.h"
#include "Logger.h"   
#include "TelegramBot.h" 

// --------------------------------


#if defined(ESP8266)
// Для ESP-01 и других ESP8266 плат
#if defined(ARDUINO_ESP8266_ESP01) || defined(ESP01)
#define LED_PIN 2  // GPIO2 для ESP-01 (синий светодиод)
#else
#define LED_PIN 2  // По умолчанию для ESP8266
#endif
#elif defined(ESP32)
// Для ESP32-S2 и других ESP32 
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(ARDUINO_ESP32S2_DEV)
#define LED_PIN 15 // GPIO15 для ESP32-S2
#else
#define LED_PIN 2  // По умолчанию для других ESP32
#endif
#endif

AppState appState;
Settings settings(appState);
Info sysInfo;
TimeModule timeModule(appState);
Ota ota(settings, appState);
DeviceManager deviceManager(appState);
Logger logger; 
Control control(deviceManager, logger); 
WiFiManager wifiManager(settings, timeModule, appState);
WebServer webServer(settings, deviceManager, appState, timeModule, sysInfo, ota);
TelegramBot telegramBot(settings, webServer, logger, appState, ota, sysInfo, deviceManager);
// -------------------------

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);

    settings.begin();

    Serial.printf("Free heap before LoadSettings: %d\n", ESP.getFreeHeap());

    if (!settings.loadSettings()) {
        Serial.println("Using default settings");
    }

    Serial.printf("Free heap after LoadSettings: %d\n", ESP.getFreeHeap());

    deviceManager.deviceInit();

    Serial.printf("Free heap after DeviceInit: %d\n", ESP.getFreeHeap());

    if (settings.ws.isWifiTurnedOn) {
        wifiManager.begin();

        if (!settings.ws.isAP) {
            Serial.println("Waiting for WiFi connection...");
            unsigned long startTime = millis();
            while (!wifiManager.isConnected() && millis() < (startTime + 5000)) {
                wifiManager.loop();
                delay(10);
            }
        } else {
            Serial.println("AP mode detected, skipping connection wait.");
        }
    } else {
#ifdef ESP32
        if (WiFi.getMode() != WIFI_MODE_NULL) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_MODE_NULL);
            delay(100);
        }
#elif defined(ESP8266)
        if (WiFi.getMode() != WIFI_OFF) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            delay(100);
        }
#endif
    }

    timeModule.beginNTP();
    if (!WiFi.isConnected()) {
        timeModule.updateTime();
    }

    if (settings.ws.isWifiTurnedOn) {
        webServer.begin();
        Serial.printf("Free heap after web server: %d\n", ESP.getFreeHeap());
        
        // Запускаем бота после инициализации веб-сервера и WiFi
        //telegramBot.begin();
        //logger.addLog("Telegram Bot started", LOG_INFO);
        // --------------------------------------
    }

    control.setup();

    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    // Проверяем флаги
    if (appState.isSaveControlRequest && !ota.isUpdate && !appState.isStartWifi) {
        static unsigned long saveTimer = 0;

        if (saveTimer == 0) {
            saveTimer = millis();
            Serial.println("Задержка сохранения 1 сек...");
        }

        if (millis() - saveTimer >= 1000) {
            deviceManager.writeDevicesToFile(deviceManager.myDevices, "/devices.json");
            deviceManager.isSaveControl = false;
            appState.isSaveControlRequest = false;
            saveTimer = 0;
            Serial.println("Сохранение выполнено");
            logger.addLog("Device settings saved", LOG_INFO); 
        }
    }

    if (appState.isSaveWifiRequest && !appState.isStartWifi) {
        delay(10);
        settings.saveSettings();
        appState.isSaveWifiRequest = false;
        logger.addLog("WiFi settings saved", LOG_INFO); 
    }

    ota.loop();

    if (!ota.isUpdate && !deviceManager.isSaveControl && !appState.isStartWifi) {
        control.loop();
    }

    if (settings.ws.isWifiTurnedOn) {
        webServer.loop();
        wifiManager.loop();
        
        if (!ota.isUpdate && !deviceManager.isSaveControl && !appState.isStartWifi && timeModule.isInternetAvailable) {
        telegramBot.loop();
        }
        // -----------------------------------
    }

    if (appState.isFormat) {
        static unsigned long formatTimer = 0;

        if (formatTimer == 0) {
            formatTimer = millis();
            Serial.println("Задержка перед форматированием 3 сек...");
            logger.addLog("System format requested. Rebooting in 3 seconds...", LOG_ERROR); // Логируем критическое событие
        }

        if (millis() - formatTimer >= 1000) {
            webServer.stop();
            logger.addLog("Formatting SPIFFS...", LOG_INFO);
            settings.format(true);
            appState.isFormat = false;
        }
    }

    if (appState.isReboot) {
        logger.addLog("System reboot requested", LOG_INFO);
        settings.reboot();
        appState.isReboot = false;
    }

    delay(10);
}
