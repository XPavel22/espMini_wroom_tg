#include "WiFiManager.h"

WiFiManager::WiFiManager(Settings& ws, TimeModule& tm, AppState& appState)
  : settings(ws),
    timeModule(tm),
    appState(appState) {}

void WiFiManager::begin() {
    if (!settings.ws.isWifiTurnedOn) {
        Serial.println("WiFi is turned off in settings.");
        return;
    }

    appState.isStartWifi = true;

#ifdef ESP32
    WiFi.setHostname(settings.ws.mDNS.c_str());
#else
    WiFi.hostname(settings.ws.mDNS.c_str());
#endif

    if (settings.ws.isAP) {
        Serial.println("WiFiManager: 'AP Only' mode is enabled. Skipping client connection.");
        startAccessPoint();
        return;
    }

    Serial.println("WiFiManager: Starting Wi-Fi connection process...");
    connectToWiFi();
}

void WiFiManager::loop() {

#ifdef ESP8266
    MDNS.update();
#endif

    if (!settings.ws.isWifiTurnedOn || settings.ws.isAP) {
        return;
    }

    if (timeUpdateScheduled && (millis() - ipObtainedTime > timeUpdateDelay)) {
        Serial.println("WiFiManager: Updating time module after IP acquisition delay");
        timeModule.updateTime();
        timeUpdateScheduled = false;
    }

    if (isInFallbackAP) {
        if (millis() - apStartTime > apTimeout) {
            Serial.println("WiFiManager: Fallback AP timeout expired. Retrying Wi-Fi connection.");
            isInFallbackAP = false;
            connectToWiFi();
        }
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (isConnecting) {
            isConnecting = false;
            appState.isStartWifi = false;

            Serial.printf("WiFiManager: Successfully connected to %s\n", WiFi.SSID().c_str());
            Serial.print("WiFiManager: IP Address: ");
            Serial.println(WiFi.localIP());

            timeUpdateScheduled = true;
            ipObtainedTime = millis();
            Serial.println("WiFiManager: Time update scheduled in 3 seconds");

            if (!MDNS.begin(settings.ws.mDNS.c_str())) {
                Serial.println("Error setting up MDNS responder!");
            } else {
                MDNS.addService("http", "tcp", 80);
                Serial.printf("MDNS responder started: http://%s.local\n", settings.ws.mDNS.c_str());
            }
        }
        return;
    }

    if (isConnecting) {
        if (millis() - connectionStartTime > connectionTimeout) {
            Serial.println("WiFiManager: Connection timeout. Starting fallback AP.");
            isConnecting = false;
            isInFallbackAP = true;
            apStartTime = millis();
            startAccessPoint();
        }
    } else {
        Serial.println("WiFiManager: Connection lost. Trying to reconnect...");
        connectToWiFi();
    }
}

void WiFiManager::connectToWiFi() {
    timeUpdateScheduled = false;
    appState.isStartWifi = true;

    if (settings.ws.networkSettings.empty()) {
        Serial.println("WiFiManager: No saved networks. Starting fallback AP.");
        isInFallbackAP = true;
        apStartTime = millis();
        startAccessPoint();
        return;
    }

    NetworkSetting& net = settings.ws.networkSettings[0];

    if (net.ssid.isEmpty()) {
        Serial.println("WiFiManager: SSID is empty. Starting fallback AP.");
        isInFallbackAP = true;
        apStartTime = millis();
        startAccessPoint();
        return;
    }

    Serial.printf("WiFiManager: Attempting to connect to SSID: %s\n", net.ssid.c_str());

    setupStationMode(net);
    WiFi.begin(net.ssid.c_str(), net.password.c_str());

    isConnecting = true;
    isInFallbackAP = false;
    connectionStartTime = millis();
}

void WiFiManager::startAccessPoint() {
    Serial.println("WiFiManager: Entering Access Point mode...");
    appState.isStartWifi = true;

    setupAccessPointMode();

    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("WiFiManager: Access Point '%s' started.\n", settings.ws.ssidAP.c_str());
    Serial.print("WiFiManager: AP IP Address: ");
    Serial.println(apIP);

    if (!MDNS.begin(settings.ws.mDNS.c_str())) {
        Serial.println("Error setting up MDNS responder in AP mode!");
    } else {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("MDNS responder started in AP mode: http://%s.local\n", settings.ws.mDNS.c_str());
    }

    appState.isStartWifi = false;
}

bool WiFiManager::isConnected() const {
    return (WiFi.status() == WL_CONNECTED);
}

String WiFiManager::getStatusString() const {
    switch (WiFi.status()) {
        case WL_CONNECTED: return "Connected";
        case WL_NO_SHIELD: return "No WiFi Shield";
        case WL_IDLE_STATUS: return "Idle";
        case WL_NO_SSID_AVAIL: return "No SSID Available";
        case WL_SCAN_COMPLETED: return "Scan Completed";
        case WL_CONNECT_FAILED: return "Connection Failed";
        case WL_CONNECTION_LOST: return "Connection Lost";
        case WL_DISCONNECTED: return "Disconnected";
        default: return "Unknown Status";
    }
}

void WiFiManager::setupStationMode(const NetworkSetting& net) {
    WiFi.disconnect();

#ifdef ESP32
    WiFi.mode(WIFI_MODE_STA);
#else
    WiFi.mode(WIFI_STA);
#endif

#ifdef ESP8266
    WiFi.setAutoConnect(true);
#endif

    WiFi.setAutoReconnect(true);

#ifdef ESP32
    WiFi.setHostname(settings.ws.mDNS.c_str());
#else
    WiFi.hostname(settings.ws.mDNS.c_str());
#endif

    if (net.useStaticIP) {
        Serial.println("WiFiManager: Using static IP.");
        WiFi.config(net.staticIP, net.staticGateway, net.staticSubnet, net.staticDNS);
    } else {
        Serial.println("WiFiManager: Using DHCP.");
    }
}

void WiFiManager::setupAccessPointMode() {
    WiFi.disconnect();

#ifdef ESP32
    WiFi.mode(WIFI_MODE_AP);
#else
    WiFi.mode(WIFI_AP);
#endif

    WiFi.softAPConfig(
        settings.ws.staticIpAP,
        settings.ws.staticIpAP,
        IPAddress(255, 255, 255, 0)
    );

    if (settings.ws.passwordAP.isEmpty()) {
        WiFi.softAP(settings.ws.ssidAP.c_str());
    } else {
        WiFi.softAP(settings.ws.ssidAP.c_str(), settings.ws.passwordAP.c_str());
        Serial.print("Password: ");
        Serial.println(settings.ws.passwordAP.c_str());
    }
}
