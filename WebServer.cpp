#include "WebServer.h"

WebServer::WebServer(Settings& settings,
                     DeviceManager& deviceManager,
                     AppState& appState,
                     TimeModule& timeModule,
                     Info& info,
                     Ota& ota)
    : server(80),
      settings(settings),
      deviceManager(deviceManager),
      appState(appState),
      timeModule(timeModule),
      info(info),
      ota(ota) {
}

void WebServer::stop() {
  server.end();

}

void WebServer::startServer() {
    server.begin();
    Serial.println("WebServer started.");
}

void WebServer::stopServer() {
    server.end();
    Serial.println("WebServer stopped.");
}

void WebServer::begin() {

 Serial.println("WebServer started Start");
    if (!settings.ws.isWifiTurnedOn) {
        Serial.println("WebServer: WiFi is turned off. Server not started.");
        return;
    }

    server.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        _webServerIsBusy = true;
        request->send(getIndexResponse(request));
        _webServerIsBusy = false;
    });

    server.onNotFound([this](AsyncWebServerRequest* request) {
        request->send(getIndexResponse(request));
    });

    server.on("/settings", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetSettings(request);
    });

    server.on("/device", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetDeviceSettings(request);
    });

    server.on("/live", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetLiveData(request);
    });

    server.on("/logs", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetLogs(request);
    });

    server.on("/saveSettings", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSaveSettings(request);
    });

    server.on("/saveDevice", HTTP_POST,
    [this](AsyncWebServerRequest* request) {

    },
    NULL,
    [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        handleSaveDeviceSettings(request, data, len, index, total);
    }
  );

server.on("/updateDevice", HTTP_POST, [this](AsyncWebServerRequest* request) {

        handleUpdateDeviceProperty(request);
    });

    server.on("/set-datetime", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSaveDateTime(request);
    });

    server.on("/relay", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleControlRelay(request);
    });

    server.on("/clearLog", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleClearLogs(request);
    });

    server.on("/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleReboot(request);
    });

    server.on("/reset", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleFullReset(request);
    });

        server.on("/resetDevice", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleResetDevice(request);
    });

        server.on("/sysinfo", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSysinfo(request);
    });

 server.on("/uploadFile", HTTP_POST,
  [](AsyncWebServerRequest * request) {
    request->send(200);
  },
  [this](AsyncWebServerRequest * request, String filename, size_t index,
         uint8_t *data, size_t len, bool final) {

         if (index == 0) {
        _webServerIsBusy = true;
        Serial.println("[WebServer] File upload started. Server is now busy.");
      }
    this->ota.handleFileUpload(request, filename, index, data, len, final);

     if (final) {
        _webServerIsBusy = false;
        Serial.println("[WebServer] File upload finished. Server is now free.");
      }
  }
           );

  server.on("/download", HTTP_POST, [](AsyncWebServerRequest * request) {
    if (request->hasParam("file", true)) {
      String filename = request->getParam("file", true)->value();
      filename = "/" + filename;

#ifdef ESP32
      if (!SPIFFS.begin(true)) {
#elif defined(ESP8266)
      if (!SPIFFS.begin()) {
#endif
        Serial.println("Failed to mount SPIFFS");
      }

      if (!SPIFFS.exists(filename)) {
        request->send(404, "text/plain", "Файл не найден");
        return;
      }

      request->send(SPIFFS, filename, "application/octet-stream");
    } else {
      request->send(400, "text/plain", "Некорректный запрос");
    }
  });

    server.begin();
    yield();
    Serial.println("WebServer started end");

}

bool WebServer::isBusy() const {
    return _webServerIsBusy;
}

void WebServer::printRequestParameters(AsyncWebServerRequest* request) {
    int params = request->params();
    for (int i = 0; i < params; i++) {
      const AsyncWebParameter* p = request->getParam(i);
      if (p->isPost()) {
        Serial.print("POST[");
      } else {
        Serial.print("GET[");
      }
      Serial.print(p->name());
      Serial.print("]: ");
      Serial.println(p->value());
    }
  }

void WebServer::sendError(AsyncWebServerRequest* request, int code, const String& message) {
    StaticJsonDocument<200> doc;
    doc["status"] = "error";
    doc["code"] = code;
    doc["message"] = message;
    sendJson(request, doc);
}

void WebServer::sendSuccess(AsyncWebServerRequest* request, const String& message) {
    StaticJsonDocument<200> doc;
    doc["status"] = "success";
    doc["message"] = message;
    sendJson(request, doc);
}

void WebServer::sendJson(AsyncWebServerRequest* request, JsonDocument& doc) {
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

AsyncWebServerResponse* WebServer::getIndexResponse(AsyncWebServerRequest *request) {
    bool hasHtml = SPIFFS.exists("/index.html");
    bool hasGz = SPIFFS.exists("/index.html.gz");
    time_t htmlTime = 0, gzTime = 0;
    time_t now = time(nullptr);

    if (hasHtml) {
        File htmlFile = SPIFFS.open("/index.html", "r");
        if (htmlFile) {
            htmlTime = htmlFile.getLastWrite();
            htmlFile.close();
        }
    }
    if (hasGz) {
        File gzFile = SPIFFS.open("/index.html.gz", "r");
        if (gzFile) {
            gzTime = gzFile.getLastWrite();
            gzFile.close();
        }
    }

    AsyncWebServerResponse *response = nullptr;
    String selectedFile = "";
    String reason = "";

    if (hasHtml && (!hasGz || htmlTime > gzTime)) {
        response = request->beginResponse(SPIFFS, "/index.html", "text/html");
        selectedFile = "/index.html";
        reason = (!hasGz) ? "GZ file not exists" : "HTML is newer";
    } else if (hasGz) {
        response = request->beginResponse(SPIFFS, "/index.html.gz", "text/html");
        response->addHeader("Content-Encoding", "gzip");
        selectedFile = "/index.html.gz";
        reason = (!hasHtml) ? "HTML file not exists" : "GZ is newer or equal";
    } else {

        Serial.println("[WebServer] No files in SPIFFS, using embedded version.");
        response = request->beginResponse_P(200, "text/html", index_html_gz, index_html_gz_len);
        response->addHeader("Content-Encoding", "gzip");
        selectedFile = "EMBEDDED";
        reason = "No files in SPIFFS";
    }

    Serial.printf("[WebServer] Serving: %s, Reason: %s\n", selectedFile.c_str(), reason.c_str());

    if (response) {
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
    }

    return response;
}

void WebServer::handleGetSettings(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("GET /settings");

    String json = settings.serializeSettings(settings.ws);
    request->send(200, "application/json", json);

    _webServerIsBusy = false;
}

void WebServer::handleGetDeviceSettings(AsyncWebServerRequest* request) {
    Serial.print("GET /device: ");

    processRequestSetting = true;

    const Device& currentDevice = deviceManager.myDevices[deviceManager.currentDeviceIndex];

    String status = deviceManager.serializeDevice(currentDevice, nullptr, request);
    if (status != "sending") {
        Serial.printf("❌ Не удалось начать потоковую отправку. Статус: %s\n", status.c_str());
        request->send(500, "text/plain", "Failed to start streaming");
    } else {
        Serial.println("✅ Потоковая отправка инициирована.");
    }

     processRequestSetting = false;
}

void WebServer::handleGetLiveData(AsyncWebServerRequest* request) {

    if (appState.isSaveWifiRequest || deviceManager.isSaveControl || processRequestSetting) {
        request->send(503, "text/plain", "Server Busy");
        return;
    }
    _webServerIsBusy = true;

    static uint32_t lastSentRelayChecksum = 0;
    static uint32_t lastSentSensorChecksum = 0;
    static uint32_t lastSentTimerChecksum = 0;
    static uint32_t lastSentSettingsChecksum = 0;

    DynamicJsonDocument doc(4096);

    uint32_t currentRelayChecksum = deviceManager.calculateOutputRelayChecksum();
    if (currentRelayChecksum != lastSentRelayChecksum) {
        Serial.println("[WebServer] Relays data changed, sending update.");

        JsonObject relaysUpdate = doc.createNestedObject("relays_update");
        deviceManager.serializeRelaysForControlTab(relaysUpdate);
        lastSentRelayChecksum = currentRelayChecksum;
    }

    uint32_t currentSensorChecksum = deviceManager.calculateSensorValuesChecksum();
    if (currentSensorChecksum != lastSentSensorChecksum) {
        Serial.println("[WebServer] Sensor data changed, sending update.");
        JsonObject sensorsUpdate = doc.createNestedObject("sensors_update");
        deviceManager.serializeSensorValues(sensorsUpdate);
        lastSentSensorChecksum = currentSensorChecksum;
    }

    uint32_t currentTimerChecksum = deviceManager.calculateTimersProgressChecksum();
    if (currentTimerChecksum != lastSentTimerChecksum) {
        Serial.println("[WebServer] Timers data changed, sending update.");
        JsonObject timersUpdate = doc.createNestedObject("timers_update");
        deviceManager.serializeTimersProgress(timersUpdate);
        lastSentTimerChecksum = currentTimerChecksum;
    }

    uint32_t currentSettingsChecksum = deviceManager.calculateDeviceFlagsChecksum();
    if (currentSettingsChecksum != lastSentSettingsChecksum) {
        Serial.println("[WebServer] Settings flags changed, sending update.");
        JsonObject settingsUpdate = doc.createNestedObject("settings_update");
        deviceManager.serializeDeviceFlags(settingsUpdate);
        lastSentSettingsChecksum = currentSettingsChecksum;
    }

    JsonObject staticInfo = doc.createNestedObject("static_info");
    staticInfo["freeHeap"] = ESP.getFreeHeap();
    staticInfo["systemLoad"] = settings.ws.systemLoading;
    staticInfo["uptime"] = millis() / 1000;
    staticInfo["dateTime"] = timeModule.getFormattedDateTime();
    staticInfo["wifiStatus"] = WiFi.status();
    staticInfo["localIP"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();

    sendJson(request, doc);
    _webServerIsBusy = false;
}

void WebServer::handleGetLogs(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("GET /logs");

    StaticJsonDocument<200> doc;
    doc["message"] = "Logs endpoint not implemented yet";
    sendJson(request, doc);

    _webServerIsBusy = false;
}

void WebServer::handleSysinfo(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /sysinfo");

     StaticJsonDocument<800> doc;

    char statusBuffer[800];

    info.getSystemStatus(statusBuffer, sizeof(statusBuffer));

    doc["message"] = statusBuffer;

    sendJson(request, doc);

    _webServerIsBusy = false;
}

void WebServer::handleSaveSettings(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /saveSettings");

    if (request->hasParam("body", true)) {
        String body = request->getParam("body", true)->value();

        DynamicJsonDocument doc(SIZE_JSON_S);
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            sendError(request, 400, "Invalid JSON: " + String(error.c_str()));
            _webServerIsBusy = false;
            return;
        }

        if (settings.deserializeSettings(doc.as<JsonObject>(), settings.ws)) {

              appState.isSaveWifiRequest = true;
              sendSuccess(request, "Settings saved");

        } else {
            sendError(request, 500, "Failed to apply settings");
        }
    } else {
        sendError(request, 400, "Missing body parameter");
    }

    _webServerIsBusy = false;
}

void WebServer::handleSaveDeviceSettings(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    processRequestSetting = true;

    static char* jsonBuffer = nullptr;
    static size_t expectedTotalSize = 0;

    if (index == 0) {

        if (jsonBuffer) {
            free(jsonBuffer);
            jsonBuffer = nullptr;
        }

        expectedTotalSize = total;

        jsonBuffer = (char*)malloc(expectedTotalSize + 1);

        if (!jsonBuffer) {
            Serial.println("❌ Критическая ошибка: Не удалось выделить память для JSON буфера!");
            request->send(500, "application/json", R"({"error":"Memory allocation failed on server"})");
            processRequestSetting = false;
            return;
        }
        Serial.printf("Выделено %u байт для JSON буфера.\n", expectedTotalSize + 1);
    }

    if (jsonBuffer) {
        memcpy(jsonBuffer + index, data, len);
    }

    if (index + len != expectedTotalSize) {
        return;
    }

    jsonBuffer[expectedTotalSize] = '\0';
    Serial.printf("JSON полностью получен. Размер: %u байт\n", expectedTotalSize);

    {
        DynamicJsonDocument doc(SIZE_JSON_S);
        DeserializationError error = deserializeJson(doc, jsonBuffer);

        if (error) {
            Serial.printf("❌ Ошибка парсинга JSON: %s\n", error.c_str());
            request->send(400, "application/json", R"({"error":"Invalid JSON content"})");
        } else {

            Device& currentDevice = deviceManager.myDevices[deviceManager.currentDeviceIndex];
            if (deviceManager.deserializeDevice(doc.as<JsonObject>(), currentDevice)) {
                appState.isSaveControlRequest = true;
                Serial.println("✅ Настройки устройства успешно применены");
                request->send(200, "application/json", R"({"status":"ok", "message":"Настройки сохранены"})");
            } else {
                Serial.println("❌ Ошибка применения настроек устройства");
                request->send(500, "application/json", R"({"error":"Failed to apply settings"})");
            }
        }
    }

    if (jsonBuffer) {
        free(jsonBuffer);
        jsonBuffer = nullptr;
    }
    Serial.println("JSON буфер освобожден.");

    processRequestSetting = false;
}

void WebServer::handleUpdateDeviceProperty(AsyncWebServerRequest* request) {
    Serial.println("Получен запрос на /api/device/update (после получения тела)");

    if (!request->hasParam("body", true)) {
        Serial.println("❌ Ошибка: Отсутствует параметр 'body' в запросе");
        request->send(400, "application/json", R"({"error":"Missing 'body' parameter in form data"})");
        return;
    }

    String jsonPayload = request->getParam("body", true)->value();
    Serial.printf("Получен JSON payload: %s\n", jsonPayload.c_str());

    if (jsonPayload.length() == 0 || !jsonPayload.startsWith("{") || !jsonPayload.endsWith("}")) {
        Serial.println("❌ Ошибка: Неверный формат JSON в параметре 'body'");
        request->send(400, "application/json", R"({"error":"Invalid JSON format in 'body' parameter"})");
        return;
    }

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, jsonPayload);

    if (error) {
        Serial.printf("❌ Ошибка парсинга JSON: %s\n", error.c_str());
        request->send(400, "application/json", R"({"error":"Invalid JSON content"})");
        return;
    }

    if (deviceManager.currentDeviceIndex >= deviceManager.myDevices.size()) {
        Serial.printf("❌ Ошибка: Неверный индекс устройства: %d\n", deviceManager.currentDeviceIndex);
        request->send(400, "application/json", R"({"error":"Invalid device index"})");
        return;
    }

    Device& currentDevice = deviceManager.myDevices[deviceManager.currentDeviceIndex];
    bool anyFieldUpdated = false;
    String updatedPropertiesList = "";

    for (JsonPair kv : doc.as<JsonObject>()) {
        const char* key = kv.key().c_str();
        JsonVariant value = kv.value();
        bool updated = false;

        String keyStr = String(key);
        int bracketStart = keyStr.indexOf('[');
        int bracketEnd = keyStr.indexOf(']');

        if (bracketStart != -1 && bracketEnd != -1 && bracketEnd > bracketStart) {
            String arrayName = keyStr.substring(0, bracketStart);

            String indexStr = keyStr.substring(bracketStart + 1, bracketEnd);
            bool isNumeric = true;
            for (unsigned int i = 0; i < indexStr.length(); i++) {
                if (!isDigit(indexStr.charAt(i))) {
                    isNumeric = false;
                    break;
                }
            }

            if (!isNumeric) {
                Serial.printf("⚠️ Пропускаем свойство с невалидным индексом (не число): %s\n", key);
                continue;
            }

            int index = indexStr.toInt();

            if (index < 0) {
                Serial.printf("⚠️ Пропускаем свойство с отрицательным индексом: %s\n", key);
                continue;
            }

            if (bracketEnd + 2 >= keyStr.length()) {
                Serial.printf("⚠️ Пропускаем свойство с неверным форматом (отсутствует имя свойства): %s\n", key);
                continue;
            }

            String propertyName = keyStr.substring(bracketEnd + 2);

            if (arrayName == "act") {

                if (index >= 0 && index < currentDevice.actions.size()) {
                    Action& action = currentDevice.actions[index];
                    if (propertyName == "use" && value.is<bool>()) {
                        action.isUseSetting = value.as<bool>();
                        updated = true;
                    } else if (propertyName == "dsc" && value.is<String>()) {
                        strlcpy(action.description, value.as<String>().c_str(), MAX_DESCRIPTION_LENGTH);
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива actions: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.actions.size());
                }
            }
            else if (arrayName == "tmr") {

                if (index >= 0 && index < currentDevice.timers.size()) {
                    Timer& timer = currentDevice.timers[index];
                    if (propertyName == "use" && value.is<bool>()) {
                        timer.isUseSetting = value.as<bool>();
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива timers: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.timers.size());
                }
            }
            else if (arrayName == "sch") {

                if (index >= 0 && index < currentDevice.scheduleScenarios.size()) {
                    ScheduleScenario& schedule = currentDevice.scheduleScenarios[index];
                    if (propertyName == "use" && value.is<bool>()) {
                        schedule.isUseSetting = value.as<bool>();
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива scheduleScenarios: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.scheduleScenarios.size());
                }
            }
            else if (arrayName == "sen") {

                if (index >= 0 && index < currentDevice.sensors.size()) {
                    Sensor& sensor = currentDevice.sensors[index];
                    if (propertyName == "use" && value.is<bool>()) {
                        sensor.isUseSetting = value.as<bool>();
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива sensors: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.sensors.size());
                }
            }
            else if (arrayName == "rel") {

                if (index >= 0 && index < currentDevice.relays.size()) {
                    Relay& relay = currentDevice.relays[index];
                    if (propertyName == "man" && value.is<bool>()) {
                        relay.manualMode = value.as<bool>();
                        updated = true;
                    } else if (propertyName == "stp" && value.is<bool>()) {
                        relay.statePin = value.as<bool>();
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива relays: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.relays.size());
                }
            }
            else if (arrayName == "pid") {

                if (index >= 0 && index < currentDevice.pids.size()) {
                    Pid& pid = currentDevice.pids[index];
                    if (propertyName == "Kp" && value.is<double>()) {
                        pid.Kp = value.as<double>();
                        updated = true;
                    }
                } else {
                    Serial.printf("⚠️ Пропускаем свойство с индексом за пределами массива pids: %s (индекс: %d, размер: %d)\n",
                                 key, index, currentDevice.pids.size());
                }
            }
            else {
                Serial.printf("⚠️ Пропускаем свойство с неизвестным именем массива: %s\n", key);
            }
        }
        else if (strcmp(key, "ite") == 0 && value.is<bool>()) {
            currentDevice.isTimersEnabled = value.as<bool>();
            updated = true;
        } else if (strcmp(key, "iet") == 0 && value.is<bool>()) {
            currentDevice.isEncyclateTimers = value.as<bool>();
            updated = true;
        } else if (strcmp(key, "ise") == 0 && value.is<bool>()) {
            currentDevice.isScheduleEnabled = value.as<bool>();
            updated = true;
        } else if (strcmp(key, "iae") == 0 && value.is<bool>()) {
            currentDevice.isActionEnabled = value.as<bool>();
            updated = true;
        } else if (strcmp(key, "tmp.use") == 0 && value.is<bool>()) {
            currentDevice.temperature.isUseSetting = value.as<bool>();
            updated = true;
        } else {
            Serial.printf("⚠️ Пропускаем неизвестное или невалидное свойство: %s\n", key);
        }

        if (updated) {
            anyFieldUpdated = true;
            updatedPropertiesList += String(key) + " ";
        }
    }

    if (anyFieldUpdated) {
        String response = "{\"status\":\"ok\",\"message\":\"Properties updated: " + updatedPropertiesList + "\"}";
        request->send(200, "application/json", response);
        Serial.printf("✅ Свойства успешно обновлены: %s\n", updatedPropertiesList.c_str());
    } else {
        request->send(400, "application/json", R"({"error":"No valid properties were provided for update"})");
        Serial.println("❌ Ошибка: Ни одно поле не было обновлено.");
    }
}

void WebServer::handleSaveDateTime(AsyncWebServerRequest* request) {

    printRequestParameters(request);

    _webServerIsBusy = true;
    Serial.println("POST /SaveTime");

    if (request->hasParam("body", true)) {
        String body = request->getParam("body", true)->value();

        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            sendError(request, 400, "Invalid JSON: " + String(error.c_str()));
            _webServerIsBusy = false;
            return;
        }

        String dateInput = doc["date"];
        String timeInput = doc["time"];
        int8_t timeZoneValue = doc["timeZone"].as<uint8_t>();

        Serial.println(timeZoneValue);

        if (timeZoneValue < -12 || timeZoneValue > 14) {
            timeZoneValue = 3;
        }

        settings.ws.timeZone = timeZoneValue;

        bool success = false;
        if (!dateInput.isEmpty() && !timeInput.isEmpty()) {
            success = timeModule.setTimeManually(dateInput, timeInput);
        }

        if (success) {
            sendSuccess(request, "Time settings saved");
        } else {
            sendError(request, 400, "Invalid date/time format");
        }
    } else {
        sendError(request, 400, "Missing body parameter");
    }

    _webServerIsBusy = false;
}

void WebServer::handleControlRelay(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /relay");

    if (request->hasParam("body", true)) {
        String body = request->getParam("body", true)->value();

        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            sendError(request, 400, "Invalid JSON: " + String(error.c_str()));
            _webServerIsBusy = false;
            return;
        }

        bool success = deviceManager.handleRelayCommand(doc.as<JsonObject>(), 0);

        if (success) {
            sendSuccess(request, "Relay command executed");
        } else {
            sendError(request, 400, "Failed to execute relay command");
        }
    } else {
        sendError(request, 400, "Missing body parameter");
    }

    _webServerIsBusy = false;
}

void WebServer::handleClearLogs(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /clearLog");

    sendSuccess(request, "Logs cleared (not implemented)");

    _webServerIsBusy = false;
}

void WebServer::handleReboot(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /reboot");

    sendSuccess(request, "Rebooting...");
     appState.isReboot = true;

    _webServerIsBusy = false;
}

void WebServer::handleFullReset(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /reset");

    sendSuccess(request, "Reset initiated");

    _webServerIsBusy = false;
    appState.isFormat = true;
}

void WebServer::handleResetDevice(AsyncWebServerRequest* request) {
    _webServerIsBusy = true;
    Serial.println("POST /handleResetDevice");

   if (SPIFFS.exists("/devices.json")) {
        bool success = SPIFFS.remove("/devices.json");
         if (success) {
            sendSuccess(request, "Reset Device executed");
            appState.isReboot = true;
        } else {
            sendError(request, 400, "Failed to execute ResetDevice");
        }
    }

    _webServerIsBusy = false;
}

void WebServer::loop() {

}
