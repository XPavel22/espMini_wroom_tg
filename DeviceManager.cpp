#include "DeviceManager.h"
#include <cstring>

DeviceManager::DeviceManager(AppState& appState)
    : appState(appState)
{

}

void DeviceManager::initializeDevice(const char* name, bool activ, bool isNewDevice) {

   Serial.printf("Start initializeDevice");

  if (!isNewDevice) {
    if (!myDevices.empty()) {
      myDevices.clear();
      myDevices.shrink_to_fit();
    }
  }

  myDevices.emplace_back();
  Device& newDevice = myDevices.back();

  strncpy_safe(newDevice.nameDevice, name, MAX_DESCRIPTION_LENGTH);
  newDevice.isSelected = activ;
  newDevice.isTimersEnabled = false;
  newDevice.isEncyclateTimers = true;
  newDevice.isScheduleEnabled = false;
  newDevice.isActionEnabled = false;

#ifdef ESP32
#ifdef CONFIG_IDF_TARGET_ESP32S2
  newDevice.pins = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 21, 33};
#elif defined(CONFIG_IDF_TARGET_ESP32)

  newDevice.pins = { 0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 };

#else
  newDevice.pins = {1, 2, 3, 4, 5, 6, 7, 8, 9};
#endif
#elif defined(ESP8266)
  newDevice.pins = {1, 2, 3, 4, 5, 12, 13, 14, 17};
#else
  newDevice.pins = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
#endif

  int nextId = 0;

  const int output_pins[] = {3, 18, 19, 21};
  const bool output_manual_mode[] = {true, false, true, false};
  const bool output_state_pin[] = {true, false, true, false};

  for (int i = 0; i < 4; i++) {
    Relay relay;
    relay.id = nextId++;
    relay.pin = output_pins[i];
    relay.manualMode = output_manual_mode[i];
    relay.statePin = output_state_pin[i];
    relay.isOutput = true;
    relay.isDigital = true;
    relay.lastState = false;
    relay.isPwm = false;
    relay.pwm = 0;
    snprintf(relay.description, MAX_DESCRIPTION_LENGTH, "Выход_%d", i + 1);
    newDevice.relays.push_back(relay);
  }

  Relay dhtInput;
  dhtInput.id = nextId++;
  dhtInput.pin = 23;
  dhtInput.manualMode = false;
  dhtInput.isOutput = false;
  dhtInput.isDigital = true;
  dhtInput.statePin = false;
  dhtInput.lastState = false;
  strncpy_safe(dhtInput.description, "DHT-11 Датчик", MAX_DESCRIPTION_LENGTH);
  newDevice.relays.push_back(dhtInput);

  Relay currentInput;
  currentInput.id = nextId++;
  currentInput.pin = 33;
  currentInput.manualMode = false;
  currentInput.isOutput = false;
  currentInput.isDigital = false;
  currentInput.statePin = false;
  currentInput.lastState = false;
  strncpy_safe(currentInput.description, "Вход датчик тока", MAX_DESCRIPTION_LENGTH);
  newDevice.relays.push_back(currentInput);

  Sensor dhtSensor;
  strncpy_safe(dhtSensor.description, "Сенсор DHT11", MAX_DESCRIPTION_LENGTH);
  dhtSensor.isUseSetting = true;
  dhtSensor.sensorId = nextId++;
  dhtSensor.relayId = 4;
  dhtSensor.typeSensor.clear();
  dhtSensor.typeSensor.set(0, true);
  dhtSensor.serial_r = 20000;
  dhtSensor.thermistor_r = 10000;
  dhtSensor.currentValue = 0.0;
  dhtSensor.humidityValue = 0.0;
  dhtSensor.dht = nullptr;
  newDevice.sensors.push_back(dhtSensor);

  Sensor currentSensor;
  strncpy_safe(currentSensor.description, "Датчик тока", MAX_DESCRIPTION_LENGTH);
  currentSensor.isUseSetting = true;
  currentSensor.sensorId = nextId++;
  currentSensor.relayId = 5;
  currentSensor.typeSensor.clear();
  currentSensor.typeSensor.set(4, true);
  currentSensor.serial_r = 20000;
  currentSensor.thermistor_r = 10000;
  currentSensor.currentValue = 0.0;
  currentSensor.humidityValue = 0.0;
  newDevice.sensors.push_back(currentSensor);

  Action touchAction;
  strncpy_safe(touchAction.description, "Действие - превышение тока", MAX_DESCRIPTION_LENGTH);
  touchAction.isUseSetting = true;
  touchAction.targetRelayId = -1;
  touchAction.relayMustBeOn = false;
  touchAction.targetSensorId = currentSensor.sensorId;
  touchAction.triggerValueMax = 1;
  touchAction.triggerValueMin = 0.5;
  touchAction.isHumidity = false;
  touchAction.actionMoreOrEqual = true;
  touchAction.isReturnSetting = true;
  touchAction.wasTriggered = false;
  touchAction.collectionSettings.clear();
  touchAction.collectionSettings.set(1, true);
  touchAction.sendMsg = "";

  OutPower defaultTouchOutput;
  defaultTouchOutput.isUseSetting = true;
  defaultTouchOutput.relayId = newDevice.relays[0].id;
  defaultTouchOutput.statePin = false;
  defaultTouchOutput.lastState = false;
  defaultTouchOutput.isPwm = false;
  defaultTouchOutput.pwm = 0;
  defaultTouchOutput.isReturn = true;

  touchAction.outputs.push_back(defaultTouchOutput);
  newDevice.actions.push_back(touchAction);

  ScheduleScenario scenario;
  strncpy_safe(scenario.description, "Мой первый сценарий 1", MAX_DESCRIPTION_LENGTH);
  scenario.isUseSetting = false;
  scenario.isActive = false;
  scenario.collectionSettings.clear();
  scenario.collectionSettings.set(3, true);
  strncpy_safe(scenario.startDate, "2012-12-12", MAX_DATE_LENGTH);
  strncpy_safe(scenario.endDate, "2222-12-12", MAX_DATE_LENGTH);

  startEndTime timeInterval;
  strncpy_safe(timeInterval.startTime, "08:00", MAX_TIME_LENGTH);
  strncpy_safe(timeInterval.endTime, "18:00", MAX_TIME_LENGTH);
  scenario.startEndTimes.push_back(timeInterval);
  scenario.week.bits = 0x7F;
  scenario.months.bits = 0xFFF;

  scenario.initialStateRelay.isUseSetting = true;
  scenario.initialStateRelay.relayId = newDevice.relays[0].id;
  scenario.initialStateRelay.statePin = true;
  scenario.initialStateRelay.lastState = false;
  scenario.initialStateRelay.isPwm = false;
  scenario.initialStateRelay.pwm = 0;
  scenario.initialStateRelay.isReturn = false;

  scenario.endStateRelay.isUseSetting = false;
  scenario.endStateRelay.relayId = newDevice.relays[0].id;
  scenario.endStateRelay.statePin = false;
  scenario.endStateRelay.lastState = false;
  scenario.endStateRelay.isPwm = false;
  scenario.endStateRelay.pwm = 0;
  scenario.endStateRelay.isReturn = false;

  scenario.temperatureUpdated = false;
  scenario.timersExecuted = false;
  scenario.initialStateApplied = false;
  scenario.endStateApplied = false;
  scenario.scenarioProcessed = false;
  newDevice.scheduleScenarios.push_back(scenario);

  newDevice.temperature.isUseSetting = false;
  newDevice.temperature.relayId = newDevice.relays[0].id;
  newDevice.temperature.lastState = false;
  newDevice.temperature.sensorId = dhtSensor.sensorId;
  newDevice.temperature.setTemperature = 22;
  newDevice.temperature.currentTemp = 0.0;
  newDevice.temperature.isSmoothly = false;
  newDevice.temperature.isIncrease = true;
  newDevice.temperature.collectionSettings.clear();
  newDevice.temperature.collectionSettings.set(0, true);
  newDevice.temperature.selectedPidIndex = 0;
  newDevice.temperature.pidOutputMs = 0;
  newDevice.temperature.sensorPtr = nullptr;
  newDevice.temperature.relayPtr = nullptr;

  Pid pid1, pid2, pid3;

  strncpy_safe(pid1.description, "Стандартный", MAX_DESCRIPTION_LENGTH);
  pid1.Kp = 2.0; pid1.Ki = 0.5; pid1.Kd = 1.0;

  strncpy_safe(pid2.description, "Быстрый", MAX_DESCRIPTION_LENGTH);
  pid2.Kp = 1.5; pid2.Ki = 0.4; pid2.Kd = 0.9;

  strncpy_safe(pid3.description, "Плавный", MAX_DESCRIPTION_LENGTH);
  pid3.Kp = 1.0; pid3.Ki = 0.3; pid3.Kd = 0.8;

  newDevice.pids.push_back(pid1);
  newDevice.pids.push_back(pid2);
  newDevice.pids.push_back(pid3);

  Timer timer;
  timer.isUseSetting = true;
  strncpy_safe(timer.time, "00:00:05", MAX_TIME_LENGTH);
  timer.collectionSettings.clear();
  timer.collectionSettings.set(1, true);

  timer.initialStateRelay.isUseSetting = true;
  timer.initialStateRelay.relayId = newDevice.relays[0].id;
  timer.initialStateRelay.statePin = true;
  timer.initialStateRelay.lastState = false;
  timer.initialStateRelay.isPwm = false;
  timer.initialStateRelay.pwm = 0;
  timer.initialStateRelay.isReturn = false;

  timer.endStateRelay.isUseSetting = true;
  timer.endStateRelay.relayId = newDevice.relays[0].id;
  timer.endStateRelay.statePin = false;
  timer.endStateRelay.lastState = false;
  timer.endStateRelay.isPwm = false;
  timer.endStateRelay.pwm = 0;
  timer.endStateRelay.isReturn = false;

  newDevice.timers.push_back(timer);

  Serial.printf("Free after initialization: %d\n", ESP.getFreeHeap());
}

String DeviceManager::serializeDevice(const Device& device, const char* fileName, AsyncWebServerRequest *request) {
    Serial.println("DeviceManager: Начинаю сериализацию устройства...");

    DynamicJsonDocument doc(SIZE_JSON_S);

    if (strlen(device.nameDevice) > 0) {
        doc["nmd"] = device.nameDevice;
    } else {
        doc["nmd"] = "";
    }
    doc["isl"] = device.isSelected;

     JsonArray pins = doc.createNestedArray("pins");
  for (const auto& pin : device.pins) {
    pins.add(pin);
  }

    JsonArray rel = doc.createNestedArray("rel");
    for (const auto& relay : device.relays) {
        JsonObject relayObj = rel.createNestedObject();
        relayObj["id"] = relay.id;
        relayObj["pin"] = relay.pin;
        relayObj["man"] = relay.manualMode;
        relayObj["stp"] = relay.statePin;
        relayObj["out"] = relay.isOutput;
        relayObj["dig"] = relay.isDigital;
        relayObj["lst"] = relay.lastState;
        relayObj["dsc"] = relay.description;
    }

    JsonArray pinL = doc.createNestedArray("pinL");
    for (const auto& pin : device.pins) {
        pinL.add(pin);
    }

    JsonArray sen = doc.createNestedArray("sen");
    for (const auto& sensor : device.sensors) {
        JsonObject sensorObj = sen.createNestedObject();

        sensorObj["dsc"] =  sensor.description;
        sensorObj["use"] = sensor.isUseSetting;
        sensorObj["sid"] = sensor.sensorId;
        sensorObj["rid"] = sensor.relayId;

        JsonArray typ = sensorObj.createNestedArray("typ");
        for (int i = 0; i < 7; i++) {
            typ.add(sensor.typeSensor.get(i));
        }

        sensorObj["ser"] = sensor.serial_r;
        sensorObj["thm"] = sensor.thermistor_r;
    }

    JsonArray act = doc.createNestedArray("act");
    for (const auto& action : device.actions) {
        JsonObject actionObj = act.createNestedObject();

        actionObj["dsc"] = action.description;
        actionObj["use"] = action.isUseSetting;
        actionObj["trd"] = action.targetRelayId;
        actionObj["rmb"] = action.relayMustBeOn;
        actionObj["tsd"] = action.targetSensorId;
        actionObj["tvm"] = action.triggerValueMax;
        actionObj["tvi"] = action.triggerValueMin;
        actionObj["hum"] = action.isHumidity;
        actionObj["ame"] = action.actionMoreOrEqual;
        actionObj["irs"] = action.isReturnSetting;
        actionObj["wtr"] = action.wasTriggered;
        actionObj["msg"] = action.sendMsg;

        JsonArray cls = actionObj.createNestedArray("cls");
        for (int i = 0; i < 4; i++) {
            cls.add(action.collectionSettings.get(i));
        }

        JsonArray outL = actionObj.createNestedArray("outL");
        for (const auto& output : action.outputs) {
            JsonObject outputObj = outL.createNestedObject();
            outputObj["use"] = output.isUseSetting;
            outputObj["rid"] = output.relayId;
            outputObj["stp"] = output.statePin;
            outputObj["lst"] = output.lastState;
            outputObj["rtn"] = output.isReturn;

        }
    }

    JsonArray sch = doc.createNestedArray("sch");
    for (const auto& scenario : device.scheduleScenarios) {
        JsonObject scenarioObj = sch.createNestedObject();
        scenarioObj["use"] = scenario.isUseSetting;
        scenarioObj["dsc"] = scenario.description;
        scenarioObj["iac"] = scenario.isActive;

        JsonArray cls = scenarioObj.createNestedArray("cls");
        for (int i = 0; i < 4; i++) {
            cls.add(scenario.collectionSettings.get(i));
        }

        scenarioObj["sdt"] = scenario.startDate;
        scenarioObj["edt"] = scenario.endDate;

        JsonArray set = scenarioObj.createNestedArray("set");
        for (const auto& timeInterval : scenario.startEndTimes) {
            JsonObject intervalObj = set.createNestedObject();
            intervalObj["stm"] = timeInterval.startTime;
            intervalObj["etm"] = timeInterval.endTime;
        }

        JsonArray wek = scenarioObj.createNestedArray("wek");
        for (int i = 0; i < 7; i++) {
            wek.add(scenario.week.get(i));
        }

        JsonArray mon = scenarioObj.createNestedArray("mon");
        for (int i = 0; i < 12; i++) {
            mon.add(scenario.months.get(i));
        }

        JsonObject isr = scenarioObj.createNestedObject("isr");
        isr["use"] = scenario.initialStateRelay.isUseSetting;
        isr["rid"] = scenario.initialStateRelay.relayId;
        isr["stp"] = scenario.initialStateRelay.statePin;
        isr["lst"] = scenario.initialStateRelay.lastState;

        JsonObject esr = scenarioObj.createNestedObject("esr");
        esr["use"] = scenario.endStateRelay.isUseSetting;
        esr["rid"] = scenario.endStateRelay.relayId;
        esr["stp"] = scenario.endStateRelay.statePin;
        esr["lst"] = scenario.endStateRelay.lastState;

    }

    JsonObject tmp = doc.createNestedObject("tmp");
    tmp["use"] = device.temperature.isUseSetting;
    tmp["rid"] = device.temperature.relayId;
    tmp["lst"] = device.temperature.lastState;
    tmp["sid"] = device.temperature.sensorId;
    tmp["stT"] = device.temperature.setTemperature;
    tmp["ctp"] = device.temperature.currentTemp;
    tmp["smt"] = device.temperature.isSmoothly;
    tmp["inc"] = device.temperature.isIncrease;

    JsonArray tempCls = tmp.createNestedArray("cls");
    for (int i = 0; i < 4; i++) {
        tempCls.add(device.temperature.collectionSettings.get(i));
    }

    tmp["spi"] = device.temperature.selectedPidIndex;

    JsonArray pid = doc.createNestedArray("pid");
    for (const auto& pid_item : device.pids) {
        JsonObject pidObject = pid.createNestedObject();
        pidObject["dsc"] = pid_item.description;
        pidObject["Kp"] = pid_item.Kp;
        pidObject["Ki"] = pid_item.Ki;
        pidObject["Kd"] = pid_item.Kd;
    }

    JsonArray tmr = doc.createNestedArray("tmr");
    for (const auto& timer : device.timers) {
        JsonObject timerObj = tmr.createNestedObject();
        timerObj["use"] = timer.isUseSetting;
        timerObj["tim"] = timer.time;

        JsonArray cls = timerObj.createNestedArray("cls");
        for (int i = 0; i < 4; i++) {
            cls.add(timer.collectionSettings.get(i));
        }

        JsonObject isr = timerObj.createNestedObject("isr");
        isr["use"] = timer.initialStateRelay.isUseSetting;
        isr["rid"] = timer.initialStateRelay.relayId;
        isr["stp"] = timer.initialStateRelay.statePin;
        isr["lst"] = timer.initialStateRelay.lastState;

        JsonObject esr = timerObj.createNestedObject("esr");
        esr["use"] = timer.endStateRelay.isUseSetting;
        esr["rid"] = timer.endStateRelay.relayId;
        esr["stp"] = timer.endStateRelay.statePin;
        esr["lst"] = timer.endStateRelay.lastState;

    }

    doc["ite"] = device.isTimersEnabled;
    doc["iet"] = device.isEncyclateTimers;
    doc["ise"] = device.isScheduleEnabled;
    doc["iae"] = device.isActionEnabled;

    if (request != nullptr) {
        Serial.println("DeviceManager: Режим 'отправка в поток' (streaming).");
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        serializeJson(doc, *response);
        request->send(response);
        return "sending";
    }

    if (fileName != nullptr) {
        Serial.printf("DeviceManager: Режим 'сохранение в файл': %s\n", fileName);
        File file = SPIFFS.open(fileName, "w");
        if (!file) {
            Serial.println("❌ DeviceManager: Не удалось открыть файл для записи.");
            return "error";
        }
        if (serializeJson(doc, file) == 0) {
            file.close();
            Serial.println("❌ DeviceManager: Ошибка записи JSON в файл.");
            return "error";
        }
        file.println();
        file.close();
        Serial.printf("✅ DeviceManager: Файл %s успешно сохранен.\n", fileName);
        return "success";
    }

    Serial.println("DeviceManager: Режим 'возврат строки'.");
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

bool DeviceManager::deserializeDevice(JsonObject doc, Device& device) {

  Serial.println("DeviceManager: Начало десериализации с короткими ключами");

   if (doc.containsKey("nmd")) {
        strncpy_safe(device.nameDevice, doc["nmd"], MAX_DESCRIPTION_LENGTH);
      }

  if (doc.containsKey("isl")) {
    device.isSelected = doc["isl"].as<bool>();
  }

  if (doc.containsKey("rel")) {
    device.relays.clear();
    JsonArray relays = doc["rel"];
    device.relays.reserve(relays.size());
    for (JsonObject relayObj : relays) {
      Relay relay;
      if (relayObj.containsKey("id")) relay.id = relayObj["id"].as<int>();
      if (relayObj.containsKey("pin")) relay.pin = relayObj["pin"].as<uint8_t>();
      if (relayObj.containsKey("man")) relay.manualMode = relayObj["man"].as<bool>();
      if (relayObj.containsKey("stp")) relay.statePin = relayObj["stp"].as<bool>();
      if (relayObj.containsKey("lst")) relay.lastState = relayObj["lst"].as<bool>();
      if (relayObj.containsKey("out")) relay.isOutput = relayObj["out"].as<bool>();
      if (relayObj.containsKey("isPwm")) relay.isPwm = relayObj["isPwm"].as<bool>();
      if (relayObj.containsKey("pwm")) relay.pwm = relayObj["pwm"].as<uint8_t>();
      if (relayObj.containsKey("dig")) relay.isDigital = relayObj["dig"].as<bool>();
      if (relayObj.containsKey("dsc")) {
        strncpy_safe(relay.description, relayObj["dsc"], MAX_DESCRIPTION_LENGTH);
      }
      device.relays.push_back(relay);
    }
  }

  if (doc.containsKey("pinL")) {
    device.pins.clear();
    JsonArray pins = doc["pinL"];
    device.pins.reserve(pins.size());
    for (JsonVariant pin : pins) {
      device.pins.push_back(pin.as<uint8_t>());
    }
  }

  if (doc.containsKey("sen")) {
    JsonArray sensorsJson = doc["sen"];
    std::vector<Sensor> newSensors;
    newSensors.reserve(sensorsJson.size());

    for (JsonObject sensorObj : sensorsJson) {
      Sensor sensor;

      if (sensorObj.containsKey("dsc")) {
        strncpy_safe(sensor.description, sensorObj["dsc"], MAX_DESCRIPTION_LENGTH);
      }
      if (sensorObj.containsKey("use")) sensor.isUseSetting = sensorObj["use"].as<bool>();
      if (sensorObj.containsKey("sid")) sensor.sensorId = sensorObj["sid"];
      if (sensorObj.containsKey("rid")) sensor.relayId = sensorObj["rid"];

      if (sensorObj.containsKey("typ")) {
        JsonArray typeSensor = sensorObj["typ"];
        for (int i = 0; i < 7 && i < typeSensor.size(); i++) {
          sensor.typeSensor.set(i, typeSensor[i].as<bool>());
        }
      }

      if (sensorObj.containsKey("ser")) sensor.serial_r = sensorObj["ser"];
      if (sensorObj.containsKey("thm")) sensor.thermistor_r = sensorObj["thm"];

      newSensors.push_back(sensor);
    }

    noInterrupts();
    device.sensors = std::move(newSensors);
    interrupts();
  }

  if (doc.containsKey("act")) {
    JsonArray actionsJson = doc["act"];
    std::vector<Action> newActions;
    newActions.reserve(actionsJson.size());

    for (JsonObject actionObj : actionsJson) {
      Action action;

      if (actionObj.containsKey("dsc")) {
        strncpy_safe(action.description, actionObj["dsc"], MAX_DESCRIPTION_LENGTH);
      }
      if (actionObj.containsKey("use")) action.isUseSetting = actionObj["use"].as<bool>();
      if (actionObj.containsKey("tsd")) action.targetSensorId = actionObj["tsd"].as<int>();;
      if (actionObj.containsKey("tvm")) action.triggerValueMax = actionObj["tvm"].as<float>();
      if (actionObj.containsKey("tvi")) action.triggerValueMin = actionObj["tvi"].as<float>();
      if (actionObj.containsKey("hum")) action.isHumidity = actionObj["hum"].as<bool>();
      if (actionObj.containsKey("ame")) action.actionMoreOrEqual = actionObj["ame"].as<bool>();
      if (actionObj.containsKey("irs")) action.isReturnSetting = actionObj["irs"].as<bool>();
      if (actionObj.containsKey("wtr")) action.wasTriggered = actionObj["wtr"].as<bool>();
      if (actionObj.containsKey("msg")) action.sendMsg = actionObj["msg"].as<String>();

      if (actionObj.containsKey("trd"))
        action.targetRelayId = actionObj["trd"];
      else
        action.targetRelayId = -1;

      if (actionObj.containsKey("rmb"))
        action.relayMustBeOn = actionObj["rmb"].as<bool>();
      else
        action.relayMustBeOn = false;

      if (actionObj.containsKey("cls")) {
        JsonArray collectionSettings = actionObj["cls"];
        for (int i = 0; i < 4 && i < collectionSettings.size(); i++) {
          action.collectionSettings.set(i, collectionSettings[i].as<bool>());
        }
      }

      if (actionObj.containsKey("outL")) {
        JsonArray outputsJson = actionObj["outL"];
        action.outputs.clear();
        action.outputs.reserve(outputsJson.size());
        for (JsonObject outputObj : outputsJson) {
          OutPower output;
          if (outputObj.containsKey("use")) output.isUseSetting = outputObj["use"].as<bool>();
          if (outputObj.containsKey("rid")) output.relayId = outputObj["rid"];
          if (outputObj.containsKey("stp")) output.statePin = outputObj["stp"].as<bool>();
          if (outputObj.containsKey("lst")) output.lastState = outputObj["lst"].as<bool>();
          if (outputObj.containsKey("rtn")) output.isReturn = outputObj["rtn"].as<bool>();

          action.outputs.push_back(output);
        }
      }

      newActions.push_back(action);
    }

    noInterrupts();
    device.actions = std::move(newActions);
    interrupts();
  }

  if (doc.containsKey("sch")) {
    device.scheduleScenarios.clear();
    JsonArray scenarios = doc["sch"];
    device.scheduleScenarios.reserve(scenarios.size());
    for (JsonObject scenarioObj : scenarios) {
      ScheduleScenario scenario;
      scenario.temperatureUpdated = false;
      scenario.timersExecuted = false;
      scenario.initialStateApplied = false;
      scenario.endStateApplied = false;
      scenario.scenarioProcessed = false;

      if (scenarioObj.containsKey("use")) scenario.isUseSetting = scenarioObj["use"].as<bool>();
      if (scenarioObj.containsKey("dsc")) {
        strncpy_safe(scenario.description, scenarioObj["dsc"], MAX_DESCRIPTION_LENGTH);
      }
      if (scenarioObj.containsKey("iac")) scenario.isActive = scenarioObj["iac"].as<bool>();

      if (scenarioObj.containsKey("cls")) {
        JsonArray collectionSettings = scenarioObj["cls"];
        for (int i = 0; i < 4 && i < collectionSettings.size(); i++) {
          scenario.collectionSettings.set(i, collectionSettings[i].as<bool>());
        }
      }

      if (scenarioObj.containsKey("sdt")) {
        strncpy_safe(scenario.startDate, scenarioObj["sdt"], MAX_DATE_LENGTH);
      }
      if (scenarioObj.containsKey("edt")) {
        strncpy_safe(scenario.endDate, scenarioObj["edt"], MAX_DATE_LENGTH);
      }

      if (scenarioObj.containsKey("set")) {
        JsonArray startEndTimes = scenarioObj["set"];
        for (JsonObject intervalObj : startEndTimes) {
          startEndTime timeInterval;
          if (intervalObj.containsKey("stm")) {
            strncpy_safe(timeInterval.startTime, intervalObj["stm"], MAX_TIME_LENGTH);
          }
          if (intervalObj.containsKey("etm")) {
            strncpy_safe(timeInterval.endTime, intervalObj["etm"], MAX_TIME_LENGTH);
          }
          scenario.startEndTimes.push_back(timeInterval);
        }
      }

      if (scenarioObj.containsKey("wek")) {
        JsonArray week = scenarioObj["wek"];
        for (int i = 0; i < 7 && i < week.size(); i++) {
          scenario.week.set(i, week[i].as<bool>());
        }
      }

      if (scenarioObj.containsKey("mon")) {
        JsonArray months = scenarioObj["mon"];
        for (int i = 0; i < 12 && i < months.size(); i++) {
          scenario.months.set(i, months[i].as<bool>());
        }
      }

      if (scenarioObj.containsKey("isr")) {
        JsonObject initialStateRelay = scenarioObj["isr"];
        if (initialStateRelay.containsKey("use")) scenario.initialStateRelay.isUseSetting = initialStateRelay["use"].as<bool>();
        if (initialStateRelay.containsKey("rid")) scenario.initialStateRelay.relayId = initialStateRelay["rid"];
        if (initialStateRelay.containsKey("stp")) scenario.initialStateRelay.statePin = initialStateRelay["stp"].as<bool>();
        if (initialStateRelay.containsKey("lst")) scenario.initialStateRelay.lastState = initialStateRelay["lst"].as<bool>();

      }

      if (scenarioObj.containsKey("esr")) {
        JsonObject endStateRelay = scenarioObj["esr"];
        if (endStateRelay.containsKey("use")) scenario.endStateRelay.isUseSetting = endStateRelay["use"].as<bool>();
        if (endStateRelay.containsKey("rid")) scenario.endStateRelay.relayId = endStateRelay["rid"];
        if (endStateRelay.containsKey("stp")) scenario.endStateRelay.statePin = endStateRelay["stp"].as<bool>();
        if (endStateRelay.containsKey("lst")) scenario.endStateRelay.lastState = endStateRelay["lst"].as<bool>();

      }

      device.scheduleScenarios.push_back(scenario);
    }
  }

  if (doc.containsKey("tmp")) {
    JsonObject temperature = doc["tmp"];
    if (temperature.containsKey("use")) device.temperature.isUseSetting = temperature["use"].as<bool>();
    if (temperature.containsKey("rid")) device.temperature.relayId = temperature["rid"];
    if (temperature.containsKey("lst")) device.temperature.lastState = temperature["lst"].as<bool>();
    if (temperature.containsKey("sid")) device.temperature.sensorId = temperature["sid"];
    if (temperature.containsKey("stT")) device.temperature.setTemperature = temperature["stT"];
    if (temperature.containsKey("ctp")) device.temperature.currentTemp = temperature["ctp"];
    if (temperature.containsKey("smt")) device.temperature.isSmoothly = temperature["smt"].as<bool>();
    if (temperature.containsKey("inc")) device.temperature.isIncrease = temperature["inc"].as<bool>();
    if (temperature.containsKey("cls")) {
      JsonArray collectionSettings = temperature["cls"];
      for (int i = 0; i < 4 && i < collectionSettings.size(); i++) {
        device.temperature.collectionSettings.set(i, collectionSettings[i].as<bool>());
      }
    }
    if (temperature.containsKey("spi")) device.temperature.selectedPidIndex = temperature["spi"];
  }

  if (doc.containsKey("pid")) {
    device.pids.clear();
    JsonArray pidsArray = doc["pid"].as<JsonArray>();
    device.pids.reserve(pidsArray.size());
    for (JsonObject pidObject : pidsArray) {
      Pid pid;
      if (pidObject.containsKey("dsc")) {
        strncpy_safe(pid.description, pidObject["dsc"], MAX_DESCRIPTION_LENGTH);
      }
      if (pidObject.containsKey("Kp")) pid.Kp = pidObject["Kp"];
      if (pidObject.containsKey("Ki")) pid.Ki = pidObject["Ki"];
      if (pidObject.containsKey("Kd")) pid.Kd = pidObject["Kd"];
      device.pids.push_back(pid);
    }
  }

  if (doc.containsKey("tmr")) {
    device.timers.clear();
    JsonArray timers = doc["tmr"];
    device.timers.reserve(timers.size());
    for (JsonObject timerObj : timers) {
      Timer timer;
      if (timerObj.containsKey("use")) timer.isUseSetting = timerObj["use"].as<bool>();
      if (timerObj.containsKey("tim")) {
        strncpy_safe(timer.time, timerObj["tim"], MAX_TIME_LENGTH);
      }
      if (timerObj.containsKey("cls")) {
        JsonArray collectionSettings = timerObj["cls"];
        for (int i = 0; i < 4 && i < collectionSettings.size(); i++) {
          timer.collectionSettings.set(i, collectionSettings[i].as<bool>());
        }
      }
      if (timerObj.containsKey("isr")) {
        JsonObject initialStateRelay = timerObj["isr"];
        if (initialStateRelay.containsKey("use")) timer.initialStateRelay.isUseSetting = initialStateRelay["use"].as<bool>();
        if (initialStateRelay.containsKey("rid")) timer.initialStateRelay.relayId = initialStateRelay["rid"];
        if (initialStateRelay.containsKey("stp")) timer.initialStateRelay.statePin = initialStateRelay["stp"].as<bool>();
        if (initialStateRelay.containsKey("lst")) timer.initialStateRelay.lastState = initialStateRelay["lst"].as<bool>();

      }
      if (timerObj.containsKey("esr")) {
        JsonObject endStateRelay = timerObj["esr"];
        if (endStateRelay.containsKey("use")) timer.endStateRelay.isUseSetting = endStateRelay["use"].as<bool>();
        if (endStateRelay.containsKey("rid")) timer.endStateRelay.relayId = endStateRelay["rid"];
        if (endStateRelay.containsKey("stp")) timer.endStateRelay.statePin = endStateRelay["stp"].as<bool>();
        if (endStateRelay.containsKey("lst")) timer.endStateRelay.lastState = endStateRelay["lst"].as<bool>();

      }
      device.timers.push_back(timer);
    }
  }

  if (doc.containsKey("ite")) device.isTimersEnabled = doc["ite"].as<bool>();
  if (doc.containsKey("iet")) device.isEncyclateTimers = doc["iet"].as<bool>();
  if (doc.containsKey("ise")) device.isScheduleEnabled = doc["ise"].as<bool>();
  if (doc.containsKey("iae")) device.isActionEnabled = doc["iae"].as<bool>();

  Serial.println("DeviceManager: конец десериализации");
  return true;
}

bool DeviceManager::deserializeDevice(const char* jsonString, Device& device) {
 Serial.println("DeviceManager: вызов обертки дессереализации");
DynamicJsonDocument doc(SIZE_JSON_S);

  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.print("Deserialization error: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonObject json = doc.as<JsonObject>();
  return deserializeDevice(json, device);
}

void DeviceManager::setRelayStateForAllDevices(uint8_t targetRelayId, bool state) {
  for (auto& device : myDevices) {

    Relay* relay = findRelayById(device, targetRelayId);
    if (relay) {
      relay->statePin = state;
    }

    if (device.temperature.isUseSetting && device.temperature.relayId == targetRelayId) {

      if (relay) {
        relay->statePin = state;
      }
    }

    for (auto& timer : device.timers) {
      if (timer.isUseSetting) {
        if (timer.initialStateRelay.relayId == targetRelayId) {
          timer.initialStateRelay.statePin = state;
        }
        if (timer.endStateRelay.relayId == targetRelayId) {
          timer.endStateRelay.statePin = state;
        }
      }
    }

    for (auto& scenario : device.scheduleScenarios) {
      if (scenario.isUseSetting) {
        if (scenario.initialStateRelay.relayId == targetRelayId) {
          scenario.initialStateRelay.statePin = state;
        }
        if (scenario.endStateRelay.relayId == targetRelayId) {
          scenario.endStateRelay.statePin = state;
        }
      }
    }
  }
}

void DeviceManager::saveRelayStates(uint8_t targetRelayId) {
  for (auto& device : myDevices) {

    Relay* relay = findRelayById(device, targetRelayId);
    if (relay) {
      relay->lastState = relay->statePin;
    }

    if (device.temperature.relayId == targetRelayId) {
      if (relay) {
        device.temperature.lastState = relay->statePin;
      }
    }

    for (auto& timer : device.timers) {
      if (timer.initialStateRelay.relayId == targetRelayId) {
        timer.initialStateRelay.lastState = relay ? relay->statePin : false;
      }
      if (timer.endStateRelay.relayId == targetRelayId) {
        timer.endStateRelay.lastState = relay ? relay->statePin : false;
      }
    }

    for (auto& scenario : device.scheduleScenarios) {
      if (scenario.initialStateRelay.relayId == targetRelayId) {
        scenario.initialStateRelay.lastState = relay ? relay->statePin : false;
      }
      if (scenario.endStateRelay.relayId == targetRelayId) {
        scenario.endStateRelay.lastState = relay ? relay->statePin : false;
      }
    }
  }
}

void DeviceManager::restoreRelayStates(uint8_t targetRelayId) {
  for (auto& device : myDevices) {

    Relay* relay = findRelayById(device, targetRelayId);
    if (relay) {
      relay->statePin = relay->lastState;
    }

    if (device.temperature.relayId == targetRelayId) {
      if (relay) {
        relay->statePin = device.temperature.lastState;
      }
    }

    for (auto& timer : device.timers) {
      if (timer.initialStateRelay.relayId == targetRelayId) {
        if (relay) {
          relay->statePin = timer.initialStateRelay.lastState;
        }
      }
      if (timer.endStateRelay.relayId == targetRelayId) {
        if (relay) {
          relay->statePin = timer.endStateRelay.lastState;
        }
      }
    }

    for (auto& scenario : device.scheduleScenarios) {
      if (scenario.initialStateRelay.relayId == targetRelayId) {
        if (relay) {
          relay->statePin = scenario.initialStateRelay.lastState;
        }
      }
      if (scenario.endStateRelay.relayId == targetRelayId) {
        if (relay) {
          relay->statePin = scenario.endStateRelay.lastState;
        }
      }
    }
  }
}

Relay* DeviceManager::findRelayById(Device& device, uint8_t relayId) {
  for (auto& relay : device.relays) {
    if (relay.id == relayId) {
      return &relay;
    }
  }
  return nullptr;
}

int DeviceManager::findRelayIndexById(const Device& device, uint8_t relayId) {
  for (size_t i = 0; i < device.relays.size(); i++) {
    if (device.relays[i].id == relayId) {
      return i;
    }
  }
  return -1;
}

int DeviceManager::findSensorIndexById(const Device& device, int sensorId) {
  for (size_t i = 0; i < device.sensors.size(); i++) {
    if (device.sensors[i].sensorId == sensorId) {
      return i;
    }
  }
  return -1;
}

void DeviceManager::validateAndSetRelayId(uint8_t& relayId, const std::vector<Relay>& relays) {
  bool found = false;
  for (const auto& relay : relays) {
    if (relay.id == relayId) {
      found = true;
      break;
    }
  }
  if (!found && !relays.empty()) {
    relayId = relays[0].id;
  }
}

void DeviceManager::validateRelayIds(Device& device) {
  validateAndSetRelayId(device.temperature.relayId, device.relays);

  for (auto& scenario : device.scheduleScenarios) {
    validateAndSetRelayId(scenario.initialStateRelay.relayId, device.relays);
    validateAndSetRelayId(scenario.endStateRelay.relayId, device.relays);
  }

  for (auto& timer : device.timers) {
    validateAndSetRelayId(timer.initialStateRelay.relayId, device.relays);
    validateAndSetRelayId(timer.endStateRelay.relayId, device.relays);
  }
}

bool DeviceManager::writeDevicesToFile(const std::vector<Device>& myDevices, const char* filename) {
  Serial.println("Начинаем запись устройств в файл...");

  for (const auto& device : myDevices) {

    serializeDevice(device, filename, nullptr);

  }

  Serial.println("Запись устройств в файл завершена.");

  return true;
}

bool DeviceManager::readDevicesFromFile(std::vector<Device>& myDevices, const char* filename) {
    Serial.println("readDevicesFromFile (reading full file)");
    Serial.printf("Free heap before: %d\n", ESP.getFreeHeap());

    File file = SPIFFS.open(filename, "r");
    if (!file || !file.available()) {
        Serial.println("Файл не найден или пуст, это нормально для первого запуска.");
        if (file) file.close();
        return false;
    }

    size_t fileSize = file.size();
    Serial.printf("Размер файла: %d байт\n", fileSize);

    DynamicJsonDocument doc(fileSize * 1.5);

    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("Ошибка десериализации файла: %s\n", error.c_str());
        return false;
    }

    myDevices.clear();

    JsonObject deviceObj = doc.as<JsonObject>();
    if (!deviceObj) {
        Serial.println("Ошибка: корневой элемент JSON не является объектом.");
        return false;
    }

    Device device;
    memset(&device, 0, sizeof(Device));

    if (deserializeDevice(deviceObj, device)) {
        myDevices.push_back(device);
        Serial.println("Успешно загружено 1 устройство из файла.");
    } else {
        Serial.println("Не удалось десериализовать устройство из файла.");
        return false;
    }

    Serial.printf("Свободная память (heap) после загрузки: %d\n", ESP.getFreeHeap());

    return true;
}

int DeviceManager::getSelectedDeviceIndex(const std::vector<Device>& myDevices) {
  for (size_t i = 0; i < myDevices.size(); ++i) {
    if (myDevices[i].isSelected) {
      return i;
    }
  }
  return -1;
}

int DeviceManager::deviceInit() {
  if (!SPIFFS.exists("/devices.json")) {
    initializeDevice("MyDevice1", true);
    writeDevicesToFile(myDevices, "/devices.json");

    Serial.println("Устройство инициализировано и сохранено в файл.");
  } else {
    if (readDevicesFromFile(myDevices, "/devices.json")) {
      return currentDeviceIndex = getSelectedDeviceIndex(myDevices);
    } else {
      Serial.println("Ошибка загрузки устройств из файла.");
      initializeDevice("MyDevice1", true);
      return currentDeviceIndex = 0;
    }
  }
  return currentDeviceIndex = 0;
}

  size_t DeviceManager::formatFullSystemStatus(char* buffer, size_t bufferSize) {
    if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
        return snprintf(buffer, bufferSize, "Нет устройств для отображения статуса\n");
    }

    const Device& device = myDevices[currentDeviceIndex];
    size_t offset = 0;

    offset += snprintf(buffer + offset, bufferSize - offset, "📡 **Датчики:**\n");
    if (device.sensors.empty()) {
        offset += snprintf(buffer + offset, bufferSize - offset, "  Нет настроенных датчиков\n\n");
    } else {
        bool hasActiveSensors = false;
        for (size_t i = 0; i < device.sensors.size(); ++i) {
            const Sensor& sensor = device.sensors[i];
            if (!sensor.isUseSetting) continue;

            hasActiveSensors = true;
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Сенсор #%d (ID: %d): ", i + 1, sensor.sensorId);

            const char* sensorType = "Неизвестный";
            if (sensor.typeSensor.get(0)) sensorType = "DHT11";
            else if (sensor.typeSensor.get(1)) sensorType = "DHT22";
            else if (sensor.typeSensor.get(2)) sensorType = "NTC";
            else if (sensor.typeSensor.get(3)) sensorType = "Touch";
            else if (sensor.typeSensor.get(4)) sensorType = "Аналоговый";

            offset += snprintf(buffer + offset, bufferSize - offset, "%s [Активен]", sensorType);

            bool isSensorOk = true;
            if (sensor.typeSensor.get(0) || sensor.typeSensor.get(1)) {
                if (isnan(sensor.currentValue) || sensor.currentValue < -100.0 || isnan(sensor.humidityValue)) {
                    isSensorOk = false;
                }
            }

            if (isSensorOk) {
                if (sensor.typeSensor.get(0) || sensor.typeSensor.get(1) || sensor.typeSensor.get(2)) {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       " - Значение: %.2f°C", (double)sensor.currentValue);
                    if (sensor.typeSensor.get(0) || sensor.typeSensor.get(1)) {
                        offset += snprintf(buffer + offset, bufferSize - offset,
                                           ", Влажность: %.1f%%", (double)sensor.humidityValue);
                    }
                } else if (sensor.typeSensor.get(3)) {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       " - Состояние: %s", sensor.currentValue > 0.5f ? "Нажато" : "Отпущено");
                } else if (sensor.typeSensor.get(4)) {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       " - Значение: %.0f", (double)sensor.currentValue);
                }
            } else {
                offset += snprintf(buffer + offset, bufferSize - offset, " - Датчик не подключен");
            }
            offset += snprintf(buffer + offset, bufferSize - offset, "\n");
        }
        if (!hasActiveSensors) {
            offset += snprintf(buffer + offset, bufferSize - offset, "  Нет активных датчиков\n");
        }
        offset += snprintf(buffer + offset, bufferSize - offset, "\n");
    }

    if (device.temperature.isUseSetting) {
        offset += snprintf(buffer + offset, bufferSize - offset, "🌡️ **Температурный контроль:**\n");
        offset += snprintf(buffer + offset, bufferSize - offset, "  • Статус: [Активен]\n");

        const Sensor* tempSensor = nullptr;
        for (const auto& sensor : device.sensors) {
            if (sensor.sensorId == device.temperature.sensorId) {
                tempSensor = &sensor;
                break;
            }
        }

        if (tempSensor) {
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Текущая температура: %.2f°C\n", (double)tempSensor->currentValue);

            if (tempSensor->typeSensor.get(0) || tempSensor->typeSensor.get(1)) {
                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  • Влажность: %.2f%%\n", (double)tempSensor->humidityValue);
            }
        } else {
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Датчик температуры не найден (ID: %d)\n", device.temperature.sensorId);
        }

        offset += snprintf(buffer + offset, bufferSize - offset,
                           "  • Установленная температура: %.2f°C\n", (double)device.temperature.setTemperature);
        offset += snprintf(buffer + offset, bufferSize - offset,
                           "  • Направление: %s\n", device.temperature.isIncrease ? "Нагрев" : "Охлаждение");
        offset += snprintf(buffer + offset, bufferSize - offset,
                           "  • Режим: %s\n", device.temperature.isSmoothly ? "Плавный (ШИМ)" : "Релейный");

        if (device.temperature.selectedPidIndex < device.pids.size()) {
            const Pid& pid = device.pids[device.temperature.selectedPidIndex];
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • PID коэффициенты: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid.Kp, pid.Ki, pid.Kd);
        }

        const Relay* tempRelay = nullptr;
        for (const auto& relay : device.relays) {
            if (relay.id == device.temperature.relayId) {
                tempRelay = &relay;
                break;
            }
        }

        if (tempRelay) {
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Управляющее реле: %s (Состояние: %s)\n", tempRelay->description,
                               tempRelay->statePin ? "Вкл" : "Выкл");
        } else {
            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Реле управления не найдено (ID: %d)\n", device.temperature.relayId);
        }
        offset += snprintf(buffer + offset, bufferSize - offset, "\n");
    }

    if (device.isScheduleEnabled) {
        offset += snprintf(buffer + offset, bufferSize - offset, "📅 **Расписания:**\n");
        if (device.scheduleScenarios.empty()) {
            offset += snprintf(buffer + offset, bufferSize - offset, "  Нет настроенных расписаний\n\n");
        } else {
            offset += snprintf(buffer + offset, bufferSize - offset, "  • Общий статус: Активен\n\n");

            for (size_t i = 0; i < device.scheduleScenarios.size(); ++i) {
                const ScheduleScenario& scenario = device.scheduleScenarios[i];
                if (!scenario.isUseSetting) continue;

                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  • Сценарий #%d: %s [Активен]", i + 1, scenario.description);

                if (scenario.isActive) {
                    offset += snprintf(buffer + offset, bufferSize - offset, " [Выполняется]");
                } else {
                    offset += snprintf(buffer + offset, bufferSize - offset, " [Ожидает]");
                }
                offset += snprintf(buffer + offset, bufferSize - offset, "\n");

                char periodText[MAX_DATE_LENGTH * 2 + 10];
                snprintf(periodText, sizeof(periodText), "%s по %s", scenario.startDate, scenario.endDate);
                offset += snprintf(buffer + offset, bufferSize - offset, "    - Период: %s\n", periodText);

                char daysStr[50];
                String tempDays = getActiveDaysString(scenario.week);
                strncpy_safe(daysStr, tempDays.c_str(), sizeof(daysStr));
                offset += snprintf(buffer + offset, bufferSize - offset, "    - Дни: %s\n", daysStr);

                char monthsStr[100];
                String tempMonths = getActiveMonthsString(scenario.months);
                strncpy_safe(monthsStr, tempMonths.c_str(), sizeof(monthsStr));
                offset += snprintf(buffer + offset, bufferSize - offset, "    - Месяцы: %s\n", monthsStr);

                if (!scenario.startEndTimes.empty()) {
                    offset += snprintf(buffer + offset, bufferSize - offset, "    - Время: ");
                    for (size_t j = 0; j < scenario.startEndTimes.size(); ++j) {
                        if (j > 0) offset += snprintf(buffer + offset, bufferSize - offset, ", ");

                        char timeText[MAX_TIME_LENGTH * 2 + 5];
                        snprintf(timeText, sizeof(timeText), "%s-%s", scenario.startEndTimes[j].startTime, scenario.startEndTimes[j].endTime);
                        offset += snprintf(buffer + offset, bufferSize - offset, "%s", timeText);

                    }
                    offset += snprintf(buffer + offset, bufferSize - offset, "\n");
                }

                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "    - Действия: %s%s%s\n",
                                   scenario.collectionSettings.get(0) ? "Температура " : "",
                                   scenario.collectionSettings.get(1) ? "Таймеры " : "",
                                   scenario.collectionSettings.get(2) ? "Реле " : "");

                offset += snprintf(buffer + offset, bufferSize - offset, "\n");
            }
            offset += snprintf(buffer + offset, bufferSize - offset, "\n");
        }
    }

    if (device.isActionEnabled) {
        offset += snprintf(buffer + offset, bufferSize - offset, "⚙️ **Действия по датчикам:**\n");

        if (device.actions.empty()) {
            offset += snprintf(buffer + offset, bufferSize - offset, "  Нет настроенных действий\n\n");
        } else {
            offset += snprintf(buffer + offset, bufferSize - offset, "  • Общий статус: Активен\n\n");

            for (size_t i = 0; i < device.actions.size(); ++i) {
                const Action& action = device.actions[i];
                if (!action.isUseSetting) continue;

                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  • Действие #%d: %s [Активен]", i + 1, action.description);

                if (action.wasTriggered) {
                    offset += snprintf(buffer + offset, bufferSize - offset, " [Сработало]");
                } else {
                    offset += snprintf(buffer + offset, bufferSize - offset, " [Ожидает]");
                }
                offset += snprintf(buffer + offset, bufferSize - offset, "\n");

                const Sensor* targetSensor = nullptr;
                for (const auto& sensor : device.sensors) {
                    if (sensor.sensorId == action.targetSensorId) {
                        targetSensor = &sensor;
                        break;
                    }
                }

                if (targetSensor) {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       "    Датчик: %s (ID: %d)\n", targetSensor->description, targetSensor->sensorId);

                    float currentValue = action.isHumidity ? targetSensor->humidityValue : targetSensor->currentValue;
                    if (!isnan(currentValue)) {
                        offset += snprintf(buffer + offset, bufferSize - offset,
                                           "    Текущее значение: %.2f\n", (double)currentValue);
                    } else {
                        offset += snprintf(buffer + offset, bufferSize - offset,
                                           "    Текущее значение: Н/Д\n");
                    }
                } else {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       "    Датчик не найден (ID: %d)\n", action.targetSensorId);
                }

                offset += snprintf(buffer + offset, bufferSize - offset, "    Пороги: ");
                if (!isnan(action.triggerValueMax)) {
                    offset += snprintf(buffer + offset, bufferSize - offset, "Срабатывание %.2f", (double)action.triggerValueMax);
                } else {
                    offset += snprintf(buffer + offset, bufferSize - offset, "Срабатывание N/A");
                }
                offset += snprintf(buffer + offset, bufferSize - offset, " / ");
                if (!isnan(action.triggerValueMin)) {
                    offset += snprintf(buffer + offset, bufferSize - offset, "Сброс %.2f\n", (double)action.triggerValueMin);
                } else {
                    offset += snprintf(buffer + offset, bufferSize - offset, "Сброс N/A\n");
                }

                char actionTypes[100];
                size_t actionOffset = 0;
                actionTypes[0] = '\0';
                if (action.collectionSettings.get(0)) {
                    actionOffset += snprintf(actionTypes + actionOffset, sizeof(actionTypes) - actionOffset, "Таймеры ");
                }
                if (action.collectionSettings.get(1)) {
                    actionOffset += snprintf(actionTypes + actionOffset, sizeof(actionTypes) - actionOffset, "Реле ");
                }
                if (action.collectionSettings.get(2)) {
                    actionOffset += snprintf(actionTypes + actionOffset, sizeof(actionTypes) - actionOffset, "Температура ");
                }
                if (action.collectionSettings.get(3)) {
                    actionOffset += snprintf(actionTypes + actionOffset, sizeof(actionTypes) - actionOffset, "Сообщение");
                }
                offset += snprintf(buffer + offset, bufferSize - offset, "    - Тип действия: %s\n", actionTypes);

                if (action.collectionSettings.get(3) && action.sendMsg.length() > 0) {

                    char msgBuffer[MAX_TXT_DESCRIPTION_LENGTH];
                    strncpy_safe(msgBuffer, action.sendMsg.c_str(), sizeof(msgBuffer));
                    offset += snprintf(buffer + offset, bufferSize - offset, "    - Сообщение: %s\n", msgBuffer);

                }

                if (!action.collectionSettings.get(3)) {
                    offset += snprintf(buffer + offset, bufferSize - offset,
                                       "    - Возврат: %s\n", action.isReturnSetting ? "Включен" : "Выключен");
                }

                offset += snprintf(buffer + offset, bufferSize - offset, "\n");
            }
            offset += snprintf(buffer + offset, bufferSize - offset, "\n");
        }
    }

    if (device.isTimersEnabled && !device.timers.empty()) {
        offset += snprintf(buffer + offset, bufferSize - offset, "⏱️ **Таймеры:**\n");
        offset += snprintf(buffer + offset, bufferSize - offset,
                           "  • Общий статус: %s\n", device.isTimersEnabled ? "Активен" : "Неактивен");
        offset += snprintf(buffer + offset, bufferSize - offset,
                           "  • Циклическое выполнение: %s\n\n", device.isEncyclateTimers ? "Включено" : "Выключено");

        for (size_t i = 0; i < device.timers.size(); ++i) {
            const Timer& timer = device.timers[i];
            if (!timer.isUseSetting) continue;

            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  • Таймер #%d: %s\n", i + 1, timer.time);

            if (timer.progress.isRunning) {
                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  - Статус: Выполняется\n");
                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  - Прошло: %d сек\n", timer.progress.elapsedTime / 1000);
                offset += snprintf(buffer + offset, bufferSize - offset,
                                   "  - Осталось: %d сек\n", timer.progress.remainingTime / 1000);
            } else if (timer.progress.isStopped) {
                offset += snprintf(buffer + offset, bufferSize - offset, "  - Статус: Остановлен\n");
            } else {
                offset += snprintf(buffer + offset, bufferSize - offset, "  - Статус: Ожидает\n");
            }

            offset += snprintf(buffer + offset, bufferSize - offset,
                               "  - Действие: %s%s%s\n",
                               timer.collectionSettings.get(0) ? "Реле " : "",
                               timer.collectionSettings.get(1) ? "Температура " : "",
                               timer.collectionSettings.get(2) ? "Без действия " : "");

            offset += snprintf(buffer + offset, bufferSize - offset, "\n");
        }
        offset += snprintf(buffer + offset, bufferSize - offset, "\n");
    }
    return offset;
}

  String DeviceManager::getActiveDaysString(const BitArray7& week) {
    String result;
    const char* dayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    for (int i = 0; i < 7; i++) {
      if (week.get(i)) {
        if (result.length() > 0) result += ",";
        result += dayNames[i];
      }
    }
    return result;
  }

  String DeviceManager::getActiveMonthsString(const BitArray12& months) {
    String result;
    const char* monthNames[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++) {
      if (months.get(i)) {
        if (result.length() > 0) result += ",";
        result += monthNames[i];
      }
    }
    return result;
  }

void DeviceManager::printDevices(const std::vector<Device>& devices) {
  if (devices.empty()) {
    Serial.println("Устройства не найдены.");
    return;
  }

  bool foundSelected = false;

  for (size_t i = 0; i < devices.size(); ++i) {
    const auto& device = devices[i];
    Serial.println("Устройство #" + String(i));
    Serial.println("  Имя: " + String(device.nameDevice));
    Serial.println("  Выбрано: " + String(device.isSelected ? "Да" : "Нет"));
    Serial.println("  Количество реле: " + String(device.relays.size()));
    Serial.println("  Количество PID: " + String(device.pids.size()));
    Serial.println("  Количество таймеров: " + String(device.timers.size()));

    Serial.println("-----------------------------");

    if (device.isSelected) {
      Serial.println("Выбранное устройство имеет индекс: " + String(i));
      foundSelected = true;
    }
  }

  if (!foundSelected) {
    Serial.println("Нет выбранного устройства.");
  }
}

uint32_t DeviceManager::calculateDeviceFlagsChecksum() {
  if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
    return 0;
  }
  const Device& device = myDevices[currentDeviceIndex];
  uint32_t hash = 5381;

  auto addToHash = [&hash](const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash = ((hash << 5) + hash) + bytes[i];
    }
  };

  addToHash(device.nameDevice, strlen(device.nameDevice));
  addToHash(&device.isSelected, sizeof(device.isSelected));
  addToHash(&device.isTimersEnabled, sizeof(device.isTimersEnabled));
  addToHash(&device.isEncyclateTimers, sizeof(device.isEncyclateTimers));
  addToHash(&device.isScheduleEnabled, sizeof(device.isScheduleEnabled));
  addToHash(&device.isActionEnabled, sizeof(device.isActionEnabled));
  addToHash(&device.temperature.isUseSetting, sizeof(device.temperature.isUseSetting));

  return hash;
}

uint32_t DeviceManager::calculateOutputRelayChecksum() {
  Device& device = myDevices[currentDeviceIndex];
  uint32_t hash = 5381;

  auto addToHash = [&hash](const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash = ((hash << 5) + hash) + bytes[i];
    }
  };

  for (const auto& relay : device.relays) {

    if (relay.isOutput) {

      addToHash(&relay.id, sizeof(relay.id));
      addToHash(&relay.pin, sizeof(relay.pin));
      addToHash(&relay.isOutput, sizeof(relay.isOutput));
      addToHash(&relay.isPwm, sizeof(relay.isPwm));
      addToHash(&relay.pwm, sizeof(relay.pwm));
      addToHash(&relay.manualMode, sizeof(relay.manualMode));
      addToHash(relay.description, strnlen(relay.description, MAX_DESCRIPTION_LENGTH));

      bool state = relay.statePin;
      addToHash(&state, sizeof(state));
    }
  }
  return hash;
}

uint32_t DeviceManager::calculateSensorValuesChecksum() {
  Device& device = myDevices[currentDeviceIndex];
  uint32_t hash = 5381;

  auto addToHash = [&hash](const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash = ((hash << 5) + hash) + bytes[i];
    }
  };

  for (const auto& sensor : device.sensors) {

    if (sensor.isUseSetting) {

      addToHash(&sensor.currentValue, sizeof(sensor.currentValue));

      addToHash(&sensor.humidityValue, sizeof(sensor.humidityValue));
    }
  }
  return hash;
}

uint32_t DeviceManager::calculateTimersProgressChecksum() {
  if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
    return 0;
  }
  const Device& device = myDevices[currentDeviceIndex];
  uint32_t hash = 5381;

  auto addToHash = [&hash](const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash = ((hash << 5) + hash) + bytes[i];
    }
  };

  for (const auto& timer : device.timers) {
    if (timer.isUseSetting) {

      addToHash(&timer.progress.elapsedTime, sizeof(timer.progress.elapsedTime));
      addToHash(&timer.progress.remainingTime, sizeof(timer.progress.remainingTime));
      addToHash(&timer.progress.isRunning, sizeof(timer.progress.isRunning));
      addToHash(&timer.progress.isStopped, sizeof(timer.progress.isStopped));
    }
  }
  return hash;
}

bool DeviceManager::handleRelayCommand(const JsonObject& command, uint32_t clientNum) {

  if (myDevices.empty()) {
    Serial.println("[DeviceManager] Error: No devices configured.");
    return false;
  }

  Serial.println("[DeviceManager] Received relay command.");

  Device& device = myDevices[currentDeviceIndex];

  int relayId = command["relay"];
  const char* action = command["action"];

  if (!action) {
    Serial.println("[DeviceManager] Error: 'action' field missing in command.");
    return false;
  }

  if (strcmp(action, "reset_all") == 0) {
    Serial.println("[DeviceManager] Executing 'reset_all' command for all relays.");
    bool anyRelayFound = false;
    for (auto& relay : device.relays) {
      if (relay.isOutput) {
        relay.statePin = false;
        relay.manualMode = false;
        anyRelayFound = true;
      }
    }
    return anyRelayFound;
  }

  if (command["relay"].isNull()) {
    Serial.println("[DeviceManager] Error: 'relay' field (ID) missing for this command.");
    return false;
  }

  bool found = false;

  Serial.printf("[DeviceManager] Searching for relay with ID: %d\n", relayId);
  for (auto& relay : device.relays) {
    if (relay.id == relayId) {

      found = true;
      Serial.printf("[DeviceManager] Found relay '%s' (ID: %d). Executing command '%s'\n", relay.description, relay.id, action);

      if (strcmp(action, "reset") == 0) {
        relay.manualMode = false;
        Serial.printf("[DeviceManager] Relay ID %d set to Auto mode.\n", relay.id);
      }

      else if (strcmp(action, "on") == 0 || strcmp(action, "off") == 0) {
        bool newState = (strcmp(action, "on") == 0);

        if (relay.statePin != newState || !relay.manualMode) {
          relay.statePin = newState;
          relay.manualMode = true;

          Serial.printf("[DeviceManager] Relay ID %d state set to '%s' and mode to Manual.\n", relay.id,  relay.statePin ? "ON" : "OFF");
        } else {
          Serial.printf("[DeviceManager] Relay ID %d already in desired state '%s'. No action taken.\n", relay.id, newState ? "ON" : "OFF");
        }
      }
      else {
        Serial.printf("[DeviceManager] Error: Unknown action '%s' for relay ID %d.\n", action, relayId);
        found = false;
      }
      break;
    }
  }

  if (!found) {
    Serial.printf("[DeviceManager] Error: Relay with ID %d not found.\n", relayId);
  }

  return found;
}

void DeviceManager::serializeRelaysForControlTab(JsonObject& target) {
    if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
        target["rel"] = JsonArray();
        return;
    }

    const Device& device = myDevices[currentDeviceIndex];
    JsonArray rel = target.createNestedArray("rel");

    for (const auto& relay : device.relays) {
        if (relay.isOutput) {
            JsonObject relayObj = rel.createNestedObject();
            relayObj["id"] = relay.id;
            relayObj["dsc"] = relay.description;
            relayObj["stp"] = relay.statePin;
            relayObj["man"] = relay.manualMode;
        }
    }
}

void DeviceManager::serializeSensorValues(JsonObject& target) {
    if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
        target["sen"] = JsonArray();
        return;
    }

    const Device& device = myDevices[currentDeviceIndex];
    JsonArray sen = target.createNestedArray("sen");

    for (const auto& sensor : device.sensors) {
        if (sensor.isUseSetting) {
            JsonObject sensorObj = sen.createNestedObject();
            sensorObj["id"] = sensor.sensorId;
            sensorObj["cv"] = sensor.currentValue;
            sensorObj["hv"] = sensor.humidityValue;
        }
    }
}

void DeviceManager::serializeTimersProgress(JsonObject& target) {
    if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
        target["tmr"] = JsonArray();
        return;
    }

    const Device& device = myDevices[currentDeviceIndex];
    JsonArray tmr = target.createNestedArray("tmr");

    for (size_t i = 0; i < device.timers.size(); ++i) {
        const Timer& timer = device.timers[i];
        const TimerInfo& progress = timer.progress;

        JsonObject timerObj = tmr.createNestedObject();
        timerObj["i"] = i;
        timerObj["e"] = timer.isUseSetting;
        timerObj["et"] = progress.elapsedTime;
        timerObj["rt"] = progress.remainingTime;
        timerObj["r"] = progress.isRunning;
        timerObj["s"] = progress.isStopped;

        tmr.add(timerObj);
    }
}

void DeviceManager::serializeDeviceFlags(JsonObject& target) {
    if (myDevices.empty() || currentDeviceIndex >= myDevices.size()) {
        return;
    }

    const Device& device = myDevices[currentDeviceIndex];

    target["nmd"] = device.nameDevice;
    target["sel"] = device.isSelected;
    target["ite"] = device.isTimersEnabled;
    target["iet"] = device.isEncyclateTimers;
    target["ise"] = device.isScheduleEnabled;
    target["iae"] = device.isActionEnabled;
    target["tmp_use"] = device.temperature.isUseSetting;
}

void DeviceManager::showMemoryInfo() {
#ifdef ESP8266
  Serial.printf("Free Heap: %u\n", ESP.getFreeHeap());
  Serial.printf("Max Free Block: %u\n", ESP.getMaxFreeBlockSize());
  Serial.printf("Heap Fragmentation: %u%%\n", ESP.getHeapFragmentation());
#elif defined(ESP32)
  Serial.printf("Free Heap: %u\n", ESP.getFreeHeap());
  Serial.printf("Max Free Block: %u\n", ESP.getMaxAllocHeap());
  Serial.printf("Min Free Heap: %u\n", ESP.getMinFreeHeap());
#endif
}
