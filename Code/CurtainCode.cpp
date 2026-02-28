#include <Arduino.h>
#include <NimBLEDevice.h>
#include <DHT.h>
#include <Preferences.h>
#include <time.h>
#include <cstring> // for strcmp

// CONFIG
#define DHTPIN           15
#define DHTTYPE          DHT11

#define DHT_POWER_PIN    4
#define PHOTO_POWER_PIN  16
#define PHOTO_ADC_PIN    34

#define RPWM 25
#define LPWM 26
#define R_EN 27
#define L_EN 14

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES 8
#define MOTOR_PWM_CH_R 0
#define MOTOR_PWM_CH_L 1

#define LED_PIN 2

const uint32_t MOTOR_RUN_MS = 9000UL;
const float TEMP_HIGH_C = 22.0f;
const float TEMP_LOW_C  = 18.0f;
const float PHOTO_VOLT_THRESHOLD = 2.5f;

const uint32_t SENSOR_INTERVAL_SECONDS = 600; 
const uint8_t OVERRIDE_HOURS_DEFAULT = 2;

const char* PREF_NS = "blinds";

// BLE server variables
NimBLEServer* pServer;
NimBLEAdvertising* pAdv;

// BLE callbacks
class ServerCB : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("BLE connected: %s\n", connInfo.getAddress().toString().c_str());
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("BLE disconnected -> restarting advertising\n");
        NimBLEDevice::startAdvertising();
    }
};

// TIME BLOCKS (defaults)
int MORNING_START = 6;
int MORNING_END   = 9;

int WORK_START    = 9;
int WORK_END      = 17;

int EVENING_START = 17;
int EVENING_END   = 22;

int SLEEP_START   = 22;
int SLEEP_END     = 6;

// BLE UUIDs
static BLEUUID svcUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID cmdUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

// DHT sensor and preferences
DHT dht(DHTPIN, DHTTYPE);
Preferences prefs;

volatile bool blinds_down = false;
uint64_t override_until_unix = 0;
unsigned long lastSensorMillis = 0;
uint32_t sensorIntervalMs = SENSOR_INTERVAL_SECONDS * 1000UL;

bool powerLossDetected = false;

// TIME HELPERS
uint64_t now_unix() {
  return (uint64_t)time(NULL);
}

int currentHour() {
  time_t t = time(NULL);
  struct tm tm;
  localtime_r(&t, &tm);
  return tm.tm_hour;
}

// Checks if current time is in a range (handles cross-midnight)
bool inTimeRange(int startH, int endH) {
  int h = currentHour();
  if (startH < endH) return (h >= startH && h < endH);
  return (h >= startH || h < endH);
}

bool inMorning() { return inTimeRange(MORNING_START, MORNING_END); }
bool inWork()    { return inTimeRange(WORK_START, WORK_END); }
bool inEvening() { return inTimeRange(EVENING_START, EVENING_END); }
bool inSleep()   { return inTimeRange(SLEEP_START, SLEEP_END); }

// PREFERENCES / SCHEDULE PERSISTENCE
void saveSchedule() {
  prefs.putInt("m_s", MORNING_START);
  prefs.putInt("m_e", MORNING_END);

  prefs.putInt("w_s", WORK_START);
  prefs.putInt("w_e", WORK_END);

  prefs.putInt("e_s", EVENING_START);
  prefs.putInt("e_e", EVENING_END);

  prefs.putInt("s_s", SLEEP_START);
  prefs.putInt("s_e", SLEEP_END);

  Serial.printf("Schedule saved: MOR %d-%d WORK %d-%d EVE %d-%d SLP %d-%d\n",
                MORNING_START, MORNING_END, WORK_START, WORK_END,
                EVENING_START, EVENING_END, SLEEP_START, SLEEP_END);
}

void loadSchedule() {
  MORNING_START = prefs.getInt("m_s", MORNING_START);
  MORNING_END   = prefs.getInt("m_e", MORNING_END);

  WORK_START    = prefs.getInt("w_s", WORK_START);
  WORK_END      = prefs.getInt("w_e", WORK_END);

  EVENING_START = prefs.getInt("e_s", EVENING_START);
  EVENING_END   = prefs.getInt("e_e", EVENING_END);

  SLEEP_START   = prefs.getInt("s_s", SLEEP_START);
  SLEEP_END     = prefs.getInt("s_e", SLEEP_END);

  Serial.printf("Schedule loaded: MOR %d-%d WORK %d-%d EVE %d-%d SLP %d-%d\n",
                MORNING_START, MORNING_END, WORK_START, WORK_END,
                EVENING_START, EVENING_END, SLEEP_START, SLEEP_END);
}

// STATE PERSISTENCE
void persistState() {
  prefs.putBool("down", blinds_down);
  prefs.putULong("ovr", (unsigned long)override_until_unix);
}

void loadState() {
  blinds_down = prefs.getBool("down", false);
  override_until_unix = (uint64_t)prefs.getULong("ovr", 0);
  Serial.printf("Loaded state: down=%d override=%llu\n", blinds_down, (unsigned long long)override_until_unix);
}

// MOTOR CONTROL
void motorPulseDown(bool commit = false) {
  if (blinds_down) {
    Serial.println("motorPulseDown: already down");
    return;
  }

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcWrite(MOTOR_PWM_CH_R, 0);
  ledcWrite(MOTOR_PWM_CH_L, 200);

  Serial.println("Motor DOWN start");
  delay(MOTOR_RUN_MS);

  ledcWrite(MOTOR_PWM_CH_L, 0);

  blinds_down = true;
  if (commit) persistState();

  Serial.println("Motor DOWN end");
}

void motorPulseUp(bool commit = false) {
  if (!blinds_down) {
    Serial.println("motorPulseUp: already up");
    return;
  }

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcWrite(MOTOR_PWM_CH_L, 0);
  ledcWrite(MOTOR_PWM_CH_R, 200);

  Serial.println("Motor UP start");
  delay(MOTOR_RUN_MS);

  ledcWrite(MOTOR_PWM_CH_R, 0);

  blinds_down = false;
  if (commit) persistState();

  Serial.println("Motor UP end");
}

// SENSORS
float readPhotoVoltage() {
  int raw = analogRead(PHOTO_ADC_PIN);
  return (raw / 4095.0f) * 3.3f;
}

void doSensorDecision() {
  digitalWrite(DHT_POWER_PIN, HIGH);
  digitalWrite(PHOTO_POWER_PIN, HIGH);
  delay(250);

  float t = dht.readTemperature();
  float v = readPhotoVoltage();

  digitalWrite(DHT_POWER_PIN, LOW);
  digitalWrite(PHOTO_POWER_PIN, LOW);

  bool tempBad = (!isnan(t) && (t > TEMP_HIGH_C || t < TEMP_LOW_C));
  bool lightBad = (v > PHOTO_VOLT_THRESHOLD);

  if (tempBad || lightBad) {
    if (!blinds_down) {
      Serial.println("Sensors bad -> DOWN");
      motorPulseDown(false);
    }
  } else {
    if (blinds_down) {
      Serial.println("Sensors good -> UP");
      motorPulseUp(false);
    }
  }
}

// BLE CALLBACKS
class CmdCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    cmd.toUpperCase();

    Serial.printf("BLE cmd received: '%s'\n", cmd.c_str());

    if (cmd == "DOWN") {
      motorPulseDown(true);
      override_until_unix = now_unix() + OVERRIDE_HOURS_DEFAULT * 3600ULL;
      persistState();
      return;
    }
    else if (cmd == "UP") {
      motorPulseUp(true);
      override_until_unix = now_unix() + OVERRIDE_HOURS_DEFAULT * 3600ULL;
      persistState();
      return;
    }
    else if (cmd == "AUTO") {
      override_until_unix = 0;
      persistState();
      return;
    }

    // SETTIME hh mm
    if (cmd.startsWith("SETTIME ")) {
      int hh, mm;
      if (sscanf(cmd.c_str(), "SETTIME %d %d", &hh, &mm) == 2) {
        struct tm t;
        time_t now = time(NULL);
        localtime_r(&now, &t);

        t.tm_hour = hh;
        t.tm_min  = mm;
        t.tm_sec  = 0;

        time_t newTime = mktime(&t);
        struct timeval tv = { .tv_sec = newTime, .tv_usec = 0 };
        settimeofday(&tv, NULL);

        digitalWrite(LED_PIN, LOW); // power-loss LED off
        powerLossDetected = false;

        Serial.printf("SETTIME applied: %02d:%02d\n", hh, mm);
      } else {
        Serial.println("SETTIME parse failed");
      }
      persistState();
      return;
    }

    // SET <BLOCK> startH endH
    if (cmd.startsWith("SET ")) {
      char block[16];
      int s, e;
      if (sscanf(cmd.c_str(), "SET %15s %d %d", block, &s, &e) == 3) {
        bool valid = false;
        if (s >= 0 && s <= 23 && e >= 0 && e <= 23) {
          if (strcmp(block, "MORNING") == 0) { MORNING_START = s; MORNING_END = e; valid = true; }
          else if (strcmp(block, "WORK") == 0) { WORK_START = s; WORK_END = e; valid = true; }
          else if (strcmp(block, "EVENING") == 0) { EVENING_START = s; EVENING_END = e; valid = true; }
          else if (strcmp(block, "SLEEP") == 0) { SLEEP_START = s; SLEEP_END = e; valid = true; }
          else { Serial.printf("SET: unknown block '%s'\n", block); }

          if (valid) {
            saveSchedule();
            Serial.printf("SET %s -> %d-%d\n", block, s, e);
          }
        } else {
          Serial.println("SET: invalid hours (0-23)");
        }
      } else {
        Serial.println("SET: parse failed (use: SET WORK 8 16)");
      }
      persistState();
      return;
    }

    Serial.println("Unknown BLE command");
    persistState();
  }
};

// SETUP AND LOOP
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\nESP32 Blinds Controller (4-mode)");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(DHT_POWER_PIN, OUTPUT);
  pinMode(PHOTO_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, LOW);
  digitalWrite(PHOTO_POWER_PIN, LOW);

  dht.begin();
  analogSetPinAttenuation(PHOTO_ADC_PIN, ADC_11db);

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcSetup(MOTOR_PWM_CH_R, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(RPWM, MOTOR_PWM_CH_R);

  ledcSetup(MOTOR_PWM_CH_L, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(LPWM, MOTOR_PWM_CH_L);

  ledcWrite(MOTOR_PWM_CH_R, 0);
  ledcWrite(MOTOR_PWM_CH_L, 0);

  prefs.begin(PREF_NS, false);

  if (prefs.getBool("running", false)) {
    powerLossDetected = true;
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Power loss detected");
  }
  prefs.putBool("running", true);

  loadSchedule();
  loadState();

  NimBLEDevice::init("Dual Mode Blinds");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  NimBLEService* svc = pServer->createService(svcUUID);

  NimBLECharacteristic* ch =
    svc->createCharacteristic(cmdUUID, NIMBLE_PROPERTY::WRITE);
  ch->setCallbacks(new CmdCB());

  svc->start();

  pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(svcUUID);
  pAdv->setName("Dual Mode Blinds");
  pAdv->enableScanResponse(true);
  pAdv->start();

  Serial.println("BLE advertising started as 'Dual Mode Blinds'");
  lastSensorMillis = millis() - sensorIntervalMs;
}

void loop() {
  // Manual override expiration
  if (override_until_unix && now_unix() > override_until_unix) {
    Serial.println("Manual override expired");
    override_until_unix = 0;
    persistState();
  }

  // Sensor interval check
  if ((millis() - lastSensorMillis) >= sensorIntervalMs) {
    lastSensorMillis = millis();

    if (override_until_unix == 0) {
      if (inSleep() || inWork()) {
        Serial.println("Mode: FORCED DOWN (sleep/work)");
        if (!blinds_down) motorPulseDown(false);
      } else if (inMorning() || inEvening()) {
        Serial.println("Mode: SENSOR MODE (morning/evening)");
        doSensorDecision();
      } else {
        Serial.println("Mode: Unknown -> forcing DOWN");
        if (!blinds_down) motorPulseDown(false);
      }
    } else {
      Serial.println("Manual override active -> skipping sensors");
    }
  }

  delay(200);
}
