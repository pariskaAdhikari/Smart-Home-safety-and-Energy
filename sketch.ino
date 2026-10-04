#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ============================================================
// WIFI CONFIGURATION
// ============================================================

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// ============================================================
// ADAFRUIT IO CONFIGURATION
// ============================================================

//login to  io.adafruit.com and get your credentials 

const char* IO_USERNAME = "YOUR_ADAFRUIT_USERNAME";
const char* IO_KEY      = "YOUR_ADAFRUIT_IO_KEY";

// Adafruit IO MQTT server
const char* MQTT_SERVER = "io.adafruit.com";
const int   MQTT_PORT   = 1883;

// ============================================================
// MQTT TOPICS
// ============================================================

// Sensor feeds
String TOPIC_TEMPERATURE = String(IO_USERNAME) + "/feeds/smart-home-temperature";
String TOPIC_HUMIDITY    = String(IO_USERNAME) + "/feeds/smart-home-humidity";
String TOPIC_GAS         = String(IO_USERNAME) + "/feeds/smart-home-gas";
String TOPIC_MOTION      = String(IO_USERNAME) + "/feeds/smart-home-motion";
String TOPIC_LIGHT       = String(IO_USERNAME) + "/feeds/smart-home-light";

// System feeds
String TOPIC_MODE        = String(IO_USERNAME) + "/feeds/smart-home-mode";
String TOPIC_STATUS      = String(IO_USERNAME) + "/feeds/smart-home-status";
String TOPIC_ALERT       = String(IO_USERNAME) + "/feeds/smart-home-alert";
String TOPIC_EVENT       = String(IO_USERNAME) + "/feeds/smart-home-event";

// --- Person 2 addition: Home Safety Index feeds ---
String TOPIC_HSI          = String(IO_USERNAME) + "/feeds/smart-home-hsi";
String TOPIC_RISK_LEVEL   = String(IO_USERNAME) + "/feeds/smart-home-risklevel";

// Remote control feed
String TOPIC_COMMAND     = String(IO_USERNAME) + "/feeds/smart-home-command";

// ============================================================
// PIN DEFINITIONS
// ============================================================

// DHT22
#define DHT_PIN 4
#define DHT_TYPE DHT22

// MQ-2
#define MQ2_PIN 34

// PIR
#define PIR_PIN 27

// LDR
#define LDR_PIN 35

// RGB LED
#define RED_PIN 25
#define GREEN_PIN 26
#define BLUE_PIN 33

// Buzzer
#define BUZZER_PIN 14

// Servo
#define SERVO_PIN 13

// ============================================================
// THRESHOLDS
// ============================================================

// Temperature threshold for ventilation
const float TEMPERATURE_THRESHOLD = 30.0;

// Gas / smoke threshold
const int GAS_THRESHOLD = 3800;

// Light threshold
const int DARK_THRESHOLD = 1500;

// ============================================================
// HOME SAFETY INDEX (HSI) CONFIGURATION
// ============================================================
// The HSI is a single 0-100 sensor-fusion risk score, combining every
// sensor into one number instead of reacting to each in isolation. It
// drives both the risk level shown on the dashboard AND the adaptive
// sensor-polling rate below.

// HSI weights (must sum to 1.0)
const float HSI_WEIGHT_GAS      = 0.40;
const float HSI_WEIGHT_TEMP     = 0.20;
const float HSI_WEIGHT_MOTION   = 0.30;
const float HSI_WEIGHT_HUMIDITY = 0.10;

// Reference points used to scale each raw sensor reading to a 0-100 sub-score
const float HSI_TEMP_NORMAL_C   = 22.0;   // comfort baseline
const float HSI_HUMIDITY_NORMAL = 50.0;   // comfort baseline (%)

// HSI alert-level bands (0-100 scale)
const int HSI_CAUTION_MIN   = 31;
const int HSI_WARNING_MIN   = 61;
const int HSI_EMERGENCY_MIN = 86;

enum AlertLevel
{
  ALERT_SAFE,
  ALERT_CAUTION,
  ALERT_WARNING,
  ALERT_EMERGENCY
};

int homeSafetyIndex = 0;
AlertLevel currentAlertLevel = ALERT_SAFE;

// ============================================================
// ADAPTIVE SENSOR TIMING CONFIGURATION
// ============================================================
// Sampling speeds up as risk rises, so the system reacts faster exactly
// when it matters, and idles gently (saving power/bandwidth) when safe.
// A floor is kept on the cloud-publish side to stay within Adafruit IO's
// free-tier rate limit (30 data points/minute = 1 every 2s).

const unsigned long SENSOR_INTERVAL_SAFE_MS      = 5000;
const unsigned long SENSOR_INTERVAL_CAUTION_MS   = 2000;
const unsigned long SENSOR_INTERVAL_WARNING_MS   = 1000;
const unsigned long SENSOR_INTERVAL_EMERGENCY_MS = 500;

const unsigned long CLOUD_INTERVAL_SAFE_MS      = 8000;
const unsigned long CLOUD_INTERVAL_CAUTION_MS   = 5000;
const unsigned long CLOUD_INTERVAL_WARNING_MS   = 2500;
const unsigned long CLOUD_INTERVAL_EMERGENCY_MS = 2000; // floor: respects Adafruit IO's rate limit

// ============================================================
// OBJECTS
// ============================================================

DHT dht(DHT_PIN, DHT_TYPE);

Servo ventilationServo;

WiFiClient espClient;

PubSubClient mqttClient(espClient);

// ============================================================
// SENSOR VARIABLES
// ============================================================

float temperature = 0.0;
float humidity = 0.0;

int gasValue = 0;
int lightValue = 0;

bool motionDetected = false;

// ============================================================
// SYSTEM MODE
// ============================================================

enum SystemMode
{
  HOME_MODE,
  NIGHT_MODE,
  AWAY_MODE
};

SystemMode currentMode = HOME_MODE;

// ============================================================
// SYSTEM STATE VARIABLES
// ============================================================

bool gasAlertActive = false;
bool ventilationActive = false;
bool securityAlertActive = false;
bool lightActive = false;

// ============================================================
// TIMING
// ============================================================

unsigned long lastSensorRead = 0;
unsigned long lastCloudPublish = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastMQTTCheck = 0;

// updates them every cycle based on the current AlertLevel (see
// updateAdaptiveIntervals() below).
unsigned long SENSOR_INTERVAL = SENSOR_INTERVAL_SAFE_MS;
unsigned long CLOUD_INTERVAL = CLOUD_INTERVAL_SAFE_MS;

const unsigned long WIFI_CHECK_INTERVAL = 10000;
const unsigned long MQTT_CHECK_INTERVAL = 5000;

// ============================================================
// RGB LED CONTROL
// ============================================================

void setRGB(bool red, bool green, bool blue)
{
  digitalWrite(RED_PIN, red ? HIGH : LOW);
  digitalWrite(GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(BLUE_PIN, blue ? HIGH : LOW);
}

// ============================================================
// RGB COLOUR FUNCTIONS
// ============================================================

void rgbOff()
{
  setRGB(false, false, false);
  lightActive = false;
}

void rgbWhite()
{
  setRGB(true, true, true);
  lightActive = true;
}

void rgbRed()
{
  setRGB(true, false, false);
  lightActive = true;
}

void rgbGreen()
{
  setRGB(false, true, false);
  lightActive = true;
}

void rgbBlue()
{
  setRGB(false, false, true);
  lightActive = true;
}

//  yellow, for the CAUTION ambient status colour
void rgbYellow()
{
  setRGB(true, true, false);
  lightActive = true;
}

// ============================================================
// MODE NAME
// ============================================================

String getModeName()
{
  switch (currentMode)
  {
    case HOME_MODE:
      return "HOME";

    case NIGHT_MODE:
      return "NIGHT";

    case AWAY_MODE:
      return "AWAY";
  }

  return "UNKNOWN";
}

// alert level as text, for logging/publishing 
String getAlertLevelName()
{
  switch (currentAlertLevel)
  {
    case ALERT_SAFE:
      return "SAFE";

    case ALERT_CAUTION:
      return "CAUTION";

    case ALERT_WARNING:
      return "WARNING";

    case ALERT_EMERGENCY:
      return "EMERGENCY";
  }

  return "UNKNOWN";
}

// ============================================================
// SET SYSTEM MODE
// ============================================================

void setSystemMode(SystemMode newMode)
{
  currentMode = newMode;

  Serial.println();
  Serial.println("======================================");
  Serial.print("SYSTEM MODE CHANGED TO: ");
  Serial.println(getModeName());
  Serial.println("======================================");

  // Reset security state when changing mode
  securityAlertActive = false;

  // Publish mode to cloud
  if (mqttClient.connected())
  {
    mqttClient.publish(
      TOPIC_MODE.c_str(),
      getModeName().c_str(),
      true
    );
  }

  String eventMessage = "System mode changed to " + getModeName();

  if (mqttClient.connected())
  {
    mqttClient.publish(
      TOPIC_EVENT.c_str(),
      eventMessage.c_str()
    );
  }
}

// ============================================================
// WIFI CONNECTION
// ============================================================

void connectToWiFi()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("CONNECTING TO WI-FI");
  Serial.println("======================================");

  WiFi.mode(WIFI_STA);

  // Channel 6 is used for Wokwi Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);

    Serial.print(".");

    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WI-FI CONNECTED!");
    Serial.print("IP ADDRESS: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WI-FI CONNECTION FAILED.");
  }
}

// ============================================================
// MQTT CALLBACK
// ============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String message = "";

  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  message.trim();
  message.toUpperCase();

  Serial.println();
  Serial.println("======================================");
  Serial.println("MQTT COMMAND RECEIVED");
  Serial.print("TOPIC: ");
  Serial.println(topic);
  Serial.print("MESSAGE: ");
  Serial.println(message);
  Serial.println("======================================");

  // ----------------------------------------------------------
  // Remote mode control
  // ----------------------------------------------------------

  if (String(topic) == TOPIC_COMMAND)
  {
    if (message == "HOME")
    {
      setSystemMode(HOME_MODE);
    }
    else if (message == "NIGHT")
    {
      setSystemMode(NIGHT_MODE);
    }
    else if (message == "AWAY")
    {
      setSystemMode(AWAY_MODE);
    }
    else
    {
      Serial.println("UNKNOWN MQTT COMMAND.");
      Serial.println("Use: HOME, NIGHT or AWAY.");
    }
  }
}

// ============================================================
// MQTT CONNECTION
// ============================================================

void connectToMQTT()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  if (mqttClient.connected())
  {
    return;
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println("CONNECTING TO ADAFRUIT IO MQTT");
  Serial.println("======================================");

  String clientID = "ESP32-SmartHome-" + String(random(0xffff), HEX);

  Serial.print("MQTT CLIENT ID: ");
  Serial.println(clientID);

  if (mqttClient.connect(
        clientID.c_str(),
        IO_USERNAME,
        IO_KEY
      ))
  {
    Serial.println("MQTT CONNECTED!");

    // Subscribe to remote command feed
    if (mqttClient.subscribe(TOPIC_COMMAND.c_str()))
    {
      Serial.println("MQTT COMMAND SUBSCRIPTION SUCCESSFUL.");
    }
    else
    {
      Serial.println("MQTT COMMAND SUBSCRIPTION FAILED.");
    }

    // Publish current mode
    mqttClient.publish(
      TOPIC_MODE.c_str(),
      getModeName().c_str(),
      true
    );

    mqttClient.publish(
      TOPIC_STATUS.c_str(),
      "ONLINE",
      true
    );

    mqttClient.publish(
      TOPIC_EVENT.c_str(),
      "ESP32 Smart Home System connected to MQTT",
      false
    );
  }
  else
  {
    Serial.print("MQTT CONNECTION FAILED. STATE = ");
    Serial.println(mqttClient.state());
  }
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  float newTemperature = dht.readTemperature();
  float newHumidity = dht.readHumidity();

  if (!isnan(newTemperature))
  {
    temperature = newTemperature;
  }

  if (!isnan(newHumidity))
  {
    humidity = newHumidity;
  }

  gasValue = analogRead(MQ2_PIN);

  lightValue = analogRead(LDR_PIN);

  motionDetected = digitalRead(PIR_PIN) == HIGH;
}

// ============================================================
// HOME SAFETY INDEX (HSI) CALCULATION
// ============================================================
// Combines gas, temperature, motion and humidity into one weighted 0-100
// risk score. Motion is mode-aware: the same PIR trigger means something
// different in Home vs Night vs Away mode, which is the sensor-fusion /
// edge-intelligence element the project brief asks for.

int calculateHomeSafetyIndex()
{
  // Gas sub-score: scale the raw MQ-2 reading (0-4095) against the
  // alert threshold, so it approaches 100 as gas nears GAS_THRESHOLD.
  float gasScore = constrain(
    ((float)gasValue / GAS_THRESHOLD) * 100.0,
    0,
    100
  );

  // Temperature sub-score: how far above the comfort baseline we are,
  // scaled so it reaches 100 at TEMPERATURE_THRESHOLD.
  float tempDelta = temperature - HSI_TEMP_NORMAL_C;
  float tempRange = TEMPERATURE_THRESHOLD - HSI_TEMP_NORMAL_C;
  float tempScore = constrain(
    (tempDelta / tempRange) * 100.0,
    0,
    100
  );

  // Motion sub-score: mode-aware, same logic as the Away/Night security
  // rules above, but expressed as a continuous risk contribution.
  float motionScore = 0;

  if (motionDetected)
  {
    if (currentMode == AWAY_MODE)
    {
      motionScore = 100; // any motion while away is fully unexpected
    }
    else if (currentMode == NIGHT_MODE)
    {
      motionScore = 70;  // protected-entry-style risk
    }
    else
    {
      motionScore = 10;  // Home mode: normal household activity
    }
  }

  // Humidity sub-score: extremes in either direction raise risk
  // (very dry raises fire risk, very humid indicates possible leaks).
  float humidityScore = constrain(
    abs(humidity - HSI_HUMIDITY_NORMAL) * 2.0,
    0,
    100
  );

  float hsi =
    (gasScore * HSI_WEIGHT_GAS) +
    (tempScore * HSI_WEIGHT_TEMP) +
    (motionScore * HSI_WEIGHT_MOTION) +
    (humidityScore * HSI_WEIGHT_HUMIDITY);

  return (int) constrain(hsi, 0, 100);
}

AlertLevel hsiToAlertLevel(int hsi)
{
  if (hsi >= HSI_EMERGENCY_MIN)
  {
    return ALERT_EMERGENCY;
  }

  if (hsi >= HSI_WARNING_MIN)
  {
    return ALERT_WARNING;
  }

  if (hsi >= HSI_CAUTION_MIN)
  {
    return ALERT_CAUTION;
  }

  return ALERT_SAFE;
}

// Re-calculates the HSI and alert level from the latest sensor readings.
// Called once per sensor-read cycle, before the automation rules run, so
// the rules, the adaptive interval and the cloud publish all see the same
// up-to-date risk level.
void updateHomeSafetyIndex()
{
  homeSafetyIndex = calculateHomeSafetyIndex();

  AlertLevel previousLevel = currentAlertLevel;
  currentAlertLevel = hsiToAlertLevel(homeSafetyIndex);

  if (currentAlertLevel != previousLevel)
  {
    Serial.println();
    Serial.println("======================================");
    Serial.print("HOME SAFETY INDEX LEVEL CHANGED: ");
    Serial.print(getAlertLevelName());
    Serial.print(" (HSI = ");
    Serial.print(homeSafetyIndex);
    Serial.println(")");
    Serial.println("======================================");

    publishEvent(
      "Home Safety Index level changed to " + getAlertLevelName() +
      " (HSI=" + String(homeSafetyIndex) + ")"
    );
  }
}

// ============================================================
// ADAPTIVE SENSOR TIMING
// ============================================================
// Speeds up both the sensor-read cycle and the cloud-publish cycle as
// the Home Safety Index rises, so the system reacts faster exactly when
// risk is high, and idles gently to save power/bandwidth when it's safe.

void updateAdaptiveIntervals()
{
  switch (currentAlertLevel)
  {
    case ALERT_EMERGENCY:
      SENSOR_INTERVAL = SENSOR_INTERVAL_EMERGENCY_MS;
      CLOUD_INTERVAL   = CLOUD_INTERVAL_EMERGENCY_MS;
      break;

    case ALERT_WARNING:
      SENSOR_INTERVAL = SENSOR_INTERVAL_WARNING_MS;
      CLOUD_INTERVAL   = CLOUD_INTERVAL_WARNING_MS;
      break;

    case ALERT_CAUTION:
      SENSOR_INTERVAL = SENSOR_INTERVAL_CAUTION_MS;
      CLOUD_INTERVAL   = CLOUD_INTERVAL_CAUTION_MS;
      break;

    default:
      SENSOR_INTERVAL = SENSOR_INTERVAL_SAFE_MS;
      CLOUD_INTERVAL   = CLOUD_INTERVAL_SAFE_MS;
      break;
  }
}

// ============================================================
//  AMBIENT STATUS COLOUR
// ============================================================
// When no rule has a more urgent, specific colour to show (gas alert,
// security alert, or Rule 1's smart lighting), the RGB LED instead shows
// the current Home Safety Index band - green/yellow/red - so the LED is
// always meaningful rather than just off.

void applyAmbientStatusColor()
{
  if (gasAlertActive || securityAlertActive || lightActive)
  {
    // A higher-priority rule already has control of the LED this cycle.
    return;
  }

  switch (currentAlertLevel)
  {
    case ALERT_SAFE:
      rgbGreen();
      break;

    case ALERT_CAUTION:
      rgbYellow();
      break;

    case ALERT_WARNING:
    case ALERT_EMERGENCY:
      rgbRed();
      break;
  }
}

// ============================================================
// PRINT SENSOR DATA
// ============================================================

void printSensorData()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("REAL-TIME SENSOR DATA");
  Serial.println("======================================");

  Serial.print("Temperature : ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  Serial.print("Humidity    : ");
  Serial.print(humidity, 2);
  Serial.println(" %");

  Serial.print("Gas / Smoke : ");
  Serial.println(gasValue);

  Serial.print("Motion      : ");
  if (motionDetected)
  {
    Serial.println("MOTION DETECTED");
  }
  else
  {
    Serial.println("NO MOTION");
  }

  Serial.print("Light Level : ");
  Serial.println(lightValue);

  Serial.print("System Mode : ");
  Serial.println(getModeName());

  // --- Person 2 addition ---
  Serial.print("Home Safety Index : ");
  Serial.print(homeSafetyIndex);
  Serial.print(" (");
  Serial.print(getAlertLevelName());
  Serial.println(")");

  Serial.print("Sensor interval   : ");
  Serial.print(SENSOR_INTERVAL);
  Serial.println(" ms");
  // --- end addition ---

  Serial.print("Wi-Fi       : ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("CONNECTED");
  }
  else
  {
    Serial.println("DISCONNECTED");
  }

  Serial.print("MQTT        : ");

  if (mqttClient.connected())
  {
    Serial.println("CONNECTED");
  }
  else
  {
    Serial.println("DISCONNECTED");
  }

  Serial.println("======================================");
}

// ============================================================
// CLOUD PUBLISH FUNCTION
// ============================================================

void publishSensorData()
{
  if (!mqttClient.connected())
  {
    return;
  }

  // ----------------------------------------------------------
  // Temperature
  // ----------------------------------------------------------

  char temperatureBuffer[20];

  dtostrf(
    temperature,
    1,
    2,
    temperatureBuffer
  );

  mqttClient.publish(
    TOPIC_TEMPERATURE.c_str(),
    temperatureBuffer,
    true
  );

  // ----------------------------------------------------------
  // Humidity
  // ----------------------------------------------------------

  char humidityBuffer[20];

  dtostrf(
    humidity,
    1,
    2,
    humidityBuffer
  );

  mqttClient.publish(
    TOPIC_HUMIDITY.c_str(),
    humidityBuffer,
    true
  );

  // ----------------------------------------------------------
  // Gas
  // ----------------------------------------------------------

  char gasBuffer[20];

  sprintf(
    gasBuffer,
    "%d",
    gasValue
  );

  mqttClient.publish(
    TOPIC_GAS.c_str(),
    gasBuffer,
    true
  );

  // ----------------------------------------------------------
  // Motion
  // ----------------------------------------------------------

  mqttClient.publish(
    TOPIC_MOTION.c_str(),
    motionDetected ? "1" : "0",
    true
  );

  // ----------------------------------------------------------
  // Light
  // ----------------------------------------------------------

  char lightBuffer[20];

  sprintf(
    lightBuffer,
    "%d",
    lightValue
  );

  mqttClient.publish(
    TOPIC_LIGHT.c_str(),
    lightBuffer,
    true
  );

  // ----------------------------------------------------------
  //  Home Safety Index + risk level
  // ----------------------------------------------------------

  char hsiBuffer[20];

  sprintf(
    hsiBuffer,
    "%d",
    homeSafetyIndex
  );

  mqttClient.publish(
    TOPIC_HSI.c_str(),
    hsiBuffer,
    true
  );

  mqttClient.publish(
    TOPIC_RISK_LEVEL.c_str(),
    getAlertLevelName().c_str(),
    true
  );

  // ----------------------------------------------------------
  // System status
  // ----------------------------------------------------------

  String statusMessage = "Mode=" + getModeName();

  statusMessage += ", HSI=" + String(homeSafetyIndex);
  statusMessage += ", Risk=" + getAlertLevelName();

  if (gasAlertActive)
  {
    statusMessage += ", GAS_ALERT";
  }

  if (ventilationActive)
  {
    statusMessage += ", VENTILATION_ON";
  }

  if (securityAlertActive)
  {
    statusMessage += ", SECURITY_ALERT";
  }

  mqttClient.publish(
    TOPIC_STATUS.c_str(),
    statusMessage.c_str(),
    true
  );
}

// ============================================================
// CLOUD EVENT
// ============================================================

void publishEvent(String eventMessage)
{
  Serial.print("EVENT: ");
  Serial.println(eventMessage);

  if (mqttClient.connected())
  {
    mqttClient.publish(
      TOPIC_EVENT.c_str(),
      eventMessage.c_str(),
      false
    );
  }
}

// ============================================================
// CLOUD ALERT
// ============================================================

void publishAlert(String alertMessage)
{
  Serial.print("ALERT: ");
  Serial.println(alertMessage);

  if (mqttClient.connected())
  {
    mqttClient.publish(
      TOPIC_ALERT.c_str(),
      alertMessage.c_str(),
      true
    );
  }
}

// ============================================================
// RULE 1
// SMART LIGHTING
// ============================================================

void smartLightingRule()
{
  bool isDark = lightValue < DARK_THRESHOLD;

  if (
    (currentMode == HOME_MODE || currentMode == NIGHT_MODE) &&
    motionDetected &&
    isDark
  )
  {
    if (!lightActive)
    {
      rgbWhite();

      Serial.println();
      Serial.println("RULE 1 ACTIVATED");
      Serial.println("SMART LIGHTING: ON");
      Serial.println("Reason: Motion detected + dark environment");

      publishEvent(
        "Rule 1: Smart lighting activated"
      );
    }
  }
  else
  {
    if (lightActive && !gasAlertActive)
    {
      rgbOff();
    }
  }
}

// ============================================================
// RULE 2
// TEMPERATURE-BASED VENTILATION
// ============================================================
//

void temperatureVentilationRule()
{
  if (temperature > TEMPERATURE_THRESHOLD)
  {
    ventilationServo.write(90);

    if (!ventilationActive)
    {
      ventilationActive = true;

      Serial.println();
      Serial.println("RULE 2 ACTIVATED");
      Serial.println("VENTILATION: OPEN");
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      publishEvent(
        "Rule 2: Temperature high - ventilation opened"
      );
    }
  }
  else
  {
    ventilationServo.write(0);

    if (ventilationActive)
    {
      ventilationActive = false;

      Serial.println();
      Serial.println("VENTILATION: CLOSED");

      publishEvent(
        "Temperature normal - ventilation closed"
      );
    }
  }
}

// ============================================================
// RULE 3
// GAS / SMOKE SAFETY
// ============================================================

void gasSafetyRule()
{
  if (gasValue > GAS_THRESHOLD)
  {
    rgbRed();

    digitalWrite(
      BUZZER_PIN,
      HIGH
    );

    if (!gasAlertActive)
    {
      gasAlertActive = true;

      Serial.println();
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("GAS / SMOKE SAFETY ALERT!");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      Serial.print("Gas value: ");
      Serial.println(gasValue);

      publishAlert(
        "CRITICAL: Gas/Smoke threshold exceeded"
      );

      publishEvent(
        "Rule 3: Gas safety alarm activated"
      );
    }
  }
  else
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    if (gasAlertActive)
    {
      gasAlertActive = false;

      Serial.println();
      Serial.println("Gas level returned below threshold.");

      publishAlert(
        "Gas/Smoke level returned to normal"
      );

      publishEvent(
        "Gas safety alarm cleared"
      );

      rgbOff();
    }
  }
}

// ============================================================
// RULE 4
// AWAY MODE SECURITY
// ============================================================

void awayModeSecurityRule()
{
  if (
    currentMode == AWAY_MODE &&
    motionDetected &&
    !gasAlertActive
  )
  {
    rgbRed();

    if (!securityAlertActive)
    {
      securityAlertActive = true;

      Serial.println();
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("AWAY MODE SECURITY ALERT!");
      Serial.println("MOTION DETECTED WHILE AWAY.");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      publishAlert(
        "Security Alert: Motion detected in AWAY mode"
      );

      publishEvent(
        "Away mode security event detected"
      );
    }
  }
  else
  {
    if (!motionDetected)
    {
      securityAlertActive = false;
    }
  }
}

// ============================================================
// EDGE INTELLIGENCE
// ============================================================

void runEdgeIntelligence()
{
  // refresh the Home Safety Index first, so every
  // rule below (and the adaptive interval, and the cloud publish) all
  // act on the same up-to-date risk assessment this cycle. ---
  updateHomeSafetyIndex();

  // ----------------------------------------------------------
  // Priority 1: Gas safety
  // ----------------------------------------------------------

  if (gasValue > GAS_THRESHOLD)
  {
    gasSafetyRule();

    // Gas safety has highest priority
    return;
  }
  else
  {
    gasSafetyRule();
  }

  // ----------------------------------------------------------
  // Priority 2: Away security
  // ----------------------------------------------------------

  if (currentMode == AWAY_MODE)
  {
    awayModeSecurityRule();
  }

  // ----------------------------------------------------------
  // Priority 3: Temperature ventilation
  // ----------------------------------------------------------

  temperatureVentilationRule();

  // ----------------------------------------------------------
  // Priority 4: Smart lighting
  // ----------------------------------------------------------

  if (!securityAlertActive)
  {
    smartLightingRule();
  }

  // ----------------------------------------------------------
  // Priority 5:  ambient HSI status colour,
  // only shown if no rule above claimed the LED this cycle.
  // ----------------------------------------------------------

  applyAmbientStatusColor();

  //  recalculate the adaptive sensor/cloud
  // intervals for the *next* cycle based on the risk level just found.
  updateAdaptiveIntervals();
}

// ============================================================
// SERIAL COMMAND HELP
// ============================================================

void printHelp()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("SMART HOME CONTROL COMMANDS");
  Serial.println("======================================");
  Serial.println("H -> HOME MODE");
  Serial.println("N -> NIGHT MODE");
  Serial.println("A -> AWAY MODE");
  Serial.println("S -> SHOW SENSOR DATA");
  Serial.println("? -> SHOW HELP");
  Serial.println("======================================");
}

// ============================================================
// SERIAL COMMAND PROCESSING
// ============================================================

void processSerialCommands()
{
  if (!Serial.available())
  {
    return;
  }

  char command = Serial.read();

  // Convert lowercase to uppercase
  if (command >= 'a' && command <= 'z')
  {
    command = command - 32;
  }

  switch (command)
  {
    case 'H':
      setSystemMode(HOME_MODE);
      break;

    case 'N':
      setSystemMode(NIGHT_MODE);
      break;

    case 'A':
      setSystemMode(AWAY_MODE);
      break;

    case 'S':
      printSensorData();
      break;

    case '?':
      printHelp();
      break;

    default:
      break;
  }
}

// ============================================================
// WIFI MONITOR
// ============================================================

void monitorWiFi()
{
  if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL)
  {
    return;
  }

  lastWiFiCheck = millis();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println();
    Serial.println("Wi-Fi connection lost.");

    connectToWiFi();
  }
}

// ============================================================
// MQTT MONITOR
// ============================================================

void monitorMQTT()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  if (millis() - lastMQTTCheck < MQTT_CHECK_INTERVAL)
  {
    return;
  }

  lastMQTTCheck = millis();

  if (!mqttClient.connected())
  {
    connectToMQTT();
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // ----------------------------------------------------------
  // Startup banner
  // ----------------------------------------------------------

  Serial.println();
  Serial.println();
  Serial.println("================================================");
  Serial.println("     SMART HOME SAFETY & ENERGY SYSTEM");
  Serial.println("================================================");
  Serial.println("ESP32 IoT Automation System");
  Serial.println("Wokwi Simulation");
  Serial.println("MQTT + Adafruit IO");
  Serial.println("Edge Intelligence Enabled");
  Serial.println("Home Safety Index + Adaptive Timing Enabled");
  Serial.println("================================================");

  // ----------------------------------------------------------
  // Pin configuration
  // ----------------------------------------------------------

  pinMode(
    MQ2_PIN,
    INPUT
  );

  pinMode(
    PIR_PIN,
    INPUT
  );

  pinMode(
    LDR_PIN,
    INPUT
  );

  pinMode(
    RED_PIN,
    OUTPUT
  );

  pinMode(
    GREEN_PIN,
    OUTPUT
  );

  pinMode(
    BLUE_PIN,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  // ----------------------------------------------------------
  // Initial actuator state
  // ----------------------------------------------------------

  rgbOff();

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  // ----------------------------------------------------------
  // DHT22
  // ----------------------------------------------------------

  dht.begin();

  // ----------------------------------------------------------
  // Servo
  // ----------------------------------------------------------

  ventilationServo.setPeriodHertz(50);

  ventilationServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  ventilationServo.write(0);

  // ----------------------------------------------------------
  // Initial system mode
  // ----------------------------------------------------------

  currentMode = HOME_MODE;

  // initial HSI/alert state
  homeSafetyIndex = 0;
  currentAlertLevel = ALERT_SAFE;
  updateAdaptiveIntervals();

  // ----------------------------------------------------------
  // MQTT configuration
  // ----------------------------------------------------------

  mqttClient.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );

  mqttClient.setCallback(
    mqttCallback
  );

  // ----------------------------------------------------------
  // Wi-Fi
  // ----------------------------------------------------------

  connectToWiFi();

  // ----------------------------------------------------------
  // MQTT
  // ----------------------------------------------------------

  connectToMQTT();

  // ----------------------------------------------------------
  // Help
  // ----------------------------------------------------------

  printHelp();

  Serial.println();
  Serial.println("SYSTEM INITIALISATION COMPLETE.");
  Serial.println("SMART HOME SYSTEM IS RUNNING.");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Process serial commands
  // ----------------------------------------------------------

  processSerialCommands();

  // ----------------------------------------------------------
  // Monitor Wi-Fi
  // ----------------------------------------------------------

  monitorWiFi();

  // ----------------------------------------------------------
  // Monitor MQTT
  // ----------------------------------------------------------

  monitorMQTT();

  // ----------------------------------------------------------
  // Keep MQTT connection alive
  // ----------------------------------------------------------

  if (mqttClient.connected())
  {
    mqttClient.loop();
  }

  // ----------------------------------------------------------
  // Read sensors - interval is now ADAPTIVE 
  // SENSOR_INTERVAL shrinks automatically as risk rises, evaluated at
  // the end of the previous runEdgeIntelligence() call.
  // ----------------------------------------------------------

  if (millis() - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = millis();

    // Read sensors
    readSensors();

    // Print readings
    printSensorData();

    // Run local edge intelligence (this also updates the HSI,
    // the ambient status colour, and the adaptive intervals)
    runEdgeIntelligence();
  }

  // ----------------------------------------------------------
  // Publish data to cloud - interval is now ADAPTIVE,
  //  with a floor that respects Adafruit IO's rate limit.
  // ----------------------------------------------------------

  if (
    mqttClient.connected() &&
    millis() - lastCloudPublish >= CLOUD_INTERVAL
  )
  {
    lastCloudPublish = millis();

    publishSensorData();
  }

  // Small delay
  delay(10);
}
