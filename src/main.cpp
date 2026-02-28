#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// things that need modification: 
// SETPOINT_C → your desired doghouse temp
// LAG_TEMP → bigger value = less on/off cycling
// PWM_ON_DUTY → higher/lower to change compressor speed (once I know what range works best)

// REQUIREMENTS
// R1: Read Temp
// R2: Display Temp
// R3: Power On/Off (Cooling)
// R4: Power On/Off (Heating)
// R5: Detect Fault
// R6: Display Fault

#define LED 2
#define TEMP_SENSOR_PIN 4



// ---------- PIN DEFINITIONS (match your diagram) ----------
const int ONE_WIRE_BUS = 4;   // D4 -> DS18B20 data
const int PWM_PIN      = 5;   // D5 -> PWM_IN on compressor controller
const int EN_PIN       = 19;  // D19 -> EN_IN on compressor controller
const int FAULT_PIN    = 23;  // D23 <- FAULT_OUT from controller

// ---------- CONTROL SETTINGS ----------
const float SETPOINT_C      = 60.0;   // desired temp (°F)
const float LAG_TEMP        = 2.0;    // +/- band to prevent rapid toggling
const int   PWM_CHANNEL     = 0;      // LEDC channel
const int   PWM_FREQ        = 5000;   // 5 kHz (within 1–10 kHz spec)
const int   PWM_RESOLUTION  = 8;      // 8-bit (0–255)
const int   PWM_ON_DUTY     = 200;    // ~80% duty for "AC ON"
const int   PWM_OFF_DUTY    = 0;      // 0% duty for "AC OFF"

// ---------- TEMPERATURE SENSOR SETUP ----------
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

bool acOn = false;

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  sensors.begin();
  Serial.println("Looking for DS18B20 temperature sensor...");
  if (sensors.getAddress(tempDeviceAddress, 0)) {
    Serial.print("Found device ");
    Serial.print(0, DEC);
    Serial.print(" with address: ");
    for (uint8_t j = 0; j < 8; j++) {
      if (tempDeviceAddress[j] < 16) Serial.print("0");
      Serial.print(tempDeviceAddress[j], HEX);
    }
    Serial.println();
  }
}

// Method loop - Constantly runs to control the AC based on temp readings and fault status
void loop() {

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

  float currentTemp = readTemperature();
  controlAC(currentTemp);
}

// Method readTemperature - Reads temp from DS18B20 and prints to Serial
float readTemperature() {
  sensors.requestTemperatures();

  float tempC = sensors.getTempC(tempDeviceAddress);
  Serial.println("Current Temperature: " + String(sensors.toFahrenheit(tempC)) + " °F");

  return sensors.toFahrenheit(tempC);
}

// Method controlAC - Takes current temp and decides whether to turn AC on or off based on setpoint and hysteresis
void controlAC(float temperature) {
  bool turnOn = false;

  if (temperature > SETPOINT_C + LAG_TEMP) {
    turnOn = true;
  } else if (temperature < SETPOINT_C - LAG_TEMP) {
    turnOn = false;
  }

  if (turnOn) 
  {
    digitalWrite(EN_PIN, HIGH);
    Serial.println("AC turned ON");
  } 
  else 
  {
    digitalWrite(EN_PIN, LOW);
    Serial.println("AC turned OFF");
  }
}

/*
// Method -setup- Power up and configure the ESP32
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 AC controller starting...");

  // Enable + fault pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(FAULT_PIN, INPUT);

  // Turn controller ON (enabled)
  digitalWrite(EN_PIN, HIGH);

  // PWM setup
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_OFF_DUTY);  // start OFF

  // DS18B20 init
  sensors.begin();
}

// Method -loop- Reads temp and powers on AC
void loop() {
  // ---- READ TEMPERATURE ----
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 not detected!");
    // fail safe: turn AC off
    acOn = false;
  } else {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");

    // ---- HYSTERESIS CONTROL ----
    if (tempC > SETPOINT_C + LAG_TEMP) {
      acOn = true;   // room too warm -> AC ON
    }
    if (tempC < SETPOINT_C - LAG_TEMP) {
      acOn = false;  // room cool enough -> AC OFF
    }
  }

  // ---- CHECK FAULT SIGNAL ----
  int faultState = digitalRead(FAULT_PIN);
  // Assume: LOW = normal, HIGH = fault (you can invert if needed)
  if (faultState == HIGH) {
    Serial.println("FAULT detected from compressor controller!");
    acOn = false;  // fail safe: turn off compressor
  }

  // ---- APPLY PWM OUTPUT ----
  if (acOn) {
    ledcWrite(PWM_CHANNEL, PWM_ON_DUTY);
  } else {
    ledcWrite(PWM_CHANNEL, PWM_OFF_DUTY);
  }

  // Debug
  Serial.print("AC state: ");
  Serial.println(acOn ? "ON" : "OFF");

  delay(1000);  // 1 second loop
}
  */