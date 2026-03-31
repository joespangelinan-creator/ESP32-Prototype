#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "server.h"
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

// ---------- PIN DEFINITIONS (match your diagram) ----------
const int LED             = 2;   // D2 -> Onboard LED for status indication
const int TEMP_SENSOR_PIN = 4;   // D4 -> DS18B20 data
const int ONE_WIRE_BUS    = 4;   // D4 -> DS18B20 data
const int PWM_PIN         = 5;   // D5 -> PWM_IN on compressor controller
const int AC_PIN          = 19;  // D19 -> EN pin on controller (active HIGH)
const int HEAT_PIN        = 21;  // D21 -> EN pin on heating element controller (active HIGH)
const int FAULT_PIN       = 23;  // D23 <- FAULT_OUT from controller

// ---------- CONTROL SETTINGS ----------
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

// ---------- CONTROL VARIABLES ----------
bool AC_ON = false; // default state of AC (off for safety)
bool HEAT_ON = false; // default state of heating (off for safety)

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(AC_PIN, OUTPUT);
  //pinMode(HEAT_PIN, OUTPUT);
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

  generateServer();
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
  int setTemp = getSetTemperature(); 
  if (temperature > setTemp + LAG_TEMP) {
    AC_ON = true;
  } else if (temperature < setTemp - LAG_TEMP) {
    AC_ON = false;
  }

  if (AC_ON) 
  {
    digitalWrite(AC_PIN, HIGH);
    Serial.println("AC turned ON");
  } 
  else 
  {
    digitalWrite(AC_PIN, LOW);
    Serial.println("AC turned OFF");
  }
}

void controlHeating(float temperature) {
  int setTemp = getSetTemperature();
  if (temperature < setTemp - LAG_TEMP) {
    HEAT_ON = true;
  } else if (temperature > setTemp + LAG_TEMP) {
    HEAT_ON = false;
  }

  if (HEAT_ON) 
  {
    digitalWrite(HEAT_PIN, HIGH);
    Serial.println("Heating turned ON");
  } 
  else 
  {
    digitalWrite(HEAT_PIN, LOW);
    Serial.println("Heating turned OFF");
  }
}

// Method loop - Constantly runs to control the AC based on temp readings and fault status
void loop() {

  float currentTemp = readTemperature();
  manageServer(currentTemp);

  bool mode = setControls();
  int setTemp = getSetTemperature();

  if (mode == true && currentTemp > setTemp) {
    controlAC(currentTemp);
  }
  else if (mode == true && currentTemp < setTemp) {
    controlHeating(currentTemp);
  }
  else if (mode == false) {
    digitalWrite(AC_PIN, LOW); // Ensure AC is off
    digitalWrite(HEAT_PIN, LOW); // Ensure heating is off
  }
}
