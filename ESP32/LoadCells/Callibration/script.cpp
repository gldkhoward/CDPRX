#include <WiFi.h>
#include <ArduinoJson.h>
#include "HX711.h"
#include "credentials.h"

const int NUM_SENSORS = 4;
const int DOUT_PINS[] = {21, 22, 23, 24};
const int SCK_PINS[] = {19, 18, 17, 16};

HX711 scales[NUM_SENSORS];
bool calibrationMode = false;
int currentSensor = 0;
float knownWeight = 0.0;

// Store multiple readings for each weight
const int SAMPLES_PER_WEIGHT = 50;
long rawReadings[SAMPLES_PER_WEIGHT];
int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize scales
  for(int i = 0; i < NUM_SENSORS; i++) {
    scales[i].begin(DOUT_PINS[i], SCK_PINS[i]);
    scales[i].set_scale(1.0); // Start with default scale
    scales[i].tare();  // Reset to 0
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  if (calibrationMode && scales[currentSensor].is_ready()) {
    if (sampleCount < SAMPLES_PER_WEIGHT) {
      rawReadings[sampleCount] = scales[currentSensor].get_value(1);
      sampleCount++;
      
      if (sampleCount == SAMPLES_PER_WEIGHT) {
        sendReadings();
      }
    }
  }
}

void processCommand(String command) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.println("{\"error\": \"Invalid command format\"}");
    return;
  }

  String action = doc["action"];
  
  if (action == "start_calibration") {
    currentSensor = doc["sensor"];
    knownWeight = doc["weight"];
    sampleCount = 0;
    calibrationMode = true;
    Serial.println("{\"status\": \"starting_calibration\"}");
  }
  else if (action == "tare") {
    int sensor = doc["sensor"];
    scales[sensor].tare();
    Serial.println("{\"status\": \"tared\", \"sensor\": " + String(sensor) + "}");
  }
}

void sendReadings() {
  StaticJsonDocument<1024> doc;
  doc["sensor"] = currentSensor;
  doc["weight"] = knownWeight;
  
  JsonArray readings = doc.createNestedArray("readings");
  for(int i = 0; i < SAMPLES_PER_WEIGHT; i++) {
    readings.add(rawReadings[i]);
  }
  
  serializeJson(doc, Serial);
  Serial.println();
  
  calibrationMode = false;
}