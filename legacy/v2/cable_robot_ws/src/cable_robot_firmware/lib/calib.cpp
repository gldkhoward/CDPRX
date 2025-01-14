#include "HX711.h"

// Pin Configuration
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 15;

// Measurement Parameters
const int READINGS_PER_MEASUREMENT = 5;     // Number of readings to average for each measurement
const int MEASUREMENTS_PER_WEIGHT = 10;     // Number of measurements to take for each weight
const int MEASUREMENT_INTERVAL_MS = 2000;   // Time between measurements in milliseconds
const int TOTAL_MEASUREMENT_TIME_MS = 20000; // Total time for all measurements per weight

// Serial Communication
const long SERIAL_BAUD_RATE = 115200;
const int SERIAL_TIMEOUT_MS = 1000;

// Data Format Parameters
const char DELIMITER = ',';
const float SUB_MEASUREMENT_INCREMENT = 0.1;  // For ID numbering (e.g., 1.1, 1.2, etc.)

// Load cell variables
HX711 scale;
float measurement_id = 1.0;  // Format: X.Y where X is weight measurement, Y is sub-measurement

// Optional scale parameters (uncomment and modify as needed)
// const float KNOWN_WEIGHT_GRAMS = 100.0;  // Weight used for calibration
// const float CALIBRATION_FACTOR = -7050;  // Calibration factor for your specific load cell
// const float ZERO_OFFSET = 0;             // Zero offset for the scale

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  
  while (!Serial) {
    delay(100);
  }
  
  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("Load Cell Testing System");
  Serial.println("Commands: 'zero' to zero the scale, 'record' to record measurements");
  
  // Optional calibration setup (uncomment if using calibration factor)
  // scale.set_scale(CALIBRATION_FACTOR);
  // scale.set_offset(ZERO_OFFSET);
  
  scale.set_scale();
  scale.tare();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "zero") {
      scale.tare();
      Serial.println("Scale zeroed");
    }
    else if (command == "record") {
      recordMeasurements();
    }
    // Optional: Add calibration command
    // else if (command == "calibrate") {
    //   calibrateScale();
    // }
  }
}

void recordMeasurements() {
  Serial.println("Enter the reference weight in grams:");
  while (!Serial.available()) {
    delay(10);
  }
  
  float reference_weight = Serial.parseFloat();
  Serial.printf("Recording %d measurements over %d seconds...\n", 
                MEASUREMENTS_PER_WEIGHT, 
                TOTAL_MEASUREMENT_TIME_MS / 1000);
  
  int whole_number = (int)measurement_id;
  float sub_measurement = SUB_MEASUREMENT_INCREMENT;
  
  for (int i = 0; i < MEASUREMENTS_PER_WEIGHT; i++) {
    float current_id = whole_number + sub_measurement;
    long value = scale.get_value(READINGS_PER_MEASUREMENT);
    
    // Print in format {ID, WEIGHT, VALUE}
    Serial.print("{");
    Serial.print(current_id, 1);
    Serial.print(DELIMITER);
    Serial.print(reference_weight);
    Serial.print(DELIMITER);
    Serial.print(value);
    Serial.println("}");
    
    sub_measurement += SUB_MEASUREMENT_INCREMENT;
    delay(MEASUREMENT_INTERVAL_MS);
  }
  
  measurement_id = whole_number + 1;
  Serial.println("Measurement set complete");
}

// Optional calibration function
/* 
void calibrateScale() {
  Serial.println("Remove all weight from scale and press any key...");
  while (!Serial.available()) delay(10);
  Serial.read();
  
  scale.tare();
  Serial.println("Place known weight on scale and press any key...");
  while (!Serial.available()) delay(10);
  Serial.read();
  
  long reading = scale.get_value(READINGS_PER_MEASUREMENT);
  float new_calibration_factor = reading / KNOWN_WEIGHT_GRAMS;
  
  Serial.print("Calibration factor: ");
  Serial.println(new_calibration_factor);
}
*/