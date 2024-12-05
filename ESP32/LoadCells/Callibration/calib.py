import serial
import json
import time
import numpy as np
import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt
from pathlib import Path

class ForceCalibrator:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200):
        self.ser = serial.Serial(port, baud_rate)
        self.calibration_data = {i: [] for i in range(4)}  # For 4 sensors
        time.sleep(2)  # Wait for Arduino to initialize
        
    def tare_sensor(self, sensor_id):
        command = {
            "action": "tare",
            "sensor": sensor_id
        }
        self.ser.write(f"{json.dumps(command)}\n".encode())
        response = self.ser.readline().decode().strip()
        return json.loads(response)
        
    def collect_readings(self, sensor_id, weight_kg):
        command = {
            "action": "start_calibration",
            "sensor": sensor_id,
            "weight": weight_kg
        }
        self.ser.write(f"{json.dumps(command)}\n".encode())
        
        # Wait for and process readings
        response = self.ser.readline().decode().strip()
        data = json.loads(response)
        return data
        
    def calibrate_sensor(self, sensor_id, weights):
        """Calibrate one sensor with multiple known weights"""
        print(f"Calibrating sensor {sensor_id}")
        
        # Tare the sensor
        self.tare_sensor(sensor_id)
        time.sleep(2)
        
        # Collect data for each weight
        for weight in weights:
            print(f"Place {weight}kg weight and press Enter...")
            input()
            
            data = self.collect_readings(sensor_id, weight)
            self.calibration_data[sensor_id].append({
                'weight_kg': weight,
                'raw_readings': data['readings']
            })
            
            print("Remove weight and press Enter...")
            input()
            time.sleep(2)
            
    def calculate_calibration_factor(self, sensor_id):
        """Calculate calibration factor using linear regression"""
        weights = []
        averages = []
        
        for data in self.calibration_data[sensor_id]:
            weights.append(data['weight_kg'])
            averages.append(np.mean(data['raw_readings']))
            
        # Force regression through zero
        calibration_factor = np.sum(np.multiply(weights, averages)) / np.sum(np.multiply(averages, averages))
        return calibration_factor
        
    def generate_report(self, sensor_id):
        """Generate calibration report with graphs"""
        factor = self.calculate_calibration_factor(sensor_id)
        
        # Create plots
        plt.figure(figsize=(12, 6))
        
        # Plot raw readings vs known weights
        weights = []
        averages = []
        std_devs = []
        
        for data in self.calibration_data[sensor_id]:
            weights.append(data['weight_kg'])
            readings = data['raw_readings']
            averages.append(np.mean(readings))
            std_devs.append(np.std(readings))
            
        plt.errorbar(weights, averages, yerr=std_devs, fmt='o', label='Measured Data')
        
        # Plot calibration line
        x_line = np.linspace(0, max(weights), 100)
        y_line = x_line * factor
        plt.plot(x_line, y_line, 'r--', label='Calibration Line')
        
        plt.xlabel('Known Weight (kg)')
        plt.ylabel('Raw Sensor Reading')
        plt.title(f'Sensor {sensor_id} Calibration Curve')
        plt.legend()
        
        # Save plot and data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_dir = Path(f"calibration_reports/sensor_{sensor_id}")
        report_dir.mkdir(parents=True, exist_ok=True)
        
        plt.savefig(report_dir / f"calibration_curve_{timestamp}.png")
        
        # Save calibration data
        report_data = {
            'timestamp': timestamp,
            'sensor_id': sensor_id,
            'calibration_factor': factor,
            'raw_data': self.calibration_data[sensor_id]
        }
        
        with open(report_dir / f"calibration_data_{timestamp}.json", 'w') as f:
            json.dump(report_data, f, indent=2)
            
        return report_data

def main():
    # Define test weights (in kg)
    test_weights = [0.5, 1.0, 2.0, 5.0]
    
    calibrator = ForceCalibrator()
    
    # Calibrate each sensor
    for sensor_id in range(4):
        print(f"\nStarting calibration for sensor {sensor_id}")
        calibrator.calibrate_sensor(sensor_id, test_weights)
        report = calibrator.generate_report(sensor_id)
        
        print(f"\nCalibration factor for sensor {sensor_id}: {report['calibration_factor']}")
        print(f"Report saved in calibration_reports/sensor_{sensor_id}/")

if __name__ == "__main__":
    main()