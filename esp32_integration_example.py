#!/usr/bin/env python3
"""
ESP32-S3 Integration Example
============================
Demonstrates how to use the trained model with real I2C sensor data from ESP32-S3.

This example shows the complete workflow:
1. Receive sensor data from ESP32 (via UART/WiFi/MQTT)
2. Preprocess data using the trained scaler
3. Run inference with quantized TFLite model
4. Generate Jarvis AI alerts
5. Send alerts back to ESP32 or cloud dashboard

For actual deployment, replace the simulated data with real sensor readings.
"""

import numpy as np
import pickle
from train_model import JarvisAI
import tensorflow as tf

class ESP32PredictiveMaintenanceInference:
    """
    Inference engine for ESP32-S3 predictive maintenance.
    
    This class handles:
    - Loading the quantized TFLite model
    - Loading the fitted scaler
    - Real-time inference on sensor data
    - Jarvis AI alert generation
    """
    
    def __init__(self, 
                 model_path: str = './outputs/esp32_predictive_maintenance_int8.tflite',
                 scaler_path: str = './outputs/scaler.pkl'):
        """
        Initialize inference engine.
        
        Args:
            model_path: Path to quantized TFLite model
            scaler_path: Path to pickled scaler (optional)
        """
        # Load TFLite model
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # Get input/output details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Load scaler if available
        try:
            with open(scaler_path, 'rb') as f:
                self.scaler = pickle.load(f)
            print(f"✓ Loaded scaler from {scaler_path}")
        except FileNotFoundError:
            print(f"⚠ Scaler not found at {scaler_path}. Using identity scaling.")
            self.scaler = None
        
        # Initialize Jarvis AI
        self.jarvis = JarvisAI()
        
        print(f"✓ Loaded TFLite model from {model_path}")
        print(f"  Input shape: {self.input_details[0]['shape']}")
        print(f"  Output shape: {self.output_details[0]['shape']}")
        print(f"  Input dtype: {self.input_details[0]['dtype']}")
        print(f"  Model ready for inference!")
    
    def preprocess(self, sensor_data: np.ndarray) -> np.ndarray:
        """
        Preprocess raw sensor data.
        
        Args:
            sensor_data: Raw sensor readings (shape: [num_features] or [batch, num_features])
        
        Returns:
            Preprocessed data ready for inference
        """
        # Ensure 2D array
        if sensor_data.ndim == 1:
            sensor_data = sensor_data.reshape(1, -1)
        
        # Apply scaling if available
        if self.scaler is not None:
            sensor_data = self.scaler.transform(sensor_data)
        
        # Convert to INT8 for quantized model
        input_scale, input_zero_point = self.input_details[0]['quantization']
        if input_scale > 0:  # Model is quantized
            sensor_data = (sensor_data / input_scale + input_zero_point).astype(np.int8)
        
        return sensor_data
    
    def predict(self, sensor_data: np.ndarray) -> tuple:
        """
        Run inference on sensor data.
        
        Args:
            sensor_data: Preprocessed sensor data
        
        Returns:
            Tuple of (predicted_class, confidence, probabilities)
        """
        # Set input tensor
        self.interpreter.set_tensor(self.input_details[0]['index'], sensor_data)
        
        # Run inference
        self.interpreter.invoke()
        
        # Get output
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        
        # Dequantize output if needed
        output_scale, output_zero_point = self.output_details[0]['quantization']
        if output_scale > 0:
            output_data = (output_data.astype(np.float32) - output_zero_point) * output_scale
        
        # Softmax to get probabilities
        probabilities = self._softmax(output_data[0])
        predicted_class = np.argmax(probabilities)
        confidence = probabilities[predicted_class]
        
        return predicted_class, confidence, probabilities
    
    def _softmax(self, x: np.ndarray) -> np.ndarray:
        """Compute softmax values."""
        exp_x = np.exp(x - np.max(x))
        return exp_x / exp_x.sum()
    
    def generate_alert(self, 
                      sensor_data: np.ndarray,
                      ttf_hours: float = None) -> str:
        """
        Complete pipeline: predict + generate alert.
        
        Args:
            sensor_data: Raw sensor readings from ESP32
            ttf_hours: Optional Time-to-Failure estimate
        
        Returns:
            Jarvis AI alert message
        """
        # Preprocess
        processed_data = self.preprocess(sensor_data)
        
        # Predict
        pred_class, confidence, _ = self.predict(processed_data)
        
        # Generate alert
        alert = self.jarvis.generate_alert(
            prediction_class=pred_class,
            confidence=confidence,
            sensor_data=sensor_data,
            ttf_hours=ttf_hours
        )
        
        return alert


def simulate_esp32_data_stream():
    """
    Simulate real-time sensor data stream from ESP32-S3.
    
    In production, replace this with:
    - Serial UART reading: serial.Serial('/dev/ttyUSB0', 115200)
    - WiFi/HTTP: requests.get('http://esp32-ip/sensors')
    - MQTT: mqtt_client.subscribe('esp32/sensors')
    """
    # Simulated I2C sensor readings from ESP32-S3
    # In reality, these come from:
    # - Temperature sensor (e.g., BMP280 via I2C)
    # - Vibration sensor (e.g., MPU6050 accelerometer)
    # - Current sensor (e.g., INA219)
    # - etc.
    
    scenarios = [
        {
            'name': 'Normal Operation',
            'data': np.array([0.1, -0.2, 0.05, 0.15, -0.1, 0.0, 0.2, -0.05, 0.1, 0.0]),
            'ttf': 280.0
        },
        {
            'name': 'Early Warning Signs',
            'data': np.array([1.5, 2.1, 1.8, 1.3, 2.0, 1.7, 1.9, 1.6, 1.4, 1.8]),
            'ttf': 52.0
        },
        {
            'name': 'Critical Failure Imminent',
            'data': np.array([3.2, 3.8, 3.5, 3.1, 3.9, 3.4, 3.7, 3.3, 3.6, 3.2]),
            'ttf': 6.5
        },
    ]
    
    return scenarios


def main():
    """Demonstrate ESP32-S3 integration workflow."""
    print("="*80)
    print("ESP32-S3 PREDICTIVE MAINTENANCE - INTEGRATION EXAMPLE")
    print("="*80)
    print("\nThis example demonstrates real-time inference with sensor data from ESP32-S3.")
    print("In production, sensor data would be received via UART, WiFi, or MQTT.\n")
    
    # Note: This demo uses simulated inference without loading the actual model
    # In production, uncomment the following:
    # inference_engine = ESP32PredictiveMaintenanceInference()
    
    # For demo purposes, we'll use Jarvis directly
    jarvis = JarvisAI()
    
    # Simulate data stream from ESP32
    data_stream = simulate_esp32_data_stream()
    
    print("-"*80)
    print("SIMULATED ESP32-S3 DATA STREAM")
    print("-"*80)
    
    for i, scenario in enumerate(data_stream, 1):
        print(f"\n[Timestamp: {i*10}s] Scenario: {scenario['name']}")
        print(f"Sensor readings from I2C bus: {scenario['data'][:5]}...")
        
        # In production, replace this with:
        # pred_class, confidence, _ = inference_engine.predict(scenario['data'])
        
        # For demo, simulate predictions
        if 'Normal' in scenario['name']:
            pred_class = 0
            confidence = 0.96
        elif 'Warning' in scenario['name']:
            pred_class = 1
            confidence = 0.87
        else:
            pred_class = 2
            confidence = 0.94
        
        # Generate alert
        alert = jarvis.generate_alert(
            prediction_class=pred_class,
            confidence=confidence,
            sensor_data=scenario['data'],
            ttf_hours=scenario['ttf']
        )
        
        print(f"\n{alert}\n")
        print("-"*80)
    
    print("\n" + "="*80)
    print("INTEGRATION NOTES")
    print("="*80)
    print("""
For ESP32-S3 deployment:

1. HARDWARE SETUP:
   - Connect I2C sensors to ESP32-S3 (SDA: GPIO9, SCL: GPIO8)
   - Recommended sensors:
     * BMP280/BME280 (Temperature, Pressure)
     * MPU6050 (Accelerometer, Gyroscope)
     * INA219 (Current, Voltage)
     * Custom vibration sensors

2. ESP32 FIRMWARE (C/C++ with ESP-IDF):
   - Read sensor data via I2C (use i2c_low_level.c driver)
   - Run TFLite inference on ESP32 (TensorFlow Lite Micro)
   - OR: Send data to Python server for inference
   
3. DATA TRANSMISSION:
   Option A: UART Serial
     ESP32: Send sensor data via Serial.print()
     Python: Read via serial.Serial('/dev/ttyUSB0')
   
   Option B: WiFi/HTTP
     ESP32: POST to server via WiFi
     Python: Flask/FastAPI server
   
   Option C: MQTT
     ESP32: Publish to MQTT broker
     Python: Subscribe and process

4. ALERT DELIVERY:
   - Send Jarvis alerts back to ESP32 for LCD display
   - Forward to cloud dashboard (AWS IoT, Azure IoT)
   - Send SMS/Email for critical alerts
   - Log to time-series database (InfluxDB, Prometheus)

5. PRODUCTION CONSIDERATIONS:
   - Implement error handling for sensor failures
   - Add data buffering for network interruptions
   - Use watchdog timers for reliability
   - Implement OTA updates for model deployment
   - Add power management for battery operation
    """)
    print("="*80)


if __name__ == "__main__":
    main()
