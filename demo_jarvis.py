#!/usr/bin/env python3
"""
Quick demonstration of Jarvis AI conversational alerts.
Run this to see example outputs without full training.
"""

from train_model import JarvisAI
import numpy as np

def main():
    print("="*80)
    print("JARVIS AI - ESP32-S3 PREDICTIVE MAINTENANCE ALERTS DEMO")
    print("="*80)
    print("\nJarvis is a GPT-like conversational AI for generating human-readable")
    print("maintenance alerts from sensor data and ML predictions.\n")
    
    # Initialize Jarvis
    jarvis = JarvisAI()
    
    # Demo 1: Normal operation
    print("\n" + "-"*80)
    print("SCENARIO 1: Normal Operation")
    print("-"*80)
    sensor_data = np.array([0.1, -0.2, 0.05, 0.15, -0.1, 0.0, 0.2, -0.05, 0.1, 0.0])
    alert = jarvis.generate_alert(
        prediction_class=0,
        confidence=0.98,
        sensor_data=sensor_data,
        ttf_hours=250.0
    )
    print(alert)
    
    # Demo 2: Warning condition
    print("\n" + "-"*80)
    print("SCENARIO 2: Warning Detected")
    print("-"*80)
    sensor_data = np.array([1.5, 2.1, 1.8, 1.3, 2.0, 1.7, 1.9, 1.6, 1.4, 1.8])
    alert = jarvis.generate_alert(
        prediction_class=1,
        confidence=0.87,
        sensor_data=sensor_data,
        ttf_hours=48.5
    )
    print(alert)
    
    # Demo 3: Critical failure imminent
    print("\n" + "-"*80)
    print("SCENARIO 3: Critical Alert")
    print("-"*80)
    sensor_data = np.array([3.2, 3.8, 3.5, 3.1, 3.9, 3.4, 3.7, 3.3, 3.6, 3.2])
    alert = jarvis.generate_alert(
        prediction_class=2,
        confidence=0.95,
        sensor_data=sensor_data,
        ttf_hours=8.2
    )
    print(alert)
    
    # Demo 4: Batch processing
    print("\n" + "-"*80)
    print("SCENARIO 4: Batch Alert Generation")
    print("-"*80)
    predictions = np.array([0, 1, 2, 0, 1])
    confidences = np.array([0.96, 0.82, 0.91, 0.99, 0.78])
    sensor_batch = np.random.randn(5, 10)
    
    alerts = jarvis.batch_generate_alerts(predictions, confidences, sensor_batch)
    for i, alert in enumerate(alerts):
        print(f"\nAlert {i+1}:")
        print(alert)
    
    print("\n" + "="*80)
    print("DEMO COMPLETE")
    print("="*80)
    print("\nKey Features:")
    print("✓ Context-aware alert generation")
    print("✓ Confidence scoring")
    print("✓ Time-to-Failure (TTF) estimation")
    print("✓ Sensor data integration")
    print("✓ Batch processing support")
    print("✓ Industrial-grade messaging")
    print("\nIntegrate Jarvis into your ESP32-S3 deployment for intelligent maintenance!")
    print("="*80)

if __name__ == "__main__":
    main()
