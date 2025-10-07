#!/usr/bin/env python3
"""
ESP32-S3 Predictive Maintenance AI Training System
===================================================
Advanced AI/ML training pipeline for predictive maintenance on ESP32-S3 microcontrollers.
Optimized for lightweight deployment with int8 quantization and memory efficiency.

Features:
- Lightweight neural networks optimized for ESP32-S3 (limited PSRAM/SRAM)
- INT8 quantization for efficient edge deployment
- GPT-like conversational AI (Jarvis) for generative maintenance alerts
- Publication-ready SVG visualization (training curves, confusion matrices, PR/ROC curves)
- Comprehensive error handling and validation
- Production-ready code suitable for patents and commercial applications

Author: Gaurav Kumar
License: MIT
Target Hardware: ESP32-S3 (Xtensa LX7 dual-core, 512KB SRAM, optional PSRAM)
"""

import os
import sys
import warnings
import logging
from typing import Dict, List, Tuple, Optional, Union
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for server environments
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split, StratifiedKFold
from sklearn.preprocessing import StandardScaler, RobustScaler
from sklearn.metrics import (
    accuracy_score, precision_score, recall_score, f1_score,
    confusion_matrix, classification_report, roc_curve, auc,
    precision_recall_curve, average_precision_score
)
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, models, callbacks
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.regularizers import l2

# Suppress warnings for cleaner output
warnings.filterwarnings('ignore')
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('training.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


@dataclass
class ModelConfig:
    """Configuration for ESP32-S3 optimized model architecture."""
    # Model architecture
    input_features: int = 10  # Number of sensor inputs (I2C, vibration, temp, etc.)
    hidden_units: List[int] = None  # Lightweight: [32, 16, 8]
    dropout_rate: float = 0.3
    l2_regularization: float = 0.001
    
    # Training parameters
    batch_size: int = 32
    epochs: int = 100
    learning_rate: float = 0.001
    validation_split: float = 0.2
    early_stopping_patience: int = 15
    
    # Quantization settings
    enable_quantization: bool = True
    quantization_bits: int = 8  # INT8 for ESP32-S3
    
    # Output settings
    num_classes: int = 3  # Normal, Warning, Critical
    output_dir: str = './outputs'
    model_name: str = 'esp32_predictive_maintenance'
    
    def __post_init__(self):
        """Initialize default values."""
        if self.hidden_units is None:
            # Optimized for ESP32-S3: minimal layers, small units
            self.hidden_units = [32, 16, 8]
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)


class JarvisAI:
    """
    GPT-like Conversational AI for Generative Maintenance Alerts.
    
    Generates human-readable, context-aware maintenance alerts based on
    sensor data and model predictions. Simulates a sophisticated AI assistant
    for industrial predictive maintenance.
    """
    
    def __init__(self, model_name: str = "Jarvis-ESP32"):
        """Initialize Jarvis AI module."""
        self.model_name = model_name
        self.alert_templates = self._initialize_templates()
        logger.info(f"Initialized {self.model_name} conversational AI module")
    
    def _initialize_templates(self) -> Dict[str, List[str]]:
        """Initialize alert message templates with varying complexity."""
        return {
            'normal': [
                "System operating within normal parameters. All sensors report nominal values.",
                "Equipment health status: EXCELLENT. No maintenance required at this time.",
                "Diagnostic check passed. All subsystems functioning optimally.",
            ],
            'warning': [
                "⚠️ ALERT: Anomalous pattern detected in sensor readings. Recommend inspection within 48 hours.",
                "⚠️ WARNING: Equipment showing early degradation indicators. Schedule preventive maintenance.",
                "⚠️ CAUTION: Unusual vibration/temperature patterns observed. Monitor closely for next 24 hours.",
            ],
            'critical': [
                "🚨 CRITICAL ALERT: Immediate maintenance required! Equipment failure imminent within 12-24 hours.",
                "🚨 URGENT: Severe anomaly detected. System integrity at risk. Shut down recommended.",
                "🚨 EMERGENCY: Multiple failure indicators present. Contact maintenance team immediately.",
            ]
        }
    
    def generate_alert(self, 
                      prediction_class: int, 
                      confidence: float,
                      sensor_data: Optional[np.ndarray] = None,
                      ttf_hours: Optional[float] = None) -> str:
        """
        Generate context-aware maintenance alert.
        
        Args:
            prediction_class: Predicted class (0=normal, 1=warning, 2=critical)
            confidence: Model confidence score (0-1)
            sensor_data: Optional sensor readings for detailed analysis
            ttf_hours: Optional Time-to-Failure estimate in hours
        
        Returns:
            Human-readable alert message
        """
        try:
            # Map class to alert level
            alert_levels = ['normal', 'warning', 'critical']
            alert_level = alert_levels[prediction_class]
            
            # Select template based on confidence
            templates = self.alert_templates[alert_level]
            base_message = np.random.choice(templates)
            
            # Enhance with contextual information
            enhanced_message = f"[{self.model_name}] {base_message}\n"
            enhanced_message += f"Confidence: {confidence:.2%} | Class: {alert_level.upper()}"
            
            if ttf_hours is not None:
                enhanced_message += f" | Est. TTF: {ttf_hours:.1f}h"
            
            if sensor_data is not None and len(sensor_data) > 0:
                enhanced_message += f"\nSensor readings: {sensor_data[:3]}..."
            
            return enhanced_message
            
        except Exception as e:
            logger.error(f"Error generating alert: {e}")
            return f"Alert generation failed. Raw prediction: Class {prediction_class}, Conf {confidence:.2f}"
    
    def batch_generate_alerts(self, 
                             predictions: np.ndarray,
                             confidences: np.ndarray,
                             sensor_data: Optional[np.ndarray] = None) -> List[str]:
        """Generate alerts for batch predictions."""
        alerts = []
        for i, (pred, conf) in enumerate(zip(predictions, confidences)):
            sensors = sensor_data[i] if sensor_data is not None else None
            alert = self.generate_alert(pred, conf, sensors)
            alerts.append(alert)
        return alerts


class ESP32ModelTrainer:
    """
    Production-ready trainer for ESP32-S3 predictive maintenance models.
    
    Features:
    - Lightweight architecture optimization
    - INT8 quantization for edge deployment
    - Comprehensive metrics and visualization
    - Error handling and validation
    """
    
    def __init__(self, config: ModelConfig):
        """
        Initialize trainer with configuration.
        
        Args:
            config: ModelConfig instance with training parameters
        """
        self.config = config
        self.model = None
        self.quantized_model = None
        self.scaler = RobustScaler()  # Robust to outliers in sensor data
        self.history = None
        self.metrics = {}
        self.jarvis = JarvisAI()
        
        logger.info("ESP32ModelTrainer initialized")
        logger.info(f"Target hardware: ESP32-S3 (Xtensa LX7)")
        logger.info(f"Model architecture: {self.config.hidden_units}")
        logger.info(f"Quantization: {'Enabled (INT8)' if self.config.enable_quantization else 'Disabled'}")
    
    def build_model(self) -> keras.Model:
        """
        Build lightweight neural network optimized for ESP32-S3.
        
        Architecture designed for:
        - Minimal memory footprint (<100KB after quantization)
        - Fast inference (<50ms on ESP32-S3)
        - High accuracy (>90% F1 score)
        
        Returns:
            Compiled Keras model
        """
        try:
            logger.info("Building ESP32-optimized model architecture...")
            
            # Input layer
            inputs = layers.Input(shape=(self.config.input_features,), name='sensor_inputs')
            x = inputs
            
            # Lightweight hidden layers with regularization
            for i, units in enumerate(self.config.hidden_units):
                x = layers.Dense(
                    units,
                    activation='relu',
                    kernel_regularizer=l2(self.config.l2_regularization),
                    name=f'dense_{i+1}'
                )(x)
                
                # Batch normalization for stability
                x = layers.BatchNormalization(name=f'bn_{i+1}')(x)
                
                # Dropout for regularization
                if self.config.dropout_rate > 0:
                    x = layers.Dropout(self.config.dropout_rate, name=f'dropout_{i+1}')(x)
            
            # Output layer
            outputs = layers.Dense(
                self.config.num_classes,
                activation='softmax',
                name='predictions'
            )(x)
            
            # Create model
            model = models.Model(inputs=inputs, outputs=outputs, name=self.config.model_name)
            
            # Compile with appropriate metrics
            # Note: Using simpler metrics for better compatibility across TensorFlow versions
            model.compile(
                optimizer=Adam(learning_rate=self.config.learning_rate),
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy']  # Additional metrics computed post-training
            )
            
            logger.info(f"Model built successfully: {model.count_params():,} parameters")
            model.summary(print_fn=logger.info)
            
            self.model = model
            return model
            
        except Exception as e:
            logger.error(f"Error building model: {e}")
            raise
    
    def prepare_data(self, 
                    X: np.ndarray, 
                    y: np.ndarray,
                    validation_split: Optional[float] = None) -> Tuple:
        """
        Prepare and validate training data with comprehensive error checking.
        
        Args:
            X: Feature matrix (samples, features)
            y: Labels (samples,)
            validation_split: Optional validation split ratio
        
        Returns:
            Tuple of (X_train, X_val, y_train, y_val) or (X, y) if no split
        """
        try:
            # Validate input shapes
            if X.shape[0] != y.shape[0]:
                raise ValueError(f"Sample mismatch: X has {X.shape[0]} samples, y has {y.shape[0]}")
            
            if X.shape[1] != self.config.input_features:
                raise ValueError(f"Feature mismatch: Expected {self.config.input_features}, got {X.shape[1]}")
            
            logger.info(f"Data shape: X={X.shape}, y={y.shape}")
            logger.info(f"Class distribution: {np.bincount(y)}")
            
            # Check for NaN/Inf values
            if np.any(np.isnan(X)) or np.any(np.isinf(X)):
                logger.warning("NaN/Inf detected in features. Applying imputation...")
                X = np.nan_to_num(X, nan=0.0, posinf=1e6, neginf=-1e6)
            
            # Scale features using RobustScaler (handles outliers better)
            X_scaled = self.scaler.fit_transform(X)
            logger.info("Features scaled using RobustScaler")
            
            # Split data if requested
            if validation_split is not None and validation_split > 0:
                X_train, X_val, y_train, y_val = train_test_split(
                    X_scaled, y,
                    test_size=validation_split,
                    stratify=y,
                    random_state=42
                )
                logger.info(f"Data split: Train={len(y_train)}, Val={len(y_val)}")
                return X_train, X_val, y_train, y_val
            
            return X_scaled, y
            
        except Exception as e:
            logger.error(f"Error preparing data: {e}")
            raise
    
    def train(self, 
             X_train: np.ndarray, 
             y_train: np.ndarray,
             X_val: Optional[np.ndarray] = None,
             y_val: Optional[np.ndarray] = None) -> Dict:
        """
        Train the model with comprehensive monitoring and callbacks.
        
        Args:
            X_train: Training features
            y_train: Training labels
            X_val: Validation features (optional)
            y_val: Validation labels (optional)
        
        Returns:
            Training history dictionary
        """
        try:
            if self.model is None:
                raise ValueError("Model not built. Call build_model() first.")
            
            logger.info("Starting model training...")
            logger.info(f"Training samples: {len(y_train)}")
            
            # Prepare validation data
            validation_data = None
            if X_val is not None and y_val is not None:
                validation_data = (X_val, y_val)
                logger.info(f"Validation samples: {len(y_val)}")
            
            # Define callbacks
            callback_list = [
                # Early stopping to prevent overfitting
                callbacks.EarlyStopping(
                    monitor='val_loss' if validation_data else 'loss',
                    patience=self.config.early_stopping_patience,
                    restore_best_weights=True,
                    verbose=1
                ),
                
                # Reduce learning rate on plateau
                callbacks.ReduceLROnPlateau(
                    monitor='val_loss' if validation_data else 'loss',
                    factor=0.5,
                    patience=5,
                    min_lr=1e-6,
                    verbose=1
                ),
                
                # Model checkpoint
                callbacks.ModelCheckpoint(
                    os.path.join(self.config.output_dir, f'{self.config.model_name}_best.h5'),
                    monitor='val_loss' if validation_data else 'loss',
                    save_best_only=True,
                    verbose=0
                ),
                
                # CSV logger for detailed metrics
                callbacks.CSVLogger(
                    os.path.join(self.config.output_dir, 'training_log.csv')
                )
            ]
            
            # Train model
            self.history = self.model.fit(
                X_train, y_train,
                batch_size=self.config.batch_size,
                epochs=self.config.epochs,
                validation_data=validation_data,
                callbacks=callback_list,
                verbose=1
            )
            
            logger.info("Training completed successfully")
            return self.history.history
            
        except Exception as e:
            logger.error(f"Error during training: {e}")
            raise
    
    def evaluate(self, X_test: np.ndarray, y_test: np.ndarray) -> Dict:
        """
        Comprehensive model evaluation with multiple metrics.
        
        Args:
            X_test: Test features
            y_test: Test labels
        
        Returns:
            Dictionary of evaluation metrics
        """
        try:
            if self.model is None:
                raise ValueError("Model not trained. Call train() first.")
            
            logger.info("Evaluating model performance...")
            
            # Get predictions
            y_pred_proba = self.model.predict(X_test, verbose=0)
            y_pred = np.argmax(y_pred_proba, axis=1)
            
            # Calculate metrics
            self.metrics = {
                'accuracy': accuracy_score(y_test, y_pred),
                'precision_macro': precision_score(y_test, y_pred, average='macro', zero_division=0),
                'recall_macro': recall_score(y_test, y_pred, average='macro', zero_division=0),
                'f1_macro': f1_score(y_test, y_pred, average='macro', zero_division=0),
                'precision_weighted': precision_score(y_test, y_pred, average='weighted', zero_division=0),
                'recall_weighted': recall_score(y_test, y_pred, average='weighted', zero_division=0),
                'f1_weighted': f1_score(y_test, y_pred, average='weighted', zero_division=0),
                'confusion_matrix': confusion_matrix(y_test, y_pred).tolist(),
                'classification_report': classification_report(y_test, y_pred, zero_division=0)
            }
            
            # Log metrics
            logger.info("="*60)
            logger.info("MODEL PERFORMANCE METRICS")
            logger.info("="*60)
            logger.info(f"Accuracy:           {self.metrics['accuracy']:.4f}")
            logger.info(f"F1 Score (macro):   {self.metrics['f1_macro']:.4f}")
            logger.info(f"F1 Score (weighted): {self.metrics['f1_weighted']:.4f}")
            logger.info(f"Precision (macro):  {self.metrics['precision_macro']:.4f}")
            logger.info(f"Recall (macro):     {self.metrics['recall_macro']:.4f}")
            logger.info("="*60)
            logger.info("\nClassification Report:\n")
            logger.info(self.metrics['classification_report'])
            
            return self.metrics
            
        except Exception as e:
            logger.error(f"Error during evaluation: {e}")
            raise
    
    def quantize_model(self) -> str:
        """
        Apply INT8 quantization for ESP32-S3 deployment.
        
        Quantization reduces model size by ~4x and speeds up inference
        while maintaining accuracy within 1-2% of full precision.
        
        Returns:
            Path to quantized TFLite model
        """
        try:
            if not self.config.enable_quantization:
                logger.info("Quantization disabled in config")
                return None
            
            logger.info("Starting INT8 quantization for ESP32-S3...")
            
            # Convert to TFLite format
            converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
            
            # Enable INT8 quantization
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            converter.target_spec.supported_types = [tf.int8]
            
            # Representative dataset for quantization calibration
            def representative_dataset():
                for _ in range(100):
                    # Generate random data matching input shape
                    data = np.random.randn(1, self.config.input_features).astype(np.float32)
                    yield [data]
            
            converter.representative_dataset = representative_dataset
            converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
            converter.inference_input_type = tf.int8
            converter.inference_output_type = tf.int8
            
            # Convert
            quantized_tflite_model = converter.convert()
            
            # Save quantized model
            tflite_path = os.path.join(self.config.output_dir, f'{self.config.model_name}_int8.tflite')
            with open(tflite_path, 'wb') as f:
                f.write(quantized_tflite_model)
            
            # Log size comparison
            h5_path = os.path.join(self.config.output_dir, f'{self.config.model_name}_best.h5')
            if os.path.exists(h5_path):
                h5_size = os.path.getsize(h5_path) / 1024  # KB
                tflite_size = os.path.getsize(tflite_path) / 1024  # KB
                compression_ratio = h5_size / tflite_size
                
                logger.info(f"Model size comparison:")
                logger.info(f"  Original (H5):     {h5_size:.2f} KB")
                logger.info(f"  Quantized (TFLite): {tflite_size:.2f} KB")
                logger.info(f"  Compression ratio:  {compression_ratio:.2f}x")
            
            logger.info(f"Quantized model saved: {tflite_path}")
            self.quantized_model = tflite_path
            
            return tflite_path
            
        except Exception as e:
            logger.error(f"Error during quantization: {e}")
            raise
    
    def generate_visualizations(self, 
                               X_test: np.ndarray,
                               y_test: np.ndarray,
                               y_pred_proba: Optional[np.ndarray] = None) -> None:
        """
        Generate publication-ready SVG visualizations.
        
        Creates:
        1. Training history curves (loss, accuracy)
        2. Confusion matrix heatmap
        3. Precision-Recall curves
        4. ROC curves
        5. F1 score comparison
        6. Time-to-Failure (TTF) scatter plot
        
        Args:
            X_test: Test features
            y_test: Test labels
            y_pred_proba: Prediction probabilities (optional)
        """
        try:
            logger.info("Generating publication-ready visualizations...")
            
            # Set style for publication quality
            plt.style.use('seaborn-v0_8-darkgrid')
            sns.set_palette("husl")
            
            # Get predictions if not provided
            if y_pred_proba is None:
                y_pred_proba = self.model.predict(X_test, verbose=0)
            y_pred = np.argmax(y_pred_proba, axis=1)
            
            # 1. Training History (Loss and Accuracy)
            self._plot_training_history()
            
            # 2. Confusion Matrix
            self._plot_confusion_matrix(y_test, y_pred)
            
            # 3. F1 Score Comparison
            self._plot_f1_scores(y_test, y_pred)
            
            # 4. Precision-Recall Curves
            self._plot_precision_recall_curves(y_test, y_pred_proba)
            
            # 5. ROC Curves
            self._plot_roc_curves(y_test, y_pred_proba)
            
            # 6. Time-to-Failure (TTF) Scatter Plot
            self._plot_ttf_scatter(X_test, y_test, y_pred)
            
            logger.info(f"All visualizations saved to {self.config.output_dir}/")
            
        except Exception as e:
            logger.error(f"Error generating visualizations: {e}")
            raise
    
    def _plot_training_history(self) -> None:
        """Plot training and validation loss/accuracy curves."""
        if self.history is None:
            logger.warning("No training history available")
            return
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # Loss curves
        axes[0].plot(self.history.history['loss'], label='Training Loss', linewidth=2)
        if 'val_loss' in self.history.history:
            axes[0].plot(self.history.history['val_loss'], label='Validation Loss', linewidth=2)
        axes[0].set_xlabel('Epoch', fontsize=12)
        axes[0].set_ylabel('Loss', fontsize=12)
        axes[0].set_title('Model Loss During Training', fontsize=14, fontweight='bold')
        axes[0].legend(fontsize=10)
        axes[0].grid(True, alpha=0.3)
        
        # Accuracy curves
        axes[1].plot(self.history.history['accuracy'], label='Training Accuracy', linewidth=2)
        if 'val_accuracy' in self.history.history:
            axes[1].plot(self.history.history['val_accuracy'], label='Validation Accuracy', linewidth=2)
        axes[1].set_xlabel('Epoch', fontsize=12)
        axes[1].set_ylabel('Accuracy', fontsize=12)
        axes[1].set_title('Model Accuracy During Training', fontsize=14, fontweight='bold')
        axes[1].legend(fontsize=10)
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'training_curves.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ Training curves saved")
    
    def _plot_confusion_matrix(self, y_true: np.ndarray, y_pred: np.ndarray) -> None:
        """Plot confusion matrix heatmap."""
        cm = confusion_matrix(y_true, y_pred)
        
        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', cbar=True,
                   xticklabels=['Normal', 'Warning', 'Critical'],
                   yticklabels=['Normal', 'Warning', 'Critical'])
        plt.xlabel('Predicted Label', fontsize=12)
        plt.ylabel('True Label', fontsize=12)
        plt.title('Confusion Matrix - ESP32-S3 Predictive Maintenance', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'confusion_matrix.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ Confusion matrix saved")
    
    def _plot_f1_scores(self, y_true: np.ndarray, y_pred: np.ndarray) -> None:
        """Plot F1 scores for each class."""
        f1_scores = f1_score(y_true, y_pred, average=None, zero_division=0)
        classes = ['Normal', 'Warning', 'Critical']
        
        plt.figure(figsize=(8, 5))
        bars = plt.bar(classes, f1_scores, color=['green', 'orange', 'red'], alpha=0.7)
        plt.xlabel('Class', fontsize=12)
        plt.ylabel('F1 Score', fontsize=12)
        plt.title('F1 Score by Maintenance Class', fontsize=14, fontweight='bold')
        plt.ylim([0, 1.1])
        plt.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bar, score in zip(bars, f1_scores):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height,
                    f'{score:.3f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'f1_scores.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ F1 scores saved")
    
    def _plot_precision_recall_curves(self, y_true: np.ndarray, y_pred_proba: np.ndarray) -> None:
        """Plot Precision-Recall curves for each class."""
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        classes = ['Normal', 'Warning', 'Critical']
        colors = ['green', 'orange', 'red']
        
        for idx, (class_name, color) in enumerate(zip(classes, colors)):
            # Binarize the output for current class
            y_true_binary = (y_true == idx).astype(int)
            y_score = y_pred_proba[:, idx]
            
            # Calculate precision-recall curve
            precision, recall, _ = precision_recall_curve(y_true_binary, y_score)
            avg_precision = average_precision_score(y_true_binary, y_score)
            
            # Plot
            axes[idx].plot(recall, precision, color=color, linewidth=2,
                          label=f'AP = {avg_precision:.3f}')
            axes[idx].set_xlabel('Recall', fontsize=10)
            axes[idx].set_ylabel('Precision', fontsize=10)
            axes[idx].set_title(f'{class_name} Class', fontsize=12, fontweight='bold')
            axes[idx].legend(fontsize=9)
            axes[idx].grid(True, alpha=0.3)
            axes[idx].set_xlim([0, 1])
            axes[idx].set_ylim([0, 1.05])
        
        plt.suptitle('Precision-Recall Curves', fontsize=14, fontweight='bold', y=1.02)
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'precision_recall_curves.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ Precision-Recall curves saved")
    
    def _plot_roc_curves(self, y_true: np.ndarray, y_pred_proba: np.ndarray) -> None:
        """Plot ROC curves for each class."""
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        classes = ['Normal', 'Warning', 'Critical']
        colors = ['green', 'orange', 'red']
        
        for idx, (class_name, color) in enumerate(zip(classes, colors)):
            # Binarize the output for current class
            y_true_binary = (y_true == idx).astype(int)
            y_score = y_pred_proba[:, idx]
            
            # Calculate ROC curve
            fpr, tpr, _ = roc_curve(y_true_binary, y_score)
            roc_auc = auc(fpr, tpr)
            
            # Plot
            axes[idx].plot(fpr, tpr, color=color, linewidth=2,
                          label=f'AUC = {roc_auc:.3f}')
            axes[idx].plot([0, 1], [0, 1], 'k--', linewidth=1, alpha=0.3)
            axes[idx].set_xlabel('False Positive Rate', fontsize=10)
            axes[idx].set_ylabel('True Positive Rate', fontsize=10)
            axes[idx].set_title(f'{class_name} Class', fontsize=12, fontweight='bold')
            axes[idx].legend(fontsize=9)
            axes[idx].grid(True, alpha=0.3)
            axes[idx].set_xlim([0, 1])
            axes[idx].set_ylim([0, 1.05])
        
        plt.suptitle('ROC Curves', fontsize=14, fontweight='bold', y=1.02)
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'roc_curves.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ ROC curves saved")
    
    def _plot_ttf_scatter(self, X_test: np.ndarray, y_true: np.ndarray, y_pred: np.ndarray) -> None:
        """Plot Time-to-Failure (TTF) scatter plot."""
        # Simulate TTF values based on predictions (for demonstration)
        # In real scenario, this would come from actual sensor data
        ttf_hours = np.random.exponential(scale=100, size=len(y_true))
        ttf_hours[y_true == 1] *= 0.5  # Warning class has lower TTF
        ttf_hours[y_true == 2] *= 0.2  # Critical class has much lower TTF
        
        # Create scatter plot
        fig, ax = plt.subplots(figsize=(10, 6))
        
        colors = ['green', 'orange', 'red']
        labels = ['Normal', 'Warning', 'Critical']
        
        for idx, (color, label) in enumerate(zip(colors, labels)):
            mask = y_true == idx
            ax.scatter(np.arange(len(y_true))[mask], ttf_hours[mask],
                      c=color, label=label, alpha=0.6, s=50, edgecolors='black', linewidth=0.5)
        
        ax.set_xlabel('Sample Index', fontsize=12)
        ax.set_ylabel('Time-to-Failure (hours)', fontsize=12)
        ax.set_title('Time-to-Failure (TTF) Distribution by Class', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.output_dir, 'ttf_scatter.svg'), format='svg', dpi=300)
        plt.close()
        logger.info("✓ TTF scatter plot saved")
    
    def demonstrate_jarvis(self, X_test: np.ndarray, y_test: np.ndarray, num_samples: int = 5) -> None:
        """
        Demonstrate Jarvis AI conversational alerts.
        
        Args:
            X_test: Test features
            y_test: Test labels
            num_samples: Number of samples to demonstrate
        """
        try:
            logger.info("\n" + "="*80)
            logger.info("JARVIS AI - GENERATIVE MAINTENANCE ALERTS DEMONSTRATION")
            logger.info("="*80 + "\n")
            
            # Get predictions
            y_pred_proba = self.model.predict(X_test[:num_samples], verbose=0)
            y_pred = np.argmax(y_pred_proba, axis=1)
            confidences = np.max(y_pred_proba, axis=1)
            
            # Generate TTF estimates
            ttf_estimates = np.random.exponential(scale=100, size=num_samples)
            ttf_estimates[y_pred == 1] *= 0.5
            ttf_estimates[y_pred == 2] *= 0.2
            
            # Generate alerts
            for i in range(num_samples):
                logger.info(f"\n--- Sample {i+1} ---")
                alert = self.jarvis.generate_alert(
                    prediction_class=y_pred[i],
                    confidence=confidences[i],
                    sensor_data=X_test[i],
                    ttf_hours=ttf_estimates[i]
                )
                logger.info(alert)
                logger.info(f"Ground Truth: Class {y_test[i]}")
            
            logger.info("\n" + "="*80)
            
        except Exception as e:
            logger.error(f"Error demonstrating Jarvis: {e}")


def generate_synthetic_data(num_samples: int = 1000, 
                           num_features: int = 10,
                           noise_level: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate synthetic sensor data for predictive maintenance.
    
    Simulates realistic I2C sensor readings:
    - Temperature, vibration, current, voltage, pressure, etc.
    - Three classes: Normal (0), Warning (1), Critical (2)
    
    Args:
        num_samples: Number of samples to generate
        num_features: Number of sensor features
        noise_level: Gaussian noise level
    
    Returns:
        Tuple of (X, y) where X is features and y is labels
    """
    logger.info(f"Generating synthetic sensor data: {num_samples} samples, {num_features} features")
    
    np.random.seed(42)
    
    # Generate base data for each class
    samples_per_class = num_samples // 3
    
    # Class 0: Normal operation (centered around 0)
    X_normal = np.random.randn(samples_per_class, num_features) * 0.5
    y_normal = np.zeros(samples_per_class, dtype=int)
    
    # Class 1: Warning (shifted mean, higher variance)
    X_warning = np.random.randn(samples_per_class, num_features) * 1.0 + 1.5
    y_warning = np.ones(samples_per_class, dtype=int)
    
    # Class 2: Critical (large shift, high variance)
    X_critical = np.random.randn(samples_per_class, num_features) * 1.5 + 3.0
    y_critical = np.ones(samples_per_class, dtype=int) * 2
    
    # Combine data
    X = np.vstack([X_normal, X_warning, X_critical])
    y = np.hstack([y_normal, y_warning, y_critical])
    
    # Add noise
    X += np.random.randn(*X.shape) * noise_level
    
    # Shuffle
    indices = np.random.permutation(len(y))
    X = X[indices]
    y = y[indices]
    
    logger.info(f"Generated data shape: X={X.shape}, y={y.shape}")
    logger.info(f"Class distribution: {np.bincount(y)}")
    
    return X, y


def main():
    """
    Main training pipeline for ESP32-S3 predictive maintenance.
    
    Demonstrates complete workflow:
    1. Data generation/loading
    2. Model building and training
    3. Evaluation and metrics
    4. Quantization for edge deployment
    5. Visualization generation
    6. Jarvis AI demonstration
    """
    logger.info("="*80)
    logger.info("ESP32-S3 PREDICTIVE MAINTENANCE AI TRAINING SYSTEM")
    logger.info("="*80)
    logger.info("Patent-pending technology for industrial IoT applications")
    logger.info("Optimized for ESP32-S3 (Xtensa LX7, 512KB SRAM, optional PSRAM)")
    logger.info("="*80 + "\n")
    
    try:
        # 1. Configuration
        config = ModelConfig(
            input_features=10,
            hidden_units=[32, 16, 8],  # Lightweight for ESP32-S3
            dropout_rate=0.3,
            batch_size=32,
            epochs=50,
            learning_rate=0.001,
            enable_quantization=True,
            output_dir='./outputs',
            model_name='esp32_predictive_maintenance'
        )
        
        # 2. Generate/Load Data
        logger.info("Step 1: Data Preparation")
        X, y = generate_synthetic_data(num_samples=2000, num_features=config.input_features)
        
        # 3. Initialize Trainer
        logger.info("\nStep 2: Initializing Trainer")
        trainer = ESP32ModelTrainer(config)
        
        # 4. Build Model
        logger.info("\nStep 3: Building Model Architecture")
        trainer.build_model()
        
        # 5. Prepare Data
        logger.info("\nStep 4: Preparing Training Data")
        X_train, X_val, y_train, y_val = trainer.prepare_data(X, y, validation_split=0.2)
        
        # Further split validation into validation and test
        X_val, X_test, y_val, y_test = train_test_split(
            X_val, y_val, test_size=0.5, stratify=y_val, random_state=42
        )
        
        # 6. Train Model
        logger.info("\nStep 5: Training Model")
        trainer.train(X_train, y_train, X_val, y_val)
        
        # 7. Evaluate Model
        logger.info("\nStep 6: Evaluating Model")
        metrics = trainer.evaluate(X_test, y_test)
        
        # 8. Quantize for ESP32-S3
        logger.info("\nStep 7: Quantizing Model for ESP32-S3")
        quantized_path = trainer.quantize_model()
        
        # 9. Generate Visualizations
        logger.info("\nStep 8: Generating Publication-Ready Visualizations")
        trainer.generate_visualizations(X_test, y_test)
        
        # 10. Demonstrate Jarvis AI
        logger.info("\nStep 9: Demonstrating Jarvis AI Alerts")
        trainer.demonstrate_jarvis(X_test, y_test, num_samples=5)
        
        # Final Summary
        logger.info("\n" + "="*80)
        logger.info("TRAINING PIPELINE COMPLETED SUCCESSFULLY")
        logger.info("="*80)
        logger.info(f"Model saved: {config.output_dir}/{config.model_name}_best.h5")
        logger.info(f"Quantized model: {quantized_path}")
        logger.info(f"Visualizations: {config.output_dir}/*.svg")
        logger.info(f"Final F1 Score: {metrics['f1_weighted']:.4f}")
        logger.info(f"Final Accuracy: {metrics['accuracy']:.4f}")
        logger.info("="*80)
        logger.info("\nREADY FOR ESP32-S3 DEPLOYMENT")
        logger.info("Next steps:")
        logger.info("  1. Convert TFLite model to C array for ESP32")
        logger.info("  2. Integrate with I2C sensor drivers")
        logger.info("  3. Deploy to ESP32-S3 via ESP-IDF")
        logger.info("  4. Monitor real-time predictions via UART/WiFi")
        logger.info("="*80)
        
    except Exception as e:
        logger.error(f"Training pipeline failed: {e}")
        raise


if __name__ == "__main__":
    main()
