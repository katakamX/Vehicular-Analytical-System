import pandas as pd
import numpy as np
import glob
import os
import joblib
from sklearn.ensemble import IsolationForest, RandomForestClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, accuracy_score

class VehicularMLPipeline:
    def __init__(self):
        # Initialize Models
        self.scaler = StandardScaler()
        self.anomaly_detector = IsolationForest(n_estimators=100, contamination=0.05, random_state=42)
        self.fault_classifier = RandomForestClassifier(n_estimators=100, max_depth=10, random_state=42)
        
        # Internal standardized names the ML models will use
        self.features = [
            'speed', 'rpm', 'load', 'temp', 
            'brake', 'steering', 'throttle', 'vibration'
        ]

    def _find_best_columns(self, available_columns):
        """
        Dynamically maps your CSV headers to the ML feature names.
        This fixes the KeyError by looking for keywords in your headers.
        """
        mapping = {}
        # Define keywords to look for in your CSV headers
        search_patterns = {
            'speed': ['speed', 'velocity', 'kmh', 'vss'],
            'rpm': ['rpm', 'engine_speed', 'revolutions'],
            'load': ['load', 'engine_load', 'throttle_pct'],
            'temp': ['temp', 'coolant', 'temperature', 'celsius'],
            'brake': ['brake', 'braking', 'pressure'],
            'steering': ['steering', 'angle', 'deg'],
            'throttle': ['throttle', 'pos', 'acceleration'],
            'vibration': ['vibration', 'accel_z', 'g-force', 'vibrate']
        }

        for target, keywords in search_patterns.items():
            for col in available_columns:
                if any(key in col.lower() for key in keywords):
                    mapping[col] = target
                    break # Found the best match for this feature
        
        return mapping

    def load_and_preprocess(self):
        """Scans folder for CSVs, merges them, and fixes column names."""
        csv_files = glob.glob("*.csv")
        
        if not csv_files:
            print("CRITICAL: No CSV files found. Check your file path.")
            return None, None, None

        print(f"Detected {len(csv_files)} log files. Merging...")
        df_list = []
        for file in csv_files:
            try:
                temp_df = pd.read_csv(file)
                df_list.append(temp_df)
            except Exception as e:
                print(f"Skipping {file} due to error: {e}")

        df = pd.concat(df_list, axis=0, ignore_index=True)
        
        # 1. Map columns dynamically to fix KeyError
        column_map = self._find_best_columns(df.columns)
        print(f"Mapped Columns: {column_map}")
        
        df.rename(columns=column_map, inplace=True)

        # 2. Verify we have the required features
        missing = [f for f in self.features if f not in df.columns]
        if missing:
            print(f"WARNING: Missing data for: {missing}. Creating dummy values for these.")
            for m in missing:
                df[m] = 0 # Fill missing columns with 0 so the model can still run

        # 3. Clean and Scale
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        df[numeric_cols] = df[numeric_cols].fillna(df[numeric_cols].mean())
        
        X = df[self.features]
        X_scaled = self.scaler.fit_transform(X)
        
        # Target for classification (if exists)
        y_fault = df['fault_code'] if 'fault_code' in df.columns else None
        
        return X_scaled, y_fault, df

    def train_anomaly_detector(self, X_scaled):
        print("Training Anomaly Detection (Isolation Forest)...")
        self.anomaly_detector.fit(X_scaled)

    def train_fault_classifier(self, X_scaled, y_fault):
        if y_fault is None or len(y_fault.unique()) < 2:
            print("Skipping Classifier: No labeled fault data found in CSVs.")
            return

        print("Training Fault Classifier (Random Forest)...")
        X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_fault, test_size=0.2)
        self.fault_classifier.fit(X_train, y_train)
        print(f"Model Accuracy: {self.fault_classifier.score(X_test, y_test):.2%}")

    def calculate_analytics(self, df):
        """Generates the metrics shown on your dashboard UI."""
        print("Calculating Dashboard Analytics...")
        
        # Driver Scoring Logic
        df['driver_score'] = 100
        if 'speed' in df.columns:
            df.loc[df['speed'] > 120, 'driver_score'] -= 10
        if 'brake' in df.columns:
            df.loc[df['brake'] > 80, 'driver_score'] -= 5
            
        # Stress Modeling (Physics Approximation)
        if 'vibration' in df.columns and 'load' in df.columns:
            df['chassis_stress'] = (df['vibration'] * 0.7) + (df['load'] * 0.3)
            
        return df

    def save_models(self):
        os.makedirs('models', exist_ok=True)
        joblib.dump(self.scaler, 'models/scaler.pkl')
        joblib.dump(self.anomaly_detector, 'models/anomaly_model.pkl')
        print("Models saved to /models folder.")

if __name__ == "__main__":
    # Create Pipeline
    pipeline = VehicularMLPipeline()
    
    # Run Pipeline
    X_scaled, y_fault, df = pipeline.load_and_preprocess()
    
    if X_scaled is not None:
        pipeline.train_anomaly_detector(X_scaled)
        pipeline.train_fault_classifier(X_scaled, y_fault)
        df = pipeline.calculate_analytics(df)
        
        # Final Output
        print("\n--- PIPELINE SUMMARY ---")
        print(f"Total Rows Processed: {len(df)}")
        print(f"Average Fleet Score: {df['driver_score'].mean():.2f}")
        if 'chassis_stress' in df.columns:
            print(f"Max Chassis Stress Detected: {df['chassis_stress'].max():.2f}")
        
        pipeline.save_models()