# WIA-AGRI-008: Yield Prediction Standard
## PHASE 3: ML Protocol & Model Training Specification

**Version:** 1.0
**Status:** Active
**Category:** AGRI
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines the machine learning protocols, model training standards, and continuous improvement processes for agricultural yield prediction systems.

## 2. Machine Learning Pipeline

### 2.1 Data Preprocessing

**Data Cleaning Steps:**

1. **Outlier Detection:**
   ```python
   # Remove yields outside 3 standard deviations
   mean = data['yield'].mean()
   std = data['yield'].std()
   data = data[(data['yield'] > mean - 3*std) &
               (data['yield'] < mean + 3*std)]
   ```

2. **Missing Data Handling:**
   - Weather data: Interpolation or nearest station
   - Soil data: Regional averages
   - Input data: Previous year values or defaults
   - Maximum 10% missing data allowed per record

3. **Normalization:**
   - Min-Max scaling for continuous variables
   - Standard scaling for weather features
   - One-hot encoding for categorical variables

### 2.2 Feature Engineering

**Required Features:**

| Category | Features | Derivation |
|----------|----------|------------|
| Weather | Growing Degree Days (GDD) | Σ(T_max + T_min)/2 - T_base |
| Weather | Water Balance | Rainfall - Evapotranspiration |
| Weather | Heat Stress Days | Count(T_max > 35°C) |
| Temporal | Day of Year (planting) | Sine/Cosine encoding |
| Soil | Nutrient Balance | N+P+K composite score |
| Historical | 5-Year Avg Yield | Rolling mean |
| Spatial | NDVI | Satellite-derived vegetation index |

**Feature Engineering Example:**

```python
def calculate_gdd(temp_max, temp_min, base_temp=10):
    """Calculate Growing Degree Days"""
    return max(0, ((temp_max + temp_min) / 2) - base_temp)

def engineer_features(data):
    # GDD calculation
    data['gdd'] = data.apply(
        lambda x: calculate_gdd(x['temp_max'], x['temp_min']),
        axis=1
    )

    # Water balance
    data['water_balance'] = data['rainfall'] - data['et']

    # Temporal encoding
    data['planting_sin'] = np.sin(2 * np.pi * data['planting_doy'] / 365)
    data['planting_cos'] = np.cos(2 * np.pi * data['planting_doy'] / 365)

    # Historical yield trend
    data['yield_5yr_avg'] = data.groupby('farmId')['yield'].transform(
        lambda x: x.rolling(5, min_periods=1).mean()
    )

    return data
```

## 3. Model Algorithms

### 3.1 Ensemble Model (Recommended)

**Architecture:**

```
Ensemble Model
├── Random Forest (40% weight)
├── Gradient Boosting (30% weight)
├── LSTM Neural Network (20% weight)
└── Ridge Regression (10% weight)
```

**Hyperparameters:**

```python
ensemble_config = {
    'random_forest': {
        'n_estimators': 500,
        'max_depth': 15,
        'min_samples_split': 10,
        'min_samples_leaf': 5,
        'max_features': 'sqrt'
    },
    'gradient_boosting': {
        'n_estimators': 300,
        'learning_rate': 0.05,
        'max_depth': 8,
        'subsample': 0.8
    },
    'lstm': {
        'units': [128, 64, 32],
        'dropout': 0.2,
        'learning_rate': 0.001,
        'batch_size': 32,
        'epochs': 100
    },
    'ridge': {
        'alpha': 1.0,
        'normalize': True
    }
}
```

### 3.2 Random Forest

**Best for:** General-purpose, handles non-linear relationships

```python
from sklearn.ensemble import RandomForestRegressor

rf_model = RandomForestRegressor(
    n_estimators=500,
    max_depth=15,
    min_samples_split=10,
    min_samples_leaf=5,
    max_features='sqrt',
    random_state=42,
    n_jobs=-1
)

rf_model.fit(X_train, y_train)
```

**Feature Importance Analysis:**

```python
importances = rf_model.feature_importances_
feature_importance_df = pd.DataFrame({
    'feature': feature_names,
    'importance': importances
}).sort_values('importance', ascending=False)

# Top features typically:
# 1. Growing Degree Days (GDD)
# 2. Total Rainfall
# 3. Historical 5-year average
# 4. Soil quality index
# 5. Fertilizer nitrogen
```

### 3.3 Gradient Boosting (XGBoost)

**Best for:** High accuracy, resistant to overfitting

```python
import xgboost as xgb

xgb_model = xgb.XGBRegressor(
    n_estimators=300,
    learning_rate=0.05,
    max_depth=8,
    subsample=0.8,
    colsample_bytree=0.8,
    gamma=0.1,
    random_state=42
)

xgb_model.fit(X_train, y_train)
```

### 3.4 LSTM Neural Network

**Best for:** Time series patterns, seasonal trends

```python
from tensorflow.keras import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout

def build_lstm_model(input_shape):
    model = Sequential([
        LSTM(128, return_sequences=True, input_shape=input_shape),
        Dropout(0.2),
        LSTM(64, return_sequences=True),
        Dropout(0.2),
        LSTM(32, return_sequences=False),
        Dropout(0.2),
        Dense(16, activation='relu'),
        Dense(1)
    ])

    model.compile(
        optimizer='adam',
        loss='mse',
        metrics=['mae']
    )

    return model

# Prepare sequence data (e.g., 5 years history)
X_seq = create_sequences(X_train, sequence_length=5)
y_seq = y_train[sequence_length-1:]

lstm_model = build_lstm_model(input_shape=(5, n_features))
lstm_model.fit(X_seq, y_seq, epochs=100, batch_size=32, validation_split=0.2)
```

### 3.5 Linear Models (Baseline)

**Best for:** Interpretability, baseline comparison

```python
from sklearn.linear_model import Ridge

ridge_model = Ridge(alpha=1.0, normalize=True)
ridge_model.fit(X_train, y_train)
```

## 4. Model Training Protocol

### 4.1 Data Split Strategy

**Temporal Cross-Validation:**

```python
from sklearn.model_selection import TimeSeriesSplit

# Use temporal split to avoid data leakage
tscv = TimeSeriesSplit(n_splits=5)

for train_idx, val_idx in tscv.split(X):
    X_train, X_val = X[train_idx], X[val_idx]
    y_train, y_val = y[train_idx], y[val_idx]

    # Train and validate model
    model.fit(X_train, y_train)
    score = model.score(X_val, y_val)
```

**Recommended Split:**
- Training: 70% (oldest data)
- Validation: 15% (middle period)
- Test: 15% (most recent data)

### 4.2 Training Procedure

**Step 1: Initial Training**

```python
def train_yield_prediction_model(data, crop_type):
    # 1. Data preprocessing
    data_clean = preprocess_data(data)

    # 2. Feature engineering
    data_features = engineer_features(data_clean)

    # 3. Split data
    X_train, X_test, y_train, y_test = temporal_train_test_split(
        data_features, test_size=0.15
    )

    # 4. Train ensemble model
    ensemble = train_ensemble(X_train, y_train)

    # 5. Validate
    metrics = evaluate_model(ensemble, X_test, y_test)

    # 6. Save model
    save_model(ensemble, f'models/{crop_type}_v{version}.pkl')

    return ensemble, metrics
```

**Step 2: Hyperparameter Tuning**

```python
from sklearn.model_selection import GridSearchCV

param_grid = {
    'n_estimators': [300, 500, 700],
    'max_depth': [10, 15, 20],
    'min_samples_split': [5, 10, 15]
}

grid_search = GridSearchCV(
    RandomForestRegressor(),
    param_grid,
    cv=5,
    scoring='neg_mean_squared_error',
    n_jobs=-1
)

grid_search.fit(X_train, y_train)
best_model = grid_search.best_estimator_
```

### 4.3 Model Validation

**Required Metrics:**

```python
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

def evaluate_model(model, X_test, y_test):
    y_pred = model.predict(X_test)

    metrics = {
        'rmse': np.sqrt(mean_squared_error(y_test, y_pred)),
        'mae': mean_absolute_error(y_test, y_pred),
        'r2': r2_score(y_test, y_pred),
        'mape': np.mean(np.abs((y_test - y_pred) / y_test)) * 100
    }

    return metrics

# Acceptance criteria
# RMSE < 500 kg/ha for grains
# MAE < 300 kg/ha for grains
# R² > 0.85
# MAPE < 10%
```

**Cross-Validation:**

```python
from sklearn.model_selection import cross_val_score

cv_scores = cross_val_score(
    model, X, y,
    cv=5,
    scoring='neg_mean_squared_error'
)

print(f"CV RMSE: {np.sqrt(-cv_scores.mean()):.2f} ± {np.sqrt(cv_scores.std()):.2f}")
```

## 5. Model Update Protocol

### 5.1 Update Triggers

**Automatic Updates:**
1. New harvest season data available (>100 new records)
2. Model accuracy degradation (RMSE increase > 10%)
3. Scheduled annual retraining (January 15th each year)

**Manual Updates:**
4. Extreme weather event occurred
5. New crop variety introduced
6. Farming practice changes (e.g., new irrigation system)

### 5.2 Retraining Procedure

```python
def retrain_model(existing_model, new_data):
    # 1. Load existing model
    model = load_model(existing_model)

    # 2. Combine with historical data
    full_data = combine_data(historical_data, new_data)

    # 3. Preprocess and feature engineer
    processed_data = preprocess_pipeline(full_data)

    # 4. Retrain model
    model_new = train_model(processed_data)

    # 5. Compare performance
    old_score = evaluate_model(model, X_test, y_test)
    new_score = evaluate_model(model_new, X_test, y_test)

    # 6. Deploy if improved
    if new_score['rmse'] < old_score['rmse']:
        deploy_model(model_new)
        return True
    else:
        log_warning("New model did not improve performance")
        return False
```

### 5.3 A/B Testing

```python
def ab_test_models(model_a, model_b, test_data, duration_days=30):
    """
    Deploy both models in production and compare
    """
    results = {
        'model_a': [],
        'model_b': []
    }

    # Route 50% traffic to each model
    for request in incoming_requests:
        if random.random() < 0.5:
            pred = model_a.predict(request.features)
            results['model_a'].append({
                'prediction': pred,
                'actual': None  # Fill in after harvest
            })
        else:
            pred = model_b.predict(request.features)
            results['model_b'].append({
                'prediction': pred,
                'actual': None
            })

    # After duration, compare performance
    winner = compare_ab_results(results)
    return winner
```

## 6. Prediction Confidence Intervals

### 6.1 Confidence Calculation

```python
from sklearn.ensemble import GradientBoostingRegressor

# Train with quantile regression
quantile_models = {
    'lower': GradientBoostingRegressor(loss='quantile', alpha=0.025),
    'upper': GradientBoostingRegressor(loss='quantile', alpha=0.975),
    'median': GradientBoostingRegressor(loss='quantile', alpha=0.5)
}

for name, model in quantile_models.items():
    model.fit(X_train, y_train)

# Predictions with 95% confidence interval
pred_lower = quantile_models['lower'].predict(X_new)
pred_median = quantile_models['median'].predict(X_new)
pred_upper = quantile_models['upper'].predict(X_new)

print(f"Prediction: {pred_median:.0f} kg/ha")
print(f"95% CI: [{pred_lower:.0f}, {pred_upper:.0f}] kg/ha")
```

### 6.2 Uncertainty Quantification

```python
def calculate_prediction_uncertainty(models, X):
    """
    Use ensemble variance as uncertainty measure
    """
    predictions = [model.predict(X) for model in models]

    mean_pred = np.mean(predictions, axis=0)
    std_pred = np.std(predictions, axis=0)

    # Confidence based on standard deviation
    confidence = 1 - (std_pred / mean_pred).clip(0, 1)

    return mean_pred, confidence
```

## 7. Model Monitoring

### 7.1 Performance Tracking

```python
def monitor_model_performance(model, production_data):
    """
    Track model performance in production
    """
    metrics = {
        'timestamp': datetime.now(),
        'predictions_count': len(production_data),
        'avg_confidence': production_data['confidence'].mean(),
        'rmse': calculate_rmse(production_data),
        'bias': production_data['error'].mean(),
        'variance': production_data['error'].var()
    }

    # Alert if performance degrades
    if metrics['rmse'] > threshold:
        alert_model_degradation(metrics)

    # Log metrics
    log_metrics(metrics)

    return metrics
```

### 7.2 Data Drift Detection

```python
from scipy.stats import ks_2samp

def detect_data_drift(train_data, production_data, threshold=0.05):
    """
    Detect if production data distribution differs from training
    """
    drift_detected = {}

    for feature in train_data.columns:
        # Kolmogorov-Smirnov test
        statistic, p_value = ks_2samp(
            train_data[feature],
            production_data[feature]
        )

        drift_detected[feature] = p_value < threshold

    if any(drift_detected.values()):
        trigger_model_retraining("Data drift detected")

    return drift_detected
```

## 8. Model Interpretability

### 8.1 SHAP Values

```python
import shap

# Calculate SHAP values
explainer = shap.TreeExplainer(model)
shap_values = explainer.shap_values(X_test)

# Plot feature importance
shap.summary_plot(shap_values, X_test, feature_names=feature_names)

# Explain individual prediction
shap.force_plot(
    explainer.expected_value,
    shap_values[0],
    X_test.iloc[0]
)
```

### 8.2 Partial Dependence Plots

```python
from sklearn.inspection import partial_dependence, plot_partial_dependence

# Show effect of rainfall on yield prediction
plot_partial_dependence(
    model,
    X_train,
    features=['rainfall', 'temperature', 'gdd'],
    feature_names=feature_names
)
```

## 9. Certification Requirements

### 9.1 Model Certification Checklist

- [ ] Training data ≥ 500 records
- [ ] Temporal cross-validation performed
- [ ] RMSE < crop-specific threshold
- [ ] R² > 0.85
- [ ] Feature importance documented
- [ ] Confidence intervals provided
- [ ] Model versioning implemented
- [ ] Monitoring system active
- [ ] Documentation complete
- [ ] Security audit passed

### 9.2 Certification Levels

| Level | Requirements | RMSE | R² | Data Size |
|-------|-------------|------|-----|-----------|
| Bronze | Basic model | < 600 | > 0.80 | 500+ |
| Silver | Ensemble model | < 400 | > 0.85 | 1000+ |
| Gold | Advanced ensemble + monitoring | < 300 | > 0.90 | 2000+ |
| Platinum | Multi-region, real-time updates | < 200 | > 0.93 | 5000+ |

---

**License:** CC BY 4.0
**Maintained by:** WIA ML Committee
**Contact:** ml@wia.org
