# Chapter 5: Pressure Sensing and Multi-Level Force Detection

## Calibration, Threshold Optimization, and Force Curve Management

### Introduction

Accurate pressure sensing is the foundation of effective 3D Touch interactions. This chapter explores multi-level force detection, calibration algorithms, hysteresis management, and techniques for creating consistent force-based experiences across diverse users, devices, and environmental conditions.

---

## Multi-Level Force Detection

### Force Level Taxonomy

**Three-Level Model (Apple 3D Touch):**

**1. Light Touch** (Hover/Resting Touch)
- **Force range**: 0-50 gf (0-0.49 N)
- **User intent**: Exploring, hovering, accidental contact
- **System response**: No action or preview hint
- **Implementation**: Standard capacitive touch detection

**2. Normal Touch/Tap**
- **Force range**: 50-150 gf (0.49-1.47 N)
- **User intent**: Standard selection, scrolling, typing
- **System response**: Standard touch events (tap, swipe, etc.)
- **Implementation**: Traditional touch event handling

**3. Peek Press** (Light Force)
- **Force range**: 150-300 gf (1.47-2.94 N)
- **User intent**: Preview content without commitment
- **System response**: Preview overlay, quick actions
- **Implementation**: Force threshold detection + haptic feedback
- **Typical threshold**: 200 gf (varies by device, user preference)

**4. Pop Press** (Deep Force)
- **Force range**: 300-500 gf (2.94-4.90 N)
- **User intent**: Full content activation, strong commitment
- **System response**: Open full content, execute action
- **Implementation**: Higher force threshold + distinct haptic
- **Typical threshold**: 400 gf

**5. Maximum Force**
- **Force range**: 500+ gf (>4.90 N)
- **User intent**: Rarely intentional, often accidental
- **System response**: Limit to Pop action (no additional response)
- **Implementation**: Saturation at maximum force

### Force Resolution

**Quantization:**
Force sensors output discrete levels, not continuous values

**Typical resolutions:**
- **Low resolution**: 256 levels (8-bit)
- **Medium resolution**: 1024 levels (10-bit) - Apple 3D Touch
- **High resolution**: 4096 levels (12-bit)
- **Ultra-high resolution**: 16384 levels (14-bit) - research systems

**Resolution vs. Range trade-off:**

```
Force Resolution = Maximum Force / Number of Levels

Example (1024 levels, 500g max):
Force Resolution = 500g / 1024 = 0.488 gf/level ≈ 0.5 gf/level
```

**Perceptual threshold:**
- Humans can distinguish ~5-10 gf force differences in touch interactions
- 0.5 gf resolution exceeds human perception (oversampled)
- Allows for software filtering and noise reduction

### Threshold Detection Algorithm

**Basic hysteresis-based detection:**

```python
class ForceThresholdDetector:
    def __init__(self, peek_threshold=200, pop_threshold=400, hysteresis=10):
        self.peek_threshold = peek_threshold  # gf
        self.pop_threshold = pop_threshold
        self.hysteresis = hysteresis  # gf (prevent flicker)
        
        self.state = "NONE"  # States: NONE, PEEK, POP
        
    def update(self, force_gf):
        if self.state == "NONE":
            if force_gf >= self.peek_threshold:
                self.state = "PEEK"
                self.trigger_peek_gesture()
                return "PEEK_ENTERED"
                
        elif self.state == "PEEK":
            if force_gf >= self.pop_threshold:
                self.state = "POP"
                self.trigger_pop_gesture()
                return "POP_ENTERED"
            elif force_gf < (self.peek_threshold - self.hysteresis):
                self.state = "NONE"
                self.cancel_peek_gesture()
                return "PEEK_EXITED"
                
        elif self.state == "POP":
            if force_gf < (self.pop_threshold - self.hysteresis):
                self.state = "PEEK"
                return "POP_EXITED"  # Rare, user reduces force
        
        return "NO_CHANGE"
```

**Hysteresis importance:**
- Prevents rapid state transitions near threshold
- Reduces haptic feedback flicker
- Typical hysteresis: 5-15% of threshold value

---

## Calibration Algorithms

### Factory Calibration Process

**Equipment required:**
- **Force applicator**: Pneumatic or servo-controlled actuator
- **Reference force sensor**: 0.1% accuracy or better (e.g., load cell)
- **Environmental chamber**: Temperature control ±1°C
- **Automated test fixture**: Applies force to multiple screen positions

**Calibration procedure:**

**1. Device Mounting**
- Secure device in test fixture
- Ensure consistent mounting (affects force distribution)

**2. Baseline Measurement (Zero Force)**
- Record all force sensor channel values with no applied force
- Ambient conditions: 25°C, 50% RH
- Average 100 samples to reduce noise

```python
baseline = {
    'sensor_1': mean(samples_1),
    'sensor_2': mean(samples_2),
    # ... for all sensors
}
```

**3. Multi-Point Calibration**
Apply calibration forces at standardized points

**Typical calibration forces:**
- 50 gf (light touch reference)
- 150 gf (normal tap reference)
- 250 gf (peek threshold reference)
- 400 gf (pop threshold reference)
- 500 gf (maximum force reference)

**For each calibration point:**
```python
for force_gf in [50, 150, 250, 400, 500]:
    actuator.apply_force(force_gf)
    sleep(100_ms)  # Allow settling
    
    sensor_readings = []
    for i in range(100):
        sensor_readings.append(read_all_force_sensors())
        sleep(1_ms)
    
    calibration_data[force_gf] = {
        'sensor_1': mean([r.sensor_1 for r in sensor_readings]),
        'sensor_2': mean([r.sensor_2 for r in sensor_readings]),
        # ... for all sensors
    }
    
    actuator.release_force()
    sleep(500_ms)  # Full release
```

**4. Spatial Calibration**
Repeat calibration at multiple screen positions

**Test positions:**
- Center
- Four corners
- Mid-points of each edge
- Additional grid points for high-precision devices

**Total calibration points:** 5 force levels × 9 spatial positions = 45 measurements minimum

**5. Temperature Calibration**
Repeat process at multiple temperatures

**Temperature points:**
- 0°C (cold environment)
- 25°C (room temperature baseline)
- 40°C (warm pocket/hand)
- 60°C (hot car dashboard - extreme)

**6. Calibration Curve Fitting**

**Approach 1: Polynomial Fit (2nd or 3rd order)**

```python
from numpy import polyfit, polyval

# Fit polynomial: Force = a0 + a1*C + a2*C^2 + a3*C^3
capacitances = [baseline_C, C_50g, C_150g, C_250g, C_400g, C_500g]
forces = [0, 50, 150, 250, 400, 500]

coefficients = polyfit(capacitances, forces, deg=3)  # 3rd-order polynomial

def capacitance_to_force(C):
    return polyval(coefficients, C)
```

**Approach 2: Piecewise Linear (more robust)**

```python
calibration_points = [
    (C_0g, 0),
    (C_50g, 50),
    (C_150g, 150),
    (C_250g, 250),
    (C_400g, 400),
    (C_500g, 500)
]

def piecewise_linear_force(C):
    for i in range(len(calibration_points) - 1):
        C_low, F_low = calibration_points[i]
        C_high, F_high = calibration_points[i+1]
        
        if C_low <= C <= C_high:
            # Linear interpolation between points
            slope = (F_high - F_low) / (C_high - C_low)
            force = F_low + slope * (C - C_low)
            return force
    
    # Extrapolation beyond calibration range
    if C < calibration_points[0][0]:
        return 0  # Clamp to zero
    else:
        return 500  # Clamp to maximum
```

**Approach 3: Lookup Table (LUT)**

For embedded systems with limited floating-point performance:

```c
// Pre-computed lookup table
const uint16_t force_lut[256] = {
    0, 2, 5, 8, 12, 16, 20, 25, ...  // Force in gf for each sensor value
};

uint16_t sensor_to_force(uint8_t sensor_value) {
    return force_lut[sensor_value];
}
```

**7. Store Calibration Data**
Write coefficients or LUT to non-volatile storage

**Storage locations:**
- Device EEPROM
- Flash memory
- Secure element (for tamper resistance)

**Data structure:**
```json
{
    "calibration_version": "1.0",
    "device_serial": "ABC123456",
    "calibration_date": "2025-01-15",
    "temperature_reference": 25.0,
    "coefficients": {
        "sensor_1": [a0, a1, a2, a3],
        "sensor_2": [a0, a1, a2, a3]
    },
    "temperature_compensation": {
        "coefficient_ppm_per_C": -200
    }
}
```

### Temperature Compensation

**Challenge:**
Force sensors drift with temperature due to:
- Dielectric material thermal expansion/contraction
- Electrode thermal expansion
- Elastic modulus changes

**Typical drift:** -0.2% to -0.5% per °C (capacitive force sensors)

**Compensation approach:**

```python
def temperature_compensated_force(raw_force, temp_celsius, cal_temp=25.0):
    # Measured temperature coefficient
    temp_coeff = -0.003  # -0.3% per °C
    
    # Apply compensation
    temp_delta = temp_celsius - cal_temp
    compensation_factor = 1 + (temp_coeff * temp_delta)
    
    compensated_force = raw_force * compensation_factor
    return compensated_force
```

**Example:**
```
Calibrated at 25°C: 200 gf applied → sensor reads "200 gf"
Device heats to 40°C:
  - Raw reading: 191 gf (drift down)
  - Temperature compensation: 191 / (1 + (-0.003 * 15)) = 191 / 0.955 = 200 gf
  - Corrected reading: 200 gf ✓
```

### Runtime Calibration

**Auto-Zero Calibration:**
Periodically recalibrate baseline (zero force) when device idle

**Trigger conditions:**
- Screen off for >5 seconds
- No touch detected for >10 seconds
- Device stationary (accelerometer confirms)
- Battery temperature stable (not charging/discharging rapidly)

**Procedure:**
```python
def auto_zero_calibration():
    if is_calibration_safe():
        # Measure current baseline
        new_baseline = mean(read_force_sensors(samples=100))
        
        # Update stored baseline
        if abs(new_baseline - stored_baseline) < MAX_DRIFT:
            stored_baseline = new_baseline
            save_to_nvram(stored_baseline)
        else:
            log_warning("Excessive baseline drift detected")
```

**User-Initiated Calibration:**
Settings option to re-calibrate force sensitivity

**UI:**
- "Force Touch Sensitivity" slider
- Options: Light, Medium, Firm
- Adjusts force thresholds (not sensor calibration)

```python
sensitivity_multipliers = {
    'light': 0.7,    # Lower thresholds (easier to trigger)
    'medium': 1.0,   # Default
    'firm': 1.4      # Higher thresholds (harder to trigger)
}

peek_threshold = base_peek_threshold * sensitivity_multipliers[user_preference]
pop_threshold = base_pop_threshold * sensitivity_multipliers[user_preference]
```

---

## Hysteresis Management

### Sources of Hysteresis

**1. Material Hysteresis**
Compliant dielectric in capacitive force sensor exhibits viscoelastic behavior

**Effect:**
- Loading curve (increasing force) differs from unloading curve
- Measured capacitance higher when force decreasing vs. increasing
- Hysteresis error: 5-15% of applied force typical

**2. Mechanical Hysteresis**
Display stack, adhesive layers, gaskets have mechanical hysteresis

**3. Sensor Hysteresis**
Strain gauges, piezoelectric materials have inherent hysteresis

### Hysteresis Compensation

**Approach 1: Separate Calibration Curves**

Calibrate loading (increasing force) and unloading (decreasing force) independently

```python
class HysteresisCompensator:
    def __init__(self):
        self.loading_curve = load_calibration("loading_curve.json")
        self.unloading_curve = load_calibration("unloading_curve.json")
        self.last_force = 0
        self.direction = 0  # 1 = increasing, -1 = decreasing
    
    def get_force(self, raw_sensor_value):
        if raw_sensor_value > self.last_raw:
            # Force increasing - use loading curve
            force = self.loading_curve.evaluate(raw_sensor_value)
            self.direction = 1
        else:
            # Force decreasing - use unloading curve
            force = self.unloading_curve.evaluate(raw_sensor_value)
            self.direction = -1
        
        self.last_raw = raw_sensor_value
        self.last_force = force
        return force
```

**Approach 2: Kalman Filtering**

Use Kalman filter to estimate true force considering hysteresis model

```python
class ForceKalmanFilter:
    def __init__(self):
        # State: [force, force_rate]
        self.state = np.array([0.0, 0.0])
        self.P = np.eye(2) * 10  # Covariance matrix
        
        # Process noise (how much force can change per sample)
        self.Q = np.array([[1.0, 0], [0, 10.0]])
        
        # Measurement noise (sensor noise + hysteresis uncertainty)
        self.R = np.array([[25.0]])  # 5 gf RMS noise
    
    def predict(self, dt):
        # State transition: force_{k} = force_{k-1} + force_rate * dt
        F = np.array([[1, dt], [0, 1]])
        
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, measured_force):
        # Measurement matrix (we measure force directly)
        H = np.array([[1, 0]])
        
        # Innovation
        y = measured_force - (H @ self.state)[0]
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T / S
        
        # Update state
        self.state = self.state + K * y
        self.P = (np.eye(2) - K @ H) @ self.P
        
        return self.state[0]  # Return estimated force
```

**Benefits:**
- Smooths force readings
- Reduces hysteresis-induced errors
- Accounts for measurement noise

**Approach 3: Machine Learning**

Train neural network to predict true force from:
- Current sensor reading
- Previous N sensor readings (history)
- Force change rate
- Touch area (from capacitive touch)

```python
# Simplified ML model (trained offline)
class MLForceEstimator:
    def __init__(self, model_path):
        self.model = load_trained_model(model_path)
        self.history = []
    
    def estimate_force(self, raw_sensor, touch_area):
        # Feature vector: [current, prev_1, prev_2, ..., prev_10, touch_area]
        self.history.append(raw_sensor)
        if len(self.history) > 10:
            self.history.pop(0)
        
        features = self.history + [touch_area]
        force = self.model.predict(features)
        return force
```

---

## Adaptive Force Thresholds

### User-Specific Adaptation

**Challenge:**
Different users apply force differently:
- **Light touchers**: 50-200 gf typical force range
- **Heavy touchers**: 200-600 gf typical force range
- **Children**: Lower force capacity
- **Elderly**: May have difficulty applying high force

**Solution: Adaptive Thresholds**

Learn user's force application pattern over time

```python
class AdaptiveThresholds:
    def __init__(self):
        self.force_history = []
        self.peek_threshold = 200  # Initial default
        self.pop_threshold = 400
        
    def record_force_event(self, force_gf, event_type):
        self.force_history.append({'force': force_gf, 'type': event_type})
        
        # Keep last 1000 events
        if len(self.force_history) > 1000:
            self.force_history.pop(0)
        
        # Periodically update thresholds
        if len(self.force_history) % 100 == 0:
            self.update_thresholds()
    
    def update_thresholds(self):
        # Analyze force distribution
        forces = [e['force'] for e in self.force_history if e['type'] in ['peek', 'pop']]
        
        if len(forces) < 50:
            return  # Insufficient data
        
        # Calculate percentiles
        p25 = np.percentile(forces, 25)
        p75 = np.percentile(forces, 75)
        
        # Adapt thresholds to user's natural force range
        self.peek_threshold = p25 * 1.2  # Slightly above 25th percentile
        self.pop_threshold = p75 * 0.9   # Slightly below 75th percentile
        
        # Clamp to reasonable ranges
        self.peek_threshold = np.clip(self.peek_threshold, 100, 300)
        self.pop_threshold = np.clip(self.pop_threshold, 250, 500)
```

### Context-Aware Thresholds

**Scenario 1: One-handed vs. Two-handed Use**

Detected via:
- Touch area (one-handed: thumb, smaller area)
- Gyroscope (device orientation, holding pattern)

```python
def adjust_for_context(base_threshold, context):
    if context == 'one_handed_thumb':
        # Thumb has less force control, lower threshold
        return base_threshold * 0.8
    elif context == 'two_handed_index':
        # Index finger has good control, standard threshold
        return base_threshold * 1.0
    elif context == 'stylus':
        # Stylus: precise, can use higher threshold
        return base_threshold * 1.2
    return base_threshold
```

**Scenario 2: Motion Context**

Device motion affects force application accuracy

```python
def motion_adjusted_threshold(base_threshold, motion_state):
    if motion_state == 'stationary':
        return base_threshold  # Normal threshold
    elif motion_state == 'walking':
        # Walking: reduce sensitivity to avoid false triggers
        return base_threshold * 1.3
    elif motion_state == 'running':
        # Running: significant reduction
        return base_threshold * 1.6
    return base_threshold
```

---

## Force Gesture Timeout

**Challenge:**
User applies force but doesn't complete gesture (e.g., Peek but not Pop)

**Solution: Time-based State Machine**

```python
class ForceGestureStateMachine:
    def __init__(self):
        self.state = 'IDLE'
        self.peek_entry_time = None
        self.PEEK_TIMEOUT = 2000  # ms
    
    def update(self, force_gf, timestamp_ms):
        if self.state == 'IDLE':
            if force_gf >= PEEK_THRESHOLD:
                self.state = 'PEEK_ACTIVE'
                self.peek_entry_time = timestamp_ms
                show_peek_preview()
        
        elif self.state == 'PEEK_ACTIVE':
            if force_gf >= POP_THRESHOLD:
                self.state = 'POP_ACTIVE'
                activate_pop_action()
            elif force_gf < PEEK_THRESHOLD - HYSTERESIS:
                self.state = 'IDLE'
                hide_peek_preview()
            elif (timestamp_ms - self.peek_entry_time) > self.PEEK_TIMEOUT:
                # User held Peek too long, auto-transition to Pop
                self.state = 'POP_ACTIVE'
                activate_pop_action()
        
        elif self.state == 'POP_ACTIVE':
            if force_gf < POP_THRESHOLD - HYSTERESIS:
                self.state = 'IDLE'
                complete_pop_action()
```

---

## Multi-Touch Force Challenges

**Problem:**
Two or more simultaneous touches create ambiguous force sensor readings

**Scenario:**
- User touches screen at position (x1, y1) with force F1
- User simultaneously touches at (x2, y2) with force F2
- Force sensors may measure combined force (F1 + F2)

**Solution Approaches:**

**1. High-Density Force Sensor Grid**
- Spatial resolution sufficient to distinguish touches (e.g., 10mm grid)
- Assign force based on nearest sensor to touch position

**2. Force Splitting Algorithm**

Based on touch area and position, estimate force contribution

```python
def split_multi_touch_force(touches, total_force):
    # touches = [(x1, y1, area1), (x2, y2, area2)]
    
    # Assume force proportional to contact area
    total_area = sum([t[2] for t in touches])
    
    forces = []
    for (x, y, area) in touches:
        force_fraction = area / total_area
        force = total_force * force_fraction
        forces.append(force)
    
    return forces  # [F1, F2, ...]
```

**3. Temporal Separation**

If touches occur at slightly different times, track forces individually

```python
class MultiTouchForceTracker:
    def __init__(self):
        self.touch_forces = {}  # touch_id -> force
    
    def update(self, touches, total_force):
        current_touch_ids = set([t.id for t in touches])
        
        # New touch
        for touch in touches:
            if touch.id not in self.touch_forces:
                # Assign incremental force
                previous_total = sum(self.touch_forces.values())
                self.touch_forces[touch.id] = total_force - previous_total
        
        # Removed touch
        removed_ids = set(self.touch_forces.keys()) - current_touch_ids
        for touch_id in removed_ids:
            del self.touch_forces[touch_id]
        
        return self.touch_forces
```

---

## Performance Metrics

### Force Sensing Accuracy

**Specification:**
- **Absolute accuracy**: ±2-5% of full scale (e.g., ±10 gf at 200 gf)
- **Repeatability**: ±1-2% (same force applied multiple times)
- **Linearity**: ±2-3% (deviation from ideal straight line)

**Testing:**
- Apply known forces with calibrated load cell
- Measure sensor response
- Calculate error metrics

### Force Resolution

**Specification:**
- **Minimum detectable force**: 0.1-1 gf
- **Force quantization**: 0.5-2 gf per level
- **Dynamic range**: 60-70 dB (max force / min force)

### Latency

**Specification:**
- **Force detection latency**: <8 ms from force application to measurement
- **Total response latency**: <20 ms (including haptic feedback)

**Measurement:**
- High-speed camera (1000 fps) captures force application
- Measure time to haptic response
- Analyze frame-by-frame

---

## Conclusion

Accurate pressure sensing requires careful calibration, hysteresis management, and adaptive thresholds to accommodate varied users and usage contexts. Multi-point factory calibration, temperature compensation, and runtime auto-zero ensure consistent force measurement over the device lifetime and environmental conditions.

**Key techniques:**

1. **Multi-point calibration**: 5+ force levels, multiple temperatures
2. **Hysteresis compensation**: Separate loading/unloading curves or Kalman filtering
3. **Adaptive thresholds**: Learn user's force patterns, adjust to context
4. **Multi-touch handling**: Spatial resolution or force splitting algorithms

**Next Chapter Preview:** Chapter 6 explores force gesture recognition, including Peek and Pop implementations, force-based long press, pressure-velocity analysis, and designing intuitive force-based interaction patterns.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
