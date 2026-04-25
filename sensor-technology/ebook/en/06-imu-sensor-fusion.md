# Chapter 6: IMU and Sensor Fusion

## Inertial Measurement Units (IMU)

An Inertial Measurement Unit combines multiple MEMS sensors to measure an object's specific force, angular rate, and sometimes magnetic field. IMUs are fundamental to navigation, robotics, drones, smartphones, and automotive systems.

### IMU Configurations

**6-axis IMU (6-DOF):**
- 3-axis accelerometer
- 3-axis gyroscope
- Measures: Linear acceleration + angular velocity
- Cannot determine absolute orientation (no magnetic reference)

**9-axis IMU (9-DOF):**
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer
- Full orientation estimation (pitch, roll, yaw/heading)
- Absolute compass heading

**10-axis IMU:**
- 9-axis + barometric pressure sensor
- Altitude/vertical positioning

### Commercial IMU Modules

**Bosch BNO085:**
- 9-axis sensor fusion
- Integrated Cortex-M0+ for sensor fusion algorithms
- Outputs: Quaternion, Euler angles, rotation vector
- Calibration: Dynamic auto-calibration
- Interfaces: I2C, SPI, UART
- Game rotation vector (no magnetometer drift)
- Accuracy: Heading ±2°, roll/pitch ±1°

**InvenSense ICM-42688-P:**
- 6-axis IMU (accel + gyro)
- Industry-leading gyro noise: 0.004 dps/√Hz
- Accelerometer noise: 70 μg/√Hz
- Programmable full-scale: ±2g to ±16g (accel), ±250 to ±2000 dps (gyro)
- ODR: Up to 32 kHz (!)
- FIFO: 2KB
- Power: 2.5 mA (both sensors @ 1 kHz)
- Applications: Premium smartphones, drones, robotics

**STMicroelectronics LSM6DSV16X:**
- 6-axis IMU
- Machine Learning Core (MLC)
- Finite State Machine (FSM) for pattern detection
- ISPU (Intelligent Sensor Processing Unit)
- Sensor fusion: qvar, MLC, FSM running on-chip
- Decision tree classifiers for activity recognition
- Power: 0.55 mA @ 1.8V, 104 Hz

**Analog Devices ADIS16505:**
- Tactical-grade IMU
- Gyro bias instability: 1.8°/hr (100x better than consumer)
- Accelerometer bias: 16 μg
- Temperature range: -40°C to +105°C
- SPI interface
- Self-test, condition monitoring
- Applications: Industrial robots, UAVs, antenna stabilization
- Cost: $500+ (vs. $5-10 for consumer IMU)

### Coordinate Systems and Conventions

**Body Frame (Sensor Frame):**
- X: Forward (nose direction)
- Y: Right (starboard)
- Z: Down (towards ground)
- Right-handed coordinate system

**Earth Frame (Navigation Frame):**
- X: North
- Y: East
- Z: Down (NED convention)

Alternative: ENU (East-North-Up)

**Euler Angles:**
- Roll (φ): Rotation about X-axis
- Pitch (θ): Rotation about Y-axis
- Yaw (ψ): Rotation about Z-axis

**Gimbal Lock:**
- Singularity when pitch = ±90°
- Roll and yaw become indistinguishable
- Solution: Use quaternions

**Quaternions:**
```
q = [q₀, q₁, q₂, q₃] = [w, x, y, z]

Where:
w = scalar part
(x, y, z) = vector part

Constraint: w² + x² + y² + z² = 1 (unit quaternion)
```

**Advantages:**
- No gimbal lock
- Compact (4 values vs. 9 in rotation matrix)
- Efficient interpolation (SLERP)
- Smooth continuous rotation

**Disadvantages:**
- Non-intuitive (hard to visualize)
- Requires conversion to Euler for display

**Rotation Matrix (DCM - Direction Cosine Matrix):**
```
R = [r11 r12 r13]
    [r21 r22 r23]
    [r31 r32 r33]

3×3 orthonormal matrix
9 elements, 6 constraints (orthonormality)
```

### Sensor Fusion Algorithms

#### Complementary Filter

**Concept:**
- High-pass filter gyroscope (integrate but drift over time)
- Low-pass filter accelerometer + magnetometer (noisy but drift-free)
- Combine: Best of both

**Implementation (Roll/Pitch):**
```
α = τ / (τ + Δt)

Where:
α = Filter coefficient (0.95-0.98 typical)
τ = Time constant (e.g., 5 seconds)
Δt = Sample period (e.g., 0.01 s @ 100 Hz)

θ_gyro = θ_prev + gyro_x × Δt
θ_accel = atan2(accel_y, accel_z)

θ_fused = α × θ_gyro + (1 - α) × θ_accel
```

**Advantages:**
- Simple, low computational cost
- Works well for slow-moving systems
- Tuning: Single parameter α

**Disadvantages:**
- Fixed time constant (not optimal for all conditions)
- Assumes constant linear acceleration
- No formal optimality

#### Kalman Filter

**Linear Kalman Filter:**

**State Equation:**
```
x_k = A × x_(k-1) + B × u_k + w_k

Where:
x_k = State vector at time k
A = State transition matrix
B = Control input matrix
u_k = Control input (gyro measurements)
w_k = Process noise (zero mean, covariance Q)
```

**Measurement Equation:**
```
z_k = H × x_k + v_k

Where:
z_k = Measurement vector (accel, mag)
H = Observation matrix
v_k = Measurement noise (zero mean, covariance R)
```

**Kalman Filter Steps:**

**Predict:**
```
x̂_k|k-1 = A × x̂_(k-1|k-1) + B × u_k
P_k|k-1 = A × P_(k-1|k-1) × A^T + Q
```

**Update:**
```
K_k = P_k|k-1 × H^T × (H × P_k|k-1 × H^T + R)^(-1)
x̂_k|k = x̂_k|k-1 + K_k × (z_k - H × x̂_k|k-1)
P_k|k = (I - K_k × H) × P_k|k-1
```

**Where:**
- x̂ = State estimate
- P = Error covariance matrix
- K = Kalman gain
- Q = Process noise covariance
- R = Measurement noise covariance

**Tuning:**
- Large Q: Trust gyro less, faster convergence to measurements
- Large R: Trust measurements less, smoother but slower response

**Extended Kalman Filter (EKF):**
- For non-linear systems (quaternion kinematics)
- Linearize around current estimate (Jacobian matrix)
- More complex but handles orientation estimation

**Unscented Kalman Filter (UKF):**
- Better handling of non-linearity
- Sigma points instead of linearization
- More accurate but computationally expensive

#### Madgwick Filter

**Gradient Descent Orientation Estimation:**

**Quaternion Derivative:**
```
q̇ = 0.5 × q ⊗ ω

Where:
q = Orientation quaternion
ω = Angular velocity vector [0, ω_x, ω_y, ω_z]
⊗ = Quaternion multiplication
```

**Objective Function:**
- Minimize error between measured gravity/magnetic field and predicted
- Gradient descent finds optimal quaternion update

**Algorithm:**
```
β = Gyro measurement error

q_ω = q + 0.5 × q ⊗ ω × Δt  (Gyro integration)
∇f = Jacobian^T × f(q, a, m)  (Error gradient)
q_∇ = q - β × (∇f / ||∇f||) × Δt  (Gradient descent)

q_fused = Normalize(q_ω + q_∇)
```

**Advantages:**
- Good accuracy
- Computationally efficient (no matrix inversions)
- Open-source implementation available
- Single tuning parameter β

**Disadvantages:**
- Assumes constant orientation or slow changes
- Less optimal than Kalman filter theoretically

**Mahony Filter:**
- Similar complementary approach
- Proportional-Integral (PI) controller
- Corrects gyro bias over time
- Tuning: Kp (proportional), Ki (integral gains)

### IMU Calibration

#### Accelerometer Calibration

**Six-Position Calibration:**

**Positions:**
1. X-axis up: [0, 0, +1g]
2. X-axis down: [0, 0, -1g]
3. Y-axis up: [0, +1g, 0]
4. Y-axis down: [0, -1g, 0]
5. Z-axis up: [+1g, 0, 0]
6. Z-axis down: [-1g, 0, 0]

**Calibration Model:**
```
a_true = S × (a_raw - b)

Where:
a_true = Calibrated acceleration
a_raw = Raw sensor reading
S = Scale factor matrix (3×3)
b = Bias offset vector [b_x, b_y, b_z]
```

**Solve for 9 parameters:**
- 3 biases (b_x, b_y, b_z)
- 3 scale factors (S_xx, S_yy, S_zz)
- 3 misalignment angles (off-diagonal S terms)

**Least-Squares Solution:**
- Collect measurements in all 6 positions
- Minimize ||S × (a_raw - b) - g_expected||²
- Iterative solver (Gauss-Newton, Levenberg-Marquardt)

**Temperature Calibration:**
- Measure bias drift vs. temperature
- Polynomial fit: b(T) = b₀ + b₁×T + b₂×T²
- Store coefficients in EEPROM

**Typical Accelerometer Errors:**
- Bias offset: ±40 mg (uncalibrated)
- Scale factor error: ±3%
- Misalignment: ±2°
- After calibration: Bias <5 mg, scale <0.5%

#### Gyroscope Calibration

**Static Bias Calibration:**
```
At rest: ω_true = 0
ω_raw = bias

Procedure:
1. Place IMU on stable surface
2. Collect 1000+ samples
3. bias = mean(ω_raw)
4. Store in non-volatile memory
```

**Temperature Compensation:**
- Gyro bias highly temperature-dependent
- Map bias across operating temperature range
- Typical drift: 0.03 dps/°C
- Store lookup table or polynomial coefficients

**Scale Factor Calibration:**
- Requires precision rate table (expensive equipment)
- Rotate at known angular velocity
- Solve: ω_true = S × (ω_raw - bias)
- Consumer IMUs: Factory calibrated, not user-adjustable

**G-Sensitivity Calibration:**
- Gyro output changes with linear acceleration
- Cross-coupling matrix [3×3]
- Requires simultaneous rotation + acceleration
- Advanced calibration only

#### Magnetometer Calibration (Hard/Soft Iron)

**Hard-Iron Distortion:**
- Constant magnetic field from permanent magnets (speakers, motors)
- Additive offset
- Correction: b = [b_x, b_y, b_z]

**Soft-Iron Distortion:**
- Ferromagnetic materials distort external field
- Scaling and rotation
- Correction: Scale and rotation matrix S

**Calibration Model:**
```
m_true = S × (m_raw - b)
```

**Figure-8 Calibration Procedure:**
1. Rotate device in all orientations
2. Collect magnetometer data (point cloud)
3. Ideal: Points lie on sphere (center = origin, radius = Earth's field)
4. Reality: Ellipsoid, offset center
5. Fit ellipsoid to data
6. Solve for b (center) and S (transform ellipsoid to sphere)

**Ellipsoid Fitting:**
- Non-linear least squares
- Minimize ||S × (m_raw - b)|| - ||Earth's field||²
- Typically requires 100+ data points covering full 3D space

**Earth's Magnetic Field:**
- Magnitude: 25-65 μT (location-dependent)
- Inclination: Angle from horizontal (0° at equator, ±90° at poles)
- Declination: Offset from true north (varies by location, changes over time)

**Local Disturbances:**
- Steel buildings, rebar in concrete
- High-voltage power lines
- Vehicles, machinery
- Solution: GPS/gyro-only heading when indoors

### Applications of IMU and Sensor Fusion

#### Smartphone Orientation

**Screen Rotation:**
- Accelerometer detects gravity direction
- Rotate UI when device tilted >45°
- Hysteresis to prevent rapid switching

**Gaming:**
- Tilt controls (racing games, marble maze)
- Absolute orientation (augmented reality)
- Motion gestures (shake to shuffle, flip to mute)

**Pedometer/Step Counting:**
- Analyze accelerometer for periodic patterns
- Peak detection in vertical axis
- Machine learning for walking/running classification

**Activity Recognition:**
- Walking, running, cycling, driving
- On-device ML (TensorFlow Lite)
- Training: Labeled datasets, feature extraction
- Inference: Real-time classification

#### Drone Stabilization and Navigation

**Attitude Estimation:**
- 400 Hz sensor fusion (gyro + accel + mag)
- Quaternion-based EKF or Madgwick filter
- Output: Roll, pitch, yaw for flight controller

**PID Control:**
```
Output = Kp × error + Ki × ∫error × dt + Kd × d(error)/dt

Example (Roll control):
error = roll_desired - roll_measured
motor_adjustment = PID(error)
```

**GPS/IMU Fusion:**
- GPS: Accurate position, low update rate (1-10 Hz), drift-free
- IMU: High update rate (100-1000 Hz), drifts over time
- Extended Kalman Filter combines both
- Dead-reckoning between GPS updates

**Vibration Isolation:**
- Soft-mount flight controller
- Digital filtering: Notch filter at propeller harmonics
- Typical: 90 Hz, 180 Hz, 270 Hz for 6-inch quad

#### Automotive Applications

**Electronic Stability Control (ESC):**
- Lateral accelerometer + yaw rate gyro
- Detect oversteer/understeer
- Brake individual wheels to correct

**Rollover Detection:**
- Z-axis gyro (roll rate)
- Lateral accelerometer
- Predict rollover, deploy side curtain airbags

**Adaptive Headlights:**
- Yaw rate gyro
- Steer headlight beam into turns
- Improve nighttime visibility

**Inertial Navigation (INS):**
- GPS outages (tunnels, urban canyons)
- Automotive-grade IMU (bias instability <1°/hr)
- Position drift: <100 m after 1 minute without GPS

**Lane Departure Warning:**
- Camera + IMU
- Predict vehicle path based on yaw rate
- Alert if crossing lane without signal

#### Robotics and AR/VR

**Robot Localization:**
- SLAM (Simultaneous Localization and Mapping)
- IMU provides motion prior for visual odometry
- Accelerometer detects collisions

**Head Tracking (VR):**
- Ultra-low latency required (<20 ms motion-to-photon)
- 1000 Hz IMU sampling
- Predictive tracking to compensate display lag
- Absolute position from external cameras (lighthouse, inside-out tracking)

**Augmented Reality:**
- Smartphone AR: IMU + camera (ARKit, ARCore)
- Overlay digital objects on real world
- IMU provides orientation, camera SLAM for position

**Gesture Recognition:**
- Wrist-worn IMU (smartwatch)
- Pattern matching: Swipe, rotate, tap
- On-device ML for efficiency

### Advanced Topics

#### Sensor Synchronization

**Time Stamping:**
- Each sensor has independent ADC, sample rate
- Timestamp each measurement
- Interpolate to common time base

**Hardware Synchronization:**
- Shared clock signal
- Simultaneous trigger (data-ready signal)
- Eliminates timing jitter

**Importance:**
- High-speed motion: 1 ms delay at 100 dps rotation = 0.1° error
- Multi-sensor fusion requires aligned timestamps

#### Vibration and Noise Rejection

**Aliasing:**
- Vibration frequency > Nyquist frequency (ODR/2)
- Appears as low-frequency noise
- Solution: Anti-aliasing filter (analog or digital)

**Mechanical Damping:**
- Isolate sensor from high-frequency vibration
- Soft mount on silicone, foam
- Trade-off: Phase lag, reduced bandwidth

**Digital Filtering:**
- Low-pass filter: Remove high-frequency noise
- Notch filter: Target specific frequencies (propeller RPM)
- Trade-off: Latency

**Sensor Bandwidth Selection:**
- Match sensor bandwidth to application
- Vibration analysis: Wide bandwidth (kHz)
- Orientation: 10-50 Hz sufficient
- Higher bandwidth = more noise

#### Zero-Velocity Update (ZUPT)

**Pedestrian Navigation:**
- Detect stationary periods (foot on ground during walking)
- Reset velocity error to zero
- Prevents unbounded drift in INS

**Detection:**
- Accelerometer variance < threshold (low motion)
- Gyroscope magnitude < threshold (not rotating)
- Typical: 100 ms stationary detection window

**Kalman Filter Update:**
- Measurement: Velocity = [0, 0, 0]
- Reset velocity state
- Reduce position drift from <10% of distance traveled

#### Adaptive Filtering

**Dynamic Noise Adjustment:**
- Increase measurement noise R when accelerometer senses non-gravitational acceleration (turning, accelerating vehicle)
- Rely more on gyro integration
- Prevents tilt error from lateral acceleration

**Magnetic Disturbance Rejection:**
- Monitor magnetometer magnitude deviation from Earth's field
- If deviation > threshold, reduce magnetometer weight
- Prevents heading error from local magnetic interference
- GPS-assisted heading when indoors

### Performance Metrics

**Heading Accuracy:**
- Gyro-only: Drift >10°/min (consumer MEMS)
- Accel + Gyro: Roll/pitch ±1-2°
- 9-axis (+ Mag): Yaw ±2-5° (outdoor, no interference)

**Update Rate:**
- Smartphones: 100-200 Hz typical
- Drones: 400-1000 Hz
- VR headsets: 1000 Hz
- Automotive safety: 100-200 Hz

**Latency:**
- Sensor readout: 1-5 ms
- Fusion algorithm: <1 ms
- Total: <10 ms (critical for VR)

**Power Consumption:**
- Low-power IMU: <1 mA @ 100 Hz
- High-performance: 2-5 mA
- Always-on motion detection: <50 μA

---

**References:**
- Woodman, O. J. (2007). *An Introduction to Inertial Navigation*. University of Cambridge.
- Madgwick, S. O. H. (2010). *An Efficient Orientation Filter for IMU and MARG Sensor Arrays*.
- Groves, P. D. (2013). *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*. Artech House.
- Bosch Sensortec: BNO055/BNO085 Datasheets and Application Notes
- InvenSense: ICM-20948 Design Guide

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
