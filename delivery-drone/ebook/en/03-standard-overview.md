# Chapter 3: Standard Architecture and Framework

## WIA-AUTO-017 Design Philosophy, Classifications, and System Requirements

---

## 3.1 WIA Standard Design Philosophy

### Core Principles

The WIA-AUTO-017 standard is built on five foundational principles that guide every technical decision:

**1. Safety First**

Every specification prioritizes safety over efficiency or cost:
- Redundant systems for critical functions
- Fail-safe behaviors in all failure modes
- Conservative operational limits
- Comprehensive emergency procedures

**2. Interoperability**

Systems from different manufacturers must work together:
- Common data formats and APIs
- Standard communication protocols
- Compatible UTM integration
- Shared safety conventions

**3. Scalability**

The standard supports operations from single-drone pilots to continent-spanning fleets:
- Modular architecture
- Hierarchical management
- Performance tiers
- Growth-oriented design

**4. Accessibility**

The philosophy of 弘益人間 demands technology that benefits everyone:
- Open standard with no licensing fees
- Reference implementations available
- Support for diverse operational contexts
- Rural and underserved area consideration

**5. Sustainability**

Environmental responsibility is integrated throughout:
- Energy efficiency optimization
- Battery lifecycle management
- Noise minimization
- End-of-life recycling guidelines

### Standard Structure

```
WIA-AUTO-017 Standard Architecture:

┌────────────────────────────────────────────────────────────────┐
│                     Application Layer                          │
│  (Delivery Management, Fleet Operations, Customer Interface)   │
├────────────────────────────────────────────────────────────────┤
│                     API Layer (Phase 2)                        │
│  (REST APIs, WebSocket, SDKs)                                  │
├────────────────────────────────────────────────────────────────┤
│                     Protocol Layer (Phase 3)                   │
│  (Flight Control, Navigation, Communication)                   │
├────────────────────────────────────────────────────────────────┤
│                     Data Format Layer (Phase 1)                │
│  (Messages, Waypoints, Telemetry, Flight Logs)                 │
├────────────────────────────────────────────────────────────────┤
│                     Integration Layer (Phase 4)                │
│  (UTM, Regulatory, Ground Systems)                             │
└────────────────────────────────────────────────────────────────┘
```

---

## 3.2 Drone Classification System

### Weight-Based Classification

The standard defines four weight classes based on Maximum Takeoff Weight (MTOW):

#### Micro Class (0-2 kg)

```yaml
micro_class:
  mtow_range: "0-2 kg"
  payload_capacity: "0.1-0.5 kg"
  typical_range: "1-3 km"
  flight_time: "10-20 min"
  max_speed: "15 m/s"
  use_cases:
    - Documents and small mail
    - Lightweight medications
    - Keys and small items
    - Urban short-distance delivery
  regulatory_category: "Low risk"
  certification: "Self-declaration in most jurisdictions"
```

**Technical Specifications**:

| Parameter | Minimum | Typical | Maximum |
|-----------|---------|---------|---------|
| Motor count | 4 | 4 | 6 |
| Motor power | 30W | 50W | 100W |
| Battery capacity | 1,000 mAh | 2,500 mAh | 4,000 mAh |
| GPS accuracy | 3m | 1m | 0.5m |
| Wind tolerance | 5 m/s | 8 m/s | 10 m/s |

#### Light Class (2-10 kg)

```yaml
light_class:
  mtow_range: "2-10 kg"
  payload_capacity: "0.5-3 kg"
  typical_range: "3-10 km"
  flight_time: "20-35 min"
  max_speed: "20 m/s"
  use_cases:
    - E-commerce packages
    - Food delivery
    - Pharmacy deliveries
    - Small electronics
  regulatory_category: "Medium risk"
  certification: "Specific authorization required"
```

**Technical Specifications**:

| Parameter | Minimum | Typical | Maximum |
|-----------|---------|---------|---------|
| Motor count | 4 | 6 | 8 |
| Motor power | 100W | 200W | 400W |
| Battery capacity | 5,000 mAh | 10,000 mAh | 16,000 mAh |
| GPS accuracy | 2m | 0.5m | 0.1m (RTK) |
| Wind tolerance | 8 m/s | 10 m/s | 12 m/s |

#### Medium Class (10-25 kg)

```yaml
medium_class:
  mtow_range: "10-25 kg"
  payload_capacity: "3-8 kg"
  typical_range: "10-30 km"
  flight_time: "30-50 min"
  max_speed: "25 m/s"
  use_cases:
    - Heavy packages
    - Grocery deliveries
    - Medical supplies
    - Emergency equipment
  regulatory_category: "Higher risk"
  certification: "Full type certification recommended"
```

**Technical Specifications**:

| Parameter | Minimum | Typical | Maximum |
|-----------|---------|---------|---------|
| Motor count | 6 | 8 | 8 |
| Motor power | 400W | 600W | 1000W |
| Battery capacity | 16,000 mAh | 30,000 mAh | 50,000 mAh |
| GPS accuracy | 1m | 0.2m | 0.05m (RTK) |
| Wind tolerance | 10 m/s | 12 m/s | 15 m/s |

#### Heavy Class (25-150 kg)

```yaml
heavy_class:
  mtow_range: "25-150 kg"
  payload_capacity: "8-50 kg"
  typical_range: "30-100 km"
  flight_time: "45-90 min"
  max_speed: "30 m/s"
  use_cases:
    - Large cargo
    - Disaster relief supplies
    - Rural area delivery
    - Industrial equipment
  regulatory_category: "Highest risk"
  certification: "Full type and production certification required"
  parachute: "MANDATORY"
```

### Capability Tiers

Beyond weight, drones are classified by capability:

| Tier | Autonomy | BVLOS | Weather | Night |
|------|----------|-------|---------|-------|
| Basic | Manual/assisted | No | Clear only | No |
| Standard | Waypoint navigation | Limited | Light conditions | With lighting |
| Advanced | Full autonomous | Yes | Moderate conditions | Full capability |
| Enterprise | Multi-vehicle coordination | Yes | Adverse conditions | Full capability |

---

## 3.3 Propulsion Systems

### Multi-Rotor Configurations

```
Quadcopter (4 motors):          Hexacopter (6 motors):
     M1    M2                    M1      M2
       \  /                        \    /
        \/                          \  /
        /\                      M6---()---M3
       /  \                         /  \
     M3    M4                      /    \
                                 M5      M4

Redundancy: None                Redundancy: Can fly with 1 motor failed
Best for: Micro/Light class     Best for: Light/Medium class


Octocopter (8 motors):          Coaxial Octo (8 motors, 4 arms):
  M1    M2                           M1/M5
    \  /                               |
 M8--\/--M3                        M8/M4---M2/M6
    /\                                 |
  M7    M4                           M7/M3
    \  /
 M6--\/--M5

Redundancy: Can fly with 2 motors failed
Best for: Medium/Heavy class
```

### Motor Specifications

```python
class MotorSpecification:
    """Standard motor specification for delivery drones."""

    def __init__(self, kv_rating, max_current, efficiency):
        self.kv = kv_rating  # RPM per volt
        self.max_current = max_current  # Amps
        self.efficiency = efficiency  # 0-1

    def calculate_thrust(self, voltage, propeller_pitch, propeller_diameter):
        """
        Estimate thrust based on motor and propeller specifications.
        Simplified model for planning purposes.
        """
        rpm = self.kv * voltage
        # Thrust coefficient approximation
        ct = 0.012 * (propeller_pitch / propeller_diameter)
        # Air density at sea level
        rho = 1.225
        # Propeller disk area
        area = 3.14159 * (propeller_diameter / 2) ** 2

        # Simplified thrust calculation
        thrust = ct * rho * area * (rpm / 60 * propeller_pitch) ** 2

        return thrust * self.efficiency

# Example: Typical medium-class motor
motor = MotorSpecification(kv_rating=320, max_current=40, efficiency=0.85)
thrust = motor.calculate_thrust(voltage=50, propeller_pitch=0.15, propeller_diameter=0.38)
print(f"Estimated thrust: {thrust:.1f} N")
```

### Propulsion System Requirements

| Parameter | Micro | Light | Medium | Heavy |
|-----------|-------|-------|--------|-------|
| Thrust/Weight ratio | >1.5 | >1.8 | >2.0 | >2.2 |
| Motor redundancy | Optional | Recommended | Required | Required |
| ESC redundancy | No | Optional | Recommended | Required |
| Propeller type | Fixed | Folding | Folding | Folding |

---

## 3.4 Power Systems

### Battery Specifications

#### Lithium-Polymer (LiPo) Requirements

```yaml
battery_requirements:
  chemistry: "Lithium Polymer (LiPo)"
  nominal_voltage: "3.7V per cell"
  max_charge_voltage: "4.2V per cell"
  min_discharge_voltage: "3.3V per cell"
  c_rating_minimum: "10C continuous discharge"
  temperature_range:
    storage: "-20°C to 45°C"
    charge: "5°C to 45°C"
    discharge: "0°C to 55°C"
  cycle_life: ">300 cycles to 80% capacity"
  protection:
    - overcurrent
    - overvoltage
    - undervoltage
    - overtemperature
    - short_circuit
```

#### Battery Sizing

```python
def size_battery(flight_time_min, hover_power_w, cruise_power_w,
                 hover_ratio=0.3, reserve_ratio=0.25):
    """
    Calculate required battery capacity for delivery mission.

    Args:
        flight_time_min: Target flight time in minutes
        hover_power_w: Power consumption while hovering
        cruise_power_w: Power consumption while cruising
        hover_ratio: Portion of flight spent hovering (0-1)
        reserve_ratio: Safety reserve (0-1)
    """
    # Average power consumption
    avg_power = hover_ratio * hover_power_w + (1 - hover_ratio) * cruise_power_w

    # Energy required (Wh)
    energy_required = avg_power * (flight_time_min / 60)

    # Add reserve
    total_energy = energy_required / (1 - reserve_ratio)

    # Typical battery voltage (6S = 22.2V nominal)
    voltage = 22.2

    # Required capacity (Ah)
    capacity_ah = total_energy / voltage

    return {
        "energy_wh": total_energy,
        "capacity_ah": capacity_ah,
        "capacity_mah": capacity_ah * 1000
    }

# Example: Light class drone
result = size_battery(
    flight_time_min=30,
    hover_power_w=600,
    cruise_power_w=400,
    hover_ratio=0.3,
    reserve_ratio=0.25
)
print(f"Required battery: {result['capacity_mah']:.0f} mAh ({result['energy_wh']:.0f} Wh)")
```

### Power Distribution

```
Power Distribution Architecture:

        ┌─────────────────────────────────────────────┐
        │              Battery Pack                    │
        │         (22.2V 6S / 44.4V 12S)              │
        └────────────────────┬────────────────────────┘
                             │
                    ┌────────┴────────┐
                    │  Main Power Bus │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────┴────┐         ┌─────┴─────┐        ┌────┴────┐
   │ Motor   │         │  5V/12V   │        │Redundant│
   │  ESCs   │         │ Regulators│        │ Power   │
   └────┬────┘         └─────┬─────┘        └────┬────┘
        │                    │                    │
   M1 M2 M3...         ┌─────┴─────┐        ┌────┴────┐
                       │           │        │ Flight  │
                  ┌────┴────┐ ┌────┴────┐   │Controller│
                  │Sensors  │ │ Payload │   │ (backup)│
                  │GPS,IMU  │ │ Release │   └─────────┘
                  └─────────┘ └─────────┘
```

---

## 3.5 Sensor Suite

### Required Sensors

| Sensor | Purpose | Accuracy | Update Rate |
|--------|---------|----------|-------------|
| IMU (6-axis) | Attitude estimation | ±0.5° | 400+ Hz |
| GNSS receiver | Position | <2m (standalone) | 5-10 Hz |
| Barometer | Altitude | ±1m | 50 Hz |
| Magnetometer | Heading | ±2° | 100 Hz |

### Recommended Sensors

| Sensor | Purpose | Range | Notes |
|--------|---------|-------|-------|
| RTK GNSS | Precision position | N/A | <2cm accuracy |
| LiDAR | Obstacle detection | 30-100m | Single or multi-beam |
| Stereo camera | Visual navigation | 10-30m | Depth perception |
| Ultrasonic | Ground proximity | 0.5-5m | Landing assist |
| Optical flow | Velocity estimation | 0.5-5m | GPS backup |
| ADS-B receiver | Aircraft detection | N/A | See-and-avoid |

### Sensor Fusion Architecture

```python
class SensorFusion:
    """
    Extended Kalman Filter for sensor fusion.
    Combines GPS, IMU, and barometer data.
    """

    def __init__(self):
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)
        self.covariance = np.eye(9) * 0.1

    def predict(self, imu_data, dt):
        """
        Predict step using IMU data.
        """
        # Extract accelerations and angular rates
        accel = imu_data['acceleration']
        gyro = imu_data['angular_velocity']

        # Update attitude
        self.state[6:9] += gyro * dt

        # Rotate acceleration to world frame
        R = self._rotation_matrix(self.state[6], self.state[7], self.state[8])
        world_accel = R @ accel - np.array([0, 0, 9.81])

        # Update velocity
        self.state[3:6] += world_accel * dt

        # Update position
        self.state[0:3] += self.state[3:6] * dt

        # Update covariance
        self._update_covariance_predict(dt)

    def update_gps(self, gps_data):
        """
        Update step using GPS position.
        """
        # Measurement: [lat, lon, alt] -> [x, y, z]
        measurement = self._gps_to_local(gps_data)

        # Kalman gain calculation
        H = np.zeros((3, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1

        R = np.diag([gps_data['hdop'] * 2, gps_data['hdop'] * 2, gps_data['vdop'] * 3])

        self._kalman_update(measurement, H, R)

    def get_position(self):
        return self.state[0:3]

    def get_velocity(self):
        return self.state[3:6]

    def get_attitude(self):
        return self.state[6:9]
```

---

## 3.6 Communication Architecture

### Communication Requirements

| Link | Purpose | Range | Latency | Redundancy |
|------|---------|-------|---------|------------|
| Command & Control | Flight control | BVLOS | <100ms | Required |
| Telemetry | Status reporting | BVLOS | <500ms | Recommended |
| Video | Situational awareness | BVLOS | <1s | Optional |
| UTM | Traffic management | Nationwide | <5s | Required |
| Remote ID | Public identification | 1km | <1s | Required |

### Communication Stack

```
Communication Protocol Stack:

┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  (Mission Control, Telemetry, Commands)                     │
├─────────────────────────────────────────────────────────────┤
│                    Security Layer                            │
│  (TLS 1.3, AES-256 encryption, authentication)              │
├─────────────────────────────────────────────────────────────┤
│                    Transport Layer                           │
│  (TCP for commands, UDP for telemetry, MQTT for events)     │
├─────────────────────────────────────────────────────────────┤
│                    Network Layer                             │
│  (IPv4/IPv6, Mobile IP)                                     │
├─────────────────────────────────────────────────────────────┤
│                    Link Layer                                │
│  Primary: LTE/5G | Backup: 900 MHz | Emergency: Satellite   │
└─────────────────────────────────────────────────────────────┘
```

### Link Budget Calculation

```python
def calculate_link_budget(frequency_mhz, distance_km, tx_power_dbm,
                          tx_antenna_gain_db, rx_antenna_gain_db,
                          rx_sensitivity_dbm):
    """
    Calculate communication link budget.

    Returns:
        Link margin in dB (positive = working, negative = failed)
    """
    import math

    # Free space path loss (dB)
    fspl = 20 * math.log10(distance_km) + 20 * math.log10(frequency_mhz) + 32.44

    # Additional losses (atmospheric, connector, etc.)
    misc_loss = 3  # dB

    # Total path loss
    total_loss = fspl + misc_loss

    # Received power
    rx_power = tx_power_dbm + tx_antenna_gain_db - total_loss + rx_antenna_gain_db

    # Link margin
    margin = rx_power - rx_sensitivity_dbm

    return {
        "fspl_db": fspl,
        "total_loss_db": total_loss,
        "rx_power_dbm": rx_power,
        "margin_db": margin,
        "status": "OK" if margin > 6 else "MARGINAL" if margin > 0 else "FAILED"
    }

# Example: 900 MHz radio link at 10 km
result = calculate_link_budget(
    frequency_mhz=915,
    distance_km=10,
    tx_power_dbm=30,      # 1W transmitter
    tx_antenna_gain_db=3,
    rx_antenna_gain_db=3,
    rx_sensitivity_dbm=-110
)
print(f"Link margin: {result['margin_db']:.1f} dB ({result['status']})")
```

---

## 3.7 Safety Systems

### Redundancy Requirements

| System | Micro | Light | Medium | Heavy |
|--------|-------|-------|--------|-------|
| Flight controller | Single | Single | Dual | Triple |
| IMU | Single | Dual | Triple | Triple |
| GNSS | Single | Dual | Dual | Triple |
| Motor (effective) | 4 | 6+ | 6+ | 8+ |
| Power | Single | Single | Dual | Dual |
| Communication | Dual | Dual | Triple | Triple |

### Parachute Systems

Required for drones >10 kg MTOW:

```yaml
parachute_specification:
  deployment_time: "<2 seconds"
  descent_rate: "<5 m/s at maximum weight"
  opening_altitude: ">30m for safe deployment"
  trigger_conditions:
    - Loss of attitude control
    - Multiple motor failure
    - Structural failure detected
    - Manual pilot command
  testing:
    - 10 successful deployments before certification
    - Annual repack required
    - Functional test before each flight
```

### Geofencing

```python
class Geofence:
    """
    Geofence enforcement for delivery drones.
    """

    PRIORITY_CRITICAL = 1  # Airports, military - hard boundary
    PRIORITY_HIGH = 2      # Schools, hospitals - soft with auth
    PRIORITY_MEDIUM = 3    # Parks, stadiums - temporal
    PRIORITY_LOW = 4       # Residential - altitude restricted

    def __init__(self):
        self.zones = []

    def add_zone(self, zone):
        self.zones.append(zone)

    def check_position(self, lat, lon, alt):
        """
        Check if position violates any geofence.

        Returns:
            (allowed: bool, action: str, zone: Zone)
        """
        for zone in sorted(self.zones, key=lambda z: z.priority):
            if zone.contains(lat, lon, alt):
                if zone.priority == self.PRIORITY_CRITICAL:
                    return False, "EMERGENCY_LAND", zone
                elif zone.priority == self.PRIORITY_HIGH:
                    if not zone.is_authorized():
                        return False, "RETURN_TO_HOME", zone
                elif zone.priority == self.PRIORITY_MEDIUM:
                    if zone.is_active():
                        return False, "REROUTE", zone
                elif zone.priority == self.PRIORITY_LOW:
                    if alt < zone.min_altitude:
                        return True, "CLIMB", zone

        return True, None, None
```

---

## Chapter Summary

The WIA-AUTO-017 standard provides a comprehensive framework for delivery drone systems, built on principles of safety, interoperability, scalability, accessibility, and sustainability. The classification system spans from micro-class drones for small packages to heavy-class vehicles for large cargo and disaster relief.

Propulsion systems range from simple quadcopters to redundant octocopter configurations, with clear specifications for motor performance and reliability. Power systems emphasize battery safety and appropriate sizing for mission requirements. The sensor suite combines required components (IMU, GNSS, barometer, magnetometer) with recommended additions for advanced autonomy.

Communication architecture provides redundant connectivity with appropriate security, while safety systems including redundancy requirements, parachutes, and geofencing ensure reliable operation in all conditions.

---

## Key Takeaways

1. **Four weight classes** (Micro, Light, Medium, Heavy) with distinct requirements
2. **Redundancy scales with risk**: More motors, dual flight controllers for heavier drones
3. **Sensor fusion** combines IMU, GPS, and barometer for accurate state estimation
4. **Communication redundancy** (cellular + radio + satellite) ensures BVLOS reliability
5. **Parachutes mandatory** for drones over 10 kg MTOW

---

## Review Questions

1. What is the minimum thrust-to-weight ratio for medium-class delivery drones?
2. Design a battery specification for a 25-minute flight with 500W average power consumption.
3. What sensors are required vs. recommended for Light-class drones?
4. Calculate the link budget for a 5 km delivery mission using LTE (1800 MHz).
5. What are the geofencing action requirements for each priority level?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
