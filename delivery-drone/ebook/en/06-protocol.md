# Chapter 6: Flight Control and Navigation Protocol

## Phase 3: Aerodynamics, Control Systems, and Autonomous Navigation

---

## 6.1 Flight Dynamics Fundamentals

### Forces Acting on a Multi-Rotor

A multi-rotor drone experiences four primary forces during flight:

```
                    Lift (L)
                      ↑
                      │
                      │
    Drag (D) ←────── [Drone] ──────→ Thrust (T)
                      │
                      │
                      ↓
                   Weight (W)

Equilibrium Conditions:
- Hover: L = W, T = D = 0
- Forward Flight: L ≈ W, T > D
- Climbing: L > W
- Descending: L < W
```

### Lift and Thrust Equations

**Total Lift Force (Multi-Rotor)**:

$$L = n \times T_{rotor}$$

Where:
- $L$ = Total lift force (N)
- $n$ = Number of rotors
- $T_{rotor}$ = Thrust per rotor (N)

**Individual Rotor Thrust**:

$$T = C_T \times \rho \times A \times (\omega \times r)^2$$

Where:
- $C_T$ = Thrust coefficient (0.01-0.015 typical)
- $\rho$ = Air density (1.225 kg/m³ at sea level)
- $A$ = Rotor disk area (m²)
- $\omega$ = Angular velocity (rad/s)
- $r$ = Rotor radius (m)

```python
def calculate_rotor_thrust(ct, rho, diameter, rpm):
    """
    Calculate thrust produced by a single rotor.

    Args:
        ct: Thrust coefficient (typically 0.012)
        rho: Air density (kg/m³)
        diameter: Propeller diameter (m)
        rpm: Rotations per minute

    Returns:
        Thrust in Newtons
    """
    import math

    radius = diameter / 2
    area = math.pi * radius ** 2
    omega = rpm * 2 * math.pi / 60  # Convert to rad/s

    thrust = ct * rho * area * (omega * radius) ** 2

    return thrust

# Example: 15" propeller at 5000 RPM
thrust = calculate_rotor_thrust(
    ct=0.012,
    rho=1.225,
    diameter=0.381,  # 15 inches
    rpm=5000
)
print(f"Single rotor thrust: {thrust:.2f} N ({thrust/9.81:.2f} kg)")
```

### Power Requirements

**Hover Power**:

$$P_{hover} = \frac{(mg)^{3/2}}{\sqrt{2\rho A_{total}}}$$

Where:
- $m$ = Total mass (kg)
- $g$ = Gravitational acceleration (9.81 m/s²)
- $A_{total}$ = Total rotor disk area (m²)

**Forward Flight Power**:

$$P_{forward} = P_{induced} + P_{profile} + P_{parasite}$$

Where:
- $P_{induced}$ = Power to generate lift
- $P_{profile}$ = Power to overcome rotor drag
- $P_{parasite}$ = Power to overcome body drag

```python
def calculate_hover_power(mass_kg, num_rotors, rotor_diameter_m, rho=1.225):
    """
    Calculate power required to hover.

    Returns:
        Power in Watts
    """
    import math

    # Total weight
    weight = mass_kg * 9.81

    # Total rotor disk area
    rotor_area = num_rotors * math.pi * (rotor_diameter_m / 2) ** 2

    # Ideal hover power
    p_ideal = weight ** 1.5 / math.sqrt(2 * rho * rotor_area)

    # Account for efficiency losses (typically 60-70% efficiency)
    efficiency = 0.65
    p_actual = p_ideal / efficiency

    return p_actual

# Example: 8kg hexacopter with 15" props
power = calculate_hover_power(
    mass_kg=8,
    num_rotors=6,
    rotor_diameter_m=0.381
)
print(f"Hover power: {power:.0f} W")
```

### Drag Forces

**Drag Equation**:

$$D = \frac{1}{2} \times \rho \times v^2 \times A_f \times C_D$$

Where:
- $v$ = Airspeed (m/s)
- $A_f$ = Frontal area (m²)
- $C_D$ = Drag coefficient (1.0-1.5 for multi-rotor)

| Configuration | Typical $C_D$ |
|---------------|---------------|
| Quadcopter (exposed) | 1.3-1.5 |
| Hexacopter (enclosed) | 1.0-1.2 |
| Streamlined delivery | 0.8-1.0 |
| Fixed-wing VTOL | 0.3-0.5 |

---

## 6.2 Stabilization and Control Systems

### Control Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                    MISSION PLANNER                               │
│  (Waypoints, Goals, Constraints)                                │
├─────────────────────────────────────────────────────────────────┤
│                    NAVIGATION CONTROLLER                         │
│  (Path Following, Position Control)                             │
│  Loop Rate: 10-50 Hz                                            │
├─────────────────────────────────────────────────────────────────┤
│                    ATTITUDE CONTROLLER                           │
│  (Roll, Pitch, Yaw Control)                                     │
│  Loop Rate: 250-500 Hz                                          │
├─────────────────────────────────────────────────────────────────┤
│                    RATE CONTROLLER                               │
│  (Angular Rate Control)                                         │
│  Loop Rate: 500-1000 Hz                                         │
├─────────────────────────────────────────────────────────────────┤
│                    MOTOR MIXER                                   │
│  (Motor Commands)                                               │
└─────────────────────────────────────────────────────────────────┘
```

### PID Controller Implementation

**Standard PID Formula**:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

```python
class PIDController:
    """
    PID controller for drone stabilization.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 100.0,
                 output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def compute(self, setpoint: float, measured: float,
                current_time: float) -> float:
        """
        Compute PID output.

        Args:
            setpoint: Desired value
            measured: Current measured value
            current_time: Current timestamp

        Returns:
            Control output
        """
        error = setpoint - measured

        # Time delta
        if self.previous_time is None:
            dt = 0.01  # Assume 100Hz on first call
        else:
            dt = current_time - self.previous_time

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                           min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative

        # Total output with limiting
        output = p_term + i_term + d_term
        output = max(-self.output_limit,
                    min(self.output_limit, output))

        # Store for next iteration
        self.previous_error = error
        self.previous_time = current_time

        return output

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None


# Typical PID gains for attitude control
attitude_controllers = {
    'roll': PIDController(kp=4.5, ki=0.02, kd=0.18),
    'pitch': PIDController(kp=4.5, ki=0.02, kd=0.18),
    'yaw': PIDController(kp=3.0, ki=0.01, kd=0.10)
}
```

### Motor Mixing

For a hexacopter in X configuration:

```python
def hexacopter_mixer(throttle, roll, pitch, yaw):
    """
    Mix control inputs to motor commands for hexacopter X configuration.

    Motor layout (top view):
          1     2
           \   /
        6   [X]   3
           /   \
          5     4

    Returns:
        List of 6 motor commands (0.0-1.0)
    """
    # Motor mixing matrix
    # Each row: [throttle, roll, pitch, yaw]
    mix = [
        [1.0, -0.5,  0.866,  1.0],  # Motor 1 (front-right)
        [1.0,  0.5,  0.866, -1.0],  # Motor 2 (front-left)
        [1.0,  1.0,  0.0,    1.0],  # Motor 3 (right)
        [1.0,  0.5, -0.866, -1.0],  # Motor 4 (rear-left)
        [1.0, -0.5, -0.866,  1.0],  # Motor 5 (rear-right)
        [1.0, -1.0,  0.0,   -1.0],  # Motor 6 (left)
    ]

    motors = []
    for i in range(6):
        cmd = (mix[i][0] * throttle +
               mix[i][1] * roll +
               mix[i][2] * pitch +
               mix[i][3] * yaw)
        # Clamp to valid range
        cmd = max(0.0, min(1.0, cmd))
        motors.append(cmd)

    return motors
```

### Sensor Fusion (Extended Kalman Filter)

```python
import numpy as np

class EKFStateEstimator:
    """
    Extended Kalman Filter for drone state estimation.
    Fuses IMU, GPS, and barometer data.
    """

    def __init__(self):
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)

        # State covariance
        self.P = np.eye(9) * 0.1

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01])

        # GPS measurement noise
        self.R_gps = np.diag([2.0, 2.0, 3.0])

        # Barometer measurement noise
        self.R_baro = np.array([[1.0]])

    def predict(self, accel, gyro, dt):
        """
        Prediction step using IMU data.
        """
        # Update attitude
        self.state[6:9] += gyro * dt

        # Rotation matrix (simplified)
        roll, pitch, yaw = self.state[6:9]
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        c_pitch, s_pitch = np.cos(pitch), np.sin(pitch)
        c_yaw, s_yaw = np.cos(yaw), np.sin(yaw)

        # Rotate acceleration to world frame
        R = np.array([
            [c_yaw*c_pitch, c_yaw*s_pitch*s_roll - s_yaw*c_roll, c_yaw*s_pitch*c_roll + s_yaw*s_roll],
            [s_yaw*c_pitch, s_yaw*s_pitch*s_roll + c_yaw*c_roll, s_yaw*s_pitch*c_roll - c_yaw*s_roll],
            [-s_pitch, c_pitch*s_roll, c_pitch*c_roll]
        ])

        world_accel = R @ accel - np.array([0, 0, 9.81])

        # Update velocity
        self.state[3:6] += world_accel * dt

        # Update position
        self.state[0:3] += self.state[3:6] * dt + 0.5 * world_accel * dt**2

        # Update covariance
        F = self._get_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, lat, lon, alt, hdop):
        """
        Update with GPS measurement.
        """
        # Convert lat/lon to local frame (simplified)
        gps_pos = self._gps_to_local(lat, lon, alt)

        # Measurement matrix (position only)
        H = np.zeros((3, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        # Dynamic measurement noise based on HDOP
        R = self.R_gps * hdop

        # Kalman gain
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Innovation
        y = gps_pos - self.state[0:3]

        # Update state
        self.state += K @ y

        # Update covariance
        self.P = (np.eye(9) - K @ H) @ self.P

    def update_barometer(self, altitude):
        """
        Update with barometer measurement.
        """
        H = np.zeros((1, 9))
        H[0, 2] = 1.0

        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T / S[0, 0]

        y = altitude - self.state[2]
        self.state += K.flatten() * y

        self.P = (np.eye(9) - np.outer(K, H)) @ self.P
```

---

## 6.3 Path Planning and Navigation

### A* Path Planning

```python
import heapq
from typing import List, Tuple, Set

class AStarPathPlanner:
    """
    A* path planning for 3D drone navigation.
    """

    def __init__(self, grid_resolution: float = 10.0):
        self.resolution = grid_resolution
        self.obstacles: Set[Tuple[int, int, int]] = set()

    def add_obstacle(self, x: float, y: float, z: float, radius: float):
        """Add cylindrical obstacle to map."""
        # Convert to grid cells
        cells_radius = int(radius / self.resolution) + 1
        cx = int(x / self.resolution)
        cy = int(y / self.resolution)
        cz = int(z / self.resolution)

        for dx in range(-cells_radius, cells_radius + 1):
            for dy in range(-cells_radius, cells_radius + 1):
                for dz in range(-cells_radius, cells_radius + 1):
                    if dx*dx + dy*dy <= cells_radius*cells_radius:
                        self.obstacles.add((cx + dx, cy + dy, cz + dz))

    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Find path from start to goal using A*.

        Returns:
            List of waypoints (x, y, z)
        """
        # Convert to grid
        start_cell = self._to_cell(start)
        goal_cell = self._to_cell(goal)

        # Priority queue: (f_score, g_score, cell, path)
        open_set = [(self._heuristic(start_cell, goal_cell), 0, start_cell, [start_cell])]
        closed_set = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)

            if current == goal_cell:
                # Convert path back to world coordinates
                return [self._to_world(cell) for cell in path]

            if current in closed_set:
                continue
            closed_set.add(current)

            # Explore neighbors (6-connected grid)
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set or neighbor in self.obstacles:
                    continue

                # Cost to reach neighbor
                new_g = g + self._distance(current, neighbor)
                new_f = new_g + self._heuristic(neighbor, goal_cell)

                heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))

        return []  # No path found

    def _to_cell(self, pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return (int(pos[0] / self.resolution),
                int(pos[1] / self.resolution),
                int(pos[2] / self.resolution))

    def _to_world(self, cell: Tuple[int, int, int]) -> Tuple[float, float, float]:
        return (cell[0] * self.resolution + self.resolution / 2,
                cell[1] * self.resolution + self.resolution / 2,
                cell[2] * self.resolution + self.resolution / 2)

    def _heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        # Euclidean distance with altitude penalty
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return ((dx*dx + dy*dy + 1.5 * dz*dz) ** 0.5) * self.resolution

    def _distance(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return ((dx*dx + dy*dy + dz*dz) ** 0.5) * self.resolution

    def _get_neighbors(self, cell: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        x, y, z = cell
        return [
            (x+1, y, z), (x-1, y, z),
            (x, y+1, z), (x, y-1, z),
            (x, y, z+1), (x, y, z-1)
        ]
```

### Dynamic Window Approach for Obstacle Avoidance

```python
class DynamicWindowApproach:
    """
    Dynamic Window Approach for real-time obstacle avoidance.
    """

    def __init__(self, max_speed: float = 20.0,
                 max_yaw_rate: float = 1.0,
                 dt: float = 0.1):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.dt = dt

        # Weights for objective function
        self.alpha = 0.3  # Heading
        self.beta = 0.5   # Clearance
        self.gamma = 0.2  # Velocity

    def compute_velocity(self, current_state, goal, obstacles):
        """
        Compute optimal velocity considering obstacles.

        Args:
            current_state: (x, y, vx, vy, heading)
            goal: (x, y)
            obstacles: List of (x, y, radius)

        Returns:
            (velocity, yaw_rate)
        """
        x, y, vx, vy, heading = current_state
        current_speed = (vx*vx + vy*vy) ** 0.5

        # Dynamic window (reachable velocities)
        min_v = max(0, current_speed - 2.0 * self.dt)
        max_v = min(self.max_speed, current_speed + 2.0 * self.dt)
        min_w = -self.max_yaw_rate
        max_w = self.max_yaw_rate

        best_score = -float('inf')
        best_v, best_w = 0, 0

        # Sample velocities
        for v in np.linspace(min_v, max_v, 10):
            for w in np.linspace(min_w, max_w, 10):
                # Simulate trajectory
                traj = self._simulate(x, y, heading, v, w)

                # Check for collisions
                if self._check_collision(traj, obstacles):
                    continue

                # Compute objective function
                heading_score = self._heading_objective(traj[-1], goal)
                clearance_score = self._clearance_objective(traj, obstacles)
                velocity_score = v / self.max_speed

                score = (self.alpha * heading_score +
                        self.beta * clearance_score +
                        self.gamma * velocity_score)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        return best_v, best_w

    def _simulate(self, x, y, heading, v, w, steps=10):
        """Simulate trajectory for given velocity."""
        trajectory = [(x, y)]
        for _ in range(steps):
            heading += w * self.dt
            x += v * np.cos(heading) * self.dt
            y += v * np.sin(heading) * self.dt
            trajectory.append((x, y))
        return trajectory
```

### Precision Landing

```python
class PrecisionLandingController:
    """
    Visual servoing for precision landing using AprilTags.
    """

    def __init__(self, camera_matrix, tag_size: float = 0.5):
        self.camera_matrix = camera_matrix
        self.tag_size = tag_size

        # Landing phases
        self.phases = ['APPROACH', 'ALIGN', 'DESCEND', 'FINAL', 'TOUCHDOWN']
        self.current_phase = 'APPROACH'

        # PID controllers for landing
        self.x_controller = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.y_controller = PIDController(kp=0.5, ki=0.01, kd=0.1)

    def process_frame(self, frame, current_altitude):
        """
        Process camera frame and compute landing commands.

        Returns:
            (vx, vy, vz, detected)
        """
        # Detect AprilTag
        tag = self._detect_tag(frame)

        if tag is None:
            return 0, 0, 0, False

        # Estimate relative position
        rel_x, rel_y = self._estimate_position(tag, current_altitude)

        # State machine
        if self.current_phase == 'APPROACH' and current_altitude < 10:
            self.current_phase = 'ALIGN'

        elif self.current_phase == 'ALIGN':
            error = (rel_x**2 + rel_y**2) ** 0.5
            if error < 0.5 and current_altitude > 3:
                self.current_phase = 'DESCEND'

        elif self.current_phase == 'DESCEND':
            if current_altitude < 3:
                self.current_phase = 'FINAL'

        elif self.current_phase == 'FINAL':
            if current_altitude < 0.5:
                self.current_phase = 'TOUCHDOWN'

        # Compute velocities based on phase
        vx = -self.x_controller.compute(0, rel_x, time.time())
        vy = -self.y_controller.compute(0, rel_y, time.time())

        descent_rates = {
            'APPROACH': 0,
            'ALIGN': 0,
            'DESCEND': -0.3,
            'FINAL': -0.1,
            'TOUCHDOWN': 0
        }
        vz = descent_rates[self.current_phase]

        return vx, vy, vz, True
```

---

## 6.4 Safety and Emergency Protocols

### Flight Mode State Machine

```python
from enum import Enum, auto

class FlightMode(Enum):
    DISARMED = auto()
    ARMED = auto()
    TAKEOFF = auto()
    HOVER = auto()
    WAYPOINT = auto()
    RETURN_TO_HOME = auto()
    LAND = auto()
    EMERGENCY = auto()

class FlightStateMachine:
    """
    Flight mode state machine with safety transitions.
    """

    def __init__(self):
        self.mode = FlightMode.DISARMED
        self.previous_mode = None

        # Define valid transitions
        self.valid_transitions = {
            FlightMode.DISARMED: [FlightMode.ARMED],
            FlightMode.ARMED: [FlightMode.TAKEOFF, FlightMode.DISARMED],
            FlightMode.TAKEOFF: [FlightMode.HOVER, FlightMode.EMERGENCY],
            FlightMode.HOVER: [FlightMode.WAYPOINT, FlightMode.RETURN_TO_HOME,
                              FlightMode.LAND, FlightMode.EMERGENCY],
            FlightMode.WAYPOINT: [FlightMode.HOVER, FlightMode.RETURN_TO_HOME,
                                  FlightMode.EMERGENCY],
            FlightMode.RETURN_TO_HOME: [FlightMode.HOVER, FlightMode.LAND,
                                        FlightMode.EMERGENCY],
            FlightMode.LAND: [FlightMode.DISARMED, FlightMode.EMERGENCY],
            FlightMode.EMERGENCY: [FlightMode.DISARMED]
        }

    def transition(self, new_mode: FlightMode) -> bool:
        """
        Attempt to transition to new flight mode.

        Returns:
            True if transition successful
        """
        if new_mode in self.valid_transitions.get(self.mode, []):
            self.previous_mode = self.mode
            self.mode = new_mode
            return True

        # Emergency can be entered from any state
        if new_mode == FlightMode.EMERGENCY:
            self.previous_mode = self.mode
            self.mode = new_mode
            return True

        return False
```

### Emergency Procedures

```python
class EmergencyHandler:
    """
    Handle emergency situations during flight.
    """

    def __init__(self, drone_controller, home_position):
        self.controller = drone_controller
        self.home = home_position

    def handle_gps_loss(self):
        """GPS signal lost procedure."""
        # 1. Switch to optical flow/visual odometry
        self.controller.enable_visual_odometry()

        # 2. Reduce altitude to 10m AGL
        self.controller.set_altitude(10)

        # 3. Hover in place
        self.controller.hold_position()

        # 4. Wait for GPS recovery (60s timeout)
        start_time = time.time()
        while time.time() - start_time < 60:
            if self.controller.gps_available():
                self.controller.resume_mission()
                return True
            time.sleep(1)

        # 5. Emergency land if not recovered
        self.controller.emergency_land()
        return False

    def handle_low_battery(self, battery_level: int):
        """Low battery procedure."""
        if battery_level <= 30 and battery_level > 20:
            # Warning - suggest RTH
            self.controller.send_warning("LOW_BATTERY_WARNING")

        elif battery_level <= 20 and battery_level > 10:
            # Automatic RTH
            self.controller.send_warning("LOW_BATTERY_RTH")
            self.controller.return_to_home()

        elif battery_level <= 10 and battery_level > 5:
            # Emergency land at nearest safe location
            safe_site = self.find_nearest_safe_landing()
            self.controller.land_at(safe_site)

        elif battery_level <= 5:
            # Immediate emergency landing
            self.controller.emergency_land()

    def handle_motor_failure(self, failed_motor: int, motor_count: int):
        """Motor failure procedure."""
        if motor_count >= 6:  # Hexacopter or larger
            # Can compensate for single motor failure
            self.controller.disable_motor(failed_motor)
            self.controller.reconfigure_mixer()
            self.controller.reduce_performance()

            # Navigate to emergency landing
            safe_site = self.find_nearest_safe_landing()
            self.controller.land_at(safe_site)
        else:
            # Quadcopter - cannot compensate
            if self.controller.has_parachute():
                self.controller.deploy_parachute()
            else:
                # Attempt controlled crash
                self.controller.cut_motors()
                self.controller.broadcast_emergency()

    def handle_communication_loss(self):
        """Communication lost procedure."""
        # 1. Continue current mission for 10s
        time.sleep(10)

        # 2. Check if communication restored
        if self.controller.communication_available():
            return

        # 3. Execute Return-On-Abort
        self.controller.climb_to_safe_altitude()
        self.controller.return_via_preplanned_route()
```

---

## Chapter Summary

Flight control and navigation form the technical core of delivery drone operations. Understanding flight dynamics—lift, thrust, drag, and power requirements—enables proper system design and performance prediction. Stabilization systems use cascaded PID controllers with careful tuning to maintain stable flight across all conditions.

Sensor fusion through Extended Kalman Filtering combines IMU, GPS, and barometer data for accurate state estimation. Path planning algorithms like A* find optimal routes, while Dynamic Window Approach enables real-time obstacle avoidance. Precision landing using visual servoing ensures accurate package delivery.

Safety systems implement comprehensive emergency procedures for GPS loss, low battery, motor failure, and communication loss, ensuring that even in failure modes, the drone behaves predictably and safely.

---

## Key Takeaways

1. **Hover power scales with mass^1.5** making payload efficiency critical
2. **Cascaded PID control** separates attitude and position regulation
3. **Extended Kalman Filter** fuses multiple sensors for robust estimation
4. **A* path planning** with altitude penalty optimizes energy consumption
5. **Emergency procedures** must handle all credible failure modes

---

## Review Questions

1. Calculate the hover power for a 12kg drone with 6x 18" propellers.
2. What are typical PID gains for roll/pitch control, and why differ from yaw?
3. Implement the measurement update step for a magnetometer in the EKF.
4. How does Dynamic Window Approach differ from A* for obstacle avoidance?
5. Design a state machine for parachute deployment decision logic.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
