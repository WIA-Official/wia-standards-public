# PHASE 3 — Protocol

> V2X coordination protocols: the collision-avoidance protocol
> stack that coordinates short-range maneuvering across nearby
> vehicles, and the platooning protocol that lets a leader-
> follower vehicle string operate as a coordinated unit.

## 11. Collision Avoidance Protocols

### 11.1 Forward Collision Warning (FCW)

**Detection Algorithm:**
```python
class ForwardCollisionWarning:
    def __init__(self):
        self.warning_threshold = 2.5  # seconds TTC
        self.critical_threshold = 1.5  # seconds TTC
        self.min_distance = 5.0  # meters

    def evaluate(self, ego_vehicle, target_vehicle):
        # Calculate relative position
        rel_pos = self.calculate_relative_position(ego_vehicle, target_vehicle)

        # Check if target is ahead
        if not self.is_vehicle_ahead(ego_vehicle.heading, rel_pos):
            return None

        # Calculate distance
        distance = self.calculate_distance(rel_pos)

        # Calculate relative velocity
        rel_velocity = ego_vehicle.speed - target_vehicle.speed

        # Calculate TTC
        if rel_velocity > 0:
            ttc = distance / rel_velocity
        else:
            return None  # Target moving away or same speed

        # Determine warning level
        if ttc < self.critical_threshold:
            return {
                'level': 'CRITICAL',
                'ttc': ttc,
                'distance': distance,
                'action': 'EMERGENCY_BRAKE',
                'deceleration': self.calculate_required_decel(distance, rel_velocity)
            }
        elif ttc < self.warning_threshold:
            return {
                'level': 'WARNING',
                'ttc': ttc,
                'distance': distance,
                'action': 'ALERT_DRIVER',
                'deceleration': self.calculate_required_decel(distance, rel_velocity)
            }

        return None

    def calculate_required_decel(self, distance, rel_velocity):
        # Physics: v² = v₀² + 2ad
        # Solve for a: a = (v² - v₀²) / (2d)
        return (rel_velocity ** 2) / (2 * distance)
```

### 11.2 Intersection Movement Assist (IMA)

**Intersection Collision Detection:**
```python
class IntersectionMovementAssist:
    def __init__(self):
        self.intersection_radius = 50  # meters
        self.warning_time = 3.0  # seconds

    def evaluate(self, ego_vehicle, other_vehicles, intersection_map):
        # Check if approaching intersection
        if not self.is_approaching_intersection(ego_vehicle, intersection_map):
            return None

        threats = []

        for other in other_vehicles:
            # Check if other vehicle also approaching
            if not self.is_approaching_intersection(other, intersection_map):
                continue

            # Calculate paths through intersection
            ego_path = self.predict_path(ego_vehicle, intersection_map)
            other_path = self.predict_path(other, intersection_map)

            # Check for path intersection
            conflict_point = self.find_conflict_point(ego_path, other_path)

            if conflict_point:
                # Calculate arrival times
                ego_arrival = self.calculate_arrival_time(ego_vehicle, conflict_point)
                other_arrival = self.calculate_arrival_time(other, conflict_point)

                # Check if arrivals overlap
                time_delta = abs(ego_arrival - other_arrival)

                if time_delta < self.warning_time:
                    threats.append({
                        'vehicle': other.id,
                        'conflict_point': conflict_point,
                        'time_delta': time_delta,
                        'ego_arrival': ego_arrival,
                        'other_arrival': other_arrival,
                        'priority': self.determine_priority(
                            ego_vehicle, other, intersection_map
                        )
                    })

        if threats:
            # Sort by time delta (most urgent first)
            threats.sort(key=lambda t: t['time_delta'])
            return {
                'level': 'WARNING',
                'threats': threats,
                'action': 'SLOW_DOWN' if threats[0]['priority'] == 'YIELD' else 'MONITOR'
            }

        return None
```

### 11.3 Blind Spot Warning / Lane Change Warning

**Blind Spot Detection:**
```python
class BlindSpotWarning:
    def __init__(self):
        self.blind_spot_zones = {
            'left': {
                'lateral_offset': (-0.5, -3.0),  # meters from vehicle center
                'longitudinal_range': (-2.0, 2.0)  # meters from vehicle center
            },
            'right': {
                'lateral_offset': (0.5, 3.0),
                'longitudinal_range': (-2.0, 2.0)
            }
        }

    def evaluate(self, ego_vehicle, nearby_vehicles):
        warnings = {'left': None, 'right': None}

        for other in nearby_vehicles:
            # Transform other vehicle to ego coordinate system
            rel_pos = self.transform_to_ego_frame(ego_vehicle, other)

            # Check each blind spot zone
            for side, zone in self.blind_spot_zones.items():
                if self.is_in_zone(rel_pos, zone):
                    # Calculate relative velocity
                    rel_velocity = self.calculate_relative_velocity(
                        ego_vehicle, other
                    )

                    warnings[side] = {
                        'vehicle_id': other.id,
                        'distance': self.calculate_distance(rel_pos),
                        'relative_velocity': rel_velocity,
                        'position': rel_pos
                    }

        return warnings

    def evaluate_lane_change(self, ego_vehicle, nearby_vehicles, target_lane):
        # Check blind spot in target lane direction
        side = 'left' if target_lane < ego_vehicle.lane else 'right'
        blind_spot = self.evaluate(ego_vehicle, nearby_vehicles)

        if blind_spot[side]:
            return {
                'safe': False,
                'reason': 'Vehicle in blind spot',
                'vehicle': blind_spot[side]
            }

        # Check target lane for approaching vehicles
        lane_clear = self.check_target_lane_clear(
            ego_vehicle, nearby_vehicles, target_lane
        )

        return {
            'safe': lane_clear,
            'reason': 'Target lane clear' if lane_clear else 'Vehicle approaching'
        }
```

### 11.4 Emergency Electronic Brake Lights (EEBL)

**Emergency Brake Detection:**
```python
class EmergencyBrakeLights:
    def __init__(self):
        self.hard_brake_threshold = -4.0  # m/s² (deceleration)
        self.relay_distance = 300  # meters

    def detect_emergency_brake(self, vehicle):
        # Check for hard braking
        if vehicle.acceleration.longitudinal < self.hard_brake_threshold:
            return {
                'type': 'EMERGENCY_BRAKE',
                'vehicle_id': vehicle.id,
                'position': vehicle.position,
                'speed': vehicle.speed,
                'deceleration': vehicle.acceleration.longitudinal,
                'timestamp': vehicle.timestamp
            }
        return None

    def broadcast_eebl(self, ego_vehicle):
        emergency = self.detect_emergency_brake(ego_vehicle)

        if emergency:
            # Create DENM message
            denm = {
                'messageType': 'DENM',
                'eventType': 'emergencyBrake',
                'severity': 'danger',
                'position': ego_vehicle.position,
                'heading': ego_vehicle.heading,
                'speed': ego_vehicle.speed,
                'deceleration': ego_vehicle.acceleration.longitudinal,
                'relevanceDistance': self.relay_distance,
                'validityDuration': 5  # seconds
            }

            return denm

        return None

    def process_eebl(self, eebl_message, ego_vehicle):
        # Check if relevant (same direction, behind emergency vehicle)
        if not self.is_relevant(eebl_message, ego_vehicle):
            return None

        # Calculate distance to emergency
        distance = self.calculate_distance(
            ego_vehicle.position, eebl_message.position
        )

        # Calculate warning level based on distance and speed
        if distance < 100 and ego_vehicle.speed > 50:  # km/h
            return {
                'level': 'CRITICAL',
                'action': 'PREPARE_TO_BRAKE',
                'distance': distance,
                'message': f'Emergency braking ahead in {distance}m'
            }
        elif distance < 200:
            return {
                'level': 'WARNING',
                'action': 'REDUCE_SPEED',
                'distance': distance,
                'message': f'Vehicle braking ahead in {distance}m'
            }

        return None
```

---


## 12. Platooning Protocols

### 12.1 Platoon Formation

**Platoon Roles:**
```
Leader:
  - Controls platoon speed
  - Makes navigation decisions
  - Broadcasts platoon status
  - Manages member join/leave

Member:
  - Follows leader's speed/direction
  - Maintains safe following distance
  - Reports status to leader
  - Can request to leave
```

**Formation Protocol:**
```python
class PlatoonFormation:
    def __init__(self):
        self.max_members = 10
        self.min_spacing = 5  # meters (inter-vehicle)
        self.max_spacing = 15  # meters
        self.target_spacing = 10  # meters

    def initiate_platoon(self, leader_vehicle):
        return {
            'platoon_id': generate_uuid(),
            'leader': leader_vehicle.id,
            'members': [],
            'formation': 'line',  # or 'column'
            'target_speed': leader_vehicle.speed,
            'spacing': self.target_spacing,
            'max_members': self.max_members,
            'status': 'FORMING'
        }

    def request_join(self, vehicle, platoon):
        # Check eligibility
        if len(platoon.members) >= platoon.max_members:
            return {'status': 'REJECTED', 'reason': 'Platoon full'}

        # Check position (must be behind platoon)
        if not self.is_behind_platoon(vehicle, platoon):
            return {'status': 'REJECTED', 'reason': 'Not in position'}

        # Check compatibility (speed, direction, vehicle type)
        if not self.is_compatible(vehicle, platoon):
            return {'status': 'REJECTED', 'reason': 'Incompatible vehicle'}

        # Calculate join position
        join_position = len(platoon.members) + 1

        return {
            'status': 'APPROVED',
            'position': join_position,
            'target_spacing': platoon.spacing,
            'target_speed': platoon.target_speed,
            'formation_point': self.calculate_formation_point(platoon, join_position)
        }

    def leave_platoon(self, vehicle, platoon):
        # Notify leader
        self.notify_leader(platoon.leader, {
            'event': 'MEMBER_LEAVING',
            'vehicle': vehicle.id
        })

        # Adjust positions of following vehicles
        position = platoon.members.index(vehicle.id)

        for i in range(position + 1, len(platoon.members)):
            self.notify_vehicle(platoon.members[i], {
                'action': 'ADJUST_POSITION',
                'new_position': i - 1
            })

        # Remove from platoon
        platoon.members.remove(vehicle.id)

        return {'status': 'LEFT', 'action': 'RESUME_MANUAL_CONTROL'}
```

### 12.2 Platoon Maintenance

**Spacing Control:**
```python
class PlatoonSpacingControl:
    def __init__(self):
        self.kp = 0.5  # Proportional gain
        self.kd = 0.2  # Derivative gain
        self.previous_error = 0

    def calculate_control(self, ego_vehicle, leader_vehicle, target_spacing):
        # Measure actual spacing
        actual_spacing = self.measure_spacing(ego_vehicle, leader_vehicle)

        # Calculate error
        error = actual_spacing - target_spacing

        # Proportional term
        p_term = self.kp * error

        # Derivative term (rate of change)
        d_term = self.kd * (error - self.previous_error)

        # PD control output
        control = p_term + d_term

        # Update previous error
        self.previous_error = error

        # Convert to acceleration command
        # Positive control = speed up, Negative = slow down
        acceleration = np.clip(control, -3.0, 2.0)  # m/s²

        return {
            'acceleration': acceleration,
            'error': error,
            'spacing': actual_spacing,
            'status': 'GOOD' if abs(error) < 1.0 else 'ADJUSTING'
        }
```

**Cooperative Adaptive Cruise Control (CACC):**
```python
class CooperativeAdaptiveCruiseControl:
    def __init__(self):
        self.time_gap = 0.6  # seconds (much shorter than ACC's 1.5-2.0s)
        self.max_accel = 2.0  # m/s²
        self.max_decel = -4.0  # m/s²

    def calculate_target_speed(self, ego_vehicle, leader_vehicle):
        # Get leader's acceleration from V2V message
        leader_accel = leader_vehicle.acceleration.longitudinal

        # Calculate desired spacing based on speed
        desired_spacing = ego_vehicle.speed * self.time_gap + self.min_spacing

        # Measure actual spacing
        actual_spacing = self.measure_spacing(ego_vehicle, leader_vehicle)

        # Spacing error
        spacing_error = actual_spacing - desired_spacing

        # Speed error
        speed_error = ego_vehicle.speed - leader_vehicle.speed

        # CACC algorithm (includes feedforward from leader acceleration)
        acceleration = (
            0.4 * spacing_error +        # Proportional to spacing error
            0.3 * speed_error +           # Proportional to speed error
            0.8 * leader_accel            # Feedforward from leader
        )

        # Limit acceleration
        acceleration = np.clip(acceleration, self.max_decel, self.max_accel)

        # Calculate target speed
        target_speed = ego_vehicle.speed + acceleration * 0.1  # 100ms update

        return {
            'target_speed': max(0, target_speed),
            'acceleration': acceleration,
            'spacing_error': spacing_error,
            'speed_error': speed_error
        }
```

### 12.3 Platoon Maneuvers

**Lane Change:**
```python
class PlatoonLaneChange:
    def execute_platoon_lane_change(self, platoon, target_lane):
        # 1. Leader announces lane change
        self.broadcast_to_platoon(platoon, {
            'action': 'LANE_CHANGE',
            'target_lane': target_lane,
            'execution_time': time.now() + 5.0  # seconds
        })

        # 2. All members acknowledge
        confirmations = self.collect_confirmations(platoon.members, timeout=2.0)

        if not all(confirmations):
            return {'status': 'ABORTED', 'reason': 'Not all members ready'}

        # 3. Increase spacing temporarily for safety
        self.set_platoon_spacing(platoon, 15)  # meters

        # 4. Leader executes lane change
        self.execute_lane_change(platoon.leader, target_lane)

        # 5. Members follow sequentially
        for member in platoon.members:
            wait_for_clearance(target_lane)
            self.execute_lane_change(member, target_lane)
            time.sleep(1.0)  # 1 second between vehicles

        # 6. Restore normal spacing
        self.set_platoon_spacing(platoon, 10)  # meters

        return {'status': 'COMPLETE'}
```

**Split Platoon:**
```python
def split_platoon(platoon, split_position):
    # Platoon A: Leader + members before split
    platoon_a = {
        'platoon_id': platoon.platoon_id,
        'leader': platoon.leader,
        'members': platoon.members[:split_position]
    }

    # Platoon B: New leader + remaining members
    new_leader = platoon.members[split_position]
    platoon_b = {
        'platoon_id': generate_uuid(),
        'leader': new_leader,
        'members': platoon.members[split_position + 1:]
    }

    # Increase spacing between platoons
    self.command_vehicle(new_leader, {
        'action': 'INCREASE_SPACING',
        'target_spacing': 50  # meters
    })

    # Notify all vehicles
    self.broadcast_to_platoon(platoon_a, {
        'event': 'PLATOON_SPLIT',
        'new_platoon': platoon_a
    })

    self.broadcast_to_platoon(platoon_b, {
        'event': 'PLATOON_SPLIT',
        'new_leader': new_leader,
        'new_platoon': platoon_b
    })

    return [platoon_a, platoon_b]
```

---


