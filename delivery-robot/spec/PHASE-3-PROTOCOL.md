# WIA ROB-010 Delivery Robot Standard - Phase 3: Navigation Protocol

> **Version**: 1.0.0  
> **Status**: Stable  
> **Last Updated**: 2025-12-26

---

## 1. Overview

Phase 3 defines the navigation protocols, path planning algorithms, obstacle avoidance systems, and real-time decision-making frameworks for autonomous delivery robots. This specification ensures safe, efficient navigation in complex urban environments.

### 1.1 Navigation Principles

- **Safety First**: Pedestrian safety is paramount  
- **Rule Compliance**: Adhere to traffic laws and regulations  
- **Predictability**: Behave in expected, understandable ways  
- **Adaptability**: Handle dynamic environments and edge cases  
- **Efficiency**: Optimize for time, energy, and user satisfaction  

---

## 2. Localization

### 2.1 Global Positioning

**Primary: GPS/GNSS**  
- Accuracy: 1-5 meters in open areas  
- Update rate: 1-10 Hz  
- Limitations: Degraded in urban canyons, tunnels, indoors  

**Enhanced: RTK-GPS**  
- Accuracy: 1-10 centimeters  
- Requires base station with known position  
- Real-time kinematic corrections  

**Fallback: Dead Reckoning**  
- Wheel odometry + IMU  
- Accumulates drift over time  
- Used when GPS unavailable  

### 2.2 Local Positioning (SLAM)

**Simultaneous Localization and Mapping**  

```
SLAM Algorithm Overview:
1. Sensor Data Acquisition (LiDAR, cameras)
2. Feature Extraction (corners, edges, landmarks)
3. Data Association (match current to previous observations)
4. State Estimation (Extended Kalman Filter or Particle Filter)
5. Map Update (add/update landmarks in map)
6. Loop Closure (recognize previously visited locations)
```

**Map Types**:  
- **Occupancy Grid**: 2D grid of occupied/free cells  
- **Feature Map**: Sparse set of distinctive landmarks  
- **Semantic Map**: Objects labeled by type (tree, building, crosswalk)  

---

## 3. Obstacle Detection

### 3.1 Sensor Fusion

Combine multiple sensor modalities for robust obstacle detection:

| Sensor | Range | FoV | Weather | Update Rate | Cost |
|--------|-------|-----|---------|-------------|------|
| LiDAR | 0.1-100m | 360° | Poor in rain/fog | 10-20 Hz | High |
| Camera | 0.5-50m | 60-120° | Poor in dark/glare | 30-60 Hz | Low |
| Ultrasonic | 0.02-5m | 15-30° | Excellent | 10-40 Hz | Very Low |
| Radar | 1-200m | 15-120° | Excellent | 10-20 Hz | Medium |

**Fusion Strategy**:  
1. Raw sensor data preprocessing  
2. Object detection per modality  
3. Track association (match detections across sensors)  
4. Kalman filter fusion  
5. Classification (pedestrian, vehicle, static obstacle)  
6. Trajectory prediction  

### 3.2 Obstacle Classification

```json
{
    "obstacle_id": "obs-001",
    "class": "pedestrian",
    "confidence": 0.95,
    "position": {"x": 5.2, "y": -1.3, "z": 0.0},
    "velocity": {"vx": 1.2, "vy": 0.3},
    "bounding_box": {"length": 0.5, "width": 0.4, "height": 1.7},
    "predicted_trajectory": [
        {"t": 1.0, "x": 6.4, "y": -1.0},
        {"t": 2.0, "x": 7.6, "y": -0.7}
    ]
}
```

**Safety Margins**:  
- Pedestrians: 1.5-2.0 meters  
- Vehicles: 2.0-3.0 meters  
- Static obstacles: 0.3-0.5 meters  
- Wheelchairs/strollers: 2.0 meters (higher priority)  

---

## 4. Path Planning

### 4.1 Global Planning (A* Algorithm)

```python
def a_star(start, goal, map):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while not open_set.empty():
        current = open_set.get()[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current):
            tentative_g = g_score[current] + cost(current, neighbor)
            
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score[neighbor], neighbor))
    
    return None  # No path found
```

**Cost Function**:  
- Distance: Base cost  
- Terrain: Multiply by difficulty (pavement=1.0, grass=1.5, gravel=2.0)  
- Slope: Penalty for uphill (energy cost)  
- Traffic density: Avoid crowded areas  
- Safety: Bonus for sidewalks, penalty for roads  

### 4.2 Local Planning (DWA - Dynamic Window Approach)

Select velocity commands that:  
1. Are kinematically feasible (within robot's capabilities)  
2. Avoid collisions with obstacles  
3. Make progress toward goal  
4. Maintain smooth motion  

```
DWA Scoring:
score = α * heading_score + β * dist_score + γ * velocity_score
  where:
    heading_score = alignment with goal direction
    dist_score = clearance to nearest obstacle
    velocity_score = forward velocity (prefer faster when safe)
```

---

## 5. Traffic Rule Compliance

### 5.1 Crosswalk Behavior

```
Crosswalk Protocol:
1. Detect crosswalk (visual + map data)
2. Check traffic signal (if present)
   - Green/Walk: Proceed with caution
   - Red/Don't Walk: Stop and wait
   - Flashing: Evaluate time remaining
3. Scan for approaching vehicles (LiDAR + camera)
4. If clear or signal allows:
   - Signal intent (lights/sound)
   - Cross at steady pace (1-2 m/s)
   - Monitor for vehicles (even on green)
5. If vehicle approaches:
   - Yield, even if robot has right-of-way
   - Resume when safe
```

### 5.2 Sidewalk Etiquette

- **Yield to pedestrians always**  
- **Maintain 1.5m distance minimum**  
- **Pass on left when possible**  
- **Slow to 0.5 m/s in crowded areas**  
- **Stop completely if path blocked**  
- **Never block wheelchair ramps or doorways**  

---

## 6. Emergency Procedures

### 6.1 Emergency Stop

Trigger Conditions:  
- Physical emergency button pressed  
- Remote emergency command received  
- Critical sensor failure detected  
- Obstacle within 0.3m  
- Loss of control (tipping, skidding)  

Response:  
1. Immediate motor cutoff (< 100ms)  
2. Engage brakes  
3. Activate hazard lights and alarm  
4. Log event with full telemetry snapshot  
5. Send alert to operations center  
6. Await manual clearance before resuming  

### 6.2 Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| GPS loss | No fix for 30s | Switch to SLAM/dead reckoning |
| LiDAR failure | No data for 5s | Rely on cameras, reduce speed 50% |
| Camera failure | No image | Rely on LiDAR/ultrasonic |
| Battery < 15% | Voltage drop | Return to charging station |
| Communication loss | No cloud contact 60s | Continue current task, log locally |
| Stuck (no progress) | Position unchanged 30s | Request remote assistance |

---

## 7. Real-Time Decision Making

### 7.1 Behavior State Machine

```
States:
- IDLE: Waiting for task assignment
- NAVIGATING: En route to destination
- WAITING: Stopped for obstacle/signal
- CHARGING: At charging station
- DELIVERING: Package handoff in progress
- ERROR: Failure requiring intervention

Transitions:
IDLE → NAVIGATING: Task assigned
NAVIGATING → WAITING: Obstacle/signal blocks path
WAITING → NAVIGATING: Path clear
NAVIGATING → DELIVERING: Arrived at destination
DELIVERING → IDLE: Package delivered successfully
ANY → ERROR: Critical failure
ERROR → IDLE: Manually cleared
```

### 7.2 Priority Handling

When multiple objectives conflict, prioritize:  
1. **Safety** (avoid collision)  
2. **Rule compliance** (obey traffic laws)  
3. **Task completion** (deliver package)  
4. **Efficiency** (optimize route/energy)  
5. **User experience** (smooth, predictable)  

---

## 8. Performance Metrics

Monitor and optimize:  
- **Position accuracy**: < 0.5m error 95% of time  
- **Obstacle detection rate**: > 99.9% (no misses)  
- **False positive rate**: < 5%  
- **Reaction time**: < 100ms from detection to evasive action  
- **Route deviation**: < 10% additional distance vs. optimal  
- **Energy efficiency**: Actual vs. predicted battery usage  

---

**Copyright 2025 WIA / SmileStory Inc.**  
**License**: MIT
