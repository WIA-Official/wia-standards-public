# WIA-AUTO-024: Fleet Management Standard
## PHASE 2 - Algorithm Specifications

**Version:** 1.0.0  
**Status:** Active  
**Category:** Automotive / Fleet Management  
**Last Updated:** January 2025

---

## Overview

Phase 2 defines comprehensive algorithms for fleet management operations including route optimization, efficiency scoring, predictive maintenance, driver behavior analysis, and fuel consumption modeling. These specifications enable consistent, high-quality implementations across different platforms.

**弘益人間 (Benefit All Humanity)** - Efficient algorithms reduce costs, improve safety, and minimize environmental impact for the benefit of all.

---

## 1. Fleet Efficiency Score (FES)

### 1.1 Formula

```
FES = (α × RT + β × FM + γ × VM + δ × DS) / (α + β + γ + δ)
```

Where:
- `FES` = Fleet Efficiency Score (0-100)
- `RT` = Route optimization efficiency (0-100)
- `FM` = Fuel management score (0-100)
- `VM` = Vehicle maintenance score (0-100)
- `DS` = Driver safety score (0-100)
- `α, β, γ, δ` = Weight coefficients (default: 0.25 each)

### 1.2 Component Calculations

**Route Optimization Efficiency (RT):**
```
RT = 100 × (1 - (Actual_Distance - Optimal_Distance) / Optimal_Distance)
```

**Fuel Management Score (FM):**
```
FM = 100 × (1 - (Actual_Consumption - Baseline_Consumption) / Baseline_Consumption)
```

**Vehicle Maintenance Score (VM):**
```
VM = 100 × (Planned_Maintenance / Total_Maintenance)
```

**Driver Safety Score (DS):**
```
DS = 100 - (Harsh_Events × Event_Penalty + Speeding_Minutes × Speed_Penalty)
```

---

## 2. Route Optimization Algorithms

### 2.1 Clarke-Wright Savings Algorithm

**Purpose:** Generate initial vehicle routes for the Vehicle Routing Problem (VRP)

**Steps:**
1. Calculate savings S(i,j) = d(0,i) + d(0,j) - d(i,j) for all customer pairs
2. Sort savings in descending order
3. For each saving S(i,j):
   - If i and j are in different routes and merge is feasible (capacity, time windows)
   - Merge the routes
4. Return final route set

**Pseudocode:**
```python
def clarke_wright(customers, depot, vehicles):
    # Initialize individual routes for each customer
    routes = [[depot, customer, depot] for customer in customers]
    
    # Calculate all savings
    savings = []
    for i in customers:
        for j in customers:
            if i != j:
                saving = distance(depot, i) + distance(depot, j) - distance(i, j)
                savings.append((saving, i, j))
    
    # Sort savings in descending order
    savings.sort(reverse=True, key=lambda x: x[0])
    
    # Merge routes based on savings
    for saving, i, j in savings:
        route_i = find_route_containing(routes, i)
        route_j = find_route_containing(routes, j)
        
        if route_i != route_j and can_merge(route_i, route_j, vehicles):
            routes = merge_routes(routes, route_i, route_j, i, j)
    
    return routes
```

### 2.2 2-Opt Local Search

**Purpose:** Improve existing routes by eliminating edge crossings

**Steps:**
1. Select a route
2. For each pair of edges (i, i+1) and (j, j+1):
   - Calculate cost of swap: reverse segment between i+1 and j
   - If swap reduces cost, apply it
3. Repeat until no improvement found

**Pseudocode:**
```python
def two_opt(route):
    improved = True
    best_distance = calculate_distance(route)
    
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1:
                    continue
                
                new_route = route[:i] + route[i:j][::-1] + route[j:]
                new_distance = calculate_distance(new_route)
                
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance
                    improved = True
                    break
            if improved:
                break
    
    return route
```

### 2.3 Genetic Algorithm for VRP

**Parameters:**
- Population size: 100-200
- Mutation rate: 0.1-0.2
- Crossover rate: 0.7-0.9
- Generations: 500-1000
- Elitism: Top 10%

**Genetic Operators:**

1. **Selection:** Tournament selection (size=5)
2. **Crossover:** Order crossover (OX) or partially matched crossover (PMX)
3. **Mutation:** Swap mutation, insertion mutation, or inversion mutation

---

## 3. Predictive Maintenance Algorithms

### 3.1 Component Failure Prediction

**Machine Learning Model:** Random Forest or Gradient Boosting

**Features:**
- Vehicle age and mileage
- Engine hours and RPM patterns
- Oil quality indicators
- Temperature trends
- Vibration patterns
- Historical maintenance records
- Driving behavior metrics

**Target:** Time until failure or maintenance needed

**Training Process:**
```python
def train_predictive_model(historical_data):
    # Feature engineering
    features = extract_features(historical_data)
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(features, labels)
    
    # Train model
    model = RandomForestRegressor(n_estimators=100, max_depth=10)
    model.fit(X_train, y_train)
    
    # Validate
    predictions = model.predict(X_test)
    mae = mean_absolute_error(y_test, predictions)
    
    return model
```

### 3.2 Maintenance Scheduling Optimization

**Objective:** Minimize total cost = maintenance_cost + downtime_cost + failure_risk_cost

**Constraints:**
- Maintenance capacity (bay availability, technician hours)
- Vehicle availability requirements
- Priority levels

**Algorithm:** Mixed Integer Linear Programming (MILP)

---

## 4. Driver Behavior Analysis

### 4.1 Safety Score Calculation

```
Safety_Score = 100 - Σ(Event_Count[i] × Severity_Weight[i])
```

**Event Severity Weights:**
- Harsh braking: 2 points
- Harsh acceleration: 1.5 points
- Sharp turn: 1.5 points
- Speeding (minor): 1 point per minute
- Speeding (major): 3 points per minute
- Seatbelt violation: 5 points per occurrence
- Mobile phone use: 10 points per occurrence

**Normalization:** Score clamped to 0-100 range

### 4.2 Driving Pattern Classification

**K-Means Clustering:**

Classify drivers into categories based on behavior patterns:
- **Efficient Drivers:** Low fuel consumption, smooth driving
- **Aggressive Drivers:** Frequent harsh events, high speed variance
- **Cautious Drivers:** Low speed, conservative acceleration
- **Inconsistent Drivers:** High variance across metrics

**Features for clustering:**
- Average acceleration
- Average braking force
- Speed variance
- Idle time percentage
- Fuel efficiency

---

## 5. Fuel Consumption Modeling

### 5.1 Baseline Consumption Model

```
Fuel_Baseline = (a × Distance + b × Weight + c × Speed² + d × Idle_Time + e)
```

Where coefficients (a, b, c, d, e) are determined through regression on historical data.

### 5.2 Real-Time Deviation Detection

```
Deviation = (Actual_Consumption - Predicted_Consumption) / Predicted_Consumption

if Deviation > threshold_high:
    trigger_alert("High fuel consumption")
elif Deviation < threshold_low:
    trigger_alert("Sensor malfunction suspected")
```

**Thresholds:**
- Normal: -10% to +10%
- Warning: -20% to -10% or +10% to +20%
- Alert: < -20% or > +20%

---

## 6. Geofence Algorithms

### 6.1 Point-in-Polygon Test (Ray Casting)

```python
def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        
        j = i
    
    return inside
```

### 6.2 Circular Geofence Entry/Exit

```python
def check_circular_geofence(current_pos, last_pos, center, radius):
    current_distance = haversine_distance(current_pos, center)
    last_distance = haversine_distance(last_pos, center)
    
    if last_distance > radius and current_distance <= radius:
        return "ENTRY"
    elif last_distance <= radius and current_distance > radius:
        return "EXIT"
    else:
        return "NO_EVENT"
```

---

## 7. Trip Analysis Algorithms

### 7.1 Stop Detection

**Algorithm:** Velocity-based with hysteresis

```python
def detect_stops(gps_trace, speed_threshold=5, min_duration=60):
    stops = []
    stop_start = None
    
    for i, point in enumerate(gps_trace):
        if point.speed < speed_threshold:
            if stop_start is None:
                stop_start = i
        else:
            if stop_start is not None:
                duration = gps_trace[i-1].timestamp - gps_trace[stop_start].timestamp
                if duration >= min_duration:
                    stops.append({
                        'start': stop_start,
                        'end': i-1,
                        'duration': duration,
                        'location': gps_trace[stop_start].location
                    })
                stop_start = None
    
    return stops
```

### 7.2 Trip Segmentation

Segment trips into phases:
- **Acceleration phase:** Speed increasing
- **Cruising phase:** Relatively constant speed
- **Deceleration phase:** Speed decreasing
- **Idle phase:** Stopped with engine running

---

## 8. Performance Benchmarking

### 8.1 Algorithm Evaluation Metrics

**Route Optimization:**
- Solution quality: % deviation from optimal
- Computation time: seconds to solution
- Scalability: performance vs. problem size

**Predictive Maintenance:**
- Prediction accuracy: MAE, RMSE
- False positive rate
- False negative rate
- Lead time accuracy

**Driver Scoring:**
- Correlation with actual accidents
- Inter-rater reliability
- Temporal consistency

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*MIT License*
