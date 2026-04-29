# Chapter 3: AI Decision-Making and Path Planning

## From Perception to Action

While perception systems provide the "eyes and ears" of an autonomous ship, artificial intelligence serves as the "brain" - processing sensor data, planning routes, and making navigation decisions that comply with maritime law and ensure safety.

### The Navigation Decision Hierarchy

Autonomous ship AI operates at multiple time scales:

**Strategic Planning (Hours to Days):**
- Ocean crossing route optimization
- Port arrival scheduling
- Fuel and weather optimization
- Multi-day path planning

**Tactical Planning (Minutes to Hours):**
- Traffic separation scheme navigation
- Passing other vessels
- Weather avoidance
- Course adjustments

**Reactive Control (Seconds):**
- Collision avoidance maneuvers
- Emergency stopping
- Sudden obstacle response
- System failure responses

### COLREGs: The Rules of the Road at Sea

The International Regulations for Preventing Collisions at Sea (COLREGs) are the maritime "traffic laws" that all vessels must follow. Autonomous ships must implement these rules in software.

#### Key COLREGs Rules for Autonomous Systems

**Rule 5: Lookout**
"Every vessel shall at all times maintain a proper lookout by sight and hearing as well as by all available means appropriate in the prevailing circumstances."

**Autonomous Implementation:**
```typescript
interface LookoutSystem {
  maintainLookout(sensors: SensorSuite): LookoutReport;
  assessVisibility(cameras: CameraFrame[]): VisibilityConditions;
  identifyRisks(environment: FusedEnvironmentModel): Risk[];
  alertOperators(risks: Risk[]): void;
}

interface LookoutReport {
  timestamp: Date;
  visibility: {
    range: number;           // Meters
    conditions: "good" | "moderate" | "restricted" | "poor";
    limitingFactors: ("fog" | "rain" | "darkness" | "spray" | "snow")[];
  };

  sensorsOperational: {
    radar: boolean;
    lidar: boolean;
    cameras: boolean;
    ais: boolean;
  };

  detectedVessels: number;
  detectedObstacles: number;
  navigationHazards: NavigationHazard[];

  lookoutStatus: "satisfactory" | "degraded" | "insufficient";
}
```

**Rule 8: Action to Avoid Collision**
"Any action to avoid collision shall be taken in accordance with the Rules and shall, if the circumstances permit, be:
- Positive, in ample time, and with due regard to good seamanship
- Made in good time
- Large enough to be readily apparent to another vessel
- Result in passing at a safe distance"

**Autonomous Implementation:**
```typescript
class CollisionAvoidanceExecutive {
  private readonly MIN_ACTION_AMPLITUDE = 30;  // Degrees course change minimum
  private readonly SAFE_PASSING_DISTANCE = 2.0;  // Nautical miles

  planAvoidanceAction(
    threat: FusedVessel,
    ownShip: ShipState,
    environment: FusedEnvironmentModel
  ): AvoidanceManeuver {

    const cpa = this.calculateCPA(threat, ownShip);

    // Determine if action needed
    if (cpa.distance > this.SAFE_PASSING_DISTANCE) {
      return { type: "maintain_course", reason: "safe_cpa" };
    }

    // Calculate action timing (Rule 8: "in ample time")
    const timeToAction = this.calculateActionTiming(cpa, ownShip.speed);
    if (timeToAction < 0) {
      return this.emergencyAction(threat, ownShip);
    }

    // Determine give-way vs. stand-on
    const colregsStatus = this.determineColregsStatus(threat, ownShip);

    if (colregsStatus.role === "give_way") {
      // Calculate maneuver
      const maneuver = this.calculateGiveWayManeuver(threat, ownShip);

      // Ensure action is "large enough to be readily apparent"
      if (Math.abs(maneuver.courseChange) < this.MIN_ACTION_AMPLITUDE) {
        maneuver.courseChange = this.MIN_ACTION_AMPLITUDE * Math.sign(maneuver.courseChange);
      }

      // Verify result is safe passing distance
      const projectedCPA = this.projectCPA(maneuver, threat);
      if (projectedCPA.distance < this.SAFE_PASSING_DISTANCE) {
        maneuver.courseChange *= 1.5;  // Increase action
      }

      return maneuver;

    } else if (colregsStatus.role === "stand_on") {
      // Maintain course and speed unless collision imminent
      if (cpa.time < 5 && cpa.distance < 0.5) {
        return this.emergencyAction(threat, ownShip);
      }
      return { type: "maintain_course", reason: "stand_on_vessel" };
    }

    return { type: "maintain_course", reason: "unclear_situation" };
  }
}
```

**Rule 13: Overtaking**
"Any vessel overtaking any other shall keep out of the way of the vessel being overtaken."

**Rule 14: Head-On Situation**
"When two power-driven vessels are meeting on reciprocal or nearly reciprocal courses so as to involve risk of collision, each shall alter her course to starboard."

**Rule 15: Crossing Situation**
"When two power-driven vessels are crossing so as to involve risk of collision, the vessel which has the other on her own starboard side shall keep out of the way."

**COLREGs Rule Engine:**
```typescript
class ColregsEngine {
  determineEncounterType(target: FusedVessel, ownShip: ShipState): EncounterType {
    const relBearing = this.calculateRelativeBearing(target, ownShip);
    const targetBearing = this.calculateBearingToTarget(ownShip, target);
    const aspect = this.calculateAspectAngle(target, ownShip);

    // Overtaking (Rule 13): Coming up from more than 22.5° abaft beam
    if (relBearing > 112.5 && relBearing < 247.5) {
      return {
        type: "overtaking",
        ownShipRole: "give_way",
        rule: "Rule 13"
      };
    }

    // Head-on (Rule 14): Meeting on reciprocal courses (±10°)
    if (Math.abs(aspect) < 10 && relBearing > 345 || relBearing < 15) {
      return {
        type: "head_on",
        ownShipRole: "give_way",
        action: "alter_course_starboard",
        rule: "Rule 14"
      };
    }

    // Crossing (Rule 15): Other vessel on starboard side
    if (relBearing > 15 && relBearing < 112.5) {
      return {
        type: "crossing",
        ownShipRole: "give_way",
        rule: "Rule 15"
      };
    }

    // Crossing: Own vessel on target's starboard
    if (relBearing > 247.5 && relBearing < 345) {
      return {
        type: "crossing",
        ownShipRole: "stand_on",
        rule: "Rule 15"
      };
    }

    return {
      type: "unclear",
      ownShipRole: "caution",
      rule: "Rule 8 (general)"
    };
  }

  calculateAvoidanceManeuver(encounter: EncounterType, target: FusedVessel): AvoidanceManeuver {
    switch (encounter.type) {
      case "overtaking":
        // Pass on port side preferably, or starboard if safer
        return this.planOvertakingManeuver(target);

      case "head_on":
        // Both vessels alter course to starboard
        return {
          type: "course_change",
          courseChange: 30,  // 30° to starboard
          speedChange: 0,
          duration: "until_clear"
        };

      case "crossing":
        if (encounter.ownShipRole === "give_way") {
          // Alter course to pass astern of stand-on vessel
          return {
            type: "course_change",
            courseChange: 45,  // Substantial alteration
            speedChange: -2,   // Reduce speed slightly
            duration: "until_clear"
          };
        } else {
          // Maintain course and speed
          return {
            type: "maintain_course",
            reason: "stand_on_vessel"
          };
        }

      default:
        // General good seamanship
        return this.generalAvoidanceManeuver(target);
    }
  }
}
```

### Path Planning Algorithms

Autonomous ships use sophisticated algorithms to plan safe, efficient routes.

#### A* Algorithm for Coastal Navigation

A* (A-star) finds the optimal path through a graph of waypoints.

```typescript
class CoastalPathPlanner {
  planRoute(
    start: GeoPosition,
    destination: GeoPosition,
    constraints: NavigationConstraints,
    chart: ElectronicChart
  ): PlannedRoute {

    // 1. Generate waypoint graph
    const graph = this.generateWaypointGraph(chart, constraints);

    // 2. Run A* algorithm
    const path = this.astar(
      graph,
      this.findNearestWaypoint(start, graph),
      this.findNearestWaypoint(destination, graph)
    );

    // 3. Smooth path
    const smoothed = this.smoothPath(path, chart);

    // 4. Add course changes
    const route = this.generateRoute(smoothed);

    // 5. Calculate metrics
    route.distance = this.calculateDistance(route.waypoints);
    route.estimatedTime = this.calculateTime(route, constraints);
    route.fuelConsumption = this.calculateFuel(route, constraints);

    return route;
  }

  private astar(
    graph: WaypointGraph,
    start: Waypoint,
    goal: Waypoint
  ): Waypoint[] {
    const openSet = new PriorityQueue<Waypoint>();
    const cameFrom = new Map<Waypoint, Waypoint>();
    const gScore = new Map<Waypoint, number>();
    const fScore = new Map<Waypoint, number>();

    gScore.set(start, 0);
    fScore.set(start, this.heuristic(start, goal));
    openSet.push(start, fScore.get(start)!);

    while (!openSet.isEmpty()) {
      const current = openSet.pop()!;

      if (current === goal) {
        return this.reconstructPath(cameFrom, current);
      }

      for (const neighbor of graph.neighbors(current)) {
        const tentativeGScore =
          gScore.get(current)! + this.distance(current, neighbor);

        if (!gScore.has(neighbor) || tentativeGScore < gScore.get(neighbor)!) {
          cameFrom.set(neighbor, current);
          gScore.set(neighbor, tentativeGScore);
          fScore.set(neighbor, tentativeGScore + this.heuristic(neighbor, goal));

          if (!openSet.contains(neighbor)) {
            openSet.push(neighbor, fScore.get(neighbor)!);
          }
        }
      }
    }

    throw new Error("No path found");
  }

  private heuristic(a: Waypoint, b: Waypoint): number {
    // Euclidean distance in nautical miles
    return this.calculateDistance(a.position, b.position);
  }
}
```

#### Dynamic Window Approach for Real-Time Obstacle Avoidance

The Dynamic Window Approach (DWA) selects optimal velocity commands in real-time.

```typescript
class DynamicWindowPlanner {
  selectVelocity(
    currentVelocity: Velocity,
    obstacles: Obstacle[],
    goal: GeoPosition,
    shipDynamics: ShipDynamics
  ): Velocity {

    // 1. Calculate dynamic window (reachable velocities)
    const dynamicWindow = this.calculateDynamicWindow(
      currentVelocity,
      shipDynamics
    );

    // 2. Score all velocity candidates
    const scoredVelocities = dynamicWindow.map(v => ({
      velocity: v,
      score: this.scoreVelocity(v, obstacles, goal, shipDynamics)
    }));

    // 3. Select highest-scoring velocity
    const best = scoredVelocities.reduce((a, b) =>
      a.score > b.score ? a : b
    );

    return best.velocity;
  }

  private scoreVelocity(
    velocity: Velocity,
    obstacles: Obstacle[],
    goal: GeoPosition,
    dynamics: ShipDynamics
  ): number {
    // Multi-objective scoring
    const headingScore = this.headingScore(velocity, goal);     // 0-1
    const distanceScore = this.obstacleDistance(velocity, obstacles); // 0-1
    const velocityScore = velocity.speed / dynamics.maxSpeed;    // 0-1

    // Weighted combination
    return (
      0.5 * headingScore +    // Prefer heading toward goal
      0.3 * distanceScore +   // Prefer large clearance from obstacles
      0.2 * velocityScore     // Prefer higher speed
    );
  }

  private calculateDynamicWindow(
    current: Velocity,
    dynamics: ShipDynamics
  ): Velocity[] {
    const dt = 1.0;  // Time step (seconds)
    const velocities: Velocity[] = [];

    // Speed range (considering acceleration limits)
    const minSpeed = Math.max(
      0,
      current.speed - dynamics.maxDeceleration * dt
    );
    const maxSpeed = Math.min(
      dynamics.maxSpeed,
      current.speed + dynamics.maxAcceleration * dt
    );

    // Heading range (considering turn rate limits)
    const minHeading = current.heading - dynamics.maxTurnRate * dt;
    const maxHeading = current.heading + dynamics.maxTurnRate * dt;

    // Sample velocity space
    for (let speed = minSpeed; speed <= maxSpeed; speed += 0.5) {
      for (let heading = minHeading; heading <= maxHeading; heading += 5) {
        velocities.push({ speed, heading: this.normalizeHeading(heading) });
      }
    }

    return velocities;
  }
}
```

### Weather Routing

Optimizing routes for weather conditions can save significant time and fuel.

```typescript
class WeatherRouter {
  optimizeRoute(
    route: PlannedRoute,
    forecast: WeatherForecast,
    ship: ShipCharacteristics
  ): OptimizedRoute {

    // 1. Fetch forecast along route
    const weatherAlongRoute = this.fetchWeatherData(route, forecast);

    // 2. Calculate ship performance in different conditions
    const performance = weatherAlongRoute.map(w =>
      this.calculatePerformance(ship, w)
    );

    // 3. Find optimal departure time
    const optimalDeparture = this.optimizeDepartureTime(
      route,
      weatherAlongRoute,
      ship
    );

    // 4. Adjust waypoints to avoid severe weather
    const adjusted = this.adjustForWeather(route, weatherAlongRoute, ship);

    return {
      waypoints: adjusted.waypoints,
      departureTime: optimalDeparture,
      estimatedArrival: this.calculateArrival(adjusted, performance),
      fuelConsumption: this.calculateFuel(adjusted, performance),
      maxWaveHeight: Math.max(...weatherAlongRoute.map(w => w.waveHeight)),
      weatherRisks: this.assessWeatherRisks(weatherAlongRoute)
    };
  }

  calculatePerformance(
    ship: ShipCharacteristics,
    weather: WeatherCondition
  ): ShipPerformance {
    // Calculate speed loss due to weather
    const speedLoss = this.calculateSpeedLoss(ship, weather);
    const fuelIncrease = this.calculateFuelIncrease(ship, weather);

    return {
      achievableSpeed: ship.serviceSpeed - speedLoss,
      fuelConsumptionRate: ship.baseFuelRate * (1 + fuelIncrease),
      seakeepingAcceptable: this.checkSeakeeping(ship, weather),
      cargoSafe: this.checkCargoSafety(ship, weather)
    };
  }

  private calculateSpeedLoss(
    ship: ShipCharacteristics,
    weather: WeatherCondition
  ): number {
    let speedLoss = 0;

    // Wave resistance
    if (weather.waveHeight > 2) {
      const waveResistance = Math.pow(weather.waveHeight / 2, 1.5);
      speedLoss += waveResistance * 0.8;  // Knots
    }

    // Wind resistance
    const headWind = this.calculateHeadWind(ship.course, weather.windDirection);
    if (headWind > 20) {
      speedLoss += (headWind - 20) * 0.05;
    }

    // Voluntary speed reduction for safety
    if (weather.waveHeight > 6) {
      speedLoss += 3;  // Slow down in heavy seas
    }

    return speedLoss;
  }
}
```

### Machine Learning for Navigation

Modern autonomous ships use machine learning to improve decision-making.

#### Reinforcement Learning for Collision Avoidance

```typescript
class RLNavigationAgent {
  private model: NeuralNetwork;
  private replayBuffer: ExperienceReplayBuffer;

  async selectAction(state: NavigationState): Promise<NavigationAction> {
    // Epsilon-greedy exploration
    if (Math.random() < this.epsilon) {
      return this.randomAction();
    }

    // Neural network inference
    const qValues = await this.model.predict(this.encodeState(state));
    const actionIndex = this.argmax(qValues);

    return this.decodeAction(actionIndex);
  }

  train(experiences: Experience[]): void {
    // Sample mini-batch
    const batch = this.replayBuffer.sample(32);

    // Calculate target Q-values
    const targets = batch.map(exp => {
      if (exp.terminal) {
        return exp.reward;
      } else {
        const nextQValues = this.model.predict(exp.nextState);
        return exp.reward + this.gamma * Math.max(...nextQValues);
      }
    });

    // Train model
    this.model.train(
      batch.map(e => e.state),
      targets
    );
  }

  private encodeState(state: NavigationState): number[] {
    // Encode state as neural network input
    return [
      state.ownShip.speed / 25,              // Normalized
      state.ownShip.course / 360,
      ...state.vessels.map(v => [
        v.range / 10,                         // Normalized to 10 nm
        v.bearing / 360,
        v.speed / 25,
        v.cpa.distance / 5,
        v.cpa.time / 30
      ]).flat()
    ];
  }

  private decodeAction(index: number): NavigationAction {
    const actions = [
      { type: "maintain", courseChange: 0, speedChange: 0 },
      { type: "starboard_small", courseChange: 15, speedChange: 0 },
      { type: "starboard_large", courseChange: 45, speedChange: 0 },
      { type: "port_small", courseChange: -15, speedChange: 0 },
      { type: "port_large", courseChange: -45, speedChange: 0 },
      { type: "reduce_speed", courseChange: 0, speedChange: -3 },
      { type: "increase_speed", courseChange: 0, speedChange: 3 }
    ];

    return actions[index];
  }
}
```

#### Learning from Historical Data

```typescript
class NavigationDataAnalyzer {
  async trainFromHistoricalVoyages(voyages: VoyageData[]): Promise<NavigationModel> {
    // Extract features from successful voyages
    const features = voyages.map(v => this.extractFeatures(v));

    // Train models for different scenarios
    const models = {
      openOcean: await this.trainOpenOceanModel(features),
      coastal: await this.trainCoastalModel(features),
      portApproach: await this.trainPortApproachModel(features),
      restrictedWaters: await this.trainRestrictedWatersModel(features)
    };

    return {
      selectModel(context: NavigationContext) {
        if (context.distanceToLand < 12) return models.portApproach;
        if (context.distanceToLand < 50) return models.coastal;
        if (context.waterDepth < 50) return models.restrictedWaters;
        return models.openOcean;
      },
      models
    };
  }

  private extractFeatures(voyage: VoyageData): VoyageFeatures {
    return {
      encounters: this.analyzeEncounters(voyage.aisData),
      weatherDecisions: this.analyzeWeatherRouting(voyage.route, voyage.weather),
      fuelEfficiency: voyage.fuelConsumed / voyage.distance,
      safetyMargins: this.calculateSafetyMargins(voyage.aisData),
      colregsCompliance: this.assessColregsCompliance(voyage.maneuvers)
    };
  }
}
```

### Real-World AI Performance

**Yara Birkeland Decision-Making:**
- Processes 200+ GB sensor data daily
- Makes navigation decisions every 100ms
- COLREGs compliance: 99.8% in testing
- Weather routing: 15% fuel savings vs. fixed route
- Collision avoidance: Zero incidents in 10,000+ hours testing

**Mayflower Autonomous Ship:**
- Crossed Atlantic using AI Captain (IBM Watson)
- Made over 1,000 autonomous navigation decisions
- Avoided storms, navigated shipping lanes
- Collected ocean data continuously
- Human intervention: Once (equipment issue, not navigation)

### Philosophy: 弘益人間

AI navigation systems embody 弘益人間 by:
- Making safer decisions than human navigators (eliminating fatigue, distraction)
- Optimizing routes for fuel efficiency (reducing environmental impact)
- Never violating COLREGs (ensuring safe interaction with all vessels)
- Operating 24/7 without rest (increasing maritime efficiency)
- Learning from all voyages (improving safety for entire fleet)

---

**Next Chapter:** Remote Monitoring and Control - how shore-based operators supervise autonomous vessels and intervene when necessary.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
