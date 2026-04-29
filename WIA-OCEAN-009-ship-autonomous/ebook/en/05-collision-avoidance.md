# Chapter 5: Collision Avoidance Systems

## Preventing the Unthinkable

Collision at sea is the most common and most feared maritime accident. Autonomous collision avoidance systems must match or exceed human navigator performance, responding to threats faster, more reliably, and in full compliance with international regulations.

### The Collision Avoidance Pipeline

Collision avoidance operates as a continuous cycle:

```typescript
interface CollisionAvoidanceSystem {
  // 1. Detection
  detectThreats(environment: FusedEnvironmentModel): Threat[];

  // 2. Assessment
  assessRisk(threats: Threat[], ownShip: ShipState): RiskAssessment[];

  // 3. Planning
  planAvoidanceManeuver(risk: RiskAssessment): AvoidanceManeuver;

  // 4. Execution
  executeManeuver(maneuver: AvoidanceManeuver): void;

  // 5. Monitoring
  monitorExecution(maneuver: AvoidanceManeuver): ExecutionStatus;

  // 6. Return to course
  planReturnToCourse(clearOfThreat: boolean): CourseResumption;
}
```

### Threat Detection and Classification

**Threat Types:**
```typescript
enum ThreatType {
  VESSEL_UNDERWAY = "vessel_underway",
  VESSEL_STATIONARY = "vessel_stationary",
  SMALL_CRAFT = "small_craft",
  FISHING_VESSEL = "fishing_vessel",
  BUOY = "buoy",
  DEBRIS = "debris",
  LANDMASS = "landmass",
  SHALLOW_WATER = "shallow_water",
  RESTRICTED_AREA = "restricted_area"
}

interface Threat {
  id: string;
  type: ThreatType;
  position: GeoPosition;
  velocity?: Velocity;  // Null for stationary objects

  // Detection sources
  sources: {
    radar?: RadarTrack;
    lidar?: LidarObject;
    ais?: AISTarget;
    chart?: ChartFeature;
  };

  // Threat characteristics
  characteristics: {
    size: { length: number; width: number };
    maneuverability: "high" | "medium" | "low" | "stationary";
    predictability: number;  // 0-1 (how predictable is behavior)
    navigationStatus?: NavigationStatus;
  };

  // Temporal data
  firstDetected: Date;
  lastUpdated: Date;
  trackHistory: Position[];
}
```

**Multi-Stage Detection:**
```typescript
class ThreatDetectionSystem {
  private readonly DETECTION_RANGES = {
    longRange: 20,    // Nautical miles - radar surveillance
    mediumRange: 5,   // Nautical miles - detailed tracking
    shortRange: 1,    // Nautical miles - immediate threats
    closeRange: 0.25  // Nautical miles - collision imminent
  };

  detectThreats(environment: FusedEnvironmentModel): Threat[] {
    const threats: Threat[] = [];

    // 1. Long-range surveillance (radar)
    const longRangeContacts = environment.vessels.filter(
      v => v.distance <= this.DETECTION_RANGES.longRange
    );

    for (const contact of longRangeContacts) {
      const threat = this.createThreat(contact);
      threats.push(threat);
    }

    // 2. Chart-based hazards
    const chartHazards = this.detectChartHazards(
      environment.ownShip.position,
      environment.ownShip.course
    );
    threats.push(...chartHazards);

    // 3. Dynamic obstacles (lidar/vision)
    const dynamicObstacles = this.detectDynamicObstacles(
      environment.obstacles,
      environment.ownShip
    );
    threats.push(...dynamicObstacles);

    return threats;
  }

  private detectChartHazards(
    position: GeoPosition,
    course: number
  ): Threat[] {
    // Project path ahead
    const projectedPath = this.projectPath(position, course, distance: 10);

    // Check for land, shallow water, restricted areas
    const hazards = this.chart.findHazardsAlongPath(projectedPath);

    return hazards.map(h => ({
      id: `chart-${h.id}`,
      type: this.classifyChartHazard(h),
      position: h.position,
      velocity: null,  // Stationary
      sources: { chart: h },
      characteristics: {
        size: h.extent,
        maneuverability: "stationary",
        predictability: 1.0
      },
      firstDetected: new Date(),
      lastUpdated: new Date(),
      trackHistory: []
    }));
  }
}
```

### Risk Assessment

**Collision Risk Calculation:**
```typescript
interface RiskAssessment {
  threat: Threat;
  risk: {
    cpa: CPACalculation;
    timeToClosestApproach: number;  // Minutes
    collisionProbability: number;   // 0-1
    severity: number;                // 0-1 (impact severity if collision occurs)
    overallRisk: number;             // Probability × severity
  };

  colregs: {
    applicable: boolean;
    encounterType: "head_on" | "crossing" | "overtaking" | "none";
    ownShipRole: "give_way" | "stand_on" | "both_give_way" | "unclear";
  };

  actionRequired: {
    urgency: "routine" | "caution" | "urgent" | "emergency";
    recommendedAction: string;
    timeToAction: number;  // Seconds until action must begin
  };
}

class CollisionRiskAssessor {
  private readonly RISK_THRESHOLDS = {
    safe: 0.1,
    caution: 0.3,
    warning: 0.6,
    critical: 0.8
  };

  assessRisk(threat: Threat, ownShip: ShipState): RiskAssessment {
    // 1. Calculate CPA
    const cpa = this.calculateCPA(threat, ownShip);

    // 2. Assess collision probability
    const collisionProb = this.calculateCollisionProbability(threat, ownShip, cpa);

    // 3. Estimate severity
    const severity = this.estimateSeverity(threat, ownShip);

    // 4. Overall risk
    const overallRisk = collisionProb * severity;

    // 5. COLREGs analysis
    const colregs = this.analyzeColregs(threat, ownShip);

    // 6. Determine action required
    const actionRequired = this.determineActionRequired(
      overallRisk,
      cpa,
      colregs
    );

    return {
      threat,
      risk: {
        cpa,
        timeToClosestApproach: cpa.time,
        collisionProbability: collisionProb,
        severity,
        overallRisk
      },
      colregs,
      actionRequired
    };
  }

  private calculateCollisionProbability(
    threat: Threat,
    ownShip: ShipState,
    cpa: CPACalculation
  ): number {
    let probability = 0;

    // Base probability from CPA distance
    if (cpa.distance < 0.1) {
      probability = 0.9;  // Very high risk
    } else if (cpa.distance < 0.5) {
      probability = 0.7;
    } else if (cpa.distance < 1.0) {
      probability = 0.4;
    } else if (cpa.distance < 2.0) {
      probability = 0.2;
    } else {
      probability = 0.05;
    }

    // Adjust for target predictability
    const unpredictability = 1 - threat.characteristics.predictability;
    probability += unpredictability * 0.2;

    // Adjust for visibility conditions
    const visibility = this.getVisibility();
    if (visibility < 2) {  // Restricted visibility
      probability *= 1.5;
    }

    // Adjust for target maneuverability (fishing vessels less predictable)
    if (threat.type === ThreatType.FISHING_VESSEL) {
      probability *= 1.3;
    }

    // Adjust for time to CPA (more time = lower probability)
    if (cpa.time < 5) {
      probability *= 1.2;
    } else if (cpa.time > 30) {
      probability *= 0.8;
    }

    return Math.min(probability, 1.0);
  }

  private estimateSeverity(threat: Threat, ownShip: ShipState): number {
    // Severity based on:
    // - Relative speeds (kinetic energy)
    // - Vessel sizes
    // - Cargo type
    // - Angle of collision

    const ownSpeed = ownShip.speed;
    const targetSpeed = threat.velocity?.speed || 0;
    const relativeSpeed = Math.sqrt(
      ownSpeed ** 2 + targetSpeed ** 2 -
      2 * ownSpeed * targetSpeed * Math.cos(this.relativeAngle(threat, ownShip))
    );

    // Kinetic energy proxy
    const kineticEnergy = 0.5 * threat.characteristics.size.length * relativeSpeed ** 2;

    // Normalize to 0-1 scale
    let severity = Math.min(kineticEnergy / 10000, 1.0);

    // Increase for tanker collisions (pollution risk)
    if (threat.type === ThreatType.VESSEL_UNDERWAY &&
        threat.sources.ais?.staticData?.vesselType === "tanker") {
      severity *= 1.5;
    }

    return Math.min(severity, 1.0);
  }
}
```

### Collision Avoidance Decision-Making

**Decision Tree:**
```typescript
class CollisionAvoidanceDecisionMaker {
  decideAction(assessment: RiskAssessment, ownShip: ShipState): AvoidanceAction {
    const { risk, colregs, actionRequired } = assessment;

    // Emergency: Immediate action regardless of COLREGs
    if (actionRequired.urgency === "emergency") {
      return this.emergencyAvoidance(assessment, ownShip);
    }

    // Apply COLREGs if applicable
    if (colregs.applicable) {
      return this.colregsAvoidance(assessment, ownShip);
    }

    // General good seamanship
    if (actionRequired.urgency === "urgent" || actionRequired.urgency === "caution") {
      return this.prudentAvoidance(assessment, ownShip);
    }

    // Monitor situation
    return {
      type: "monitor",
      reason: "Risk within acceptable limits"
    };
  }

  private colregsAvoidance(
    assessment: RiskAssessment,
    ownShip: ShipState
  ): AvoidanceAction {
    const { colregs, threat } = assessment;

    if (colregs.ownShipRole === "give_way") {
      // Must take early, substantial action

      switch (colregs.encounterType) {
        case "head_on":
          // Rule 14: Both alter course to starboard
          return {
            type: "alter_course",
            direction: "starboard",
            amount: 30,  // Degrees
            timing: "immediate",
            reason: "COLREGs Rule 14 - Head-on encounter"
          };

        case "crossing":
          // Rule 15: Alter course to pass astern
          const bearing = this.calculateBearing(ownShip, threat);

          if (bearing < 180) {
            // Target ahead, alter course to starboard to pass astern
            return {
              type: "alter_course",
              direction: "starboard",
              amount: 45,
              timing: "immediate",
              reason: "COLREGs Rule 15 - Crossing, give way"
            };
          } else {
            // Target astern, reduce speed
            return {
              type: "reduce_speed",
              amount: 5,  // Knots
              timing: "immediate",
              reason: "COLREGs Rule 15 - Crossing, give way"
            };
          }

        case "overtaking":
          // Rule 13: Keep clear, pass on either side
          const clearSide = this.determineClearSide(threat, ownShip);

          return {
            type: "alter_course",
            direction: clearSide,
            amount: 30,
            timing: "immediate",
            reason: "COLREGs Rule 13 - Overtaking"
          };

        default:
          return this.prudentAvoidance(assessment, ownShip);
      }

    } else if (colregs.ownShipRole === "stand_on") {
      // Maintain course and speed, but be prepared to act

      const cpa = assessment.risk.cpa;

      if (cpa.distance < 0.5 && cpa.time < 5) {
        // Stand-on vessel may act when collision imminent
        return {
          type: "alter_course",
          direction: "starboard",  // Never to port for stand-on
          amount: 45,
          timing: "immediate",
          reason: "COLREGs Rule 17 - Stand-on vessel action to avoid collision"
        };
      }

      return {
        type: "maintain_course",
        reason: "COLREGs - Stand-on vessel"
      };
    }

    return this.prudentAvoidance(assessment, ownShip);
  }

  private emergencyAvoidance(
    assessment: RiskAssessment,
    ownShip: ShipState
  ): AvoidanceAction {
    // Collision imminent - take most effective action immediately

    const threat = assessment.threat;
    const bearing = this.calculateRelativeBearing(ownShip, threat);

    // Determine best evasive action
    if (bearing > 180) {
      // Threat coming from astern - increase speed
      return {
        type: "emergency_maneuver",
        course: "increase_speed",
        speed: ownShip.maxSpeed,
        reason: "Emergency: Collision imminent from astern"
      };
    } else if (bearing < 90 || bearing > 270) {
      // Threat to port or starboard - turn away
      const turnDirection = bearing < 180 ? "starboard" : "port";

      return {
        type: "emergency_maneuver",
        course: "hard_turn",
        direction: turnDirection,
        rudderAngle: 35,  // Hard rudder
        reason: `Emergency: Collision imminent from ${turnDirection} side`
      };
    } else {
      // Threat ahead - emergency stop or hard turn
      return {
        type: "emergency_maneuver",
        course: "emergency_stop",
        reason: "Emergency: Collision imminent ahead"
      };
    }
  }
}
```

### Maneuver Execution

**Autopilot Integration:**
```typescript
class ManeuverExecutor {
  private autopilot: AutopilotSystem;
  private steeringControl: SteeringControl;
  private engineControl: EngineControl;

  async executeManeuver(maneuver: AvoidanceManeuver): Promise<void> {
    console.log(`Executing maneuver: ${maneuver.type}`);

    switch (maneuver.type) {
      case "alter_course":
        await this.executeCourseChange(maneuver);
        break;

      case "reduce_speed":
        await this.executeSpeedReduction(maneuver);
        break;

      case "emergency_stop":
        await this.executeEmergencyStop(maneuver);
        break;

      case "maintain_course":
        // No action needed
        console.log("Maintaining current course and speed");
        break;
    }

    // Log maneuver for COLREGs compliance audit
    this.logManeuver(maneuver);
  }

  private async executeCourseChange(maneuver: CourseChangeManeuver): Promise<void> {
    const currentCourse = this.autopilot.getCurrentCourse();
    const newCourse = this.calculateNewCourse(currentCourse, maneuver);

    console.log(`Altering course from ${currentCourse}° to ${newCourse}°`);

    // Calculate turn rate (max 3°/second for cargo ship)
    const turnRate = Math.min(Math.abs(maneuver.amount) / 10, 3);

    // Command autopilot
    await this.autopilot.setCourse(newCourse, {
      turnRate,
      urgency: maneuver.timing
    });

    // Monitor execution
    await this.monitorCourseChange(newCourse);

    console.log(`Course change complete. Now on ${newCourse}°`);
  }

  private async executeSpeedReduction(maneuver: SpeedChangeManeuver): Promise<void> {
    const currentSpeed = this.engineControl.getCurrentSpeed();
    const targetSpeed = currentSpeed - maneuver.amount;

    console.log(`Reducing speed from ${currentSpeed} to ${targetSpeed} knots`);

    // Gradual reduction (0.5 knots per minute typically)
    const reductionRate = 0.5 / 60;  // Per second

    await this.engineControl.setSpeed(targetSpeed, {
      rate: reductionRate,
      urgency: maneuver.timing
    });

    console.log(`Speed reduction complete. Now at ${targetSpeed} knots`);
  }

  private async executeEmergencyStop(maneuver: EmergencyStopManeuver): Promise<void> {
    console.log("EMERGENCY STOP INITIATED");

    // Full astern
    await this.engineControl.setThrottle("full_astern");

    // Sound whistle (5 short blasts - danger signal)
    await this.whistleControl.sound({ pattern: "short", count: 5 });

    // Broadcast emergency AIS message
    await this.aisTransmitter.broadcast({
      type: "safety_message",
      text: "Emergency stop in progress"
    });

    console.log("Emergency stop in progress");
  }
}
```

### Multi-Threat Scenarios

Handling multiple simultaneous threats:

```typescript
class MultiThreatManager {
  resolveMultipleThreats(
    assessments: RiskAssessment[],
    ownShip: ShipState
  ): AvoidanceManeuver {
    // Sort by risk level
    const sorted = assessments.sort((a, b) => b.risk.overallRisk - a.risk.overallRisk);

    // Check if single maneuver can clear all threats
    const comprehensiveManeuver = this.findComprehensiveManeuver(sorted, ownShip);

    if (comprehensiveManeuver) {
      return comprehensiveManeuver;
    }

    // Handle threats sequentially by priority
    return this.sequentialManeuvers(sorted, ownShip);
  }

  private findComprehensiveManeuver(
    threats: RiskAssessment[],
    ownShip: ShipState
  ): AvoidanceManeuver | null {
    // Try different course changes
    for (let courseChange = 15; courseChange <= 90; courseChange += 15) {
      for (const direction of ["starboard", "port"]) {
        const testManeuver = {
          type: "alter_course",
          direction,
          amount: courseChange
        };

        // Simulate this maneuver
        const simulated = this.simulateManeuver(testManeuver, ownShip);

        // Check if it clears all threats
        const clearsAll = threats.every(threat => {
          const newCPA = this.calculateCPA(threat.threat, simulated);
          return newCPA.distance > 2.0;  // 2 nm safe distance
        });

        if (clearsAll) {
          return testManeuver;
        }
      }
    }

    return null;  // No single maneuver clears all threats
  }
}
```

### Return to Planned Route

After avoiding a collision, return to the original route:

```typescript
class CourseResumptionPlanner {
  planReturnToCourse(
    currentPosition: GeoPosition,
    originalRoute: Route,
    clearedThreats: Threat[]
  ): CourseResumptionPlan {
    // 1. Verify threats are clear
    const allClear = clearedThreats.every(t => {
      const distance = this.calculateDistance(currentPosition, t.position);
      return distance > 3.0;  // 3 nm minimum
    });

    if (!allClear) {
      return {
        action: "continue_avoidance",
        reason: "Threats not yet clear"
      };
    }

    // 2. Find optimal point to rejoin route
    const rejoinPoint = this.findOptimalRejoinPoint(
      currentPosition,
      originalRoute
    );

    // 3. Calculate course back to route
    const returnCourse = this.calculateCourse(currentPosition, rejoinPoint);

    // 4. Plan turn to resume original course
    return {
      action: "return_to_route",
      turnTowards: returnCourse,
      rejoinPoint,
      estimatedTime: this.calculateTime(currentPosition, rejoinPoint),
      reason: "Threats clear, resuming planned route"
    };
  }
}
```

### Testing and Validation

**Simulation Testing:**
```typescript
interface CollisionAvoidanceTest {
  scenario: {
    name: string;
    description: string;
    ownShip: ShipState;
    threats: Threat[];
    environment: EnvironmentConditions;
  };

  expectedOutcome: {
    minCPA: number;           // Minimum acceptable CPA
    colregsCompliant: boolean;
    maneuverType: string;
    maxExecutionTime: number; // Seconds
  };

  results?: {
    actualCPA: number;
    compliant: boolean;
    maneuverExecuted: AvoidanceManeuver;
    executionTime: number;
    passed: boolean;
  };
}

class CollisionAvoidanceTester {
  private scenarios: CollisionAvoidanceTest[] = [
    {
      scenario: {
        name: "Head-on encounter",
        description: "Two vessels meeting on reciprocal courses",
        ownShip: { position: {lat: 60, lon: 5}, course: 90, speed: 15 },
        threats: [{
          type: ThreatType.VESSEL_UNDERWAY,
          position: {lat: 60, lon: 5.5},
          velocity: { course: 270, speed: 15 }
        }],
        environment: { visibility: 10, weather: "good" }
      },
      expectedOutcome: {
        minCPA: 1.0,
        colregsCompliant: true,
        maneuverType: "alter_course_starboard",
        maxExecutionTime: 300
      }
    },
    // ... more scenarios
  ];

  async runAllTests(): Promise<TestResults> {
    const results = await Promise.all(
      this.scenarios.map(test => this.runTest(test))
    );

    const passed = results.filter(r => r.passed).length;
    const failed = results.length - passed;

    return {
      total: results.length,
      passed,
      failed,
      passRate: passed / results.length,
      details: results
    };
  }

  private async runTest(test: CollisionAvoidanceTest): Promise<TestResults> {
    const startTime = Date.now();

    // Run collision avoidance system
    const system = new CollisionAvoidanceSystem();
    const threats = system.detectThreats(test.scenario);
    const assessments = threats.map(t => system.assessRisk(t, test.scenario.ownShip));
    const maneuver = system.planAvoidanceManeuver(assessments[0]);

    system.executeManeuver(maneuver);

    // Simulate execution
    const simulation = this.simulateExecution(maneuver, test.scenario);

    const executionTime = Date.now() - startTime;

    // Verify results
    const passed =
      simulation.minCPA >= test.expectedOutcome.minCPA &&
      simulation.colregsCompliant === test.expectedOutcome.colregsCompliant &&
      executionTime <= test.expectedOutcome.maxExecutionTime;

    return {
      ...test,
      results: {
        actualCPA: simulation.minCPA,
        compliant: simulation.colregsCompliant,
        maneuverExecuted: maneuver,
        executionTime,
        passed
      }
    };
  }
}
```

### Real-World Performance

**Yara Birkeland Collision Avoidance:**
- 10,000+ hours autonomous testing
- 500+ collision avoidance scenarios tested
- Zero collisions in testing
- Average response time: 3.2 seconds from threat detection to maneuver start
- COLREGs compliance: 99.8%

**Mayflower Atlantic Crossing:**
- 47 collision avoidance maneuvers during crossing
- All maneuvers successful
- Closest approach: 0.8 nautical miles (safe)
- Average time from detection to maneuver: 4.1 seconds

### Philosophy: 弘益人間

Collision avoidance systems embody 弘益人間 by:
- Protecting lives at sea through faster, more reliable threat response
- Eliminating collision accidents caused by human error
- Following maritime law precisely, ensuring safe interactions
- Operating 24/7 without fatigue, maintaining constant vigilance
- Sharing knowledge across fleets to improve safety for all mariners

---

**Next Chapter:** Regulatory and Legal Framework - international regulations, liability, and certification for autonomous ships.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
