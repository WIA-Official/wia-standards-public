# @wia/space - WIA-SPACE TypeScript SDK

> 弘益人間 (Benefit All Humanity)

TypeScript SDK for the WIA-SPACE standard, providing comprehensive tools for space operations, orbital mechanics, mission planning, spacecraft control, and space debris tracking.

## Installation

```bash
npm install @wia/space
```

## Features

- **Orbital Mechanics** - Calculate orbits, state vectors, and trajectories
- **Mission Planning** - Design and optimize space missions
- **Telemetry Processing** - Process and validate spacecraft telemetry
- **Spacecraft Control** - Generate commands and control spacecraft systems
- **Debris Tracking** - Track space debris and predict conjunctions
- **Launch Operations** - Plan launches and select launch vehicles

## Quick Start

```typescript
import { createWIASpace, OrbitType, SpacecraftType } from '@wia/space';

// Create WIASpace instance
const wiaSpace = createWIASpace();

// Create a spacecraft
const spacecraft = wiaSpace.createSpacecraft({
  name: 'Explorer-1',
  type: SpacecraftType.SATELLITE,
  orbitType: OrbitType.LEO,
  mass: 500, // kg
});

// Plan a mission
const mission = wiaSpace.planMission({
  name: 'Earth Observation Mission',
  description: 'Monitor climate and environmental changes',
  spacecraft: [spacecraft],
  duration: 365, // days
});

console.log('Mission created:', mission.name);
```

## Orbital Mechanics

Calculate orbital elements and propagate orbits:

```typescript
import { OrbitalMechanicsEngine, OrbitalElements } from '@wia/space';

const orbitEngine = new OrbitalMechanicsEngine();

// Define orbital elements
const elements: OrbitalElements = {
  semiMajorAxis: 6771, // km (400 km altitude LEO)
  eccentricity: 0.001,
  inclination: 51.6, // degrees (ISS orbit)
  longitudeOfAscendingNode: 0,
  argumentOfPeriapsis: 0,
  trueAnomaly: 0,
  epoch: new Date()
};

// Convert to state vector
const stateVector = orbitEngine.elementsToStateVector(elements);
console.log('Position:', stateVector.position);
console.log('Velocity:', stateVector.velocity);

// Calculate orbital period
const period = orbitEngine.calculateOrbitalPeriod(elements.semiMajorAxis);
console.log(`Orbital period: ${period / 60} minutes`);

// Calculate Hohmann transfer
const deltaV = orbitEngine.calculateHohmannTransfer(6771, 42164); // LEO to GEO
console.log(`Total delta-V required: ${deltaV} km/s`);
```

## Mission Planning

Plan missions and calculate contact windows:

```typescript
import { MissionPlanningEngine } from '@wia/space';

const planner = new MissionPlanningEngine();

// Calculate contact windows with ground station
const groundStation = {
  id: 'GS-001',
  name: 'Mission Control',
  location: { latitude: 37.4, longitude: -122.1, altitude: 0 },
  antennaSize: 12,
  frequency: [2200, 8400],
  capabilities: ['Telemetry', 'Command'],
  contactSchedule: []
};

const contactWindows = planner.calculateContactWindows(
  stateVector,
  groundStation,
  86400 // 24 hours
);

console.log(`Found ${contactWindows.length} contact opportunities`);
```

## Telemetry Processing

Process and validate spacecraft telemetry:

```typescript
import { TelemetryProcessingEngine } from '@wia/space';

const telemetryEngine = new TelemetryProcessingEngine();

// Validate telemetry
const validation = telemetryEngine.validateTelemetry(telemetry);

if (!validation.valid) {
  console.error('Telemetry errors:', validation.errors);
}

if (validation.warnings.length > 0) {
  console.warn('Telemetry warnings:', validation.warnings);
}

// Detect anomalies
const anomalies = telemetryEngine.detectAnomalies(telemetryHistory);
if (anomalies.length > 0) {
  console.warn('Anomalies detected:', anomalies);
}
```

## Spacecraft Control

Generate commands and control spacecraft:

```typescript
import { SpacecraftControlEngine, CommandType } from '@wia/space';

const controlEngine = new SpacecraftControlEngine();

// Generate maneuver command
const maneuver = {
  id: 'MNV-001',
  type: 'ORBITAL_INSERTION',
  deltaV: { x: 0, y: 0.5, z: 0 },
  executionTime: new Date(Date.now() + 3600000),
  duration: 120,
  fuelMass: 10,
  status: 'PLANNED'
};

const command = controlEngine.generateManeuverCommand(spacecraft, maneuver);
console.log('Command generated:', command.id);

// Calculate attitude control
const controlTorque = controlEngine.calculateAttitudeControl(
  currentOrientation,
  targetOrientation
);
console.log('Control torque:', controlTorque);
```

## Debris Tracking

Track space debris and predict collisions:

```typescript
import { DebrisTrackingEngine } from '@wia/space';

const debrisEngine = new DebrisTrackingEngine();

// Identify conjunction events
const conjunctions = debrisEngine.identifyConjunctions(
  spacecraft,
  debrisCatalog,
  5 // 5 km threshold
);

for (const conj of conjunctions) {
  console.log(`Conjunction with ${conj.secondaryObject}`);
  console.log(`Miss distance: ${conj.missDistance} km`);
  console.log(`Collision probability: ${conj.collisionProbability}`);

  if (conj.status === 'ACTIONABLE') {
    console.warn('Avoidance maneuver recommended!');
  }
}
```

## Launch Operations

Plan launches and select vehicles:

```typescript
import { LaunchOperationsEngine } from '@wia/space';

const launchEngine = new LaunchOperationsEngine();

// Calculate optimal launch time
const launchSite = { latitude: 28.5, longitude: -80.6 }; // Cape Canaveral
const launchTime = launchEngine.calculateOptimalLaunchTime(launchSite, targetOrbit);

// Select launch vehicle
const selectedVehicle = launchEngine.selectLaunchVehicle(
  spacecraft,
  OrbitType.LEO,
  availableVehicles
);

if (selectedVehicle) {
  console.log(`Selected: ${selectedVehicle.name}`);
  console.log(`Cost: $${selectedVehicle.cost.toLocaleString()}`);
}

// Monitor launch countdown
const countdown = launchEngine.monitorLaunchCountdown(launchTime, checks);
if (countdown.readyForLaunch) {
  console.log('GO for launch!');
} else {
  console.warn('Launch holds:', countdown.issues);
}
```

## API Reference

### Core Classes

- `WIASpace` - Main SDK class
- `OrbitalMechanicsEngine` - Orbital calculations
- `MissionPlanningEngine` - Mission planning tools
- `TelemetryProcessingEngine` - Telemetry processing
- `SpacecraftControlEngine` - Spacecraft control
- `DebrisTrackingEngine` - Debris tracking
- `LaunchOperationsEngine` - Launch operations

### Types

See [types.ts](./src/types.ts) for complete type definitions including:

- `Spacecraft` - Spacecraft configuration
- `OrbitalElements` - Keplerian orbital elements
- `StateVector` - Position and velocity
- `Telemetry` - Spacecraft telemetry data
- `Command` - Spacecraft commands
- `Mission` - Mission configuration
- `SpaceDebris` - Debris object data
- `LaunchVehicle` - Launch vehicle specs

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test

# Lint
npm run lint

# Format
npm run format
```

## Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

This SDK embodies the principle of 弘益人間 by providing tools to advance humanity's exploration and utilization of space for the benefit of all. Use these tools to create space systems that expand human knowledge and capabilities beyond Earth.

## License

MIT

## Contributing

Contributions are welcome! Please see the [WIA Standards Repository](https://github.com/WIA-Official/wia-standards) for guidelines.

## Links

- [GitHub Repository](https://github.com/WIA-Official/wia-standards/tree/main/space)
- [Issue Tracker](https://github.com/WIA-Official/wia-standards/issues)
- [WIA Standards](https://github.com/WIA-Official/wia-standards)

---

© 2025 SmileStory Inc. / WIA
