# Chapter 2: Physical Sensors (Temperature, Pressure, Salinity)

## Measuring the Ocean's Fundamental Properties

Physical sensors measure the basic state variables of seawater: temperature, pressure (depth), and salinity. These three parameters define seawater density, which drives ocean circulation, stratification, and mixing. They are the most widely measured ocean properties, with observations dating back centuries and modern networks providing global coverage.

### Temperature Sensors

Ocean temperature ranges from -2°C in polar seas to over 30°C in tropical surface waters, and from 0-2°C in the deep ocean to 400°C at hydrothermal vents. Temperature affects biological metabolism, chemical reaction rates, acoustic propagation, and seawater density.

#### Thermistor-Based Sensors

Modern ocean temperature sensors primarily use **thermistors** - semiconductor devices whose electrical resistance changes predictably with temperature.

**Principle:** Resistance decreases as temperature increases (negative temperature coefficient)

**Advantages:**
- High precision: ±0.001°C achievable
- Fast response time: < 0.1 second
- Small size: can be millimeters
- Low power consumption
- Stable calibration

**Implementation:**
```typescript
interface ThermistorSensor {
  type: "thermistor";
  serialNumber: string;

  // Steinhart-Hart equation coefficients
  calibration: {
    A: number;
    B: number;
    C: number;
    D?: number;              // Optional fourth-order term
  };

  // Measurement specifications
  range: {
    min: number;             // °C
    max: number;             // °C
  };
  resolution: number;        // °C
  accuracy: number;          // °C
  responseTime: number;      // seconds

  // Current reading
  resistance: number;        // Ohms
}

function calculateTemperature(sensor: ThermistorSensor): number {
  const R = sensor.resistance;
  const lnR = Math.log(R);

  // Steinhart-Hart equation: 1/T = A + B*ln(R) + C*(ln(R))^3
  const { A, B, C, D } = sensor.calibration;

  const invT = A + B * lnR + C * Math.pow(lnR, 3) + (D || 0) * Math.pow(lnR, 4);

  const kelvin = 1 / invT;
  const celsius = kelvin - 273.15;

  return celsius;
}

// Example usage
const thermistor: ThermistorSensor = {
  type: "thermistor",
  serialNumber: "SBE3-001234",
  calibration: {
    A: 0.00127534,
    B: 0.000235267,
    C: 0.00000009876,
  },
  range: { min: -5, max: 35 },
  resolution: 0.0001,
  accuracy: 0.002,
  responseTime: 0.065,
  resistance: 5423.7
};

const temperature = calculateTemperature(thermistor);
console.log(`Temperature: ${temperature.toFixed(4)}°C`);
```

#### Platinum Resistance Thermometers (PRT)

For the highest accuracy applications, **platinum resistance thermometers** are used:

**Principle:** Resistance of pure platinum wire increases linearly with temperature

**Advantages:**
- Excellent stability: drift < 0.001°C/year
- Very high accuracy: ±0.0001°C possible
- International standard (ITS-90 temperature scale)
- Reproducible manufacturing

**Disadvantages:**
- Slower response time
- Higher cost
- Larger size

#### Temperature Measurement System

A complete temperature measurement integrates sensor, signal conditioning, and data processing:

```typescript
interface TemperatureMeasurement {
  timestamp: Date;
  location: GeographicPosition;

  // Raw measurements
  resistance: number;              // Ohms
  temperature: number;             // °C, ITS-90 scale

  // Corrections applied
  corrections: {
    sensorDrift: number;           // °C
    selfHeating: number;           // °C, thermistor power dissipation
    pressureEffect: number;        // °C, deep ocean pressure
    responseTimeCorrection: number; // °C, for rapid profiling
  };

  // Final value
  temperatureCorrected: number;    // °C
  uncertainty: number;             // °C, total measurement uncertainty

  // Quality
  qualityFlag: QualityFlag;

  // Metadata
  sensor: ThermistorSensor | PlatinumRT;
  platform: string;                // CTD, mooring, float, etc.
}

function applyTemperatureCorrections(
  rawTemp: number,
  sensor: ThermistorSensor,
  environment: { pressure: number; speed: number }
): TemperatureMeasurement {

  let correctedTemp = rawTemp;
  const corrections = {
    sensorDrift: 0,
    selfHeating: 0,
    pressureEffect: 0,
    responseTimeCorrection: 0
  };

  // Pressure effect: Thermistors experience small resistance change under pressure
  if (environment.pressure > 100) {  // > 100 dbar = 100m depth
    const pressureCoeff = 0.000005;  // °C/dbar, sensor-specific
    corrections.pressureEffect = pressureCoeff * environment.pressure;
    correctedTemp += corrections.pressureEffect;
  }

  // Self-heating: Measurement current causes slight heating
  if (sensor.type === "thermistor") {
    corrections.selfHeating = -0.0002;  // Typical value
    correctedTemp += corrections.selfHeating;
  }

  // Response time correction for fast profiling
  if (environment.speed > 0.5) {  // m/s descent/ascent rate
    const tau = sensor.responseTime;
    const gradient = 0.1;  // °C/m, estimated temperature gradient
    corrections.responseTimeCorrection = tau * environment.speed * gradient;
    correctedTemp += corrections.responseTimeCorrection;
  }

  return {
    timestamp: new Date(),
    location: { latitude: 0, longitude: 0, depth: environment.pressure, datum: "WGS84" },
    resistance: sensor.resistance,
    temperature: rawTemp,
    corrections,
    temperatureCorrected: correctedTemp,
    uncertainty: 0.002,  // Combined uncertainty
    qualityFlag: { status: "good", automated: true, tests: [] },
    sensor,
    platform: "CTD"
  };
}
```

### Pressure Sensors

Pressure sensors measure depth (1 dbar ≈ 1 meter) and detect pressure variations from tides, tsunamis, and internal waves.

#### Strain Gauge Pressure Sensors

Most ocean pressure sensors use strain gauges bonded to a diaphragm:

**Principle:** Pressure deflects diaphragm, straining bonded resistors, changing resistance

**Types:**
- **Piezoresistive silicon:** High sensitivity, temperature-dependent
- **Metal foil strain gauge:** Robust, stable, lower sensitivity
- **Resonant quartz:** Highest accuracy, frequency output, expensive

```typescript
interface PressureSensor {
  type: "strain_gauge" | "quartz_resonator";
  serialNumber: string;

  calibration: {
    // Pressure = (frequency^2 * C) + D, for quartz
    // or Pressure = A + B*voltage, for strain gauge
    coefficients: Record<string, number>;
    temperatureCompensation: {
      enabled: boolean;
      coefficients?: number[];
    };
  };

  range: {
    min: number;              // dbar (decibars)
    max: number;              // dbar
  };
  resolution: number;         // dbar
  accuracy: number;           // dbar

  // Drift characteristics
  drift: {
    rate: number;             // dbar/year
    lastCalibration: Date;
  };
}

function calculatePressure(
  sensor: PressureSensor,
  rawFrequency: number,
  temperature: number
): number {
  const { coefficients, temperatureCompensation } = sensor.calibration;

  let pressure: number;

  if (sensor.type === "quartz_resonator") {
    // Digiquartz formula: P = C*(T^2) + D
    const T = rawFrequency;
    pressure = coefficients.C * Math.pow(T, 2) + coefficients.D;

    // Temperature compensation
    if (temperatureCompensation.enabled && temperatureCompensation.coefficients) {
      const [U0, Y1, Y2, Y3] = temperatureCompensation.coefficients;
      const tempCorrection = Y1 * temperature + Y2 * Math.pow(temperature, 2) +
                            Y3 * Math.pow(temperature, 3);
      pressure += tempCorrection;
    }
  } else {
    // Linear strain gauge
    pressure = coefficients.A + coefficients.B * rawFrequency;
  }

  // Apply drift correction
  const daysSinceCalibration =
    (Date.now() - sensor.drift.lastCalibration.getTime()) / (1000 * 60 * 60 * 24);
  const driftCorrection = (sensor.drift.rate / 365) * daysSinceCalibration;
  pressure -= driftCorrection;

  return pressure;
}

// Convert pressure to depth
function pressureToDepth(pressure: number, latitude: number): number {
  // UNESCO formula for depth from pressure
  const g = 9.780318 * (1 + (5.2788e-3 + 2.36e-5 * latitude) *
            Math.pow(Math.sin(latitude * Math.PI / 180), 2));

  const c1 = 9.72659;
  const c2 = -2.2512e-5;
  const c3 = 2.279e-10;
  const c4 = -1.82e-15;

  const depth = c1 * pressure + c2 * Math.pow(pressure, 2) +
                c3 * Math.pow(pressure, 3) + c4 * Math.pow(pressure, 4);

  return depth;
}
```

#### Tsunami Detection Systems

Seafloor pressure sensors detect tsunamis as they pass overhead:

```typescript
interface TsunamiSensor {
  location: GeographicPosition;
  sensor: PressureSensor;

  // High-precision measurements
  samplingRate: number;           // Hz, typically 1-15 Hz

  // Detection algorithm
  detection: {
    baselinePressure: number;     // dbar, 15-minute average
    threshold: number;            // dbar, detection threshold (typically 0.01-0.05)
    anomalyDuration: number;      // seconds, sustained anomaly

    currentAnomaly: number;       // dbar, current deviation
    alertStatus: "normal" | "possible_tsunami" | "confirmed_tsunami";
    lastAlert?: Date;
  };

  // Tide removal
  tidalPrediction: {
    harmonics: TidalHarmonic[];
    residual: number;             // Observed - predicted
  };
}

interface TidalHarmonic {
  constituent: string;            // M2, S2, K1, O1, etc.
  amplitude: number;              // meters
  phase: number;                  // degrees
  frequency: number;              // cycles/hour
}

function detectTsunami(sensor: TsunamiSensor, currentPressure: number): boolean {
  // Remove tidal signal
  const predictedTide = calculateTidalPrediction(sensor.tidalPrediction.harmonics);
  const detrended = currentPressure - sensor.detection.baselinePressure - predictedTide;

  sensor.detection.currentAnomaly = detrended;

  // Check for sustained anomaly
  if (Math.abs(detrended) > sensor.detection.threshold) {
    sensor.detection.alertStatus = "possible_tsunami";
    return true;
  }

  return false;
}
```

### Salinity Sensors (Conductivity)

Salinity quantifies dissolved salts in seawater, primarily sodium chloride. It affects density, freezing point, and sound speed. Salinity is derived from electrical conductivity measurements.

#### Conductivity Cells

**Principle:** Seawater conducts electricity; conductivity increases with salinity and temperature

**Cell Types:**
- **Inductive (electrode-free):** Two toroidal transformers, seawater completes circuit, resistant to fouling
- **Electrode (ducted):** Precise geometry, electrodes measure conductance, higher accuracy but fouling-prone

```typescript
interface ConductivitySensor {
  type: "inductive" | "electrode";
  serialNumber: string;

  // Cell characteristics
  cellConstant: number;           // Geometric factor

  calibration: {
    coefficients: {
      g: number;                  // Cell coefficient
      h: number;                  // Temperature coefficient
      i: number;                  // Pressure coefficient
      j: number;                  // Quadratic pressure coefficient
    };
    date: Date;
  };

  range: {
    min: number;                  // S/m (Siemens/meter)
    max: number;
  };
  resolution: number;             // S/m
  accuracy: number;               // S/m
}

function calculateSalinity(
  conductivity: number,           // S/m
  temperature: number,            // °C
  pressure: number                // dbar
): number {
  // PSS-78 (Practical Salinity Scale 1978)
  // Salinity is dimensionless, approximately parts per thousand

  // Conductivity ratio: sample / standard seawater (S=35, T=15°C, P=0)
  const C_standard = 4.2914;      // S/m at S=35, T=15, P=0
  const R = conductivity / C_standard;

  // Temperature correction
  const rt = temperatureCorrection(temperature);
  const Rt = R / rt;

  // Pressure correction
  const rp = pressureCorrection(pressure, temperature);
  const Rp = Rt / rp;

  // Salinity calculation
  const salinity = salinitySeries(Rp, temperature);

  return salinity;
}

function temperatureCorrection(t: number): number {
  // rt = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4
  const c = [0.6766097, 2.00564e-2, 1.104259e-4, -6.9698e-7, 1.0031e-9];

  return c[0] + c[1]*t + c[2]*Math.pow(t,2) + c[3]*Math.pow(t,3) + c[4]*Math.pow(t,4);
}

function pressureCorrection(p: number, t: number): number {
  // Complex polynomial - simplified version
  const e1 = 2.070e-5;
  const e2 = -6.370e-10;
  const e3 = 3.989e-15;

  return 1 + (e1 + e2*p + e3*Math.pow(p,2)) * p;
}

function salinitySeries(Rp: number, t: number): number {
  // Polynomial coefficients from PSS-78
  const a = [0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081];
  const b = [0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144];

  const k = 0.0162;
  const dt = t - 15;

  let salinity = 0;
  for (let i = 0; i < a.length; i++) {
    salinity += a[i] * Math.pow(Rp, i/2);
  }

  // Temperature correction to salinity
  let tempCorrection = 0;
  for (let i = 0; i < b.length; i++) {
    tempCorrection += b[i] * Math.pow(Rp, i/2);
  }
  salinity += tempCorrection * dt / (1 + k*dt);

  return salinity;
}
```

#### CTD Profilers

The **Conductivity-Temperature-Depth** (CTD) profiler is the workhorse of physical oceanography:

```typescript
interface CTDProfile {
  cast: {
    castId: string;
    date: Date;
    location: GeographicPosition;
    ship: string;
    maxDepth: number;
    operator: string;
  };

  // Sensors
  sensors: {
    temperature: ThermistorSensor[];   // Primary + secondary
    conductivity: ConductivitySensor[]; // Primary + secondary
    pressure: PressureSensor;
  };

  // Measurements during descent
  measurements: {
    pressure: number;              // dbar
    depth: number;                 // meters
    temperature1: number;          // °C, primary
    temperature2: number;          // °C, secondary
    conductivity1: number;         // S/m, primary
    conductivity2: number;         // S/m, secondary
    salinity1: number;             // PSU
    salinity2: number;             // PSU
    soundVelocity: number;         // m/s
    density: number;               // kg/m³
    oxygen?: number;               // μmol/kg
    fluorescence?: number;         // mg/m³ chlorophyll
    turbidity?: number;            // NTU

    qualityFlags: Record<string, QualityFlag>;
  }[];

  // Derived products
  derived: {
    mixedLayerDepth: number;       // meters
    thermoclineDepth: number;      // meters
    maxBuoyancyFrequency: number;  // rad/s
    temperatureInversion: boolean;
  };
}

function processCTDData(profile: CTDProfile): CTDProfile {
  // Apply calibrations and corrections
  for (const meas of profile.measurements) {
    // Calculate derived salinity
    meas.salinity1 = calculateSalinity(
      meas.conductivity1,
      meas.temperature1,
      meas.pressure
    );

    // Calculate seawater density
    meas.density = calculateDensity(
      meas.salinity1,
      meas.temperature1,
      meas.pressure
    );

    // Calculate sound velocity
    meas.soundVelocity = calculateSoundVelocity(
      meas.salinity1,
      meas.temperature1,
      meas.pressure
    );
  }

  // Compute derived properties
  profile.derived.mixedLayerDepth = findMixedLayerDepth(profile.measurements);
  profile.derived.thermoclineDepth = findThermoclineDepth(profile.measurements);

  return profile;
}

function calculateDensity(S: number, T: number, P: number): number {
  // UNESCO equation of state (simplified)
  // Full implementation would use TEOS-10 standard

  const S0 = 35;
  const T0 = 0;
  const P0 = 0;

  // Density at atmospheric pressure
  const rho0 = 1028 + 0.785*S - 0.07*Math.pow(T,1) - 0.0008*Math.pow(T,2);

  // Pressure effect
  const K = 20000 + 2*P;  // Bulk modulus (simplified)
  const rho = rho0 / (1 - P/K);

  return rho;
}

function findMixedLayerDepth(measurements: any[]): number {
  // Find depth where density increases by 0.125 kg/m³ from surface
  const surfaceDensity = measurements[0].density;
  const threshold = 0.125;

  for (const meas of measurements) {
    if (meas.density - surfaceDensity > threshold) {
      return meas.depth;
    }
  }

  return measurements[measurements.length - 1].depth;
}
```

### Current Sensors

Ocean currents transport heat, salt, nutrients, larvae, and pollutants. Measuring currents is essential for understanding ocean circulation.

#### Acoustic Doppler Current Profilers (ADCP)

ADCPs measure current velocity by detecting the Doppler shift of sound scattered by particles in the water:

```typescript
interface ADCPSensor {
  frequency: number;               // kHz (38, 75, 150, 300, 600, 1200)
  beamAngle: number;               // degrees from vertical (typically 20-30°)
  numberOfBeams: 3 | 4 | 5;

  configuration: {
    binSize: number;               // meters, vertical resolution
    pingsPerEnsemble: number;
    blankingDistance: number;      // meters, near-field blind zone
    maximumRange: number;          // meters
  };

  // Velocity measurement
  velocityRange: number;           // m/s, ±velocity
  velocityResolution: number;      // m/s
  accuracy: number;                // m/s or % of velocity
}

interface ADCPMeasurement {
  timestamp: Date;
  location: GeographicPosition;

  // Velocity profiles
  bins: {
    depth: number;                 // meters
    velocity: {
      east: number;                // m/s
      north: number;               // m/s
      up: number;                  // m/s
      error: number;               // m/s, velocity error estimate
    };
    echo: number;                  // dB, backscatter intensity
    correlation: number;           // %, signal quality 0-100
    percentGood: number;           // %, data quality 0-100
  }[];

  // Instrument orientation
  heading: number;                 // degrees
  pitch: number;                   // degrees
  roll: number;                    // degrees
  temperature: number;             // °C

  qualityFlags: QualityFlag;
}

function processADCPVelocity(
  beamVelocities: number[],        // [beam1, beam2, beam3, beam4]
  beamAngle: number,
  heading: number,
  pitch: number,
  roll: number
): { east: number; north: number; up: number; error: number } {

  // Transform from beam coordinates to instrument coordinates
  const [b1, b2, b3, b4] = beamVelocities;
  const a = 1 / (2 * Math.sin(beamAngle * Math.PI / 180));
  const b = 1 / (4 * Math.cos(beamAngle * Math.PI / 180));

  const x = a * (b1 - b2);         // Along-instrument axis
  const y = a * (b4 - b3);         // Cross-instrument axis
  const z = b * (b1 + b2 + b3 + b4); // Vertical
  const err = b * (b1 + b2 - b3 - b4); // Error velocity

  // Apply heading, pitch, roll corrections (simplified)
  const h = heading * Math.PI / 180;
  const east = x * Math.cos(h) - y * Math.sin(h);
  const north = x * Math.sin(h) + y * Math.cos(h);
  const up = z;

  return { east, north, up, error: err };
}
```

### Integrated Multi-Sensor Systems

Modern platforms combine multiple physical sensors:

```typescript
interface OceanographicMooring {
  mooringId: string;
  location: GeographicPosition;
  deployment: {
    date: Date;
    waterDepth: number;            // meters
    operator: string;
  };

  // Sensor suite at multiple depths
  instruments: {
    depth: number;                 // meters

    // Physical sensors
    CTD?: {
      temperature: ThermistorSensor;
      conductivity: ConductivitySensor;
      pressure: PressureSensor;
    };

    currentMeter?: {
      type: "acoustic" | "electromagnetic" | "rotor";
      sensor: ADCPSensor | any;
    };

    // Optional additional sensors
    opticalBackscatter?: any;
    fluorometer?: any;
    oxygenSensor?: any;

    samplingInterval: number;      // seconds
    dataStorage: "internal" | "real-time" | "both";
  }[];

  // Surface telemetry
  telemetry: {
    system: "Iridium" | "Argos" | "cellular";
    transmissionSchedule: string;
    lastTransmission: Date;
    batteryVoltage: number;
  };
}
```

### Philosophy: 弘益人間 (Benefit All Humanity)

Physical ocean sensors embody 弘益人間 by providing fundamental knowledge that benefits all:

**Climate Understanding:** Temperature and salinity measurements document ocean heat uptake and circulation changes, informing climate projections for every nation

**Maritime Safety:** Accurate measurements of currents, temperature, and salinity support safe navigation and search-rescue operations worldwide

**Food Security:** Physical oceanography data helps predict fish habitat, supporting sustainable fisheries management

**Disaster Prevention:** Tsunami sensors and storm surge monitoring protect coastal communities globally

**Shared Knowledge:** Physical ocean data flows freely through international databases, enabling research and operational services for all countries

The ocean's temperature, salinity, and currents know no borders. Measuring them accurately and sharing that knowledge openly serves all humanity.

---

**Next Chapter:** We'll explore chemical sensors that measure pH, oxygen, nutrients, and the ocean's carbon system - critical for understanding ocean health and climate change.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
