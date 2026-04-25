# Chapter 6: Acoustic Sensors (Hydrophones, ADCP)

## Sound as the Ocean's Primary Sense

Acoustic sensors exploit sound propagation in seawater to measure currents, detect marine life, map the seafloor, and monitor underwater noise. Sound travels **4.5 times faster and much farther in water than in air**, making acoustics the dominant remote sensing method for ocean interiors. From low-frequency whale calls traveling thousands of kilometers to high-frequency sonar imaging centimeter-scale features, acoustic sensors span frequencies from 10 Hz to several MHz.

### Sound Propagation in Seawater

Understanding acoustic propagation is fundamental to interpreting acoustic sensor data.

#### Sound Speed in Seawater

Sound speed depends on temperature, salinity, and pressure (depth):

**Typical values:** 1450-1550 m/s
**Increase with:** Higher temperature, higher salinity, greater depth
**Gradient:** Creates sound channels and shadow zones

```typescript
interface SoundSpeedProfile {
  depth: number[];               // meters
  soundSpeed: number[];          // m/s
  temperature: number[];         // °C
  salinity: number[];            // PSU

  // Derived features
  soundChannel: {
    exists: boolean;
    depth: number;               // meters, SOFAR channel axis
    soundSpeedMin: number;       // m/s
  };

  criticalDepth?: number;        // meters, where sound speed = surface value
}

function calculateSoundSpeed(
  temperature: number,           // °C
  salinity: number,              // PSU
  depth: number                  // meters
): number {
  // Mackenzie (1981) equation

  const T = temperature;
  const S = salinity;
  const D = depth;

  const c = 1448.96 +
           4.591 * T -
           5.304e-2 * Math.pow(T, 2) +
           2.374e-4 * Math.pow(T, 3) +
           1.340 * (S - 35) +
           1.630e-2 * D +
           1.675e-7 * Math.pow(D, 2) -
           1.025e-2 * T * (S - 35) -
           7.139e-13 * T * Math.pow(D, 3);

  return c;  // m/s
}

function findSOFARChannel(profile: SoundSpeedProfile): {
  depth: number;
  soundSpeed: number;
} {
  // SOFAR (Sound Fixing And Ranging) channel: sound speed minimum
  // Typically at 1000m depth, traps sound in channel

  let minSpeed = Infinity;
  let minDepth = 0;

  for (let i = 0; i < profile.depth.length; i++) {
    if (profile.soundSpeed[i] < minSpeed) {
      minSpeed = profile.soundSpeed[i];
      minDepth = profile.depth[i];
    }
  }

  return { depth: minDepth, soundSpeed: minSpeed };
}
```

#### Acoustic Absorption

Sound energy is absorbed by water and converted to heat, limiting range:

**Frequency dependence:** Higher frequencies absorbed more strongly
**Long range:** Low-frequency sounds travel thousands of km
**Short range:** High-frequency sounds limited to meters/kilometers

```typescript
interface AcousticAbsorption {
  frequency: number;             // Hz
  temperature: number;           // °C
  salinity: number;              // PSU
  depth: number;                 // meters
  pH: number;

  absorptionCoefficient: number; // dB/km

  // Absorption mechanisms
  mechanisms: {
    viscous: number;             // dB/km, molecular viscosity
    boricAcid: number;           // dB/km, relaxation below 1 kHz
    magnesiumSulfate: number;    // dB/km, relaxation above 1 kHz
  };
}

function calculateAbsorption(
  frequency: number,             // kHz
  temperature: number,           // °C
  salinity: number,              // PSU
  depth: number,                 // meters
  pH: number
): number {
  // Ainslie & McColm (1998) equation

  const f = frequency;
  const T = temperature;
  const S = salinity;
  const D = depth;

  // Boric acid contribution (dominant at low freq)
  const f1 = 0.78 * Math.sqrt(S / 35) * Math.exp(T / 26);
  const A1 = 0.106 * (f1 * Math.pow(f, 2)) / (Math.pow(f1, 2) + Math.pow(f, 2));

  // Magnesium sulfate contribution (dominant at mid freq)
  const f2 = 42 * Math.exp(T / 17);
  const A2 = 0.52 * (1 + T / 43) * (S / 35) * (f2 * Math.pow(f, 2)) /
             (Math.pow(f2, 2) + Math.pow(f, 2));

  // Pure water contribution (dominant at high freq)
  const A3 = 0.00049 * Math.pow(f, 2) * Math.exp(-(T / 27 + D / 17));

  const alpha = A1 + A2 + A3;  // dB/km

  return alpha;
}

function calculateTransmissionLoss(
  range: number,                 // km
  frequency: number,             // kHz
  absorption: number             // dB/km
): number {
  // Transmission loss = spreading loss + absorption loss

  // Spherical spreading: 20*log10(range) for 3D propagation
  const spreadingLoss = 20 * Math.log10(range * 1000);  // Convert km to m

  // Absorption loss
  const absorptionLoss = absorption * range;

  const totalLoss = spreadingLoss + absorptionLoss;

  return totalLoss;  // dB
}
```

### Hydrophones and Passive Acoustics

**Hydrophones** detect underwater sound without transmitting, used for marine mammal monitoring, seismic detection, and noise measurement.

#### Hydrophone Sensors

**Principle:** Convert pressure variations (sound waves) into electrical signals

**Types:**
- **Piezoelectric:** Ceramic or crystal generates voltage from pressure
- **Fiber optic:** Light interferometry measures pressure
- **MEMS:** Microelectromechanical systems, miniaturized

```typescript
interface Hydrophone {
  type: "piezoelectric" | "fiber_optic" | "MEMS";
  serialNumber: string;

  // Acoustic characteristics
  sensitivity: number;           // dB re 1V/μPa at 1m
  frequencyResponse: {
    lowCutoff: number;           // Hz, -3dB point
    highCutoff: number;          // Hz, -3dB point
    flatResponse: [number, number]; // Hz, ±1dB range
  };

  // Directivity
  directivity: {
    pattern: "omnidirectional" | "cardioid" | "dipole";
    beamWidth?: number;          // degrees, -3dB beam width
  };

  // Self-noise
  equivalentNoise: number;       // dB re 1μPa, at 1 kHz

  // Dynamic range
  dynamicRange: number;          // dB
  maximumSPL: number;            // dB re 1μPa, maximum sound pressure level

  // Calibration
  calibration: {
    date: Date;
    method: "pistonphone" | "comparison" | "reciprocity";
    sensitivity: number;         // dB re 1V/μPa
    referenceFrequency: number;  // Hz
  };
}

interface AcousticRecording {
  timestamp: Date;
  location: GeographicPosition;
  depth: number;

  // Audio data
  audio: {
    sampleRate: number;          // Hz, e.g., 96000
    bitDepth: number;            // bits, e.g., 24
    duration: number;            // seconds
    channels: number;            // 1 (mono) or multiple
    format: "WAV" | "FLAC" | "compressed";
  };

  // Spectral analysis
  spectrum: {
    frequencies: number[];       // Hz
    powerSpectralDensity: number[]; // dB re 1μPa²/Hz
    soundPressureLevel: number;  // dB re 1μPa, broadband
  };

  // Environmental
  windSpeed?: number;            // m/s, affects ambient noise
  shipping?: "low" | "moderate" | "high"; // Shipping noise level

  qualityFlag: QualityFlag;
}

function calculateSoundPressureLevel(
  voltage: number,               // V, hydrophone output
  sensitivity: number            // dB re 1V/μPa
): number {
  // Convert voltage to sound pressure level

  const voltage_dB = 20 * Math.log10(voltage);
  const SPL = voltage_dB - sensitivity;

  return SPL;  // dB re 1μPa
}
```

#### Marine Mammal Detection

Passive acoustic monitoring detects whales, dolphins, and other marine mammals:

**Applications:**
- Presence/absence surveys
- Population density estimation
- Migration tracking
- Ship strike prevention
- Protected area monitoring

```typescript
interface MarineMammalDetection {
  species: string;
  timestamp: Date;
  location: GeographicPosition;

  // Call characteristics
  call: {
    type: "click" | "whistle" | "moan" | "pulse" | "song";
    frequency: {
      min: number;               // Hz
      max: number;               // Hz
      peak: number;              // Hz, dominant frequency
    };
    duration: number;            // seconds
    repetitionRate?: number;     // Hz, for clicks
    soundLevel: number;          // dB re 1μPa
  };

  // Detection confidence
  detection: {
    method: "template_matching" | "neural_network" | "spectrogram";
    confidence: number;          // 0-1
    signalToNoise: number;       // dB
    manualVerification: boolean;
  };

  // Species identification
  identification: {
    species: string;
    confidence: "certain" | "probable" | "possible";
    alternativeSpecies?: string[];
  };

  // Localization
  localization?: {
    method: "array" | "triangulation" | "time_difference";
    bearingEstimate: number;     // degrees
    bearingUncertainty: number;  // degrees
    rangeEstimate?: number;      // km
  };

  qualityFlag: QualityFlag;
}

function detectMarineMammalCalls(
  audio: number[],               // Audio samples
  sampleRate: number,            // Hz
  templates: CallTemplate[]
): MarineMammalDetection[] {
  const detections: MarineMammalDetection[] = [];

  // Perform spectrogram analysis
  const spectrogram = computeSpectrogram(audio, sampleRate);

  // Template matching or ML detection
  for (const template of templates) {
    const matches = templateMatch(spectrogram, template);

    for (const match of matches) {
      if (match.score > 0.7) {  // Confidence threshold
        detections.push({
          species: template.species,
          timestamp: new Date(),
          location: { latitude: 0, longitude: 0, datum: "WGS84" },
          call: {
            type: template.callType,
            frequency: {
              min: match.freqMin,
              max: match.freqMax,
              peak: match.peakFreq
            },
            duration: match.duration,
            soundLevel: match.SPL
          },
          detection: {
            method: "template_matching",
            confidence: match.score,
            signalToNoise: match.SNR,
            manualVerification: false
          },
          identification: {
            species: template.species,
            confidence: match.score > 0.9 ? "certain" :
                      match.score > 0.8 ? "probable" : "possible"
          },
          qualityFlag: { status: "good", automated: true, tests: [] }
        });
      }
    }
  }

  return detections;
}

interface CallTemplate {
  species: string;
  callType: "click" | "whistle" | "moan" | "pulse" | "song";
  spectralSignature: number[][];
  freqRange: [number, number];
  durationRange: [number, number];
}

// Placeholder functions
function computeSpectrogram(audio: number[], sampleRate: number): number[][] {
  return [[]];
}

function templateMatch(spectrogram: number[][], template: CallTemplate): any[] {
  return [];
}
```

#### Underwater Soundscape Monitoring

Acoustic recordings reveal overall underwater soundscape:

**Natural sounds:**
- Wind and waves
- Rain
- Marine mammals
- Snapping shrimp
- Fish vocalizations

**Anthropogenic sounds:**
- Shipping
- Sonar
- Pile driving
- Seismic airguns
- Offshore construction

```typescript
interface SoundscapeAnalysis {
  location: GeographicPosition;
  timeperiod: {
    start: Date;
    end: Date;
    duration: number;            // hours
  };

  // Frequency bands
  spectralLevels: {
    band: string;                // "10-100 Hz", "100-1000 Hz", etc.
    frequencyRange: [number, number];
    meanSPL: number;             // dB re 1μPa
    percentiles: {
      p5: number;
      p50: number;               // Median
      p95: number;
    };
  }[];

  // Sound sources
  sources: {
    source: "shipping" | "marine_mammals" | "wind" | "rain" | "biological";
    contribution: number;        // % of total acoustic energy
    prevalence: number;          // % of time present
  }[];

  // Noise metrics
  metrics: {
    ambientNoise: number;        // dB re 1μPa, 1/3 octave band
    soundExposureLevel: number;  // dB re 1μPa²·s
    peakLevel: number;           // dB re 1μPa, maximum instantaneous
  };

  // Anthropogenic impact assessment
  noiseImpact: {
    level: "low" | "moderate" | "high" | "severe";
    maskingPotential: "low" | "moderate" | "high"; // Masks animal communication
    complianceStatus: "compliant" | "exceedance";
    regulatoryLimit?: number;    // dB re 1μPa
  };
}
```

### Acoustic Doppler Current Profilers (ADCP)

**ADCPs** measure water velocity by transmitting sound pulses and analyzing Doppler-shifted echoes from suspended particles.

#### ADCP Principles

**Doppler Effect:** Moving particles shift sound frequency proportional to velocity

**Multi-beam geometry:** 3-4 beams at angles resolve 3D velocity vector

**Profiling:** Divide water column into depth bins, measure velocity in each

```typescript
interface ADCPSystem {
  frequency: number;             // kHz (38, 75, 150, 300, 600, 1200)
  manufacturer: string;
  model: string;

  // Beam configuration
  beams: {
    numberOfBeams: 3 | 4 | 5;
    beamAngle: number;           // degrees from vertical, typically 20-30°
    orientation: "up" | "down" | "horizontal";
  };

  // Measurement configuration
  configuration: {
    binSize: number;             // meters, vertical resolution
    numberOfBins: number;
    blankingDistance: number;    // meters, near-field exclusion
    pingInterval: number;        // seconds
    pingsPerEnsemble: number;
  };

  // Performance specifications
  performance: {
    velocityRange: number;       // m/s, ±max velocity
    velocityResolution: number;  // m/s
    velocityAccuracy: number;    // m/s or % of reading
    maximumRange: number;        // meters
    profileRate: number;         // profiles per hour
  };

  // Sensors
  auxiliarySensors: {
    compass: boolean;
    tilt: boolean;
    temperature: boolean;
    pressure: boolean;
  };
}

interface ADCPVelocityProfile {
  timestamp: Date;
  location: GeographicPosition;

  // Velocity data by depth bin
  bins: {
    depth: number;               // meters (center of bin)
    velocity: {
      east: number;              // m/s
      north: number;             // m/s
      up: number;                // m/s
      error: number;             // m/s, error velocity
    };
    echoIntensity: number;       // dB, backscatter strength
    correlation: number;         // %, 0-100
    percentGood: number;         // %, valid data
  }[];

  // Platform motion (for moving ADCPs)
  platformVelocity?: {
    east: number;
    north: number;
    up: number;
  };

  // Quality metrics
  quality: {
    heading: number;             // degrees
    pitch: number;               // degrees
    roll: number;                // degrees
    temperature: number;         // °C
    bottomTrack: boolean;        // Bottom tracking enabled
    ensembleQuality: "good" | "questionable" | "bad";
  };

  qualityFlag: QualityFlag;
}

function calculateVelocityFromDoppler(
  dopplerShift: number,          // Hz
  frequency: number,             // Hz, transmitted frequency
  soundSpeed: number,            // m/s
  beamAngle: number              // degrees
): number {
  // Doppler equation: Δf = 2 * v * cos(θ) * f0 / c
  // Solve for v: v = Δf * c / (2 * f0 * cos(θ))

  const theta_rad = beamAngle * Math.PI / 180;
  const velocity = (dopplerShift * soundSpeed) /
                  (2 * frequency * Math.cos(theta_rad));

  return velocity;  // m/s
}

function transformBeamToEarth(
  beamVelocities: number[],      // [beam1, beam2, beam3, beam4]
  beamAngle: number,             // degrees
  heading: number,               // degrees
  pitch: number,                 // degrees
  roll: number                   // degrees
): { east: number; north: number; up: number; error: number } {
  // Transform from beam coordinates to Earth coordinates
  // Accounts for instrument orientation

  const [b1, b2, b3, b4] = beamVelocities;
  const a = 1 / (2 * Math.sin(beamAngle * Math.PI / 180));
  const b = 1 / (4 * Math.cos(beamAngle * Math.PI / 180));

  // Instrument coordinates
  const x = a * (b1 - b2);
  const y = a * (b4 - b3);
  const z = b * (b1 + b2 + b3 + b4);
  const err = b * (b1 + b2 - b3 - b4);

  // Rotation to Earth coordinates (simplified - full version uses rotation matrices)
  const h = heading * Math.PI / 180;

  const east = x * Math.cos(h) - y * Math.sin(h);
  const north = x * Math.sin(h) + y * Math.cos(h);
  const up = z;

  return { east, north, up, error: err };
}
```

#### Ocean Current Applications

ADCPs measure currents from surface waves to deep ocean circulation:

**Moored ADCPs:** Long-term time series at fixed locations
**Ship-mounted ADCPs:** Spatial surveys along transects
**ADCP on gliders:** Autonomous profiling
**Bottom-mounted ADCPs:** Continuous monitoring in shallower water

```typescript
interface CurrentTimeSeries {
  mooringId: string;
  location: GeographicPosition;
  deployment: {
    start: Date;
    end: Date;
    waterDepth: number;
    instrumentDepth: number;
  };

  // Processed current data
  timeSeries: {
    timestamp: Date;
    depth: number[];
    eastVelocity: number[];      // m/s
    northVelocity: number[];     // m/s
    speed: number[];             // m/s, current speed
    direction: number[];         // degrees, current direction (toward)
  }[];

  // Tidal analysis
  tidalAnalysis?: {
    constituents: {
      name: string;              // M2, S2, K1, O1, etc.
      amplitude: number;         // m/s
      phase: number;             // degrees
      frequency: number;         // cycles/day
    }[];
    tidalEllipse: {
      majorAxis: number;         // m/s
      minorAxis: number;         // m/s
      orientation: number;       // degrees
    }[];
  };

  // Statistics
  statistics: {
    meanFlow: { east: number; north: number };
    eddyKineticEnergy: number;  // m²/s²
    maximumSpeed: number;        // m/s
    percentData: number;         // % valid data
  };
}
```

### Multibeam Echo Sounders

**Multibeam sonars** map the seafloor by transmitting a fan of acoustic beams:

```typescript
interface MultibeamEchoSounder {
  frequency: number;             // kHz (30, 100, 200, 400)
  beams: number;                 // Number of beams, e.g., 256, 512

  swathCoverage: number;         // degrees, e.g., 120-150°
  depthRange: {
    min: number;                 // meters
    max: number;                 // meters
  };

  resolution: {
    acrossTrack: number;         // meters
    alongTrack: number;          // meters
    vertical: number;            // meters
  };

  // Applications
  applications: "bathymetry" | "backscatter" | "water_column";
}
```

### Philosophy: 弘益人間 (Benefit All Humanity)

Acoustic sensors serve humanity by listening to and imaging the ocean:

**Marine Life Protection:** Hydrophones detect whales and dolphins, preventing ship strikes and protecting critical habitats

**Ocean Circulation Understanding:** ADCPs measure currents that drive climate, fisheries, and navigation

**Seafloor Mapping:** Acoustic sensors reveal underwater terrain, supporting safe navigation, resource management, and scientific discovery

**Noise Impact Assessment:** Soundscape monitoring protects marine life from harmful noise pollution

**Shared Acoustic Data:** Acoustic observations flow into international databases, enabling global ocean understanding

Sound connects all parts of the ocean. By measuring it carefully and sharing that knowledge, we benefit all humanity and protect marine life.

---

**Next Chapter:** We'll explore sensor networks and data integration, examining how individual sensors combine into observing systems that provide comprehensive ocean understanding.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
