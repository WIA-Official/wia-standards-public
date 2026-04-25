# Chapter 7: Sensor Networks and Data Integration

## From Individual Sensors to Integrated Ocean Observing Systems

Individual sensors provide point measurements, but understanding ocean processes requires integrated networks that span spatial and temporal scales. Modern ocean observing combines satellite remote sensing, autonomous platforms, moored arrays, ship surveys, and coastal stations into coherent systems. Data integration transforms raw measurements into actionable knowledge, enabling weather forecasting, climate monitoring, fisheries management, and maritime operations.

### Ocean Observing System Architecture

Comprehensive ocean observation requires coordinated multi-platform networks:

```typescript
interface OceanObservingSystem {
  systemName: string;
  region: "global" | "regional" | "coastal" | "local";
  operator: string;

  // Observing platforms
  platforms: {
    satellites: SatellitePlatform[];
    ships: ResearchVessel[];
    mooredBuoys: MooredBuoy[];
    driftingBuoys: DriftingBuoy[];
    profilers: ArgoFloat[];
    gliders: OceanGlider[];
    AUVs: AutonomousUnderwaterVehicle[];
    coastalStations: CoastalStation[];
    seafloorObservatories: SeafloorObservatory[];
  };

  // Essential Ocean Variables (EOVs)
  measuredVariables: {
    physical: string[];          // Temperature, salinity, currents, sea level, etc.
    biogeochemical: string[];    // Oxygen, nutrients, pH, pCO2, chlorophyll, etc.
    biological: string[];        // Phytoplankton, zooplankton, fish, marine mammals
    ecosystem: string[];         // Productivity, biodiversity, habitat
  };

  // Data flow
  dataManagement: {
    realtimeTransmission: boolean;
    dataLatency: number;         // hours, time to public availability
    qualityControl: "real-time" | "delayed-mode" | "both";
    dataRepository: string;      // URL
    dataFormat: string[];        // NetCDF-CF, CSV, etc.
  };

  // Integration
  modelingIntegration: {
    dataAssimilation: boolean;   // Feeds operational models
    models: string[];            // Ocean forecast models using this data
    reanalysis: boolean;         // Contributes to climate reanalysis
  };
}
```

### Global Ocean Observing System (GOOS)

GOOS coordinates international ocean observation through the **Framework for Ocean Observing (FOO)**:

#### Essential Ocean Variables (EOVs)

EOVs are key measurements required for ocean understanding:

```typescript
interface EssentialOceanVariable {
  name: string;
  category: "physical" | "biogeochemical" | "biological" | "ecosystem";

  // Measurement requirements
  requirements: {
    spatialResolution: number;   // km
    temporalResolution: number;  // days
    accuracy: number;
    coverage: number;            // % of ocean area
  };

  // Current observing capacity
  currentStatus: {
    coverage: number;            // % achieved
    platforms: string[];         // Platforms measuring this EOV
    dataAvailability: "good" | "fair" | "poor" | "very_poor";
    gaps: string[];              // Geographic or temporal gaps
  };

  // Applications
  applications: {
    weatherForecasting: boolean;
    climatemonitoring: boolean;
    marineEcosystems: boolean;
    marineResources: boolean;
    coastalHazards: boolean;
  };

  // Readiness
  readiness: {
    technologyReadiness: number; // 1-9 scale
    networkReadiness: number;    // Fraction of required observations
    impact: "high" | "medium" | "low"; // Scientific/societal impact
  };
}

const EOV_SeaSurfaceTemperature: EssentialOceanVariable = {
  name: "Sea Surface Temperature",
  category: "physical",
  requirements: {
    spatialResolution: 10,       // km
    temporalResolution: 1,       // daily
    accuracy: 0.1,               // °C
    coverage: 100                // % global ocean
  },
  currentStatus: {
    coverage: 100,               // Excellent satellite coverage
    platforms: ["satellites", "ships", "moorings", "drifters", "floats"],
    dataAvailability: "good",
    gaps: ["under sea ice", "high-latitude winter"]
  },
  applications: {
    weatherForecasting: true,
    climatemonitoring: true,
    marineEcosystems: true,
    marineResources: true,
    coastalHazards: true
  },
  readiness: {
    technologyReadiness: 9,      // Mature technology
    networkReadiness: 1.0,       // Full network
    impact: "high"
  }
};
```

### Multi-Platform Integration

Combining data from different platforms provides complementary information:

#### Satellite + In-Situ Integration

Satellites provide global coverage; in-situ sensors provide validation and subsurface data:

```typescript
interface SatelliteInSituIntegration {
  timeWindow: Date;
  location: GeographicCoordinate;

  // Satellite observations
  satellite: {
    sensor: "MODIS" | "VIIRS" | "Sentinel-3" | "Landsat";
    parameter: string;
    value: number;
    resolution: number;          // km
    uncertainty: number;
    cloudCover: number;          // %
  };

  // In-situ validation
  inSitu: {
    platform: "ship" | "mooring" | "float" | "glider";
    platformId: string;
    parameter: string;
    value: number;
    depth: number;
    uncertainty: number;
    matchupDistance: number;     // km from satellite pixel
    matchupTime: number;         // hours from satellite overpass
  };

  // Comparison
  validation: {
    bias: number;                // Satellite - in situ
    RMSE: number;                // Root mean square error
    correlation: number;         // R²
    withinUncertainty: boolean;
  };
}

function validateSatelliteSST(
  satelliteSST: number,
  inSituSST: number,
  matchupTime: number,           // hours
  matchupDistance: number        // km
): boolean {
  // Acceptable matchup criteria
  const MAX_TIME_DIFF = 3;       // hours
  const MAX_DISTANCE = 5;        // km
  const MAX_DIFFERENCE = 1.0;    // °C

  if (matchupTime > MAX_TIME_DIFF || matchupDistance > MAX_DISTANCE) {
    return false;
  }

  const difference = Math.abs(satelliteSST - inSituSST);
  return difference < MAX_DIFFERENCE;
}
```

#### Glider-Mooring Coordination

Gliders sample between moorings, creating a virtual array:

```typescript
interface GliderMooringNetwork {
  region: string;
  moorings: MooredBuoy[];
  gliders: OceanGlider[];

  // Coordinated sampling
  samplingStrategy: {
    gliderMissions: {
      gliderId: string;
      waypoints: GeographicCoordinate[];
      objectiveFunction: "fill_gaps" | "feature_tracking" | "survey";
      adaptiveSampling: boolean;
    }[];

    mooringIntegration: {
      calibrationCrossovers: boolean; // Glider passes near mooring for comparison
      interpolationNetwork: boolean;  // Gliders fill spatial gaps
      processStudies: boolean;        // Gliders investigate features detected by moorings
    };
  };

  // Data products
  products: {
    gridded4D: boolean;          // Temperature(x,y,z,t) on regular grid
    transportEstimates: boolean; // Volume/heat/salt transport
    eddyTracking: boolean;       // Mesoscale features
  };
}
```

### Data Quality Control

Integrated networks require consistent quality control:

#### Real-Time Quality Control

Automated tests flag suspect data immediately:

```typescript
interface RealtimeQualityControl {
  tests: {
    // Range test: value within expected bounds
    rangeTest: {
      parameter: string;
      value: number;
      minExpected: number;
      maxExpected: number;
      result: "pass" | "fail";
    };

    // Spike test: value differs excessively from neighbors
    spikeTest: {
      value: number;
      previousValue: number;
      nextValue: number;
      threshold: number;
      result: "pass" | "fail";
    };

    // Gradient test: spatial gradient within limits
    gradientTest: {
      value: number;
      neighborValue: number;
      distance: number;           // km
      maxGradient: number;        // units per km
      result: "pass" | "fail";
    };

    // Stuck value test: repeated identical values
    stuckValueTest: {
      consecutiveIdentical: number;
      threshold: number;          // Max allowed
      result: "pass" | "fail";
    };

    // Climatology test: value near climatological mean
    climatologyTest: {
      value: number;
      climatologyMean: number;
      climatologyStdDev: number;
      deviations: number;         // Number of std dev from mean
      threshold: number;          // Max allowed deviations
      result: "pass" | "fail";
    };
  };

  // Overall flag
  overallQuality: "good" | "probably_good" | "probably_bad" | "bad";

  // Actions
  actions: {
    dataPublished: boolean;
    alertGenerated: boolean;
    manualReviewRequired: boolean;
  };
}

function performRealtimeQC(
  measurement: SensorMeasurement,
  historicalData: SensorMeasurement[],
  climatology: { mean: number; stdDev: number }
): RealtimeQualityControl {
  const tests = {
    rangeTest: {
      parameter: measurement.parameter,
      value: measurement.value,
      minExpected: -2,
      maxExpected: 35,
      result: (measurement.value >= -2 && measurement.value <= 35) ? "pass" : "fail"
    },

    spikeTest: {
      value: measurement.value,
      previousValue: historicalData[historicalData.length - 1]?.value || 0,
      nextValue: 0,
      threshold: 2.0,
      result: "pass"  // Simplified
    },

    gradientTest: {
      value: measurement.value,
      neighborValue: 0,
      distance: 0,
      maxGradient: 0,
      result: "pass"
    },

    stuckValueTest: {
      consecutiveIdentical: 0,
      threshold: 5,
      result: "pass"
    },

    climatologyTest: {
      value: measurement.value,
      climatologyMean: climatology.mean,
      climatologyStdDev: climatology.stdDev,
      deviations: Math.abs(measurement.value - climatology.mean) / climatology.stdDev,
      threshold: 3,
      result: Math.abs(measurement.value - climatology.mean) / climatology.stdDev < 3 ?
              "pass" : "fail"
    }
  };

  const allPass = Object.values(tests).every(test => test.result === "pass");
  const anyFail = Object.values(tests).some(test => test.result === "fail");

  return {
    tests,
    overallQuality: allPass ? "good" : anyFail ? "probably_bad" : "probably_good",
    actions: {
      dataPublished: allPass,
      alertGenerated: anyFail,
      manualReviewRequired: anyFail
    }
  };
}
```

#### Delayed-Mode Quality Control

Expert scientists review data after collection:

```typescript
interface DelayedModeQualityControl {
  parameter: string;
  datasetId: string;

  // Expert review
  review: {
    scientist: string;
    reviewDate: Date;
    methodology: string;
  };

  // Adjustments
  adjustments: {
    sensorDriftCorrection: {
      applied: boolean;
      method: string;
      correction: number[];      // By measurement
    };

    calibrationUpdate: {
      applied: boolean;
      newCoefficients: Record<string, number>;
      retroactiveApplication: boolean;
    };

    outlierRemoval: {
      pointsRemoved: number;
      criteria: string;
    };
  };

  // Final quality flags
  finalFlags: {
    good: number;                // % of data
    probablyGood: number;
    probablyBad: number;
    bad: number;
    missing: number;
  };

  // Documentation
  documentation: {
    report: string;              // PDF or text file
    changeLog: string;
    DOI?: string;                // For published dataset
  };
}
```

### Data Fusion and Gridded Products

Raw sensor data is processed into gridded analysis products:

#### Objective Analysis

Interpolate irregular observations onto regular grids:

```typescript
interface ObjectiveAnalysis {
  region: {
    latitudeRange: [number, number];
    longitudeRange: [number, number];
    depthLevels: number[];       // meters
  };

  // Grid specification
  grid: {
    latitudeResolution: number;  // degrees
    longitudeResolution: number; // degrees
    timeStep: number;            // days
  };

  // Input observations
  observations: SensorMeasurement[];

  // Analysis method
  method: "optimal_interpolation" | "kriging" | "spline" | "variational";

  parameters: {
    correlationLength: number;   // km, spatial decorrelation scale
    correlationTime: number;     // days, temporal decorrelation scale
    observationError: number;    // Measurement uncertainty
    backgroundError: number;     // First-guess error
  };

  // Output
  griddedField: {
    latitude: number[];
    longitude: number[];
    depth: number[];
    time: Date[];
    values: number[][][][];      // [time][depth][lat][lon]
    uncertainty: number[][][][]; // Analysis uncertainty
  };
}

function optimalInterpolation(
  observations: { location: [number, number]; value: number }[],
  gridPoints: [number, number][],
  correlationLength: number
): number[] {
  // Simplified optimal interpolation
  // Full version uses covariance matrices

  const gridValues: number[] = [];

  for (const gridPoint of gridPoints) {
    let weightedSum = 0;
    let weightSum = 0;

    for (const obs of observations) {
      const distance = calculateDistance(gridPoint, obs.location);

      // Gaussian correlation function
      const weight = Math.exp(-Math.pow(distance / correlationLength, 2));

      weightedSum += weight * obs.value;
      weightSum += weight;
    }

    const gridValue = weightSum > 0 ? weightedSum / weightSum : NaN;
    gridValues.push(gridValue);
  }

  return gridValues;
}

function calculateDistance(
  point1: [number, number],
  point2: [number, number]
): number {
  // Haversine formula for distance between lat/lon points
  const [lat1, lon1] = point1;
  const [lat2, lon2] = point2;

  const R = 6371;  // Earth radius in km
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;

  const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
           Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
           Math.sin(dLon/2) * Math.sin(dLon/2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const distance = R * c;

  return distance;
}
```

#### Data Assimilation into Models

Observations constrain numerical ocean models:

```typescript
interface DataAssimilationSystem {
  model: {
    name: string;
    type: "OGCM" | "ROMS" | "NEMO" | "MITgcm" | "HYCOM";
    domain: string;
    resolution: { horizontal: number; vertical: number; temporal: number };
  };

  // Assimilation method
  method: "3DVAR" | "4DVAR" | "EnKF" | "EnOI";

  // Observations assimilated
  observations: {
    type: string;
    platforms: string[];
    count: number;               // Number of observations
    impact: number;              // Reduction in forecast error, %
  }[];

  // Forecast products
  forecasts: {
    parameter: string;
    leadTime: number;            // hours
    skillScore: number;          // 0-1, forecast accuracy
    updateFrequency: number;     // hours
  }[];

  // Output
  outputs: {
    analysisField: boolean;      // Best estimate of current state
    forecastField: boolean;      // Prediction of future state
    uncertainty: boolean;        // Ensemble spread
    dataRepository: string;
  };
}
```

### Regional Ocean Observing Systems

Regional systems provide enhanced coverage for specific areas:

```typescript
interface RegionalObservingSystem extends OceanObservingSystem {
  region: string;                // e.g., "California Current", "Mediterranean"

  // Regional priorities
  priorities: {
    marineEcosystems: boolean;
    fisheries: boolean;
    coastalHazards: boolean;
    waterQuality: boolean;
    maritimeSafety: boolean;
    climateMonitoring: boolean;
  };

  // Stakeholders
  stakeholders: {
    name: string;
    type: "research" | "operational" | "commercial" | "regulatory" | "public";
    dataUse: string;
  }[];

  // Enhanced measurements
  enhancedVariables: string[];   // Beyond global EOVs

  // Regional products
  products: {
    name: string;
    description: string;
    updateFrequency: string;
    users: string[];
    format: string;
  }[];
}
```

### Sensor Web and Internet of Ocean Things

Modern networks enable sensor interoperability and discovery:

```typescript
interface SensorWebNode {
  sensorId: string;
  sensorType: string;
  location: GeographicPosition;

  // Capabilities
  capabilities: {
    parameters: string[];
    samplingRate: number;
    dataFormats: string[];
    communicationProtocols: string[];
  };

  // Services
  services: {
    sensorObservationService: {
      endpoint: string;          // OGC SOS URL
      operations: string[];      // GetObservation, GetCapabilities, etc.
    };

    sensorPlanningService?: {
      endpoint: string;
      tasking: boolean;          // Can sensor be tasked remotely
    };

    alertService?: {
      endpoint: string;
      triggers: {
        parameter: string;
        threshold: number;
        action: string;
      }[];
    };
  };

  // Metadata
  metadata: {
    SensorML: string;            // OGC SensorML document URL
    ISO19115: string;            // ISO metadata
    schema_org: any;             // Schema.org structured data
  };

  // Discovery
  registration: {
    catalogues: string[];        // Where sensor is registered
    keywords: string[];
    license: string;
  };
}
```

### Philosophy: 弘益人間 (Benefit All Humanity)

Integrated ocean observing networks embody 弘익人間 by serving all people:

**Global Cooperation:** International data sharing enables all nations to benefit from ocean observations, regardless of observing capacity

**Disaster Prevention:** Integrated systems provide tsunami warnings, hurricane forecasts, and harmful algal bloom alerts protecting coastal communities worldwide

**Climate Understanding:** Global networks document ocean changes, informing climate policy for all nations

**Sustainable Resources:** Observing systems support fisheries management and marine spatial planning, ensuring ocean resources for future generations

**Open Data:** Most ocean observing data is freely available, democratizing ocean knowledge

The ocean connects all humanity. Observing it through integrated global networks and sharing that knowledge freely serves the common good.

---

**Next Chapter:** We'll explore the future of marine sensing, examining emerging technologies and envisioning the next generation of ocean observation.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
