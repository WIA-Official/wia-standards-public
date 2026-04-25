# Chapter 3: Environmental Monitoring Data

## Measuring the Ocean's Physical and Chemical Parameters

Marine organisms live in a complex environment shaped by temperature, salinity, currents, nutrients, and countless other factors. Understanding species distributions, ecosystem health, and climate change impacts requires comprehensive environmental monitoring data collected from sensors deployed across the global ocean.

### The Ocean Observing System

The **Global Ocean Observing System (GOOS)** coordinates international efforts to monitor ocean conditions through a distributed network of platforms:

**Argo Float Array:**
- **3,800+ autonomous floats** drifting with ocean currents
- Dive to **2,000 meters** depth every 10 days
- Measure temperature, salinity, pressure
- **Over 2 million profiles** collected since 2000
- Real-time data transmission via satellite

**Moored Buoys:**
- **1,200+ fixed-location** weather and ocean buoys
- Continuous measurements of surface conditions
- Time-series data at specific locations
- Critical for climate monitoring and model validation

**Research Vessels:**
- **100+ active** oceanographic research ships
- CTD casts (Conductivity-Temperature-Depth)
- Water sampling for chemical and biological analysis
- Multi-beam sonar bathymetry

**Satellites:**
- Sea surface temperature (SST)
- Ocean color (chlorophyll, phytoplankton)
- Sea surface height (currents, eddies)
- Sea ice extent and thickness
- Wave height and surface winds

**Autonomous Underwater Vehicles (AUVs):**
- High-resolution surveys of specific regions
- Under-ice measurements
- Deep-sea exploration
- Adaptive sampling of dynamic features

### Core Environmental Parameters

Marine biologists need comprehensive environmental context for species observations:

```typescript
interface OceanEnvironmentalData {
  metadata: {
    platformType: "argo_float" | "moored_buoy" | "ship_ctd" | "glider" | "satellite";
    platformID: string;
    cruiseID?: string;            // For ship-based data
    program: string;              // e.g., "GOOS", "CalCOFI"
    dataProvider: string;
  };

  spatiotemporal: {
    timestamp: Date;              // UTC
    latitude: number;             // Decimal degrees
    longitude: number;
    depth: number;                // Meters (positive down)
    bottomDepth?: number;         // Total water depth
  };

  physical: {
    temperature: {
      value: number;              // Celsius
      unit: "degC";
      precision: number;
      sensor: string;
      calibrationDate: Date;
      qualityFlag: QualityFlag;
    };

    salinity: {
      value: number;              // PSU (Practical Salinity Units)
      unit: "PSU";
      precision: number;
      calculationMethod: "measured" | "calculated_from_conductivity";
      qualityFlag: QualityFlag;
    };

    pressure: {
      value: number;              // Decibars
      unit: "dbar";
      precision: number;
      qualityFlag: QualityFlag;
    };

    density: {
      value: number;              // kg/m³
      unit: "kg/m3";
      type: "sigma_t" | "sigma_theta" | "potential_density";
      qualityFlag: QualityFlag;
    };

    current: {
      velocity: number;           // m/s
      direction: number;          // Degrees true (0-360)
      uComponent: number;         // East-west (m/s)
      vComponent: number;         // North-south (m/s)
      method: "ADCP" | "surface_drifter" | "model";
      qualityFlag: QualityFlag;
    };
  };

  chemical: {
    dissolvedOxygen: {
      value: number;              // μmol/kg or mg/L
      unit: "umol/kg" | "mg/L";
      saturation: number;         // Percentage
      method: "Winkler" | "optical_sensor";
      qualityFlag: QualityFlag;
    };

    pH: {
      value: number;              // pH units
      scale: "total" | "seawater" | "free";
      temperature: number;        // Temperature at which pH was measured
      qualityFlag: QualityFlag;
    };

    nutrients: {
      nitrate: Measurement;       // μmol/L
      nitrite: Measurement;
      phosphate: Measurement;
      silicate: Measurement;
      ammonia?: Measurement;
      method: "colorimetric" | "chemiluminescence";
    };

    carbonSystem: {
      DIC: Measurement;           // Dissolved Inorganic Carbon (μmol/kg)
      alkalinity: Measurement;    // Total alkalinity (μmol/kg)
      pCO2: Measurement;          // Partial pressure CO2 (μatm)
      method: string;
    };

    chlorophyll: {
      value: number;              // mg/m³
      method: "fluorometry" | "HPLC" | "satellite";
      qualityFlag: QualityFlag;
    };
  };

  optical: {
    turbidity?: {
      value: number;              // NTU or FTU
      unit: "NTU" | "FTU";
      qualityFlag: QualityFlag;
    };

    PAR?: {                       // Photosynthetically Active Radiation
      value: number;              // μmol photons/m²/s
      depth: number;
      qualityFlag: QualityFlag;
    };

    secchiDepth?: number;         // Meters (water clarity)
  };

  biologicalSamples?: {
    planktonNet?: {
              meshSize: number;           // Micrometers
      volumeFiltered: number;     // Cubic meters
      abundance: number;          // Organisms per cubic meter
    };

    waterSample?: {
      volume: number;             // Liters
      preservation: string;       // "frozen", "formalin", "alcohol"
      analyses: string[];         // "eDNA", "phytoplankton", "nutrients"
    };
  };
}

type QualityFlag =
  | 0  // No quality control
  | 1  // Good data
  | 2  // Probably good data
  | 3  // Potentially correctable bad data
  | 4  // Bad data
  | 5  // Value changed
  | 6  // Not used
  | 7  // Nominal value
  | 8  // Interpolated value
  | 9  // Missing value;
```

### CTD Profiling: The Gold Standard

**CTD (Conductivity-Temperature-Depth)** instruments are the workhorse of oceanography, providing vertical profiles of ocean properties:

```typescript
interface CTDCast {
  castID: string;
  ship: string;
  cruise: string;
  station: string;
  castDate: Date;

  location: {
    latitude: number;
    longitude: number;
    bottomDepth: number;          // Meters
  };

  instrument: {
    manufacturer: "Sea-Bird" | "RBR" | "other";
    model: string;
    serialNumber: string;
    lastCalibration: Date;
    sensors: {
      temperature: SensorSpec;
      conductivity: SensorSpec;
      pressure: SensorSpec;
      oxygen?: SensorSpec;
      fluorometer?: SensorSpec;
      transmissometer?: SensorSpec;
    };
  };

  profile: {
    maxDepth: number;             // Meters
    downcastRate: number;         // m/s
    upcastRate: number;
    sampleInterval: number;       // Seconds
    binSize: number;              // Meter intervals for averaged data
  };

  data: CTDDataPoint[];           // Array of measurements at each depth

  processing: {
    software: string;             // "SBE Data Processing", "TEOS-10"
    version: string;
    filters: string[];            // Applied filters (e.g., "7-point median")
    derivedVariables: string[];   // Calculated parameters
  };

  qualityControl: {
    performedBy: string;
    date: Date;
    flags: Record<string, QualityFlag>;
    notes: string;
  };
}

interface CTDDataPoint {
  depth: number;                  // Meters
  pressure: number;               // Decibars
  temperature: number;            // Celsius (ITS-90)
  conductivity: number;           // S/m
  salinity: number;               // PSU
  density: number;                // kg/m³ (sigma-theta)
  soundVelocity: number;          // m/s
  oxygen?: number;                // μmol/kg
  fluorescence?: number;          // mg/m³ (chlorophyll)
  transmittance?: number;         // Percentage
  qualityFlags: Record<string, QualityFlag>;
}

// Real-world example: CalCOFI cruise data
const ctdCast: CTDCast = {
  castID: "CalCOFI_2401_093.70_046.9_001",
  ship: "R/V Robert Gordon Sproul",
  cruise: "2401",
  station: "093.70 046.9",
  castDate: new Date("2024-01-15T14:30:00Z"),

  location: {
    latitude: 33.467,
    longitude: -120.033,
    bottomDepth: 1850
  },

  instrument: {
    manufacturer: "Sea-Bird",
    model: "SBE 911plus",
    serialNumber: "09P52876",
    lastCalibration: new Date("2023-12-01"),
    sensors: {
      temperature: {
        model: "SBE 3plus",
        accuracy: 0.001,
        resolution: 0.0001,
        units: "degC"
      },
      conductivity: {
        model: "SBE 4C",
        accuracy: 0.0003,
        resolution: 0.00004,
        units: "S/m"
      },
      pressure: {
        model: "Digiquartz",
        accuracy: 0.015,
        resolution: 0.001,
        units: "dbar"
      },
      oxygen: {
        model: "SBE 43",
        accuracy: 2,
        resolution: 0.1,
        units: "umol/kg"
      },
      fluorometer: {
        model: "Seapoint",
        accuracy: 0.01,
        resolution: 0.001,
        units: "mg/m3"
      }
    }
  },

  profile: {
    maxDepth: 515,
    downcastRate: 1.0,
    upcastRate: 0.8,
    sampleInterval: 0.04,          // 24 Hz
    binSize: 1                      // 1-meter bins
  },

  data: [
    // Depth 0m (surface)
    {
      depth: 0,
      pressure: 0,
      temperature: 15.234,
      conductivity: 4.1234,
      salinity: 33.456,
      density: 1024.5,
      soundVelocity: 1505.2,
      oxygen: 245.6,
      fluorescence: 1.23,
      transmittance: 89.5,
      qualityFlags: { temperature: 1, salinity: 1, oxygen: 1 }
    },
    // ... more data points every 1 meter to 515m
  ],

  processing: {
    software: "SBE Data Processing",
    version: "7.26.7",
    filters: ["7-point median filter", "thermal mass correction"],
    derivedVariables: ["salinity", "sigma-theta", "sound_velocity"]
  },

  qualityControl: {
    performedBy: "CalCOFI Data QC Team",
    date: new Date("2024-01-20"),
    flags: { overall: 1 },
    notes: "All sensors performing nominally. Oxygen sensor calibrated against Winkler titrations."
  }
};

interface SensorSpec {
  model: string;
  accuracy: number;
  resolution: number;
  units: string;
}
```

### Argo Float Data

Argo floats provide the backbone of global ocean monitoring:

```typescript
interface ArgoFloat {
  floatID: string;                // WMO number
  program: string;                // "Argo", "BGC-Argo", "Deep Argo"
  manufacturer: string;
  model: string;
  deploymentDate: Date;
  deploymentLocation: GeographicCoordinate;
  status: "active" | "inactive" | "dead";

  cycleConfig: {
    parkingDepth: number;         // Meters (typically 1000m)
    profileDepth: number;         // Meters (typically 2000m)
    cyclePeriod: number;          // Days (typically 10)
    surfaceTime: number;          // Hours for satellite transmission
  };

  sensors: {
    CTD: SensorSpec;
    oxygen?: SensorSpec;          // BGC-Argo
    nitrate?: SensorSpec;         // BGC-Argo
    pH?: SensorSpec;              // BGC-Argo
    chlorophyll?: SensorSpec;     // BGC-Argo
    backscatter?: SensorSpec;     // BGC-Argo
    irradiance?: SensorSpec;      // BGC-Argo
  };

  profiles: ArgoProfile[];
}

interface ArgoProfile {
  cycleNumber: number;
  profileDate: Date;
  latitude: number;
  longitude: number;

  measurements: {
    depth: number;
    pressure: number;
    temperature: number;
    salinity: number;
    oxygen?: number;
    nitrate?: number;
    pH?: number;
    chlorophyll?: number;
    qualityFlags: Record<string, QualityFlag>;
  }[];

  dataMode: "R" | "A" | "D";      // Real-time, Adjusted, Delayed-mode
  qcPerformed: string;
}

// Example: BGC-Argo float in North Pacific
const argoFloat: ArgoFloat = {
  floatID: "5906464",
  program: "BGC-Argo",
  manufacturer: "NKE",
  model: "PROVOR",
  deploymentDate: new Date("2021-06-15"),
  deploymentLocation: { latitude: 45.0, longitude: -145.0, datum: "WGS84", uncertainty: 100 },
  status: "active",

  cycleConfig: {
    parkingDepth: 1000,
    profileDepth: 2000,
    cyclePeriod: 10,
    surfaceTime: 6
  },

  sensors: {
    CTD: { model: "SBE41CP", accuracy: 0.002, resolution: 0.001, units: "degC" },
    oxygen: { model: "Aanderaa 4330F", accuracy: 8, resolution: 1, units: "umol/kg" },
    nitrate: { model: "SUNA V2", accuracy: 2, resolution: 0.5, units: "umol/kg" },
    pH: { model: "SBE DuraFET", accuracy: 0.01, resolution: 0.001, units: "pH_total" },
    chlorophyll: { model: "ECO Triplet", accuracy: 0.01, resolution: 0.001, units: "mg/m3" },
    backscatter: { model: "ECO Triplet", accuracy: 0.001, resolution: 0.0001, units: "1/m" }
  },

  profiles: [
    // Profile 1
    {
      cycleNumber: 1,
      profileDate: new Date("2021-06-25T12:00:00Z"),
      latitude: 45.234,
      longitude: -145.567,
      measurements: [/* depth-resolved data */],
      dataMode: "D",
      qcPerformed: "Argo QC procedures v3.1"
    },
    // ... ~150 profiles per year
  ]
};
```

### Satellite Remote Sensing

Satellites provide synoptic views of ocean properties impossible to achieve with in-situ sensors:

```typescript
interface SatelliteOceanProduct {
  product: "SST" | "chlorophyll" | "SSH" | "sea_ice" | "wind_speed";
  satellite: string;              // "MODIS", "VIIRS", "Sentinel-3"
  sensor: string;

  spatialResolution: {
    horizontal: number;           // Kilometers
    gridType: "regular" | "swath";
  };

  temporalResolution: {
    revisit: number;              // Days
    composite: number;            // Days (for averaged products)
  };

  coverage: {
    spatial: "global" | "regional";
    latitude: { min: number; max: number };
    longitude: { min: number; max: number };
  };

  data: SatelliteDataPoint[];

  processing: {
    level: "L2" | "L3" | "L4";    // L2=swath, L3=gridded, L4=gap-filled
    algorithm: string;
    version: string;
    cloudMasking: boolean;
    qualityFiltering: boolean;
  };

  validation: {
    method: string;               // In-situ comparison
    bias: number;
    RMSE: number;                 // Root Mean Square Error
    correlation: number;
  };
}

interface SatelliteDataPoint {
  timestamp: Date;
  latitude: number;
  longitude: number;
  value: number;
  unit: string;
  qualityFlag: number;
  cloudCover: number;             // Percentage
}

// Example: MODIS chlorophyll product
const modisChlorophyll: SatelliteOceanProduct = {
  product: "chlorophyll",
  satellite: "Aqua",
  sensor: "MODIS",

  spatialResolution: {
    horizontal: 4,                // 4km resolution
    gridType: "regular"
  },

  temporalResolution: {
    revisit: 1,                   // Daily coverage
    composite: 8                  // 8-day composites
  },

  coverage: {
    spatial: "global",
    latitude: { min: -90, max: 90 },
    longitude: { min: -180, max: 180 }
  },

  data: [
    {
      timestamp: new Date("2024-01-15T00:00:00Z"),
      latitude: 34.5,
      longitude: -120.5,
      value: 1.23,
      unit: "mg/m3",
      qualityFlag: 1,
      cloudCover: 5
    },
    // ... millions of grid cells
  ],

  processing: {
    level: "L3",
    algorithm: "OC3M",
    version: "R2022.0",
    cloudMasking: true,
    qualityFiltering: true
  },

  validation: {
    method: "in-situ HPLC chlorophyll comparison",
    bias: 0.05,
    RMSE: 0.31,
    correlation: 0.87
  }
};
```

### Data Quality Control

Environmental data quality is critical for marine biology research:

```typescript
class OceanDataQualityControl {

  // Range check: values within physically realistic bounds
  rangeCheck(data: CTDDataPoint): QualityFlag {
    const ranges = {
      temperature: { min: -2, max: 40 },      // Celsius
      salinity: { min: 0, max: 42 },          // PSU
      oxygen: { min: 0, max: 500 },           // μmol/kg
      depth: { min: 0, max: 11000 }           // Meters
    };

    if (data.temperature < ranges.temperature.min ||
        data.temperature > ranges.temperature.max) {
      return 4;  // Bad data
    }
    if (data.salinity < ranges.salinity.min ||
        data.salinity > ranges.salinity.max) {
      return 4;
    }

    return 1;  // Good data
  }

  // Spike check: detect anomalous single points
  spikeCheck(data: CTDDataPoint[], index: number): QualityFlag {
    if (index === 0 || index === data.length - 1) return 1;

    const prev = data[index - 1];
    const curr = data[index];
    const next = data[index + 1];

    const tempGradient1 = Math.abs(curr.temperature - prev.temperature);
    const tempGradient2 = Math.abs(next.temperature - curr.temperature);

    // Temperature spike threshold: 2°C change in 1 meter
    if (tempGradient1 > 2 || tempGradient2 > 2) {
      return 3;  // Potentially correctable
    }

    return 1;
  }

  // Stability check: detect stuck sensors
  stabilityCheck(data: CTDDataPoint[], windowSize: number = 10): QualityFlag {
    const window = data.slice(0, windowSize);
    const temps = window.map(d => d.temperature);

    const variance = this.calculateVariance(temps);

    // Variance too low suggests stuck sensor
    if (variance < 0.0001) {
      return 4;  // Bad data
    }

    return 1;
  }

  // Gradient check: physically unrealistic vertical gradients
  gradientCheck(data: CTDDataPoint[]): QualityFlag[] {
    const flags: QualityFlag[] = [];

    for (let i = 1; i < data.length; i++) {
      const prev = data[i - 1];
      const curr = data[i];

      const depthChange = curr.depth - prev.depth;
      const tempChange = curr.temperature - prev.temperature;

      // Vertical temperature gradient
      const gradient = tempChange / depthChange;

      // Realistic gradients: -0.5°C/m to +0.5°C/m
      if (Math.abs(gradient) > 0.5) {
        flags.push(3);  // Potentially correctable
      } else {
        flags.push(1);  // Good
      }
    }

    return flags;
  }

  // Climatology comparison: compare to historical averages
  climatologyCheck(data: CTDDataPoint, location: GeographicCoordinate, month: number): QualityFlag {
    // Load World Ocean Atlas climatology for this location/month/depth
    const climatology = this.loadClimatology(location, month, data.depth);

    const tempDiff = Math.abs(data.temperature - climatology.temperature);
    const salDiff = Math.abs(data.salinity - climatology.salinity);

    // Flag if >3 standard deviations from climatology
    if (tempDiff > 3 * climatology.tempStdDev ||
        salDiff > 3 * climatology.salStdDev) {
      return 2;  // Probably good (could be real event, not error)
    }

    return 1;
  }

  private calculateVariance(values: number[]): number {
    const mean = values.reduce((a, b) => a + b) / values.length;
    return values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;
  }

  private loadClimatology(location: GeographicCoordinate, month: number, depth: number) {
    // Implementation would query World Ocean Atlas or similar
    return {
      temperature: 15.0,
      salinity: 35.0,
      tempStdDev: 1.5,
      salStdDev: 0.5
    };
  }
}
```

### Best Practices for Environmental Data

**1. Always Include Metadata**
Record instrument type, calibration dates, and measurement methods.

**2. Use Standard Units**
- Temperature: degrees Celsius (ITS-90)
- Salinity: Practical Salinity Units (PSU)
- Pressure: decibars (dbar)
- Oxygen: micromoles per kilogram (μmol/kg)
- Nutrients: micromoles per liter (μmol/L)

**3. Implement Quality Flags**
Use standardized quality flagging (IODE/IOC scheme) for all parameters.

**4. Document Calibrations**
Pre- and post-deployment calibrations with dates and methods.

**5. Preserve Raw Data**
Keep unprocessed data alongside processed versions.

**6. Use Standard Formats**
NetCDF-CF for gridded data, CSV with metadata for tabular data.

**7. Reference Standard Seawater**
Calibrate salinity sensors against IAPSO Standard Seawater.

### Philosophy: Measuring the Ocean for All

Environmental monitoring data is a global public good. The Argo program alone represents $500 million in international investment, with all data freely available within 24 hours. This embodies 弘益人間 - benefiting all humanity.

When you download a CTD profile or satellite image, you access the work of thousands of scientists, engineers, and mariners who deployed sensors, maintained platforms, and ensured data quality. Use it wisely, share it freely, and contribute back when you can.

---

**Next Chapter:** Genomic and DNA Sequencing - using molecular tools to identify species, understand evolution, and reveal hidden biodiversity.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
