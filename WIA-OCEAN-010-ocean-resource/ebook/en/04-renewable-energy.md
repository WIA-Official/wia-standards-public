# Chapter 4: Renewable Energy (Wind, Wave, Tidal)

## The Clean Energy Revolution at Sea

The ocean holds **phenomenal renewable energy potential**: **71,000 TWh/year from wind**, **32,000 TWh/year from waves**, and **800 TWh/year from tides**. For context, global electricity consumption is approximately **28,000 TWh/year**.

Offshore renewable energy is **the fastest-growing ocean resource sector**, with offshore wind capacity reaching **75 GW globally in 2024** and projected to exceed **380 GW by 2030**. This clean energy revolution offers a path to decarbonize while creating ocean-based economic opportunities.

### Offshore Wind Energy

#### Global Offshore Wind Landscape

```typescript
interface OffshoreWindIndustry {
  year: 2024;

  installedCapacity: {
    global: 75000,                    // Megawatts
    underConstruction: 45000,
    pipeline: 260000,                 // Projects in planning
  };

  generation: {
    annualProduction: 280,            // Terawatt-hours
    capacity Factor: 42,              // Percentage (higher than onshore 35%)
    homesSupplied: 75000000,
  };

  regions: {
    europe: {
      capacity: 35000,                // MW - global leader
      countries: {
        uk: 15000,
        germany: 8500,
        netherlands: 4000,
        denmark: 2500,
        belgium: 2300
      }
    },
    china: {
      capacity: 37000,                // MW - fastest growth
      annualAdditions: 7000,
      targetCapacity: 100000          // MW by 2030
    },
    usa: {
      capacity: 140,                  // MW - emerging market
      target Capacity: 30000,          // MW by 2030
      pipelineProjects: 15
    }
  };

  economics: {
    capitalCost: 3500,                // USD per kW
    operatingCost: 75,                // USD per kW per year
    lcoe: 65,                         // Levelized cost of energy USD per MWh
    investment2024: 85000000000       // USD
  };
}
```

### Offshore Wind Farm Data Model

```typescript
interface OffshoreWindFarm {
  farmId: string;
  name: string;
  location: WindFarmArea;
  status: "operational" | "construction" | "planning" | "decommissioned";
  operator: string;

  technical: {
    capacity: number;                 // Megawatts
    turbines: Turbine[];
    foundation: FoundationType;
    grid Connection: GridConnection;
  };

  environmental: {
    impact Assessment: EnvironmentalImpact;
    monitoring: EnvironmentalMonitoring;
    mitigationMeasures: Mitigation[];
  };

  performance: {
    generation: GenerationData;
    availability: number;             // Percentage
    capacity Factor: number;
    maintenance: MaintenanceData;
  };

  economics: {
    totalInvestment: number;          // USD
    subsidies: number;
    powerPurchaseAgreement: PPATerms;
    revenue: RevenueData;
  };
}

interface Turbine {
  turbineId: string;
  model: string;
  manufacturer: string;

  specifications: {
    ratedPower: number;               // Megawatts (8-15 MW common in 2025)
    rotorDiameter: number;            // Meters (180-220m)
    hubHeight: number;                // Meters above sea level
    tipHeight: number;                // Maximum height
    bladeMaterial: string;            // "carbon_fiber_composite"
  };

  location: {
    coordinates: GeographicCoordinate;
    waterDepth: number;
    distanceToShore: number;          // Kilometers
  };

  performance: {
    production: TimeSeries;           // Megawatt-hours
    windSpeed: TimeSeries;            // Meters per second
    availability: number;             // Percentage
    curtailment: number;              // Hours per year when stopped for grid reasons
  };

  condition: {
    health Status: "excellent" | "good" | "fair" | "poor";
    vibration: VibrationMonitoring;
    temperature: ComponentTemperature;
    oil Analysis: LubricantAnalysis;
    corrosion: CorrosionAssessment;
  };
}
```

#### Foundation Technologies

Water depth determines foundation type:

```typescript
enum FoundationType {
  MONOPILE = "monopile",                      // 0-35m depth, most common
  JACKET = "jacket",                          // 20-60m depth
  TRIPOD = "tripod",                          // 20-50m depth
  GRAVITY_BASE = "gravity_base",              // 0-40m depth
  SUCTION_BUCKET = "suction_bucket",          // 20-50m depth
  FLOATING_SPAR = "floating_spar",            // 50-300m+ depth
  FLOATING_SEMI = "floating_semi_submersible", // 50-300m+ depth
  FLOATING_TLP = "floating_tension_leg"       // 50-200m depth
}

interface Foundation {
  type: FoundationType;
  waterDepth: number;

  monopile?: {
    diameter: number;                 // Meters (8-12m typical)
    wallThickness: number;            // Millimeters
    penetrationDepth: number;         // Meters into seabed
    weight: number;                   // Tons (800-2000)
    installedLength: number;          // Meters
    cathodicProtection: boolean;
  };

  floatingSystem?: {
    type: "spar" | "semi_submersible" | "tension_leg";
    displacement: number;             // Tons
    draft: number;                    // Meters
    mooringLines: {
      number: number;                 // Typically 3-4
      length: number;                 // Meters
      material: string;               // "chain", "synthetic_rope"
      anchorType: string;             // "drag_embedment", "suction_pile"
    };
    stability: {
      pitch: number;                  // Degrees maximum tilt
      roll: number;
      heave: number;                  // Meters vertical motion
    };
  };

  installation: {
    vessel Required: string;           // "jack_up", "heavy_lift", "crane_barge"
    duration: number;                 // Days per turbine
    weather Window: {
      maxWaveHeight: number;          // Meters
      maxWindSpeed: number;           // Meters per second
      workabilityPercentage: number;  // Percentage of year suitable
    };
  };

  scourProtection: {
    required: boolean;
    material: string;                 // "rock_armor", "concrete_mattress"
    extent: number;                   // Meters radius
  };
}
```

**Floating Wind - The Future:**

Floating wind unlocks **80% of global offshore wind resource** in deep waters:

```typescript
const floatingWindDevelopment = {
  waterDepthsAccessible: "50+ meters",
  globalPotential: 56000,             // TWh per year

  currentStatus: {
    operational: 235,                 // MW globally
    underDevelopment: 12000,          // MW
    commercialProjects: 25
  },

  leadingProjects: {
    hywind Tampen: {
      location: "Norway",
      capacity: 88,                   // MW
      turbines: 11,
      waterDepth: 260,                // Meters
      type: "spar",
      operator: "Equinor"
    },
    kincardine: {
      location: "Scotland",
      capacity: 50,
      turbines: 5,
      waterDepth: 60,
      type: "semi_submersible"
    }
  },

  costReduction: {
    current: 110,                     // USD per MWh LCOE
    target2030: 65,                   // Competitive with fixed-bottom
    drivers: [
      "Serial production",
      "Larger turbines",
      "Local supply chains",
      "Installation optimization"
    ]
  }
};
```

### Wind Resource Assessment

Comprehensive wind data drives site selection:

```typescript
interface WindResourceAssessment {
  location: GeographicCoordinate;
  measurementPeriod: {
    start: Date;
    end: Date;
    duration: number;                 // Months (minimum 12)
  };

  windData: {
    meanWindSpeed: number;            // Meters per second at hub height
    weibullParameters: {
      shapeK: number;
      scaleA: number;
    };
    turbulenceIntensity: number;      // Percentage
    windShear: number;                // Power law exponent
    directionality: WindRose;
  };

  energyProduction: {
    grossAEP: number;                 // Annual energy production GWh
    losses: {
      wake: number;                   // Percentage (8-15% typical)
      availability: number;           // (2-5%)
      electrical: number;             // (3%)
      environmental: number;          // (2% for curtailment)
      total: number;
    };
    netAEP: number;
    capacity Factor: number;           // Percentage (40-50% offshore)
    p50: number;                      // 50% probability of exceedance
    p90: number;                      // 90% probability (conservative estimate)
  };

  extremeConditions: {
    maxWindSpeed50year: number;       // Meters per second
    maxWaveHeight50year: number;      // Meters
    iceLoading: boolean;
    earthquakeRisk: number;
    lightningDensity: number;         // Flashes per km² per year
  };

  measurements: {
    lidar: LidarData;
    metiOcean buoy: MetoceanData;
    satellite: SatelliteWindData;
    mesoscaleModeling: NumericalModel;
  };
}

interface LidarData {
  device: "floating_lidar" | "fixed_lidar";
  measurementHeights: number[];       // Meters [50, 100, 150, 200, 250]
  dataAvailability: number;           // Percentage (target >90%)
  measurements: {
    windSpeed: TimeSeries;
    windDirection: TimeSeries;
    verticalProfile: VerticalWindProfile;
  };
}
```

### Environmental Impact and Mitigation

Offshore wind farms affect marine ecosystems:

```typescript
interface EnvironmentalImpact {
  construction: {
    piledriving: {
      noiseLevel: number;             // Decibels at 750m (170-180 dB typical)
      duration: number;               // Hours per monopile
      impact: "Marine mammal disturbance, fish avoidance";
      mitigation: [
        "Bubble curtains (10-20 dB reduction)",
        "Marine mammal observers",
        "Acoustic deterrent devices",
        "Seasonal restrictions",
        "Slow soft-start ramp-up"
      ];
    };

    seabedDisturbance: {
      areaAffected: number;           // Square kilometers
      habitatLoss: string[];          // "Sandy bottom", "gravel"
      sedimentPlume: number;          // Kilometers extent
      recovery Time: number;           // Years
    };

    vesselTraffic: {
      trips: number;                  // During construction
      collisionRisk: "marine mammals";
      mitigation: "Speed restrictions, observers";
    };
  };

  operation: {
    avianImpact: {
      collisionRisk: {
        species: string[];            // Seabirds, migratory birds
        mortality Estimate: number;    // Birds per MW per year
        populationImpact: "low" | "medium" | "high";
      };

      displacement: {
        species: string[];
        displacementDistance: number;  // Kilometers
        habitatLoss: number;           // Percentage of population range
      };

      mitigation: [
        "Radar monitoring",
        "Seasonal shutdowns during migration",
        "Turbine lighting optimization",
        "Deterrent systems",
        "Offshore habitat compensation"
      ];
    };

    marineLife: {
      reefEffect: {
        description: "Foundations create artificial reef habitat";
        biomassIncrease: number;      // Percentage within wind farm
        species: string[];            // Mussels, crustaceans, fish
        benefit: "Increased fish stocks, biodiversity";
      };

      electromagneticFields: {
        source: "Subsea cables";
        strength: number;             // Microtesla
        affectedSpecies: string[];    // Elasmobranchs, marine mammals
        impactLevel: "low";
      };

      noise: {
        operationalNoise: number;     // Decibels (120-135 dB near turbine)
        frequency: number;            // Hertz
        propagation: number;          // Kilometers
        impact: "Temporary avoidance by marine mammals";
      };
    };

    fisheriesInteraction: {
      areaRestricted: number;         // Square kilometers
      compensationValue: number;      // USD per year
      coexistence: {
        allowedFishing: string[];     // "Pot fishing", "angling"
        prohibited: string[];         // "Bottom trawling", "dredging"
      };
    };
  };

  decommissioning: {
    removal: "Full removal of above-seabed structures";
    foundationRemoval: "Cut 1-3m below seabed, or full removal";
    ecosystemImpact: "Loss of artificial reef";
    restoration: "Seabed recovery 2-5 years";
  };
}
```

### Wave Energy

Wave power offers consistent, predictable renewable energy:

```typescript
interface WaveEnergyPotential {
  globalResource: 32000,              // TWh per year theoretical
  technicalPotential: 2000,           // TWh per year realistically extractable

  hotspots: {
    pacificWest: {
      powerDensity: 50,               // kW per meter of wave crest
      regions: ["Oregon", "British Columbia", "Chile", "Australia"]
    },
    atlanticEast: {
      powerDensity: 40,
      regions: ["Ireland", "Scotland", "Portugal", "Morocco"]
    },
    southernOcean: {
      powerDensity: 70,
      regions: ["New Zealand", "South Africa", "Southern Chile"]
    }
  };

  currentStatus: {
    installedCapacity: 35,            // MW - nascent industry
    prototypes: 150,
    commercialProjects: 8,
    investmentNeeded: "Technology maturation"
  };
}

interface WaveEnergyDevice {
  deviceId: string;
  type: WaveEnergyConverterType;
  location: GeographicCoordinate;
  waterDepth: number;

  specifications: {
    ratedPower: number;               // Kilowatts (100-1000 kW typical)
    survivalMode: number;             // Maximum wave height in meters
    dimensions: {
      length: number;
      width: number;
      height: number;
    };
    weight: number;                   // Tons
  };

  performance: {
    capacityFactor: number;           // Percentage (25-35% typical)
    efficiency: number;               // Percentage of wave power extracted
    generation: TimeSeries;
    availability: number;
  };

  mooring: {
    type: string;                     // "Catenary", "taut", "dynamic"
    lines: number;
    anchorDepth: number;
  };
}

enum WaveEnergyConverterType {
  POINT_ABSORBER = "point_absorber",              // Float bobbing vertically
  OSCILLATING_SURGE = "oscillating_surge_wave",   // Flap pivoting with waves
  OSCILLATING_WATER_COLUMN = "oscillating_water_column", // Air turbine
  OVERTOPPING = "overtopping_device",             // Reservoir filling
  ATTENUATOR = "attenuator"                       // Long floating structure
}
```

**Leading Wave Energy Technologies:**

```typescript
const waveTechnologies = {
  pelamis: {
    type: "attenuator",
    length: 180,                      // Meters - snake-like
    power: 750,                       // kW
    status: "Decommissioned - proved concept",
    location: "Portugal Aguçadoura 2008"
  },

  corPower: {
    type: "point_absorber",
    power: 300,                       // kW
    status: "Commercial development",
    location: "Portugal HiWave-5 project",
    innovation: "Wave-by-wave energy maximization control"
  },

  ecoWave: {
    type: "oscillating_surge",
    power: 400,                       // kW per floater
    status: "Operating",
    location: "Israel, Gibraltar",
    installation: "Attached to breakwaters, piers"
  }
};
```

### Tidal Energy

Tidal power provides **100% predictable** renewable energy:

```typescript
interface TidalEnergyResource {
  globalPotential: 800,               // TWh per year at high-resource sites
  installedCapacity: 520,             // MW globally (2024)

  tidalRange: {
    description: "Barrage and lagoon systems";
    operational: {
      laRance: {
        location: "France",
        capacity: 240,                // MW
        operational Since: 1966,
        annualGeneration: 500,        // GWh
        tidalRange: 8.5               // Meters
      },
      sihwaLake: {
        location: "South Korea",
        capacity: 254,                // MW
        tidalRange: 7.8,
        notes: "World's largest tidal power plant"
      }
    }
  };

  tidalStream: {
    description: "Underwater turbines in strong currents";
    installedCapacity: 26,            // MW
    pipeline: 2500,                   // MW in development

    topSites: {
      pentlandFirth: {
        location: "Scotland",
        peakCurrentSpeed: 5,          // Meters per second
        capacity Potential: 1800,      // MW
        projects: ["MeyGen 6MW operational, 80MW planned"]
      },
      bayOfFundy: {
        location: "Canada",
        tidalRange: 16,               // Meters - highest in world
        currentSpeed: 4,
        potential: 7000                // MW
      }
    }
  };
}

interface TidalStreamDevice {
  deviceId: string;
  type: "horizontal_axis" | "vertical_axis" | "ducted" | "reciprocating";
  location: GeographicCoordinate;

  hydrodynamics: {
    peakCurrentSpeed: number;         // Meters per second
    tidalCycle: {
      floodDirection: number;         // Degrees
      ebbDirection: number;
      slackDuration: number;          // Minutes
      cyclesPerDay: number;           // Typically 2
    };
  };

  turbine: {
    ratedPower: number;               // Kilowatts (500-2000 kW)
    rotorDiameter: number;            // Meters
    cutInSpeed: number;               // Meters per second (0.5-1.0)
    ratedSpeed: number;               // Meters per second (2.5-3.5)
    survivalSpeed: number;            // Meters per second (5-7)
  };

  foundation: {
    type: "seabed_mounted" | "floating";
    depth: number;
    cableConnection: string;
  };

  performance: {
    capacityFactor: number;           // Percentage (35-45%, better than wind)
    predictability: "100% - tides are perfectly predictable";
    availability: number;
    generationProfile: PredictableProfile;
  };

  environmental: {
    fishInteraction: "Low - fish avoid rotors";
    marineeMammal: "Low risk - slow moving, predictable";
    sedimentTransport: "Minimal change";
    noise: "Lower than wind turbines";
  };
}
```

### Ocean Thermal Energy Conversion (OTEC)

Harvesting energy from ocean temperature gradients:

```typescript
interface OTECSystem {
  principle: "Use temperature difference between warm surface water (25°C) and cold deep water (5°C)";

  requirements: {
    temperatureDifference: number;    // Minimum 20°C
    suitableLocations: string[];      // Tropical and subtropical waters
    depthRequired: 1000               // Meters for cold water access
  };

  technology: {
    type: "closed_cycle" | "open_cycle" | "hybrid";
    workingFluid: string;             // "Ammonia" for closed cycle
    powerOutput: number;              // Megawatts (1-100 MW)
    efficiency: number;               // Percentage (2-4%, limited by Carnot)
  };

  closedCycle: {
    warmWaterFlow: number;            // Cubic meters per second
    coldWaterFlow: number;
    pipeLength: number;               // Meters (1000m)
    pipeDiameter: number;             // Meters (10m)
    turbineType: "Ammonia vapor turbine";
  };

  benefits: [
    "Baseload 24/7 generation",
    "No fuel required",
    "Freshwater byproduct from desalination",
    "Nutrients from deep water enable aquaculture",
    "Hurricane-resistant (subsea)"
  ];

  challenges: [
    "Low efficiency requires large infrastructure",
    "Expensive cold water pipe",
    "Biofouling management",
    "Limited to tropical locations",
    "Limited commercial deployment"
  ];

  currentProjects: {
    hawaii Natural Energy Institute: {
      capacity: 1,                    // MW test facility
      status: "Experimental",
      location: "Hawaii"
    },
    martinique: {
      capacity: 10,                   // MW planned
      status: "Development",
      operator: "Naval Energies / DCNS"
    }
  };
}
```

### Grid Integration and Energy Storage

Offshore renewables require sophisticated grid solutions:

```typescript
interface GridIntegration {
  transmission: {
    cables: SubseaCable[];
    voltage: number;                  // Kilovolts (150-400 kV HVAC, ±320 kV HVDC)
    capacity: number;                 // Megawatts
    length: number;                   // Kilometers to shore
    losses: number;                   // Percentage (2-3% per 100km AC, 0.7% DC)
  };

  substations: {
    offshore: {
      capacity: number;               // Megawatts
      voltage Transformation: string;  // "66kV to 220kV"
      type: "HVAC" | "HVDC";
      reactive Power Compensation: boolean;
    };
    onshore: {
      gridConnectionPoint: string;
      transformerCapacity: number;
    };
  };

  variabilityManagement: {
    forecasting: {
      horizon: string[];              // "1-hour-ahead", "1-day-ahead"
      accuracy: number;               // Percentage
      method: "Numerical weather prediction, ML";
    };

    storage: {
      battery: {
        capacity: number;             // Megawatt-hours
        power: number;                // Megawatts
        duration: number;             // Hours (2-4 typical)
        chemistry: "Lithium-ion";
      };

      pumped Hydro: {
        available: boolean;
        capacityMWh: number;
        efficiency: number;           // Percentage (75-85%)
      };

      hydrogen: {
        electrolysis: number;         // MW capacity
        storage: number;              // Tons H2
        roundTripEfficiency: number;  // Percentage (30-40%)
        use: "Long-duration storage, industry feedstock";
      };
    };

    demandResponse: {
      flexible Load: number;           // MW controllable
      industries: string[];           // "Aluminum", "hydrogen", "desalination"
      curtailment Compensation: number; // USD per MWh
    };
  };

  marineSpatialPlanning: {
    conflictsWith: string[];          // "Shipping lanes", "fishing", "military"
    coexistence: {
      shipping: "Traffic separation schemes through wind farms";
      fishing: "Selective fishing allowed";
      cables: "Burial depth 1-3m, crossing agreements";
    };
  };
}
```

### Philosophy: 弘益人間 in Ocean Renewables

Ocean renewable energy embodies 弘益人間 - benefiting all humanity:

**Clean Energy for All:**
- **No carbon emissions** during operation
- **Abundant resource** accessible to coastal nations
- **Energy independence** from fossil fuel imports
- **Economic development** in coastal communities

**Environmental Stewardship:**
- **Protecting oceans** from climate change
- **Minimal ecosystem impact** compared to fossil fuels
- **Artificial reefs** from foundations
- **Careful siting** to avoid sensitive areas

**Technological Advancement:**
- **Innovation** driving cost reductions (85% for offshore wind since 2010)
- **Job creation** - 250,000+ in offshore wind globally
- **Knowledge sharing** through international collaboration

The ocean offers humanity a path to abundant, clean energy. Developing it wisely benefits all.

---

**Next Chapter:** Deep sea mining - extracting minerals from the abyssal depths for renewable energy technologies.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
