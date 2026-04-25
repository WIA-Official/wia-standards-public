# Chapter 5: Ecosystem Modeling

## Integrating Data to Understand Marine Ecosystems

Marine ecosystems are complex networks of interacting species and environmental processes. Ecosystem models integrate biological observations, environmental data, and genomic information to understand food webs, predict fisheries yields, assess climate change impacts, and guide conservation decisions.

### Types of Marine Ecosystem Models

Different modeling approaches serve different purposes:

```typescript
interface EcosystemModel {
  modelType: "food_web" | "population" | "biogeochemical" | "end_to_end" | "spatial";
  purpose: string;
  spatialExtent: "local" | "regional" | "global";
  temporalExtent: {
    start: Date;
    end: Date;
    timestep: string;          // "hourly", "daily", "monthly"
  };

  components: ModelComponent[];
  forcingData: ForcingData[];
  parameters: ModelParameter[];
  calibration: CalibrationData;
  validation: ValidationData;
}

interface ModelComponent {
  name: string;
  type: "physical" | "chemical" | "biological";
  variables: Variable[];
  equations: string[];           // Mathematical formulations
}

interface Variable {
  name: string;
  symbol: string;
  unit: string;
  initialValue: number;
  uncertainty?: number;
}
```

#### Food Web Models

Describe trophic relationships and energy flow:

```typescript
interface FoodWebModel {
  ecosystem: string;             // "coral_reef", "kelp_forest", "open_ocean"
  location: string;

  species: SpeciesNode[];
  interactions: TrophicLink[];

  structure: {
    trophicLevels: number;       // Typically 4-6
    connectance: number;         // Fraction of realized links (0-1)
    complexity: number;          // Species × links
  };

  energetics: {
    primaryProduction: number;   // gC/m²/year
    totalBiomass: number;        // gC/m²
    transferEfficiency: number;  // Between trophic levels (typically 10%)
  };
}

interface SpeciesNode {
  aphiaID: number;
  scientificName: string;
  functionalGroup: string;       // "phytoplankton", "herbivorous_fish", "apex_predator"

  trophicLevel: number;          // 1 (producers) to 5+ (apex predators)
  biomass: number;               // gC/m² or tonnes/km²
  production: number;            // gC/m²/year
  consumption: number;           // gC/m²/year

  lifeHistory: {
    maxAge: number;              // Years
    maturityAge: number;
    growth: GrowthFunction;
    mortality: MortalityFunction;
    reproduction: ReproductionFunction;
  };
}

interface TrophicLink {
  predator: number;              // AphiaID
  prey: number;                  // AphiaID
  strength: number;              // Proportion of prey in predator diet (0-1)
  consumptionRate: number;       // gC/m²/year
  efficiency: number;            // Assimilation efficiency (0-1)
}

type GrowthFunction = {
  type: "von_Bertalanffy" | "exponential" | "logistic";
  parameters: {
    L_infinity?: number;         // Maximum length
    K?: number;                  // Growth rate
    t0?: number;                 // Theoretical age at length 0
  };
};

type MortalityFunction = {
  natural: number;               // Natural mortality rate (year⁻¹)
  fishing?: number;              // Fishing mortality rate (year⁻¹)
  predation: number;             // Predation mortality rate (year⁻¹)
};

type ReproductionFunction = {
  spawningSeasonality: "year_round" | "seasonal" | "lunar";
  fecundity: number;             // Eggs per reproductive event
  recruitmentRate: number;       // Proportion surviving to adulthood
};

// Example: Coral reef food web
const coralReefFoodWeb: FoodWebModel = {
  ecosystem: "coral_reef",
  location: "Great Barrier Reef, Australia",

  species: [
    {
      aphiaID: 148899,           // Zooxanthellae (symbiotic algae)
      scientificName: "Symbiodinium sp.",
      functionalGroup: "primary_producer",
      trophicLevel: 1.0,
      biomass: 500,              // gC/m²
      production: 2000,          // gC/m²/year
      consumption: 0,
      lifeHistory: {
        maxAge: 0.1,             // Short generation time
        maturityAge: 0.01,
        growth: { type: "exponential", parameters: { K: 2.0 } },
        mortality: { natural: 10, predation: 8 },
        reproduction: { spawningSeasonality: "year_round", fecundity: 1000000, recruitmentRate: 0.001 }
      }
    },
    {
      aphiaID: 267808,           // Parrotfish
      scientificName: "Scarus rivulatus",
      functionalGroup: "herbivorous_fish",
      trophicLevel: 2.1,
      biomass: 50,
      production: 25,
      consumption: 100,
      lifeHistory: {
        maxAge: 10,
        maturityAge: 2,
        growth: {
          type: "von_Bertalanffy",
          parameters: { L_infinity: 40, K: 0.3, t0: -0.5 }
        },
        mortality: { natural: 0.3, fishing: 0.2, predation: 0.1 },
        reproduction: { spawningSeasonality: "seasonal", fecundity: 50000, recruitmentRate: 0.0001 }
      }
    },
    {
      aphiaID: 212844,           // Coral grouper (from earlier example)
      scientificName: "Plectropomus leopardus",
      functionalGroup: "piscivorous_fish",
      trophicLevel: 4.2,
      biomass: 10,
      production: 3,
      consumption: 15,
      lifeHistory: {
        maxAge: 15,
        maturityAge: 4,
        growth: {
          type: "von_Bertalanffy",
          parameters: { L_infinity: 120, K: 0.15, t0: -1.0 }
        },
        mortality: { natural: 0.2, fishing: 0.5, predation: 0.05 },
        reproduction: { spawningSeasonality: "lunar", fecundity: 200000, recruitmentRate: 0.00001 }
      }
    }
  ],

  interactions: [
    {
      predator: 267808,          // Parrotfish
      prey: 148899,              // Zooxanthellae (via algae grazing)
      strength: 0.8,             // 80% of diet
      consumptionRate: 80,
      efficiency: 0.15
    },
    {
      predator: 212844,          // Coral grouper
      prey: 267808,              // Parrotfish
      strength: 0.3,             // 30% of diet
      consumptionRate: 4.5,
      efficiency: 0.20
    }
  ],

  structure: {
    trophicLevels: 5,
    connectance: 0.15,
    complexity: 234              // 78 species × 3 links average
  },

  energetics: {
    primaryProduction: 2500,     // gC/m²/year (high for coral reefs)
    totalBiomass: 1500,          // gC/m²
    transferEfficiency: 0.12     // 12% between levels
  }
};
```

#### Biogeochemical Models

Track nutrient cycles and primary production:

```typescript
interface BiogeochemicalModel {
  modelName: string;             // "NPZD", "PISCES", "BFM"

  state_variables: {
    nutrients: {
      nitrate: Variable;         // NO₃
      phosphate: Variable;       // PO₄
      silicate: Variable;        // SiO₄
      iron: Variable;            // Fe
      ammonia: Variable;         // NH₄
    };

    phytoplankton: {
      diatoms: Variable;         // Large cells, siliceous
      flagellates: Variable;     // Small cells
      cyanobacteria: Variable;   // Nitrogen fixers
    };

    zooplankton: {
      microzooplankton: Variable;
      mesozooplankton: Variable;
    };

    detritus: {
      particulateOrganic: Variable;  // POC
      dissolvedOrganic: Variable;    // DOC
    };

    oxygen: Variable;
    carbon: {
      DIC: Variable;             // Dissolved Inorganic Carbon
      alkalinity: Variable;
      pH: Variable;
    };
  };

  processes: {
    photosynthesis: PhotosynthesisModel;
    nutrientUptake: NutrientUptakeModel;
    grazing: GrazingModel;
    respiration: RespirationModel;
    remineralization: RemineralizationModel;
    sinking: SinkingModel;
  };

  forcing: {
    light: number;               // μmol photons/m²/s (PAR)
    temperature: number;         // Celsius
    mixedLayerDepth: number;     // Meters
    upwelling: number;           // m/day
  };
}

interface PhotosynthesisModel {
  type: "Michaelis-Menten" | "quota" | "acclimation";

  lightResponse: {
    maxRate: number;             // Maximum photosynthesis rate (day⁻¹)
    alpha: number;               // Initial slope
    beta?: number;               // Photoinhibition parameter
  };

  nutrientLimitation: {
    halfSaturation: {
      nitrate: number;           // μmol/L
      phosphate: number;
      iron: number;
    };
  };

  temperatureResponse: {
    Q10: number;                 // Temperature coefficient
    optimum: number;             // Optimal temperature (°C)
  };
}

// Example: NPZD (Nutrient-Phytoplankton-Zooplankton-Detritus) model
const npzdModel: BiogeochemicalModel = {
  modelName: "NPZD",

  state_variables: {
    nutrients: {
      nitrate: { name: "Nitrate", symbol: "N", unit: "mmol N/m³", initialValue: 5.0 },
      phosphate: { name: "Phosphate", symbol: "P", unit: "mmol P/m³", initialValue: 0.3 },
      silicate: { name: "Silicate", symbol: "Si", unit: "mmol Si/m³", initialValue: 2.0 },
      iron: { name: "Iron", symbol: "Fe", unit: "nmol Fe/m³", initialValue: 0.5 },
      ammonia: { name: "Ammonia", symbol: "NH4", unit: "mmol N/m³", initialValue: 0.1 }
    },

    phytoplankton: {
      diatoms: { name: "Diatoms", symbol: "PD", unit: "mmol N/m³", initialValue: 1.0 },
      flagellates: { name: "Flagellates", symbol: "PF", unit: "mmol N/m³", initialValue: 0.5 },
      cyanobacteria: { name: "Cyanobacteria", symbol: "PC", unit: "mmol N/m³", initialValue: 0.1 }
    },

    zooplankton: {
      microzooplankton: { name: "Microzooplankton", symbol: "ZM", unit: "mmol N/m³", initialValue: 0.2 },
      mesozooplankton: { name: "Mesozooplankton", symbol: "ZL", unit: "mmol N/m³", initialValue: 0.1 }
    },

    detritus: {
      particulateOrganic: { name: "PON", symbol: "D", unit: "mmol N/m³", initialValue: 0.5 },
      dissolvedOrganic: { name: "DON", symbol: "DON", unit: "mmol N/m³", initialValue: 2.0 }
    },

    oxygen: { name: "Oxygen", symbol: "O2", unit: "mmol O2/m³", initialValue: 250 },

    carbon: {
      DIC: { name: "DIC", symbol: "DIC", unit: "mmol C/m³", initialValue: 2100 },
      alkalinity: { name: "Alkalinity", symbol: "Alk", unit: "mmol/m³", initialValue: 2300 },
      pH: { name: "pH", symbol: "pH", unit: "pH units", initialValue: 8.1 }
    }
  },

  processes: {
    photosynthesis: {
      type: "Michaelis-Menten",
      lightResponse: {
        maxRate: 2.0,
        alpha: 0.1,
        beta: 0.01
      },
      nutrientLimitation: {
        halfSaturation: {
          nitrate: 1.0,
          phosphate: 0.1,
          iron: 0.05
        }
      },
      temperatureResponse: {
        Q10: 2.0,
        optimum: 20
      }
    },
    nutrientUptake: {
      /* Model details */
    },
    grazing: {
      /* Model details */
    },
    respiration: {
      /* Model details */
    },
    remineralization: {
      /* Model details */
    },
    sinking: {
      /* Model details */
    }
  } as any,

  forcing: {
    light: 500,
    temperature: 18,
    mixedLayerDepth: 50,
    upwelling: 0.5
  }
};
```

#### Spatial Ecosystem Models

Simulate ecosystem dynamics across space and time:

```typescript
interface SpatialEcosystemModel {
  domain: {
    bounds: {
      latitudeMin: number;
      latitudeMax: number;
      longitudeMin: number;
      longitudeMax: number;
    };

    resolution: {
      horizontal: number;        // Kilometers
      vertical: number[];        // Depth layers (meters)
    };

    grid: {
      type: "regular" | "curvilinear" | "unstructured";
      cells: number;
      vertices: number;
    };
  };

  physics: {
    hydrodynamicModel: string;   // "ROMS", "MOM6", "NEMO"
    current: boolean;
    temperature: boolean;
    salinity: boolean;
    mixingVertical: boolean;
    mixingHorizontal: boolean;
  };

  biology: {
    model: string;               // "NPZD", "PISCES", "Ecopath"
    planktonGroups: number;
    fishGroups: number;
    benthicGroups: number;
  };

  coupling: {
    onlineOffline: "online" | "offline";
    timestep: number;            // Seconds
    outputFrequency: string;     // "daily", "monthly"
  };
}

// Example: California Current ecosystem model
const californiaCurrentModel: SpatialEcosystemModel = {
  domain: {
    bounds: {
      latitudeMin: 30,
      latitudeMax: 48,
      longitudeMin: -130,
      longitudeMax: -115
    },

    resolution: {
      horizontal: 10,            // 10 km grid
      vertical: [0, 5, 10, 20, 30, 50, 75, 100, 150, 200, 300, 500, 1000, 2000]
    },

    grid: {
      type: "regular",
      cells: 18000,
      vertices: 18225
    }
  },

  physics: {
    hydrodynamicModel: "ROMS",
    current: true,
    temperature: true,
    salinity: true,
    mixingVertical: true,
    mixingHorizontal: true
  },

  biology: {
    model: "NPZD+Fish",
    planktonGroups: 4,           // diatoms, flagellates, microzoo, mesozoo
    fishGroups: 3,               // small pelagic, large pelagic, demersal
    benthicGroups: 0
  },

  coupling: {
    onlineOffline: "online",
    timestep: 300,               // 5 minutes for physics, daily for biology
    outputFrequency: "daily"
  }
};
```

### Model Calibration and Validation

```typescript
interface ModelCalibration {
  method: "least_squares" | "bayesian" | "ensemble" | "machine_learning";

  observations: {
    source: string;
    dataType: string[];          // ["chlorophyll", "temperature", "fish_abundance"]
    spatialCoverage: string;
    temporalCoverage: { start: Date; end: Date };
    uncertainty: number;
  }[];

  parameters: {
    name: string;
    priorValue: number;
    priorRange: [number, number];
    posteriorValue: number;
    posteriorUncertainty: number;
    sensitivity: number;         // Normalized sensitivity coefficient
  }[];

  optimization: {
    algorithm: string;           // "MCMC", "genetic_algorithm", "gradient_descent"
    iterations: number;
    convergence: boolean;
    computeTime: number;         // Hours
  };

  performance: {
    RMSE: number;                // Root Mean Square Error
    bias: number;
    correlation: number;
    skillScore: number;          // Murphy skill score
  };
}

interface ModelValidation {
  type: "hindcast" | "forecast" | "cross_validation";
  period: { start: Date; end: Date };

  metrics: {
    variable: string;
    observations: number[];
    predictions: number[];
    RMSE: number;
    bias: number;
    correlation: number;
    percentBias: number;
  }[];

  skillAssessment: {
    overall: "excellent" | "good" | "fair" | "poor";
    byVariable: Record<string, string>;
    confidence: number;
  };
}

class EcosystemModelValidator {
  validateModel(
    observations: number[],
    predictions: number[]
  ): {
    RMSE: number;
    bias: number;
    correlation: number;
    skillScore: number;
  } {
    const n = observations.length;

    // Root Mean Square Error
    const squaredErrors = observations.map((obs, i) =>
      Math.pow(obs - predictions[i], 2)
    );
    const RMSE = Math.sqrt(squaredErrors.reduce((a, b) => a + b) / n);

    // Bias (mean error)
    const errors = observations.map((obs, i) => predictions[i] - obs);
    const bias = errors.reduce((a, b) => a + b) / n;

    // Correlation
    const correlation = this.calculateCorrelation(observations, predictions);

    // Murphy Skill Score (1 = perfect, 0 = no skill, <0 = worse than climatology)
    const obsVariance = this.calculateVariance(observations);
    const skillScore = 1 - (RMSE * RMSE) / obsVariance;

    return { RMSE, bias, correlation, skillScore };
  }

  private calculateCorrelation(x: number[], y: number[]): number {
    const n = x.length;
    const meanX = x.reduce((a, b) => a + b) / n;
    const meanY = y.reduce((a, b) => a + b) / n;

    let numerator = 0;
    let denomX = 0;
    let denomY = 0;

    for (let i = 0; i < n; i++) {
      const dx = x[i] - meanX;
      const dy = y[i] - meanY;
      numerator += dx * dy;
      denomX += dx * dx;
      denomY += dy * dy;
    }

    return numerator / Math.sqrt(denomX * denomY);
  }

  private calculateVariance(values: number[]): number {
    const mean = values.reduce((a, b) => a + b) / values.length;
    return values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;
  }
}
```

### Applications of Ecosystem Models

#### Fisheries Management

```typescript
interface FisheriesManagementModel {
  stockAssessment: {
    species: string;
    stockBoundary: string;
    currentBiomass: number;      // Tonnes
    spawningBiomass: number;
    recruitmentRate: number;
    naturalMortality: number;
    fishingMortality: number;
  };

  referencePoints: {
    MSY: number;                 // Maximum Sustainable Yield (tonnes/year)
    Bmsy: number;                // Biomass at MSY
    Fmsy: number;                // Fishing mortality at MSY
    B_limit: number;             // Limit reference point (collapse threshold)
  };

  projection: {
    scenarios: {
      name: string;
      fishingRate: number;
      projectedBiomass: number[];    // By year
      projectedCatch: number[];
      probability_B_above_Bmsy: number;
    }[];

    recommendation: string;
  };
}

// Example: Pacific bluefin tuna
const bluefinManagement: FisheriesManagementModel = {
  stockAssessment: {
    species: "Thunnus orientalis",
    stockBoundary: "North Pacific",
    currentBiomass: 32000,       // Tonnes
    spawningBiomass: 18000,
    recruitmentRate: 0.8,
    naturalMortality: 0.15,
    fishingMortality: 0.45
  },

  referencePoints: {
    MSY: 15000,                  // Tonnes/year
    Bmsy: 50000,
    Fmsy: 0.30,
    B_limit: 20000
  },

  projection: {
    scenarios: [
      {
        name: "Current fishing",
        fishingRate: 0.45,
        projectedBiomass: [32000, 30000, 28000, 26000, 24000],
        projectedCatch: [14000, 13000, 12000, 11000, 10000],
        probability_B_above_Bmsy: 0.05
      },
      {
        name: "Reduced fishing (F=0.3)",
        fishingRate: 0.30,
        projectedBiomass: [32000, 35000, 39000, 43000, 47000],
        projectedCatch: [12000, 13000, 14000, 15000, 16000],
        probability_B_above_Bmsy: 0.65
      },
      {
        name: "Conservation (F=0.1)",
        fishingRate: 0.10,
        projectedBiomass: [32000, 40000, 52000, 65000, 75000],
        projectedCatch: [4000, 5000, 6500, 8000, 9000],
        probability_B_above_Bmsy: 0.95
      }
    ],

    recommendation: "Reduce fishing mortality to F=0.3 to rebuild stock to Bmsy by 2030"
  }
};
```

#### Climate Change Impact Assessment

Ecosystem models predict how warming and acidification affect marine life:

```typescript
interface ClimateImpactProjection {
  scenario: "SSP1-2.6" | "SSP2-4.5" | "SSP3-7.0" | "SSP5-8.5";  // IPCC scenarios
  timeHorizon: number;           // Years into future

  environmentalChanges: {
    temperature: {
      current: number;
      projected: number;
      uncertainty: number;
    };
    pH: {
      current: number;
      projected: number;
      uncertainty: number;
    };
    oxygen: {
      current: number;
      projected: number;
      percentChange: number;
    };
  };

  ecosystemResponses: {
    primaryProduction: {
      currentRate: number;       // gC/m²/year
      projectedRate: number;
      percentChange: number;
    };

    speciesShifts: {
      species: string;
      currentRange: string;
      projectedRange: string;
      polewardShift: number;     // Kilometers
    }[];

    phenologyChanges: {
      event: string;             // "spring_bloom", "spawning", "migration"
      currentTiming: number;     // Day of year
      projectedTiming: number;
      shift: number;             // Days earlier/later
    }[];
  };

  vulnerabilityAssessment: {
    species: string;
    exposureScore: number;       // 0-1 (environmental change magnitude)
    sensitivityScore: number;    // 0-1 (physiological tolerance)
    adaptiveCapacity: number;    // 0-1 (ability to shift/adapt)
    overallVulnerability: "low" | "moderate" | "high" | "very_high";
  }[];
}
```

### Best Practices for Ecosystem Modeling

**1. Model Selection**
Choose appropriate complexity for your question. Simple models for understanding, complex for prediction.

**2. Parameter Uncertainty**
Always quantify and propagate parameter uncertainties through model outputs.

**3. Multiple Data Sources**
Calibrate with diverse observations: satellite, in-situ, surveys, experiments.

**4. Validation**
Test on independent data not used in calibration.

**5. Ensemble Approaches**
Run multiple models to capture structural uncertainty.

**6. Transparency**
Document all assumptions, equations, parameters, and data sources.

**7. Open Source**
Share model code and data to enable reproducibility and improvement.

### Philosophy: Models as Shared Understanding

Ecosystem models embody our collective understanding of marine ecosystems. They integrate thousands of observations by hundreds of scientists into coherent frameworks. Following 弘益人間, we share models openly to benefit all humanity - enabling better fisheries management, climate adaptation, and conservation worldwide.

---

**Next Chapter:** Data Sharing and Interoperability - repositories, APIs, and standards that make marine biology data accessible to all.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
