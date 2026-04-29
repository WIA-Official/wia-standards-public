# Chapter 2: Fisheries and Aquaculture

## Feeding Humanity from the Ocean

**3.3 billion people** depend on seafood for **20% of their animal protein**. Global fisheries and aquaculture produce **178 million tons** of seafood annually, valued at **$401 billion**. This sector supports **60 million livelihoods** directly and hundreds of millions indirectly.

Yet we face a critical paradox: while demand for seafood grows **2.5% annually**, **35.4% of wild fish stocks** are overfished, and many ecosystems are collapsing under the pressure of industrial-scale extraction.

### The Global Fisheries Landscape

#### Wild Capture Fisheries

Marine capture fisheries harvested **84.4 million tons** in 2024:

**By Ocean Region:**
- Northwest Pacific: **21.9 million tons** (26% of total)
- Southeast Pacific: **11.2 million tons** (13%)
- Western Central Pacific: **13.4 million tons** (16%)
- Northeast Atlantic: **8.9 million tons** (11%)
- Eastern Indian Ocean: **7.3 million tons** (9%)

**Top Fishing Nations (2024):**
```typescript
interface FishingNation {
  country: string;
  marineCapture: number;              // Million tons
  aquaculture: number;
  totalProduction: number;
  numberOfVessels: number;
  employmentFisheries: number;
}

const topFishingNations: FishingNation[] = [
  {
    country: "China",
    marineCapture: 13.2,
    aquaculture: 49.6,
    totalProduction: 62.8,
    numberOfVessels: 694000,
    employmentFisheries: 14000000
  },
  {
    country: "Indonesia",
    marineCapture: 7.2,
    aquaculture: 16.6,
    totalProduction: 23.8,
    numberOfVessels: 280000,
    employmentFisheries: 7000000
  },
  {
    country: "Peru",
    marineCapture: 7.0,
    aquaculture: 0.1,
    totalProduction: 7.1,
    numberOfVessels: 14500,
    employmentFisheries: 180000
  },
  {
    country: "India",
    marineCapture: 3.6,
    aquaculture: 8.0,
    totalProduction: 11.6,
    numberOfVessels: 267000,
    employmentFisheries: 16000000
  }
];
```

#### Stock Status Assessment

Fish stock health is categorized by exploitation level:

```typescript
interface StockStatus {
  category: "underfished" | "maximally_fished" | "overfished";
  percentage: number;
  biomass: BiomassMetrics;
  fishing Mortality: number;
  trend: "improving" | "stable" | "declining";
}

interface GlobalStockAssessment {
  totalAssessedStocks: number;
  year: number;

  status: {
    biologicallySustainable: {
      percentage: 64.6,
      description: "Stocks fished within biologically sustainable levels"
    },
    maximallyFished: {
      percentage: 57.3,
      description: "Stocks at or near maximum sustainable yield"
    },
    overfished: {
      percentage: 35.4,
      description: "Stocks fished beyond biological sustainability"
    }
  };

  regionalVariation: {
    mediterranean: { overfished: 62.5 },
    northeastAtlantic: { overfished: 27.8 },
    northwestPacific: { overfished: 24.1 },
    southeastPacific: { overfished: 47.3 }
  };
}
```

### Fisheries Data Model

WIA-OCEAN-010 defines comprehensive fisheries data structures:

```typescript
interface Fishery {
  fisheryId: string;
  name: string;
  location: FisheryArea;
  targetSpecies: TargetSpecies[];
  managementBody: string[];
  status: StockStatus;
  quotas: QuotaSystem;
  fleet: FleetComposition;
  landings: LandingsData;
  economics: EconomicData;
  sustainability: SustainabilityAssessment;
}

interface FisheryArea {
  faoArea: string;                    // FAO Major Fishing Area code
  coordinates: GeoJSON.MultiPolygon;
  jurisdiction: string[];
  depth: { min: number; max: number };
  habitat: string[];                  // "continental_shelf", "seamount", "upwelling"
  marineProtectedAreas: string[];
}

interface TargetSpecies {
  scientificName: string;
  commonName: string;
  aphiaID: number;                    // WoRMS identifier
  stockUnit: string;                  // "North Sea cod", "Pacific bluefin tuna"
  biomass: {
    current: number;                  // Tons
    historical: number;               // Virgin biomass
    bmsy: number;                     // Biomass at MSY
    blim: number;                     // Limit reference point
  };
  mortality: {
    fishing: number;                  // Fishing mortality rate
    fmsy: number;                     // F at MSY
    natural: number;                  // Natural mortality
  };
  recruitment: {
    annualRecruitment: number;        // Young fish entering stock
    spawningStockBiomass: number;
    recruitmentVariability: number;
  };
}
```

### Quota Management Systems

Sustainable fisheries require strict catch limits:

```typescript
interface QuotaSystem {
  quotaType: "TAC" | "ITQ" | "effort" | "size_limit";
  managementYear: number;
  totalAllowableChatch: number;       // Tons
  quotaAllocations: Allocation[];
  compliance: ComplianceMetrics;
}

interface Allocation {
  entity: string;                     // Country, company, or cooperative
  allocationType: "national" | "vessel" | "individual";
  quotaAmount: number;                // Tons or percentage
  utilized: number;
  remaining: number;
  tradeable: boolean;                 // For ITQ systems
  expiryDate: Date;
}

interface ComplianceMetrics {
  reportingRate: number;              // 0-1 scale
  quotaCompliance: number;            // Percentage within quota
  violations: Violation[];
  enforcement: {
    inspections: number;
    boardings: number;
    citations: number;
    fines: number;                    // USD
  };
}
```

#### Individual Transferable Quotas (ITQs)

ITQs create market-based fishery management:

```typescript
interface ITQProgram {
  fishery: string;
  implementationYear: number;
  totalQuota: number;
  numberOfQuotaHolders: number;
  quotaShares: QuotaShare[];
  tradeHistory: QuotaTrade[];

  outcomes: {
    stockRecovery: number;            // Percentage improvement
    fleetConsolidation: number;       // Vessels reduced
    profitability: number;            // Revenue per vessel increase
    bycatchReduction: number;
    compliance: number;
  };
}

interface QuotaShare {
  holderId: string;
  sharePercentage: number;            // Percentage of TAC
  annualQuota: number;                // Tons
  acquisitionDate: Date;
  acquisitionPrice: number;           // USD per share
  currentValue: number;
}

interface QuotaTrade {
  transactionId: string;
  date: Date;
  seller: string;
  buyer: string;
  quotaAmount: number;                // Tons or percentage
  price: number;                      // USD
  term: "annual" | "permanent";
  blockchain: string;                 // Transaction hash for verification
}
```

**Real Example: New Zealand ITQ System**

New Zealand's Quota Management System (QMS), established 1986:

```typescript
const newZealandQMS: ITQProgram = {
  fishery: "New Zealand EEZ",
  implementationYear: 1986,
  totalQuota: 634000,                 // Tons across all species
  numberOfQuotaHolders: 1500,

  outcomes: {
    stockRecovery: 65,                // 65% of stocks rebuilt
    fleetConsolidation: 40,           // 40% fewer vessels, more efficient
    profitability: 120,               // 120% increase in profit per vessel
    bycatchReduction: 35,
    compliance: 97
  }
};
```

### Vessel Monitoring and Tracking

Modern fisheries management requires real-time vessel oversight:

```typescript
interface VesselMonitoringSystem {
  vessel: FishingVessel;
  vms: VMSData;                       // Vessel Monitoring System
  ais: AISData;                       // Automatic Identification System
  observers: ObserverProgram;
  electronicMonitoring: EMSystem;
  logbook: ElectronicLogbook;
}

interface FishingVessel {
  vesselId: string;
  imo: string;                        // International Maritime Org number
  name: string;
  flagState: string;
  vesselType: "trawler" | "longliner" | "purse_seiner" | "gillnetter" | "trap";
  length: number;                     // Meters
  tonnage: number;                    // Gross registered tons
  enginePower: number;                // Kilowatts
  crew: number;
  licenses: License[];
  ownership: string;
}

interface VMSData {
  timestamp: Date;
  position: {
    latitude: number;
    longitude: number;
    accuracy: number;
  };
  speed: number;                      // Knots
  heading: number;                    // Degrees
  activity: "steaming" | "fishing" | "drifting" | "in_port";
  transmissionInterval: number;       // Minutes
  zone: string;                       // Current fishing zone
  authorized: boolean;
}

interface ElectronicMonitoring {
  cameras: {
    count: number;
    locations: string[];              // "deck", "hauling", "discard_chute"
    resolution: string;
    recording: boolean;
  };
  sensors: {
    winchSensor: boolean;             // Detects gear deployment
    hydraulicPressure: boolean;       // Indicates fishing activity
    flowMeter: boolean;               // Measures catch volume
  };
  aiAnalysis: {
    speciesIdentification: boolean;
    catchEstimation: boolean;
    bycatchDetection: boolean;
    discardMonitoring: boolean;
  };
}
```

#### AI-Powered Catch Analysis

Machine learning analyzes electronic monitoring data:

```typescript
interface CatchAnalysisAI {
  model: "cnn_species_classifier" | "yolo_fish_detector";
  trainingData: {
    images: number;
    species: number;
    accuracy: number;                 // Validation accuracy
  };

  realTimeAnalysis: {
    videoStream: string;
    framesPerSecond: number;
    detectedSpecies: DetectedFish[];
    catchEstimate: CatchEstimate;
    violations: AutoDetectedViolation[];
  };
}

interface DetectedFish {
  species: string;
  confidence: number;                 // 0-1
  size: number;                       // Centimeters
  legal: boolean;                     // Above minimum size
  quota: boolean;                     // Quota available
  timestamp: Date;
  frameId: string;
}

interface CatchEstimate {
  species: string;
  count: number;
  weight: number;                     // Kilograms
  confidence: number;
  discarded: number;                  // Bycatch discarded
  retained: number;
}
```

### Aquaculture: Farming the Sea

Aquaculture now produces **52% of seafood** consumed globally, growing **5% annually**:

```typescript
interface AquacultureOperation {
  farmId: string;
  name: string;
  location: GeographicCoordinate;
  type: "marine_cage" | "land_based" | "integrated" | "offshore";
  species: CulturedSpecies[];
  production: ProductionData;
  environmental: EnvironmentalMonitoring;
  certification: Certification[];
  economics: EconomicData;
}

interface CulturedSpecies {
  scientificName: string;
  commonName: string;
  productionCycle: {
    hatchery: number;                 // Days
    nursery: number;
    growOut: number;
    totalTime: number;
    mortalityRate: number;            // Percentage
  };
  stocking: {
    density: number;                  // Fish per m³
    totalStocked: number;
    stockingDate: Date;
  };
  feeding: {
    feedType: string;                 // "pellet", "natural"
    feedConversionRatio: number;      // Feed kg per growth kg
    dailyFeedRate: number;            // Percentage of biomass
    feedSource: string[];             // Fishmeal, soy, insects
  };
  health: {
    diseaseHistory: Disease[];
    vaccinationProgram: string[];
    antibioticUse: number;            // Grams per ton of production
    parasiticLoadd: number;
  };
}

interface ProductionData {
  annualProduction: number;           // Tons
  harvestSchedule: Harvest[];
  qualityMetrics: {
    averageWeight: number;            // Grams
    uniformity: number;               // Coefficient of variation
    fatContent: number;               // Percentage
    omega3Content: number;            // Grams per 100g
    contaminants: ContaminantLevel[];
  };
}
```

#### Environmental Monitoring for Aquaculture

Intensive monitoring ensures water quality and minimizes impact:

```typescript
interface AquacultureEnvironment {
  waterQuality: {
    temperature: TimeSeries;
    dissolvedOxygen: TimeSeries;
    salinity: TimeSeries;
    pH: TimeSeries;
    ammonia: TimeSeries;
    nitrite: TimeSeries;
    turbidity: TimeSeries;
  };

  effluent: {
    nitrogenDischarge: number;        // Kg per ton of production
    phosphorusDischarge: number;
    organicMatter: number;
    treatment: string[];              // "settling", "biofilter", "constructed_wetland"
  };

  ecosystem: {
    benthicImpact: {
      sulfidesLevel: number;          // Millimolar
      redoxPotential: number;
      macrofaunaChange: number;       // Percentage change
      recoveryDistance: number;       // Meters from cages
    };
    wildInteraction: {
      escapes: number;                // Number per year
      predatorInteraction: string[];  // Seals, sea lions, birds
      diseaseTransmission: boolean;
    };
  };

  sensors: Sensor[];
  alertThresholds: {
    parameter: string;
    warning: number;
    critical: number;
    action: string;
  }[];
}
```

### Sustainable Aquaculture Systems

Innovation drives sustainability:

#### Recirculating Aquaculture Systems (RAS)

```typescript
interface RASFacility {
  type: "land_based_ras";
  waterRecirculation: number;         // Percentage (95-99%)

  biofilter: {
    type: "moving_bed" | "fluidized" | "trickling";
    volume: number;                   // Cubic meters
    nitrificationRate: number;        // Grams N per m³ per day
    efficiency: number;
  };

  oxygenation: {
    system: "pure_oxygen" | "air_diffusion";
    oxygenEfficiency: number;         // Percentage
    targetDO: number;                 // mg/L
  };

  temperatureControl: {
    heatingSource: string;
    coolingSource: string;
    targetTemperature: number;        // Celsius
    energyUse: number;                // kWh per kg production
  };

  advantages: [
    "Zero discharge to environment",
    "Disease control",
    "Precise environmental control",
    "Urban location possible",
    "Year-round production"
  ];

  challenges: [
    "High capital cost",
    "High energy use",
    "Technical complexity",
    "Biofilter stability"
  ];
}
```

#### Integrated Multi-Trophic Aquaculture (IMTA)

Nature-inspired farming combining species at different trophic levels:

```typescript
interface IMTASystem {
  type: "integrated_multi_trophic";

  species: {
    fedSpecies: {                     // Receive feed input
      species: "Atlantic salmon";
      production: 1000;               // Tons per year
      wasteOutput: {
        dissolvedNitrogen: 48;        // Tons per year
        dissolvedPhosphorus: 7;
        particulate Organic: 350;
      };
    };

    extractiveSpecies: [
      {
        type: "inorganic";
        species: "Sugar kelp";        // Absorbs dissolved nutrients
        production: 200;              // Tons per year
        nutrientUptake: {
          nitrogen: 24;               // Tons removed
          phosphorus: 3;
        };
      },
      {
        type: "organic";
        species: "Blue mussel";       // Filters particulate matter
        production: 150;
        organicRemoval: 175;          // Tons particulate removed
      },
      {
        type: "organic";
        species: "Sea cucumber";      // Consumes settled waste
        production: 50;
        organicRemoval: 100;
      }
    ];
  };

  benefits: {
    wasteReduction: 70,               // Percentage
    additionalRevenue: 400000,        // USD from extractive species
    environmentalImpact: -60,         // Percentage reduction
    ecosystem Services: "carbon_sequestration, habitat, biodiversity"
  };
}
```

### Bycatch Reduction

Bycatch - unintended catch - kills **millions of tons** of marine life annually:

```typescript
interface BycatchReduction {
  fishery: string;
  bycatchSpecies: BycatchSpecies[];
  reductionDevices: ReductionDevice[];
  effectiveness: EffectivenessMetrics;
}

interface BycatchSpecies {
  species: string;
  conservation Status: "endangered" | "threatened" | "vulnerable" | "least_concern";
  estimatedBycatch: number;           // Individuals per year
  mortality: number;                  // Percentage mortality
  regulatoryLimit: number;            // Maximum allowed bycatch
}

interface ReductionDevice {
  name: string;
  description: string;
  targetBycatch: string[];
  effectiveness: number;              // Percentage reduction
  targetCatchImpact: number;          // Percentage reduction in target catch
  cost: number;                       // USD per installation

  examples: {
    turtleExcluderDevice: {
      effectiveness: 97,              // 97% reduction in sea turtle bycatch
      shrimploss: 2,                  // 2% reduction in shrimp catch
      mandatory: ["US Gulf of Mexico", "Australia"]
    };

    circleHooks: {
      effectiveness: 89,              // 89% reduction in sea turtle hooking
      targetReduction: 0,
      fishery: "pelagic_longline"
    };

    ledLights: {
      type: "green_LED_net_illumination";
      effectiveness: 63,              // 63% reduction in sea turtle bycatch
      targetReduction: 0,
      mechanism: "Sea turtles avoid green light"
    };
  };
}
```

### Fisheries Certification and Sustainability

Market-based certification drives sustainable practices:

```typescript
interface SustainabilityCertification {
  standard: "MSC" | "ASC" | "BAP" | "Friend_of_the_Sea";
  certifiedSince: Date;
  validUntil: Date;

  assessmentCriteria: {
    stockStatus: {
      required: "Healthy stock levels above MSY";
      score: number;                  // 0-100
    };
    ecosystemImpact: {
      required: "Minimal habitat damage and bycatch";
      score: number;
    };
    management: {
      required: "Effective management system";
      score: number;
    };
  };

  chainOfCustody: CustodyRecord[];
  marketPremium: number;              // Percentage price premium
}

// Marine Stewardship Council (MSC) - 500+ certified fisheries
interface MSCCertification {
  principle1: "Sustainable fish stocks";
  principle2: "Minimizing environmental impact";
  principle3: "Effective fisheries management";

  globalImpact: {
    certifiedFisheries: 500,
    certifiedCatch: 15000000,         // Tons
    percentageGlobalCatch: 18,
    countriesParticipating: 36
  };
}
```

### Climate Change and Fisheries

Climate change fundamentally alters fisheries:

```typescript
interface ClimateImpact {
  temperatureChange: {
    sst Increase: number;             // Celsius since 1900
    projected2050: number;
    projected2100: number;
  };

  speciesShift: {
    polewardMigration: number;        // Km per decade
    depthChange: number;              // Meters deeper
    affectedSpecies: number;          // Percentage of species
  };

  productivity: {
    tropicalReduction: -40,           // Percentage reduction by 2100
    polarIncrease: 30,                // Percentage increase
    globalChange: -12;                // Net change
  };

  adaptationStrategies: [
    "Dynamic ocean management",
    "Flexible quota allocation",
    "Range-shift aware management",
    "Ecosystem-based adaptation",
    "Climate-ready infrastructure"
  ];
}
```

### Philosophy: 弘益人間 in Fisheries

The principle of 弘益人間 - benefiting all humanity - requires:

**Intergenerational Equity:** Preserve fish stocks for our children
**Global Cooperation:** Shared stocks need collaborative management
**Ecosystem Respect:** Fish are part of webs of life
**Livelihood Protection:** 60 million people depend on fisheries
**Food Security:** 3.3 billion people depend on seafood protein

Sustainable fisheries mean:
- **Fishing at or below MSY** for all stocks
- **Eliminating illegal fishing** that costs $23 billion/year
- **Rebuilding depleted stocks** to historical abundance
- **Protecting critical habitats** that support fisheries
- **Adapting to climate change** proactively

---

**Next Chapter:** We'll explore offshore oil and gas resources, examining how to extract energy while protecting marine environments.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
