# Chapter 6: Desalination and Water Resources

## Freshwater from the Sea

**2.2 billion people** lack access to safely managed drinking water, and **4 billion** experience severe water scarcity at least one month per year. As climate change intensifies droughts and populations grow, **ocean desalination** has become critical infrastructure, producing **95 million m³ of freshwater daily** from **21,000+ plants** worldwide.

Yet desalination is **energy-intensive** (2-4 kWh per cubic meter), produces **concentrated brine** requiring disposal, and affects marine ecosystems through intake and discharge. Sustainable desalination requires balancing human water needs with ocean health.

### Global Desalination Landscape

```typescript
interface GlobalDesalination {
  year: 2024;

  capacity: {
    global: 95000000,                 // Cubic meters per day
    plants: 21000,
    growth Rate: 9,                    // Percentage per year
    projectionsfor2030: 160000000      // m³/day
  };

  technology: {
    reverseOsmosis: {
      percentage: 69,
      capacity: 65000000,             // m³/day
      energyUse: 3,                   // kWh per m³
      dominance: "Seawater and brackish water"
    };
    multiStageFlash: {
      percentage: 20,
      capacity: 19000000,
      energyUse: 15,                  // kWh per m³ (thermal)
      dominance: "Middle East with waste heat"
    };
    multiEffectDistillation: {
      percentage: 8,
      energyUse: 10
    };
    other: {
      percentage: 3,
      technologies: ["Electrodialysis", "Nanofiltration", "Forward osmosis"]
    };
  };

  regions: {
    middleEast: {
      capacity: 42000000,             // m³/day (44% of global)
      countries: {
        saudiArabia: 11000000,        // 70% of drinking water from desalination
        uae: 9500000,                 // 42% of global desalination capacity in Gulf
        kuwait: 3200000,
        qatar: 2500000
      };
      driver: "Arid climate, no surface water, low energy cost";
    };

    eastAsia: {
      capacity: 12000000,
      china: 6500000,
      japan: 1800000,
      driver: "Urban water scarcity, islands";
    };

    northAmerica: {
      capacity: 10000000,
      usa: 7500000,                   // California, Florida, Texas
      driver: "Drought, groundwater depletion";
    };

    europe: {
      capacity: 7500000,
      spain: 3500000,                 // Largest in Europe
      driver: "Mediterranean drought, tourism";
    };

    australia: {
      capacity: 3000000,
      driver: "Millennium drought, climate adaptation";
    };
  };

  applications: {
    municipalWater: 65,               // Percentage
    industrial: 25,
    agriculture: 8,
    other: 2
  };

  economics: {
    capitalCost: 1500,                // USD per m³/day capacity (RO)
    waterCost: 0.75,                  // USD per m³ (RO seawater)
    energyCost: 50,                   // Percentage of operating cost
    totalMarketValue: 18000000000     // USD per year
  };
}
```

### Desalination Technologies

#### Reverse Osmosis (RO)

The dominant technology, using pressure to force water through membranes:

```typescript
interface ReverseOsmosisPlant {
  plantId: string;
  name: string;
  location: GeographicCoordinate;
  capacity: number;                   // Cubic meters per day
  status: "operational" | "construction" | "planning";

  intake: {
    type: "open_ocean" | "beach_well" | "subsurface";
    depth: number;                    // Meters
    flow Rate: number;                 // m³/day
    pretreatment: Pretreatment;
  };

  roProcess: {
    stages: number;                   // 1 or 2 stage
    membranes: {
      type: "spiral_wound" | "hollow_fiber";
      material: "Polyamide thin-film composite";
      elements: number;
      area: number;                   // Square meters per element
      lifespan: number;               // Years (5-7 typical)
    };

    operating Parameters: {
      pressure: number;               // Bar (55-70 bar for seawater)
      temperature: number;            // Celsius (optimal 25°C)
      feedWaterTDS: number;           // Total dissolved solids mg/L (35000)
      permeateQuality: number;        // TDS mg/L (<500)
      recovery: number;               // Percentage (35-50% seawater)
    };

    energyRecovery: {
      device: "pressure_exchanger" | "turbocharger" | "pelton_wheel";
      efficiency: number;             // Percentage (95%+ modern)
      energySavings: number;          // kWh per m³ saved
    };
  };

  postTreatment: {
    remineralization: {
      reason: "RO water too pure, corrosive";
      additions: string[];            // "Calcite", "limestone", "CO2"
      targetHardness: number;         // mg/L CaCO3
      targetpH: number;               // 7.5-8.5
    };
    disinfection: string;             // "Chlorine", "UV", "ozone"
    blending: {
      percentage: number;             // Blend with source water
      reason: "Reduce energy, improve quality";
    };
  };

  energy: {
    specificConsumption: number;      // kWh per m³ (2.5-3.5 modern)
    totalConsumption: number;         // Megawatt-hours per day
    powerSource: PowerSource[];
    renewable Integration: RenewableEnergy;
  };

  brine: {
    salinity: number;                 // TDS mg/L (60000-70000)
    volume: number;                   // m³/day
    temperature: number;              // Celsius
    chemicals: Chemical[];
    discharge: BrineDisposal;
  };

  monitoring: {
    permeateQuality: Continuous;
    pressures: Continuous;
    flows: Continuous;
    membranePerformance: {
      normalizedFlux: number;
      saltRejection: number;          // Percentage
      pressureDrop: number;
      cleaningFrequency: number;      // Times per year
    };
  };
}

interface Pretreatment {
  screening: {
    barScreen: number;                // Millimeters (10mm)
    microscreen: number;              // Microns (100 µm)
  };

  coagulationFlocculation: {
    coagulant: string;                // "Ferric chloride", "aluminum sulfate"
    flocculant: string;               // "Polymer"
    dose: number;                     // mg/L
  };

  filtration: {
    mediaFilter: boolean;             // Sand, anthracite
    ultrafiltration: {
      poreSize: number;               // Nanometers (10-100 nm)
      flux: number;                   // Liters per m² per hour
      backwash: number;               // Minutes per cycle
    };
  };

  antiscalant: {
    chemical: string;                 // "Phosphonates", "polycarboxylates"
    dose: number;                     // mg/L (2-5)
    purpose: "Prevent calcium, magnesium precipitation on membranes";
  };
}
```

**State-of-the-Art RO Plant Example:**

```typescript
const sorekDesalination = {
  name: "Sorek Desalination Plant",
  location: "Israel",
  capacity: 624000,                   // m³/day - world's largest RO (2024)
  operational: 2013,

  performance: {
    energyConsumption: 3.0,           // kWh per m³ (world-leading efficiency)
    waterCost: 0.52,                  // USD per m³
    recovery: 50,                     // Percentage
    availability: 97                  // Percentage
  };

  technology: {
    membranes: 50000,                 // 16-inch diameter spiral wound
    pressureVessels: 5000,
    energyRecovery: "isobaric pressure exchangers",
    automation: "Advanced process control"
  };

  impact: {
    israelWaterSupply: 20,            // Percentage of national supply
    populationServed: 1500000
  };

  innovation: [
    "Economies of scale reduce cost",
    "High recovery rate",
    "Low energy through optimization",
    "Minimal chemical use"
  ]
};
```

### Seawater Intake Technologies

Intake design critically affects marine life:

```typescript
interface SeawaterIntake {
  type: IntakeType;
  location: GeographicCoordinate;
  depth: number;
  capacity: number;                   // m³/day

  environmentalConcerns: {
    impingement: {
      definition: "Organisms trapped on intake screens";
      affected: string[];             // "Fish", "jellyfish", "crabs"
      mortality: number;              // Percentage
      mitigation: string[];
    };

    entrainment: {
      definition: "Small organisms drawn into system";
      affected: string[];             // "Eggs", "larvae", "plankton"
      mortalityRate: 100,             // Percentage - thermal/pressure death
      volumeAffected: number;         // m³/day
      impactScale: string;
    };
  };
}

enum IntakeType {
  OPEN_OCEAN = "open_ocean_surface",
  SUBSURFACE = "subsurface_beach_well",
  OFFSHORE = "offshore_submerged"
}

interface OpenOceanIntake {
  type: "open_ocean_surface";
  location: "Shoreline or near-shore";

  design: {
    screenVelocity: number;           // Meters per second (<0.5 recommended)
    meshSize: number;                 // Millimeters
    screening: "Passive" | "Active (e.g., fish return)";
  };

  impingementMortality: {
    fish: number;                     // Individuals per m³
    estimateAnnual: number;
    mitigations: [
      "Low velocity (<0.15 m/s)",
      "Fish return systems",
      "Acoustic deterrents",
      "Seasonal closures",
      "Offshore location"
    ];
  };

  entrainmentMortality: {
    larvaePerM3: number;
    annualMortality: number;          // Millions of larvae
    populationImpact: "Difficult to quantify";
    mitigations: [
      "Offshore intake reduces coastal larvae concentration",
      "Depth selection to avoid larvae layers"
    ];
  };

  advantages: [
    "Lower capital cost",
    "Easier maintenance",
    "Proven technology"
  ];

  disadvantages: [
    "High impingement/entrainment",
    "Vulnerable to storms, algal blooms",
    "Requires extensive pretreatment"
  ];
}

interface SubsurfaceIntake {
  type: "subsurface_beach_well" | "infiltration_gallery";
  location: "Below beach or seabed";

  design: {
    wells: {
      number: number;
      depth: number;                  // Meters below seabed (5-30)
      diameter: number;
      spacing: number;
    };
    flowPath: "Seawater filters through sand/sediment before collection";
  };

  advantages: [
    "Zero impingement/entrainment (sand filters out organisms)",
    "Natural pretreatment (removes particles, organics)",
    "Protected from storms",
    "Better water quality for RO"
  ];

  disadvantages: [
    "Site-specific feasibility (requires permeable geology)",
    "Higher capital cost (20-30% more)",
    "Limited capacity (drawdown constraints)",
    "Potential groundwater impacts"
  ];

  environmental: {
    impingement: 0,
    entrainment: 0,
    benefit: "Preferred option for marine life protection";
  };

  examples: {
    perth: {
      location: "Australia",
      capacity: 144000,               // m³/day
      wells: 3,
      depth: 7                        // Meters
    };
    fukuoka: {
      location: "Japan",
      capacity: 50000,
      type: "Intake under seafloor"
    };
  };
}
```

### Brine Disposal

Concentrated brine requires careful management:

```typescript
interface BrineDisposal {
  volume: number;                     // m³/day (1.5-2x product water)
  salinity: number;                   // PSU (60-80 vs seawater 35)
  temperature: number;                // Celsius (5-10°C above ambient if thermal desalination)

  composition: {
    salts: {
      sodium Chloride: number;         // mg/L
      magnesium: number;
      calcium: number;
      sulfate: number;
    };

    chemicals: {
      antiscalants: number;           // mg/L
      coagulants: number;
      chlorine: number;
      cleaningChemicals: number;
      heavyMetals: {
        copper: number;               // From corrosion
        nickel: number;
      };
    };
  };

  disposalMethod: {
    surfaceDischarge: SurfaceDischarge;
    deepWellInjection?: DeepWell;
    evaporationPond?: EvaporationPond;
    zincrementallyBlended?: BlendedDischarge;
  };

  environmentalImpact: EnvironmentalImpact;
  monitoring: MonitoringProgram;
}

interface SurfaceDischarge {
  method: "surface_discharge_to_ocean";
  percentage: 80,                     // 80% of desalination plants

  location: {
    outfallType: "pipe" | "diffuser" | "co-location";
    distance: number;                 // Meters from shore
    depth: number;                    // Meters
    currentVelocity: number;          // m/s (higher = better dilution)
  };

  diffuser: {
    type: "multiport_diffuser";
    length: number;                   // Meters
    ports: number;
    spacing: number;                  // Meters
    jetVelocity: number;              // m/s
    dilution: number;                 // Ratio at edge of mixing zone (10-20x typical)
  };

  mixingZone: {
    areaAllowed: number;              // Square meters
    salinityLimit: number;            // PSU above ambient (typically +2 PSU at boundary)
    dimensions: {
      length: number;
      width: number;
    };
    complianceMonitoring: boolean;
  };

  impact: {
    benthicCommunity: {
      hypersalinity: "Stress at 40+ PSU";
      mortality: "At 50+ PSU for sensitive species";
      recoveryDistance: number;       // Meters from outfall
    };

    waterColumn: {
      stratification: "Dense brine sinks, reduces mixing";
      oxygen: "Can create hypoxic layer if stratified";
      phytoplankton: "Reduced growth at high salinity";
    };

    seagrass: {
      sensitivity: "High - 5 PSU increase can cause mortality";
      impactRadius: number;           // Meters
      mitigation: "Site outfall away from seagrass";
    };
  };

  coLocation: {
    concept: "Discharge with power plant cooling water";
    advantage: "Large dilution flow, existing outfall";
    percentage: 35,                   // Percentage of plants co-located
    example: "Many Middle East plants co-located with oil/gas infrastructure";
  };
}

interface BeneficialUse {
  concept: "Use brine productively instead of disposal";

  applications: {
    saltProduction: {
      process: "Evaporate brine to harvest salt";
      products: ["Table salt", "industrial salt", "magnesium", "gypsum"];
      economics: "Marginally viable at large scale";
    };

    aquaculture: {
      species: string[];              // "Brine shrimp", "halophilic algae"
      feasibility: "Experimental";
      challenge: "Chemical contamination from antiscalants";
    };

    salinityGradientPower: {
      technology: "Pressure-retarded osmosis or reverse electrodialysis";
      potential: number;              // kW
      status: "Pilot stage";
      concept: "Generate power from salinity difference";
    };
  };

  challenges: [
    "Economics usually unfavorable",
    "Scale mismatch (brine volume too large)",
    "Chemical contamination limits reuse",
    "Energy for further processing"
  ];
}
```

### Energy and Renewable Integration

Reducing desalination's energy footprint:

```typescript
interface DesalinationEnergy {
  energyIntensity: {
    reverseOsmosis: {
      current: 3.0,                   // kWh per m³ (2025 best practice)
      historical: 6.0,                // kWh per m³ (1990s)
      theoretical Minimum: 1.06,       // kWh per m³ (thermodynamic limit)
      target2030: 2.0                 // Achievable with innovation
    };

    thermalDesalination: {
      mSF: 15,                        // kWh per m³ (mostly thermal energy)
      mED: 10
    };
  };

  renewableIntegration: {
    solar: {
      pVDirectCoupled: {
        concept: "Solar PV directly powers RO (no grid)";
        challenge: "Intermittency - RO operates 24/7";
        solution: "Battery storage or grid hybrid";
      };

      solarPVWithStorage: {
        capacity: number;             // MW solar
        storage: number;              // MWh battery
        autonomy: number;             // Hours without sun
        economics: "Competitive in sunny regions with falling battery costs";
      };

      concentratedSolar: {
        type: "CSP with thermal storage";
        advantage: "Can drive thermal desalination + power RO";
        projects: "Several in Middle East";
      };
    };

    wind: {
      offshoreWindDesalination: {
        concept: "Co-locate desalination with offshore wind";
        advantage: "Abundant seawater, renewable power, land scarcity";
        challenge: "Offshore plant complexity, O&M cost";
        status: "Pilot projects";
      };

      onshoreWind: {
        feasibility: "High in coastal windy regions";
        example: "Australia, California projects";
      };
    };

    waveOsmotic: {
      wavePoweredRO: {
        concept: "Wave energy directly pressurizes RO";
        advantage: "Eliminates electric conversion losses";
        status: "Experimental prototypes";
      };
    };

    nuclearDesalination: {
      cogeneration: "Nuclear power plant waste heat drives desalination";
      scale: "Large (>100,000 m³/day)";
      examples: "Kazakhstan (Aktau), Japan, India planned";
      controversy: "Safety concerns vs carbon-free energy";
    };
  };

  energyRecoveryInnovation: {
    isobaricDevices: {
      type: "Pressure exchangers";
      efficiency: 97,                 // Percentage
      savings: 60,                    // Percentage of energy
      adoption: "Standard in new plants";
    };

    batchRO: {
      concept: "Operate RO in batches to reach higher recovery";
      benefit: "Reduces energy 10-20%";
      status: "Commercializing";
    };

    closedCircuitRO: {
      concept: "Recirculate concentrate to increase recovery";
      benefit: "Lower energy, less brine";
      challenge: "Scaling management";
    };
  };

  carbonFootprint: {
    gridPowered: {
      emissionsPerM3: number;         // Kg CO2 (depends on grid mix)
      global Average: 2.5,             // kg CO2 per m³
      coalHeavy: 5.0,
      renewableHeavy: 0.2
    };

    mitigation: [
      "Renewable energy integration",
      "Energy efficiency improvements",
      "Carbon offsets",
      "Green hydrogen for desalination"
    ];
  };
}
```

### Emerging Desalination Technologies

Innovation aims for lower energy and environmental impact:

```typescript
interface EmergingTechnologies {
  forwardOsmosis: {
    principle: "Use osmotic gradient instead of pressure";
    process: "Draw solution pulls water through membrane, then separate draw solute";
    advantages: [
      "Lower fouling",
      "Lower energy potential",
      "Works at low pressure"
    ];
    challenges: [
      "Draw solution regeneration energy",
      "Lower flux than RO",
      "Draw solute development"
    ];
    status: "Pilot scale";
  };

  membraneDistillation: {
    principle: "Heat-driven membrane process";
    advantage: "Can use waste heat, solar thermal";
    challenge: "Membrane wetting, lower flux";
    niche: "Small-scale, remote, waste heat available";
  };

  capacitiveDeionization: {
    principle: "Electrodes remove ions from water";
    advantage: "Low energy for brackish water";
    limitation: "Not suitable for high salinity seawater";
    application: "Brackish water desalination";
  };

  grapheneMembranes: {
    promise: "Nanoporous graphene filters salt, passes water";
    potential: "10x flux increase, lower energy";
    status: "Laboratory research";
    challenge: "Scaling up production, defect-free membranes";
  };

  biomimetic: {
    aquaporinMembranes: {
      concept: "Embed aquaporin proteins (natural water channels) in membrane";
      advantage: "Perfect selectivity, high flux";
      commercialization: "Aquaporin A/S (Denmark) producing";
      challenge: "Cost, stability";
    };
  };

  solarStills: {
    principle: "Solar heat evaporates water, condenses pure";
    scale: "Very small (liters/day)";
    use: "Off-grid, emergency, developing regions";
    cost: "Low capital, zero operating energy";
  };
}
```

### Philosophy: 弘益人間 and Water

The principle of 弘益人間 - benefiting all humanity - demands responsible desalination:

**Universal Right to Water:**
- **2.2 billion people** lack safe water
- Desalination can provide **drought-proof supply**
- Must be **affordable and accessible** to all

**Environmental Responsibility:**
- **Marine life protection** through subsurface intakes
- **Brine dilution** to safe levels
- **Renewable energy** to minimize carbon footprint
- **Careful siting** to avoid sensitive ecosystems

**Sustainable Management:**
- Desalination as **one tool** in portfolio (efficiency, conservation, reuse first)
- **Energy efficiency** to reduce cost and emissions
- **Technology innovation** to minimize impacts
- **Benefit sharing** - water for communities, not just profit

The ocean can help solve humanity's water crisis, but only if we extract freshwater responsibly.

---

**Next Chapter:** Environmental protection and sustainability - safeguarding ocean resources for future generations.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
