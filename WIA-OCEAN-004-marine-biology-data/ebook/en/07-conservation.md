# Chapter 7: Conservation Applications

## Using Data to Protect Marine Biodiversity

Marine biology data directly informs conservation decisions worldwide - from designating marine protected areas to assessing endangered species status to tracking illegal fishing. This chapter explores how standardized data enables effective ocean conservation following the principle of 弘益人間 (benefiting all humanity).

### IUCN Red List Assessments

The **International Union for Conservation of Nature (IUCN) Red List** assesses extinction risk for species worldwide based on standardized criteria and data:

```typescript
interface IUCNRedListAssessment {
  assessmentID: string;
  aphiaID: number;
  scientificName: string;
  commonName: string;

  category: "EX" | "EW" | "CR" | "EN" | "VU" | "NT" | "LC" | "DD" | "NE";
  /*
    EX = Extinct
    EW = Extinct in the Wild
    CR = Critically Endangered
    EN = Endangered
    VU = Vulnerable
    NT = Near Threatened
    LC = Least Concern
    DD = Data Deficient
    NE = Not Evaluated
  */

  criteria: string[];            // IUCN criteria codes (e.g., "A2cd+3cd", "B1ab(iii)")

  rationale: {
    populationTrend: "increasing" | "stable" | "decreasing" | "unknown";
    populationDecline: number;   // Percentage over time period
    timeFrame: number;           // Years or generations

    rangeSize: {
      EOO: number;               // Extent of Occurrence (km²)
      AOO: number;               // Area of Occupancy (km²)
      fragmentation: boolean;
      decline: boolean;
      fluctuations: boolean;
    };

    populationSize: {
      matureIndividuals: number;
      uncertainty: [number, number];  // Min, max estimate
      numberOfSubpopulations: number;
      largestSubpopulation: number;
    };

    threats: Threat[];
  };

  assessment: {
    assessor: string[];
    reviewer: string[];
    assessmentDate: Date;
    previousCategory?: string;
    previousAssessmentDate?: Date;
    reasonForChange?: "genuine" | "new_information" | "taxonomy" | "criteria_revision";
  };

  distribution: {
    countries: string[];         // ISO country codes
    marineRegions: string[];     // Marine regions present
    depthRange: { min: number; max: number };
    habitat: string[];
  };

  population: {
    populationTrend: string;
    justification: string;
  };

  habitat_ecology: {
    habitat: string;
    depth: string;
    systems: string[];           // "marine", "freshwater", "terrestrial"
    movementPatterns: string;
  };

  threats: Threat[];
  conservationActions: ConservationAction[];
  useTrade: string;
  dataQuality: {
    adequacy: "adequate" | "inadequate";
    documentation: string;
    needsUpdating: boolean;
  };
}

interface Threat {
  code: string;                  // IUCN Threats Classification Scheme
  timing: "ongoing" | "past" | "future" | "unknown";
  scope: "whole" | "majority" | "minority" | "unknown";
  severity: "very_rapid" | "rapid" | "slow" | "negligible" | "unknown";
  score: "high" | "medium" | "low" | "unknown";
  stresses: string[];
}

interface ConservationAction {
  code: string;                  // IUCN Conservation Actions Classification
  timing: "ongoing" | "needed" | "completed";
  scope: "whole" | "majority" | "minority";
  description: string;
}

// Example: Whale shark (Rhincodon typus)
const whaleSharkAssessment: IUCNRedListAssessment = {
  assessmentID: "IUCN-RL-19488-0",
  aphiaID: 105847,
  scientificName: "Rhincodon typus",
  commonName: "Whale Shark",

  category: "EN",                // Endangered
  criteria: ["A2bd+3d"],

  rationale: {
    populationTrend: "decreasing",
    populationDecline: 63,       // 63% decline over 75 years (3 generations)
    timeFrame: 75,

    rangeSize: {
      EOO: 100000000,            // Global distribution
      AOO: 50000,
      fragmentation: true,
      decline: true,
      fluctuations: false
    },

    populationSize: {
      matureIndividuals: 200000,
      uncertainty: [100000, 400000],
      numberOfSubpopulations: 12,
      largestSubpopulation: 50000
    },

    threats: [
      {
        code: "5.4.1",           // Fishing & harvesting - intentional use
        timing: "ongoing",
        scope: "majority",
        severity: "rapid",
        score: "high",
        stresses: ["2.1.2"]      // Species mortality
      },
      {
        code: "5.4.2",           // Fishing & harvesting - unintentional
        timing: "ongoing",
        scope: "majority",
        severity: "slow",
        score: "medium",
        stresses: ["2.1.2"]
      }
    ]
  },

  assessment: {
    assessor: ["Pierce, S.J.", "Norman, B."],
    reviewer: ["Brooks, E.", "Dulvy, N.K."],
    assessmentDate: new Date("2016-04-01"),
    previousCategory: "VU",
    previousAssessmentDate: new Date("2005-07-01"),
    reasonForChange: "genuine"
  },

  distribution: {
    countries: ["USA", "AUS", "MEX", "PHL", "THA", "MDV", /* many more */],
    marineRegions: ["Tropical Atlantic", "Tropical Pacific", "Indian Ocean"],
    depthRange: { min: 0, max: 1928 },
    habitat: ["pelagic", "coastal"]
  },

  population: {
    populationTrend: "decreasing",
    justification: "Suspected population reduction of at least 50% over the past three generations (75 years) based on direct observation, decline in AOO, and actual or potential levels of exploitation."
  },

  habitat_ecology: {
    habitat: "Pelagic and coastal waters, often near coral reefs, continental shelves",
    depth: "0-1928 m, typically 0-100 m",
    systems: ["marine"],
    movementPatterns: "Highly migratory, seasonal aggregations at specific sites"
  },

  threats: [
    {
      code: "5.4.1",
      timing: "ongoing",
      scope: "majority",
      severity: "rapid",
      score: "high",
      stresses: ["2.1.2"]
    },
    {
      code: "5.4.4",             // Bycatch
      timing: "ongoing",
      scope: "minority",
      severity: "slow",
      score: "medium",
      stresses: ["2.1.2"]
    },
    {
      code: "4.1",               // Shipping lanes
      timing: "ongoing",
      scope: "minority",
      severity: "slow",
      score: "low",
      stresses: ["2.1.2"]        // Ship strikes
    }
  ],

  conservationActions: [
    {
      code: "1.1",               // Site/area protection
      timing: "ongoing",
      scope: "minority",
      description: "Protected in several MPAs including Galapagos, Maldives, Philippines"
    },
    {
      code: "5.1.2",             // National legislation
      timing: "ongoing",
      scope: "majority",
      description: "Protected by law in >25 countries"
    },
    {
      code: "5.1.3",             // International legislation
      timing: "ongoing",
      scope: "whole",
      description: "Listed on CMS Appendix I & II, CITES Appendix II"
    },
    {
      code: "3.1",               // Species management
      timing: "ongoing",
      scope: "minority",
      description: "Population monitoring at key aggregation sites"
    }
  ],

  useTrade: "Used for meat, fins, liver oil in some countries. International trade regulated by CITES.",

  dataQuality: {
    adequacy: "adequate",
    documentation: "Good population data from key aggregation sites; broader population trends inferred",
    needsUpdating: false
  }
};
```

### Marine Protected Area (MPA) Design

Data-driven MPA designation:

```typescript
interface MarineProtectedArea {
  mpaID: string;
  name: string;
  designation: string;           // "National Park", "Marine Reserve", "UNESCO Site"
  iucnCategory: "Ia" | "Ib" | "II" | "III" | "IV" | "V" | "VI";
  /*
    Ia = Strict Nature Reserve
    Ib = Wilderness Area
    II = National Park
    III = Natural Monument
    IV = Habitat/Species Management Area
    V = Protected Landscape/Seascape
    VI = Protected area with sustainable use
  */

  geography: {
    location: {
      latitude: number;
      longitude: number;
    };
    boundary: string;            // WKT polygon
    area: number;                // km²
    marineArea: number;          // km²
    depthRange: { min: number; max: number };
  };

  governance: {
    country: string;
    governanceType: "government" | "co-management" | "private" | "community" | "indigenous";
    managingAuthority: string;
    establishedDate: Date;
    legalStatus: string;
  };

  protection: {
    level: "no_take" | "restricted_take" | "multiple_use";
    regulations: Regulation[];
    enforcement: "high" | "medium" | "low" | "none";
  };

  biodiversity: {
    habitatTypes: string[];
    totalSpecies: number;
    endangeredSpecies: number;
    endemicSpecies: number;
    keySpecies: {
      aphiaID: number;
      scientificName: string;
      conservationStatus: string;
      role: "flagship" | "keystone" | "endangered" | "endemic";
    }[];
  };

  effectiveness: {
    biomassInside: number;       // kg/hectare
    biomassOutside: number;
    speciesRichnessInside: number;
    speciesRichnessOutside: number;
    assessmentDate: Date;
    methodology: string;
  };

  threats: {
    threat: string;
    severity: "high" | "medium" | "low";
    trend: "increasing" | "stable" | "decreasing";
  }[];

  monitoring: {
    program: string;
    frequency: string;           // "annual", "biannual", "continuous"
    parameters: string[];
    dataAvailability: string;    // URL or repository
  };
}

interface Regulation {
  activity: string;              // "fishing", "anchoring", "diving", "extraction"
  status: "prohibited" | "allowed" | "permitted" | "seasonal";
  details: string;
}

// Example: Papahānaumokuākea Marine National Monument
const papahanaumokuakea: MarineProtectedArea = {
  mpaID: "WDPA-555565429",
  name: "Papahānaumokuākea Marine National Monument",
  designation: "National Monument / Marine National Monument",
  iucnCategory: "Ia",            // Strict Nature Reserve

  geography: {
    location: {
      latitude: 25.7,
      longitude: -170.0
    },
    boundary: "POLYGON((-178.0 23.0, -161.0 23.0, -161.0 29.0, -178.0 29.0, -178.0 23.0))",
    area: 1508870,               // 1.5 million km² - one of world's largest MPAs
    marineArea: 1508870,
    depthRange: { min: 0, max: 5800 }
  },

  governance: {
    country: "USA",
    governanceType: "government",
    managingAuthority: "NOAA, US Fish & Wildlife Service, State of Hawaii",
    establishedDate: new Date("2006-06-15"),
    legalStatus: "Presidential Proclamation 8031"
  },

  protection: {
    level: "no_take",
    regulations: [
      {
        activity: "commercial_fishing",
        status: "prohibited",
        details: "All commercial fishing prohibited"
      },
      {
        activity: "recreational_fishing",
        status: "prohibited",
        details: "Recreational fishing prohibited except in Midway Atoll Special Management Area"
      },
      {
        activity: "anchoring",
        status: "permitted",
        details: "Only for permitted research and education vessels"
      },
      {
        activity: "mining",
        status: "prohibited",
        details: "All extractive activities prohibited"
      }
    ],
    enforcement: "high"
  },

  biodiversity: {
    habitatTypes: ["coral_reefs", "seamounts", "deep_sea", "pelagic", "atolls"],
    totalSpecies: 7000,
    endangeredSpecies: 23,
    endemicSpecies: 400,
    keySpecies: [
      {
        aphiaID: 137206,
        scientificName: "Monachus schauinslandi",
        conservationStatus: "Endangered",
        role: "flagship"
      },
      {
        aphiaID: 137063,
        scientificName: "Chelonia mydas",
        conservationStatus: "Endangered",
        role: "flagship"
      },
      {
        aphiaID: 212153,
        scientificName: "Thunnus alalunga",
        conservationStatus: "Least Concern",
        role: "keystone"
      }
    ]
  },

  effectiveness: {
    biomassInside: 385,          // kg/hectare (very high)
    biomassOutside: 145,
    speciesRichnessInside: 156,
    speciesRichnessOutside: 98,
    assessmentDate: new Date("2023-06-01"),
    methodology: "Underwater visual census, baited remote underwater video"
  },

  threats: [
    {
      threat: "Marine debris (plastic pollution)",
      severity: "high",
      trend: "increasing"
    },
    {
      threat: "Climate change (coral bleaching, ocean acidification)",
      severity: "high",
      trend: "increasing"
    },
    {
      threat: "Illegal fishing",
      severity: "low",
      trend: "stable"
    }
  ],

  monitoring: {
    program: "Pacific Islands Fisheries Science Center - Coral Reef Ecosystem Division",
    frequency: "biannual",
    parameters: ["fish_biomass", "coral_cover", "species_richness", "water_quality", "monk_seal_population"],
    dataAvailability: "https://www.fisheries.noaa.gov/inport/item/15986"
  }
};
```

### Species Distribution Modeling for Conservation

Predict where endangered species occur to guide protection:

```typescript
interface SpeciesDistributionModel {
  species: {
    aphiaID: number;
    scientificName: string;
    conservationStatus: string;
  };

  occurrenceData: {
    source: string;              // "OBIS", "GBIF", "field_surveys"
    records: number;
    spatialBias: boolean;
    temporalRange: { start: Date; end: Date };
  };

  environmentalPredictors: {
    variable: string;
    source: string;
    resolution: string;
    importance: number;          // Variable importance (0-1)
  }[];

  modelingApproach: {
    algorithm: "MaxEnt" | "GLM" | "GAM" | "RandomForest" | "BRT" | "ensemble";
    software: string;
    validationMethod: "k-fold_CV" | "spatial_block_CV" | "temporal_split";
  };

  performance: {
    AUC: number;                 // Area Under ROC Curve (0.5-1.0)
    TSS: number;                 // True Skill Statistic (-1 to 1)
    sensitivity: number;         // True positive rate
    specificity: number;         // True negative rate
    omissionRate: number;
  };

  predictions: {
    suitabilityMap: string;      // URL to GeoTIFF
    thresholdApplied: number;    // Presence/absence threshold
    predictedRange: number;      // km² of suitable habitat
    uncertainty: string;         // URL to uncertainty map
  };

  conservation_implications: {
    criticalHabitat: string;     // WKT polygons
    protectedProportion: number; // Percentage in MPAs
    unprotectedHotspots: string; // Priority areas needing protection
    climateRefugia: string;      // Areas likely to remain suitable
  };
}

// Example: Vaquita porpoise (most endangered marine mammal)
const vaquitaSDM: SpeciesDistributionModel = {
  species: {
    aphiaID: 137117,
    scientificName: "Phocoena sinus",
    conservationStatus: "Critically Endangered"
  },

  occurrenceData: {
    source: "Acoustic monitoring + visual surveys",
    records: 47,                 // Very limited data
    spatialBias: true,           // Concentrated in surveyed areas
    temporalRange: {
      start: new Date("2015-01-01"),
      end: new Date("2024-12-31")
    }
  },

  environmentalPredictors: [
    {
      variable: "sea_surface_temperature",
      source: "MODIS satellite",
      resolution: "4km",
      importance: 0.65
    },
    {
      variable: "bathymetry",
      source: "GEBCO",
      resolution: "500m",
      importance: 0.82            // Most important - shallow Gulf waters
    },
    {
      variable: "chlorophyll",
      source: "MODIS",
      resolution: "4km",
      importance: 0.43
    },
    {
      variable: "distance_to_shore",
      source: "calculated",
      resolution: "100m",
      importance: 0.56
    }
  ],

  modelingApproach: {
    algorithm: "MaxEnt",
    software: "MaxEnt 3.4.4",
    validationMethod: "k-fold_CV"
  },

  performance: {
    AUC: 0.94,                   // Excellent discrimination
    TSS: 0.85,
    sensitivity: 0.91,
    specificity: 0.94,
    omissionRate: 0.09
  },

  predictions: {
    suitabilityMap: "https://example.org/vaquita_habitat.tif",
    thresholdApplied: 0.5,
    predictedRange: 2235,        // Only 2,235 km² of suitable habitat
    uncertainty: "https://example.org/vaquita_uncertainty.tif"
  },

  conservation_implications: {
    criticalHabitat: "POLYGON((-114.5 31.0, -114.0 31.0, -114.0 31.3, -114.5 31.3, -114.5 31.0))",
    protectedProportion: 67,     // 67% in protected area
    unprotectedHotspots: "Northern Gulf of California core area",
    climateRefugia: "Upper Gulf remains suitable under warming scenarios"
  }
};
```

### Illegal Fishing Detection

Using data to combat illegal, unreported, and unregulated (IUU) fishing:

```typescript
interface VesselTrackingData {
  vesselID: string;              // IMO or MMSI number
  vesselName: string;
  flag: string;                  // Country of registration
  vesselType: string;

  ais: {                         // Automatic Identification System
    timestamp: Date;
    latitude: number;
    longitude: number;
    speed: number;               // Knots
    course: number;              // Degrees
    status: string;
  }[];

  vms?: {                        // Vessel Monitoring System (if equipped)
    timestamp: Date;
    latitude: number;
    longitude: number;
  }[];

  fishing_events: {
    timestamp: Date;
    location: { latitude: number; longitude: number };
    detectionMethod: "AIS_speed_pattern" | "VMS" | "satellite_imagery" | "ML_classifier";
    confidence: number;
  }[];

  violations: {
    type: "MPA_incursion" | "unlicensed_fishing" | "AIS_gap" | "transshipment";
    timestamp: Date;
    location: { latitude: number; longitude: number };
    evidence: string;
    severity: "high" | "medium" | "low";
  }[];
}

class IUUFishingDetector {
  // Detect fishing activity from AIS speed patterns
  detectFishingFromAIS(ais: VesselTrackingData['ais']): {
    timestamp: Date;
    location: { latitude: number; longitude: number };
    confidence: number;
  }[] {
    const fishingEvents: any[] = [];

    for (let i = 1; i < ais.length - 1; i++) {
      const prev = ais[i - 1];
      const curr = ais[i];
      const next = ais[i + 1];

      // Fishing vessels typically move slowly with frequent course changes
      const avgSpeed = (prev.speed + curr.speed + next.speed) / 3;
      const courseChange = Math.abs(curr.course - prev.course);

      if (avgSpeed < 3 && courseChange > 30) {
        fishingEvents.push({
          timestamp: curr.timestamp,
          location: {
            latitude: curr.latitude,
            longitude: curr.longitude
          },
          confidence: 0.75
        });
      }
    }

    return fishingEvents;
  }

  // Detect MPA violations
  detectMPAViolations(
    vessel: VesselTrackingData,
    mpas: MarineProtectedArea[]
  ): VesselTrackingData['violations'] {
    const violations: VesselTrackingData['violations'] = [];

    vessel.ais.forEach(point => {
      mpas.forEach(mpa => {
        if (this.pointInMPA(point.latitude, point.longitude, mpa)) {
          // Check if fishing is prohibited
          const fishingProhibited = mpa.protection.regulations.some(
            reg => reg.activity === 'commercial_fishing' && reg.status === 'prohibited'
          );

          if (fishingProhibited) {
            violations.push({
              type: "MPA_incursion",
              timestamp: point.timestamp,
              location: {
                latitude: point.latitude,
                longitude: point.longitude
              },
              evidence: `Vessel in ${mpa.name} where fishing prohibited`,
              severity: "high"
            });
          }
        }
      });
    });

    return violations;
  }

  // Detect suspicious AIS gaps (possible intentional disabling)
  detectAISGaps(ais: VesselTrackingData['ais']): VesselTrackingData['violations'] {
    const violations: VesselTrackingData['violations'] = [];

    for (let i = 1; i < ais.length; i++) {
      const prev = ais[i - 1];
      const curr = ais[i];

      const gapHours = (curr.timestamp.getTime() - prev.timestamp.getTime()) / (1000 * 60 * 60);

      // AIS should transmit every few minutes; >4 hour gap is suspicious
      if (gapHours > 4) {
        violations.push({
          type: "AIS_gap",
          timestamp: curr.timestamp,
          location: {
            latitude: curr.latitude,
            longitude: curr.longitude
          },
          evidence: `${gapHours.toFixed(1)} hour AIS gap`,
          severity: gapHours > 24 ? "high" : "medium"
        });
      }
    }

    return violations;
  }

  private pointInMPA(lat: number, lon: number, mpa: MarineProtectedArea): boolean {
    // Simplified point-in-polygon check
    // Real implementation would use robust geospatial library
    return false;  // Placeholder
  }
}
```

### Conservation Success Metrics

```typescript
interface ConservationMetrics {
  mpa: string;
  assessmentPeriod: { start: Date; end: Date };

  biodiversity: {
    speciesRichness: {
      before: number;
      after: number;
      percentChange: number;
    };

    endemicSpecies: {
      before: number;
      after: number;
    };

    threatenedSpecies: {
      improvementCount: number;  // Species moving to lower threat category
      declineCount: number;
    };
  };

  biomass: {
    totalFishBiomass: {
      before: number;             // kg/hectare
      after: number;
      percentChange: number;
    };

    targetSpeciesBiomass: Record<string, {
      before: number;
      after: number;
      percentChange: number;
    }>;
  };

  habitatHealth: {
    coralCover?: {
      before: number;             // Percentage
      after: number;
      percentChange: number;
    };

    seagrassCoverage?: {
      before: number;             // Hectares
      after: number;
    };
  };

  socialEconomic: {
    tourismRevenue: {
      before: number;             // USD/year
      after: number;
    };

    localEmployment: {
      before: number;
      after: number;
    };

    spilloverBenefit: {
      adjacentFishingImprovement: number;  // Percentage
    };
  };

  effectiveness: "highly_effective" | "effective" | "partially_effective" | "ineffective";
}
```

### Best Practices for Conservation Data

**1. Long-term Monitoring**
Establish baseline before protection, monitor changes over decades.

**2. Standardized Methods**
Use consistent survey protocols for temporal comparability.

**3. Before-After-Control-Impact (BACI) Design**
Compare protected areas to unprotected controls.

**4. Multiple Metrics**
Assess biodiversity, biomass, habitat, and socioeconomic outcomes.

**5. Adaptive Management**
Use monitoring data to adjust management strategies.

**6. Stakeholder Engagement**
Include local communities in monitoring and decision-making.

**7. Open Data Sharing**
Share conservation outcomes to inform global best practices.

### Philosophy: Conservation Through Knowledge

Conservation data embodies 弘益人間 - benefiting all humanity by protecting the ocean for current and future generations. When we share data on what works in conservation, we enable:

- Effective protection of endangered species
- Successful marine protected area management
- Evidence-based policy decisions
- Global learning and improvement
- Hope for ocean recovery

Every observation contributes to safeguarding marine biodiversity. Every dataset shared accelerates conservation worldwide.

---

**Next Chapter:** Future of Marine Biology Data - emerging technologies and the next frontier of ocean science.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
