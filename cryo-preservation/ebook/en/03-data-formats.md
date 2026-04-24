# Chapter 3: Data Formats and Schema Definitions

**弘益人間 (Benefit All Humanity)**

---

## Overview

This chapter provides comprehensive data format specifications for the WIA Cryo Preservation Standard. All schemas are defined using Zod for runtime type validation and TypeScript for compile-time type safety. These schemas cover the complete lifecycle of cryopreserved specimens from collection through long-term storage and eventual disposition.

---

## Core Data Structures

### Specimen Core Schema

```typescript
/**
 * WIA Cryo Preservation - Complete Data Format Specifications
 * Comprehensive Zod schemas for all cryopreservation entities
 */

import { z } from 'zod';

/**
 * UUID validation helper
 */
const UUIDSchema = z.string().uuid();

/**
 * ISO 8601 date-time schema
 */
const DateTimeSchema = z.union([z.date(), z.string().datetime()]).transform(val =>
  typeof val === 'string' ? new Date(val) : val
);

/**
 * Temperature measurement schema
 */
export const TemperatureSchema = z.object({
  value: z.number(),
  unit: z.enum(['CELSIUS', 'FAHRENHEIT', 'KELVIN']),
  timestamp: DateTimeSchema,
  measurementMethod: z.enum(['DIGITAL', 'ANALOG', 'INFRARED', 'THERMOCOUPLE']).optional()
});

/**
 * Volume measurement schema
 */
export const VolumeSchema = z.object({
  value: z.number().positive(),
  unit: z.enum(['ML', 'L', 'UL', 'CC']),
  measurementMethod: z.enum(['PIPETTE', 'GRADUATED_CYLINDER', 'AUTOMATED']).optional()
});

/**
 * Concentration measurement schema
 */
export const ConcentrationSchema = z.object({
  value: z.number().positive(),
  unit: z.enum(['CELLS_PER_ML', 'MILLION_PER_ML', 'PERCENT', 'MOL_PER_L']),
  cellType: z.string().optional(),
  countingMethod: z.enum(['HEMOCYTOMETER', 'AUTOMATED_COUNTER', 'FLOW_CYTOMETRY']).optional()
});

/**
 * Person/operator schema
 */
export const PersonSchema = z.object({
  personId: UUIDSchema,
  firstName: z.string(),
  lastName: z.string(),
  credentials: z.array(z.string()).optional(),
  role: z.enum([
    'EMBRYOLOGIST',
    'TECHNICIAN',
    'NURSE',
    'PHYSICIAN',
    'SCIENTIST',
    'ADMINISTRATOR'
  ]),
  licenseNumber: z.string().optional(),
  certifications: z.array(z.string()).optional()
});

/**
 * Facility/location schema
 */
export const FacilitySchema = z.object({
  facilityId: UUIDSchema,
  name: z.string(),
  type: z.enum([
    'IVF_CLINIC',
    'BIOBANK',
    'CORD_BLOOD_BANK',
    'RESEARCH_LAB',
    'HOSPITAL',
    'COMMERCIAL_FACILITY'
  ]),
  address: z.object({
    street: z.string(),
    city: z.string(),
    state: z.string().optional(),
    country: z.string(),
    postalCode: z.string(),
    coordinates: z.object({
      latitude: z.number(),
      longitude: z.number()
    }).optional()
  }),
  accreditations: z.array(z.enum(['AABB', 'CAP', 'FACT', 'FDA', 'ISO_15189', 'ISO_9001'])),
  contactInfo: z.object({
    phone: z.string(),
    email: z.string().email(),
    website: z.string().url().optional()
  }),
  operatingHours: z.string().optional()
});

/**
 * Complete specimen schema with all metadata
 */
export const SpecimenSchema = z.object({
  // Primary identification
  specimenId: UUIDSchema,
  externalId: z.string().optional(),
  barcode: z.string().min(1).max(100),
  rfidTag: z.string().optional(),
  qrCode: z.string().optional(),

  // Specimen classification
  category: z.enum([
    'GAMETES',
    'EMBRYOS',
    'CORD_BLOOD',
    'TISSUES',
    'CELLS',
    'DNA_RNA',
    'MICROORGANISMS',
    'PLANT_MATERIAL'
  ]),
  subcategory: z.string(),
  description: z.string(),
  scientificName: z.string().optional(),

  // Donor/source information
  donor: z.object({
    donorId: UUIDSchema,
    anonymized: z.boolean(),
    donorType: z.enum(['AUTOLOGOUS', 'ALLOGENEIC', 'DIRECTED', 'ANONYMOUS']),
    demographics: z.object({
      age: z.number().min(0).max(120),
      ageAtCollection: z.number().min(0).max(120),
      sex: z.enum(['MALE', 'FEMALE', 'INTERSEX', 'UNKNOWN']),
      ethnicity: z.string().optional(),
      bloodType: z.enum(['A+', 'A-', 'B+', 'B-', 'AB+', 'AB-', 'O+', 'O-', 'UNKNOWN']).optional()
    }),
    medicalHistory: z.object({
      hasGeneticConditions: z.boolean(),
      geneticConditions: z.array(z.string()).optional(),
      infectiousDiseaseTesting: z.array(z.object({
        test: z.string(),
        result: z.enum(['NEGATIVE', 'POSITIVE', 'INDETERMINATE']),
        testDate: DateTimeSchema
      })),
      medications: z.array(z.string()).optional(),
      relevantConditions: z.array(z.string()).optional()
    }).optional()
  }),

  // Collection details
  collection: z.object({
    collectionId: UUIDSchema,
    collectionDate: DateTimeSchema,
    collectionTime: z.string().optional(), // HH:MM format
    collectedBy: PersonSchema,
    collectionSite: FacilitySchema,
    collectionMethod: z.string(),
    collectionKit: z.string().optional(),
    complications: z.array(z.string()).optional(),
    notes: z.string().optional()
  }),

  // Specimen physical properties
  physicalProperties: z.object({
    volume: VolumeSchema,
    concentration: ConcentrationSchema.optional(),
    cellCount: z.number().optional(),
    viability: z.object({
      percentage: z.number().min(0).max(100),
      assessmentMethod: z.enum([
        'TRYPAN_BLUE',
        'FLOW_CYTOMETRY',
        'ATP_ASSAY',
        'MICROSCOPY',
        'AUTOMATED_ANALYZER'
      ]),
      assessmentDate: DateTimeSchema,
      assessedBy: PersonSchema
    }).optional(),
    motility: z.object({
      percentage: z.number().min(0).max(100),
      progressiveMotility: z.number().min(0).max(100).optional(),
      assessmentMethod: z.enum(['MICROSCOPY', 'CASA', 'MANUAL_COUNT']),
      assessmentDate: DateTimeSchema
    }).optional(),
    morphology: z.object({
      normalPercentage: z.number().min(0).max(100),
      assessmentMethod: z.string(),
      strictCriteria: z.boolean().optional(),
      assessmentDate: DateTimeSchema
    }).optional(),
    color: z.string().optional(),
    turbidity: z.enum(['CLEAR', 'SLIGHTLY_TURBID', 'TURBID', 'OPAQUE']).optional(),
    pH: z.number().min(0).max(14).optional()
  }),

  // Quality assessment
  quality: z.object({
    overallGrade: z.enum(['EXCELLENT', 'GOOD', 'FAIR', 'POOR', 'UNACCEPTABLE']),
    qualityScore: z.number().min(0).max(100),
    gradingSystem: z.string(), // e.g., "Gardner", "ASRM"
    assessmentDate: DateTimeSchema,
    assessedBy: PersonSchema,
    assessmentNotes: z.string().optional(),
    images: z.array(z.object({
      imageId: UUIDSchema,
      url: z.string().url(),
      captureDate: DateTimeSchema,
      magnification: z.string().optional(),
      annotations: z.string().optional()
    })).optional()
  }),

  // Processing history
  processing: z.array(z.object({
    processingId: UUIDSchema,
    step: z.string(),
    date: DateTimeSchema,
    performedBy: PersonSchema,
    protocol: z.string(),
    reagents: z.array(z.object({
      name: z.string(),
      lotNumber: z.string(),
      expirationDate: DateTimeSchema,
      volume: VolumeSchema.optional()
    })).optional(),
    equipment: z.array(z.object({
      equipmentId: z.string(),
      type: z.string(),
      model: z.string().optional()
    })).optional(),
    duration: z.number().optional(), // minutes
    result: z.enum(['SUCCESS', 'PARTIAL', 'FAILED']),
    notes: z.string().optional()
  })),

  // Cryopreservation details
  cryopreservation: z.object({
    protocol: z.object({
      protocolId: UUIDSchema,
      name: z.string(),
      version: z.string(),
      approvedBy: z.string(),
      approvalDate: DateTimeSchema
    }),
    cryoprotectant: z.object({
      primary: z.string(),
      concentration: z.number(),
      secondary: z.string().optional(),
      secondaryConcentration: z.number().optional(),
      equilibrationTime: z.number(), // minutes
      exposureTime: z.number(), // minutes
      lotNumber: z.string().optional()
    }),
    freezeMethod: z.enum([
      'SLOW_FREEZE',
      'VITRIFICATION',
      'DIRECTIONAL_FREEZE',
      'PASSIVE',
      'TWO_STEP'
    ]),
    freezingCurve: z.array(z.object({
      stepNumber: z.number(),
      startTemp: TemperatureSchema,
      endTemp: TemperatureSchema,
      rate: z.string(), // e.g., "-1°C/min"
      duration: z.number(), // minutes
      actualDuration: z.number().optional() // minutes
    })),
    freezeDate: DateTimeSchema,
    freezeStartTime: z.string().optional(),
    freezeEndTime: z.string().optional(),
    performedBy: PersonSchema,
    equipment: z.object({
      freezerId: z.string(),
      model: z.string(),
      calibrationDate: DateTimeSchema.optional()
    }),
    container: z.object({
      type: z.enum(['STRAW', 'VIAL', 'CRYOTUBE', 'BAG', 'AMPULE', 'CUSTOM']),
      manufacturer: z.string(),
      lotNumber: z.string().optional(),
      volume: VolumeSchema,
      sterile: z.boolean()
    }),
    finalTemperature: TemperatureSchema,
    qualityControl: z.object({
      postFreezeSampleTaken: z.boolean(),
      viabilityCheck: z.number().min(0).max(100).optional(),
      integrityCheck: z.enum(['PASSED', 'FAILED', 'NOT_PERFORMED']),
      notes: z.string().optional()
    }).optional()
  }),

  // Storage information
  storage: z.object({
    storageId: UUIDSchema,
    facility: FacilitySchema,
    tank: z.object({
      tankId: z.string(),
      tankName: z.string(),
      type: z.enum(['LIQUID_NITROGEN', 'VAPOR_PHASE', 'MECHANICAL_FREEZER', 'DRY_ICE']),
      capacity: z.number(),
      currentLoad: z.number().optional()
    }),
    location: z.object({
      canister: z.string(),
      goblet: z.string().optional(),
      cane: z.string().optional(),
      box: z.string().optional(),
      position: z.string(),
      level: z.string().optional()
    }),
    storageDate: DateTimeSchema,
    storageTemperature: TemperatureSchema,
    storagePhase: z.enum(['VAPOR', 'LIQUID']),
    monitoringFrequency: z.string(), // e.g., "Every 4 hours"
    alarmSystem: z.boolean(),
    backupSystem: z.boolean()
  }),

  // Consent and legal
  consent: z.object({
    consentId: UUIDSchema,
    status: z.enum(['OBTAINED', 'PENDING', 'WITHDRAWN', 'EXPIRED']),
    consentDate: DateTimeSchema.optional(),
    expirationDate: DateTimeSchema.optional(),
    consentType: z.array(z.enum([
      'STORAGE',
      'CLINICAL_USE',
      'RESEARCH',
      'DONATION',
      'DISPOSAL',
      'TRANSFER',
      'GENETIC_TESTING'
    ])),
    restrictions: z.array(z.string()).optional(),
    signedDocument: z.object({
      documentId: UUIDSchema,
      url: z.string().url(),
      signatureDate: DateTimeSchema,
      witnessName: z.string().optional()
    }).optional()
  }),

  // Regulatory compliance
  regulatory: z.object({
    status: z.enum(['APPROVED', 'PENDING', 'RESTRICTED', 'QUARANTINE', 'REJECTED']),
    approvalDate: DateTimeSchema.optional(),
    approvedBy: z.string().optional(),
    certifications: z.array(z.string()),
    inspections: z.array(z.object({
      inspectionId: UUIDSchema,
      date: DateTimeSchema,
      inspector: z.string(),
      result: z.enum(['PASSED', 'FAILED', 'CONDITIONAL']),
      findings: z.string().optional()
    })).optional(),
    adverseEvents: z.array(z.object({
      eventId: UUIDSchema,
      date: DateTimeSchema,
      description: z.string(),
      severity: z.enum(['MINOR', 'MODERATE', 'SEVERE', 'CRITICAL']),
      reportedTo: z.array(z.string()),
      resolution: z.string().optional()
    })).optional()
  }),

  // Chain of custody
  chainOfCustody: z.array(z.object({
    eventId: UUIDSchema,
    timestamp: DateTimeSchema,
    eventType: z.enum([
      'COLLECTION',
      'RECEIVED',
      'PROCESSED',
      'FROZEN',
      'STORED',
      'MOVED',
      'THAWED',
      'DISPOSED',
      'TRANSFERRED'
    ]),
    performedBy: PersonSchema,
    location: z.string(),
    witnessedBy: PersonSchema.optional(),
    notes: z.string().optional()
  })),

  // Specimen status
  status: z.enum([
    'ACTIVE',
    'QUARANTINE',
    'THAWED',
    'IN_USE',
    'DISPOSED',
    'TRANSFERRED',
    'LOST',
    'DAMAGED'
  ]),
  statusDate: DateTimeSchema,
  statusReason: z.string().optional(),

  // Disposition
  disposition: z.object({
    dispositionDate: DateTimeSchema.optional(),
    dispositionType: z.enum([
      'CLINICAL_USE',
      'RESEARCH_USE',
      'DISCARDED',
      'TRANSFERRED',
      'DONATED',
      'PENDING'
    ]).optional(),
    destinationFacility: FacilitySchema.optional(),
    authorizedBy: PersonSchema.optional(),
    documentation: z.string().optional()
  }).optional(),

  // Financial
  financial: z.object({
    ownershipType: z.enum(['PRIVATE', 'PUBLIC', 'RESEARCH', 'COMMERCIAL']),
    fees: z.array(z.object({
      feeType: z.enum(['COLLECTION', 'PROCESSING', 'STORAGE', 'TESTING', 'TRANSFER', 'OTHER']),
      amount: z.number(),
      currency: z.string().length(3), // ISO 4217
      billingDate: DateTimeSchema,
      paidDate: DateTimeSchema.optional(),
      status: z.enum(['PENDING', 'PAID', 'OVERDUE', 'WAIVED'])
    })).optional(),
    insurance: z.object({
      provider: z.string(),
      policyNumber: z.string(),
      coverageAmount: z.number(),
      expirationDate: DateTimeSchema
    }).optional()
  }).optional(),

  // Metadata
  metadata: z.object({
    createdAt: DateTimeSchema,
    createdBy: PersonSchema,
    updatedAt: DateTimeSchema,
    updatedBy: PersonSchema,
    version: z.number().int().positive(),
    dataSource: z.string().optional(),
    importedFrom: z.string().optional(),
    tags: z.array(z.string()).optional(),
    customFields: z.record(z.any()).optional()
  })
});

export type Specimen = z.infer<typeof SpecimenSchema>;
```

---

## Preservation Protocol Schema

```typescript
/**
 * Comprehensive preservation protocol schema
 */

export const PreservationProtocolSchema = z.object({
  protocolId: UUIDSchema,
  name: z.string(),
  version: z.string(),
  status: z.enum(['DRAFT', 'ACTIVE', 'DEPRECATED', 'EXPERIMENTAL']),

  // Metadata
  createdDate: DateTimeSchema,
  createdBy: PersonSchema,
  lastModified: DateTimeSchema,
  modifiedBy: PersonSchema,
  approvedDate: DateTimeSchema.optional(),
  approvedBy: z.array(PersonSchema).optional(),

  // Applicability
  applicability: z.object({
    specimenCategories: z.array(z.string()),
    specimenSubcategories: z.array(z.string()),
    ageRestrictions: z.object({
      minAge: z.number().optional(),
      maxAge: z.number().optional(),
      unit: z.enum(['HOURS', 'DAYS', 'WEEKS', 'YEARS']).optional()
    }).optional(),
    volumeRestrictions: z.object({
      minVolume: VolumeSchema.optional(),
      maxVolume: VolumeSchema.optional()
    }).optional(),
    contraindications: z.array(z.string()).optional()
  }),

  // Pre-freeze preparation
  preparation: z.object({
    steps: z.array(z.object({
      stepNumber: z.number(),
      name: z.string(),
      description: z.string(),
      duration: z.number(), // minutes
      temperature: TemperatureSchema.optional(),
      reagents: z.array(z.object({
        name: z.string(),
        concentration: z.string(),
        volume: VolumeSchema,
        supplier: z.string().optional(),
        catalogNumber: z.string().optional()
      })).optional(),
      equipment: z.array(z.string()).optional(),
      criticalControlPoints: z.array(z.string()).optional(),
      acceptanceCriteria: z.string().optional()
    })),
    totalPreparationTime: z.number(), // minutes
    environmentalRequirements: z.object({
      temperature: z.string(),
      humidity: z.string().optional(),
      cleanroomClass: z.string().optional()
    }).optional()
  }),

  // Cryoprotectant formulation
  cryoprotectant: z.object({
    formulation: z.array(z.object({
      component: z.string(),
      type: z.enum([
        'PENETRATING',
        'NON_PENETRATING',
        'PROTEIN',
        'SUGAR',
        'POLYMER',
        'BUFFER',
        'OTHER'
      ]),
      concentration: z.number(),
      unit: z.string(), // e.g., "%", "M", "mM"
      purpose: z.string()
    })),
    preparationInstructions: z.string(),
    equilibrationProtocol: z.object({
      steps: z.array(z.object({
        stepNumber: z.number(),
        concentration: z.number(),
        duration: z.number(), // minutes
        temperature: TemperatureSchema
      })),
      totalEquilibrationTime: z.number() // minutes
    }),
    washingProtocol: z.object({
      required: z.boolean(),
      steps: z.array(z.object({
        stepNumber: z.number(),
        washingSolution: z.string(),
        duration: z.number(), // minutes
        centrifugationSpeed: z.number().optional() // rpm
      }))
    }).optional(),
    toxicityProfile: z.object({
      maxExposureTime: z.number(), // minutes
      maxConcentration: z.number(),
      symptoms: z.array(z.string())
    }).optional()
  }),

  // Cooling protocol
  cooling: z.object({
    method: z.enum([
      'SLOW_PROGRAMMABLE',
      'VITRIFICATION',
      'DIRECTIONAL',
      'PASSIVE',
      'TWO_STEP',
      'ULTRA_RAPID'
    ]),
    equipment: z.object({
      type: z.string(),
      manufacturer: z.string().optional(),
      model: z.string().optional(),
      calibrationRequired: z.boolean(),
      calibrationFrequency: z.string().optional()
    }),
    steps: z.array(z.object({
      stepNumber: z.number(),
      name: z.string(),
      startTemperature: TemperatureSchema,
      endTemperature: TemperatureSchema,
      coolingRate: z.string(), // e.g., "-1°C/min"
      rateRange: z.object({
        min: z.string(),
        max: z.string()
      }).optional(),
      duration: z.number(), // minutes
      holdTime: z.number().optional(), // minutes at end temperature
      criticalParameters: z.array(z.string()).optional(),
      monitoringRequired: z.boolean()
    })),
    seedingRequired: z.boolean(),
    seedingTemperature: TemperatureSchema.optional(),
    totalCoolingTime: z.number() // minutes
  }),

  // Storage specifications
  storage: z.object({
    temperature: TemperatureSchema,
    temperatureTolerance: z.object({
      upper: TemperatureSchema,
      lower: TemperatureSchema
    }),
    phase: z.enum(['VAPOR', 'LIQUID', 'MECHANICAL']),
    containerType: z.array(z.string()),
    maxStorageDuration: z.object({
      value: z.number(),
      unit: z.enum(['DAYS', 'MONTHS', 'YEARS', 'INDEFINITE'])
    }).optional(),
    monitoringRequirements: z.object({
      frequency: z.string(), // e.g., "Every 4 hours"
      parameters: z.array(z.string()), // e.g., ["Temperature", "LN2 level"]
      alarmLimits: z.record(z.string())
    })
  }),

  // Thawing protocol
  thawing: z.object({
    steps: z.array(z.object({
      stepNumber: z.number(),
      name: z.string(),
      temperature: TemperatureSchema,
      method: z.string(), // e.g., "Water bath", "Room temperature"
      duration: z.string(), // e.g., "Until ice disappears"
      agitation: z.boolean().optional(),
      monitoring: z.string().optional()
    })),
    cryoprotectantRemoval: z.object({
      required: z.boolean(),
      method: z.string().optional(),
      steps: z.array(z.object({
        stepNumber: z.number(),
        solution: z.string(),
        duration: z.number(), // minutes
        concentration: z.number().optional()
      }))
    }),
    postThawAssessment: z.array(z.string()), // e.g., ["Viability", "Motility"]
    totalThawingTime: z.number() // minutes
  }),

  // Quality control
  qualityControl: z.object({
    processingControls: z.array(z.object({
      parameter: z.string(),
      specification: z.string(),
      method: z.string(),
      frequency: z.string()
    })),
    releaseTestingRequired: z.array(z.string()),
    expectedOutcomes: z.object({
      minViability: z.number(), // percentage
      minRecovery: z.number(), // percentage
      maxAcceptableVariation: z.number() // percentage
    }),
    validationRequirements: z.object({
      minimumValidationRuns: z.number(),
      successCriteria: z.string(),
      validationDate: DateTimeSchema.optional(),
      revalidationFrequency: z.string().optional()
    })
  }),

  // Safety and precautions
  safety: z.object({
    personnelRequirements: z.object({
      minimumTraining: z.array(z.string()),
      certificationRequired: z.boolean(),
      experienceLevel: z.enum(['ENTRY', 'INTERMEDIATE', 'ADVANCED', 'EXPERT'])
    }),
    personalProtectiveEquipment: z.array(z.string()),
    hazards: z.array(z.object({
      type: z.enum(['CHEMICAL', 'BIOLOGICAL', 'PHYSICAL', 'CRYOGENIC']),
      description: z.string(),
      severity: z.enum(['LOW', 'MODERATE', 'HIGH', 'CRITICAL']),
      mitigation: z.string()
    })),
    emergencyProcedures: z.array(z.object({
      scenario: z.string(),
      response: z.string(),
      contacts: z.array(z.string())
    })).optional()
  }),

  // References and validation
  references: z.object({
    publications: z.array(z.object({
      title: z.string(),
      authors: z.array(z.string()),
      journal: z.string(),
      year: z.number(),
      doi: z.string().optional(),
      url: z.string().url().optional()
    })).optional(),
    guidelines: z.array(z.string()).optional(),
    validationStudies: z.array(z.object({
      studyId: z.string(),
      description: z.string(),
      date: DateTimeSchema,
      sampleSize: z.number(),
      results: z.string(),
      documentUrl: z.string().url().optional()
    })).optional(),
    regulatoryApprovals: z.array(z.object({
      agency: z.string(), // e.g., "FDA", "EMA"
      approvalNumber: z.string(),
      approvalDate: DateTimeSchema,
      expirationDate: DateTimeSchema.optional()
    })).optional()
  }),

  // Notes and comments
  notes: z.string().optional(),
  changeLog: z.array(z.object({
    version: z.string(),
    date: DateTimeSchema,
    author: PersonSchema,
    changes: z.string()
  })).optional()
});

export type PreservationProtocol = z.infer<typeof PreservationProtocolSchema>;
```

---

## Storage Tank Schema

```typescript
/**
 * Cryogenic storage tank schema
 */

export const StorageTankSchema = z.object({
  tankId: UUIDSchema,
  tankName: z.string(),
  serialNumber: z.string(),

  // Tank specifications
  specifications: z.object({
    manufacturer: z.string(),
    model: z.string(),
    manufactureDate: DateTimeSchema,
    installDate: DateTimeSchema,
    warrantyExpiration: DateTimeSchema.optional(),

    type: z.enum([
      'LIQUID_NITROGEN_DEWAR',
      'VAPOR_PHASE_FREEZER',
      'MECHANICAL_FREEZER',
      'DRY_SHIPPER',
      'CRYOGENIC_REFRIGERATOR'
    ]),

    capacity: z.object({
      total: z.number(), // liters
      usable: z.number(), // liters
      specimenCapacity: z.number() // maximum specimens
    }),

    dimensions: z.object({
      height: z.number(), // cm
      diameter: z.number(), // cm
      weight: z.number() // kg
    }),

    temperatureRange: z.object({
      nominal: TemperatureSchema,
      minimum: TemperatureSchema,
      maximum: TemperatureSchema
    }),

    insulation: z.object({
      type: z.string(),
      vacuumSeal: z.boolean(),
      staticEvaporationRate: z.number() // liters/day
    })
  }),

  // Current status
  status: z.object({
    operational: z.boolean(),
    currentTemperature: TemperatureSchema,
    liquidNitrogenLevel: z.number(), // percentage
    lastFill: DateTimeSchema.optional(),
    nextScheduledFill: DateTimeSchema.optional(),
    currentLoad: z.number(), // number of specimens
    utilizationRate: z.number(), // percentage

    alarms: z.array(z.object({
      alarmId: UUIDSchema,
      type: z.enum(['TEMPERATURE', 'LEVEL', 'POWER', 'DOOR', 'SYSTEM']),
      severity: z.enum(['INFO', 'WARNING', 'CRITICAL']),
      triggeredAt: DateTimeSchema,
      clearedAt: DateTimeSchema.optional(),
      message: z.string(),
      acknowledged: z.boolean(),
      acknowledgedBy: PersonSchema.optional()
    }))
  }),

  // Monitoring system
  monitoring: z.object({
    sensors: z.array(z.object({
      sensorId: z.string(),
      type: z.enum(['TEMPERATURE', 'LEVEL', 'PRESSURE', 'HUMIDITY']),
      location: z.string(), // e.g., "Top", "Middle", "Bottom"
      manufacturer: z.string().optional(),
      model: z.string().optional(),
      calibrationDate: DateTimeSchema,
      nextCalibration: DateTimeSchema,
      accuracy: z.string() // e.g., "±0.5°C"
    })),

    dataLogging: z.object({
      enabled: z.boolean(),
      interval: z.number(), // seconds
      retentionPeriod: z.number(), // days
      cloudBackup: z.boolean(),
      alertChannels: z.array(z.enum(['EMAIL', 'SMS', 'PHONE', 'PAGER', 'SYSTEM']))
    }),

    alarmConfiguration: z.object({
      temperatureAlarms: z.array(z.object({
        threshold: TemperatureSchema,
        type: z.enum(['HIGH', 'LOW']),
        delay: z.number(), // seconds before triggering
        escalation: z.array(z.object({
          level: z.number(),
          delayMinutes: z.number(),
          contacts: z.array(z.string())
        }))
      })),
      levelAlarms: z.array(z.object({
        threshold: z.number(), // percentage
        type: z.enum(['LOW', 'CRITICAL']),
        contacts: z.array(z.string())
      }))
    })
  }),

  // Organization structure
  organization: z.object({
    levels: z.number(), // Number of vertical levels
    canisters: z.array(z.object({
      canisterId: z.string(),
      level: z.number(),
      capacity: z.number(), // specimens
      currentLoad: z.number(),
      goblets: z.array(z.object({
        gobletId: z.string(),
        capacity: z.number(),
        currentLoad: z.number(),
        specimenIds: z.array(UUIDSchema)
      })).optional(),
      canes: z.array(z.object({
        caneId: z.string(),
        capacity: z.number(),
        currentLoad: z.number(),
        specimenIds: z.array(UUIDSchema)
      })).optional()
    })),
    inventoryMap: z.record(z.string()).optional() // specimenId -> location path
  }),

  // Maintenance history
  maintenance: z.array(z.object({
    maintenanceId: UUIDSchema,
    date: DateTimeSchema,
    type: z.enum([
      'ROUTINE',
      'PREVENTIVE',
      'CORRECTIVE',
      'EMERGENCY',
      'CALIBRATION',
      'CLEANING'
    ]),
    performedBy: z.object({
      name: z.string(),
      company: z.string(),
      certification: z.string().optional()
    }),
    workPerformed: z.string(),
    partsReplaced: z.array(z.object({
      part: z.string(),
      serialNumber: z.string().optional(),
      cost: z.number().optional()
    })).optional(),
    duration: z.number(), // minutes
    cost: z.number().optional(),
    nextScheduledMaintenance: DateTimeSchema.optional(),
    certificationDocument: z.string().url().optional()
  })),

  // Regulatory and compliance
  compliance: z.object({
    certifications: z.array(z.string()),
    lastInspection: DateTimeSchema.optional(),
    nextInspection: DateTimeSchema.optional(),
    inspectionResults: z.array(z.object({
      inspectionId: UUIDSchema,
      date: DateTimeSchema,
      inspector: z.string(),
      agency: z.string(),
      result: z.enum(['PASSED', 'FAILED', 'CONDITIONAL']),
      findings: z.string().optional(),
      correctiveActions: z.array(z.string()).optional()
    })).optional(),
    sops: z.array(z.object({
      sopId: z.string(),
      title: z.string(),
      version: z.string(),
      effectiveDate: DateTimeSchema
    }))
  }),

  // Location
  location: z.object({
    facility: FacilitySchema,
    room: z.string(),
    coordinates: z.object({
      x: z.number().optional(),
      y: z.number().optional(),
      zone: z.string().optional()
    }).optional(),
    accessRestrictions: z.array(z.string()).optional()
  }),

  // Metadata
  metadata: z.object({
    createdAt: DateTimeSchema,
    updatedAt: DateTimeSchema,
    tags: z.array(z.string()).optional(),
    notes: z.string().optional()
  })
});

export type StorageTank = z.infer<typeof StorageTankSchema>;
```

---

## Temperature Monitoring Schema

```typescript
/**
 * Temperature monitoring and alarm schema
 */

export const TemperatureMonitoringSchema = z.object({
  monitoringId: UUIDSchema,
  tankId: UUIDSchema,

  // Reading details
  readings: z.array(z.object({
    readingId: UUIDSchema,
    timestamp: DateTimeSchema,
    sensorId: z.string(),
    sensorLocation: z.string(),

    temperature: TemperatureSchema,
    pressure: z.number().optional(), // psi
    humidity: z.number().optional(), // percentage
    liquidLevel: z.number().optional(), // percentage

    quality: z.enum(['GOOD', 'QUESTIONABLE', 'BAD']),
    flags: z.array(z.enum([
      'CALIBRATION_DUE',
      'SENSOR_MALFUNCTION',
      'OUT_OF_RANGE',
      'RAPID_CHANGE',
      'COMMUNICATION_ERROR'
    ])).optional()
  })),

  // Statistical analysis
  statistics: z.object({
    period: z.object({
      start: DateTimeSchema,
      end: DateTimeSchema
    }),
    temperatureStats: z.object({
      mean: z.number(),
      median: z.number(),
      min: z.number(),
      max: z.number(),
      standardDeviation: z.number(),
      range: z.number()
    }),
    excursions: z.array(z.object({
      excursionId: UUIDSchema,
      startTime: DateTimeSchema,
      endTime: DateTimeSchema.optional(),
      peakDeviation: z.number(),
      duration: z.number(), // minutes
      affectedSpecimens: z.array(UUIDSchema).optional(),
      rootCause: z.string().optional(),
      correctiveAction: z.string().optional(),
      investigationComplete: z.boolean()
    })),
    uptime: z.number(), // percentage
    dataCompleteness: z.number() // percentage
  }),

  // Alerts and notifications
  alerts: z.array(z.object({
    alertId: UUIDSchema,
    timestamp: DateTimeSchema,
    type: z.enum([
      'HIGH_TEMPERATURE',
      'LOW_TEMPERATURE',
      'RAPID_RISE',
      'RAPID_FALL',
      'SENSOR_FAILURE',
      'LOW_LIQUID_LEVEL',
      'POWER_FAILURE'
    ]),
    severity: z.enum(['INFO', 'WARNING', 'CRITICAL', 'EMERGENCY']),
    value: z.number(),
    threshold: z.number(),
    message: z.string(),

    notifications: z.array(z.object({
      notificationId: UUIDSchema,
      channel: z.enum(['EMAIL', 'SMS', 'PHONE', 'PAGER', 'SYSTEM']),
      recipient: z.string(),
      sentAt: DateTimeSchema,
      deliveryStatus: z.enum(['SENT', 'DELIVERED', 'FAILED', 'BOUNCED']),
      acknowledged: z.boolean(),
      acknowledgedAt: DateTimeSchema.optional(),
      acknowledgedBy: PersonSchema.optional()
    })),

    response: z.object({
      responseTime: z.number().optional(), // minutes
      actionTaken: z.string().optional(),
      resolvedAt: DateTimeSchema.optional(),
      resolvedBy: PersonSchema.optional()
    }).optional()
  }))
});

export type TemperatureMonitoring = z.infer<typeof TemperatureMonitoringSchema>;
```

---

## Thawing Record Schema

```typescript
/**
 * Specimen thawing and post-thaw assessment schema
 */

export const ThawingRecordSchema = z.object({
  thawingId: UUIDSchema,
  specimenId: UUIDSchema,

  // Thawing details
  thawDate: DateTimeSchema,
  thawStartTime: z.string(), // HH:MM
  thawEndTime: z.string(), // HH:MM
  totalDuration: z.number(), // minutes

  // Personnel
  performedBy: PersonSchema,
  witnessedBy: PersonSchema.optional(),
  supervisedBy: PersonSchema.optional(),

  // Protocol
  protocol: z.object({
    protocolId: UUIDSchema,
    name: z.string(),
    version: z.string()
  }),

  // Storage information before thaw
  preThawStorage: z.object({
    tankId: z.string(),
    location: z.string(),
    storageDuration: z.number(), // days
    storageTemperature: TemperatureSchema,
    retrievalTime: DateTimeSchema
  }),

  // Thawing process
  thawingSteps: z.array(z.object({
    stepNumber: z.number(),
    description: z.string(),
    startTime: z.string(), // HH:MM
    endTime: z.string(), // HH:MM
    temperature: TemperatureSchema,
    method: z.string(), // e.g., "37°C water bath"
    observations: z.string().optional(),
    deviations: z.string().optional()
  })),

  // Cryoprotectant removal
  cryoprotectantRemoval: z.object({
    performed: z.boolean(),
    method: z.string().optional(),
    steps: z.array(z.object({
      stepNumber: z.number(),
      solution: z.string(),
      volume: VolumeSchema,
      duration: z.number(), // minutes
      centrifugationSpeed: z.number().optional(), // rpm
      centrifugationTime: z.number().optional() // minutes
    })).optional(),
    finalWash: z.object({
      solution: z.string(),
      volume: VolumeSchema
    }).optional()
  }),

  // Post-thaw assessment
  postThawAssessment: z.object({
    assessmentTime: DateTimeSchema,
    assessedBy: PersonSchema,

    viability: z.object({
      percentage: z.number().min(0).max(100),
      method: z.string(),
      notes: z.string().optional()
    }).optional(),

    motility: z.object({
      percentage: z.number().min(0).max(100),
      progressiveMotility: z.number().min(0).max(100).optional(),
      method: z.string()
    }).optional(),

    morphology: z.object({
      normalPercentage: z.number().min(0).max(100),
      assessment: z.string(),
      method: z.string()
    }).optional(),

    integrity: z.object({
      status: z.enum(['INTACT', 'PARTIALLY_DAMAGED', 'SEVERELY_DAMAGED', 'NON_VIABLE']),
      details: z.string().optional()
    }),

    recovery: z.object({
      percentage: z.number().min(0).max(100),
      calculationMethod: z.string()
    }).optional(),

    overallQuality: z.enum(['EXCELLENT', 'GOOD', 'FAIR', 'POOR', 'UNACCEPTABLE']),
    qualityScore: z.number().min(0).max(100),

    microscopyImages: z.array(z.object({
      imageId: UUIDSchema,
      url: z.string().url(),
      magnification: z.string(),
      captureTime: DateTimeSchema,
      annotations: z.string().optional()
    })).optional()
  }),

  // Comparison with pre-freeze
  comparison: z.object({
    preFreezeViability: z.number().optional(),
    postThawViability: z.number().optional(),
    viabilityRetention: z.number().optional(), // percentage
    preFreezeMotility: z.number().optional(),
    postThawMotility: z.number().optional(),
    motilityRetention: z.number().optional(), // percentage
    meetsExpectedOutcomes: z.boolean(),
    comments: z.string().optional()
  }),

  // Disposition
  disposition: z.object({
    dispositionType: z.enum([
      'CLINICAL_USE',
      'RESEARCH_USE',
      'QUALITY_CONTROL',
      'DISCARDED',
      'REFROZEN'
    ]),
    dispositionDate: DateTimeSchema,
    destination: z.string().optional(),
    recipientPatientId: UUIDSchema.optional(),
    procedureId: UUIDSchema.optional(),
    authorizedBy: PersonSchema,
    documentation: z.string().optional()
  }),

  // Deviations and incidents
  deviations: z.array(z.object({
    deviationId: UUIDSchema,
    description: z.string(),
    severity: z.enum(['MINOR', 'MODERATE', 'MAJOR', 'CRITICAL']),
    potentialImpact: z.string(),
    correctiveAction: z.string().optional(),
    reportedTo: z.array(z.string()).optional()
  })).optional(),

  // Quality control
  qualityControl: z.object({
    reviewed: z.boolean(),
    reviewedBy: PersonSchema.optional(),
    reviewDate: DateTimeSchema.optional(),
    approved: z.boolean(),
    approvedBy: PersonSchema.optional(),
    approvalDate: DateTimeSchema.optional(),
    comments: z.string().optional()
  }),

  // Metadata
  metadata: z.object({
    createdAt: DateTimeSchema,
    updatedAt: DateTimeSchema,
    dataEnteredBy: PersonSchema,
    verified: z.boolean(),
    verifiedBy: PersonSchema.optional(),
    verificationDate: DateTimeSchema.optional()
  })
});

export type ThawingRecord = z.infer<typeof ThawingRecordSchema>;
```

---

## Audit Log Schema

```typescript
/**
 * Comprehensive audit logging schema
 */

export const AuditLogSchema = z.object({
  logId: UUIDSchema,
  timestamp: DateTimeSchema,

  // Actor information
  actor: z.object({
    userId: UUIDSchema,
    username: z.string(),
    person: PersonSchema,
    ipAddress: z.string().ip().optional(),
    userAgent: z.string().optional(),
    sessionId: z.string().optional()
  }),

  // Action details
  action: z.object({
    type: z.enum([
      'CREATE',
      'READ',
      'UPDATE',
      'DELETE',
      'FREEZE',
      'THAW',
      'MOVE',
      'TRANSFER',
      'DISPOSE',
      'LOGIN',
      'LOGOUT',
      'SEARCH',
      'EXPORT',
      'PRINT',
      'APPROVE',
      'REJECT'
    ]),
    resource: z.enum([
      'SPECIMEN',
      'PROTOCOL',
      'TANK',
      'FACILITY',
      'USER',
      'CONSENT',
      'REPORT',
      'CONFIGURATION'
    ]),
    resourceId: z.string(),
    resourceName: z.string().optional(),
    description: z.string()
  }),

  // Data changes
  changes: z.object({
    before: z.record(z.any()).optional(),
    after: z.record(z.any()).optional(),
    diff: z.array(z.object({
      field: z.string(),
      oldValue: z.any(),
      newValue: z.any()
    })).optional()
  }).optional(),

  // Context
  context: z.object({
    facility: z.string().optional(),
    department: z.string().optional(),
    purpose: z.string().optional(),
    relatedEntities: z.array(z.object({
      type: z.string(),
      id: z.string()
    })).optional()
  }).optional(),

  // Result
  result: z.object({
    status: z.enum(['SUCCESS', 'FAILURE', 'PARTIAL']),
    errorMessage: z.string().optional(),
    errorCode: z.string().optional(),
    duration: z.number().optional() // milliseconds
  }),

  // Security
  security: z.object({
    authenticationMethod: z.enum(['PASSWORD', 'SSO', 'API_KEY', 'CERTIFICATE', 'BIOMETRIC']),
    authorizationPassed: z.boolean(),
    requiredPermissions: z.array(z.string()),
    grantedPermissions: z.array(z.string()),
    mfaUsed: z.boolean().optional(),
    riskScore: z.number().min(0).max(100).optional()
  }).optional(),

  // Compliance
  compliance: z.object({
    dataAccessJustification: z.string().optional(),
    hipaaCompliant: z.boolean().optional(),
    gdprCompliant: z.boolean().optional(),
    retentionPeriod: z.number().optional(), // days
    archived: z.boolean()
  }).optional()
});

export type AuditLog = z.infer<typeof AuditLogSchema>;
```

---

## Data Validation Examples

```typescript
/**
 * Example validation functions using the schemas
 */

export class CryoDataValidator {
  /**
   * Validate specimen data
   */
  static validateSpecimen(data: unknown): Specimen {
    try {
      return SpecimenSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        console.error('Specimen validation failed:', error.errors);
        throw new Error(`Invalid specimen data: ${error.errors.map(e => e.message).join(', ')}`);
      }
      throw error;
    }
  }

  /**
   * Validate preservation protocol
   */
  static validateProtocol(data: unknown): PreservationProtocol {
    try {
      return PreservationProtocolSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        console.error('Protocol validation failed:', error.errors);
        throw new Error(`Invalid protocol data: ${error.errors.map(e => e.message).join(', ')}`);
      }
      throw error;
    }
  }

  /**
   * Validate storage tank data
   */
  static validateTank(data: unknown): StorageTank {
    try {
      return StorageTankSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        console.error('Tank validation failed:', error.errors);
        throw new Error(`Invalid tank data: ${error.errors.map(e => e.message).join(', ')}`);
      }
      throw error;
    }
  }

  /**
   * Partial validation for updates
   */
  static validatePartialSpecimen(data: unknown): Partial<Specimen> {
    const PartialSchema = SpecimenSchema.partial();
    try {
      return PartialSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        throw new Error(`Invalid partial specimen data: ${error.errors.map(e => e.message).join(', ')}`);
      }
      throw error;
    }
  }

  /**
   * Validate array of specimens
   */
  static validateSpecimenBatch(data: unknown): Specimen[] {
    const BatchSchema = z.array(SpecimenSchema);
    try {
      return BatchSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        throw new Error(`Invalid specimen batch: ${error.errors.map(e => e.message).join(', ')}`);
      }
      throw error;
    }
  }
}
```

---

## Summary

This chapter provides comprehensive Zod schemas for all entities in the WIA Cryo Preservation Standard:

- **Specimen Schema**: Complete specimen metadata with 15+ major sections
- **Preservation Protocol Schema**: Detailed protocol specifications with all steps
- **Storage Tank Schema**: Tank specifications, monitoring, and maintenance
- **Temperature Monitoring Schema**: Real-time monitoring and alarm data
- **Thawing Record Schema**: Post-thaw assessment and disposition
- **Audit Log Schema**: Comprehensive audit trail for compliance

All schemas include:
- Type-safe validation with Zod
- Comprehensive field coverage
- Nested object support
- Optional and required field distinctions
- Enum constraints for controlled vocabularies
- Date/time handling
- Extensibility through custom fields

---

**弘益人間 (Benefit All Humanity)**

*Standardized data formats enable interoperability, quality assurance, and global advancement in cryopreservation technology.*
