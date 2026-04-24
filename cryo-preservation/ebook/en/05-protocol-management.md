# Chapter 5: Protocol Management - Freezing and Thawing Procedures

**弘益人間 (Benefit All Humanity)**

---

## Overview

This chapter provides comprehensive protocol management for cryopreservation, covering freezing procedures (slow freeze, vitrification, directional freezing), cryoprotectant formulations, thawing procedures, and quality control. These protocols are critical for maintaining specimen viability and ensuring consistent outcomes.

---

## Freezing Protocol Management System

### Protocol Repository

```typescript
/**
 * WIA Cryo Preservation - Protocol Management System
 * Comprehensive freezing and thawing protocol library
 */

import { z } from 'zod';
import {
  PreservationProtocol,
  SpecimenCategory,
  CoolingMethod,
  CryoprotectantType
} from './schemas';

/**
 * Protocol validation result
 */
export interface ProtocolValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  recommendations: string[];
}

/**
 * Protocol execution step
 */
export interface ProtocolExecutionStep {
  stepNumber: number;
  name: string;
  description: string;
  duration: number; // minutes
  temperature?: number; // Celsius
  completed: boolean;
  completedAt?: Date;
  performedBy?: string;
  notes?: string;
  deviations?: string[];
}

/**
 * Protocol execution context
 */
export interface ProtocolExecutionContext {
  protocolId: string;
  specimenId: string;
  startTime: Date;
  currentStep: number;
  steps: ProtocolExecutionStep[];
  status: 'NOT_STARTED' | 'IN_PROGRESS' | 'COMPLETED' | 'ABORTED';
  completedAt?: Date;
  performedBy: string;
}

/**
 * Protocol Manager
 * Central system for protocol storage, retrieval, and execution
 */
export class ProtocolManager {
  private protocols: Map<string, PreservationProtocol> = new Map();
  private executionContexts: Map<string, ProtocolExecutionContext> = new Map();

  /**
   * Register a new protocol
   */
  registerProtocol(protocol: PreservationProtocol): void {
    const validation = this.validateProtocol(protocol);

    if (!validation.valid) {
      throw new Error(
        `Protocol validation failed: ${validation.errors.join(', ')}`
      );
    }

    if (validation.warnings.length > 0) {
      console.warn('Protocol warnings:', validation.warnings);
    }

    this.protocols.set(protocol.protocolId, protocol);
    console.log(`Protocol registered: ${protocol.name} v${protocol.version}`);
  }

  /**
   * Validate protocol
   */
  validateProtocol(protocol: PreservationProtocol): ProtocolValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Check required fields
    if (!protocol.name || protocol.name.trim().length === 0) {
      errors.push('Protocol name is required');
    }

    if (!protocol.version || protocol.version.trim().length === 0) {
      errors.push('Protocol version is required');
    }

    // Validate preparation steps
    if (!protocol.preparation?.steps || protocol.preparation.steps.length === 0) {
      warnings.push('No preparation steps defined');
    } else {
      const stepNumbers = protocol.preparation.steps.map(s => s.stepNumber);
      const uniqueSteps = new Set(stepNumbers);
      if (uniqueSteps.size !== stepNumbers.length) {
        errors.push('Duplicate step numbers in preparation');
      }
    }

    // Validate cooling protocol
    if (!protocol.cooling?.steps || protocol.cooling.steps.length === 0) {
      errors.push('Cooling protocol must have at least one step');
    } else {
      // Check temperature progression
      for (let i = 1; i < protocol.cooling.steps.length; i++) {
        const prevEnd = protocol.cooling.steps[i - 1].endTemperature.value;
        const currStart = protocol.cooling.steps[i].startTemperature.value;

        if (Math.abs(currStart - prevEnd) > 5) {
          warnings.push(
            `Temperature discontinuity between step ${i} and ${i + 1}: ${prevEnd}°C to ${currStart}°C`
          );
        }

        if (currStart >= prevEnd) {
          errors.push(
            `Step ${i + 1} does not decrease temperature (${currStart}°C to ${prevEnd}°C)`
          );
        }
      }
    }

    // Validate cryoprotectant
    if (protocol.cryoprotectant?.formulation?.length === 0) {
      errors.push('Cryoprotectant formulation is required');
    }

    // Validate thawing protocol
    if (!protocol.thawing?.steps || protocol.thawing.steps.length === 0) {
      warnings.push('No thawing steps defined');
    }

    // Recommendations
    if (!protocol.qualityControl?.expectedOutcomes) {
      recommendations.push('Define expected outcomes for quality control');
    }

    if (!protocol.references?.publications || protocol.references.publications.length === 0) {
      recommendations.push('Add scientific references to support protocol');
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      recommendations
    };
  }

  /**
   * Find optimal protocol for specimen
   */
  findOptimalProtocol(params: {
    category: SpecimenCategory;
    subcategory: string;
    volume?: number;
    donorAge?: number;
  }): PreservationProtocol | null {
    const candidates = Array.from(this.protocols.values()).filter(protocol => {
      // Check status
      if (protocol.status !== 'ACTIVE') {
        return false;
      }

      // Check applicability
      if (!protocol.applicability.specimenCategories.includes(params.category)) {
        return false;
      }

      if (!protocol.applicability.specimenSubcategories.includes(params.subcategory)) {
        return false;
      }

      // Check volume restrictions
      if (params.volume && protocol.applicability.volumeRestrictions) {
        const volRestrictions = protocol.applicability.volumeRestrictions;
        if (volRestrictions.minVolume && params.volume < volRestrictions.minVolume.value) {
          return false;
        }
        if (volRestrictions.maxVolume && params.volume > volRestrictions.maxVolume.value) {
          return false;
        }
      }

      // Check age restrictions
      if (params.donorAge && protocol.applicability.ageRestrictions) {
        const ageRestrictions = protocol.applicability.ageRestrictions;
        if (ageRestrictions.minAge && params.donorAge < ageRestrictions.minAge) {
          return false;
        }
        if (ageRestrictions.maxAge && params.donorAge > ageRestrictions.maxAge) {
          return false;
        }
      }

      return true;
    });

    // Sort by expected viability (descending)
    candidates.sort((a, b) => {
      const aViability = a.qualityControl?.expectedOutcomes?.minViability ?? 0;
      const bViability = b.qualityControl?.expectedOutcomes?.minViability ?? 0;
      return bViability - aViability;
    });

    return candidates[0] || null;
  }

  /**
   * Start protocol execution
   */
  startExecution(
    protocolId: string,
    specimenId: string,
    performedBy: string
  ): ProtocolExecutionContext {
    const protocol = this.protocols.get(protocolId);
    if (!protocol) {
      throw new Error(`Protocol not found: ${protocolId}`);
    }

    const steps: ProtocolExecutionStep[] = [];

    // Add preparation steps
    protocol.preparation.steps.forEach(step => {
      steps.push({
        stepNumber: step.stepNumber,
        name: step.name,
        description: step.description,
        duration: step.duration,
        temperature: step.temperature?.value,
        completed: false
      });
    });

    // Add cooling steps
    protocol.cooling.steps.forEach((step, index) => {
      steps.push({
        stepNumber: protocol.preparation.steps.length + index + 1,
        name: step.name,
        description: `Cool from ${step.startTemperature.value}°C to ${step.endTemperature.value}°C at ${step.coolingRate}`,
        duration: step.duration,
        temperature: step.endTemperature.value,
        completed: false
      });
    });

    const context: ProtocolExecutionContext = {
      protocolId,
      specimenId,
      startTime: new Date(),
      currentStep: 0,
      steps,
      status: 'NOT_STARTED',
      performedBy
    };

    this.executionContexts.set(specimenId, context);
    return context;
  }

  /**
   * Complete current step
   */
  completeStep(
    specimenId: string,
    notes?: string,
    deviations?: string[]
  ): ProtocolExecutionContext {
    const context = this.executionContexts.get(specimenId);
    if (!context) {
      throw new Error(`Execution context not found for specimen: ${specimenId}`);
    }

    if (context.status === 'COMPLETED' || context.status === 'ABORTED') {
      throw new Error(`Cannot complete step: protocol execution is ${context.status}`);
    }

    const currentStep = context.steps[context.currentStep];
    if (!currentStep) {
      throw new Error('No current step to complete');
    }

    currentStep.completed = true;
    currentStep.completedAt = new Date();
    currentStep.performedBy = context.performedBy;
    currentStep.notes = notes;
    currentStep.deviations = deviations;

    // Move to next step
    context.currentStep++;

    // Check if all steps completed
    if (context.currentStep >= context.steps.length) {
      context.status = 'COMPLETED';
      context.completedAt = new Date();
    } else if (context.status === 'NOT_STARTED') {
      context.status = 'IN_PROGRESS';
    }

    return context;
  }

  /**
   * Get execution progress
   */
  getExecutionProgress(specimenId: string): {
    totalSteps: number;
    completedSteps: number;
    currentStep: number;
    percentComplete: number;
    estimatedTimeRemaining: number; // minutes
  } {
    const context = this.executionContexts.get(specimenId);
    if (!context) {
      throw new Error(`Execution context not found for specimen: ${specimenId}`);
    }

    const completedSteps = context.steps.filter(s => s.completed).length;
    const remainingSteps = context.steps.slice(context.currentStep);
    const estimatedTimeRemaining = remainingSteps.reduce(
      (sum, step) => sum + step.duration,
      0
    );

    return {
      totalSteps: context.steps.length,
      completedSteps,
      currentStep: context.currentStep,
      percentComplete: (completedSteps / context.steps.length) * 100,
      estimatedTimeRemaining
    };
  }
}
```

---

## Slow Freeze Protocols

### Sperm Slow Freeze Protocol

```typescript
/**
 * Standard slow freeze protocol for human sperm
 * Based on World Health Organization guidelines
 */

export function createSpermSlowFreezeProtocol(): PreservationProtocol {
  return {
    protocolId: crypto.randomUUID(),
    name: 'Human Sperm Slow Freeze Protocol',
    version: '2.1',
    status: 'ACTIVE',

    createdDate: new Date('2024-01-01'),
    createdBy: {
      personId: crypto.randomUUID(),
      firstName: 'John',
      lastName: 'Smith',
      role: 'EMBRYOLOGIST',
      credentials: ['PhD', 'HCLD']
    } as any,
    lastModified: new Date(),
    modifiedBy: {
      personId: crypto.randomUUID(),
      firstName: 'John',
      lastName: 'Smith',
      role: 'EMBRYOLOGIST'
    } as any,

    applicability: {
      specimenCategories: ['GAMETES'],
      specimenSubcategories: ['SPERM', 'SEMEN'],
      volumeRestrictions: {
        minVolume: { value: 0.25, unit: 'ML' },
        maxVolume: { value: 5.0, unit: 'ML' }
      }
    },

    preparation: {
      steps: [
        {
          stepNumber: 1,
          name: 'Sample Collection',
          description: 'Collect semen sample in sterile container via masturbation after 2-5 days abstinence',
          duration: 30,
          acceptanceCriteria: 'Volume ≥ 1.5ml, complete liquefaction within 60 minutes'
        },
        {
          stepNumber: 2,
          name: 'Liquefaction',
          description: 'Allow sample to liquefy at room temperature (20-25°C)',
          duration: 30,
          temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          acceptanceCriteria: 'Sample fully liquefied, no viscous strands'
        },
        {
          stepNumber: 3,
          name: 'Initial Analysis',
          description: 'Perform semen analysis: volume, concentration, motility, morphology',
          duration: 15,
          criticalControlPoints: [
            'Use calibrated pipettes',
            'Use validated counting method',
            'Record all measurements'
          ]
        },
        {
          stepNumber: 4,
          name: 'Wash and Concentrate (Optional)',
          description: 'Centrifuge at 300g for 10 minutes, remove seminal plasma if needed',
          duration: 15,
          equipment: ['Centrifuge calibrated to 300g'],
          acceptanceCriteria: 'Pellet visible, supernatant clear'
        }
      ],
      totalPreparationTime: 90,
      environmentalRequirements: {
        temperature: '20-25°C',
        humidity: '30-60%',
        cleanroomClass: 'ISO 7 or better'
      }
    },

    cryoprotectant: {
      formulation: [
        {
          component: 'Glycerol',
          type: 'PENETRATING',
          concentration: 10,
          unit: '%',
          purpose: 'Primary cryoprotectant, prevents ice crystal formation'
        },
        {
          component: 'Egg Yolk',
          type: 'NON_PENETRATING',
          concentration: 20,
          unit: '%',
          purpose: 'Membrane stabilizer, protects acrosome'
        },
        {
          component: 'TEST Yolk Buffer',
          type: 'BUFFER',
          concentration: 70,
          unit: '%',
          purpose: 'Maintains pH and osmolality'
        }
      ],
      preparationInstructions: 'Mix components in sterile conditions, filter through 0.22μm filter, warm to 37°C before use',
      equilibrationProtocol: {
        steps: [
          {
            stepNumber: 1,
            concentration: 5,
            duration: 10,
            temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() }
          },
          {
            stepNumber: 2,
            concentration: 10,
            duration: 10,
            temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() }
          }
        ],
        totalEquilibrationTime: 20
      },
      toxicityProfile: {
        maxExposureTime: 120,
        maxConcentration: 15,
        symptoms: ['Reduced motility if exposed > 2 hours', 'Osmotic stress if concentration > 15%']
      }
    },

    cooling: {
      method: 'SLOW_PROGRAMMABLE',
      equipment: {
        type: 'Programmable Rate Freezer',
        manufacturer: 'Planer',
        calibrationRequired: true,
        calibrationFrequency: 'Annual'
      },
      steps: [
        {
          stepNumber: 1,
          name: 'Initial Cooling',
          startTemperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: 5, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: '-1°C/min',
          duration: 17,
          monitoringRequired: true
        },
        {
          stepNumber: 2,
          name: 'Seeding',
          startTemperature: { value: 5, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: -8, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: '-5°C/min',
          duration: 3,
          criticalParameters: ['Manual seeding at -5°C to -7°C', 'Observe ice crystal formation']
        },
        {
          stepNumber: 3,
          name: 'Slow Freeze Phase',
          startTemperature: { value: -8, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: -80, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: '-10°C/min',
          duration: 8,
          monitoringRequired: true
        },
        {
          stepNumber: 4,
          name: 'Plunge into Liquid Nitrogen',
          startTemperature: { value: -80, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: 'Rapid plunge',
          duration: 1,
          criticalParameters: ['Ensure complete submersion', 'Avoid temperature shock']
        }
      ],
      seedingRequired: true,
      seedingTemperature: { value: -6, unit: 'CELSIUS', timestamp: new Date() },
      totalCoolingTime: 29
    },

    storage: {
      temperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
      temperatureTolerance: {
        upper: { value: -150, unit: 'CELSIUS', timestamp: new Date() },
        lower: { value: -196, unit: 'CELSIUS', timestamp: new Date() }
      },
      phase: 'LIQUID',
      containerType: ['0.5ml Straw', '0.25ml Straw', 'Cryovial'],
      maxStorageDuration: {
        value: 10,
        unit: 'YEARS'
      },
      monitoringRequirements: {
        frequency: 'Continuous',
        parameters: ['Temperature', 'LN2 Level'],
        alarmLimits: {
          temperature: '-150°C',
          ln2Level: '30%'
        }
      }
    },

    thawing: {
      steps: [
        {
          stepNumber: 1,
          name: 'Remove from Storage',
          temperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          method: 'Rapid removal from LN2 tank',
          duration: 'Less than 30 seconds'
        },
        {
          stepNumber: 2,
          name: 'Rapid Thaw',
          temperature: { value: 37, unit: 'CELSIUS', timestamp: new Date() },
          method: 'Water bath at 37°C with gentle agitation',
          duration: 'Until ice completely melted (typically 30-60 seconds)',
          agitation: true,
          monitoring: 'Observe until last ice crystal disappears'
        }
      ],
      cryoprotectantRemoval: {
        required: true,
        method: 'Stepwise dilution',
        steps: [
          {
            stepNumber: 1,
            solution: 'TEST Yolk Buffer (no glycerol)',
            duration: 5,
            concentration: 50
          },
          {
            stepNumber: 2,
            solution: 'TEST Yolk Buffer (no glycerol)',
            duration: 5,
            concentration: 0
          },
          {
            stepNumber: 3,
            solution: 'Wash medium',
            duration: 10
          }
        ]
      },
      postThawAssessment: [
        'Motility assessment',
        'Viability staining',
        'Concentration count',
        'Morphology evaluation'
      ],
      totalThawingTime: 25
    },

    qualityControl: {
      processingControls: [
        {
          parameter: 'Temperature',
          specification: 'As per protocol ± 2°C',
          method: 'Calibrated thermocouple',
          frequency: 'Every freeze cycle'
        },
        {
          parameter: 'Cooling Rate',
          specification: 'As per protocol ± 10%',
          method: 'Data logger',
          frequency: 'Every freeze cycle'
        }
      ],
      releaseTestingRequired: ['Post-thaw motility', 'Post-thaw viability'],
      expectedOutcomes: {
        minViability: 50,
        minRecovery: 40,
        maxAcceptableVariation: 20
      },
      validationRequirements: {
        minimumValidationRuns: 10,
        successCriteria: '≥50% post-thaw motility in ≥80% of samples',
        revalidationFrequency: 'Annual or after protocol change'
      }
    },

    safety: {
      personnelRequirements: {
        minimumTraining: ['Cryopreservation theory', 'LN2 safety', 'Quality control'],
        certificationRequired: true,
        experienceLevel: 'INTERMEDIATE'
      },
      personalProtectiveEquipment: [
        'Cryogenic gloves',
        'Face shield',
        'Lab coat',
        'Closed-toe shoes'
      ],
      hazards: [
        {
          type: 'CRYOGENIC',
          description: 'Liquid nitrogen exposure',
          severity: 'HIGH',
          mitigation: 'Use PPE, handle in well-ventilated area, avoid skin contact'
        },
        {
          type: 'CHEMICAL',
          description: 'Glycerol and egg yolk components',
          severity: 'LOW',
          mitigation: 'Standard laboratory precautions, avoid ingestion'
        }
      ],
      emergencyProcedures: [
        {
          scenario: 'LN2 spill',
          response: 'Evacuate area, ensure ventilation, do not touch spill',
          contacts: ['Safety Officer: x1234', 'Emergency: 911']
        }
      ]
    },

    references: {
      publications: [
        {
          title: 'WHO laboratory manual for the examination and processing of human semen',
          authors: ['World Health Organization'],
          journal: 'WHO Press',
          year: 2021,
          url: 'https://www.who.int/publications/i/item/9789240030787'
        }
      ],
      guidelines: ['AABB Standards', 'CAP Laboratory Accreditation'],
      validationStudies: []
    },

    notes: 'This protocol is suitable for normozoospermic samples. Adjust for oligozoospermic samples.'
  } as any;
}
```

---

## Vitrification Protocols

### Oocyte Vitrification Protocol

```typescript
/**
 * Ultra-rapid vitrification protocol for mature human oocytes (MII)
 * High survival rates and maintains developmental competence
 */

export function createOocyteVitrificationProtocol(): PreservationProtocol {
  return {
    protocolId: crypto.randomUUID(),
    name: 'Human Oocyte Vitrification (Cryotop Method)',
    version: '3.0',
    status: 'ACTIVE',

    createdDate: new Date('2024-01-01'),
    createdBy: {
      personId: crypto.randomUUID(),
      firstName: 'Sarah',
      lastName: 'Johnson',
      role: 'EMBRYOLOGIST',
      credentials: ['MSc', 'HCLD', 'ELD']
    } as any,
    lastModified: new Date(),
    modifiedBy: {
      personId: crypto.randomUUID(),
      firstName: 'Sarah',
      lastName: 'Johnson',
      role: 'EMBRYOLOGIST'
    } as any,

    applicability: {
      specimenCategories: ['GAMETES'],
      specimenSubcategories: ['OOCYTES_MII', 'MATURE_OOCYTES'],
      ageRestrictions: {
        minAge: 0,
        maxAge: 4, // hours post-retrieval
        unit: 'HOURS'
      }
    },

    preparation: {
      steps: [
        {
          stepNumber: 1,
          name: 'Oocyte Assessment',
          description: 'Confirm MII stage (presence of first polar body), assess quality',
          duration: 5,
          temperature: { value: 37, unit: 'CELSIUS', timestamp: new Date() },
          acceptanceCriteria: 'First polar body visible, intact zona pellucida, normal cytoplasm'
        },
        {
          stepNumber: 2,
          name: 'Equilibration Media Preparation',
          description: 'Warm equilibration and vitrification solutions to room temperature',
          duration: 10,
          temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          reagents: [
            {
              name: 'Equilibration Solution (ES)',
              concentration: '7.5% EG + 7.5% DMSO',
              volume: { value: 500, unit: 'UL' }
            },
            {
              name: 'Vitrification Solution (VS)',
              concentration: '15% EG + 15% DMSO + 0.5M sucrose',
              volume: { value: 500, unit: 'UL' }
            }
          ]
        },
        {
          stepNumber: 3,
          name: 'Prepare Vitrification Device',
          description: 'Label Cryotop device, prepare handling tools',
          duration: 5,
          equipment: ['Cryotop', 'Fine bore pipette', 'Stereomicroscope']
        }
      ],
      totalPreparationTime: 20,
      environmentalRequirements: {
        temperature: '20-25°C',
        humidity: '30-60%',
        cleanroomClass: 'ISO 5'
      }
    },

    cryoprotectant: {
      formulation: [
        {
          component: 'Ethylene Glycol (EG)',
          type: 'PENETRATING',
          concentration: 15,
          unit: '%',
          purpose: 'Primary penetrating cryoprotectant, low toxicity'
        },
        {
          component: 'Dimethyl Sulfoxide (DMSO)',
          type: 'PENETRATING',
          concentration: 15,
          unit: '%',
          purpose: 'Secondary penetrating cryoprotectant, synergistic with EG'
        },
        {
          component: 'Sucrose',
          type: 'NON_PENETRATING',
          concentration: 0.5,
          unit: 'M',
          purpose: 'Osmotic dehydration, ice crystal inhibition'
        },
        {
          component: 'Human Serum Albumin',
          type: 'PROTEIN',
          concentration: 20,
          unit: '%',
          purpose: 'Membrane stabilization'
        }
      ],
      preparationInstructions: 'Use commercial vitrification kit (e.g., Kitazato, Irvine Scientific) prepared according to manufacturer instructions',
      equilibrationProtocol: {
        steps: [
          {
            stepNumber: 1,
            concentration: 15, // 7.5% EG + 7.5% DMSO
            duration: 10, // 10-13 minutes
            temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() }
          },
          {
            stepNumber: 2,
            concentration: 30, // 15% EG + 15% DMSO + sucrose
            duration: 1, // 45-60 seconds - CRITICAL
            temperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() }
          }
        ],
        totalEquilibrationTime: 11
      },
      toxicityProfile: {
        maxExposureTime: 60, // VS exposure < 60 seconds
        maxConcentration: 30,
        symptoms: [
          'Oocyte shrinkage if exposed > 60 seconds',
          'Zona hardening',
          'Spindle depolymerization'
        ]
      }
    },

    cooling: {
      method: 'VITRIFICATION',
      equipment: {
        type: 'Cryotop vitrification device',
        manufacturer: 'Kitazato',
        model: 'Cryotop SC',
        calibrationRequired: false
      },
      steps: [
        {
          stepNumber: 1,
          name: 'Load onto Cryotop',
          startTemperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: 'N/A',
          duration: 0.5, // 30 seconds
          criticalParameters: [
            'Minimal volume (~0.1 μL)',
            'Oocyte centered on film strip',
            'No air bubbles',
            'Work quickly to minimize exposure'
          ],
          monitoringRequired: true
        },
        {
          stepNumber: 2,
          name: 'Plunge into LN2',
          startTemperature: { value: 22, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: '>20,000°C/min',
          duration: 0.1, // <10 seconds
          criticalParameters: [
            'Direct plunge into LN2',
            'No hesitation',
            'Ensure complete submersion',
            'Avoid vapor phase during plunge'
          ],
          monitoringRequired: true
        },
        {
          stepNumber: 3,
          name: 'Secure Cover',
          startTemperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          endTemperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          coolingRate: 'N/A',
          duration: 0.2, // <15 seconds
          criticalParameters: [
            'Attach protective cover while submerged in LN2',
            'Keep submerged at all times'
          ]
        }
      ],
      seedingRequired: false,
      totalCoolingTime: 1 // < 1 minute total
    },

    storage: {
      temperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
      temperatureTolerance: {
        upper: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
        lower: { value: -196, unit: 'CELSIUS', timestamp: new Date() }
      },
      phase: 'LIQUID',
      containerType: ['Cryotop'],
      maxStorageDuration: {
        value: 10,
        unit: 'YEARS'
      },
      monitoringRequirements: {
        frequency: 'Continuous',
        parameters: ['Temperature', 'LN2 Level'],
        alarmLimits: {
          temperature: '-196°C (must remain in liquid phase)',
          ln2Level: '40%'
        }
      }
    },

    thawing: {
      steps: [
        {
          stepNumber: 1,
          name: 'Prepare Thawing Solutions',
          temperature: { value: 37, unit: 'CELSIUS', timestamp: new Date() },
          method: 'Warm thawing solutions (TS, DS, WS) to 37°C',
          duration: '10 minutes prior to thawing'
        },
        {
          stepNumber: 2,
          name: 'Remove from Storage',
          temperature: { value: -196, unit: 'CELSIUS', timestamp: new Date() },
          method: 'Remove Cryotop from LN2, remove cover while in LN2 vapor',
          duration: '<10 seconds'
        },
        {
          stepNumber: 3,
          name: 'Rapid Warming',
          temperature: { value: 37, unit: 'CELSIUS', timestamp: new Date() },
          method: 'Immediately plunge into 1.0M sucrose thawing solution at 37°C',
          duration: '1 minute',
          agitation: true,
          monitoring: 'Oocyte should rehydrate and return to normal appearance'
        }
      ],
      cryoprotectantRemoval: {
        required: true,
        method: 'Stepwise dilution in decreasing sucrose concentrations',
        steps: [
          {
            stepNumber: 1,
            solution: 'Thawing Solution (1.0M sucrose)',
            duration: 1,
            concentration: 100
          },
          {
            stepNumber: 2,
            solution: 'Dilution Solution (0.5M sucrose)',
            duration: 3,
            concentration: 50
          },
          {
            stepNumber: 3,
            solution: 'Washing Solution 1 (no sucrose)',
            duration: 5,
            concentration: 0
          },
          {
            stepNumber: 4,
            solution: 'Washing Solution 2',
            duration: 5,
            concentration: 0
          },
          {
            stepNumber: 5,
            solution: 'Culture medium',
            duration: 10,
            concentration: 0
          }
        ]
      },
      postThawAssessment: [
        'Oocyte survival (intact zona, normal cytoplasm)',
        'Recovery period 1-2 hours before ICSI',
        'Spindle recovery assessment if available'
      ],
      totalThawingTime: 25
    },

    qualityControl: {
      processingControls: [
        {
          parameter: 'CPA exposure time',
          specification: 'VS < 60 seconds',
          method: 'Timer',
          frequency: 'Every vitrification'
        },
        {
          parameter: 'Plunge speed',
          specification: 'Direct, no hesitation',
          method: 'Visual observation',
          frequency: 'Every vitrification'
        },
        {
          parameter: 'Solution temperature',
          specification: '37°C ± 0.5°C',
          method: 'Calibrated thermometer',
          frequency: 'Daily'
        }
      ],
      releaseTestingRequired: ['Post-thaw survival', 'Fertilization rate', 'Embryo development'],
      expectedOutcomes: {
        minViability: 90, // >90% survival
        minRecovery: 90,
        maxAcceptableVariation: 10
      },
      validationRequirements: {
        minimumValidationRuns: 20,
        successCriteria: '≥90% survival rate, ≥70% fertilization rate (ICSI)',
        revalidationFrequency: 'Annual'
      }
    },

    safety: {
      personnelRequirements: {
        minimumTraining: [
          'Oocyte handling',
          'Vitrification technique',
          'ICSI procedures',
          'LN2 safety'
        ],
        certificationRequired: true,
        experienceLevel: 'EXPERT'
      },
      personalProtectiveEquipment: [
        'Cryogenic gloves',
        'Face shield',
        'Lab coat',
        'Closed-toe shoes'
      ],
      hazards: [
        {
          type: 'CRYOGENIC',
          description: 'Liquid nitrogen exposure, frostbite risk',
          severity: 'HIGH',
          mitigation: 'Use PPE, work quickly but carefully, maintain LN2 level'
        },
        {
          type: 'CHEMICAL',
          description: 'DMSO and EG exposure',
          severity: 'MODERATE',
          mitigation: 'Work in hood, avoid skin contact, proper disposal'
        },
        {
          type: 'BIOLOGICAL',
          description: 'Human gametes - potential pathogen transmission',
          severity: 'MODERATE',
          mitigation: 'Universal precautions, separate storage for infectious samples'
        }
      ]
    },

    references: {
      publications: [
        {
          title: 'Oocyte cryopreservation: a review of current practices',
          authors: ['Kuwayama M'],
          journal: 'Reproductive BioMedicine Online',
          year: 2007,
          doi: '10.1016/S1472-6483(10)60804-4'
        }
      ],
      guidelines: ['ASRM Practice Committee', 'ESHRE Good Practice Guidelines']
    }
  } as any;
}
```

---

## Protocol Execution Tracker

```typescript
/**
 * Real-time protocol execution tracking system
 */

export class ProtocolExecutionTracker {
  private activeExecutions: Map<string, ProtocolExecutionContext> = new Map();

  /**
   * Start tracking protocol execution
   */
  startTracking(context: ProtocolExecutionContext): void {
    this.activeExecutions.set(context.specimenId, context);
  }

  /**
   * Update execution progress
   */
  updateProgress(specimenId: string, stepUpdate: Partial<ProtocolExecutionStep>): void {
    const context = this.activeExecutions.get(specimenId);
    if (!context) {
      throw new Error(`No active execution for specimen: ${specimenId}`);
    }

    const currentStep = context.steps[context.currentStep];
    if (!currentStep) {
      throw new Error('No current step to update');
    }

    Object.assign(currentStep, stepUpdate);
  }

  /**
   * Get real-time execution status
   */
  getExecutionStatus(specimenId: string): {
    status: string;
    currentStepName: string;
    elapsedTime: number; // minutes
    estimatedCompletion: Date;
    completionPercentage: number;
  } {
    const context = this.activeExecutions.get(specimenId);
    if (!context) {
      throw new Error(`No active execution for specimen: ${specimenId}`);
    }

    const currentStep = context.steps[context.currentStep];
    const elapsedTime = (Date.now() - context.startTime.getTime()) / 60000; // minutes

    const remainingSteps = context.steps.slice(context.currentStep);
    const estimatedRemainingTime = remainingSteps.reduce(
      (sum, step) => sum + step.duration,
      0
    );

    const estimatedCompletion = new Date(Date.now() + estimatedRemainingTime * 60000);

    const completedSteps = context.steps.filter(s => s.completed).length;
    const completionPercentage = (completedSteps / context.steps.length) * 100;

    return {
      status: context.status,
      currentStepName: currentStep?.name || 'Completed',
      elapsedTime: Math.round(elapsedTime),
      estimatedCompletion,
      completionPercentage: Math.round(completionPercentage)
    };
  }

  /**
   * Generate execution report
   */
  generateExecutionReport(specimenId: string): string {
    const context = this.activeExecutions.get(specimenId);
    if (!context) {
      throw new Error(`No execution context for specimen: ${specimenId}`);
    }

    const status = this.getExecutionStatus(specimenId);
    const protocol = context.protocolId;

    let report = `
Protocol Execution Report
=========================
Specimen ID: ${specimenId}
Protocol ID: ${protocol}
Performed By: ${context.performedBy}
Start Time: ${context.startTime.toISOString()}
Status: ${context.status}

Current Progress: ${status.completionPercentage}%
Current Step: ${status.currentStepName}
Elapsed Time: ${status.elapsedTime} minutes
${context.status === 'IN_PROGRESS' ? `Estimated Completion: ${status.estimatedCompletion.toISOString()}` : ''}

Steps:
------
`;

    context.steps.forEach((step, index) => {
      const status = step.completed ? '✓' : index === context.currentStep ? '→' : ' ';
      report += `${status} ${step.stepNumber}. ${step.name} (${step.duration} min)\n`;

      if (step.completed) {
        report += `   Completed: ${step.completedAt?.toISOString()}\n`;
        if (step.notes) {
          report += `   Notes: ${step.notes}\n`;
        }
        if (step.deviations && step.deviations.length > 0) {
          report += `   ⚠️ Deviations: ${step.deviations.join(', ')}\n`;
        }
      }
    });

    if (context.status === 'COMPLETED') {
      const totalTime = (context.completedAt!.getTime() - context.startTime.getTime()) / 60000;
      report += `\nTotal Execution Time: ${Math.round(totalTime)} minutes\n`;
    }

    return report;
  }
}
```

---

## Summary

This chapter provides comprehensive protocol management for cryopreservation:

- **Protocol Repository**: Centralized system for protocol storage and retrieval
- **Protocol Validation**: Automated validation of protocol completeness and safety
- **Slow Freeze Protocols**: Detailed sperm slow freeze protocol with step-by-step instructions
- **Vitrification Protocols**: Ultra-rapid oocyte vitrification protocol (Cryotop method)
- **Execution Tracking**: Real-time protocol execution monitoring and reporting

Key Features:
- Specimen-specific protocol selection
- Detailed cryoprotectant formulations
- Quality control expectations
- Safety requirements and PPE
- Step-by-step execution guidance
- Real-time progress tracking
- Deviation documentation

---

**弘益人間 (Benefit All Humanity)**

*Standardized preservation protocols ensure consistent quality, maximize specimen viability, and advance reproductive medicine globally.*
