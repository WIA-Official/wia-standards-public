/**
 * WIA-DEF-013: NBC Defense SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for NBC/CBRN defense including:
 * - Agent detection and identification
 * - Threat level assessment
 * - Protection requirements calculation
 * - Decontamination planning
 * - Medical countermeasure management
 * - Emergency response coordination
 */

import {
  CBRNDetectionRequest,
  CBRNDetectionResponse,
  ThreatAssessmentRequest,
  ThreatAssessmentResponse,
  ProtectionRequest,
  ProtectionResponse,
  DecontaminationRequest,
  DecontaminationPlan,
  CountermeasureRequest,
  CountermeasureResponse,
  EmergencyResponseRequest,
  EmergencyResponsePlan,
  CBRNIncident,
  CBRN_CONSTANTS,
  NBCErrorCode,
  NBCDefenseError,
  AgentType,
  ThreatLevel,
  MOPPLevel,
  PPEItem,
  WorkRestCycle,
  DeconSupply,
  DeconStation,
  MedicalCountermeasure,
  Action,
  ResponsePhase,
  ChemicalAgentType,
  BiologicalAgentType,
  RadiationIsotope,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-013 NBC Defense SDK
 */
export class NBCDefenseSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Detect and identify CBRN agent from sensor data
   *
   * @param request - Detection request with sensor data
   * @returns Detection response with agent identification
   */
  detectCBRNAgent(request: CBRNDetectionRequest): CBRNDetectionResponse {
    const { sensorData, location, timestamp } = request;

    // Validate inputs
    if (!sensorData || sensorData.length === 0) {
      throw new NBCDefenseError(
        NBCErrorCode.SENSOR_MALFUNCTION,
        'No sensor data provided'
      );
    }

    // Analyze sensor readings
    const primarySensor = sensorData[0];
    const avgValue = sensorData.reduce((sum, s) => sum + s.value, 0) / sensorData.length;
    const avgConfidence = sensorData.reduce((sum, s) => sum + s.confidence, 0) / sensorData.length;

    // Determine agent type based on sensor type and value
    let agentType: AgentType;
    let agentSubtype: string | undefined;
    let concentration: number;
    let unit: string;
    let threatLevel: ThreatLevel;

    switch (primarySensor.sensorType) {
      case 'chemical':
        agentType = 'chemical';
        concentration = avgValue;
        unit = 'mg/m³';

        // Determine chemical subtype based on concentration patterns
        if (avgValue > 0.1) {
          agentSubtype = 'nerve-agent-v';
          threatLevel = 5;
        } else if (avgValue > 0.01) {
          agentSubtype = 'nerve-agent-g';
          threatLevel = 4;
        } else if (avgValue > 1) {
          agentSubtype = 'blister-sulfur';
          threatLevel = 3;
        } else {
          agentSubtype = 'riot-control';
          threatLevel = 2;
        }
        break;

      case 'biological':
        agentType = 'biological';
        concentration = avgValue;
        unit = 'CFU/m³';

        if (avgValue > 1000) {
          agentSubtype = 'bacteria-anthrax';
          threatLevel = 5;
        } else if (avgValue > 100) {
          agentSubtype = 'bacteria-plague';
          threatLevel = 4;
        } else {
          agentSubtype = 'bacteria-tularemia';
          threatLevel = 3;
        }
        break;

      case 'radiological':
        agentType = 'radiological';
        concentration = avgValue;
        unit = 'R/hr';

        if (avgValue > 100) {
          threatLevel = 5;
        } else if (avgValue > 10) {
          threatLevel = 4;
        } else if (avgValue > 1) {
          threatLevel = 3;
        } else if (avgValue > 0.1) {
          threatLevel = 2;
        } else {
          threatLevel = 1;
        }
        agentSubtype = 'cesium-137';
        break;

      default:
        agentType = 'chemical';
        concentration = avgValue;
        unit = 'mg/m³';
        threatLevel = 2;
    }

    // Generate recommendations
    const recommendations: string[] = [];

    if (threatLevel >= 4) {
      recommendations.push('IMMEDIATE EVACUATION REQUIRED');
      recommendations.push('Don MOPP 4 immediately');
      recommendations.push('Activate emergency response plan');
    } else if (threatLevel >= 3) {
      recommendations.push('Don MOPP 3 protective equipment');
      recommendations.push('Begin decontamination procedures');
      recommendations.push('Request medical support');
    } else if (threatLevel >= 2) {
      recommendations.push('Increase monitoring frequency');
      recommendations.push('Prepare MOPP 2 equipment');
      recommendations.push('Alert response teams');
    } else {
      recommendations.push('Continue monitoring');
      recommendations.push('Maintain readiness MOPP 1');
    }

    return {
      agentType,
      agentSubtype: agentSubtype as ChemicalAgentType | BiologicalAgentType | RadiationIsotope,
      concentration,
      unit: unit as 'mg/m³' | 'Bq/m³' | 'R/hr' | 'mSv/hr' | 'CFU/m³',
      confidence: avgConfidence,
      threatLevel,
      detectionMethod: primarySensor.sensorType,
      detectionTime: timestamp,
      recommendations,
    };
  }

  /**
   * Assess threat level from CBRN agent
   *
   * @param request - Threat assessment parameters
   * @returns Threat assessment with casualty estimates
   */
  assessThreatLevel(request: ThreatAssessmentRequest): ThreatAssessmentResponse {
    const { agentType, concentration, weatherData, population } = request;

    // Calculate affected area based on wind and concentration
    const windSpeed = weatherData.windSpeed || 10;
    const dispersionFactor = Math.sqrt(concentration * windSpeed);
    const affectedArea = Math.min(dispersionFactor * 0.1, 100); // km²

    // Estimate casualties based on agent type and population density
    let casualtyRate: number;
    let fatalityRate: number;

    if (agentType === 'chemical') {
      if (concentration > 0.1) {
        casualtyRate = 0.8;
        fatalityRate = 0.5;
      } else if (concentration > 0.01) {
        casualtyRate = 0.5;
        fatalityRate = 0.2;
      } else {
        casualtyRate = 0.2;
        fatalityRate = 0.05;
      }
    } else if (agentType === 'biological') {
      casualtyRate = 0.6;
      fatalityRate = 0.3;
    } else if (agentType === 'radiological' || agentType === 'nuclear') {
      if (concentration > 100) {
        casualtyRate = 0.9;
        fatalityRate = 0.7;
      } else {
        casualtyRate = 0.4;
        fatalityRate = 0.1;
      }
    } else {
      casualtyRate = 0.1;
      fatalityRate = 0.01;
    }

    const populationDensity = population / 100; // people per km²
    const exposedPopulation = Math.min(affectedArea * populationDensity, population);
    const estimatedCasualties = Math.round(exposedPopulation * casualtyRate);
    const estimatedFatalities = Math.round(estimatedCasualties * fatalityRate);

    // Determine threat level
    let level: ThreatLevel;
    if (estimatedCasualties > 1000) {
      level = 5;
    } else if (estimatedCasualties > 100) {
      level = 4;
    } else if (estimatedCasualties > 20) {
      level = 3;
    } else if (estimatedCasualties > 5) {
      level = 2;
    } else {
      level = 1;
    }

    // Time to impact (based on wind speed and distance)
    const timeToImpact = (affectedArea / windSpeed) * 3600; // seconds

    // Generate recommended actions
    const recommendedActions: Action[] = [];

    if (level >= 4) {
      recommendedActions.push({
        priority: 'immediate',
        type: 'evacuate',
        description: 'Evacuate all personnel within affected area',
        affectedPopulation: exposedPopulation,
        estimatedDuration: 3600,
      });
    }

    recommendedActions.push({
      priority: 'immediate',
      type: 'don-ppe',
      description: `Don MOPP ${Math.min(level + 1, 4)} protective equipment`,
      estimatedDuration: 300,
    });

    if (level >= 3) {
      recommendedActions.push({
        priority: 'urgent',
        type: 'decontaminate',
        description: 'Establish decontamination corridors',
        estimatedDuration: 1800,
      });

      recommendedActions.push({
        priority: 'urgent',
        type: 'medical',
        description: 'Dispense medical countermeasures',
        affectedPopulation: estimatedCasualties,
        estimatedDuration: 7200,
      });
    }

    const description = `${level === 5 ? 'CATASTROPHIC' : level === 4 ? 'SEVERE' : level === 3 ? 'SUBSTANTIAL' : level === 2 ? 'MODERATE' : 'LOW'} threat - ${estimatedCasualties} estimated casualties`;

    return {
      level,
      description,
      affectedArea,
      estimatedCasualties,
      estimatedFatalities,
      timeToImpact,
      recommendedActions,
    };
  }

  /**
   * Calculate protection requirements for CBRN environment
   *
   * @param request - Protection requirement parameters
   * @returns Protection requirements including MOPP level and PPE
   */
  calculateProtectionRequired(request: ProtectionRequest): ProtectionResponse {
    const { agentType, concentration, duration, personnel } = request;

    // Determine MOPP level
    let moppLevel: MOPPLevel;
    if (concentration > 0.1 || agentType === 'nuclear') {
      moppLevel = 4;
    } else if (concentration > 0.01) {
      moppLevel = 3;
    } else if (concentration > 0.001) {
      moppLevel = 2;
    } else {
      moppLevel = 1;
    }

    // Build PPE list
    const ppeRequired: PPEItem[] = [];

    if (moppLevel >= 1) {
      ppeRequired.push({
        type: 'suit',
        model: 'JSLIST',
        protectionLevel: 'C',
        protectedAgents: ['chemical', 'biological'],
        duration: 86400, // 24 hours
      });
    }

    if (moppLevel >= 2) {
      ppeRequired.push({
        type: 'boots',
        model: 'Chemical Protective Overboots',
        protectionLevel: 'C',
        protectedAgents: ['chemical'],
        duration: 86400,
      });
    }

    if (moppLevel >= 3) {
      ppeRequired.push({
        type: 'mask',
        model: 'M50 JSGPM',
        protectionLevel: 'B',
        protectedAgents: ['chemical', 'biological', 'radiological'],
        duration: 1296000, // 15 days
      });
    }

    if (moppLevel >= 4) {
      ppeRequired.push({
        type: 'gloves',
        model: 'Butyl Rubber Gloves',
        protectionLevel: 'C',
        protectedAgents: ['chemical'],
        duration: 86400,
      });
    }

    // Calculate work/rest cycle (assuming 30°C / 86°F)
    const workRestCycle: WorkRestCycle = this.calculateWorkRestCycle(30, moppLevel);

    // Maximum exposure time
    const maxExposureTime = this.calculateMaxExposure(agentType, concentration);

    // Respiratory protection
    const respiratoryProtection = {
      type: 'apr' as const,
      filter: 'C2A1 CBRN',
      protectionFactor: 1000,
      duration: 1296000, // 15 days
    };

    // Heat stress risk
    let heatStressRisk: 'low' | 'moderate' | 'high' | 'extreme';
    if (moppLevel === 4) {
      heatStressRisk = 'high';
    } else if (moppLevel === 3) {
      heatStressRisk = 'moderate';
    } else {
      heatStressRisk = 'low';
    }

    const precautions = [
      'Monitor for heat stress symptoms',
      'Maintain hydration',
      'Use buddy system',
      'Limit exposure time',
    ];

    if (agentType === 'chemical') {
      precautions.push('Avoid skin contact');
      precautions.push('Decontaminate immediately after exposure');
    }

    return {
      moppLevel,
      ppeRequired,
      workRestCycle,
      maxExposureTime,
      respiratoryProtection,
      heatStressRisk,
      precautions,
    };
  }

  /**
   * Plan decontamination operation
   *
   * @param request - Decontamination planning parameters
   * @returns Decontamination plan with timeline and resources
   */
  planDecontamination(request: DecontaminationRequest): DecontaminationPlan {
    const { affectedPersonnel, contaminant, area, priority = 'operational' } = request;

    // Determine decon type based on priority
    const type = priority;

    // Calculate time and throughput
    let timePerPerson: number;
    let throughput: number;

    switch (type) {
      case 'immediate':
        timePerPerson = 3; // minutes
        throughput = 20;   // people per hour
        break;
      case 'operational':
        timePerPerson = 8;
        throughput = 8;
        break;
      case 'thorough':
        timePerPerson = 25;
        throughput = 2.4;
        break;
    }

    const estimatedTime = Math.ceil(affectedPersonnel / throughput * 60); // minutes

    // Required supplies
    const requiredSupplies: DeconSupply[] = [
      {
        item: 'Water',
        quantity: affectedPersonnel * 50,
        unit: 'liters',
        priority: 'critical',
      },
      {
        item: '0.5% Bleach Solution',
        quantity: affectedPersonnel * 5,
        unit: 'liters',
        priority: 'essential',
      },
      {
        item: 'Soap/Detergent',
        quantity: Math.ceil(affectedPersonnel / 10),
        unit: 'bottles',
        priority: 'essential',
      },
      {
        item: 'Towels',
        quantity: affectedPersonnel,
        unit: 'each',
        priority: 'essential',
      },
      {
        item: 'Plastic Bags (waste)',
        quantity: affectedPersonnel * 2,
        unit: 'each',
        priority: 'critical',
      },
    ];

    if (type === 'thorough') {
      requiredSupplies.push({
        item: 'Clean Clothing Sets',
        quantity: affectedPersonnel,
        unit: 'sets',
        priority: 'essential',
      });
    }

    // Personnel requirements (1 staff per 10 victims)
    const personnelRequired = Math.ceil(affectedPersonnel / 10) + 5; // +5 for support

    // Decon corridor stations
    const corridorLayout: DeconStation[] = [
      {
        stationNumber: 1,
        type: 'triage',
        description: 'Initial assessment and prioritization',
        timePerPerson: 30,
        personnelRequired: 2,
        equipment: ['Triage tags', 'Assessment forms'],
      },
      {
        stationNumber: 2,
        type: 'disrobe',
        description: 'Remove contaminated clothing',
        timePerPerson: 60,
        personnelRequired: 3,
        equipment: ['Privacy screens', 'Plastic bags', 'Scissors'],
      },
      {
        stationNumber: 3,
        type: 'wash',
        description: 'Wash with soap and water',
        timePerPerson: 180,
        personnelRequired: 4,
        equipment: ['Shower heads', 'Soap', 'Brushes'],
      },
      {
        stationNumber: 4,
        type: 'rinse',
        description: 'Rinse with clean water',
        timePerPerson: 120,
        personnelRequired: 2,
        equipment: ['Shower heads', 'Clean water'],
      },
      {
        stationNumber: 5,
        type: 'dress',
        description: 'Provide clean clothing',
        timePerPerson: 60,
        personnelRequired: 2,
        equipment: ['Clean clothes', 'Blankets'],
      },
      {
        stationNumber: 6,
        type: 'medical',
        description: 'Medical assessment and treatment',
        timePerPerson: 300,
        personnelRequired: 3,
        equipment: ['Medical supplies', 'Antidotes', 'Monitoring equipment'],
      },
    ];

    // Waste management
    const wasteManagement = {
      wasteType: 'mixed' as const,
      estimatedVolume: affectedPersonnel * 15, // liters
      containment: '55-gallon drums with lids',
      disposal: 'Hazardous waste contractor per EPA regulations',
      specialHandling: [
        'Double-bag all contaminated items',
        'Label with agent type and date',
        'Store in designated area',
      ],
    };

    // Special considerations
    const specialConsiderations: string[] = [];

    if (contaminant.includes('nerve')) {
      specialConsiderations.push('Administer Mark I kits as needed');
      specialConsiderations.push('Monitor for cholinergic symptoms');
    }

    if (contaminant.includes('blister')) {
      specialConsiderations.push('Avoid breaking skin blisters');
      specialConsiderations.push('Apply skin decon within 1 minute');
    }

    if (area === 'urban') {
      specialConsiderations.push('Coordinate with local authorities');
      specialConsiderations.push('Manage public information');
    }

    return {
      planId: `DECON-${Date.now()}`,
      type,
      estimatedTime,
      throughput,
      requiredSupplies,
      personnelRequired,
      corridorLayout,
      wasteManagement,
      specialConsiderations,
    };
  }

  /**
   * Dispense medical countermeasures for CBRN exposure
   *
   * @param request - Countermeasure dispensing parameters
   * @returns Countermeasure plan with dosing and administration
   */
  dispenseCountermeasures(request: CountermeasureRequest): CountermeasureResponse {
    const { agent, population, severity } = request;

    let primary: MedicalCountermeasure;
    const alternatives: MedicalCountermeasure[] = [];

    // Determine appropriate countermeasure based on agent
    if (agent.includes('nerve')) {
      primary = {
        name: 'MARK I Kit',
        genericName: 'Atropine + 2-PAM Chloride',
        type: 'antidote',
        dosage: '2mg atropine + 600mg 2-PAM',
        route: 'IM',
        frequency: severity === 'severe' ? 'Every 5-10 minutes (max 3)' : 'Once',
        duration: 'Until symptoms resolve',
        efficacy: 0.9,
      };

      alternatives.push({
        name: 'CANA',
        genericName: 'Diazepam',
        type: 'antidote',
        dosage: '10mg',
        route: 'IM',
        frequency: 'Once (for seizures)',
        duration: 'Single dose',
        efficacy: 0.85,
      });
    } else if (agent.includes('anthrax')) {
      primary = {
        name: 'Ciprofloxacin',
        genericName: 'Ciprofloxacin',
        type: 'antibiotic',
        dosage: '500mg',
        route: 'PO',
        frequency: 'Twice daily',
        duration: '60 days',
        efficacy: 0.95,
      };

      alternatives.push({
        name: 'Doxycycline',
        type: 'antibiotic',
        dosage: '100mg',
        route: 'PO',
        frequency: 'Twice daily',
        duration: '60 days',
        efficacy: 0.93,
      });
    } else if (agent.includes('cesium') || agent.includes('iodine')) {
      primary = {
        name: 'Potassium Iodide',
        genericName: 'KI',
        type: 'antidote',
        dosage: '130mg',
        route: 'PO',
        frequency: 'Once daily',
        duration: 'Until exposure risk passes',
        efficacy: 0.98,
      };
    } else {
      primary = {
        name: 'Supportive Care',
        type: 'supportive',
        dosage: 'As needed',
        route: 'IV',
        frequency: 'Continuous',
        duration: 'Until recovery',
        efficacy: 0.6,
      };
    }

    // Calculate required quantity
    const requiredQuantity = severity === 'severe' ? population * 3 : population;

    // Administration protocol
    const administrationProtocol = {
      steps: [
        'Verify agent exposure',
        'Assess severity of symptoms',
        'Administer appropriate dose',
        'Monitor vital signs',
        'Document administration',
      ],
      timing: 'As soon as possible after exposure',
      monitoring: [
        'Heart rate',
        'Blood pressure',
        'Respiratory rate',
        'Pupil size',
        'Level of consciousness',
      ],
      sideEffects: [
        'Dry mouth',
        'Blurred vision',
        'Tachycardia',
        'Urinary retention',
      ],
      escalationCriteria: [
        'Symptoms worsen despite treatment',
        'Severe adverse reactions',
        'Respiratory distress',
        'Altered mental status',
      ],
    };

    // Expected outcomes
    const expectedOutcomes = [
      'Reduction in cholinergic symptoms within 10-20 minutes',
      'Improved breathing and reduced secretions',
      'Prevention of seizures',
      'Full recovery within 24-48 hours',
    ];

    // Stockpile status
    const stockpileStatus = requiredQuantity > 10000 ? 'insufficient' : 'adequate';

    return {
      primary,
      alternatives: alternatives.length > 0 ? alternatives : undefined,
      requiredQuantity,
      administrationProtocol,
      expectedOutcomes,
      stockpileStatus: stockpileStatus as 'adequate' | 'limited' | 'insufficient' | 'depleted',
    };
  }

  /**
   * Generate emergency response plan for CBRN scenario
   *
   * @param request - Emergency response parameters
   * @returns Comprehensive response plan
   */
  generateEmergencyResponse(request: EmergencyResponseRequest): EmergencyResponsePlan {
    const { scenario, location, expectedThreat, population } = request;

    // Incident command structure
    const incidentCommand = {
      commander: 'On-Scene Incident Commander',
      safetyOfficer: 'Safety Officer',
      publicInfoOfficer: 'Public Information Officer',
      operations: 'Operations Section Chief',
      planning: 'Planning Section Chief',
      logistics: 'Logistics Section Chief',
      financeAdmin: 'Finance/Admin Section Chief',
    };

    // Response phases
    const phases: ResponsePhase[] = [
      {
        phaseNumber: 1,
        name: 'Recognition and Notification',
        description: 'Detect incident, activate alarms, notify emergency services',
        startTime: 0,
        duration: 5,
        keyActivities: [
          'Detect CBRN incident',
          'Sound alarms',
          'Notify 911/emergency services',
          'Activate emergency response plan',
        ],
        successCriteria: [
          'All personnel notified within 5 minutes',
          'Emergency services dispatched',
        ],
      },
      {
        phaseNumber: 2,
        name: 'Initial Response',
        description: 'Establish command, secure scene, rescue casualties',
        startTime: 5,
        duration: 25,
        keyActivities: [
          'Establish incident command',
          'Define hot/warm/cold zones',
          'Don PPE (MOPP 4)',
          'Rescue exposed personnel',
          'Begin immediate decontamination',
        ],
        successCriteria: [
          'Incident command post established',
          'Zones clearly marked',
          'All casualties removed from hot zone',
        ],
      },
      {
        phaseNumber: 3,
        name: 'Extended Response',
        description: 'Agent identification, mass decon, medical treatment',
        startTime: 30,
        duration: 690,
        keyActivities: [
          'Laboratory agent identification',
          'Establish mass decon corridors',
          'Dispense medical countermeasures',
          'Transport casualties to hospitals',
          'Evidence collection',
        ],
        successCriteria: [
          'Agent identified',
          'All exposed personnel decontaminated',
          'Critical casualties stabilized',
        ],
      },
      {
        phaseNumber: 4,
        name: 'Recovery',
        description: 'Area decon, environmental monitoring, investigation',
        startTime: 720,
        duration: 10080, // 7 days
        keyActivities: [
          'Area decontamination',
          'Environmental sampling',
          'Victim follow-up',
          'Criminal/terrorism investigation',
          'After-action review',
        ],
        successCriteria: [
          'Area cleared for reoccupation',
          'All victims accounted for',
          'Investigation complete',
        ],
      },
    ];

    // Resource requirements
    const resourceRequirements = [
      {
        type: 'CBRN Detection Equipment',
        quantity: 5,
        priority: 'critical' as const,
        whenNeeded: 0,
      },
      {
        type: 'MOPP 4 Protective Suits',
        quantity: Math.ceil(population / 100),
        priority: 'critical' as const,
        whenNeeded: 5,
      },
      {
        type: 'Decontamination Tents',
        quantity: Math.ceil(population / 500),
        priority: 'essential' as const,
        whenNeeded: 30,
      },
      {
        type: 'Medical Countermeasures',
        quantity: population,
        priority: 'critical' as const,
        whenNeeded: 30,
      },
      {
        type: 'Ambulances',
        quantity: Math.ceil(population * 0.1 / 2),
        priority: 'essential' as const,
        whenNeeded: 60,
      },
    ];

    // Contingencies
    const contingencies = [
      {
        trigger: 'Agent unknown after 1 hour',
        action: 'Request state/federal lab support',
        resources: ['Advanced detection equipment', 'Mobile lab'],
        timelineImpact: 120,
      },
      {
        trigger: 'Mass casualties exceed local capacity',
        action: 'Request mutual aid, activate regional response',
        resources: ['Additional ambulances', 'Field hospitals', 'Medical teams'],
        timelineImpact: 180,
      },
      {
        trigger: 'Countermeasures depleted',
        action: 'Request Strategic National Stockpile',
        resources: ['SNS Push Package'],
        timelineImpact: 720,
      },
    ];

    return {
      planId: `ERP-${Date.now()}`,
      incidentCommand,
      phases,
      resourceRequirements,
      estimatedTimeline: phases.reduce((sum, p) => sum + p.duration, 0),
      successCriteria: [
        'All exposed personnel decontaminated',
        'Zero secondary contamination',
        'All casualties receive medical care',
        'Area cleared for reoccupation',
        'Public confidence restored',
      ],
      contingencies,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate work/rest cycle based on temperature and MOPP level
   */
  private calculateWorkRestCycle(temperature: number, moppLevel: MOPPLevel): WorkRestCycle {
    let workDuration: number;
    let restDuration: number;
    let fluidIntake: number;

    if (moppLevel === 4) {
      if (temperature < 24) {
        workDuration = 3600;
        restDuration = 900;
        fluidIntake = 1.0;
      } else if (temperature < 29) {
        workDuration = 2400;
        restDuration = 1200;
        fluidIntake = 1.5;
      } else if (temperature < 32) {
        workDuration = 1800;
        restDuration = 1800;
        fluidIntake = 2.0;
      } else {
        workDuration = 1200;
        restDuration = 2400;
        fluidIntake = 2.5;
      }
    } else {
      workDuration = 7200;
      restDuration = 900;
      fluidIntake = 1.0;
    }

    return {
      workDuration,
      restDuration,
      fluidIntake,
      temperature,
      moppLevel,
    };
  }

  /**
   * Calculate maximum safe exposure time
   */
  private calculateMaxExposure(agentType: AgentType, concentration: number): number {
    if (agentType === 'chemical') {
      if (concentration > 0.1) {
        return 300; // 5 minutes
      } else if (concentration > 0.01) {
        return 1800; // 30 minutes
      } else {
        return 7200; // 2 hours
      }
    } else if (agentType === 'radiological' || agentType === 'nuclear') {
      // Based on dose limit of 25 rem for emergency operations
      return Math.floor(25 / concentration * 3600);
    } else {
      return 3600; // 1 hour default
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Detect CBRN agent (standalone function)
 */
export function detectCBRNAgent(request: CBRNDetectionRequest): CBRNDetectionResponse {
  const sdk = new NBCDefenseSDK();
  return sdk.detectCBRNAgent(request);
}

/**
 * Assess threat level (standalone function)
 */
export function assessThreatLevel(request: ThreatAssessmentRequest): ThreatAssessmentResponse {
  const sdk = new NBCDefenseSDK();
  return sdk.assessThreatLevel(request);
}

/**
 * Calculate protection requirements (standalone function)
 */
export function calculateProtectionRequired(request: ProtectionRequest): ProtectionResponse {
  const sdk = new NBCDefenseSDK();
  return sdk.calculateProtectionRequired(request);
}

/**
 * Plan decontamination (standalone function)
 */
export function planDecontamination(request: DecontaminationRequest): DecontaminationPlan {
  const sdk = new NBCDefenseSDK();
  return sdk.planDecontamination(request);
}

/**
 * Dispense countermeasures (standalone function)
 */
export function dispenseCountermeasures(request: CountermeasureRequest): CountermeasureResponse {
  const sdk = new NBCDefenseSDK();
  return sdk.dispenseCountermeasures(request);
}

/**
 * Generate emergency response plan (standalone function)
 */
export function generateEmergencyResponse(request: EmergencyResponseRequest): EmergencyResponsePlan {
  const sdk = new NBCDefenseSDK();
  return sdk.generateEmergencyResponse(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { NBCDefenseSDK };
export default NBCDefenseSDK;
