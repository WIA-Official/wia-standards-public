/**
 * WIA-BIO-019: Bio-Banking SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive bio-banking functionality including:
 * - Sample registration and tracking
 * - Quality assessment and integrity scoring
 * - Chain of custody management
 * - Inventory queries
 * - Storage management
 */

import {
  Sample,
  SampleType,
  StorageCondition,
  SampleStatus,
  DonorInfo,
  QualityMetrics,
  IntegrityScoreParams,
  IntegrityResult,
  StorageQualityParams,
  CustodyEvent,
  CustodyAction,
  ChainOfCustody,
  Consent,
  SampleRequest,
  RequestStatus,
  InventoryQuery,
  InventorySummary,
  StorageEquipment,
  BIO_CONSTANTS,
  BioErrorCode,
  BioBankingError,
  StorageLocation,
  Aliquot,
  TemperatureExcursion,
} from './types';

import * as crypto from 'crypto';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-019 Bio-Banking SDK
 */
export class BioBankingSDK {
  private version = '1.0.0';
  private samples: Map<string, Sample> = new Map();
  private custodyChains: Map<string, ChainOfCustody> = new Map();
  private consents: Map<string, Consent> = new Map();

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Register a new sample in the biobank
   *
   * @param sampleData - Sample information
   * @returns Registered sample
   */
  registerSample(sampleData: Partial<Sample> & {
    id: string;
    type: SampleType;
    donor: DonorInfo;
    volume: number;
    volumeUnit: string;
    storageCondition: StorageCondition;
  }): Sample {
    // Validate required fields
    if (!sampleData.id) {
      throw new BioBankingError(
        BioErrorCode.INVALID_SAMPLE_ID,
        'Sample ID is required'
      );
    }

    if (this.samples.has(sampleData.id)) {
      throw new BioBankingError(
        BioErrorCode.INVALID_SAMPLE_ID,
        `Sample ${sampleData.id} already exists`
      );
    }

    // Validate consent if provided
    if (sampleData.donor.consentId) {
      const consent = this.consents.get(sampleData.donor.consentId);
      if (consent && !consent.isValid) {
        throw new BioBankingError(
          BioErrorCode.CONSENT_EXPIRED,
          `Consent ${sampleData.donor.consentId} is not valid`
        );
      }
    }

    const now = new Date();

    const sample: Sample = {
      id: sampleData.id,
      type: sampleData.type,
      donor: sampleData.donor,
      collectionDate: sampleData.collectionDate || now,
      volume: sampleData.volume,
      volumeUnit: sampleData.volumeUnit,
      storageCondition: sampleData.storageCondition,
      location: sampleData.location || {
        freezer: 'UNASSIGNED',
        fullLocation: 'UNASSIGNED',
      },
      status: SampleStatus.COLLECTED,
      quality: sampleData.quality,
      aliquots: [],
      riskGroup: sampleData.riskGroup,
      notes: sampleData.notes,
      createdAt: now,
      updatedAt: now,
    };

    this.samples.set(sample.id, sample);

    // Initialize chain of custody
    this.initializeChainOfCustody(sample.id, 'SYSTEM', 'BIOBANK');

    return sample;
  }

  /**
   * Retrieve a sample by ID
   *
   * @param sampleId - Sample identifier
   * @returns Sample if found
   */
  getSample(sampleId: string): Sample {
    const sample = this.samples.get(sampleId);
    if (!sample) {
      throw new BioBankingError(
        BioErrorCode.SAMPLE_NOT_FOUND,
        `Sample ${sampleId} not found`
      );
    }
    return sample;
  }

  /**
   * Update sample information
   *
   * @param sampleId - Sample identifier
   * @param updates - Fields to update
   * @returns Updated sample
   */
  updateSample(sampleId: string, updates: Partial<Sample>): Sample {
    const sample = this.getSample(sampleId);

    const updatedSample: Sample = {
      ...sample,
      ...updates,
      id: sample.id, // Prevent ID change
      updatedAt: new Date(),
    };

    this.samples.set(sampleId, updatedSample);

    return updatedSample;
  }

  /**
   * Calculate Sample Integrity Score
   *
   * @param params - Integrity calculation parameters
   * @returns Integrity result
   */
  calculateSampleIntegrity(params: IntegrityScoreParams): IntegrityResult {
    const {
      initialViability,
      currentViability,
      tempDeviation,
      maxTempDeviation,
      storageTime,
      maxStorageTime,
    } = params;

    // Validate inputs
    if (
      initialViability < 0 ||
      initialViability > 1 ||
      currentViability < 0 ||
      currentViability > 1
    ) {
      throw new BioBankingError(
        BioErrorCode.QUALITY_BELOW_THRESHOLD,
        'Viability must be between 0 and 1'
      );
    }

    // Calculate factors
    const viabilityFactor = currentViability / initialViability;
    const temperatureFactor = Math.max(0, 1 - tempDeviation / maxTempDeviation);
    const timeFactor = Math.max(0, 1 - storageTime / maxStorageTime);

    // Calculate SIS
    const score = viabilityFactor * temperatureFactor * timeFactor;

    // Determine interpretation
    let interpretation: IntegrityResult['interpretation'];
    let recommendation: string;

    if (score > 0.9) {
      interpretation = 'excellent';
      recommendation = 'Sample is in excellent condition. Suitable for all applications.';
    } else if (score >= 0.8) {
      interpretation = 'good';
      recommendation = 'Sample is in good condition. Suitable for most applications.';
    } else if (score >= 0.7) {
      interpretation = 'acceptable';
      recommendation =
        'Sample quality is acceptable. Verify suitability for intended use.';
    } else if (score >= 0.6) {
      interpretation = 'marginal';
      recommendation =
        'Sample quality is marginal. Use with caution. Not recommended for sensitive assays.';
    } else {
      interpretation = 'poor';
      recommendation =
        'Sample quality is poor. Consider discarding or use only for non-critical purposes.';
    }

    return {
      score,
      interpretation,
      recommendation,
      details: {
        viabilityFactor,
        temperatureFactor,
        timeFactor,
      },
    };
  }

  /**
   * Calculate Storage Quality Index using Arrhenius equation
   *
   * @param params - Storage quality parameters
   * @returns Storage Quality Index (0-1)
   */
  calculateStorageQuality(params: StorageQualityParams): number {
    const { sampleType, storageTime, storageTemp } = params;

    // Get degradation parameters for sample type
    const degradParams = BIO_CONSTANTS.DEGRADATION_PARAMS[sampleType];
    if (!degradParams) {
      // Default parameters
      const k = params.degradationRate || 0.01;
      const Ea = params.activationEnergy || 70;
      const R = BIO_CONSTANTS.GAS_CONSTANT;

      const exponent = -k * storageTime * Math.exp((Ea * 1000) / (R * storageTemp));
      return Math.exp(exponent);
    }

    const { k, Ea } = degradParams;
    const R = BIO_CONSTANTS.GAS_CONSTANT;

    // SQI = exp(-k × t × exp(Ea/RT))
    const exponent = -k * storageTime * Math.exp((Ea * 1000) / (R * storageTemp));
    const sqi = Math.exp(exponent);

    return Math.max(0, Math.min(1, sqi));
  }

  /**
   * Create an aliquot from a parent sample
   *
   * @param parentSampleId - Parent sample ID
   * @param aliquotVolume - Volume for aliquot
   * @param aliquotId - Optional aliquot ID
   * @returns Created aliquot
   */
  createAliquot(
    parentSampleId: string,
    aliquotVolume: number,
    aliquotId?: string
  ): Aliquot {
    const parent = this.getSample(parentSampleId);

    // Check sufficient volume
    if (parent.volume < aliquotVolume) {
      throw new BioBankingError(
        BioErrorCode.INSUFFICIENT_VOLUME,
        `Insufficient volume in sample ${parentSampleId}. Available: ${parent.volume} ${parent.volumeUnit}, Requested: ${aliquotVolume} ${parent.volumeUnit}`
      );
    }

    const id = aliquotId || `${parentSampleId}-A${(parent.aliquots?.length || 0) + 1}`;

    const aliquot: Aliquot = {
      id,
      parentSampleId,
      volume: aliquotVolume,
      volumeUnit: parent.volumeUnit,
      location: { ...parent.location },
      createdDate: new Date(),
      status: SampleStatus.STORED,
    };

    // Update parent sample
    parent.volume -= aliquotVolume;
    parent.aliquots = parent.aliquots || [];
    parent.aliquots.push(aliquot);
    parent.updatedAt = new Date();

    this.samples.set(parentSampleId, parent);

    // Add custody event
    this.addCustodyEvent(parentSampleId, {
      action: CustodyAction.ALIQUOTING,
      operator: 'SYSTEM',
      location: parent.location.fullLocation || 'BIOBANK',
      condition: {
        volume: parent.volume,
      },
      notes: `Created aliquot ${id} with volume ${aliquotVolume} ${parent.volumeUnit}`,
    });

    return aliquot;
  }

  /**
   * Initialize chain of custody for a sample
   */
  private initializeChainOfCustody(
    sampleId: string,
    operator: string,
    location: string
  ): void {
    const firstEvent = this.createCustodyEvent({
      sampleId,
      action: CustodyAction.COLLECTION,
      operator,
      location,
      condition: {},
    });

    const chain: ChainOfCustody = {
      sampleId,
      events: [firstEvent],
      intact: true,
      currentCustodian: operator,
    };

    this.custodyChains.set(sampleId, chain);
  }

  /**
   * Add custody event to sample chain
   *
   * @param sampleId - Sample ID
   * @param eventData - Event data
   */
  addCustodyEvent(
    sampleId: string,
    eventData: Partial<CustodyEvent> & {
      action: CustodyAction;
      operator: string;
      location: string;
    }
  ): CustodyEvent {
    const chain = this.custodyChains.get(sampleId);
    if (!chain) {
      throw new BioBankingError(
        BioErrorCode.CHAIN_OF_CUSTODY_BROKEN,
        `Chain of custody not found for sample ${sampleId}`
      );
    }

    const event = this.createCustodyEvent({
      sampleId,
      ...eventData,
      previousHash:
        chain.events.length > 0
          ? chain.events[chain.events.length - 1].hash
          : undefined,
    });

    chain.events.push(event);
    chain.currentCustodian = eventData.operator;
    chain.verifiedAt = new Date();

    this.custodyChains.set(sampleId, chain);

    return event;
  }

  /**
   * Create a custody event with hash
   */
  private createCustodyEvent(
    data: Partial<CustodyEvent> & {
      sampleId: string;
      action: CustodyAction;
      operator: string;
      location: string;
    }
  ): CustodyEvent {
    const timestamp = new Date();
    const id = `EVT-${timestamp.getTime()}-${Math.random().toString(36).substr(2, 9)}`;

    const event: CustodyEvent = {
      id,
      timestamp,
      action: data.action,
      operator: data.operator,
      location: data.location,
      sampleId: data.sampleId,
      condition: data.condition,
      previousHash: data.previousHash,
      hash: '', // Will be calculated
      notes: data.notes,
    };

    // Calculate hash (excluding hash field itself)
    event.hash = this.calculateEventHash(event);

    return event;
  }

  /**
   * Calculate SHA-256 hash for custody event
   */
  private calculateEventHash(event: CustodyEvent): string {
    const data = {
      id: event.id,
      timestamp: event.timestamp.toISOString(),
      action: event.action,
      operator: event.operator,
      location: event.location,
      sampleId: event.sampleId,
      condition: event.condition,
      previousHash: event.previousHash,
      notes: event.notes,
    };

    return crypto.createHash('sha256').update(JSON.stringify(data)).digest('hex');
  }

  /**
   * Verify chain of custody integrity
   *
   * @param sampleId - Sample ID
   * @returns True if chain is intact
   */
  verifyChainOfCustody(sampleId: string): boolean {
    const chain = this.custodyChains.get(sampleId);
    if (!chain) {
      return false;
    }

    // Verify each event's hash
    for (let i = 0; i < chain.events.length; i++) {
      const event = chain.events[i];
      const calculatedHash = this.calculateEventHash(event);

      if (calculatedHash !== event.hash) {
        chain.intact = false;
        return false;
      }

      // Verify hash chain linkage
      if (i > 0) {
        const previousHash = chain.events[i - 1].hash;
        if (event.previousHash !== previousHash) {
          chain.intact = false;
          return false;
        }
      }
    }

    chain.intact = true;
    return true;
  }

  /**
   * Get chain of custody for a sample
   *
   * @param sampleId - Sample ID
   * @returns Chain of custody
   */
  getChainOfCustody(sampleId: string): ChainOfCustody {
    const chain = this.custodyChains.get(sampleId);
    if (!chain) {
      throw new BioBankingError(
        BioErrorCode.CHAIN_OF_CUSTODY_BROKEN,
        `Chain of custody not found for sample ${sampleId}`
      );
    }
    return chain;
  }

  /**
   * Register donor consent
   *
   * @param consent - Consent information
   */
  registerConsent(consent: Consent): void {
    this.consents.set(consent.consentId, consent);
  }

  /**
   * Validate consent for sample use
   *
   * @param consentId - Consent ID
   * @param purpose - Intended use purpose
   * @returns True if consent is valid for purpose
   */
  validateConsent(consentId: string, purpose: string): boolean {
    const consent = this.consents.get(consentId);
    if (!consent) {
      throw new BioBankingError(
        BioErrorCode.CONSENT_EXPIRED,
        `Consent ${consentId} not found`
      );
    }

    // Check if consent is valid
    if (!consent.isValid) {
      return false;
    }

    // Check if withdrawn
    if (consent.withdrawalDate) {
      return false;
    }

    // Check expiry
    if (consent.expiryDate && new Date() > consent.expiryDate) {
      return false;
    }

    // Check if purpose is allowed
    if (consent.purposes.length > 0) {
      const purposeAllowed = consent.purposes.some(
        (p) => p.toLowerCase() === purpose.toLowerCase() || p === 'any'
      );
      if (!purposeAllowed) {
        return false;
      }
    }

    return true;
  }

  /**
   * Query inventory with filters
   *
   * @param query - Query parameters
   * @returns Matching samples
   */
  queryInventory(query: InventoryQuery): Sample[] {
    let results = Array.from(this.samples.values());

    // Apply filters
    if (query.sampleType) {
      results = results.filter((s) => s.type === query.sampleType);
    }

    if (query.storageCondition) {
      results = results.filter((s) => s.storageCondition === query.storageCondition);
    }

    if (query.minVolume !== undefined) {
      results = results.filter((s) => s.volume >= query.minVolume!);
    }

    if (query.availableOnly) {
      results = results.filter((s) => s.status !== SampleStatus.DEPLETED);
    }

    if (query.collectionDateRange) {
      results = results.filter(
        (s) =>
          s.collectionDate >= query.collectionDateRange!.from &&
          s.collectionDate <= query.collectionDateRange!.to
      );
    }

    if (query.minQualityScore !== undefined) {
      results = results.filter(
        (s) =>
          s.quality?.integrityScore !== undefined &&
          s.quality.integrityScore >= query.minQualityScore!
      );
    }

    if (query.donorAgeRange) {
      results = results.filter(
        (s) =>
          s.donor.age !== undefined &&
          s.donor.age >= query.donorAgeRange!.min &&
          s.donor.age <= query.donorAgeRange!.max
      );
    }

    if (query.donorSex) {
      results = results.filter((s) => s.donor.sex === query.donorSex);
    }

    // Pagination
    const offset = query.offset || 0;
    const limit = query.limit || results.length;
    results = results.slice(offset, offset + limit);

    return results;
  }

  /**
   * Get inventory summary
   *
   * @returns Summary statistics
   */
  getInventorySummary(): InventorySummary {
    const samples = Array.from(this.samples.values());

    const byType: Record<SampleType, number> = {} as any;
    const byStorage: Record<StorageCondition, number> = {} as any;

    let totalVolume = 0;
    let availableSamples = 0;
    let depletedSamples = 0;

    samples.forEach((sample) => {
      // Count by type
      byType[sample.type] = (byType[sample.type] || 0) + 1;

      // Count by storage
      byStorage[sample.storageCondition] =
        (byStorage[sample.storageCondition] || 0) + 1;

      // Sum volume
      totalVolume += sample.volume;

      // Count status
      if (sample.status === SampleStatus.DEPLETED) {
        depletedSamples++;
      } else {
        availableSamples++;
      }
    });

    return {
      totalSamples: samples.length,
      availableSamples,
      depletedSamples,
      byType,
      byStorage,
      totalVolume: {
        value: totalVolume,
        unit: 'mL', // Assuming mL as default
      },
      lastUpdated: new Date(),
    };
  }

  /**
   * Record temperature excursion
   *
   * @param sampleId - Sample ID
   * @param excursion - Temperature excursion data
   */
  recordTemperatureExcursion(
    sampleId: string,
    excursion: Omit<TemperatureExcursion, 'id'>
  ): void {
    const sample = this.getSample(sampleId);

    const excursionWithId: TemperatureExcursion = {
      ...excursion,
      id: `EXC-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
    };

    if (!sample.quality) {
      sample.quality = {};
    }

    if (!sample.quality.temperatureExcursions) {
      sample.quality.temperatureExcursions = [];
    }

    sample.quality.temperatureExcursions.push(excursionWithId);
    sample.updatedAt = new Date();

    this.samples.set(sampleId, sample);

    // Add custody event
    this.addCustodyEvent(sampleId, {
      action: CustodyAction.QUALITY_CHECK,
      operator: 'SYSTEM',
      location: sample.location.fullLocation || 'BIOBANK',
      notes: `Temperature excursion recorded: ${excursion.severity} severity, ${excursion.duration} minutes`,
    });
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate Sample Integrity Score (standalone function)
 */
export function calculateSampleIntegrity(
  params: IntegrityScoreParams
): IntegrityResult {
  const sdk = new BioBankingSDK();
  return sdk.calculateSampleIntegrity(params);
}

/**
 * Calculate Storage Quality Index (standalone function)
 */
export function calculateStorageQuality(params: StorageQualityParams): number {
  const sdk = new BioBankingSDK();
  return sdk.calculateStorageQuality(params);
}

/**
 * Generate unique sample ID
 */
export function generateSampleId(prefix: string = 'SAM'): string {
  const timestamp = Date.now();
  const random = Math.random().toString(36).substr(2, 9).toUpperCase();
  return `${prefix}-${timestamp}-${random}`;
}

/**
 * Convert temperature from Celsius to Kelvin
 */
export function celsiusToKelvin(celsius: number): number {
  return celsius + 273.15;
}

/**
 * Convert temperature from Kelvin to Celsius
 */
export function kelvinToCelsius(kelvin: number): number {
  return kelvin - 273.15;
}

/**
 * Format volume for display
 */
export function formatVolume(volume: number, unit: string): string {
  if (volume < 0.001) {
    return `${(volume * 1000000).toFixed(2)} μ${unit}`;
  } else if (volume < 1) {
    return `${(volume * 1000).toFixed(2)} m${unit}`;
  } else if (volume < 1000) {
    return `${volume.toFixed(2)} ${unit}`;
  } else {
    return `${(volume / 1000).toFixed(2)} L`;
  }
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BioBankingSDK };
export default BioBankingSDK;
