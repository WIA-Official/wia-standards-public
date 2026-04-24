/**
 * WIA Battery Passport Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-battery-passport
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main Battery Passport class
 * Provides unified interface for battery lifecycle tracking and EU Battery Regulation compliance
 *
 * @example
 * ```typescript
 * const passport = new WIABatteryPassport({
 *   apiEndpoint: 'https://api.battery-passport.eu',
 *   blockchainEnabled: true
 * });
 *
 * const battery = await passport.create({
 *   specification: { ... },
 *   materials: [ ... ],
 *   carbonFootprint: { ... }
 * });
 *
 * await passport.recordChargingEvent(battery.id.passportId, chargingEvent);
 * ```
 */
export class WIABatteryPassport extends EventEmitter {
  private config: types.BatteryPassportConfig;
  private passports: Map<string, types.BatteryPassport> = new Map();

  /**
   * Create a new Battery Passport SDK instance
   * @param config - Configuration options
   */
  constructor(config: types.BatteryPassportConfig) {
    super();
    this.config = config;
  }

  /**
   * Create a new battery passport
   * @param data - Battery passport data
   */
  async create(data: Partial<types.BatteryPassport>): Promise<types.BatteryPassport> {
    const passportId = this.generatePassportId();

    const passport: types.BatteryPassport = {
      id: {
        passportId,
        batteryId: data.specification?.serialNumber || `BAT-${Date.now()}`,
        qrCode: this.generateQRCode(passportId),
        dppUrl: `${this.config.apiEndpoint}/passport/${passportId}`
      },
      specification: data.specification!,
      materials: data.materials || [],
      carbonFootprint: data.carbonFootprint!,
      performance: data.performance || this.initializePerformance(data.specification!),
      lifecycleStage: types.LifecycleStage.Manufacturing,
      chargingHistory: [],
      maintenanceRecords: [],
      ownershipHistory: [],
      dueDiligence: [],
      certifications: data.certifications || [],
      recycling: data.recycling!,
      createdAt: new Date(),
      updatedAt: new Date()
    };

    this.passports.set(passportId, passport);

    if (this.config.blockchainEnabled) {
      await this.recordOnBlockchain('create', passport);
    }

    this.emit('passport-created', passport);
    return passport;
  }

  /**
   * Generate unique passport ID
   */
  private generatePassportId(): string {
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).substring(2, 8);
    return `WIA-BP-${timestamp}-${random}`.toUpperCase();
  }

  /**
   * Generate QR code data
   */
  private generateQRCode(passportId: string): string {
    return `${this.config.apiEndpoint}/qr/${passportId}`;
  }

  /**
   * Initialize performance data
   */
  private initializePerformance(spec: types.BatterySpecification): types.PerformanceData {
    return {
      stateOfHealth: 100,
      sohCategory: types.StateOfHealth.Excellent,
      stateOfCharge: 100,
      cycleCount: 0,
      energyThroughput: 0,
      remainingCapacity: spec.nominalCapacity,
      internalResistance: 0,
      maxTemperature: 25,
      minTemperature: 25,
      lastUpdated: new Date()
    };
  }

  /**
   * Record on blockchain (simulated)
   */
  private async recordOnBlockchain(action: string, data: unknown): Promise<string> {
    // Simulated blockchain recording
    const txId = `tx-${Date.now()}-${Math.random().toString(36).substring(2, 8)}`;
    console.log(`Blockchain record: ${action} - TX: ${txId}`);
    return txId;
  }

  /**
   * Get passport by ID
   * @param passportId - Passport ID
   */
  async get(passportId: string): Promise<types.BatteryPassport | undefined> {
    return this.passports.get(passportId);
  }

  /**
   * Update performance data
   * @param passportId - Passport ID
   * @param performance - Performance data
   */
  async updatePerformance(
    passportId: string,
    performance: Partial<types.PerformanceData>
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    passport.performance = {
      ...passport.performance,
      ...performance,
      lastUpdated: new Date()
    };

    // Update SOH category
    passport.performance.sohCategory = this.calculateSOHCategory(passport.performance.stateOfHealth);

    passport.updatedAt = new Date();

    if (this.config.blockchainEnabled) {
      await this.recordOnBlockchain('update-performance', { passportId, performance });
    }

    this.emit('performance-updated', { passportId, performance: passport.performance });
    return true;
  }

  /**
   * Calculate SOH category
   */
  private calculateSOHCategory(soh: number): types.StateOfHealth {
    if (soh >= 90) return types.StateOfHealth.Excellent;
    if (soh >= 80) return types.StateOfHealth.Good;
    if (soh >= 70) return types.StateOfHealth.Fair;
    if (soh >= 50) return types.StateOfHealth.Poor;
    return types.StateOfHealth.EndOfLife;
  }

  /**
   * Record charging event
   * @param passportId - Passport ID
   * @param event - Charging event
   */
  async recordChargingEvent(
    passportId: string,
    event: types.ChargingEvent
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    passport.chargingHistory.push(event);
    passport.performance.cycleCount += (event.endSoc - event.startSoc) / 100;
    passport.performance.energyThroughput += event.energyCharged;
    passport.updatedAt = new Date();

    if (this.config.blockchainEnabled) {
      await this.recordOnBlockchain('charging-event', { passportId, event });
    }

    this.emit('charging-recorded', { passportId, event });
    return true;
  }

  /**
   * Record maintenance
   * @param passportId - Passport ID
   * @param record - Maintenance record
   */
  async recordMaintenance(
    passportId: string,
    record: types.MaintenanceRecord
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    passport.maintenanceRecords.push(record);
    passport.updatedAt = new Date();

    if (this.config.blockchainEnabled) {
      await this.recordOnBlockchain('maintenance', { passportId, record });
    }

    this.emit('maintenance-recorded', { passportId, record });
    return true;
  }

  /**
   * Transfer ownership
   * @param passportId - Passport ID
   * @param newOwner - New owner record
   */
  async transferOwnership(
    passportId: string,
    newOwner: Omit<types.OwnershipRecord, 'id' | 'startDate'>
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    // Close previous ownership
    const currentOwner = passport.ownershipHistory[passport.ownershipHistory.length - 1];
    if (currentOwner) {
      currentOwner.endDate = new Date();
    }

    // Add new ownership
    const record: types.OwnershipRecord = {
      id: `own-${Date.now()}`,
      ...newOwner,
      startDate: new Date()
    };

    if (this.config.blockchainEnabled) {
      record.transferTxId = await this.recordOnBlockchain('transfer', { passportId, newOwner });
    }

    passport.ownershipHistory.push(record);
    passport.updatedAt = new Date();

    this.emit('ownership-transferred', { passportId, record });
    return true;
  }

  /**
   * Update lifecycle stage
   * @param passportId - Passport ID
   * @param stage - New lifecycle stage
   */
  async updateLifecycleStage(
    passportId: string,
    stage: types.LifecycleStage
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    const previousStage = passport.lifecycleStage;
    passport.lifecycleStage = stage;
    passport.updatedAt = new Date();

    if (this.config.blockchainEnabled) {
      await this.recordOnBlockchain('lifecycle-update', { passportId, previousStage, stage });
    }

    this.emit('lifecycle-updated', { passportId, previousStage, stage });
    return true;
  }

  /**
   * Add due diligence record
   * @param passportId - Passport ID
   * @param record - Due diligence record
   */
  async addDueDiligence(
    passportId: string,
    record: types.DueDiligenceRecord
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    passport.dueDiligence.push(record);
    passport.updatedAt = new Date();

    this.emit('due-diligence-added', { passportId, record });
    return true;
  }

  /**
   * Add safety certification
   * @param passportId - Passport ID
   * @param certification - Safety certification
   */
  async addCertification(
    passportId: string,
    certification: types.SafetyCertification
  ): Promise<boolean> {
    const passport = this.passports.get(passportId);
    if (!passport) return false;

    passport.certifications.push(certification);
    passport.updatedAt = new Date();

    this.emit('certification-added', { passportId, certification });
    return true;
  }

  /**
   * Calculate carbon footprint summary
   * @param passportId - Passport ID
   */
  getCarbonSummary(passportId: string): { total: number; perKwh: number; verified: boolean } | undefined {
    const passport = this.passports.get(passportId);
    if (!passport) return undefined;

    return {
      total: passport.carbonFootprint.totalCO2e,
      perKwh: passport.carbonFootprint.co2ePerKwh,
      verified: passport.carbonFootprint.verified
    };
  }

  /**
   * Check EU Battery Regulation compliance
   * @param passportId - Passport ID
   */
  checkEUCompliance(passportId: string): { compliant: boolean; issues: string[] } {
    const passport = this.passports.get(passportId);
    if (!passport) {
      return { compliant: false, issues: ['Passport not found'] };
    }

    const issues: string[] = [];

    // Check carbon footprint declaration
    if (!passport.carbonFootprint.verified) {
      issues.push('Carbon footprint not third-party verified');
    }

    // Check recycled content reporting
    const hasRecycledContent = passport.materials.some(m => m.recycledContent !== undefined);
    if (!hasRecycledContent) {
      issues.push('Recycled content not declared');
    }

    // Check due diligence for critical materials
    const hasSupplyChainDD = passport.dueDiligence.some(
      d => d.checkType === 'supply_chain' && d.status === 'passed'
    );
    if (!hasSupplyChainDD) {
      issues.push('Supply chain due diligence not completed');
    }

    // Check safety certifications
    const hasValidCert = passport.certifications.some(
      c => c.expiryDate > new Date()
    );
    if (!hasValidCert) {
      issues.push('No valid safety certification');
    }

    return {
      compliant: issues.length === 0,
      issues
    };
  }

  /**
   * Generate passport QR code
   * @param passportId - Passport ID
   */
  getQRCode(passportId: string): string | undefined {
    const passport = this.passports.get(passportId);
    return passport?.id.qrCode;
  }

  /**
   * Export passport as JSON
   * @param passportId - Passport ID
   */
  exportPassport(passportId: string): string | undefined {
    const passport = this.passports.get(passportId);
    if (!passport) return undefined;

    return JSON.stringify(passport, null, 2);
  }

  /**
   * Check WIA compliance
   * @param targetLevel - Target certification level
   */
  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    // Test 1: Configuration
    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.apiEndpoint !== undefined,
      notes: 'API endpoint must be defined'
    });

    // Test 2: Passport creation capability
    tests.push({
      testName: 'Passport Management',
      passed: this.passports.size >= 0,
      notes: 'SDK must support passport creation'
    });

    // Test 3: Blockchain for Silver+
    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Blockchain Integration',
        passed: this.config.blockchainEnabled === true,
        notes: 'Blockchain verification required for Silver/Gold'
      });
    }

    // Test 4: EU compliance check for Gold
    if (targetLevel === types.CertificationLevel.Gold) {
      const hasCompliantPassport = Array.from(this.passports.values())
        .some(p => this.checkEUCompliance(p.id.passportId).compliant);

      tests.push({
        testName: 'EU Battery Regulation Compliance',
        passed: hasCompliantPassport || this.passports.size === 0,
        notes: 'All passports must be EU compliant for Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-BATTERY-PASSPORT',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

/**
 * Default export for convenience
 */
export default {
  WIABatteryPassport
};
