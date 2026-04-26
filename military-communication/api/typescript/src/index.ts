/**
 * WIA-DEF-016: Military Communication SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Communications Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for military communications including:
 * - Link budget calculations
 * - Frequency management
 * - Encryption and COMSEC
 * - Tactical data links
 * - Anti-jamming techniques
 * - Interoperability support
 */

import {
  LinkBudgetParams,
  LinkBudgetResult,
  EncryptionRequest,
  EncryptionResponse,
  DecryptionRequest,
  FrequencyAllocationRequest,
  FrequencyAllocationResponse,
  RadioConfig,
  RadioStatus,
  PositionReport,
  ContactReport,
  JammingReport,
  SATCOMLinkParams,
  SATCOMLinkStatus,
  RF_CONSTANTS,
  FREQUENCY_BANDS,
  CommErrorCode,
  MilitaryCommError,
  GeoCoordinate,
  FrequencyBand,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-016 Military Communication SDK
 */
export class MilitaryCommunicationSDK {
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
   * Calculate link budget for RF communications
   *
   * @param params - Link budget parameters
   * @returns Link budget calculation results
   */
  calculateLinkBudget(params: LinkBudgetParams): LinkBudgetResult {
    const {
      transmitPower,
      frequency,
      distance,
      txAntennaGain = 0,
      rxAntennaGain = 0,
      txHeight = 2,
      rxHeight = 2,
      terrain = 'open',
      additionalLosses = 0,
      atmosphericLoss = 0,
      rainLoss = 0,
    } = params;

    // Validate inputs
    if (transmitPower <= 0) {
      throw new MilitaryCommError(
        CommErrorCode.INSUFFICIENT_POWER,
        'Transmit power must be positive'
      );
    }

    if (frequency <= 0) {
      throw new MilitaryCommError(
        CommErrorCode.INVALID_FREQUENCY,
        'Frequency must be positive'
      );
    }

    // Convert power to dBW
    const ptxDbw = 10 * Math.log10(transmitPower);
    const ptxDbm = ptxDbw + 30;

    // Calculate free-space path loss (dB)
    // L_fs = 20 log10(d) + 20 log10(f) + 32.45
    // where d is in km and f is in MHz
    const distanceKm = distance / 1000;
    const pathLoss = 20 * Math.log10(distanceKm) + 20 * Math.log10(frequency) + 32.45;

    // Apply terrain-specific additional losses
    let terrainLoss = 0;
    switch (terrain) {
      case 'open':
        terrainLoss = 0;
        break;
      case 'urban':
        terrainLoss = 10;
        break;
      case 'forest':
        terrainLoss = 15;
        break;
      case 'mountainous':
        terrainLoss = 20;
        break;
      case 'desert':
        terrainLoss = 2;
        break;
    }

    // Total loss
    const totalLoss =
      pathLoss + terrainLoss + additionalLosses + atmosphericLoss + rainLoss;

    // Calculate received power (dBm)
    // P_rx = P_tx + G_tx - L_total + G_rx
    const receivedPower = ptxDbm + txAntennaGain - totalLoss + rxAntennaGain;

    // Calculate noise floor
    // Assume 10 kHz bandwidth and 8 dB noise figure
    const bandwidth = 10000; // Hz
    const noiseFigure = 8; // dB
    const noiseFloor =
      RF_CONSTANTS.THERMAL_NOISE_FLOOR + 10 * Math.log10(bandwidth) + noiseFigure;

    // Calculate SNR
    const snr = receivedPower - noiseFloor;

    // Calculate link margin (assuming required SNR of 10 dB)
    const requiredSnr = 10;
    const linkMargin = snr - requiredSnr;

    // Calculate maximum distance
    // Solve for distance where link margin = 0
    // maxLinkLoss = P_tx + G_tx + G_rx - (noiseFloor + requiredSnr)
    const maxLinkLoss = ptxDbm + txAntennaGain + rxAntennaGain - noiseFloor - requiredSnr;
    const maxPathLoss = maxLinkLoss - terrainLoss - additionalLosses - atmosphericLoss;

    // Solve: maxPathLoss = 20 log10(d_max) + 20 log10(f) + 32.45
    // d_max = 10^((maxPathLoss - 20 log10(f) - 32.45) / 20)
    const maxDistanceKm = Math.pow(
      10,
      (maxPathLoss - 20 * Math.log10(frequency) - 32.45) / 20
    );

    // Determine feasibility
    let feasibility: LinkBudgetResult['feasibility'];
    if (linkMargin >= 20) {
      feasibility = 'excellent';
    } else if (linkMargin >= 10) {
      feasibility = 'good';
    } else if (linkMargin >= 0) {
      feasibility = 'marginal';
    } else if (linkMargin >= -10) {
      feasibility = 'poor';
    } else {
      feasibility = 'impossible';
    }

    // Generate warnings
    const warnings: string[] = [];
    if (linkMargin < 10) {
      warnings.push('Link margin is low. Consider increasing transmit power or antenna gain.');
    }
    if (terrainLoss > 10) {
      warnings.push(`High terrain loss (${terrainLoss} dB). Line-of-sight may be obstructed.`);
    }
    if (rainLoss > 3) {
      warnings.push('Significant rain attenuation. Link may degrade in weather.');
    }
    if (distance > maxDistanceKm * 1000 * 0.9) {
      warnings.push('Operating near maximum range. Link reliability may be affected.');
    }

    return {
      receivedPower,
      pathLoss,
      totalLoss,
      linkMargin,
      maxDistance: maxDistanceKm,
      snr,
      feasibility,
      warnings,
    };
  }

  /**
   * Encrypt a message using military-grade encryption
   *
   * @param request - Encryption request
   * @returns Encrypted message
   */
  encryptMessage(request: EncryptionRequest): EncryptionResponse {
    const { message, encryptionLevel, algorithm, keyId, aad } = request;

    // Validate key
    // In a real implementation, this would retrieve the key from secure key storage
    if (!keyId) {
      throw new MilitaryCommError(
        CommErrorCode.ENCRYPTION_FAILED,
        'Key ID is required for encryption'
      );
    }

    // Generate random IV (16 bytes for AES)
    const iv = this.generateRandomBytes(16);
    const ivBase64 = Buffer.from(iv).toString('base64');

    // Simulate encryption (in real implementation, use crypto library)
    // For demonstration, we'll create a mock ciphertext
    const ciphertext = Buffer.from(
      `[ENCRYPTED:${algorithm}:${message.length}bytes]`
    ).toString('base64');

    // Generate auth tag for GCM mode
    let authTag: string | undefined;
    if (algorithm.includes('gcm')) {
      const tag = this.generateRandomBytes(16);
      authTag = Buffer.from(tag).toString('base64');
    }

    // Set expiration based on classification level
    const now = new Date();
    const expiration = new Date(now);
    switch (encryptionLevel) {
      case 'top-secret':
      case 'ts-sci':
        expiration.setHours(expiration.getHours() + 24); // 24 hours
        break;
      case 'secret':
        expiration.setDate(expiration.getDate() + 7); // 7 days
        break;
      default:
        expiration.setDate(expiration.getDate() + 30); // 30 days
    }

    return {
      ciphertext,
      iv: ivBase64,
      authTag,
      timestamp: now,
      expiration,
      algorithm,
      keyId,
    };
  }

  /**
   * Decrypt an encrypted message
   *
   * @param request - Decryption request
   * @returns Decrypted plaintext
   */
  decryptMessage(request: DecryptionRequest): string {
    const { ciphertext, iv, authTag, keyId, algorithm } = request;

    // Validate inputs
    if (!keyId) {
      throw new MilitaryCommError(
        CommErrorCode.ENCRYPTION_FAILED,
        'Key ID is required for decryption'
      );
    }

    // In real implementation, would perform actual decryption
    // For demonstration, return mock plaintext
    return '[DECRYPTED MESSAGE]';
  }

  /**
   * Validate and allocate frequency
   *
   * @param request - Frequency allocation request
   * @returns Allocation response
   */
  validateFrequency(request: FrequencyAllocationRequest): FrequencyAllocationResponse {
    const { frequency, bandwidth, location, purpose, priority } = request;

    const conflicts: any[] = [];
    const alternativeFrequencies: number[] = [];

    // Determine frequency band
    const band = this.getFrequencyBand(frequency);

    // Check if frequency is in authorized military band
    const isAuthorized = this.isFrequencyAuthorized(frequency, band);

    if (!isAuthorized) {
      return {
        isApproved: false,
        conflicts: [],
        alternativeFrequencies: this.getSuggestedFrequencies(band, bandwidth),
        recommendation:
          'Requested frequency is not in authorized military allocation. See alternatives.',
      };
    }

    // Simulate conflict detection
    // In real implementation, would check against frequency database
    const hasConflicts = Math.random() < 0.2; // 20% chance of conflict

    if (hasConflicts) {
      conflicts.push({
        frequency: frequency + 0.025,
        distance: 50000, // 50 km
        conflictingUnit: 'BRAVO-COMPANY',
        severity: 'moderate',
        recommendation: 'Coordinate with conflicting unit or use alternative frequency',
      });
    }

    const isApproved = !hasConflicts || priority === 'flash';

    return {
      isApproved,
      allocatedFrequency: isApproved ? frequency : undefined,
      allocationId: isApproved ? `FREQ-${Date.now()}` : undefined,
      conflicts,
      alternativeFrequencies:
        conflicts.length > 0 ? this.getSuggestedFrequencies(band, bandwidth) : undefined,
      recommendation: isApproved
        ? 'Frequency approved for use'
        : 'Conflicts detected. Use alternative frequency or coordinate with units.',
    };
  }

  /**
   * Establish a secure communication link
   *
   * @param config - Radio configuration
   * @returns Radio status
   */
  establishSecureLink(config: RadioConfig): RadioStatus {
    const { radioId, frequency, power, encryptionEnabled } = config;

    // Validate encryption requirement
    if (!encryptionEnabled) {
      throw new MilitaryCommError(
        CommErrorCode.UNAUTHORIZED_ACCESS,
        'Encryption is required for all military communications'
      );
    }

    // Simulate link establishment
    const status: RadioStatus = {
      radioId,
      status: 'operational',
      currentFrequency: frequency,
      rssi: -75 + Math.random() * 20, // -75 to -55 dBm
      snr: 20 + Math.random() * 10, // 20-30 dB
      ber: Math.random() * 1e-6, // Very low BER
      batteryLevel: 80 + Math.random() * 20,
      lastUpdate: new Date(),
      activeConnections: 1,
      jammingDetected: false,
    };

    return status;
  }

  /**
   * Detect jamming on communication link
   *
   * @param radioStatus - Current radio status
   * @returns Jamming report if detected, null otherwise
   */
  detectJamming(radioStatus: RadioStatus): JammingReport | null {
    const { rssi, snr, currentFrequency, radioId } = radioStatus;

    // Simple jamming detection: SNR drops significantly
    const isJammed = snr < 10 && rssi > -60;

    if (!isJammed) {
      return null;
    }

    // Determine jamming type based on characteristics
    let jammingType: JammingReport['jammingType'];
    if (rssi > -40) {
      jammingType = 'barrage'; // High-power across band
    } else if (rssi > -50) {
      jammingType = 'spot'; // Targeted jamming
    } else {
      jammingType = 'wideband';
    }

    const jsr = rssi - (rssi - 10 * Math.log10(snr));

    return {
      detectionId: `JAM-${Date.now()}`,
      frequency: currentFrequency,
      jammingType,
      jammingPower: rssi,
      jammerToSignalRatio: jsr,
      timestamp: new Date(),
      affectedSystems: [radioId],
      countermeasures: ['frequency-hopping', 'power-increase', 'directional-antenna'],
      impact: jsr > 20 ? 'severe' : jsr > 10 ? 'moderate' : 'minor',
    };
  }

  /**
   * Calculate SATCOM link budget
   *
   * @param params - SATCOM link parameters
   * @returns Link status
   */
  calculateSATCOMLink(params: SATCOMLinkParams): SATCOMLinkStatus {
    const {
      terminalType,
      satelliteId,
      uplinkFrequency,
      downlinkFrequency,
      antennaDiameter,
      transmitPower,
      dataRate,
    } = params;

    // Calculate antenna gain (parabolic dish)
    // G = η × (π × D × f / c)²
    // Assume 60% efficiency
    const efficiency = 0.6;
    const wavelength =
      (RF_CONSTANTS.SPEED_OF_LIGHT / (downlinkFrequency * 1e6)) * 100; // cm
    const antennaGain =
      efficiency * Math.pow((Math.PI * antennaDiameter * 100) / wavelength, 2);
    const antennaGainDb = 10 * Math.log10(antennaGain);

    // Calculate free-space loss to GEO (36,000 km)
    const distance = 36000; // km
    const pathLoss =
      20 * Math.log10(distance) + 20 * Math.log10(downlinkFrequency) + 32.45;

    // Satellite EIRP (typical for WGS)
    const satelliteEirp = 50; // dBW

    // Received power
    const rxPower = satelliteEirp - pathLoss + antennaGainDb - 2; // -2 dB atmospheric

    // Noise temperature
    const systemTemp = 150; // K (clear sky)
    const noisePower =
      10 * Math.log10(RF_CONSTANTS.BOLTZMANN * systemTemp * dataRate * 1000) + 30; // dBm

    // C/N ratio
    const cnr = rxPower - noisePower;

    // Calculate Eb/N0
    const ebno = cnr + 10 * Math.log10(dataRate * 1000);

    // Simulate satellite pointing
    const elevation = 30 + Math.random() * 50; // 30-80 degrees
    const azimuth = Math.random() * 360;

    return {
      linkId: `SAT-${Date.now()}`,
      status: cnr > 10 ? 'locked' : cnr > 5 ? 'tracking' : 'acquiring',
      cnr,
      ber: cnr > 15 ? 1e-7 : cnr > 10 ? 1e-5 : 1e-3,
      ebno,
      elevation,
      azimuth,
      rainMargin: cnr - 10, // Margin above threshold
      lastUpdate: new Date(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Generate random bytes for IV and keys
   */
  private generateRandomBytes(length: number): Uint8Array {
    const bytes = new Uint8Array(length);
    for (let i = 0; i < length; i++) {
      bytes[i] = Math.floor(Math.random() * 256);
    }
    return bytes;
  }

  /**
   * Determine frequency band from frequency value
   */
  private getFrequencyBand(frequency: number): FrequencyBand {
    if (frequency >= FREQUENCY_BANDS.HF.min && frequency < FREQUENCY_BANDS.HF.max) {
      return 'hf';
    } else if (frequency >= FREQUENCY_BANDS.VHF.min && frequency < FREQUENCY_BANDS.VHF.max) {
      return 'vhf';
    } else if (frequency >= FREQUENCY_BANDS.UHF.min && frequency < FREQUENCY_BANDS.UHF.max) {
      return 'uhf';
    } else if (
      frequency >= FREQUENCY_BANDS.L_BAND.min &&
      frequency < FREQUENCY_BANDS.L_BAND.max
    ) {
      return 'l-band';
    } else if (
      frequency >= FREQUENCY_BANDS.S_BAND.min &&
      frequency < FREQUENCY_BANDS.S_BAND.max
    ) {
      return 's-band';
    } else if (
      frequency >= FREQUENCY_BANDS.X_BAND.min &&
      frequency < FREQUENCY_BANDS.X_BAND.max
    ) {
      return 'x-band';
    } else if (
      frequency >= FREQUENCY_BANDS.KU_BAND.min &&
      frequency < FREQUENCY_BANDS.KU_BAND.max
    ) {
      return 'ku-band';
    } else if (
      frequency >= FREQUENCY_BANDS.KA_BAND.min &&
      frequency < FREQUENCY_BANDS.KA_BAND.max
    ) {
      return 'ka-band';
    } else {
      return 'ehf';
    }
  }

  /**
   * Check if frequency is in authorized military allocation
   */
  private isFrequencyAuthorized(frequency: number, band: FrequencyBand): boolean {
    // Simplified authorization check
    // In real implementation, would check against frequency allocation table
    const authorizedBands: FrequencyBand[] = ['hf', 'vhf', 'uhf', 'x-band', 'ka-band'];
    return authorizedBands.includes(band);
  }

  /**
   * Get suggested alternative frequencies
   */
  private getSuggestedFrequencies(band: FrequencyBand, bandwidth: number): number[] {
    const bandDef = FREQUENCY_BANDS[band.toUpperCase().replace('-', '_') as keyof typeof FREQUENCY_BANDS];
    const alternatives: number[] = [];

    // Suggest 3 frequencies within band
    for (let i = 0; i < 3; i++) {
      const freq =
        bandDef.min + (Math.random() * (bandDef.max - bandDef.min - bandwidth / 1000));
      alternatives.push(Math.round(freq * 1000) / 1000);
    }

    return alternatives.sort((a, b) => a - b);
  }

  /**
   * Calculate radio horizon distance
   */
  calculateRadioHorizon(antennaHeight: number): number {
    // Radio horizon: d = K × √(2 × R × h)
    // where K = 4/3 (atmospheric refraction factor)
    // R = Earth radius (6371 km)
    // h = antenna height (meters)
    const K = RF_CONSTANTS.K_FACTOR;
    const R = RF_CONSTANTS.EARTH_RADIUS * 1000; // Convert to meters
    const horizon = K * Math.sqrt(2 * R * antennaHeight);
    return horizon / 1000; // Convert to km
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate link budget (standalone function)
 */
export function calculateLinkBudget(params: LinkBudgetParams): LinkBudgetResult {
  const sdk = new MilitaryCommunicationSDK();
  return sdk.calculateLinkBudget(params);
}

/**
 * Encrypt message (standalone function)
 */
export function encryptMessage(request: EncryptionRequest): EncryptionResponse {
  const sdk = new MilitaryCommunicationSDK();
  return sdk.encryptMessage(request);
}

/**
 * Validate frequency (standalone function)
 */
export function validateFrequency(
  request: FrequencyAllocationRequest
): FrequencyAllocationResponse {
  const sdk = new MilitaryCommunicationSDK();
  return sdk.validateFrequency(request);
}

/**
 * Establish secure link (standalone function)
 */
export function establishSecureLink(config: RadioConfig): RadioStatus {
  const sdk = new MilitaryCommunicationSDK();
  return sdk.establishSecureLink(config);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MilitaryCommunicationSDK };
export default MilitaryCommunicationSDK;
