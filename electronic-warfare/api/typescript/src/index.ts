/**
 * WIA-DEF-006: Electronic Warfare SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for electronic warfare including:
 * - Jamming power calculations
 * - Signal intelligence analysis
 * - Spectrum management
 * - Countermeasure generation
 * - Safety compliance
 */

import {
  JammingParameters,
  JammingResult,
  SIGINTParameters,
  SIGINTResult,
  FrequencyHoppingConfig,
  SpreadSpectrumParams,
  DetectedSignal,
  SpectrumScanResult,
  FrequencyRange,
  RFSafetyParams,
  DirectionFindingResult,
  GeographicLocation,
  EW_CONSTANTS,
  FREQUENCY_BANDS,
  EWErrorCode,
  EWError,
  ModulationType,
  SignalType,
  FrequencyBand,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-006 Electronic Warfare SDK
 */
export class ElectronicWarfareSDK {
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
   * Calculate jamming power requirements
   *
   * @param params - Jamming parameters
   * @returns Jamming calculation results
   */
  calculateJammingPower(params: JammingParameters): JammingResult {
    const {
      targetFrequency,
      targetDistance,
      targetPower,
      requiredJSRatio,
      jammingType,
      jammerGain = 10, // dBi
      targetGain = 3,  // dBi
      targetBandwidth = 1e6, // 1 MHz default
    } = params;

    // Validate inputs
    if (targetFrequency <= 0) {
      throw new EWError(
        EWErrorCode.INVALID_PARAMETERS,
        'Target frequency must be positive'
      );
    }

    if (targetDistance <= 0) {
      throw new EWError(
        EWErrorCode.INVALID_PARAMETERS,
        'Target distance must be positive'
      );
    }

    // Convert dBi to linear
    const jammerGainLinear = Math.pow(10, jammerGain / 10);
    const targetGainLinear = Math.pow(10, targetGain / 10);
    const jsRatioLinear = Math.pow(10, requiredJSRatio / 10);

    // Assume jammer is half the distance to target (self-screening)
    const jammerDistance = targetDistance / 2;

    // Calculate required jammer power
    // J/S = (Pj × Gj) / (Ps × Gs) × (Rs/Rj)²
    // Pj = J/S × Ps × Gs × (Rj/Rs)² / Gj
    const distanceRatio = jammerDistance / targetDistance;
    const jammerPower = jsRatioLinear * targetPower * targetGainLinear *
      Math.pow(distanceRatio, 2) / jammerGainLinear;

    // Calculate wavelength
    const wavelength = EW_CONSTANTS.SPEED_OF_LIGHT / targetFrequency;

    // Calculate effective range (burn-through range)
    // Range where J/S drops to minimum effective level
    const minJS = jammingType === 'deception'
      ? EW_CONSTANTS.MIN_JS_DECEPTION
      : EW_CONSTANTS.MIN_JS_NOISE;
    const minJSLinear = Math.pow(10, minJS / 10);

    const effectiveRange = Math.sqrt(
      (jammerPower * jammerGainLinear) /
      (targetPower * targetGainLinear * minJSLinear)
    ) * targetDistance;

    // Calculate burn-through range (where signal overcomes jamming)
    const burnThroughRange = Math.pow(
      (targetPower * Math.pow(targetGainLinear, 2) * Math.pow(wavelength, 2)) /
      (64 * Math.pow(Math.PI, 3) * jammerPower * jammerGainLinear * jsRatioLinear),
      0.25
    );

    // Power consumption estimate (assumes 50% efficiency)
    const powerConsumption = jammerPower / 0.5;

    // Effectiveness assessment
    const effectiveness = Math.min(1.0, jammerPower / (targetPower * 10));

    // Feasibility assessment
    let feasibility: JammingResult['feasibility'];
    if (jammerPower < 1000) {
      feasibility = 'possible';
    } else if (jammerPower < 10000) {
      feasibility = 'difficult';
    } else {
      feasibility = 'impossible';
    }

    // Generate warnings
    const warnings: string[] = [];
    if (jammerPower > 1000) {
      warnings.push('High power requirement may be impractical');
    }
    if (effectiveRange < targetDistance * 1.5) {
      warnings.push('Limited effective range');
    }
    if (burnThroughRange < targetDistance) {
      warnings.push('Target may burn through jamming');
    }

    // Achieved J/S ratio
    const achievedJSRatio = 10 * Math.log10(
      (jammerPower * jammerGainLinear) /
      (targetPower * targetGainLinear * Math.pow(distanceRatio, 2))
    );

    return {
      jammerPower,
      effectiveRange,
      achievedJSRatio,
      burnThroughRange,
      powerConsumption,
      effectiveness,
      feasibility,
      warnings,
    };
  }

  /**
   * Analyze signal intelligence data
   *
   * @param params - SIGINT parameters
   * @returns Signal intelligence results
   */
  analyzeSignalIntelligence(params: SIGINTParameters): SIGINTResult {
    const {
      frequencyRange,
      duration,
      rbw,
      threshold,
      classification = true,
    } = params;

    // Simulate signal detection and classification
    // In real implementation, this would process actual signal samples

    const frequency = (frequencyRange.start + frequencyRange.end) / 2;
    const bandwidth = frequencyRange.end - frequencyRange.start;

    // Classify signal based on frequency and bandwidth
    let modulation: ModulationType = 'unknown' as ModulationType;
    let signalType: SignalType = 'unknown';
    let confidence = 0.0;

    if (classification) {
      // Simple classification based on bandwidth
      if (bandwidth < 10000) {
        modulation = 'FM';
        signalType = 'communication';
        confidence = 0.7;
      } else if (bandwidth < 1000000) {
        modulation = 'OFDM';
        signalType = 'communication';
        confidence = 0.8;
      } else if (bandwidth > 10000000) {
        modulation = 'chirp' as ModulationType;
        signalType = 'radar';
        confidence = 0.85;
      } else {
        modulation = 'QAM';
        signalType = 'datalink';
        confidence = 0.6;
      }
    }

    // Estimate power based on frequency and distance
    const power = -80; // dBm (simulated)

    return {
      id: `SIGINT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      frequency,
      bandwidth,
      power,
      modulation,
      signalType,
      confidence,
    };
  }

  /**
   * Generate frequency hopping pattern
   *
   * @param channels - Number of frequency channels
   * @param dwellTime - Dwell time per channel in milliseconds
   * @param baseFreq - Base frequency in Hz
   * @param channelSpacing - Channel spacing in Hz
   * @returns Frequency hopping configuration
   */
  generateFrequencyHopping(
    channels: number,
    dwellTime: number,
    baseFreq: number = 2400e6,
    channelSpacing: number = 1e6
  ): FrequencyHoppingConfig {
    if (channels < 2) {
      throw new EWError(
        EWErrorCode.INVALID_PARAMETERS,
        'Must have at least 2 channels for frequency hopping'
      );
    }

    // Generate channel frequencies
    const channelFreqs: number[] = [];
    for (let i = 0; i < channels; i++) {
      channelFreqs.push(baseFreq + i * channelSpacing);
    }

    // Generate pseudo-random hopping sequence
    const sequence: number[] = [];
    const used = new Set<number>();

    while (sequence.length < channels) {
      const idx = Math.floor(Math.random() * channels);
      if (!used.has(idx)) {
        sequence.push(idx);
        used.add(idx);
      }
    }

    // Calculate hop rate
    const hopRate = 1000 / dwellTime; // hops per second

    // Calculate processing gain
    // PG = 10 log(BW_ss / BW_info)
    const spreadBandwidth = channels * channelSpacing;
    const infoBandwidth = channelSpacing;
    const processingGain = 10 * Math.log10(spreadBandwidth / infoBandwidth);

    return {
      id: `FHSS-${Date.now()}`,
      channels: channelFreqs,
      hopRate,
      dwellTime,
      sequence,
      syncTime: new Date(),
      processingGain,
    };
  }

  /**
   * Calculate spread spectrum parameters
   *
   * @param infoBandwidth - Information bandwidth in Hz
   * @param spreadFactor - Spreading factor
   * @returns Spread spectrum parameters
   */
  calculateSpreadSpectrum(
    infoBandwidth: number,
    spreadFactor: number
  ): SpreadSpectrumParams {
    const spreadBandwidth = infoBandwidth * spreadFactor;
    const chipRate = spreadBandwidth; // For DSSS

    // Processing gain
    const processingGain = 10 * Math.log10(spreadFactor);

    // Jamming margin (assume 3 dB implementation loss)
    const implementationLoss = 3; // dB
    const requiredMargin = 6; // dB
    const jammingMargin = processingGain - implementationLoss - requiredMargin;

    return {
      type: 'DSSS',
      spreadBandwidth,
      infoBandwidth,
      chipRate,
      processingGain,
      jammingMargin,
    };
  }

  /**
   * Perform spectrum scan
   *
   * @param range - Frequency range to scan
   * @param rbw - Resolution bandwidth in Hz
   * @returns Spectrum scan results
   */
  spectrumScan(range: FrequencyRange, rbw: number = 1e6): SpectrumScanResult {
    const signals: DetectedSignal[] = [];

    // Simulate signal detection
    // In real implementation, this would use SDR hardware
    const numScans = Math.ceil((range.end - range.start) / rbw);
    const numSignals = Math.floor(Math.random() * 5) + 1;

    for (let i = 0; i < numSignals; i++) {
      const freq = range.start + Math.random() * (range.end - range.start);
      const bw = rbw * (0.1 + Math.random() * 0.5);
      const power = -100 + Math.random() * 50; // -100 to -50 dBm

      signals.push({
        id: `SIG-${Date.now()}-${i}`,
        timestamp: new Date(),
        frequency: freq,
        bandwidth: bw,
        power,
        confidence: 0.7 + Math.random() * 0.3,
      });
    }

    // Calculate spectrum occupancy
    const totalBandwidth = range.end - range.start;
    const occupiedBandwidth = signals.reduce((sum, sig) => sum + sig.bandwidth, 0);
    const occupancy = Math.min(1.0, occupiedBandwidth / totalBandwidth);

    return {
      id: `SCAN-${Date.now()}`,
      range,
      timestamp: new Date(),
      signals,
      occupancy,
      rbw,
    };
  }

  /**
   * Calculate RF safety parameters
   *
   * @param frequency - Frequency in Hz
   * @param eirp - EIRP in watts
   * @returns RF safety parameters
   */
  calculateRFSafety(frequency: number, eirp: number): RFSafetyParams {
    // Convert frequency to MHz
    const freqMHz = frequency / 1e6;

    // Calculate MPE based on frequency
    let mpe: number; // mW/cm²

    if (freqMHz >= 0.3 && freqMHz <= 3) {
      mpe = 180 / Math.pow(freqMHz, 2);
    } else if (freqMHz > 3 && freqMHz <= 15) {
      mpe = 1.0;
    } else if (freqMHz > 15 && freqMHz <= 300) {
      mpe = freqMHz / 15;
    } else {
      mpe = 1.0; // Default conservative value
    }

    // Convert MPE from mW/cm² to W/m²
    const mpeWm2 = mpe * 10;

    // Calculate safe distance
    // S = EIRP / (4πR²)
    // R = √(EIRP / (4π × MPE))
    const safeDistance = Math.sqrt(eirp / (4 * Math.PI * mpeWm2));

    // Safety zone with 20% margin
    const safetyZone = safeDistance * 1.2;

    // Time limit (if applicable, simplified)
    const timeLimit = mpe > 5 ? undefined : 6; // minutes

    return {
      mpe,
      frequency,
      eirp,
      safeDistance,
      timeLimit,
      safetyZone,
    };
  }

  /**
   * Perform direction finding
   *
   * @param signal - Detected signal
   * @param receiverLocations - Array of receiver locations
   * @returns Direction finding result
   */
  directionFinding(
    signal: DetectedSignal,
    receiverLocations: GeographicLocation[]
  ): DirectionFindingResult {
    if (receiverLocations.length < 2) {
      throw new EWError(
        EWErrorCode.INVALID_PARAMETERS,
        'At least 2 receivers required for direction finding'
      );
    }

    // Simulate direction finding
    // In real implementation, this would use actual phase/amplitude measurements

    // Random azimuth for simulation
    const azimuth = Math.random() * 360;

    // Accuracy improves with more receivers
    const baseAccuracy = 5; // degrees
    const accuracy = baseAccuracy / Math.sqrt(receiverLocations.length);

    // Elevation (if 3+ receivers)
    const elevation = receiverLocations.length >= 3
      ? Math.random() * 90
      : undefined;

    // Method selection based on number of receivers
    let method: DirectionFindingResult['method'];
    if (receiverLocations.length >= 4) {
      method = 'tdoa';
    } else if (receiverLocations.length >= 3) {
      method = 'phase-interferometry';
    } else {
      method = 'amplitude-comparison';
    }

    // Confidence based on SNR and method
    const snr = signal.snr || 10;
    const confidence = Math.min(0.95, 0.5 + snr / 40);

    return {
      id: `DF-${Date.now()}`,
      timestamp: new Date(),
      frequency: signal.frequency,
      azimuth,
      elevation,
      accuracy,
      method,
      confidence,
    };
  }

  /**
   * Get frequency band for a given frequency
   *
   * @param frequency - Frequency in Hz
   * @returns Frequency band name
   */
  getFrequencyBand(frequency: number): FrequencyBand {
    if (frequency >= FREQUENCY_BANDS.HF.start && frequency < FREQUENCY_BANDS.HF.end) {
      return 'HF';
    } else if (frequency >= FREQUENCY_BANDS.VHF.start && frequency < FREQUENCY_BANDS.VHF.end) {
      return 'VHF';
    } else if (frequency >= FREQUENCY_BANDS.UHF.start && frequency < FREQUENCY_BANDS.UHF.end) {
      return 'UHF';
    } else if (frequency >= FREQUENCY_BANDS.SHF.start && frequency < FREQUENCY_BANDS.SHF.end) {
      return 'SHF';
    } else if (frequency >= FREQUENCY_BANDS.EHF.start && frequency < FREQUENCY_BANDS.EHF.end) {
      return 'EHF';
    }
    throw new EWError(
      EWErrorCode.FREQUENCY_OUT_OF_RANGE,
      `Frequency ${frequency} Hz is out of supported range`
    );
  }

  /**
   * Calculate path loss
   *
   * @param distance - Distance in meters
   * @param frequency - Frequency in Hz
   * @returns Path loss in dB
   */
  calculatePathLoss(distance: number, frequency: number): number {
    // Free space path loss
    // PL = 20 log(d) + 20 log(f) + 32.45
    // where d is in km and f is in MHz

    const distanceKm = distance / 1000;
    const freqMHz = frequency / 1e6;

    const pathLoss = 20 * Math.log10(distanceKm) + 20 * Math.log10(freqMHz) + 32.45;

    return pathLoss;
  }

  /**
   * Calculate link budget
   *
   * @param txPower - Transmit power in dBm
   * @param txGain - Transmit antenna gain in dBi
   * @param rxGain - Receive antenna gain in dBi
   * @param pathLoss - Path loss in dB
   * @param otherLosses - Other losses in dB
   * @returns Received power in dBm
   */
  calculateLinkBudget(
    txPower: number,
    txGain: number,
    rxGain: number,
    pathLoss: number,
    otherLosses: number = 0
  ): number {
    return txPower + txGain + rxGain - pathLoss - otherLosses;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Convert watts to dBm
   */
  wattsToDbm(watts: number): number {
    return 10 * Math.log10(watts * 1000);
  }

  /**
   * Convert dBm to watts
   */
  dbmToWatts(dbm: number): number {
    return Math.pow(10, dbm / 10) / 1000;
  }

  /**
   * Convert dBi to linear gain
   */
  dbiToLinear(dbi: number): number {
    return Math.pow(10, dbi / 10);
  }

  /**
   * Convert linear gain to dBi
   */
  linearToDbi(linear: number): number {
    return 10 * Math.log10(linear);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate jamming power (standalone function)
 */
export function calculateJammingPower(params: JammingParameters): JammingResult {
  const sdk = new ElectronicWarfareSDK();
  return sdk.calculateJammingPower(params);
}

/**
 * Analyze signal intelligence (standalone function)
 */
export function analyzeSignalIntelligence(params: SIGINTParameters): SIGINTResult {
  const sdk = new ElectronicWarfareSDK();
  return sdk.analyzeSignalIntelligence(params);
}

/**
 * Generate frequency hopping pattern (standalone function)
 */
export function generateFrequencyHopping(
  channels: number,
  dwellTime: number,
  baseFreq?: number,
  channelSpacing?: number
): FrequencyHoppingConfig {
  const sdk = new ElectronicWarfareSDK();
  return sdk.generateFrequencyHopping(channels, dwellTime, baseFreq, channelSpacing);
}

/**
 * Calculate spread spectrum parameters (standalone function)
 */
export function calculateSpreadSpectrum(
  infoBandwidth: number,
  spreadFactor: number
): SpreadSpectrumParams {
  const sdk = new ElectronicWarfareSDK();
  return sdk.calculateSpreadSpectrum(infoBandwidth, spreadFactor);
}

/**
 * Perform spectrum scan (standalone function)
 */
export function spectrumScan(range: FrequencyRange, rbw?: number): SpectrumScanResult {
  const sdk = new ElectronicWarfareSDK();
  return sdk.spectrumScan(range, rbw);
}

/**
 * Calculate RF safety (standalone function)
 */
export function calculateRFSafety(frequency: number, eirp: number): RFSafetyParams {
  const sdk = new ElectronicWarfareSDK();
  return sdk.calculateRFSafety(frequency, eirp);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ElectronicWarfareSDK };
export default ElectronicWarfareSDK;
