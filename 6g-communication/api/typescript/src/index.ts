/**
 * WIA-COMM-001: 6G Communication SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for 6G communications including:
 * - THz spectrum management
 * - Data rate calculations
 * - Beamforming optimization
 * - IRS configuration
 * - Network simulation
 */

import {
  DataRateParams,
  DataRateResult,
  SpectrumAllocation,
  SpectrumValidation,
  BeamformingParams,
  BeamConfig,
  THzPropagation,
  PropagationLoss,
  NetworkSimulation,
  SimulationResult,
  IRSOptimization,
  IRSConfig,
  DigitalTwinSync,
  SIXG_CONSTANTS,
  FREQUENCY_BANDS,
  CommErrorCode,
  SixGCommError,
  Vector3D,
  FrequencyBand,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-001 6G Communication SDK
 */
export class SixGSDK {
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
   * Calculate 6G data rate
   *
   * @param params - Data rate parameters
   * @returns Data rate calculation results
   */
  calculate6GDataRate(params: DataRateParams): DataRateResult {
    const {
      frequency,
      bandwidth,
      snr,
      modulation,
      mimo = { tx: 1, rx: 1 },
      codingRate = 0.9,
    } = params;

    // Validate inputs
    if (bandwidth <= 0 || bandwidth > 1e12) {
      throw new SixGCommError(
        CommErrorCode.INSUFFICIENT_BANDWIDTH,
        'Bandwidth must be between 0 and 1 THz'
      );
    }

    if (!this.isValidFrequency(frequency)) {
      throw new SixGCommError(
        CommErrorCode.INVALID_FREQUENCY,
        'Frequency must be in 6G range (100 GHz - 10 THz)'
      );
    }

    // Get modulation order
    const modulationOrder = this.getModulationOrder(modulation);

    // Calculate spatial streams (min of tx and rx)
    const spatialStreams = Math.min(mimo.tx, mimo.rx);

    // Shannon capacity: C = B × log₂(1 + SNR)
    const snrLinear = Math.pow(10, snr / 10);
    const shannonCapacity = bandwidth * Math.log2(1 + snrLinear);

    // Practical data rate with modulation and MIMO
    const bitsPerSymbol = Math.log2(modulationOrder);
    const peakRate =
      bandwidth * bitsPerSymbol * spatialStreams * codingRate;

    // Average rate (80% of peak due to overhead and real-world conditions)
    const averageRate = peakRate * 0.8;

    // Spectral efficiency
    const spectralEfficiency = peakRate / bandwidth;

    // Feasibility assessment
    let feasibility: 'achievable' | 'challenging' | 'theoretical';
    if (peakRate <= 100e9) {
      feasibility = 'achievable';
    } else if (peakRate <= 500e9) {
      feasibility = 'challenging';
    } else {
      feasibility = 'theoretical';
    }

    return {
      peakRate,
      averageRate,
      spectralEfficiency,
      spatialStreams,
      modulationOrder,
      feasibility,
      rateFormatted: this.formatDataRate(peakRate),
    };
  }

  /**
   * Validate spectrum allocation
   *
   * @param allocation - Spectrum allocation request
   * @returns Validation result
   */
  validateSpectrumAllocation(
    allocation: SpectrumAllocation
  ): SpectrumValidation {
    const { band, frequency, bandwidth, region } = allocation;
    const errors: string[] = [];
    const warnings: string[] = [];

    // Check if frequency is within band
    const bandRange = FREQUENCY_BANDS[band];
    if (frequency < bandRange.min || frequency > bandRange.max) {
      errors.push(
        `Frequency ${frequency / 1e9} GHz is outside ${band} range (${
          bandRange.min / 1e9
        }-${bandRange.max / 1e9} GHz)`
      );
    }

    // Check bandwidth
    const maxBandwidth = this.getMaxBandwidth(band);
    if (bandwidth > maxBandwidth) {
      warnings.push(
        `Bandwidth ${bandwidth / 1e9} GHz exceeds recommended maximum ${
          maxBandwidth / 1e9
        } GHz for ${band}`
      );
    }

    // Check if allocation fits within band
    const startFreq = frequency - bandwidth / 2;
    const endFreq = frequency + bandwidth / 2;

    if (startFreq < bandRange.min || endFreq > bandRange.max) {
      errors.push(
        `Allocation range ${startFreq / 1e9}-${
          endFreq / 1e9
        } GHz exceeds band limits`
      );
    }

    const isValid = errors.length === 0;

    return {
      isValid,
      allocation: isValid
        ? {
            centerFreq: frequency,
            bandwidth,
            startFreq,
            endFreq,
          }
        : undefined,
      errors,
      warnings,
      interferenceLevel: this.estimateInterference(frequency, region),
    };
  }

  /**
   * Calculate THz propagation loss
   *
   * @param params - Propagation parameters
   * @returns Path loss breakdown
   */
  calculateTHzPropagation(params: THzPropagation): PropagationLoss {
    const {
      frequency,
      distance,
      temperature = 20,
      humidity = 50,
      pressure = 101.325,
      rainRate = 0,
    } = params;

    // Free space path loss (Friis formula)
    // FSPL(dB) = 20 log₁₀(d) + 20 log₁₀(f) + 20 log₁₀(4π/c)
    const c = SIXG_CONSTANTS.SPEED_OF_LIGHT;
    const freeSpaceLoss =
      20 * Math.log10(distance) +
      20 * Math.log10(frequency) +
      20 * Math.log10((4 * Math.PI) / c);

    // Atmospheric absorption (simplified ITU-R P.676 model)
    const { oxygenLoss, waterVaporLoss } = this.calculateAtmosphericLoss(
      frequency,
      distance,
      temperature,
      humidity,
      pressure
    );

    const atmosphericLoss = oxygenLoss + waterVaporLoss;

    // Rain attenuation (ITU-R P.838 simplified)
    const rainLoss = this.calculateRainLoss(frequency, distance, rainRate);

    // Total path loss
    const totalLoss = freeSpaceLoss + atmosphericLoss + rainLoss;

    // Estimate maximum range (assuming 30 dB link budget margin)
    const maxRange = this.estimateMaxRange(frequency, totalLoss);

    return {
      freeSpaceLoss,
      atmosphericLoss,
      rainLoss,
      totalLoss,
      maxRange,
      breakdown: {
        oxygenLoss,
        waterVaporLoss,
        rainLoss,
      },
    };
  }

  /**
   * Generate beamforming configuration
   *
   * @param params - Beamforming parameters
   * @returns Beam configuration
   */
  generateBeamConfig(params: BeamformingParams): BeamConfig {
    const {
      elements,
      target,
      beamWidth = 5,
      algorithm = 'digital',
    } = params;

    // Calculate beamforming weights using steering vector
    const weights = this.calculateBeamWeights(
      elements,
      target.azimuth,
      target.elevation,
      algorithm
    );

    // Calculate beam gain (approximation)
    const gain = 10 * Math.log10(elements);

    return {
      id: `beam-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      elements,
      azimuthAngle: target.azimuth,
      elevationAngle: target.elevation,
      beamWidth,
      gain,
      weights,
      status: 'active',
    };
  }

  /**
   * Simulate 6G network
   *
   * @param params - Simulation parameters
   * @returns Simulation results
   */
  simulate6GNetwork(params: NetworkSimulation): SimulationResult {
    const {
      id,
      area,
      numBaseStations,
      numUsers,
      frequency,
      bandwidth,
      irsDeployment,
      trafficModel,
      duration,
    } = params;

    try {
      // Simple simulation model
      const cellArea = area / numBaseStations;
      const usersPerCell = numUsers / numBaseStations;

      // Calculate per-user data rate
      const dataRateResult = this.calculate6GDataRate({
        frequency,
        bandwidth: bandwidth / usersPerCell,
        snr: 20, // Assumed average SNR
        modulation: 'QAM-256',
        mimo: { tx: 64, rx: 4 },
      });

      // IRS gain (if deployed)
      let irsGain = 1;
      if (irsDeployment) {
        irsGain = 1 + 0.1 * irsDeployment.numPanels; // 10% gain per panel
      }

      const averageDataRate = dataRateResult.averageRate * irsGain;
      const peakDataRate = dataRateResult.peakRate * irsGain;

      // Latency estimation
      const propagationDelay = Math.sqrt(cellArea / Math.PI) / SIXG_CONSTANTS.SPEED_OF_LIGHT * 1000;
      const processingDelay = 0.1; // ms
      const averageLatency = propagationDelay + processingDelay;

      // Coverage (simplified)
      const coverage = Math.min(99.5, 90 + numBaseStations / area * 10);

      // User satisfaction (based on QoS)
      const satisfied = averageLatency < 1 ? 95 : 80;

      return {
        id,
        metrics: {
          averageDataRate,
          peakDataRate,
          averageLatency,
          latency99Percentile: averageLatency * 1.5,
          coverage,
          spectralEfficiency: dataRateResult.spectralEfficiency,
          energyEfficiency: 1e6, // bits/Joule (placeholder)
        },
        userStats: {
          satisfied,
          blocked: 100 - satisfied,
          handovers: Math.floor(numUsers * duration / 3600), // Approximate
        },
        resourceUtil: {
          spectrum: 70, // %
          baseStations: 80, // %
          irs: irsDeployment ? 60 : 0, // %
        },
        success: true,
        logs: [
          `Simulation completed for ${numUsers} users over ${area} km²`,
          `Average data rate: ${this.formatDataRate(averageDataRate)}`,
          `Latency: ${averageLatency.toFixed(2)} ms`,
        ],
      };
    } catch (error) {
      throw new SixGCommError(
        CommErrorCode.SIMULATION_FAILED,
        `Simulation failed: ${error}`,
        { params }
      );
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Check if frequency is valid for 6G
   */
  private isValidFrequency(frequency: number): boolean {
    return frequency >= 100e9 && frequency <= 10e12;
  }

  /**
   * Get modulation order
   */
  private getModulationOrder(
    modulation: 'QPSK' | 'QAM-16' | 'QAM-64' | 'QAM-256' | 'QAM-1024'
  ): number {
    const orders: Record<string, number> = {
      QPSK: 4,
      'QAM-16': 16,
      'QAM-64': 64,
      'QAM-256': 256,
      'QAM-1024': 1024,
    };
    return orders[modulation];
  }

  /**
   * Get maximum recommended bandwidth for band
   */
  private getMaxBandwidth(band: FrequencyBand): number {
    const maxBandwidths: Record<FrequencyBand, number> = {
      'sub-thz': 10e9, // 10 GHz
      'thz-1': 50e9, // 50 GHz
      'thz-2': 200e9, // 200 GHz
      'thz-3': 1e12, // 1 THz
    };
    return maxBandwidths[band];
  }

  /**
   * Estimate interference level
   */
  private estimateInterference(frequency: number, region: string): number {
    // Simplified model: higher frequency = lower interference
    const baseInterference = region === 'global' ? -90 : -100; // dBm
    const frequencyFactor = frequency / 1e12; // Normalized to THz
    return baseInterference - 10 * Math.log10(frequencyFactor);
  }

  /**
   * Calculate atmospheric loss (ITU-R P.676 simplified)
   */
  private calculateAtmosphericLoss(
    frequency: number,
    distance: number,
    temperature: number,
    humidity: number,
    pressure: number
  ): { oxygenLoss: number; waterVaporLoss: number } {
    const f_GHz = frequency / 1e9;

    // Oxygen absorption (simplified)
    let oxygenAbs = 0;
    if (f_GHz < 60) {
      oxygenAbs = 0.1 * Math.pow(f_GHz / 60, 2);
    } else if (f_GHz < 120) {
      oxygenAbs = 15 * Math.exp(-Math.pow((f_GHz - 60) / 10, 2));
    } else {
      oxygenAbs = 0.5;
    }

    // Water vapor absorption (simplified)
    const waterVaporDensity = this.calculateWaterVaporDensity(
      temperature,
      humidity
    );
    let waterVaporAbs = 0;
    if (f_GHz > 100) {
      waterVaporAbs =
        0.05 * waterVaporDensity * Math.pow((f_GHz - 183) / 50, 2);
    }

    const oxygenLoss = oxygenAbs * (distance / 1000); // dB
    const waterVaporLoss = waterVaporAbs * (distance / 1000); // dB

    return { oxygenLoss, waterVaporLoss };
  }

  /**
   * Calculate water vapor density
   */
  private calculateWaterVaporDensity(
    temperature: number,
    humidity: number
  ): number {
    // Simplified formula: ρ = RH × ρ_sat(T)
    const T_kelvin = temperature + 273.15;
    const saturationDensity = 216.7 * (6.1078 * Math.exp((17.27 * temperature) / (temperature + 237.3)) / T_kelvin);
    return (humidity / 100) * saturationDensity;
  }

  /**
   * Calculate rain attenuation (ITU-R P.838 simplified)
   */
  private calculateRainLoss(
    frequency: number,
    distance: number,
    rainRate: number
  ): number {
    if (rainRate === 0) return 0;

    const f_GHz = frequency / 1e9;

    // Specific attenuation γ_R = k × R^α (dB/km)
    // Coefficients for horizontal polarization (simplified)
    const k =
      4.21e-5 * Math.pow(f_GHz, 2.42) / (1 + 2.5e-4 * Math.pow(f_GHz, 1.5));
    const alpha = 1.41 / (1 + 0.78 * Math.log(f_GHz) - 0.38 * (1 - Math.exp(-2 * f_GHz)));

    const specificAttenuation = k * Math.pow(rainRate, alpha);
    return specificAttenuation * (distance / 1000); // dB
  }

  /**
   * Estimate maximum communication range
   */
  private estimateMaxRange(frequency: number, pathLoss: number): number {
    // Assume 30 dB link budget margin, transmit power 20 dBm, receiver sensitivity -80 dBm
    const linkBudget = 20 - (-80); // 100 dB
    const allowedLoss = linkBudget - 30; // 70 dB margin

    // Simplified: range inversely proportional to path loss
    const rangeRatio = Math.pow(10, (allowedLoss - pathLoss) / 20);
    return Math.max(10, 100 * rangeRatio); // Minimum 10m, scale from 100m baseline
  }

  /**
   * Calculate beamforming weights
   */
  private calculateBeamWeights(
    elements: number,
    azimuth: number,
    elevation: number,
    algorithm: string
  ): { real: number; imag: number }[] {
    const weights: { real: number; imag: number }[] = [];

    // Convert angles to radians
    const azRad = (azimuth * Math.PI) / 180;
    const elRad = (elevation * Math.PI) / 180;

    // Uniform linear array (ULA) steering vector
    const wavelength = SIXG_CONSTANTS.SPEED_OF_LIGHT / 300e9; // Assume 300 GHz
    const d = wavelength / 2; // Half-wavelength spacing

    for (let n = 0; n < elements; n++) {
      // Phase shift for element n
      const phase =
        (2 * Math.PI * d * n * Math.sin(elRad) * Math.cos(azRad)) /
        wavelength;

      weights.push({
        real: Math.cos(phase) / Math.sqrt(elements),
        imag: Math.sin(phase) / Math.sqrt(elements),
      });
    }

    return weights;
  }

  /**
   * Format data rate for human readability
   */
  private formatDataRate(rate: number): string {
    if (rate < 1e3) {
      return `${rate.toFixed(2)} bps`;
    } else if (rate < 1e6) {
      return `${(rate / 1e3).toFixed(2)} kbps`;
    } else if (rate < 1e9) {
      return `${(rate / 1e6).toFixed(2)} Mbps`;
    } else if (rate < 1e12) {
      return `${(rate / 1e9).toFixed(2)} Gbps`;
    } else {
      return `${(rate / 1e12).toFixed(2)} Tbps`;
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate 6G data rate (standalone)
 */
export function calculate6GDataRate(params: DataRateParams): DataRateResult {
  const sdk = new SixGSDK();
  return sdk.calculate6GDataRate(params);
}

/**
 * Validate spectrum allocation (standalone)
 */
export function validateSpectrumAllocation(
  allocation: SpectrumAllocation
): SpectrumValidation {
  const sdk = new SixGSDK();
  return sdk.validateSpectrumAllocation(allocation);
}

/**
 * Calculate THz propagation (standalone)
 */
export function calculateTHzPropagation(
  params: THzPropagation
): PropagationLoss {
  const sdk = new SixGSDK();
  return sdk.calculateTHzPropagation(params);
}

/**
 * Generate beam configuration (standalone)
 */
export function generateBeamConfig(params: BeamformingParams): BeamConfig {
  const sdk = new SixGSDK();
  return sdk.generateBeamConfig(params);
}

/**
 * Simulate 6G network (standalone)
 */
export function simulate6GNetwork(params: NetworkSimulation): SimulationResult {
  const sdk = new SixGSDK();
  return sdk.simulate6GNetwork(params);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { SixGSDK };
