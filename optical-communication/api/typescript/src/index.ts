/**
 * WIA-COMM-007: Optical Communication SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for optical communication including:
 * - Link budget calculations
 * - DWDM system design
 * - Dispersion analysis
 * - Transceiver modeling
 * - Coherent transmission simulation
 */

import {
  LinkBudgetParams,
  LinkBudgetResult,
  DWDMSystem,
  WDMChannel,
  DispersionParams,
  DispersionResult,
  OpticalTransceiver,
  OpticalAmplifier,
  PowerConversion,
  WavelengthConversion,
  FiberType,
  ModulationFormat,
  DWDMSpacing,
  WavelengthBand,
  OPTICAL_COMM_CONSTANTS,
  OpticalCommErrorCode,
  OpticalCommError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-007 Optical Communication SDK
 */
export class OpticalCommunicationSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Calculate optical link budget
   *
   * @param params - Link budget calculation parameters
   * @returns Link budget result with margin and status
   */
  calculateLinkBudget(params: LinkBudgetParams): LinkBudgetResult {
    const {
      txPower,
      rxSensitivity = -28,
      fiberLength,
      fiberType,
      wavelength,
      fiberLoss,
      connectors = 2,
      connectorLoss = 0.5,
      splices = 0,
      spliceLoss = 0.1,
      amplifiers = [],
      requiredMargin = 3,
    } = params;

    // Validate inputs
    if (fiberLength <= 0) {
      throw new OpticalCommError(
        OpticalCommErrorCode.INVALID_PARAMETERS,
        'Fiber length must be positive'
      );
    }

    // Get fiber attenuation
    const alpha = fiberLoss ?? this.getFiberAttenuation(fiberType, wavelength);

    // Calculate losses
    const distanceKm = fiberLength / 1000;
    const totalFiberLoss = alpha * distanceKm;
    const totalConnectorLoss = connectors * connectorLoss;
    const totalSpliceLoss = splices * spliceLoss;

    // Calculate amplifier gain
    const totalAmplifierGain = amplifiers.reduce((sum, amp) => sum + amp.gain, 0);

    // Calculate total path loss
    const totalLoss =
      totalFiberLoss + totalConnectorLoss + totalSpliceLoss - totalAmplifierGain;

    // Calculate received power
    const rxPower = txPower - totalLoss;

    // Calculate margin
    const margin = rxPower - rxSensitivity;

    // Calculate OSNR if amplifiers present
    let osnr: number | undefined;
    if (amplifiers.length > 0) {
      osnr = this.calculateOSNR(amplifiers, txPower, totalFiberLoss);
    }

    // Determine status
    let status: 'pass' | 'fail' | 'marginal';
    if (margin >= requiredMargin) {
      status = 'pass';
    } else if (margin >= 0) {
      status = 'marginal';
    } else {
      status = 'fail';
    }

    return {
      txPower,
      rxPower,
      fiberLoss: totalFiberLoss,
      connectorLoss: totalConnectorLoss,
      spliceLoss: totalSpliceLoss,
      amplifierGain: totalAmplifierGain,
      totalLoss,
      margin,
      osnr,
      status,
      distanceKm,
    };
  }

  /**
   * Design DWDM system
   *
   * @param config - DWDM configuration parameters
   * @returns Complete DWDM system specification
   */
  designDWDMSystem(config: {
    channels: number;
    spacing: DWDMSpacing;
    startFrequency?: number;
    dataRate: number;
    modulation?: ModulationFormat;
    band?: WavelengthBand;
  }): DWDMSystem {
    const {
      channels,
      spacing,
      startFrequency = OPTICAL_COMM_CONSTANTS.ITU_REFERENCE_FREQUENCY,
      dataRate,
      modulation = 'DP-QPSK',
      band = 'C',
    } = config;

    // Parse spacing
    const spacingHz = this.parseSpacing(spacing);

    // Generate channel list
    const channelList: WDMChannel[] = [];
    for (let i = 0; i < channels; i++) {
      const frequency = startFrequency + i * spacingHz;
      const wavelength = OPTICAL_COMM_CONSTANTS.SPEED_OF_LIGHT / frequency;

      channelList.push({
        channelNumber: i + 1,
        frequency,
        wavelength,
        power: 0, // To be set by amplifier design
        dataRate,
        modulation,
      });
    }

    // Calculate total capacity
    const totalCapacity = channels * dataRate;

    return {
      id: `DWDM-${channels}CH-${spacing}`,
      type: 'DWDM',
      spacing,
      channels,
      startFrequency,
      channelList,
      totalCapacity,
      band,
      fiberType: 'SMF',
    };
  }

  /**
   * Calculate chromatic dispersion and pulse broadening
   *
   * @param params - Dispersion calculation parameters
   * @returns Dispersion results with compensation recommendations
   */
  calculateDispersion(params: DispersionParams): DispersionResult {
    const {
      fiberLength,
      fiberType,
      wavelength,
      dispersionCoeff,
      spectralWidth,
      dataRate,
    } = params;

    // Get dispersion coefficient
    const D =
      dispersionCoeff ?? this.getDispersionCoefficient(fiberType, wavelength);

    const distanceKm = fiberLength / 1000;

    // Calculate total chromatic dispersion (ps/nm)
    const chromaticDispersion = D * distanceKm;

    // Calculate spectral width in nm
    const spectralWidthNm = spectralWidth * 1e9;

    // Calculate pulse broadening (ps)
    const pulseBroadening = Math.abs(chromaticDispersion * spectralWidthNm);

    // Calculate bit period (ps)
    const bitPeriod = 1e12 / dataRate;

    // Calculate dispersion penalty (approximate)
    const dispersionRatio = pulseBroadening / bitPeriod;
    const penalty = 10 * Math.log10(1 + (dispersionRatio / 2) ** 2);

    // Determine if compensation required (>10% of bit period)
    const compensationRequired = pulseBroadening > 0.1 * bitPeriod;

    // Calculate required DCF length if needed
    let dcfLength: number | undefined;
    if (compensationRequired) {
      // DCF typically has -80 ps/(nm·km)
      dcfLength = (Math.abs(chromaticDispersion) / 80) * 1000; // meters
    }

    // Calculate PMD (approximate)
    const pmdCoeff = 0.1; // ps/√km (typical modern fiber)
    const pmd = pmdCoeff * Math.sqrt(distanceKm);

    return {
      chromaticDispersion,
      pulseBroadening,
      penalty,
      compensationRequired,
      dcfLength,
      pmd,
    };
  }

  /**
   * Analyze optical transceiver
   *
   * @param config - Transceiver configuration
   * @returns Transceiver specifications
   */
  analyzeTransceiver(config: {
    formFactor: string;
    dataRate: number;
    wavelength: number;
    reach: number;
    fiberType?: FiberType;
  }): OpticalTransceiver {
    const { formFactor, dataRate, wavelength, reach, fiberType = 'SMF' } = config;

    // Determine parameters based on data rate and reach
    const lanes = this.calculateLanes(dataRate);
    const modulation = this.selectModulation(dataRate, reach);
    const txPower = this.calculateTxPower(reach);
    const rxSensitivity = this.calculateRxSensitivity(modulation, dataRate);

    return {
      id: `${formFactor}-${dataRate / 1e9}G`,
      formFactor: formFactor as any,
      dataRate,
      wavelength,
      txPower,
      rxSensitivity,
      maxReach: reach,
      fiberType,
      lanes,
      modulation,
      powerConsumption: this.estimatePowerConsumption(dataRate, reach),
      temperatureRange: [0, 70],
    };
  }

  /**
   * Convert power between units
   *
   * @param value - Power value
   * @param unit - Input unit
   * @returns Power in all common units
   */
  convertPower(value: number, unit: 'dBm' | 'mW' | 'W'): PowerConversion {
    let mW: number;

    switch (unit) {
      case 'dBm':
        mW = Math.pow(10, value / 10);
        break;
      case 'mW':
        mW = value;
        break;
      case 'W':
        mW = value * 1000;
        break;
    }

    const dBm = 10 * Math.log10(mW);
    const W = mW / 1000;

    return {
      input: value,
      inputUnit: unit,
      dBm,
      mW,
      W,
    };
  }

  /**
   * Convert wavelength to frequency and vice versa
   *
   * @param value - Wavelength value
   * @param unit - Input unit
   * @returns Wavelength in all common units and frequency
   */
  convertWavelength(value: number, unit: 'nm' | 'um' | 'm'): WavelengthConversion {
    let meters: number;

    switch (unit) {
      case 'nm':
        meters = value * 1e-9;
        break;
      case 'um':
        meters = value * 1e-6;
        break;
      case 'm':
        meters = value;
        break;
    }

    const nanometers = meters * 1e9;
    const frequency = OPTICAL_COMM_CONSTANTS.SPEED_OF_LIGHT / meters;

    return {
      input: value,
      inputUnit: unit,
      meters,
      nanometers,
      frequency,
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private getFiberAttenuation(fiberType: FiberType, wavelength: number): number {
    const wl_nm = wavelength * 1e9;

    switch (fiberType) {
      case 'SMF':
        if (wl_nm >= 1520 && wl_nm <= 1570) {
          return 0.2; // C-band
        } else if (wl_nm >= 1280 && wl_nm <= 1340) {
          return 0.35; // O-band
        } else if (wl_nm >= 840 && wl_nm <= 860) {
          return 2.5; // 850 nm
        }
        return 0.5; // default
      case 'MMF':
        return 2.5; // @ 850 nm
      case 'NZDSF':
        return 0.22;
      case 'DCF':
        return 0.5;
      default:
        return 0.3;
    }
  }

  private getDispersionCoefficient(fiberType: FiberType, wavelength: number): number {
    const wl_nm = wavelength * 1e9;

    switch (fiberType) {
      case 'SMF':
        if (Math.abs(wl_nm - 1310) < 50) {
          return 0; // Zero dispersion @ 1310 nm
        } else if (wl_nm >= 1520 && wl_nm <= 1570) {
          return 17; // C-band
        }
        return 10;
      case 'DSF':
        if (Math.abs(wl_nm - 1550) < 50) {
          return 0; // Zero dispersion @ 1550 nm
        }
        return 5;
      case 'NZDSF':
        return 4; // Small positive dispersion
      case 'DCF':
        return -80; // Negative dispersion for compensation
      default:
        return 10;
    }
  }

  private parseSpacing(spacing: DWDMSpacing): number {
    switch (spacing) {
      case '12.5GHz':
        return 12.5e9;
      case '25GHz':
        return 25e9;
      case '50GHz':
        return 50e9;
      case '100GHz':
        return 100e9;
    }
  }

  private calculateOSNR(
    amplifiers: OpticalAmplifier[],
    signalPower: number,
    spanLoss: number
  ): number {
    // Simplified OSNR calculation
    // OSNR (dB) = Signal Power - ASE Power (in 0.1 nm bandwidth)

    const h = OPTICAL_COMM_CONSTANTS.PLANCK_CONSTANT;
    const c = OPTICAL_COMM_CONSTANTS.SPEED_OF_LIGHT;
    const lambda = 1550e-9; // Assume C-band
    const nu = c / lambda;
    const Bo = 12.5e9; // 0.1 nm ~ 12.5 GHz @ 1550 nm

    let aseTotal = 0;

    amplifiers.forEach((amp) => {
      const nsp = Math.pow(10, amp.noiseFigure / 10) / 2;
      const G = Math.pow(10, amp.gain / 10);
      const asePerAmp = 2 * nsp * h * nu * (G - 1) * Bo;
      aseTotal += asePerAmp;
    });

    // Convert signal power from dBm to W
    const signalW = Math.pow(10, signalPower / 10) / 1000;

    // OSNR in linear units
    const osnrLinear = signalW / (aseTotal || 1e-12);

    // Convert to dB
    return 10 * Math.log10(osnrLinear);
  }

  private calculateLanes(dataRate: number): number {
    // Determine number of lanes based on data rate
    if (dataRate <= 10e9) return 1;
    if (dataRate <= 25e9) return 1;
    if (dataRate <= 40e9) return 4;
    if (dataRate <= 100e9) return 4;
    if (dataRate <= 200e9) return 4;
    if (dataRate <= 400e9) return 8;
    return 8;
  }

  private selectModulation(dataRate: number, reach: number): ModulationFormat {
    // Select modulation based on data rate and reach
    if (dataRate <= 10e9) {
      return 'OOK';
    } else if (dataRate <= 100e9) {
      if (reach <= 10000) return 'PAM4';
      return 'DP-QPSK';
    } else if (dataRate <= 200e9) {
      if (reach <= 2000) return 'PAM4';
      return 'DP-16QAM';
    } else {
      if (reach <= 2000) return 'PAM4';
      return 'DP-16QAM';
    }
  }

  private calculateTxPower(reach: number): number {
    // Estimate TX power based on reach
    if (reach <= 500) return -5; // Short reach
    if (reach <= 10000) return 0; // Medium reach
    return 3; // Long reach
  }

  private calculateRxSensitivity(
    modulation: ModulationFormat,
    dataRate: number
  ): number {
    // Estimate RX sensitivity based on modulation and rate
    const baseRx = -28; // dBm for OOK at 10G

    // Adjust for data rate (higher rate = worse sensitivity)
    const rateMultiplier = dataRate / 10e9;
    const ratePenalty = 10 * Math.log10(rateMultiplier);

    // Adjust for modulation
    let modPenalty = 0;
    switch (modulation) {
      case 'PAM4':
        modPenalty = 3;
        break;
      case 'DP-QPSK':
        modPenalty = -3;
        break;
      case 'DP-16QAM':
        modPenalty = 7;
        break;
      case 'DP-64QAM':
        modPenalty = 14;
        break;
    }

    return baseRx + ratePenalty + modPenalty;
  }

  private estimatePowerConsumption(dataRate: number, reach: number): number {
    // Estimate power consumption in watts
    const baselinePower = 1.5; // W for 10G
    const rateMultiplier = dataRate / 10e9;
    const reachFactor = reach > 10000 ? 1.5 : 1.0;

    return baselinePower * rateMultiplier * reachFactor;
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Calculate optical link budget
 */
export function calculateLinkBudget(params: LinkBudgetParams): LinkBudgetResult {
  const sdk = new OpticalCommunicationSDK();
  return sdk.calculateLinkBudget(params);
}

/**
 * Design DWDM system
 */
export function designDWDMSystem(config: {
  channels: number;
  spacing: DWDMSpacing;
  startFrequency?: number;
  dataRate: number;
  modulation?: ModulationFormat;
  band?: WavelengthBand;
}): DWDMSystem {
  const sdk = new OpticalCommunicationSDK();
  return sdk.designDWDMSystem(config);
}

/**
 * Calculate dispersion
 */
export function calculateDispersion(params: DispersionParams): DispersionResult {
  const sdk = new OpticalCommunicationSDK();
  return sdk.calculateDispersion(params);
}

/**
 * Analyze transceiver
 */
export function analyzeTransceiver(config: {
  formFactor: string;
  dataRate: number;
  wavelength: number;
  reach: number;
  fiberType?: FiberType;
}): OpticalTransceiver {
  const sdk = new OpticalCommunicationSDK();
  return sdk.analyzeTransceiver(config);
}

/**
 * Convert power units
 */
export function convertPower(
  value: number,
  unit: 'dBm' | 'mW' | 'W'
): PowerConversion {
  const sdk = new OpticalCommunicationSDK();
  return sdk.convertPower(value, unit);
}

/**
 * Convert wavelength units
 */
export function convertWavelength(
  value: number,
  unit: 'nm' | 'um' | 'm'
): WavelengthConversion {
  const sdk = new OpticalCommunicationSDK();
  return sdk.convertWavelength(value, unit);
}

// Export SDK class and types
export { OpticalCommunicationSDK };
export * from './types';

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
