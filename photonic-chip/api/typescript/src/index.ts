/**
 * WIA-SEMI-006 Photonic Chip SDK
 * Main entry point for silicon photonics design and simulation tools
 *
 * @author World Certification Industry Association (WIA)
 * @license MIT
 * @version 1.0.0
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import * as math from 'mathjs';

export * from './types';
import type {
  Wavelength,
  RefractiveIndex,
  WaveguideGeometry,
  WaveguideProperties,
  ModulatorSpecs,
  PhotodetectorSpecs,
  WDMChannelGrid,
  WDMChannel,
  LinkBudget,
  OpticalPower,
  DataRate,
  Temperature,
  BERTestResult,
} from './types';

// ============================================================================
// Physical Constants
// ============================================================================

export const PhysicalConstants = {
  /** Speed of light in vacuum (m/s) */
  SPEED_OF_LIGHT: 299792458,
  /** Planck's constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,
  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,
  /** Boltzmann constant (J/K) */
  BOLTZMANN_CONSTANT: 1.380649e-23,
} as const;

// ============================================================================
// Material Properties
// ============================================================================

export const MaterialProperties = {
  Silicon: {
    refractiveIndex1550nm: 3.476,
    refractiveIndex1310nm: 3.505,
    thermoOpticCoefficient: 1.86e-4, // 1/K
    bandgap: 1.12, // eV
    thermalConductivity: 148, // W/(m·K)
  },
  SiliconDioxide: {
    refractiveIndex1550nm: 1.444,
    refractiveIndex1310nm: 1.448,
    thermoOpticCoefficient: 1.0e-5, // 1/K
  },
  SiliconNitride: {
    refractiveIndex1550nm: 2.0,
    refractiveIndex1310nm: 2.01,
    thermoOpticCoefficient: 2.5e-5, // 1/K
  },
  Germanium: {
    refractiveIndex1550nm: 4.225,
    refractiveIndex1310nm: 4.260,
    bandgap: 0.66, // eV
    absorptionCoefficient1550nm: 1e4, // cm^-1
  },
} as const;

// ============================================================================
// Waveguide Calculations
// ============================================================================

export class WaveguideCalculator {
  /**
   * Calculate effective index of a strip waveguide (approximate)
   * @param geometry Waveguide geometry parameters
   * @param wavelength Operating wavelength in nm
   * @returns Effective refractive index
   */
  static calculateEffectiveIndex(
    geometry: WaveguideGeometry,
    wavelength: Wavelength
  ): RefractiveIndex {
    const nCore =
      geometry.material === 'Si'
        ? MaterialProperties.Silicon.refractiveIndex1550nm
        : MaterialProperties.SiliconNitride.refractiveIndex1550nm;
    const nClad = MaterialProperties.SiliconDioxide.refractiveIndex1550nm;

    // Simplified effective index approximation
    const aspectRatio = geometry.height / geometry.width;
    const confinement = 0.7 + 0.2 * Math.tanh(aspectRatio - 0.5);
    const neff = nClad + (nCore - nClad) * confinement;

    return neff;
  }

  /**
   * Calculate group index from effective index
   * @param neff Effective refractive index
   * @param wavelength Wavelength in nm
   * @returns Group index
   */
  static calculateGroupIndex(
    neff: RefractiveIndex,
    wavelength: Wavelength
  ): number {
    // Simplified: ng ≈ neff - λ × dneff/dλ
    // For silicon at telecom wavelengths, ng ≈ 4.2
    const dispersion = wavelength > 1500 ? -0.001 : -0.0008;
    const ng = neff - wavelength * dispersion;
    return ng;
  }

  /**
   * Estimate propagation loss for a waveguide
   * @param geometry Waveguide geometry
   * @param sidewallRoughness RMS sidewall roughness in nm
   * @returns Propagation loss in dB/cm
   */
  static estimatePropagationLoss(
    geometry: WaveguideGeometry,
    sidewallRoughness: number = 2
  ): number {
    // Loss increases with roughness and decreases with waveguide width
    const roughnessFactor = Math.pow(sidewallRoughness / 2, 2);
    const widthFactor = 450 / geometry.width;
    const baseLoss = 0.5; // dB/cm for ideal waveguide

    return baseLoss + roughnessFactor * widthFactor * 2;
  }

  /**
   * Calculate waveguide properties
   * @param geometry Waveguide geometry
   * @param wavelength Operating wavelength in nm
   * @returns Complete waveguide properties
   */
  static calculateProperties(
    geometry: WaveguideGeometry,
    wavelength: Wavelength
  ): WaveguideProperties {
    const effectiveIndex = this.calculateEffectiveIndex(geometry, wavelength);
    const groupIndex = this.calculateGroupIndex(effectiveIndex, wavelength);
    const propagationLoss = this.estimatePropagationLoss(geometry);
    const modeFieldDiameter = Math.sqrt(
      (wavelength * 1e-9) / (Math.PI * (effectiveIndex - 1.444))
    ) * 1e6; // Convert to µm
    const confinementFactor = 0.85; // Typical for strip waveguides

    return {
      effectiveIndex,
      groupIndex,
      propagationLoss,
      modeFieldDiameter,
      confinementFactor,
    };
  }
}

// ============================================================================
// WDM Channel Grid Generator
// ============================================================================

export class WDMChannelCalculator {
  /**
   * Generate WDM channel grid based on ITU-T G.694.1
   * @param grid WDM channel grid specification
   * @returns Array of WDM channels
   */
  static generateChannelGrid(grid: WDMChannelGrid): WDMChannel[] {
    const channels: WDMChannel[] = [];
    const c = PhysicalConstants.SPEED_OF_LIGHT * 1e-12; // THz·nm

    for (let i = 0; i < grid.numChannels; i++) {
      const freqOffset =
        (i - Math.floor(grid.numChannels / 2)) * (grid.channelSpacing / 1000); // THz
      const frequency = grid.centerFrequency + freqOffset;
      const wavelength = c / frequency;

      channels.push({
        channelNumber: i + 1,
        wavelength,
        frequency,
        dataRate: 100, // Default 100 Gbps per channel
        modulation: {
          type: 'PAM4',
          bitsPerSymbol: 2,
          symbolRate: 53.125,
          requiredOSNR: 23,
        },
      });
    }

    return channels;
  }

  /**
   * Calculate wavelength from frequency
   * @param frequency Frequency in THz
   * @returns Wavelength in nm
   */
  static frequencyToWavelength(frequency: number): Wavelength {
    const c = PhysicalConstants.SPEED_OF_LIGHT * 1e-12; // THz·nm
    return c / frequency;
  }

  /**
   * Calculate frequency from wavelength
   * @param wavelength Wavelength in nm
   * @returns Frequency in THz
   */
  static wavelengthToFrequency(wavelength: Wavelength): number {
    const c = PhysicalConstants.SPEED_OF_LIGHT * 1e-12; // THz·nm
    return c / wavelength;
  }
}

// ============================================================================
// Link Budget Calculator
// ============================================================================

export class LinkBudgetCalculator {
  /**
   * Calculate optical link budget
   * @param txPower Transmitter power in dBm
   * @param fiberLoss Fiber loss in dB
   * @param connectorLoss Connector loss in dB
   * @param otherLosses Other losses in dB
   * @param rxSensitivity Receiver sensitivity in dBm
   * @returns Complete link budget
   */
  static calculateLinkBudget(
    txPower: OpticalPower,
    fiberLoss: number,
    connectorLoss: number,
    otherLosses: number,
    rxSensitivity: OpticalPower
  ): LinkBudget {
    const totalLoss = fiberLoss + connectorLoss + otherLosses;
    const linkMargin = txPower - totalLoss - rxSensitivity;

    return {
      txPower,
      fiberLoss,
      connectorLoss,
      otherLosses,
      totalLoss,
      rxSensitivity,
      linkMargin,
    };
  }

  /**
   * Calculate fiber loss for given length and fiber type
   * @param lengthKm Length in kilometers
   * @param wavelength Wavelength in nm
   * @param fiberType "SMF" or "MMF"
   * @returns Fiber loss in dB
   */
  static calculateFiberLoss(
    lengthKm: number,
    wavelength: Wavelength,
    fiberType: 'SMF' | 'MMF'
  ): number {
    let attenuationCoeff: number; // dB/km

    if (fiberType === 'SMF') {
      // Single-mode fiber attenuation
      if (wavelength < 1350) {
        attenuationCoeff = 0.35; // O-band: 0.35 dB/km
      } else {
        attenuationCoeff = 0.2; // C-band: 0.2 dB/km
      }
    } else {
      // Multi-mode fiber attenuation (higher)
      attenuationCoeff = wavelength < 1350 ? 0.7 : 0.5;
    }

    return lengthKm * attenuationCoeff;
  }

  /**
   * Convert optical power between mW and dBm
   * @param value Power value
   * @param fromUnit Input unit ("mW" or "dBm")
   * @returns Power in the opposite unit
   */
  static convertPower(value: number, fromUnit: 'mW' | 'dBm'): number {
    if (fromUnit === 'mW') {
      // mW to dBm
      return 10 * Math.log10(value);
    } else {
      // dBm to mW
      return Math.pow(10, value / 10);
    }
  }
}

// ============================================================================
// Modulator Performance Calculator
// ============================================================================

export class ModulatorCalculator {
  /**
   * Calculate modulator insertion loss
   * @param vPi Vπ voltage in volts
   * @param length Modulator length in mm
   * @param driveVoltage Applied drive voltage in Vpp
   * @returns Insertion loss in dB
   */
  static calculateInsertionLoss(
    vPi: number,
    length: number,
    driveVoltage: number
  ): number {
    // Simplified model: loss increases with length and decreases with drive voltage
    const baseLoss = 3.0; // dB (coupling and waveguide loss)
    const modulationLoss = (driveVoltage / vPi) * 0.5;
    const lengthLoss = length * 0.2; // Assume 0.2 dB/mm

    return baseLoss + lengthLoss - modulationLoss;
  }

  /**
   * Estimate bandwidth from RC time constant
   * @param resistance Resistance in Ohms
   * @param capacitance Capacitance in fF
   * @returns 3dB bandwidth in GHz
   */
  static estimateBandwidth(resistance: number, capacitance: number): number {
    const capacitanceFarad = capacitance * 1e-15; // fF to F
    const rcTimeConstant = resistance * capacitanceFarad; // seconds
    const bandwidth3dB = 1 / (2 * Math.PI * rcTimeConstant); // Hz
    return bandwidth3dB / 1e9; // Convert to GHz
  }

  /**
   * Calculate energy per bit for modulator
   * @param specs Modulator specifications
   * @param dataRate Data rate in Gbps
   * @returns Energy per bit in fJ/bit
   */
  static calculateEnergyPerBit(
    specs: ModulatorSpecs,
    dataRate: DataRate
  ): number {
    // P = C × V² × f
    // E/bit = P / dataRate
    const capacitance = 30e-15; // Assume 30 fF typical
    const voltage = specs.driveVoltage;
    const frequency = dataRate * 1e9; // Hz
    const power = capacitance * voltage * voltage * frequency; // Watts
    const energyPerBit = power / (dataRate * 1e9); // J/bit
    return energyPerBit * 1e15; // Convert to fJ/bit
  }
}

// ============================================================================
// Photodetector Calculator
// ============================================================================

export class PhotodetectorCalculator {
  /**
   * Calculate photocurrent from optical power and responsivity
   * @param opticalPower Optical power in mW
   * @param responsivity Responsivity in A/W
   * @returns Photocurrent in mA
   */
  static calculatePhotocurrent(
    opticalPower: number,
    responsivity: number
  ): number {
    return opticalPower * responsivity;
  }

  /**
   * Calculate receiver sensitivity
   * @param specs Photodetector specifications
   * @param dataRate Data rate in Gbps
   * @param targetBER Target bit error rate
   * @returns Sensitivity in dBm
   */
  static calculateSensitivity(
    specs: PhotodetectorSpecs,
    dataRate: DataRate,
    targetBER: number = 1e-12
  ): OpticalPower {
    // Simplified sensitivity calculation
    // Real calculation would involve Q-factor, noise analysis, etc.
    const qFactor = Math.sqrt(2) * this.erfcinv(2 * targetBER);
    const bandwidth = dataRate * 1e9; // Hz
    const thermalNoise = 4 * PhysicalConstants.BOLTZMANN_CONSTANT * 300 * bandwidth; // Assume 300K, 50Ω
    const shotNoise = specs.darkCurrent * 1e-9 * PhysicalConstants.ELEMENTARY_CHARGE * bandwidth * 2;

    const requiredCurrent = qFactor * Math.sqrt(thermalNoise + shotNoise) / PhysicalConstants.ELEMENTARY_CHARGE;
    const requiredPower = requiredCurrent / specs.responsivity; // Watts
    const sensitivityDBm = 10 * Math.log10(requiredPower * 1000); // Convert to dBm

    return sensitivityDBm;
  }

  private static erfcinv(x: number): number {
    // Approximate inverse complementary error function
    // Good enough for sensitivity estimates
    if (x >= 2) return -100;
    if (x <= 0) return 100;

    const a = 0.147;
    const lnx = Math.log(x * (2 - x));
    const part1 = (2 / (Math.PI * a)) + lnx / 2;
    const part2 = lnx / a;

    return Math.sign(1 - x) * Math.sqrt(-part1 + Math.sqrt(part1 * part1 - part2));
  }
}

// ============================================================================
// Thermal Calculator
// ============================================================================

export class ThermalCalculator {
  /**
   * Calculate wavelength shift due to temperature change
   * @param wavelength Initial wavelength in nm
   * @param tempChange Temperature change in °C
   * @param thermoOpticCoeff Thermo-optic coefficient in nm/°C
   * @returns Wavelength shift in nm
   */
  static calculateWavelengthShift(
    wavelength: Wavelength,
    tempChange: Temperature,
    thermoOpticCoeff: number = 0.1
  ): number {
    return tempChange * thermoOpticCoeff;
  }

  /**
   * Calculate junction temperature from power dissipation
   * @param powerDissipation Power in watts
   * @param thermalResistance Thermal resistance in °C/W
   * @param ambientTemp Ambient temperature in °C
   * @returns Junction temperature in °C
   */
  static calculateJunctionTemperature(
    powerDissipation: number,
    thermalResistance: number,
    ambientTemp: Temperature
  ): Temperature {
    return ambientTemp + powerDissipation * thermalResistance;
  }
}

// ============================================================================
// BER and Signal Quality
// ============================================================================

export class SignalQualityCalculator {
  /**
   * Calculate Q-factor from BER
   * @param ber Bit error rate
   * @returns Q-factor in dB
   */
  static berToQFactor(ber: number): number {
    const qLinear = Math.sqrt(2) * this.erfcinv(2 * ber);
    return 20 * Math.log10(qLinear);
  }

  /**
   * Calculate BER from Q-factor
   * @param qFactorDB Q-factor in dB
   * @returns Bit error rate
   */
  static qFactorToBER(qFactorDB: number): number {
    const qLinear = Math.pow(10, qFactorDB / 20);
    return 0.5 * this.erfc(qLinear / Math.sqrt(2));
  }

  private static erfcinv(x: number): number {
    return PhotodetectorCalculator['erfcinv'](x);
  }

  private static erfc(x: number): number {
    // Complementary error function approximation
    const t = 1 / (1 + 0.5 * Math.abs(x));
    const tau = t * Math.exp(-x * x - 1.26551223 +
      t * (1.00002368 + t * (0.37409196 + t * (0.09678418 +
      t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 +
      t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))));
    return x >= 0 ? tau : 2 - tau;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

export class PhotonicUtils {
  /**
   * Check if device meets WIA-SEMI-006 compliance
   * @param deviceSpecs Device specifications
   * @returns Compliance check result
   */
  static checkCompliance(deviceSpecs: any): {
    compliant: boolean;
    issues: string[];
  } {
    const issues: string[] = [];

    // Check modulator specs (if applicable)
    if ('bandwidth' in deviceSpecs && deviceSpecs.bandwidth < 50) {
      issues.push('Bandwidth below 50 GHz requirement');
    }

    if ('insertionLoss' in deviceSpecs && deviceSpecs.insertionLoss > 6) {
      issues.push('Insertion loss exceeds 6 dB limit');
    }

    // Check photodetector specs (if applicable)
    if ('responsivity' in deviceSpecs && deviceSpecs.responsivity < 0.8) {
      issues.push('Responsivity below 0.8 A/W requirement');
    }

    if ('darkCurrent' in deviceSpecs && deviceSpecs.darkCurrent > 100) {
      issues.push('Dark current exceeds 100 nA limit');
    }

    return {
      compliant: issues.length === 0,
      issues,
    };
  }

  /**
   * Format wavelength for ITU-T grid designation
   * @param wavelength Wavelength in nm
   * @returns ITU-T band designation
   */
  static getITUBand(wavelength: Wavelength): string {
    if (wavelength >= 1260 && wavelength < 1360) return 'O-Band';
    if (wavelength >= 1360 && wavelength < 1460) return 'E-Band';
    if (wavelength >= 1460 && wavelength < 1530) return 'S-Band';
    if (wavelength >= 1530 && wavelength < 1565) return 'C-Band';
    if (wavelength >= 1565 && wavelength <= 1625) return 'L-Band';
    return 'Unknown';
  }
}

// ============================================================================
// Default Export
// ============================================================================

export default {
  PhysicalConstants,
  MaterialProperties,
  WaveguideCalculator,
  WDMChannelCalculator,
  LinkBudgetCalculator,
  ModulatorCalculator,
  PhotodetectorCalculator,
  ThermalCalculator,
  SignalQualityCalculator,
  PhotonicUtils,
};
