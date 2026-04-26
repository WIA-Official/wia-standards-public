/**
 * WIA-QUA-009: Photonics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for photonics including:
 * - Photon energy and wavelength calculations
 * - Optical fiber and waveguide design
 * - Laser system analysis
 * - Photodetector characterization
 * - LiDAR simulation
 * - Photonic crystal modeling
 */

import {
  PhotonEnergyParams,
  PhotonEnergyResult,
  FiberDesignParams,
  OpticalFiber,
  LaserSystem,
  Photodetector,
  LiDARSystem,
  LiDARPoint,
  PhotonicCrystal,
  WavelengthConversion,
  WavelengthUnit,
  EnergyUnit,
  PHOTONICS_CONSTANTS,
  PhotonicsErrorCode,
  PhotonicsError,
  Photon,
  OpticalMaterial,
  SellmeierCoefficients,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-009 Photonics SDK
 */
export class PhotonicsSDK {
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
   * Calculate photon energy from wavelength or frequency
   *
   * @param params - Photon energy calculation parameters
   * @returns Photon energy result with multiple representations
   */
  calculatePhotonEnergy(params: PhotonEnergyParams): PhotonEnergyResult {
    const { wavelength, frequency, count, unit = 'J' } = params;

    // Validate inputs
    if (!wavelength && !frequency) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Either wavelength or frequency must be provided'
      );
    }

    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;
    const h = PHOTONICS_CONSTANTS.PLANCK_CONSTANT;

    // Calculate wavelength and frequency
    let lambda: number;
    let nu: number;

    if (wavelength) {
      if (wavelength <= 0) {
        throw new PhotonicsError(
          PhotonicsErrorCode.INVALID_WAVELENGTH,
          'Wavelength must be positive'
        );
      }
      lambda = wavelength;
      nu = c / lambda;
    } else {
      if (frequency! <= 0) {
        throw new PhotonicsError(
          PhotonicsErrorCode.INVALID_PARAMETERS,
          'Frequency must be positive'
        );
      }
      nu = frequency!;
      lambda = c / nu;
    }

    // Calculate energy: E = hν = hc/λ
    const energy = h * nu;

    // Convert to requested unit
    let value: number;
    switch (unit) {
      case 'J':
        value = energy;
        break;
      case 'eV':
        value = energy / PHOTONICS_CONSTANTS.EV_TO_JOULE;
        break;
      case 'meV':
        value = (energy / PHOTONICS_CONSTANTS.EV_TO_JOULE) * 1000;
        break;
      case 'keV':
        value = energy / PHOTONICS_CONSTANTS.EV_TO_JOULE / 1000;
        break;
      case 'MeV':
        value = energy / PHOTONICS_CONSTANTS.EV_TO_JOULE / 1e6;
        break;
      default:
        value = energy;
    }

    // Calculate total power if count is provided
    const power = count ? energy * count : undefined;

    // Determine color for visible wavelengths
    const color = this.wavelengthToColor(lambda);

    return {
      wavelength: lambda,
      frequency: nu,
      energy,
      value,
      unit,
      power,
      color,
    };
  }

  /**
   * Convert wavelength between units
   *
   * @param value - Wavelength value
   * @param fromUnit - Source unit
   * @param toUnit - Target unit (default: meters)
   * @returns Wavelength conversion result
   */
  convertWavelength(
    value: number,
    fromUnit: WavelengthUnit,
    toUnit: WavelengthUnit = 'm'
  ): WavelengthConversion {
    // Convert to meters first
    let meters: number;
    switch (fromUnit) {
      case 'm':
        meters = value;
        break;
      case 'mm':
        meters = value * 1e-3;
        break;
      case 'um':
        meters = value * 1e-6;
        break;
      case 'nm':
        meters = value * 1e-9;
        break;
      case 'Å':
        meters = value * 1e-10;
        break;
      default:
        meters = value;
    }

    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;
    const h = PHOTONICS_CONSTANTS.PLANCK_CONSTANT;

    const frequency = c / meters;
    const energy = h * frequency;
    const energyEV = energy / PHOTONICS_CONSTANTS.EV_TO_JOULE;
    const nanometers = meters * 1e9;

    return {
      input: value,
      inputUnit: fromUnit,
      meters,
      nanometers,
      frequency,
      energyEV,
    };
  }

  /**
   * Design an optical fiber with given parameters
   *
   * @param params - Fiber design parameters
   * @returns Optical fiber configuration
   */
  designOpticalFiber(params: FiberDesignParams): OpticalFiber {
    const {
      coreRadius,
      claddingRadius,
      coreIndex,
      claddingIndex,
      wavelength,
      preferredType,
    } = params;

    // Validate inputs
    if (coreRadius <= 0 || claddingRadius <= 0) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Radii must be positive'
      );
    }

    if (claddingRadius <= coreRadius) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Cladding radius must be greater than core radius'
      );
    }

    if (coreIndex <= claddingIndex) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Core index must be greater than cladding index for guiding'
      );
    }

    // Calculate numerical aperture: NA = √(n₁² - n₂²)
    const numericalAperture = Math.sqrt(
      coreIndex * coreIndex - claddingIndex * claddingIndex
    );

    // Calculate V-number: V = (2π/λ) × a × NA
    const vNumber = (2 * Math.PI * coreRadius * numericalAperture) / wavelength;

    // Determine fiber type
    let type: OpticalFiber['type'];
    if (preferredType) {
      type = preferredType;
    } else {
      // Single-mode if V < 2.405, multi-mode otherwise
      type = vNumber < 2.405 ? 'single-mode' : 'multi-mode';
    }

    // Calculate mode field diameter (for single-mode)
    const modeFieldDiameter =
      type === 'single-mode'
        ? 2 *
          coreRadius *
          (0.65 + 1.619 * Math.pow(vNumber, -3 / 2) + 2.879 * Math.pow(vNumber, -6))
        : undefined;

    // Estimate attenuation (dB/km) - simplified model
    const attenuationBase = wavelength === 1550e-9 ? 0.2 : wavelength === 1310e-9 ? 0.35 : 0.5;
    const attenuation = attenuationBase;

    // Estimate dispersion (ps/(nm·km)) - simplified
    const dispersion = wavelength === 1550e-9 ? 17 : wavelength === 1310e-9 ? 0 : 3.5;

    // Calculate effective area (m²)
    const effectiveArea = modeFieldDiameter
      ? Math.PI * Math.pow(modeFieldDiameter / 2, 2)
      : Math.PI * Math.pow(coreRadius, 2);

    // Nonlinear coefficient (1/(W·m)) - typical for silica
    const nonlinearCoefficient = 1.3e-3 / effectiveArea;

    // Generate fiber ID
    const id = `FIBER-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    return {
      id,
      type,
      coreRadius,
      claddingRadius,
      coreIndex,
      claddingIndex,
      wavelength,
      numericalAperture,
      modeFieldDiameter,
      attenuation,
      dispersion,
      effectiveArea,
      nonlinearCoefficient,
    };
  }

  /**
   * Analyze laser system performance
   *
   * @param laser - Laser system configuration
   * @returns Analysis results including efficiency, beam parameters
   */
  analyzeLaserSystem(laser: LaserSystem): {
    outputPower: number;
    electricalPower: number;
    beamDiameter: number;
    intensity: number;
    photonFlux: number;
    heatDissipation: number;
  } {
    const { power, efficiency, wavelength, beamQuality = 1.0, divergence = 1e-3 } = laser;

    // Calculate electrical input power
    const electricalPower = power / efficiency;

    // Heat dissipation
    const heatDissipation = electricalPower - power;

    // Estimate beam diameter (at 1/e² point)
    // Simplified: assume 1mm for typical laser pointer
    const beamDiameter = 1e-3 * beamQuality;

    // Calculate intensity (W/m²)
    const beamArea = Math.PI * Math.pow(beamDiameter / 2, 2);
    const intensity = power / beamArea;

    // Calculate photon flux (photons/s)
    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;
    const h = PHOTONICS_CONSTANTS.PLANCK_CONSTANT;
    const lambda = Array.isArray(wavelength) ? wavelength[0] : wavelength;
    const photonEnergy = (h * c) / lambda;
    const photonFlux = power / photonEnergy;

    return {
      outputPower: power,
      electricalPower,
      beamDiameter,
      intensity,
      photonFlux,
      heatDissipation,
    };
  }

  /**
   * Simulate LiDAR range measurement
   *
   * @param system - LiDAR system configuration
   * @param targetDistance - Distance to target in meters
   * @param targetReflectivity - Target reflectivity (0-1)
   * @returns Simulated LiDAR measurement
   */
  simulateLiDAR(
    system: LiDARSystem,
    targetDistance: number,
    targetReflectivity: number = 0.1
  ): {
    detected: boolean;
    range: number;
    timeOfFlight: number;
    returnPower: number;
    snr: number;
    accuracy: number;
  } {
    const { pulseEnergy, wavelength, sensitivity, rangeResolution } = system;

    if (targetDistance < 0) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Target distance must be non-negative'
      );
    }

    if (targetReflectivity < 0 || targetReflectivity > 1) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Reflectivity must be between 0 and 1'
      );
    }

    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;

    // Calculate time of flight (round trip)
    const timeOfFlight = (2 * targetDistance) / c;

    // LiDAR equation: P_r = (P_t × ρ × A_r) / (4π × R²)
    // Simplified: geometric spreading loss
    const geometricLoss = Math.pow(targetDistance, 2);
    const atmosphericLoss = Math.exp(-0.1 * targetDistance / 1000); // Simplified atmospheric absorption
    const receiverArea = 0.01; // 10 cm² aperture (typical)

    const returnPower =
      (pulseEnergy * targetReflectivity * receiverArea * atmosphericLoss) /
      (4 * Math.PI * geometricLoss);

    // Check if signal is above sensitivity
    const detected = returnPower >= sensitivity;

    // Calculate SNR (simplified)
    const noiseFloor = sensitivity / 10; // Assume 10dB SNR at sensitivity
    const snr = detected ? 10 * Math.log10(returnPower / noiseFloor) : 0;

    // Range accuracy depends on pulse duration and SNR
    const accuracy = detected ? rangeResolution / Math.sqrt(1 + snr / 10) : Infinity;

    return {
      detected,
      range: detected ? targetDistance : NaN,
      timeOfFlight,
      returnPower,
      snr,
      accuracy,
    };
  }

  /**
   * Calculate refractive index using Sellmeier equation
   *
   * @param wavelength - Wavelength in meters
   * @param coeffs - Sellmeier coefficients
   * @returns Refractive index
   */
  calculateRefractiveIndex(
    wavelength: number,
    coeffs: SellmeierCoefficients
  ): number {
    // Sellmeier equation: n² = 1 + B₁λ²/(λ² - C₁) + B₂λ²/(λ² - C₂) + B₃λ²/(λ² - C₃)
    const lambda_um = wavelength * 1e6; // Convert to micrometers
    const lambda2 = lambda_um * lambda_um;

    const n2 =
      1 +
      (coeffs.B1 * lambda2) / (lambda2 - coeffs.C1) +
      (coeffs.B2 * lambda2) / (lambda2 - coeffs.C2) +
      (coeffs.B3 * lambda2) / (lambda2 - coeffs.C3);

    return Math.sqrt(n2);
  }

  /**
   * Calculate LED luminous efficacy
   *
   * @param electricalPower - Input electrical power in watts
   * @param luminousFlux - Output luminous flux in lumens
   * @returns Efficacy in lumens per watt
   */
  calculateLEDEfficacy(electricalPower: number, luminousFlux: number): number {
    if (electricalPower <= 0) {
      throw new PhotonicsError(
        PhotonicsErrorCode.INVALID_PARAMETERS,
        'Electrical power must be positive'
      );
    }

    return luminousFlux / electricalPower;
  }

  /**
   * Calculate photodetector quantum efficiency
   *
   * @param responsivity - Responsivity in A/W
   * @param wavelength - Operating wavelength in meters
   * @returns Quantum efficiency (0-1)
   */
  calculateQuantumEfficiency(responsivity: number, wavelength: number): number {
    const h = PHOTONICS_CONSTANTS.PLANCK_CONSTANT;
    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;
    const e = PHOTONICS_CONSTANTS.ELEMENTARY_CHARGE;

    // QE = (R × h × c) / (e × λ)
    const qe = (responsivity * h * c) / (e * wavelength);

    return Math.min(qe, 1.0); // Cap at 100%
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Convert wavelength to approximate color name
   */
  private wavelengthToColor(wavelength: number): string | undefined {
    const lambda_nm = wavelength * 1e9;

    if (lambda_nm < 380 || lambda_nm > 750) {
      return undefined; // Not visible
    }

    if (lambda_nm < 450) return 'violet';
    if (lambda_nm < 495) return 'blue';
    if (lambda_nm < 570) return 'green';
    if (lambda_nm < 590) return 'yellow';
    if (lambda_nm < 620) return 'orange';
    return 'red';
  }

  /**
   * Calculate group velocity in material
   */
  private calculateGroupVelocity(
    wavelength: number,
    refractiveIndex: number,
    dispersion: number
  ): number {
    const c = PHOTONICS_CONSTANTS.SPEED_OF_LIGHT;

    // Simplified: v_g = c / (n - λ × dn/dλ)
    // Assume dn/dλ ≈ -dispersion × 1e-6
    const dnDlambda = -dispersion * 1e-6;
    const vg = c / (refractiveIndex - wavelength * dnDlambda);

    return vg;
  }

  /**
   * Format wavelength in human-readable form
   */
  formatWavelength(wavelength: number): string {
    const nm = wavelength * 1e9;
    const um = wavelength * 1e6;

    if (nm < 1000) {
      return `${nm.toFixed(1)} nm`;
    } else if (um < 1000) {
      return `${um.toFixed(2)} μm`;
    } else {
      return `${wavelength.toExponential(2)} m`;
    }
  }

  /**
   * Format power in human-readable form
   */
  formatPower(power: number): string {
    if (power < 1e-9) return `${(power * 1e12).toFixed(2)} pW`;
    if (power < 1e-6) return `${(power * 1e9).toFixed(2)} nW`;
    if (power < 1e-3) return `${(power * 1e6).toFixed(2)} μW`;
    if (power < 1) return `${(power * 1e3).toFixed(2)} mW`;
    if (power < 1e3) return `${power.toFixed(2)} W`;
    if (power < 1e6) return `${(power / 1e3).toFixed(2)} kW`;
    return `${(power / 1e6).toFixed(2)} MW`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate photon energy (standalone function)
 */
export function calculatePhotonEnergy(params: PhotonEnergyParams): PhotonEnergyResult {
  const sdk = new PhotonicsSDK();
  return sdk.calculatePhotonEnergy(params);
}

/**
 * Design optical fiber (standalone function)
 */
export function designOpticalFiber(params: FiberDesignParams): OpticalFiber {
  const sdk = new PhotonicsSDK();
  return sdk.designOpticalFiber(params);
}

/**
 * Analyze laser system (standalone function)
 */
export function analyzeLaserSystem(laser: LaserSystem) {
  const sdk = new PhotonicsSDK();
  return sdk.analyzeLaserSystem(laser);
}

/**
 * Simulate LiDAR (standalone function)
 */
export function simulateLiDAR(
  system: LiDARSystem,
  targetDistance: number,
  targetReflectivity?: number
) {
  const sdk = new PhotonicsSDK();
  return sdk.simulateLiDAR(system, targetDistance, targetReflectivity);
}

// ============================================================================
// Material Database
// ============================================================================

/**
 * Common optical materials with Sellmeier coefficients
 */
export const OPTICAL_MATERIALS = {
  /** Fused silica (SiO₂) */
  FUSED_SILICA: {
    B1: 0.6961663,
    B2: 0.4079426,
    B3: 0.8974794,
    C1: 0.0684043,
    C2: 0.1162414,
    C3: 9.896161,
  } as SellmeierCoefficients,

  /** BK7 glass */
  BK7: {
    B1: 1.03961212,
    B2: 0.231792344,
    B3: 1.01046945,
    C1: 0.00600069867,
    C2: 0.0200179144,
    C3: 103.560653,
  } as SellmeierCoefficients,

  /** Sapphire (Al₂O₃) */
  SAPPHIRE: {
    B1: 1.4313493,
    B2: 0.65054713,
    B3: 5.3414021,
    C1: 0.0052799261,
    C2: 0.0142382647,
    C3: 325.017834,
  } as SellmeierCoefficients,
};

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { PhotonicsSDK, OPTICAL_MATERIALS };
export default PhotonicsSDK;

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
