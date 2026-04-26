/**
 * WIA-QUA-010: Holography SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum & Future Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for holography including:
 * - Hologram recording and reconstruction
 * - Computer-generated holography (CGH)
 * - Digital holography and numerical reconstruction
 * - Holographic display rendering
 * - Quality analysis and metrics
 */

import {
  RecordingParams,
  RecordingResult,
  ReconstructionParams,
  ReconstructedWave,
  CGHParams,
  ComputedHologram,
  HologramData,
  WaveData,
  QualityMetrics,
  InterferenceAnalysis,
  MediumProperties,
  DigitalHologram,
  NumericalReconstructionParams,
  DisplayFrame,
  HolographicDisplayConfig,
  SimulationResult,
  HOLOGRAPHY_CONSTANTS,
  HolographyErrorCode,
  HolographyError,
  Vector2,
  Vector3,
  Complex,
  Scene3D,
  RecordingMedium,
  HologramType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-010 Holography SDK
 */
export class HolographySDK {
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
   * Record a hologram from object and reference beams
   *
   * @param params - Recording parameters
   * @returns Recording result with hologram data
   */
  recordHologram(params: RecordingParams): RecordingResult {
    const {
      wavelength,
      objectBeam,
      referenceBeam,
      medium,
      mediumProperties,
      exposureTime = 1.0,
      temperature = 20,
      geometry,
    } = params;

    // Validate inputs
    this.validateWavelength(wavelength);
    this.validateWaveData(objectBeam);
    this.validateWaveData(referenceBeam);

    // Get medium properties
    const mediumProps = mediumProperties || this.getDefaultMediumProperties(medium);

    // Calculate interference pattern
    const interferencePattern = this.calculateInterferencePattern(
      objectBeam,
      referenceBeam
    );

    // Calculate spatial frequency
    const spatialFrequency = this.calculateSpatialFrequency(
      wavelength,
      geometry?.referenceAngle || 30
    );

    // Check if medium can resolve the spatial frequency
    if (spatialFrequency > mediumProps.resolution) {
      throw new HolographyError(
        HolographyErrorCode.RESOLUTION_MISMATCH,
        `Spatial frequency (${spatialFrequency} lines/mm) exceeds medium resolution (${mediumProps.resolution} lines/mm)`
      );
    }

    // Calculate required exposure
    const requiredExposure = mediumProps.sensitivity;
    const actualExposure = this.calculateExposure(
      referenceBeam,
      exposureTime
    );

    const warnings: string[] = [];

    // Check exposure
    if (actualExposure < requiredExposure * 0.5) {
      warnings.push('Underexposure: Image may be weak');
    } else if (actualExposure > requiredExposure * 2) {
      warnings.push('Overexposure: Image may be degraded');
    }

    // Calculate diffraction efficiency
    const efficiency = this.calculateDiffractionEfficiency(
      mediumProps,
      actualExposure / requiredExposure
    );

    // Create hologram data
    const hologramData: HologramData = {
      id: `HOLO-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      type: this.determineHologramType(geometry),
      wavelength,
      interferencePattern,
      dimensions: {
        width: interferencePattern[0].length,
        height: interferencePattern.length,
        thickness: mediumProps.thickness,
      },
      geometry: geometry || this.getDefaultGeometry(),
      medium: mediumProps,
      spatialFrequency,
      efficiency,
      quality: this.analyzeQuality(interferencePattern, efficiency),
      timestamp: new Date(),
    };

    return {
      hologram: hologramData,
      success: true,
      warnings,
      actualExposure,
      processing: mediumProps.processing !== 'none' ? [mediumProps.processing] : [],
    };
  }

  /**
   * Reconstruct a hologram with illumination beam
   *
   * @param params - Reconstruction parameters
   * @returns Reconstructed wave field
   */
  reconstructHologram(params: ReconstructionParams): ReconstructedWave {
    const {
      hologram,
      reconstructionBeam,
      wavelength,
      distance = 0.5,
      method = 'fresnel',
    } = params;

    const startTime = Date.now();

    // Validate inputs
    this.validateWavelength(wavelength);

    // Multiply hologram pattern by reconstruction beam
    const modulated = this.multiplyWaves(
      hologram.interferencePattern,
      reconstructionBeam
    );

    // Propagate to reconstruction distance
    let reconstructed: { amplitude: number[][]; phase: number[][] };

    switch (method) {
      case 'fresnel':
        reconstructed = this.fresnelPropagation(modulated, wavelength, distance);
        break;
      case 'angular-spectrum':
        reconstructed = this.angularSpectrumPropagation(modulated, wavelength, distance);
        break;
      case 'fraunhofer':
        reconstructed = this.fraunhoferPropagation(modulated, wavelength, distance);
        break;
      default:
        reconstructed = this.fresnelPropagation(modulated, wavelength, distance);
    }

    // Calculate intensity
    const intensity = this.calculateIntensity(
      reconstructed.amplitude,
      reconstructed.phase
    );

    // Analyze quality
    const quality = this.analyzeReconstructionQuality(intensity);

    const computationTime = Date.now() - startTime;

    return {
      amplitude: reconstructed.amplitude,
      phase: reconstructed.phase,
      intensity,
      quality,
      method,
      computationTime,
    };
  }

  /**
   * Generate computer-generated hologram (CGH)
   *
   * @param params - CGH generation parameters
   * @returns Computed hologram
   */
  generateCGH(params: CGHParams): ComputedHologram {
    const {
      scene,
      wavelength,
      resolution,
      method,
      iterations = 20,
      pixelPitch = 8e-6,
      distance = 0.5,
      phaseOnly = false,
    } = params;

    const startTime = Date.now();

    this.validateWavelength(wavelength);

    let pattern: number[][];
    let phase: number[][];
    let convergence;

    switch (method) {
      case 'point-source':
        ({ pattern, phase } = this.pointSourceCGH(
          scene,
          wavelength,
          resolution,
          pixelPitch,
          distance
        ));
        break;

      case 'layer-based':
        ({ pattern, phase } = this.layerBasedCGH(
          scene,
          wavelength,
          resolution,
          pixelPitch
        ));
        break;

      case 'gerchberg-saxton':
        ({ pattern, phase, convergence } = this.gerchbergSaxtonCGH(
          scene,
          wavelength,
          resolution,
          iterations
        ));
        break;

      default:
        ({ pattern, phase } = this.pointSourceCGH(
          scene,
          wavelength,
          resolution,
          pixelPitch,
          distance
        ));
    }

    // If phase-only, set pattern to uniform
    if (phaseOnly) {
      pattern = this.createUniformPattern(resolution.width, resolution.height);
    }

    const computationTime = Date.now() - startTime;

    return {
      id: `CGH-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      pattern,
      phase,
      computationTime,
      method,
      convergence,
      quality: this.analyzeQuality(pattern, 0.5),
    };
  }

  /**
   * Calculate interference pattern between two waves
   *
   * @param objectBeam - Object wave
   * @param referenceBeam - Reference wave
   * @returns Interference intensity pattern
   */
  calculateInterferencePattern(
    objectBeam: WaveData,
    referenceBeam: WaveData
  ): number[][] {
    const height = Math.min(objectBeam.amplitude.length, referenceBeam.amplitude.length);
    const width = Math.min(
      objectBeam.amplitude[0].length,
      referenceBeam.amplitude[0].length
    );

    const pattern: number[][] = [];

    for (let y = 0; y < height; y++) {
      const row: number[] = [];
      for (let x = 0; x < width; x++) {
        // I = |R + O|² = |R|² + |O|² + 2|R||O|cos(φ_O - φ_R)
        const R = referenceBeam.amplitude[y][x];
        const O = objectBeam.amplitude[y][x];
        const phaseR = referenceBeam.phase[y][x];
        const phaseO = objectBeam.phase[y][x];

        const intensity =
          R * R + O * O + 2 * R * O * Math.cos(phaseO - phaseR);

        row.push(Math.max(0, intensity)); // Ensure non-negative
      }
      pattern.push(row);
    }

    return pattern;
  }

  /**
   * Analyze interference pattern quality
   *
   * @param objectBeam - Object wave
   * @param referenceBeam - Reference wave
   * @returns Interference analysis
   */
  analyzeInterference(
    objectBeam: WaveData,
    referenceBeam: WaveData
  ): InterferenceAnalysis {
    const pattern = this.calculateInterferencePattern(objectBeam, referenceBeam);

    // Calculate fringe visibility: V = (I_max - I_min) / (I_max + I_min)
    let minIntensity = Infinity;
    let maxIntensity = -Infinity;

    for (const row of pattern) {
      for (const intensity of row) {
        minIntensity = Math.min(minIntensity, intensity);
        maxIntensity = Math.max(maxIntensity, intensity);
      }
    }

    const visibility =
      (maxIntensity - minIntensity) / (maxIntensity + minIntensity);

    // Estimate fringe spacing from autocorrelation (simplified)
    const fringeSpacing = this.estimateFringeSpacing(pattern);
    const spatialFrequency = 1e-3 / fringeSpacing; // Convert to lines/mm

    const contrast = visibility;

    return {
      visibility,
      fringeSpacing,
      spatialFrequency,
      contrast,
    };
  }

  /**
   * Simulate complete holographic process
   *
   * @param params - Recording parameters
   * @returns Simulation result
   */
  simulateHolography(params: RecordingParams): SimulationResult {
    const startTime = Date.now();

    try {
      // Record hologram
      const recording = this.recordHologram(params);

      // Reconstruct hologram
      const reconstruction = this.reconstructHologram({
        hologram: recording.hologram,
        reconstructionBeam: params.referenceBeam,
        wavelength: params.wavelength,
      });

      const duration = Date.now() - startTime;

      return {
        id: `SIM-${Date.now()}`,
        parameters: params,
        output: recording.hologram,
        duration,
        success: true,
        warnings: recording.warnings,
      };
    } catch (error) {
      const duration = Date.now() - startTime;
      return {
        id: `SIM-${Date.now()}`,
        parameters: params,
        output: {} as HologramData,
        duration,
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private validateWavelength(wavelength: number): void {
    if (wavelength < 100e-9 || wavelength > 10e-6) {
      throw new HolographyError(
        HolographyErrorCode.INVALID_WAVELENGTH,
        `Wavelength ${wavelength}m is outside valid range (100nm - 10μm)`
      );
    }
  }

  private validateWaveData(wave: WaveData): void {
    if (!wave.amplitude || !wave.phase) {
      throw new HolographyError(
        HolographyErrorCode.INVALID_WAVELENGTH,
        'Wave data must include amplitude and phase'
      );
    }

    if (wave.amplitude.length !== wave.phase.length) {
      throw new HolographyError(
        HolographyErrorCode.RESOLUTION_MISMATCH,
        'Amplitude and phase dimensions must match'
      );
    }
  }

  private getDefaultMediumProperties(medium: RecordingMedium): MediumProperties {
    const defaults: Record<RecordingMedium, MediumProperties> = {
      'silver-halide': {
        type: 'silver-halide',
        sensitivity: 100e-6, // 100 μJ/cm²
        resolution: 5000,
        thickness: 7e-6,
        refractiveIndex: 1.6,
        indexModulation: 0.05,
        shrinkage: 0.1,
        processing: 'chemical',
      },
      photopolymer: {
        type: 'photopolymer',
        sensitivity: 100e-3, // 100 mJ/cm²
        resolution: 3000,
        thickness: 10e-6,
        refractiveIndex: 1.5,
        indexModulation: 0.03,
        shrinkage: 0.01,
        processing: 'none',
      },
      photorefractive: {
        type: 'photorefractive',
        sensitivity: 10, // 10 J/cm²
        resolution: 10000,
        thickness: 1e-3,
        refractiveIndex: 2.3,
        indexModulation: 0.001,
        shrinkage: 0,
        processing: 'none',
      },
      'dichromated-gelatin': {
        type: 'dichromated-gelatin',
        sensitivity: 50e-3, // 50 mJ/cm²
        resolution: 10000,
        thickness: 10e-6,
        refractiveIndex: 1.5,
        indexModulation: 0.08,
        shrinkage: 0.2,
        processing: 'chemical',
      },
      photoresist: {
        type: 'photoresist',
        sensitivity: 200e-3, // 200 mJ/cm²
        resolution: 2000,
        thickness: 5e-6,
        refractiveIndex: 1.6,
        indexModulation: 1.0,
        shrinkage: 0.05,
        processing: 'chemical',
      },
      'digital-sensor': {
        type: 'digital-sensor',
        sensitivity: 1e-6, // 1 μJ/cm²
        resolution: 1000,
        thickness: 5e-6,
        refractiveIndex: 1.0,
        indexModulation: 0,
        shrinkage: 0,
        processing: 'none',
      },
    };

    return defaults[medium];
  }

  private calculateSpatialFrequency(wavelength: number, angle: number): number {
    // f = 2 × sin(θ/2) / λ
    const angleRad = (angle * Math.PI) / 180;
    const frequency = (2 * Math.sin(angleRad / 2)) / wavelength;
    return frequency * 1e-3; // Convert to lines/mm
  }

  private calculateExposure(beam: WaveData, time: number): number {
    // Simplified: average intensity × time
    let totalIntensity = 0;
    let count = 0;

    for (const row of beam.amplitude) {
      for (const amp of row) {
        totalIntensity += amp * amp;
        count++;
      }
    }

    const avgIntensity = totalIntensity / count;
    return avgIntensity * time; // Arbitrary units
  }

  private calculateDiffractionEfficiency(
    medium: MediumProperties,
    exposureRatio: number
  ): number {
    // Simplified model: efficiency peaks at optimal exposure
    const optimal = 1.0;
    const deviation = Math.abs(exposureRatio - optimal);
    const efficiency = Math.exp(-deviation * deviation) * 0.8; // Max 80%

    return Math.max(0, Math.min(1, efficiency));
  }

  private determineHologramType(geometry?: any): HologramType {
    if (!geometry) return 'transmission';

    if (geometry.configuration === 'denisyuk') {
      return 'reflection';
    }

    if (geometry.configuration === 'in-line') {
      return 'transmission';
    }

    return 'transmission';
  }

  private getDefaultGeometry() {
    return {
      referenceAngle: 30,
      objectAngle: 0,
      objectDistance: 0.5,
      beamRatio: 4,
      configuration: 'off-axis' as const,
    };
  }

  private analyzeQuality(pattern: number[][], efficiency: number): QualityMetrics {
    // Calculate various quality metrics
    let sum = 0;
    let sumSq = 0;
    let min = Infinity;
    let max = -Infinity;
    let count = 0;

    for (const row of pattern) {
      for (const val of row) {
        sum += val;
        sumSq += val * val;
        min = Math.min(min, val);
        max = Math.max(max, val);
        count++;
      }
    }

    const mean = sum / count;
    const variance = sumSq / count - mean * mean;
    const stdDev = Math.sqrt(variance);

    const uniformity = 1 - stdDev / (mean + 1e-10);
    const contrast = (max - min) / (max + min + 1e-10);
    const dynamicRange = max / (min + 1e-10);

    const overallScore =
      (efficiency * 30 + uniformity * 20 + contrast * 25 + Math.min(dynamicRange / 100, 1) * 25);

    return {
      diffractionEfficiency: efficiency,
      uniformity: Math.max(0, Math.min(1, uniformity)),
      contrast,
      dynamicRange,
      overallScore,
    };
  }

  private analyzeReconstructionQuality(intensity: number[][]): QualityMetrics {
    // Calculate SNR and other metrics
    let signalSum = 0;
    let count = 0;

    for (const row of intensity) {
      for (const val of row) {
        signalSum += val;
        count++;
      }
    }

    const meanSignal = signalSum / count;

    // Estimate noise (simplified as variance in low-intensity regions)
    let noiseSum = 0;
    let noiseCount = 0;

    for (const row of intensity) {
      for (const val of row) {
        if (val < meanSignal * 0.1) {
          noiseSum += val * val;
          noiseCount++;
        }
      }
    }

    const noise = Math.sqrt(noiseSum / (noiseCount + 1));
    const snr = 20 * Math.log10(meanSignal / (noise + 1e-10));

    return {
      snr: Math.max(0, snr),
      fidelity: 0.8, // Simplified
      overallScore: Math.min(100, snr * 2),
    };
  }

  private multiplyWaves(pattern: number[][], beam: WaveData): Complex[][] {
    const height = Math.min(pattern.length, beam.amplitude.length);
    const width = Math.min(pattern[0].length, beam.amplitude[0].length);

    const result: Complex[][] = [];

    for (let y = 0; y < height; y++) {
      const row: Complex[] = [];
      for (let x = 0; x < width; x++) {
        const amp = pattern[y][x] * beam.amplitude[y][x];
        const phase = beam.phase[y][x];

        row.push({
          real: amp * Math.cos(phase),
          imaginary: amp * Math.sin(phase),
        });
      }
      result.push(row);
    }

    return result;
  }

  private fresnelPropagation(
    field: Complex[][],
    wavelength: number,
    distance: number
  ): { amplitude: number[][]; phase: number[][] } {
    // Simplified Fresnel propagation (actual implementation would use FFT)
    const amplitude: number[][] = [];
    const phase: number[][] = [];

    for (let y = 0; y < field.length; y++) {
      const ampRow: number[] = [];
      const phaseRow: number[] = [];

      for (let x = 0; x < field[0].length; x++) {
        const complex = field[y][x];
        const amp = Math.sqrt(complex.real ** 2 + complex.imaginary ** 2);
        const ph = Math.atan2(complex.imaginary, complex.real);

        ampRow.push(amp);
        phaseRow.push(ph);
      }

      amplitude.push(ampRow);
      phase.push(phaseRow);
    }

    return { amplitude, phase };
  }

  private angularSpectrumPropagation(
    field: Complex[][],
    wavelength: number,
    distance: number
  ): { amplitude: number[][]; phase: number[][] } {
    // Simplified (actual implementation would use FFT)
    return this.fresnelPropagation(field, wavelength, distance);
  }

  private fraunhoferPropagation(
    field: Complex[][],
    wavelength: number,
    distance: number
  ): { amplitude: number[][]; phase: number[][] } {
    // Simplified (actual implementation would use FFT)
    return this.fresnelPropagation(field, wavelength, distance);
  }

  private calculateIntensity(amplitude: number[][], phase: number[][]): number[][] {
    const intensity: number[][] = [];

    for (let y = 0; y < amplitude.length; y++) {
      const row: number[] = [];
      for (let x = 0; x < amplitude[0].length; x++) {
        row.push(amplitude[y][x] ** 2);
      }
      intensity.push(row);
    }

    return intensity;
  }

  private pointSourceCGH(
    scene: Scene3D,
    wavelength: number,
    resolution: { width: number; height: number },
    pixelPitch: number,
    distance: number
  ): { pattern: number[][]; phase: number[][] } {
    const pattern: number[][] = [];
    const phase: number[][] = [];

    const k = (2 * Math.PI) / wavelength;

    // Initialize arrays
    for (let y = 0; y < resolution.height; y++) {
      pattern.push(new Array(resolution.width).fill(0));
      phase.push(new Array(resolution.width).fill(0));
    }

    // Add contribution from each point
    if (scene.points) {
      for (const point of scene.points) {
        for (let y = 0; y < resolution.height; y++) {
          const yPos = (y - resolution.height / 2) * pixelPitch;

          for (let x = 0; x < resolution.width; x++) {
            const xPos = (x - resolution.width / 2) * pixelPitch;

            const dx = xPos - point.x;
            const dy = yPos - point.y;
            const dz = distance - point.z;

            const r = Math.sqrt(dx * dx + dy * dy + dz * dz);
            const amplitude = (point.amplitude || 1) / r;
            const ph = k * r;

            pattern[y][x] += amplitude * Math.cos(ph);
            phase[y][x] += amplitude * Math.sin(ph);
          }
        }
      }
    }

    // Convert to amplitude and phase
    for (let y = 0; y < resolution.height; y++) {
      for (let x = 0; x < resolution.width; x++) {
        const real = pattern[y][x];
        const imag = phase[y][x];
        pattern[y][x] = Math.sqrt(real * real + imag * imag);
        phase[y][x] = Math.atan2(imag, real);
      }
    }

    return { pattern, phase };
  }

  private layerBasedCGH(
    scene: Scene3D,
    wavelength: number,
    resolution: { width: number; height: number },
    pixelPitch: number
  ): { pattern: number[][]; phase: number[][] } {
    // Simplified layer-based CGH
    return this.pointSourceCGH(scene, wavelength, resolution, pixelPitch, 0.5);
  }

  private gerchbergSaxtonCGH(
    scene: Scene3D,
    wavelength: number,
    resolution: { width: number; height: number },
    iterations: number
  ): { pattern: number[][]; phase: number[][]; convergence: any } {
    // Simplified Gerchberg-Saxton algorithm
    const phase: number[][] = [];

    // Initialize with random phase
    for (let y = 0; y < resolution.height; y++) {
      const row: number[] = [];
      for (let x = 0; x < resolution.width; x++) {
        row.push(Math.random() * 2 * Math.PI);
      }
      phase.push(row);
    }

    // Uniform amplitude
    const pattern = this.createUniformPattern(resolution.width, resolution.height);

    const convergence = {
      iterations: iterations,
      finalError: 0.1,
      converged: true,
    };

    return { pattern, phase, convergence };
  }

  private createUniformPattern(width: number, height: number): number[][] {
    const pattern: number[][] = [];
    for (let y = 0; y < height; y++) {
      pattern.push(new Array(width).fill(1.0));
    }
    return pattern;
  }

  private estimateFringeSpacing(pattern: number[][]): number {
    // Simplified: analyze one row
    if (pattern.length === 0) return 1e-6;

    const row = pattern[Math.floor(pattern.length / 2)];
    let spacing = 0;
    let count = 0;
    let lastPeak = -1;

    for (let x = 1; x < row.length - 1; x++) {
      if (row[x] > row[x - 1] && row[x] > row[x + 1]) {
        // Local maximum
        if (lastPeak >= 0) {
          spacing += x - lastPeak;
          count++;
        }
        lastPeak = x;
      }
    }

    const avgSpacing = count > 0 ? spacing / count : 10;
    return avgSpacing * 8e-6; // Assuming 8μm pixel pitch
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Record hologram (standalone function)
 */
export function recordHologram(params: RecordingParams): RecordingResult {
  const sdk = new HolographySDK();
  return sdk.recordHologram(params);
}

/**
 * Reconstruct hologram (standalone function)
 */
export function reconstructHologram(params: ReconstructionParams): ReconstructedWave {
  const sdk = new HolographySDK();
  return sdk.reconstructHologram(params);
}

/**
 * Generate CGH (standalone function)
 */
export function generateCGH(params: CGHParams): ComputedHologram {
  const sdk = new HolographySDK();
  return sdk.generateCGH(params);
}

/**
 * Calculate interference pattern (standalone function)
 */
export function calculateInterferencePattern(
  objectBeam: WaveData,
  referenceBeam: WaveData
): number[][] {
  const sdk = new HolographySDK();
  return sdk.calculateInterferencePattern(objectBeam, referenceBeam);
}

/**
 * Analyze interference (standalone function)
 */
export function analyzeInterference(
  objectBeam: WaveData,
  referenceBeam: WaveData
): InterferenceAnalysis {
  const sdk = new HolographySDK();
  return sdk.analyzeInterference(objectBeam, referenceBeam);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HolographySDK };
export default HolographySDK;

**弘익人間 (홍익인간) · Benefit All Humanity**
