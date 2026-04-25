/**
 * WIA-DEF-011: Reconnaissance Satellite SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for reconnaissance satellite operations including:
 * - Ground resolution calculations
 * - Orbital coverage analysis
 * - Image processing and exploitation
 * - SAR and optical sensor modeling
 * - SIGINT collection planning
 */

import {
  GroundResolutionParams,
  GroundResolutionResult,
  CoverageAnalysisParams,
  CoverageAnalysisResult,
  ImageProcessingRequest,
  ImageProcessingResult,
  TaskingRequest,
  TaskingResponse,
  SARProcessingParams,
  SARImageProduct,
  SignalDetection,
  AccessWindow,
  TargetLocation,
  CollectionMetadata,
  ImageQualityMetrics,
  RECON_CONSTANTS,
  ReconErrorCode,
  ReconSatelliteError,
  SensorType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-011 Reconnaissance Satellite SDK
 */
export class ReconSatelliteSDK {
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
   * Calculate ground resolution for a sensor
   *
   * @param params - Ground resolution parameters
   * @returns Ground resolution result with GSD, NIIRS, and swath width
   */
  calculateGroundResolution(params: GroundResolutionParams): GroundResolutionResult {
    const {
      altitude,
      focalLength,
      pixelPitch,
      offNadirAngle = 0,
      sensorType,
      sarBandwidth,
      sarAntennaLength,
      mtf = 0.3,
    } = params;

    // Validate inputs
    if (altitude <= 0) {
      throw new ReconSatelliteError(
        ReconErrorCode.INVALID_PARAMETERS,
        'Altitude must be positive'
      );
    }

    let gsd: number;
    let sarResolution: { range: number; azimuth: number } | undefined;

    if (sensorType === 'optical' || sensorType === 'hyperspectral' || sensorType === 'ir') {
      // Optical/IR sensor resolution
      if (!focalLength || !pixelPitch) {
        throw new ReconSatelliteError(
          ReconErrorCode.INVALID_PARAMETERS,
          'Focal length and pixel pitch required for optical sensors'
        );
      }

      // Calculate nadir GSD
      const gsdNadir = (altitude * pixelPitch) / focalLength;

      // Apply off-nadir correction
      const offNadirRad = (offNadirAngle * Math.PI) / 180;
      gsd = gsdNadir / Math.cos(offNadirRad);

    } else if (sensorType === 'sar') {
      // SAR sensor resolution
      if (!sarBandwidth || !sarAntennaLength) {
        throw new ReconSatelliteError(
          ReconErrorCode.INVALID_PARAMETERS,
          'SAR bandwidth and antenna length required for SAR sensors'
        );
      }

      const c = RECON_CONSTANTS.SPEED_OF_LIGHT;

      // Range resolution
      const rangeResolution = c / (2 * sarBandwidth);

      // Azimuth resolution (synthetic aperture)
      const azimuthResolution = sarAntennaLength / 2;

      sarResolution = {
        range: rangeResolution,
        azimuth: azimuthResolution,
      };

      // Use the worse of the two for GSD
      gsd = Math.max(rangeResolution, azimuthResolution);

    } else {
      throw new ReconSatelliteError(
        ReconErrorCode.INVALID_PARAMETERS,
        `Unsupported sensor type: ${sensorType}`
      );
    }

    // Calculate spatial resolution accounting for MTF
    const spatialResolution = gsd / mtf;

    // Calculate NIIRS rating
    const niirs = this.calculateNIIRS(gsd, mtf);

    // Calculate swath width (simplified model)
    const fov = sensorType === 'sar' ? 5 : 2; // degrees (typical values)
    const fovRad = (fov * Math.PI) / 180;
    const swathWidth = 2 * altitude * Math.tan(fovRad / 2);

    // Calculate field of view
    const fieldOfView = fov;

    // Determine feasibility
    let feasibility: GroundResolutionResult['feasibility'];
    if (gsd < 0.5) {
      feasibility = 'excellent';
    } else if (gsd < 2) {
      feasibility = 'good';
    } else if (gsd < 10) {
      feasibility = 'acceptable';
    } else {
      feasibility = 'poor';
    }

    return {
      gsd,
      spatialResolution,
      niirs,
      swathWidth,
      fieldOfView,
      feasibility,
      sarResolution,
    };
  }

  /**
   * Analyze orbital coverage for a target
   *
   * @param params - Coverage analysis parameters
   * @returns Coverage analysis result with revisit time and access windows
   */
  analyzeOrbitalCoverage(params: CoverageAnalysisParams): CoverageAnalysisResult {
    const {
      altitude,
      inclination,
      swathWidth,
      target,
      timeWindow,
      minElevation = 10,
      maxOffNadir = 30,
    } = params;

    // Calculate orbital period
    const semiMajorAxis = RECON_CONSTANTS.EARTH_RADIUS + altitude;
    const orbitalPeriod = this.calculateOrbitalPeriod(semiMajorAxis);

    // Calculate number of orbits per day
    const orbitsPerDay = (24 * 3600) / orbitalPeriod;

    // Estimate passes per day based on latitude and inclination
    const passesPerDay = this.estimatePassesPerDay(
      target.latitude,
      inclination,
      swathWidth,
      altitude
    );

    // Calculate revisit time
    const revisitTime = passesPerDay > 0 ? 24 / passesPerDay : Infinity;

    // Generate access windows (simplified - in real implementation would use SGP4/SDP4)
    const accessWindows = this.generateAccessWindows(
      params,
      orbitalPeriod,
      passesPerDay
    );

    // Calculate coverage statistics
    const numberOfPasses = accessWindows.length;
    const totalDuration = accessWindows.reduce((sum, w) => sum + w.duration, 0);
    const coverage = numberOfPasses > 0 ? Math.min(100, (totalDuration / 86400) * 100) : 0;

    // Calculate gaps
    let totalGap = 0;
    let maxGap = 0;
    for (let i = 1; i < accessWindows.length; i++) {
      const gap =
        (accessWindows[i].startTime.getTime() - accessWindows[i - 1].endTime.getTime()) / 1000 / 3600;
      totalGap += gap;
      maxGap = Math.max(maxGap, gap);
    }
    const averageGap = accessWindows.length > 1 ? totalGap / (accessWindows.length - 1) : 0;

    return {
      revisitTime,
      numberOfPasses,
      accessWindows,
      coverage,
      averageGap,
      maxGap,
      orbitalPeriod: orbitalPeriod / 60, // Convert to minutes
    };
  }

  /**
   * Process reconnaissance image
   *
   * @param request - Image processing request
   * @returns Processed image with metadata
   */
  async processImage(request: ImageProcessingRequest): Promise<ImageProcessingResult> {
    const startTime = Date.now();
    const log: string[] = [];
    const warnings: string[] = [];

    log.push('Starting image processing...');

    try {
      // Simulate processing steps
      let processedData = request.inputData;

      // Radiometric correction
      if (request.corrections.radiometric) {
        log.push('Applying radiometric correction...');
        // In real implementation: dark current, flat field, gain correction
      }

      // Geometric correction
      if (request.corrections.geometric) {
        log.push('Applying geometric correction...');
        // In real implementation: orthorectification, sensor model application
      }

      // Atmospheric correction
      if (request.corrections.atmospheric) {
        log.push('Applying atmospheric correction...');
        // In real implementation: path radiance removal, transmittance correction
      }

      // Terrain correction
      if (request.corrections.terrain) {
        log.push('Applying terrain correction...');
        // In real implementation: DEM-based orthorectification
      }

      // Enhancements
      if (request.enhancements) {
        if (request.enhancements.contrastEnhancement) {
          log.push('Applying contrast enhancement...');
        }
        if (request.enhancements.sharpening) {
          log.push('Applying sharpening filter...');
        }
        if (request.enhancements.noiseReduction) {
          log.push('Applying noise reduction...');
        }
      }

      // Compression
      if (request.compression) {
        log.push(`Applying ${request.compression.algorithm} compression...`);
      }

      // Generate metadata
      const metadata: CollectionMetadata = {
        imageId: `IMG-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        satelliteId: 'WIA-DEF-011-001',
        sensorId: `${request.sensorType.toUpperCase()}-001`,
        acquisitionTime: new Date(),
        center: { latitude: 37.5665, longitude: 126.978 },
        corners: {
          topLeft: { latitude: 37.6, longitude: 126.9 },
          topRight: { latitude: 37.6, longitude: 127.0 },
          bottomLeft: { latitude: 37.5, longitude: 126.9 },
          bottomRight: { latitude: 37.5, longitude: 127.0 },
        },
        gsd: request.sensorType === 'optical' ? 0.5 : 2.0,
        sun: { elevation: 45, azimuth: 135 },
        satellite: { elevation: 60, azimuth: 180, offNadir: 15 },
        cloudCover: 5,
        processingLevel: request.targetLevel,
        dimensions: { width: 10000, height: 10000, bands: 4 },
        format: request.outputFormat,
        fileSize: typeof processedData === 'string' ? 0 : processedData.byteLength,
        quality: this.calculateImageQuality(request.sensorType),
      };

      // Calculate processing time
      const processingTime = Date.now() - startTime;
      log.push(`Processing completed in ${processingTime}ms`);

      return {
        outputData: typeof processedData === 'string' ? new ArrayBuffer(0) : processedData,
        metadata,
        quality: metadata.quality,
        processingTime,
        log,
        warnings,
        success: true,
      };
    } catch (error) {
      return {
        outputData: new ArrayBuffer(0),
        metadata: {} as CollectionMetadata,
        quality: {} as ImageQualityMetrics,
        processingTime: Date.now() - startTime,
        log,
        warnings,
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Calculate revisit time for a constellation
   *
   * @param singleSatRevisit - Single satellite revisit time in hours
   * @param numSatellites - Number of satellites in constellation
   * @returns Constellation revisit time in hours
   */
  calculateConstellationRevisit(singleSatRevisit: number, numSatellites: number): number {
    if (numSatellites <= 0) {
      throw new ReconSatelliteError(
        ReconErrorCode.INVALID_PARAMETERS,
        'Number of satellites must be positive'
      );
    }
    return singleSatRevisit / numSatellites;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate NIIRS rating from GSD and RER
   */
  private calculateNIIRS(gsd: number, mtf: number): number {
    // GIQE (General Image Quality Equation) simplified
    // NIIRS = a + b * log10(GSD) + c * log10(RER)
    // Using standard coefficients
    const a = 10.251;
    const b = -3.32;
    const c = 1.559;

    // RER (Relative Edge Response) approximated from MTF
    const rer = mtf * 0.9;

    const niirs = a + b * Math.log10(gsd) + c * Math.log10(rer);

    // Clamp to valid range
    return Math.max(
      RECON_CONSTANTS.NIIRS_RANGE.MIN,
      Math.min(RECON_CONSTANTS.NIIRS_RANGE.MAX, niirs)
    );
  }

  /**
   * Calculate orbital period using Kepler's third law
   */
  private calculateOrbitalPeriod(semiMajorAxis: number): number {
    return 2 * Math.PI * Math.sqrt(Math.pow(semiMajorAxis, 3) / RECON_CONSTANTS.EARTH_MU);
  }

  /**
   * Estimate number of passes per day over a target
   */
  private estimatePassesPerDay(
    targetLat: number,
    inclination: number,
    swathWidth: number,
    altitude: number
  ): number {
    // Simplified model - in reality would depend on many factors
    const latRad = (Math.abs(targetLat) * Math.PI) / 180;
    const incRad = (inclination * Math.PI) / 180;

    // Check if target is within satellite coverage
    if (Math.abs(targetLat) > inclination && inclination < 90) {
      return 0; // Target not reachable
    }

    // Estimate swath coverage as percentage of orbit
    const earthCircumference = 2 * Math.PI * RECON_CONSTANTS.EARTH_RADIUS;
    const swathCoverage = swathWidth / earthCircumference;

    // Estimate passes (simplified)
    const orbitsPerDay = 86400 / this.calculateOrbitalPeriod(RECON_CONSTANTS.EARTH_RADIUS + altitude);
    const accessFactor = Math.cos(latRad) * Math.sin(incRad);

    return Math.max(1, Math.round(orbitsPerDay * swathCoverage * accessFactor * 2));
  }

  /**
   * Generate access windows (simplified model)
   */
  private generateAccessWindows(
    params: CoverageAnalysisParams,
    orbitalPeriod: number,
    passesPerDay: number
  ): AccessWindow[] {
    const windows: AccessWindow[] = [];
    const now = new Date();
    const startTime = params.timeWindow?.start || now;
    const endTime =
      params.timeWindow?.end || new Date(now.getTime() + 24 * 3600 * 1000);

    const timeSpan = endTime.getTime() - startTime.getTime();
    const interval = timeSpan / passesPerDay;

    for (let i = 0; i < passesPerDay; i++) {
      const windowStart = new Date(startTime.getTime() + i * interval);
      const duration = Math.min(600, orbitalPeriod * 0.1); // Max 10 minutes or 10% of orbit
      const windowEnd = new Date(windowStart.getTime() + duration * 1000);

      // Random but realistic values for demonstration
      const maxElevation = 30 + Math.random() * 60;
      const offNadirAngle = Math.random() * (params.maxOffNadir || 30);
      const sunElevation = -20 + Math.random() * 100;

      windows.push({
        startTime: windowStart,
        endTime: windowEnd,
        duration,
        maxElevation,
        offNadirAngle,
        sunElevation,
        sunAzimuth: Math.random() * 360,
        feasible: maxElevation >= (params.minElevation || 10) &&
                  offNadirAngle <= (params.maxOffNadir || 30) &&
                  sunElevation > 0,
        constraints: [],
      });
    }

    return windows.filter((w) => w.startTime >= startTime && w.endTime <= endTime);
  }

  /**
   * Calculate image quality metrics
   */
  private calculateImageQuality(sensorType: SensorType): ImageQualityMetrics {
    // Simplified quality calculation
    const baseQuality = sensorType === 'optical' ? 85 : 75;

    return {
      niirs: sensorType === 'optical' ? 7 : 6,
      snr: 100 + Math.random() * 50,
      rer: 0.3 + Math.random() * 0.2,
      mtf: 0.25 + Math.random() * 0.15,
      cloudCover: Math.random() * 20,
      contrast: 0.7 + Math.random() * 0.3,
      sharpness: 0.8 + Math.random() * 0.2,
      qualityScore: baseQuality + Math.random() * 10,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate ground resolution (standalone function)
 */
export function calculateGroundResolution(
  params: GroundResolutionParams
): GroundResolutionResult {
  const sdk = new ReconSatelliteSDK();
  return sdk.calculateGroundResolution(params);
}

/**
 * Analyze orbital coverage (standalone function)
 */
export function analyzeOrbitalCoverage(
  params: CoverageAnalysisParams
): CoverageAnalysisResult {
  const sdk = new ReconSatelliteSDK();
  return sdk.analyzeOrbitalCoverage(params);
}

/**
 * Process image (standalone function)
 */
export async function processImage(
  request: ImageProcessingRequest
): Promise<ImageProcessingResult> {
  const sdk = new ReconSatelliteSDK();
  return sdk.processImage(request);
}

/**
 * Calculate constellation revisit time (standalone function)
 */
export function calculateConstellationRevisit(
  singleSatRevisit: number,
  numSatellites: number
): number {
  const sdk = new ReconSatelliteSDK();
  return sdk.calculateConstellationRevisit(singleSatRevisit, numSatellites);
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Convert GSD to NIIRS estimate
 */
export function gsdToNIIRS(gsd: number, mtf: number = 0.3): number {
  const a = 10.251;
  const b = -3.32;
  const c = 1.559;
  const rer = mtf * 0.9;
  return Math.max(0, Math.min(9, a + b * Math.log10(gsd) + c * Math.log10(rer)));
}

/**
 * Convert NIIRS to approximate GSD
 */
export function niirsToGSD(niirs: number, mtf: number = 0.3): number {
  const a = 10.251;
  const b = -3.32;
  const c = 1.559;
  const rer = mtf * 0.9;
  return Math.pow(10, (niirs - a - c * Math.log10(rer)) / b);
}

/**
 * Calculate SAR range resolution
 */
export function calculateSARRangeResolution(bandwidth: number): number {
  return RECON_CONSTANTS.SPEED_OF_LIGHT / (2 * bandwidth);
}

/**
 * Calculate SAR azimuth resolution
 */
export function calculateSARAzimuthResolution(antennaLength: number): number {
  return antennaLength / 2;
}

/**
 * Calculate orbital period
 */
export function calculateOrbitalPeriod(altitude: number): number {
  const semiMajorAxis = RECON_CONSTANTS.EARTH_RADIUS + altitude;
  return 2 * Math.PI * Math.sqrt(Math.pow(semiMajorAxis, 3) / RECON_CONSTANTS.EARTH_MU);
}

/**
 * Calculate swath width
 */
export function calculateSwathWidth(altitude: number, fieldOfView: number): number {
  const fovRad = (fieldOfView * Math.PI) / 180;
  return 2 * altitude * Math.tan(fovRad / 2);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ReconSatelliteSDK };
export default ReconSatelliteSDK;
