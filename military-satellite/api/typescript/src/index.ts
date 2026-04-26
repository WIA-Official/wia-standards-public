/**
 * WIA-DEF-010: Military Satellite SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for military satellite operations including:
 * - Orbital mechanics calculations
 * - Communication link analysis
 * - Reconnaissance mission planning
 * - Navigation solutions
 * - Ground track prediction
 */

import {
  OrbitalRequest,
  OrbitalResponse,
  LinkValidation,
  LinkResult,
  LinkBudget,
  TrackingRequest,
  SatellitePosition,
  ImagingRequest,
  PositionSolution,
  GroundTrackPoint,
  SatellitePass,
  GeographicCoordinate,
  EncryptionKey,
  EncryptedMessage,
  SATELLITE_CONSTANTS,
  SatelliteErrorCode,
  SatelliteError,
  Vector3,
  MilitarySatellite,
  OrbitalElements,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-010 Military Satellite SDK
 */
export class MilitarySatelliteSDK {
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
   * Calculate orbital parameters for a given altitude
   *
   * @param params - Orbital request parameters
   * @returns Orbital parameters including velocity, period, etc.
   */
  calculateOrbitalParameters(params: OrbitalRequest): OrbitalResponse {
    const { altitude, inclination = 0, eccentricity = 0, orbitalType } = params;

    // Validate inputs
    if (altitude < 160000) {
      throw new SatelliteError(
        SatelliteErrorCode.INVALID_PARAMETERS,
        'Altitude must be at least 160 km (minimum LEO)'
      );
    }

    // Calculate orbital radius
    const radius = SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN + altitude;

    // For circular orbit (e = 0)
    const mu = SATELLITE_CONSTANTS.EARTH_MU;

    // Semi-major axis (for circular, a = r)
    const semiMajorAxis = radius / (1 - eccentricity);

    // Orbital velocity: v = √(μ/r)
    const velocity = Math.sqrt(mu / radius);

    // Orbital period: T = 2π√(a³/μ)
    const period = 2 * Math.PI * Math.sqrt(Math.pow(semiMajorAxis, 3) / mu);

    // Angular velocity: ω = 2π/T
    const angularVelocity = (2 * Math.PI) / period;

    // Apogee and perigee
    const apogee = semiMajorAxis * (1 + eccentricity) - SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN;
    const perigee = semiMajorAxis * (1 - eccentricity) - SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN;

    return {
      velocity,
      period,
      semiMajorAxis,
      apogee,
      perigee,
      radius,
      angularVelocity,
    };
  }

  /**
   * Calculate orbital velocity for a specific altitude
   *
   * @param altitude - Altitude in meters
   * @returns Orbital velocity in m/s
   */
  calculateOrbitalVelocity(altitude: number): number {
    const radius = SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN + altitude;
    return Math.sqrt(SATELLITE_CONSTANTS.EARTH_MU / radius);
  }

  /**
   * Validate a satellite communication link
   *
   * @param validation - Link validation parameters
   * @returns Link validation result with signal strength and margin
   */
  validateSatelliteLink(validation: LinkValidation): LinkResult {
    const {
      satelliteId,
      groundStation,
      frequency,
      bandwidth = 10e6,
      encryptionLevel,
      weather,
      minSignalStrength = -120,
    } = validation;

    const errors: string[] = [];
    const warnings: string[] = [];

    // Calculate wavelength
    const wavelength = SATELLITE_CONSTANTS.SPEED_OF_LIGHT / frequency;

    // Estimate satellite altitude based on frequency band
    let altitude: number;
    if (frequency < 3e9) {
      altitude = 35786000; // GEO for UHF/L-band
    } else if (frequency < 12e9) {
      altitude = 800000; // LEO for X-band
    } else {
      altitude = 35786000; // GEO for higher frequencies
    }

    const distance = Math.sqrt(
      Math.pow(altitude + SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN, 2) +
      Math.pow(SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN, 2)
    );

    // Calculate path loss: L = 20*log10(4πd/λ)
    const pathLoss = 20 * Math.log10((4 * Math.PI * distance) / wavelength);

    // Atmospheric and weather losses
    let atmosphericLoss = 0.5; // dB (clear sky)
    let rainAttenuation = 0;

    if (weather) {
      atmosphericLoss += weather.cloudCover * 2; // 0-2 dB
      rainAttenuation = this.calculateRainAttenuation(
        frequency,
        weather.precipitationRate,
        90 - 45 // Assume 45° elevation
      );
    }

    // Transmitter EIRP (typical values)
    const eirp = frequency > 10e9 ? 55 : 50; // dBW

    // Receiver G/T (typical ground station)
    const gOverT = frequency > 10e9 ? 30 : 25; // dB/K

    // System noise temperature
    const systemNoise = 290; // K (typical)

    // Boltzmann constant in dBW/K/Hz
    const k_dB = -228.6;

    // C/N0 = EIRP + G/T - pathLoss - atmosphericLoss - rainAttenuation - k
    const cn0 = eirp + gOverT - pathLoss - atmosphericLoss - rainAttenuation - k_dB;

    // Required C/N0 for given bandwidth (assume QPSK, rate 1/2)
    const requiredEbN0 = 4.5; // dB (for BER 10^-6)
    const dataRate = bandwidth * 0.5; // bps (QPSK, rate 1/2)
    const requiredCn0 = requiredEbN0 + 10 * Math.log10(dataRate);

    // Link margin
    const linkMargin = cn0 - requiredCn0;

    // C/N ratio
    const cnr = cn0 - 10 * Math.log10(bandwidth);

    // Signal strength at receiver (approximate)
    const signalStrength = eirp - pathLoss - atmosphericLoss - rainAttenuation - 30; // dBm

    // Latency (two-way)
    const latency = (2 * distance) / SATELLITE_CONSTANTS.SPEED_OF_LIGHT * 1000; // ms

    // Build link budget
    const linkBudget: LinkBudget = {
      eirp,
      pathLoss,
      atmosphericLoss,
      rainAttenuation,
      gOverT,
      systemNoise,
      cn0,
      requiredCn0,
      margin: linkMargin,
    };

    // Validation checks
    if (linkMargin < 0) {
      errors.push(`Link margin is negative (${linkMargin.toFixed(2)} dB). Link will fail.`);
    } else if (linkMargin < 3) {
      warnings.push(`Link margin is low (${linkMargin.toFixed(2)} dB). Consider increasing power or antenna gain.`);
    }

    if (signalStrength < minSignalStrength) {
      errors.push(`Signal strength (${signalStrength.toFixed(2)} dBm) below minimum (${minSignalStrength} dBm)`);
    }

    if (rainAttenuation > 10) {
      warnings.push(`High rain attenuation (${rainAttenuation.toFixed(2)} dB). Link may be degraded.`);
    }

    // Encryption overhead warning
    if (encryptionLevel !== 'none') {
      warnings.push(`Encryption enabled (${encryptionLevel}). Additional latency: ~1-5 ms`);
    }

    return {
      isValid: errors.length === 0,
      signalStrength,
      linkMargin,
      cnr,
      dataRate,
      latency,
      errors,
      warnings,
      linkBudget,
    };
  }

  /**
   * Track satellite position over time
   *
   * @param request - Tracking request parameters
   * @returns Array of satellite positions
   */
  trackSatellitePosition(request: TrackingRequest): SatellitePosition[] {
    const { satelliteId, startTime, duration, observerLocation, timeStep = 60 } = request;

    const positions: SatellitePosition[] = [];

    // Simplified orbit propagation (circular LEO at 550 km for demonstration)
    const altitude = 550000; // meters
    const radius = SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN + altitude;
    const angularVelocity = Math.sqrt(SATELLITE_CONSTANTS.EARTH_MU / Math.pow(radius, 3));
    const period = 2 * Math.PI / angularVelocity;

    let currentTime = new Date(startTime);
    const endTime = new Date(startTime.getTime() + duration * 1000);

    while (currentTime <= endTime) {
      const t = (currentTime.getTime() - startTime.getTime()) / 1000; // seconds since start
      const angle = angularVelocity * t;

      // ECI position (simplified circular orbit in equatorial plane)
      const position: Vector3 = {
        x: radius * Math.cos(angle),
        y: radius * Math.sin(angle),
        z: 0,
      };

      // ECI velocity
      const velocity: Vector3 = {
        x: -radius * angularVelocity * Math.sin(angle),
        y: radius * angularVelocity * Math.cos(angle),
        z: 0,
      };

      // Convert to geographic (simplified)
      const latitude = 0; // Equatorial orbit
      const longitude = (angle * 180 / Math.PI) % 360 - 180;

      const geographic: GeographicCoordinate = {
        latitude,
        longitude,
        altitude,
      };

      const satPos: SatellitePosition = {
        time: new Date(currentTime),
        position,
        velocity,
        geographic,
      };

      // Calculate observer-relative parameters if observer specified
      if (observerLocation) {
        const observer = this.calculateObserverAngles(position, observerLocation);
        satPos.observer = observer;
      }

      positions.push(satPos);

      currentTime = new Date(currentTime.getTime() + timeStep * 1000);
    }

    return positions;
  }

  /**
   * Predict next satellite pass over ground station
   *
   * @param satelliteId - Satellite identifier
   * @param groundStation - Ground station location
   * @param startTime - Start search time
   * @param minElevation - Minimum elevation angle in degrees
   * @returns Next satellite pass
   */
  predictNextPass(
    satelliteId: string,
    groundStation: GeographicCoordinate,
    startTime: Date,
    minElevation: number = 5
  ): SatellitePass | null {
    // Simplified pass prediction (would use SGP4/SDP4 in production)
    const altitude = 550000;
    const period = 2 * Math.PI * Math.sqrt(
      Math.pow(SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN + altitude, 3) /
      SATELLITE_CONSTANTS.EARTH_MU
    );

    // Estimate next pass (simplified)
    const riseTime = new Date(startTime.getTime() + 30 * 60 * 1000); // +30 min
    const maxElevationTime = new Date(riseTime.getTime() + 5 * 60 * 1000); // +5 min
    const setTime = new Date(maxElevationTime.getTime() + 5 * 60 * 1000); // +5 min

    return {
      satelliteId,
      riseTime,
      riseAzimuth: 45,
      maxElevationTime,
      maxElevation: 60,
      setTime,
      setAzimuth: 315,
      duration: (setTime.getTime() - riseTime.getTime()) / 1000,
      averageRange: altitude + 500000,
    };
  }

  /**
   * Calculate ground track
   *
   * @param satelliteId - Satellite identifier
   * @param startTime - Start time
   * @param duration - Duration in seconds
   * @param timeStep - Time step in seconds
   * @returns Ground track points
   */
  calculateGroundTrack(
    satelliteId: string,
    startTime: Date,
    duration: number,
    timeStep: number = 60
  ): GroundTrackPoint[] {
    const positions = this.trackSatellitePosition({
      satelliteId,
      startTime,
      duration,
      timeStep,
    });

    return positions.map((pos) => ({
      time: pos.time,
      position: pos.geographic,
    }));
  }

  /**
   * Generate encryption key
   *
   * @param algorithm - Encryption algorithm
   * @param purpose - Key purpose
   * @param validityDays - Validity period in days
   * @returns Encryption key
   */
  generateEncryptionKey(
    algorithm: 'aes-128' | 'aes-256' | 'rsa-2048' | 'rsa-4096',
    purpose: 'uplink' | 'downlink' | 'storage' | 'authentication',
    validityDays: number = 30
  ): EncryptionKey {
    const keyId = `KEY-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    const created = new Date();
    const expires = new Date(created.getTime() + validityDays * 24 * 60 * 60 * 1000);

    // Generate random key material (base64 encoded)
    const keyLength = algorithm.includes('aes') ?
      (algorithm === 'aes-256' ? 32 : 16) :
      (algorithm === 'rsa-4096' ? 512 : 256);

    const keyMaterial = this.generateRandomBytes(keyLength);

    return {
      id: keyId,
      type: algorithm.includes('rsa') ? 'asymmetric' : 'symmetric',
      algorithm,
      keyMaterial,
      created,
      expires,
      purpose,
      status: 'active',
    };
  }

  /**
   * Encrypt message
   *
   * @param message - Plain text message
   * @param key - Encryption key
   * @returns Encrypted message
   */
  encryptMessage(message: string, key: EncryptionKey): EncryptedMessage {
    // Simplified encryption (would use actual crypto library in production)
    const iv = this.generateRandomBytes(16);
    const payload = Buffer.from(message).toString('base64');
    const mac = this.calculateMAC(payload, key.keyMaterial);

    return {
      id: `MSG-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      algorithm: key.algorithm,
      keyId: key.id,
      iv,
      payload,
      mac,
      timestamp: new Date(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate rain attenuation
   */
  private calculateRainAttenuation(
    frequency: number,
    rainRate: number,
    elevationAngle: number
  ): number {
    // ITU-R P.838 rain attenuation model (simplified)
    const f_GHz = frequency / 1e9;

    // Regression coefficients (horizontal polarization)
    const kH = 0.0000387 * Math.pow(f_GHz, 0.2571);
    const alphaH = 2.33 * Math.pow(f_GHz, -0.1547);

    // Specific attenuation
    const gamma = kH * Math.pow(rainRate, alphaH); // dB/km

    // Effective path length (simplified)
    const pathLength = 5 / Math.sin(elevationAngle * Math.PI / 180); // km

    return gamma * pathLength;
  }

  /**
   * Calculate observer-relative angles
   */
  private calculateObserverAngles(
    satPosition: Vector3,
    observer: GeographicCoordinate
  ): { azimuth: number; elevation: number; range: number; rangeRate: number } {
    // Simplified calculation (would use proper coordinate transformations)
    const range = Math.sqrt(
      satPosition.x * satPosition.x +
      satPosition.y * satPosition.y +
      satPosition.z * satPosition.z
    ) - SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN;

    return {
      azimuth: 180,
      elevation: 45,
      range,
      rangeRate: 0,
    };
  }

  /**
   * Generate random bytes (base64 encoded)
   */
  private generateRandomBytes(length: number): string {
    // Simplified random generation (would use crypto.randomBytes in production)
    const bytes = Array.from({ length }, () =>
      Math.floor(Math.random() * 256)
    );
    return Buffer.from(bytes).toString('base64');
  }

  /**
   * Calculate message authentication code
   */
  private calculateMAC(payload: string, key: string): string {
    // Simplified MAC (would use HMAC-SHA256 in production)
    return Buffer.from(payload + key).toString('base64').substr(0, 32);
  }

  /**
   * Convert ECI to geographic coordinates
   */
  private eciToGeographic(position: Vector3, time: Date): GeographicCoordinate {
    // Simplified conversion (would use proper coordinate transformations)
    const r = Math.sqrt(position.x ** 2 + position.y ** 2 + position.z ** 2);
    const latitude = Math.asin(position.z / r) * 180 / Math.PI;
    const longitude = Math.atan2(position.y, position.x) * 180 / Math.PI;
    const altitude = r - SATELLITE_CONSTANTS.EARTH_RADIUS_MEAN;

    return { latitude, longitude, altitude };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate orbital velocity (standalone function)
 */
export function calculateOrbitalVelocity(altitude: number): number {
  const sdk = new MilitarySatelliteSDK();
  return sdk.calculateOrbitalVelocity(altitude);
}

/**
 * Calculate orbital parameters (standalone function)
 */
export function calculateOrbitalParameters(params: OrbitalRequest): OrbitalResponse {
  const sdk = new MilitarySatelliteSDK();
  return sdk.calculateOrbitalParameters(params);
}

/**
 * Validate satellite link (standalone function)
 */
export function validateSatelliteLink(validation: LinkValidation): LinkResult {
  const sdk = new MilitarySatelliteSDK();
  return sdk.validateSatelliteLink(validation);
}

/**
 * Track satellite position (standalone function)
 */
export function trackSatellitePosition(request: TrackingRequest): SatellitePosition[] {
  const sdk = new MilitarySatelliteSDK();
  return sdk.trackSatellitePosition(request);
}

/**
 * Generate encryption key (standalone function)
 */
export function generateEncryptionKey(
  algorithm: 'aes-128' | 'aes-256' | 'rsa-2048' | 'rsa-4096',
  purpose: 'uplink' | 'downlink' | 'storage' | 'authentication',
  validityDays: number = 30
): EncryptionKey {
  const sdk = new MilitarySatelliteSDK();
  return sdk.generateEncryptionKey(algorithm, purpose, validityDays);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MilitarySatelliteSDK };
export default MilitarySatelliteSDK;
