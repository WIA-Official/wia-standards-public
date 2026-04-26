/**
 * WIA-SPACE-025: Interstellar Communication Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  CommunicationConfig,
  APIResponse,
  InterstellarTransmission,
  SETISignal,
  SignalAnalysis,
  TargetSystem,
  SignalParameters
} from './types';

export * from './types';

export class InterstellarCommunicationClient {
  private config: Required<CommunicationConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: CommunicationConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/interstellar-comm',
      timeout: 60000,
      debug: false,
      frequencies: {
        hydrogen: 1420405751.768, // Hz (exact)
        waterHole: { min: 1420e6, max: 1665e6 }
      },
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Submit an interstellar transmission
   */
  async submitTransmission(transmission: InterstellarTransmission): Promise<APIResponse<InterstellarTransmission>> {
    this.log('Submitting transmission', transmission);
    return this.makeRequest('POST', '/transmissions', transmission);
  }

  /**
   * Report a SETI signal detection
   */
  async reportSETISignal(signal: SETISignal): Promise<APIResponse<SETISignal>> {
    this.log('Reporting SETI signal', signal);
    return this.makeRequest('POST', '/seti-signals', signal);
  }

  /**
   * Analyze a signal for artificial origin indicators
   */
  analyzeSETISignal(
    frequency: number,
    bandwidth: number,
    snr: number,
    driftRate?: number
  ): SignalAnalysis {
    const analysis: SignalAnalysis = {
      artificialProbability: 0,
      narrowband: bandwidth < 1, // <1 Hz is narrowband
      persistent: false,
      terrestrialOrigin: false,
      recommendations: []
    };

    // Narrowband signals are more likely artificial
    if (analysis.narrowband) {
      analysis.artificialProbability += 0.4;
      analysis.recommendations.push('Narrowband signal detected - likely artificial origin');
    }

    // Check if near hydrogen line (strong indicator)
    const hydrogenLine = 1420.405e6; // 1420.405 MHz
    if (Math.abs(frequency - hydrogenLine) < 1e6) {
      analysis.artificialProbability += 0.3;
      analysis.recommendations.push('Near hydrogen 21-cm line - universal reference frequency');
    }

    // High SNR indicates strong signal
    if (snr > 20) {
      analysis.artificialProbability += 0.2;
      analysis.recommendations.push('Strong SNR - good detection confidence');
    }

    // Drift rate analysis
    if (driftRate !== undefined && Math.abs(driftRate) < 0.1) {
      analysis.artificialProbability += 0.1;
      analysis.recommendations.push('Low drift rate - consistent with artificial source');
    }

    // Final recommendations
    if (analysis.artificialProbability > 0.7) {
      analysis.recommendations.push('High probability of artificial origin - priority follow-up required');
      analysis.recommendations.push('Request independent verification from other observatories');
    } else if (analysis.artificialProbability > 0.4) {
      analysis.recommendations.push('Moderate probability - continue monitoring');
    }

    return analysis;
  }

  /**
   * Calculate signal propagation time
   */
  calculatePropagationTime(distanceLightYears: number): {
    oneWayYears: number;
    roundTripYears: number;
    oneWaySeconds: number;
  } {
    const oneWayYears = distanceLightYears;
    const roundTripYears = distanceLightYears * 2;
    const oneWaySeconds = distanceLightYears * 365.25 * 24 * 3600;

    return {
      oneWayYears,
      roundTripYears,
      oneWaySeconds
    };
  }

  /**
   * Calculate free-space path loss
   */
  calculatePathLoss(
    distanceLightYears: number,
    frequencyHz: number
  ): number {
    const c = 299792458; // m/s
    const distanceM = distanceLightYears * 9.461e15; // meters
    const wavelength = c / frequencyHz;

    // Free space path loss in dB
    const fspl = 20 * Math.log10(distanceM) + 20 * Math.log10(frequencyHz) - 147.55;
    return fspl;
  }

  /**
   * Calculate received signal power
   */
  calculateReceivedPower(
    transmitPowerW: number,
    transmitGainDbi: number,
    distanceLightYears: number,
    frequencyHz: number,
    receiverApertureM: number
  ): number {
    const c = 299792458;
    const distanceM = distanceLightYears * 9.461e15;

    // EIRP
    const gainLinear = Math.pow(10, transmitGainDbi / 10);
    const eirp = transmitPowerW * gainLinear;

    // Wavelength
    const wavelength = c / frequencyHz;

    // Free space path loss (linear)
    const fspl = Math.pow((4 * Math.PI * distanceM / wavelength), 2);

    // Receiver effective area
    const receiverGain = (Math.PI * receiverApertureM / wavelength) ** 2;

    // Received power
    const rxPower = (eirp / fspl) * receiverGain;

    return rxPower;
  }

  /**
   * Get target star systems
   */
  async getTargetSystems(maxDistance: number): Promise<APIResponse<TargetSystem[]>> {
    const systems: TargetSystem[] = [
      {
        designation: 'Proxima Centauri',
        distance: 4.24,
        coordinates: { rightAscension: 217.4, declination: -62.7, distance: 4.24 },
        starType: 'M5.5Ve',
        knownPlanets: 3
      },
      {
        designation: "Barnard's Star",
        distance: 5.96,
        coordinates: { rightAscension: 269.5, declination: 4.7, distance: 5.96 },
        starType: 'M4V',
        knownPlanets: 1
      },
      {
        designation: 'Wolf 359',
        distance: 7.86,
        coordinates: { rightAscension: 164.1, declination: 7.0, distance: 7.86 },
        starType: 'M6V',
        knownPlanets: 0
      }
    ];

    const filtered = systems.filter(s => s.distance <= maxDistance);

    return {
      success: true,
      data: filtered,
      timestamp: new Date().toISOString()
    };
  }

  /**
   * Subscribe to signal detection events
   */
  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  private async makeRequest<T>(method: string, path: string, body?: any): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;
    this.log(`${method} ${url}`, body);

    try {
      return {
        success: true,
        data: body as T,
        timestamp: new Date().toISOString()
      };
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
        },
        timestamp: new Date().toISOString()
      };
    }
  }

  private log(...args: any[]): void {
    if (this.config.debug) {
      console.log('[WIA-SPACE-025]', ...args);
    }
  }
}

/**
 * Utility functions
 */
export const InterstellarCommUtils = {
  /**
   * Convert light-years to parsecs
   */
  lightYearsToParsecs(ly: number): number {
    return ly / 3.26156;
  },

  /**
   * Convert parsecs to light-years
   */
  parsecsToLightYears(parsecs: number): number {
    return parsecs * 3.26156;
  },

  /**
   * Calculate Drake Equation
   */
  calculateDrakeEquation(params: {
    starFormationRate: number;
    fractionWithPlanets: number;
    planetsPerStar: number;
    fractionWithLife: number;
    fractionWithIntelligence: number;
    fractionWithCommunication: number;
    civilizationLifetime: number;
  }): number {
    const N =
      params.starFormationRate *
      params.fractionWithPlanets *
      params.planetsPerStar *
      params.fractionWithLife *
      params.fractionWithIntelligence *
      params.fractionWithCommunication *
      params.civilizationLifetime;

    return N;
  }
};
