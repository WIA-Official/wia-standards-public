/**
 * WIA-COMM-004: 5G/6G Spectrum Management SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for 5G/6G spectrum management including:
 * - Spectrum availability queries
 * - Dynamic spectrum access (DSA)
 * - Carrier configuration and aggregation
 * - Link budget calculations
 * - Interference analysis
 * - Regulatory compliance checking
 */

import {
  NRBand,
  SpectrumAllocation,
  DSASpectrumQuery,
  DSASpectrumResponse,
  CBSDRegistration,
  SpectrumGrant,
  CarrierConfig,
  CarrierAggregationConfig,
  BeamformingConfig,
  InterferenceAnalysis,
  LinkBudget,
  CoveragePrediction,
  SpectrumEfficiency,
  RegulatoryCompliance,
  SpectrumUtilization,
  NetworkPerformance,
  GeoLocation,
  SpectrumErrorCode,
  SpectrumError,
  SPECTRUM_CONSTANTS,
  Modulation,
  MIMOConfig,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-004 Spectrum Management SDK
 */
export class SpectrumManagementSDK {
  private version = '1.0.0';
  private allocations: Map<string, SpectrumAllocation> = new Map();
  private grants: Map<string, SpectrumGrant> = new Map();

  constructor() {
    // Initialize SDK
    this.initializeNRBands();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Initialize NR band database
   */
  private initializeNRBands(): void {
    // Implementation would load NR band definitions
  }

  // ==========================================================================
  // Spectrum Query and Availability
  // ==========================================================================

  /**
   * Query available spectrum in a specific location
   */
  async queryAvailableSpectrum(params: {
    region: string;
    band?: string;
    frequencyRange?: { min: number; max: number };
    bandwidth?: number;
    location: GeoLocation;
  }): Promise<{
    availableBands: NRBand[];
    availableBandwidth: number;
    interferenceScore: number;
    recommendedPower: number;
  }> {
    const { region, band, frequencyRange, bandwidth, location } = params;

    // Simulate spectrum availability check
    const availableBands = this.getRegionalBands(region);

    // Filter by frequency range if specified
    let filtered = availableBands;
    if (frequencyRange) {
      filtered = availableBands.filter(
        (b) =>
          b.downlinkFrequency.min >= frequencyRange.min &&
          b.downlinkFrequency.max <= frequencyRange.max
      );
    }

    // Filter by specific band if requested
    if (band) {
      filtered = filtered.filter((b) => b.band === band);
    }

    // Calculate available bandwidth
    const availableBandwidth = filtered.reduce(
      (sum, b) => sum + b.totalBandwidth,
      0
    );

    // Simulate interference score (0-10, lower is better)
    const interferenceScore = Math.random() * 3 + 1; // 1-4

    // Calculate recommended power based on location and interference
    const recommendedPower = this.calculateRecommendedPower(
      location,
      interferenceScore
    );

    return {
      availableBands: filtered,
      availableBandwidth,
      interferenceScore,
      recommendedPower,
    };
  }

  /**
   * Get NR bands for a specific region
   */
  private getRegionalBands(region: string): NRBand[] {
    // Sample bands - real implementation would have complete database
    const bands: NRBand[] = [
      {
        band: 'n77',
        range: 'FR1',
        duplexMode: 'TDD',
        downlinkFrequency: { min: 3300, max: 4200 },
        totalBandwidth: 900,
        regions: ['Global', 'US', 'EU', 'CN'],
      },
      {
        band: 'n78',
        range: 'FR1',
        duplexMode: 'TDD',
        downlinkFrequency: { min: 3300, max: 3800 },
        totalBandwidth: 500,
        regions: ['Global', 'EU', 'IN'],
      },
      {
        band: 'n257',
        range: 'FR2',
        duplexMode: 'TDD',
        downlinkFrequency: { min: 26500, max: 29500 },
        totalBandwidth: 3000,
        regions: ['Global', 'US', 'KR'],
      },
    ];

    return bands.filter((b) => b.regions.includes(region));
  }

  /**
   * Calculate recommended transmit power
   */
  private calculateRecommendedPower(
    location: GeoLocation,
    interferenceScore: number
  ): number {
    // Base power
    let power = 46; // dBm (typical BS)

    // Reduce power for high interference areas
    if (interferenceScore > 5) {
      power -= 6;
    } else if (interferenceScore > 3) {
      power -= 3;
    }

    // Indoor adjustment (simulated)
    if (location.altitude && location.altitude < 50) {
      power -= 10;
    }

    return Math.max(23, Math.min(68, power));
  }

  // ==========================================================================
  // Carrier Configuration
  // ==========================================================================

  /**
   * Configure 5G NR carrier
   */
  configureCarrier(params: {
    band: string;
    centerFrequency: number;
    bandwidth: number;
    numerology?: number;
    tddPattern?: string;
    mimoLayers?: number;
  }): CarrierConfig {
    const {
      band,
      centerFrequency,
      bandwidth,
      numerology = 1,
      tddPattern = 'DDDSU',
      mimoLayers = 4,
    } = params;

    // Map numerology to SCS
    const scsMap: { [key: number]: 15 | 30 | 60 | 120 | 240 } = {
      0: 15,
      1: 30,
      2: 60,
      3: 120,
      4: 240,
    };
    const subcarrierSpacing = scsMap[numerology] || 30;

    // Calculate number of PRBs
    const numPRBs = this.calculatePRBs(bandwidth, subcarrierSpacing);

    const carrier: CarrierConfig = {
      id: `CARRIER-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      band,
      centerFrequency,
      bandwidth,
      subcarrierSpacing,
      duplexMode: 'TDD',
      tddPattern,
      tddPeriodicity: 5,
      mimo: {
        txAntennas: mimoLayers,
        rxAntennas: mimoLayers,
        maxLayers: mimoLayers,
        type: mimoLayers >= 8 ? 'Massive-MIMO' : 'MU-MIMO',
        beamforming: mimoLayers >= 4,
        numBeams: mimoLayers >= 8 ? 64 : undefined,
        beamWidth: mimoLayers >= 8 ? 10 : undefined,
      },
      maxTxPower: 46,
      mcs: 20,
      numPRBs,
    };

    return carrier;
  }

  /**
   * Calculate number of PRBs for given bandwidth and SCS
   */
  private calculatePRBs(bandwidth: number, scs: number): number {
    const prbBandwidth = (12 * scs) / 1000; // MHz
    return Math.floor(bandwidth / prbBandwidth);
  }

  /**
   * Perform carrier aggregation
   */
  async aggregateCarriers(
    carriers: Array<{ band: string; bandwidth: number }>
  ): Promise<CarrierAggregationConfig> {
    if (carriers.length === 0) {
      throw new SpectrumError(
        SpectrumErrorCode.INVALID_CONFIGURATION,
        'At least one carrier required for aggregation'
      );
    }

    // Configure all carriers
    const configuredCarriers = carriers.map((c, idx) => {
      const centerFreq = this.getDefaultCenterFrequency(c.band);
      return this.configureCarrier({
        band: c.band,
        centerFrequency: centerFreq,
        bandwidth: c.bandwidth,
        numerology: c.band.startsWith('n2') ? 2 : 1, // FR2 uses higher numerology
      });
    });

    const primaryCarrier = configuredCarriers[0];
    const secondaryCarriers = configuredCarriers.slice(1);

    // Determine CA type
    const allSameBand = carriers.every((c) => c.band === carriers[0].band);
    const type = allSameBand ? 'intra-band-contiguous' : 'inter-band';

    // Calculate total bandwidth
    const totalBandwidth = carriers.reduce((sum, c) => sum + c.bandwidth, 0);

    // Estimate throughput (simplified)
    const spectralEfficiency = 5.5; // bps/Hz for 256-QAM, 4x4 MIMO
    const overhead = 0.75; // 25% overhead
    const peakThroughput =
      totalBandwidth * 1e6 * spectralEfficiency * overhead * 0.001; // Mbps

    return {
      id: `CA-${Date.now()}`,
      primaryCarrier,
      secondaryCarriers,
      type,
      totalBandwidth,
      peakThroughput,
      spectralEfficiency,
    };
  }

  /**
   * Get default center frequency for a band
   */
  private getDefaultCenterFrequency(band: string): number {
    const defaults: { [key: string]: number } = {
      n77: 3600,
      n78: 3550,
      n79: 4700,
      n257: 28000,
      n258: 25875,
      n260: 38500,
    };
    return defaults[band] || 3600;
  }

  // ==========================================================================
  // Dynamic Spectrum Access (DSA)
  // ==========================================================================

  /**
   * Create DSA session for CBRS or other shared spectrum
   */
  createDSASession(params: {
    mode: 'CBRS' | 'LSA' | 'LAA';
    tier?: 'PAL' | 'GAA';
    location: GeoLocation;
    maxPower?: number;
  }): {
    requestSpectrum: (req: {
      bandwidth: number;
      duration: number;
      qos?: string;
    }) => Promise<SpectrumGrant>;
  } {
    const { mode, tier = 'GAA', location, maxPower = 30 } = params;

    return {
      requestSpectrum: async (req) => {
        const { bandwidth, duration, qos = 'best-effort' } = req;

        // Simulate SAS request for CBRS
        if (mode === 'CBRS') {
          return this.requestCBRSSpectrum(
            tier,
            location,
            bandwidth,
            duration,
            maxPower
          );
        }

        throw new SpectrumError(
          SpectrumErrorCode.INVALID_CONFIGURATION,
          `DSA mode ${mode} not yet implemented`
        );
      },
    };
  }

  /**
   * Request CBRS spectrum from SAS
   */
  private async requestCBRSSpectrum(
    tier: 'PAL' | 'GAA',
    location: GeoLocation,
    bandwidth: number,
    duration: number,
    maxPower: number
  ): Promise<SpectrumGrant> {
    // Check power limits
    const powerLimit = tier === 'GAA' ? 30 : 47;
    if (maxPower > powerLimit) {
      throw new SpectrumError(
        SpectrumErrorCode.POWER_EXCEEDS_LIMIT,
        `Requested power ${maxPower} dBm exceeds ${tier} limit of ${powerLimit} dBm`
      );
    }

    // Simulate SAS grant
    const frequencyBase = tier === 'PAL' ? 3550 : 3650;
    const frequencyOffset = Math.floor(Math.random() * 50);

    const grant: SpectrumGrant = {
      grantId: `GRANT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      frequencyRange: {
        min: frequencyBase + frequencyOffset,
        max: frequencyBase + frequencyOffset + bandwidth,
      },
      maxEirp: maxPower,
      grantExpireTime: new Date(Date.now() + duration * 1000),
      heartbeatInterval: 240, // 4 minutes
      channelType: tier,
    };

    this.grants.set(grant.grantId, grant);
    return grant;
  }

  // ==========================================================================
  // Interference Analysis
  // ==========================================================================

  /**
   * Analyze interference for a deployment
   */
  async analyzeInterference(params: {
    band: string;
    location: GeoLocation;
    transmitPower: number;
    antennaHeight: number;
  }): Promise<InterferenceAnalysis> {
    const { band, location, transmitPower, antennaHeight } = params;

    // Simulate interference calculation
    const frequency = this.getDefaultCenterFrequency(band);

    // Co-channel interference (same frequency)
    const coChannelInterference = this.calculateCoChannelInterference(
      frequency,
      location,
      transmitPower
    );

    // Adjacent channel interference
    const adjacentChannelInterference = coChannelInterference - 10; // ~10 dB better

    // Aggregate interference
    const aggregateInterference = -70 + Math.random() * 20; // dBm

    // Protection ratio
    const protectionRatio = transmitPower - aggregateInterference;

    return {
      id: `INTERFERENCE-${Date.now()}`,
      targetSystem: {
        band,
        frequency,
        location,
        transmitPower,
        antennaHeight,
      },
      coChannelInterference,
      adjacentChannelInterference,
      aggregateInterference,
      protectionRatio,
      meetsProtection: protectionRatio >= 10, // 10 dB C/I minimum
      mitigation: protectionRatio < 10 ? ['Reduce power', 'Use beamforming', 'Increase antenna height'] : undefined,
    };
  }

  /**
   * Calculate co-channel interference
   */
  private calculateCoChannelInterference(
    frequency: number,
    location: GeoLocation,
    transmitPower: number
  ): number {
    // Simplified interference calculation
    // Real implementation would use propagation models
    const distance = 1000; // meters (assumed)
    const pathLoss = this.calculatePathLoss(frequency, distance);
    return transmitPower - pathLoss;
  }

  /**
   * Calculate path loss (simplified UMa model)
   */
  private calculatePathLoss(frequency: number, distance: number): number {
    // UMa path loss model (3GPP)
    const fc = frequency / 1000; // GHz
    const d = distance; // meters
    const pl = 28.0 + 22 * Math.log10(d) + 20 * Math.log10(fc);
    return pl;
  }

  // ==========================================================================
  // Link Budget Calculation
  // ==========================================================================

  /**
   * Calculate link budget
   */
  calculateLinkBudget(params: {
    frequency: number;
    distance: number;
    transmitPower: number;
    txAntennaGain?: number;
    rxAntennaGain?: number;
    bandwidth?: number;
  }): LinkBudget {
    const {
      frequency,
      distance,
      transmitPower,
      txAntennaGain = 17,
      rxAntennaGain = 0,
      bandwidth = 100,
    } = params;

    // Transmitter
    const cableLoss = 3; // dB
    const eirp = transmitPower + txAntennaGain - cableLoss;

    // Path loss calculation
    const freeSpaceLoss = this.calculatePathLoss(frequency, distance);
    const shadowFading = 8; // dB
    const buildingPenetration = distance > 500 ? 15 : 0; // dB
    const foliageLoss = 0; // dB
    const totalPathLoss = freeSpaceLoss + shadowFading + buildingPenetration + foliageLoss;

    // Receiver
    const noiseFigure = 7; // dB
    const thermalNoise = SPECTRUM_CONSTANTS.THERMAL_NOISE_DENSITY;
    const bandwidthHz = bandwidth * 1e6;
    const noisePower = thermalNoise + 10 * Math.log10(bandwidthHz) + noiseFigure;
    const sensitivity = noisePower + 3; // 3 dB SINR for QPSK

    // Margins
    const requiredSINR = 3; // dB
    const interferenceMargin = 3; // dB
    const fadeMargin = 5; // dB
    const totalMargin = requiredSINR + interferenceMargin + fadeMargin;

    // Link closes?
    const receivedPower = eirp - totalPathLoss + rxAntennaGain;
    const excessMargin = receivedPower - sensitivity - totalMargin;
    const linkCloses = excessMargin >= 0;

    return {
      transmitter: {
        power: transmitPower,
        antennaGain: txAntennaGain,
        cableLoss,
        eirp,
      },
      pathLoss: {
        freeSpace: freeSpaceLoss,
        shadowFading,
        buildingPenetration,
        foliageLoss,
        total: totalPathLoss,
      },
      receiver: {
        antennaGain: rxAntennaGain,
        cableLoss: 0,
        noiseFigure,
        noisePower,
        sensitivity,
      },
      margin: {
        required: requiredSINR,
        interference: interferenceMargin,
        fade: fadeMargin,
        total: totalMargin,
      },
      linkCloses,
      excessMargin,
    };
  }

  // ==========================================================================
  // Spectrum Efficiency
  // ==========================================================================

  /**
   * Calculate spectrum efficiency
   */
  calculateSpectrumEfficiency(params: {
    modulation: Modulation;
    mimoLayers: number;
    bandwidth: number;
    codeRate?: number;
    overhead?: number;
  }): SpectrumEfficiency {
    const {
      modulation,
      mimoLayers,
      bandwidth,
      codeRate = 0.75,
      overhead = 0.75,
    } = params;

    // Bits per symbol
    const modulationMap: { [key in Modulation]: number } = {
      QPSK: 2,
      '16-QAM': 4,
      '64-QAM': 6,
      '256-QAM': 8,
      '1024-QAM': 10,
    };
    const bitsPerSymbol = modulationMap[modulation];

    // Spectral efficiency
    const theoreticalEfficiency = bitsPerSymbol * codeRate * mimoLayers;
    const practicalEfficiency = theoreticalEfficiency * overhead;

    // Throughput
    const peakThroughput = bandwidth * theoreticalEfficiency;
    const averageThroughput = bandwidth * practicalEfficiency;

    // Required SINR
    const sinrMap: { [key in Modulation]: number } = {
      QPSK: 5,
      '16-QAM': 12,
      '64-QAM': 18,
      '256-QAM': 22,
      '1024-QAM': 27,
    };
    const requiredSINR = sinrMap[modulation];

    return {
      modulation,
      codeRate,
      mimoLayers,
      bandwidth,
      overhead,
      theoreticalEfficiency,
      practicalEfficiency,
      peakThroughput,
      averageThroughput,
      requiredSINR,
    };
  }

  // ==========================================================================
  // Regulatory Compliance
  // ==========================================================================

  /**
   * Check regulatory compliance
   */
  checkCompliance(params: {
    region: string;
    band: string;
    frequency: number;
    transmitPower: number;
  }): RegulatoryCompliance {
    const { region, band, frequency, transmitPower } = params;

    // Simplified compliance check
    // Real implementation would have comprehensive regulatory database

    let maxAllowedPower = 68; // dBm (typical BS max)
    let licenseRequired = true;
    let coordinationRequired = false;
    const regulations: string[] = [];

    // Region-specific rules
    if (region === 'US') {
      if (band === 'n71') {
        maxAllowedPower = 68;
        regulations.push('FCC Part 27');
      } else if (frequency >= 3550 && frequency <= 3700) {
        // CBRS
        maxAllowedPower = 30; // GAA outdoor
        licenseRequired = false;
        regulations.push('FCC Part 96 (CBRS)');
      } else if (frequency >= 24000) {
        // mmWave
        maxAllowedPower = 75;
        regulations.push('FCC Part 30');
      }
    } else if (region === 'EU') {
      if (band === 'n78') {
        maxAllowedPower = 68;
        coordinationRequired = true;
        regulations.push('ECC Decision (18)06');
      }
    }

    const compliant = transmitPower <= maxAllowedPower;

    return {
      region: region as any,
      band,
      frequency,
      transmitPower,
      compliant,
      maxAllowedPower,
      regulations,
      restrictions: !compliant ? [`Power exceeds limit of ${maxAllowedPower} dBm`] : undefined,
      licenseRequired,
      coordinationRequired,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate path loss using 3GPP models
 */
export function calculatePathLoss(
  model: 'UMa' | 'UMi' | 'InH',
  frequency: number,
  distance: number,
  options?: {
    height?: number;
    indoor?: boolean;
  }
): number {
  const fc = frequency / 1000; // GHz
  const d = distance; // meters

  switch (model) {
    case 'UMa':
      return 28.0 + 22 * Math.log10(d) + 20 * Math.log10(fc);
    case 'UMi':
      return 32.4 + 21 * Math.log10(d) + 20 * Math.log10(fc);
    case 'InH':
      return 17.3 + 38.3 * Math.log10(d) + 20 * Math.log10(fc / 5);
    default:
      throw new Error(`Unknown propagation model: ${model}`);
  }
}

/**
 * Convert dBm to Watts
 */
export function dbmToWatts(dbm: number): number {
  return Math.pow(10, dbm / 10) / 1000;
}

/**
 * Convert Watts to dBm
 */
export function wattsToDbm(watts: number): number {
  return 10 * Math.log10(watts * 1000);
}

/**
 * Calculate EIRP
 */
export function calculateEIRP(
  txPower: number,
  antennaGain: number,
  cableLoss: number = 0
): number {
  return txPower + antennaGain - cableLoss;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SpectrumManagementSDK };
export default SpectrumManagementSDK;

/**
 * 弘益人間 (Benefit All Humanity)
 */
