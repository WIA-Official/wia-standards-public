/**
 * WIA-QUA-020: Scientific Instrument - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Scientific Instrumentation Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  InstrumentType,
  InstrumentConfig,
  InstrumentInit,
  AcceleratorConfig,
  MassSpectrometerConfig,
  ElectronMicroscopeConfig,
  XRayCrystallographyConfig,
  NMRSpectrometerConfig,
  TelescopeConfig,
  SpectrophotometerConfig,
  ChromatographyConfig,
  CalorimeterConfig,
  MeasurementRequest,
  MeasurementResult,
  CalibrationResult,
  InstrumentError,
  InstrumentErrorCode,
  Result,
  AsyncResult,
  CollisionEvent,
  MassSpectrum,
  MicroscopeImage,
  CrystalStructure,
  NMRSpectrum,
  AstronomicalObservation,
  AbsorptionSpectrum,
  Chromatogram,
  CalorimetryData,
  PHYSICAL_CONSTANTS,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-020 Scientific Instrument SDK
 */
export class ScientificInstrumentSDK {
  private version = '1.0.0';
  private instruments: Map<string, BaseInstrument> = new Map();

  constructor() {
    console.log(`🔬 WIA-QUA-020 Scientific Instrument SDK v${this.version}`);
    console.log('弘益人間 (Benefit All Humanity)');
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a particle accelerator
   */
  createAccelerator(config: AcceleratorConfig): ParticleAccelerator {
    const accelerator = new ParticleAccelerator(config);
    this.instruments.set(config.serialNumber, accelerator);
    return accelerator;
  }

  /**
   * Create a mass spectrometer
   */
  createMassSpectrometer(config: MassSpectrometerConfig): MassSpectrometer {
    const massSpec = new MassSpectrometer(config);
    this.instruments.set(config.serialNumber, massSpec);
    return massSpec;
  }

  /**
   * Create an electron microscope
   */
  createElectronMicroscope(config: ElectronMicroscopeConfig): ElectronMicroscope {
    const microscope = new ElectronMicroscope(config);
    this.instruments.set(config.serialNumber, microscope);
    return microscope;
  }

  /**
   * Create an X-ray crystallography system
   */
  createXRayCrystallography(config: XRayCrystallographyConfig): XRayCrystallography {
    const xray = new XRayCrystallography(config);
    this.instruments.set(config.serialNumber, xray);
    return xray;
  }

  /**
   * Create an NMR spectrometer
   */
  createNMRSpectrometer(config: NMRSpectrometerConfig): NMRSpectrometer {
    const nmr = new NMRSpectrometer(config);
    this.instruments.set(config.serialNumber, nmr);
    return nmr;
  }

  /**
   * Create a telescope
   */
  createTelescope(config: TelescopeConfig): Telescope {
    const telescope = new Telescope(config);
    this.instruments.set(config.serialNumber, telescope);
    return telescope;
  }

  /**
   * Create a spectrophotometer
   */
  createSpectrophotometer(config: SpectrophotometerConfig): Spectrophotometer {
    const spectro = new Spectrophotometer(config);
    this.instruments.set(config.serialNumber, spectro);
    return spectro;
  }

  /**
   * Create a chromatography system
   */
  createChromatography(config: ChromatographyConfig): Chromatography {
    const chrom = new Chromatography(config);
    this.instruments.set(config.serialNumber, chrom);
    return chrom;
  }

  /**
   * Create a calorimeter
   */
  createCalorimeter(config: CalorimeterConfig): Calorimeter {
    const cal = new Calorimeter(config);
    this.instruments.set(config.serialNumber, cal);
    return cal;
  }

  /**
   * Get instrument by serial number
   */
  getInstrument(serialNumber: string): BaseInstrument | undefined {
    return this.instruments.get(serialNumber);
  }

  /**
   * List all instruments
   */
  listInstruments(): InstrumentConfig[] {
    return Array.from(this.instruments.values()).map((inst) => inst.getConfig());
  }
}

// ============================================================================
// Base Instrument Class
// ============================================================================

abstract class BaseInstrument {
  protected config: InstrumentConfig;
  protected initialized = false;
  protected calibrationStatus: 'valid' | 'expired' | 'unknown' = 'unknown';

  constructor(config: InstrumentConfig) {
    this.config = config;
  }

  /**
   * Initialize instrument
   */
  async initialize(params?: InstrumentInit): Promise<Result<void>> {
    try {
      if (params?.selfTest) {
        await this.selfTest();
      }

      this.checkCalibration();
      this.initialized = true;

      return { success: true, data: undefined };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error(String(error)),
      };
    }
  }

  /**
   * Get instrument configuration
   */
  getConfig(): InstrumentConfig {
    return this.config;
  }

  /**
   * Check calibration status
   */
  protected checkCalibration(): void {
    const calibDate = new Date(this.config.calibrationDate);
    const now = new Date();
    const daysSinceCalibration = (now.getTime() - calibDate.getTime()) / (1000 * 60 * 60 * 24);

    if (daysSinceCalibration > 365) {
      this.calibrationStatus = 'expired';
    } else {
      this.calibrationStatus = 'valid';
    }
  }

  /**
   * Perform self-test
   */
  protected async selfTest(): Promise<void> {
    // Simulate self-test
    await new Promise((resolve) => setTimeout(resolve, 100));
  }

  /**
   * Calibrate instrument
   */
  abstract calibrate(): Promise<CalibrationResult>;

  /**
   * Get instrument status
   */
  getStatus(): {
    initialized: boolean;
    calibration: string;
    config: InstrumentConfig;
  } {
    return {
      initialized: this.initialized,
      calibration: this.calibrationStatus,
      config: this.config,
    };
  }
}

// ============================================================================
// Particle Accelerator
// ============================================================================

export class ParticleAccelerator extends BaseInstrument {
  private config: AcceleratorConfig;

  constructor(config: AcceleratorConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Collide particles
   */
  async collide(params: {
    particle1: string;
    particle2: string;
    luminosity: number;
    detectors: string[];
  }): Promise<CollisionEvent> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Accelerator not initialized'
      );
    }

    const event: CollisionEvent = {
      eventId: `EVT-${Date.now()}`,
      timestamp: new Date().toISOString(),
      energy: this.config.energy,
      luminosity: params.luminosity,
      particles: {
        detected: Math.floor(Math.random() * 100),
        identified: [],
      },
    };

    return event;
  }

  /**
   * Get beam parameters
   */
  getBeamParameters() {
    return {
      energy: this.config.energy,
      current: this.config.beamCurrent,
      rfFrequency: this.config.rfFrequency,
      magneticField: this.config.magneticField,
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Beam energy calibration',
      standard: 'CERN reference',
      results: [
        {
          parameter: 'beam energy',
          measuredValue: this.config.energy,
          referenceValue: this.config.energy,
          deviation: 0.001,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.0005,
        systematic: 0.0003,
        combined: 0.00058,
        expandedUncertainty: 0.00116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Mass Spectrometer
// ============================================================================

export class MassSpectrometer extends BaseInstrument {
  private config: MassSpectrometerConfig;

  constructor(config: MassSpectrometerConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Analyze sample
   */
  async analyze(params: {
    sample: string;
    ionization: string;
    polarity: 'positive' | 'negative';
  }): Promise<MassSpectrum> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Mass spectrometer not initialized'
      );
    }

    // Simulate mass spectrum
    const peaks = this.generateSimulatedPeaks();
    const basePeak = peaks.reduce((max, peak) => (peak.intensity > max.intensity ? peak : max));

    return {
      spectrumId: `MS-${Date.now()}`,
      timestamp: new Date().toISOString(),
      msLevel: 1,
      peaks,
      basePeak,
      totalIonCurrent: peaks.reduce((sum, p) => sum + p.intensity, 0),
      scanRange: this.config.massRange,
    };
  }

  private generateSimulatedPeaks() {
    const peaks = [];
    const numPeaks = Math.floor(Math.random() * 50) + 10;

    for (let i = 0; i < numPeaks; i++) {
      peaks.push({
        mz: Math.random() * (this.config.massRange[1] - this.config.massRange[0]) +
          this.config.massRange[0],
        intensity: Math.random() * 1e6,
        charge: 1,
        resolution: this.config.resolution,
        signalToNoise: Math.random() * 100 + 10,
      });
    }

    return peaks.sort((a, b) => a.mz - b.mz);
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Mass calibration',
      standard: 'Caffeine + Reserpine',
      results: [
        {
          parameter: 'mass accuracy',
          measuredValue: 1.2,
          referenceValue: this.config.massAccuracy,
          deviation: 0.2,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.1,
        systematic: 0.05,
        combined: 0.11,
        expandedUncertainty: 0.22,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Electron Microscope
// ============================================================================

export class ElectronMicroscope extends BaseInstrument {
  private config: ElectronMicroscopeConfig;

  constructor(config: ElectronMicroscopeConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Acquire image
   */
  async acquire(params: {
    mode: 'bright-field' | 'dark-field';
    exposureTime: number;
    defocus?: number;
  }): Promise<MicroscopeImage> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Electron microscope not initialized'
      );
    }

    return {
      imageId: `IMG-${Date.now()}`,
      timestamp: new Date().toISOString(),
      mode: params.mode,
      magnification: 100000,
      resolution: this.config.resolution,
      accelerationVoltage: this.config.accelerationVoltage,
      pixelSize: 0.1,
      dimensions: {
        width: 2048,
        height: 2048,
      },
      defocus: params.defocus,
      exposureTime: params.exposureTime,
      data: 'base64_encoded_image_data',
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Resolution calibration',
      standard: 'Gold nanoparticles',
      results: [
        {
          parameter: 'resolution',
          measuredValue: this.config.resolution,
          referenceValue: this.config.resolution,
          deviation: 0.01,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.005,
        systematic: 0.003,
        combined: 0.0058,
        expandedUncertainty: 0.0116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 180 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// X-ray Crystallography
// ============================================================================

export class XRayCrystallography extends BaseInstrument {
  private config: XRayCrystallographyConfig;

  constructor(config: XRayCrystallographyConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Collect diffraction data
   */
  async collectData(params: {
    exposureTime: number;
    oscillationRange: number;
  }): Promise<CrystalStructure> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'X-ray system not initialized'
      );
    }

    return {
      structureId: `STRUCT-${Date.now()}`,
      spaceGroup: 'P 21 21 21',
      unitCell: {
        a: 10.5,
        b: 12.3,
        c: 15.7,
        alpha: 90,
        beta: 90,
        gamma: 90,
      },
      resolution: this.config.resolution,
      rFactor: 0.185,
      atoms: [],
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Wavelength calibration',
      standard: 'Silicon standard',
      results: [
        {
          parameter: 'wavelength',
          measuredValue: this.config.wavelength,
          referenceValue: this.config.wavelength,
          deviation: 0.0001,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.00005,
        systematic: 0.00003,
        combined: 0.000058,
        expandedUncertainty: 0.000116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 180 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// NMR Spectrometer
// ============================================================================

export class NMRSpectrometer extends BaseInstrument {
  private config: NMRSpectrometerConfig;

  constructor(config: NMRSpectrometerConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Acquire NMR spectrum
   */
  async acquireSpectrum(params: {
    nucleus: '1H' | '13C' | '15N' | '31P';
    pulseSequence: string;
    scans: number;
  }): Promise<NMRSpectrum> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'NMR spectrometer not initialized'
      );
    }

    return {
      spectrumId: `NMR-${Date.now()}`,
      timestamp: new Date().toISOString(),
      nucleus: params.nucleus,
      frequency: this.config.fieldStrength,
      pulseSequence: params.pulseSequence as any,
      chemicalShifts: [0.9, 1.2, 7.2, 7.8],
      intensities: [3, 2, 2, 2],
      peaks: [
        { shift: 0.9, intensity: 3, width: 5, multiplicity: 't' },
        { shift: 1.2, intensity: 2, width: 5, multiplicity: 'd' },
        { shift: 7.2, intensity: 2, width: 10, multiplicity: 'm' },
        { shift: 7.8, intensity: 2, width: 8, multiplicity: 'd' },
      ],
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Field homogeneity calibration',
      standard: 'TMS (tetramethylsilane)',
      results: [
        {
          parameter: 'homogeneity',
          measuredValue: this.config.homogeneity,
          referenceValue: this.config.homogeneity,
          deviation: 0.01,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.005,
        systematic: 0.003,
        combined: 0.0058,
        expandedUncertainty: 0.0116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Telescope
// ============================================================================

export class Telescope extends BaseInstrument {
  private config: TelescopeConfig;

  constructor(config: TelescopeConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Observe target
   */
  async observe(params: {
    target: string;
    exposureTime: number;
    filter?: string;
  }): Promise<AstronomicalObservation> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Telescope not initialized'
      );
    }

    return {
      observationId: `OBS-${Date.now()}`,
      timestamp: new Date().toISOString(),
      target: params.target,
      coordinates: {
        ra: 83.63,
        dec: -5.39,
      },
      exposureTime: params.exposureTime,
      wavelength: 550,
      magnitude: 0.45,
      data: 'FITS_file_path',
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Pointing calibration',
      standard: 'Polaris',
      results: [
        {
          parameter: 'pointing accuracy',
          measuredValue: 1.0,
          referenceValue: 1.0,
          deviation: 0.1,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.05,
        systematic: 0.03,
        combined: 0.058,
        expandedUncertainty: 0.116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Spectrophotometer
// ============================================================================

export class Spectrophotometer extends BaseInstrument {
  private config: SpectrophotometerConfig;

  constructor(config: SpectrophotometerConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Measure absorption spectrum
   */
  async measureSpectrum(params: {
    sample: any;
    wavelengthRange?: [number, number];
  }): Promise<AbsorptionSpectrum> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Spectrophotometer not initialized'
      );
    }

    const wavelengths = Array.from({ length: 100 }, (_, i) => 200 + i * 5);
    const absorbances = wavelengths.map(() => Math.random() * 2);

    return {
      spectrumId: `SPEC-${Date.now()}`,
      timestamp: new Date().toISOString(),
      wavelengths,
      absorbances,
      peaks: [
        { wavelength: 280, absorbance: 1.5, width: 20 },
        { wavelength: 450, absorbance: 0.8, width: 30 },
      ],
      sample: params.sample,
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Wavelength accuracy calibration',
      standard: 'Holmium oxide filter',
      results: [
        {
          parameter: 'wavelength accuracy',
          measuredValue: 0.5,
          referenceValue: 0.5,
          deviation: 0.1,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.05,
        systematic: 0.03,
        combined: 0.058,
        expandedUncertainty: 0.116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Chromatography
// ============================================================================

export class Chromatography extends BaseInstrument {
  private config: ChromatographyConfig;

  constructor(config: ChromatographyConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Run chromatographic separation
   */
  async runSeparation(params: {
    sample: any;
    flowRate: number;
    runtime: number;
  }): Promise<Chromatogram> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Chromatography system not initialized'
      );
    }

    const numPoints = 1000;
    const retentionTimes = Array.from({ length: numPoints }, (_, i) => i * params.runtime / numPoints);
    const intensities = retentionTimes.map(() => Math.random() * 1000);

    return {
      chromatogramId: `CHROM-${Date.now()}`,
      timestamp: new Date().toISOString(),
      retentionTimes,
      intensities,
      peaks: [
        { retentionTime: 5.2, area: 1500, height: 200, width: 0.1 },
        { retentionTime: 8.7, area: 2300, height: 350, width: 0.12 },
        { retentionTime: 12.4, area: 1800, height: 280, width: 0.11 },
      ],
      sample: params.sample,
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Flow rate calibration',
      standard: 'Caffeine standard',
      results: [
        {
          parameter: 'flow rate',
          measuredValue: 1.0,
          referenceValue: 1.0,
          deviation: 0.01,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.005,
        systematic: 0.003,
        combined: 0.0058,
        expandedUncertainty: 0.0116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Calorimeter
// ============================================================================

export class Calorimeter extends BaseInstrument {
  private config: CalorimeterConfig;

  constructor(config: CalorimeterConfig) {
    super(config);
    this.config = config;
  }

  /**
   * Run thermal analysis
   */
  async runAnalysis(params: {
    sample: any;
    heatingRate: number;
    temperatureRange: [number, number];
  }): Promise<CalorimetryData> {
    if (!this.initialized) {
      throw new InstrumentError(
        InstrumentErrorCode.INSTRUMENT_NOT_INITIALIZED,
        'Calorimeter not initialized'
      );
    }

    const numPoints = 500;
    const temperatures = Array.from(
      { length: numPoints },
      (_, i) => params.temperatureRange[0] + i * (params.temperatureRange[1] - params.temperatureRange[0]) / numPoints
    );
    const heatFlows = temperatures.map(() => Math.random() * 10 - 5);

    return {
      dataId: `CAL-DATA-${Date.now()}`,
      timestamp: new Date().toISOString(),
      temperatures,
      heatFlows,
      transitions: [
        { temperature: 156.6, enthalpy: 28.5, type: 'melting' },
        { temperature: 80.2, enthalpy: -45.3, type: 'crystallization' },
      ],
      sample: params.sample,
    };
  }

  async calibrate(): Promise<CalibrationResult> {
    return {
      calibrationId: `CAL-${Date.now()}`,
      instrumentId: this.config.serialNumber,
      timestamp: new Date().toISOString(),
      procedure: 'Temperature calibration',
      standard: 'Indium standard',
      results: [
        {
          parameter: 'melting point',
          measuredValue: 156.6,
          referenceValue: 156.6,
          deviation: 0.1,
          pass: true,
        },
      ],
      uncertainty: {
        statistical: 0.05,
        systematic: 0.03,
        combined: 0.058,
        expandedUncertainty: 0.116,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000).toISOString(),
    };
  }
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';

export {
  ParticleAccelerator,
  MassSpectrometer,
  ElectronMicroscope,
  XRayCrystallography,
  NMRSpectrometer,
  Telescope,
  Spectrophotometer,
  Chromatography,
  Calorimeter,
};

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
