/**
 * WIA-SEMI-010 MicroLED Standard - TypeScript Type Definitions
 * © 2025 SmileStory Inc. / WIA
 */

export enum ChipSize {
  ULTRA_MICRO = 'ultra-micro', // <10μm
  SMALL = 'small',             // 10-30μm
  MEDIUM = 'medium',           // 30-75μm
  LARGE = 'large'              // 75-150μm
}

export enum LEDColor {
  BLUE = 'blue',
  GREEN = 'green',
  RED_ALGAINP = 'red-algainp',
  RED_INGAN = 'red-ingan'
}

export enum TransferMethod {
  ELASTOMER_STAMP = 'elastomer-stamp',
  LIFT = 'lift',
  FLUIDIC_ASSEMBLY = 'fluidic-assembly',
  PICK_AND_PLACE = 'pick-and-place'
}

export enum TFTTechnology {
  LTPS = 'ltps',
  OXIDE = 'oxide',
  LTPO = 'ltpo',
  CMOS = 'cmos'
}

export interface ChipSpecification {
  size: ChipSize;
  dimensions: {
    width: number;  // micrometers
    height: number; // micrometers
    thickness: number; // micrometers
  };
  color: LEDColor;
  opticalProperties: OpticalProperties;
  electricalProperties: ElectricalProperties;
  tolerance: {
    sizeVariation: number; // percentage
    wavelength: number; // nanometers
  };
}

export interface OpticalProperties {
  peakWavelength: number; // nanometers
  spectralWidth: number;  // FWHM in nanometers
  luminousIntensity: number; // millicandelas
  externalQuantumEfficiency: number; // percentage
  viewingAngle: number; // degrees at 50% intensity
}

export interface ElectricalProperties {
  forwardVoltage: {
    typical: number; // volts
    min: number;
    max: number;
  };
  reverseLeakage: number; // nanoamperes at -5V
  seriesResistance: number; // ohms
  capacitance: number; // picofarads
}

export interface TransferSpecification {
  method: TransferMethod;
  yieldRequirement: number; // percentage (e.g., 99.99)
  placementAccuracy: {
    position: number; // micrometers (±)
    rotation: number; // degrees (±)
    height: number; // micrometers (±)
  };
  throughput: number; // chips per hour
  parameters?: TransferParameters;
}

export interface TransferParameters {
  // Elastomer Stamp
  stampMaterial?: string;
  pickupPressure?: number; // MPa
  contactTime?: number; // seconds
  releaseVelocity?: number; // mm/s
  temperature?: number; // Celsius

  // LIFT
  laserWavelength?: number; // nanometers
  pulseDuration?: number; // nanoseconds
  energy?: number; // microjoules
  spotSize?: number; // micrometers
  repetitionRate?: number; // kHz

  // Fluidic
  fluidType?: string;
  flowRate?: number; // mL/min
  assemblyTime?: number; // minutes

  // Pick-and-Place
  vacuumPressure?: number; // mbar
  placementForce?: number; // grams
  speed?: number; // chips/hour
}

export interface DisplaySpecification {
  application: string;
  resolution: {
    width: number; // pixels
    height: number; // pixels
  };
  pixelPitch: number; // millimeters
  chipSize: ChipSize;
  totalChips: number;
  tftTechnology: TFTTechnology;
  brightness: {
    typical: number; // nits
    peak: number; // nits
  };
  colorGamut: string; // e.g., "DCI-P3", "Rec.2020"
  refreshRate: number; // Hz
}

export interface QualityMetrics {
  transferYield: number; // percentage
  defectRate: {
    missing: number; // ppm
    misplaced: number; // ppm
    damaged: number; // ppm
    total: number; // ppm
  };
  uniformity: {
    brightness: number; // percentage variation
    color: number; // Delta E (CIEDE2000)
  };
  reliability: {
    lifetime: number; // hours to 70% brightness
    failureRate: number; // FIT (failures in time)
  };
}

export interface CalibrationData {
  pixelCorrections: PixelCorrection[];
  gamma: GammaCalibration;
  whitePoint: {
    x: number;
    y: number;
  };
}

export interface PixelCorrection {
  address: {
    row: number;
    column: number;
  };
  correction: {
    red: number; // 0.0 - 2.0 multiplier
    green: number;
    blue: number;
  };
}

export interface GammaCalibration {
  targetGamma: number; // typically 2.2 or 2.4
  lookupTable: number[]; // 256 or 1024 entries
}

export interface DefectMap {
  totalPixels: number;
  defectivePixels: DefectPixel[];
  classification: DefectClassification;
}

export interface DefectPixel {
  address: {
    row: number;
    column: number;
  };
  type: DefectType;
  severity: DefectSeverity;
}

export enum DefectType {
  DEAD = 'dead',
  BRIGHT = 'bright',
  STUCK = 'stuck',
  MISPLACED = 'misplaced',
  DAMAGED = 'damaged'
}

export enum DefectSeverity {
  CRITICAL = 'critical',
  MAJOR = 'major',
  MINOR = 'minor'
}

export interface DefectClassification {
  class: 'I' | 'II' | 'III' | 'IV'; // ISO 9241-305
  deadPixels: number;
  brightPixels: number;
  totalDefects: number;
}

export interface TestResult {
  timestamp: Date;
  testType: string;
  passed: boolean;
  measurements: {
    [key: string]: number | string;
  };
  notes?: string;
}

export interface ManufacturingParameters {
  waferSize: number; // inches
  epitaxyMethod: 'MOCVD' | 'MBE';
  substrate: 'Sapphire' | 'SiC' | 'GaN' | 'Silicon';
  processTemperature: number; // Celsius
  chipYield: number; // percentage
  batchSize: number; // wafers or panels
}
