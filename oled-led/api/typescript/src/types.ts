/**
 * WIA-SEMI-009: OLED/LED Display Standards
 * TypeScript Type Definitions
 */

/**
 * Display Technology Types
 */
export enum DisplayTechnology {
  WOLED = 'WOLED',
  QD_OLED = 'QD-OLED',
  TANDEM_OLED = 'TANDEM-OLED',
  RGB_OLED = 'RGB-OLED',
  MINI_LED = 'MINI-LED',
  EDGE_LED = 'EDGE-LED',
  DIRECT_LED = 'DIRECT-LED',
  MICRO_LED = 'MICRO-LED',
}

/**
 * Display Application Categories
 */
export enum ApplicationCategory {
  SMARTPHONE = 'SMARTPHONE',
  TABLET = 'TABLET',
  LAPTOP = 'LAPTOP',
  MONITOR = 'MONITOR',
  TV_CONSUMER = 'TV_CONSUMER',
  TV_PREMIUM = 'TV_PREMIUM',
  AUTOMOTIVE = 'AUTOMOTIVE',
  MEDICAL = 'MEDICAL',
  PROFESSIONAL = 'PROFESSIONAL',
  SIGNAGE = 'SIGNAGE',
}

/**
 * Color Space Standards
 */
export enum ColorSpace {
  SRGB = 'sRGB',
  DCI_P3 = 'DCI-P3',
  REC_2020 = 'Rec.2020',
  ADOBE_RGB = 'Adobe RGB',
}

/**
 * Luminance Measurement
 */
export interface LuminanceSpec {
  /** Full-screen white luminance in Cd/m² (nits) */
  fullScreen: number;
  
  /** Peak brightness (10% window) in Cd/m² */
  peak10Percent: number;
  
  /** Peak brightness (3% window) in Cd/m² */
  peak3Percent?: number;
  
  /** Black level in Cd/m² */
  blackLevel: number;
  
  /** Measurement temperature in °C */
  temperature: number;
  
  /** ABL (Automatic Brightness Limiter) reduction percentage */
  ablReduction?: number;
}

/**
 * Contrast Ratio Specifications
 */
export interface ContrastSpec {
  /** Native contrast ratio (without local dimming) */
  native: number;
  
  /** ANSI contrast (16x16 checkerboard) */
  ansi: number;
  
  /** Sequential contrast (full white / full black) */
  sequential?: number;
  
  /** Dynamic contrast (with local dimming, if applicable) */
  dynamic?: number;
}

/**
 * Color Performance Metrics
 */
export interface ColorPerformance {
  /** Color gamut coverage percentages */
  gamut: {
    [ColorSpace.SRGB]?: number;
    [ColorSpace.DCI_P3]?: number;
    [ColorSpace.REC_2020]?: number;
  };
  
  /** Color accuracy in ΔE2000 */
  accuracy: {
    /** Average ΔE across test patches */
    average: number;
    
    /** Maximum ΔE */
    maximum: number;
  };
  
  /** CIE 1931 xy coordinates of primaries */
  primaries?: {
    red: { x: number; y: number };
    green: { x: number; y: number };
    blue: { x: number; y: number };
    white: { x: number; y: number };
  };
}

/**
 * Response Time Measurements
 */
export interface ResponseTime {
  /** Gray-to-gray response time in milliseconds */
  grayToGray: number;
  
  /** Black to white rise time in ms */
  blackToWhite?: number;
  
  /** White to black fall time in ms */
  whiteToBlack?: number;
  
  /** Input lag in ms */
  inputLag?: number;
}

/**
 * OLED Lifetime Specifications
 */
export interface OLEDLifetime {
  /** LT95 (hours to 95% initial luminance) */
  lt95: number;
  
  /** LT90 (hours to 90% initial luminance) */
  lt90?: number;
  
  /** LT50 (hours to 50% initial luminance) */
  lt50?: number;
  
  /** Test luminance in Cd/m² */
  testLuminance: number;
  
  /** Test temperature in °C */
  testTemperature: number;
  
  /** Acceleration factors used */
  acceleration?: {
    /** Activation energy for temperature acceleration (eV) */
    activationEnergy?: number;
    
    /** Power law exponent for luminance acceleration */
    luminanceExponent?: number;
  };
}

/**
 * Mini-LED Backlight Specifications
 */
export interface MiniLEDBacklight {
  /** Number of LEDs in backlight */
  ledCount: number;
  
  /** Number of independent dimming zones */
  zoneCount: number;
  
  /** LED chip size in micrometers */
  ledSize: number;
  
  /** Optical distance (OD) in millimeters */
  opticalDistance: number;
  
  /** PWM dimming frequency in Hz */
  pwmFrequency?: number;
  
  /** Blooming characteristics */
  blooming?: {
    /** Halo extent as multiple of zone size */
    haloExtent: number;
    
    /** Halo luminance as percentage of peak white */
    haloLuminance: number;
  };
}

/**
 * Complete Display Specification
 */
export interface DisplaySpecification {
  /** Display technology type */
  technology: DisplayTechnology;
  
  /** Application category */
  application: ApplicationCategory;
  
  /** Screen size in inches (diagonal) */
  screenSize: number;
  
  /** Resolution */
  resolution: {
    width: number;
    height: number;
  };
  
  /** Luminance specifications */
  luminance: LuminanceSpec;
  
  /** Contrast specifications */
  contrast: ContrastSpec;
  
  /** Color performance */
  color: ColorPerformance;
  
  /** Response time */
  responseTime: ResponseTime;
  
  /** Power consumption in watts (typical content) */
  powerConsumption?: number;
  
  /** Refresh rate in Hz */
  refreshRate?: number;
  
  /** OLED lifetime (if applicable) */
  oledLifetime?: OLEDLifetime;
  
  /** Mini-LED backlight (if applicable) */
  miniLED?: MiniLEDBacklight;
  
  /** Additional notes */
  notes?: string;
}

/**
 * Measurement Conditions
 */
export interface MeasurementConditions {
  /** Ambient temperature in °C */
  temperature: number;
  
  /** Relative humidity percentage */
  humidity: number;
  
  /** Ambient light in lux */
  ambientLight: number;
  
  /** Display warm-up time in minutes */
  warmupTime: number;
  
  /** Measurement equipment used */
  equipment?: string;
}

/**
 * Test Result
 */
export interface TestResult {
  /** Test name/identifier */
  testName: string;
  
  /** Pass/fail result */
  passed: boolean;
  
  /** Measured value */
  measuredValue: number | string;
  
  /** Specification limit */
  specLimit?: number | string;
  
  /** Units */
  unit?: string;
  
  /** Notes */
  notes?: string;
}

/**
 * Certification Level
 */
export enum CertificationLevel {
  BASIC = 1,
  STANDARD = 2,
  PREMIUM = 3,
}

/**
 * Certification Result
 */
export interface CertificationResult {
  /** Certification level achieved */
  level: CertificationLevel;
  
  /** Pass/fail for certification */
  certified: boolean;
  
  /** Individual test results */
  tests: TestResult[];
  
  /** Certification date */
  date: Date;
  
  /** Certificate number (if certified) */
  certificateNumber?: string;
}

/**
 * Calculation utilities types
 */
export interface LuminanceCalculation {
  /** Calculate luminous efficacy in lm/W */
  efficacy: number;
  
  /** Power per area in W/m² */
  powerPerArea: number;
  
  /** Energy rating */
  energyRating?: string;
}
