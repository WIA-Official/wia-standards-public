/**
 * WIA-SEMI-008 Display Technology Standard - TypeScript Type Definitions
 * Copyright © 2025 SmileStory Inc. / WIA
 * 弘益人間 (Benefit All Humanity)
 */

/**
 * Display technology types
 */
export enum DisplayTechnology {
  LCD_TN = 'LCD_TN',
  LCD_IPS = 'LCD_IPS',
  LCD_VA = 'LCD_VA',
  OLED = 'OLED',
  AMOLED = 'AMOLED',
  WOLED = 'WOLED',
  QD_OLED = 'QD_OLED',
  MicroLED = 'MicroLED',
  MiniLED_LCD = 'MiniLED_LCD',
}

/**
 * Color space standards
 */
export enum ColorSpace {
  sRGB = 'sRGB',
  AdobeRGB = 'AdobeRGB',
  DCI_P3 = 'DCI_P3',
  Display_P3 = 'Display_P3',
  Rec2020 = 'Rec2020',
}

/**
 * HDR standard formats
 */
export enum HDRFormat {
  None = 'None',
  HDR10 = 'HDR10',
  HDR10_Plus = 'HDR10_Plus',
  DolbyVision = 'DolbyVision',
  HLG = 'HLG',
}

/**
 * Display interface protocols
 */
export enum DisplayInterface {
  HDMI_1_4 = 'HDMI_1_4',
  HDMI_2_0 = 'HDMI_2_0',
  HDMI_2_1 = 'HDMI_2_1',
  DisplayPort_1_2 = 'DisplayPort_1_2',
  DisplayPort_1_4 = 'DisplayPort_1_4',
  DisplayPort_2_0 = 'DisplayPort_2_0',
  MIPI_DSI = 'MIPI_DSI',
  eDP = 'eDP',
  LVDS = 'LVDS',
  V_by_One_HS = 'V_by_One_HS',
}

/**
 * Display specifications
 */
export interface DisplaySpecification {
  /** Technology type */
  technology: DisplayTechnology;

  /** Screen diagonal in inches */
  diagonalInches: number;

  /** Resolution width in pixels */
  resolutionWidth: number;

  /** Resolution height in pixels */
  resolutionHeight: number;

  /** Refresh rate in Hz */
  refreshRate: number;

  /** Peak brightness in nits */
  peakBrightness: number;

  /** Black level in nits (0 for OLED) */
  blackLevel: number;

  /** Color space */
  colorSpace: ColorSpace;

  /** Color gamut coverage percentage */
  gamutCoverage: number;

  /** Bit depth (6, 8, 10, 12) */
  bitDepth: number;

  /** HDR format support */
  hdrFormat: HDRFormat;

  /** Response time in milliseconds (GTG) */
  responseTimeMs: number;
}

/**
 * Display calculations result
 */
export interface DisplayCalculations {
  /** Pixels per inch */
  ppi: number;

  /** Total number of pixels */
  totalPixels: number;

  /** Pixel pitch in millimeters */
  pixelPitchMm: number;

  /** Screen width in inches */
  screenWidthInches: number;

  /** Screen height in inches */
  screenHeightInches: number;

  /** Aspect ratio */
  aspectRatio: string;

  /** Contrast ratio */
  contrastRatio: number | string;

  /** Display category */
  category: string;
}

/**
 * Color performance metrics
 */
export interface ColorPerformance {
  /** Contrast ratio */
  contrastRatio: number | string;

  /** Dynamic range in stops */
  dynamicRangeStops: number;

  /** Total number of colors */
  totalColors: number;

  /** HDR compliance status */
  hdrCompliance: string;

  /** Professional grade classification */
  professionalGrade: string;
}

/**
 * Bandwidth requirements
 */
export interface BandwidthRequirement {
  /** Required bandwidth in Gbps */
  requiredGbps: number;

  /** Interface maximum bandwidth in Gbps */
  interfaceMaxGbps: number;

  /** Compatibility status */
  compatible: boolean;

  /** Recommended cable type */
  recommendedCable: string;
}

/**
 * DDI (Display Driver IC) specifications
 */
export interface DDISpecification {
  /** Panel type */
  panelType: 'LCD' | 'OLED' | 'AMOLED' | 'MicroLED';

  /** Number of gate lines */
  gateLines: number;

  /** Number of source driver channels */
  sourceChannels: number;

  /** DDI packaging type */
  ddiType: 'COG' | 'COF' | 'TAB' | 'COB';

  /** Power management type */
  powerManagement: 'Standard' | 'LTPO' | 'AOD';
}

/**
 * DDI analysis result
 */
export interface DDIAnalysis {
  /** Number of DDI chips required */
  ddiChipsNeeded: number;

  /** TCON requirements */
  tconRequirements: string;

  /** Estimated power consumption in watts */
  powerConsumptionW: number;

  /** Manufacturing complexity */
  manufacturingComplexity: string;
}

/**
 * Test result for display quality
 */
export interface DisplayTestResult {
  /** Test name */
  testName: string;

  /** Measured value */
  measuredValue: number;

  /** Target value */
  targetValue: number;

  /** Tolerance */
  tolerance: number;

  /** Pass/fail status */
  passed: boolean;

  /** Unit of measurement */
  unit: string;
}

/**
 * Quality metrics
 */
export interface QualityMetrics {
  /** Brightness uniformity percentage */
  brightnessUniformity: number;

  /** Color accuracy (Delta E) */
  colorAccuracyDeltaE: number;

  /** Dead pixel count */
  deadPixelCount: number;

  /** Response time in ms */
  responseTimeMs: number;

  /** Overall quality grade */
  qualityGrade: 'Excellent' | 'Good' | 'Acceptable' | 'Poor';
}

/**
 * Interface bandwidth details
 */
export interface InterfaceBandwidth {
  /** Interface type */
  interface: DisplayInterface;

  /** Maximum bandwidth in Gbps */
  maxBandwidthGbps: number;

  /** Maximum resolution string */
  maxResolution: string;

  /** Key features */
  features: string[];
}
