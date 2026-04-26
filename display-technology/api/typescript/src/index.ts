/**
 * WIA-SEMI-008 Display Technology Standard - TypeScript SDK
 * Copyright © 2025 SmileStory Inc. / WIA
 * 弘益人間 (Benefit All Humanity)
 */

import {
  DisplayTechnology,
  ColorSpace,
  HDRFormat,
  DisplayInterface,
  DisplaySpecification,
  DisplayCalculations,
  ColorPerformance,
  BandwidthRequirement,
  DDISpecification,
  DDIAnalysis,
  DisplayTestResult,
  QualityMetrics,
  InterfaceBandwidth,
} from './types';

// Re-export types
export * from './types';

/**
 * Calculate display specifications including PPI, pixel pitch, and dimensions
 */
export function calculateDisplaySpecs(
  diagonalInches: number,
  resolutionWidth: number,
  resolutionHeight: number,
  technology: DisplayTechnology = DisplayTechnology.LCD_IPS
): DisplayCalculations {
  // Calculate diagonal in pixels
  const diagonalPixels = Math.sqrt(
    resolutionWidth * resolutionWidth + resolutionHeight * resolutionHeight
  );

  // Calculate PPI (Pixels Per Inch)
  const ppi = diagonalPixels / diagonalInches;

  // Total pixels
  const totalPixels = resolutionWidth * resolutionHeight;

  // Pixel pitch in millimeters
  const pixelPitchMm = 25.4 / ppi;

  // Calculate aspect ratio
  const aspectRatio = resolutionWidth / resolutionHeight;

  // Calculate screen dimensions
  const screenHeightInches =
    diagonalInches / Math.sqrt(1 + aspectRatio * aspectRatio);
  const screenWidthInches = screenHeightInches * aspectRatio;

  // Determine category based on PPI
  let category: string;
  if (ppi > 400) {
    category = 'Ultra High-End Mobile (Retina+)';
  } else if (ppi > 300) {
    category = 'High-End Mobile (Retina)';
  } else if (ppi > 200) {
    category = 'Premium Desktop/Laptop';
  } else if (ppi > 100) {
    category = 'Standard Desktop';
  } else {
    category = 'Large Format Display/TV';
  }

  // Contrast ratio depends on technology
  let contrastRatio: number | string;
  switch (technology) {
    case DisplayTechnology.LCD_TN:
      contrastRatio = 1000;
      break;
    case DisplayTechnology.LCD_IPS:
      contrastRatio = 1200;
      break;
    case DisplayTechnology.LCD_VA:
      contrastRatio = 5000;
      break;
    case DisplayTechnology.MiniLED_LCD:
      contrastRatio = 50000;
      break;
    case DisplayTechnology.OLED:
    case DisplayTechnology.AMOLED:
    case DisplayTechnology.WOLED:
    case DisplayTechnology.QD_OLED:
    case DisplayTechnology.MicroLED:
      contrastRatio = 'Infinite';
      break;
    default:
      contrastRatio = 1000;
  }

  const aspectRatioString = `${resolutionWidth}:${resolutionHeight}`;

  return {
    ppi: parseFloat(ppi.toFixed(2)),
    totalPixels,
    pixelPitchMm: parseFloat(pixelPitchMm.toFixed(4)),
    screenWidthInches: parseFloat(screenWidthInches.toFixed(2)),
    screenHeightInches: parseFloat(screenHeightInches.toFixed(2)),
    aspectRatio: aspectRatioString,
    contrastRatio,
    category,
  };
}

/**
 * Calculate color performance metrics
 */
export function calculateColorMetrics(
  peakBrightness: number,
  blackLevel: number,
  bitDepth: number,
  gamutCoverage: number,
  colorSpace: ColorSpace,
  hdrFormat: HDRFormat
): ColorPerformance {
  // Contrast ratio
  const contrastRatio =
    blackLevel > 0 ? peakBrightness / blackLevel : 'Infinite';

  // Dynamic range in stops
  const dynamicRangeStops =
    typeof contrastRatio === 'number'
      ? parseFloat(Math.log2(contrastRatio).toFixed(1))
      : 20; // Infinite contrast ~ 20 stops practical

  // Total colors
  const totalColors = Math.pow(2, bitDepth * 3);

  // HDR compliance
  let hdrCompliance = 'No HDR';
  if (hdrFormat === HDRFormat.HDR10 && peakBrightness >= 400) {
    hdrCompliance = 'HDR10 Certified';
  }
  if (hdrFormat === HDRFormat.HDR10_Plus && peakBrightness >= 600) {
    hdrCompliance = 'HDR10+ Certified';
  }
  if (hdrFormat === HDRFormat.DolbyVision && peakBrightness >= 1000) {
    hdrCompliance = 'Dolby Vision Certified';
  }
  if (hdrFormat === HDRFormat.HLG) {
    hdrCompliance = 'HLG Compatible';
  }

  // Professional grade
  let professionalGrade = 'Consumer Grade';
  if (gamutCoverage >= 95 && bitDepth >= 10) {
    professionalGrade = 'Professional Grade';
  }
  if (gamutCoverage >= 98 && bitDepth >= 12) {
    professionalGrade = 'Reference Grade';
  }

  return {
    contrastRatio,
    dynamicRangeStops,
    totalColors,
    hdrCompliance,
    professionalGrade,
  };
}

/**
 * Calculate bandwidth requirements and interface compatibility
 */
export function calculateBandwidth(
  resolutionWidth: number,
  resolutionHeight: number,
  refreshRate: number,
  colorDepth: number,
  compressionRatio: number = 1, // 1 = none, 3 = DSC 3:1
  chromaSubsampling: '4:4:4' | '4:2:2' | '4:2:0' = '4:4:4'
): number {
  // Chroma subsampling multiplier
  let chromaMultiplier = 1.0;
  if (chromaSubsampling === '4:2:2') chromaMultiplier = 0.667;
  if (chromaSubsampling === '4:2:0') chromaMultiplier = 0.5;

  // Calculate bandwidth in Gbps
  const pixels = resolutionWidth * resolutionHeight;
  let bandwidth =
    ((pixels * refreshRate * colorDepth * chromaMultiplier) / 1000000000) *
    1.1; // 10% blanking overhead

  // Apply compression
  bandwidth = bandwidth / compressionRatio;

  return parseFloat(bandwidth.toFixed(2));
}

/**
 * Check interface compatibility
 */
export function checkInterfaceCompatibility(
  requiredBandwidthGbps: number,
  displayInterface: DisplayInterface
): BandwidthRequirement {
  const interfaceBandwidth: Record<DisplayInterface, number> = {
    [DisplayInterface.HDMI_1_4]: 10.2,
    [DisplayInterface.HDMI_2_0]: 18,
    [DisplayInterface.HDMI_2_1]: 48,
    [DisplayInterface.DisplayPort_1_2]: 21.6,
    [DisplayInterface.DisplayPort_1_4]: 32.4,
    [DisplayInterface.DisplayPort_2_0]: 80,
    [DisplayInterface.MIPI_DSI]: 10, // 4-lane @ 2.5 Gbps
    [DisplayInterface.eDP]: 32.4,
    [DisplayInterface.LVDS]: 7,
    [DisplayInterface.V_by_One_HS]: 30, // 8 lanes typical for 4K
  };

  const maxBandwidth = interfaceBandwidth[displayInterface];
  const compatible = requiredBandwidthGbps <= maxBandwidth;

  let recommendedCable = 'Standard Cable';
  if (displayInterface.includes('HDMI_2_1')) {
    recommendedCable = 'Ultra High Speed HDMI Cable';
  } else if (displayInterface.includes('HDMI_2_0')) {
    recommendedCable = 'Premium High Speed HDMI Cable';
  } else if (displayInterface.includes('DisplayPort')) {
    recommendedCable = 'DisplayPort Certified Cable';
  }

  return {
    requiredGbps: requiredBandwidthGbps,
    interfaceMaxGbps: maxBandwidth,
    compatible,
    recommendedCable,
  };
}

/**
 * Analyze DDI (Display Driver IC) requirements
 */
export function analyzeDDI(spec: DDISpecification): DDIAnalysis {
  const channelsPerChip = 384; // Typical source driver channels per chip
  const ddiChipsNeeded = Math.ceil(spec.sourceChannels / channelsPerChip);

  let tconRequirements = 'Single TCON';
  if (spec.gateLines > 2160) {
    tconRequirements = 'Dual TCON required';
  }

  // Estimate power consumption
  let powerEstimate =
    (spec.gateLines * spec.sourceChannels) / 1000000 / 2; // W

  // Adjust for panel type
  if (spec.panelType === 'OLED' || spec.panelType === 'AMOLED') {
    powerEstimate *= 0.7;
  }

  // LTPO saves power
  if (spec.powerManagement === 'LTPO') {
    powerEstimate *= 0.8;
  }

  let complexity = 'Medium';
  if (spec.ddiType === 'COG') {
    complexity = 'High (requires specialized equipment)';
  } else if (spec.ddiType === 'COF') {
    complexity = 'Very High (flexible substrate bonding)';
  }

  return {
    ddiChipsNeeded,
    tconRequirements,
    powerConsumptionW: parseFloat(powerEstimate.toFixed(2)),
    manufacturingComplexity: complexity,
  };
}

/**
 * Evaluate display quality metrics
 */
export function evaluateQuality(
  brightnessUniformity: number,
  colorAccuracyDeltaE: number,
  deadPixelCount: number,
  responseTimeMs: number
): QualityMetrics {
  let qualityGrade: 'Excellent' | 'Good' | 'Acceptable' | 'Poor';

  // Scoring system (0-100)
  let score = 0;

  // Brightness uniformity (0-30 points)
  if (brightnessUniformity >= 95) score += 30;
  else if (brightnessUniformity >= 90) score += 25;
  else if (brightnessUniformity >= 85) score += 15;
  else score += 5;

  // Color accuracy (0-30 points)
  if (colorAccuracyDeltaE < 1) score += 30;
  else if (colorAccuracyDeltaE < 2) score += 25;
  else if (colorAccuracyDeltaE < 3) score += 15;
  else score += 5;

  // Dead pixels (0-20 points)
  if (deadPixelCount === 0) score += 20;
  else if (deadPixelCount <= 2) score += 15;
  else if (deadPixelCount <= 5) score += 5;
  else score += 0;

  // Response time (0-20 points)
  if (responseTimeMs < 1) score += 20;
  else if (responseTimeMs < 3) score += 18;
  else if (responseTimeMs < 5) score += 15;
  else if (responseTimeMs < 10) score += 10;
  else score += 5;

  // Determine grade
  if (score >= 90) qualityGrade = 'Excellent';
  else if (score >= 75) qualityGrade = 'Good';
  else if (score >= 60) qualityGrade = 'Acceptable';
  else qualityGrade = 'Poor';

  return {
    brightnessUniformity,
    colorAccuracyDeltaE,
    deadPixelCount,
    responseTimeMs,
    qualityGrade,
  };
}

/**
 * Get interface bandwidth details
 */
export function getInterfaceDetails(
  displayInterface: DisplayInterface
): InterfaceBandwidth {
  const details: Record<DisplayInterface, InterfaceBandwidth> = {
    [DisplayInterface.HDMI_2_1]: {
      interface: DisplayInterface.HDMI_2_1,
      maxBandwidthGbps: 48,
      maxResolution: '8K@60Hz, 4K@120Hz',
      features: ['VRR', 'DSC', 'eARC', 'ALLM', 'QMS'],
    },
    [DisplayInterface.HDMI_2_0]: {
      interface: DisplayInterface.HDMI_2_0,
      maxBandwidthGbps: 18,
      maxResolution: '4K@60Hz',
      features: ['HDR', 'ARC'],
    },
    [DisplayInterface.HDMI_1_4]: {
      interface: DisplayInterface.HDMI_1_4,
      maxBandwidthGbps: 10.2,
      maxResolution: '4K@30Hz',
      features: ['3D', 'ARC'],
    },
    [DisplayInterface.DisplayPort_2_0]: {
      interface: DisplayInterface.DisplayPort_2_0,
      maxBandwidthGbps: 80,
      maxResolution: '16K@60Hz, 8K@120Hz',
      features: ['DSC', 'MST', 'VRR', 'Panel Replay'],
    },
    [DisplayInterface.DisplayPort_1_4]: {
      interface: DisplayInterface.DisplayPort_1_4,
      maxBandwidthGbps: 32.4,
      maxResolution: '8K@60Hz, 4K@120Hz',
      features: ['DSC', 'HDR', 'MST', 'Adaptive Sync'],
    },
    [DisplayInterface.DisplayPort_1_2]: {
      interface: DisplayInterface.DisplayPort_1_2,
      maxBandwidthGbps: 21.6,
      maxResolution: '4K@60Hz',
      features: ['MST', 'Adaptive Sync'],
    },
    [DisplayInterface.MIPI_DSI]: {
      interface: DisplayInterface.MIPI_DSI,
      maxBandwidthGbps: 10,
      maxResolution: 'Mobile displays',
      features: ['Low power', 'Embedded', 'Command mode'],
    },
    [DisplayInterface.eDP]: {
      interface: DisplayInterface.eDP,
      maxBandwidthGbps: 32.4,
      maxResolution: '4K@120Hz',
      features: ['PSR', 'Adaptive Sync', 'Panel Self-Refresh'],
    },
    [DisplayInterface.LVDS]: {
      interface: DisplayInterface.LVDS,
      maxBandwidthGbps: 7,
      maxResolution: '1080p@60Hz',
      features: ['Legacy', 'Low EMI'],
    },
    [DisplayInterface.V_by_One_HS]: {
      interface: DisplayInterface.V_by_One_HS,
      maxBandwidthGbps: 30,
      maxResolution: '4K internal',
      features: ['Embedded clock', 'TV internal use'],
    },
  };

  return details[displayInterface];
}

/**
 * Default export of main SDK object
 */
export default {
  calculateDisplaySpecs,
  calculateColorMetrics,
  calculateBandwidth,
  checkInterfaceCompatibility,
  analyzeDDI,
  evaluateQuality,
  getInterfaceDetails,
  DisplayTechnology,
  ColorSpace,
  HDRFormat,
  DisplayInterface,
};
