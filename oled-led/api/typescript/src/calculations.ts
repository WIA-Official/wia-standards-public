/**
 * Calculation utilities for OLED/LED displays
 */

import { LuminanceSpec, LuminanceCalculation } from './types';

/**
 * Calculate luminous efficacy
 * 
 * @param luminousFlux - Luminous flux in lumens
 * @param powerConsumption - Power consumption in watts
 * @returns Luminous efficacy in lm/W
 */
export function calculateEfficacy(luminousFlux: number, powerConsumption: number): number {
  if (powerConsumption <= 0) {
    throw new Error('Power consumption must be greater than zero');
  }
  return luminousFlux / powerConsumption;
}

/**
 * Calculate average luminance from luminous flux and screen area
 * 
 * @param luminousFlux - Luminous flux in lumens
 * @param screenArea - Screen area in m²
 * @returns Average luminance in Cd/m²
 */
export function calculateAverageLuminance(luminousFlux: number, screenArea: number): number {
  if (screenArea <= 0) {
    throw new Error('Screen area must be greater than zero');
  }
  // L = Φ / (A × π) for Lambertian source
  return luminousFlux / (screenArea * Math.PI);
}

/**
 * Calculate screen area from diagonal size and aspect ratio
 * 
 * @param diagonal - Diagonal size in inches
 * @param aspectWidth - Aspect ratio width (e.g., 16 for 16:9)
 * @param aspectHeight - Aspect ratio height (e.g., 9 for 16:9)
 * @returns Screen area in m²
 */
export function calculateScreenArea(
  diagonal: number,
  aspectWidth: number = 16,
  aspectHeight: number = 9
): number {
  // Diagonal² = width² + height²
  // width = aspectWidth × k, height = aspectHeight × k
  // diagonal² = (aspectWidth × k)² + (aspectHeight × k)²
  // k = diagonal / sqrt(aspectWidth² + aspectHeight²)
  
  const k = diagonal / Math.sqrt(aspectWidth ** 2 + aspectHeight ** 2);
  const widthInches = aspectWidth * k;
  const heightInches = aspectHeight * k;
  
  // Convert to meters (1 inch = 0.0254 m)
  const widthMeters = widthInches * 0.0254;
  const heightMeters = heightInches * 0.0254;
  
  return widthMeters * heightMeters;
}

/**
 * Perform comprehensive luminance calculations
 * 
 * @param luminousFlux - Luminous flux in lumens
 * @param powerConsumption - Power consumption in watts
 * @param screenArea - Screen area in m²
 * @returns Calculation results
 */
export function performLuminanceCalculation(
  luminousFlux: number,
  powerConsumption: number,
  screenArea: number
): LuminanceCalculation {
  const efficacy = calculateEfficacy(luminousFlux, powerConsumption);
  const powerPerArea = powerConsumption / screenArea;
  
  // Energy rating based on efficacy
  let energyRating = 'B';
  if (efficacy > 10) energyRating = 'A+++';
  else if (efficacy > 8) energyRating = 'A++';
  else if (efficacy > 6) energyRating = 'A+';
  else if (efficacy > 4) energyRating = 'A';
  
  return {
    efficacy: parseFloat(efficacy.toFixed(1)),
    powerPerArea: parseFloat(powerPerArea.toFixed(1)),
    energyRating,
  };
}

/**
 * Calculate contrast ratio
 * 
 * @param whiteLuminance - White luminance in Cd/m²
 * @param blackLuminance - Black luminance in Cd/m²
 * @returns Contrast ratio
 */
export function calculateContrast(whiteLuminance: number, blackLuminance: number): number {
  if (blackLuminance <= 0) {
    return Infinity; // OLED case
  }
  return whiteLuminance / blackLuminance;
}

/**
 * Calculate OLED lifetime at different luminance using power law
 * 
 * @param knownLifetime - Known lifetime in hours
 * @param knownLuminance - Luminance at which lifetime is known (Cd/m²)
 * @param targetLuminance - Target luminance (Cd/m²)
 * @param exponent - Power law exponent (typically 1.5-2.0)
 * @returns Estimated lifetime at target luminance
 */
export function calculateOLEDLifetime(
  knownLifetime: number,
  knownLuminance: number,
  targetLuminance: number,
  exponent: number = 1.7
): number {
  // τ₂ = τ₁ × (L₁/L₂)^n
  return knownLifetime * Math.pow(knownLuminance / targetLuminance, exponent);
}

/**
 * Calculate color gamut coverage percentage
 * 
 * @param displayPrimaries - Display primary coordinates {red, green, blue}
 * @param targetPrimaries - Target color space primaries
 * @returns Coverage percentage (0-100)
 */
export function calculateGamutCoverage(
  displayPrimaries: { red: {x: number, y: number}, green: {x: number, y: number}, blue: {x: number, y: number} },
  targetPrimaries: { red: {x: number, y: number}, green: {x: number, y: number}, blue: {x: number, y: number} }
): number {
  // Calculate triangle area using cross product
  const displayArea = calculateTriangleArea(
    displayPrimaries.red,
    displayPrimaries.green,
    displayPrimaries.blue
  );
  
  const targetArea = calculateTriangleArea(
    targetPrimaries.red,
    targetPrimaries.green,
    targetPrimaries.blue
  );
  
  return (displayArea / targetArea) * 100;
}

/**
 * Calculate triangle area from three points
 */
function calculateTriangleArea(
  p1: {x: number, y: number},
  p2: {x: number, y: number},
  p3: {x: number, y: number}
): number {
  // Area = 0.5 × |x1(y2-y3) + x2(y3-y1) + x3(y1-y2)|
  return 0.5 * Math.abs(
    p1.x * (p2.y - p3.y) +
    p2.x * (p3.y - p1.y) +
    p3.x * (p1.y - p2.y)
  );
}

/**
 * Estimate Mini-LED blooming based on zone count and optical distance
 * 
 * @param zoneCount - Number of dimming zones
 * @param screenArea - Screen area in m²
 * @param opticalDistance - Optical distance in mm
 * @returns Blooming estimate (halo extent as multiple of zone size)
 */
export function estimateBlooming(
  zoneCount: number,
  screenArea: number,
  opticalDistance: number
): number {
  const zoneArea = screenArea / zoneCount;
  const zoneSize = Math.sqrt(zoneArea); // Approximate zone dimension in meters
  
  // Empirical model: blooming increases with OD, decreases with zone count
  // This is a simplified model for estimation
  const odMeters = opticalDistance / 1000; // Convert mm to m
  const haloExtent = 1.0 + (odMeters / zoneSize) * 0.5;
  
  return parseFloat(haloExtent.toFixed(2));
}
