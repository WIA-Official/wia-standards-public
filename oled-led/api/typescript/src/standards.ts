/**
 * WIA-SEMI-009 Standard Requirements
 */

import { ApplicationCategory, CertificationLevel } from './types';

/**
 * Standard requirements interface
 */
export interface StandardRequirements {
  minFullScreen?: number;        // Minimum full-screen luminance (Cd/m²)
  minPeak?: number;              // Minimum peak brightness (Cd/m²)
  minContrast?: number;          // Minimum ANSI contrast
  minColorGamut?: number;        // Minimum DCI-P3 coverage (%)
  maxDeltaE?: number;            // Maximum average ΔE2000
  maxResponseTime?: number;      // Maximum response time (ms)
  minLifetime?: number;          // Minimum LT95 lifetime (hours)
}

/**
 * Get standard requirements for application and certification level
 */
export function getStandardRequirements(
  application: ApplicationCategory | string,
  level: CertificationLevel
): StandardRequirements {
  const baseReqs = getBaseRequirements(application);
  return applyLevelModifier(baseReqs, level);
}

/**
 * Get base requirements by application
 */
function getBaseRequirements(application: ApplicationCategory | string): StandardRequirements {
  const appReqs: Record<string, StandardRequirements> = {
    [ApplicationCategory.SMARTPHONE]: {
      minFullScreen: 400,
      minPeak: 800,
      minContrast: 100000,
      minColorGamut: 95,
      maxDeltaE: 3.0,
      maxResponseTime: 5,
      minLifetime: 30000,
    },
    [ApplicationCategory.TABLET]: {
      minFullScreen: 400,
      minPeak: 600,
      minContrast: 50000,
      minColorGamut: 90,
      maxDeltaE: 3.0,
      maxResponseTime: 8,
      minLifetime: 40000,
    },
    [ApplicationCategory.TV_CONSUMER]: {
      minFullScreen: 200,
      minPeak: 500,
      minContrast: 10000,
      minColorGamut: 85,
      maxDeltaE: 5.0,
      maxResponseTime: 15,
      minLifetime: 50000,
    },
    [ApplicationCategory.TV_PREMIUM]: {
      minFullScreen: 250,
      minPeak: 1000,
      minContrast: 50000,
      minColorGamut: 95,
      maxDeltaE: 3.0,
      maxResponseTime: 10,
      minLifetime: 80000,
    },
    [ApplicationCategory.MONITOR]: {
      minFullScreen: 250,
      minPeak: 400,
      minContrast: 5000,
      minColorGamut: 95,
      maxDeltaE: 2.0,
      maxResponseTime: 10,
    },
    [ApplicationCategory.AUTOMOTIVE]: {
      minFullScreen: 300,
      minPeak: 1500,
      minContrast: 20000,
      minColorGamut: 85,
      maxDeltaE: 3.0,
      maxResponseTime: 10,
      minLifetime: 100000,
    },
    [ApplicationCategory.PROFESSIONAL]: {
      minFullScreen: 250,
      minPeak: 400,
      minContrast: 10000,
      minColorGamut: 98,
      maxDeltaE: 2.0,
      maxResponseTime: 10,
      minLifetime: 80000,
    },
  };
  
  return appReqs[application] || appReqs[ApplicationCategory.TV_CONSUMER];
}

/**
 * Apply certification level modifier to requirements
 */
function applyLevelModifier(baseReqs: StandardRequirements, level: CertificationLevel): StandardRequirements {
  const modifiedReqs = { ...baseReqs };
  
  switch (level) {
    case CertificationLevel.BASIC:
      // Relax requirements by 20%
      if (modifiedReqs.minFullScreen) modifiedReqs.minFullScreen *= 0.8;
      if (modifiedReqs.minPeak) modifiedReqs.minPeak *= 0.8;
      if (modifiedReqs.minContrast) modifiedReqs.minContrast *= 0.8;
      if (modifiedReqs.minColorGamut) modifiedReqs.minColorGamut *= 0.9;
      if (modifiedReqs.maxDeltaE) modifiedReqs.maxDeltaE *= 1.5;
      if (modifiedReqs.maxResponseTime) modifiedReqs.maxResponseTime *= 1.3;
      if (modifiedReqs.minLifetime) modifiedReqs.minLifetime *= 0.7;
      break;
      
    case CertificationLevel.PREMIUM:
      // Tighten requirements by 20%
      if (modifiedReqs.minFullScreen) modifiedReqs.minFullScreen *= 1.2;
      if (modifiedReqs.minPeak) modifiedReqs.minPeak *= 1.2;
      if (modifiedReqs.minContrast) modifiedReqs.minContrast *= 1.5;
      if (modifiedReqs.minColorGamut) modifiedReqs.minColorGamut = Math.min(100, modifiedReqs.minColorGamut * 1.05);
      if (modifiedReqs.maxDeltaE) modifiedReqs.maxDeltaE *= 0.7;
      if (modifiedReqs.maxResponseTime) modifiedReqs.maxResponseTime *= 0.7;
      if (modifiedReqs.minLifetime) modifiedReqs.minLifetime *= 1.5;
      break;
      
    case CertificationLevel.STANDARD:
    default:
      // Use base requirements
      break;
  }
  
  // Round to reasonable precision
  if (modifiedReqs.minFullScreen) modifiedReqs.minFullScreen = Math.round(modifiedReqs.minFullScreen);
  if (modifiedReqs.minPeak) modifiedReqs.minPeak = Math.round(modifiedReqs.minPeak);
  if (modifiedReqs.minContrast) modifiedReqs.minContrast = Math.round(modifiedReqs.minContrast);
  if (modifiedReqs.minColorGamut) modifiedReqs.minColorGamut = Math.round(modifiedReqs.minColorGamut);
  if (modifiedReqs.maxDeltaE) modifiedReqs.maxDeltaE = Math.round(modifiedReqs.maxDeltaE * 10) / 10;
  if (modifiedReqs.maxResponseTime) modifiedReqs.maxResponseTime = Math.round(modifiedReqs.maxResponseTime);
  if (modifiedReqs.minLifetime) modifiedReqs.minLifetime = Math.round(modifiedReqs.minLifetime / 1000) * 1000;
  
  return modifiedReqs;
}

/**
 * Get color space primary coordinates
 */
export const COLOR_SPACE_PRIMARIES = {
  'sRGB': {
    red: { x: 0.640, y: 0.330 },
    green: { x: 0.300, y: 0.600 },
    blue: { x: 0.150, y: 0.060 },
    white: { x: 0.3127, y: 0.3290 },
  },
  'DCI-P3': {
    red: { x: 0.680, y: 0.320 },
    green: { x: 0.265, y: 0.690 },
    blue: { x: 0.150, y: 0.060 },
    white: { x: 0.314, y: 0.351 },
  },
  'Rec.2020': {
    red: { x: 0.708, y: 0.292 },
    green: { x: 0.170, y: 0.797 },
    blue: { x: 0.131, y: 0.046 },
    white: { x: 0.3127, y: 0.3290 },
  },
};
