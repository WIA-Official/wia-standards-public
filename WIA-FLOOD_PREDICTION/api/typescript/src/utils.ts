/**
 * WIA-FLOOD_PREDICTION TypeScript SDK - Utilities
 *
 * Helper functions for flood prediction API
 *
 * @version 1.0.0
 * @license MIT
 */

import { GeoPoint, RiskLevel, FloodPrediction } from './types';

/**
 * Calculate distance between two geographic points using Haversine formula
 * @param point1 First coordinate
 * @param point2 Second coordinate
 * @returns Distance in kilometers
 */
export function calculateDistance(point1: GeoPoint, point2: GeoPoint): number {
  const R = 6371; // Earth's radius in kilometers
  const dLat = toRadians(point2.lat - point1.lat);
  const dLng = toRadians(point2.lng - point1.lng);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(toRadians(point1.lat)) *
      Math.cos(toRadians(point2.lat)) *
      Math.sin(dLng / 2) *
      Math.sin(dLng / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

/**
 * Convert degrees to radians
 */
function toRadians(degrees: number): number {
  return degrees * (Math.PI / 180);
}

/**
 * Convert radians to degrees
 */
export function toDegrees(radians: number): number {
  return radians * (180 / Math.PI);
}

/**
 * Calculate bounding box around a point
 * @param center Center point
 * @param radiusKm Radius in kilometers
 * @returns Bounding box [minLng, minLat, maxLng, maxLat]
 */
export function calculateBoundingBox(
  center: GeoPoint,
  radiusKm: number
): [number, number, number, number] {
  const latDelta = radiusKm / 111.32; // 1 degree latitude ≈ 111.32 km
  const lngDelta = radiusKm / (111.32 * Math.cos(toRadians(center.lat)));

  return [
    Math.max(-180, center.lng - lngDelta), // minLng
    Math.max(-90, center.lat - latDelta),  // minLat
    Math.min(180, center.lng + lngDelta),  // maxLng
    Math.min(90, center.lat + latDelta),   // maxLat
  ];
}

/**
 * Check if a point is within a bounding box
 */
export function isPointInBBox(
  point: GeoPoint,
  bbox: [number, number, number, number]
): boolean {
  const [minLng, minLat, maxLng, maxLat] = bbox;
  return (
    point.lng >= minLng &&
    point.lng <= maxLng &&
    point.lat >= minLat &&
    point.lat <= maxLat
  );
}

/**
 * Convert risk level to numeric severity (for sorting/comparison)
 */
export function riskLevelToSeverity(level: RiskLevel): number {
  const severityMap: Record<RiskLevel, number> = {
    low: 1,
    medium: 2,
    high: 3,
    extreme: 4,
  };
  return severityMap[level];
}

/**
 * Convert numeric severity to risk level
 */
export function severityToRiskLevel(severity: number): RiskLevel {
  if (severity <= 1) return 'low';
  if (severity <= 2) return 'medium';
  if (severity <= 3) return 'high';
  return 'extreme';
}

/**
 * Calculate risk level from probability
 * @param probability Flood probability (0.0 - 1.0)
 * @returns Risk level
 */
export function probabilityToRiskLevel(probability: number): RiskLevel {
  if (probability < 0.3) return 'low';
  if (probability < 0.6) return 'medium';
  if (probability < 0.85) return 'high';
  return 'extreme';
}

/**
 * Format date to ISO 8601 string (UTC)
 */
export function formatISODateTime(date: Date): string {
  return date.toISOString();
}

/**
 * Parse ISO 8601 datetime string
 */
export function parseISODateTime(dateStr: string): Date {
  return new Date(dateStr);
}

/**
 * Calculate time until event
 * @param eventDate Event datetime (ISO string or Date)
 * @param fromDate Reference datetime (default: now)
 * @returns Time difference in milliseconds
 */
export function timeUntilEvent(
  eventDate: string | Date,
  fromDate: Date = new Date()
): number {
  const eventTime = typeof eventDate === 'string' ? parseISODateTime(eventDate) : eventDate;
  return eventTime.getTime() - fromDate.getTime();
}

/**
 * Format time difference in human-readable format
 * @param milliseconds Time difference in milliseconds
 * @returns Formatted string (e.g., "2 hours", "3 days")
 */
export function formatTimeDifference(milliseconds: number): string {
  const seconds = Math.floor(milliseconds / 1000);
  const minutes = Math.floor(seconds / 60);
  const hours = Math.floor(minutes / 60);
  const days = Math.floor(hours / 24);

  if (days > 0) return `${days} day${days !== 1 ? 's' : ''}`;
  if (hours > 0) return `${hours} hour${hours !== 1 ? 's' : ''}`;
  if (minutes > 0) return `${minutes} minute${minutes !== 1 ? 's' : ''}`;
  return `${seconds} second${seconds !== 1 ? 's' : ''}`;
}

/**
 * Sort predictions by risk level (descending) then probability (descending)
 */
export function sortPredictionsByRisk(
  predictions: FloodPrediction[]
): FloodPrediction[] {
  return predictions.sort((a, b) => {
    const severityDiff = riskLevelToSeverity(b.risk_level) - riskLevelToSeverity(a.risk_level);
    if (severityDiff !== 0) return severityDiff;
    return b.probability - a.probability;
  });
}

/**
 * Filter predictions by minimum probability
 */
export function filterByProbability(
  predictions: FloodPrediction[],
  minProbability: number
): FloodPrediction[] {
  return predictions.filter((p) => p.probability >= minProbability);
}

/**
 * Filter predictions by risk level
 */
export function filterByRiskLevel(
  predictions: FloodPrediction[],
  riskLevels: RiskLevel[]
): FloodPrediction[] {
  return predictions.filter((p) => riskLevels.includes(p.risk_level));
}

/**
 * Filter predictions by distance from a point
 */
export function filterByDistance(
  predictions: FloodPrediction[],
  center: GeoPoint,
  maxDistanceKm: number
): FloodPrediction[] {
  return predictions.filter((p) => {
    const distance = calculateDistance(center, p.location);
    return distance <= maxDistanceKm;
  });
}

/**
 * Group predictions by risk level
 */
export function groupByRiskLevel(
  predictions: FloodPrediction[]
): Record<RiskLevel, FloodPrediction[]> {
  const grouped: Record<RiskLevel, FloodPrediction[]> = {
    low: [],
    medium: [],
    high: [],
    extreme: [],
  };

  predictions.forEach((p) => {
    grouped[p.risk_level].push(p);
  });

  return grouped;
}

/**
 * Calculate average probability for a set of predictions
 */
export function calculateAverageProbability(predictions: FloodPrediction[]): number {
  if (predictions.length === 0) return 0;
  const sum = predictions.reduce((acc, p) => acc + p.probability, 0);
  return sum / predictions.length;
}

/**
 * Find nearest prediction to a point
 */
export function findNearestPrediction(
  predictions: FloodPrediction[],
  point: GeoPoint
): FloodPrediction | null {
  if (predictions.length === 0) return null;

  let nearest = predictions[0];
  let minDistance = calculateDistance(point, nearest.location);

  predictions.slice(1).forEach((p) => {
    const distance = calculateDistance(point, p.location);
    if (distance < minDistance) {
      minDistance = distance;
      nearest = p;
    }
  });

  return nearest;
}

/**
 * Convert meters to feet
 */
export function metersToFeet(meters: number): number {
  return meters * 3.28084;
}

/**
 * Convert feet to meters
 */
export function feetToMeters(feet: number): number {
  return feet / 3.28084;
}

/**
 * Convert cubic meters per second to cubic feet per second
 */
export function m3sToFt3s(m3s: number): number {
  return m3s * 35.3147;
}

/**
 * Convert cubic feet per second to cubic meters per second
 */
export function ft3sToM3s(ft3s: number): number {
  return ft3s / 35.3147;
}

/**
 * Format depth with unit
 */
export function formatDepth(meters: number, unit: 'metric' | 'imperial' = 'metric'): string {
  if (unit === 'imperial') {
    const feet = metersToFeet(meters);
    return `${feet.toFixed(1)} ft`;
  }
  return `${meters.toFixed(1)} m`;
}

/**
 * Format discharge with unit
 */
export function formatDischarge(m3s: number, unit: 'metric' | 'imperial' = 'metric'): string {
  if (unit === 'imperial') {
    const ft3s = m3sToFt3s(m3s);
    return `${ft3s.toFixed(0)} ft³/s`;
  }
  return `${m3s.toFixed(0)} m³/s`;
}

/**
 * Format area with unit
 */
export function formatArea(km2: number, unit: 'metric' | 'imperial' = 'metric'): string {
  if (unit === 'imperial') {
    const sqMiles = km2 * 0.386102;
    return `${sqMiles.toFixed(2)} mi²`;
  }
  return `${km2.toFixed(2)} km²`;
}

/**
 * Generate prediction ID from location and date
 */
export function generatePredictionId(location: GeoPoint, date: Date): string {
  const dateStr = date.toISOString().replace(/[:.]/g, '').substring(0, 13); // YYYYMMDDTHHmm
  const lat = location.lat.toFixed(4).replace('.', '');
  const lng = location.lng.toFixed(4).replace('.', '').replace('-', 'n');
  return `pred_${dateStr}_${lat}_${lng}`;
}

/**
 * Retry a function with exponential backoff
 */
export async function retryWithBackoff<T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  initialDelayMs: number = 1000
): Promise<T> {
  let lastError: Error;

  for (let attempt = 0; attempt < maxRetries; attempt++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;

      if (attempt < maxRetries - 1) {
        const delay = initialDelayMs * Math.pow(2, attempt);
        await sleep(delay);
      }
    }
  }

  throw lastError!;
}

/**
 * Sleep for specified milliseconds
 */
export function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

/**
 * Debounce function (useful for API calls)
 */
export function debounce<T extends (...args: any[]) => any>(
  func: T,
  waitMs: number
): (...args: Parameters<T>) => void {
  let timeoutId: NodeJS.Timeout | null = null;

  return function (this: any, ...args: Parameters<T>) {
    if (timeoutId) {
      clearTimeout(timeoutId);
    }

    timeoutId = setTimeout(() => {
      func.apply(this, args);
    }, waitMs);
  };
}

/**
 * Throttle function (rate limiting)
 */
export function throttle<T extends (...args: any[]) => any>(
  func: T,
  limitMs: number
): (...args: Parameters<T>) => void {
  let lastRan: number = 0;

  return function (this: any, ...args: Parameters<T>) {
    const now = Date.now();

    if (now - lastRan >= limitMs) {
      func.apply(this, args);
      lastRan = now;
    }
  };
}

/**
 * Deep clone an object
 */
export function deepClone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj));
}

/**
 * Check if two geographic points are approximately equal
 */
export function arePointsEqual(
  point1: GeoPoint,
  point2: GeoPoint,
  toleranceMeters: number = 10
): boolean {
  const distanceMeters = calculateDistance(point1, point2) * 1000;
  return distanceMeters <= toleranceMeters;
}

/**
 * Generate a simple hash code for a string (for caching keys)
 */
export function hashCode(str: string): number {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = (hash << 5) - hash + char;
    hash = hash & hash; // Convert to 32-bit integer
  }
  return hash;
}

/**
 * Build query string from object
 */
export function buildQueryString(params: Record<string, any>): string {
  const entries = Object.entries(params)
    .filter(([_, value]) => value !== undefined && value !== null)
    .map(([key, value]) => {
      if (Array.isArray(value)) {
        return value.map((v) => `${encodeURIComponent(key)}=${encodeURIComponent(v)}`).join('&');
      }
      return `${encodeURIComponent(key)}=${encodeURIComponent(value)}`;
    });

  return entries.join('&');
}

/**
 * Parse query string to object
 */
export function parseQueryString(queryString: string): Record<string, string> {
  const params: Record<string, string> = {};

  if (!queryString) return params;

  const pairs = queryString.replace(/^\?/, '').split('&');

  pairs.forEach((pair) => {
    const [key, value] = pair.split('=').map(decodeURIComponent);
    if (key) {
      params[key] = value || '';
    }
  });

  return params;
}
