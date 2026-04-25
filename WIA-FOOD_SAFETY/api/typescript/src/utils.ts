/**
 * WIA-FOOD_SAFETY Utility Functions
 * Helper functions for food safety operations
 */

import { createHash } from 'crypto';

/**
 * Generate unique batch ID from product data
 */
export function generateBatchId(prefix: string, date: Date, sequence: number): string {
  const year = date.getFullYear();
  const sequenceStr = sequence.toString().padStart(6, '0');
  return `${prefix}-${year}-${sequenceStr}`;
}

/**
 * Calculate SHA-256 hash of data
 */
export function calculateHash(data: string | object): string {
  const dataStr = typeof data === 'string' ? data : JSON.stringify(data);
  return createHash('sha256').update(dataStr).digest('hex');
}

/**
 * Calculate Merkle root for batch of temperature readings
 */
export function calculateMerkleRoot(readings: any[]): string {
  if (readings.length === 0) {
    return calculateHash('');
  }

  // Hash each reading
  let hashes = readings.map(reading => calculateHash(reading));

  // Build Merkle tree
  while (hashes.length > 1) {
    const newHashes: string[] = [];
    for (let i = 0; i < hashes.length; i += 2) {
      if (i + 1 < hashes.length) {
        newHashes.push(calculateHash(hashes[i] + hashes[i + 1]));
      } else {
        newHashes.push(hashes[i]);
      }
    }
    hashes = newHashes;
  }

  return hashes[0];
}

/**
 * Check if temperature is within critical limits
 */
export function isWithinCriticalLimit(
  temperature: number,
  criticalLimit: { min?: number; max?: number }
): boolean {
  if (criticalLimit.min !== undefined && temperature < criticalLimit.min) {
    return false;
  }
  if (criticalLimit.max !== undefined && temperature > criticalLimit.max) {
    return false;
  }
  return true;
}

/**
 * Calculate traceability query time
 */
export function calculateTraceabilityTime(startTime: number): string {
  const endTime = Date.now();
  const durationMs = endTime - startTime;
  const durationSec = (durationMs / 1000).toFixed(2);
  return `${durationSec}s`;
}

/**
 * Format GPS coordinates for display
 */
export function formatCoordinates(latitude: number, longitude: number): string {
  const latDir = latitude >= 0 ? 'N' : 'S';
  const lonDir = longitude >= 0 ? 'E' : 'W';
  return `${Math.abs(latitude).toFixed(4)}°${latDir}, ${Math.abs(longitude).toFixed(4)}°${lonDir}`;
}

/**
 * Calculate distance between two GPS coordinates (Haversine formula)
 * Returns distance in kilometers
 */
export function calculateDistance(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 6371; // Earth's radius in km
  const dLat = toRadians(lat2 - lat1);
  const dLon = toRadians(lon2 - lon1);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(toRadians(lat1)) *
      Math.cos(toRadians(lat2)) *
      Math.sin(dLon / 2) *
      Math.sin(dLon / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

function toRadians(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

/**
 * Format temperature with unit
 */
export function formatTemperature(celsius: number, unit: 'C' | 'F' = 'C'): string {
  if (unit === 'F') {
    const fahrenheit = (celsius * 9) / 5 + 32;
    return `${fahrenheit.toFixed(1)}°F`;
  }
  return `${celsius.toFixed(1)}°C`;
}

/**
 * Convert Celsius to Fahrenheit
 */
export function celsiusToFahrenheit(celsius: number): number {
  return (celsius * 9) / 5 + 32;
}

/**
 * Convert Fahrenheit to Celsius
 */
export function fahrenheitToCelsius(fahrenheit: number): number {
  return ((fahrenheit - 32) * 5) / 9;
}

/**
 * Calculate expiration warning days
 */
export function calculateDaysUntilExpiration(expirationDate: Date): number {
  const now = new Date();
  const diff = expirationDate.getTime() - now.getTime();
  return Math.ceil(diff / (1000 * 60 * 60 * 24));
}

/**
 * Check if product is expired
 */
export function isExpired(expirationDate: Date): boolean {
  return new Date() > expirationDate;
}

/**
 * Format date for display
 */
export function formatDate(date: Date): string {
  return date.toISOString().split('T')[0];
}

/**
 * Format duration in milliseconds to human-readable format
 */
export function formatDuration(ms: number): string {
  const seconds = Math.floor(ms / 1000);
  const minutes = Math.floor(seconds / 60);
  const hours = Math.floor(minutes / 60);
  const days = Math.floor(hours / 24);

  if (days > 0) {
    return `${days}d ${hours % 24}h`;
  } else if (hours > 0) {
    return `${hours}h ${minutes % 60}m`;
  } else if (minutes > 0) {
    return `${minutes}m ${seconds % 60}s`;
  } else {
    return `${seconds}s`;
  }
}

/**
 * Generate QR code data URL (Base64 encoded PNG)
 * Note: In production, use a proper QR code library like 'qrcode'
 */
export function generateQRCodeDataUrl(batchId: string): string {
  // Placeholder: In production, use proper QR code generation
  const data = `https://wia-food-safety.org/trace/${batchId}`;
  return `data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==`;
}

/**
 * Paginate array results
 */
export function paginate<T>(
  items: T[],
  page: number = 1,
  limit: number = 20
): {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    totalPages: number;
    totalRecords: number;
  };
} {
  const totalRecords = items.length;
  const totalPages = Math.ceil(totalRecords / limit);
  const startIndex = (page - 1) * limit;
  const endIndex = startIndex + limit;
  const data = items.slice(startIndex, endIndex);

  return {
    data,
    pagination: {
      page,
      limit,
      totalPages,
      totalRecords
    }
  };
}

/**
 * Retry async function with exponential backoff
 */
export async function retryWithBackoff<T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  delayMs: number = 1000
): Promise<T> {
  let lastError: Error | undefined;

  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;
      if (i < maxRetries - 1) {
        const delay = delayMs * Math.pow(2, i);
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }
  }

  throw lastError;
}

/**
 * Sanitize string for SQL/NoSQL injection prevention
 */
export function sanitizeString(input: string): string {
  return input
    .replace(/[<>]/g, '') // Remove HTML tags
    .replace(/['";]/g, '') // Remove SQL special characters
    .trim();
}

/**
 * Generate unique ID (UUID v4 alternative)
 */
export function generateId(): string {
  return Date.now().toString(36) + Math.random().toString(36).substring(2);
}

/**
 * Check if CCP violation requires corrective action
 */
export function requiresCorrectiveAction(
  violationDurationMinutes: number,
  ccpType: string
): boolean {
  // FDA requirement: Corrective action if out-of-spec for > 4 minutes
  if (violationDurationMinutes > 4) {
    return true;
  }

  // Additional criteria based on CCP type
  if (ccpType === 'COOKING_TEMP' && violationDurationMinutes > 1) {
    return true; // Cooking temperature is more critical
  }

  return false;
}
