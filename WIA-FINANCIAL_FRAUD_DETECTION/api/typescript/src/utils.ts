/**
 * WIA-FINANCIAL_FRAUD_DETECTION - Utility Functions
 *
 * @version 1.0.0
 * @standard WIA-FINANCIAL_FRAUD_DETECTION
 * @organization WIA (World Certification Industry Association)
 */

import { Decision, RiskLevel, FraudAssessment } from './types';

/**
 * Calculate risk level from risk score
 *
 * @param riskScore - Risk score (0-1)
 * @returns Risk level category
 *
 * @example
 * ```typescript
 * const riskLevel = getRiskLevel(0.87); // RiskLevel.HIGH
 * ```
 */
export function getRiskLevel(riskScore: number): RiskLevel {
  if (riskScore >= 0.90) {
    return RiskLevel.CRITICAL;
  } else if (riskScore >= 0.70) {
    return RiskLevel.HIGH;
  } else if (riskScore >= 0.50) {
    return RiskLevel.MEDIUM;
  } else if (riskScore >= 0.30) {
    return RiskLevel.LOW;
  } else {
    return RiskLevel.VERY_LOW;
  }
}

/**
 * Calculate decision from risk score
 *
 * @param riskScore - Risk score (0-1)
 * @returns Decision
 *
 * @example
 * ```typescript
 * const decision = getDecision(0.95); // Decision.BLOCK
 * ```
 */
export function getDecision(riskScore: number): Decision {
  if (riskScore >= 0.90) {
    return Decision.BLOCK;
  } else if (riskScore >= 0.70) {
    return Decision.REVIEW;
  } else if (riskScore >= 0.50) {
    return Decision.CHALLENGE;
  } else {
    return Decision.APPROVE;
  }
}

/**
 * Format currency amount
 *
 * @param amount - Amount
 * @param currency - ISO 4217 currency code
 * @returns Formatted currency string
 *
 * @example
 * ```typescript
 * formatCurrency(149.99, 'USD'); // "$149.99"
 * formatCurrency(1000, 'EUR'); // "€1,000.00"
 * ```
 */
export function formatCurrency(amount: number, currency: string): string {
  try {
    return new Intl.NumberFormat('en-US', {
      style: 'currency',
      currency,
    }).format(amount);
  } catch (error) {
    // Fallback if currency code is invalid
    return `${amount.toFixed(2)} ${currency}`;
  }
}

/**
 * Format timestamp to human-readable string
 *
 * @param timestamp - ISO 8601 timestamp
 * @returns Formatted timestamp
 *
 * @example
 * ```typescript
 * formatTimestamp('2026-01-11T10:30:00Z'); // "Jan 11, 2026, 10:30 AM"
 * ```
 */
export function formatTimestamp(timestamp: string): string {
  try {
    const date = new Date(timestamp);
    return date.toLocaleString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
    });
  } catch (error) {
    return timestamp;
  }
}

/**
 * Calculate distance between two coordinates (Haversine formula)
 *
 * @param lat1 - Latitude of point 1
 * @param lon1 - Longitude of point 1
 * @param lat2 - Latitude of point 2
 * @param lon2 - Longitude of point 2
 * @returns Distance in kilometers
 *
 * @example
 * ```typescript
 * const distance = calculateDistance(37.7749, -122.4194, 34.0522, -118.2437);
 * console.log(distance); // ~559 km (San Francisco to Los Angeles)
 * ```
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

/**
 * Convert degrees to radians
 */
function toRadians(degrees: number): number {
  return degrees * (Math.PI / 180);
}

/**
 * Check if travel between two locations is impossible
 *
 * @param lat1 - Latitude of location 1
 * @param lon1 - Longitude of location 1
 * @param timestamp1 - Timestamp at location 1 (ISO 8601)
 * @param lat2 - Latitude of location 2
 * @param lon2 - Longitude of location 2
 * @param timestamp2 - Timestamp at location 2 (ISO 8601)
 * @returns True if travel is impossible (assuming max speed of 1000 km/h)
 *
 * @example
 * ```typescript
 * const impossible = isImpossibleTravel(
 *   37.7749, -122.4194, '2026-01-11T10:00:00Z', // San Francisco
 *   34.0522, -118.2437, '2026-01-11T10:30:00Z'  // Los Angeles (30 min later)
 * );
 * console.log(impossible); // true (559 km in 30 min = 1118 km/h, impossible)
 * ```
 */
export function isImpossibleTravel(
  lat1: number,
  lon1: number,
  timestamp1: string,
  lat2: number,
  lon2: number,
  timestamp2: string
): boolean {
  const distance = calculateDistance(lat1, lon1, lat2, lon2);
  const time1 = new Date(timestamp1).getTime();
  const time2 = new Date(timestamp2).getTime();
  const timeDiffHours = Math.abs(time2 - time1) / (1000 * 60 * 60);

  // Maximum realistic speed: 1000 km/h (considering fastest commercial flights)
  const maxSpeed = 1000;
  const requiredSpeed = distance / timeDiffHours;

  return requiredSpeed > maxSpeed;
}

/**
 * Mask card number (show first 6 and last 4 digits)
 *
 * @param cardBin - First 6 digits
 * @param cardLast4 - Last 4 digits
 * @returns Masked card number
 *
 * @example
 * ```typescript
 * maskCardNumber('424242', '4242'); // "424242******4242"
 * ```
 */
export function maskCardNumber(cardBin: string, cardLast4: string): string {
  const maskedMiddle = '*'.repeat(6); // Assuming 16-digit card
  return `${cardBin}${maskedMiddle}${cardLast4}`;
}

/**
 * Mask email (show first 2 characters and domain)
 *
 * @param email - Email address
 * @returns Masked email
 *
 * @example
 * ```typescript
 * maskEmail('john.doe@example.com'); // "jo****@example.com"
 * ```
 */
export function maskEmail(email: string): string {
  const [local, domain] = email.split('@');
  if (!domain) {
    return email; // Invalid email
  }
  const maskedLocal = local.length > 2 ? local.substring(0, 2) + '****' : '****';
  return `${maskedLocal}@${domain}`;
}

/**
 * Mask phone number (show last 4 digits)
 *
 * @param phone - Phone number (E.164 format)
 * @returns Masked phone number
 *
 * @example
 * ```typescript
 * maskPhone('+15551234567'); // "+1***1234567" -> Wait, that's not right
 * // Actually:
 * maskPhone('+15551234567'); // "****4567"
 * ```
 */
export function maskPhone(phone: string): string {
  if (phone.length <= 4) {
    return phone;
  }
  return '*'.repeat(phone.length - 4) + phone.substring(phone.length - 4);
}

/**
 * Generate idempotency key (UUID v4)
 *
 * @returns UUID v4 string
 *
 * @example
 * ```typescript
 * const key = generateIdempotencyKey(); // "550e8400-e29b-41d4-a716-446655440000"
 * ```
 */
export function generateIdempotencyKey(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Sleep for specified milliseconds
 *
 * @param ms - Milliseconds to sleep
 * @returns Promise that resolves after delay
 *
 * @example
 * ```typescript
 * await sleep(1000); // Wait 1 second
 * ```
 */
export function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

/**
 * Retry function with exponential backoff
 *
 * @param fn - Function to retry
 * @param maxRetries - Maximum retry attempts
 * @param initialDelay - Initial delay in milliseconds
 * @returns Result of function
 *
 * @example
 * ```typescript
 * const result = await retryWithBackoff(
 *   async () => await apiCall(),
 *   3,
 *   1000
 * );
 * ```
 */
export async function retryWithBackoff<T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  initialDelay: number = 1000
): Promise<T> {
  let lastError: Error | undefined;

  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;
      if (i < maxRetries - 1) {
        const delay = initialDelay * Math.pow(2, i);
        await sleep(delay);
      }
    }
  }

  throw lastError;
}

/**
 * Truncate string with ellipsis
 *
 * @param str - String to truncate
 * @param maxLength - Maximum length
 * @returns Truncated string
 *
 * @example
 * ```typescript
 * truncate('This is a very long string', 10); // "This is a..."
 * ```
 */
export function truncate(str: string, maxLength: number): string {
  if (str.length <= maxLength) {
    return str;
  }
  return str.substring(0, maxLength - 3) + '...';
}

/**
 * Calculate percentage
 *
 * @param value - Value
 * @param total - Total
 * @returns Percentage (0-100)
 *
 * @example
 * ```typescript
 * percentage(25, 100); // 25
 * percentage(1, 3); // 33.33
 * ```
 */
export function percentage(value: number, total: number): number {
  if (total === 0) {
    return 0;
  }
  return (value / total) * 100;
}

/**
 * Calculate fraud rate
 *
 * @param fraudCount - Number of fraud cases
 * @param totalTransactions - Total transactions
 * @returns Fraud rate (0-1)
 *
 * @example
 * ```typescript
 * fraudRate(50, 10000); // 0.005 (0.5%)
 * ```
 */
export function fraudRate(fraudCount: number, totalTransactions: number): number {
  if (totalTransactions === 0) {
    return 0;
  }
  return fraudCount / totalTransactions;
}

/**
 * Format fraud assessment as human-readable summary
 *
 * @param assessment - Fraud assessment
 * @returns Human-readable summary
 *
 * @example
 * ```typescript
 * const summary = formatFraudAssessment(assessment);
 * console.log(summary);
 * // Output:
 * // Transaction: txn_123
 * // Decision: APPROVE (Risk Score: 0.23)
 * // Risk Level: LOW
 * // Top Reasons:
 * // - Customer has 365-day account history
 * // - Device previously seen with this customer
 * ```
 */
export function formatFraudAssessment(assessment: FraudAssessment): string {
  let summary = `Transaction: ${assessment.transaction_id}\n`;
  summary += `Decision: ${assessment.decision.toUpperCase()} (Risk Score: ${assessment.risk_score.toFixed(2)})\n`;
  summary += `Risk Level: ${assessment.risk_level.toUpperCase()}\n`;

  if (assessment.decision_reasons.length > 0) {
    summary += `\nTop Reasons:\n`;
    assessment.decision_reasons.slice(0, 3).forEach((reason) => {
      summary += `- ${reason}\n`;
    });
  }

  if (assessment.feature_importance.length > 0) {
    summary += `\nTop Features:\n`;
    assessment.feature_importance.slice(0, 3).forEach((feature) => {
      summary += `- ${feature.feature}: ${feature.importance.toFixed(2)} (${feature.contribution})\n`;
    });
  }

  summary += `\nAnalyzed at: ${formatTimestamp(assessment.analyzed_at)}`;
  summary += `\nModel version: ${assessment.model_version}`;

  return summary;
}

/**
 * Deep clone object (using JSON serialization)
 *
 * @param obj - Object to clone
 * @returns Cloned object
 *
 * @example
 * ```typescript
 * const clone = deepClone(originalObject);
 * ```
 */
export function deepClone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj));
}

/**
 * Check if object is empty
 *
 * @param obj - Object to check
 * @returns True if object is empty
 *
 * @example
 * ```typescript
 * isEmpty({}); // true
 * isEmpty({ key: 'value' }); // false
 * ```
 */
export function isEmpty(obj: Record<string, any>): boolean {
  return Object.keys(obj).length === 0;
}

/**
 * Remove undefined values from object
 *
 * @param obj - Object to clean
 * @returns Cleaned object
 *
 * @example
 * ```typescript
 * cleanObject({ a: 1, b: undefined, c: null }); // { a: 1, c: null }
 * ```
 */
export function cleanObject<T extends Record<string, any>>(obj: T): Partial<T> {
  const cleaned: any = {};
  for (const key in obj) {
    if (obj[key] !== undefined) {
      cleaned[key] = obj[key];
    }
  }
  return cleaned;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  getRiskLevel,
  getDecision,
  formatCurrency,
  formatTimestamp,
  calculateDistance,
  isImpossibleTravel,
  maskCardNumber,
  maskEmail,
  maskPhone,
  generateIdempotencyKey,
  sleep,
  retryWithBackoff,
  truncate,
  percentage,
  fraudRate,
  formatFraudAssessment,
  deepClone,
  isEmpty,
  cleanObject,
};
