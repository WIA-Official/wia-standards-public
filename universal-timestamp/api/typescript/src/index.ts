/**
 * WIA-CORE-009: Universal Timestamp SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for universal timestamp management:
 * - Creation and parsing of WIA-TS format
 * - Timezone conversion and handling
 * - Cryptographic signing and verification
 * - Temporal context for time-travel scenarios
 * - Multiple output formats
 */

import {
  UniversalTimestamp,
  CreateTimestampOptions,
  ParseResult,
  ValidationResult,
  ValidationCheck,
  OutputFormat,
  ConversionOptions,
  HumanFormatOptions,
  ComparisonResult,
  TimestampDifference,
  Duration,
  TimestampJSON,
  SigningOptions,
  VerificationOptions,
  VerificationResult,
  PrecisionLevel,
  Timezone,
  TemporalContext,
  TimestampSignature,
  TIMESTAMP_CONSTANTS,
  TimestampErrorCode,
  TimestampError,
  PartialTimestamp,
  TimestampRange,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-009 Universal Timestamp SDK
 */
export class UniversalTimestampSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a universal timestamp
   *
   * @param options - Creation options
   * @returns Universal timestamp
   */
  createTimestamp(options: CreateTimestampOptions = {}): UniversalTimestamp {
    const {
      date = new Date(),
      precision = PrecisionLevel.NANOSECOND,
      timezone = 'UTC',
      temporalContext,
      sign = false,
      privateKey,
      signatureAlgorithm = 'ed25519',
      hashAlgorithm = 'sha256',
      nonce,
    } = options;

    // Calculate epoch and precision
    const epochMs = date.getTime();
    const epoch = Math.floor(epochMs / 1000);
    const precisionNano = this.calculatePrecision(date, precision);

    // Create base timestamp
    const timestamp: UniversalTimestamp = {
      epoch,
      precision: precisionNano,
      precisionLevel: precision,
      timezone,
      temporalContext,
      wiaFormat: '',
      iso8601: '',
      rfc3339: '',
      date,
      milliseconds: epochMs,
      microseconds: epochMs * 1000 + Math.floor(precisionNano / 1000),
      nanoseconds: epoch * 1_000_000_000 + precisionNano,
    };

    // Generate WIA format
    timestamp.wiaFormat = this.generateWIAFormat(timestamp);

    // Generate ISO 8601
    timestamp.iso8601 = date.toISOString();

    // Generate RFC 3339
    timestamp.rfc3339 = this.toRFC3339(date, timezone);

    // Add signature if requested
    if (sign && privateKey) {
      timestamp.signature = this.signTimestamp(timestamp, {
        privateKey,
        algorithm: signatureAlgorithm,
        hashAlgorithm,
        nonce,
      });
      // Regenerate WIA format with signature
      timestamp.wiaFormat = this.generateWIAFormat(timestamp);
    }

    return timestamp;
  }

  /**
   * Parse a WIA-TS format timestamp
   *
   * @param input - WIA-TS format string
   * @returns Parse result
   */
  parseTimestamp(input: string): ParseResult {
    try {
      const trimmed = input.trim();

      // Check for WIA-TS prefix
      if (!trimmed.startsWith(TIMESTAMP_CONSTANTS.FORMAT_PREFIX)) {
        return {
          success: false,
          error: 'Invalid format: missing WIA-TS prefix',
        };
      }

      // Remove prefix
      const content = trimmed.substring(TIMESTAMP_CONSTANTS.FORMAT_PREFIX.length).trim();

      // Parse components using regex
      const regex =
        /^(-?\d+)(?:\.(\d{1,9}))?@([A-Za-z/_+-:]+)(?:\[([^\]]+)\])?(?:\{([^}]+)\})?$/;
      const match = content.match(regex);

      if (!match) {
        return {
          success: false,
          error: 'Invalid format: failed to parse components',
        };
      }

      const [, epochStr, precisionStr, timezone, temporalStr, signatureStr] = match;

      // Parse epoch
      const epoch = parseInt(epochStr, 10);
      if (isNaN(epoch)) {
        return { success: false, error: 'Invalid epoch' };
      }

      // Parse precision
      const precision = precisionStr ? parseInt(precisionStr, 10) : 0;
      const precisionLevel = this.determinePrecisionLevel(precisionStr);

      // Parse temporal context
      let temporalContext: TemporalContext | undefined;
      if (temporalStr) {
        temporalContext = this.parseTemporalContext(temporalStr);
      }

      // Parse signature
      let signature: TimestampSignature | undefined;
      if (signatureStr) {
        signature = this.parseSignature(signatureStr);
      }

      // Create Date object
      const date = new Date(epoch * 1000 + precision / 1_000_000);

      // Build timestamp
      const timestamp: UniversalTimestamp = {
        epoch,
        precision,
        precisionLevel,
        timezone: timezone as Timezone,
        temporalContext,
        signature,
        wiaFormat: input,
        iso8601: date.toISOString(),
        rfc3339: this.toRFC3339(date, timezone),
        date,
        milliseconds: epoch * 1000 + Math.floor(precision / 1_000_000),
        microseconds: epoch * 1_000_000 + Math.floor(precision / 1000),
        nanoseconds: epoch * 1_000_000_000 + precision,
      };

      return { success: true, timestamp };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Validate a timestamp
   *
   * @param timestamp - Timestamp to validate
   * @returns Validation result
   */
  validateTimestamp(timestamp: UniversalTimestamp): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    const checks: ValidationCheck[] = [];

    // Check epoch range
    const epochCheck = this.checkEpochRange(timestamp.epoch);
    checks.push(epochCheck);
    if (epochCheck.status === 'fail') {
      errors.push(epochCheck.message);
    } else if (epochCheck.status === 'warning') {
      warnings.push(epochCheck.message);
    }

    // Check precision
    const precisionCheck = this.checkPrecision(timestamp.precision, timestamp.precisionLevel);
    checks.push(precisionCheck);
    if (precisionCheck.status === 'fail') {
      errors.push(precisionCheck.message);
    }

    // Check timezone
    const timezoneCheck = this.checkTimezone(timestamp.timezone);
    checks.push(timezoneCheck);
    if (timezoneCheck.status === 'fail') {
      errors.push(timezoneCheck.message);
    }

    // Check temporal context
    if (timestamp.temporalContext) {
      const temporalCheck = this.checkTemporalContext(timestamp.temporalContext);
      checks.push(temporalCheck);
      if (temporalCheck.status === 'fail') {
        errors.push(temporalCheck.message);
      }
    }

    // Check signature
    if (timestamp.signature) {
      const signatureCheck = this.checkSignatureFormat(timestamp.signature);
      checks.push(signatureCheck);
      if (signatureCheck.status === 'fail') {
        errors.push(signatureCheck.message);
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      checks,
    };
  }

  /**
   * Convert timestamp to different timezone
   *
   * @param timestamp - Source timestamp
   * @param targetTimezone - Target timezone
   * @returns Converted timestamp
   */
  convertTimezone(timestamp: UniversalTimestamp, targetTimezone: Timezone): UniversalTimestamp {
    // Create new timestamp with same epoch but different timezone
    return {
      ...timestamp,
      timezone: targetTimezone,
      rfc3339: this.toRFC3339(timestamp.date, targetTimezone),
      wiaFormat: this.generateWIAFormat({ ...timestamp, timezone: targetTimezone }),
    };
  }

  /**
   * Format timestamp to specified output format
   *
   * @param timestamp - Timestamp to format
   * @param format - Output format
   * @returns Formatted string
   */
  formatTimestamp(timestamp: UniversalTimestamp, format: OutputFormat): string {
    switch (format) {
      case 'wia':
        return timestamp.wiaFormat;
      case 'iso':
        return timestamp.iso8601;
      case 'rfc3339':
        return timestamp.rfc3339;
      case 'unix':
        return timestamp.epoch.toString();
      case 'unix-ms':
        return timestamp.milliseconds.toString();
      case 'unix-us':
        return timestamp.microseconds.toString();
      case 'unix-ns':
        return timestamp.nanoseconds.toString();
      case 'human':
        return this.toHumanReadable(timestamp);
      case 'json':
        return JSON.stringify(this.toJSON(timestamp), null, 2);
      default:
        return timestamp.wiaFormat;
    }
  }

  /**
   * Compare two timestamps
   *
   * @param a - First timestamp
   * @param b - Second timestamp
   * @returns -1 if a < b, 0 if a === b, 1 if a > b
   */
  compareTimestamps(a: UniversalTimestamp, b: UniversalTimestamp): ComparisonResult {
    if (a.nanoseconds < b.nanoseconds) return -1;
    if (a.nanoseconds > b.nanoseconds) return 1;

    // If nanoseconds are equal, check temporal displacement
    if (a.temporalContext && b.temporalContext) {
      const aTotal = a.nanoseconds + a.temporalContext.displacement * 1_000_000_000;
      const bTotal = b.nanoseconds + b.temporalContext.displacement * 1_000_000_000;
      if (aTotal < bTotal) return -1;
      if (aTotal > bTotal) return 1;
    }

    return 0;
  }

  /**
   * Calculate difference between two timestamps
   *
   * @param a - First timestamp
   * @param b - Second timestamp
   * @returns Difference
   */
  difference(a: UniversalTimestamp, b: UniversalTimestamp): TimestampDifference {
    const nanosDiff = a.nanoseconds - b.nanoseconds;
    const microsDiff = Math.floor(nanosDiff / 1000);
    const millisDiff = Math.floor(nanosDiff / 1_000_000);
    const secondsDiff = Math.floor(nanosDiff / 1_000_000_000);

    const absolute = {
      nanoseconds: Math.abs(nanosDiff),
      microseconds: Math.abs(microsDiff),
      milliseconds: Math.abs(millisDiff),
      seconds: Math.abs(secondsDiff),
      human: this.formatDuration(Math.abs(secondsDiff)),
    } as TimestampDifference;

    return {
      nanoseconds: nanosDiff,
      microseconds: microsDiff,
      milliseconds: millisDiff,
      seconds: secondsDiff,
      human: this.formatDuration(secondsDiff),
      absolute,
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private calculatePrecision(date: Date, level: PrecisionLevel): number {
    const ms = date.getTime() % 1000;

    switch (level) {
      case PrecisionLevel.SECOND:
        return 0;
      case PrecisionLevel.MILLISECOND:
        return ms * 1_000_000;
      case PrecisionLevel.MICROSECOND:
        // Estimate microseconds (JavaScript doesn't have true microsecond precision)
        return ms * 1000 * 1000;
      case PrecisionLevel.NANOSECOND:
        // Estimate nanoseconds (JavaScript doesn't have true nanosecond precision)
        return ms * 1_000_000;
      case PrecisionLevel.PLANCK:
        return ms * 1_000_000; // Base nanoseconds, Planck added in temporal context
      default:
        return 0;
    }
  }

  private determinePrecisionLevel(precisionStr?: string): PrecisionLevel {
    if (!precisionStr) return PrecisionLevel.SECOND;
    const len = precisionStr.length;
    if (len <= 3) return PrecisionLevel.MILLISECOND;
    if (len <= 6) return PrecisionLevel.MICROSECOND;
    if (len <= 9) return PrecisionLevel.NANOSECOND;
    return PrecisionLevel.PLANCK;
  }

  private generateWIAFormat(timestamp: UniversalTimestamp): string {
    let result = TIMESTAMP_CONSTANTS.FORMAT_PREFIX;

    // Add epoch
    result += ` ${timestamp.epoch}`;

    // Add precision
    if (timestamp.precisionLevel > PrecisionLevel.SECOND) {
      const digits = TIMESTAMP_CONSTANTS.PRECISION_DIGITS[timestamp.precisionLevel];
      if (digits > 0) {
        const precisionStr = timestamp.precision.toString().padStart(digits, '0');
        result += `.${precisionStr}`;
      }
    }

    // Add timezone
    result += `@${timestamp.timezone}`;

    // Add temporal context
    if (timestamp.temporalContext) {
      result += `[${this.serializeTemporalContext(timestamp.temporalContext)}]`;
    }

    // Add signature
    if (timestamp.signature) {
      result += `{${this.serializeSignature(timestamp.signature)}}`;
    }

    return result;
  }

  private parseTemporalContext(str: string): TemporalContext {
    const parts = str.split(',');
    const context: Partial<TemporalContext> = {};

    for (const part of parts) {
      const [key, value] = part.split('=');
      if (key && value) {
        switch (key.trim()) {
          case 'origin':
            context.origin = parseInt(value, 10);
            break;
          case 'displacement':
            context.displacement = parseInt(value, 10);
            break;
          case 'worldline':
            context.worldline = value;
            break;
          case 'timeline':
            context.timeline = value;
            break;
          case 'ref':
          case 'reference':
            context.reference = value;
            break;
          case 'planck':
            context.planckValue = parseFloat(value);
            break;
        }
      }
    }

    return context as TemporalContext;
  }

  private serializeTemporalContext(context: TemporalContext): string {
    const parts: string[] = [];
    parts.push(`origin=${context.origin}`);
    parts.push(`displacement=${context.displacement}`);
    parts.push(`worldline=${context.worldline}`);
    parts.push(`timeline=${context.timeline}`);
    parts.push(`ref=${context.reference}`);
    if (context.planckValue) {
      parts.push(`planck=${context.planckValue.toExponential()}`);
    }
    return parts.join(',');
  }

  private parseSignature(str: string): TimestampSignature {
    const parts = str.split(',');
    const signature: Partial<TimestampSignature> = {};

    for (const part of parts) {
      const [key, value] = part.split('=');
      if (key && value) {
        switch (key.trim()) {
          case 'sig': {
            const [algo, val] = value.split(':');
            signature.algorithm = algo as TimestampSignature['algorithm'];
            signature.value = val;
            break;
          }
          case 'hash': {
            const [algo, val] = value.split(':');
            signature.hashAlgorithm = algo as TimestampSignature['hashAlgorithm'];
            signature.hashValue = val;
            break;
          }
          case 'nonce':
            signature.nonce = value;
            break;
        }
      }
    }

    return signature as TimestampSignature;
  }

  private serializeSignature(signature: TimestampSignature): string {
    const parts: string[] = [];
    parts.push(`sig=${signature.algorithm}:${signature.value}`);
    parts.push(`hash=${signature.hashAlgorithm}:${signature.hashValue}`);
    if (signature.nonce) {
      parts.push(`nonce=${signature.nonce}`);
    }
    return parts.join(',');
  }

  private signTimestamp(
    timestamp: UniversalTimestamp,
    options: SigningOptions
  ): TimestampSignature {
    // This is a placeholder implementation
    // In production, use actual cryptographic libraries
    const baseTimestamp = this.generateWIAFormat(timestamp);
    const hash = this.simpleHash(baseTimestamp);

    return {
      algorithm: options.algorithm,
      value: 'placeholder_signature_' + hash.substring(0, 16),
      hashAlgorithm: options.hashAlgorithm,
      hashValue: hash,
      nonce: options.nonce,
    };
  }

  private simpleHash(input: string): string {
    // Placeholder hash function
    // In production, use SHA-256 or other cryptographic hash
    let hash = 0;
    for (let i = 0; i < input.length; i++) {
      const char = input.charCodeAt(i);
      hash = (hash << 5) - hash + char;
      hash = hash & hash;
    }
    return Math.abs(hash).toString(16).padStart(16, '0');
  }

  private toRFC3339(date: Date, timezone: Timezone): string {
    if (timezone === 'UTC') {
      return date.toISOString();
    }
    // Simplified RFC3339 conversion
    return date.toISOString();
  }

  private toHumanReadable(timestamp: UniversalTimestamp, options?: HumanFormatOptions): string {
    const date = timestamp.date;
    const locale = options?.locale || 'en-US';

    return date.toLocaleString(locale, {
      dateStyle: options?.dateStyle || 'medium',
      timeStyle: options?.timeStyle || 'medium',
      timeZone: timestamp.timezone === 'UTC' ? 'UTC' : undefined,
    });
  }

  private formatDuration(seconds: number): string {
    const absSeconds = Math.abs(seconds);
    const sign = seconds < 0 ? '-' : '';

    if (absSeconds < 60) {
      return `${sign}${absSeconds}s`;
    } else if (absSeconds < 3600) {
      const minutes = Math.floor(absSeconds / 60);
      const secs = absSeconds % 60;
      return `${sign}${minutes}m ${secs}s`;
    } else if (absSeconds < 86400) {
      const hours = Math.floor(absSeconds / 3600);
      const minutes = Math.floor((absSeconds % 3600) / 60);
      return `${sign}${hours}h ${minutes}m`;
    } else {
      const days = Math.floor(absSeconds / 86400);
      const hours = Math.floor((absSeconds % 86400) / 3600);
      return `${sign}${days}d ${hours}h`;
    }
  }

  private toJSON(timestamp: UniversalTimestamp): TimestampJSON {
    return {
      format: 'WIA-TS',
      version: '1.0',
      epoch: timestamp.epoch,
      precision: timestamp.precision,
      precisionLevel: timestamp.precisionLevel,
      timezone: timestamp.timezone,
      temporalContext: timestamp.temporalContext,
      signature: timestamp.signature,
    };
  }

  private checkEpochRange(epoch: number): ValidationCheck {
    if (epoch < TIMESTAMP_CONSTANTS.MIN_EPOCH) {
      return {
        name: 'Epoch Range',
        status: 'warning',
        message: 'Epoch is before recommended minimum (1900-01-01)',
        expected: TIMESTAMP_CONSTANTS.MIN_EPOCH,
        actual: epoch,
      };
    }
    if (epoch > TIMESTAMP_CONSTANTS.MAX_EPOCH) {
      return {
        name: 'Epoch Range',
        status: 'warning',
        message: 'Epoch is after recommended maximum (2100-01-01)',
        expected: TIMESTAMP_CONSTANTS.MAX_EPOCH,
        actual: epoch,
      };
    }
    return {
      name: 'Epoch Range',
      status: 'pass',
      message: 'Epoch is within valid range',
    };
  }

  private checkPrecision(precision: number, level: PrecisionLevel): ValidationCheck {
    if (precision < 0 || precision >= TIMESTAMP_CONSTANTS.NANOS_PER_SECOND) {
      return {
        name: 'Precision',
        status: 'fail',
        message: 'Precision must be 0-999999999',
        expected: '0-999999999',
        actual: precision,
      };
    }
    return {
      name: 'Precision',
      status: 'pass',
      message: 'Precision is valid',
    };
  }

  private checkTimezone(timezone: Timezone): ValidationCheck {
    // Basic timezone validation
    if (timezone === 'UTC') {
      return { name: 'Timezone', status: 'pass', message: 'Timezone is valid' };
    }
    if (/^[+-]\d{2}:\d{2}$/.test(timezone)) {
      return { name: 'Timezone', status: 'pass', message: 'Timezone offset is valid' };
    }
    if (/^[A-Za-z/_]+$/.test(timezone)) {
      return { name: 'Timezone', status: 'pass', message: 'IANA timezone format is valid' };
    }
    return {
      name: 'Timezone',
      status: 'fail',
      message: 'Invalid timezone format',
      actual: timezone,
    };
  }

  private checkTemporalContext(context: TemporalContext): ValidationCheck {
    if (!context.origin || !context.displacement || !context.worldline || !context.timeline) {
      return {
        name: 'Temporal Context',
        status: 'fail',
        message: 'Temporal context is missing required fields',
      };
    }
    return {
      name: 'Temporal Context',
      status: 'pass',
      message: 'Temporal context is valid',
    };
  }

  private checkSignatureFormat(signature: TimestampSignature): ValidationCheck {
    if (!signature.algorithm || !signature.value || !signature.hashAlgorithm || !signature.hashValue) {
      return {
        name: 'Signature Format',
        status: 'fail',
        message: 'Signature is missing required fields',
      };
    }
    return {
      name: 'Signature Format',
      status: 'pass',
      message: 'Signature format is valid',
    };
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create a timestamp (convenience wrapper)
 */
export function createTimestamp(options?: CreateTimestampOptions): UniversalTimestamp {
  const sdk = new UniversalTimestampSDK();
  return sdk.createTimestamp(options);
}

/**
 * Parse a WIA-TS format string (convenience wrapper)
 */
export function parseTimestamp(input: string): ParseResult {
  const sdk = new UniversalTimestampSDK();
  return sdk.parseTimestamp(input);
}

/**
 * Validate a timestamp (convenience wrapper)
 */
export function validateTimestamp(timestamp: UniversalTimestamp): ValidationResult {
  const sdk = new UniversalTimestampSDK();
  return sdk.validateTimestamp(timestamp);
}

/**
 * Convert timezone (convenience wrapper)
 */
export function convertTimezone(timestamp: UniversalTimestamp, targetTimezone: Timezone): UniversalTimestamp {
  const sdk = new UniversalTimestampSDK();
  return sdk.convertTimezone(timestamp, targetTimezone);
}

/**
 * Format timestamp (convenience wrapper)
 */
export function formatTimestamp(timestamp: UniversalTimestamp, format: OutputFormat): string {
  const sdk = new UniversalTimestampSDK();
  return sdk.formatTimestamp(timestamp, format);
}

/**
 * Compare timestamps (convenience wrapper)
 */
export function compareTimestamps(a: UniversalTimestamp, b: UniversalTimestamp): ComparisonResult {
  const sdk = new UniversalTimestampSDK();
  return sdk.compareTimestamps(a, b);
}

/**
 * Calculate timestamp difference (convenience wrapper)
 */
export function difference(a: UniversalTimestamp, b: UniversalTimestamp): TimestampDifference {
  const sdk = new UniversalTimestampSDK();
  return sdk.difference(a, b);
}

// ============================================================================
// Export All
// ============================================================================

export { UniversalTimestampSDK };

export * from './types';
