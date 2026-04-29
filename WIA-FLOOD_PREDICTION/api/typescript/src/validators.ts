/**
 * WIA-FLOOD_PREDICTION TypeScript SDK - Validators
 *
 * Data validation utilities for flood prediction API
 *
 * @version 1.0.0
 * @license MIT
 */

import { FloodPrediction, RiverGauge, GeoPoint, RiskLevel } from './types';

/**
 * ValidationError class for validation failures
 */
export class ValidationError extends Error {
  constructor(
    message: string,
    public field: string,
    public value: any,
    public constraint: string
  ) {
    super(message);
    this.name = 'ValidationError';
  }
}

/**
 * Validate geographic coordinates
 */
export function validateGeoPoint(point: GeoPoint, fieldName: string = 'location'): void {
  if (typeof point.lat !== 'number' || isNaN(point.lat)) {
    throw new ValidationError(
      `${fieldName}.lat must be a valid number`,
      `${fieldName}.lat`,
      point.lat,
      'type: number'
    );
  }

  if (point.lat < -90 || point.lat > 90) {
    throw new ValidationError(
      `${fieldName}.lat must be between -90 and 90`,
      `${fieldName}.lat`,
      point.lat,
      'range: -90 to 90'
    );
  }

  if (typeof point.lng !== 'number' || isNaN(point.lng)) {
    throw new ValidationError(
      `${fieldName}.lng must be a valid number`,
      `${fieldName}.lng`,
      point.lng,
      'type: number'
    );
  }

  if (point.lng < -180 || point.lng > 180) {
    throw new ValidationError(
      `${fieldName}.lng must be between -180 and 180`,
      `${fieldName}.lng`,
      point.lng,
      'range: -180 to 180'
    );
  }

  if (point.elevation_m !== undefined) {
    if (typeof point.elevation_m !== 'number' || isNaN(point.elevation_m)) {
      throw new ValidationError(
        `${fieldName}.elevation_m must be a valid number`,
        `${fieldName}.elevation_m`,
        point.elevation_m,
        'type: number'
      );
    }

    if (point.elevation_m < -500 || point.elevation_m > 9000) {
      throw new ValidationError(
        `${fieldName}.elevation_m must be between -500 and 9000 meters`,
        `${fieldName}.elevation_m`,
        point.elevation_m,
        'range: -500 to 9000'
      );
    }
  }
}

/**
 * Validate risk level enum
 */
export function validateRiskLevel(level: string, fieldName: string = 'risk_level'): void {
  const validLevels: RiskLevel[] = ['low', 'medium', 'high', 'extreme'];

  if (!validLevels.includes(level as RiskLevel)) {
    throw new ValidationError(
      `${fieldName} must be one of: ${validLevels.join(', ')}`,
      fieldName,
      level,
      `enum: ${validLevels.join('|')}`
    );
  }
}

/**
 * Validate probability value (0.0 - 1.0)
 */
export function validateProbability(value: number, fieldName: string = 'probability'): void {
  if (typeof value !== 'number' || isNaN(value)) {
    throw new ValidationError(
      `${fieldName} must be a valid number`,
      fieldName,
      value,
      'type: number'
    );
  }

  if (value < 0 || value > 1) {
    throw new ValidationError(
      `${fieldName} must be between 0.0 and 1.0`,
      fieldName,
      value,
      'range: 0.0 to 1.0'
    );
  }
}

/**
 * Validate ISO 8601 datetime string
 */
export function validateISODateTime(dateStr: string, fieldName: string = 'date'): void {
  if (typeof dateStr !== 'string') {
    throw new ValidationError(
      `${fieldName} must be a string`,
      fieldName,
      dateStr,
      'type: string'
    );
  }

  const date = new Date(dateStr);
  if (isNaN(date.getTime())) {
    throw new ValidationError(
      `${fieldName} must be a valid ISO 8601 datetime`,
      fieldName,
      dateStr,
      'format: ISO 8601 (e.g., 2026-01-11T14:30:00Z)'
    );
  }

  // Check if string is in ISO format (contains 'T' and optionally 'Z' or timezone)
  const isoRegex = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?(Z|[+-]\d{2}:\d{2})?$/;
  if (!isoRegex.test(dateStr)) {
    throw new ValidationError(
      `${fieldName} must be in ISO 8601 format`,
      fieldName,
      dateStr,
      'format: YYYY-MM-DDTHH:mm:ss.sssZ'
    );
  }
}

/**
 * Validate date range (start must be before end)
 */
export function validateDateRange(startDate: string, endDate: string): void {
  validateISODateTime(startDate, 'start_date');
  validateISODateTime(endDate, 'end_date');

  const start = new Date(startDate);
  const end = new Date(endDate);

  if (start >= end) {
    throw new ValidationError(
      'start_date must be before end_date',
      'start_date',
      startDate,
      `must be < ${endDate}`
    );
  }
}

/**
 * Validate radius (in kilometers)
 */
export function validateRadius(radius: number, maxRadius: number = 100): void {
  if (typeof radius !== 'number' || isNaN(radius)) {
    throw new ValidationError(
      'radius_km must be a valid number',
      'radius_km',
      radius,
      'type: number'
    );
  }

  if (radius <= 0) {
    throw new ValidationError(
      'radius_km must be greater than 0',
      'radius_km',
      radius,
      'min: 0 (exclusive)'
    );
  }

  if (radius > maxRadius) {
    throw new ValidationError(
      `radius_km must be less than or equal to ${maxRadius}`,
      'radius_km',
      radius,
      `max: ${maxRadius}`
    );
  }
}

/**
 * Validate flood prediction object
 */
export function validateFloodPrediction(prediction: FloodPrediction): void {
  // ID
  if (!prediction.id || typeof prediction.id !== 'string') {
    throw new ValidationError(
      'id must be a non-empty string',
      'id',
      prediction.id,
      'type: string, minLength: 1'
    );
  }

  // Location
  validateGeoPoint(prediction.location, 'location');

  // Dates
  validateISODateTime(prediction.forecast_date, 'forecast_date');
  validateISODateTime(prediction.issued_at, 'issued_at');

  // Risk level
  validateRiskLevel(prediction.risk_level, 'risk_level');

  // Probability & confidence
  validateProbability(prediction.probability, 'probability');
  validateProbability(prediction.confidence, 'confidence');

  // Physical predictions (non-negative)
  if (prediction.predicted_depth_meters < 0) {
    throw new ValidationError(
      'predicted_depth_meters must be non-negative',
      'predicted_depth_meters',
      prediction.predicted_depth_meters,
      'min: 0'
    );
  }

  if (prediction.predicted_velocity_ms < 0) {
    throw new ValidationError(
      'predicted_velocity_ms must be non-negative',
      'predicted_velocity_ms',
      prediction.predicted_velocity_ms,
      'min: 0'
    );
  }

  if (prediction.affected_area_km2 < 0) {
    throw new ValidationError(
      'affected_area_km2 must be non-negative',
      'affected_area_km2',
      prediction.affected_area_km2,
      'min: 0'
    );
  }

  // Peak time
  validateISODateTime(prediction.peak_time, 'peak_time');

  // Model version
  if (!prediction.model_version || typeof prediction.model_version !== 'string') {
    throw new ValidationError(
      'model_version must be a non-empty string',
      'model_version',
      prediction.model_version,
      'type: string, minLength: 1'
    );
  }

  // Data sources (array of strings)
  if (!Array.isArray(prediction.data_sources)) {
    throw new ValidationError(
      'data_sources must be an array',
      'data_sources',
      prediction.data_sources,
      'type: array'
    );
  }

  prediction.data_sources.forEach((source, index) => {
    if (typeof source !== 'string') {
      throw new ValidationError(
        `data_sources[${index}] must be a string`,
        `data_sources[${index}]`,
        source,
        'type: string'
      );
    }
  });

  // Uncertainty range
  if (prediction.uncertainty_range) {
    const { depth_min, depth_max } = prediction.uncertainty_range;

    if (typeof depth_min !== 'number' || depth_min < 0) {
      throw new ValidationError(
        'uncertainty_range.depth_min must be a non-negative number',
        'uncertainty_range.depth_min',
        depth_min,
        'type: number, min: 0'
      );
    }

    if (typeof depth_max !== 'number' || depth_max < 0) {
      throw new ValidationError(
        'uncertainty_range.depth_max must be a non-negative number',
        'uncertainty_range.depth_max',
        depth_max,
        'type: number, min: 0'
      );
    }

    if (depth_min > depth_max) {
      throw new ValidationError(
        'uncertainty_range.depth_min must be <= depth_max',
        'uncertainty_range.depth_min',
        depth_min,
        `must be <= ${depth_max}`
      );
    }
  }
}

/**
 * Validate river gauge object
 */
export function validateRiverGauge(gauge: RiverGauge): void {
  // ID (USGS site code)
  if (!gauge.id || typeof gauge.id !== 'string') {
    throw new ValidationError(
      'id must be a non-empty string (USGS site code)',
      'id',
      gauge.id,
      'type: string, minLength: 1'
    );
  }

  // Name
  if (!gauge.name || typeof gauge.name !== 'string') {
    throw new ValidationError(
      'name must be a non-empty string',
      'name',
      gauge.name,
      'type: string, minLength: 1'
    );
  }

  // Location
  validateGeoPoint(gauge.location, 'location');

  // Current level (can be negative for datum below sea level)
  if (typeof gauge.current_level !== 'number' || isNaN(gauge.current_level)) {
    throw new ValidationError(
      'current_level must be a valid number',
      'current_level',
      gauge.current_level,
      'type: number'
    );
  }

  // Forecast array
  if (!Array.isArray(gauge.forecast)) {
    throw new ValidationError(
      'forecast must be an array',
      'forecast',
      gauge.forecast,
      'type: array'
    );
  }

  gauge.forecast.forEach((item, index) => {
    validateISODateTime(item.timestamp, `forecast[${index}].timestamp`);

    if (typeof item.level !== 'number' || isNaN(item.level)) {
      throw new ValidationError(
        `forecast[${index}].level must be a valid number`,
        `forecast[${index}].level`,
        item.level,
        'type: number'
      );
    }

    if (typeof item.discharge !== 'number' || isNaN(item.discharge) || item.discharge < 0) {
      throw new ValidationError(
        `forecast[${index}].discharge must be a non-negative number`,
        `forecast[${index}].discharge`,
        item.discharge,
        'type: number, min: 0'
      );
    }

    validateProbability(item.confidence, `forecast[${index}].confidence`);
  });
}

/**
 * Validate API key format
 */
export function validateAPIKey(apiKey: string): void {
  if (typeof apiKey !== 'string') {
    throw new ValidationError(
      'API key must be a string',
      'apiKey',
      apiKey,
      'type: string'
    );
  }

  // Format: wia_[env]_[32-char random]
  const apiKeyRegex = /^wia_(live|test)_[a-z0-9]{32}$/;
  if (!apiKeyRegex.test(apiKey)) {
    throw new ValidationError(
      'API key format invalid',
      'apiKey',
      apiKey.substring(0, 12) + '...',
      'format: wia_[live|test]_[32-char alphanumeric]'
    );
  }
}

/**
 * Sanitize string input (remove dangerous characters)
 */
export function sanitizeString(input: string, maxLength: number = 1000): string {
  if (typeof input !== 'string') {
    return '';
  }

  // Remove null bytes, control characters, and limit length
  return input
    .replace(/\0/g, '')
    .replace(/[\x00-\x1F\x7F]/g, '')
    .substring(0, maxLength)
    .trim();
}

/**
 * Validate email format
 */
export function validateEmail(email: string): void {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

  if (!emailRegex.test(email)) {
    throw new ValidationError(
      'Invalid email format',
      'email',
      email,
      'format: user@example.com'
    );
  }

  if (email.length > 254) {
    throw new ValidationError(
      'Email too long (max 254 characters)',
      'email',
      email,
      'maxLength: 254'
    );
  }
}

/**
 * Validate phone number (E.164 format)
 */
export function validatePhoneNumber(phone: string): void {
  // E.164 format: +[country code][number] (max 15 digits)
  const phoneRegex = /^\+[1-9]\d{1,14}$/;

  if (!phoneRegex.test(phone)) {
    throw new ValidationError(
      'Invalid phone number format (must be E.164)',
      'phone',
      phone,
      'format: +12025551234'
    );
  }
}

/**
 * Validate UUID format
 */
export function validateUUID(uuid: string, fieldName: string = 'id'): void {
  const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;

  if (!uuidRegex.test(uuid)) {
    throw new ValidationError(
      `${fieldName} must be a valid UUID`,
      fieldName,
      uuid,
      'format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx'
    );
  }
}

/**
 * Validate pagination parameters
 */
export function validatePagination(page: number, limit: number): void {
  if (!Number.isInteger(page) || page < 1) {
    throw new ValidationError(
      'page must be a positive integer',
      'page',
      page,
      'type: integer, min: 1'
    );
  }

  if (!Number.isInteger(limit) || limit < 1 || limit > 1000) {
    throw new ValidationError(
      'limit must be an integer between 1 and 1000',
      'limit',
      limit,
      'type: integer, min: 1, max: 1000'
    );
  }
}
