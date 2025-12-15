/**
 * WIA AAC Signal Validator
 * Validates signals against JSON Schema
 */

import Ajv, { ValidateFunction } from 'ajv';
import addFormats from 'ajv-formats';
import { WiaAacSignal, SensorType } from '../types';

// Inline base schema (simplified for runtime)
const BASE_SCHEMA = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  type: 'object',
  required: ['version', 'type', 'timestamp', 'device', 'data'],
  properties: {
    version: { type: 'string', pattern: '^\\d+\\.\\d+\\.\\d+$' },
    type: {
      type: 'string',
      enum: ['eye_tracker', 'switch', 'muscle_sensor', 'brain_interface', 'breath', 'head_movement', 'custom']
    },
    timestamp: {
      type: 'object',
      required: ['unix_ms'],
      properties: {
        unix_ms: { type: 'number', minimum: 0 },
        iso8601: { type: 'string' }
      }
    },
    sequence: { type: 'integer', minimum: 0 },
    device: {
      type: 'object',
      required: ['manufacturer', 'model'],
      properties: {
        manufacturer: { type: 'string', minLength: 1 },
        model: { type: 'string', minLength: 1 },
        firmware: { type: 'string' },
        serial: { type: 'string' }
      }
    },
    data: { type: 'object' },
    meta: {
      type: 'object',
      properties: {
        confidence: { type: 'number', minimum: 0, maximum: 1 },
        validity: { type: 'boolean' },
        raw: { type: 'object' }
      }
    }
  }
};

export interface ValidationResult {
  valid: boolean;
  errors: string[];
}

export class SignalValidator {
  private ajv: Ajv;
  private baseValidator: ValidateFunction;

  constructor() {
    this.ajv = new Ajv({ allErrors: true, strict: false });
    addFormats(this.ajv);
    this.baseValidator = this.ajv.compile(BASE_SCHEMA);
  }

  /**
   * Validate a signal
   */
  validate(signal: unknown): ValidationResult {
    const valid = this.baseValidator(signal);

    if (!valid) {
      return {
        valid: false,
        errors: this.baseValidator.errors?.map(err =>
          `${err.instancePath || '/'}: ${err.message}`
        ) ?? ['Unknown validation error']
      };
    }

    return { valid: true, errors: [] };
  }

  /**
   * Check if signal is valid (throws on invalid)
   */
  assertValid(signal: unknown): asserts signal is WiaAacSignal {
    const result = this.validate(signal);
    if (!result.valid) {
      throw new Error(`Invalid signal: ${result.errors.join(', ')}`);
    }
  }

  /**
   * Type guard for WiaAacSignal
   */
  isValidSignal(signal: unknown): signal is WiaAacSignal {
    return this.validate(signal).valid;
  }
}

// Singleton instance
let validatorInstance: SignalValidator | null = null;

export function getSignalValidator(): SignalValidator {
  if (!validatorInstance) {
    validatorInstance = new SignalValidator();
  }
  return validatorInstance;
}
