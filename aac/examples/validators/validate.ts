/**
 * WIA AAC Signal Format Validator
 *
 * TypeScript implementation for validating WIA AAC Signal messages
 * against JSON Schema.
 *
 * Usage:
 *   ts-node validate.ts <file1.json> [file2.json] ...
 *   npx ts-node validate.ts ../sample-data/*.json
 *
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

import Ajv, { ValidateFunction, ErrorObject } from 'ajv';
import addFormats from 'ajv-formats';
import * as fs from 'fs';
import * as path from 'path';

// Schema paths
const SCHEMA_DIR = path.join(__dirname, '../../spec/schemas');

// Sensor types
type SensorType =
  | 'eye_tracker'
  | 'switch'
  | 'muscle_sensor'
  | 'brain_interface'
  | 'breath'
  | 'head_movement'
  | 'custom';

// Schema file mapping
const SENSOR_SCHEMA_MAP: Record<SensorType, string> = {
  'eye_tracker': 'eye-tracker.schema.json',
  'switch': 'switch.schema.json',
  'muscle_sensor': 'muscle-sensor.schema.json',
  'brain_interface': 'brain-interface.schema.json',
  'breath': 'breath.schema.json',
  'head_movement': 'head-movement.schema.json',
  'custom': ''
};

// Base message interface
interface WiaAacMessage {
  $schema?: string;
  version: string;
  type: SensorType;
  timestamp: {
    unix_ms: number;
    iso8601?: string;
  };
  sequence?: number;
  device: {
    manufacturer: string;
    model: string;
    firmware?: string;
    serial?: string;
  };
  data: Record<string, unknown>;
  meta?: {
    confidence?: number;
    validity?: boolean;
    raw?: Record<string, unknown>;
  };
}

// Validation result
interface ValidationResult {
  valid: boolean;
  file: string;
  type?: SensorType;
  errors: ErrorObject[] | null | undefined;
}

/**
 * WIA AAC Validator class
 */
class WiaAacValidator {
  private ajv: Ajv;
  private baseValidator: ValidateFunction | null = null;
  private sensorValidators: Map<SensorType, ValidateFunction> = new Map();

  constructor() {
    this.ajv = new Ajv({
      allErrors: true,
      verbose: true,
      strict: false
    });
    addFormats(this.ajv);
    this.loadSchemas();
  }

  /**
   * Load all schemas
   */
  private loadSchemas(): void {
    // Load base schema
    const baseSchemaPath = path.join(SCHEMA_DIR, 'wia-aac-signal-v1.schema.json');
    if (fs.existsSync(baseSchemaPath)) {
      const baseSchema = JSON.parse(fs.readFileSync(baseSchemaPath, 'utf-8'));
      // Remove $ref to external files for now (validate separately)
      delete baseSchema.allOf;
      this.baseValidator = this.ajv.compile(baseSchema);
    }

    // Load sensor-specific schemas
    for (const [sensorType, schemaFile] of Object.entries(SENSOR_SCHEMA_MAP)) {
      if (!schemaFile) continue;

      const schemaPath = path.join(SCHEMA_DIR, schemaFile);
      if (fs.existsSync(schemaPath)) {
        const schema = JSON.parse(fs.readFileSync(schemaPath, 'utf-8'));
        const validator = this.ajv.compile(schema);
        this.sensorValidators.set(sensorType as SensorType, validator);
      }
    }
  }

  /**
   * Validate a single message
   */
  validate(message: unknown, filePath: string = 'unknown'): ValidationResult {
    const result: ValidationResult = {
      valid: false,
      file: filePath,
      errors: null
    };

    // Check if message is an object
    if (typeof message !== 'object' || message === null) {
      result.errors = [{
        keyword: 'type',
        instancePath: '',
        schemaPath: '#/type',
        params: { type: 'object' },
        message: 'must be an object'
      }];
      return result;
    }

    const msg = message as WiaAacMessage;
    result.type = msg.type;

    // Validate against base schema
    if (this.baseValidator) {
      const baseValid = this.baseValidator(message);
      if (!baseValid) {
        result.errors = this.baseValidator.errors;
        return result;
      }
    }

    // Validate against sensor-specific schema
    const sensorValidator = this.sensorValidators.get(msg.type);
    if (sensorValidator && msg.data) {
      const sensorValid = sensorValidator(msg.data);
      if (!sensorValid) {
        result.errors = sensorValidator.errors?.map(err => ({
          ...err,
          instancePath: `/data${err.instancePath}`
        }));
        return result;
      }
    }

    result.valid = true;
    return result;
  }

  /**
   * Validate a JSON file
   */
  validateFile(filePath: string): ValidationResult {
    try {
      const content = fs.readFileSync(filePath, 'utf-8');
      const message = JSON.parse(content);
      return this.validate(message, filePath);
    } catch (error) {
      return {
        valid: false,
        file: filePath,
        errors: [{
          keyword: 'parse',
          instancePath: '',
          schemaPath: '',
          params: {},
          message: error instanceof Error ? error.message : 'Unknown error'
        }]
      };
    }
  }
}

/**
 * Format validation errors for display
 */
function formatErrors(errors: ErrorObject[] | null | undefined): string {
  if (!errors || errors.length === 0) return '';

  return errors.map(err => {
    const path = err.instancePath || '/';
    return `  - ${path}: ${err.message}`;
  }).join('\n');
}

/**
 * Main function
 */
function main(): void {
  const args = process.argv.slice(2);

  if (args.length === 0) {
    console.log('WIA AAC Signal Format Validator');
    console.log('================================');
    console.log('');
    console.log('Usage: ts-node validate.ts <file1.json> [file2.json] ...');
    console.log('');
    console.log('Examples:');
    console.log('  ts-node validate.ts sample.json');
    console.log('  ts-node validate.ts ../sample-data/*.json');
    process.exit(0);
  }

  const validator = new WiaAacValidator();
  let allValid = true;
  let totalFiles = 0;
  let validFiles = 0;

  console.log('WIA AAC Signal Format Validator');
  console.log('================================\n');

  for (const filePath of args) {
    totalFiles++;
    const result = validator.validateFile(filePath);

    if (result.valid) {
      validFiles++;
      console.log(`✅ VALID: ${path.basename(filePath)}`);
      console.log(`   Type: ${result.type}`);
    } else {
      allValid = false;
      console.log(`❌ INVALID: ${path.basename(filePath)}`);
      console.log(`   Type: ${result.type || 'unknown'}`);
      console.log(`   Errors:`);
      console.log(formatErrors(result.errors));
    }
    console.log('');
  }

  console.log('================================');
  console.log(`Results: ${validFiles}/${totalFiles} files valid`);

  process.exit(allValid ? 0 : 1);
}

// Run main function
main();
