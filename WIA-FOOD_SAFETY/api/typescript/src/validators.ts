/**
 * WIA-FOOD_SAFETY Validators
 * Input validation functions for food safety data
 */

/**
 * Validate batch ID format
 * Format: AAA-YYYY-NNNNNN (e.g., FARM-2026-001234)
 */
export function validateBatchId(batchId: string): boolean {
  const regex = /^[A-Z]{3,4}-\d{4}-\d{6,12}$/;
  return regex.test(batchId);
}

/**
 * Validate temperature value
 * Range: -50°C to 100°C
 */
export function validateTemperature(temp: number): boolean {
  return typeof temp === 'number' &&
         !isNaN(temp) &&
         temp >= -50 &&
         temp <= 100;
}

/**
 * Validate humidity value
 * Range: 0% to 100%
 */
export function validateHumidity(humidity: number): boolean {
  return typeof humidity === 'number' &&
         !isNaN(humidity) &&
         humidity >= 0 &&
         humidity <= 100;
}

/**
 * Validate GPS coordinates
 */
export function validateCoordinates(latitude: number, longitude: number): boolean {
  return validateLatitude(latitude) && validateLongitude(longitude);
}

export function validateLatitude(lat: number): boolean {
  return typeof lat === 'number' &&
         !isNaN(lat) &&
         lat >= -90 &&
         lat <= 90;
}

export function validateLongitude(lon: number): boolean {
  return typeof lon === 'number' &&
         !isNaN(lon) &&
         lon >= -180 &&
         lon <= 180;
}

/**
 * Validate email address (RFC 5322 simplified)
 */
export function validateEmail(email: string): boolean {
  const regex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return regex.test(email);
}

/**
 * Validate phone number (international format)
 * Format: +1-555-123-4567 or +15551234567
 */
export function validatePhoneNumber(phone: string): boolean {
  const regex = /^\+\d{1,3}[\s-]?\d{3,4}[\s-]?\d{3,4}[\s-]?\d{4}$/;
  return regex.test(phone);
}

/**
 * Validate ISO 8601 date string
 */
export function validateISODate(dateString: string): boolean {
  const date = new Date(dateString);
  return date instanceof Date && !isNaN(date.getTime());
}

/**
 * Validate Ethereum address
 */
export function validateEthereumAddress(address: string): boolean {
  const regex = /^0x[a-fA-F0-9]{40}$/;
  return regex.test(address);
}

/**
 * Validate blockchain transaction hash
 */
export function validateTxHash(txHash: string): boolean {
  const regex = /^0x[a-fA-F0-9]{64}$/;
  return regex.test(txHash);
}

/**
 * Validate product category
 */
export enum ProductCategory {
  FRESH_PRODUCE = 'FRESH_PRODUCE',
  MEAT = 'MEAT',
  POULTRY = 'POULTRY',
  SEAFOOD = 'SEAFOOD',
  DAIRY = 'DAIRY',
  EGGS = 'EGGS',
  PROCESSED_FOODS = 'PROCESSED_FOODS',
  BEVERAGES = 'BEVERAGES',
  BAKERY = 'BAKERY'
}

export function validateProductCategory(category: string): boolean {
  return Object.values(ProductCategory).includes(category as ProductCategory);
}

/**
 * Validate CCP type
 */
export enum CCPType {
  RECEIVING_TEMP = 'RECEIVING_TEMP',
  COOKING_TEMP = 'COOKING_TEMP',
  COOLING_TIME = 'COOLING_TIME',
  COLD_STORAGE = 'COLD_STORAGE',
  HOT_HOLDING = 'HOT_HOLDING',
  PH_CONTROL = 'PH_CONTROL',
  WATER_ACTIVITY = 'WATER_ACTIVITY',
  METAL_DETECTION = 'METAL_DETECTION'
}

export function validateCCPType(ccpType: string): boolean {
  return Object.values(CCPType).includes(ccpType as CCPType);
}

/**
 * Validate recall severity class
 */
export enum RecallClass {
  CLASS_I = 'CLASS_I',
  CLASS_II = 'CLASS_II',
  CLASS_III = 'CLASS_III'
}

export function validateRecallClass(recallClass: string): boolean {
  return Object.values(RecallClass).includes(recallClass as RecallClass);
}

/**
 * Validate quantity
 */
export function validateQuantity(value: number, unit: string): boolean {
  if (typeof value !== 'number' || value <= 0) {
    return false;
  }

  const validUnits = ['kg', 'g', 'lb', 'oz', 'L', 'mL', 'gal', 'unit', 'box', 'pallet'];
  return validUnits.includes(unit);
}

/**
 * Validate sensor ID format
 * Format: SENSOR-NNNNNN (e.g., TEMP-SENSOR-001234)
 */
export function validateSensorId(sensorId: string): boolean {
  const regex = /^[A-Z]+-SENSOR-\d{6}$/;
  return regex.test(sensorId);
}

/**
 * Comprehensive validation for FoodProduct registration
 */
export function validateProductRegistration(data: any): { valid: boolean; errors: string[] } {
  const errors: string[] = [];

  // Required fields
  if (!data.productName || typeof data.productName !== 'string') {
    errors.push('productName is required and must be a string');
  }

  if (!data.batchId || !validateBatchId(data.batchId)) {
    errors.push('batchId is required and must be in format AAA-YYYY-NNNNNN');
  }

  if (!data.category || !validateProductCategory(data.category)) {
    errors.push('category is required and must be a valid ProductCategory');
  }

  if (!data.harvestDate || !validateISODate(data.harvestDate)) {
    errors.push('harvestDate is required and must be a valid ISO 8601 date');
  }

  if (!data.expirationDate || !validateISODate(data.expirationDate)) {
    errors.push('expirationDate is required and must be a valid ISO 8601 date');
  }

  // Validate origin
  if (!data.origin || typeof data.origin !== 'object') {
    errors.push('origin is required and must be an object');
  } else {
    if (!data.origin.location || typeof data.origin.location !== 'object') {
      errors.push('origin.location is required');
    } else {
      if (!validateLatitude(data.origin.location.latitude)) {
        errors.push('origin.location.latitude must be between -90 and 90');
      }
      if (!validateLongitude(data.origin.location.longitude)) {
        errors.push('origin.location.longitude must be between -180 and 180');
      }
    }
  }

  // Validate quantity
  if (!data.quantity || typeof data.quantity !== 'object') {
    errors.push('quantity is required and must be an object');
  } else {
    if (!validateQuantity(data.quantity.value, data.quantity.unit)) {
      errors.push('quantity.value must be positive and quantity.unit must be valid');
    }
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

/**
 * Validate CCP record submission
 */
export function validateCCPRecord(data: any): { valid: boolean; errors: string[] } {
  const errors: string[] = [];

  if (!data.batchId || !validateBatchId(data.batchId)) {
    errors.push('batchId is required and must be valid');
  }

  if (!data.ccpType || !validateCCPType(data.ccpType)) {
    errors.push('ccpType is required and must be valid');
  }

  if (!data.measurement || typeof data.measurement !== 'object') {
    errors.push('measurement is required and must be an object');
  } else {
    if (data.measurement.temperature !== undefined && !validateTemperature(data.measurement.temperature)) {
      errors.push('measurement.temperature must be between -50 and 100');
    }
    if (data.measurement.humidity !== undefined && !validateHumidity(data.measurement.humidity)) {
      errors.push('measurement.humidity must be between 0 and 100');
    }
  }

  if (!data.sensorId || !validateSensorId(data.sensorId)) {
    errors.push('sensorId is required and must be valid format');
  }

  return {
    valid: errors.length === 0,
    errors
  };
}
