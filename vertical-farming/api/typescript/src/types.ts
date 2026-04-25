/**
 * WIA Vertical Farming Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

export type FarmType = 'INDOOR' | 'GREENHOUSE' | 'CONTAINER' | 'WAREHOUSE';
export type CropType = 'LEAFY_GREEN' | 'HERB' | 'MICROGREEN' | 'FRUIT' | 'VEGETABLE';
export type GrowthStage = 'SEED' | 'GERMINATION' | 'SEEDLING' | 'VEGETATIVE' | 'FLOWERING' | 'HARVEST';
export type LightingType = 'LED' | 'HPS' | 'NATURAL' | 'HYBRID';
export type SystemStatus = 'NORMAL' | 'WARNING' | 'CRITICAL' | 'OFFLINE';

export interface VerticalFarm {
  farmId: string;
  farmName: string;
  farmType: FarmType;
  location: {
    address: string;
    latitude: number;
    longitude: number;
  };
  totalArea: number;
  numberOfLevels: number;
  capacity: number;
  establishedDate: string;
}

export interface GrowLayer {
  layerId: string;
  farmId: string;
  level: number;
  area: number;
  cropType: CropType;
  plantingDate: string;
  expectedHarvestDate: string;
  currentStage: GrowthStage;
}

export interface EnvironmentalData {
  sensorId: string;
  layerId: string;
  timestamp: string;
  temperature: {
    value: number;
    unit: string;
    status: SystemStatus;
  };
  humidity: {
    value: number;
    unit: string;
    status: SystemStatus;
  };
  co2: {
    value: number;
    unit: string;
    status: SystemStatus;
  };
  light: {
    intensity: number;
    spectrum: string;
    photoperiod: number;
    type: LightingType;
  };
}

export interface NutrientData {
  recordId: string;
  layerId: string;
  timestamp: string;
  ph: number;
  ec: number;
  nutrients: {
    nitrogen: number;
    phosphorus: number;
    potassium: number;
    calcium: number;
    magnesium: number;
  };
  waterTemperature: number;
}

export interface HarvestRecord {
  harvestId: string;
  layerId: string;
  harvestDate: string;
  cropType: CropType;
  quantity: number;
  unit: string;
  quality: string;
  destination: string;
}

export interface EnergyConsumption {
  recordId: string;
  farmId: string;
  timestamp: string;
  lighting: number;
  climate: number;
  irrigation: number;
  total: number;
  unit: string;
  costPerKwh: number;
}

export interface AutomationConfig {
  configId: string;
  layerId: string;
  lighting: {
    enabled: boolean;
    schedule: string[];
    intensity: number;
  };
  irrigation: {
    enabled: boolean;
    frequency: number;
    duration: number;
  };
  climate: {
    targetTemperature: number;
    targetHumidity: number;
    targetCO2: number;
  };
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

export interface VerticalFarmQuery {
  farmId?: string;
  layerId?: string;
  cropType?: CropType;
  growthStage?: GrowthStage;
  startDate?: string;
  endDate?: string;
  limit?: number;
  offset?: number;
}
