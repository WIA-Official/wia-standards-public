/**
 * WIA Agricultural Supply Chain Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

export type ProductType = 'FRESH_PRODUCE' | 'DAIRY' | 'MEAT' | 'SEAFOOD' | 'FROZEN' | 'GRAIN';
export type ShipmentStatus = 'PENDING' | 'IN_TRANSIT' | 'ARRIVED' | 'DELIVERED' | 'REJECTED';
export type WaypointType = 'ORIGIN' | 'TRANSFER' | 'DESTINATION';
export type SensorStatus = 'NORMAL' | 'WARNING' | 'CRITICAL';
export type DoorStatus = 'OPEN' | 'CLOSED';

export interface Location {
  address: string;
  latitude: number;
  longitude: number;
}

export interface ProductInfo {
  productId: string;
  productName: string;
  productType: ProductType;
  variety: string;
  quantity: number;
  unit: string;
  batchNumber: string;
  harvestDate: string;
  shelfLife: number;
  certifications: string[];
}

export interface Origin {
  farmId: string;
  farmName: string;
  location: Location;
  operator: string;
  contactPhone: string;
}

export interface Destination {
  facilityId: string;
  facilityName: string;
  location: Location;
  contactPhone: string;
}

export interface Waypoint {
  seq: number;
  location: string;
  timestamp: string;
  type: WaypointType;
}

export interface Route {
  waypoints: Waypoint[];
  totalDistance: number;
  estimatedDuration: number;
}

export interface Shipment {
  shipmentId: string;
  timestamp: string;
  productInfo: ProductInfo;
  origin: Origin;
  destination: Destination;
  route: Route;
  status: ShipmentStatus;
}

export interface TemperatureSensor {
  current: number;
  min: number;
  max: number;
  unit: string;
  status: SensorStatus;
}

export interface HumiditySensor {
  current: number;
  min: number;
  max: number;
  unit: string;
  status: SensorStatus;
}

export interface DoorSensor {
  status: DoorStatus;
  lastOpened: string;
  openDuration: number;
}

export interface ShockSensor {
  detected: boolean;
  level: number;
  threshold: number;
}

export interface BatterySensor {
  level: number;
  voltage: number;
  estimatedLife: number;
}

export interface ColdChainData {
  shipmentId: string;
  deviceId: string;
  timestamp: string;
  location: { latitude: number; longitude: number; accuracy: number };
  temperature: TemperatureSensor;
  humidity: HumiditySensor;
  door: DoorSensor;
  shock: ShockSensor;
  battery: BatterySensor;
  alerts: string[];
}

export interface ProvenanceRecord {
  recordId: string;
  shipmentId: string;
  event: string;
  timestamp: string;
  actor: string;
  location: string;
  blockchainHash?: string;
}

export interface QualityControl {
  inspectionId: string;
  shipmentId: string;
  timestamp: string;
  inspector: string;
  grade: string;
  passed: boolean;
  notes: string;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

export interface SupplyChainQuery {
  shipmentId?: string;
  productType?: ProductType;
  status?: ShipmentStatus;
  startDate?: string;
  endDate?: string;
  limit?: number;
  offset?: number;
}
