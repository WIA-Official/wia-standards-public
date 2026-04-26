/**
 * WIA Elevator System Standard - TypeScript SDK
 * Phase 1-4 Type Definitions
 * 
 * @package @wia/elevator-sdk
 * @version 1.0.0
 * @license MIT
 * 弘益人間 · Benefit All Humanity
 */

// ============================================================================
// Phase 1: Data Format Types
// ============================================================================

export type Direction = 'UP' | 'DOWN' | 'IDLE';
export type DoorStatus = 'OPEN' | 'CLOSED' | 'OPENING' | 'CLOSING';
export type DoorSensorStatus = 'NORMAL' | 'OBSTRUCTION' | 'ERROR';
export type EventType = 'DOOR_CYCLE' | 'FLOOR_CALL' | 'CAR_CALL' | 'TRIP_COMPLETE' | 'MAINTENANCE_START' | 'FAULT_DETECTED' | 'EMERGENCY_STOP';
export type Severity = 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
export type ServiceType = 'ROUTINE' | 'PREVENTIVE' | 'CORRECTIVE' | 'EMERGENCY' | 'INSPECTION';
export type Priority = 'LOW' | 'NORMAL' | 'HIGH' | 'EMERGENCY';

export interface ElevatorStatus {
  elevatorId: string;
  buildingId: string;
  timestamp: string;
  status: {
    currentFloor: number;
    direction: Direction;
    doorStatus: DoorStatus;
    occupancy?: number;
    maxCapacity?: number;
    speed?: number;
    position?: number;
  };
}

export interface SensorData {
  elevatorId: string;
  timestamp: string;
  sensors: {
    loadWeight?: number;
    temperature?: number;
    vibration?: number;
    doorSensor?: DoorSensorStatus;
    emergencyButton?: boolean;
    overloadSensor?: boolean;
    ropeWear?: number;
    brakeWear?: number;
  };
}

export interface ElevatorEvent {
  eventId: string;
  elevatorId: string;
  buildingId: string;
  timestamp: string;
  eventType: EventType;
  severity: Severity;
  floor?: number;
  metadata?: Record<string, any>;
}

export interface ElevatorAlarm {
  alarmId: string;
  elevatorId: string;
  buildingId: string;
  timestamp: string;
  alarmType: string;
  severity: Severity;
  description: string;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: string;
  resolved: boolean;
  resolvedAt?: string;
}

export interface MaintenanceRecord {
  maintenanceId: string;
  elevatorId: string;
  serviceDate: string;
  technicianId: string;
  serviceType: ServiceType;
  workPerformed: Array<{
    task: string;
    partsReplaced?: Array<{
      partNumber: string;
      partName: string;
      quantity: number;
      serialNumber?: string;
    }>;
  }>;
  nextServiceDate?: string;
  tripsSinceService?: number;
  operatingHours?: number;
  notes?: string;
}

export interface TrafficMetrics {
  elevatorId: string;
  buildingId: string;
  period: {
    start: string;
    end: string;
  };
  metrics: {
    totalTrips: number;
    totalPassengers: number;
    averageWaitTime: number;
    averageTravelTime: number;
    peakHourTrips: number;
    energyConsumed: number;
    energyRegenerated: number;
    floorDistribution: Record<number, number>;
  };
}

export interface EnergyMetrics {
  elevatorId: string;
  buildingId: string;
  timestamp: string;
  energyMetrics: {
    consumed: {
      value: number;
      unit: string;
      period: string;
    };
    regenerated: {
      value: number;
      unit: string;
      period: string;
    };
    netConsumption: {
      value: number;
      unit: string;
    };
    efficiency: {
      value: number;
      unit: string;
      description: string;
    };
    peakPower: {
      value: number;
      unit: string;
      timestamp: string;
    };
  };
}

export interface AccessibilityFeatures {
  elevatorId: string;
  accessibilityFeatures: {
    audioAnnouncements: {
      enabled: boolean;
      language: string;
      volume: number;
    };
    brailleButtons: {
      present: boolean;
      layout: string;
    };
    visualIndicators: {
      enabled: boolean;
      type: string;
      fontSize: number;
    };
    doorTimings: {
      openDuration: number;
      closeDuration: number;
      reopenSensitivity: string;
    };
    wheelchairAccommodation: {
      depth: number;
      width: number;
      unit: string;
    };
    emergencyComm: {
      type: string;
      videoEnabled: boolean;
    };
  };
}

// ============================================================================
// Phase 2: API Interface Types
// ============================================================================

export interface DispatchRequest {
  originFloor: number;
  destinationFloor: number;
  passengerCount?: number;
  priority?: Priority;
  accessibility?: {
    wheelchairRequired?: boolean;
    audioAssistance?: boolean;
    visualAssistance?: boolean;
  };
}

export interface DispatchResponse {
  requestId: string;
  assignedElevator: string;
  estimatedWaitTime: number;
  estimatedArrival: string;
}

export interface CommandRequest {
  command: string;
  parameters: Record<string, any>;
  authentication?: {
    token: string;
  };
}

export interface CommandResponse {
  commandId: string;
  status: 'ACCEPTED' | 'REJECTED' | 'EXECUTING' | 'COMPLETED' | 'FAILED';
  message?: string;
  timestamp: string;
}

export interface Building {
  buildingId: string;
  elevatorCount: number;
  elevators: Array<{
    elevatorId: string;
    type: string;
    floors: number[];
    status: string;
  }>;
}

export interface APIError {
  error: {
    code: string;
    message: string;
    requestId: string;
    timestamp: string;
    details?: Record<string, any>;
  };
}

export interface WebSocketMessage {
  type: 'STATUS_UPDATE' | 'TELEMETRY' | 'EVENT' | 'ALARM' | 'COMMAND' | 'HEARTBEAT';
  elevatorId: string;
  timestamp: string;
  payload: any;
}

export interface SubscriptionRequest {
  action: 'SUBSCRIBE' | 'UNSUBSCRIBE';
  streams: Array<'STATUS' | 'TELEMETRY' | 'ALARMS' | 'EVENTS'>;
  elevatorIds: string[];
}

// ============================================================================
// Phase 3: Protocol Types
// ============================================================================

export interface SafetyInterlock {
  type: 'SAFETY_INTERLOCK';
  component: 'DOOR' | 'LOAD' | 'OVERSPEED';
  timestamp: string;
  checks?: Record<string, any>;
  result: 'SAFE_TO_OPERATE' | 'UNSAFE' | 'OVERLOAD_DETECTED' | 'APPROACHING_TRIP';
  action?: string;
}

export interface MQTTConfig {
  topic: string;
  qos: 0 | 1 | 2;
  retain?: boolean;
  payload: any;
}

export interface EmergencyStopEvent {
  type: 'EMERGENCY_STOP';
  elevatorId: string;
  timestamp: string;
  triggeredBy: 'PASSENGER_BUTTON' | 'SYSTEM' | 'REMOTE';
  location: {
    floor: number;
    position: number;
  };
  actions: string[];
}

export interface FireServiceMode {
  mode: 'FIREMAN_SERVICE_PHASE_I' | 'FIREMAN_SERVICE_PHASE_II';
  elevatorId: string;
  controlMethod: 'KEY_SWITCH' | 'DIGITAL_PANEL';
  authentication: {
    keyType: string;
    keyId: string;
  };
  manualControl?: {
    targetFloor: number;
    doorControl: 'AUTOMATIC' | 'MANUAL';
    carLightsOn: boolean;
  };
}

// ============================================================================
// Phase 4: Integration Types
// ============================================================================

export interface BACnetObject {
  objectType: 'ANALOG_VALUE' | 'BINARY_VALUE' | 'MULTI_STATE_VALUE';
  objectInstance: number;
  presentValue: any;
  description?: string;
}

export interface ModbusRegister {
  address: number;
  type: 'HOLDING' | 'INPUT' | 'COIL' | 'DISCRETE';
  value: number;
  description: string;
  access: 'R' | 'W' | 'R/W';
}

export interface AccessControlEvent {
  event: 'CARD_READ' | 'PIN_ENTERED' | 'BIOMETRIC_SCAN';
  elevatorId: string;
  cardData?: string;
  timestamp: string;
  action: string;
  authorization: {
    userId: string;
    authorizedFloors: number[];
    accessGranted: boolean;
  };
}

export interface CloudConfig {
  platform: 'AWS' | 'AZURE' | 'GCP';
  deviceId: string;
  credentials: {
    certificatePath?: string;
    keyPath?: string;
    connectionString?: string;
  };
  topics?: string[];
}

export interface PredictiveMaintenancePrediction {
  elevatorId: string;
  timestamp: string;
  predictions: Array<{
    component: string;
    prediction: 'NORMAL' | 'WARNING' | 'REPLACEMENT_NEEDED';
    confidence: number;
    estimatedDays: number;
    recommendedAction: string;
  }>;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAElevatorClientConfig {
  apiKey?: string;
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;
  websocketEnabled?: boolean;
}
