/**
 * WIA-DEF-016: Military Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Communications Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Communication Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number;
  datum?: 'WGS84' | 'NAD83' | 'MGRS';
}

/**
 * Military Grid Reference System coordinate
 */
export interface MGRSCoordinate {
  grid: string;        // e.g., "38SMB4484"
  precision: number;   // Digits of precision (2, 4, 6, 8, 10)
}

/**
 * Frequency band designation
 */
export type FrequencyBand =
  | 'hf'      // 3-30 MHz
  | 'vhf'     // 30-300 MHz
  | 'uhf'     // 300-3000 MHz
  | 'l-band'  // 1-2 GHz
  | 's-band'  // 2-4 GHz
  | 'x-band'  // 8-12 GHz
  | 'ku-band' // 12-18 GHz
  | 'ka-band' // 26-40 GHz
  | 'ehf';    // 30-300 GHz

/**
 * Classification level
 */
export type ClassificationLevel =
  | 'unclassified'
  | 'cui'           // Controlled Unclassified Information
  | 'confidential'
  | 'secret'
  | 'top-secret'
  | 'ts-sci';       // Top Secret / Sensitive Compartmented Information

// ============================================================================
// Link Budget and RF Calculations
// ============================================================================

/**
 * Link budget calculation parameters
 */
export interface LinkBudgetParams {
  /** Transmit power in watts */
  transmitPower: number;

  /** Operating frequency in MHz */
  frequency: number;

  /** Distance in meters */
  distance: number;

  /** Transmit antenna gain in dBi (optional, default: 0) */
  txAntennaGain?: number;

  /** Receive antenna gain in dBi (optional, default: 0) */
  rxAntennaGain?: number;

  /** Transmit antenna height in meters */
  txHeight?: number;

  /** Receive antenna height in meters */
  rxHeight?: number;

  /** Terrain type affecting propagation */
  terrain?: 'open' | 'urban' | 'forest' | 'mountainous' | 'desert';

  /** Additional losses in dB (cable, connectors, etc.) */
  additionalLosses?: number;

  /** Atmospheric absorption in dB */
  atmosphericLoss?: number;

  /** Rain attenuation in dB (for high frequencies) */
  rainLoss?: number;
}

/**
 * Link budget calculation result
 */
export interface LinkBudgetResult {
  /** Received signal power in dBm */
  receivedPower: number;

  /** Free-space path loss in dB */
  pathLoss: number;

  /** Total system loss in dB */
  totalLoss: number;

  /** Link margin in dB */
  linkMargin: number;

  /** Maximum achievable distance in km */
  maxDistance: number;

  /** Signal-to-noise ratio in dB */
  snr: number;

  /** Link quality assessment */
  feasibility: 'excellent' | 'good' | 'marginal' | 'poor' | 'impossible';

  /** Warnings or recommendations */
  warnings: string[];
}

// ============================================================================
// Encryption and COMSEC
// ============================================================================

/**
 * Encryption algorithm type
 */
export type EncryptionAlgorithm =
  | 'aes-128-cbc'
  | 'aes-256-cbc'
  | 'aes-128-gcm'
  | 'aes-256-gcm'
  | 'chacha20-poly1305';

/**
 * Encryption request
 */
export interface EncryptionRequest {
  /** Message to encrypt */
  message: string;

  /** Classification level */
  encryptionLevel: ClassificationLevel;

  /** Encryption algorithm */
  algorithm: EncryptionAlgorithm;

  /** Key identifier */
  keyId: string;

  /** Additional authenticated data (for GCM mode) */
  aad?: string;
}

/**
 * Encryption response
 */
export interface EncryptionResponse {
  /** Encrypted ciphertext (Base64 encoded) */
  ciphertext: string;

  /** Initialization vector (Base64 encoded) */
  iv: string;

  /** Authentication tag (for GCM mode, Base64 encoded) */
  authTag?: string;

  /** Encryption timestamp */
  timestamp: Date;

  /** Key expiration time */
  expiration: Date;

  /** Algorithm used */
  algorithm: EncryptionAlgorithm;

  /** Key identifier */
  keyId: string;
}

/**
 * Decryption request
 */
export interface DecryptionRequest {
  /** Ciphertext to decrypt (Base64 encoded) */
  ciphertext: string;

  /** Initialization vector (Base64 encoded) */
  iv: string;

  /** Authentication tag (for GCM mode, Base64 encoded) */
  authTag?: string;

  /** Key identifier */
  keyId: string;

  /** Algorithm used */
  algorithm: EncryptionAlgorithm;

  /** Additional authenticated data (for GCM mode) */
  aad?: string;
}

/**
 * Key material information
 */
export interface KeyMaterial {
  /** Key identifier */
  keyId: string;

  /** Key type */
  keyType: 'symmetric' | 'asymmetric-public' | 'asymmetric-private';

  /** Algorithm */
  algorithm: string;

  /** Key length in bits */
  keyLength: number;

  /** Effective date */
  effectiveDate: Date;

  /** Expiration date */
  expirationDate: Date;

  /** Classification level */
  classification: ClassificationLevel;

  /** Key status */
  status: 'active' | 'inactive' | 'revoked' | 'expired';
}

// ============================================================================
// Frequency Management
// ============================================================================

/**
 * Frequency allocation request
 */
export interface FrequencyAllocationRequest {
  /** Requested frequency in MHz */
  frequency: number;

  /** Required bandwidth in kHz */
  bandwidth: number;

  /** Operating location */
  location: GeoCoordinate;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Purpose of communication */
  purpose: 'voice' | 'data' | 'video' | 'telemetry';

  /** Priority level */
  priority: 'routine' | 'priority' | 'immediate' | 'flash';

  /** Requesting unit */
  requestingUnit: string;
}

/**
 * Frequency allocation response
 */
export interface FrequencyAllocationResponse {
  /** Was allocation successful? */
  isApproved: boolean;

  /** Allocated frequency (may differ from requested) */
  allocatedFrequency?: number;

  /** Allocation ID */
  allocationId?: string;

  /** Conflicts detected */
  conflicts: FrequencyConflict[];

  /** Alternative frequencies suggested */
  alternativeFrequencies?: number[];

  /** Recommendation message */
  recommendation: string;

  /** Restrictions or special instructions */
  restrictions?: string[];
}

/**
 * Frequency conflict information
 */
export interface FrequencyConflict {
  /** Conflicting frequency */
  frequency: number;

  /** Distance to conflicting transmitter */
  distance: number;

  /** Conflicting unit */
  conflictingUnit: string;

  /** Severity of conflict */
  severity: 'minor' | 'moderate' | 'major' | 'critical';

  /** Recommended action */
  recommendation: string;
}

// ============================================================================
// Tactical Radio Systems
// ============================================================================

/**
 * Radio system type
 */
export type RadioType =
  | 'manpack-hf'
  | 'vhf-tactical'
  | 'uhf-tactical'
  | 'sincgars'
  | 'jtrs'
  | 'satcom-terminal'
  | 'handheld';

/**
 * Radio configuration
 */
export interface RadioConfig {
  /** Radio identifier */
  radioId: string;

  /** Radio type */
  radioType: RadioType;

  /** Operating frequency in MHz */
  frequency: number;

  /** Transmit power in watts */
  power: number;

  /** Modulation type */
  modulation: 'am' | 'fm' | 'usb' | 'lsb' | 'dsss' | 'fhss';

  /** Bandwidth in kHz */
  bandwidth: number;

  /** Call sign */
  callSign: string;

  /** Encryption enabled */
  encryptionEnabled: boolean;

  /** Encryption key ID */
  encryptionKeyId?: string;

  /** Anti-jam mode */
  antiJamMode?: 'none' | 'fhss' | 'dsss' | 'adaptive-nulling';

  /** Operating mode */
  mode: 'voice' | 'data' | 'mixed';
}

/**
 * Radio status
 */
export interface RadioStatus {
  /** Radio identifier */
  radioId: string;

  /** Operational status */
  status: 'operational' | 'degraded' | 'offline' | 'maintenance';

  /** Current frequency */
  currentFrequency: number;

  /** Received signal strength in dBm */
  rssi: number;

  /** Signal-to-noise ratio in dB */
  snr: number;

  /** Bit error rate */
  ber: number;

  /** Battery level (percentage) */
  batteryLevel: number;

  /** Last update timestamp */
  lastUpdate: Date;

  /** Active connections */
  activeConnections: number;

  /** Jamming detected */
  jammingDetected: boolean;

  /** Jamming type (if detected) */
  jammingType?: 'narrowband' | 'wideband' | 'sweep' | 'barrage';
}

// ============================================================================
// Tactical Data Links
// ============================================================================

/**
 * Tactical data link type
 */
export type DataLinkType =
  | 'link-16'   // JTIDS/MIDS
  | 'link-11'   // Naval tactical
  | 'link-22'   // NATO improved Link 11
  | 'sadl'      // Situational Awareness Data Link
  | 'ttnt';     // Tactical Targeting Network Technology

/**
 * Link 16 J-Series message type
 */
export type Link16MessageType =
  | 'j2.0'   // Indirect interface unit
  | 'j3.0'   // Fighter-to-fighter
  | 'j3.2'   // Air control
  | 'j7.0'   // Weapons coordination
  | 'j12.0'  // Control
  | 'j14.0'; // Missile

/**
 * Tactical data link message
 */
export interface TacticalDataLinkMessage {
  /** Message identifier */
  messageId: string;

  /** Data link type */
  linkType: DataLinkType;

  /** Message type (link-specific) */
  messageType: string;

  /** Sender track number */
  senderTrack: string;

  /** Message priority */
  priority: 'routine' | 'priority' | 'immediate' | 'flash';

  /** Message payload */
  payload: Record<string, unknown>;

  /** Timestamp */
  timestamp: Date;

  /** Classification */
  classification: ClassificationLevel;

  /** Security label */
  securityLabel?: string;
}

// ============================================================================
// Position Reporting
// ============================================================================

/**
 * Position report (POSREP)
 */
export interface PositionReport {
  /** Reporting unit identifier */
  unitId: string;

  /** Unit call sign */
  callSign: string;

  /** Position */
  position: GeoCoordinate;

  /** Heading in degrees (0-359) */
  heading: number;

  /** Speed in m/s */
  speed: number;

  /** Unit status */
  status: 'green' | 'amber' | 'red' | 'black';

  /** Timestamp */
  timestamp: Date;

  /** Classification */
  classification: ClassificationLevel;

  /** Additional information */
  remarks?: string;
}

/**
 * Contact report (CONREP)
 */
export interface ContactReport {
  /** Reporting unit */
  reportingUnit: string;

  /** Contact type */
  contactType:
    | 'enemy_infantry'
    | 'enemy_armor'
    | 'enemy_aircraft'
    | 'enemy_artillery'
    | 'unknown'
    | 'civilian';

  /** Contact location */
  location: GeoCoordinate | MGRSCoordinate;

  /** Estimated strength */
  strength: 'individual' | 'squad_size' | 'platoon_size' | 'company_size' | 'battalion_plus';

  /** Activity */
  activity: string;

  /** Confidence level */
  confidence: 'possible' | 'probable' | 'confirmed';

  /** Timestamp */
  timestamp: Date;

  /** Classification */
  classification: ClassificationLevel;

  /** Additional details */
  remarks?: string;
}

// ============================================================================
// Network-Centric Warfare
// ============================================================================

/**
 * Common Operating Picture (COP) entity
 */
export interface COPEntity {
  /** Track number */
  trackNumber: string;

  /** Entity type */
  entityType: 'friendly' | 'hostile' | 'neutral' | 'unknown';

  /** Platform type */
  platformType: string;

  /** Position */
  position: GeoCoordinate;

  /** Heading in degrees */
  heading: number;

  /** Speed in m/s */
  speed: number;

  /** Altitude in meters */
  altitude: number;

  /** Last update timestamp */
  lastUpdate: Date;

  /** Source of information */
  source: string;

  /** Confidence in track */
  confidence: number; // 0-1

  /** Track quality */
  quality: 'high' | 'medium' | 'low';
}

/**
 * Blue Force Tracking update
 */
export interface BlueForceUpdate {
  /** Unit identifier */
  unitId: string;

  /** Platform type */
  platformType: string;

  /** Position */
  position: GeoCoordinate;

  /** Heading */
  heading: number;

  /** Speed */
  speed: number;

  /** Status */
  status: 'mission_capable' | 'mission_degraded' | 'mission_incapable';

  /** Fuel state (percentage) */
  fuelState: number;

  /** Ammunition state */
  ammoState: 'green' | 'amber' | 'red' | 'black';

  /** Personnel status */
  personnelStatus: {
    total: number;
    casualties: number;
  };

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Anti-Jamming and Electronic Warfare
// ============================================================================

/**
 * Jamming detection report
 */
export interface JammingReport {
  /** Detection ID */
  detectionId: string;

  /** Affected frequency */
  frequency: number;

  /** Jamming type */
  jammingType: 'narrowband' | 'wideband' | 'sweep' | 'barrage' | 'follower' | 'spot';

  /** Jamming power in dBm */
  jammingPower: number;

  /** Jammer-to-signal ratio in dB */
  jammerToSignalRatio: number;

  /** Estimated jammer location (if triangulated) */
  jammerLocation?: GeoCoordinate;

  /** Detection timestamp */
  timestamp: Date;

  /** Affected systems */
  affectedSystems: string[];

  /** Countermeasures applied */
  countermeasures: string[];

  /** Impact assessment */
  impact: 'none' | 'minor' | 'moderate' | 'severe' | 'critical';
}

/**
 * Frequency hopping parameters
 */
export interface FrequencyHoppingParams {
  /** Hop set (frequencies in MHz) */
  hopSet: number[];

  /** Hop rate (hops per second) */
  hopRate: number;

  /** Hop pattern seed */
  seed: string;

  /** Dwell time per frequency (milliseconds) */
  dwellTime: number;

  /** Synchronization reference */
  syncReference: Date;
}

// ============================================================================
// Interoperability
// ============================================================================

/**
 * Coalition network configuration
 */
export interface CoalitionNetworkConfig {
  /** Network identifier */
  networkId: string;

  /** Coalition members */
  members: string[];

  /** Shared classification level */
  sharedClassification: ClassificationLevel;

  /** Data sharing agreement */
  dataSharingAgreement: string;

  /** Gateway address */
  gatewayAddress: string;

  /** Encryption standard */
  encryptionStandard: string;

  /** Active status */
  active: boolean;
}

/**
 * STANAG compliance information
 */
export interface STANAGCompliance {
  /** STANAG number */
  stanagNumber: string;

  /** STANAG title */
  title: string;

  /** Compliance level */
  complianceLevel: 'full' | 'partial' | 'none';

  /** Version */
  version: string;

  /** Certification date */
  certificationDate?: Date;

  /** Limitations or caveats */
  limitations?: string[];
}

// ============================================================================
// Satellite Communications
// ============================================================================

/**
 * SATCOM terminal type
 */
export type SATCOMTerminalType =
  | 'strategic'      // Large fixed terminals (>2.4m)
  | 'tactical'       // Mobile terminals (0.6-1.2m)
  | 'manpack'        // Portable terminals (<10kg)
  | 'handheld';      // Handheld terminals (<2kg)

/**
 * SATCOM link parameters
 */
export interface SATCOMLinkParams {
  /** Terminal type */
  terminalType: SATCOMTerminalType;

  /** Satellite identifier */
  satelliteId: string;

  /** Uplink frequency in MHz */
  uplinkFrequency: number;

  /** Downlink frequency in MHz */
  downlinkFrequency: number;

  /** Terminal location */
  location: GeoCoordinate;

  /** Antenna diameter in meters */
  antennaDiameter: number;

  /** Transmit power in watts */
  transmitPower: number;

  /** Data rate in kbps */
  dataRate: number;

  /** Modulation and coding */
  modcod: string;
}

/**
 * SATCOM link status
 */
export interface SATCOMLinkStatus {
  /** Link identifier */
  linkId: string;

  /** Link status */
  status: 'acquiring' | 'locked' | 'tracking' | 'lost';

  /** Carrier-to-noise ratio in dB */
  cnr: number;

  /** Bit error rate */
  ber: number;

  /** Eb/N0 in dB */
  ebno: number;

  /** Satellite elevation angle in degrees */
  elevation: number;

  /** Satellite azimuth angle in degrees */
  azimuth: number;

  /** Rain fade margin in dB */
  rainMargin: number;

  /** Last update */
  lastUpdate: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and RF constants
 */
export const RF_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant in J/K */
  BOLTZMANN: 1.380649e-23,

  /** Standard temperature in Kelvin */
  STANDARD_TEMP: 290,

  /** Noise figure in dB (typical receiver) */
  NOISE_FIGURE: 8,

  /** Thermal noise floor at 290K, 1Hz bandwidth (dBm/Hz) */
  THERMAL_NOISE_FLOOR: -174,

  /** Earth radius in km (for radio horizon calculations) */
  EARTH_RADIUS: 6371,

  /** Radio horizon constant K-factor */
  K_FACTOR: 4 / 3,
} as const;

/**
 * Frequency band definitions in MHz
 */
export const FREQUENCY_BANDS = {
  HF: { min: 3, max: 30 },
  VHF: { min: 30, max: 300 },
  UHF: { min: 300, max: 3000 },
  L_BAND: { min: 1000, max: 2000 },
  S_BAND: { min: 2000, max: 4000 },
  X_BAND: { min: 8000, max: 12000 },
  KU_BAND: { min: 12000, max: 18000 },
  KA_BAND: { min: 26000, max: 40000 },
  EHF: { min: 30000, max: 300000 },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-016 error codes
 */
export enum CommErrorCode {
  INSUFFICIENT_POWER = 'C001',
  FREQUENCY_CONFLICT = 'C002',
  ENCRYPTION_FAILED = 'C003',
  LINK_BUDGET_FAILED = 'C004',
  JAMMING_DETECTED = 'C005',
  KEY_EXPIRED = 'C006',
  INVALID_FREQUENCY = 'C007',
  UNAUTHORIZED_ACCESS = 'C008',
  SATCOM_LINK_LOST = 'C009',
  INTEROP_FAILURE = 'C010',
}

/**
 * Military communication error
 */
export class MilitaryCommError extends Error {
  constructor(
    public code: CommErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MilitaryCommError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  GeoCoordinate,
  MGRSCoordinate,
  FrequencyBand,
  ClassificationLevel,
  LinkBudgetParams,
  LinkBudgetResult,
  EncryptionRequest,
  EncryptionResponse,
  DecryptionRequest,
  KeyMaterial,
  FrequencyAllocationRequest,
  FrequencyAllocationResponse,
  FrequencyConflict,
  RadioConfig,
  RadioStatus,
  TacticalDataLinkMessage,
  PositionReport,
  ContactReport,
  COPEntity,
  BlueForceUpdate,
  JammingReport,
  FrequencyHoppingParams,
  CoalitionNetworkConfig,
  STANAGCompliance,
  SATCOMLinkParams,
  SATCOMLinkStatus,
};

export { RF_CONSTANTS, FREQUENCY_BANDS, CommErrorCode, MilitaryCommError };
