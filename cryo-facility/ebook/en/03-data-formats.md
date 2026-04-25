# Chapter 3: Cryogenic Facility Data Formats and Schemas

## Comprehensive Data Architecture for Facility Management

This chapter defines the complete data structures and schemas used in the WIA Cryo Facility Standard. These formats enable standardized facility configuration, equipment management, operational procedures, and environmental monitoring across all cryogenic storage facilities.

---

## Core Data Architecture

### Project Root Schema

```typescript
/**
 * WIA Cryo Facility Standard - Core Data Types
 * Version 1.0.0
 */

/**
 * Root project schema - Complete cryogenic facility definition
 */
interface WIACryoFacilityProject {
  // Standard identification
  standard: 'WIA-CRYO-FACILITY';
  version: string;

  // Facility metadata
  metadata: ProjectMetadata;

  // Physical facility configuration
  facility: FacilityConfiguration;

  // Equipment inventory and management
  equipment: EquipmentInventory;

  // Operational procedures
  operations: OperationalProcedures;

  // Safety management
  safety: SafetyManagement;

  // Quality management
  quality: QualityManagement;

  // Environmental control
  environmental: EnvironmentalControl;

  // Monitoring systems
  monitoring: MonitoringSystem;

  // Extension point
  extensions?: Record<string, unknown>;
}

/**
 * Project metadata - Facility identification and organization
 */
interface ProjectMetadata {
  // Unique identifier
  id: string;

  // Human-readable facility name
  name: string;

  // Description
  description?: string;

  // Facility type classification
  type: FacilityType;

  // Physical location
  location: FacilityLocation;

  // Operating organization
  organization: Organization;

  // Regulatory licenses
  licenses: License[];

  // Quality certifications
  certifications: Certification[];

  // Creation timestamp
  createdAt: string;

  // Current operational status
  status: FacilityStatus;
}

/**
 * Facility type enumeration
 */
type FacilityType =
  | 'biobank'              // Large-scale biological sample storage
  | 'tissue-bank'          // Human tissue preservation
  | 'fertility-center'     // Reproductive cells and embryos
  | 'research-facility'    // Research specimens and cell lines
  | 'hospital-unit'        // Clinical specimen storage
  | 'commercial-storage';  // Third-party storage services

/**
 * Facility operational status
 */
type FacilityStatus =
  | 'operational'          // Normal operations
  | 'limited-operations'   // Partial capacity
  | 'maintenance'          // Scheduled maintenance
  | 'emergency'            // Emergency situation
  | 'offline';             // Not operational
```

---

## Facility Location and Organization

### Location and Contact Schemas

```typescript
/**
 * Facility Location Schema
 * Physical location with precise coordinates
 */
interface FacilityLocation {
  // Street address
  address: string;

  // City name
  city: string;

  // State or province
  state?: string;

  // Country (ISO 3166-1)
  country: string;

  // Postal/ZIP code
  postalCode: string;

  // GPS coordinates
  coordinates: Coordinates;

  // IANA timezone
  timezone: string;
}

interface Coordinates {
  latitude: number;   // -90 to +90
  longitude: number;  // -180 to +180
}

/**
 * Organization Schema
 * Operating entity information
 */
interface Organization {
  // Organization name
  name: string;

  // Organization type
  type: string;

  // Legal registration number
  registrationNumber?: string;

  // Primary contact
  contact: ContactInfo;
}

interface ContactInfo {
  // Contact person name
  name: string;

  // Email address
  email: string;

  // Phone number (E.164 format)
  phone: string;

  // Emergency contact number
  emergencyPhone?: string;
}

/**
 * License Schema
 * Regulatory licensing information
 */
interface License {
  // License type
  type: string;

  // License number
  number: string;

  // Issuing authority
  issuer: string;

  // Effective date
  validFrom: string;  // ISO 8601

  // Expiration date
  validTo: string;    // ISO 8601

  // Authorized activities
  scope: string[];

  // Current license status
  status: 'active' | 'expired' | 'suspended' | 'pending';
}

/**
 * Certification Schema
 * Quality and accreditation certifications
 */
interface Certification {
  // Certification name
  name: string;

  // Certifying body
  body: string;

  // Certificate number
  number: string;

  // Certified scope
  scope: string[];

  // Effective date
  validFrom: string;

  // Expiration date
  validTo: string;

  // Current status
  status: 'active' | 'expired' | 'pending';
}

// Location validation utilities
class LocationValidator {
  static validateCoordinates(coords: Coordinates): ValidationResult {
    const errors: ValidationError[] = [];

    if (coords.latitude < -90 || coords.latitude > 90) {
      errors.push({
        path: 'coordinates.latitude',
        message: 'Latitude must be between -90 and 90',
        value: coords.latitude
      });
    }

    if (coords.longitude < -180 || coords.longitude > 180) {
      errors.push({
        path: 'coordinates.longitude',
        message: 'Longitude must be between -180 and 180',
        value: coords.longitude
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  static validatePhone(phone: string): boolean {
    // E.164 format validation
    const e164Pattern = /^\+[1-9]\d{1,14}$/;
    return e164Pattern.test(phone);
  }

  static validateEmail(email: string): boolean {
    const emailPattern = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailPattern.test(email);
  }
}

interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}
```

---

## Facility Configuration

### Layout and Zone Schemas

```typescript
/**
 * Facility Configuration Schema
 * Physical layout and infrastructure
 */
interface FacilityConfiguration {
  // Physical layout
  layout: FacilityLayout;

  // Functional zones
  zones: FacilityZone[];

  // Capacity metrics
  capacity: CapacityMetrics;

  // Infrastructure systems
  infrastructure: InfrastructureConfig;

  // Access control
  access: AccessControlConfig;
}

/**
 * Facility Layout Schema
 * Physical space configuration
 */
interface FacilityLayout {
  // Total facility area
  totalArea: number;

  // Storage area
  storageArea: number;

  // Processing area
  processingArea: number;

  // Office/administrative area
  officeArea: number;

  // Area unit
  unit: 'sqft' | 'sqm';

  // Floor definitions
  floors: Floor[];

  // Floor plan document reference
  floorPlan?: string;
}

interface Floor {
  // Floor level (-1, 0, 1, 2, etc.)
  level: number;

  // Floor name
  name: string;

  // Floor area
  area: number;

  // Zones on this floor
  zones: string[];
}

/**
 * Facility Zone Schema
 * Functional area definitions
 */
interface FacilityZone {
  // Unique zone identifier
  id: string;

  // Zone name
  name: string;

  // Zone type classification
  type: ZoneType;

  // Cleanroom classification
  classification: CleanroomClass;

  // Zone area
  area: number;

  // Equipment in zone
  equipment: string[];

  // Access restriction level
  accessLevel: AccessLevel;

  // Environmental requirements
  environmentalRequirements: EnvironmentalRequirements;
}

type ZoneType =
  | 'storage'           // Specimen storage
  | 'processing'        // Sample processing
  | 'quality-control'   // QC testing
  | 'receiving'         // Sample receiving
  | 'shipping'          // Sample shipping
  | 'preparation'       // Preparation area
  | 'office'            // Administrative
  | 'utility';          // Utility/mechanical

type CleanroomClass =
  | 'ISO-5'             // Class 100
  | 'ISO-6'             // Class 1,000
  | 'ISO-7'             // Class 10,000
  | 'ISO-8'             // Class 100,000
  | 'non-classified';   // No cleanroom requirements

type AccessLevel =
  | 'public'            // General access
  | 'restricted'        // Employee only
  | 'controlled'        // Authorized personnel
  | 'high-security';    // Critical areas

/**
 * Environmental Requirements Schema
 * Zone environmental specifications
 */
interface EnvironmentalRequirements {
  // Temperature range
  temperature: TemperatureRange;

  // Humidity range
  humidity: HumidityRange;

  // Particulate limits
  particulate?: ParticulateLimit;

  // Pressure differential
  pressure?: PressureDifferential;
}

interface TemperatureRange {
  min: number;
  max: number;
  unit: 'celsius' | 'fahrenheit';
}

interface HumidityRange {
  min: number;  // Percentage
  max: number;  // Percentage
}

interface ParticulateLimit {
  size: number;   // Microns
  count: number;  // Particles per cubic meter
}

interface PressureDifferential {
  value: number;
  unit: 'pascals' | 'inches-h2o';
}

// Zone configuration builder
class ZoneConfigurationBuilder {
  private zone: Partial<FacilityZone> = {};

  static create(id: string, name: string): ZoneConfigurationBuilder {
    const builder = new ZoneConfigurationBuilder();
    builder.zone.id = id;
    builder.zone.name = name;
    return builder;
  }

  setType(type: ZoneType): this {
    this.zone.type = type;
    return this;
  }

  setClassification(classification: CleanroomClass): this {
    this.zone.classification = classification;
    return this;
  }

  setArea(area: number): this {
    this.zone.area = area;
    return this;
  }

  setAccessLevel(level: AccessLevel): this {
    this.zone.accessLevel = level;
    return this;
  }

  setEnvironment(requirements: EnvironmentalRequirements): this {
    this.zone.environmentalRequirements = requirements;
    return this;
  }

  addEquipment(...equipment: string[]): this {
    this.zone.equipment = [...(this.zone.equipment || []), ...equipment];
    return this;
  }

  build(): FacilityZone {
    // Validate required fields
    if (!this.zone.id || !this.zone.name || !this.zone.type) {
      throw new Error('Zone requires id, name, and type');
    }

    return {
      id: this.zone.id,
      name: this.zone.name,
      type: this.zone.type,
      classification: this.zone.classification || 'non-classified',
      area: this.zone.area || 0,
      equipment: this.zone.equipment || [],
      accessLevel: this.zone.accessLevel || 'restricted',
      environmentalRequirements: this.zone.environmentalRequirements || {
        temperature: { min: 20, max: 25, unit: 'celsius' },
        humidity: { min: 30, max: 60 }
      }
    };
  }
}
```

---

## Capacity and Infrastructure

### Capacity Metrics Schema

```typescript
/**
 * Capacity Metrics Schema
 * Facility capacity measurements
 */
interface CapacityMetrics {
  // Storage capacity
  storage: StorageCapacity;

  // Processing capacity
  processing: ProcessingCapacity;

  // Personnel capacity
  personnel: PersonnelCapacity;
}

interface StorageCapacity {
  // Number of storage tanks/units
  tanks: number;

  // Total specimen capacity
  totalSpecimens: number;

  // Current specimen count
  currentSpecimens: number;

  // Utilization percentage
  utilizationPercent: number;
}

interface ProcessingCapacity {
  // Daily throughput
  daily: number;

  // Weekly throughput
  weekly: number;

  // Unit of measurement
  unit: string;
}

interface PersonnelCapacity {
  // Maximum staff capacity
  maximum: number;

  // Current staff count
  current: number;

  // Shift configurations
  shifts: ShiftConfig[];
}

interface ShiftConfig {
  // Shift name
  name: string;

  // Start time (HH:MM)
  start: string;

  // End time (HH:MM)
  end: string;

  // Days of operation
  days: string[];

  // Minimum staff required
  minimumStaff: number;
}

/**
 * Infrastructure Configuration Schema
 * Building systems and utilities
 */
interface InfrastructureConfig {
  // Electrical power
  power: PowerSystem;

  // HVAC system
  hvac: HVACSystem;

  // Gas supply
  gasSupply: GasSupply;

  // Backup systems
  backup: BackupSystems;

  // Network infrastructure
  networking: NetworkingConfig;
}

interface PowerSystem {
  // Main power description
  mainSupply: string;

  // Total capacity
  capacity: number;

  // Capacity unit
  unit: 'kW';

  // Redundant power feeds
  redundancy: boolean;

  // UPS configuration
  ups: UPSConfig;
}

interface UPSConfig {
  // UPS capacity (kVA)
  capacity: number;

  // Runtime in minutes
  runtime: number;

  // Number of UPS units
  units: number;
}

interface HVACSystem {
  // HVAC type description
  type: string;

  // Number of controlled zones
  zones: number;

  // Redundant systems
  redundancy: boolean;

  // Air filtration type
  filtration: string;
}

interface GasSupply {
  // Liquid nitrogen supply
  liquidNitrogen: LN2Supply;

  // CO2 supply
  co2?: GasConfig;

  // Medical gases
  medicalGases?: GasConfig[];
}

interface LN2Supply {
  // Bulk storage tank
  bulkTank: {
    capacity: number;
    unit: 'liters';
  };

  // Delivery schedule
  deliverySchedule: string;

  // Backup supply available
  backupSupply: boolean;

  // Level monitoring
  monitoring: boolean;
}

interface GasConfig {
  // Gas type
  type: string;

  // Supply method
  supply: string;

  // Backup available
  backup: boolean;
}

interface BackupSystems {
  // Generator configuration
  generator: GeneratorConfig;

  // Alternate storage facility available
  alternateStorage: boolean;

  // Disaster recovery plan reference
  disasterRecovery: string;
}

interface GeneratorConfig {
  // Generator type
  type: string;

  // Capacity (kW)
  capacity: number;

  // Fuel type
  fuelType: string;

  // Fuel capacity (gallons/liters)
  fuelCapacity: number;

  // Automatic start capability
  autoStart: boolean;

  // Testing schedule
  testingSchedule: string;
}

interface NetworkingConfig {
  // Network type
  type: string;

  // Redundant connections
  redundancy: boolean;

  // Bandwidth (Mbps)
  bandwidth: number;

  // Security measures
  security: string[];
}

// Infrastructure validator
class InfrastructureValidator {
  validatePowerRedundancy(config: InfrastructureConfig): ValidationResult {
    const errors: ValidationError[] = [];

    // Check for UPS
    if (!config.power.ups || config.power.ups.runtime < 15) {
      errors.push({
        path: 'power.ups',
        message: 'UPS with minimum 15 minutes runtime required'
      });
    }

    // Check for generator
    if (!config.backup.generator.autoStart) {
      errors.push({
        path: 'backup.generator.autoStart',
        message: 'Generator should have automatic start capability'
      });
    }

    // Check for redundant power feeds
    if (!config.power.redundancy) {
      errors.push({
        path: 'power.redundancy',
        message: 'Redundant power feeds recommended for critical facilities',
        value: false
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  validateLN2Supply(gasSupply: GasSupply): ValidationResult {
    const errors: ValidationError[] = [];

    if (!gasSupply.liquidNitrogen.monitoring) {
      errors.push({
        path: 'gasSupply.liquidNitrogen.monitoring',
        message: 'LN2 level monitoring is required'
      });
    }

    if (!gasSupply.liquidNitrogen.backupSupply) {
      errors.push({
        path: 'gasSupply.liquidNitrogen.backupSupply',
        message: 'Backup LN2 supply arrangement required'
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }
}
```

---

## Equipment Inventory

### Cryogenic Equipment Schemas

```typescript
/**
 * Equipment Inventory Schema
 * Complete equipment tracking
 */
interface EquipmentInventory {
  // Cryogenic storage equipment
  cryoStorage: CryoStorageEquipment[];

  // Processing equipment
  processing: ProcessingEquipment[];

  // Monitoring equipment
  monitoring: MonitoringEquipment[];

  // Safety equipment
  safety: SafetyEquipment[];

  // Maintenance schedules
  maintenance: MaintenanceSchedule;
}

/**
 * Cryogenic Storage Equipment Schema
 * LN2 tanks, freezers, and storage units
 */
interface CryoStorageEquipment {
  // Unique equipment identifier
  id: string;

  // Equipment type
  type: CryoEquipmentType;

  // Model name
  model: string;

  // Manufacturer
  manufacturer: string;

  // Serial number
  serialNumber: string;

  // Location (zone ID)
  location: string;

  // Specimen capacity
  capacity: number;

  // Operating temperature (°C)
  temperature: number;

  // Current status
  status: EquipmentStatus;

  // Installation information
  installation: InstallationInfo;

  // Maintenance information
  maintenance: MaintenanceInfo;

  // Monitoring configuration
  monitoring: EquipmentMonitoringConfig;
}

type CryoEquipmentType =
  | 'ln2-tank'              // Liquid nitrogen dewar
  | 'ln2-freezer'           // LN2-cooled freezer
  | 'mechanical-freezer'    // -80°C mechanical freezer
  | 'controlled-rate-freezer' // Programmable freezer
  | 'incubator';            // CO2 incubator

type EquipmentStatus =
  | 'operational'    // Normal operation
  | 'warning'        // Minor issues
  | 'alarm'          // Critical issues
  | 'maintenance'    // Under maintenance
  | 'offline'        // Not in use
  | 'decommissioned'; // Retired

interface InstallationInfo {
  // Installation date
  date: string;

  // Installer name
  installer: string;

  // Warranty information
  warranty: {
    expires: string;
    provider: string;
  };
}

interface MaintenanceInfo {
  // Last service date
  lastService: string;

  // Next service date
  nextService: string;

  // Service provider
  serviceProvider: string;

  // Service history
  history: ServiceRecord[];
}

interface ServiceRecord {
  // Service date
  date: string;

  // Service type
  type: string;

  // Technician name
  technician: string;

  // Service description
  description: string;

  // Parts replaced
  parts?: string[];
}

/**
 * Equipment Monitoring Configuration
 * Sensor and alert definitions
 */
interface EquipmentMonitoringConfig {
  // Attached sensors
  sensors: Sensor[];

  // Monitoring interval (seconds)
  interval: number;

  // Alert configurations
  alerts: AlertConfig[];
}

interface Sensor {
  // Sensor identifier
  id: string;

  // Sensor type
  type: 'temperature' | 'level' | 'pressure' | 'humidity';

  // Sensor location on equipment
  location: string;

  // Accuracy specification
  accuracy: number;

  // Calibration information
  calibration: CalibrationInfo;
}

interface CalibrationInfo {
  // Last calibration date
  lastCalibration: string;

  // Next calibration due
  nextCalibration: string;

  // Calibration certificate reference
  certificate?: string;
}

interface AlertConfig {
  // Monitored parameter
  parameter: string;

  // Threshold value
  threshold: number;

  // Alert condition
  condition: 'above' | 'below' | 'equals';

  // Alert severity
  severity: 'info' | 'warning' | 'critical';

  // Notification recipients
  recipients: string[];

  // Escalation configuration
  escalation?: EscalationConfig;
}

interface EscalationConfig {
  // Delay before escalation (minutes)
  delay: number;

  // Escalation recipients
  recipients: string[];
}

// Equipment registry service
class EquipmentRegistry {
  private equipment: Map<string, CryoStorageEquipment> = new Map();

  register(equipment: CryoStorageEquipment): void {
    if (this.equipment.has(equipment.id)) {
      throw new Error(`Equipment ${equipment.id} already registered`);
    }
    this.equipment.set(equipment.id, equipment);
  }

  findById(id: string): CryoStorageEquipment | undefined {
    return this.equipment.get(id);
  }

  findByZone(zoneId: string): CryoStorageEquipment[] {
    return Array.from(this.equipment.values())
      .filter(eq => eq.location === zoneId);
  }

  findByStatus(status: EquipmentStatus): CryoStorageEquipment[] {
    return Array.from(this.equipment.values())
      .filter(eq => eq.status === status);
  }

  findMaintenanceDue(days: number = 30): CryoStorageEquipment[] {
    const cutoff = new Date();
    cutoff.setDate(cutoff.getDate() + days);

    return Array.from(this.equipment.values())
      .filter(eq => new Date(eq.maintenance.nextService) <= cutoff);
  }

  calculateUtilization(): UtilizationReport {
    const all = Array.from(this.equipment.values());
    const operational = all.filter(eq => eq.status === 'operational');

    const totalCapacity = all.reduce((sum, eq) => sum + eq.capacity, 0);
    const operationalCapacity = operational.reduce((sum, eq) => sum + eq.capacity, 0);

    return {
      totalEquipment: all.length,
      operationalEquipment: operational.length,
      operationalPercentage: (operational.length / all.length) * 100,
      totalCapacity,
      operationalCapacity,
      capacityUtilization: (operationalCapacity / totalCapacity) * 100
    };
  }
}

interface UtilizationReport {
  totalEquipment: number;
  operationalEquipment: number;
  operationalPercentage: number;
  totalCapacity: number;
  operationalCapacity: number;
  capacityUtilization: number;
}
```

---

## Processing and Safety Equipment

### Additional Equipment Types

```typescript
/**
 * Processing Equipment Schema
 * Laboratory processing equipment
 */
interface ProcessingEquipment {
  // Equipment identifier
  id: string;

  // Equipment name
  name: string;

  // Equipment type
  type: string;

  // Manufacturer
  manufacturer: string;

  // Model
  model: string;

  // Location
  location: string;

  // Current status
  status: EquipmentStatus;

  // Processing capabilities
  capabilities: string[];
}

/**
 * Monitoring Equipment Schema
 * Environmental and security monitoring systems
 */
interface MonitoringEquipment {
  // Equipment identifier
  id: string;

  // Equipment type
  type: string;

  // Coverage zones
  coverage: string[];

  // Current status
  status: EquipmentStatus;
}

/**
 * Safety Equipment Schema
 * Emergency and safety equipment
 */
interface SafetyEquipment {
  // Equipment identifier
  id: string;

  // Equipment type
  type: SafetyEquipmentType;

  // Location description
  location: string;

  // Quantity
  quantity: number;

  // Last inspection date
  lastInspection: string;

  // Next inspection date
  nextInspection: string;

  // Current status
  status: 'ready' | 'needs-attention' | 'expired';
}

type SafetyEquipmentType =
  | 'oxygen-monitor'      // O2 depletion alarm
  | 'fire-extinguisher'   // Fire suppression
  | 'emergency-shower'    // Decontamination
  | 'eyewash'            // Eye wash station
  | 'first-aid-kit'      // First aid supplies
  | 'ppe-station'        // PPE storage
  | 'spill-kit';         // Chemical spill response

/**
 * Maintenance Schedule Schema
 * Preventive maintenance program
 */
interface MaintenanceSchedule {
  // Preventive maintenance tasks
  preventive: MaintenanceTask[];

  // Predictive maintenance enabled
  predictive: boolean;

  // Service contracts
  contracts: MaintenanceContract[];
}

interface MaintenanceTask {
  // Equipment pattern (supports wildcards)
  equipment: string;

  // Task description
  task: string;

  // Frequency
  frequency: string;

  // Responsible party
  responsible: string;

  // Documentation reference
  documentation: string;
}

interface MaintenanceContract {
  // Service provider
  provider: string;

  // Covered equipment
  scope: string[];

  // Contract start date
  start: string;

  // Contract end date
  end: string;

  // SLA terms
  sla: string;
}

// Safety equipment inspection tracker
class SafetyEquipmentTracker {
  private equipment: SafetyEquipment[] = [];

  register(item: SafetyEquipment): void {
    this.equipment.push(item);
  }

  getInspectionsDue(days: number = 30): SafetyEquipment[] {
    const cutoff = new Date();
    cutoff.setDate(cutoff.getDate() + days);

    return this.equipment.filter(item =>
      new Date(item.nextInspection) <= cutoff
    );
  }

  getExpiredItems(): SafetyEquipment[] {
    return this.equipment.filter(item => item.status === 'expired');
  }

  getByZone(zoneId: string): SafetyEquipment[] {
    return this.equipment.filter(item =>
      item.location.includes(zoneId)
    );
  }

  generateInspectionReport(): InspectionReport {
    const total = this.equipment.length;
    const ready = this.equipment.filter(e => e.status === 'ready').length;
    const needsAttention = this.equipment.filter(e => e.status === 'needs-attention').length;
    const expired = this.equipment.filter(e => e.status === 'expired').length;

    return {
      totalItems: total,
      readyItems: ready,
      needsAttentionItems: needsAttention,
      expiredItems: expired,
      complianceRate: (ready / total) * 100,
      actionRequired: needsAttention + expired > 0,
      items: this.equipment
    };
  }
}

interface InspectionReport {
  totalItems: number;
  readyItems: number;
  needsAttentionItems: number;
  expiredItems: number;
  complianceRate: number;
  actionRequired: boolean;
  items: SafetyEquipment[];
}
```

---

## Operational Procedures

### SOP and Workflow Schemas

```typescript
/**
 * Operational Procedures Schema
 * SOPs, workflows, and training
 */
interface OperationalProcedures {
  // Standard Operating Procedures
  sops: StandardProcedure[];

  // Process workflows
  workflows: Workflow[];

  // Training program
  training: TrainingProgram;

  // Documentation system
  documentation: DocumentationSystem;
}

/**
 * Standard Operating Procedure Schema
 */
interface StandardProcedure {
  // SOP identifier
  id: string;

  // SOP title
  title: string;

  // Version number
  version: string;

  // Category
  category: string;

  // Scope description
  scope: string;

  // Procedure steps
  steps: ProcedureStep[];

  // Approval information
  approvedBy: string;
  approvedAt: string;

  // Review date
  reviewDate: string;
}

interface ProcedureStep {
  // Step number
  number: number;

  // Action description
  action: string;

  // Responsible role
  responsibility: string;

  // Documentation requirements
  documentation?: string;

  // Critical step flag
  criticalStep: boolean;
}

/**
 * Workflow Schema
 */
interface Workflow {
  // Workflow identifier
  id: string;

  // Workflow name
  name: string;

  // Workflow type
  type: string;

  // Step references
  steps: string[];

  // Automation percentage
  automation: number;
}

/**
 * Training Program Schema
 */
interface TrainingProgram {
  // Training requirements
  requirements: TrainingRequirement[];

  // Record keeping enabled
  records: boolean;

  // Refresh training interval
  refreshInterval: string;

  // Competency testing
  competencyAssessment: boolean;
}

interface TrainingRequirement {
  // Training topic
  topic: string;

  // Training frequency
  frequency: string;

  // Mandatory flag
  mandatory: boolean;

  // Applicable roles
  roles: string[];
}

/**
 * Documentation System Schema
 */
interface DocumentationSystem {
  // Document format
  format: string;

  // Storage location
  storage: string;

  // Retention period
  retention: string;

  // Version control enabled
  version: boolean;

  // Audit trail enabled
  audit: boolean;
}

// SOP management service
class SOPManager {
  private sops: Map<string, StandardProcedure> = new Map();

  register(sop: StandardProcedure): void {
    this.sops.set(sop.id, sop);
  }

  findById(id: string): StandardProcedure | undefined {
    return this.sops.get(id);
  }

  findByCategory(category: string): StandardProcedure[] {
    return Array.from(this.sops.values())
      .filter(sop => sop.category === category);
  }

  findReviewDue(days: number = 30): StandardProcedure[] {
    const cutoff = new Date();
    cutoff.setDate(cutoff.getDate() + days);

    return Array.from(this.sops.values())
      .filter(sop => new Date(sop.reviewDate) <= cutoff);
  }

  getCriticalSteps(sopId: string): ProcedureStep[] {
    const sop = this.sops.get(sopId);
    if (!sop) return [];
    return sop.steps.filter(step => step.criticalStep);
  }

  generateComplianceReport(): SOPComplianceReport {
    const all = Array.from(this.sops.values());
    const current = all.filter(sop =>
      new Date(sop.reviewDate) > new Date()
    );

    return {
      totalSOPs: all.length,
      currentSOPs: current.length,
      overdueSOPs: all.length - current.length,
      complianceRate: (current.length / all.length) * 100,
      reviewSchedule: this.createReviewSchedule(all)
    };
  }

  private createReviewSchedule(
    sops: StandardProcedure[]
  ): { month: string; count: number }[] {
    const schedule: Map<string, number> = new Map();

    for (const sop of sops) {
      const month = sop.reviewDate.substring(0, 7); // YYYY-MM
      schedule.set(month, (schedule.get(month) || 0) + 1);
    }

    return Array.from(schedule.entries())
      .map(([month, count]) => ({ month, count }))
      .sort((a, b) => a.month.localeCompare(b.month));
  }
}

interface SOPComplianceReport {
  totalSOPs: number;
  currentSOPs: number;
  overdueSOPs: number;
  complianceRate: number;
  reviewSchedule: { month: string; count: number }[];
}
```

---

## Safety and Quality Management

### Safety Management Schemas

```typescript
/**
 * Safety Management Schema
 * Comprehensive safety system
 */
interface SafetyManagement {
  // Safety policies
  policies: SafetyPolicy[];

  // Hazard assessments
  hazards: HazardAssessment[];

  // Emergency procedures
  emergency: EmergencyProcedures;

  // Incident management
  incidents: IncidentManagement;

  // PPE requirements
  ppe: PPERequirements;
}

interface SafetyPolicy {
  // Policy identifier
  id: string;

  // Policy title
  title: string;

  // Scope description
  scope: string;

  // Requirements
  requirements: string[];

  // Review schedule
  review: string;
}

interface HazardAssessment {
  // Hazard description
  hazard: string;

  // Locations where hazard exists
  location: string[];

  // Risk level
  risk: 'low' | 'medium' | 'high';

  // Control measures
  controls: string[];

  // Monitoring method
  monitoring: string;
}

interface EmergencyProcedures {
  // Emergency contacts
  contacts: EmergencyContact[];

  // Emergency procedures
  procedures: EmergencyProcedure[];

  // Drill schedule
  drills: DrillSchedule;

  // Emergency equipment
  equipment: string[];
}

interface EmergencyContact {
  // Role title
  role: string;

  // Contact name
  name: string;

  // Phone number
  phone: string;

  // Availability
  available: string;
}

interface EmergencyProcedure {
  // Emergency scenario
  scenario: string;

  // Response steps
  steps: string[];

  // Responsible party
  responsible: string;

  // Notification requirements
  notification: string[];
}

interface DrillSchedule {
  // Drill type
  type: string;

  // Frequency
  frequency: string;

  // Last drill date
  lastDrill: string;

  // Next drill date
  nextDrill: string;
}

interface IncidentManagement {
  // Reporting procedure
  reporting: string;

  // Investigation process
  investigation: string;

  // Corrective action process
  corrective: string;

  // Tracking system
  tracking: boolean;
}

interface PPERequirements {
  // Zone-specific requirements
  zones: { zone: string; required: string[] }[];

  // PPE training required
  training: boolean;

  // Inspection frequency
  inspection: string;
}

/**
 * Quality Management Schema
 */
interface QualityManagement {
  // Quality system standard
  system: string;

  // Audit program
  audits: AuditProgram;

  // Deviation management
  deviations: DeviationManagement;

  // CAPA system
  capa: CAPASystem;
}

interface AuditProgram {
  // Internal audits
  internal: {
    frequency: string;
    scope: string[];
  };

  // External audits
  external: {
    frequency: string;
    bodies: string[];
  };

  // Audit tracking
  tracking: boolean;
}

interface DeviationManagement {
  // Deviation categories
  categories: string[];

  // Investigation timeline
  investigation: string;

  // Resolution timeline
  timeline: string;
}

interface CAPASystem {
  // CAPA enabled
  enabled: boolean;

  // Workflow type
  workflow: string;

  // Tracking system
  tracking: boolean;

  // Effectiveness verification timeline
  effectiveness: string;
}

// Safety compliance calculator
class SafetyComplianceCalculator {
  calculateHazardRisk(
    hazards: HazardAssessment[]
  ): HazardRiskSummary {
    const high = hazards.filter(h => h.risk === 'high').length;
    const medium = hazards.filter(h => h.risk === 'medium').length;
    const low = hazards.filter(h => h.risk === 'low').length;

    const riskScore = (high * 3 + medium * 2 + low * 1) / hazards.length;

    return {
      totalHazards: hazards.length,
      highRisk: high,
      mediumRisk: medium,
      lowRisk: low,
      overallRiskScore: riskScore,
      riskLevel: riskScore > 2 ? 'high' : riskScore > 1.5 ? 'medium' : 'low'
    };
  }

  validatePPECoverage(
    zones: FacilityZone[],
    ppeRequirements: PPERequirements
  ): PPECoverageReport {
    const covered = zones.filter(zone =>
      ppeRequirements.zones.some(req => req.zone === zone.id)
    );

    return {
      totalZones: zones.length,
      coveredZones: covered.length,
      uncoveredZones: zones.length - covered.length,
      coverageRate: (covered.length / zones.length) * 100,
      missingCoverage: zones
        .filter(zone => !ppeRequirements.zones.some(req => req.zone === zone.id))
        .map(z => z.id)
    };
  }
}

interface HazardRiskSummary {
  totalHazards: number;
  highRisk: number;
  mediumRisk: number;
  lowRisk: number;
  overallRiskScore: number;
  riskLevel: 'low' | 'medium' | 'high';
}

interface PPECoverageReport {
  totalZones: number;
  coveredZones: number;
  uncoveredZones: number;
  coverageRate: number;
  missingCoverage: string[];
}
```

---

## Environmental and Monitoring Systems

### Environmental Control Schemas

```typescript
/**
 * Environmental Control Schema
 * Environmental monitoring and control
 */
interface EnvironmentalControl {
  // Monitoring configuration
  monitoring: EnvironmentalMonitoring;

  // Control systems
  controls: EnvironmentalControls;

  // Alert configuration
  alerts: EnvironmentalAlerts;
}

interface EnvironmentalMonitoring {
  // Monitored parameters
  parameters: MonitoredParameter[];

  // Monitoring frequency
  frequency: string;

  // Data logging enabled
  logging: boolean;
}

interface MonitoredParameter {
  // Parameter name
  name: string;

  // Unit of measurement
  unit: string;

  // Acceptable range
  range: {
    min: number;
    max: number;
  };

  // Monitoring locations
  locations: string[];
}

interface EnvironmentalControls {
  // HVAC control
  hvac: boolean;

  // Humidity control
  humidification: boolean;

  // Air filtration
  filtration: string;

  // Pressure control
  pressurization: boolean;
}

interface EnvironmentalAlerts {
  // Alerting enabled
  enabled: boolean;

  // Alert thresholds
  thresholds: {
    parameter: string;
    warning: number;
    critical: number;
  }[];

  // Notification channels
  notifications: string[];
}

/**
 * Monitoring System Schema
 * Integrated monitoring platform
 */
interface MonitoringSystem {
  // Real-time monitoring
  realtime: RealtimeConfig;

  // Alert system
  alerts: AlertSystem;

  // Reporting configuration
  reporting: ReportingConfig;

  // Integration configuration
  integration: IntegrationConfig;
}

interface RealtimeConfig {
  // Real-time enabled
  enabled: boolean;

  // Dashboard type
  dashboard: string;

  // Refresh interval (seconds)
  refresh: number;
}

interface AlertSystem {
  // Alert channels
  channels: string[];

  // Escalation configuration
  escalation: {
    levels: number;
    timeout: number;  // minutes
  };

  // Acknowledgement requirements
  acknowledgement: string;
}

interface ReportingConfig {
  // Automated reports
  automated: {
    type: string;
    frequency: string;
    recipients: string[];
  }[];

  // On-demand report types
  onDemand: string[];
}

interface IntegrationConfig {
  // LIMS integration
  lims: boolean;

  // Building management system
  buildingManagement: boolean;

  // External integrations
  external: string[];
}

// Environmental monitoring service
class EnvironmentalMonitoringService {
  private readings: Map<string, EnvironmentalReading[]> = new Map();

  recordReading(
    parameterId: string,
    value: number,
    location: string
  ): void {
    const reading: EnvironmentalReading = {
      parameterId,
      value,
      location,
      timestamp: new Date().toISOString()
    };

    const existing = this.readings.get(parameterId) || [];
    existing.push(reading);
    this.readings.set(parameterId, existing);
  }

  checkThresholds(
    alerts: EnvironmentalAlerts
  ): ThresholdAlert[] {
    const violations: ThresholdAlert[] = [];

    for (const threshold of alerts.thresholds) {
      const readings = this.readings.get(threshold.parameter) || [];
      const latest = readings[readings.length - 1];

      if (latest) {
        if (latest.value >= threshold.critical) {
          violations.push({
            parameter: threshold.parameter,
            value: latest.value,
            threshold: threshold.critical,
            severity: 'critical',
            timestamp: latest.timestamp
          });
        } else if (latest.value >= threshold.warning) {
          violations.push({
            parameter: threshold.parameter,
            value: latest.value,
            threshold: threshold.warning,
            severity: 'warning',
            timestamp: latest.timestamp
          });
        }
      }
    }

    return violations;
  }

  getHistoricalData(
    parameterId: string,
    startTime: string,
    endTime: string
  ): EnvironmentalReading[] {
    const readings = this.readings.get(parameterId) || [];
    return readings.filter(r =>
      r.timestamp >= startTime && r.timestamp <= endTime
    );
  }

  calculateStatistics(parameterId: string): ParameterStatistics {
    const readings = this.readings.get(parameterId) || [];
    if (readings.length === 0) {
      throw new Error(`No readings for parameter: ${parameterId}`);
    }

    const values = readings.map(r => r.value);
    const sum = values.reduce((a, b) => a + b, 0);
    const mean = sum / values.length;
    const min = Math.min(...values);
    const max = Math.max(...values);

    const squaredDiffs = values.map(v => Math.pow(v - mean, 2));
    const variance = squaredDiffs.reduce((a, b) => a + b, 0) / values.length;
    const stdDev = Math.sqrt(variance);

    return {
      parameterId,
      count: readings.length,
      mean,
      min,
      max,
      standardDeviation: stdDev,
      range: max - min
    };
  }
}

interface EnvironmentalReading {
  parameterId: string;
  value: number;
  location: string;
  timestamp: string;
}

interface ThresholdAlert {
  parameter: string;
  value: number;
  threshold: number;
  severity: 'warning' | 'critical';
  timestamp: string;
}

interface ParameterStatistics {
  parameterId: string;
  count: number;
  mean: number;
  min: number;
  max: number;
  standardDeviation: number;
  range: number;
}
```

---

## Data Exchange Formats

### JSON Schema Definitions

```typescript
/**
 * JSON Schema for WIA Cryo Facility Project
 * Validation and interoperability
 */

const WIACryoFacilitySchema = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  $id: 'https://wia.org/schemas/cryo-facility/v1.0.0/project.json',
  title: 'WIA Cryo Facility Project',
  description: 'Complete cryogenic facility configuration',
  type: 'object',
  required: ['standard', 'version', 'metadata', 'facility', 'equipment'],
  properties: {
    standard: {
      type: 'string',
      const: 'WIA-CRYO-FACILITY'
    },
    version: {
      type: 'string',
      pattern: '^\\d+\\.\\d+\\.\\d+$'
    },
    metadata: {
      $ref: '#/definitions/ProjectMetadata'
    },
    facility: {
      $ref: '#/definitions/FacilityConfiguration'
    },
    equipment: {
      $ref: '#/definitions/EquipmentInventory'
    }
  },
  definitions: {
    ProjectMetadata: {
      type: 'object',
      required: ['id', 'name', 'type', 'location', 'organization', 'status'],
      properties: {
        id: { type: 'string' },
        name: { type: 'string' },
        type: {
          type: 'string',
          enum: ['biobank', 'tissue-bank', 'fertility-center',
                 'research-facility', 'hospital-unit', 'commercial-storage']
        },
        location: { $ref: '#/definitions/FacilityLocation' },
        organization: { $ref: '#/definitions/Organization' },
        status: {
          type: 'string',
          enum: ['operational', 'limited-operations', 'maintenance',
                 'emergency', 'offline']
        }
      }
    },
    FacilityLocation: {
      type: 'object',
      required: ['address', 'city', 'country', 'postalCode', 'coordinates', 'timezone'],
      properties: {
        address: { type: 'string' },
        city: { type: 'string' },
        state: { type: 'string' },
        country: { type: 'string' },
        postalCode: { type: 'string' },
        coordinates: {
          type: 'object',
          properties: {
            latitude: { type: 'number', minimum: -90, maximum: 90 },
            longitude: { type: 'number', minimum: -180, maximum: 180 }
          }
        },
        timezone: { type: 'string' }
      }
    },
    Organization: {
      type: 'object',
      required: ['name', 'type', 'contact'],
      properties: {
        name: { type: 'string' },
        type: { type: 'string' },
        registrationNumber: { type: 'string' },
        contact: { $ref: '#/definitions/ContactInfo' }
      }
    },
    ContactInfo: {
      type: 'object',
      required: ['name', 'email', 'phone'],
      properties: {
        name: { type: 'string' },
        email: { type: 'string', format: 'email' },
        phone: { type: 'string' },
        emergencyPhone: { type: 'string' }
      }
    }
  }
};

// Schema validation service
class SchemaValidator {
  private schema: object;

  constructor(schema: object) {
    this.schema = schema;
  }

  validate(data: unknown): ValidationResult {
    // Simplified validation - production should use a JSON Schema library
    const errors: ValidationError[] = [];

    if (typeof data !== 'object' || data === null) {
      errors.push({
        path: '$',
        message: 'Root must be an object'
      });
      return { valid: false, errors };
    }

    const project = data as Partial<WIACryoFacilityProject>;

    if (project.standard !== 'WIA-CRYO-FACILITY') {
      errors.push({
        path: 'standard',
        message: 'Invalid standard identifier',
        value: project.standard
      });
    }

    if (!project.version || !/^\d+\.\d+\.\d+$/.test(project.version)) {
      errors.push({
        path: 'version',
        message: 'Version must be in semver format (x.y.z)',
        value: project.version
      });
    }

    if (!project.metadata) {
      errors.push({
        path: 'metadata',
        message: 'Metadata is required'
      });
    }

    if (!project.facility) {
      errors.push({
        path: 'facility',
        message: 'Facility configuration is required'
      });
    }

    if (!project.equipment) {
      errors.push({
        path: 'equipment',
        message: 'Equipment inventory is required'
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }
}
```

---

## Chapter Summary

This chapter defined the complete data architecture for the WIA Cryo Facility Standard:

- **Core Schemas**: Project structure, metadata, and facility identification
- **Facility Configuration**: Layout, zones, capacity, and infrastructure
- **Equipment Inventory**: Storage equipment, sensors, and maintenance
- **Operational Procedures**: SOPs, workflows, and training
- **Safety Management**: Hazards, emergencies, and PPE
- **Quality Management**: Audits, deviations, and CAPA
- **Environmental Control**: Monitoring, controls, and alerts
- **Data Exchange**: JSON schemas for interoperability

---

*© 2025 World Industry Association. All rights reserved.*

*弘益人間 (Benefit All Humanity)*
