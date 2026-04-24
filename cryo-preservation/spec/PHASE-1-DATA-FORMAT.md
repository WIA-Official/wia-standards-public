# Phase 1: Data Format Specification

## WIA-CRYO-PRESERVATION Data Structures

> Standardized data formats for cryopreservation records and monitoring.

---

## 1. Core Data Types

### 1.1 Temperature Units

```typescript
type TemperatureUnit = "CELSIUS" | "KELVIN" | "FAHRENHEIT";

interface Temperature {
    value: number;
    unit: TemperatureUnit;
    precision: number;  // decimal places
    timestamp: ISO8601DateTime;
    sensorId: string;
}

// Standard: Always store in Kelvin internally
// Display: Convert to user preference
// Precision: Minimum 0.1°C for cryogenic ranges
```

### 1.2 Pressure Units

```typescript
type PressureUnit = "ATM" | "BAR" | "PSI" | "KPA" | "MMHG";

interface Pressure {
    value: number;
    unit: PressureUnit;
    timestamp: ISO8601DateTime;
    sensorId: string;
}
```

### 1.3 Time Formats

```typescript
type ISO8601DateTime = string;  // "2024-01-15T10:30:00Z"
type Duration = string;         // ISO 8601 duration "P1Y2M3DT4H5M6S"

interface TimeRange {
    start: ISO8601DateTime;
    end: ISO8601DateTime;
    duration: Duration;
}
```

---

## 2. Subject Identification

### 2.1 Subject Document

```typescript
interface SubjectDocument {
    // Identity
    id: SubjectId;
    did: DID;  // "did:wia:cryo:subject:{uuid}"
    type: SubjectType;

    // Biological Information
    species: SpeciesCode;
    biologicalAge: number;  // years at preservation
    chronologicalAge: number;
    sex: BiologicalSex;
    bloodType: BloodType;

    // Biometrics (pre-preservation)
    biometrics: BiometricData;

    // Medical History
    medicalHistory: MedicalRecord[];
    allergies: Allergy[];
    medications: Medication[];

    // Preservation Details
    preservationDate: ISO8601DateTime;
    preservationFacility: FacilityId;
    preservationMethod: PreservationMethod;
    preservationTeam: TeamMember[];

    // Legal
    consentDocument: ConsentReference;
    legalStatus: LegalStatus;

    // Metadata
    createdAt: ISO8601DateTime;
    updatedAt: ISO8601DateTime;
    version: string;
}

type SubjectType =
    | "WHOLE_BODY"
    | "NEURO"  // Head/brain only
    | "ORGAN"
    | "TISSUE"
    | "CELL_CULTURE"
    | "EMBRYO"
    | "GAMETE";

type SpeciesCode = "HUMAN" | "CANINE" | "FELINE" | "OTHER";

type BiologicalSex = "MALE" | "FEMALE" | "INTERSEX" | "UNKNOWN";

interface BloodType {
    abo: "A" | "B" | "AB" | "O";
    rh: "POSITIVE" | "NEGATIVE";
    additional?: string[];  // Kell, Duffy, etc.
}
```

### 2.2 Biometric Data

```typescript
interface BiometricData {
    // Physical Measurements
    height: Measurement;  // cm
    weight: Measurement;  // kg
    bmi: number;

    // DNA
    dnaProfile: DNAProfile;

    // Fingerprints
    fingerprints?: FingerprintSet;

    // Facial
    facialGeometry?: FacialData;

    // Iris
    irisPatterns?: IrisData;

    // Dental
    dentalRecords?: DentalData;

    // Medical Imaging
    scans?: MedicalScan[];
}

interface DNAProfile {
    sampleId: string;
    collectionDate: ISO8601DateTime;
    sequenceType: "FULL_GENOME" | "EXOME" | "SNP_PANEL" | "STR_PROFILE";
    dataReference: DataReference;
    hash: string;  // SHA-256 of sequence data
}

interface Measurement {
    value: number;
    unit: string;
    timestamp: ISO8601DateTime;
    method: string;
    operator?: string;
}
```

---

## 3. Preservation Protocol Data

### 3.1 Protocol Document

```typescript
interface PreservationProtocol {
    id: ProtocolId;
    version: string;
    name: string;
    description: string;

    // Target specifications
    targetSubjectType: SubjectType[];
    targetSpecies: SpeciesCode[];

    // Phases
    phases: ProtocolPhase[];

    // Cryoprotectants
    cpaProtocol: CPAProtocol;

    // Temperature profile
    coolingProfile: CoolingProfile;

    // Quality requirements
    qualityRequirements: QualityRequirements;

    // Safety
    safetyChecklist: SafetyCheck[];

    // Validation
    validatedBy: Certification[];
    lastValidation: ISO8601DateTime;
}

interface ProtocolPhase {
    id: string;
    name: string;
    order: number;

    // Timing
    startCondition: PhaseCondition;
    duration: Duration;
    endCondition: PhaseCondition;

    // Parameters
    targetTemperature: TemperatureRange;
    targetPressure?: PressureRange;
    cpaConcentration?: ConcentrationRange;

    // Actions
    actions: ProtocolAction[];

    // Monitoring
    monitoringInterval: Duration;
    criticalParameters: string[];

    // Alerts
    alertThresholds: AlertThreshold[];
}

interface PhaseCondition {
    type: "TIME" | "TEMPERATURE" | "CPA_LEVEL" | "MANUAL" | "SENSOR";
    parameter?: string;
    operator?: "EQ" | "LT" | "GT" | "LTE" | "GTE";
    value?: number;
    unit?: string;
}
```

### 3.2 Cryoprotectant Protocol

```typescript
interface CPAProtocol {
    // Solution composition
    solutions: CPASolution[];

    // Loading sequence
    loadingSteps: CPALoadingStep[];

    // Timing
    totalLoadingTime: Duration;

    // Temperature during loading
    loadingTemperature: Temperature;

    // Perfusion parameters
    perfusionPressure?: Pressure;
    perfusionRate?: number;  // mL/min
}

interface CPASolution {
    id: string;
    name: string;

    // Components
    components: CPAComponent[];

    // Properties
    osmolality: number;  // mOsm/kg
    viscosity: number;   // cP at loading temp
    glassTransitionTemp: Temperature;  // Tg

    // Preparation
    preparationSteps: string[];
    storageConditions: StorageCondition;
    shelfLife: Duration;
}

interface CPAComponent {
    name: string;
    type: CPAType;
    concentration: number;
    unit: ConcentrationUnit;

    // Properties
    molecularWeight: number;
    penetrating: boolean;
    toxicityIndex: number;  // 0-10 scale
}

type CPAType =
    | "DMSO"
    | "GLYCEROL"
    | "ETHYLENE_GLYCOL"
    | "PROPYLENE_GLYCOL"
    | "FORMAMIDE"
    | "TREHALOSE"
    | "SUCROSE"
    | "PVP"
    | "HES"
    | "OTHER";

type ConcentrationUnit = "PERCENT_V" | "PERCENT_W" | "MOLAR" | "MILLIMOLAR";
```

### 3.3 Cooling Profile

```typescript
interface CoolingProfile {
    // Overall parameters
    startTemperature: Temperature;
    targetTemperature: Temperature;
    totalDuration: Duration;

    // Segments
    segments: CoolingSegment[];

    // Critical points
    criticalPoints: CriticalPoint[];

    // Monitoring
    monitoringFrequency: Duration;
    tolerances: CoolingTolerance;
}

interface CoolingSegment {
    id: string;
    order: number;

    // Temperature range
    startTemp: Temperature;
    endTemp: Temperature;

    // Rate
    coolingRate: number;  // °C/min (negative for cooling)
    rateType: "LINEAR" | "EXPONENTIAL" | "STEP" | "CONTROLLED";

    // Duration
    duration: Duration;

    // Control
    controlMethod: "PROGRAMMED" | "PASSIVE" | "MANUAL";
    controllerSettings?: ControllerSettings;
}

interface CriticalPoint {
    name: string;
    temperature: Temperature;
    description: string;
    action: string;

    // Ice nucleation
    nucleationRisk?: boolean;

    // Glass transition
    glassTransition?: boolean;
}

// Common critical points:
// - 0°C: Freezing point (avoid slow crossing)
// - -40°C: Homogeneous nucleation temperature
// - -130°C: Glass transition (Tg) for most CPA solutions
// - -196°C: Liquid nitrogen temperature
```

---

## 4. Storage Records

### 4.1 Storage Container

```typescript
interface StorageContainer {
    id: ContainerId;
    type: ContainerType;

    // Physical location
    facility: FacilityId;
    location: StorageLocation;

    // Specifications
    capacity: number;  // liters LN2
    dimensions: Dimensions;
    material: string;
    manufacturer: string;
    model: string;
    serialNumber: string;

    // Current state
    currentLN2Level: number;  // percentage
    currentTemperature: Temperature;
    currentPressure?: Pressure;

    // Contents
    contents: StorageSlot[];

    // Maintenance
    installationDate: ISO8601DateTime;
    lastMaintenance: ISO8601DateTime;
    nextMaintenance: ISO8601DateTime;

    // Monitoring
    sensors: SensorConfiguration[];
    alarmSettings: AlarmSettings;
}

type ContainerType =
    | "DEWAR_SMALL"      // < 50L
    | "DEWAR_MEDIUM"     // 50-200L
    | "DEWAR_LARGE"      // 200-500L
    | "DEWAR_INDUSTRIAL" // > 500L
    | "MVE_CRYOSYSTEM"
    | "CUSTOM";

interface StorageLocation {
    building: string;
    room: string;
    row?: string;
    position?: string;
    gpsCoordinates?: GPSCoordinates;
}

interface StorageSlot {
    slotId: string;
    position: SlotPosition;

    // Content
    subjectId?: SubjectId;
    subjectType?: SubjectType;

    // Container within dewar
    innerContainer?: InnerContainer;

    // Status
    occupied: boolean;
    reservedFor?: SubjectId;

    // History
    occupancyHistory: OccupancyRecord[];
}

interface SlotPosition {
    canister?: number;
    level?: number;
    position?: number;
    customId?: string;
}
```

### 4.2 Monitoring Data

```typescript
interface MonitoringRecord {
    containerId: ContainerId;
    timestamp: ISO8601DateTime;

    // Measurements
    temperature: Temperature;
    ln2Level: number;  // percentage
    pressure?: Pressure;
    humidity?: number;

    // Sensor health
    sensorStatus: SensorStatus[];

    // Calculated metrics
    estimatedHoldTime: Duration;  // time until LN2 exhaustion
    evaporationRate: number;      // liters/day

    // Alerts
    activeAlerts: Alert[];

    // Quality
    dataQuality: DataQuality;
}

interface SensorStatus {
    sensorId: string;
    type: SensorType;
    status: "ONLINE" | "OFFLINE" | "ERROR" | "CALIBRATING";
    lastReading: ISO8601DateTime;
    batteryLevel?: number;
    calibrationDate?: ISO8601DateTime;
}

type SensorType =
    | "TEMPERATURE_PT100"
    | "TEMPERATURE_THERMOCOUPLE"
    | "LN2_LEVEL_CAPACITIVE"
    | "LN2_LEVEL_RESISTIVE"
    | "LN2_LEVEL_WEIGHT"
    | "PRESSURE"
    | "HUMIDITY"
    | "VIBRATION"
    | "DOOR_SENSOR";
```

---

## 5. Quality Assessment

### 5.1 Vitrification Quality Index

```typescript
interface VQIAssessment {
    subjectId: SubjectId;
    assessmentDate: ISO8601DateTime;
    assessor: PersonId;

    // Component scores (0.0 - 1.0)
    iceFraction: number;
    cpaDistribution: number;
    coolingRateScore: number;
    integrityScore: number;

    // Calculated VQI
    vqi: number;
    vqiGrade: VQIGrade;

    // Detailed findings
    findings: Finding[];

    // Recommendations
    recommendations: string[];

    // Supporting data
    imagingData?: ImagingReference[];
    sampleAnalysis?: SampleAnalysis[];
}

type VQIGrade =
    | "EXCELLENT"    // VQI >= 0.95
    | "GOOD"         // VQI >= 0.85
    | "ACCEPTABLE"   // VQI >= 0.70
    | "MARGINAL"     // VQI >= 0.50
    | "POOR"         // VQI >= 0.30
    | "CRITICAL";    // VQI < 0.30

interface Finding {
    category: FindingCategory;
    severity: Severity;
    location?: string;
    description: string;
    evidence?: string;
    impactOnRevival: string;
}

type FindingCategory =
    | "ICE_DAMAGE"
    | "CPA_TOXICITY"
    | "THERMAL_STRESS"
    | "STRUCTURAL_DAMAGE"
    | "PERFUSION_ISSUE"
    | "EQUIPMENT_FAILURE"
    | "PROTOCOL_DEVIATION"
    | "OTHER";

type Severity = "INFO" | "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";
```

### 5.2 Sample Analysis

```typescript
interface SampleAnalysis {
    sampleId: string;
    subjectId: SubjectId;
    sampleType: SampleType;
    collectionTime: ISO8601DateTime;
    analysisTime: ISO8601DateTime;

    // Analysis results
    cellViability?: CellViabilityResult;
    iceContent?: IceContentResult;
    cpaConcentration?: CPAConcentrationResult;
    structuralIntegrity?: StructuralResult;

    // Method details
    analysisMethod: string;
    equipment: string[];
    operator: PersonId;

    // Raw data reference
    rawDataRef: DataReference;
}

type SampleType =
    | "TISSUE_BIOPSY"
    | "PERFUSATE"
    | "SURFACE_SWAB"
    | "IMAGING"
    | "OTHER";

interface CellViabilityResult {
    method: "TRYPAN_BLUE" | "MTT" | "FLOW_CYTOMETRY" | "OTHER";
    viabilityPercent: number;
    cellCount?: number;
    confidence: number;
}

interface IceContentResult {
    method: "DSC" | "CRYO_SEM" | "X_RAY" | "MRI" | "OTHER";
    iceVolumePercent: number;
    iceCrystalSize?: number;  // micrometers
    distribution: "UNIFORM" | "LOCALIZED" | "NONE_DETECTED";
}
```

---

## 6. Event Logging

### 6.1 Preservation Event

```typescript
interface PreservationEvent {
    eventId: string;
    subjectId: SubjectId;
    timestamp: ISO8601DateTime;

    eventType: PreservationEventType;
    phase: ProtocolPhase;

    // Details
    description: string;
    parameters: Record<string, any>;

    // Personnel
    performedBy: PersonId[];
    witnessedBy?: PersonId[];

    // Outcome
    outcome: EventOutcome;
    deviations?: Deviation[];

    // Documentation
    notes?: string;
    attachments?: AttachmentReference[];
}

type PreservationEventType =
    | "CONSENT_OBTAINED"
    | "STANDBY_INITIATED"
    | "LEGAL_DEATH_DECLARED"
    | "STABILIZATION_START"
    | "BLOOD_WASHOUT"
    | "CPA_PERFUSION_START"
    | "CPA_PERFUSION_COMPLETE"
    | "COOLING_START"
    | "GLASS_TRANSITION_REACHED"
    | "TARGET_TEMP_REACHED"
    | "TRANSFER_TO_STORAGE"
    | "STORAGE_CONFIRMED"
    | "MONITORING_CHECK"
    | "MAINTENANCE"
    | "INCIDENT"
    | "TRANSFER_OUT"
    | "TRANSFER_IN";

type EventOutcome = "SUCCESS" | "PARTIAL" | "FAILED" | "ABORTED";

interface Deviation {
    deviationId: string;
    severity: Severity;
    description: string;
    rootCause?: string;
    correctiveAction?: string;
    impact: string;
}
```

---

## 7. Data Exchange Formats

### 7.1 Standard Document Container

```typescript
interface CryoDocument {
    "@context": "https://wia.org/contexts/cryo-preservation/v1";
    id: string;
    type: CryoDocumentType;
    version: string;

    // Content
    payload: SubjectDocument | PreservationProtocol | MonitoringRecord | VQIAssessment;

    // Metadata
    createdAt: ISO8601DateTime;
    createdBy: DID;
    facility: FacilityId;

    // Integrity
    hash: string;
    signature: DigitalSignature;

    // Chain of custody
    custody: CustodyRecord[];
}

type CryoDocumentType =
    | "SUBJECT_RECORD"
    | "PROTOCOL"
    | "MONITORING"
    | "QUALITY_ASSESSMENT"
    | "EVENT_LOG"
    | "TRANSFER_MANIFEST"
    | "FACILITY_CERTIFICATION";
```

### 7.2 Transfer Manifest

```typescript
interface TransferManifest {
    manifestId: string;
    type: "TRANSFER_MANIFEST";

    // Transfer details
    transferId: string;
    transferType: "FACILITY_TO_FACILITY" | "INTERNAL" | "EMERGENCY";

    // Subjects being transferred
    subjects: SubjectTransferRecord[];

    // Origin
    originFacility: FacilityId;
    originContainer: ContainerId;
    departureTime: ISO8601DateTime;
    departureConditions: EnvironmentalConditions;

    // Destination
    destinationFacility: FacilityId;
    destinationContainer: ContainerId;
    expectedArrival: ISO8601DateTime;

    // Transport
    transportMethod: TransportMethod;
    transportContainer: TransportContainerSpec;
    route?: TransportRoute;

    // Chain of custody
    custodyTransfers: CustodyTransfer[];

    // Monitoring during transport
    transitMonitoring: TransitMonitoringConfig;

    // Authorization
    authorizedBy: PersonId;
    authorization: AuthorizationDocument;

    // Signatures
    originSignature: DigitalSignature;
    destinationSignature?: DigitalSignature;
}

interface TransportContainerSpec {
    type: "DRY_SHIPPER" | "LN2_DEWAR" | "CONTROLLED_RATE";
    model: string;
    serialNumber: string;
    capacity: number;
    holdTime: Duration;  // Guaranteed hold time
    currentCharge: number;  // LN2 level or hours remaining
}
```

---

## 8. Enumerations Reference

### 8.1 Status Codes

```typescript
enum PreservationStatus {
    STANDBY = "STANDBY",
    IN_PROGRESS = "IN_PROGRESS",
    COMPLETED = "COMPLETED",
    STORED = "STORED",
    IN_TRANSIT = "IN_TRANSIT",
    REVIVING = "REVIVING",
    REVIVED = "REVIVED",
    TERMINATED = "TERMINATED"
}

enum AlertLevel {
    INFO = "INFO",
    WARNING = "WARNING",
    CRITICAL = "CRITICAL",
    EMERGENCY = "EMERGENCY"
}

enum FacilityStatus {
    OPERATIONAL = "OPERATIONAL",
    MAINTENANCE = "MAINTENANCE",
    LIMITED = "LIMITED",
    EMERGENCY = "EMERGENCY",
    OFFLINE = "OFFLINE"
}
```

---

**Phase 1 Data Format Specification**
**WIA-CRYO-PRESERVATION v1.0.0**
