# WIA-MICROPLASTIC_DETECTION
## PHASE 1: Data Format Specification v1.0

**Status**: FULL Implementation
**Philosophy**: 弘益人間 (Benefit All Humanity)
**Last Updated**: 2026-01-12

---

## 1. Overview

This specification defines the standardized data formats for microplastic detection, analysis, and reporting. It encompasses particle characterization, spectroscopic data, sample metadata, polymer identification, and environmental context.

### 1.1 Design Principles

- **Interoperability**: Seamless data exchange between detection systems
- **Traceability**: Complete chain of custody from sampling to analysis
- **Extensibility**: Support for emerging detection technologies
- **Accuracy**: Precise particle characterization and polymer identification
- **Environmental Context**: Geographic and ecological metadata integration

### 1.2 Scope

This phase covers:
- Particle data structures (size, shape, color, count)
- Spectral data formats (Raman, FTIR, fluorescence)
- Sample metadata (location, time, environmental conditions)
- Polymer type classifications (PE, PP, PS, PET, PVC, etc.)
- Detection result aggregations and reports

---

## 2. Core Data Structures

### 2.1 Microplastic Particle

```typescript
interface MicroplasticParticle {
  // Identity
  particleId: string;              // UUID
  sampleId: string;                // Parent sample reference

  // Physical Properties
  size: ParticleSize;
  shape: ParticleShape;
  color: ParticleColor;

  // Composition
  polymerType: PolymerType;
  polymerConfidence: number;       // 0.0-1.0
  additives: ChemicalAdditive[];

  // Spatial Data
  positionInSample: Coordinate3D;

  // Detection Metadata
  detectionMethod: DetectionMethod;
  detectedAt: ISO8601DateTime;
  detectedBy: string;              // Instrument ID

  // Verification
  verified: boolean;
  verifiedBy?: string;             // Analyst ID
  verificationNotes?: string;
}
```

### 2.2 Particle Size

```typescript
interface ParticleSize {
  // Dimensions (micrometers)
  length: number;
  width: number;
  height?: number;                 // Optional for 2D analysis

  // Derived Measurements
  area: number;                    // μm²
  volume?: number;                 // μm³
  equivalentSphericalDiameter: number;  // μm

  // Size Classification
  sizeClass: SizeClass;

  // Measurement Uncertainty
  measurementError: number;        // Percentage
  measurementMethod: MeasurementMethod;
}

enum SizeClass {
  NANOPLASTIC = "NANOPLASTIC",     // < 1 μm
  SMALL_MICROPLASTIC = "SMALL",    // 1-100 μm
  LARGE_MICROPLASTIC = "LARGE",    // 100-1000 μm
  MESOPLASTIC = "MESOPLASTIC",     // 1-5 mm
  MACROPLASTIC = "MACROPLASTIC"    // > 5 mm
}
```

### 2.3 Particle Shape

```typescript
interface ParticleShape {
  primaryShape: ShapeCategory;
  aspectRatio: number;
  circularity: number;             // 0.0-1.0
  sphericity: number;              // 0.0-1.0
  roughness: number;               // Surface texture metric

  // Shape descriptors
  perimeter: number;               // μm
  convexHullArea: number;          // μm²
  solidity: number;                // Area / Convex Hull Area
}

enum ShapeCategory {
  FIBER = "FIBER",
  FRAGMENT = "FRAGMENT",
  FILM = "FILM",
  FOAM = "FOAM",
  BEAD = "BEAD",
  PELLET = "PELLET",
  FLAKE = "FLAKE",
  OTHER = "OTHER"
}
```

### 2.4 Particle Color

```typescript
interface ParticleColor {
  // Color Space Representations
  rgb: RGBColor;
  hsv: HSVColor;
  lab: LABColor;

  // Visual Description
  primaryColor: ColorCategory;
  transparency: TransparencyLevel;

  // Spectral Properties
  reflectanceSpectrum?: SpectralData;
}

enum ColorCategory {
  TRANSPARENT = "TRANSPARENT",
  WHITE = "WHITE",
  BLACK = "BLACK",
  RED = "RED",
  BLUE = "BLUE",
  GREEN = "GREEN",
  YELLOW = "YELLOW",
  ORANGE = "ORANGE",
  BROWN = "BROWN",
  MULTICOLOR = "MULTICOLOR"
}
```

---

## 3. Polymer Type Data

### 3.1 Polymer Classification

```typescript
interface PolymerType {
  // Primary Identification
  polymerCode: PolymerCode;
  polymerName: string;
  chemicalFormula: string;
  casNumber?: string;              // Chemical Abstracts Service number

  // Properties
  density: number;                 // g/cm³
  meltingPoint?: number;           // °C
  glassTransitionTemp?: number;    // °C

  // Environmental Characteristics
  degradationRate: DegradationRate;
  toxicityLevel: ToxicityLevel;
  bioaccumulation: BioaccumulationPotential;

  // Source Information
  commonSources: string[];
  typicalApplications: string[];
}

enum PolymerCode {
  // Common Polymers
  PE = "PE",                       // Polyethylene
  PP = "PP",                       // Polypropylene
  PS = "PS",                       // Polystyrene
  PET = "PET",                     // Polyethylene terephthalate
  PVC = "PVC",                     // Polyvinyl chloride
  PA = "PA",                       // Polyamide (Nylon)
  PC = "PC",                       // Polycarbonate
  PMMA = "PMMA",                   // Polymethyl methacrylate (Acrylic)
  PU = "PU",                       // Polyurethane
  PTFE = "PTFE",                   // Polytetrafluoroethylene (Teflon)

  // Specialty Polymers
  EVA = "EVA",                     // Ethylene-vinyl acetate
  ABS = "ABS",                     // Acrylonitrile butadiene styrene
  PEEK = "PEEK",                   // Polyether ether ketone

  // Unknown/Mixed
  UNKNOWN = "UNKNOWN",
  MIXED = "MIXED"
}
```

### 3.2 Chemical Additives

```typescript
interface ChemicalAdditive {
  additiveName: string;
  casNumber: string;
  concentration?: number;          // mg/kg
  purpose: AdditiveFunction;
  hazardLevel: HazardLevel;
}

enum AdditiveFunction {
  PLASTICIZER = "PLASTICIZER",
  STABILIZER = "STABILIZER",
  COLORANT = "COLORANT",
  FLAME_RETARDANT = "FLAME_RETARDANT",
  UV_ABSORBER = "UV_ABSORBER",
  FILLER = "FILLER",
  OTHER = "OTHER"
}
```

---

## 4. Spectroscopic Data

### 4.1 Raman Spectrum

```typescript
interface RamanSpectrum {
  spectrumId: string;
  particleId: string;

  // Spectral Data
  wavenumbers: number[];           // cm⁻¹
  intensities: number[];           // Arbitrary units

  // Acquisition Parameters
  laserWavelength: number;         // nm (typically 532, 633, 785)
  laserPower: number;              // mW
  exposureTime: number;            // seconds
  accumulations: number;

  // Analysis Results
  peakPositions: number[];         // cm⁻¹
  peakIntensities: number[];
  baselineCorrected: boolean;

  // Identification
  matchedPolymer: PolymerType;
  matchConfidence: number;         // 0.0-1.0
  spectralLibrary: string;         // Reference library used

  // Quality Metrics
  signalToNoiseRatio: number;
  spectrumQuality: QualityLevel;
}
```

### 4.2 FTIR Spectrum

```typescript
interface FTIRSpectrum {
  spectrumId: string;
  particleId: string;

  // Spectral Data
  wavenumbers: number[];           // cm⁻¹ (typically 4000-400)
  absorbance: number[];            // AU
  transmittance?: number[];        // Percentage

  // Acquisition Parameters
  technique: FTIRTechnique;
  resolution: number;              // cm⁻¹
  scans: number;

  // Analysis Results
  functionalGroups: FunctionalGroup[];
  matchedPolymer: PolymerType;
  matchConfidence: number;

  // Quality Metrics
  signalToNoiseRatio: number;
  spectrumQuality: QualityLevel;
}

enum FTIRTechnique {
  ATR = "ATR",                     // Attenuated Total Reflectance
  TRANSMISSION = "TRANSMISSION",
  REFLECTANCE = "REFLECTANCE",
  MICRO_FTIR = "MICRO_FTIR"
}
```

### 4.3 Fluorescence Data

```typescript
interface FluorescenceData {
  dataId: string;
  particleId: string;

  // Excitation/Emission
  excitationWavelength: number;    // nm
  emissionWavelength: number;      // nm
  intensity: number;               // Arbitrary units

  // Staining Information
  fluorophore?: string;
  stainingProtocol?: string;

  // Image Data
  imageUrl?: string;
  imageResolution?: ImageResolution;
}
```

---

## 5. Sample Metadata

### 5.1 Sample Information

```typescript
interface MicroplasticSample {
  // Identity
  sampleId: string;                // UUID
  sampleName: string;
  collectionId?: string;           // Campaign or study ID

  // Collection Information
  collectedAt: ISO8601DateTime;
  collectedBy: string;             // Collector ID or name
  collectionMethod: SamplingMethod;

  // Location
  location: GeoLocation;
  environmentType: EnvironmentType;

  // Sample Characteristics
  sampleType: SampleType;
  sampleVolume?: number;           // Liters (for water)
  sampleMass?: number;             // Grams (for soil/sediment)
  sampleArea?: number;             // m² (for surface sampling)

  // Environmental Conditions
  temperature?: number;            // °C
  pH?: number;
  salinity?: number;               // PSU (Practical Salinity Units)
  turbidity?: number;              // NTU

  // Processing
  processingDate?: ISO8601DateTime;
  processingMethod?: ProcessingMethod;
  filtrationSize?: number;         // μm

  // Results Summary
  totalParticleCount: number;
  particleConcentration?: number;  // particles/L or particles/kg
  dominantPolymer?: PolymerCode;

  // Quality Control
  blankControl?: SampleControl;
  recoveryRate?: number;           // Percentage
  qcStatus: QCStatus;
}
```

### 5.2 Geographic Location

```typescript
interface GeoLocation {
  // Coordinates
  latitude: number;                // Decimal degrees
  longitude: number;               // Decimal degrees
  altitude?: number;               // Meters above sea level
  depth?: number;                  // Meters below surface (for aquatic)

  // Location Description
  siteName?: string;
  region?: string;
  country?: string;
  waterbody?: string;              // Ocean, sea, river, lake name

  // Coordinate System
  coordinateSystem: string;        // e.g., "WGS84"
  accuracy: number;                // Meters
}

enum EnvironmentType {
  MARINE_SURFACE = "MARINE_SURFACE",
  MARINE_SUBSURFACE = "MARINE_SUBSURFACE",
  MARINE_SEDIMENT = "MARINE_SEDIMENT",
  FRESHWATER_SURFACE = "FRESHWATER_SURFACE",
  FRESHWATER_SUBSURFACE = "FRESHWATER_SUBSURFACE",
  FRESHWATER_SEDIMENT = "FRESHWATER_SEDIMENT",
  SOIL = "SOIL",
  AIR = "AIR",
  WASTEWATER = "WASTEWATER",
  DRINKING_WATER = "DRINKING_WATER",
  BIOTA = "BIOTA"
}
```

---

## 6. Detection Results

### 6.1 Analysis Results

```typescript
interface DetectionResult {
  resultId: string;
  sampleId: string;

  // Analysis Information
  analyzedAt: ISO8601DateTime;
  analyzedBy: string;              // Analyst or system ID
  analysisMethod: AnalysisMethod;

  // Particle Data
  particles: MicroplasticParticle[];
  totalParticles: number;

  // Statistical Summary
  sizeDistribution: SizeDistribution;
  polymerDistribution: PolymerDistribution;
  shapeDistribution: ShapeDistribution;
  colorDistribution: ColorDistribution;

  // Concentration
  concentration: ConcentrationData;

  // Quality Metrics
  detectionLimit: number;          // μm
  falsePositiveRate?: number;
  falseNegativeRate?: number;

  // Environmental Impact
  riskAssessment?: RiskAssessment;
}
```

### 6.2 Statistical Distributions

```typescript
interface SizeDistribution {
  bins: SizeBin[];
  meanSize: number;                // μm
  medianSize: number;              // μm
  standardDeviation: number;
  sizeRange: [number, number];     // [min, max]
}

interface PolymerDistribution {
  polymers: {
    polymerType: PolymerCode;
    count: number;
    percentage: number;
  }[];
}

interface ShapeDistribution {
  shapes: {
    shapeType: ShapeCategory;
    count: number;
    percentage: number;
  }[];
}
```

### 6.3 Concentration Data

```typescript
interface ConcentrationData {
  // Absolute Concentration
  particlesPerUnit: number;
  unit: ConcentrationUnit;

  // Mass Concentration
  massPerUnit?: number;            // mg/L or mg/kg

  // Size-specific Concentrations
  nanoplasticConcentration?: number;
  microplasticConcentration?: number;

  // Confidence Interval
  confidenceLevel: number;         // e.g., 0.95 for 95%
  confidenceInterval: [number, number];
}

enum ConcentrationUnit {
  PARTICLES_PER_LITER = "particles/L",
  PARTICLES_PER_CUBIC_METER = "particles/m³",
  PARTICLES_PER_KILOGRAM = "particles/kg",
  PARTICLES_PER_SQUARE_METER = "particles/m²"
}
```

---

## 7. Data Validation

### 7.1 Validation Rules

```typescript
interface ValidationRules {
  // Particle Validation
  minParticleSize: number;         // μm
  maxParticleSize: number;         // μm
  minPolymerConfidence: number;    // 0.0-1.0

  // Sample Validation
  requiredMetadata: string[];
  locationPrecision: number;       // Decimal places

  // Spectral Validation
  minSignalToNoise: number;
  requiredWavenumberRange: [number, number];
}
```

### 7.2 Quality Levels

```typescript
enum QualityLevel {
  EXCELLENT = "EXCELLENT",         // SNR > 50, confidence > 0.95
  GOOD = "GOOD",                   // SNR > 20, confidence > 0.85
  ACCEPTABLE = "ACCEPTABLE",       // SNR > 10, confidence > 0.70
  POOR = "POOR",                   // SNR < 10, confidence < 0.70
  FAILED = "FAILED"                // Analysis failed
}

enum QCStatus {
  PASSED = "PASSED",
  PASSED_WITH_NOTES = "PASSED_WITH_NOTES",
  FAILED = "FAILED",
  PENDING = "PENDING"
}
```

---

## 8. File Formats

### 8.1 JSON Format

Standard JSON format for data exchange:

```json
{
  "standard": "WIA-MICROPLASTIC_DETECTION",
  "version": "1.0",
  "sample": {
    "sampleId": "sample-2026-001",
    "collectedAt": "2026-01-12T10:30:00Z",
    "location": {
      "latitude": 34.0522,
      "longitude": -118.2437,
      "siteName": "Santa Monica Bay"
    }
  },
  "particles": [...],
  "results": {...}
}
```

### 8.2 CSV Format

Simplified CSV format for particle lists:

```csv
particleId,sampleId,size_μm,shape,polymerType,confidence
p001,sample-2026-001,125.5,FRAGMENT,PE,0.92
p002,sample-2026-001,45.2,FIBER,PP,0.87
```

---

## 9. Compliance and Standards

### 9.1 International Standards

- ISO 24187:2023 - Microplastics in water
- ASTM D8333 - Standard Practice for Microplastics Sampling
- NOAA Marine Debris Program protocols
- GESAMP guidelines for microplastic monitoring

### 9.2 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Express detection intentions
- **WIA-OMNI-API**: Unified API access
- **WIA-DATA_QUALITY**: Quality assurance
- **WIA-ENVIRONMENTAL_MONITORING**: Environmental context

---

**Document Version**: 1.0
**Last Updated**: 2026-01-12
**Status**: FULL Implementation
**Philosophy**: 弘益人間 - Protecting our planet for all humanity

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
