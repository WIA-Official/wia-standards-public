# ✅ WIA-IND-025: Quality Control Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-025
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND (Industry)
> **Color:** Amber (#F59E0B)

---

## 🌟 Overview

The WIA-IND-025 standard defines a comprehensive framework for quality control and quality assurance in manufacturing and production environments. This standard supports inspection protocols, statistical process control (SPC), defect detection and classification, quality metrics (including Six Sigma), calibration management, non-conformance handling, corrective action tracking, audit management, document control, and quality certifications (ISO 9001, ISO 13485, IATF 16949, etc.).

**弘익人間 (Benefit All Humanity)** - This standard aims to ensure product quality, safety, and reliability that protects consumers and enhances manufacturing excellence worldwide.

## 🎯 Key Features

- **Inspection Protocols**: Structured inspection procedures with configurable checkpoints
- **Statistical Process Control (SPC)**: Real-time process monitoring with control charts
- **Defect Detection**: AI-powered defect classification and root cause analysis
- **Quality Metrics**: Six Sigma, Cpk, OEE, PPM, and other industry metrics
- **Calibration Management**: Equipment calibration scheduling and tracking
- **Non-Conformance Handling**: NCR creation, investigation, and resolution
- **Corrective Action (CAPA)**: Systematic corrective and preventive action tracking
- **Audit Management**: Internal and external audit planning and execution
- **Document Control**: Version-controlled quality documentation
- **Quality Certifications**: ISO 9001, ISO 13485, IATF 16949 compliance

## 📊 Core Concepts

### 1. Quality Data Model

```typescript
{
  "inspectionId": "INS-2025-001234",
  "productSku": "WIDGET-A100",
  "batchNumber": "BATCH-20251227-01",
  "inspectionType": "first-article",
  "timestamp": "2025-12-27T10:30:00Z",
  "inspector": {
    "id": "EMP-5678",
    "name": "Jane Smith",
    "certification": "ASQ-CQI"
  },
  "measurements": [{
    "characteristic": "diameter",
    "nominal": 50.0,
    "measured": 50.02,
    "tolerance": 0.05,
    "unit": "mm",
    "status": "pass"
  }],
  "defects": [{
    "type": "surface-scratch",
    "severity": "minor",
    "location": "top-surface",
    "count": 2
  }],
  "result": "pass",
  "spcData": {
    "cpk": 1.67,
    "cp": 1.85,
    "mean": 50.01,
    "stdDev": 0.009
  }
}
```

### 2. Statistical Process Control (SPC)

Monitor process capability in real-time using control charts:

```
Process Capability Indices:
Cp = (USL - LSL) / (6 × σ)
Cpk = min[(USL - μ) / (3 × σ), (μ - LSL) / (3 × σ)]

Six Sigma Level = Cpk × 3
DPMO = (Defects / Opportunities) × 1,000,000
```

### 3. Quality Control Chart Types

```
X-bar & R Chart → Monitor process mean and range
P Chart → Track proportion of defectives
C Chart → Count defects per unit
CUSUM → Detect small process shifts
EWMA → Exponentially weighted moving average
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  QualityControlSDK,
  createInspection,
  calculateSPC,
  detectDefects,
  manageCalibration,
  trackCAPA
} from '@wia/ind-025';

const sdk = new QualityControlSDK({
  apiKey: 'your-api-key',
  certificationLevel: 'ISO-9001',
  industry: 'automotive'
});

// Create quality inspection
const inspection = await sdk.createInspection({
  productSku: 'WIDGET-A100',
  batchNumber: 'BATCH-20251227-01',
  inspectionType: 'first-article',
  measurements: [{
    characteristic: 'diameter',
    nominal: 50.0,
    measured: 50.02,
    tolerance: 0.05,
    unit: 'mm'
  }, {
    characteristic: 'hardness',
    nominal: 62,
    measured: 61.5,
    tolerance: 2,
    unit: 'HRC'
  }]
});

console.log(`Inspection: ${inspection.id}`);
console.log(`Result: ${inspection.result}`);
console.log(`Cpk: ${inspection.spcData.cpk}`);

// Calculate SPC metrics
const spc = await sdk.calculateSPC({
  characteristic: 'diameter',
  measurements: [50.02, 49.98, 50.01, 50.03, 49.99],
  specification: {
    nominal: 50.0,
    usl: 50.05,
    lsl: 49.95
  }
});

console.log(`Cp: ${spc.cp}`);
console.log(`Cpk: ${spc.cpk}`);
console.log(`Sigma Level: ${spc.sigmaLevel}`);
console.log(`DPMO: ${spc.dpmo}`);
console.log(`Out of Control: ${spc.outOfControl}`);

// AI-powered defect detection
const defects = await sdk.detectDefects({
  imageUrl: 'https://example.com/product-image.jpg',
  productModel: 'WIDGET-A100',
  aiModel: 'yolov8-defect-detection'
});

console.log(`Defects found: ${defects.length}`);
defects.forEach(defect => {
  console.log(`  - ${defect.type}: ${defect.severity} (confidence: ${defect.confidence})`);
});

// Manage equipment calibration
const calibration = await sdk.scheduleCalibration({
  equipmentId: 'CMM-001',
  equipmentType: 'coordinate-measuring-machine',
  lastCalibration: '2024-12-27',
  interval: 365, // days
  standard: 'ISO-10360'
});

console.log(`Next calibration: ${calibration.nextDueDate}`);
console.log(`Status: ${calibration.status}`);

// Create non-conformance report (NCR)
const ncr = await sdk.createNCR({
  productSku: 'WIDGET-A100',
  batchNumber: 'BATCH-20251227-01',
  issueDescription: 'Diameter out of tolerance',
  severity: 'major',
  disposition: 'rework',
  rootCause: 'tooling-wear'
});

console.log(`NCR: ${ncr.id}`);
console.log(`CAPA required: ${ncr.capaRequired}`);

// Track corrective action (CAPA)
const capa = await sdk.createCAPA({
  ncrId: ncr.id,
  type: 'corrective',
  description: 'Replace worn cutting tool',
  assignedTo: 'EMP-9012',
  dueDate: '2025-12-30',
  preventiveAction: 'Implement tool wear monitoring system'
});

console.log(`CAPA: ${capa.id}`);
console.log(`Status: ${capa.status}`);
```

### CLI Tool

```bash
# Create quality inspection
wia-ind-025 inspect --product WIDGET-A100 --batch BATCH-20251227-01 --type first-article

# Calculate SPC metrics
wia-ind-025 spc --characteristic diameter --nominal 50.0 --usl 50.05 --lsl 49.95 --data "50.02,49.98,50.01"

# Detect defects using AI
wia-ind-025 detect-defects --image product.jpg --model WIDGET-A100

# Schedule equipment calibration
wia-ind-025 calibrate --equipment CMM-001 --interval 365 --standard ISO-10360

# Create non-conformance report
wia-ind-025 ncr create --product WIDGET-A100 --issue "Diameter out of spec" --severity major

# Track CAPA
wia-ind-025 capa create --ncr NCR-2025-001234 --action "Replace tool" --assignee EMP-9012

# Generate audit report
wia-ind-025 audit generate --type internal --standard ISO-9001 --period 2025-Q4

# Check calibration status
wia-ind-025 calibration-status --equipment all

# Quality metrics dashboard
wia-ind-025 metrics --period 2025-12 --format dashboard

# Six Sigma analysis
wia-ind-025 six-sigma --process machining --characteristic diameter --period 30d
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-025-v1.0.md](./spec/WIA-IND-025-v1.0.md) | Complete specification with protocols and metrics |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-025.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/quality-control

# Run installation script
./install.sh

# Verify installation
wia-ind-025 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-025

# Or yarn
yarn add @wia/ind-025
```

```typescript
import { QualityControlSDK } from '@wia/ind-025';

const sdk = new QualityControlSDK({
  apiKey: process.env.WIA_API_KEY,
  certificationLevel: 'ISO-9001'
});

// Perform quality inspection
async function performQualityCheck() {
  // Create inspection
  const inspection = await sdk.createInspection({
    productSku: 'ENGINE-BLOCK-V8',
    batchNumber: 'BATCH-2025-Q4-001',
    inspectionType: 'in-process',
    measurements: [{
      characteristic: 'bore-diameter',
      nominal: 95.0,
      measured: 95.01,
      tolerance: 0.02,
      unit: 'mm'
    }]
  });

  console.log(`Inspection: ${inspection.id}`);
  console.log(`Result: ${inspection.result}`);
  console.log(`Cpk: ${inspection.spcData.cpk}`);

  // Check if process is in control
  if (inspection.spcData.cpk < 1.33) {
    console.warn('Process capability below minimum threshold!');

    // Create NCR if needed
    const ncr = await sdk.createNCR({
      productSku: 'ENGINE-BLOCK-V8',
      batchNumber: 'BATCH-2025-Q4-001',
      issueDescription: 'Cpk below 1.33',
      severity: 'major',
      disposition: 'review'
    });

    console.log(`NCR created: ${ncr.id}`);
  }

  // Calculate Six Sigma level
  const sigmaLevel = inspection.spcData.cpk * 3;
  console.log(`Six Sigma Level: ${sigmaLevel.toFixed(2)}`);
}

performQualityCheck();
```

## 📊 Quality Metrics

| Metric | Description | Formula | Target |
|--------|-------------|---------|--------|
| Cpk | Process capability index | min[(USL-μ)/(3σ), (μ-LSL)/(3σ)] | ≥ 1.33 |
| Cp | Process capability | (USL - LSL) / (6σ) | ≥ 1.33 |
| PPM | Parts per million defective | (Defects / Total) × 1,000,000 | < 100 |
| DPMO | Defects per million opportunities | (Defects / Opportunities) × 1,000,000 | < 3.4 (6σ) |
| Sigma Level | Six Sigma level | Cpk × 3 | ≥ 4.0 |
| First Pass Yield | % passed first inspection | (Pass / Total) × 100 | ≥ 99% |
| Defect Rate | % of defective units | (Defective / Total) × 100 | < 1% |
| COPQ | Cost of poor quality | Total Quality Cost / Revenue × 100 | < 3% |
| MTBF | Mean time between failures | Operating Time / Failures | Maximize |

## 🌍 Use Cases

### 1. Automotive Manufacturing
Ensure IATF 16949 compliance with rigorous inspection protocols, SPC monitoring, and PPAP documentation for safety-critical components.

### 2. Medical Device Production
Maintain ISO 13485 compliance with complete traceability, validation records, and design control for patient safety.

### 3. Aerospace Components
Meet AS9100 requirements with first article inspection, material certifications, and strict process control for flight-critical parts.

### 4. Electronics Assembly
Implement automated optical inspection (AOI), X-ray inspection, and functional testing with real-time SPC monitoring.

### 5. Pharmaceutical Manufacturing
Ensure GMP compliance with batch record review, deviation management, and validation documentation.

### 6. Food & Beverage
Maintain HACCP compliance with critical control point monitoring, pathogen testing, and allergen control.

## ⚙️ Inspection Types

### 1. Receiving Inspection
Verify incoming materials and components meet specifications before acceptance.

```typescript
const receivingInspection = await sdk.createInspection({
  inspectionType: 'receiving',
  supplierId: 'SUP-5678',
  poNumber: 'PO-2025-001234',
  lotNumber: 'LOT-98765',
  sampleSize: 50,
  acceptanceLevel: 'AQL-2.5'
});
```

### 2. First Article Inspection (FAI)
Comprehensive inspection of first production unit to verify setup.

```typescript
const fai = await sdk.createInspection({
  inspectionType: 'first-article',
  workOrder: 'WO-2025-567',
  fullDimensionalReport: true,
  cmm: true,
  materialCert: true
});
```

### 3. In-Process Inspection
Monitor quality during production to detect issues early.

```typescript
const inProcess = await sdk.createInspection({
  inspectionType: 'in-process',
  workstation: 'STATION-05',
  samplingPlan: 'hourly',
  controlCharts: ['xbar-r', 'p-chart']
});
```

### 4. Final Inspection
Verify finished products meet all specifications before shipment.

```typescript
const finalInspection = await sdk.createInspection({
  inspectionType: 'final',
  functionalTest: true,
  visualInspection: true,
  packagingCheck: true,
  labelVerification: true
});
```

## 📈 Statistical Process Control (SPC)

### Control Chart Types

```typescript
// X-bar and R Chart (variables data)
const xbarR = await sdk.createControlChart({
  type: 'xbar-r',
  characteristic: 'diameter',
  subgroupSize: 5,
  controlLimits: 'calculated'
});

// P Chart (attributes data - proportion defective)
const pChart = await sdk.createControlChart({
  type: 'p-chart',
  characteristic: 'defect-rate',
  sampleSize: 100
});

// C Chart (attributes data - count of defects)
const cChart = await sdk.createControlChart({
  type: 'c-chart',
  characteristic: 'surface-defects'
});

// CUSUM Chart (detect small shifts)
const cusum = await sdk.createControlChart({
  type: 'cusum',
  characteristic: 'weight',
  target: 500,
  allowance: 5
});
```

### Process Capability Analysis

```typescript
const capability = await sdk.analyzeProcessCapability({
  characteristic: 'tensile-strength',
  data: measurements,
  specification: {
    nominal: 500,
    usl: 550,
    lsl: 450
  }
});

console.log(`Cp: ${capability.cp}`);           // Process potential
console.log(`Cpk: ${capability.cpk}`);         // Process capability
console.log(`Pp: ${capability.pp}`);           // Process performance
console.log(`Ppk: ${capability.ppk}`);         // Process performance index
console.log(`Sigma: ${capability.sigma}`);     // Process sigma
console.log(`DPMO: ${capability.dpmo}`);       // Defects per million
```

## 🔍 Defect Classification

### Defect Severity Levels

| Level | Description | Action |
|-------|-------------|--------|
| Critical | Safety hazard, product unusable | 100% quarantine, immediate CAPA |
| Major | Significant deviation from spec | Containment, investigation |
| Minor | Small deviation, still functional | Document, monitor trend |
| Cosmetic | Appearance only, no function impact | Accept with concession |

### Common Defect Types

```typescript
const defectTypes = {
  dimensional: ['oversize', 'undersize', 'out-of-tolerance'],
  surface: ['scratch', 'dent', 'corrosion', 'contamination'],
  assembly: ['missing-part', 'wrong-part', 'loose-fastener'],
  functional: ['performance-failure', 'leakage', 'malfunction'],
  cosmetic: ['discoloration', 'blemish', 'finish-defect']
};
```

## 🛠️ Calibration Management

### Calibration Scheduling

```typescript
// Schedule equipment calibration
const calibration = await sdk.scheduleCalibration({
  equipmentId: 'CMM-001',
  equipmentType: 'coordinate-measuring-machine',
  calibrationStandard: 'ISO-10360-2',
  interval: 365, // days
  vendor: 'NIST-traceable lab',
  criticalEquipment: true
});

// Check calibration status
const status = await sdk.getCalibrationStatus('CMM-001');
console.log(`Status: ${status.current}`);
console.log(`Next due: ${status.nextDueDate}`);
console.log(`Days remaining: ${status.daysRemaining}`);

// Generate calibration certificate
const certificate = await sdk.generateCalibrationCertificate('CMM-001');
```

### Equipment Categories

- **Critical**: Direct product measurement (requires frequent calibration)
- **Non-critical**: Support equipment (longer calibration intervals)
- **Reference standards**: Master gauges (highest accuracy, annual calibration)

## 📋 Non-Conformance & CAPA

### Non-Conformance Report (NCR)

```typescript
const ncr = await sdk.createNCR({
  productSku: 'WIDGET-A100',
  batchNumber: 'BATCH-20251227-01',
  issueDescription: 'Hardness below specification',
  detectedBy: 'EMP-5678',
  detectionPoint: 'final-inspection',
  severity: 'major',
  quantity: 50,
  disposition: 'rework', // or: scrap, use-as-is, return-to-supplier
  rootCause: 'heat-treatment-temperature-deviation',
  containmentAction: 'Quarantine all units from shift',
  customerImpact: 'none' // caught before shipment
});
```

### Corrective and Preventive Action (CAPA)

```typescript
const capa = await sdk.createCAPA({
  ncrId: 'NCR-2025-001234',
  type: 'corrective', // or: preventive
  problemStatement: 'Heat treatment temperature 10°C below target',
  rootCauseAnalysis: {
    method: '5-why',
    findings: [
      'Why 1: Temperature sensor reading incorrect',
      'Why 2: Sensor calibration expired',
      'Why 3: Calibration tracking system not automated',
      'Why 4: No automatic alerts for due calibrations',
      'Why 5: Preventive maintenance system incomplete'
    ],
    rootCause: 'Lack of automated calibration tracking'
  },
  correctiveAction: {
    description: 'Recalibrate temperature sensor',
    assignedTo: 'EMP-9012',
    dueDate: '2025-12-28',
    status: 'in-progress'
  },
  preventiveAction: {
    description: 'Implement automated calibration tracking system',
    assignedTo: 'EMP-3456',
    dueDate: '2026-01-15',
    status: 'planned'
  },
  verification: {
    method: 'Monitor heat treatment temperatures for 30 days',
    responsible: 'Quality Manager',
    effectivenessCheck: true
  }
});
```

## 🎯 Quality Certifications

### ISO 9001:2015 (Quality Management)

```typescript
const iso9001Audit = await sdk.planAudit({
  standard: 'ISO-9001:2015',
  auditType: 'internal',
  scope: 'all-processes',
  clauses: [
    '4.1 - Understanding organization and context',
    '5.1 - Leadership and commitment',
    '6.1 - Risk-based thinking',
    '7.1 - Resources',
    '8.1 - Operational planning and control',
    '9.1 - Monitoring, measurement, analysis',
    '10.1 - Improvement'
  ],
  auditor: 'Lead Auditor ASQ-CQA',
  startDate: '2026-01-15',
  duration: 3 // days
});
```

### ISO 13485 (Medical Devices)

```typescript
const iso13485Compliance = await sdk.checkCompliance({
  standard: 'ISO-13485:2016',
  requirements: [
    'Design controls',
    'Risk management (ISO 14971)',
    'Traceability',
    'Validation',
    'Document control',
    'CAPA',
    'Complaint handling'
  ]
});
```

### IATF 16949 (Automotive)

```typescript
const iatf16949 = await sdk.checkCompliance({
  standard: 'IATF-16949:2016',
  requirements: [
    'PPAP (Production Part Approval Process)',
    'APQP (Advanced Product Quality Planning)',
    'FMEA (Failure Mode Effects Analysis)',
    'MSA (Measurement Systems Analysis)',
    'SPC (Statistical Process Control)',
    'Control plan',
    'Layered process audit'
  ]
});
```

## 📁 Document Control

```typescript
// Create controlled document
const document = await sdk.createControlledDocument({
  type: 'quality-procedure',
  title: 'QP-001 Inspection Procedure',
  version: '1.2',
  effectiveDate: '2026-01-01',
  author: 'Quality Manager',
  approver: 'VP of Quality',
  reviewCycle: 365, // days
  distribution: ['quality-team', 'production-supervisors']
});

// Track document revisions
const revision = await sdk.reviseDocument({
  documentId: 'DOC-QP-001',
  version: '1.3',
  changes: 'Updated acceptance criteria for visual inspection',
  reasonForChange: 'Customer specification change',
  approvalRequired: true
});

// Document training records
const training = await sdk.recordTraining({
  documentId: 'DOC-QP-001',
  version: '1.3',
  trainee: 'EMP-5678',
  trainer: 'EMP-9012',
  date: '2026-01-05',
  assessmentPassed: true
});
```

## 🔗 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language quality queries
- **WIA-OMNI-API**: Universal quality data API
- **WIA-IND-024**: Manufacturing automation integration
- **WIA-IND-023**: Supply chain quality tracking
- **WIA-AI**: AI-powered defect detection and root cause analysis
- **WIA-BLOCKCHAIN**: Immutable quality records and certifications

## 🌍 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

This standard embodies the Korean philosophy of 弘益人間, aiming to benefit all humanity through innovation and standardization.

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
