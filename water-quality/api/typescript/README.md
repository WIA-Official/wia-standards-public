# @wia/water-quality

WIA Water Quality Standard - TypeScript SDK for comprehensive water quality monitoring and regulatory compliance.

## Installation

```bash
npm install @wia/water-quality
```

## Features

- **Water Quality Monitoring**: Record and analyze water samples
- **Compliance Checking**: Validate against WHO, EPA, and other standards
- **Water Quality Index**: Calculate WQI scores
- **Contaminant Detection**: Identify biological, chemical, and heavy metal contaminants
- **Treatment Monitoring**: Track water treatment processes
- **Trend Analysis**: Analyze parameter trends over time
- **Station Management**: Manage monitoring stations
- **Event System**: Subscribe to water quality events
- **Reporting**: Generate comprehensive water quality reports

## Quick Start

```typescript
import { createWaterQuality, WaterSample } from '@wia/water-quality';

// Create instance
const wq = createWaterQuality();

// Record a water sample
const sample: WaterSample = {
  sampleId: 'WQ-2025-001',
  timestamp: new Date(),
  location: {
    stationId: 'ST-001',
    stationName: 'Main Reservoir',
    latitude: 37.7749,
    longitude: -122.4194,
    waterBody: 'Crystal Lake',
    sourceType: 'surface',
  },
  sampleType: 'grab',
  physical: {
    temperature: 18.5,
    turbidity: 2.3,
    totalSuspendedSolids: 12,
  },
  chemical: {
    pH: 7.2,
    dissolvedOxygen: 7.8,
    BOD: 2.1,
    COD: 15.3,
    ammonia: 0.05,
    nitrate: 1.2,
  },
  biological: {
    totalColiform: 0,
    eColi: 0,
  },
};

wq.recordSample(sample);

// Calculate Water Quality Index
const wqi = wq.calculateWQI('WQ-2025-001');
console.log(`Water Quality Index: ${wqi.toFixed(2)}/100`);

// Check compliance
const compliance = wq.checkCompliance('WQ-2025-001');
const violations = compliance.filter(r => !r.isCompliant);

if (violations.length === 0) {
  console.log('✓ All parameters within compliance');
} else {
  console.log('⚠ Compliance violations detected:');
  violations.forEach(v => {
    console.log(`  - ${v.parameter}: ${v.action}`);
  });
}

// Detect contaminants
const contaminants = wq.detectContaminants('WQ-2025-001');
if (contaminants.length > 0) {
  console.log('Contaminants detected:', contaminants);
}
```

## Station Management

```typescript
import { MonitoringStation } from '@wia/water-quality';

const station: MonitoringStation = {
  stationId: 'ST-001',
  name: 'Main Reservoir Monitoring Station',
  location: {
    stationId: 'ST-001',
    stationName: 'Main Reservoir',
    latitude: 37.7749,
    longitude: -122.4194,
    waterBody: 'Crystal Lake',
    sourceType: 'surface',
  },
  parametersMonitored: ['pH', 'turbidity', 'dissolvedOxygen', 'temperature'],
  samplingFrequency: 'daily',
  isActive: true,
  installDate: new Date('2023-01-15'),
};

wq.registerStation(station);
```

## Event Handling

```typescript
// Subscribe to compliance violations
wq.on('compliance-violation', (event) => {
  console.log('Compliance violation detected!');
  console.log('Station:', event.stationId);
  console.log('Violations:', event.data.violations);
});

// Subscribe to contaminant detection
wq.on('contaminant-detected', (event) => {
  console.log('Contaminants detected!');
  console.log('Contaminants:', event.data.contaminants);
});

// Subscribe to sample recording
wq.on('sample-recorded', (event) => {
  console.log('New sample recorded:', event.sampleId);
});
```

## Trend Analysis

```typescript
// Analyze pH trend over last 30 days
const trend = wq.analyzeTrend('ST-001', 'pH', 30);
console.log(`pH Trend: ${trend.trend}`);
console.log(`Change Rate: ${trend.changeRate.toFixed(2)}%`);
console.log(`Based on ${trend.samples} samples`);
```

## Treatment Monitoring

```typescript
import { TreatmentProcess } from '@wia/water-quality';

const treatment: TreatmentProcess = {
  processId: 'TRT-001',
  processType: 'filtration',
  inlet: {
    physical: { temperature: 20, turbidity: 15.2 },
    chemical: { pH: 7.1, dissolvedOxygen: 6.5 },
  },
  outlet: {
    physical: { temperature: 20, turbidity: 1.8 },
    chemical: { pH: 7.2, dissolvedOxygen: 7.2 },
  },
  timestamp: new Date(),
};

wq.monitorTreatment(treatment);
console.log(`Treatment Efficiency: ${treatment.efficiency?.toFixed(2)}%`);
```

## Reporting

```typescript
// Generate water quality report
const report = wq.generateReport(
  'ST-001',
  new Date('2025-01-01'),
  new Date('2025-01-31')
);

console.log('Water Quality Report');
console.log('===================');
console.log(`Station: ${report.station?.name}`);
console.log(`Sample Count: ${report.sampleCount}`);
console.log(`Average WQI: ${report.averageWQI.toFixed(2)}/100`);
console.log(`Compliance Rate: ${report.complianceRate.toFixed(2)}%`);
console.log(`Violations: ${report.violations.length}`);
console.log(`Contaminants: ${report.contaminants.length}`);
```

## Custom Compliance Thresholds

```typescript
import { ComplianceThreshold } from '@wia/water-quality';

// Add custom threshold
const customThreshold: ComplianceThreshold = {
  parameter: 'copper',
  maxLevel: 1.3,
  unit: 'mg/L',
  standard: 'EPA',
  actionLevel: 1.0,
};

wq.addThreshold(customThreshold);
```

## Water Quality Parameters

### Physical Parameters
- Temperature (°C)
- Turbidity (NTU)
- Color (Pt-Co units)
- Total Suspended Solids (mg/L)
- Total Dissolved Solids (mg/L)
- Conductivity (μS/cm)

### Chemical Parameters
- pH (0-14)
- Dissolved Oxygen (mg/L)
- BOD/COD (mg/L)
- Nutrients (ammonia, nitrate, nitrite, phosphate)
- Chlorine (free/total)
- Alkalinity/Hardness

### Biological Parameters
- Total Coliform (CFU/100mL)
- Fecal Coliform (CFU/100mL)
- E. coli (CFU/100mL)
- Enterococci (CFU/100mL)

### Heavy Metals
- Lead, Copper, Zinc, Iron, Manganese
- Arsenic, Mercury, Cadmium, Chromium, Nickel

## Compliance Standards

- **WHO**: World Health Organization Guidelines
- **EPA**: US Environmental Protection Agency
- **EU**: European Union Drinking Water Directive
- **ISO**: International Standards
- **National/Local**: Country-specific standards

## TypeScript Support

Full TypeScript support with comprehensive type definitions:

```typescript
import type {
  WaterSample,
  PhysicalParameters,
  ChemicalParameters,
  BiologicalParameters,
  MetalParameters,
  ComplianceResult,
  ComplianceThreshold,
  Contaminant,
  MonitoringStation,
  TreatmentProcess,
} from '@wia/water-quality';
```

## License

MIT © WIA (World Certification Industry Association)

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

Clean water is a fundamental human right. This standard aims to make water quality monitoring accessible and effective for communities worldwide.

---

Part of the [WIA Standards](https://github.com/WIA-Official/wia-standards) ecosystem.
