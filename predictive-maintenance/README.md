# WIA-IND-026: Predictive Maintenance 🔧

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


**World Certification Industry Association**
**Standard ID**: WIA-IND-026
**Category**: IND (Industry) - Amber (#F59E0B)
**Version**: 1.0
**Status**: Production Ready

## Overview

WIA-IND-026 establishes a comprehensive standard for predictive maintenance systems, enabling organizations to anticipate equipment failures before they occur, optimize maintenance schedules, and maximize operational efficiency through advanced sensor data analysis and machine learning.

### Key Features

- **Multi-Modal Sensor Integration**: Vibration, thermal, acoustic, and oil analysis
- **ML-Powered Predictions**: Advanced failure prediction algorithms
- **Real-Time Monitoring**: Continuous condition monitoring and alerting
- **Maintenance Optimization**: Intelligent scheduling and spare parts management
- **Industry Agnostic**: Applicable across manufacturing, energy, transportation, and more
- **Edge & Cloud Ready**: Deploy on-premise or in the cloud

## Quick Start

### Installation

```bash
curl -fsSL https://raw.githubusercontent.com/WIA-Official/wia-standards/main/standards/predictive-maintenance/install.sh | bash
```

Or install the SDK:

```bash
npm install @wia/ind-026
```

### Basic Usage

#### TypeScript/JavaScript SDK

```typescript
import { PredictiveMaintenanceSDK, SensorType, AnalysisType } from '@wia/ind-026';

// Initialize the SDK
const pdm = new PredictiveMaintenanceSDK({
  apiKey: 'your-api-key',
  organizationId: 'your-org-id',
  endpoint: 'https://api.wia.org/ind-026'
});

// Register an asset
const motor = await pdm.assets.register({
  assetId: 'MOTOR-001',
  assetType: 'ROTATING_MACHINERY',
  location: 'Plant-A-Line-1',
  specifications: {
    manufacturer: 'ABB',
    model: 'M3BP-315',
    powerRating: '150kW',
    operatingSpeed: '1480rpm'
  }
});

// Add sensors to the asset
await pdm.sensors.attach({
  assetId: 'MOTOR-001',
  sensors: [
    {
      sensorId: 'VIB-001',
      type: SensorType.VIBRATION,
      location: 'MOTOR_DE',
      samplingRate: 25600 // Hz
    },
    {
      sensorId: 'TEMP-001',
      type: SensorType.THERMAL,
      location: 'MOTOR_BEARING',
      samplingRate: 1 // Hz
    }
  ]
});

// Stream sensor data
pdm.data.stream({
  assetId: 'MOTOR-001',
  onData: async (data) => {
    await pdm.data.ingest({
      sensorId: data.sensorId,
      timestamp: Date.now(),
      values: data.values,
      unit: data.unit
    });
  }
});

// Perform vibration analysis
const vibrationAnalysis = await pdm.analysis.vibration({
  assetId: 'MOTOR-001',
  sensorId: 'VIB-001',
  analysisTypes: [
    AnalysisType.FFT,
    AnalysisType.ENVELOPE,
    AnalysisType.CREST_FACTOR
  ]
});

console.log('Bearing Health:', vibrationAnalysis.bearingCondition);
console.log('Predicted RUL:', vibrationAnalysis.remainingUsefulLife);

// Get failure predictions
const predictions = await pdm.predictions.analyze({
  assetId: 'MOTOR-001',
  timeHorizon: 30 // days
});

predictions.forEach(pred => {
  console.log(`${pred.failureMode}: ${pred.probability}% (${pred.daysToFailure} days)`);

  if (pred.probability > 70) {
    // Generate maintenance work order
    pdm.maintenance.createWorkOrder({
      assetId: 'MOTOR-001',
      priority: pred.severity,
      predictedFailure: pred.failureMode,
      recommendedActions: pred.recommendations,
      scheduledDate: pred.recommendedMaintenanceDate
    });
  }
});

// Thermal imaging analysis
const thermalScan = await pdm.analysis.thermal({
  assetId: 'MOTOR-001',
  imageData: thermalImageBuffer,
  referenceTemperature: 25, // °C
  algorithm: 'ANOMALY_DETECTION'
});

if (thermalScan.hotspots.length > 0) {
  console.log('⚠️  Hotspots detected:');
  thermalScan.hotspots.forEach(spot => {
    console.log(`  - ${spot.location}: ${spot.temperature}°C (ΔT: ${spot.delta}°C)`);
  });
}

// Oil analysis
const oilSample = await pdm.analysis.oil({
  assetId: 'GEARBOX-001',
  sampleId: 'OIL-2025-001',
  tests: {
    viscosity: 46.2, // cSt @ 40°C
    waterContent: 150, // ppm
    particleCount: {
      '4μm': 12000,
      '6μm': 3200,
      '14μm': 850
    },
    metals: {
      iron: 45, // ppm
      copper: 12,
      aluminum: 8
    }
  }
});

console.log('Oil Condition:', oilSample.condition);
console.log('Recommendations:', oilSample.recommendations);

// Acoustic monitoring
const acousticAnalysis = await pdm.analysis.acoustic({
  assetId: 'COMPRESSOR-001',
  audioData: audioBuffer,
  samplingRate: 44100,
  analysisTypes: ['LEAK_DETECTION', 'CAVITATION', 'MECHANICAL_LOOSENESS']
});

acousticAnalysis.anomalies.forEach(anomaly => {
  console.log(`${anomaly.type} detected at ${anomaly.frequency}Hz: ${anomaly.severity}`);
});

// Schedule optimization
const schedule = await pdm.maintenance.optimize({
  assets: ['MOTOR-001', 'PUMP-002', 'COMPRESSOR-001'],
  constraints: {
    maintenanceWindows: [
      { start: '2025-01-15T00:00:00Z', end: '2025-01-15T08:00:00Z' }
    ],
    availableCrews: 2,
    spareParts: await pdm.inventory.check()
  },
  objectives: ['MINIMIZE_DOWNTIME', 'MAXIMIZE_RELIABILITY']
});

console.log('Optimized Schedule:', schedule.workOrders);
```

#### CLI Usage

```bash
# Register an asset
wia-ind-026 asset register \
  --id MOTOR-001 \
  --type ROTATING_MACHINERY \
  --location "Plant-A-Line-1" \
  --specs "manufacturer=ABB,model=M3BP-315"

# Attach sensors
wia-ind-026 sensor attach \
  --asset MOTOR-001 \
  --sensor VIB-001 \
  --type vibration \
  --location MOTOR_DE \
  --sampling-rate 25600

# Ingest sensor data
wia-ind-026 data ingest \
  --sensor VIB-001 \
  --file vibration-data.csv \
  --format csv

# Run vibration analysis
wia-ind-026 analyze vibration \
  --asset MOTOR-001 \
  --sensor VIB-001 \
  --analysis fft,envelope,crest-factor \
  --output report.json

# Get predictions
wia-ind-026 predict \
  --asset MOTOR-001 \
  --horizon 30 \
  --threshold 70 \
  --format table

# Thermal analysis
wia-ind-026 analyze thermal \
  --asset MOTOR-001 \
  --image thermal-scan.jpg \
  --reference-temp 25 \
  --output thermal-report.pdf

# Oil analysis
wia-ind-026 analyze oil \
  --asset GEARBOX-001 \
  --sample-id OIL-2025-001 \
  --viscosity 46.2 \
  --water-content 150 \
  --particle-count "4μm:12000,6μm:3200,14μm:850"

# Acoustic monitoring
wia-ind-026 analyze acoustic \
  --asset COMPRESSOR-001 \
  --audio audio-recording.wav \
  --analysis leak-detection,cavitation

# Create work order
wia-ind-026 maintenance create-wo \
  --asset MOTOR-001 \
  --priority high \
  --failure-mode "Bearing Wear" \
  --scheduled-date 2025-01-15

# Optimize maintenance schedule
wia-ind-026 maintenance optimize \
  --assets MOTOR-001,PUMP-002,COMPRESSOR-001 \
  --window "2025-01-15T00:00:00Z/2025-01-15T08:00:00Z" \
  --crews 2

# Real-time monitoring dashboard
wia-ind-026 monitor dashboard \
  --assets MOTOR-001,PUMP-002 \
  --refresh 5s
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA-IND-026 Architecture                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │             Sensor Data Collection Layer                  │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  Vibration │ Thermal │ Acoustic │ Oil │ Electrical │...  │  │
│  └────────┬─────────────────────────────────────────────────┘  │
│           │                                                     │
│  ┌────────▼─────────────────────────────────────────────────┐  │
│  │            Data Ingestion & Processing                    │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  • Real-time streaming                                    │  │
│  │  • Data validation & normalization                        │  │
│  │  • Edge processing & filtering                            │  │
│  └────────┬─────────────────────────────────────────────────┘  │
│           │                                                     │
│  ┌────────▼─────────────────────────────────────────────────┐  │
│  │              Analysis & Feature Extraction                │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  FFT │ Envelope │ Time Domain │ Frequency Domain │ ML     │  │
│  └────────┬─────────────────────────────────────────────────┘  │
│           │                                                     │
│  ┌────────▼─────────────────────────────────────────────────┐  │
│  │           Predictive Models & AI Engine                   │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  • Failure mode prediction                                │  │
│  │  • RUL estimation                                         │  │
│  │  • Anomaly detection                                      │  │
│  └────────┬─────────────────────────────────────────────────┘  │
│           │                                                     │
│  ┌────────▼─────────────────────────────────────────────────┐  │
│  │        Maintenance Planning & Optimization                │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  • Work order generation                                  │  │
│  │  • Schedule optimization                                  │  │
│  │  • Resource allocation                                    │  │
│  └────────┬─────────────────────────────────────────────────┘  │
│           │                                                     │
│  ┌────────▼─────────────────────────────────────────────────┐  │
│  │          Visualization & Reporting Layer                  │  │
│  ├──────────────────────────────────────────────────────────┤  │
│  │  Dashboards │ Alerts │ Reports │ Mobile Apps              │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Use Cases

### Manufacturing
- **CNC Machines**: Spindle bearing monitoring, tool wear prediction
- **Conveyor Systems**: Belt tracking, roller bearing health
- **Pumps & Motors**: Bearing condition, alignment monitoring
- **Compressors**: Valve health, efficiency degradation

### Energy
- **Wind Turbines**: Gearbox monitoring, blade condition
- **Power Generation**: Generator health, transformer monitoring
- **Oil & Gas**: Pipeline integrity, pump monitoring

### Transportation
- **Railways**: Wheel bearing monitoring, track condition
- **Aviation**: Engine health monitoring, landing gear
- **Maritime**: Propulsion systems, auxiliary machinery

### Heavy Industry
- **Mining Equipment**: Crusher monitoring, conveyor systems
- **Steel Mills**: Rolling mill bearings, furnace monitoring
- **Chemical Plants**: Reactor monitoring, pump health

## Supported Analysis Types

### Vibration Analysis
- **Time Domain**: RMS, Peak, Crest Factor, Kurtosis
- **Frequency Domain**: FFT, Power Spectrum, Harmonics
- **Advanced**: Envelope Analysis, Order Analysis, Orbit Analysis

### Thermal Imaging
- **Pattern Recognition**: Hotspot detection, temperature gradients
- **Comparative Analysis**: Historical trending, baseline comparison
- **AI-Based**: Anomaly detection, failure prediction

### Oil Analysis
- **Physical Properties**: Viscosity, water content, flash point
- **Contamination**: Particle counting, ISO cleanliness codes
- **Wear Metals**: Spectrometric analysis, ferrous content

### Acoustic Monitoring
- **Ultrasonic**: Leak detection, electrical discharge
- **Airborne**: Bearing defects, mechanical looseness
- **Structure-Borne**: Cavitation, flow turbulence

## Integration

WIA-IND-026 integrates seamlessly with:

- **WIA-IOT-001**: IoT sensor networks
- **WIA-AI-015**: Machine learning models
- **WIA-DATA-008**: Time-series databases
- **WIA-CLOUD-012**: Cloud infrastructure
- **WIA-MOBILE-019**: Mobile maintenance apps
- **WIA-ERP-023**: Enterprise resource planning

## Compliance & Standards

- ISO 13374: Condition monitoring and diagnostics
- ISO 17359: General guidelines on condition monitoring
- ISO 18436: Vibration condition monitoring
- ISO 20816: Mechanical vibration measurement
- MIMOSA OSA-CBM: Condition-based maintenance
- IEC 60812: Failure modes and effects analysis (FMEA)

## Performance Metrics

- **Prediction Accuracy**: >90% for major failure modes
- **False Positive Rate**: <5%
- **Data Latency**: <100ms for critical sensors
- **Analysis Throughput**: >10,000 samples/second
- **Model Update Frequency**: Continuous learning

## Support & Documentation

- **Full Specification**: [spec/WIA-IND-026-v1.0.md](spec/WIA-IND-026-v1.0.md)
- **API Reference**: [TypeScript SDK Documentation](api/typescript/)
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **Community**: https://community.wia.org/ind-026
- **Enterprise Support**: enterprise@wia.org

## License

MIT License - see LICENSE file for details

---

**홍익인간 (弘益人間) · Benefit All Humanity**

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍

## 🌍 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

This standard embodies the Korean philosophy of 弘益人間, aiming to benefit all humanity through innovation and standardization.

