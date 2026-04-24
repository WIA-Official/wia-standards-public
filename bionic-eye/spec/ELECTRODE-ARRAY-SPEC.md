# WIA Bionic Eye - Electrode Array Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-15

## 1. Overview

이 문서는 인공 시각 임플란트의 전극 어레이 표준을 정의합니다.

## 2. Implant Types

### 2.1 Supported Implant Types

| Type | Location | Target | Resolution |
|------|----------|--------|------------|
| Epiretinal | 망막 표면 | 신경절 세포 | 60-1500 electrodes |
| Subretinal | 망막 하부 | 양극 세포 | 1500+ electrodes |
| Suprachoroidal | 맥락막 상부 | 망막 | 44-99 electrodes |
| Optic Nerve | 시신경 | 시신경 섬유 | 4-20 electrodes |
| Cortical | 시각 피질 | V1 영역 | 60-1000 electrodes |

### 2.2 Commercial Systems Reference

| System | Type | Electrodes | Status |
|--------|------|------------|--------|
| Argus II | Epiretinal | 60 | FDA Approved |
| Alpha AMS | Subretinal | 1600 | CE Marked |
| PRIMA | Subretinal | 378 | Clinical Trial |
| Orion | Cortical | 60 | Clinical Trial |

## 3. Electrode Array Structure

### 3.1 Physical Specifications

```typescript
interface ElectrodeArray {
  arrayId: string;
  type: ImplantType;
  manufacturer: string;
  model: string;
  serialNumber: string;

  physical: {
    totalElectrodes: number;
    activeElectrodes: number;
    rows: number;
    columns: number;
    electrodeDiameter: number;    // μm (100-500)
    electrodeSpacing: number;     // μm center-to-center (200-1000)
    electrodeArea: number;        // mm² per electrode
    arrayWidth: number;           // mm
    arrayHeight: number;          // mm
    thickness: number;            // μm
    material: ElectrodeMaterial;
    substrateMaterial: string;
  };

  placement: {
    eye: 'left' | 'right';
    location: ImplantLocation;
    implantDate: Date;
    surgeon: string;
    hospital: string;
  };
}

enum ElectrodeMaterial {
  PLATINUM = 'platinum',
  PLATINUM_IRIDIUM = 'platinum_iridium',
  IRIDIUM_OXIDE = 'iridium_oxide',
  TITANIUM_NITRIDE = 'titanium_nitride',
  PEDOT = 'pedot',
  CARBON_NANOTUBE = 'carbon_nanotube',
}
```

### 3.2 Electrode Status

```typescript
interface ElectrodeInfo {
  index: number;
  gridPosition: { row: number; col: number };
  physicalPosition: { x: number; y: number };  // μm from center

  status: {
    functional: boolean;
    enabled: boolean;
    failureDate?: Date;
    failureReason?: string;
  };

  electrical: {
    impedance: number;            // kΩ at 1kHz
    impedancePhase: number;       // degrees
    chargeCapacity: number;       // mC/cm²
    thresholdCurrent: number;     // μA (perception threshold)
    maxSafeCurrent: number;       // μA
    voltageCompliance: number;    // V
  };

  lastMeasurement: Date;
}
```

### 3.3 Impedance Requirements

| Parameter | Normal | Warning | Critical |
|-----------|--------|---------|----------|
| Impedance (kΩ) | 1-50 | 50-100 | >100 or <0.5 |
| Phase (degrees) | -20 to -80 | -10 to -90 | outside range |
| Stability (%) | <10% change | 10-30% change | >30% change |

## 4. Stimulation Safety Limits

### 4.1 Charge Density Limits

```typescript
interface SafetyLimits {
  // Shannon limit: log(Q/A) = k - log(Q)
  shannonK: number;               // 1.5-1.85 (conservative: 1.5)

  maxChargeDensity: number;       // μC/cm² (< 35 recommended)
  maxChargePerPhase: number;      // μC per electrode
  maxCurrentPerElectrode: number; // μA (typically < 1000)
  maxTotalCurrent: number;        // mA (all electrodes)

  maxFrequency: number;           // Hz
  maxDutyCycle: number;           // % (< 50%)

  dcLeakageMax: number;           // nA (< 100)
}
```

### 4.2 Material-Specific Limits

| Material | Charge Capacity (mC/cm²) | Recommended Max |
|----------|--------------------------|-----------------|
| Platinum | 0.05-0.15 | 0.05 |
| Pt-Ir | 0.05-0.15 | 0.05 |
| IrOx | 1-5 | 1.0 |
| TiN | 0.5-1 | 0.5 |
| PEDOT | 2-15 | 2.0 |

## 5. Waveform Specifications

```typescript
interface StimulationWaveform {
  type: WaveformType;

  cathodicFirst: boolean;         // cathodic phase first
  cathodicAmplitude: number;      // μA
  cathodicWidth: number;          // μs

  anodicAmplitude: number;        // μA
  anodicWidth: number;            // μs

  interphaseGap: number;          // μs (0-100)

  frequency: number;              // Hz
  pulsesPerBurst: number;
  burstDuration: number;          // ms
}

enum WaveformType {
  BIPHASIC_SYMMETRIC = 'biphasic_symmetric',
  BIPHASIC_ASYMMETRIC = 'biphasic_asymmetric',
  TRIPHASIC = 'triphasic',
}
```

## 6. Related Specifications

- [STIMULATION-PARAMS-SPEC.md](./STIMULATION-PARAMS-SPEC.md)
- [PHOSPHENE-MAPPING-SPEC.md](./PHOSPHENE-MAPPING-SPEC.md)
- [ELECTRICAL-SAFETY-SPEC.md](./ELECTRICAL-SAFETY-SPEC.md)
