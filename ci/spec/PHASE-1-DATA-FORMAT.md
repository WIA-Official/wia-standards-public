# Phase 1: CI Data Format Specification

## WIA-CI Data Format Standard

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

---

## 1. 개요

WIA-CI(인공와우) 데이터 포맷 표준은 인공와우 기기 간 데이터 호환성을 보장하고,
옥타브/피치 정보를 포함한 확장된 오디오 데이터 교환을 정의합니다.

### 1.1 목적

- 제조사 간 CI 데이터 호환성 확보
- 옥타브 정보를 포함한 확장 오디오 포맷 정의
- 실시간 처리를 위한 효율적 데이터 구조

### 1.2 적용 범위

- Cochlear, MED-EL, Advanced Bionics 등 모든 CI 제조사
- 22채널 기준 (다른 채널 수 확장 가능)
- 실시간 스트리밍 및 저장용 포맷

---

## 2. Audio Input Format

### 2.1 기본 오디오 스펙

```typescript
interface CIAudioInput {
  // 샘플링
  sampleRate: 16000 | 22050 | 44100 | 48000; // Hz
  bitDepth: 16 | 24 | 32;
  channels: 1;  // 모노 (CI는 단측)

  // 프레임
  frameSize: number;     // 샘플 수 (권장: 256 @ 16kHz = 16ms)
  hopSize: number;       // 프레임 간 이동 (권장: 128 = 8ms)

  // 형식
  encoding: 'pcm_s16le' | 'pcm_f32le' | 'pcm_s24le';
}
```

### 2.2 권장 설정

| 용도 | 샘플레이트 | 프레임 크기 | Hop | 지연 |
|------|-----------|------------|-----|------|
| 실시간 어음 | 16 kHz | 256 | 128 | 16ms |
| 음악 | 44.1 kHz | 1024 | 512 | 23ms |
| 저지연 | 16 kHz | 128 | 64 | 8ms |

---

## 3. Electrode Mapping Format

### 3.1 22채널 표준 주파수 할당표 (FAT)

```typescript
interface FrequencyAllocationTable {
  electrodeCount: 22;

  // 전극별 중심 주파수 (Hz)
  // electrode[0] = 최고주파(base), electrode[21] = 최저주파(apex)
  centerFrequencies: [
    7438, 6500, 5688, 5000, 4375, 3813, 3313, 2875,
    2500, 2188, 1938, 1688, 1438, 1250, 1125, 1000,
    875, 750, 625, 500, 375, 250
  ];

  // 전극별 대역폭 (Hz)
  bandwidths: number[];  // 각 전극의 주파수 범위

  // 대수적 분포 파라미터
  logarithmic: {
    minFreq: 250;   // Hz
    maxFreq: 8000;  // Hz
    octaves: 5;     // 총 옥타브 범위
  };
}
```

### 3.2 전극-주파수 매핑 데이터

```typescript
interface ElectrodeMap {
  electrode: number;        // 1-22
  centerFreq: number;       // Hz
  lowCutoff: number;        // Hz
  highCutoff: number;       // Hz
  insertionDepth: number;   // mm (달팽이관 내 삽입 깊이)
  tonotopicAngle: number;   // degrees (Greenwood function 기반)
}
```

### 3.3 개인화 매핑 (선택)

```typescript
interface PersonalizedMap {
  patientId: string;

  // 청력학적 측정 기반 조정
  audiometricThresholds: number[];  // 전극별 역치
  comfortLevels: number[];          // 전극별 쾌적 레벨

  // Tonotopic 불일치 보정
  frequencyShift: number[];         // 전극별 주파수 이동량

  // 활성 전극
  activeElectrodes: number[];       // 활성화된 전극 번호
  disabledElectrodes: number[];     // 비활성 전극 (예: 얼굴신경 자극)
}
```

---

## 4. Envelope Data Format

### 4.1 기본 Envelope 구조

```typescript
interface EnvelopeFrame {
  timestamp: number;        // ms (프레임 시작 시간)
  frameIndex: number;       // 프레임 번호

  // 22채널 envelope 값
  envelopes: Float32Array;  // length: 22, range: 0.0-1.0

  // 신호 통계
  rms: number;              // 전체 RMS 레벨
  peak: number;             // 피크 레벨

  // 선택된 채널 (n-of-m)
  selectedChannels: number[]; // ACE: 상위 8-12채널 인덱스
}
```

### 4.2 확장 Envelope (옥타브 포함)

```typescript
interface ExtendedEnvelopeFrame extends EnvelopeFrame {
  // WIA-CI 확장: 옥타브 정보
  octaveInfo: {
    fundamentalFreq: number;     // 추정 기본 주파수 (Hz)
    octaveNumber: number;        // 옥타브 번호 (0-8)
    octaveConfidence: number;    // 신뢰도 (0.0-1.0)

    // 하모닉 분석 결과
    harmonics: HarmonicInfo[];
  };

  // Temporal Modulation 파라미터
  temporalModulation: {
    pattern: 'AM' | 'FM' | 'mixed';
    frequency: number;           // 변조 주파수 (Hz)
    depth: number;               // 변조 깊이 (0.0-1.0)
  };
}

interface HarmonicInfo {
  harmonic: number;      // 1 = fundamental, 2 = 2nd harmonic...
  frequency: number;     // Hz
  amplitude: number;     // 상대 진폭
  present: boolean;      // 존재 여부
}
```

---

## 5. Stimulation Data Format

### 5.1 전극 자극 명령

```typescript
interface StimulationCommand {
  timestamp: number;          // μs (마이크로초 정밀도)
  electrode: number;          // 1-22

  // 자극 파라미터
  amplitude: number;          // μA (0-2000)
  pulseWidth: number;         // μs (일반적으로 25-50)
  interphaseGap: number;      // μs (일반적으로 8)

  // 극성
  polarity: 'cathodic_first' | 'anodic_first';

  // WIA-CI 확장
  octaveModulation?: {
    enabled: boolean;
    pattern: number[];        // 변조 패턴
    rate: number;             // 변조율 (Hz)
  };
}
```

### 5.2 자극 프레임 (다중 전극)

```typescript
interface StimulationFrame {
  frameTimestamp: number;     // ms
  stimulations: StimulationCommand[];

  // CIS 인터리빙 보장
  interleaved: boolean;       // true면 동시 자극 없음

  // 총 전류
  totalCurrent: number;       // μA
  compliance: boolean;        // 전압 순응 여부
}
```

---

## 6. Stream Format

### 6.1 실시간 스트림 패킷

```typescript
interface CIStreamPacket {
  header: {
    magic: 0x57494143;        // 'WIAC'
    version: number;          // 프로토콜 버전
    packetType: PacketType;
    sequenceNumber: number;
    timestamp: number;        // μs
    payloadLength: number;
  };

  payload: AudioPayload | EnvelopePayload | StimulationPayload;

  checksum: number;           // CRC32
}

type PacketType =
  | 0x01  // Audio Input
  | 0x02  // Envelope Data
  | 0x03  // Stimulation Commands
  | 0x04  // Status/Telemetry
  | 0x05  // Configuration
  | 0x10  // Extended (with Octave);
```

### 6.2 바이너리 레이아웃

```
┌────────────────────────────────────────────────────┐
│ Header (16 bytes)                                  │
├────────────────────────────────────────────────────┤
│ Magic       │ Version │ Type │ Seq   │ Timestamp  │
│ 4 bytes     │ 2 bytes │ 1 b  │ 2 b   │ 4 bytes    │
├────────────────────────────────────────────────────┤
│ PayloadLen  │ Reserved                             │
│ 2 bytes     │ 1 byte                               │
├────────────────────────────────────────────────────┤
│ Payload (variable length)                          │
│ ...                                                │
├────────────────────────────────────────────────────┤
│ Checksum (4 bytes)                                 │
└────────────────────────────────────────────────────┘
```

---

## 7. File Format

### 7.1 WIA-CI 파일 (.wiac)

```typescript
interface WIACFile {
  header: {
    magic: 'WIAC';
    version: [1, 0, 0];

    // 메타데이터
    createdAt: ISO8601;
    duration: number;         // ms

    // 장비 정보
    device: {
      manufacturer: string;   // 'Cochlear', 'MED-EL', 'AB'
      model: string;
      serialNumber?: string;
    };

    // 환자 정보 (익명화)
    patient?: {
      id: string;             // 익명 ID
      implantSide: 'left' | 'right';
      implantDate?: ISO8601;
    };
  };

  // 설정
  configuration: {
    audioInput: CIAudioInput;
    frequencyTable: FrequencyAllocationTable;
    processingStrategy: 'CIS' | 'ACE' | 'FSP' | 'WIA-OCTAVE';
    personalizedMap?: PersonalizedMap;
  };

  // 데이터 블록
  blocks: DataBlock[];
}
```

### 7.2 데이터 블록

```typescript
interface DataBlock {
  type: 'audio' | 'envelope' | 'stimulation' | 'extended';
  startTime: number;          // ms
  endTime: number;            // ms
  sampleCount: number;
  compressedSize: number;
  compression: 'none' | 'lz4' | 'zstd';
  data: Uint8Array;
}
```

---

## 8. JSON Schema

### 8.1 Configuration Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.family/schemas/ci/config.json",
  "title": "WIA-CI Configuration",
  "type": "object",
  "required": ["version", "audioInput", "electrodeMap"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "audioInput": {
      "type": "object",
      "properties": {
        "sampleRate": { "enum": [16000, 22050, 44100, 48000] },
        "bitDepth": { "enum": [16, 24, 32] },
        "frameSize": { "type": "integer", "minimum": 64 },
        "hopSize": { "type": "integer", "minimum": 32 }
      }
    },
    "electrodeMap": {
      "type": "object",
      "properties": {
        "electrodeCount": { "type": "integer", "default": 22 },
        "centerFrequencies": {
          "type": "array",
          "items": { "type": "number" }
        }
      }
    },
    "processingStrategy": {
      "enum": ["CIS", "ACE", "FSP", "WIA-OCTAVE"]
    }
  }
}
```

---

## 9. 호환성

### 9.1 제조사별 변환

| 제조사 | 채널 수 | 변환 방법 |
|--------|---------|----------|
| Cochlear | 22 | 직접 매핑 |
| MED-EL | 12 | 2:1 다운샘플링 |
| Advanced Bionics | 16 | 보간 |

### 9.2 레거시 포맷 지원

- Cochlear NRT 파일 가져오기
- MED-EL MAESTRO 파일 가져오기
- AB SoundWave 파일 가져오기

---

## 10. 성능 요구사항

| 지표 | 요구사항 | 목표 |
|------|----------|------|
| 지연 | < 50ms | < 20ms |
| 처리량 | > 100 프레임/초 | > 200 프레임/초 |
| 메모리 | < 10MB | < 5MB |
| CPU | < 20% (모바일) | < 10% |

---

**Document ID**: WIA-CI-PHASE1-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License
