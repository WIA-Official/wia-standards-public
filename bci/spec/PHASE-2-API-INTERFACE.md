# WIA BCI API Interface Specification

**Phase 2: API Interface Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA BCI API는 뇌-컴퓨터 인터페이스 기기와 상호작용하기 위한 표준 프로그래밍 인터페이스입니다. 이 API는 다양한 BCI 기기를 동일한 인터페이스로 제어할 수 있게 하여, 개발자가 특정 하드웨어에 종속되지 않고 BCI 애플리케이션을 개발할 수 있도록 합니다.

### 1.2 Design Goals

1. **Device Agnostic**: 어떤 BCI 기기든 동일한 API로 제어
2. **Event-Driven**: 비동기 이벤트 기반 아키텍처
3. **Type-Safe**: 강력한 타입 정의
4. **Extensible**: 새로운 기기 타입 쉽게 추가
5. **Cross-Platform**: TypeScript/Python 모두 지원

### 1.3 Scope

- **In Scope**:
  - 연결 관리 (Connection Management)
  - 이벤트 처리 (Event Handling)
  - 데이터 스트리밍 (Data Streaming)
  - 기기 검색 (Device Discovery)
  - 신호 처리 유틸리티 (Signal Processing Utilities)

- **Out of Scope** (Phase 2):
  - 실제 하드웨어 드라이버 구현
  - 실시간 통신 프로토콜 (Phase 3)
  - AI/ML 파이프라인 (Phase 4)

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Session** | BCI 기기와의 연결 세션 |
| **Stream** | 실시간 신경 신호 데이터 스트림 |
| **Marker** | 이벤트 마커/트리거 |
| **Epoch** | 특정 시간 구간의 데이터 |
| **Adapter** | 특정 기기 유형을 위한 어댑터 |
| **Signal** | WIA BCI 표준 신호 객체 |

---

## 3. Core Interfaces

### 3.1 WiaBci (Main Class)

메인 진입점 클래스입니다.

```typescript
class WiaBci {
  // 생성자
  constructor(options?: WiaBciOptions);

  // 연결 관리
  connect(config: DeviceConfig): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // 기기 검색
  listDevices(): Promise<DeviceInfo[]>;
  getDeviceInfo(): DeviceInfo | null;

  // 데이터 스트리밍
  startStream(): Promise<void>;
  stopStream(): Promise<void>;
  isStreaming(): boolean;

  // 이벤트 처리
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EventType>(event: T, handler: EventHandler<T>): void;
  emit<T extends EventType>(event: T, data: EventData<T>): void;

  // 상태 관리
  getState(): BciState;
  getChannels(): ChannelInfo[];

  // 리소스 정리
  dispose(): void;
}
```

### 3.2 Configuration Types

```typescript
interface WiaBciOptions {
  autoReconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  bufferSize?: number;
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}

interface DeviceConfig {
  type: DeviceType;
  device?: {
    manufacturer?: string;
    model?: string;
    serialNumber?: string;
  };
  connection?: {
    protocol?: ConnectionProtocol;
    address?: string;
    port?: number;
  };
  acquisition?: {
    samplingRate?: number;
    channels?: string[];
    filters?: FilterConfig[];
  };
}

type DeviceType =
  | 'eeg_headset'
  | 'eeg_cap'
  | 'implant_cortical'
  | 'implant_endovascular'
  | 'fnirs'
  | 'hybrid'
  | 'simulator';

type ConnectionProtocol =
  | 'usb'
  | 'bluetooth'
  | 'bluetooth_le'
  | 'wifi'
  | 'serial'
  | 'lsl';

interface FilterConfig {
  type: 'highpass' | 'lowpass' | 'bandpass' | 'notch';
  frequency: number | [number, number];
  order?: number;
}
```

### 3.3 Device Information

```typescript
interface DeviceInfo {
  id: string;
  name: string;
  type: DeviceType;
  manufacturer: string;
  model: string;
  serialNumber?: string;
  firmwareVersion?: string;

  capabilities: DeviceCapabilities;
  status: DeviceStatus;
}

interface DeviceCapabilities {
  channels: number;
  maxSamplingRate: number;
  supportedSamplingRates: number[];
  resolution: number;

  hasAccelerometer: boolean;
  hasGyroscope: boolean;
  hasImpedanceCheck: boolean;
  hasBatteryIndicator: boolean;

  supportedProtocols: ConnectionProtocol[];
}

type DeviceStatus =
  | 'available'
  | 'connected'
  | 'streaming'
  | 'error'
  | 'disconnected';
```

---

## 4. Event System

### 4.1 Event Types

```typescript
type EventType =
  // 연결 이벤트
  | 'connected'
  | 'disconnected'
  | 'reconnecting'
  | 'connection_error'

  // 스트리밍 이벤트
  | 'stream_started'
  | 'stream_stopped'
  | 'data'
  | 'signal'

  // 마커 이벤트
  | 'marker'
  | 'trigger'

  // 분류 이벤트
  | 'classification'
  | 'prediction'

  // 기기 이벤트
  | 'battery'
  | 'impedance'
  | 'quality'

  // 에러 이벤트
  | 'error'
  | 'warning';
```

### 4.2 Event Data Types

```typescript
// 신호 데이터 이벤트
interface SignalEvent {
  timestamp: number;
  sampleIndex: number;
  channels: number[];
  data: Float32Array;
}

// 마커 이벤트
interface MarkerEvent {
  timestamp: number;
  sampleIndex: number;
  code: number;
  label?: string;
  value?: any;
}

// 분류 결과 이벤트
interface ClassificationEvent {
  timestamp: number;
  classId: number;
  className: string;
  confidence: number;
  probabilities?: Record<string, number>;
}

// 품질 이벤트
interface QualityEvent {
  timestamp: number;
  channelQualities: ChannelQuality[];
  overallQuality: number;
}

interface ChannelQuality {
  channel: number;
  label: string;
  impedance?: number;
  signalQuality: number;
  artifacts: string[];
}

// 에러 이벤트
interface ErrorEvent {
  code: string;
  message: string;
  details?: any;
  recoverable: boolean;
}
```

### 4.3 Event Handler Types

```typescript
type EventHandler<T extends EventType> =
  T extends 'data' | 'signal' ? (event: SignalEvent) => void :
  T extends 'marker' | 'trigger' ? (event: MarkerEvent) => void :
  T extends 'classification' | 'prediction' ? (event: ClassificationEvent) => void :
  T extends 'quality' | 'impedance' ? (event: QualityEvent) => void :
  T extends 'error' | 'warning' ? (event: ErrorEvent) => void :
  T extends 'battery' ? (level: number) => void :
  T extends 'connected' | 'disconnected' ? () => void :
  (data: any) => void;
```

---

## 5. Adapters

### 5.1 Base Adapter Interface

```typescript
interface IBciAdapter {
  // 기본 정보
  readonly type: DeviceType;
  readonly name: string;

  // 연결 관리
  connect(config: DeviceConfig): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // 스트리밍
  startStream(): Promise<void>;
  stopStream(): Promise<void>;
  isStreaming(): boolean;

  // 기기 정보
  getDeviceInfo(): DeviceInfo | null;
  getChannels(): ChannelInfo[];

  // 이벤트
  onData(handler: (data: SignalEvent) => void): void;
  onError(handler: (error: ErrorEvent) => void): void;

  // 리소스
  dispose(): void;
}
```

### 5.2 Adapter Implementations

| Adapter | Device Type | Description |
|---------|-------------|-------------|
| `OpenBciAdapter` | eeg_headset | OpenBCI Cyton/Ganglion |
| `EmotivAdapter` | eeg_headset | Emotiv EPOC/Insight |
| `MuseAdapter` | eeg_headset | InteraXon Muse |
| `NeurableAdapter` | eeg_headset | Neurable devices |
| `LslAdapter` | any | Lab Streaming Layer |
| `SimulatorAdapter` | simulator | Test/Development |

### 5.3 OpenBCI Adapter Example

```typescript
class OpenBciAdapter implements IBciAdapter {
  readonly type = 'eeg_headset';
  readonly name = 'OpenBCI';

  private board: OpenBCIBoard | null = null;
  private streaming = false;
  private dataHandler?: (data: SignalEvent) => void;

  async connect(config: DeviceConfig): Promise<void> {
    const protocol = config.connection?.protocol ?? 'serial';

    if (protocol === 'serial') {
      this.board = new OpenBCICyton({
        port: config.connection?.address,
        baudRate: 115200
      });
    } else if (protocol === 'wifi') {
      this.board = new OpenBCIWifi({
        host: config.connection?.address
      });
    }

    await this.board.connect();
  }

  async startStream(): Promise<void> {
    if (!this.board) throw new Error('Not connected');

    this.board.on('sample', (sample) => {
      const event: SignalEvent = {
        timestamp: Date.now(),
        sampleIndex: sample.sampleNumber,
        channels: sample.channelData.map((_, i) => i),
        data: new Float32Array(sample.channelData)
      };

      this.dataHandler?.(event);
    });

    await this.board.streamStart();
    this.streaming = true;
  }

  // ... 나머지 메서드
}
```

---

## 6. Signal Processing Utilities

### 6.1 SignalProcessor Class

```typescript
class SignalProcessor {
  // 필터링
  static highpass(data: Float32Array, cutoff: number, fs: number): Float32Array;
  static lowpass(data: Float32Array, cutoff: number, fs: number): Float32Array;
  static bandpass(data: Float32Array, low: number, high: number, fs: number): Float32Array;
  static notch(data: Float32Array, freq: number, fs: number, Q?: number): Float32Array;

  // FFT
  static fft(data: Float32Array): ComplexArray;
  static ifft(data: ComplexArray): Float32Array;
  static psd(data: Float32Array, fs: number): PowerSpectrum;

  // 밴드 파워
  static bandpower(data: Float32Array, fs: number, band: FrequencyBand): number;
  static allBandPowers(data: Float32Array, fs: number): BandPowers;

  // 아티팩트
  static detectBlinks(eog: Float32Array, threshold?: number): number[];
  static detectMotion(accel: Float32Array, threshold?: number): number[];

  // 윈도잉
  static epoch(data: Float32Array, start: number, end: number): Float32Array;
  static sliding(data: Float32Array, windowSize: number, step: number): Float32Array[];
}

interface BandPowers {
  delta: number;   // 0.5-4 Hz
  theta: number;   // 4-8 Hz
  alpha: number;   // 8-13 Hz
  beta: number;    // 13-30 Hz
  gamma: number;   // 30-100 Hz
}

type FrequencyBand = 'delta' | 'theta' | 'alpha' | 'beta' | 'gamma' | [number, number];
```

### 6.2 Feature Extractor

```typescript
class FeatureExtractor {
  // 시간 영역 특징
  static mean(data: Float32Array): number;
  static variance(data: Float32Array): number;
  static rms(data: Float32Array): number;
  static zeroCrossings(data: Float32Array): number;
  static hjorthParameters(data: Float32Array): HjorthParams;

  // 주파수 영역 특징
  static spectralEntropy(psd: PowerSpectrum): number;
  static spectralEdge(psd: PowerSpectrum, percent?: number): number;
  static peakFrequency(psd: PowerSpectrum): number;

  // 연결성
  static coherence(ch1: Float32Array, ch2: Float32Array, fs: number): number;
  static correlation(ch1: Float32Array, ch2: Float32Array): number;

  // 통합 추출
  static extractAll(data: Float32Array, fs: number): FeatureVector;
}

interface HjorthParams {
  activity: number;
  mobility: number;
  complexity: number;
}

type FeatureVector = Record<string, number>;
```

---

## 7. Error Handling

### 7.1 Error Codes

| Code | Name | Description |
|------|------|-------------|
| `E001` | CONNECTION_FAILED | 기기 연결 실패 |
| `E002` | DEVICE_NOT_FOUND | 기기를 찾을 수 없음 |
| `E003` | PERMISSION_DENIED | 접근 권한 없음 |
| `E004` | STREAM_ERROR | 스트리밍 오류 |
| `E005` | INVALID_CONFIG | 잘못된 설정 |
| `E006` | ADAPTER_ERROR | 어댑터 오류 |
| `E007` | TIMEOUT | 작업 시간 초과 |
| `E008` | DISCONNECTED | 연결 끊김 |

### 7.2 Error Handling Pattern

```typescript
import { WiaBci, BciError } from 'wia-bci';

const bci = new WiaBci();

try {
  await bci.connect({ type: 'eeg_headset' });
} catch (error) {
  if (error instanceof BciError) {
    switch (error.code) {
      case 'E001':
        console.error('Connection failed:', error.message);
        break;
      case 'E002':
        console.error('Device not found');
        break;
      default:
        console.error('BCI Error:', error);
    }
  }
}

// 이벤트 기반 에러 처리
bci.on('error', (error) => {
  console.error(`[${error.code}] ${error.message}`);

  if (!error.recoverable) {
    bci.disconnect();
  }
});
```

---

## 8. Usage Examples

### 8.1 Basic Usage (TypeScript)

```typescript
import { WiaBci } from 'wia-bci';

async function main() {
  const bci = new WiaBci({ logLevel: 'info' });

  // 사용 가능한 기기 검색
  const devices = await bci.listDevices();
  console.log('Available devices:', devices);

  // 기기 연결
  await bci.connect({
    type: 'eeg_headset',
    device: { manufacturer: 'OpenBCI' },
    acquisition: {
      samplingRate: 250,
      channels: ['Fp1', 'Fp2', 'C3', 'C4', 'O1', 'O2']
    }
  });

  console.log('Connected:', bci.isConnected());

  // 데이터 수신
  bci.on('signal', (event) => {
    console.log(`Sample ${event.sampleIndex}:`, event.data);
  });

  // 스트리밍 시작
  await bci.startStream();

  // 10초 후 종료
  setTimeout(async () => {
    await bci.stopStream();
    await bci.disconnect();
    bci.dispose();
  }, 10000);
}

main();
```

### 8.2 Motor Imagery Classification

```typescript
import { WiaBci, SignalProcessor, FeatureExtractor } from 'wia-bci';

const bci = new WiaBci();
const buffer: Float32Array[] = [];
const EPOCH_SIZE = 250; // 1초 @ 250Hz

await bci.connect({ type: 'eeg_headset' });

bci.on('signal', (event) => {
  buffer.push(event.data);

  if (buffer.length >= EPOCH_SIZE) {
    // 1초 에폭 생성
    const epoch = concatenateBuffers(buffer.splice(0, EPOCH_SIZE));

    // 전처리
    const filtered = SignalProcessor.bandpass(epoch, 8, 30, 250);

    // 특징 추출
    const features = FeatureExtractor.extractAll(filtered, 250);
    const bandPowers = SignalProcessor.allBandPowers(filtered, 250);

    // 분류 결과 발행
    const prediction = classifier.predict({ ...features, ...bandPowers });

    bci.emit('classification', {
      timestamp: Date.now(),
      classId: prediction.class,
      className: ['rest', 'left', 'right', 'feet'][prediction.class],
      confidence: prediction.confidence
    });
  }
});

await bci.startStream();
```

### 8.3 Basic Usage (Python)

```python
import asyncio
from wia_bci import WiaBci

async def main():
    bci = WiaBci(log_level='info')

    # 사용 가능한 기기 검색
    devices = await bci.list_devices()
    print(f"Available devices: {devices}")

    # 기기 연결
    await bci.connect(
        type='eeg_headset',
        device={'manufacturer': 'OpenBCI'},
        acquisition={
            'sampling_rate': 250,
            'channels': ['Fp1', 'Fp2', 'C3', 'C4', 'O1', 'O2']
        }
    )

    print(f"Connected: {bci.is_connected()}")

    # 데이터 수신 (데코레이터 방식)
    @bci.on('signal')
    def on_signal(event):
        print(f"Sample {event.sample_index}: {event.data}")

    # 스트리밍 시작
    await bci.start_stream()

    # 10초 대기
    await asyncio.sleep(10)

    # 정리
    await bci.stop_stream()
    await bci.disconnect()
    bci.dispose()

asyncio.run(main())
```

---

## 9. API Reference Summary

### 9.1 TypeScript Package

```typescript
// 메인 내보내기
export {
  // 핵심 클래스
  WiaBci,
  SignalProcessor,
  FeatureExtractor,

  // 어댑터
  OpenBciAdapter,
  EmotivAdapter,
  MuseAdapter,
  LslAdapter,
  SimulatorAdapter,

  // 타입
  DeviceConfig,
  DeviceInfo,
  DeviceType,
  EventType,
  SignalEvent,
  MarkerEvent,
  ClassificationEvent,

  // 에러
  BciError,

  // 유틸리티
  createRecording,
  validateRecording
};
```

### 9.2 Python Package

```python
# 메인 내보내기
from wia_bci import (
    # 핵심 클래스
    WiaBci,
    SignalProcessor,
    FeatureExtractor,

    # 어댑터
    OpenBciAdapter,
    EmotivAdapter,
    MuseAdapter,
    LslAdapter,
    SimulatorAdapter,

    # 타입 (typing용)
    DeviceConfig,
    DeviceInfo,
    DeviceType,
    EventType,
    SignalEvent,
    MarkerEvent,
    ClassificationEvent,

    # 에러
    BciError,

    # 유틸리티
    create_recording,
    validate_recording
)
```

---

## 10. Compatibility

### 10.1 Phase 1 Data Format Compatibility

Phase 2 API는 Phase 1의 데이터 형식과 완전히 호환됩니다:

```typescript
import { WiaBci, createRecording } from 'wia-bci';

const bci = new WiaBci();
await bci.connect({ type: 'eeg_headset' });

// Phase 1 형식으로 녹화 생성
const recording = createRecording({
  device: bci.getDeviceInfo(),
  channels: bci.getChannels()
});

// 데이터 저장
bci.on('signal', (event) => {
  recording.addSample(event);
});

// Phase 1 형식으로 내보내기
await recording.save('./recording');
// 생성되는 파일:
// - recording.json
// - device.json
// - channels.json
// - events.json
// - data/eeg.bin
```

### 10.2 Lab Streaming Layer (LSL) Compatibility

```typescript
import { LslAdapter } from 'wia-bci';

const bci = new WiaBci();

// LSL 스트림에서 데이터 수신
await bci.connect({
  type: 'eeg_headset',
  connection: {
    protocol: 'lsl',
    address: 'type=EEG'  // LSL 쿼리
  }
});
```

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group

---

弘益人間 - *Benefit All Humanity*
