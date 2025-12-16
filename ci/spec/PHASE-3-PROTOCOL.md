# Phase 3: CI Communication Protocol Specification

## WIA-CI Protocol Standard

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

---

## 1. 개요

WIA-CI 프로토콜은 사운드 프로세서, 임플란트, 외부 기기 간의
표준화된 통신을 정의합니다.

### 1.1 통신 경로

```
┌─────────────────────────────────────────────────────────────┐
│                      WIA-CI 통신 구조                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [마이크/오디오] → [사운드 프로세서] → [RF 코일] → [임플란트] │
│        ↑               ↓      ↑          ↓                  │
│        │          [텔레메트리]←─────────────┘                │
│        │               ↓                                    │
│  [스마트폰 앱] ←→ [Bluetooth LE]                             │
│        ↓                                                    │
│  [클라우드/WIA API]                                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 통신 레이어

| 레이어 | 프로토콜 | 역할 |
|--------|----------|------|
| Application | WIA-CI Protocol | 데이터 교환 |
| Session | WIA-Session | 세션 관리 |
| Transport | BLE / RF | 물리적 전송 |
| Physical | Bluetooth 5.0 / 49MHz RF | 하드웨어 |

---

## 2. RF Communication Protocol (프로세서 ↔ 임플란트)

### 2.1 RF 링크 스펙

```typescript
interface RFLinkSpec {
  // 캐리어
  carrierFrequency: 49;         // MHz (ISM band)
  bandwidth: 2;                 // MHz

  // 변조
  modulation: 'ASK' | 'FSK';    // Amplitude/Frequency Shift Keying
  dataRate: 1000000;            // 1 Mbps

  // 전력
  powerTransfer: {
    enabled: true;
    frequency: 5;               // MHz
    maxPower: 40;               // mW
  };

  // 거리
  maxDistance: 10;              // mm (피부 통과)
}
```

### 2.2 RF 패킷 구조

```typescript
interface RFPacket {
  header: {
    sync: 0xAA55;               // 동기화 워드
    type: RFPacketType;
    length: number;             // payload 바이트
    sequence: number;           // 0-255
  };

  payload: StimulationPayload | TelemetryPayload | CommandPayload;

  footer: {
    crc16: number;              // CRC-16-CCITT
  };
}

type RFPacketType =
  | 0x01   // Stimulation Data
  | 0x02   // Telemetry Request
  | 0x03   // Telemetry Response
  | 0x04   // Configuration
  | 0x05   // Status
  | 0x10   // Extended (WIA Octave);
```

### 2.3 자극 데이터 페이로드

```typescript
interface StimulationPayload {
  type: 0x01;

  // 프레임 정보
  frameNumber: number;          // 32-bit
  timestamp: number;            // μs

  // 자극 명령 (압축)
  stimulations: CompressedStim[];
}

interface CompressedStim {
  // 4바이트로 압축
  // [electrode:5][amplitude:11][pulseWidth:6][flags:10]
  packed: number;
}

// 압축/해제
function packStimulation(stim: StimulationCommand): number {
  return (
    ((stim.electrode & 0x1F) << 27) |
    ((stim.amplitude & 0x7FF) << 16) |
    ((stim.pulseWidth & 0x3F) << 10) |
    (stim.flags & 0x3FF)
  );
}

function unpackStimulation(packed: number): StimulationCommand {
  return {
    electrode: (packed >> 27) & 0x1F,
    amplitude: (packed >> 16) & 0x7FF,
    pulseWidth: (packed >> 10) & 0x3F,
    flags: packed & 0x3FF
  };
}
```

### 2.4 텔레메트리 페이로드

```typescript
interface TelemetryPayload {
  type: 0x03;

  // 임플란트 상태
  implantStatus: {
    voltage: number;            // 내부 전압 (mV)
    current: number;            // 총 전류 (μA)
    temperature: number;        // 온도 (°C × 10)
    compliance: boolean;        // 전압 순응
  };

  // 전극 상태
  electrodeStatus: {
    impedances: Uint16Array;    // 22채널 임피던스 (Ω)
    activeCount: number;        // 활성 전극 수
    shortCircuits: number;      // 비트마스크
    openCircuits: number;       // 비트마스크
  };

  // NRT/ECAP 측정 (선택)
  neuralResponse?: {
    electrode: number;
    amplitude: number;          // μV
    latency: number;            // μs
    waveform: Int16Array;       // 샘플링된 파형
  };
}
```

---

## 3. BLE Communication Protocol (프로세서 ↔ 앱)

### 3.1 BLE 서비스 정의

```typescript
// WIA-CI GATT 서비스
const WIA_CI_SERVICE = {
  uuid: '0000FE01-0000-1000-8000-00805F9B34FB',

  characteristics: {
    // 상태 읽기
    STATUS: {
      uuid: '0000FE02-...',
      properties: ['read', 'notify'],
      descriptor: 'CI Status'
    },

    // 오디오 스트림 (실시간)
    AUDIO_STREAM: {
      uuid: '0000FE03-...',
      properties: ['notify'],
      descriptor: 'Audio Stream'
    },

    // 설정 쓰기
    CONFIG_WRITE: {
      uuid: '0000FE04-...',
      properties: ['write'],
      descriptor: 'Configuration'
    },

    // 텔레메트리
    TELEMETRY: {
      uuid: '0000FE05-...',
      properties: ['read', 'notify'],
      descriptor: 'Telemetry Data'
    },

    // WIA 확장 (옥타브)
    WIA_EXTENDED: {
      uuid: '0000FE10-...',
      properties: ['read', 'write', 'notify'],
      descriptor: 'WIA Octave Enhancement'
    }
  }
};
```

### 3.2 BLE 패킷 프로토콜

```typescript
interface BLEPacket {
  header: {
    version: number;            // 프로토콜 버전
    messageType: BLEMessageType;
    flags: number;
    length: number;
  };

  payload: Uint8Array;

  // MTU 분할 지원
  fragmentation?: {
    fragmentIndex: number;
    totalFragments: number;
    fragmentOffset: number;
  };
}

type BLEMessageType =
  | 0x01   // Status Request
  | 0x02   // Status Response
  | 0x03   // Audio Data
  | 0x04   // Config Get
  | 0x05   // Config Set
  | 0x06   // Telemetry
  | 0x07   // Firmware Update
  | 0x10   // WIA Octave Data
  | 0x11   // WIA Music Mode
  | 0x12   // WIA Octave Config;
```

### 3.3 실시간 오디오 스트리밍

```typescript
interface AudioStreamProtocol {
  // 스트림 파라미터
  config: {
    sampleRate: 16000;
    frameSize: 256;
    codec: 'opus' | 'pcm' | 'adpcm';
    bitrateKbps: 32;
  };

  // 프레임 구조
  frame: {
    sequence: number;           // 16-bit, wrap-around
    timestamp: number;          // ms
    audioData: Uint8Array;      // 압축된 오디오

    // WIA 확장
    wiaExtension?: {
      f0: number;               // 추정 F0
      octave: number;           // 옥타브 번호
      isMusic: boolean;         // 음악 감지
    };
  };
}

// 스트림 시작
async function startAudioStream(config: AudioStreamConfig): Promise<void> {
  await ble.write(CHARACTERISTICS.CONFIG_WRITE, {
    command: 'START_STREAM',
    config: config
  });

  ble.subscribe(CHARACTERISTICS.AUDIO_STREAM, (data) => {
    const frame = decodeAudioFrame(data);
    processAudioFrame(frame);
  });
}
```

---

## 4. Configuration Protocol

### 4.1 설정 구조

```typescript
interface CIConfiguration {
  // 버전
  version: string;
  lastModified: ISO8601;

  // 프로세서 설정
  processor: {
    sensitivity: number;        // 마이크 감도 (0-100)
    volume: number;             // 볼륨 (0-100)
    noiseReduction: 'off' | 'low' | 'medium' | 'high';
    directionalMic: boolean;
  };

  // 매핑 설정
  mapping: {
    strategy: 'CIS' | 'ACE' | 'FSP' | 'WIA-OCTAVE';
    maxima: number;             // n-of-m의 n (8-12)
    stimulationRate: number;    // pps per channel
    pulseWidth: number;         // μs
  };

  // 전극별 설정
  electrodes: ElectrodeConfig[];

  // WIA 확장 설정
  wiaExtension?: {
    octaveEnhancement: boolean;
    musicMode: 'auto' | 'on' | 'off';
    tfsEncoding: boolean;
    harmonicPriority: boolean;
  };
}

interface ElectrodeConfig {
  electrode: number;            // 1-22
  enabled: boolean;
  threshold: number;            // T-level (μA)
  comfort: number;              // C-level (μA)
  frequencyShift: number;       // Hz (tonotopic 보정)
}
```

### 4.2 설정 읽기/쓰기 프로토콜

```typescript
// 설정 읽기
interface ConfigReadRequest {
  command: 'CONFIG_GET';
  section: 'all' | 'processor' | 'mapping' | 'electrodes' | 'wia';
}

interface ConfigReadResponse {
  status: 'success' | 'error';
  config: Partial<CIConfiguration>;
  checksum: number;
}

// 설정 쓰기
interface ConfigWriteRequest {
  command: 'CONFIG_SET';
  section: string;
  data: Partial<CIConfiguration>;
  validate: boolean;            // 적용 전 검증
}

interface ConfigWriteResponse {
  status: 'success' | 'error' | 'validation_failed';
  appliedAt?: ISO8601;
  errors?: ValidationError[];
}

// 설정 검증
function validateConfig(config: CIConfiguration): ValidationResult {
  const errors: ValidationError[] = [];

  // T-level < C-level 확인
  for (const electrode of config.electrodes) {
    if (electrode.threshold >= electrode.comfort) {
      errors.push({
        field: `electrodes[${electrode.electrode}]`,
        message: 'T-level must be less than C-level'
      });
    }
  }

  // 최대 전류 확인
  const maxCurrent = config.electrodes.reduce(
    (sum, e) => sum + e.comfort,
    0
  );
  if (maxCurrent > 5000) {  // 5mA 제한
    errors.push({
      field: 'electrodes',
      message: 'Total current exceeds safety limit'
    });
  }

  return {
    valid: errors.length === 0,
    errors: errors
  };
}
```

---

## 5. Session Management

### 5.1 세션 상태

```typescript
interface CISession {
  sessionId: string;            // UUID
  deviceId: string;             // 프로세서 ID
  implantId: string;            // 임플란트 ID

  state: 'connecting' | 'connected' | 'streaming' | 'paused' | 'disconnected';

  // 연결 정보
  connection: {
    rfLinkQuality: number;      // 0-100%
    bleRssi: number;            // dBm
    latency: number;            // ms
  };

  // 스트리밍 통계
  streaming?: {
    startTime: ISO8601;
    framesSent: number;
    framesDropped: number;
    currentMode: 'speech' | 'music' | 'mixed';
  };
}
```

### 5.2 세션 제어

```typescript
// 세션 시작
interface SessionStartRequest {
  command: 'SESSION_START';
  config: SessionConfig;
}

interface SessionStartResponse {
  status: 'success' | 'error';
  sessionId: string;
  implantInfo: {
    manufacturer: string;
    model: string;
    serialNumber: string;
    electrodeCount: number;
  };
}

// 세션 종료
interface SessionEndRequest {
  command: 'SESSION_END';
  sessionId: string;
  reason: 'user' | 'timeout' | 'error';
}

// 상태 전환
async function transitionState(
  session: CISession,
  newState: SessionState
): Promise<void> {
  const validTransitions: Record<SessionState, SessionState[]> = {
    'connecting': ['connected', 'disconnected'],
    'connected': ['streaming', 'disconnected'],
    'streaming': ['paused', 'connected', 'disconnected'],
    'paused': ['streaming', 'connected', 'disconnected'],
    'disconnected': ['connecting']
  };

  if (!validTransitions[session.state].includes(newState)) {
    throw new Error(`Invalid transition: ${session.state} → ${newState}`);
  }

  session.state = newState;
  await notifyStateChange(session);
}
```

---

## 6. Real-time Streaming Protocol

### 6.1 스트림 동기화

```typescript
interface StreamSync {
  // 타임스탬프 동기화
  timestampSync: {
    processorClock: number;     // μs
    implantClock: number;       // μs
    offset: number;             // 보정값
    drift: number;              // ppm
  };

  // 버퍼 관리
  buffer: {
    targetLatency: number;      // ms (목표 지연)
    currentLatency: number;     // ms (현재 지연)
    jitterBuffer: number;       // ms (지터 버퍼)
    underruns: number;          // 버퍼 부족 횟수
    overruns: number;           // 버퍼 초과 횟수
  };
}

// 동기화 프로토콜
function syncTimestamps(): void {
  // 1. 프로세서가 타임스탬프 요청 전송
  const t1 = processorClock();
  sendTimestampRequest();

  // 2. 임플란트가 자신의 타임스탬프와 함께 응답
  const response = await receiveTimestampResponse();
  const t2 = response.implantTimestamp;
  const t3 = response.implantTimestamp;  // 응답 시점
  const t4 = processorClock();

  // 3. 오프셋 계산 (NTP 방식)
  const roundTrip = (t4 - t1) - (t3 - t2);
  const offset = ((t2 - t1) + (t3 - t4)) / 2;

  // 4. 드리프트 보정
  updateClockOffset(offset);
}
```

### 6.2 품질 제어 (QoS)

```typescript
interface StreamQoS {
  // 패킷 손실 처리
  packetLoss: {
    detection: 'sequence' | 'timestamp';
    concealment: 'repeat' | 'interpolate' | 'mute';
    maxConsecutiveLoss: number;  // 최대 연속 손실 허용
  };

  // 적응적 비트레이트
  adaptiveBitrate: {
    enabled: boolean;
    minBitrate: number;         // kbps
    maxBitrate: number;         // kbps
    stepSize: number;           // kbps
  };

  // 지연 제어
  latencyControl: {
    target: number;             // ms
    tolerance: number;          // ms
    action: 'drop' | 'accelerate' | 'buffer';
  };
}

function handlePacketLoss(
  lastSequence: number,
  currentSequence: number,
  qos: StreamQoS
): void {
  const lostCount = currentSequence - lastSequence - 1;

  if (lostCount > qos.packetLoss.maxConsecutiveLoss) {
    // 너무 많은 손실 - 재연결
    initiateReconnect();
    return;
  }

  switch (qos.packetLoss.concealment) {
    case 'repeat':
      // 마지막 프레임 반복
      repeatLastFrame(lostCount);
      break;

    case 'interpolate':
      // 프레임 보간
      interpolateFrames(lostCount);
      break;

    case 'mute':
      // 묵음 삽입
      insertSilence(lostCount);
      break;
  }
}
```

---

## 7. Error Handling

### 7.1 에러 코드

```typescript
enum CIErrorCode {
  // 연결 에러 (0x01xx)
  CONNECTION_TIMEOUT = 0x0101,
  CONNECTION_LOST = 0x0102,
  RF_LINK_FAILURE = 0x0103,
  BLE_DISCONNECTED = 0x0104,

  // 프로토콜 에러 (0x02xx)
  INVALID_PACKET = 0x0201,
  CRC_ERROR = 0x0202,
  SEQUENCE_ERROR = 0x0203,
  UNSUPPORTED_VERSION = 0x0204,

  // 설정 에러 (0x03xx)
  CONFIG_INVALID = 0x0301,
  CONFIG_OUT_OF_RANGE = 0x0302,
  ELECTRODE_DISABLED = 0x0303,

  // 하드웨어 에러 (0x04xx)
  IMPLANT_NOT_RESPONDING = 0x0401,
  ELECTRODE_SHORT = 0x0402,
  ELECTRODE_OPEN = 0x0403,
  COMPLIANCE_LIMIT = 0x0404,
  OVER_TEMPERATURE = 0x0405,
  LOW_BATTERY = 0x0406,

  // WIA 확장 에러 (0x10xx)
  WIA_NOT_SUPPORTED = 0x1001,
  OCTAVE_DETECTION_FAILED = 0x1002,
  MUSIC_MODE_UNAVAILABLE = 0x1003
}
```

### 7.2 에러 복구

```typescript
interface ErrorRecovery {
  // 자동 재시도
  autoRetry: {
    maxAttempts: number;
    backoffMs: number;          // 초기 대기 시간
    backoffMultiplier: number;  // 지수 증가율
  };

  // 폴백 동작
  fallback: {
    onRFFailure: 'bluetooth' | 'disconnect';
    onConfigError: 'default' | 'last_known';
    onOctaveFailure: 'standard_processing';
  };
}

async function handleError(
  error: CIError,
  recovery: ErrorRecovery
): Promise<RecoveryResult> {
  console.error(`CI Error: ${error.code} - ${error.message}`);

  // 심각도 분류
  const severity = classifyErrorSeverity(error.code);

  switch (severity) {
    case 'critical':
      // 즉시 중단 + 사용자 알림
      await emergencyStop();
      notifyUser(error, 'critical');
      return { success: false, action: 'stopped' };

    case 'recoverable':
      // 자동 재시도
      for (let i = 0; i < recovery.autoRetry.maxAttempts; i++) {
        const delay = recovery.autoRetry.backoffMs *
                      Math.pow(recovery.autoRetry.backoffMultiplier, i);
        await sleep(delay);

        const retryResult = await retryOperation(error.operation);
        if (retryResult.success) {
          return { success: true, action: 'recovered', attempts: i + 1 };
        }
      }
      // 폴백
      return applyFallback(error, recovery.fallback);

    case 'warning':
      // 로그만 남기고 계속
      logWarning(error);
      return { success: true, action: 'continued' };
  }
}
```

---

## 8. Security

### 8.1 인증

```typescript
interface CIAuthentication {
  // 장치 페어링
  pairing: {
    method: 'secure_simple' | 'oob' | 'passkey';
    bondingRequired: true;
  };

  // 세션 인증
  session: {
    tokenBased: boolean;
    tokenExpiry: number;        // seconds
    refreshEnabled: boolean;
  };
}

// 장치 인증
async function authenticateDevice(
  deviceId: string,
  credentials: DeviceCredentials
): Promise<AuthResult> {
  // 1. 챌린지-응답
  const challenge = generateChallenge();
  const response = await sendChallenge(deviceId, challenge);

  // 2. 서명 검증
  const valid = verifySignature(
    challenge,
    response.signature,
    credentials.publicKey
  );

  if (!valid) {
    throw new AuthenticationError('Invalid device signature');
  }

  // 3. 세션 토큰 발급
  const token = generateSessionToken(deviceId);
  return { authenticated: true, token };
}
```

### 8.2 암호화

```typescript
interface CIEncryption {
  // BLE 암호화
  ble: {
    mode: 'AES-128-CCM';
    keyExchange: 'ECDH';
  };

  // 설정 데이터 암호화
  config: {
    algorithm: 'AES-256-GCM';
    keyDerivation: 'PBKDF2';
  };

  // 환자 데이터 (HIPAA 준수)
  patientData: {
    encryption: 'AES-256-GCM';
    anonymization: boolean;
    auditLog: boolean;
  };
}

// 데이터 암호화
function encryptSensitiveData(
  data: Uint8Array,
  key: CryptoKey
): EncryptedData {
  const iv = crypto.getRandomValues(new Uint8Array(12));

  const encrypted = await crypto.subtle.encrypt(
    { name: 'AES-GCM', iv },
    key,
    data
  );

  return {
    ciphertext: new Uint8Array(encrypted),
    iv: iv,
    authTag: extractAuthTag(encrypted)
  };
}
```

---

## 9. API Interface

### 9.1 REST API

```typescript
// 프로세서 상태 조회
// GET /api/v1/processors/{processorId}/status
interface ProcessorStatusResponse {
  status: 'online' | 'offline' | 'streaming';
  battery: number;              // %
  rfLink: number;               // %
  lastSeen: ISO8601;
}

// 설정 업데이트
// PUT /api/v1/processors/{processorId}/config
interface ConfigUpdateRequest {
  config: Partial<CIConfiguration>;
  applyImmediate: boolean;
}

// 텔레메트리 조회
// GET /api/v1/processors/{processorId}/telemetry
interface TelemetryResponse {
  timestamp: ISO8601;
  impedances: number[];
  voltage: number;
  temperature: number;
}

// WIA 옥타브 설정
// PUT /api/v1/processors/{processorId}/wia/octave
interface OctaveConfigRequest {
  enabled: boolean;
  musicMode: 'auto' | 'on' | 'off';
  tfsEncoding: boolean;
}
```

### 9.2 WebSocket API

```typescript
// 실시간 스트림 구독
// ws://api.wia.ci/v1/stream/{processorId}

interface WebSocketMessage {
  type: 'audio' | 'envelope' | 'stimulation' | 'octave' | 'status';
  timestamp: number;
  data: any;
}

// 클라이언트 구현
class CIWebSocketClient {
  private ws: WebSocket;

  connect(processorId: string): void {
    this.ws = new WebSocket(`wss://api.wia.ci/v1/stream/${processorId}`);

    this.ws.onmessage = (event) => {
      const message: WebSocketMessage = JSON.parse(event.data);
      this.handleMessage(message);
    };
  }

  private handleMessage(message: WebSocketMessage): void {
    switch (message.type) {
      case 'audio':
        this.emit('audio', message.data);
        break;
      case 'octave':
        this.emit('octave', {
          f0: message.data.f0,
          octaveNumber: message.data.octaveNumber,
          isMusic: message.data.isMusic
        });
        break;
      case 'status':
        this.emit('status', message.data);
        break;
    }
  }
}
```

---

## 10. Protocol Extensions

### 10.1 WIA Octave Extension

```typescript
// 확장 패킷 타입: 0x10
interface WIAOctavePacket {
  header: {
    magic: 0x5749414F;          // 'WIAO'
    version: 1;
    type: 0x10;
  };

  // 옥타브 정보
  octaveData: {
    f0: number;                 // Hz (16.16 fixed point)
    octaveNumber: number;       // 0-8
    confidence: number;         // 0-255 → 0.0-1.0
    harmonicMask: number;       // 비트마스크 (상위 10개 하모닉)
  };

  // 음악 모드 정보
  musicMode: {
    active: boolean;
    detectionScore: number;     // 0-255
  };

  // TFS 정보 (저주파 채널)
  tfsData?: {
    channels: number;           // 적용 채널 비트마스크
    patterns: Uint8Array;       // 채널별 TFS 패턴
  };
}
```

### 10.2 펌웨어 업데이트 프로토콜

```typescript
interface FirmwareUpdateProtocol {
  // 업데이트 시작
  initiate: {
    version: string;
    size: number;
    checksum: string;           // SHA-256
    signature: string;          // 제조사 서명
  };

  // 청크 전송
  chunk: {
    offset: number;
    data: Uint8Array;
    chunkChecksum: number;      // CRC32
  };

  // 완료 및 검증
  finalize: {
    verify: boolean;
    reboot: boolean;
  };
}

// 안전한 펌웨어 업데이트
async function updateFirmware(
  firmware: Uint8Array,
  signature: string
): Promise<UpdateResult> {
  // 1. 서명 검증
  if (!verifyFirmwareSignature(firmware, signature)) {
    throw new Error('Invalid firmware signature');
  }

  // 2. 백업 생성
  await createFirmwareBackup();

  // 3. 청크 전송
  const chunkSize = 256;
  for (let offset = 0; offset < firmware.length; offset += chunkSize) {
    const chunk = firmware.slice(offset, offset + chunkSize);
    await sendFirmwareChunk(offset, chunk);
    reportProgress(offset / firmware.length);
  }

  // 4. 검증 및 적용
  const verified = await verifyFirmwareIntegrity();
  if (!verified) {
    await restoreFirmwareBackup();
    throw new Error('Firmware verification failed');
  }

  // 5. 재부팅
  await rebootProcessor();
  return { success: true };
}
```

---

## 11. Compliance

### 11.1 의료기기 규격 준수

| 규격 | 요구사항 | WIA-CI 구현 |
|------|----------|-------------|
| IEC 60601-1 | 전기 안전 | 전류 제한, 절연 |
| IEC 62304 | SW 수명주기 | 설계 문서화 |
| ISO 14971 | 위험 관리 | FMEA 수행 |
| HIPAA | 환자 정보 | 암호화, 익명화 |

### 11.2 무선 규격

| 지역 | 규격 | 요구사항 |
|------|------|----------|
| 미국 | FCC Part 15 | ISM 대역 사용 |
| 유럽 | CE/RED | 전파 적합성 |
| 한국 | KC | 전파인증 |

---

**Document ID**: WIA-CI-PHASE3-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License
