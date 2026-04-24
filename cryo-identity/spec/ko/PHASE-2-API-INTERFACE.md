# WIA Cryo-Identity API Interface 표준
## Phase 2 사양

---

**버전**: 1.0.0
**상태**: Draft
**날짜**: 2025-01
**작성자**: WIA Standards Committee
**라이선스**: MIT
**대표 색상**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [용어 정의](#용어-정의)
3. [핵심 인터페이스](#핵심-인터페이스)
4. [API Endpoint](#api-endpoint)
5. [신원 관리](#신원-관리)
6. [생체인식 서비스](#생체인식-서비스)
7. [검증 서비스](#검증-서비스)
8. [이벤트 시스템](#이벤트-시스템)
9. [오류 처리](#오류-처리)
10. [사용 예제](#사용-예제)
11. [참고문헌](#참고문헌)

---

## 개요

### 1.1 목적

WIA Cryo-Identity API Interface 표준은 동결보존된 개인의 암호화 보안 신원 레코드를 관리하기 위한 포괄적인 프로그래밍 인터페이스를 정의합니다. 이 Phase 2 사양은 Phase 1 데이터 형식을 기반으로 하여, 보존 시설, 법적 기관, 미래 소생 센터 전반의 신원 시스템과 상호작용할 수 있는 표준화된 API를 개발자에게 제공합니다.

**핵심 목표**:
- 모든 cryo-identity 작업을 위한 통합 API 제공
- 안전한 신원 등록 및 검증 지원
- 다중 요소 생체인식 인증 지원
- 신원 생애주기 전반에 걸친 암호화 무결성 보장
- 소생 시나리오를 위한 신원 복구 절차 지원

### 1.2 적용 범위

본 표준은 다음을 정의합니다:

| 구성요소 | 설명 |
|----------|------|
| **Core API** | 메인 CryoIdentity 클래스 인터페이스 |
| **REST Endpoint** | 신원 작업을 위한 HTTP API |
| **Event System** | 실시간 신원 이벤트 알림 |
| **Adapter** | 생체인식 장치 및 blockchain과의 통합 |
| **Type** | TypeScript/Python 타입 정의 |

### 1.3 Phase 1 호환성

Phase 2 API는 Phase 1 데이터 형식과 완전히 호환됩니다:

```
Phase 1: 데이터 형식 (JSON 구조)
    ↓
Phase 2: API Interface (프로그래밍 인터페이스)
    ↓
Phase 3: Protocol (통신 프로토콜)
    ↓
Phase 4: Integration (생태계 통합)
```

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **CryoIdentity** | 신원 작업을 위한 메인 API 클래스 |
| **IdentityRecord** | 완전한 신원 데이터 패키지 (Phase 1) |
| **BiometricProvider** | 생체인식 캡처 장치용 인터페이스 |
| **VerificationLevel** | 신원 매칭의 신뢰도 수준 |
| **RecoveryAgent** | 신원 복구 권한을 가진 엔티티 |
| **IdentityAnchor** | Blockchain 기반 불변 신원 참조 |

### 2.2 이벤트 유형

| 이벤트 | 설명 | 데이터 Payload |
|--------|------|----------------|
| `identity:registered` | 새 신원 생성됨 | IdentityRecord |
| `identity:verified` | 신원 검증 완료됨 | VerificationResult |
| `biometric:captured` | 생체 데이터 캡처됨 | BiometricData |
| `blockchain:anchored` | Blockchain에 신원 앵커됨 | AnchorResult |
| `recovery:initiated` | 복구 프로세스 시작됨 | RecoveryRequest |
| `error` | 오류 발생 | ErrorDetails |

---

## 핵심 인터페이스

### 3.1 CryoIdentity 클래스

신원 작업을 위한 메인 API 진입점입니다.

#### TypeScript

```typescript
class CryoIdentity {
  // 생성자
  constructor(options?: CryoIdentityOptions);

  // 신원 관리
  createIdentity(data: IdentityCreationData): Promise<IdentityRecord>;
  getIdentity(identityId: string): Promise<IdentityRecord | null>;
  updateIdentity(identityId: string, updates: Partial<IdentityRecord>): Promise<IdentityRecord>;
  deleteIdentity(identityId: string): Promise<void>;
  searchIdentities(query: IdentityQuery): Promise<IdentityRecord[]>;

  // 생체인식 작업
  captureBiometric(type: BiometricType, options?: CaptureOptions): Promise<BiometricData>;
  registerBiometric(identityId: string, biometric: BiometricData): Promise<void>;
  verifyBiometric(identityId: string, biometric: BiometricData): Promise<VerificationResult>;

  // 암호화 작업
  generateKeyPair(algorithm?: KeyAlgorithm): Promise<KeyPair>;
  signIdentity(identityId: string, privateKey: PrivateKey): Promise<Signature>;
  verifySignature(identityId: string, signature: Signature): Promise<boolean>;

  // Blockchain 작업
  anchorToBlockchain(identityId: string, network?: BlockchainNetwork): Promise<AnchorResult>;
  verifyBlockchainAnchor(identityId: string, transactionHash: string): Promise<boolean>;
  getBlockchainHistory(identityId: string): Promise<AnchorHistory[]>;

  // 복구 작업
  createRecoveryKey(identityId: string, threshold: number, totalShares: number): Promise<RecoveryKey>;
  recoverIdentity(shares: RecoveryShare[]): Promise<IdentityRecord>;
  verifyRecoveryShare(share: RecoveryShare): Promise<boolean>;

  // 이벤트 처리
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EventType>(event: T, handler: EventHandler<T>): void;

  // 상태 관리
  getStatus(): IdentitySystemStatus;
  getStatistics(): IdentityStatistics;
}
```

#### Python

```python
class CryoIdentity:
    def __init__(self, options: Optional[CryoIdentityOptions] = None):
        ...

    # 신원 관리
    async def create_identity(self, data: IdentityCreationData) -> IdentityRecord: ...
    async def get_identity(self, identity_id: str) -> Optional[IdentityRecord]: ...
    async def update_identity(self, identity_id: str, updates: dict) -> IdentityRecord: ...
    async def delete_identity(self, identity_id: str) -> None: ...
    async def search_identities(self, query: IdentityQuery) -> List[IdentityRecord]: ...

    # 생체인식 작업
    async def capture_biometric(self, type: BiometricType, options: Optional[CaptureOptions] = None) -> BiometricData: ...
    async def register_biometric(self, identity_id: str, biometric: BiometricData) -> None: ...
    async def verify_biometric(self, identity_id: str, biometric: BiometricData) -> VerificationResult: ...

    # 암호화 작업
    async def generate_key_pair(self, algorithm: KeyAlgorithm = KeyAlgorithm.ED25519) -> KeyPair: ...
    async def sign_identity(self, identity_id: str, private_key: PrivateKey) -> Signature: ...
    async def verify_signature(self, identity_id: str, signature: Signature) -> bool: ...

    # Blockchain 작업
    async def anchor_to_blockchain(self, identity_id: str, network: Optional[BlockchainNetwork] = None) -> AnchorResult: ...
    async def verify_blockchain_anchor(self, identity_id: str, transaction_hash: str) -> bool: ...
    async def get_blockchain_history(self, identity_id: str) -> List[AnchorHistory]: ...

    # 복구 작업
    async def create_recovery_key(self, identity_id: str, threshold: int, total_shares: int) -> RecoveryKey: ...
    async def recover_identity(self, shares: List[RecoveryShare]) -> IdentityRecord: ...
    async def verify_recovery_share(self, share: RecoveryShare) -> bool: ...

    # 이벤트 처리
    def on(self, event: EventType, handler: EventHandler) -> None: ...
    def off(self, event: EventType, handler: EventHandler) -> None: ...
    def once(self, event: EventType, handler: EventHandler) -> None: ...
```

### 3.2 CryoIdentityOptions

```typescript
interface CryoIdentityOptions {
  // Storage 설정
  storageProvider?: 'memory' | 'file' | 'database' | 'ipfs';
  storagePath?: string;

  // Blockchain 설정
  blockchainNetwork?: 'ethereum' | 'polygon' | 'avalanche';
  blockchainRPC?: string;

  // 생체인식 provider
  biometricProviders?: BiometricProviderConfig[];

  // 보안 설정
  encryptionKey?: string;
  requireSignatures?: boolean;
  autoAnchorToBlockchain?: boolean;

  // 로깅
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}
```

---

## API Endpoint

### 4.1 REST API 개요

모든 endpoint는 JSON payload와 함께 RESTful 규칙을 따릅니다.

**Base URL**: `https://api.wia.live/cryo-identity/v1`

**인증**: Bearer token (JWT)

**Content-Type**: `application/json`

### 4.2 Identity Endpoint

#### POST /identities

새 신원 레코드를 생성합니다.

```http
POST /identities
Authorization: Bearer <token>
Content-Type: application/json

{
  "personal": {
    "legalName": {
      "given": "John",
      "family": "Doe"
    },
    "dateOfBirth": "1990-01-15",
    "nationality": ["US"]
  },
  "biometrics": {
    "fingerprints": [...],
    "dna": {...}
  }
}
```

**응답** (201 Created):
```json
{
  "identityId": "ID-2025-000001",
  "status": "pending",
  "created": "2025-01-15T10:00:00Z"
}
```

#### GET /identities/:id

신원 레코드를 조회합니다.

```http
GET /identities/ID-2025-000001
Authorization: Bearer <token>
```

**응답** (200 OK):
```json
{
  "$schema": "https://wia.live/cryo-identity/v1/schema.json",
  "version": "1.0.0",
  "identityId": "ID-2025-000001",
  "status": "active",
  ...
}
```

#### PATCH /identities/:id

신원 레코드를 업데이트합니다.

```http
PATCH /identities/ID-2025-000001
Authorization: Bearer <token>
Content-Type: application/json

{
  "status": "preserved",
  "biometrics": {
    "retinal": {...}
  }
}
```

#### DELETE /identities/:id

신원 레코드를 삭제합니다 (소프트 삭제).

```http
DELETE /identities/ID-2025-000001
Authorization: Bearer <token>
```

#### GET /identities

쿼리 파라미터로 신원을 검색합니다.

```http
GET /identities?status=active&nationality=US&limit=10
Authorization: Bearer <token>
```

### 4.3 Biometric Endpoint

#### POST /identities/:id/biometrics

새 생체 데이터를 등록합니다.

```http
POST /identities/ID-2025-000001/biometrics
Authorization: Bearer <token>
Content-Type: application/json

{
  "type": "fingerprint",
  "finger": "right_index",
  "template": "base64-encoded-template",
  "quality": 0.95
}
```

#### POST /identities/:id/biometrics/verify

저장된 데이터와 생체 데이터를 검증합니다.

```http
POST /identities/ID-2025-000001/biometrics/verify
Authorization: Bearer <token>
Content-Type: application/json

{
  "type": "fingerprint",
  "template": "base64-encoded-template"
}
```

**응답**:
```json
{
  "verified": true,
  "confidence": 0.98,
  "matchedBiometric": "fingerprint:right_index"
}
```

### 4.4 Cryptographic Endpoint

#### POST /identities/:id/keys

암호화 키를 생성하고 등록합니다.

```http
POST /identities/ID-2025-000001/keys
Authorization: Bearer <token>
Content-Type: application/json

{
  "algorithm": "Ed25519",
  "purpose": "primary"
}
```

#### POST /identities/:id/sign

신원 데이터에 서명합니다.

```http
POST /identities/ID-2025-000001/sign
Authorization: Bearer <token>
Content-Type: application/json

{
  "privateKey": "encrypted-private-key",
  "data": "identity-hash"
}
```

### 4.5 Blockchain Endpoint

#### POST /identities/:id/blockchain/anchor

Blockchain에 신원을 앵커합니다.

```http
POST /identities/ID-2025-000001/blockchain/anchor
Authorization: Bearer <token>
Content-Type: application/json

{
  "network": "ethereum",
  "contractAddress": "0x..."
}
```

**응답**:
```json
{
  "transactionHash": "0xabc123...",
  "blockNumber": 12345678,
  "timestamp": "2025-01-15T10:00:00Z",
  "gasUsed": 21000
}
```

#### GET /identities/:id/blockchain/history

Blockchain 앵커 히스토리를 가져옵니다.

```http
GET /identities/ID-2025-000001/blockchain/history
Authorization: Bearer <token>
```

#### POST /identities/:id/blockchain/verify

Blockchain 앵커를 검증합니다.

```http
POST /identities/ID-2025-000001/blockchain/verify
Authorization: Bearer <token>
Content-Type: application/json

{
  "transactionHash": "0xabc123..."
}
```

### 4.6 Recovery Endpoint

#### POST /identities/:id/recovery

복구 키 share를 생성합니다.

```http
POST /identities/ID-2025-000001/recovery
Authorization: Bearer <token>
Content-Type: application/json

{
  "threshold": 3,
  "totalShares": 5,
  "shareHolders": ["facility", "family", "legal", "trustee", "backup"]
}
```

#### POST /recovery/verify-share

복구 share를 검증합니다.

```http
POST /recovery/verify-share
Authorization: Bearer <token>
Content-Type: application/json

{
  "shareId": "share-001",
  "shareData": "encrypted-share"
}
```

#### POST /recovery/reconstruct

Share로부터 신원을 재구성합니다.

```http
POST /recovery/reconstruct
Authorization: Bearer <token>
Content-Type: application/json

{
  "shares": [
    {"shareId": "share-001", "shareData": "..."},
    {"shareId": "share-002", "shareData": "..."},
    {"shareId": "share-003", "shareData": "..."}
  ]
}
```

### 4.7 Statistics Endpoint

#### GET /statistics

시스템 전체 통계를 가져옵니다.

```http
GET /statistics
Authorization: Bearer <token>
```

**응답**:
```json
{
  "totalIdentities": 1523,
  "activeIdentities": 1420,
  "preservedIdentities": 98,
  "revivedIdentities": 5,
  "blockchainAnchors": 1523,
  "biometricRecords": {
    "fingerprints": 3046,
    "facial": 1523,
    "dna": 1523,
    "retinal": 1420
  }
}
```

---

## 신원 관리

### 5.1 신원 생성

```typescript
import { CryoIdentity } from 'wia-cryo-identity';

const identity = new CryoIdentity({
  storageProvider: 'database',
  blockchainNetwork: 'ethereum'
});

const record = await identity.createIdentity({
  personal: {
    legalName: {
      given: 'Jane',
      family: 'Smith'
    },
    dateOfBirth: '1985-03-20',
    nationality: ['US']
  },
  biometrics: {
    fingerprints: fingerprintData,
    dna: dnaData
  }
});

console.log('생성된 신원:', record.identityId);
```

### 5.2 Identity Query 인터페이스

```typescript
interface IdentityQuery {
  // 필터 기준
  status?: IdentityStatus | IdentityStatus[];
  nationality?: string[];
  createdAfter?: Date;
  createdBefore?: Date;
  verificationLevel?: VerificationLevel;

  // 페이지네이션
  limit?: number;
  offset?: number;

  // 정렬
  sortBy?: 'created' | 'lastVerified' | 'identityId';
  sortOrder?: 'asc' | 'desc';

  // 전체 텍스트 검색
  searchText?: string;
}
```

### 5.3 배치 작업

```typescript
// 배치 신원 생성
const identities = await identity.createIdentities([
  { personal: {...}, biometrics: {...} },
  { personal: {...}, biometrics: {...} },
  { personal: {...}, biometrics: {...} }
]);

// 배치 검증
const results = await identity.verifyIdentities([
  'ID-2025-000001',
  'ID-2025-000002',
  'ID-2025-000003'
]);
```

---

## 생체인식 서비스

### 6.1 Biometric Provider 인터페이스

```typescript
interface IBiometricProvider {
  // Provider 메타데이터
  readonly type: BiometricType;
  readonly name: string;
  readonly version: string;

  // 기능
  initialize(): Promise<void>;
  isAvailable(): boolean;
  getSupportedFormats(): BiometricFormat[];

  // 캡처 작업
  capture(options?: CaptureOptions): Promise<BiometricData>;
  validateQuality(data: BiometricData): QualityScore;

  // 템플릿 작업
  createTemplate(rawData: Buffer): Promise<BiometricTemplate>;
  compareTemplates(template1: BiometricTemplate, template2: BiometricTemplate): Promise<MatchScore>;

  // 정리
  dispose(): Promise<void>;
}
```

### 6.2 지원되는 Biometric 타입

| 타입 | Provider | 표준 형식 |
|------|----------|-----------|
| Fingerprint | DigitalPersona, Suprema | ISO-19794-2 |
| Facial | Face++ | ISO-19794-5 |
| DNA | 23andMe, Illumina | VCF, FASTQ |
| Retinal | EyeLock | ISO-19794-6 |
| Voice | Nuance | Custom |

### 6.3 Biometric 캡처 예제

```typescript
// 지문 provider 초기화
const fpProvider = new FingerprintProvider({
  device: 'DigitalPersona U.are.U 5000',
  quality: 0.8
});

await fpProvider.initialize();

// 지문 캡처
const fpData = await fpProvider.capture({
  finger: 'right_index',
  timeout: 10000
});

// 신원에 등록
await identity.registerBiometric('ID-2025-000001', fpData);
```

---

## 검증 서비스

### 7.1 다중 요소 검증

```typescript
interface VerificationRequest {
  identityId: string;
  factors: VerificationFactor[];
  requiredLevel: VerificationLevel;
}

interface VerificationFactor {
  type: 'biometric' | 'cryptographic' | 'knowledge';
  data: any;
}

interface VerificationResult {
  verified: boolean;
  level: VerificationLevel;
  confidence: number;
  factors: FactorResult[];
  timestamp: Date;
}
```

### 7.2 검증 워크플로우

```typescript
const verificationRequest: VerificationRequest = {
  identityId: 'ID-2025-000001',
  factors: [
    { type: 'biometric', data: fingerprintTemplate },
    { type: 'biometric', data: facialTemplate },
    { type: 'cryptographic', data: digitalSignature }
  ],
  requiredLevel: 'cryptographic'
};

const result = await identity.verify(verificationRequest);

if (result.verified && result.confidence > 0.95) {
  console.log('신원이 다음 수준에서 검증됨:', result.level);
}
```

### 7.3 Verification Level 요구사항

| Level | 요구사항 | 일반적인 사용 사례 |
|-------|----------|--------------------|
| `basic` | 정부 발급 ID 스캔 | 초기 등록 |
| `enhanced` | ID + 1개 생체인식 | 표준 검증 |
| `biometric` | 2개 이상 생체인식 | 보존 전 검증 |
| `cryptographic` | 생체인식 + 서명 | 법적 문서화 |
| `full` | 모든 요소 + DNA | 소생 검증 |

---

## 이벤트 시스템

### 8.1 이벤트 구독

```typescript
// 신원 이벤트에 구독
identity.on('identity:registered', (record) => {
  console.log('새 신원 등록됨:', record.identityId);
  sendNotification(`신원 ${record.identityId} 생성됨`);
});

identity.on('biometric:captured', (data) => {
  console.log('생체 데이터 캡처됨:', data.type);
});

identity.on('blockchain:anchored', (result) => {
  console.log('Blockchain에 앵커됨:', result.transactionHash);
});

identity.on('error', (error) => {
  console.error('오류:', error.message);
  logError(error);
});
```

### 8.2 이벤트 필터링

```typescript
// 신원별로 이벤트 필터링
identity.on('identity:verified', (result) => {
  if (result.identityId === 'ID-2025-000001') {
    console.log('대상 신원 검증됨');
  }
}, {
  filter: (result) => result.identityId === 'ID-2025-000001'
});
```

---

## 오류 처리

### 9.1 에러 코드

```typescript
enum CryoIdentityErrorCode {
  // 신원 오류 (1xxx)
  IDENTITY_NOT_FOUND = 1001,
  IDENTITY_ALREADY_EXISTS = 1002,
  INVALID_IDENTITY_DATA = 1003,
  IDENTITY_DELETED = 1004,

  // 생체인식 오류 (2xxx)
  BIOMETRIC_CAPTURE_FAILED = 2001,
  BIOMETRIC_QUALITY_TOO_LOW = 2002,
  BIOMETRIC_MATCH_FAILED = 2003,
  DUPLICATE_BIOMETRIC = 2004,

  // 암호화 오류 (3xxx)
  KEY_GENERATION_FAILED = 3001,
  SIGNATURE_INVALID = 3002,
  ENCRYPTION_FAILED = 3003,

  // Blockchain 오류 (4xxx)
  BLOCKCHAIN_CONNECTION_FAILED = 4001,
  ANCHOR_FAILED = 4002,
  TRANSACTION_FAILED = 4003,

  // 복구 오류 (5xxx)
  INSUFFICIENT_SHARES = 5001,
  INVALID_RECOVERY_SHARE = 5002,
  RECOVERY_FAILED = 5003
}
```

### 9.2 오류 처리 예제

```typescript
try {
  await identity.createIdentity(data);
} catch (error) {
  if (error instanceof CryoIdentityError) {
    switch (error.code) {
      case CryoIdentityErrorCode.IDENTITY_ALREADY_EXISTS:
        console.log('신원이 이미 존재합니다');
        break;
      case CryoIdentityErrorCode.BIOMETRIC_QUALITY_TOO_LOW:
        console.log('생체 데이터를 다시 캡처해주세요');
        break;
      default:
        console.error('알 수 없는 오류:', error.message);
    }
  }
}
```

---

## 사용 예제

### 10.1 완전한 신원 등록

```typescript
import { CryoIdentity, FingerprintProvider, DNAProvider } from 'wia-cryo-identity';

async function registerNewIdentity() {
  const identity = new CryoIdentity({
    storageProvider: 'database',
    blockchainNetwork: 'ethereum',
    autoAnchorToBlockchain: true
  });

  // 생체 데이터 캡처
  const fpProvider = new FingerprintProvider();
  await fpProvider.initialize();

  const fingerprints = [];
  for (const finger of ['right_index', 'left_index']) {
    const fp = await fpProvider.capture({ finger });
    fingerprints.push(fp);
  }

  // 신원 생성
  const record = await identity.createIdentity({
    personal: {
      legalName: { given: 'Alice', family: 'Johnson' },
      dateOfBirth: '1992-07-10',
      nationality: ['US']
    },
    biometrics: {
      fingerprints,
      dna: await captureDNA()
    }
  });

  // 암호화 키 생성
  const keyPair = await identity.generateKeyPair();
  await identity.registerKey(record.identityId, keyPair.publicKey);

  // 신원 서명
  const signature = await identity.signIdentity(record.identityId, keyPair.privateKey);

  console.log('신원 등록됨:', record.identityId);
  console.log('Blockchain 앵커:', record.cryptographic.blockchainAnchors[0]);

  return record;
}
```

### 10.2 신원 검증 플로우

```typescript
async function verifyIdentity(identityId: string) {
  const identity = new CryoIdentity();

  // 현재 지문 캡처
  const fpProvider = new FingerprintProvider();
  await fpProvider.initialize();
  const currentFP = await fpProvider.capture({ finger: 'right_index' });

  // 저장된 데이터와 검증
  const result = await identity.verifyBiometric(identityId, currentFP);

  if (result.verified && result.confidence > 0.95) {
    console.log('신원 검증 성공');
    console.log('신뢰도:', result.confidence);
    return true;
  } else {
    console.log('검증 실패');
    return false;
  }
}
```

### 10.3 복구 키 생성

```typescript
async function setupRecovery(identityId: string) {
  const identity = new CryoIdentity();

  // 3-of-5 임계값으로 Shamir 비밀 공유 생성
  const recoveryKey = await identity.createRecoveryKey(identityId, 3, 5);

  // 다양한 엔티티에 share 배포
  const shares = recoveryKey.shares;
  await distributeShare(shares[0], 'preservation_facility');
  await distributeShare(shares[1], 'family_member');
  await distributeShare(shares[2], 'legal_representative');
  await distributeShare(shares[3], 'backup_trustee');
  await distributeShare(shares[4], 'emergency_contact');

  console.log('3-of-5 임계값으로 복구 키 생성됨');
}
```

### 10.4 신원 복구

```typescript
async function recoverIdentity(shares: RecoveryShare[]) {
  const identity = new CryoIdentity();

  // 충분한 share가 있는지 확인
  if (shares.length < 3) {
    throw new Error('복구를 위한 share가 부족합니다');
  }

  // 각 share 검증
  for (const share of shares) {
    const valid = await identity.verifyRecoveryShare(share);
    if (!valid) {
      throw new Error(`잘못된 share: ${share.shareId}`);
    }
  }

  // 신원 복구
  const record = await identity.recoverIdentity(shares);
  console.log('신원 복구됨:', record.identityId);

  return record;
}
```

### 10.5 Blockchain 앵커 검증

```typescript
async function verifyBlockchainAnchor(identityId: string, txHash: string) {
  const identity = new CryoIdentity({
    blockchainNetwork: 'ethereum',
    blockchainRPC: 'https://mainnet.infura.io/v3/YOUR_KEY'
  });

  const verified = await identity.verifyBlockchainAnchor(identityId, txHash);

  if (verified) {
    console.log('Blockchain 앵커 검증됨');

    // 전체 히스토리 가져오기
    const history = await identity.getBlockchainHistory(identityId);
    console.log('총 앵커 수:', history.length);
    history.forEach(anchor => {
      console.log(`- 블록 ${anchor.blockNumber}: ${anchor.transactionHash}`);
    });
  }
}
```

---

## 참고문헌

### 관련 표준

- [WIA Cryo-Identity 데이터 형식 (Phase 1)](/cryo-identity/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Cryo-Identity Protocol (Phase 3)](/cryo-identity/spec/PHASE-3-PROTOCOL.md)
- [ISO/IEC 19794 - Biometric Data Interchange Formats](https://www.iso.org/standard/50867.html)

### 암호화 표준

- [NIST FIPS 186-4 - Digital Signature Standard](https://csrc.nist.gov/publications/detail/fips/186/4/final)
- [RFC 8032 - Edwards-Curve Digital Signature Algorithm (EdDSA)](https://tools.ietf.org/html/rfc8032)

### Blockchain 표준

- [ERC-725 - Ethereum Identity Standard](https://eips.ethereum.org/EIPS/eip-725)
- [W3C Decentralized Identifiers (DIDs)](https://www.w3.org/TR/did-core/)

---

<div align="center">

**WIA Cryo-Identity API Interface 표준 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
