# 제4장: 불변 기록을 위한 블록체인 통합

**WIA-AGRI-016 전자책 시리즈**

---

## 이력 추적에 블록체인이 필요한 이유

기존 데이터베이스는 본질적인 신뢰 문제가 있습니다:
- 중앙 집중식 제어 (단일 장애 지점)
- 가변 기록 (소급 변경 가능)
- 신뢰 의존성 (데이터베이스 소유자를 신뢰해야 함)

블록체인은 다음을 제공합니다:
- **불변성:** 기록이 작성되면 변경할 수 없음
- **분산화:** 단일 제어 지점이 없음
- **투명성:** 모든 참가자가 기록을 검증할 수 있음
- **신뢰:** 암호화 증명이 제도적 신뢰를 대체

---

## 식품 이력 추적을 위한 블록체인 아키텍처

### 하이브리드 접근 방식

```
┌─────────────────────────────────────────────────┐
│           애플리케이션 계층                       │
│   (모바일 앱, 웹 포털, 분석)                     │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│         이력 추적 미들웨어                        │
│  (비즈니스 로직, 접근 제어, 검증)                │
└──────────────────┬──────────────────────────────┘
                   │
          ┌────────┴────────┐
          ▼                  ▼
┌──────────────┐    ┌──────────────┐
│  오프체인    │    │  블록체인     │
│  데이터베이스│    │  네트워크     │
│              │    │              │
│ • 전체 데이터│    │ • 해시       │
│ • 검색 가능  │    │ • 증명       │
│ • 빠름       │    │ • 불변       │
└──────────────┘    └──────────────┘
```

**근거:**
- 성능을 위해 전체 이벤트 데이터를 오프체인에 저장
- 불변성을 위해 암호화 해시를 온체인에 저장
- 두 세계의 장점: 속도 + 보안

---

## 스마트 계약 구현

### 배치 레지스트리 계약

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract FoodTraceabilityRegistry {
    struct Batch {
        bytes32 dataHash;
        string ipfsUrl;
        address registeredBy;
        uint256 timestamp;
        bool exists;
    }

    struct Event {
        bytes32 eventHash;
        uint256 timestamp;
        address recordedBy;
    }

    mapping(string => Batch) public batches;
    mapping(string => Event[]) public batchEvents;

    event BatchRegistered(
        string indexed batchId,
        bytes32 dataHash,
        string ipfsUrl,
        address registeredBy,
        uint256 timestamp
    );

    event EventRecorded(
        string indexed batchId,
        bytes32 eventHash,
        address recordedBy,
        uint256 timestamp
    );

    function registerBatch(
        string memory batchId,
        bytes32 dataHash,
        string memory ipfsUrl
    ) public {
        require(!batches[batchId].exists, "배치가 이미 등록되었습니다");

        batches[batchId] = Batch({
            dataHash: dataHash,
            ipfsUrl: ipfsUrl,
            registeredBy: msg.sender,
            timestamp: block.timestamp,
            exists: true
        });

        emit BatchRegistered(batchId, dataHash, ipfsUrl, msg.sender, block.timestamp);
    }

    function recordEvent(
        string memory batchId,
        bytes32 eventHash
    ) public {
        require(batches[batchId].exists, "배치가 등록되지 않았습니다");

        batchEvents[batchId].push(Event({
            eventHash: eventHash,
            timestamp: block.timestamp,
            recordedBy: msg.sender
        }));

        emit EventRecorded(batchId, eventHash, msg.sender, block.timestamp);
    }

    function verifyBatch(
        string memory batchId,
        bytes32 dataHash
    ) public view returns (bool) {
        return batches[batchId].exists && batches[batchId].dataHash == dataHash;
    }
}
```

---

## 이력 추적 시스템과의 통합

### JavaScript SDK

```javascript
const { ethers } = require('ethers');

class BlockchainTraceability {
  constructor(config) {
    this.provider = new ethers.JsonRpcProvider(config.rpcUrl);
    this.wallet = new ethers.Wallet(config.privateKey, this.provider);

    this.batchRegistry = new ethers.Contract(
      config.batchRegistryAddress,
      BatchRegistryABI,
      this.wallet
    );
  }

  async registerBatch(batchData) {
    // 배치 데이터의 해시 생성
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(batchData))
    );

    // IPFS에 전체 데이터 업로드
    const ipfsHash = await this.uploadToIPFS(batchData);

    // 블록체인에 제출
    const tx = await this.batchRegistry.registerBatch(
      batchData.batchId,
      dataHash,
      `ipfs://${ipfsHash}`
    );

    const receipt = await tx.wait();

    return {
      batchId: batchData.batchId,
      transactionHash: receipt.hash,
      blockNumber: receipt.blockNumber,
      dataHash: dataHash,
      ipfsUrl: `ipfs://${ipfsHash}`,
      gasUsed: receipt.gasUsed.toString()
    };
  }

  async recordEvent(batchId, eventData) {
    // 이벤트 데이터 해시
    const eventHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(eventData))
    );

    // 블록체인에 제출
    const tx = await this.batchRegistry.recordEvent(batchId, eventHash);
    const receipt = await tx.wait();

    // 오프체인에 전체 이벤트 저장
    await this.storeEvent(eventData);

    return {
      batchId: batchId,
      eventId: eventData.eventId,
      eventHash: eventHash,
      transactionHash: receipt.hash,
      blockNumber: receipt.blockNumber
    };
  }

  async verifyBatch(batchId, batchData) {
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(batchData))
    );

    const isValid = await this.batchRegistry.verifyBatch(batchId, dataHash);

    return {
      batchId: batchId,
      verified: isValid,
      dataHash: dataHash
    };
  }
}

// 사용 예시
const blockchain = new BlockchainTraceability({
  rpcUrl: 'https://mainnet.infura.io/v3/YOUR-PROJECT-ID',
  privateKey: process.env.PRIVATE_KEY,
  batchRegistryAddress: '0x...'
});

// 새 배치 등록
const batchData = {
  batchId: '01234567890128.LOT2025001',
  product: '유기농 사과',
  origin: 'ABC 유기농 농장',
  harvestDate: '2025-12-01',
  quantity: 1000,
  certifications: ['USDA 유기농']
};

const registration = await blockchain.registerBatch(batchData);
console.log('배치 등록됨:', registration.transactionHash);
```

---

## W3C 검증 가능한 자격 증명

### 자격 증명 구조

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/traceability/v1"
  ],
  "type": ["VerifiableCredential", "OrganicCertificationCredential"],
  "id": "urn:uuid:a1234567-89ab-cdef-0123-456789abcdef",
  "issuer": {
    "id": "did:web:rda.go.kr",
    "name": "농촌진흥청 유기농 인증 프로그램"
  },
  "issuanceDate": "2025-01-15T00:00:00Z",
  "expirationDate": "2027-01-15T23:59:59Z",
  "credentialSubject": {
    "id": "did:web:abcfarm.com",
    "name": "ABC 유기농 농장",
    "certifications": [
      {
        "type": "유기농 인증",
        "certificationNumber": "ORG-123456",
        "scope": ["신선 농산물", "사과"],
        "certifiedSince": "2020-01-15"
      }
    ]
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T08:00:00Z",
    "verificationMethod": "did:web:rda.go.kr#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQjecWRftX..."
  }
}
```

---

## 블록체인 네트워크 선택

### 공개 vs. 프라이빗

**공개 블록체인 (이더리움, 폴리곤):**
- 장점: 최대 투명성, 중앙 권한 없음
- 단점: 높은 비용, 느린 거래, 공개 데이터

**프라이빗/컨소시엄 블록체인 (하이퍼레저 패브릭):**
- 장점: 낮은 비용, 빠름, 권한 있는 접근
- 단점: 덜 분산화, 거버넌스 필요

### 규모별 권장 사항

| 규모 | 권장 사항 | 이유 |
|------|----------|------|
| 소규모/중규모 | 폴리곤 | 낮은 가스 수수료, 빠름, 이더리움 호환 |
| 대기업 | 하이퍼레저 패브릭 | 높은 처리량, 개인 정보 제어 |
| 글로벌 컨소시엄 | 이더리움 L2 (Arbitrum/Optimism) | 보안 + 확장성 |

---

## 비용 최적화

### 가스 최적화 기법

```solidity
// 비용이 많이 듦: 온체인에 전체 문자열 저장
function registerBatchBad(string memory batchId, string memory fullData) public {
    batches[batchId] = fullData;  // 매우 비쌈!
}

// 최적화: 해시만 저장
function registerBatchGood(string memory batchId, bytes32 dataHash, string memory ipfsUrl) public {
    batches[batchId] = Batch(dataHash, ipfsUrl, msg.sender, block.timestamp);
}
```

---

## 장 요약

블록체인은 식품 이력 추적에 불변성과 신뢰를 추가합니다:

**주요 이점:**
- 변조 방지 기록
- 분산 검증
- 진위성의 암호화 증명
- 중개자 없는 자동화된 신뢰

**구현 접근 방식:**
- 하이브리드 아키텍처 (오프체인 + 온체인)
- 레지스트리 및 검증을 위한 스마트 계약
- 분산 저장을 위한 IPFS
- 인증을 위한 W3C 검증 가능한 자격 증명

---

## 다음 장

**제5장: IoT 센서 및 실시간 모니터링**

지속적인 콜드체인 모니터링 및 품질 추적을 위한 IoT 센서 통합 방법을 배웁니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
