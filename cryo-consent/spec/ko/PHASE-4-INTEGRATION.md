# WIA Cryo-Consent Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [법적 시스템 통합](#법적-시스템-통합)
3. [의료 시스템 통합](#의료-시스템-통합)
4. [Blockchain 통합](#blockchain-통합)
5. [신원 확인 통합](#신원-확인-통합)
6. [데이터 교환 표준](#데이터-교환-표준)
7. [구현 예제](#구현-예제)
8. [버전 이력](#버전-이력)

---

## 개요

### 1.1 목적

WIA Cryo-Consent Integration Standard는 동의 관리 시스템을 법적 프레임워크, 의료 시스템, blockchain 네트워크 및 신원 확인 서비스와 연결하기 위한 포괄적인 통합 패턴 및 인터페이스를 정의합니다. 이 표준은 법적 유효성과 규제 준수를 유지하면서 전체 냉동보존 생태계에 걸쳐 원활한 상호운용성을 보장합니다.

**핵심 목표**:
- 법적 문서 관리 시스템과 통합 지원
- 전자 공증 및 디지털 증인 플랫폼 지원
- 의료 기록 시스템과 인터페이스 (HL7 FHIR, DICOM)
- 불변 감사 추적을 위한 blockchain anchoring 구현
- 신원 확인 및 KYC 서비스와 통합
- 규제 준수 보고 및 감사 촉진
- 국경 간 법적 프레임워크 동기화 지원

### 1.2 통합 계층

| Layer | 설명 | Standards |
|-------|------|-----------|
| Legal Integration | 법원 시스템, 공증인, 법적 문서 저장소 | eLaw, eNotary, ACORD |
| Healthcare Integration | EHR/EMR 시스템, 의료 기록 | HL7 FHIR, DICOM, CDA |
| Blockchain Integration | 분산 원장 anchoring | Ethereum, Hyperledger, IPFS |
| Identity Integration | KYC, 생체 인식 확인 | OAuth 2.0, OpenID Connect, FIDO2 |
| Compliance Integration | 규제 보고, 감사 | GDPR, HIPAA, SOC2 |

### 1.3 아키텍처 개요

```
┌────────────────────────────────────────────────────────────┐
│              WIA Cryo-Consent 시스템                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │           Consent Management Core                    │  │
│  └────────────┬──────────────┬─────────────┬────────────┘  │
│               │              │             │                │
│    ┌──────────▼────┐  ┌─────▼─────┐  ┌───▼────────┐      │
│    │ Legal Adapter │  │Healthcare │  │Blockchain  │      │
│    │               │  │  Adapter  │  │  Adapter   │      │
│    └──────┬────────┘  └─────┬─────┘  └───┬────────┘      │
└───────────┼──────────────────┼────────────┼───────────────┘
            │                  │            │
   ┌────────▼────────┐ ┌──────▼──────┐ ┌──▼──────────┐
   │  Legal Systems  │ │   EHR/EMR   │ │  Ethereum   │
   │  - Courts       │ │   Systems   │ │  Hyperledger│
   │  - Notaries     │ │   - Epic    │ │  IPFS       │
   │  - DocuSign     │ │   - Cerner  │ └─────────────┘
   └─────────────────┘ └─────────────┘
```

---

## 법적 시스템 통합

### 2.1 전자 공증 통합

#### 2.1.1 공증 서비스 인터페이스

```typescript
interface NotarizationRequest {
  documentId: string;
  documentType: 'consent_record' | 'consent_modification' | 'consent_revocation';
  documentHash: string;
  signers: Array<{
    id: string;
    name: string;
    email: string;
    role: 'grantor' | 'witness' | 'guardian';
  }>;
  jurisdiction: string;
  notaryPreference?: {
    notaryId?: string;
    language?: string;
    timezone?: string;
  };
}

interface NotarizationResponse {
  notarizationId: string;
  status: 'pending' | 'scheduled' | 'in_progress' | 'completed' | 'failed';
  notary: {
    id: string;
    name: string;
    licenseNumber: string;
    jurisdiction: string;
    commission: {
      issuedDate: string;
      expiryDate: string;
    };
  };
  session?: {
    scheduledAt: string;
    meetingUrl: string;
    recordingUrl?: string;
  };
  certificate?: {
    certificateId: string;
    issuedAt: string;
    sealNumber: string;
    certificateUrl: string;
  };
}
```

#### 2.1.2 공증 제공자

| Provider | Integration Type | 지원 관할권 |
|----------|------------------|-----------|
| DocuSign Notary | REST API | US (50개 주) |
| Notarize.com | REST API + Webhook | US, Canada |
| NotaryCam | REST API | US, UK, EU |
| Proof | REST API + SDK | US, Canada, Australia |
| eNotaryLog | REST API | US |

#### 2.1.3 DocuSign 통합 예제

```typescript
import axios from 'axios';

class DocuSignNotaryAdapter {
  private baseUrl = 'https://demo.docusign.net/restapi';
  private accountId: string;
  private accessToken: string;

  constructor(accountId: string, accessToken: string) {
    this.accountId = accountId;
    this.accessToken = accessToken;
  }

  async createNotarizationEnvelope(
    request: NotarizationRequest
  ): Promise<NotarizationResponse> {
    const envelope = {
      emailSubject: '냉동보존 동의 - 공증 필요',
      status: 'sent',
      documents: [
        {
          documentId: '1',
          name: '동의 문서',
          documentBase64: await this.getDocumentBase64(request.documentId),
          fileExtension: 'pdf'
        }
      ],
      recipients: {
        signers: request.signers.map((signer, index) => ({
          email: signer.email,
          name: signer.name,
          recipientId: (index + 1).toString(),
          routingOrder: (index + 1).toString(),
          tabs: {
            signHereTabs: [
              {
                documentId: '1',
                pageNumber: '1',
                xPosition: '100',
                yPosition: '100'
              }
            ]
          }
        })),
        inPersonSigners: [
          {
            email: request.signers[0].email,
            name: request.signers[0].name,
            recipientId: '99',
            hostEmail: 'notary@example.com',
            hostName: 'Assigned Notary',
            requireIdLookup: 'true',
            identityVerification: {
              workflowId: 'notary-verification'
            }
          }
        ]
      },
      notification: {
        useAccountDefaults: 'false',
        reminders: {
          reminderEnabled: 'true',
          reminderDelay: '2',
          reminderFrequency: '2'
        }
      }
    };

    const response = await axios.post(
      `${this.baseUrl}/v2.1/accounts/${this.accountId}/envelopes`,
      envelope,
      {
        headers: {
          'Authorization': `Bearer ${this.accessToken}`,
          'Content-Type': 'application/json'
        }
      }
    );

    return {
      notarizationId: response.data.envelopeId,
      status: 'pending',
      notary: {
        id: 'assigned-by-docusign',
        name: '할당 예정',
        licenseNumber: '',
        jurisdiction: request.jurisdiction,
        commission: {
          issuedDate: '',
          expiryDate: ''
        }
      }
    };
  }

  async getNotarizationStatus(
    notarizationId: string
  ): Promise<NotarizationResponse> {
    const response = await axios.get(
      `${this.baseUrl}/v2.1/accounts/${this.accountId}/envelopes/${notarizationId}`,
      {
        headers: {
          'Authorization': `Bearer ${this.accessToken}`
        }
      }
    );

    return this.mapDocuSignResponse(response.data);
  }

  private async getDocumentBase64(documentId: string): Promise<string> {
    // 문서 가져오기 및 base64로 변환
    // 구현은 문서 저장소에 따라 다름
    return '';
  }

  private mapDocuSignResponse(data: any): NotarizationResponse {
    // DocuSign 응답을 표준 형식으로 매핑
    return {
      notarizationId: data.envelopeId,
      status: this.mapStatus(data.status),
      notary: {
        id: data.notaryId || '',
        name: data.notaryName || '',
        licenseNumber: '',
        jurisdiction: '',
        commission: {
          issuedDate: '',
          expiryDate: ''
        }
      }
    };
  }

  private mapStatus(docusignStatus: string): string {
    const statusMap: Record<string, string> = {
      'created': 'pending',
      'sent': 'pending',
      'delivered': 'scheduled',
      'signed': 'in_progress',
      'completed': 'completed',
      'declined': 'failed',
      'voided': 'failed'
    };
    return statusMap[docusignStatus] || 'pending';
  }
}
```

### 2.2 법적 문서 저장소 통합

#### 2.2.1 문서 저장소 제공자

| Provider | Type | 기능 | Compliance |
|----------|------|------|------------|
| Box Legal Hold | Cloud Storage | 버전 제어, 보존 정책 | HIPAA, GDPR |
| NetDocuments | Legal DMS | 사건 중심, 암호화 | SOC2, ISO 27001 |
| iManage | Enterprise DMS | 법적 워크플로, 검색 | SEC, FINRA |
| SharePoint + AIP | On-premise/Cloud | 분류, DLP | GDPR, HIPAA |

#### 2.2.2 문서 저장소 인터페이스

```python
from abc import ABC, abstractmethod
from typing import Optional, Dict, List
from datetime import datetime

class DocumentStorageAdapter(ABC):
    @abstractmethod
    def store_document(
        self,
        document_id: str,
        document_type: str,
        content: bytes,
        metadata: Dict
    ) -> str:
        """법적 문서를 저장하고 저장소 ID를 반환합니다."""
        pass

    @abstractmethod
    def retrieve_document(
        self,
        storage_id: str,
        version: Optional[int] = None
    ) -> tuple[bytes, Dict]:
        """문서 내용 및 메타데이터를 검색합니다."""
        pass

    @abstractmethod
    def create_legal_hold(
        self,
        document_ids: List[str],
        hold_reason: str,
        hold_until: Optional[datetime] = None
    ) -> str:
        """문서에 대한 법적 보존을 생성합니다."""
        pass

    @abstractmethod
    def verify_integrity(
        self,
        storage_id: str,
        expected_hash: str
    ) -> bool:
        """hash에 대해 문서 무결성을 검증합니다."""
        pass

class BoxLegalHoldAdapter(DocumentStorageAdapter):
    def __init__(self, client_id: str, client_secret: str):
        from boxsdk import OAuth2, Client
        auth = OAuth2(
            client_id=client_id,
            client_secret=client_secret,
            access_token=self._get_access_token()
        )
        self.client = Client(auth)

    def store_document(
        self,
        document_id: str,
        document_type: str,
        content: bytes,
        metadata: Dict
    ) -> str:
        """메타데이터와 함께 Box에 문서를 저장합니다."""
        folder_id = self._get_or_create_folder('cryo-consent', metadata.get('jurisdiction'))

        # 파일 업로드
        file = self.client.folder(folder_id).upload_stream(
            content,
            f"{document_id}.pdf"
        )

        # 메타데이터 추가
        metadata_template = {
            'document_type': document_type,
            'consent_id': metadata.get('consent_id'),
            'grantor_id': metadata.get('grantor_id'),
            'jurisdiction': metadata.get('jurisdiction'),
            'created_at': datetime.utcnow().isoformat(),
            'hash': metadata.get('hash')
        }

        file.metadata(scope='enterprise', template='legalDocument').set(metadata_template)

        return file.id

    def retrieve_document(
        self,
        storage_id: str,
        version: Optional[int] = None
    ) -> tuple[bytes, Dict]:
        """Box에서 문서를 검색합니다."""
        file = self.client.file(storage_id)

        if version:
            # 특정 버전 가져오기
            versions = file.get_versions()
            file_version = next(v for v in versions if v.version_number == version)
            content = file_version.download_to(None)
        else:
            # 최신 버전 가져오기
            content = file.content()

        # 메타데이터 가져오기
        metadata_instance = file.metadata(scope='enterprise', template='legalDocument').get()
        metadata = dict(metadata_instance)

        return (content, metadata)

    def create_legal_hold(
        self,
        document_ids: List[str],
        hold_reason: str,
        hold_until: Optional[datetime] = None
    ) -> str:
        """Box에서 법적 보존 정책을 생성합니다."""
        legal_hold_policy = self.client.create_legal_hold_policy(
            policy_name=f"Consent Hold - {hold_reason}",
            description=hold_reason,
            is_ongoing=(hold_until is None),
            release_date=hold_until.isoformat() if hold_until else None
        )

        # 법적 보존에 파일 할당
        for doc_id in document_ids:
            file = self.client.file(doc_id)
            legal_hold_policy.assign(file)

        return legal_hold_policy.id

    def verify_integrity(
        self,
        storage_id: str,
        expected_hash: str
    ) -> bool:
        """문서 SHA-256 hash를 검증합니다."""
        file = self.client.file(storage_id)
        file_info = file.get()

        # Box는 SHA-1을 제공하므로 SHA-256을 계산해야 합니다
        content = file.content()
        import hashlib
        actual_hash = hashlib.sha256(content).hexdigest()

        return actual_hash == expected_hash

    def _get_access_token(self) -> str:
        # OAuth token 가져오기 구현
        return ''

    def _get_or_create_folder(self, base_name: str, jurisdiction: str) -> str:
        # 폴더 관리 구현
        return '0'  # Root folder
```

### 2.3 법원 시스템 통합

#### 2.3.1 제출 표준

| Jurisdiction | System | Standard | Format |
|--------------|--------|----------|--------|
| US Federal | CM/ECF | NIEM | XML |
| California | EFM | LegalXML | XML |
| New York | NYSCEF | OASIS LegalXML | XML/PDF |
| UK | CE-File | CJSE Standard | XML/PDF |
| EU | e-CODEX | OASIS LegalDocML | XML |

#### 2.3.2 전자 제출 인터페이스

```typescript
interface CourtFilingRequest {
  filingId: string;
  court: {
    jurisdiction: string;
    courtId: string;
    division?: string;
  };
  caseNumber?: string;
  filingType: 'new_case' | 'subsequent_filing';
  documentType: 'petition' | 'consent_verification' | 'objection';
  documents: Array<{
    documentId: string;
    title: string;
    content: string; // Base64 인코딩
    format: 'pdf' | 'xml';
  }>;
  parties: Array<{
    role: 'petitioner' | 'respondent' | 'interested_party';
    name: string;
    address: string;
    representation?: {
      attorneyName: string;
      barNumber: string;
    };
  }>;
  filingFee?: {
    amount: number;
    currency: string;
    paymentMethod: string;
  };
}

interface CourtFilingResponse {
  filingId: string;
  status: 'submitted' | 'accepted' | 'rejected' | 'pending_review';
  caseNumber: string;
  filingDate: string;
  confirmationNumber: string;
  nextAction?: {
    description: string;
    dueDate: string;
  };
  rejectionReason?: string;
}
```

---

## 의료 시스템 통합

### 3.1 HL7 FHIR 통합

#### 3.1.1 Consent Resource 매핑

```json
{
  "resourceType": "Consent",
  "id": "cryo-consent-001",
  "identifier": [
    {
      "system": "https://wia.live/cryo-consent",
      "value": "550e8400-e29b-41d4-a716-446655440001"
    }
  ],
  "status": "active",
  "scope": {
    "coding": [
      {
        "system": "http://terminology.hl7.org/CodeSystem/consentscope",
        "code": "treatment",
        "display": "Treatment"
      }
    ]
  },
  "category": [
    {
      "coding": [
        {
          "system": "http://loinc.org",
          "code": "59284-0",
          "display": "Consent Document"
        }
      ]
    }
  ],
  "patient": {
    "reference": "Patient/PERSON-001"
  },
  "dateTime": "2025-01-15T10:30:00Z",
  "performer": [
    {
      "reference": "Patient/PERSON-001"
    }
  ],
  "organization": [
    {
      "reference": "Organization/FAC-001"
    }
  ],
  "sourceReference": {
    "reference": "DocumentReference/consent-document-001"
  },
  "policy": [
    {
      "authority": "https://wia.live",
      "uri": "https://wia.live/cryo-consent/v1/schema.json"
    }
  ],
  "provision": {
    "type": "permit",
    "period": {
      "start": "2025-01-15T00:00:00Z"
    },
    "purpose": [
      {
        "system": "http://terminology.hl7.org/CodeSystem/v3-ActReason",
        "code": "TREAT",
        "display": "Treatment"
      }
    ],
    "data": [
      {
        "meaning": "related",
        "reference": {
          "reference": "Procedure/cryopreservation-001"
        }
      }
    ]
  },
  "extension": [
    {
      "url": "https://wia.live/fhir/StructureDefinition/cryo-consent-scope",
      "extension": [
        {
          "url": "preservation",
          "valueBoolean": true
        },
        {
          "url": "preservationType",
          "valueCodeableConcept": {
            "coding": [
              {
                "system": "https://wia.live/cryo-preservation/types",
                "code": "whole_body"
              }
            ]
          }
        },
        {
          "url": "research",
          "valueBoolean": true
        },
        {
          "url": "researchCategories",
          "valueCodeableConcept": {
            "coding": [
              {
                "system": "https://wia.live/research/categories",
                "code": "medical"
              },
              {
                "system": "https://wia.live/research/categories",
                "code": "scientific"
              }
            ]
          }
        },
        {
          "url": "revival",
          "valueBoolean": true
        },
        {
          "url": "minimumViabilityThreshold",
          "valueDecimal": 0.7
        }
      ]
    }
  ]
}
```

#### 3.1.2 FHIR Adapter 구현

```typescript
import { Client } from 'fhir-kit-client';

class FHIRConsentAdapter {
  private client: Client;

  constructor(baseUrl: string, credentials: any) {
    this.client = new Client({
      baseUrl: baseUrl,
      customHeaders: {
        Authorization: `Bearer ${credentials.accessToken}`
      }
    });
  }

  async createConsent(consentRecord: any): Promise<any> {
    const fhirConsent = this.mapToFHIRConsent(consentRecord);

    const response = await this.client.create({
      resourceType: 'Consent',
      body: fhirConsent
    });

    return response;
  }

  async updateConsent(consentId: string, consentRecord: any): Promise<any> {
    const fhirConsent = this.mapToFHIRConsent(consentRecord);
    fhirConsent.id = consentId;

    const response = await this.client.update({
      resourceType: 'Consent',
      id: consentId,
      body: fhirConsent
    });

    return response;
  }

  async getConsent(consentId: string): Promise<any> {
    const response = await this.client.read({
      resourceType: 'Consent',
      id: consentId
    });

    return this.mapFromFHIRConsent(response);
  }

  async searchConsents(params: {
    patient?: string;
    status?: string;
    date?: string;
  }): Promise<any[]> {
    const searchParams: any = {};

    if (params.patient) searchParams.patient = params.patient;
    if (params.status) searchParams.status = params.status;
    if (params.date) searchParams.date = params.date;

    const response = await this.client.search({
      resourceType: 'Consent',
      searchParams: searchParams
    });

    return response.entry?.map((entry: any) =>
      this.mapFromFHIRConsent(entry.resource)
    ) || [];
  }

  private mapToFHIRConsent(consentRecord: any): any {
    return {
      resourceType: 'Consent',
      identifier: [
        {
          system: 'https://wia.live/cryo-consent',
          value: consentRecord.consentId
        }
      ],
      status: this.mapStatus(consentRecord.consent.status),
      scope: {
        coding: [
          {
            system: 'http://terminology.hl7.org/CodeSystem/consentscope',
            code: 'treatment'
          }
        ]
      },
      category: [
        {
          coding: [
            {
              system: 'http://loinc.org',
              code: '59284-0',
              display: 'Consent Document'
            }
          ]
        }
      ],
      patient: {
        reference: `Patient/${consentRecord.grantor.id}`
      },
      dateTime: consentRecord.timestamp.created,
      extension: this.buildExtensions(consentRecord.consent.scope)
    };
  }

  private mapFromFHIRConsent(fhirConsent: any): any {
    // FHIR Consent를 WIA 형식으로 다시 매핑
    return {
      consentId: fhirConsent.identifier?.[0]?.value,
      status: fhirConsent.status,
      // ... 추가 매핑
    };
  }

  private mapStatus(wiaStatus: string): string {
    const statusMap: Record<string, string> = {
      'active': 'active',
      'pending': 'draft',
      'revoked': 'inactive',
      'expired': 'inactive',
      'superseded': 'inactive'
    };
    return statusMap[wiaStatus] || 'unknown';
  }

  private buildExtensions(scope: any): any[] {
    return [
      {
        url: 'https://wia.live/fhir/StructureDefinition/cryo-consent-scope',
        extension: [
          {
            url: 'preservation',
            valueBoolean: scope.preservation.authorized
          },
          {
            url: 'research',
            valueBoolean: scope.research.authorized
          },
          {
            url: 'revival',
            valueBoolean: scope.revival.authorized
          }
        ]
      }
    ];
  }
}
```

### 3.2 EHR 시스템 커넥터

| EHR System | Integration Method | Authentication | Standards |
|------------|-------------------|----------------|-----------|
| Epic | FHIR API | OAuth 2.0 | SMART on FHIR |
| Cerner | FHIR API | OAuth 2.0 | SMART on FHIR |
| Allscripts | REST API | API Key | Proprietary + FHIR |
| Meditech | HL7 v2 / FHIR | Certificate | HL7 v2.x, FHIR |
| athenahealth | REST API | OAuth 2.0 | FHIR R4 |

---

## Blockchain 통합

### 4.1 Blockchain Anchoring

#### 4.1.1 지원되는 Blockchain

| Blockchain | Use Case | Consensus | Finality |
|------------|----------|-----------|----------|
| Ethereum | 공개 anchoring | Proof of Stake | 12-15분 |
| Polygon | 비용 효율적 | Proof of Stake | ~2초 |
| Hyperledger Fabric | Private/consortium | PBFT | 즉시 |
| Hedera | 높은 처리량 | Hashgraph | 3-5초 |
| IPFS + Filecoin | 문서 저장소 | Proof of Replication | 가변적 |

#### 4.1.2 Ethereum Smart Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

contract CryoConsentRegistry {
    struct ConsentRecord {
        bytes32 consentHash;
        uint256 version;
        uint256 timestamp;
        address grantor;
        ConsentStatus status;
        bytes32 previousHash;
    }

    enum ConsentStatus {
        Active,
        Revoked,
        Superseded
    }

    mapping(bytes32 => ConsentRecord[]) public consentHistory;
    mapping(bytes32 => uint256) public latestVersion;

    event ConsentAnchored(
        bytes32 indexed consentId,
        bytes32 consentHash,
        uint256 version,
        address grantor
    );

    event ConsentRevoked(
        bytes32 indexed consentId,
        uint256 version,
        uint256 timestamp
    );

    function anchorConsent(
        bytes32 consentId,
        bytes32 consentHash,
        address grantor
    ) external returns (uint256) {
        uint256 version = latestVersion[consentId] + 1;
        bytes32 previousHash = version > 1
            ? consentHistory[consentId][version - 2].consentHash
            : bytes32(0);

        ConsentRecord memory record = ConsentRecord({
            consentHash: consentHash,
            version: version,
            timestamp: block.timestamp,
            grantor: grantor,
            status: ConsentStatus.Active,
            previousHash: previousHash
        });

        consentHistory[consentId].push(record);
        latestVersion[consentId] = version;

        // 이전 버전을 superseded로 표시
        if (version > 1) {
            consentHistory[consentId][version - 2].status = ConsentStatus.Superseded;
        }

        emit ConsentAnchored(consentId, consentHash, version, grantor);

        return version;
    }

    function revokeConsent(bytes32 consentId) external {
        uint256 version = latestVersion[consentId];
        require(version > 0, "동의가 존재하지 않습니다");

        ConsentRecord storage record = consentHistory[consentId][version - 1];
        require(record.status == ConsentStatus.Active, "동의가 활성화되지 않음");
        require(msg.sender == record.grantor, "grantor만 철회 가능");

        record.status = ConsentStatus.Revoked;

        emit ConsentRevoked(consentId, version, block.timestamp);
    }

    function verifyConsent(
        bytes32 consentId,
        uint256 version,
        bytes32 expectedHash
    ) external view returns (bool) {
        require(version > 0 && version <= latestVersion[consentId], "유효하지 않은 버전");

        ConsentRecord memory record = consentHistory[consentId][version - 1];
        return record.consentHash == expectedHash;
    }

    function getConsentStatus(
        bytes32 consentId
    ) external view returns (ConsentStatus, uint256) {
        uint256 version = latestVersion[consentId];
        require(version > 0, "동의가 존재하지 않습니다");

        ConsentRecord memory record = consentHistory[consentId][version - 1];
        return (record.status, version);
    }

    function getConsentHistory(
        bytes32 consentId
    ) external view returns (ConsentRecord[] memory) {
        return consentHistory[consentId];
    }
}
```

#### 4.1.3 Blockchain Adapter

```python
from web3 import Web3
from typing import Optional, Dict
import json

class EthereumConsentAdapter:
    def __init__(
        self,
        provider_url: str,
        contract_address: str,
        private_key: str
    ):
        self.w3 = Web3(Web3.HTTPProvider(provider_url))
        self.account = self.w3.eth.account.from_key(private_key)

        # Contract ABI 로드
        with open('CryoConsentRegistry.json', 'r') as f:
            contract_json = json.load(f)

        self.contract = self.w3.eth.contract(
            address=contract_address,
            abi=contract_json['abi']
        )

    def anchor_consent(
        self,
        consent_id: str,
        consent_hash: str,
        grantor_address: str
    ) -> Dict:
        """
        Ethereum blockchain에 동의 기록을 anchor합니다.

        Args:
            consent_id: 고유 동의 식별자
            consent_hash: 동의 문서의 SHA-256 hash
            grantor_address: grantor의 Ethereum 주소

        Returns:
            버전 번호가 포함된 transaction receipt
        """
        # bytes32로 변환
        consent_id_bytes = self.w3.keccak(text=consent_id)
        consent_hash_bytes = bytes.fromhex(consent_hash.replace('sha256:', ''))

        # Transaction 빌드
        transaction = self.contract.functions.anchorConsent(
            consent_id_bytes,
            consent_hash_bytes,
            Web3.toChecksumAddress(grantor_address)
        ).buildTransaction({
            'from': self.account.address,
            'nonce': self.w3.eth.get_transaction_count(self.account.address),
            'gas': 200000,
            'gasPrice': self.w3.eth.gas_price
        })

        # 서명 및 전송
        signed_txn = self.w3.eth.account.sign_transaction(
            transaction,
            self.account.key
        )
        tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)

        # 확인 대기
        receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)

        # Event log에서 버전 파싱
        version = self._parse_version_from_receipt(receipt)

        return {
            'transactionHash': tx_hash.hex(),
            'blockNumber': receipt['blockNumber'],
            'version': version,
            'status': 'confirmed' if receipt['status'] == 1 else 'failed',
            'gasUsed': receipt['gasUsed']
        }

    def verify_consent(
        self,
        consent_id: str,
        version: int,
        expected_hash: str
    ) -> bool:
        """
        Blockchain 기록에 대해 동의 무결성을 검증합니다.
        """
        consent_id_bytes = self.w3.keccak(text=consent_id)
        expected_hash_bytes = bytes.fromhex(expected_hash.replace('sha256:', ''))

        is_valid = self.contract.functions.verifyConsent(
            consent_id_bytes,
            version,
            expected_hash_bytes
        ).call()

        return is_valid

    def get_consent_status(self, consent_id: str) -> Dict:
        """
        Blockchain에서 현재 동의 상태를 가져옵니다.
        """
        consent_id_bytes = self.w3.keccak(text=consent_id)

        status, version = self.contract.functions.getConsentStatus(
            consent_id_bytes
        ).call()

        status_map = ['Active', 'Revoked', 'Superseded']

        return {
            'status': status_map[status],
            'version': version
        }

    def get_consent_history(self, consent_id: str) -> list:
        """
        Blockchain에서 완전한 동의 이력을 검색합니다.
        """
        consent_id_bytes = self.w3.keccak(text=consent_id)

        history = self.contract.functions.getConsentHistory(
            consent_id_bytes
        ).call()

        return [
            {
                'version': i + 1,
                'hash': record[0].hex(),
                'timestamp': record[2],
                'grantor': record[3],
                'status': ['Active', 'Revoked', 'Superseded'][record[4]],
                'previousHash': record[5].hex()
            }
            for i, record in enumerate(history)
        ]

    def _parse_version_from_receipt(self, receipt) -> int:
        """Transaction receipt에서 버전 번호를 파싱합니다."""
        event_signature_hash = self.w3.keccak(
            text='ConsentAnchored(bytes32,bytes32,uint256,address)'
        )

        for log in receipt['logs']:
            if log['topics'][0] == event_signature_hash:
                # 버전은 3번째 인덱스된 매개변수 (uint256)
                version = int(log['data'].hex()[:66], 16)
                return version

        return 0
```

---

## 신원 확인 통합

### 5.1 신원 확인 제공자

| Provider | Methods | Coverage | Compliance |
|----------|---------|----------|------------|
| Onfido | ID + Biometric | Global | KYC, AML |
| Jumio | ID + Liveness | 200+ 국가 | KYC, AML, GDPR |
| Veriff | ID + Video | Global | KYC, GDPR |
| Persona | ID + Biometric | US, EU, APAC | KYC, GDPR |
| Stripe Identity | ID + Selfie | 33개국 | KYC, PSD2 |

### 5.2 검증 흐름 통합

```typescript
interface VerificationRequest {
  userId: string;
  verificationType: 'identity' | 'liveness' | 'document' | 'biometric';
  requiredDocuments: string[];
  jurisdiction: string;
  returnUrl: string;
}

interface VerificationResult {
  verificationId: string;
  status: 'pending' | 'approved' | 'rejected' | 'requires_review';
  userId: string;
  completedAt?: string;
  identity?: {
    fullName: string;
    dateOfBirth: string;
    nationality: string;
    documentNumber: string;
    documentType: string;
  };
  checks: {
    identityVerification: boolean;
    livenessCheck: boolean;
    documentAuthenticity: boolean;
    facialComparison: boolean;
  };
  riskScore?: number;
  rejectionReasons?: string[];
}
```

---

## 데이터 교환 표준

### 6.1 내보내기 형식

| Format | Use Case | Standard |
|--------|----------|----------|
| JSON | API 통합 | WIA Schema |
| XML | 법적 시스템 | LegalXML |
| PDF/A | 보관 | ISO 19005 |
| HL7 FHIR | 의료 | FHIR R4 |
| CSV | 보고 | RFC 4180 |

### 6.2 데이터 교환 프로토콜

| Protocol | Security | Use Case |
|----------|----------|----------|
| HTTPS REST | TLS 1.3 | 동기 API |
| SFTP | SSH | 일괄 전송 |
| AS2 | S/MIME | EDI 교환 |
| FHIR Messaging | TLS + OAuth | 의료 |
| Blockchain | Cryptographic | 감사 추적 |

---

## 구현 예제

### 7.1 완전한 통합 예제

```typescript
// 포괄적인 통합 orchestrator
class ConsentIntegrationOrchestrator {
  constructor(
    private fhirAdapter: FHIRConsentAdapter,
    private blockchainAdapter: EthereumConsentAdapter,
    private notaryAdapter: DocuSignNotaryAdapter,
    private storageAdapter: BoxLegalHoldAdapter
  ) {}

  async createConsentWithIntegrations(
    consentData: any
  ): Promise<any> {
    const results: any = {};

    try {
      // 1. 기본 시스템에서 동의 생성
      const consent = await this.createPrimaryConsent(consentData);
      results.consentId = consent.consentId;

      // 2. 법적 문서 저장소에 저장
      results.storage = await this.storageAdapter.store_document(
        consent.consentId,
        'consent_record',
        this.generatePDF(consent),
        {
          consent_id: consent.consentId,
          grantor_id: consent.grantor.id,
          jurisdiction: consent.legal.jurisdiction,
          hash: consent.meta.hash
        }
      );

      // 3. Blockchain에 anchor
      results.blockchain = await this.blockchainAdapter.anchor_consent(
        consent.consentId,
        consent.meta.hash,
        consent.grantor.ethereumAddress
      );

      // 4. FHIR를 통해 EHR과 동기화
      results.fhir = await this.fhirAdapter.createConsent(consent);

      // 5. 필요한 경우 공증 시작
      if (this.requiresNotarization(consent)) {
        results.notarization = await this.notaryAdapter.createNotarizationEnvelope({
          documentId: consent.consentId,
          documentType: 'consent_record',
          documentHash: consent.meta.hash,
          signers: this.extractSigners(consent),
          jurisdiction: consent.legal.jurisdiction
        });
      }

      return {
        success: true,
        consentId: consent.consentId,
        integrations: results
      };

    } catch (error) {
      console.error('통합 오류:', error);

      // 실패 시 롤백
      await this.rollback(results);

      throw error;
    }
  }

  private async rollback(results: any): Promise<void> {
    // 보상 트랜잭션 구현
    console.log('통합 롤백 중:', results);
  }

  private generatePDF(consent: any): Buffer {
    // PDF 문서 생성
    return Buffer.from('');
  }

  private requiresNotarization(consent: any): boolean {
    // 관할권 요구사항 확인
    const notaryJurisdictions = ['US-CA', 'US-NY'];
    return notaryJurisdictions.includes(consent.legal.jurisdiction);
  }

  private extractSigners(consent: any): any[] {
    return [
      {
        id: consent.grantor.id,
        name: 'Grantor Name',
        email: 'grantor@example.com',
        role: 'grantor'
      }
    ];
  }

  private async createPrimaryConsent(consentData: any): Promise<any> {
    // 기본 동의 관리 시스템에서 생성
    return consentData;
  }
}
```

---

## 버전 이력

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | 초기 릴리스 |

---

<div align="center">

**WIA Cryo-Consent Integration Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
