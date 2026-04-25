# WIA-PET-003: Pet Genome 프로토콜 명세서
## Phase 3: 통신 및 보안 프로토콜

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. 소개

### 1.1 목적
본 명세서는 반려동물 유전체 데이터 교환을 위한 통신 프로토콜, 보안 표준 및 운영 절차를 정의합니다. 연구소, 수의 클리닉, 번식 조직 및 연구 기관 간의 안전하고 신뢰할 수 있으며 표준화된 민감한 유전 정보 전송을 보장합니다.

### 1.2 적용 범위
본 표준은 다음을 다룹니다:
- 전송 계층 보안 프로토콜 (Transport Layer Security)
- 데이터 암호화 표준 (Data Encryption Standards)
- 인증 및 권한 부여 프로토콜 (Authentication and Authorization)
- 동의 관리 프로토콜 (Consent Management)
- 데이터 보존 및 삭제 프로토콜 (Data Retention and Deletion)
- 감사 로깅 및 규정 준수 (Audit Logging and Compliance)
- 연구소 간 데이터 교환 (Inter-laboratory Data Exchange)
- 긴급 접근 프로토콜 (Emergency Access)
- 백업 및 재해 복구 (Backup and Disaster Recovery)
- 국경 간 데이터 전송 규정 준수 (Cross-border Data Transfer)

### 1.3 프로토콜 스택 개요

```
┌─────────────────────────────────────────┐
│     애플리케이션 계층 (HTTPS/WSS)        │
├─────────────────────────────────────────┤
│     보안 계층 (TLS 1.3)                  │
├─────────────────────────────────────────┤
│     인증 (OAuth 2.0/JWT)                 │
├─────────────────────────────────────────┤
│     데이터 형식 (JSON/VCF/BAM)           │
├─────────────────────────────────────────┤
│     암호화 (AES-256-GCM)                 │
├─────────────────────────────────────────┤
│     전송 (TCP/IP)                        │
└─────────────────────────────────────────┘
```

---

## 2. 전송 보안

### 2.1 TLS 구성

#### 최소 TLS 요구사항

| 파라미터 | 요구사항 | 참고사항 |
|-----------|-------------|-------|
| TLS 버전 | TLS 1.3 | TLS 1.2는 승인된 암호와 함께 허용 가능 |
| 인증서 유형 | X.509 v3 | 신뢰할 수 있는 CA에서 발급 필수 |
| 인증서 유효기간 | ≤ 398일 | CA/Browser Forum 요구사항 준수 |
| 키 크기 | RSA 2048비트 또는 ECC 256비트 | 성능을 위해 ECC 선호 |
| Certificate Transparency | 필수 | CT 로그에 기록되어야 함 |

#### 승인된 암호 스위트 (TLS 1.3)

```
TLS_AES_256_GCM_SHA384
TLS_CHACHA20_POLY1305_SHA256
TLS_AES_128_GCM_SHA256
```

#### 승인된 암호 스위트 (TLS 1.2 - 레거시 지원)

```
TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256
```

### 2.2 인증서 고정 (Certificate Pinning)

인증서 고정은 중간자 공격(MITM)을 방지하기 위해 특정 인증서 또는 공개 키만 신뢰하도록 구성하는 보안 메커니즘입니다.

```python
import ssl
import certifi
from urllib3.util import ssl_

# 인증서 고정 구성
PINNED_CERTIFICATES = {
    'api.pet-genome.wia.org': [
        # 허용된 인증서의 SHA-256 지문
        '5d41402abc4b2a76b9719d911017c592a9f8a76b9719d911017c5925d41402ab',
        '7f8a9b5c3d1e2a4f6b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a'
    ]
}

def create_secure_context():
    """인증서 고정을 사용한 SSL 컨텍스트 생성"""
    context = ssl.create_default_context(cafile=certifi.where())
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.maximum_version = ssl.TLSVersion.TLSv1_3

    # 인증서 검증 활성화
    context.check_hostname = True
    context.verify_mode = ssl.CERT_REQUIRED

    return context
```

### 2.3 Perfect Forward Secrecy (PFS)

모든 연결은 Perfect Forward Secrecy를 제공하는 암호 스위트를 사용해야 합니다:
- Ephemeral Diffie-Hellman (DHE)
- Ephemeral Elliptic Curve Diffie-Hellman (ECDHE)

PFS는 장기 비밀 키가 손상되더라도 과거 세션 키가 복호화될 수 없도록 보장합니다.

---

## 3. 데이터 암호화

### 3.1 정지 상태 암호화 (Encryption at Rest)

#### AES-256-GCM 구성

AES-256-GCM은 대칭 키 암호화의 산업 표준으로, 기밀성과 무결성을 모두 제공합니다.

```json
{
  "encryptionStandard": {
    "algorithm": "AES-256-GCM",
    "keySize": 256,
    "blockSize": 128,
    "mode": "GCM",
    "ivLength": 96,
    "tagLength": 128,
    "keyDerivation": "PBKDF2-SHA256",
    "iterations": 100000
  },
  "keyManagement": {
    "system": "AWS-KMS",
    "keyRotationPeriod": "P90D",
    "masterKeyBackup": true,
    "hardwareSecurityModule": true
  }
}
```

#### 암호화 구현 예제

```python
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import os
import json

class GenomicDataEncryption:
    """AES-256-GCM을 사용한 유전체 데이터 암호화/복호화"""

    def __init__(self, master_key: bytes):
        self.master_key = master_key
        self.backend = default_backend()

    def encrypt_profile(self, profile_data: dict) -> dict:
        """
        유전체 프로파일 데이터 암호화

        Args:
            profile_data: 유전체 프로파일 딕셔너리

        Returns:
            메타데이터가 포함된 암호화된 데이터 패키지
        """
        # 데이터 직렬화
        plaintext = json.dumps(profile_data).encode('utf-8')

        # 랜덤 IV 생성 (GCM의 경우 96비트)
        iv = os.urandom(12)

        # 암호 생성
        cipher = Cipher(
            algorithms.AES(self.master_key),
            modes.GCM(iv),
            backend=self.backend
        )

        # 암호화
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()

        # 암호화된 패키지 반환
        return {
            'version': '1.0',
            'algorithm': 'AES-256-GCM',
            'iv': iv.hex(),
            'ciphertext': ciphertext.hex(),
            'tag': encryptor.tag.hex(),
            'keyId': self._get_key_id(),
            'timestamp': self._get_timestamp()
        }

    def decrypt_profile(self, encrypted_package: dict) -> dict:
        """
        유전체 프로파일 데이터 복호화

        Args:
            encrypted_package: 암호화된 데이터 패키지

        Returns:
            복호화된 유전체 프로파일
        """
        # 구성 요소 추출
        iv = bytes.fromhex(encrypted_package['iv'])
        ciphertext = bytes.fromhex(encrypted_package['ciphertext'])
        tag = bytes.fromhex(encrypted_package['tag'])

        # 암호 생성
        cipher = Cipher(
            algorithms.AES(self.master_key),
            modes.GCM(iv, tag),
            backend=self.backend
        )

        # 복호화
        decryptor = cipher.decryptor()
        plaintext = decryptor.update(ciphertext) + decryptor.finalize()

        # 역직렬화
        return json.loads(plaintext.decode('utf-8'))

    def _get_key_id(self) -> str:
        """현재 암호화 키 식별자 가져오기"""
        import hashlib
        return hashlib.sha256(self.master_key).hexdigest()[:16]

    def _get_timestamp(self) -> str:
        """현재 ISO 타임스탬프 가져오기"""
        from datetime import datetime
        return datetime.utcnow().isoformat() + 'Z'

# 사용 예제
master_key = os.urandom(32)  # 256비트 키
encryptor = GenomicDataEncryption(master_key)

# 프로파일 암호화
profile = {'profileId': 'PGP-ABCD12345678', 'species': 'CANIS_FAMILIARIS'}
encrypted = encryptor.encrypt_profile(profile)

# 프로파일 복호화
decrypted = encryptor.decrypt_profile(encrypted)
```

### 3.2 전송 중 암호화 (Encryption in Transit)

#### HTTPS Strict Transport Security (HSTS)

HSTS는 브라우저가 항상 HTTPS를 통해 사이트에 연결하도록 강제합니다.

```http
Strict-Transport-Security: max-age=63072000; includeSubDomains; preload
```

**HSTS 파라미터 설명:**
- `max-age=63072000`: 2년(초 단위)
- `includeSubDomains`: 모든 하위 도메인에도 적용
- `preload`: 브라우저의 사전 로드 목록에 포함

#### Content Security Policy (CSP)

CSP는 XSS 공격을 방지하기 위해 허용되는 콘텐츠 소스를 정의합니다.

```http
Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; connect-src 'self' https://api.pet-genome.wia.org
```

### 3.3 필드 수준 암호화 (Field-Level Encryption)

민감한 필드는 개별적으로 암호화되어야 합니다:

```json
{
  "profileId": "PGP-ABCD12345678",
  "petIdentification": {
    "registeredName": "Champion Golden Star",
    "microchipId": {
      "_encrypted": true,
      "algorithm": "AES-256-GCM",
      "ciphertext": "a1b2c3d4e5f6...",
      "iv": "1a2b3c4d5e6f7a8b9c0d1e2f",
      "keyId": "KEY-2025-12-001"
    }
  },
  "ownerInformation": {
    "_encrypted": true,
    "algorithm": "AES-256-GCM",
    "ciphertext": "x9y8z7w6v5u4...",
    "iv": "9x8y7z6w5v4u3t2s1r0q",
    "keyId": "KEY-2025-12-001"
  }
}
```

---

## 4. 인증 프로토콜

### 4.1 OAuth 2.0 구현

OAuth 2.0은 안전한 위임 접근을 위한 산업 표준 프로토콜입니다.

#### Authorization Code Flow

```
┌─────────┐                                   ┌─────────────┐
│         │                                   │             │
│  Client │                                   │   Auth      │
│  App    │                                   │   Server    │
│         │                                   │             │
└────┬────┘                                   └──────┬──────┘
     │                                               │
     │ 1. 인가 요청                                  │
     │──────────────────────────────────────────────>│
     │                                               │
     │ 2. 인가 코드                                  │
     │<──────────────────────────────────────────────│
     │                                               │
     │ 3. 토큰 교환                                  │
     │──────────────────────────────────────────────>│
     │                                               │
     │ 4. Access Token + Refresh Token               │
     │<──────────────────────────────────────────────│
     │                                               │
     │ 5. Access Token으로 API 요청                  │
     │──────────────────────────────────────────────>│
     │                                               │
```

#### 토큰 요청 예제

```http
POST /oauth/token HTTP/1.1
Host: auth.pet-genome.wia.org
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE_HERE
&redirect_uri=https://your-app.com/callback
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&code_verifier=CODE_VERIFIER_FOR_PKCE
```

**파라미터 설명:**
- `grant_type`: 인가 유형 (authorization_code)
- `code`: 인가 서버에서 받은 인가 코드
- `redirect_uri`: 등록된 리디렉션 URI
- `client_id`: 클라이언트 식별자
- `client_secret`: 클라이언트 비밀키
- `code_verifier`: PKCE 검증자

### 4.2 JWT 토큰 구조

JWT (JSON Web Token)는 당사자 간에 정보를 안전하게 전송하기 위한 컴팩트한 자체 포함 방식입니다.

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "key-2025-12-001"
  },
  "payload": {
    "iss": "https://auth.pet-genome.wia.org",
    "sub": "user-12345",
    "aud": "https://api.pet-genome.wia.org",
    "exp": 1703001600,
    "iat": 1702998000,
    "nbf": 1702998000,
    "jti": "jwt-unique-id-12345",
    "scope": "genomic:read genomic:write breed:analyze",
    "client_id": "client-app-12345",
    "organization": "ORG-VETLAB-001",
    "roles": ["veterinarian", "genomic_analyst"]
  },
  "signature": "..."
}
```

**페이로드 필드 설명:**
- `iss` (Issuer): 토큰 발급자
- `sub` (Subject): 토큰 주체 (사용자)
- `aud` (Audience): 토큰 수신자
- `exp` (Expiration Time): 만료 시간
- `iat` (Issued At): 발급 시간
- `nbf` (Not Before): 사용 시작 시간
- `jti` (JWT ID): 토큰 고유 식별자

### 4.3 다단계 인증 (Multi-Factor Authentication)

MFA는 사용자 인증에 두 가지 이상의 검증 요소를 요구하여 보안을 강화합니다.

```python
import pyotp
import qrcode

class MFAManager:
    """유전체 데이터 접근을 위한 다단계 인증 관리"""

    def generate_secret(self, user_id: str) -> dict:
        """사용자를 위한 TOTP 비밀 생성"""
        secret = pyotp.random_base32()

        totp_uri = pyotp.totp.TOTP(secret).provisioning_uri(
            name=user_id,
            issuer_name='Pet Genome WIA'
        )

        # QR 코드 생성
        qr = qrcode.QRCode(version=1, box_size=10, border=5)
        qr.add_data(totp_uri)
        qr.make(fit=True)

        return {
            'secret': secret,
            'totpUri': totp_uri,
            'qrCode': qr.make_image()
        }

    def verify_totp(self, secret: str, token: str) -> bool:
        """TOTP 토큰 검증"""
        totp = pyotp.TOTP(secret)
        return totp.verify(token, valid_window=1)

    def generate_backup_codes(self, count: int = 10) -> list:
        """계정 복구를 위한 백업 코드 생성"""
        import secrets
        return [secrets.token_hex(8) for _ in range(count)]

# 사용 예제
mfa = MFAManager()
user_mfa = mfa.generate_secret('user@example.com')

# 토큰 검증
is_valid = mfa.verify_totp(user_mfa['secret'], '123456')
```

---

## 5. 동의 관리 프로토콜

### 5.1 동의 수집 워크플로

동의 수집은 유전체 데이터 처리의 법적 기반이며, GDPR 및 기타 개인정보 보호 규정의 핵심 요구사항입니다.

```
┌──────────┐     ┌────────────┐     ┌─────────────┐     ┌──────────┐
│          │     │            │     │             │     │          │
│   Pet    │────>│   동의     │────>│   전자      │────>│  유전체  │
│  소유자  │     │   양식     │     │  서명       │     │  검사    │
│          │     │            │     │             │     │          │
└──────────┘     └────────────┘     └─────────────┘     └──────────┘
```

### 5.2 동의 기록 Schema

```json
{
  "consentId": "CONSENT-2025-12-18-001234",
  "version": "2.0",
  "profileId": "PGP-ABCD12345678",
  "consentTimestamp": "2025-12-18T14:30:00Z",
  "expirationDate": "2030-12-18T14:30:00Z",
  "ownerIdentity": {
    "identityVerified": true,
    "verificationMethod": "GOVERNMENT_ID",
    "verificationTimestamp": "2025-12-18T14:25:00Z",
    "verifiedBy": "VET-CLINIC-001"
  },
  "consentCategories": {
    "geneticTesting": {
      "granted": true,
      "scope": ["DISEASE_SCREENING", "BREED_IDENTIFICATION", "TRAIT_ANALYSIS"],
      "timestamp": "2025-12-18T14:30:00Z"
    },
    "dataStorage": {
      "granted": true,
      "retentionPeriod": "P10Y",
      "deletionRights": true,
      "timestamp": "2025-12-18T14:30:00Z"
    },
    "researchParticipation": {
      "granted": false,
      "anonymization": "REQUIRED",
      "commercialUse": false,
      "timestamp": "2025-12-18T14:30:00Z"
    },
    "dataSharing": {
      "veterinarians": {
        "granted": true,
        "scope": "TREATING_VETERINARIAN_ONLY"
      },
      "breeders": {
        "granted": true,
        "scope": "HEALTH_SCREENING_RESULTS_ONLY"
      },
      "insurance": {
        "granted": false
      },
      "researchers": {
        "granted": false
      }
    }
  },
  "digitalSignature": {
    "algorithm": "RSA-SHA256",
    "signature": "BASE64_ENCODED_SIGNATURE",
    "publicKey": "BASE64_ENCODED_PUBLIC_KEY",
    "signatureTimestamp": "2025-12-18T14:30:00Z"
  },
  "legalJurisdiction": "EU-GDPR",
  "consentLanguage": "ko-KR",
  "withdrawalRights": {
    "withdrawalAllowed": true,
    "withdrawalMethod": "ONLINE_PORTAL_OR_WRITTEN_REQUEST",
    "withdrawalEffectivePeriod": "P30D"
  },
  "auditLog": [
    {
      "action": "CONSENT_CREATED",
      "timestamp": "2025-12-18T14:30:00Z",
      "actor": "OWNER-001"
    }
  ]
}
```

---

## 6. 데이터 보존 및 삭제 프로토콜

### 6.1 보존 정책 매트릭스

데이터 보존 정책은 법적 요구사항과 비즈니스 요구사항의 균형을 맞춥니다.

| 데이터 카테고리 | 보존 기간 | 삭제 방법 | 백업 보존 |
|--------------|------------------|-----------------|------------------|
| 원시 시퀀싱 데이터 | 10년 | 안전한 삭제 (NIST SP 800-88) | 5년 |
| 처리된 변이 | 15년 | 안전한 삭제 | 10년 |
| 건강 마커 | 평생 + 5년 | 안전한 삭제 | 평생 + 3년 |
| 품종 정보 | 무기한 (동의 하에) | 요청 시 안전한 삭제 | N/A |
| 소유자 개인식별정보 | 동의 기간 + 1년 | 안전한 삭제 | 없음 |
| 감사 로그 | 7년 | 안전한 삭제 | 7년 |

---

## 7. 감사 로깅 프로토콜

### 7.1 감사 이벤트 Schema

감사 로그는 모든 데이터 접근 및 수정 활동의 추적 가능성을 보장합니다.

```json
{
  "eventId": "AUDIT-2025-12-18-001234",
  "timestamp": "2025-12-18T14:30:00.123Z",
  "eventType": "DATA_ACCESS",
  "severity": "INFO",
  "actor": {
    "userId": "USER-12345",
    "userType": "VETERINARIAN",
    "organization": "VET-CLINIC-001",
    "ipAddress": "192.168.1.100",
    "userAgent": "PetGenomeSDK/1.2.3",
    "sessionId": "SESSION-abcd1234"
  },
  "resource": {
    "resourceType": "GENOMIC_PROFILE",
    "resourceId": "PGP-ABCD12345678",
    "ownerConsent": "CONSENT-2025-12-18-001"
  },
  "action": {
    "actionType": "READ",
    "actionScope": "HEALTH_MARKERS",
    "authorized": true,
    "authorizationMethod": "OAUTH2_TOKEN"
  },
  "outcome": {
    "status": "SUCCESS",
    "statusCode": 200,
    "dataReturned": true,
    "recordsAccessed": 15
  },
  "context": {
    "requestId": "req_1234567890abcdef",
    "clinicalPurpose": "GENETIC_COUNSELING",
    "emergencyAccess": false
  },
  "compliance": {
    "gdprLawfulBasis": "CONSENT",
    "hipaaCompliant": true,
    "dataClassification": "HIGHLY_SENSITIVE"
  }
}
```

---

## 8. 긴급 접근 프로토콜

### 8.1 Break-Glass 접근 절차

Break-Glass 프로토콜은 긴급 의료 상황에서 생명을 구하기 위한 제한된 데이터 접근을 허용합니다.

```python
from datetime import datetime, timedelta
from enum import Enum

class EmergencyAccessLevel(Enum):
    CRITICAL = "CRITICAL"  # 생명을 위협하는 상황
    URGENT = "URGENT"      # 시간에 민감한 의료 필요
    ELEVATED = "ELEVATED"  # 비일상적 접근 필요

class EmergencyAccessManager:
    """유전체 데이터에 대한 긴급 접근 관리"""

    def __init__(self, audit_logger):
        self.audit_logger = audit_logger
        self.active_sessions = {}

    def request_emergency_access(self, request_data: dict) -> dict:
        """
        유전체 프로파일에 대한 긴급 접근 요청

        Args:
            request_data: 긴급 접근 요청 세부사항

        Returns:
            긴급 접근 승인 또는 거부
        """
        # 요청 검증
        if not self._validate_emergency_request(request_data):
            return {
                'granted': False,
                'reason': 'INVALID_REQUEST'
            }

        # 접근 수준 결정
        access_level = EmergencyAccessLevel[request_data['emergencyLevel']]

        # 긴급 세션 생성
        session_id = self._generate_emergency_session_id()

        emergency_session = {
            'sessionId': session_id,
            'profileId': request_data['profileId'],
            'requestedBy': request_data['veterinarian'],
            'emergencyLevel': access_level.value,
            'justification': request_data['justification'],
            'grantedAt': datetime.utcnow().isoformat() + 'Z',
            'expiresAt': (datetime.utcnow() + timedelta(hours=24)).isoformat() + 'Z',
            'accessToken': self._generate_emergency_token(),
            'restrictions': self._determine_restrictions(access_level)
        }

        # 긴급 접근 로깅
        self.audit_logger.log_security_event(
            event_type='EMERGENCY_ACCESS_GRANTED',
            severity='CRITICAL',
            details={
                'sessionId': session_id,
                'profileId': request_data['profileId'],
                'requestedBy': request_data['veterinarian'],
                'emergencyLevel': access_level.value,
                'justification': request_data['justification']
            }
        )

        # 세션 저장
        self.active_sessions[session_id] = emergency_session

        # 데이터 보호 담당자에게 알림
        self._notify_dpo(emergency_session)

        return {
            'granted': True,
            'sessionId': session_id,
            'accessToken': emergency_session['accessToken'],
            'expiresAt': emergency_session['expiresAt'],
            'restrictions': emergency_session['restrictions']
        }

    def _validate_emergency_request(self, request_data: dict) -> bool:
        """긴급 접근 요청 검증"""
        required_fields = ['profileId', 'veterinarian', 'emergencyLevel', 'justification']
        return all(field in request_data for field in required_fields)

    def _generate_emergency_session_id(self) -> str:
        """긴급 세션 ID 생성"""
        import uuid
        return f"EMERG-{uuid.uuid4().hex[:12].upper()}"

    def _generate_emergency_token(self) -> str:
        """긴급 접근 토큰 생성"""
        import secrets
        return f"emerg_{secrets.token_urlsafe(32)}"

    def _determine_restrictions(self, access_level: EmergencyAccessLevel) -> dict:
        """긴급 수준에 따른 접근 제한 결정"""
        if access_level == EmergencyAccessLevel.CRITICAL:
            return {
                'dataAccess': 'FULL',
                'exportAllowed': True,
                'modificationAllowed': False
            }
        elif access_level == EmergencyAccessLevel.URGENT:
            return {
                'dataAccess': 'HEALTH_MARKERS_ONLY',
                'exportAllowed': False,
                'modificationAllowed': False
            }
        else:
            return {
                'dataAccess': 'LIMITED',
                'exportAllowed': False,
                'modificationAllowed': False
            }

    def _notify_dpo(self, session: dict):
        """데이터 보호 담당자에게 긴급 접근 알림"""
        print(f"DPO 알림: 긴급 접근 {session['sessionId']}")
```

---

## 9. 국경 간 데이터 전송

### 9.1 데이터 전송 평가

국경 간 데이터 전송은 각 관할권의 개인정보 보호법을 준수해야 합니다.

| 출발 지역 | 도착 지역 | 전송 메커니즘 | 추가 안전 조치 |
|--------------|-------------------|-------------------|----------------------|
| EU | US | Standard Contractual Clauses | 암호화, 접근 제어 |
| US | EU | EU-US Data Privacy Framework | GDPR 준수 |
| EU | EU | GDPR Adequacy | 표준 처리 |
| US | UK | UK Adequacy Decision | 표준 처리 |
| 한국 | EU | 개인정보 보호 적정성 결정 대기 | SCC + 암호화 |

### 9.2 전송 영향 평가 템플릿

```json
{
  "transferAssessmentId": "TIA-2025-12-001",
  "assessmentDate": "2025-12-18",
  "dataExporter": {
    "organization": "한국 Pet Genomics Lab",
    "country": "대한민국",
    "legalBasis": "개인정보보호법 Article 17 - 동의"
  },
  "dataImporter": {
    "organization": "US Research Institute",
    "country": "United States",
    "adequacyDecision": false
  },
  "dataCategories": [
    "유전체 변이",
    "품종 정보",
    "건강 마커"
  ],
  "transferVolume": {
    "profiles": 1000,
    "estimatedDataSize": "500 GB"
  },
  "safeguards": [
    "Standard Contractual Clauses (2021)",
    "AES-256 전송 중 및 정지 상태 암호화",
    "접근 제어 및 인증",
    "데이터 최소화",
    "목적 제한",
    "감사 로깅"
  ],
  "riskAssessment": {
    "surveillanceRisk": "MEDIUM",
    "accessByAuthorities": "LOW",
    "dataProtectionLaws": "ADEQUATE",
    "overallRisk": "LOW"
  },
  "approvalStatus": "APPROVED",
  "approvedBy": "데이터 보호 담당자",
  "reviewDate": "2026-12-18"
}
```

---

## 10. 버전 이력

| Version | Date | 변경사항 | 작성자 |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | 초기 프로토콜 명세서 | WIA 표준 위원회 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
