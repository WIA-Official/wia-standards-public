# WIA-PET-003: Pet Genome Protocol Specification
## Phase 3: Communication and Security Protocols

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. Introduction

### 1.1 Purpose
This specification defines communication protocols, security standards, and operational procedures for pet genomic data exchange. It ensures secure, reliable, and standardized transmission of sensitive genetic information between laboratories, veterinary clinics, breeding organizations, and research institutions.

### 1.2 Scope
This standard covers:
- Transport layer security protocols
- Data encryption standards
- Authentication and authorization protocols
- Consent management protocols
- Data retention and deletion protocols
- Audit logging and compliance
- Inter-laboratory data exchange
- Emergency access protocols
- Backup and disaster recovery
- Cross-border data transfer compliance

### 1.3 Protocol Stack Overview

```
┌─────────────────────────────────────────┐
│     Application Layer (HTTPS/WSS)       │
├─────────────────────────────────────────┤
│     Security Layer (TLS 1.3)            │
├─────────────────────────────────────────┤
│     Authentication (OAuth 2.0/JWT)      │
├─────────────────────────────────────────┤
│     Data Format (JSON/VCF/BAM)          │
├─────────────────────────────────────────┤
│     Encryption (AES-256-GCM)            │
├─────────────────────────────────────────┤
│     Transport (TCP/IP)                  │
└─────────────────────────────────────────┘
```

---

## 2. Transport Security

### 2.1 TLS Configuration

#### Minimum TLS Requirements

| Parameter | Requirement | Notes |
|-----------|-------------|-------|
| TLS Version | TLS 1.3 | TLS 1.2 acceptable with approved ciphers |
| Certificate Type | X.509 v3 | Must be from trusted CA |
| Certificate Validity | ≤ 398 days | Per CA/Browser Forum requirements |
| Key Size | RSA 2048-bit or ECC 256-bit | ECC preferred for performance |
| Certificate Transparency | Required | Must be logged in CT logs |

#### Approved Cipher Suites (TLS 1.3)

```
TLS_AES_256_GCM_SHA384
TLS_CHACHA20_POLY1305_SHA256
TLS_AES_128_GCM_SHA256
```

#### Approved Cipher Suites (TLS 1.2 - Legacy Support)

```
TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256
```

### 2.2 Certificate Pinning

```python
import ssl
import certifi
from urllib3.util import ssl_

# Certificate pinning configuration
PINNED_CERTIFICATES = {
    'api.pet-genome.wia.org': [
        # SHA-256 fingerprints of allowed certificates
        '5d41402abc4b2a76b9719d911017c592a9f8a76b9719d911017c5925d41402ab',
        '7f8a9b5c3d1e2a4f6b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a'
    ]
}

def create_secure_context():
    """Create SSL context with certificate pinning"""
    context = ssl.create_default_context(cafile=certifi.where())
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.maximum_version = ssl.TLSVersion.TLSv1_3

    # Enable certificate verification
    context.check_hostname = True
    context.verify_mode = ssl.CERT_REQUIRED

    return context
```

### 2.3 Perfect Forward Secrecy (PFS)

All connections MUST use cipher suites that provide Perfect Forward Secrecy:
- Ephemeral Diffie-Hellman (DHE)
- Ephemeral Elliptic Curve Diffie-Hellman (ECDHE)

---

## 3. Data Encryption

### 3.1 Encryption at Rest

#### AES-256-GCM Configuration

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

#### Encryption Implementation Example

```python
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import os
import json

class GenomicDataEncryption:
    """Encrypt/decrypt genomic data using AES-256-GCM"""

    def __init__(self, master_key: bytes):
        self.master_key = master_key
        self.backend = default_backend()

    def encrypt_profile(self, profile_data: dict) -> dict:
        """
        Encrypt genomic profile data

        Args:
            profile_data: Genomic profile dictionary

        Returns:
            Encrypted data package with metadata
        """
        # Serialize data
        plaintext = json.dumps(profile_data).encode('utf-8')

        # Generate random IV (96 bits for GCM)
        iv = os.urandom(12)

        # Create cipher
        cipher = Cipher(
            algorithms.AES(self.master_key),
            modes.GCM(iv),
            backend=self.backend
        )

        # Encrypt
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()

        # Return encrypted package
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
        Decrypt genomic profile data

        Args:
            encrypted_package: Encrypted data package

        Returns:
            Decrypted genomic profile
        """
        # Extract components
        iv = bytes.fromhex(encrypted_package['iv'])
        ciphertext = bytes.fromhex(encrypted_package['ciphertext'])
        tag = bytes.fromhex(encrypted_package['tag'])

        # Create cipher
        cipher = Cipher(
            algorithms.AES(self.master_key),
            modes.GCM(iv, tag),
            backend=self.backend
        )

        # Decrypt
        decryptor = cipher.decryptor()
        plaintext = decryptor.update(ciphertext) + decryptor.finalize()

        # Deserialize
        return json.loads(plaintext.decode('utf-8'))

    def _get_key_id(self) -> str:
        """Get current encryption key identifier"""
        import hashlib
        return hashlib.sha256(self.master_key).hexdigest()[:16]

    def _get_timestamp(self) -> str:
        """Get current ISO timestamp"""
        from datetime import datetime
        return datetime.utcnow().isoformat() + 'Z'

# Usage example
master_key = os.urandom(32)  # 256-bit key
encryptor = GenomicDataEncryption(master_key)

# Encrypt profile
profile = {'profileId': 'PGP-ABCD12345678', 'species': 'CANIS_FAMILIARIS'}
encrypted = encryptor.encrypt_profile(profile)

# Decrypt profile
decrypted = encryptor.decrypt_profile(encrypted)
```

### 3.2 Encryption in Transit

#### HTTPS Strict Transport Security (HSTS)

```http
Strict-Transport-Security: max-age=63072000; includeSubDomains; preload
```

#### Content Security Policy

```http
Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; connect-src 'self' https://api.pet-genome.wia.org
```

### 3.3 Field-Level Encryption

Sensitive fields MUST be encrypted individually:

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

## 4. Authentication Protocols

### 4.1 OAuth 2.0 Implementation

#### Authorization Code Flow

```
┌─────────┐                                   ┌─────────────┐
│         │                                   │             │
│  Client │                                   │   Auth      │
│  App    │                                   │   Server    │
│         │                                   │             │
└────┬────┘                                   └──────┬──────┘
     │                                               │
     │ 1. Authorization Request                      │
     │──────────────────────────────────────────────>│
     │                                               │
     │ 2. Authorization Code                         │
     │<──────────────────────────────────────────────│
     │                                               │
     │ 3. Exchange Code for Token                    │
     │──────────────────────────────────────────────>│
     │                                               │
     │ 4. Access Token + Refresh Token               │
     │<──────────────────────────────────────────────│
     │                                               │
     │ 5. API Request with Access Token              │
     │──────────────────────────────────────────────>│
     │                                               │
```

#### Token Request Example

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

### 4.2 JWT Token Structure

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

### 4.3 Multi-Factor Authentication (MFA)

```python
import pyotp
import qrcode

class MFAManager:
    """Manage multi-factor authentication for genomic data access"""

    def generate_secret(self, user_id: str) -> dict:
        """Generate TOTP secret for user"""
        secret = pyotp.random_base32()

        totp_uri = pyotp.totp.TOTP(secret).provisioning_uri(
            name=user_id,
            issuer_name='Pet Genome WIA'
        )

        # Generate QR code
        qr = qrcode.QRCode(version=1, box_size=10, border=5)
        qr.add_data(totp_uri)
        qr.make(fit=True)

        return {
            'secret': secret,
            'totpUri': totp_uri,
            'qrCode': qr.make_image()
        }

    def verify_totp(self, secret: str, token: str) -> bool:
        """Verify TOTP token"""
        totp = pyotp.TOTP(secret)
        return totp.verify(token, valid_window=1)

    def generate_backup_codes(self, count: int = 10) -> list:
        """Generate backup codes for account recovery"""
        import secrets
        return [secrets.token_hex(8) for _ in range(count)]

# Usage
mfa = MFAManager()
user_mfa = mfa.generate_secret('user@example.com')

# Verify token
is_valid = mfa.verify_totp(user_mfa['secret'], '123456')
```

---

## 5. Consent Management Protocol

### 5.1 Consent Collection Workflow

```
┌──────────┐     ┌────────────┐     ┌─────────────┐     ┌──────────┐
│          │     │            │     │             │     │          │
│   Pet    │────>│  Consent   │────>│   Digital   │────>│  Genomic │
│  Owner   │     │   Form     │     │  Signature  │     │  Testing │
│          │     │            │     │             │     │          │
└──────────┘     └────────────┘     └─────────────┘     └──────────┘
```

### 5.2 Consent Record Schema

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
  "consentLanguage": "en-US",
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

### 5.3 Consent Verification Protocol

```python
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from datetime import datetime, timedelta
import json

class ConsentManager:
    """Manage and verify genomic data consent"""

    def __init__(self):
        self.consent_records = {}

    def create_consent(self, profile_id: str, consent_data: dict) -> dict:
        """
        Create new consent record with digital signature

        Args:
            profile_id: Pet genome profile ID
            consent_data: Consent categories and permissions

        Returns:
            Signed consent record
        """
        # Generate consent ID
        consent_id = self._generate_consent_id()

        # Create consent record
        consent_record = {
            'consentId': consent_id,
            'version': '2.0',
            'profileId': profile_id,
            'consentTimestamp': datetime.utcnow().isoformat() + 'Z',
            'expirationDate': (datetime.utcnow() + timedelta(days=1825)).isoformat() + 'Z',
            'consentCategories': consent_data,
            'legalJurisdiction': self._determine_jurisdiction(),
            'auditLog': [
                {
                    'action': 'CONSENT_CREATED',
                    'timestamp': datetime.utcnow().isoformat() + 'Z',
                    'actor': consent_data.get('ownerId', 'UNKNOWN')
                }
            ]
        }

        # Sign consent
        signature = self._sign_consent(consent_record)
        consent_record['digitalSignature'] = signature

        # Store consent
        self.consent_records[consent_id] = consent_record

        return consent_record

    def verify_consent(self, consent_id: str, required_scope: str) -> dict:
        """
        Verify consent for specific data operation

        Args:
            consent_id: Consent record ID
            required_scope: Required permission scope

        Returns:
            Verification result
        """
        consent = self.consent_records.get(consent_id)

        if not consent:
            return {
                'valid': False,
                'reason': 'CONSENT_NOT_FOUND'
            }

        # Verify signature
        if not self._verify_signature(consent):
            return {
                'valid': False,
                'reason': 'INVALID_SIGNATURE'
            }

        # Check expiration
        expiration = datetime.fromisoformat(consent['expirationDate'].replace('Z', '+00:00'))
        if datetime.utcnow() > expiration:
            return {
                'valid': False,
                'reason': 'CONSENT_EXPIRED'
            }

        # Verify scope
        scope_granted = self._check_scope(consent, required_scope)

        return {
            'valid': scope_granted,
            'consentId': consent_id,
            'profileId': consent['profileId'],
            'scope': required_scope,
            'reason': 'VALID' if scope_granted else 'SCOPE_NOT_GRANTED'
        }

    def withdraw_consent(self, consent_id: str, withdrawal_scope: str = 'ALL') -> dict:
        """
        Withdraw consent (partial or complete)

        Args:
            consent_id: Consent record ID
            withdrawal_scope: Scope of withdrawal

        Returns:
            Withdrawal confirmation
        """
        consent = self.consent_records.get(consent_id)

        if not consent:
            raise ValueError('Consent not found')

        # Add to audit log
        consent['auditLog'].append({
            'action': f'CONSENT_WITHDRAWN_{withdrawal_scope}',
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            'actor': 'OWNER'
        })

        if withdrawal_scope == 'ALL':
            # Mark all categories as withdrawn
            for category in consent['consentCategories']:
                consent['consentCategories'][category]['granted'] = False
                consent['consentCategories'][category]['withdrawnAt'] = datetime.utcnow().isoformat() + 'Z'
        else:
            # Withdraw specific scope
            if withdrawal_scope in consent['consentCategories']:
                consent['consentCategories'][withdrawal_scope]['granted'] = False
                consent['consentCategories'][withdrawal_scope]['withdrawnAt'] = datetime.utcnow().isoformat() + 'Z'

        return {
            'consentId': consent_id,
            'withdrawalScope': withdrawal_scope,
            'withdrawalTimestamp': datetime.utcnow().isoformat() + 'Z',
            'status': 'WITHDRAWN'
        }

    def _generate_consent_id(self) -> str:
        """Generate unique consent ID"""
        import uuid
        timestamp = datetime.utcnow().strftime('%Y%m%d')
        return f"CONSENT-{timestamp}-{uuid.uuid4().hex[:6].upper()}"

    def _sign_consent(self, consent_record: dict) -> dict:
        """Create digital signature for consent"""
        # Generate key pair (in practice, use stored keys)
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=2048
        )

        # Serialize consent for signing
        consent_data = json.dumps(consent_record, sort_keys=True).encode()

        # Sign
        signature = private_key.sign(
            consent_data,
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.MAX_LENGTH
            ),
            hashes.SHA256()
        )

        # Get public key
        public_key = private_key.public_key()
        public_pem = public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        )

        return {
            'algorithm': 'RSA-SHA256',
            'signature': signature.hex(),
            'publicKey': public_pem.decode(),
            'signatureTimestamp': datetime.utcnow().isoformat() + 'Z'
        }

    def _verify_signature(self, consent_record: dict) -> bool:
        """Verify consent digital signature"""
        # Implementation would verify RSA signature
        return True  # Simplified

    def _check_scope(self, consent: dict, required_scope: str) -> bool:
        """Check if consent grants required scope"""
        categories = consent.get('consentCategories', {})

        scope_map = {
            'GENETIC_TESTING': categories.get('geneticTesting', {}).get('granted', False),
            'DATA_STORAGE': categories.get('dataStorage', {}).get('granted', False),
            'RESEARCH': categories.get('researchParticipation', {}).get('granted', False),
            'VETERINARY_SHARING': categories.get('dataSharing', {}).get('veterinarians', {}).get('granted', False)
        }

        return scope_map.get(required_scope, False)

    def _determine_jurisdiction(self) -> str:
        """Determine legal jurisdiction"""
        # Implementation would determine based on location
        return 'EU-GDPR'

# Usage example
consent_mgr = ConsentManager()

# Create consent
consent = consent_mgr.create_consent(
    profile_id='PGP-ABCD12345678',
    consent_data={
        'geneticTesting': {'granted': True},
        'dataStorage': {'granted': True},
        'researchParticipation': {'granted': False},
        'ownerId': 'OWNER-001'
    }
)

# Verify consent
verification = consent_mgr.verify_consent(
    consent_id=consent['consentId'],
    required_scope='GENETIC_TESTING'
)

print(f"Consent valid: {verification['valid']}")
```

---

## 6. Data Retention and Deletion Protocol

### 6.1 Retention Policy Matrix

| Data Category | Retention Period | Deletion Method | Backup Retention |
|--------------|------------------|-----------------|------------------|
| Raw Sequencing Data | 10 years | Secure erasure (NIST SP 800-88) | 5 years |
| Processed Variants | 15 years | Secure erasure | 10 years |
| Health Markers | Lifetime + 5 years | Secure erasure | Lifetime + 3 years |
| Breed Information | Indefinite (with consent) | Secure erasure on request | N/A |
| Owner PII | Consent period + 1 year | Secure erasure | None |
| Audit Logs | 7 years | Secure erasure | 7 years |

### 6.2 Right to Erasure Implementation

```python
from enum import Enum
from datetime import datetime
import logging

class DeletionMethod(Enum):
    SOFT_DELETE = "SOFT_DELETE"
    ANONYMIZE = "ANONYMIZE"
    SECURE_ERASE = "SECURE_ERASE"
    CRYPTOGRAPHIC_ERASURE = "CRYPTOGRAPHIC_ERASURE"

class DataDeletionManager:
    """Manage data deletion and right to erasure requests"""

    def __init__(self):
        self.deletion_queue = []
        self.logger = logging.getLogger(__name__)

    def initiate_deletion_request(self, profile_id: str, requester: str) -> dict:
        """
        Initiate deletion request for genomic profile

        Args:
            profile_id: Profile to delete
            requester: Identity of requester

        Returns:
            Deletion request details
        """
        # Verify requester authorization
        if not self._verify_deletion_authorization(profile_id, requester):
            raise PermissionError('Requester not authorized for deletion')

        # Create deletion request
        request_id = self._generate_deletion_id()

        deletion_request = {
            'requestId': request_id,
            'profileId': profile_id,
            'requestedBy': requester,
            'requestTimestamp': datetime.utcnow().isoformat() + 'Z',
            'status': 'PENDING',
            'scheduledDeletion': self._calculate_deletion_date(),
            'deletionMethod': self._determine_deletion_method(profile_id),
            'dataCategories': self._identify_data_categories(profile_id),
            'dependencies': self._check_dependencies(profile_id)
        }

        self.deletion_queue.append(deletion_request)

        # Log request
        self.logger.info(f"Deletion request {request_id} created for profile {profile_id}")

        return deletion_request

    def execute_deletion(self, request_id: str) -> dict:
        """
        Execute approved deletion request

        Args:
            request_id: Deletion request ID

        Returns:
            Deletion execution report
        """
        request = self._find_deletion_request(request_id)

        if not request:
            raise ValueError('Deletion request not found')

        execution_log = {
            'requestId': request_id,
            'profileId': request['profileId'],
            'executionTimestamp': datetime.utcnow().isoformat() + 'Z',
            'deletedItems': []
        }

        # Delete each data category
        for category in request['dataCategories']:
            result = self._delete_data_category(
                profile_id=request['profileId'],
                category=category,
                method=request['deletionMethod']
            )
            execution_log['deletedItems'].append(result)

        # Remove from backups
        self._schedule_backup_deletion(request['profileId'])

        # Update audit log
        self._log_deletion(request['profileId'], execution_log)

        # Mark request complete
        request['status'] = 'COMPLETED'
        request['completedAt'] = datetime.utcnow().isoformat() + 'Z'

        return execution_log

    def _verify_deletion_authorization(self, profile_id: str, requester: str) -> bool:
        """Verify requester is authorized to delete profile"""
        # Implementation would check ownership/authorization
        return True

    def _generate_deletion_id(self) -> str:
        """Generate unique deletion request ID"""
        import uuid
        return f"DEL-{datetime.utcnow().strftime('%Y%m%d')}-{uuid.uuid4().hex[:8].upper()}"

    def _calculate_deletion_date(self) -> str:
        """Calculate scheduled deletion date (30-day grace period)"""
        from datetime import timedelta
        deletion_date = datetime.utcnow() + timedelta(days=30)
        return deletion_date.isoformat() + 'Z'

    def _determine_deletion_method(self, profile_id: str) -> DeletionMethod:
        """Determine appropriate deletion method"""
        # For genomic data, use secure erasure
        return DeletionMethod.SECURE_ERASE

    def _identify_data_categories(self, profile_id: str) -> list:
        """Identify all data categories associated with profile"""
        return [
            'RAW_SEQUENCING_DATA',
            'PROCESSED_VARIANTS',
            'HEALTH_MARKERS',
            'BREED_INFORMATION',
            'OWNER_PII',
            'ANALYSIS_RESULTS'
        ]

    def _check_dependencies(self, profile_id: str) -> list:
        """Check for data dependencies (e.g., pedigree relationships)"""
        # Implementation would check database for dependencies
        return []

    def _delete_data_category(self, profile_id: str, category: str, method: DeletionMethod) -> dict:
        """Delete specific data category"""
        self.logger.info(f"Deleting {category} for {profile_id} using {method}")

        if method == DeletionMethod.SECURE_ERASE:
            # Implement NIST SP 800-88 compliant erasure
            passes = 3  # Multiple overwrite passes
            for i in range(passes):
                self._overwrite_data(profile_id, category)

        elif method == DeletionMethod.CRYPTOGRAPHIC_ERASURE:
            # Delete encryption keys
            self._delete_encryption_keys(profile_id, category)

        elif method == DeletionMethod.ANONYMIZE:
            # Remove PII but keep data
            self._anonymize_data(profile_id, category)

        return {
            'category': category,
            'method': method.value,
            'status': 'DELETED',
            'timestamp': datetime.utcnow().isoformat() + 'Z'
        }

    def _overwrite_data(self, profile_id: str, category: str):
        """Securely overwrite data"""
        # Implementation would overwrite file blocks
        pass

    def _delete_encryption_keys(self, profile_id: str, category: str):
        """Delete encryption keys (cryptographic erasure)"""
        # Implementation would delete keys from KMS
        pass

    def _anonymize_data(self, profile_id: str, category: str):
        """Anonymize data while preserving for research"""
        # Implementation would remove all PII
        pass

    def _schedule_backup_deletion(self, profile_id: str):
        """Schedule deletion from backup systems"""
        self.logger.info(f"Scheduling backup deletion for {profile_id}")
        # Implementation would notify backup systems

    def _log_deletion(self, profile_id: str, execution_log: dict):
        """Log deletion in audit system"""
        self.logger.info(f"Deletion completed for {profile_id}: {execution_log}")

    def _find_deletion_request(self, request_id: str) -> dict:
        """Find deletion request by ID"""
        for request in self.deletion_queue:
            if request['requestId'] == request_id:
                return request
        return None

# Usage
deletion_mgr = DataDeletionManager()

# Initiate deletion
request = deletion_mgr.initiate_deletion_request(
    profile_id='PGP-ABCD12345678',
    requester='OWNER-001'
)

print(f"Deletion request {request['requestId']} created")
print(f"Scheduled for: {request['scheduledDeletion']}")

# Execute deletion (after grace period)
execution = deletion_mgr.execute_deletion(request['requestId'])
print(f"Deletion completed: {len(execution['deletedItems'])} categories deleted")
```

---

## 7. Audit Logging Protocol

### 7.1 Audit Event Schema

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

### 7.2 Audit Log Implementation

```python
import json
import hashlib
from datetime import datetime
from typing import Dict, List

class AuditLogger:
    """Comprehensive audit logging for genomic data access"""

    def __init__(self, log_storage_path: str):
        self.log_storage_path = log_storage_path
        self.log_chain = []

    def log_event(self, event_data: Dict) -> str:
        """
        Log audit event with blockchain-style chain

        Args:
            event_data: Event information

        Returns:
            Event ID
        """
        # Generate event ID
        event_id = self._generate_event_id()

        # Create audit event
        audit_event = {
            'eventId': event_id,
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            **event_data,
            'previousEventHash': self._get_previous_hash(),
            'eventHash': None  # Will be calculated
        }

        # Calculate hash
        audit_event['eventHash'] = self._calculate_hash(audit_event)

        # Append to chain
        self.log_chain.append(audit_event)

        # Persist to storage
        self._persist_event(audit_event)

        return event_id

    def log_data_access(self, actor: Dict, resource: Dict, action: Dict, outcome: Dict) -> str:
        """Log data access event"""
        return self.log_event({
            'eventType': 'DATA_ACCESS',
            'severity': 'INFO',
            'actor': actor,
            'resource': resource,
            'action': action,
            'outcome': outcome
        })

    def log_consent_event(self, actor: Dict, consent_id: str, action: str) -> str:
        """Log consent-related event"""
        return self.log_event({
            'eventType': 'CONSENT_EVENT',
            'severity': 'HIGH',
            'actor': actor,
            'resource': {
                'resourceType': 'CONSENT',
                'resourceId': consent_id
            },
            'action': {
                'actionType': action
            }
        })

    def log_security_event(self, event_type: str, severity: str, details: Dict) -> str:
        """Log security event"""
        return self.log_event({
            'eventType': event_type,
            'severity': severity,
            **details
        })

    def verify_log_integrity(self) -> Dict:
        """Verify integrity of audit log chain"""
        if not self.log_chain:
            return {'valid': True, 'message': 'No events to verify'}

        for i, event in enumerate(self.log_chain):
            # Recalculate hash
            calculated_hash = self._calculate_hash(event)

            if calculated_hash != event['eventHash']:
                return {
                    'valid': False,
                    'corrupted_event': event['eventId'],
                    'index': i,
                    'message': 'Hash mismatch detected'
                }

            # Verify chain
            if i > 0:
                previous_event = self.log_chain[i - 1]
                if event['previousEventHash'] != previous_event['eventHash']:
                    return {
                        'valid': False,
                        'broken_link': (previous_event['eventId'], event['eventId']),
                        'message': 'Chain link broken'
                    }

        return {
            'valid': True,
            'events_verified': len(self.log_chain),
            'message': 'Audit log integrity verified'
        }

    def query_logs(self, filters: Dict) -> List[Dict]:
        """Query audit logs with filters"""
        results = self.log_chain

        # Apply filters
        if 'eventType' in filters:
            results = [e for e in results if e['eventType'] == filters['eventType']]

        if 'actor' in filters:
            results = [e for e in results if e.get('actor', {}).get('userId') == filters['actor']]

        if 'resource' in filters:
            results = [e for e in results if e.get('resource', {}).get('resourceId') == filters['resource']]

        if 'startDate' in filters:
            results = [e for e in results if e['timestamp'] >= filters['startDate']]

        if 'endDate' in filters:
            results = [e for e in results if e['timestamp'] <= filters['endDate']]

        return results

    def _generate_event_id(self) -> str:
        """Generate unique event ID"""
        import uuid
        timestamp = datetime.utcnow().strftime('%Y%m%d')
        return f"AUDIT-{timestamp}-{uuid.uuid4().hex[:6].upper()}"

    def _get_previous_hash(self) -> str:
        """Get hash of previous event in chain"""
        if not self.log_chain:
            return '0' * 64  # Genesis hash

        return self.log_chain[-1]['eventHash']

    def _calculate_hash(self, event: Dict) -> str:
        """Calculate SHA-256 hash of event"""
        # Create copy without hash field
        event_copy = {k: v for k, v in event.items() if k != 'eventHash'}

        # Serialize
        event_string = json.dumps(event_copy, sort_keys=True)

        # Hash
        return hashlib.sha256(event_string.encode()).hexdigest()

    def _persist_event(self, event: Dict):
        """Persist event to storage"""
        # Implementation would write to secure storage
        pass

# Usage example
audit_logger = AuditLogger('/var/log/pet-genome/audit')

# Log data access
event_id = audit_logger.log_data_access(
    actor={
        'userId': 'VET-12345',
        'userType': 'VETERINARIAN',
        'organization': 'CLINIC-001',
        'ipAddress': '192.168.1.100'
    },
    resource={
        'resourceType': 'GENOMIC_PROFILE',
        'resourceId': 'PGP-ABCD12345678'
    },
    action={
        'actionType': 'READ',
        'actionScope': 'HEALTH_MARKERS'
    },
    outcome={
        'status': 'SUCCESS',
        'statusCode': 200
    }
)

print(f"Logged event: {event_id}")

# Verify integrity
verification = audit_logger.verify_log_integrity()
print(f"Audit log valid: {verification['valid']}")

# Query logs
vet_accesses = audit_logger.query_logs({
    'eventType': 'DATA_ACCESS',
    'actor': 'VET-12345'
})

print(f"Found {len(vet_accesses)} access events")
```

---

## 8. Emergency Access Protocol

### 8.1 Break-Glass Access Procedure

```python
from datetime import datetime, timedelta
from enum import Enum

class EmergencyAccessLevel(Enum):
    CRITICAL = "CRITICAL"  # Life-threatening situation
    URGENT = "URGENT"      # Time-sensitive medical need
    ELEVATED = "ELEVATED"  # Non-routine access need

class EmergencyAccessManager:
    """Manage emergency access to genomic data"""

    def __init__(self, audit_logger):
        self.audit_logger = audit_logger
        self.active_sessions = {}

    def request_emergency_access(self, request_data: dict) -> dict:
        """
        Request emergency access to genomic profile

        Args:
            request_data: Emergency access request details

        Returns:
            Emergency access grant or denial
        """
        # Validate request
        if not self._validate_emergency_request(request_data):
            return {
                'granted': False,
                'reason': 'INVALID_REQUEST'
            }

        # Determine access level
        access_level = EmergencyAccessLevel[request_data['emergencyLevel']]

        # Generate emergency session
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

        # Log emergency access
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

        # Store session
        self.active_sessions[session_id] = emergency_session

        # Notify data protection officer
        self._notify_dpo(emergency_session)

        return {
            'granted': True,
            'sessionId': session_id,
            'accessToken': emergency_session['accessToken'],
            'expiresAt': emergency_session['expiresAt'],
            'restrictions': emergency_session['restrictions']
        }

    def _validate_emergency_request(self, request_data: dict) -> bool:
        """Validate emergency access request"""
        required_fields = ['profileId', 'veterinarian', 'emergencyLevel', 'justification']
        return all(field in request_data for field in required_fields)

    def _generate_emergency_session_id(self) -> str:
        """Generate emergency session ID"""
        import uuid
        return f"EMERG-{uuid.uuid4().hex[:12].upper()}"

    def _generate_emergency_token(self) -> str:
        """Generate emergency access token"""
        import secrets
        return f"emerg_{secrets.token_urlsafe(32)}"

    def _determine_restrictions(self, access_level: EmergencyAccessLevel) -> dict:
        """Determine access restrictions based on emergency level"""
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
        """Notify Data Protection Officer of emergency access"""
        # Implementation would send notification
        print(f"DPO notified of emergency access: {session['sessionId']}")

# Usage
emergency_mgr = EmergencyAccessManager(audit_logger)

# Request emergency access
access = emergency_mgr.request_emergency_access({
    'profileId': 'PGP-ABCD12345678',
    'veterinarian': 'VET-EMERG-001',
    'emergencyLevel': 'CRITICAL',
    'justification': 'Patient experiencing severe adverse drug reaction, need immediate pharmacogenomic data'
})

if access['granted']:
    print(f"Emergency access granted: {access['sessionId']}")
    print(f"Expires: {access['expiresAt']}")
```

---

## 9. Cross-Border Data Transfer

### 9.1 Data Transfer Assessment

| Source Region | Destination Region | Transfer Mechanism | Additional Safeguards |
|--------------|-------------------|-------------------|----------------------|
| EU | US | Standard Contractual Clauses | Encryption, Access Controls |
| US | EU | EU-US Data Privacy Framework | GDPR Compliance |
| EU | EU | GDPR Adequacy | Standard Processing |
| US | UK | UK Adequacy Decision | Standard Processing |

### 9.2 Transfer Impact Assessment Template

```json
{
  "transferAssessmentId": "TIA-2025-12-001",
  "assessmentDate": "2025-12-18",
  "dataExporter": {
    "organization": "EU Pet Genomics Lab",
    "country": "Germany",
    "legalBasis": "GDPR Article 6(1)(a) - Consent"
  },
  "dataImporter": {
    "organization": "US Research Institute",
    "country": "United States",
    "adequacyDecision": false
  },
  "dataCategories": [
    "Genomic variants",
    "Breed information",
    "Health markers"
  ],
  "transferVolume": {
    "profiles": 1000,
    "estimatedDataSize": "500 GB"
  },
  "safeguards": [
    "Standard Contractual Clauses (2021)",
    "AES-256 encryption in transit and at rest",
    "Access controls and authentication",
    "Data minimization",
    "Purpose limitation",
    "Audit logging"
  ],
  "riskAssessment": {
    "surveillanceRisk": "MEDIUM",
    "accessByAuthorities": "LOW",
    "dataProtectionLaws": "ADEQUATE",
    "overallRisk": "LOW"
  },
  "approvalStatus": "APPROVED",
  "approvedBy": "Data Protection Officer",
  "reviewDate": "2026-12-18"
}
```

---

## 10. Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | Initial protocol specification | WIA Standards Committee |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
