# WIA-DIGITAL-WILL Specification v1.0

## 1. Introduction

### 1.1 Purpose
This specification defines the WIA-DIGITAL-WILL standard for creating, storing, validating, and executing digital last wills and testaments within the WIA ecosystem.

### 1.2 Scope
- Will document structure
- Asset declaration format
- Beneficiary designation
- Witness requirements
- Signature and authentication
- Storage and retrieval
- Revocation and amendment
- Legal compliance

### 1.3 Philosophy
Hongik Ingan: Every person has the right to determine the fate of their digital legacy with the same legal protection afforded to physical property.

## 2. Will Structure

### 2.1 Document Format

```
DIGITAL-WILL Document
=====================

Header:
  will_id: string (UUID)
  version: number
  created_at: timestamp
  last_modified: timestamp
  jurisdiction: string (ISO country code)
  language: string (ISO language code)
  status: 'draft' | 'signed' | 'revoked'

Testator:
  wia_id: string
  full_name: string
  date_of_birth: date
  identification: {
    type: string
    number: string
    issuer: string
  }

Executor:
  primary: {
    wia_id: string
    relationship: string
    acceptance_status: 'pending' | 'accepted' | 'declined'
  }
  backup: {
    wia_id: string
    relationship: string
    activate_after_days: number
  }

Bequests: [
  {
    bequest_id: string
    asset: AssetDeclaration
    beneficiary: BeneficiaryDesignation
    instructions: string
    conditions: Condition[]
  }
]

Witnesses: [
  {
    wia_id: string
    full_name: string
    relationship: string
    attestation_time: timestamp
    signature: string
  }
]

Signatures:
  testator: {
    signature: string
    timestamp: timestamp
    method: 'biometric' | 'key' | 'passphrase'
  }
  notary: {
    wia_id: string
    signature: string
    timestamp: timestamp
  } (optional)

Metadata:
  previous_versions: string[]
  revokes: string | null
  revoked_by: string | null
```

### 2.2 Asset Declaration

```
AssetDeclaration
================

type: 'account' | 'files' | 'crypto' | 'domain' | 'ip' | 'purchase'

For account:
  platform: string
  username: string
  account_id: string (optional)
  estimated_value: number (optional)

For files:
  source: string
  paths: string[]
  include_patterns: string[]
  exclude_patterns: string[]
  estimated_size: string

For crypto:
  currency: string
  wallet_type: 'hardware' | 'software' | 'exchange'
  wallet_address: string
  estimated_value: number

For domain:
  domain_name: string
  registrar: string
  expires: date

For ip:
  type: 'code' | 'design' | 'content' | 'other'
  description: string
  locations: string[]
  license: string

For purchase:
  platform: string
  item_type: 'game' | 'ebook' | 'music' | 'software'
  items: string[]
```

### 2.3 Beneficiary Designation

```
BeneficiaryDesignation
======================

primary: {
  wia_id: string
  full_name: string
  relationship: string
  share: number (percentage)
}

alternates: [
  {
    wia_id: string
    full_name: string
    relationship: string
    condition: string
  }
]

fallback: 'executor' | 'delete' | 'archive'
```

## 3. Witness Requirements

### 3.1 Minimum Requirements
- At least 2 witnesses required
- Witnesses must have valid WIA ID
- Witnesses cannot be beneficiaries
- Witnesses must be adult age in jurisdiction
- Witnesses must attest within 30 days of testator signature

### 3.2 Witness Attestation

```javascript
const attestation = {
  statement: 'I attest that I witnessed the testator sign this will of their own free will.',
  testator_identity_confirmed: true,
  testator_appeared_competent: true,
  no_coercion_observed: true,
  witness_signature: '...',
  timestamp: '2025-01-01T10:00:00Z'
};
```

### 3.3 Remote Witnessing
Allowed under following conditions:
- Video verification of testator identity
- Real-time witnessing via video call
- Recording of signing session stored
- Both parties in supported jurisdictions

## 4. Signature and Authentication

### 4.1 Signature Methods

```
Biometric:
  type: 'fingerprint' | 'face' | 'voice'
  device_id: string
  confidence: number (0-1)
  verification_hash: string

Cryptographic Key:
  algorithm: 'ed25519' | 'rsa-4096'
  public_key: string
  signature: string

Passphrase:
  method: 'argon2id'
  verification_hash: string
  recovery_options: string[]
```

### 4.2 Multi-Factor Requirements
For signing a will:
- Primary authentication (password/biometric)
- Secondary verification (SMS/email code)
- Identity confirmation (video/ID scan)

### 4.3 Signature Validity
- Signature must be timestamped
- Timestamp must be from trusted source
- Signature algorithm must be current standard
- Key must not be on revocation list

## 5. Storage and Retrieval

### 5.1 Primary Storage
- Encrypted at rest (AES-256-GCM)
- Distributed across multiple nodes
- Minimum 3 replica copies
- Geographic distribution

### 5.2 Access Control

```javascript
const accessPolicy = {
  testator: {
    read: true,
    write: true,
    revoke: true
  },
  executor: {
    read: 'after_death_declared',
    write: false,
    execute: 'after_grace_period'
  },
  beneficiary: {
    read: 'own_bequests_only',
    write: false
  },
  witness: {
    read: 'attestation_only',
    write: false
  }
};
```

### 5.3 Backup and Recovery
- Testator receives encrypted backup key
- Key can be stored in physical vault
- Recovery requires 2-of-3 key holders
- Annual verification of accessibility

## 6. Revocation and Amendment

### 6.1 Revocation

```javascript
await will.revoke({
  reason: 'superseded_by_new_will',
  new_will_id: 'will-uuid-5678',
  signature: testator_signature
});
```

### 6.2 Codicil (Amendment)

```javascript
const codicil = await will.createCodicil({
  changes: [
    {
      type: 'modify_bequest',
      bequest_id: 'bequest-123',
      new_beneficiary: 'wia:new.beneficiary.9999'
    },
    {
      type: 'add_bequest',
      asset: { type: 'crypto', currency: 'bitcoin', ... },
      beneficiary: 'wia:child.3333'
    }
  ]
});

await codicil.addWitnesses([...]);
await codicil.sign();
```

### 6.3 Version History
- All versions retained
- Changes tracked with diff
- Only latest signed version executes
- Previous versions for audit trail

## 7. Legal Compliance

### 7.1 Jurisdiction Support

```
Supported Jurisdictions:
  KR: Korea
    - 민법 제1060조 자필증서의 방식
    - Electronic Signatures Act compliance
    - 2 witnesses required

  US: United States
    - RUFADAA compliance
    - State-specific requirements
    - Typically 2 witnesses

  EU: European Union
    - eIDAS regulation compliance
    - GDPR data handling
    - Member state variations
```

### 7.2 Legal Validity Elements
- Clear testamentary intent
- Testator identity verified
- Mental capacity (implied by authentication)
- No undue influence (witness attestation)
- Proper execution (signature + witnesses)

### 7.3 Court Integration
- Export to court-recognized formats
- Witness contact information preserved
- Authentication logs available
- Chain of custody documented

## 8. API Reference

### 8.1 Will Creation

```javascript
import { DigitalWill } from '@anthropic/wia-digital-will';

const will = new DigitalWill({
  testator: 'wia:testator.1234',
  executor: 'wia:executor.5678',
  jurisdiction: 'KR',
  language: 'ko'
});
```

### 8.2 Adding Bequests

```javascript
will.bequeath({
  asset: {
    type: 'account',
    platform: 'instagram',
    username: '@myaccount'
  },
  beneficiary: {
    wia_id: 'wia:spouse.5678',
    share: 100
  },
  instructions: 'memorialize'
});
```

### 8.3 Witness Management

```javascript
// Request witness
await will.requestWitness('wia:witness.1111', {
  message: 'Please witness my digital will'
});

// Witness attestation
await will.recordAttestation('wia:witness.1111', attestation);
```

### 8.4 Signing

```javascript
// Sign will
await will.sign({
  method: 'biometric',
  secondary: 'sms_code'
});

// Verify signatures
const valid = await will.verify();
```

### 8.5 Retrieval

```javascript
// Load will
const will = await DigitalWill.load('will-uuid-1234');

// Get summary
const summary = will.getSummary();

// Export for legal purposes
const legalDoc = await will.export('pdf');
```

## 9. Security Considerations

### 9.1 Threat Model
- Unauthorized access
- Forged signatures
- Coerced creation
- Post-death tampering
- Key compromise

### 9.2 Mitigations
- Strong encryption
- Multi-factor authentication
- Witness requirements
- Immutable audit logs
- Key rotation support

### 9.3 Privacy
- Will contents encrypted
- Beneficiaries notified only after death
- Asset details hidden until execution
- Witness identities protected

## 10. Interoperability

### 10.1 With DIGITAL-FUNERAL
- Will provides asset inventory
- Funeral plan references will
- Execution follows will instructions

### 10.2 With DIGITAL-EXECUTOR
- Executor permissions defined in will
- Executor duties documented
- Reporting requirements specified

### 10.3 Export Formats
- JSON (machine readable)
- PDF (human readable)
- XML (legal interchange)
- Plain text (accessibility)

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01 | Initial release |

---

WIA-DIGITAL-WILL: Your digital legacy, legally protected.
