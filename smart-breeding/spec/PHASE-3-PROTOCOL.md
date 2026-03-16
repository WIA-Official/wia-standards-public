# WIA Smart Breeding Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Data Exchange Protocol](#data-exchange-protocol)
3. [Genomic Data Sharing](#genomic-data-sharing)
4. [Federated Learning Protocol](#federated-learning-protocol)
5. [Privacy-Preserving Computation](#privacy-preserving-computation)
6. [Blockchain for Pedigree Verification](#blockchain-for-pedigree-verification)
7. [Real-Time Phenotype Streaming](#real-time-phenotype-streaming)
8. [Cross-Border Data Transfer](#cross-border-data-transfer)
9. [Quality Assurance Protocol](#quality-assurance-protocol)
10. [Compliance & Ethics](#compliance--ethics)

---

## Overview

### 1.1 Purpose

The WIA Smart Breeding Protocol Standard defines communication protocols for secure, privacy-preserving, and compliant genomic data exchange between breeding organizations, gene banks, research institutes, and breeding companies worldwide.

**Protocol Objectives**:
- Enable secure genomic data sharing across organizations
- Support federated learning without raw data sharing
- Implement privacy-preserving genomic computations
- Provide blockchain-based pedigree verification
- Enable real-time phenotype data streaming
- Ensure compliance with international regulations (GDPR, Nagoya Protocol)

### 1.2 Protocol Stack

```
┌────────────────────────────────────────┐
│   Application Layer (APIs)             │
├────────────────────────────────────────┤
│   Security Layer (Encryption, Auth)    │
├────────────────────────────────────────┤
│   Protocol Layer (WIA-BREEDING-v1)     │
├────────────────────────────────────────┤
│   Transport Layer (HTTPS, WebSocket)   │
├────────────────────────────────────────┤
│   Network Layer (Internet)             │
└────────────────────────────────────────┘
```

---

## Data Exchange Protocol

### 2.1 Protocol Header

Every data exchange message includes a standard header:

```json
{
  "wia_breeding_version": "1.0",
  "message_id": "MSG-2025-001-abc123",
  "timestamp": "2025-01-15T10:30:00Z",
  "sender": {
    "organization": "National Livestock Research Institute",
    "organization_id": "did:wia:breeding:nias-kr",
    "country": "KR",
    "contact": "breeding@nias.go.kr"
  },
  "recipient": {
    "organization": "Genomic Breeding Consortium",
    "organization_id": "did:wia:breeding:gbc-us"
  },
  "message_type": "GENOMIC_DATA_REQUEST",
  "encryption": "AES-256-GCM",
  "signature": "SHA-256-RSA",
  "payload": {...}
}
```

### 2.2 Message Types

| Message Type | Direction | Purpose |
|--------------|-----------|---------|
| `DATA_REQUEST` | Client → Server | Request genomic/phenotype data |
| `DATA_RESPONSE` | Server → Client | Deliver requested data |
| `DATA_UPLOAD` | Client → Server | Upload new breeding records |
| `DATA_UPLOAD_ACK` | Server → Client | Acknowledge upload |
| `EBV_CALCULATION_REQUEST` | Client → Server | Request breeding value calculation |
| `EBV_CALCULATION_RESULT` | Server → Client | Return calculated EBVs |
| `PEDIGREE_VERIFICATION` | Client → Server | Verify pedigree authenticity |
| `BLOCKCHAIN_NOTARIZATION` | Server → Blockchain | Record on blockchain |

### 2.3 Handshake Protocol

**Step 1: Client Hello**

```json
{
  "message_type": "CLIENT_HELLO",
  "protocol_version": "1.0",
  "supported_encryptions": ["AES-256-GCM", "ChaCha20-Poly1305"],
  "supported_compressions": ["GZIP", "ZSTD"],
  "client_public_key": "-----BEGIN PUBLIC KEY-----\n..."
}
```

**Step 2: Server Hello**

```json
{
  "message_type": "SERVER_HELLO",
  "protocol_version": "1.0",
  "selected_encryption": "AES-256-GCM",
  "selected_compression": "ZSTD",
  "server_public_key": "-----BEGIN PUBLIC KEY-----\n...",
  "session_id": "SESSION-2025-001"
}
```

**Step 3: Key Exchange (Diffie-Hellman)**

```json
{
  "message_type": "KEY_EXCHANGE",
  "dh_public_value": "0x1234567890abcdef...",
  "signature": "SHA-256 signature of DH public value"
}
```

**Step 4: Session Established**

```json
{
  "message_type": "SESSION_ESTABLISHED",
  "session_id": "SESSION-2025-001",
  "session_key_hash": "SHA-256 hash of derived session key",
  "expiration": "2025-01-15T18:30:00Z"
}
```

---

## Genomic Data Sharing

### 3.1 Data Request Protocol

**Request:**

```json
{
  "message_type": "GENOMIC_DATA_REQUEST",
  "request_id": "REQ-2025-001",
  "individuals": ["BULL-2025-001", "COW-2025-002"],
  "data_types": ["SNP_GENOTYPES", "SEQUENCE_DATA"],
  "chromosome_filter": [1, 2, 3],
  "format": "VCF",
  "authorization": {
    "material_transfer_agreement": "MTA-2025-001",
    "consent_type": "BROAD_CONSENT",
    "data_use_agreement": "RESEARCH_ONLY"
  }
}
```

**Response:**

```json
{
  "message_type": "GENOMIC_DATA_RESPONSE",
  "request_id": "REQ-2025-001",
  "status": "APPROVED",
  "data_package": {
    "package_id": "PKG-2025-001",
    "download_url": "https://secure-cdn.wia-breeding.org/packages/PKG-2025-001.vcf.gz",
    "checksum": "SHA-256:abcdef123456...",
    "size_bytes": 125648392,
    "expires_at": "2025-01-16T10:30:00Z"
  },
  "metadata": {
    "total_individuals": 2,
    "total_markers": 108002,
    "reference_genome": "ARS-UCD1.2",
    "genotyping_platform": "Illumina BovineSNP50"
  }
}
```

### 3.2 Data Upload Protocol

**Upload Initiation:**

```json
{
  "message_type": "DATA_UPLOAD_INIT",
  "upload_id": "UPL-2025-001",
  "file_name": "holstein_kr_genotypes_2025.vcf.gz",
  "file_size": 525648392,
  "checksum": "SHA-256:fedcba654321...",
  "metadata": {
    "total_individuals": 150,
    "total_markers": 54001,
    "species": "CATTLE",
    "breed": "Holstein"
  }
}
```

**Upload Acknowledgment:**

```json
{
  "message_type": "DATA_UPLOAD_ACK",
  "upload_id": "UPL-2025-001",
  "status": "READY",
  "upload_url": "https://upload.wia-breeding.org/v1/uploads/UPL-2025-001",
  "chunk_size": 10485760,
  "total_chunks": 51
}
```

**Chunked Upload:**

```http
PUT /v1/uploads/UPL-2025-001/chunk/1
Content-Type: application/octet-stream
Content-Range: bytes 0-10485759/525648392
X-Chunk-Checksum: SHA-256:chunk1hash...

[Binary data chunk 1]
```

**Upload Complete:**

```json
{
  "message_type": "DATA_UPLOAD_COMPLETE",
  "upload_id": "UPL-2025-001",
  "status": "PROCESSING",
  "estimated_completion": "2025-01-15T11:00:00Z"
}
```

---

## Federated Learning Protocol

### 4.1 Federated GBLUP Protocol

Enables collaborative breeding value estimation without sharing raw genomic data.

**Step 1: Global Model Initialization**

```json
{
  "message_type": "FEDERATED_INIT",
  "model_id": "GBLUP-FEDERATED-2025-001",
  "trait": "MILK_YIELD",
  "participants": [
    {"org_id": "did:wia:breeding:nias-kr", "population_size": 5000},
    {"org_id": "did:wia:breeding:usda-us", "population_size": 12000},
    {"org_id": "did:wia:breeding:csiro-au", "population_size": 3000}
  ],
  "hyperparameters": {
    "heritability": 0.35,
    "training_rounds": 10,
    "local_epochs": 5
  }
}
```

**Step 2: Local Training**

Each organization trains on their local data:

```json
{
  "message_type": "LOCAL_TRAINING_RESULT",
  "model_id": "GBLUP-FEDERATED-2025-001",
  "round": 1,
  "org_id": "did:wia:breeding:nias-kr",
  "model_update": {
    "gradient_vector": [0.123, -0.045, 0.678, ...],
    "sample_size": 5000,
    "training_loss": 0.245
  },
  "encrypted": true,
  "encryption_method": "SECURE_AGGREGATION"
}
```

**Step 3: Global Aggregation**

Central server aggregates updates:

```json
{
  "message_type": "GLOBAL_AGGREGATION",
  "model_id": "GBLUP-FEDERATED-2025-001",
  "round": 1,
  "aggregated_model": {
    "global_gradient": [0.098, -0.023, 0.456, ...],
    "total_samples": 20000,
    "convergence_metric": 0.012
  },
  "next_round_start": "2025-01-15T12:00:00Z"
}
```

**Step 4: Model Convergence**

```json
{
  "message_type": "FEDERATED_COMPLETE",
  "model_id": "GBLUP-FEDERATED-2025-001",
  "final_round": 10,
  "converged": true,
  "model_accuracy": 0.88,
  "model_download_url": "https://models.wia-breeding.org/GBLUP-FEDERATED-2025-001.bin"
}
```

### 4.2 Differential Privacy

Apply differential privacy to protect individual records:

```json
{
  "privacy_parameters": {
    "epsilon": 1.0,
    "delta": 1e-5,
    "clipping_threshold": 4.0,
    "noise_mechanism": "GAUSSIAN"
  }
}
```

---

## Privacy-Preserving Computation

### 5.1 Homomorphic Encryption for EBV Calculation

Calculate breeding values on encrypted genomic data:

**Client encrypts genotype:**

```json
{
  "message_type": "ENCRYPTED_GENOTYPE",
  "individual_id": "BULL-2025-001",
  "encryption_scheme": "CKKS_HOMOMORPHIC",
  "public_key_id": "PK-2025-001",
  "encrypted_genotypes": "BASE64_ENCODED_CIPHERTEXT..."
}
```

**Server computes EBV on encrypted data:**

```json
{
  "message_type": "ENCRYPTED_EBV_RESULT",
  "individual_id": "BULL-2025-001",
  "encrypted_ebv": "BASE64_ENCODED_CIPHERTEXT_EBV...",
  "computation_proof": "ZERO_KNOWLEDGE_PROOF..."
}
```

**Client decrypts result:**

```javascript
const decrypted_ebv = decrypt(encrypted_ebv, private_key);
// Result: GEBV = 920, Accuracy = 0.88
```

### 5.2 Secure Multi-Party Computation (MPC)

Enable joint genetic diversity calculation without revealing individual data:

```json
{
  "message_type": "MPC_DIVERSITY_CALCULATION",
  "participants": [
    {"org_id": "did:wia:breeding:nias-kr"},
    {"org_id": "did:wia:breeding:usda-us"},
    {"org_id": "did:wia:breeding:csiro-au"}
  ],
  "protocol": "SPDZ",
  "shares": {
    "party_1_share": "SECRET_SHARE_1...",
    "party_2_share": "SECRET_SHARE_2...",
    "party_3_share": "SECRET_SHARE_3..."
  },
  "result": {
    "effective_population_size": 142,
    "heterozygosity": 0.365
  }
}
```

---

## Blockchain for Pedigree Verification

### 6.1 Pedigree Registration on Blockchain

**Transaction Structure:**

```json
{
  "blockchain": "WIA-BREEDING-CHAIN",
  "transaction_type": "PEDIGREE_REGISTRATION",
  "transaction_id": "0xabcdef123456...",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "individual_id": "BULL-2025-001",
    "species": "CATTLE",
    "breed": "Holstein",
    "sire_id": "BULL-2020-045",
    "dam_id": "COW-2018-123",
    "birth_date": "2025-01-15",
    "genomic_hash": "SHA-256:genome_hash...",
    "registrar": "did:wia:breeding:nias-kr"
  },
  "signature": "ECDSA_SIGNATURE...",
  "block_number": 1234567,
  "confirmations": 12
}
```

### 6.2 Pedigree Verification

**Verification Request:**

```json
{
  "message_type": "PEDIGREE_VERIFY",
  "individual_id": "BULL-2025-001",
  "claimed_sire": "BULL-2020-045",
  "claimed_dam": "COW-2018-123"
}
```

**Verification Response:**

```json
{
  "message_type": "PEDIGREE_VERIFY_RESULT",
  "individual_id": "BULL-2025-001",
  "verified": true,
  "blockchain_tx": "0xabcdef123456...",
  "block_number": 1234567,
  "confirmations": 150,
  "genomic_match": {
    "parent_offspring_correlation": 0.52,
    "mendelian_consistency": 0.998
  }
}
```

### 6.3 Smart Contracts for Breeding Rights

```solidity
// Simplified smart contract example
contract BreedingRights {
    mapping(string => address) public individualOwner;
    mapping(string => bool) public breedingApproved;

    function registerIndividual(string memory individualId, address owner) public {
        individualOwner[individualId] = owner;
    }

    function approveForBreeding(string memory individualId) public {
        require(msg.sender == individualOwner[individualId], "Not owner");
        breedingApproved[individualId] = true;
    }

    function verifyBreedingRights(string memory individualId) public view returns (bool) {
        return breedingApproved[individualId];
    }
}
```

---

## Real-Time Phenotype Streaming

### 7.1 WebSocket Protocol

Enable real-time streaming of phenotype data from IoT devices:

**Connection Establishment:**

```javascript
const ws = new WebSocket('wss://stream.wia-breeding.org/v1/phenotype');

ws.onopen = () => {
  ws.send(JSON.stringify({
    "message_type": "SUBSCRIBE",
    "individual_id": "BULL-2025-001",
    "traits": ["ACTIVITY", "RUMINATION", "TEMPERATURE"]
  }));
};
```

**Real-Time Data Stream:**

```json
{
  "message_type": "PHENOTYPE_STREAM",
  "individual_id": "BULL-2025-001",
  "timestamp": "2025-01-15T10:30:15Z",
  "measurements": [
    {
      "trait": "ACTIVITY",
      "value": 145,
      "unit": "steps/hour"
    },
    {
      "trait": "RUMINATION",
      "value": 42,
      "unit": "minutes/hour"
    },
    {
      "trait": "BODY_TEMPERATURE",
      "value": 38.5,
      "unit": "celsius"
    }
  ],
  "sensor_id": "IOT-COLLAR-2025-001"
}
```

### 7.2 MQTT for IoT Phenotype Devices

**Topic Structure:**

```
wia-breeding/{organization_id}/{farm_id}/{individual_id}/{trait}
```

**Example:**

```bash
# Subscribe to milk yield data
mosquitto_sub -t "wia-breeding/nias-kr/farm-001/+/milk_yield"

# Publish temperature data
mosquitto_pub -t "wia-breeding/nias-kr/farm-001/BULL-2025-001/temperature" \
  -m '{"value": 38.5, "timestamp": "2025-01-15T10:30:15Z"}'
```

---

## Cross-Border Data Transfer

### 8.1 Material Transfer Agreement (MTA) Protocol

**MTA Request:**

```json
{
  "message_type": "MTA_REQUEST",
  "mta_id": "MTA-2025-001",
  "provider": {
    "organization": "National Livestock Research Institute",
    "country": "KR"
  },
  "recipient": {
    "organization": "USDA Agricultural Research Service",
    "country": "US"
  },
  "material_description": {
    "type": "GENOMIC_DATA",
    "individuals": 150,
    "species": "CATTLE",
    "markers": 54001
  },
  "terms": {
    "permitted_use": ["RESEARCH", "BREEDING"],
    "prohibited_use": ["COMMERCIAL_SALE"],
    "data_retention_years": 10,
    "citation_required": true
  },
  "nagoya_protocol_compliance": true
}
```

**MTA Approval:**

```json
{
  "message_type": "MTA_APPROVAL",
  "mta_id": "MTA-2025-001",
  "status": "APPROVED",
  "approval_date": "2025-01-20",
  "expiration_date": "2035-01-20",
  "digital_signature": "PROVIDER_SIGNATURE...",
  "recipient_signature": "RECIPIENT_SIGNATURE..."
}
```

### 8.2 GDPR Compliance

```json
{
  "gdpr_compliance": {
    "data_controller": "National Livestock Research Institute",
    "data_processor": "WIA Breeding Platform",
    "legal_basis": "LEGITIMATE_INTEREST",
    "data_subjects_informed": true,
    "right_to_erasure_implemented": true,
    "data_portability_enabled": true,
    "dpia_completed": true,
    "dpo_contact": "dpo@nias.go.kr"
  }
}
```

---

## Quality Assurance Protocol

### 9.1 Data Quality Metrics

```json
{
  "quality_report": {
    "upload_id": "UPL-2025-001",
    "total_individuals": 150,
    "total_markers": 54001,
    "quality_checks": [
      {
        "check": "SNP_CALL_RATE",
        "passed": 148,
        "failed": 2,
        "threshold": 0.90,
        "status": "PASS"
      },
      {
        "check": "MENDELIAN_INHERITANCE",
        "passed": 145,
        "failed": 5,
        "threshold": 0.95,
        "status": "WARNING"
      },
      {
        "check": "HETEROZYGOSITY_RANGE",
        "passed": 150,
        "failed": 0,
        "range": [0.25, 0.45],
        "status": "PASS"
      }
    ],
    "overall_quality_score": 0.96
  }
}
```

### 9.2 Audit Trail

```json
{
  "audit_log": [
    {
      "timestamp": "2025-01-15T10:30:00Z",
      "action": "DATA_UPLOAD",
      "user": "researcher@nias.go.kr",
      "resource": "UPL-2025-001",
      "status": "SUCCESS"
    },
    {
      "timestamp": "2025-01-15T10:45:00Z",
      "action": "EBV_CALCULATION",
      "user": "system",
      "resource": "BULL-2025-001",
      "method": "GBLUP",
      "status": "SUCCESS"
    }
  ]
}
```

---

## Compliance & Ethics

### 10.1 Animal Welfare Compliance

```json
{
  "animal_welfare": {
    "standard": "ISO 34700:2016",
    "phenotype_collection_methods": "NON_INVASIVE",
    "stress_minimization": true,
    "veterinary_oversight": true,
    "ethical_approval_number": "NIAS-IACUC-2025-001"
  }
}
```

### 10.2 Nagoya Protocol Compliance

```json
{
  "nagoya_protocol": {
    "access_permit": "ABS-2025-KR-001",
    "benefit_sharing_agreement": true,
    "prior_informed_consent": true,
    "mutually_agreed_terms": true,
    "country_of_origin": "KR",
    "genetic_resource_type": "LIVESTOCK_GERMPLASM"
  }
}
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License**: MIT
