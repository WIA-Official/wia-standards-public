# WIA-LEG-009: Right to be Forgotten Standard
## Version 1.0
## Status: Stable
## Date: 2025-01-15

---

## Abstract

This specification defines the WIA-LEG-009 standard for implementing the Right to be Forgotten (also known as the Right to Erasure) across digital systems, providing technical protocols, data formats, APIs, and verification mechanisms for GDPR Article 17, CCPA Section 1798.105, LGPD Article 18, and similar privacy regulations worldwide.

## 1. Introduction

### 1.1 Purpose

The Right to be Forgotten is a fundamental privacy right enabling individuals to request deletion of their personal data under specific legal circumstances. This standard provides:

- Uniform data formats for deletion requests and certificates
- Technical protocols for secure data erasure
- Verification and certification mechanisms
- API specifications for interoperability
- Compliance frameworks for global privacy regulations

### 1.2 Scope

This standard applies to:

- Data controllers and processors handling personal data
- Software systems storing or processing personal information
- Search engines and information aggregators
- Blockchain and distributed ledger systems
- Third-party data processors and service providers

### 1.3 Principles

弘益人間 (Benefit All Humanity) - This standard is guided by:

- **Privacy by Design**: Deletion capabilities built into systems from inception
- **Data Minimization**: Collect and retain only necessary data
- **Transparency**: Clear communication about deletion processes
- **Verifiability**: Cryptographic proof of deletion
- **Interoperability**: Standards-based implementation across platforms

## 2. Deletion Request Format

### 2.1 Request Schema

```json
{
  "$schema": "https://wia.org/schemas/deletion-request-v1.0.json",
  "requestId": "REQ-{timestamp}-{random}",
  "version": "1.0",
  "timestamp": "ISO8601 timestamp",
  "requestType": "gdpr_article17 | ccpa_deletion | lgpd_deletion | custom",
  "dataSubject": {
    "identifier": "email | userId | custom",
    "identifierValue": "string",
    "verification": {
      "method": "email_otp | sms_otp | account_login | government_id | custom",
      "verificationToken": "string",
      "verified": boolean,
      "verificationTimestamp": "ISO8601 timestamp"
    }
  },
  "dataCategories": ["profile", "activity", "financial", "communications", "..."],
  "legalBasis": "string - GDPR Article 17(1)(a-f) or equivalent",
  "jurisdiction": "EU | US_CA | BR | UK | ...",
  "additionalInformation": "string - optional context"
}
```

### 2.2 Data Categories

Standard data category identifiers:

- `profile`: User profile information (name, email, phone, address)
- `activity`: Activity logs, browsing history, usage patterns
- `financial`: Payment methods, transaction history, billing information  
- `location`: GPS coordinates, location history, check-ins
- `communications`: Messages, emails, call logs
- `media`: Photos, videos, audio recordings
- `behavioral`: Analytics, preferences, recommendations
- `biometric`: Fingerprints, facial recognition, voice prints
- `health`: Medical records, fitness data, genetic information
- `custom:{name}`: Custom category defined by controller

## 3. Deletion Protocol

### 3.1 Request Processing Workflow

1. **Receipt & Validation**
   - Receive deletion request via API or portal
   - Validate request schema and completeness
   - Assign unique request ID
   - Log request in audit trail

2. **Identity Verification**
   - Verify requester identity using specified method
   - Prevent fraudulent deletion requests
   - Document verification for audit

3. **Legal Assessment**
   - Evaluate legal grounds for deletion
   - Check for exceptions (legal obligations, public interest, etc.)
   - Document legal basis for decision

4. **Data Discovery**
   - Locate all instances of personal data across systems
   - Identify data in databases, backups, caches, logs, third parties
   - Generate comprehensive data inventory

5. **Deletion Execution**
   - Execute secure deletion using approved methods
   - Coordinate deletion across distributed systems
   - Handle backups, caches, and edge locations
   - Notify third-party processors

6. **Verification**
   - Verify deletion completion across all systems
   - Generate cryptographic proofs
   - Create Merkle tree of deletion events

7. **Certification**
   - Generate deletion certificate
   - Deliver certificate to data subject
   - Archive for compliance

8. **Monitoring**
   - Monitor for data resurrection
   - Schedule periodic verification
   - Maintain audit trail

### 3.2 Secure Deletion Methods

#### 3.2.1 DoD 5220.22-M (3-pass)

```
Pass 1: Overwrite with random character
Pass 2: Overwrite with complement of Pass 1
Pass 3: Overwrite with random character and verify
```

#### 3.2.2 Database Deletion

- Execute DELETE statements with proper WHERE clauses
- Handle foreign key cascades
- Run VACUUM or equivalent to reclaim space
- Address WAL and transaction logs
- Manage replication lag

#### 3.2.3 Cryptographic Erasure

- Encrypt data before storage
- Destroy encryption keys when deletion requested
- Provide cryptographic proof of key destruction
- Suitable for blockchain and immutable systems

### 3.3 Timelines

| Regulation | Standard Timeline | Extension | Maximum Total |
|------------|------------------|-----------|---------------|
| GDPR | 30 days | +60 days (if complex) | 90 days |
| CCPA/CPRA | 45 days | +45 days | 90 days |
| LGPD | 15 days | Case-by-case | 30 days |
| WIA-LEG-009 Recommendation | 14 days | +14 days | 28 days |

## 4. Deletion Certificate Format

### 4.1 Certificate Schema

```json
{
  "$schema": "https://wia.org/schemas/deletion-certificate-v1.0.json",
  "certificateId": "CERT-DEL-{timestamp}-{random}",
  "version": "1.0",
  "requestId": "string - original request ID",
  "issuedAt": "ISO8601 timestamp",
  "dataSubject": {
    "identifierHash": "SHA256 hash of identifier",
    "jurisdiction": "string"
  },
  "deletionDetails": {
    "categoriesDeleted": ["array of data category strings"],
    "recordsDeleted": integer,
    "storageLocations": ["array of storage location identifiers"],
    "deletionMethod": "DoD-5220.22-M | cryptographic_erasure | database_delete | ...",
    "completionTimestamp": "ISO8601 timestamp"
  },
  "verification": {
    "method": "merkle_proof | cryptographic_signature | third_party_audit | ...",
    "merkleRoot": "string - if using Merkle tree",
    "proofElements": ["array of proof components"],
    "witnessSignatures": ["array of witness attestations"]
  },
  "cryptographicProof": {
    "algorithm": "ED25519 | RSA-4096 | ECDSA | ...",
    "publicKey": "string",
    "signature": "string",
    "timestamp": "ISO8601 timestamp"
  },
  "blockchainAnchor": {
    "enabled": boolean,
    "blockchain": "ethereum | bitcoin | hyperledger | ...",
    "transactionHash": "string",
    "blockNumber": integer,
    "timestamp": "ISO8601 timestamp"
  },
  "auditTrail": {
    "events": [{
      "timestamp": "ISO8601",
      "actor": "string",
      "action": "string",
      "details": "object"
    }]
  },
  "issuer": {
    "organization": "string",
    "dpoContact": "string",
    "certificateUrl": "string - verification URL"
  }
}
```

## 5. API Specification

### 5.1 RESTful API Endpoints

#### Submit Deletion Request

```
POST /api/v1/deletion-requests
Content-Type: application/json
Authorization: Bearer {api_key}

Request Body: DeletionRequest (see Section 2.1)

Response: 201 Created
{
  "requestId": "string",
  "status": "pending_verification | processing | completed | denied",
  "estimatedCompletion": "ISO8601 timestamp",
  "trackingUrl": "string"
}
```

#### Check Deletion Status

```
GET /api/v1/deletion-requests/{requestId}
Authorization: Bearer {api_key}

Response: 200 OK
{
  "requestId": "string",
  "status": "string",
  "currentStage": "string",
  "progress": integer (0-100),
  "estimatedCompletion": "ISO8601 timestamp",
  "events": [...]
}
```

#### Retrieve Deletion Certificate

```
GET /api/v1/deletion-certificates/{requestId}
Authorization: Bearer {api_key}

Response: 200 OK
{DeletionCertificate} (see Section 4.1)
```

#### Verify Certificate

```
POST /api/v1/verification/certificate
Content-Type: application/json

{
  "certificateId": "string",
  "certificate": {DeletionCertificate}
}

Response: 200 OK
{
  "valid": boolean,
  "verificationDetails": {...}
}
```

## 6. Blockchain Integration

### 6.1 Cryptographic Erasure Protocol

For blockchain systems where traditional deletion is impossible:

1. Encrypt all personal data before storing on-chain
2. Store encryption keys in off-chain key management system
3. Upon deletion request, provably destroy encryption keys
4. Record key destruction on blockchain
5. Data remains on-chain but is cryptographically erased (unrecoverable)

### 6.2 Smart Contract Example

```solidity
contract DeletionRegistry {
    struct DeletionRecord {
        bytes32 requestHash;
        address dataController;
        uint256 timestamp;
        bytes32 merkleRoot;
        bool verified;
    }
    
    mapping(bytes32 => DeletionRecord) public deletions;
    
    event DeletionRecorded(bytes32 indexed requestHash, address indexed controller);
    
    function recordDeletion(bytes32 requestHash, bytes32 merkleRoot, bytes memory signature) public {
        // Verify signature
        // Record deletion
        deletions[requestHash] = DeletionRecord({
            requestHash: requestHash,
            dataController: msg.sender,
            timestamp: block.timestamp,
            merkleRoot: merkleRoot,
            verified: true
        });
        
        emit DeletionRecorded(requestHash, msg.sender);
    }
}
```

## 7. Compliance Framework

### 7.1 GDPR Compliance

- Implement all six grounds for erasure (Article 17(1))
- Support exceptions under Article 17(3)
- Respond within 30 days (extendable to 90)
- Inform third parties of deletion (Article 17(2))
- Maintain deletion audit trails

### 7.2 CCPA/CPRA Compliance

- Provide two methods for submission (web form, toll-free number)
- Verify requester identity
- Direct service providers to delete
- Respond within 45 days (extendable to 90)
- Support exceptions under Section 1798.105(d)

### 7.3 LGPD Compliance

- Respond within 15 days
- Support deletion for consent withdrawal
- Handle anonymization as alternative
- Coordinate with ANPD guidance

## 8. Security Considerations

### 8.1 Identity Verification

Prevent fraudulent deletion requests through:
- Multi-factor authentication
- Email/SMS OTP
- Account credentials
- Government ID verification (for sensitive data)
- Knowledge-based authentication

### 8.2 Audit Logging

Maintain immutable audit logs including:
- All deletion requests received
- Identity verification attempts
- Deletion execution events
- Verification results
- Certificate issuance
- Access to deletion systems

### 8.3 Access Controls

- Role-based access control (RBAC)
- Separation of duties
- Principle of least privilege
- Regular access reviews
- Secure credential management

## 9. Implementation Guidance

### 9.1 Prerequisites

- Comprehensive data mapping
- Data inventory and flow documentation
- Privacy impact assessments
- DPO appointment and training
- System architecture documentation

### 9.2 Recommended Architecture

- Centralized deletion orchestration service
- Message queue for async processing
- Distributed deletion workers
- Verification service
- Certificate generation service
- Audit logging infrastructure
- Monitoring and alerting

### 9.3 Testing Requirements

- Unit tests for deletion functions
- Integration tests across systems
- End-to-end deletion workflows
- Verification testing
- Performance testing at scale
- Disaster recovery testing
- Security penetration testing

## 10. Conformance

### 10.1 Conformance Levels

**Level 1 - Basic**: Supports deletion request acceptance, basic deletion, and confirmation
**Level 2 - Standard**: Adds verification, certification, and audit logging
**Level 3 - Advanced**: Includes cryptographic proofs, blockchain anchoring, and automated compliance reporting

### 10.2 Certification

Organizations may seek WIA-LEG-009 certification through:
- Self-assessment against spec
- Third-party audit
- Continuous compliance monitoring

## Appendix A: Glossary

- **Data Controller**: Entity determining purposes and means of processing
- **Data Processor**: Entity processing data on behalf of controller
- **Data Subject**: Individual whose personal data is processed
- **Erasure**: Permanent deletion making recovery infeasible
- **Pseudonymization**: Processing rendering data non-identifiable without additional information
- **Right to be Forgotten**: Legal right to request data deletion

## Appendix B: References

- GDPR (EU) 2016/679
- CCPA California Civil Code §1798.100-199
- LGPD Lei Nº 13.709/2018
- ISO/IEC 27001:2013
- NIST SP 800-88 Rev. 1
- DoD 5220.22-M

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
