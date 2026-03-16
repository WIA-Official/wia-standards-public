# WIA-SOC-013 PHASE 3: PROTOCOL SPECIFICATION

**Public Document Standard - Security Protocols**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. Digital Signature Protocol

### 1.1 Supported Algorithms

- **ECDSA**: secp256r1 (P-256), secp384r1 (P-384)
- **RSA**: 2048, 3072, 4096 bits
- **EdDSA**: Ed25519, Ed448
- **Post-Quantum**: CRYSTALS-Dilithium (future-ready)

### 1.2 Signature Process

1. Calculate SHA-256 hash of document
2. Sign hash with private key
3. Embed signature in PDF using PAdES-LTV
4. Include certificate chain
5. Add RFC 3161 timestamp
6. Anchor hash to blockchain

## 2. PKI Infrastructure

### 2.1 Certificate Hierarchy

```
Root CA (20-year validity)
  ├── Intermediate CA (10-year)
  │   ├── Issuing CA (5-year)
  │   │   └── End-Entity Cert (2-year)
```

### 2.2 Certificate Validation

1. Verify signature chain to trusted root
2. Check certificate expiration
3. Query OCSP for revocation status
4. Validate certificate policies
5. Check key usage extensions

## 3. Encryption

### 3.1 At Rest

- Algorithm: AES-256-GCM
- Key Management: AWS KMS / Azure Key Vault / HSM
- Key Rotation: Annual

### 3.2 In Transit

- Protocol: TLS 1.3
- Cipher Suites: TLS_AES_256_GCM_SHA384
- Certificate Pinning: Recommended for mobile apps

## 4. Blockchain Anchoring

### 4.1 Supported Networks

- Ethereum Mainnet
- Polygon (Layer 2)
- Solana
- Hyperledger Fabric (permissioned)

### 4.2 Anchoring Process

```solidity
contract DocumentRegistry {
    mapping(bytes32 => DocumentAnchor) public anchors;
    
    struct DocumentAnchor {
        bytes32 documentHash;
        address issuer;
        uint256 timestamp;
        string documentId;
    }
    
    function anchor(bytes32 hash, string memory docId) public {
        anchors[hash] = DocumentAnchor(hash, msg.sender, block.timestamp, docId);
    }
}
```

## 5. Zero-Knowledge Proofs

### 5.1 Selective Disclosure

Prove attributes without revealing full document:

- Prove "age > 18" without revealing date of birth
- Prove "citizen of country X" without revealing full ID
- Prove "valid license" without revealing license number

### 5.2 zk-SNARK Implementation

```
Circuit:
  public input: attribute_commitment
  private input: full_document, salt
  
  verify:
    hash(full_document, salt) == attribute_commitment
    attribute(full_document) satisfies condition
```

## 6. Revocation

### 6.1 CRL (Certificate Revocation List)

- Published every 24 hours
- Signed by issuing CA
- Available via HTTP and LDAP

### 6.2 OCSP (Online Certificate Status Protocol)

- Real-time status queries
- Response signed by OCSP responder
- Must-staple for critical documents

## 7. Audit Trail

Every operation logged with:
- Timestamp (RFC 3339)
- Actor (DID or username)
- Action (create, read, sign, verify, revoke)
- Document ID
- IP address
- Result (success/failure)

---

© 2025 SmileStory Inc. / WIA
