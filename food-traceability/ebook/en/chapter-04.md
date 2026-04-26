# Chapter 4: Blockchain Integration for Immutable Records

**WIA-AGRI-016 eBook Series**

---

## Why Blockchain for Traceability?

Traditional databases have inherent trust issues:
- Centralized control (single point of failure)
- Mutable records (can be altered retroactively)
- Trust dependency (requires trusting the database owner)

Blockchain provides:
- **Immutability:** Records cannot be altered once written
- **Decentralization:** No single point of control
- **Transparency:** All participants can verify records
- **Trust:** Cryptographic proof replaces institutional trust

---

## Blockchain Architecture for Food Traceability

### Hybrid Approach

```
┌─────────────────────────────────────────────────┐
│           Application Layer                      │
│   (Mobile Apps, Web Portals, Analytics)         │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│         Traceability Middleware                  │
│  (Business Logic, Access Control, Validation)   │
└──────────────────┬──────────────────────────────┘
                   │
          ┌────────┴────────┐
          ▼                  ▼
┌──────────────┐    ┌──────────────┐
│  Off-Chain   │    │  Blockchain  │
│  Database    │    │  Network     │
│              │    │              │
│ • Full data  │    │ • Hashes     │
│ • Searchable │    │ • Proofs     │
│ • Fast       │    │ • Immutable  │
└──────────────┘    └──────────────┘
```

**Rationale:**
- Store full event data off-chain for performance
- Store cryptographic hashes on-chain for immutability
- Best of both worlds: speed + security

---

## Smart Contract Implementation

### Batch Registry Contract

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
        require(!batches[batchId].exists, "Batch already registered");

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
        require(batches[batchId].exists, "Batch not registered");

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

    function getBatchEventCount(string memory batchId) public view returns (uint256) {
        return batchEvents[batchId].length;
    }

    function getBatchEvent(string memory batchId, uint256 index)
        public view returns (bytes32, uint256, address)
    {
        require(index < batchEvents[batchId].length, "Index out of bounds");
        Event memory ev = batchEvents[batchId][index];
        return (ev.eventHash, ev.timestamp, ev.recordedBy);
    }
}
```

### Verifiable Credentials Contract

```solidity
contract CertificationRegistry {
    struct Certification {
        string certType;
        bytes32 credentialHash;
        address issuer;
        uint256 issuedDate;
        uint256 expiryDate;
        bool revoked;
    }

    mapping(address => mapping(bytes32 => Certification)) public certifications;

    event CertificationIssued(
        address indexed holder,
        bytes32 indexed certId,
        string certType,
        address issuer,
        uint256 expiryDate
    );

    event CertificationRevoked(
        address indexed holder,
        bytes32 indexed certId,
        address revokedBy
    );

    function issueCertification(
        address holder,
        bytes32 certId,
        string memory certType,
        bytes32 credentialHash,
        uint256 expiryDate
    ) public {
        certifications[holder][certId] = Certification({
            certType: certType,
            credentialHash: credentialHash,
            issuer: msg.sender,
            issuedDate: block.timestamp,
            expiryDate: expiryDate,
            revoked: false
        });

        emit CertificationIssued(holder, certId, certType, msg.sender, expiryDate);
    }

    function revokeCertification(address holder, bytes32 certId) public {
        Certification storage cert = certifications[holder][certId];
        require(cert.issuer == msg.sender, "Only issuer can revoke");

        cert.revoked = true;
        emit CertificationRevoked(holder, certId, msg.sender);
    }

    function verifyCertification(
        address holder,
        bytes32 certId,
        bytes32 credentialHash
    ) public view returns (bool valid, bool expired, bool revoked) {
        Certification memory cert = certifications[holder][certId];

        valid = cert.credentialHash == credentialHash;
        expired = block.timestamp > cert.expiryDate;
        revoked = cert.revoked;

        return (valid, expired, revoked);
    }
}
```

---

## Integration with Traceability System

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
    // Create hash of batch data
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(batchData))
    );

    // Upload full data to IPFS
    const ipfsHash = await this.uploadToIPFS(batchData);

    // Submit to blockchain
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
    // Hash event data
    const eventHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(eventData))
    );

    // Submit to blockchain
    const tx = await this.batchRegistry.recordEvent(batchId, eventHash);
    const receipt = await tx.wait();

    // Store full event off-chain
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

  async getBatchHistory(batchId) {
    const eventCount = await this.batchRegistry.getBatchEventCount(batchId);

    const events = [];
    for (let i = 0; i < eventCount; i++) {
      const [eventHash, timestamp, recordedBy] = await this.batchRegistry.getBatchEvent(batchId, i);
      events.push({
        index: i,
        eventHash: eventHash,
        timestamp: Number(timestamp),
        recordedBy: recordedBy
      });
    }

    return {
      batchId: batchId,
      totalEvents: Number(eventCount),
      events: events
    };
  }
}
```

### Usage Example

```javascript
const blockchain = new BlockchainTraceability({
  rpcUrl: 'https://mainnet.infura.io/v3/YOUR-PROJECT-ID',
  privateKey: process.env.PRIVATE_KEY,
  batchRegistryAddress: '0x...'
});

// Register new batch
const batchData = {
  batchId: '01234567890128.LOT2025001',
  product: 'Organic Apples',
  origin: 'ABC Organic Farm',
  harvestDate: '2025-12-01',
  quantity: 1000,
  certifications: ['USDA Organic']
};

const registration = await blockchain.registerBatch(batchData);
console.log('Batch registered:', registration.transactionHash);

// Record event
const event = {
  eventId: 'evt_001',
  eventType: 'shipping',
  timestamp: new Date().toISOString(),
  location: 'Distribution Center',
  temperature: 4.2
};

const eventRecord = await blockchain.recordEvent(batchData.batchId, event);
console.log('Event recorded:', eventRecord.transactionHash);

// Verify batch
const verification = await blockchain.verifyBatch(batchData.batchId, batchData);
console.log('Batch verified:', verification.verified);
```

---

## IPFS Integration

### Distributed Storage

IPFS (InterPlanetary File System) provides decentralized storage:

```javascript
const IPFS = require('ipfs-http-client');

class IPFSStorage {
  constructor() {
    this.client = IPFS.create({
      host: 'ipfs.infura.io',
      port: 5001,
      protocol: 'https'
    });
  }

  async upload(data) {
    const result = await this.client.add(JSON.stringify(data));
    return result.path;  // IPFS hash
  }

  async download(ipfsHash) {
    const stream = this.client.cat(ipfsHash);
    let data = '';

    for await (const chunk of stream) {
      data += chunk.toString();
    }

    return JSON.parse(data);
  }
}

// Usage
const ipfs = new IPFSStorage();

const batchData = {
  batchId: '01234567890128.LOT2025001',
  product: 'Organic Apples',
  // ... full batch data
};

const ipfsHash = await ipfs.upload(batchData);
console.log('IPFS Hash:', ipfsHash);
// Output: QmXxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// Retrieve data
const retrieved = await ipfs.download(ipfsHash);
console.log('Retrieved:', retrieved);
```

---

## W3C Verifiable Credentials

### Credential Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/traceability/v1"
  ],
  "type": ["VerifiableCredential", "OrganicCertificationCredential"],
  "id": "urn:uuid:a1234567-89ab-cdef-0123-456789abcdef",
  "issuer": {
    "id": "did:web:usda.gov",
    "name": "USDA Organic Certification Program"
  },
  "issuanceDate": "2025-01-15T00:00:00Z",
  "expirationDate": "2027-01-15T23:59:59Z",
  "credentialSubject": {
    "id": "did:web:abcfarm.com",
    "name": "ABC Organic Farm",
    "certifications": [
      {
        "type": "USDA Organic",
        "certificationNumber": "ORG-123456",
        "scope": ["Fresh Produce", "Apples"],
        "certifiedSince": "2020-01-15"
      }
    ]
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T08:00:00Z",
    "verificationMethod": "did:web:usda.gov#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQjecWRftX..."
  }
}
```

### Credential Verification

```javascript
const { verifyCredential } = require('@digitalbazaar/vc');

async function verifyOrganicCertificate(credential) {
  try {
    const result = await verifyCredential({
      credential: credential,
      suite: new Ed25519Signature2020(),
      documentLoader: customDocumentLoader
    });

    if (result.verified) {
      // Check expiration
      const now = new Date();
      const expiryDate = new Date(credential.expirationDate);

      if (now > expiryDate) {
        return {
          verified: false,
          reason: 'Credential expired'
        };
      }

      // Check blockchain revocation status
      const onChainStatus = await checkBlockchainRevocation(credential.id);

      return {
        verified: !onChainStatus.revoked,
        reason: onChainStatus.revoked ? 'Credential revoked' : 'Valid',
        issuer: credential.issuer.name,
        subject: credential.credentialSubject.name,
        expiryDate: credential.expirationDate
      };
    }

    return {
      verified: false,
      reason: 'Invalid signature'
    };
  } catch (error) {
    return {
      verified: false,
      reason: error.message
    };
  }
}
```

---

## Choosing a Blockchain Network

### Public vs. Private

**Public Blockchains (Ethereum, Polygon):**
- Pros: Maximum transparency, no central authority
- Cons: Higher costs, slower transactions, public data

**Private/Consortium Blockchains (Hyperledger Fabric):**
- Pros: Lower costs, faster, permissioned access
- Cons: Less decentralization, requires governance

### Recommendation by Scale

| Scale | Recommended | Reason |
|-------|-------------|--------|
| Small/Medium | Polygon | Low gas fees, fast, Ethereum-compatible |
| Large Enterprise | Hyperledger Fabric | High throughput, privacy controls |
| Global Consortium | Ethereum L2 (Arbitrum/Optimism) | Security + scalability |

---

## Cost Optimization

### Gas Optimization Techniques

```solidity
// Expensive: Storing full string on-chain
function registerBatchBad(string memory batchId, string memory fullData) public {
    batches[batchId] = fullData;  // Very expensive!
}

// Optimized: Store only hash
function registerBatchGood(string memory batchId, bytes32 dataHash, string memory ipfsUrl) public {
    batches[batchId] = Batch(dataHash, ipfsUrl, msg.sender, block.timestamp);
}
```

### Batching Transactions

```javascript
// Batch multiple events into single transaction
async function batchRecordEvents(batchId, events) {
  // Merkle tree of event hashes
  const tree = new MerkleTree(events.map(e => hashEvent(e)));
  const root = tree.getRoot();

  // Single blockchain transaction with Merkle root
  const tx = await contract.recordEventBatch(batchId, root);

  // Store individual events off-chain with Merkle proofs
  for (const event of events) {
    const proof = tree.getProof(hashEvent(event));
    await storeEventWithProof(event, proof);
  }

  return tx;
}
```

---

## Security Best Practices

### 1. Private Key Management

```javascript
// Bad: Hardcoded private key
const privateKey = '0x1234...';  // Never do this!

// Good: Environment variables + Hardware Security Module
const privateKey = process.env.PRIVATE_KEY;

// Better: AWS KMS, Azure Key Vault, or HSM
const signer = new KMSSigner(kmsKeyId);
```

### 2. Access Control

```solidity
import "@openzeppelin/contracts/access/AccessControl.sol";

contract SecureRegistry is AccessControl {
    bytes32 public constant REGISTRAR_ROLE = keccak256("REGISTRAR_ROLE");
    bytes32 public constant AUDITOR_ROLE = keccak256("AUDITOR_ROLE");

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
    }

    function registerBatch(string memory batchId, bytes32 dataHash)
        public
        onlyRole(REGISTRAR_ROLE)
    {
        // Only authorized registrars can register batches
    }

    function auditBatch(string memory batchId)
        public
        view
        onlyRole(AUDITOR_ROLE)
        returns (Batch memory)
    {
        // Only auditors can access full audit data
    }
}
```

---

## Chapter Summary

Blockchain adds immutability and trust to food traceability:

**Key Benefits:**
- Tamper-proof records
- Decentralized verification
- Cryptographic proof of authenticity
- Automated trust without intermediaries

**Implementation Approach:**
- Hybrid architecture (off-chain + on-chain)
- Smart contracts for registry and verification
- IPFS for distributed storage
- W3C Verifiable Credentials for certifications

**Best Practices:**
- Choose appropriate network (public vs. private)
- Optimize gas costs (hashes, not data)
- Secure private key management
- Role-based access control

---

## Next Chapter

**Chapter 5: IoT Sensors and Real-Time Monitoring**

Learn how to integrate IoT sensors for continuous cold chain monitoring and quality tracking.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
