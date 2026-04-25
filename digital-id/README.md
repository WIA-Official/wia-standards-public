# WIA-SOC-002: Digital ID Standard

**전 세계 어디서든 인정되는 디지털 신원 표준**
*Globally recognized digital identity standard*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-SOC-002 is a comprehensive open standard for digital identity, defining self-sovereign identity (SSI) frameworks, verifiable credentials, decentralized identifiers (DIDs), and zero-knowledge proof protocols. This standard enables privacy-preserving, secure, and interoperable digital identity across global ecosystems.

### Key Features

- **Self-Sovereign Identity**: Users own and control their digital identities
- **Decentralized Identifiers (DIDs)**: Blockchain-anchored, globally unique identifiers
- **Verifiable Credentials**: W3C-compliant credential format
- **Zero-Knowledge Proofs**: Prove claims without revealing data
- **Blockchain Integration**: Ethereum, Polygon, Solana support
- **Privacy-First Design**: Selective disclosure, minimal data exposure
- **Interoperability**: Compatible with existing SSI frameworks
- **Open Source**: MIT licensed reference implementations

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | DID documents, verifiable credentials format |
| 2 | [PHASE-2-API.md](spec/PHASE-2-API.md) | RESTful API and SDK specifications |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Verification protocols and cryptographic methods |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | Blockchain and ecosystem integration |

---

## Quick Start

### TypeScript SDK

```bash
cd api/typescript

# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test
```

### Example Usage

```typescript
import { WiaDigitalID, VerifiableCredential } from 'wia-soc-002';

// Create a new DID
const did = await WiaDigitalID.create({
  method: 'wia',
  blockchain: 'ethereum'
});

console.log(`DID: ${did.id}`);
console.log(`DID Document: ${JSON.stringify(did.document)}`);

// Issue a verifiable credential
const credential = await VerifiableCredential.issue({
  issuer: did.id,
  subject: 'did:wia:0x123...',
  type: ['VerifiableCredential', 'IdentityCredential'],
  claims: {
    name: 'John Doe',
    dateOfBirth: '1990-01-01'
  }
});

// Verify a credential
const isValid = await credential.verify({
  method: 'signature',
  checkRevocation: true
});

console.log(`Credential valid: ${isValid}`);

// Create zero-knowledge proof
const proof = await credential.createZKProof({
  reveal: ['type'], // Only reveal credential type
  hide: ['name', 'dateOfBirth'] // Hide personal data
});

// Verify ZK proof
const proofValid = await proof.verify();
console.log(`ZK Proof valid: ${proofValid}`);
```

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/did/create` | Create new DID |
| GET | `/did/{did}` | Resolve DID document |
| PUT | `/did/{did}` | Update DID document |
| DELETE | `/did/{did}` | Deactivate DID |
| POST | `/credentials/issue` | Issue verifiable credential |
| POST | `/credentials/verify` | Verify credential |
| GET | `/credentials/{id}` | Get credential |
| POST | `/credentials/revoke` | Revoke credential |
| POST | `/proofs/create` | Create ZK proof |
| POST | `/proofs/verify` | Verify ZK proof |

---

## Interactive Demos

### 🎮 Simulator

Try the full-featured digital identity simulator with 99 language support:

```bash
# Open simulator
cd simulator
open index.html
```

Features:
- 5 interactive tabs (Identity, Credentials, Verification, Privacy, Logs)
- DID creation and management
- Credential issuance and verification
- Zero-knowledge proof demonstration
- Privacy controls
- 99 language dropdown
- Dark theme UI

### 📚 Ebook

Comprehensive documentation in English and Korean:

- **English**: [ebook/en/index.html](ebook/en/index.html)
- **Korean**: [ebook/ko/index.html](ebook/ko/index.html)

8 chapters covering:
1. Introduction to Digital Identity and Self-Sovereign Identity
2. Decentralized Identifiers (DIDs) Architecture
3. Verifiable Credentials Standard (W3C VC)
4. Zero-Knowledge Proofs and Privacy Technologies
5. Blockchain Integration and Anchoring
6. Identity Verification and Trust Frameworks
7. API Implementation and SDK Development
8. Future of Digital Identity and Global Adoption

---

## Technical Specifications

### DID Methods

- **wia**: Primary WIA DID method
- **ethr**: Ethereum-based DIDs
- **polygon**: Polygon network DIDs
- **solana**: Solana blockchain DIDs
- **web**: Web-based DIDs for testing

### Cryptographic Algorithms

- **Signing**: EdDSA (Ed25519), ECDSA (secp256k1)
- **Hashing**: SHA-256, SHA-3, Blake2b
- **Encryption**: AES-256-GCM, ChaCha20-Poly1305
- **ZK Proofs**: zk-SNARKs, zk-STARKs, Bulletproofs

### Blockchain Networks

- Ethereum Mainnet / Testnets
- Polygon PoS
- Solana Mainnet
- Binance Smart Chain
- Avalanche C-Chain

---

## Verifiable Credentials

### Credential Types

- **Identity Credentials**: Basic identity information
- **Educational Credentials**: Degrees, certificates, transcripts
- **Professional Credentials**: Licenses, certifications
- **Financial Credentials**: Credit scores, bank verification
- **Health Credentials**: Vaccination records, medical licenses
- **Government Credentials**: Passport, driver's license, voter ID

### Credential Lifecycle

1. **Issuance**: Issuer creates and signs credential
2. **Storage**: Holder stores in digital wallet
3. **Presentation**: Holder presents to verifier
4. **Verification**: Verifier checks signature and validity
5. **Revocation**: Issuer can revoke if needed

---

## Privacy Features

### Zero-Knowledge Proofs

Prove statements about credentials without revealing underlying data:

```typescript
// Prove age > 18 without revealing exact birthdate
const ageProof = await credential.createZKProof({
  statement: 'age >= 18',
  reveal: [] // Don't reveal any data
});

// Verifier checks proof
const isOver18 = await ageProof.verify();
```

### Selective Disclosure

Choose exactly what information to share:

```typescript
const presentation = await credential.present({
  reveal: ['name', 'country'], // Only share name and country
  hide: ['address', 'ssn', 'dateOfBirth'] // Hide sensitive data
});
```

---

## Security & Compliance

### Standards Compliance

- **W3C DID Core**: Full compliance with W3C DID specification
- **W3C Verifiable Credentials**: W3C VC Data Model v1.1
- **GDPR**: Right to erasure, data portability
- **eIDAS**: EU electronic identification regulation
- **NIST**: NIST 800-63-3 digital identity guidelines

### Security Features

- Multi-signature support
- Hardware wallet integration
- Biometric authentication
- Time-locked credentials
- Revocation lists (on-chain and off-chain)
- DID rotation and recovery

---

## Development

### Prerequisites

- Node.js 18+
- TypeScript 5+
- Web3 provider (MetaMask, WalletConnect)
- Modern browser for simulator

### Building from Source

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/digital-id

# Install dependencies
cd api/typescript
npm install

# Build
npm run build

# Test
npm test

# Lint
npm run lint
```

### Running the Simulator

```bash
# No build required - pure HTML/CSS/JS
cd simulator
python3 -m http.server 8000
# Open http://localhost:8000
```

---

## Use Cases

### Government & Public Services

- Digital passports and national IDs
- Voter registration and verification
- Social benefits distribution
- Tax filing and verification

### Education

- University degrees and diplomas
- Professional certifications
- Skill badges and micro-credentials
- Transcript sharing

### Healthcare

- Vaccination certificates
- Medical licenses and credentials
- Health insurance verification
- Prescription management

### Financial Services

- KYC/AML compliance
- Credit scoring
- Bank account verification
- Cross-border payments

### Enterprise

- Employee identity management
- Access control and permissions
- Vendor verification
- Supply chain credentials

---

## Integration Examples

### With Web3 Wallets

```typescript
import { ethers } from 'ethers';
import { WiaDigitalID } from 'wia-soc-002';

// Connect to MetaMask
const provider = new ethers.providers.Web3Provider(window.ethereum);
await provider.send("eth_requestAccounts", []);
const signer = provider.getSigner();

// Create DID with Web3 wallet
const did = await WiaDigitalID.createWithWallet({
  signer,
  network: 'ethereum'
});
```

### With IPFS

```typescript
import { create } from 'ipfs-http-client';

const ipfs = create({ url: 'https://ipfs.infura.io:5001' });

// Store credential on IPFS
const { cid } = await ipfs.add(JSON.stringify(credential));
console.log(`Credential stored at: ipfs://${cid}`);
```

---

## Certification

Products implementing WIA-SOC-002 can apply for official certification:

1. **Basic Level**: DID creation and resolution
2. **Standard Level**: Full credential lifecycle
3. **Advanced Level**: ZK proofs and privacy features

Certification includes:
- Automated compliance testing
- Security audit
- Interoperability verification
- Privacy assessment
- Official WIA certification badge

Apply at: https://cert.wiastandards.com

---

## Roadmap

### Version 1.1 (Q2 2026)
- Selective disclosure v2.0
- Advanced ZK proof circuits
- Multi-chain DID resolution
- Improved wallet integration

### Version 2.0 (2027)
- Quantum-resistant cryptography
- AI-powered identity verification
- Decentralized reputation systems
- Cross-chain credential portability

---

## Support

- **Documentation**: https://wiastandards.com/soc-002
- **Forum**: https://forum.wiastandards.com
- **GitHub Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Email**: support@wiastandards.com

---

## License

MIT License - See [LICENSE](../LICENSE) for details

© 2025 WIA / SmileStory Inc.

---

## Philosophy

홍익인간 (弘益人間) (홍익인간) - Hongik Ingan - Benefit All Humanity

We believe digital identity should:
- Empower individuals with self-sovereignty
- Protect privacy as a fundamental right
- Enable global interoperability
- Be accessible to everyone
- Remain open and transparent

---

## Acknowledgments

Thanks to the global community of identity experts, cryptographers, and advocates who contributed to this standard. Special recognition to:

- W3C DID Working Group
- W3C Verifiable Credentials Working Group
- Decentralized Identity Foundation (DIF)
- Sovrin Foundation
- Ethereum Name Service (ENS)
- Open source ZK proof libraries

---

**For the latest updates, visit:** https://wiastandards.com/soc-002

홍익인간 (弘益人間) - Benefit All Humanity
