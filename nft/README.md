# WIA-FIN-009: NFT (Non-Fungible Token) Standard 🖼️

> **대체 불가능 토큰 표준**
> World Certification Industry Association (WIA) Official Standard

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Standard: WIA-FIN-009](https://img.shields.io/badge/Standard-WIA--FIN--009-22c55e)](https://wia.org/standards/fin-009)
[![Version: 1.0.0](https://img.shields.io/badge/Version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)

## 📋 Table of Contents

- [Overview](#overview)
- [Philosophy](#philosophy)
- [Features](#features)
- [Quick Start](#quick-start)
- [Directory Structure](#directory-structure)
- [Implementation Phases](#implementation-phases)
- [Standards & Specifications](#standards--specifications)
- [API & SDK](#api--sdk)
- [Interactive Tools](#interactive-tools)
- [Documentation](#documentation)
- [Use Cases](#use-cases)
- [Security](#security)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Overview

WIA-FIN-009 is a comprehensive standard for Non-Fungible Tokens (NFTs), providing complete specifications, APIs, tools, and documentation for implementing NFT systems. This standard covers everything from token formats and metadata schemas to marketplace integration and enterprise applications.

### What are NFTs?

Non-Fungible Tokens (NFTs) are unique digital assets stored on a blockchain that represent ownership and authenticity of both digital and physical items. Unlike cryptocurrencies like Bitcoin or Ethereum, which are fungible (interchangeable), each NFT is unique and cannot be directly exchanged on a one-to-one basis.

### Why WIA-FIN-009?

- **Comprehensive Coverage**: From basic concepts to enterprise integration
- **Practical Implementation**: Working code examples and SDKs
- **Interoperable**: Compatible with major marketplaces and platforms
- **Future-Proof**: Designed for extensibility and evolution
- **Community-Driven**: Open standard with transparent development

## Philosophy

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

This standard is built on the principle of creating technology that benefits humanity. NFTs should:
- Empower creators and artists
- Enable true digital ownership
- Foster transparent and fair economies
- Be accessible to all, not just technical experts
- Respect environmental sustainability

## Features

### 🎨 Core NFT Standards

- **ERC-721 Support**: Standard non-fungible token implementation
- **ERC-1155 Support**: Multi-token standard for efficiency
- **ERC-2981 Royalties**: Programmable creator royalties
- **Metadata Standards**: OpenSea-compatible metadata schemas
- **IPFS Integration**: Decentralized storage for permanence

### 🔧 Developer Tools

- **TypeScript SDK**: Full-featured SDK for NFT operations
- **REST API**: Complete API for minting, transferring, querying
- **Smart Contracts**: Audited, gas-optimized contract templates
- **Testing Suite**: Comprehensive test coverage
- **CLI Tools**: Command-line interface for common operations

### 🎯 Interactive Simulators

- **Metadata Generator**: Create ERC-721/ERC-1155 metadata
- **Minting Simulator**: Test NFT minting workflows
- **Marketplace Simulator**: Simulate listing and trading
- **Royalty Calculator**: Calculate creator earnings
- **Collection Manager**: Manage and verify collections

### 📚 Complete Documentation

- **English eBook**: 8 comprehensive chapters (300+ pages)
- **Korean eBook**: Full Korean translation
- **Specification Docs**: 4-phase technical specifications
- **API Reference**: Complete API documentation
- **Best Practices**: Industry-standard guidelines

## Quick Start

### Installation

```bash
npm install @wia/nft-sdk
```

### Basic Usage

```typescript
import { NFTClient } from '@wia/nft-sdk';

// Initialize client
const client = new NFTClient({
  apiKey: 'your-api-key',
  network: 'mainnet'
});

// Mint an NFT
const result = await client.mint({
  recipient: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
  metadata: {
    name: 'My First NFT',
    description: 'A unique digital collectible',
    image: 'ipfs://QmHash...',
    attributes: [
      { trait_type: 'Rarity', value: 'Legendary' }
    ]
  },
  royaltyPercentage: 10
});

console.log('Token ID:', result.tokenId);

// Get NFT data
const nft = await client.getNFT(contractAddress, tokenId);
console.log('Owner:', nft.owner);
console.log('Metadata:', nft.metadata);
```

### Smart Contract Example

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";
import "@openzeppelin/contracts/token/ERC721/extensions/ERC721URIStorage.sol";
import "@openzeppelin/contracts/token/common/ERC2981.sol";
import "@openzeppelin/contracts/access/Ownable.sol";

contract MyNFT is ERC721, ERC721URIStorage, ERC2981, Ownable {
    uint256 private _tokenIdCounter;
    uint256 public constant MAX_SUPPLY = 10000;
    uint256 public constant MINT_PRICE = 0.08 ether;

    constructor() ERC721("MyNFT", "MNFT") {
        _setDefaultRoyalty(msg.sender, 750); // 7.5% royalty
    }

    function mint(string memory uri) public payable returns (uint256) {
        require(_tokenIdCounter < MAX_SUPPLY, "Max supply reached");
        require(msg.value >= MINT_PRICE, "Insufficient payment");

        uint256 tokenId = _tokenIdCounter++;
        _safeMint(msg.sender, tokenId);
        _setTokenURI(tokenId, uri);

        return tokenId;
    }

    // Required overrides
    function tokenURI(uint256 tokenId)
        public view override(ERC721, ERC721URIStorage)
        returns (string memory)
    {
        return super.tokenURI(tokenId);
    }

    function supportsInterface(bytes4 interfaceId)
        public view override(ERC721, ERC721URIStorage, ERC2981)
        returns (bool)
    {
        return super.supportsInterface(interfaceId);
    }

    function _burn(uint256 tokenId)
        internal override(ERC721, ERC721URIStorage)
    {
        super._burn(tokenId);
    }
}
```

## Directory Structure

```
nft/
├── index.html              # Landing page with animations
├── simulator/              # Interactive NFT tools
│   └── index.html         # 5-tab simulator
├── ebook/                 # Complete documentation
│   ├── en/               # English chapters (9 files)
│   └── ko/               # Korean chapters (9 files)
├── spec/                 # Technical specifications
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/                  # SDK implementations
│   └── typescript/       # TypeScript SDK
│       ├── package.json
│       └── src/
│           ├── types.ts
│           └── index.ts
└── README.md            # This file
```

## Implementation Phases

### Phase 1: Data Format

Establishes foundational data structures and metadata schemas.

**Key Components:**
- ERC-721 token format specifications
- ERC-1155 multi-token standard
- Metadata schema (JSON format)
- IPFS integration guidelines
- Attribute and trait standards

**Deliverables:**
- Metadata schema documentation
- Token standard specifications
- URI format guidelines
- Validation rules

[📖 Read Phase 1 Specification](spec/PHASE-1-DATA-FORMAT.md)

### Phase 2: API

Defines RESTful APIs and SDKs for NFT operations.

**Key Components:**
- Minting API endpoints
- Transfer and query APIs
- Marketplace listing APIs
- Royalty calculation APIs
- Analytics and statistics APIs

**Deliverables:**
- REST API documentation
- TypeScript SDK
- Python SDK (planned)
- API client libraries

[📖 Read Phase 2 Specification](spec/PHASE-2-API.md)

### Phase 3: Protocol

Specifies blockchain protocols and smart contract standards.

**Key Components:**
- Smart contract architecture
- Security best practices
- Gas optimization strategies
- Royalty implementation (ERC-2981)
- Cross-chain protocols

**Deliverables:**
- Smart contract templates
- Security audit guidelines
- Testing frameworks
- Deployment scripts

[📖 Read Phase 3 Specification](spec/PHASE-3-PROTOCOL.md)

### Phase 4: Integration

Covers ecosystem integration with marketplaces and platforms.

**Key Components:**
- OpenSea integration
- Wallet connectivity (MetaMask, WalletConnect)
- Metaverse platform integration
- Gaming engine SDKs (Unity, Unreal)
- Enterprise system integration

**Deliverables:**
- Integration guides
- Platform-specific SDKs
- Example implementations
- Best practices documentation

[📖 Read Phase 4 Specification](spec/PHASE-4-INTEGRATION.md)

## Standards & Specifications

### Token Standards

#### ERC-721: Non-Fungible Token Standard

The original NFT standard for unique, indivisible tokens.

**Use Cases:**
- Digital art (1/1 pieces)
- Virtual real estate
- Domain names
- Unique collectibles
- Digital identity tokens

**Characteristics:**
- Each token has unique ID
- Single owner per token
- Indivisible
- Full transfer history on-chain

#### ERC-1155: Multi-Token Standard

Efficient standard for managing multiple token types in single contract.

**Use Cases:**
- Gaming items (mix of unique and fungible)
- Large collections (10,000+ items)
- Fractional ownership scenarios
- Complex ecosystems

**Advantages:**
- Gas-efficient batch operations
- Single contract for multiple types
- Supports both fungible and non-fungible
- Reduced deployment costs

#### ERC-2981: NFT Royalty Standard

Standard interface for royalty payments on secondary sales.

**Features:**
- Marketplace-agnostic royalty info
- Programmable royalty percentages
- Per-token or collection-wide royalties
- Automatic royalty calculations

**Implementation:**
```solidity
function royaltyInfo(uint256 tokenId, uint256 salePrice)
    external view returns (address receiver, uint256 royaltyAmount);
```

### Metadata Standards

Complete metadata schema following OpenSea standard:

```json
{
  "name": "Token Name",
  "description": "Detailed description supporting markdown",
  "image": "ipfs://QmHash.../image.png",
  "external_url": "https://project.com/token/1",
  "attributes": [
    {
      "trait_type": "Background",
      "value": "Blue"
    },
    {
      "trait_type": "Power",
      "value": 95,
      "max_value": 100,
      "display_type": "boost_percentage"
    }
  ],
  "animation_url": "ipfs://QmHash.../animation.mp4",
  "background_color": "0066FF"
}
```

### IPFS Integration

**Why IPFS?**
- Decentralized storage
- Content-addressed (immutable)
- Censorship-resistant
- No single point of failure

**Best Practices:**
- Use IPFS for metadata and assets
- Pin content on multiple services
- Use CIDv1 for compatibility
- Implement backup strategies

**Recommended Services:**
- Pinata
- NFT.Storage
- Infura IPFS
- Web3.Storage

## API & SDK

### TypeScript SDK

Full-featured SDK for NFT operations:

```typescript
import { NFTClient, createNFTClient } from '@wia/nft-sdk';

const client = createNFTClient({
  apiKey: 'your-api-key',
  network: 'mainnet',
  ipfsGateway: 'https://ipfs.io/ipfs'
});

// Mint NFT
const mintResult = await client.mint({
  recipient: '0x...',
  metadata: { name: '...', image: '...' },
  royaltyPercentage: 10
});

// Transfer NFT
await client.transfer({
  from: '0x...',
  to: '0x...',
  tokenId: '1',
  contractAddress: '0x...'
});

// Get collection stats
const stats = await client.getCollectionStats('0x...');
console.log('Floor price:', stats.floorPrice);

// Upload to IPFS
const ipfsResult = await client.uploadToIPFS({
  name: 'My NFT',
  image: 'base64...',
  attributes: []
});
```

### REST API

Complete REST API for all NFT operations:

```bash
# Get NFT metadata
GET https://api.wia.org/v1/nft/{contractAddress}/{tokenId}

# Mint NFT
POST https://api.wia.org/v1/nft/mint
{
  "recipient": "0x...",
  "metadata": {...},
  "royaltyPercentage": 10
}

# Get collection stats
GET https://api.wia.org/v1/analytics/collection/{contractAddress}

# Upload to IPFS
POST https://api.wia.org/v1/ipfs/upload
Content-Type: multipart/form-data
```

## Interactive Tools

### NFT Simulator

Comprehensive 5-tab simulator for testing NFT operations:

1. **Metadata Generator**
   - Create ERC-721 or ERC-1155 metadata
   - Add custom attributes
   - Preview JSON output
   - Validate against standards

2. **Minting Simulator**
   - Simulate minting process
   - Calculate gas costs
   - Track minted NFTs
   - Test different scenarios

3. **Marketplace Simulator**
   - List NFTs for sale
   - Simulate buy/sell transactions
   - View active listings
   - Test marketplace flows

4. **Royalty Calculator**
   - Calculate creator royalties
   - Project multi-sale earnings
   - Compare marketplace fees
   - Optimize royalty percentages

5. **Collection Manager**
   - Create collections
   - Verify authenticity
   - View collection stats
   - Manage multiple collections

[🚀 Try the Simulator](simulator/index.html)

## Documentation

### English Documentation

Comprehensive 8-chapter guide covering all aspects of NFTs:

1. **Introduction to NFTs** - Fundamentals and history
2. **Token Standards** - ERC-721 & ERC-1155 deep dive
3. **Metadata & IPFS** - Data structures and storage
4. **Smart Contract Development** - Building secure contracts
5. **Marketplaces & Trading** - Integration and protocols
6. **Royalties & Creator Economics** - Monetization strategies
7. **Use Cases & Applications** - Real-world implementations
8. **Future & Best Practices** - Emerging trends and guidelines

[📖 Read English Documentation](ebook/en/index.html)

### Korean Documentation

Complete Korean translation of all documentation:

[📖 한국어 문서 읽기](ebook/ko/index.html)

## Use Cases

### Digital Art

- One-of-one artwork
- Limited edition collections
- Generative art
- Photography NFTs
- 3D sculptures

### Gaming

- In-game items and weapons
- Character skins
- Virtual real estate
- Achievement badges
- Play-to-earn assets

### Music & Entertainment

- Album releases
- Concert tickets
- Exclusive content
- Royalty-bearing tracks
- Fan engagement tokens

### Enterprise

- Supply chain tracking
- Authentication certificates
- Digital credentials
- Intellectual property
- Fractional ownership

### Identity & Credentials

- Educational diplomas
- Professional certifications
- Event attendance proof
- Membership tokens
- Soulbound tokens (non-transferable)

## Security

### Smart Contract Security

- Use audited libraries (OpenZeppelin)
- Implement reentrancy protection
- Proper access control
- Integer overflow protection
- Emergency pause mechanisms

### Audit Requirements

Recommended before mainnet deployment:
- OpenZeppelin Security Audits
- Trail of Bits
- ConsenSys Diligence
- CertiK
- Quantstamp

### Best Practices

1. Test extensively on testnets
2. Use multi-signature wallets
3. Implement upgrade mechanisms carefully
4. Monitor for suspicious activity
5. Have emergency response plans
6. Document all security measures

## Contributing

We welcome contributions from the community!

### How to Contribute

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

### Development Setup

```bash
git clone https://github.com/WIA-Official/wia-standards
cd wia-standards/nft
npm install
npm test
```

### Code Standards

- Follow TypeScript best practices
- Write comprehensive tests
- Document all public APIs
- Use meaningful commit messages

## License

MIT License - see [LICENSE](../LICENSE) for details.

## Contact

- **Website**: [https://wia.org](https://wia.org)
- **GitHub**: [https://github.com/WIA-Official](https://github.com/WIA-Official)
- **Email**: standards@wia.org
- **Twitter**: [@WIA_Official](https://twitter.com/WIA_Official)

## Acknowledgments

Special thanks to:
- Ethereum Foundation
- OpenZeppelin team
- OpenSea for metadata standards
- IPFS community
- All contributors and supporters

---

**© 2025 SmileStory Inc. / WIA**
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

Built with ❤️ for the NFT community
