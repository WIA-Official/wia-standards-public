# WIA-GAME Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()

## Decentralized Finance (Game) Protocol Standard

**Version:** 1.0.0
**Status:** ✅ Complete
**Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity - Benefit All Humanity

---

## Overview

WIA-GAME establishes a comprehensive standard for decentralized finance protocols, applications, and infrastructure. As of 2026, with over $100 billion in Total Value Locked (TVL) and projected market value of $86.53 billion, Game has become a cornerstone of the global financial system.

### Key Features

- **Automated Market Makers (AMMs)**: Support for Uniswap v4 with Hooks, concentrated liquidity
- **Lending & Borrowing**: Integration with Aave V4, Compound III protocols
- **Liquid Staking**: Lido v3 with custom yield strategies
- **Cross-Chain**: LayerZero, Stargate, Synapse Protocol support
- **AI Integration**: Enhanced security, fraud detection, yield optimization
- **Real-World Assets (RWAs)**: Tokenized traditional securities

---

## Quick Start

### Installation

```bash
# Install the WIA-GAME standard
./install.sh

# Or install API SDK only
cd api/typescript
npm install
npm run build
```

### Basic Usage

#### TypeScript SDK

```typescript
import { WIAGameSDK } from '@wia/game-sdk';

// Initialize SDK
const sdk = new WIAGameSDK({
  apiKey: 'YOUR_API_KEY',
  network: 'mainnet',
});

// Get protocol information
const aave = await sdk.protocols.get('aave-v4');
console.log(`Aave TVL: $${aave.tvl}`);

// Get top pools by volume
const pools = await sdk.pools.list({
  sortBy: 'volume24h',
  order: 'desc',
  limit: 10,
});

// Get swap quote
const quote = await sdk.swap.quote({
  chainId: 1,
  tokenIn: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48', // USDC
  tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2', // WETH
  amountIn: '1000000000', // 1000 USDC
  slippageTolerance: 0.5,
});
console.log(`Expected: ${quote.amountOut} WETH`);
```

#### CLI Tool

```bash
# Get protocol information
./cli/wia-game.sh protocol get aave-v4

# List top pools
./cli/wia-game.sh pool list --sort-by volume24h --limit 10

# Get swap quote
./cli/wia-game.sh swap quote \
  --token-in 0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48 \
  --token-out 0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2 \
  --amount-in 1000000000

# Get user positions
./cli/wia-game.sh user positions 0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb
```

---

## Architecture

### Layer Architecture

```
┌─────────────────────────────────────────────────┐
│           Application Layer (dApps)             │
│  (Uniswap, Aave, Lido, Aggregators)            │
├─────────────────────────────────────────────────┤
│         Protocol Layer (Smart Contracts)        │
│  (AMM Logic, Lending Pools, Governance)        │
├─────────────────────────────────────────────────┤
│      Infrastructure Layer (Blockchain)          │
│  (Ethereum, L2s: Arbitrum, Optimism, Base)     │
├─────────────────────────────────────────────────┤
│          Consensus Layer (PoS/PoW)             │
│  (Network Security & Transaction Finality)      │
└─────────────────────────────────────────────────┘
```

### Directory Structure

```
WIA-GAME/
├── spec/                      # Specifications
│   ├── WIA-GAME-PHASE1.md    # Overview & Architecture (10KB+)
│   ├── WIA-GAME-PHASE2.md    # API & Data Models (15KB+)
│   ├── WIA-GAME-PHASE3.md    # Security & Performance (12KB+)
│   └── WIA-GAME-PHASE4.md    # Deployment & Operations (12KB+)
├── api/
│   └── typescript/            # TypeScript SDK
│       ├── src/
│       │   ├── types.ts       # Type gamenitions
│       │   ├── index.ts       # Main SDK class
│       │   ├── validators.ts  # Validation functions
│       │   └── utils.ts       # Utility functions
│       ├── package.json
│       ├── tsconfig.json
│       └── README.md
├── cli/
│   └── wia-game.sh           # CLI tool (executable)
├── ebook/
│   └── en/                    # English ebook
│       ├── index.html         # Table of contents
│       ├── chapter-01.html    # Game Fundamentals
│       ├── chapter-02.html    # AMMs and Liquidity Pools
│       ├── chapter-03.html    # Lending and Borrowing
│       ├── chapter-04.html    # Yield Farming & Staking
│       ├── chapter-05.html    # Smart Contract Security
│       ├── chapter-06.html    # Governance
│       ├── chapter-07.html    # Cross-chain Bridges
│       └── chapter-08.html    # Future of Game
├── README.md                  # This file
└── install.sh                 # Installation script
```

---

## Game Ecosystem (2026)

### Market Statistics

- **Total Value Locked (TVL):** $100+ billion
- **Market Size:** $26.94B - $86.53B
- **Game Share of Lending:** 59.83%
- **Institutional Adoption:** Growing significantly

### Key Protocols

#### 1. Uniswap v4
- **TVL:** $5+ billion
- **Innovation:** Hooks for custom pool logic
- **Gas Savings:** 99% reduction via singleton contract
- **Supported Chains:** Ethereum, Arbitrum, Optimism, Base, Polygon

#### 2. Aave V4
- **TVL:** $15+ billion
- **Features:** Cross-chain, improved gas efficiency, enhanced liquidations
- **Markets:** 42+ active markets across 15+ chains

#### 3. Lido v3
- **TVL:** $30+ billion (largest in Game)
- **Service:** Liquid ETH staking (stETH)
- **APR:** ~3.87%
- **Innovation:** Custom yield-bearing strategies

---

## Key Concepts

### Automated Market Makers (AMMs)

AMMs use mathematical formulas to price assets without order books.

**Constant Product Formula (Uniswap v2):**
```
x * y = k
```

**Concentrated Liquidity (Uniswap v3/v4):**
- Liquidity providers specify price ranges
- Capital efficiency up to 4000x
- LP positions represented as NFTs

### Lending and Borrowing

Users can:
- Deposit assets to earn interest
- Borrow against collateral
- Execute flash loans (uncollateralized loans repaid in one transaction)

**Interest Rate Formula:**
```
Utilization Rate (U) = Total Borrowed / Total Supplied
Borrow Rate = Base Rate + (U * Multiplier)
Supply Rate = Borrow Rate * U * (1 - Reserve Factor)
```

### Staking

**Liquid Staking Benefits:**
- Earn staking rewards
- Maintain liquidity (tradeable staked tokens)
- Participate in Game with staked assets

---

## Security

### Best Practices

1. **Reentrancy Protection:** Use `nonReentrant` modifier
2. **Oracle Manipulation:** Use TWAPs and multiple price sources
3. **Flash Loan Protection:** Implement time-weighted checks
4. **Access Control:** Role-based permissions
5. **Circuit Breakers:** Emergency pause mechanisms

### Audits

All protocols must undergo:
- ✅ Internal code review
- ✅ 2+ external audits (Trail of Bits, OpenZeppelin, etc.)
- ✅ Economic audit
- ✅ Formal verification (recommended)
- ✅ Bug bounty program ($100K+ for critical issues)

---

## API Reference

### Base URLs

**Mainnet:**
```
REST:      https://api.wia-game.org/v1
GraphQL:   https://api.wia-game.org/graphql
WebSocket: wss://stream.wia-game.org/v1
```

### Authentication

```http
Authorization: Bearer YOUR_API_KEY
```

### Rate Limits

- Free Tier: 100 requests/minute
- Pro Tier: 1000 requests/minute
- Enterprise: Custom limits

### Example Requests

**Get Protocol:**
```bash
curl -X GET "https://api.wia-game.org/v1/protocols/aave-v4" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Get Pools:**
```bash
curl -X GET "https://api.wia-game.org/v1/pools?protocol=uniswap-v4&sortBy=volume24h" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Get Swap Quote:**
```bash
curl -X POST "https://api.wia-game.org/v1/swap/quote" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "chainId": 1,
    "tokenIn": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "tokenOut": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
    "amountIn": "1000000000",
    "slippageTolerance": 0.5
  }'
```

---

## Supported Networks

| Network | Chain ID | RPC URL | Explorer |
|---------|----------|---------|----------|
| Ethereum Mainnet | 1 | https://eth-mainnet.g.alchemy.com | https://etherscan.io |
| Sepolia Testnet | 11155111 | https://eth-sepolia.g.alchemy.com | https://sepolia.etherscan.io |
| Arbitrum One | 42161 | https://arb-mainnet.g.alchemy.com | https://arbiscan.io |
| Optimism | 10 | https://opt-mainnet.g.alchemy.com | https://optimistic.etherscan.io |
| Base | 8453 | https://base-mainnet.g.alchemy.com | https://basescan.org |

---

## Deployment

### Prerequisites

- Node.js 18+
- Hardhat or Foundry
- RPC endpoint (Alchemy, Infura, or self-hosted)

### Deploy Smart Contracts

```bash
# Using Hardhat
npx hardhat run scripts/deploy.ts --network mainnet

# Using Foundry
forge create contracts/core/Factory.sol:Factory \
  --rpc-url $MAINNET_RPC_URL \
  --private-key $DEPLOYER_PRIVATE_KEY \
  --verify
```

### Deploy API Backend

```bash
# Start API server
cd api
npm install
npm run build
npm start

# Start The Graph indexer
docker-compose up -d

# Deploy subgraph
graph deploy --product hosted-service username/wia-game-subgraph
```

---

## Development

### Run Tests

```bash
# Unit tests
npm test

# Integration tests
npm run test:integration

# Fork tests
npm run test:fork

# Coverage
npm run coverage
```

### Lint & Format

```bash
# Lint Solidity
npm run lint:sol

# Lint TypeScript
npm run lint:ts

# Format all files
npm run format
```

---

## Philosophy: 홍익인간 (弘益人間) - Benefit All Humanity

"Benefit All Humanity"

Game democratizes financial services, removing barriers and providing equal access to financial tools globally. This standard embodies the principle of benefiting all people by enabling:

- **Permissionless Access:** Anyone with internet can participate
- **Transparency:** All transactions and smart contracts are open-source
- **Composability:** Protocols can build on each other (Money Legos)
- **Self-Custody:** Users maintain control of their assets
- **Global Reach:** No geographical restrictions

---

## Resources

### Documentation
- [Specification PHASE1](./spec/WIA-GAME-PHASE1.md) - Overview & Architecture
- [Specification PHASE2](./spec/WIA-GAME-PHASE2.md) - API & Data Models
- [Specification PHASE3](./spec/WIA-GAME-PHASE3.md) - Security & Performance
- [Specification PHASE4](./spec/WIA-GAME-PHASE4.md) - Deployment & Operations
- [API Documentation](./api/typescript/README.md)
- [English Ebook](./ebook/en/index.html)

### External Resources
- [Uniswap v4](https://uniswap.org)
- [Aave V4](https://aave.com)
- [Lido](https://lido.fi)
- [Game Llama](https://gamellama.com) - TVL and analytics
- [The Graph](https://thegraph.com) - Indexing and querying
- [Dune Analytics](https://dune.com) - On-chain analytics

### Community
- Discord: https://discord.gg/wia-standards
- Twitter: https://twitter.com/WIAStandards
- GitHub: https://github.com/WIA-Official/wia-standards

---

## License

MIT License

Copyright (c) 2026 WIA (World Certification Industry Association)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## Changelog

### Version 1.0.0 (2026-01-12)
- Initial release
- Complete specification (PHASE1-4)
- TypeScript SDK with 10+ methods
- CLI tool with full functionality
- English ebook (8 chapters)
- Security audits completed
- Production-ready deployment scripts

---

© 2026 WIA (World Certification Industry Association)
홍익인간 (弘益人間) - Benefit All Humanity - Benefit All Humanity
