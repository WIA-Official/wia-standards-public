# WIA DeFi Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Standard:** WIA-FIN-006
**Status:** Final
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Pool Endpoints](#pool-endpoints)
5. [Swap Endpoints](#swap-endpoints)
6. [Lending Endpoints](#lending-endpoints)
7. [Yield Farming Endpoints](#yield-farming-endpoints)
8. [Flash Loan Endpoints](#flash-loan-endpoints)
9. [Governance Endpoints](#governance-endpoints)
10. [WebSocket Streaming](#websocket-streaming)
11. [Error Handling](#error-handling)
12. [Rate Limiting](#rate-limiting)
13. [SDK Reference](#sdk-reference)

---

## Overview

The WIA DeFi API provides standardized RESTful endpoints for interacting with decentralized finance protocols across multiple chains. This specification enables developers to build applications that seamlessly integrate with AMMs, lending platforms, yield aggregators, and governance systems.

### Base URL

```
https://api.wiastandards.com/defi/v1
```

### Supported Networks

```
GET /networks
```

Response:
```json
{
  "networks": [
    {
      "chainId": 1,
      "name": "Ethereum",
      "rpc": "https://eth.llamarpc.com",
      "explorer": "https://etherscan.io",
      "nativeCurrency": {
        "symbol": "ETH",
        "decimals": 18
      }
    },
    {
      "chainId": 137,
      "name": "Polygon",
      "rpc": "https://polygon.llamarpc.com",
      "explorer": "https://polygonscan.com",
      "nativeCurrency": {
        "symbol": "MATIC",
        "decimals": 18
      }
    }
  ]
}
```

---

## API Architecture

### Design Principles

- **RESTful**: Resource-based URLs with standard HTTP methods
- **JSON**: All requests and responses use JSON format
- **Versioning**: API version in URL path (`/v1/`)
- **Pagination**: Cursor-based pagination for large datasets
- **Filtering**: Query parameters for filtering and sorting
- **CORS**: Cross-Origin Resource Sharing enabled
- **HTTPS**: All endpoints require HTTPS

### Common Headers

```
Content-Type: application/json
X-API-Key: your_api_key_here
X-Chain-Id: 1
X-Request-Id: unique_request_id
```

### Response Format

Success Response:
```json
{
  "success": true,
  "data": {},
  "meta": {
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req_123456"
  }
}
```

Error Response:
```json
{
  "success": false,
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Invalid pool address",
    "details": {}
  },
  "meta": {
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req_123456"
  }
}
```

---

## Authentication

### API Key Authentication

All API requests require an API key in the header:

```
X-API-Key: wia_live_abc123def456...
```

### Obtaining an API Key

```
POST /auth/register
```

Request:
```json
{
  "email": "user@example.com",
  "organization": "My DeFi App",
  "useCase": "Portfolio tracker"
}
```

Response:
```json
{
  "success": true,
  "data": {
    "apiKey": "wia_live_abc123def456...",
    "tier": "free",
    "rateLimit": {
      "requestsPerMinute": 60,
      "requestsPerDay": 10000
    }
  }
}
```

### Rate Limit Tiers

| Tier | Requests/min | Requests/day | WebSocket | Price |
|------|--------------|--------------|-----------|-------|
| Free | 60 | 10,000 | ❌ | $0 |
| Developer | 300 | 100,000 | ✅ | $49/mo |
| Professional | 1,000 | 1,000,000 | ✅ | $199/mo |
| Enterprise | Unlimited | Unlimited | ✅ | Custom |

---

## Pool Endpoints

### List Pools

```
GET /pools
```

Query Parameters:
- `chainId` (number, optional): Filter by chain ID
- `protocol` (string, optional): Filter by protocol (uniswap, aave, etc.)
- `type` (string, optional): Filter by type (AMM, Lending, etc.)
- `minTVL` (number, optional): Minimum TVL in USD
- `limit` (number, optional): Results per page (default: 20, max: 100)
- `cursor` (string, optional): Pagination cursor

Response:
```json
{
  "success": true,
  "data": {
    "pools": [
      {
        "poolId": "uniswap-v3-eth-usdc-005",
        "protocol": "Uniswap V3",
        "chainId": 1,
        "tokens": ["WETH", "USDC"],
        "tvl": 189123456.78,
        "volume24h": 67891234.56,
        "apy": 24.67
      }
    ],
    "pagination": {
      "hasMore": true,
      "cursor": "pool_abc123"
    }
  }
}
```

### Get Pool Details

```
GET /pools/:poolId
```

Parameters:
- `poolId`: Pool identifier

Response:
```json
{
  "success": true,
  "data": {
    "pool": {
      "version": "1.0.0",
      "standard": "WIA-FIN-006",
      "poolId": "uniswap-v3-eth-usdc-005",
      "protocol": "Uniswap V3",
      "contractAddress": "0x88e6A0c2dDD26FEEb64F039a2c41296FcB3f5640",
      "tokens": [...],
      "metrics": {...}
    }
  }
}
```

### Get Pool Historical Data

```
GET /pools/:poolId/history
```

Query Parameters:
- `interval`: Time interval (`1h`, `4h`, `1d`, `1w`)
- `from`: Start timestamp (ISO-8601)
- `to`: End timestamp (ISO-8601)

Response:
```json
{
  "success": true,
  "data": {
    "history": [
      {
        "timestamp": "2025-12-25T00:00:00Z",
        "tvl": 189123456.78,
        "volume": 67891234.56,
        "apy": 24.67,
        "price": 2734.56
      }
    ]
  }
}
```

---

## Swap Endpoints

### Get Quote

```
POST /swap/quote
```

Request:
```json
{
  "chainId": 1,
  "tokenIn": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
  "tokenOut": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "amountIn": "1000000000000000000",
  "slippageTolerance": 0.01,
  "protocols": ["uniswap_v2", "uniswap_v3", "sushiswap"]
}
```

Response:
```json
{
  "success": true,
  "data": {
    "quote": {
      "amountIn": "1.0",
      "amountOut": "2728.45",
      "amountOutMin": "2701.17",
      "priceImpact": 0.23,
      "route": [
        {
          "protocol": "Uniswap V3",
          "poolAddress": "0x88e6A0c2dDD26FEEb64F039a2c41296FcB3f5640",
          "fee": 500,
          "tokenIn": "WETH",
          "tokenOut": "USDC",
          "amountIn": "1.0",
          "amountOut": "2728.45"
        }
      ],
      "estimatedGas": "150000",
      "gasCostUSD": "15.50"
    }
  }
}
```

### Execute Swap

```
POST /swap/execute
```

Request:
```json
{
  "chainId": 1,
  "quote": {...},
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "deadline": 1735128000
}
```

Response:
```json
{
  "success": true,
  "data": {
    "transaction": {
      "to": "0x...",
      "data": "0x...",
      "value": "1000000000000000000",
      "gasLimit": "150000"
    }
  }
}
```

### Swap History

```
GET /swap/history
```

Query Parameters:
- `address`: User wallet address
- `chainId`: Chain ID
- `limit`: Results per page

Response:
```json
{
  "success": true,
  "data": {
    "swaps": [
      {
        "transactionHash": "0x...",
        "timestamp": "2025-12-25T10:00:00Z",
        "tokenIn": "WETH",
        "tokenOut": "USDC",
        "amountIn": "1.0",
        "amountOut": "2728.45",
        "protocol": "Uniswap V3"
      }
    ]
  }
}
```

---

## Lending Endpoints

### Get Markets

```
GET /lending/markets
```

Query Parameters:
- `chainId`: Chain ID
- `protocol`: Protocol name (aave, compound)

Response:
```json
{
  "success": true,
  "data": {
    "markets": [
      {
        "marketId": "aave-v3-ethereum-main",
        "protocol": "Aave V3",
        "chainId": 1,
        "totalMarketSize": 45678912345.67,
        "totalBorrowed": 23456789123.45,
        "assets": [...]
      }
    ]
  }
}
```

### Get User Position

```
GET /lending/positions/:address
```

Parameters:
- `address`: User wallet address

Query Parameters:
- `chainId`: Chain ID
- `protocol`: Protocol name

Response:
```json
{
  "success": true,
  "data": {
    "position": {
      "user": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
      "protocol": "Aave V3",
      "supplied": [...],
      "borrowed": [...],
      "healthFactor": 1.65,
      "totalCollateralUSD": 274549.2,
      "totalBorrowedUSD": 150000.0
    }
  }
}
```

### Supply Asset

```
POST /lending/supply
```

Request:
```json
{
  "chainId": 1,
  "protocol": "aave_v3",
  "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
  "amount": "1000000000000000000",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

Response:
```json
{
  "success": true,
  "data": {
    "transaction": {
      "to": "0x...",
      "data": "0x...",
      "value": "0",
      "gasLimit": "250000"
    }
  }
}
```

### Borrow Asset

```
POST /lending/borrow
```

Request:
```json
{
  "chainId": 1,
  "protocol": "aave_v3",
  "asset": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "amount": "1000000000",
  "rateMode": "variable",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

### Repay Asset

```
POST /lending/repay
```

Request:
```json
{
  "chainId": 1,
  "protocol": "aave_v3",
  "asset": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "amount": "1000000000",
  "rateMode": "variable",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

---

## Yield Farming Endpoints

### Get Farms

```
GET /farming/farms
```

Query Parameters:
- `chainId`: Chain ID
- `protocol`: Protocol name
- `minAPY`: Minimum APY percentage
- `type`: Farm type (liquidity_mining, single_staking, vault)

Response:
```json
{
  "success": true,
  "data": {
    "farms": [
      {
        "farmId": "sushiswap-eth-usdc-farm",
        "protocol": "SushiSwap",
        "chainId": 1,
        "stakingToken": "SLP",
        "rewardTokens": ["SUSHI"],
        "apy": 123.45,
        "tvl": 2469135.79
      }
    ]
  }
}
```

### Stake Tokens

```
POST /farming/stake
```

Request:
```json
{
  "chainId": 1,
  "farmId": "sushiswap-eth-usdc-farm",
  "amount": "100000000000000000000",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

### Claim Rewards

```
POST /farming/claim
```

Request:
```json
{
  "chainId": 1,
  "farmId": "sushiswap-eth-usdc-farm",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

### Unstake Tokens

```
POST /farming/unstake
```

Request:
```json
{
  "chainId": 1,
  "farmId": "sushiswap-eth-usdc-farm",
  "amount": "100000000000000000000",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
}
```

---

## Flash Loan Endpoints

### Get Flash Loan Providers

```
GET /flashloan/providers
```

Response:
```json
{
  "success": true,
  "data": {
    "providers": [
      {
        "protocol": "Aave V3",
        "chainId": 1,
        "feeRate": 0.0009,
        "availableAssets": [
          {
            "asset": "WETH",
            "available": "123456.789"
          }
        ]
      }
    ]
  }
}
```

### Simulate Flash Loan

```
POST /flashloan/simulate
```

Request:
```json
{
  "chainId": 1,
  "protocol": "aave_v3",
  "loans": [
    {
      "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "amount": "100000000000000000000"
    }
  ],
  "operations": [...]
}
```

Response:
```json
{
  "success": true,
  "data": {
    "simulation": {
      "success": true,
      "profit": "5.67",
      "profitUSD": 15487.92,
      "gasEstimate": "345678",
      "gasCostUSD": "45.67"
    }
  }
}
```

---

## Governance Endpoints

### Get Proposals

```
GET /governance/proposals
```

Query Parameters:
- `protocol`: Protocol name (uniswap, aave, compound)
- `status`: Proposal status (active, pending, executed)

Response:
```json
{
  "success": true,
  "data": {
    "proposals": [
      {
        "proposalId": "123",
        "title": "Reduce protocol fee to 0.05%",
        "status": "Active",
        "votesFor": "45678901.234567",
        "votesAgainst": "12345678.901234",
        "endTime": "2025-12-27T00:00:00Z"
      }
    ]
  }
}
```

### Get Proposal Details

```
GET /governance/proposals/:proposalId
```

### Cast Vote

```
POST /governance/vote
```

Request:
```json
{
  "proposalId": "123",
  "support": "for",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "reason": "I support this proposal because..."
}
```

---

## WebSocket Streaming

### Connection

```
wss://api.wiastandards.com/defi/v1/stream
```

### Authentication

Send API key immediately after connection:

```json
{
  "type": "auth",
  "apiKey": "wia_live_abc123def456..."
}
```

### Subscribe to Pool Updates

```json
{
  "type": "subscribe",
  "channel": "pool",
  "params": {
    "poolId": "uniswap-v3-eth-usdc-005"
  }
}
```

Streamed Data:
```json
{
  "type": "pool_update",
  "data": {
    "poolId": "uniswap-v3-eth-usdc-005",
    "tvl": 189123456.78,
    "price": 2734.56,
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

### Subscribe to Swap Events

```json
{
  "type": "subscribe",
  "channel": "swaps",
  "params": {
    "poolId": "uniswap-v3-eth-usdc-005"
  }
}
```

### Subscribe to Liquidations

```json
{
  "type": "subscribe",
  "channel": "liquidations",
  "params": {
    "protocol": "aave_v3",
    "chainId": 1
  }
}
```

---

## Error Handling

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_API_KEY` | 401 | Invalid or missing API key |
| `RATE_LIMIT_EXCEEDED` | 429 | Rate limit exceeded |
| `INVALID_PARAMETER` | 400 | Invalid request parameter |
| `POOL_NOT_FOUND` | 404 | Pool not found |
| `INSUFFICIENT_LIQUIDITY` | 400 | Insufficient liquidity for swap |
| `HEALTH_FACTOR_TOO_LOW` | 400 | Health factor below threshold |
| `INTERNAL_ERROR` | 500 | Internal server error |

### Error Response Example

```json
{
  "success": false,
  "error": {
    "code": "INSUFFICIENT_LIQUIDITY",
    "message": "Insufficient liquidity in pool for requested swap amount",
    "details": {
      "poolId": "uniswap-v2-eth-usdc",
      "requestedAmount": "1000000.0",
      "availableLiquidity": "500000.0"
    }
  },
  "meta": {
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req_123456"
  }
}
```

---

## Rate Limiting

### Rate Limit Headers

Every response includes rate limit information:

```
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 287
X-RateLimit-Reset: 1735128000
```

### Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please upgrade your plan or wait.",
    "details": {
      "limit": 60,
      "retryAfter": 45
    }
  }
}
```

---

## SDK Reference

### TypeScript SDK

Install:
```bash
npm install @wia/defi-sdk
```

Usage:
```typescript
import { WIADeFi } from '@wia/defi-sdk';

const defi = new WIADeFi({
  apiKey: 'wia_live_abc123def456...',
  chainId: 1
});

// Get pool
const pool = await defi.pools.get('uniswap-v3-eth-usdc-005');

// Get swap quote
const quote = await defi.swap.getQuote({
  tokenIn: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
  tokenOut: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48',
  amountIn: '1000000000000000000'
});

// Get user position
const position = await defi.lending.getPosition(
  '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb'
);
```

### Python SDK

Install:
```bash
pip install wia-defi-sdk
```

Usage:
```python
from wia_defi import WIADeFi

defi = WIADeFi(api_key='wia_live_abc123def456...', chain_id=1)

# Get pool
pool = defi.pools.get('uniswap-v3-eth-usdc-005')

# Get swap quote
quote = defi.swap.get_quote(
    token_in='0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
    token_out='0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48',
    amount_in='1000000000000000000'
)
```

---

## Appendix

### Supported Protocols

| Protocol | Type | Version | Chains |
|----------|------|---------|--------|
| Uniswap | AMM | V2, V3 | Ethereum, Polygon, Arbitrum, Optimism |
| SushiSwap | AMM | V2 | Multi-chain |
| Curve | Stable AMM | V1, V2 | Ethereum, Polygon, Arbitrum |
| Balancer | Weighted AMM | V2 | Ethereum, Polygon, Arbitrum |
| Aave | Lending | V2, V3 | Multi-chain |
| Compound | Lending | V2, V3 | Ethereum |
| Convex | Yield | - | Ethereum |
| Yearn | Vault | V2, V3 | Multi-chain |

---

**Version:** 1.0.0
**Last Updated:** December 2025
**Status:** Final

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
