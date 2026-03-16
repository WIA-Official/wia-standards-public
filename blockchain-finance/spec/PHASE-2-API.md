# WIA Blockchain Finance Standard
## Phase 2: API Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [REST API Endpoints](#rest-api-endpoints)
3. [Web3 Authentication](#web3-authentication)
4. [Request/Response Format](#requestresponse-format)
5. [Error Codes](#error-codes)
6. [Rate Limiting](#rate-limiting)
7. [WebSocket API](#websocket-api)
8. [SDK Integration](#sdk-integration)

---

## Overview

The WIA Blockchain Finance API provides a standardized RESTful and WebSocket interface for interacting with blockchain networks, smart contracts, and DeFi protocols.

### Base URL

```
Production:  https://api.wia.finance/v1
Testnet:     https://testnet-api.wia.finance/v1
Development: http://localhost:3000/v1
```

### Protocol Support

- **REST API**: HTTP/HTTPS with JSON
- **WebSocket**: Real-time events and subscriptions
- **GraphQL**: Advanced queries (optional)
- **gRPC**: High-performance RPC (optional)

---

## REST API Endpoints

### Authentication

#### POST /auth/connect

Connect wallet and obtain authentication token.

**Request:**
```json
{
  "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "signature": "0x...",
  "message": "Sign this message to authenticate with WIA at 2025-12-25T10:30:00Z",
  "chainId": 1
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "expiresAt": "2025-12-26T10:30:00Z",
    "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "chainId": 1
  }
}
```

#### POST /auth/refresh

Refresh authentication token.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "expiresAt": "2025-12-27T10:30:00Z"
  }
}
```

---

### Blockchain Operations

#### GET /chains

Get supported blockchain networks.

**Response:**
```json
{
  "success": true,
  "data": {
    "chains": [
      {
        "id": 1,
        "name": "Ethereum Mainnet",
        "symbol": "ETH",
        "rpcUrl": "https://eth-mainnet.wia.finance",
        "explorerUrl": "https://etherscan.io",
        "nativeCurrency": {
          "name": "Ether",
          "symbol": "ETH",
          "decimals": 18
        },
        "status": "active"
      },
      {
        "id": 56,
        "name": "BNB Smart Chain",
        "symbol": "BNB",
        "rpcUrl": "https://bsc-mainnet.wia.finance",
        "explorerUrl": "https://bscscan.com",
        "nativeCurrency": {
          "name": "BNB",
          "symbol": "BNB",
          "decimals": 18
        },
        "status": "active"
      }
    ]
  }
}
```

#### GET /chains/:chainId/status

Get blockchain status and metrics.

**Response:**
```json
{
  "success": true,
  "data": {
    "chainId": 1,
    "blockNumber": 18900000,
    "blockTime": 12,
    "gasPrice": "25000000000",
    "baseFee": "20000000000",
    "priorityFee": "2000000000",
    "difficulty": "58750003716598352816469",
    "hashRate": "1000000000000000",
    "timestamp": "2025-12-25T10:30:00Z"
  }
}
```

---

### Account Management

#### GET /accounts/:address

Get account information.

**Parameters:**
- `address`: Ethereum address
- `chainId`: Chain ID (optional, defaults to 1)

**Response:**
```json
{
  "success": true,
  "data": {
    "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "chainId": 1,
    "balance": "5000000000000000000",
    "nonce": 42,
    "tokens": [
      {
        "contract": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
        "symbol": "USDC",
        "name": "USD Coin",
        "decimals": 6,
        "balance": "1000000000"
      }
    ],
    "nfts": [
      {
        "contract": "0xBC4CA0EdA7647A8aB7C2061c2E118A18a936f13D",
        "tokenId": "1337",
        "name": "Bored Ape #1337"
      }
    ]
  }
}
```

#### GET /accounts/:address/transactions

Get account transaction history.

**Parameters:**
- `address`: Ethereum address
- `chainId`: Chain ID (optional)
- `page`: Page number (default: 1)
- `limit`: Results per page (default: 20, max: 100)
- `type`: Transaction type filter (optional)
- `startBlock`: Starting block number (optional)
- `endBlock`: Ending block number (optional)

**Response:**
```json
{
  "success": true,
  "data": {
    "transactions": [
      {
        "hash": "0xabcdef...",
        "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
        "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
        "value": "1000000000000000000",
        "blockNumber": 18899999,
        "timestamp": "2025-12-25T10:00:00Z",
        "status": "success"
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total": 150,
      "pages": 8
    }
  }
}
```

---

### Transaction Management

#### POST /transactions/send

Send a transaction.

**Request:**
```json
{
  "chainId": 1,
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "value": "1000000000000000000",
  "data": "0x",
  "gasLimit": "21000",
  "maxFeePerGas": "30000000000",
  "maxPriorityFeePerGas": "2000000000",
  "nonce": 42
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "hash": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
    "status": "pending",
    "estimatedConfirmation": "2025-12-25T10:32:00Z"
  }
}
```

#### GET /transactions/:hash

Get transaction details.

**Response:**
```json
{
  "success": true,
  "data": {
    "hash": "0xabcdef...",
    "chainId": 1,
    "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
    "value": "1000000000000000000",
    "data": "0x",
    "nonce": 42,
    "gasLimit": "21000",
    "gasUsed": "21000",
    "gasPrice": "25000000000",
    "blockNumber": 18900000,
    "blockHash": "0x123456...",
    "timestamp": "2025-12-25T10:30:00Z",
    "confirmations": 12,
    "status": "success",
    "logs": []
  }
}
```

#### POST /transactions/estimate

Estimate gas for a transaction.

**Request:**
```json
{
  "chainId": 1,
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "value": "1000000000000000000",
  "data": "0x"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "gasLimit": "21000",
    "baseFee": "20000000000",
    "priorityFee": "2000000000",
    "maxFeePerGas": "30000000000",
    "estimatedCost": "630000000000000",
    "estimatedCostUSD": "2.52"
  }
}
```

---

### Token Operations

#### GET /tokens/:contract

Get token information.

**Parameters:**
- `contract`: Token contract address
- `chainId`: Chain ID (optional)

**Response:**
```json
{
  "success": true,
  "data": {
    "contract": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "chainId": 1,
    "standard": "ERC-20",
    "name": "USD Coin",
    "symbol": "USDC",
    "decimals": 6,
    "totalSupply": "42000000000000000",
    "holders": 1500000,
    "price": {
      "usd": 1.00,
      "change24h": 0.02
    }
  }
}
```

#### POST /tokens/transfer

Transfer tokens.

**Request:**
```json
{
  "chainId": 1,
  "contract": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
  "amount": "1000000"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "hash": "0x111111...",
    "status": "pending"
  }
}
```

#### GET /tokens/:contract/holders

Get token holders.

**Response:**
```json
{
  "success": true,
  "data": {
    "holders": [
      {
        "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
        "balance": "1000000000",
        "percentage": 2.38
      }
    ],
    "total": 1500000
  }
}
```

---

### NFT Operations

#### GET /nfts/:contract/:tokenId

Get NFT details.

**Response:**
```json
{
  "success": true,
  "data": {
    "contract": "0xBC4CA0EdA7647A8aB7C2061c2E118A18a936f13D",
    "tokenId": "1337",
    "chainId": 1,
    "standard": "ERC-721",
    "name": "Bored Ape #1337",
    "description": "A bored ape from the BAYC collection",
    "image": "ipfs://QmeSjSinHpPnmXmspMjwiXyN6zS4E9zccariGR3jxcaWtq/1337",
    "attributes": [
      { "trait_type": "Background", "value": "Blue" },
      { "trait_type": "Fur", "value": "Golden" }
    ],
    "owner": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "royalty": {
      "receiver": "0x...",
      "percentage": 5
    }
  }
}
```

#### POST /nfts/transfer

Transfer NFT.

**Request:**
```json
{
  "chainId": 1,
  "contract": "0xBC4CA0EdA7647A8aB7C2061c2E118A18a936f13D",
  "tokenId": "1337",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "hash": "0x222222...",
    "status": "pending"
  }
}
```

---

### DeFi Operations

#### GET /defi/pools

Get liquidity pools.

**Response:**
```json
{
  "success": true,
  "data": {
    "pools": [
      {
        "id": "0x...",
        "protocol": "Uniswap V3",
        "pair": "ETH/USDC",
        "token0": "0x...",
        "token1": "0x...",
        "tvl": "100000000",
        "volume24h": "5000000",
        "apr": 12.5,
        "fee": 0.3
      }
    ]
  }
}
```

#### POST /defi/swap

Swap tokens.

**Request:**
```json
{
  "chainId": 1,
  "protocol": "Uniswap V3",
  "tokenIn": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
  "tokenOut": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
  "amountIn": "1000000000000000000",
  "slippage": 0.5
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "amountOut": "2500000000",
    "priceImpact": 0.1,
    "route": ["ETH", "USDC"],
    "hash": "0x333333..."
  }
}
```

---

## Web3 Authentication

### SIWE (Sign-In with Ethereum)

WIA uses EIP-4361 (Sign-In with Ethereum) for wallet authentication.

#### Authentication Flow

1. **Client requests nonce:**
```
GET /auth/nonce?address=0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb
```

Response:
```json
{
  "nonce": "abc123def456",
  "issuedAt": "2025-12-25T10:30:00Z"
}
```

2. **Client signs message:**
```
Sign this message to authenticate with WIA:

Domain: wia.finance
Address: 0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb
Nonce: abc123def456
Issued At: 2025-12-25T10:30:00Z
Chain ID: 1
```

3. **Client submits signature:**
```
POST /auth/connect
{
  "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "signature": "0x...",
  "message": "...",
  "chainId": 1
}
```

4. **Server returns JWT token**

### JWT Token Format

```json
{
  "header": {
    "alg": "HS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "iat": 1735124400,
    "exp": 1735210800,
    "chainId": 1,
    "scope": "read write"
  }
}
```

---

## Request/Response Format

### Standard Request Headers

```
Content-Type: application/json
Authorization: Bearer {jwt_token}
X-Chain-ID: 1
X-API-Version: 1.0.0
User-Agent: WIA-SDK/1.0.0
```

### Standard Response Format

**Success Response:**
```json
{
  "success": true,
  "data": { ... },
  "metadata": {
    "timestamp": "2025-12-25T10:30:00Z",
    "version": "1.0.0"
  }
}
```

**Error Response:**
```json
{
  "success": false,
  "error": {
    "code": "INVALID_ADDRESS",
    "message": "Invalid Ethereum address format",
    "details": {
      "field": "address",
      "value": "invalid"
    }
  },
  "metadata": {
    "timestamp": "2025-12-25T10:30:00Z",
    "version": "1.0.0"
  }
}
```

---

## Error Codes

### HTTP Status Codes

- `200` - Success
- `201` - Created
- `400` - Bad Request
- `401` - Unauthorized
- `403` - Forbidden
- `404` - Not Found
- `429` - Too Many Requests
- `500` - Internal Server Error
- `503` - Service Unavailable

### WIA Error Codes

| Code | Description |
|------|-------------|
| `INVALID_ADDRESS` | Invalid Ethereum address format |
| `INVALID_SIGNATURE` | Invalid transaction signature |
| `INSUFFICIENT_BALANCE` | Insufficient account balance |
| `INSUFFICIENT_GAS` | Insufficient gas for transaction |
| `NONCE_TOO_LOW` | Transaction nonce too low |
| `NONCE_TOO_HIGH` | Transaction nonce too high |
| `GAS_PRICE_TOO_LOW` | Gas price below minimum |
| `TRANSACTION_FAILED` | Transaction execution failed |
| `CONTRACT_NOT_FOUND` | Smart contract not found |
| `TOKEN_NOT_FOUND` | Token contract not found |
| `CHAIN_NOT_SUPPORTED` | Blockchain not supported |
| `RATE_LIMIT_EXCEEDED` | API rate limit exceeded |
| `INVALID_TOKEN` | Invalid authentication token |
| `TOKEN_EXPIRED` | Authentication token expired |
| `NETWORK_ERROR` | Blockchain network error |

---

## Rate Limiting

### Rate Limit Tiers

| Tier | Requests/Minute | Requests/Day | WebSocket Connections |
|------|-----------------|--------------|----------------------|
| Free | 60 | 10,000 | 5 |
| Basic | 300 | 100,000 | 20 |
| Pro | 1,200 | 500,000 | 100 |
| Enterprise | Unlimited | Unlimited | Unlimited |

### Rate Limit Headers

```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735124460
```

### Handling Rate Limits

When rate limit is exceeded:
```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "details": {
      "limit": 60,
      "remaining": 0,
      "resetAt": "2025-12-25T10:31:00Z"
    }
  }
}
```

---

## WebSocket API

### Connection

```javascript
const ws = new WebSocket('wss://api.wia.finance/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};
```

### Subscriptions

#### Subscribe to New Blocks

```json
{
  "type": "subscribe",
  "channel": "blocks",
  "params": {
    "chainId": 1
  }
}
```

Response:
```json
{
  "type": "block",
  "data": {
    "chainId": 1,
    "blockNumber": 18900001,
    "blockHash": "0x...",
    "timestamp": "2025-12-25T10:30:12Z"
  }
}
```

#### Subscribe to Address Activity

```json
{
  "type": "subscribe",
  "channel": "address",
  "params": {
    "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "chainId": 1
  }
}
```

#### Subscribe to Token Transfers

```json
{
  "type": "subscribe",
  "channel": "tokens",
  "params": {
    "contract": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "chainId": 1
  }
}
```

---

## SDK Integration

### TypeScript SDK

```typescript
import { WIAClient } from '@wia/blockchain-finance';

const client = new WIAClient({
  apiKey: 'your-api-key',
  network: 'mainnet'
});

// Authenticate with wallet
await client.auth.connect(address, signature);

// Send transaction
const tx = await client.transactions.send({
  to: '0x...',
  value: '1000000000000000000'
});

// Get account balance
const balance = await client.accounts.getBalance(address);
```

### Python SDK

```python
from wia_blockchain_finance import WIAClient

client = WIAClient(api_key='your-api-key', network='mainnet')

# Authenticate
client.auth.connect(address, signature)

# Send transaction
tx = client.transactions.send(to='0x...', value='1000000000000000000')

# Get balance
balance = client.accounts.get_balance(address)
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Interoperability Alliance
MIT License
