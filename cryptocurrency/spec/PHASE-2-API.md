# WIA Cryptocurrency Standard
## Phase 2: API Specification

**Version:** 1.0.0
**Standard:** WIA-FIN-003
**Status:** Draft
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Wallet API](#wallet-api)
3. [Exchange API](#exchange-api)
4. [Node RPC API](#node-rpc-api)
5. [Authentication](#authentication)
6. [Error Codes](#error-codes)
7. [Rate Limiting](#rate-limiting)
8. [Webhooks](#webhooks)
9. [Security](#security)
10. [Examples](#examples)

---

## Overview

The WIA Cryptocurrency API Standard defines RESTful and JSON-RPC interfaces for cryptocurrency wallets, exchanges, and blockchain nodes. This specification ensures consistent API design across different implementations and platforms.

### Key Principles

- **RESTful Design**: Follow REST architectural principles
- **JSON-RPC Support**: Compatible with blockchain node standards
- **Standardized Errors**: Consistent error codes and messages
- **Security First**: Built-in authentication and rate limiting
- **Versioning**: API version management
- **Idempotency**: Safe retry mechanisms

### API Versions

- **v1**: Current stable version
- **v2**: Beta features (optional)

### Base URL Format

```
https://api.example.com/v1/cryptocurrency
```

---

## Wallet API

### Wallet Management

#### Create Wallet

```http
POST /v1/wallets
Content-Type: application/json
Authorization: Bearer {token}

{
  "walletType": "HD|single|multisig",
  "cryptocurrency": "BTC|ETH|multi",
  "name": "My Wallet",
  "password": "encrypted_password",
  "mnemonicWordCount": 12,
  "derivationPath": "m/44'/0'/0'/0/0"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "walletId": "wallet_1234567890abcdef",
    "walletType": "HD",
    "cryptocurrency": "BTC",
    "name": "My Wallet",
    "mnemonic": "encrypted_mnemonic_phrase",
    "addresses": [
      {
        "address": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
        "path": "m/44'/0'/0'/0/0",
        "index": 0
      }
    ],
    "createdAt": "2025-12-25T10:00:00Z"
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

#### Get Wallet Details

```http
GET /v1/wallets/{walletId}
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "walletId": "wallet_1234567890abcdef",
    "walletType": "HD",
    "cryptocurrency": "BTC",
    "name": "My Wallet",
    "balance": {
      "confirmed": "500000000",
      "unconfirmed": "10000000",
      "total": "510000000"
    },
    "addresses": [
      {
        "address": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
        "path": "m/44'/0'/0'/0/0",
        "balance": "500000000",
        "transactionCount": 42
      }
    ],
    "createdAt": "2025-12-25T10:00:00Z",
    "lastUsed": "2025-12-25T12:00:00Z"
  }
}
```

#### List Wallets

```http
GET /v1/wallets?page=1&limit=10&cryptocurrency=BTC
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": [
    {
      "walletId": "wallet_1234567890abcdef",
      "name": "My Wallet",
      "cryptocurrency": "BTC",
      "balance": "500000000",
      "addressCount": 5
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 3,
    "totalPages": 1
  }
}
```

#### Generate New Address

```http
POST /v1/wallets/{walletId}/addresses
Authorization: Bearer {token}

{
  "addressType": "P2WPKH|P2TR|P2PKH",
  "label": "Payment Address #42"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "address": "bc1p5d7rjq7g6rdk2yhzks9smlaqtedr4dekq08ge8ztwac72sfr9rusxg3297",
    "addressType": "P2TR",
    "publicKey": "0x1234567890abcdef...",
    "path": "m/44'/0'/0'/0/42",
    "index": 42,
    "label": "Payment Address #42",
    "createdAt": "2025-12-25T10:00:00Z"
  }
}
```

### Transaction Operations

#### Create Transaction

```http
POST /v1/wallets/{walletId}/transactions
Authorization: Bearer {token}
Content-Type: application/json

{
  "to": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
  "amount": "50000000",
  "feeRate": "10",
  "message": "Payment for services",
  "replaceable": true
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "transactionId": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "from": ["bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh"],
    "to": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
    "amount": "50000000",
    "fee": "2240",
    "feeRate": "10",
    "size": 224,
    "status": "pending",
    "replaceable": true,
    "createdAt": "2025-12-25T10:00:00Z"
  }
}
```

#### Sign Transaction

```http
POST /v1/wallets/{walletId}/transactions/{txId}/sign
Authorization: Bearer {token}

{
  "password": "wallet_password"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "transactionId": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "signedTransaction": "0x0200000000010...",
    "signatures": [
      {
        "inputIndex": 0,
        "signature": "304402201234...",
        "publicKey": "031234..."
      }
    ],
    "status": "signed"
  }
}
```

#### Broadcast Transaction

```http
POST /v1/transactions/broadcast
Authorization: Bearer {token}

{
  "signedTransaction": "0x0200000000010...",
  "network": "mainnet"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "transactionId": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "status": "broadcasted",
    "broadcastedAt": "2025-12-25T10:00:00Z",
    "estimatedConfirmation": "2025-12-25T10:10:00Z"
  }
}
```

#### Get Transaction Status

```http
GET /v1/transactions/{txId}
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "transactionId": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "cryptocurrency": "BTC",
    "from": ["bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh"],
    "to": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
    "amount": "50000000",
    "fee": "2240",
    "status": "confirmed",
    "confirmations": 6,
    "blockHeight": 820000,
    "blockHash": "00000000000000000001234...",
    "timestamp": "2025-12-25T10:00:00Z",
    "confirmedAt": "2025-12-25T10:10:00Z"
  }
}
```

#### List Transactions

```http
GET /v1/wallets/{walletId}/transactions?page=1&limit=20&status=confirmed
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": [
    {
      "transactionId": "a1b2c3d4...",
      "type": "send",
      "amount": "-50000000",
      "fee": "2240",
      "status": "confirmed",
      "confirmations": 6,
      "timestamp": "2025-12-25T10:00:00Z"
    },
    {
      "transactionId": "f0e9d8c7...",
      "type": "receive",
      "amount": "100000000",
      "status": "confirmed",
      "confirmations": 100,
      "timestamp": "2025-12-24T10:00:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 42,
    "totalPages": 3
  }
}
```

---

## Exchange API

### Market Data

#### Get Ticker

```http
GET /v1/exchange/ticker/{pair}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "pair": "BTC/USD",
    "exchange": "WIA Exchange",
    "last": "42000.50",
    "bid": "41999.00",
    "ask": "42001.00",
    "high24h": "43000.00",
    "low24h": "41000.00",
    "volume24h": "1234.567",
    "volumeQuote24h": "51852378.50",
    "change24h": "500.50",
    "changePercent24h": "1.21",
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

#### Get Order Book

```http
GET /v1/exchange/orderbook/{pair}?depth=10
```

**Response:**

```json
{
  "success": true,
  "data": {
    "pair": "BTC/USD",
    "bids": [
      {
        "price": "41999.00",
        "amount": "0.5",
        "total": "20999.50"
      },
      {
        "price": "41998.00",
        "amount": "1.2",
        "total": "50397.60"
      }
    ],
    "asks": [
      {
        "price": "42001.00",
        "amount": "0.3",
        "total": "12600.30"
      },
      {
        "price": "42002.00",
        "amount": "0.8",
        "total": "33601.60"
      }
    ],
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

### Trading

#### Place Order

```http
POST /v1/exchange/orders
Authorization: Bearer {token}
Content-Type: application/json

{
  "pair": "BTC/USD",
  "type": "limit|market",
  "side": "buy|sell",
  "amount": "0.5",
  "price": "42000.00",
  "timeInForce": "GTC|IOC|FOK",
  "stopPrice": "41000.00",
  "clientOrderId": "client_order_123"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "orderId": "order_1234567890abcdef",
    "clientOrderId": "client_order_123",
    "pair": "BTC/USD",
    "type": "limit",
    "side": "buy",
    "amount": "0.5",
    "price": "42000.00",
    "filled": "0",
    "remaining": "0.5",
    "status": "open",
    "createdAt": "2025-12-25T10:00:00Z"
  }
}
```

#### Cancel Order

```http
DELETE /v1/exchange/orders/{orderId}
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "orderId": "order_1234567890abcdef",
    "status": "cancelled",
    "cancelledAt": "2025-12-25T10:00:00Z"
  }
}
```

#### Get Order Status

```http
GET /v1/exchange/orders/{orderId}
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "orderId": "order_1234567890abcdef",
    "clientOrderId": "client_order_123",
    "pair": "BTC/USD",
    "type": "limit",
    "side": "buy",
    "amount": "0.5",
    "price": "42000.00",
    "filled": "0.3",
    "remaining": "0.2",
    "averagePrice": "41995.00",
    "status": "partially_filled",
    "trades": [
      {
        "tradeId": "trade_abc123",
        "price": "41990.00",
        "amount": "0.15",
        "fee": "6.2985",
        "feeCurrency": "USD",
        "timestamp": "2025-12-25T10:05:00Z"
      },
      {
        "tradeId": "trade_def456",
        "price": "42000.00",
        "amount": "0.15",
        "fee": "6.3",
        "feeCurrency": "USD",
        "timestamp": "2025-12-25T10:06:00Z"
      }
    ],
    "createdAt": "2025-12-25T10:00:00Z",
    "updatedAt": "2025-12-25T10:06:00Z"
  }
}
```

#### List Open Orders

```http
GET /v1/exchange/orders?status=open&pair=BTC/USD
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": [
    {
      "orderId": "order_1234567890abcdef",
      "pair": "BTC/USD",
      "type": "limit",
      "side": "buy",
      "amount": "0.5",
      "price": "42000.00",
      "filled": "0.3",
      "status": "partially_filled",
      "createdAt": "2025-12-25T10:00:00Z"
    }
  ]
}
```

### Account

#### Get Balance

```http
GET /v1/exchange/balance
Authorization: Bearer {token}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "balances": [
      {
        "currency": "BTC",
        "available": "5.5",
        "locked": "0.5",
        "total": "6.0"
      },
      {
        "currency": "USD",
        "available": "100000.00",
        "locked": "21000.00",
        "total": "121000.00"
      },
      {
        "currency": "ETH",
        "available": "50.0",
        "locked": "0",
        "total": "50.0"
      }
    ],
    "totalValueUSD": "570000.00",
    "updatedAt": "2025-12-25T10:00:00Z"
  }
}
```

#### Deposit

```http
POST /v1/exchange/deposit
Authorization: Bearer {token}

{
  "currency": "BTC",
  "network": "bitcoin"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "currency": "BTC",
    "network": "bitcoin",
    "address": "bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4",
    "memo": null,
    "minDeposit": "0.0001",
    "confirmationsRequired": 3,
    "instructions": "Send BTC to the address above. Minimum 3 confirmations required."
  }
}
```

#### Withdraw

```http
POST /v1/exchange/withdraw
Authorization: Bearer {token}

{
  "currency": "BTC",
  "network": "bitcoin",
  "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
  "amount": "0.5",
  "memo": "",
  "twoFactorCode": "123456"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "withdrawalId": "withdrawal_abc123",
    "currency": "BTC",
    "network": "bitcoin",
    "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
    "amount": "0.5",
    "fee": "0.0001",
    "totalDeducted": "0.5001",
    "status": "pending",
    "txId": null,
    "createdAt": "2025-12-25T10:00:00Z",
    "estimatedCompletion": "2025-12-25T10:30:00Z"
  }
}
```

---

## Node RPC API

### Bitcoin JSON-RPC

#### Get Blockchain Info

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "getblockchaininfo",
  "params": []
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "chain": "main",
    "blocks": 820000,
    "headers": 820000,
    "bestblockhash": "00000000000000000001234...",
    "difficulty": 65512345678901,
    "mediantime": 1735120000,
    "verificationprogress": 0.9999,
    "chainwork": "00000000000000000000000000000000000000005d1f8...",
    "pruned": false,
    "softforks": {
      "taproot": {
        "type": "buried",
        "active": true,
        "height": 709632
      }
    }
  }
}
```

#### Get Block

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "getblock",
  "params": ["00000000000000000001234...", 2]
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "hash": "00000000000000000001234...",
    "confirmations": 6,
    "height": 820000,
    "version": 536870912,
    "versionHex": "20000000",
    "merkleroot": "abc123def456...",
    "time": 1735120000,
    "mediantime": 1735119000,
    "nonce": 2573587392,
    "bits": "170ba48f",
    "difficulty": 65512345678901,
    "chainwork": "00000000000000000000000000000000000000005d1f8...",
    "nTx": 2500,
    "previousblockhash": "00000000000000000001fed...",
    "tx": [
      {
        "txid": "a1b2c3d4...",
        "hash": "a1b2c3d4...",
        "version": 2,
        "size": 250,
        "vsize": 169,
        "weight": 676,
        "locktime": 0,
        "vin": [],
        "vout": []
      }
    ]
  }
}
```

#### Send Raw Transaction

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "sendrawtransaction",
  "params": ["0x0200000000010..."]
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2"
}
```

### Ethereum JSON-RPC

#### Get Balance

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "eth_getBalance",
  "params": ["0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb", "latest"]
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": "0x8ac7230489e80000"
}
```

#### Send Transaction

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "eth_sendTransaction",
  "params": [
    {
      "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
      "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
      "value": "0xde0b6b3a7640000",
      "gas": "0x5208",
      "maxFeePerGas": "0x5d21dba00",
      "maxPriorityFeePerGas": "0x77359400"
    }
  ]
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890"
}
```

#### Get Transaction Receipt

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "eth_getTransactionReceipt",
  "params": ["0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890"]
}
```

**Response:**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "transactionHash": "0xabcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
    "transactionIndex": "0x1",
    "blockHash": "0x1234567890abcdef...",
    "blockNumber": "0x11a4a1c",
    "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "to": "0x5aAeb6053F3E94C9b9A09f33669435E7Ef1BeAed",
    "cumulativeGasUsed": "0x5208",
    "gasUsed": "0x5208",
    "contractAddress": null,
    "logs": [],
    "logsBloom": "0x00000000...",
    "status": "0x1",
    "effectiveGasPrice": "0x4e3b29200"
  }
}
```

---

## Authentication

### API Key Authentication

```http
GET /v1/wallets
Authorization: Bearer sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER
```

### OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTH_CODE&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET&
redirect_uri=REDIRECT_URI
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_1234567890abcdef",
  "scope": "wallet:read wallet:write exchange:trade"
}
```

### Two-Factor Authentication

Required for sensitive operations:

```http
POST /v1/exchange/withdraw
Authorization: Bearer {token}
X-2FA-Code: 123456

{
  "currency": "BTC",
  "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
  "amount": "0.5"
}
```

---

## Error Codes

### Standard Error Response

```json
{
  "success": false,
  "error": {
    "code": "INSUFFICIENT_FUNDS",
    "message": "Insufficient balance to complete transaction",
    "details": {
      "required": "51000000",
      "available": "50000000",
      "currency": "satoshi"
    },
    "requestId": "req_abc123",
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

### Error Code Reference

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Invalid request parameters |
| `UNAUTHORIZED` | 401 | Missing or invalid authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `INVALID_ADDRESS` | 400 | Invalid cryptocurrency address |
| `INSUFFICIENT_FUNDS` | 400 | Insufficient balance |
| `INVALID_SIGNATURE` | 400 | Invalid transaction signature |
| `DUPLICATE_TRANSACTION` | 409 | Transaction already exists |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `NETWORK_ERROR` | 503 | Blockchain network error |
| `INTERNAL_ERROR` | 500 | Internal server error |
| `MAINTENANCE` | 503 | Service under maintenance |

---

## Rate Limiting

### Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1735120800
```

### Rate Limit Tiers

| Tier | Requests/Minute | Requests/Hour | Requests/Day |
|------|----------------|---------------|--------------|
| Free | 60 | 1,000 | 10,000 |
| Basic | 300 | 10,000 | 100,000 |
| Pro | 1,000 | 50,000 | 1,000,000 |
| Enterprise | Custom | Custom | Custom |

### Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please try again later.",
    "details": {
      "limit": 1000,
      "remaining": 0,
      "resetAt": "2025-12-25T11:00:00Z"
    }
  }
}
```

---

## Webhooks

### Register Webhook

```http
POST /v1/webhooks
Authorization: Bearer {token}

{
  "url": "https://your-server.com/webhooks/crypto",
  "events": [
    "transaction.confirmed",
    "transaction.pending",
    "deposit.completed",
    "withdrawal.completed",
    "order.filled"
  ],
  "secret": "webhook_secret_key"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "webhookId": "webhook_abc123",
    "url": "https://your-server.com/webhooks/crypto",
    "events": ["transaction.confirmed", "transaction.pending"],
    "status": "active",
    "createdAt": "2025-12-25T10:00:00Z"
  }
}
```

### Webhook Payload

```json
{
  "event": "transaction.confirmed",
  "webhookId": "webhook_abc123",
  "data": {
    "transactionId": "a1b2c3d4e5f6...",
    "cryptocurrency": "BTC",
    "amount": "50000000",
    "confirmations": 6,
    "confirmedAt": "2025-12-25T10:10:00Z"
  },
  "timestamp": "2025-12-25T10:10:00Z",
  "signature": "sha256=1234567890abcdef..."
}
```

### Webhook Signature Verification

```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const hmac = crypto.createHmac('sha256', secret);
  const digest = 'sha256=' + hmac.update(JSON.stringify(payload)).digest('hex');
  return crypto.timingSafeEqual(Buffer.from(signature), Buffer.from(digest));
}
```

---

## Security

### HTTPS Only

All API endpoints must use HTTPS. HTTP requests will be rejected.

### Request Signing

For high-security operations, requests can be signed:

```http
POST /v1/exchange/withdraw
Authorization: Bearer {token}
X-Signature: sha256=1234567890abcdef...
X-Timestamp: 1735120000
X-Nonce: random_nonce_123

{
  "currency": "BTC",
  "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
  "amount": "0.5"
}
```

### IP Whitelisting

Optional IP address whitelisting for API keys:

```http
POST /v1/api-keys/{keyId}/whitelist
Authorization: Bearer {token}

{
  "ipAddresses": [
    "192.168.1.100",
    "10.0.0.50"
  ]
}
```

---

## Examples

### Complete Wallet Workflow

```javascript
// 1. Create wallet
const wallet = await fetch('https://api.example.com/v1/wallets', {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer sk_live_...',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    walletType: 'HD',
    cryptocurrency: 'BTC',
    name: 'My BTC Wallet',
    password: 'encrypted_password',
    mnemonicWordCount: 12
  })
});

// 2. Generate new address
const address = await fetch(`https://api.example.com/v1/wallets/${wallet.walletId}/addresses`, {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer sk_live_...',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    addressType: 'P2WPKH',
    label: 'Payment Address'
  })
});

// 3. Create transaction
const tx = await fetch(`https://api.example.com/v1/wallets/${wallet.walletId}/transactions`, {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer sk_live_...',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    to: 'bc1qw508d6qejxtdg4y5r3zarvary0c5xw7kv8f3t4',
    amount: '50000000',
    feeRate: '10'
  })
});

// 4. Sign transaction
const signed = await fetch(`https://api.example.com/v1/wallets/${wallet.walletId}/transactions/${tx.transactionId}/sign`, {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer sk_live_...',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    password: 'wallet_password'
  })
});

// 5. Broadcast transaction
const broadcast = await fetch('https://api.example.com/v1/transactions/broadcast', {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer sk_live_...',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    signedTransaction: signed.signedTransaction,
    network: 'mainnet'
  })
});
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Interoperability Alliance
MIT License
