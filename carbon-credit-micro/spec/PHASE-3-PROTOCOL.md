# WIA Carbon Credit Micro Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Transport Layer](#transport-layer)
4. [Message Format](#message-format)
5. [Connection Lifecycle](#connection-lifecycle)
6. [Real-Time Market Protocol](#real-time-market-protocol)
7. [Blockchain Communication](#blockchain-communication)
8. [Security](#security)
9. [Error Handling](#error-handling)
10. [Protocol Examples](#protocol-examples)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Carbon Credit Micro Protocol Standard defines the communication protocols for real-time carbon footprint tracking, credit trading, and blockchain verification. This Phase 3 specification builds upon Phase 1 Data Format and Phase 2 API Interface, providing the network-level protocols for distributed carbon credit systems.

**Core Objectives**:
- Enable real-time bidirectional communication for market data
- Support high-frequency trading with low latency
- Provide reliable blockchain event streaming
- Ensure secure and encrypted data transmission
- Enable peer-to-peer credit transfers

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Transport Protocols** | HTTPS, WebSocket, gRPC |
| **Message Formats** | JSON-RPC, Protocol Buffers |
| **Event Streaming** | Server-Sent Events, WebSocket streams |
| **Blockchain Integration** | Web3, Ethereum JSON-RPC |
| **Security Layer** | TLS 1.3, JWT, OAuth 2.0 |

### 1.3 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer (Phase 2)     │
│  (API Endpoints, Business Logic)    │
└─────────────────────────────────────┘
                 ↕
┌─────────────────────────────────────┐
│      Protocol Layer (Phase 3)       │
│  (Message Format, Event Streams)    │
└─────────────────────────────────────┘
                 ↕
┌─────────────────────────────────────┐
│        Transport Layer              │
│  (HTTPS, WebSocket, gRPC)          │
└─────────────────────────────────────┘
                 ↕
┌─────────────────────────────────────┐
│        Security Layer               │
│  (TLS 1.3, Authentication)         │
└─────────────────────────────────────┘
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **WebSocket** | Full-duplex communication protocol over TCP |
| **JSON-RPC** | Remote procedure call protocol using JSON |
| **Server-Sent Events (SSE)** | Server-to-client event streaming |
| **gRPC** | High-performance RPC framework using Protocol Buffers |
| **Message Broker** | Middleware for asynchronous message passing |
| **Event Stream** | Continuous flow of real-time events |

### 2.2 Protocol Types

| Protocol | Use Case | Latency | Reliability |
|----------|----------|---------|-------------|
| HTTPS | REST API requests | Medium | High |
| WebSocket | Real-time market data | Low | Medium |
| gRPC | High-performance trading | Very Low | High |
| SSE | One-way event notifications | Low | Medium |

---

## Transport Layer

### 3.1 HTTPS (REST)

Primary protocol for request-response operations.

**Configuration**:
```
Protocol: HTTPS
TLS Version: 1.3 or higher
Port: 443
Base URL: https://api.wia.live/carbon-credit-micro/v1
Content-Type: application/json
```

**Request Headers**:
```http
GET /footprint/daily/2025-01-15 HTTP/1.1
Host: api.wia.live
Authorization: Bearer <jwt_token>
Content-Type: application/json
Accept: application/json
User-Agent: WIA-CCM-Client/1.0.0
X-Request-ID: req_abc123xyz789
```

**Response Headers**:
```http
HTTP/1.1 200 OK
Content-Type: application/json
X-Request-ID: req_abc123xyz789
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1642262400
```

### 3.2 WebSocket

Bidirectional protocol for real-time market data and notifications.

**Connection URL**:
```
wss://ws.wia.live/carbon-credit-micro/v1/stream
```

**Connection Flow**:
```
Client                          Server
  |                               |
  |--- WebSocket Upgrade -------->|
  |                               |
  |<-- 101 Switching Protocols ---|
  |                               |
  |--- Authentication Message ---->|
  |                               |
  |<-- Auth Success --------------|
  |                               |
  |--- Subscribe to Channels ----->|
  |                               |
  |<-- Subscription Confirmed ----|
  |                               |
  |<== Real-time Data Stream ====>|
  |                               |
```

**Connection Request**:
```http
GET /carbon-credit-micro/v1/stream HTTP/1.1
Host: ws.wia.live
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: wccm-v1
Origin: https://yourapp.com
```

**Connection Response**:
```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
Sec-WebSocket-Protocol: wccm-v1
```

### 3.3 gRPC

High-performance protocol for trading operations.

**Service Definition** (Protocol Buffers):
```protobuf
syntax = "proto3";

package wia.carbon_credit_micro.v1;

service CarbonCreditTrading {
  rpc GetMarketPrice(MarketPriceRequest) returns (MarketPriceResponse);
  rpc PlaceOrder(OrderRequest) returns (OrderResponse);
  rpc CancelOrder(CancelOrderRequest) returns (CancelOrderResponse);
  rpc StreamMarketData(StreamRequest) returns (stream MarketUpdate);
  rpc StreamOrders(StreamRequest) returns (stream OrderUpdate);
}

message MarketPriceRequest {
  string currency = 1;
}

message MarketPriceResponse {
  double current_price = 1;
  double change_24h = 2;
  double volume_24h = 3;
  int64 timestamp = 4;
}

message OrderRequest {
  string user_id = 1;
  OrderType type = 2;
  double amount = 3;
  double price = 4;
}

enum OrderType {
  BUY = 0;
  SELL = 1;
}

message OrderResponse {
  string order_id = 1;
  OrderStatus status = 2;
  string message = 3;
}

enum OrderStatus {
  PENDING = 0;
  EXECUTED = 1;
  CANCELLED = 2;
  FAILED = 3;
}
```

**gRPC Configuration**:
```
Endpoint: grpc.wia.live:443
TLS: Enabled (TLS 1.3)
Compression: gzip
Max Message Size: 4 MB
Keepalive: 30 seconds
```

---

## Message Format

### 4.1 JSON-RPC 2.0

Standard message format for WebSocket communication.

**Request Format**:
```json
{
  "jsonrpc": "2.0",
  "id": "req_123",
  "method": "subscribe",
  "params": {
    "channel": "market.price",
    "symbols": ["WCCM/USD"]
  }
}
```

**Response Format**:
```json
{
  "jsonrpc": "2.0",
  "id": "req_123",
  "result": {
    "subscribed": true,
    "channel": "market.price",
    "symbols": ["WCCM/USD"]
  }
}
```

**Error Response**:
```json
{
  "jsonrpc": "2.0",
  "id": "req_123",
  "error": {
    "code": -32602,
    "message": "Invalid params",
    "data": {
      "field": "symbols",
      "reason": "Array cannot be empty"
    }
  }
}
```

**Notification (Server Push)**:
```json
{
  "jsonrpc": "2.0",
  "method": "market.price.update",
  "params": {
    "symbol": "WCCM/USD",
    "price": 25.5,
    "change": 0.5,
    "timestamp": "2025-01-15T12:00:00Z"
  }
}
```

### 4.2 Event Message Format

**Market Price Update**:
```json
{
  "type": "market.price",
  "event": "update",
  "timestamp": "2025-01-15T12:00:00Z",
  "data": {
    "symbol": "WCCM/USD",
    "price": 25.5,
    "change24h": 0.5,
    "changePercent24h": 2.0,
    "volume24h": 1250.5,
    "bid": 25.4,
    "ask": 25.6,
    "spread": 0.2
  }
}
```

**Order Executed**:
```json
{
  "type": "order",
  "event": "executed",
  "timestamp": "2025-01-15T12:05:30Z",
  "data": {
    "orderId": "ORD-2025-0001",
    "userId": "USER-2025-001",
    "type": "buy",
    "amount": 0.5,
    "price": 25.5,
    "totalPrice": 12.75,
    "status": "executed",
    "blockchainTx": "0xabc123..."
  }
}
```

**Credit Balance Update**:
```json
{
  "type": "wallet",
  "event": "balance_changed",
  "timestamp": "2025-01-15T12:05:31Z",
  "data": {
    "userId": "USER-2025-001",
    "previousBalance": 2.0,
    "newBalance": 2.5,
    "change": 0.5,
    "reason": "purchase",
    "transactionId": "TXN-2025-0001"
  }
}
```

**Footprint Tracked**:
```json
{
  "type": "footprint",
  "event": "activity_tracked",
  "timestamp": "2025-01-15T08:30:00Z",
  "data": {
    "userId": "USER-2025-001",
    "activityId": "ACT-2025-0001",
    "category": "transportation",
    "co2Amount": 5.2,
    "verified": true,
    "totalDaily": 5.2
  }
}
```

**Blockchain Confirmation**:
```json
{
  "type": "blockchain",
  "event": "transaction_confirmed",
  "timestamp": "2025-01-15T12:10:00Z",
  "data": {
    "transactionHash": "0xabc123...",
    "blockNumber": 18234567,
    "confirmations": 12,
    "status": "confirmed",
    "recordId": "CCR-2025-000001"
  }
}
```

### 4.3 Binary Message Format (Protocol Buffers)

For high-performance trading via gRPC:

```protobuf
message MarketUpdate {
  string symbol = 1;
  double price = 2;
  double volume = 3;
  int64 timestamp = 4;
  PriceChange change = 5;
}

message PriceChange {
  double absolute = 1;
  double percent = 2;
  TimePeriod period = 3;
}

enum TimePeriod {
  HOUR_1 = 0;
  HOUR_24 = 1;
  DAY_7 = 2;
  DAY_30 = 3;
}
```

---

## Connection Lifecycle

### 5.1 WebSocket Connection

**1. Connection Establishment**:
```javascript
const ws = new WebSocket('wss://ws.wia.live/carbon-credit-micro/v1/stream');

ws.on('open', () => {
  console.log('WebSocket connected');
});
```

**2. Authentication**:
```javascript
// Send authentication message
ws.send(JSON.stringify({
  jsonrpc: '2.0',
  id: 'auth_1',
  method: 'authenticate',
  params: {
    token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }
}));

// Wait for auth response
ws.on('message', (data) => {
  const message = JSON.parse(data);
  if (message.id === 'auth_1' && message.result.authenticated) {
    console.log('Authenticated successfully');
  }
});
```

**3. Channel Subscription**:
```javascript
// Subscribe to market price updates
ws.send(JSON.stringify({
  jsonrpc: '2.0',
  id: 'sub_1',
  method: 'subscribe',
  params: {
    channels: [
      'market.price',
      'market.trades',
      'wallet.balance',
      'blockchain.confirmations'
    ]
  }
}));
```

**4. Receive Updates**:
```javascript
ws.on('message', (data) => {
  const message = JSON.parse(data);

  // Handle different event types
  if (message.method === 'market.price.update') {
    console.log('Price update:', message.params);
  } else if (message.method === 'wallet.balance_changed') {
    console.log('Balance changed:', message.params);
  }
});
```

**5. Heartbeat/Ping-Pong**:
```javascript
// Server sends ping every 30 seconds
ws.on('ping', () => {
  ws.pong();
});

// Client can also send pings
setInterval(() => {
  ws.ping();
}, 30000);
```

**6. Graceful Disconnection**:
```javascript
// Unsubscribe before closing
ws.send(JSON.stringify({
  jsonrpc: '2.0',
  id: 'unsub_1',
  method: 'unsubscribe',
  params: {
    channels: ['market.price']
  }
}));

// Close connection
ws.close(1000, 'Client disconnect');
```

### 5.2 Connection Recovery

**Automatic Reconnection**:
```javascript
class ResilientWebSocket {
  constructor(url) {
    this.url = url;
    this.reconnectDelay = 1000;
    this.maxReconnectDelay = 30000;
    this.connect();
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.on('open', () => {
      console.log('Connected');
      this.reconnectDelay = 1000;
      this.resubscribe();
    });

    this.ws.on('close', (code, reason) => {
      console.log('Disconnected:', code, reason);
      this.scheduleReconnect();
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });
  }

  scheduleReconnect() {
    setTimeout(() => {
      console.log('Reconnecting...');
      this.connect();
    }, this.reconnectDelay);

    // Exponential backoff
    this.reconnectDelay = Math.min(
      this.reconnectDelay * 2,
      this.maxReconnectDelay
    );
  }

  resubscribe() {
    // Re-subscribe to previous channels
    if (this.subscriptions) {
      this.ws.send(JSON.stringify({
        jsonrpc: '2.0',
        method: 'subscribe',
        params: { channels: this.subscriptions }
      }));
    }
  }
}
```

### 5.3 Session Management

**Session State**:
```json
{
  "sessionId": "sess_abc123",
  "userId": "USER-2025-001",
  "connectedAt": "2025-01-15T10:00:00Z",
  "lastActivity": "2025-01-15T12:30:00Z",
  "subscriptions": [
    "market.price",
    "wallet.balance"
  ],
  "expiresAt": "2025-01-15T22:00:00Z"
}
```

**Session Refresh**:
```javascript
// Refresh session before expiration
ws.send(JSON.stringify({
  jsonrpc: '2.0',
  id: 'refresh_1',
  method: 'session.refresh',
  params: {
    sessionId: 'sess_abc123'
  }
}));
```

---

## Real-Time Market Protocol

### 6.1 Market Data Channels

**Available Channels**:

| Channel | Description | Update Frequency |
|---------|-------------|------------------|
| `market.price` | Current market price | Real-time |
| `market.trades` | Recent trades | Per trade |
| `market.orderbook` | Order book depth | Real-time |
| `market.candles` | OHLCV candles | Per interval |
| `market.statistics` | 24h statistics | Every 5 seconds |

**Subscribe to Market Price**:
```json
{
  "jsonrpc": "2.0",
  "id": "sub_price",
  "method": "subscribe",
  "params": {
    "channel": "market.price",
    "symbols": ["WCCM/USD", "WCCM/EUR"]
  }
}
```

**Price Update Stream**:
```json
{
  "jsonrpc": "2.0",
  "method": "market.price.update",
  "params": {
    "symbol": "WCCM/USD",
    "price": 25.5,
    "bid": 25.4,
    "ask": 25.6,
    "spread": 0.2,
    "volume24h": 1250.5,
    "change24h": 0.5,
    "timestamp": "2025-01-15T12:00:00.123Z"
  }
}
```

### 6.2 Trade Execution Protocol

**Place Order**:
```json
{
  "jsonrpc": "2.0",
  "id": "order_1",
  "method": "order.place",
  "params": {
    "type": "limit",
    "side": "buy",
    "amount": 0.5,
    "price": 25.0,
    "timeInForce": "GTC"
  }
}
```

**Order Response**:
```json
{
  "jsonrpc": "2.0",
  "id": "order_1",
  "result": {
    "orderId": "ORD-2025-0001",
    "status": "pending",
    "createdAt": "2025-01-15T12:00:00Z",
    "estimatedExecution": "immediate"
  }
}
```

**Order Status Update**:
```json
{
  "jsonrpc": "2.0",
  "method": "order.update",
  "params": {
    "orderId": "ORD-2025-0001",
    "status": "executed",
    "executedAmount": 0.5,
    "executedPrice": 25.0,
    "totalPrice": 12.5,
    "executedAt": "2025-01-15T12:00:05Z",
    "blockchainTx": "0xabc123..."
  }
}
```

### 6.3 Order Book Protocol

**Subscribe to Order Book**:
```json
{
  "jsonrpc": "2.0",
  "id": "sub_orderbook",
  "method": "subscribe",
  "params": {
    "channel": "market.orderbook",
    "symbol": "WCCM/USD",
    "depth": 20
  }
}
```

**Order Book Snapshot**:
```json
{
  "jsonrpc": "2.0",
  "method": "market.orderbook.snapshot",
  "params": {
    "symbol": "WCCM/USD",
    "timestamp": "2025-01-15T12:00:00Z",
    "bids": [
      [25.0, 10.5],
      [24.9, 5.2],
      [24.8, 8.1]
    ],
    "asks": [
      [25.1, 7.3],
      [25.2, 12.4],
      [25.3, 6.8]
    ]
  }
}
```

**Order Book Delta**:
```json
{
  "jsonrpc": "2.0",
  "method": "market.orderbook.delta",
  "params": {
    "symbol": "WCCM/USD",
    "timestamp": "2025-01-15T12:00:01Z",
    "changes": [
      { "side": "bid", "price": 25.0, "amount": 12.3 },
      { "side": "ask", "price": 25.2, "amount": 0 }
    ]
  }
}
```

---

## Blockchain Communication

### 7.1 Web3 Provider Integration

**Connection Configuration**:
```javascript
const Web3 = require('web3');

const web3 = new Web3(
  new Web3.providers.WebsocketProvider(
    'wss://mainnet.infura.io/ws/v3/YOUR_PROJECT_ID',
    {
      reconnect: {
        auto: true,
        delay: 5000,
        maxAttempts: 10,
        onTimeout: false
      }
    }
  )
);
```

**Smart Contract Interface**:
```javascript
const contractABI = [...];  // Carbon Credit Token ABI
const contractAddress = '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1';

const carbonContract = new web3.eth.Contract(
  contractABI,
  contractAddress
);
```

### 7.2 Event Listening

**Token Transfer Events**:
```javascript
// Listen for Transfer events
carbonContract.events.Transfer({
  filter: { from: userWalletAddress },
  fromBlock: 'latest'
})
.on('data', (event) => {
  console.log('Transfer event:', {
    from: event.returnValues.from,
    to: event.returnValues.to,
    amount: event.returnValues.value,
    txHash: event.transactionHash
  });
})
.on('error', (error) => {
  console.error('Event error:', error);
});
```

**Credit Minted Events**:
```javascript
carbonContract.events.CreditMinted({
  fromBlock: 'latest'
})
.on('data', (event) => {
  console.log('Credit minted:', {
    recipient: event.returnValues.recipient,
    amount: event.returnValues.amount,
    creditId: event.returnValues.creditId
  });
});
```

### 7.3 Transaction Submission

**Submit Transaction**:
```javascript
async function mintCarbonCredit(amount, creditId) {
  const gasPrice = await web3.eth.getGasPrice();

  const tx = carbonContract.methods.mint(
    userWalletAddress,
    web3.utils.toWei(amount.toString(), 'ether'),
    creditId
  );

  const gas = await tx.estimateGas({ from: userWalletAddress });

  const receipt = await tx.send({
    from: userWalletAddress,
    gas: gas,
    gasPrice: gasPrice
  });

  return receipt.transactionHash;
}
```

**Transaction Monitoring**:
```javascript
function monitorTransaction(txHash) {
  return new Promise((resolve, reject) => {
    const interval = setInterval(async () => {
      const receipt = await web3.eth.getTransactionReceipt(txHash);

      if (receipt) {
        clearInterval(interval);

        if (receipt.status) {
          resolve({
            confirmed: true,
            blockNumber: receipt.blockNumber,
            gasUsed: receipt.gasUsed
          });
        } else {
          reject(new Error('Transaction failed'));
        }
      }
    }, 3000);
  });
}
```

### 7.4 Block Confirmation Stream

**Subscribe to New Blocks**:
```javascript
web3.eth.subscribe('newBlockHeaders')
  .on('data', (blockHeader) => {
    console.log('New block:', {
      number: blockHeader.number,
      hash: blockHeader.hash,
      timestamp: blockHeader.timestamp
    });

    // Check confirmations for pending transactions
    checkPendingTransactions(blockHeader.number);
  });
```

---

## Security

### 8.1 Transport Layer Security

**TLS Configuration**:
```
TLS Version: 1.3
Cipher Suites:
  - TLS_AES_256_GCM_SHA384
  - TLS_AES_128_GCM_SHA256
  - TLS_CHACHA20_POLY1305_SHA256

Certificate Validation: Strict
HSTS: Enabled (max-age=31536000)
Certificate Pinning: Recommended for mobile apps
```

### 8.2 Authentication & Authorization

**JWT Token Structure**:
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "wia-key-001"
  },
  "payload": {
    "sub": "USER-2025-001",
    "iss": "https://auth.wia.live",
    "aud": "carbon-credit-micro",
    "exp": 1642262400,
    "iat": 1642258800,
    "scope": [
      "footprint:read",
      "footprint:write",
      "credits:read",
      "credits:write",
      "trading:execute"
    ]
  }
}
```

**API Key Format**:
```
Format: wccm_{environment}_{random}
Example: wccm_live_sk_abc123xyz789def456ghi012

Components:
- Prefix: wccm (WIA Carbon Credit Micro)
- Environment: live, test, dev
- Type: sk (secret key), pk (public key)
- Random: 24-character alphanumeric
```

### 8.3 Message Signing

**Sign Message**:
```javascript
const crypto = require('crypto');

function signMessage(message, privateKey) {
  const sign = crypto.createSign('SHA256');
  sign.update(JSON.stringify(message));
  sign.end();

  return sign.sign(privateKey, 'base64');
}

// Sign order request
const orderMessage = {
  userId: 'USER-2025-001',
  type: 'buy',
  amount: 0.5,
  price: 25.0,
  timestamp: Date.now()
};

const signature = signMessage(orderMessage, privateKey);
```

**Verify Signature**:
```javascript
function verifyMessage(message, signature, publicKey) {
  const verify = crypto.createVerify('SHA256');
  verify.update(JSON.stringify(message));
  verify.end();

  return verify.verify(publicKey, signature, 'base64');
}
```

### 8.4 Rate Limiting

**Rate Limit Headers**:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1642262400
X-RateLimit-Window: 3600
```

**Rate Limit Tiers**:

| Tier | Requests/Hour | WebSocket Connections | Trading Orders/Minute |
|------|---------------|----------------------|----------------------|
| Free | 1,000 | 1 | 10 |
| Basic | 10,000 | 5 | 50 |
| Pro | 100,000 | 20 | 200 |
| Enterprise | Unlimited | Unlimited | Unlimited |

**Rate Limit Exceeded Response**:
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Too many requests",
    "retryAfter": 3600,
    "limit": 1000,
    "window": 3600
  }
}
```

---

## Error Handling

### 9.1 WebSocket Error Codes

| Code | Description | Action |
|------|-------------|--------|
| 1000 | Normal closure | None |
| 1001 | Going away | Reconnect |
| 1002 | Protocol error | Check message format |
| 1003 | Unsupported data | Check data type |
| 1006 | Abnormal closure | Reconnect |
| 1008 | Policy violation | Check authentication |
| 1011 | Server error | Retry with backoff |
| 4000 | Authentication failed | Re-authenticate |
| 4001 | Invalid subscription | Check channel name |
| 4002 | Rate limit exceeded | Wait and retry |

### 9.2 JSON-RPC Error Codes

| Code | Message | Description |
|------|---------|-------------|
| -32700 | Parse error | Invalid JSON |
| -32600 | Invalid request | Invalid JSON-RPC |
| -32601 | Method not found | Unknown method |
| -32602 | Invalid params | Invalid parameters |
| -32603 | Internal error | Server error |
| -32000 | Server error | Generic server error |
| -32001 | Unauthorized | Authentication failed |
| -32002 | Forbidden | Insufficient permissions |
| -32003 | Not found | Resource not found |

### 9.3 Error Recovery

**Retry Strategy**:
```javascript
async function retryWithBackoff(operation, maxRetries = 5) {
  let lastError;

  for (let i = 0; i < maxRetries; i++) {
    try {
      return await operation();
    } catch (error) {
      lastError = error;

      // Don't retry on client errors
      if (error.code >= 400 && error.code < 500) {
        throw error;
      }

      // Exponential backoff
      const delay = Math.min(1000 * Math.pow(2, i), 30000);
      await new Promise(resolve => setTimeout(resolve, delay));
    }
  }

  throw lastError;
}
```

---

## Protocol Examples

### 10.1 Complete Trading Session

```javascript
const WebSocket = require('ws');

async function tradingSession() {
  // 1. Connect to WebSocket
  const ws = new WebSocket('wss://ws.wia.live/carbon-credit-micro/v1/stream');

  await new Promise((resolve) => {
    ws.on('open', resolve);
  });

  // 2. Authenticate
  ws.send(JSON.stringify({
    jsonrpc: '2.0',
    id: 'auth_1',
    method: 'authenticate',
    params: {
      token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
    }
  }));

  // 3. Subscribe to market data
  ws.send(JSON.stringify({
    jsonrpc: '2.0',
    id: 'sub_1',
    method: 'subscribe',
    params: {
      channels: ['market.price', 'order.updates', 'wallet.balance']
    }
  }));

  // 4. Listen for price updates
  ws.on('message', (data) => {
    const message = JSON.parse(data);

    if (message.method === 'market.price.update') {
      const price = message.params.price;

      // 5. Execute trade if price is favorable
      if (price < 25.0) {
        ws.send(JSON.stringify({
          jsonrpc: '2.0',
          id: 'order_' + Date.now(),
          method: 'order.place',
          params: {
            type: 'market',
            side: 'buy',
            amount: 1.0
          }
        }));
      }
    }

    // 6. Handle order execution
    if (message.method === 'order.update') {
      console.log('Order executed:', message.params);
    }
  });
}
```

### 10.2 Blockchain Event Monitoring

```python
from web3 import Web3
from web3.middleware import geth_poa_middleware

# Connect to blockchain
w3 = Web3(Web3.WebsocketProvider(
    'wss://polygon-mainnet.infura.io/ws/v3/YOUR_PROJECT_ID'
))
w3.middleware_onion.inject(geth_poa_middleware, layer=0)

# Load contract
contract_address = '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1'
contract_abi = [...]  # ABI definition

carbon_contract = w3.eth.contract(
    address=contract_address,
    abi=contract_abi
)

# Subscribe to events
def handle_transfer(event):
    print(f"Transfer: {event['args']['from']} -> {event['args']['to']}")
    print(f"Amount: {event['args']['value']} CCM")
    print(f"Transaction: {event['transactionHash'].hex()}")

transfer_filter = carbon_contract.events.Transfer.create_filter(
    fromBlock='latest'
)

# Event loop
while True:
    for event in transfer_filter.get_new_entries():
        handle_transfer(event)
    time.sleep(2)
```

---

## References

### Related Standards

- [WIA Carbon Credit Micro Data Format (Phase 1)](/carbon-credit-micro/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Carbon Credit Micro API Interface (Phase 2)](/carbon-credit-micro/spec/PHASE-2-API-INTERFACE.md)

### Protocol Standards

- [RFC 6455 - The WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [JSON-RPC 2.0 Specification](https://www.jsonrpc.org/specification)
- [gRPC Documentation](https://grpc.io/docs/)
- [Protocol Buffers](https://developers.google.com/protocol-buffers)

### Security Standards

- [RFC 8446 - TLS 1.3](https://tools.ietf.org/html/rfc8446)
- [RFC 7519 - JSON Web Token (JWT)](https://tools.ietf.org/html/rfc7519)
- [OAuth 2.0 Authorization Framework](https://tools.ietf.org/html/rfc6749)

### Blockchain Standards

- [Ethereum JSON-RPC Specification](https://ethereum.org/en/developers/docs/apis/json-rpc/)
- [EIP-1193 - Ethereum Provider JavaScript API](https://eips.ethereum.org/EIPS/eip-1193)

---

<div align="center">

**WIA Carbon Credit Micro Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
