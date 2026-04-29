# WIA-DEFI Specification - PHASE 2
# API Specifications & Data Models

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [API Overview](#api-overview)
2. [RESTful API Design](#restful-api-design)
3. [GraphQL API](#graphql-api)
4. [WebSocket Streaming](#websocket-streaming)
5. [Data Models](#data-models)
6. [Code Examples](#code-examples)

---

## API Overview

### API Architecture

The WIA-DEFI standard supports multiple API paradigms:

```
┌──────────────────────────────────────────┐
│         Client Applications              │
├──────────────────────────────────────────┤
│  REST API │ GraphQL │ WebSocket │ JSON-RPC │
├──────────────────────────────────────────┤
│         API Gateway / Load Balancer       │
├──────────────────────────────────────────┤
│     The Graph (Indexer) │ Direct RPC     │
├──────────────────────────────────────────┤
│         Blockchain (Ethereum/L2s)        │
└──────────────────────────────────────────┘
```

### Base URLs

**Mainnet:**
```
REST:      https://api.wia-defi.org/v1
GraphQL:   https://api.wia-defi.org/graphql
WebSocket: wss://stream.wia-defi.org/v1
```

**Testnet (Sepolia):**
```
REST:      https://testnet.api.wia-defi.org/v1
GraphQL:   https://testnet.api.wia-defi.org/graphql
WebSocket: wss://testnet.stream.wia-defi.org/v1
```

---

## RESTful API Design

### Authentication

**API Key Authentication:**
```http
GET /v1/protocols
Authorization: Bearer YOUR_API_KEY
```

**Rate Limits:**
- Free Tier: 100 requests/minute
- Pro Tier: 1000 requests/minute
- Enterprise: Custom limits

### Endpoints

#### 1. Get Protocol Information

```http
GET /v1/protocols/{protocolId}
```

**Request:**
```bash
curl -X GET "https://api.wia-defi.org/v1/protocols/aave-v4" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response (200 OK):**
```json
{
  "protocolId": "aave-v4",
  "name": "Aave V4",
  "version": "4.0.0",
  "chainId": 1,
  "tvl": "15432100000.50",
  "tvlChange24h": "2.34",
  "totalBorrowed": "8234500000.25",
  "totalSupplied": "15432100000.50",
  "utilization": "53.37",
  "markets": 42,
  "activeUsers24h": 12453,
  "volume24h": "456789012.34",
  "deployedAt": "2026-03-15T00:00:00Z",
  "contracts": {
    "pool": "0x87870Bca3F3fD6335C3F4ce8392D69350B4fA4E2",
    "poolDataProvider": "0x7B4EB56E7CD4b454BA8ff71E4518426369a138a3",
    "oracle": "0x54586bE62E3c3580375aE3723C145253060Ca0C2"
  },
  "links": {
    "website": "https://aave.com",
    "docs": "https://docs.aave.com",
    "github": "https://github.com/aave/aave-v4"
  }
}
```

#### 2. Get Liquidity Pools

```http
GET /v1/pools
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| protocol | string | Filter by protocol (e.g., "uniswap-v4") |
| chain | integer | Chain ID (1 = Ethereum, 42161 = Arbitrum) |
| token0 | string | First token address |
| token1 | string | Second token address |
| minTvl | number | Minimum TVL in USD |
| sortBy | string | Sort field (tvl, volume24h, apr) |
| order | string | asc or desc |
| limit | integer | Results per page (max 100) |
| offset | integer | Pagination offset |

**Request:**
```bash
curl -X GET "https://api.wia-defi.org/v1/pools?\
protocol=uniswap-v4&\
chain=1&\
minTvl=1000000&\
sortBy=volume24h&\
order=desc&\
limit=10" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response (200 OK):**
```json
{
  "total": 1547,
  "limit": 10,
  "offset": 0,
  "pools": [
    {
      "poolId": "0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640",
      "protocol": "uniswap-v4",
      "version": "4.0",
      "chain": 1,
      "token0": {
        "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
        "symbol": "USDC",
        "name": "USD Coin",
        "decimals": 6,
        "priceUsd": "1.0002"
      },
      "token1": {
        "address": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
        "symbol": "WETH",
        "name": "Wrapped Ether",
        "decimals": 18,
        "priceUsd": "3245.67"
      },
      "reserve0": "125000000.000000",
      "reserve1": "38500.123456789012345678",
      "tvl": "250123456.78",
      "volume24h": "45678901.23",
      "fees24h": "13703.67",
      "apr": "21.87",
      "feeTier": "0.05",
      "liquidity": "125061728.39",
      "sqrtPriceX96": "1987234567890123456789012345678901",
      "tick": -276324,
      "hooks": ["0x1234...5678"],
      "createdAt": "2026-04-01T12:00:00Z"
    }
  ]
}
```

#### 3. Get User Positions

```http
GET /v1/users/{address}/positions
```

**Request:**
```bash
curl -X GET "https://api.wia-defi.org/v1/users/0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb/positions" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response (200 OK):**
```json
{
  "address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "totalValueUsd": "1250000.45",
  "positions": [
    {
      "positionId": "lp-12345",
      "type": "liquidity",
      "protocol": "uniswap-v4",
      "poolId": "0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640",
      "token0": "USDC",
      "token1": "WETH",
      "liquidity": "50000.123456",
      "amount0": "25000.000000",
      "amount1": "7.692307692307692308",
      "valueUsd": "50000.45",
      "unclaimedFees0": "12.345678",
      "unclaimedFees1": "0.003801",
      "unclaimedFeesUsd": "24.67",
      "tickLower": -276400,
      "tickUpper": -276200,
      "inRange": true
    },
    {
      "positionId": "lend-67890",
      "type": "lending",
      "protocol": "aave-v4",
      "asset": "USDC",
      "supplied": "1000000.000000",
      "suppliedUsd": "1000100.00",
      "borrowed": "0",
      "borrowedUsd": "0",
      "apy": "4.23",
      "healthFactor": null,
      "rewards": [
        {
          "token": "AAVE",
          "amount": "1.234567890123456789",
          "valueUsd": "123.45"
        }
      ]
    },
    {
      "positionId": "stake-24680",
      "type": "staking",
      "protocol": "lido",
      "asset": "ETH",
      "staked": "61.234567890123456789",
      "stakedUsd": "198765.43",
      "rewards": "0.123456789012345678",
      "rewardsUsd": "401.23",
      "apr": "3.87"
    }
  ]
}
```

#### 4. Execute Swap (Simulation)

```http
POST /v1/swap/quote
```

**Request:**
```bash
curl -X POST "https://api.wia-defi.org/v1/swap/quote" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "chainId": 1,
    "tokenIn": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "tokenOut": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
    "amountIn": "1000000000",
    "slippageTolerance": "0.5",
    "recipient": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
  }'
```

**Response (200 OK):**
```json
{
  "quoteId": "quote-abc123def456",
  "chainId": 1,
  "tokenIn": {
    "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "symbol": "USDC",
    "decimals": 6
  },
  "tokenOut": {
    "address": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
    "symbol": "WETH",
    "decimals": 18
  },
  "amountIn": "1000.000000",
  "amountOut": "0.307890123456789012",
  "priceImpact": "0.12",
  "route": [
    {
      "protocol": "uniswap-v4",
      "poolId": "0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640",
      "percentage": 100
    }
  ],
  "estimatedGas": "150000",
  "gasPriceGwei": "25",
  "gasCostUsd": "12.18",
  "expiresAt": "2026-01-12T10:35:00Z",
  "transaction": {
    "to": "0x68b3465833fb72A70ecDF485E0e4C7bD8665Fc45",
    "data": "0x...",
    "value": "0"
  }
}
```

---

## GraphQL API

### Schema Definition

```graphql
type Protocol {
  id: ID!
  name: String!
  version: String!
  chainId: Int!
  tvl: BigDecimal!
  tvlChange24h: Float!
  totalBorrowed: BigDecimal!
  totalSupplied: BigDecimal!
  utilization: Float!
  markets: [Market!]!
  volume24h: BigDecimal!
  fees24h: BigDecimal!
  createdAt: DateTime!
}

type Pool {
  id: ID!
  protocol: Protocol!
  token0: Token!
  token1: Token!
  reserve0: BigDecimal!
  reserve1: BigDecimal!
  tvl: BigDecimal!
  volume24h: BigDecimal!
  fees24h: BigDecimal!
  apr: Float!
  feeTier: Float!
  liquidity: BigDecimal!
  tick: Int!
  sqrtPriceX96: BigInt!
  swaps(first: Int, skip: Int, orderBy: String): [Swap!]!
  positions(first: Int, skip: Int): [Position!]!
}

type Token {
  id: ID!
  address: String!
  symbol: String!
  name: String!
  decimals: Int!
  priceUsd: BigDecimal!
  volume24h: BigDecimal!
  totalSupply: BigDecimal!
}

type Position {
  id: ID!
  owner: String!
  pool: Pool!
  liquidity: BigDecimal!
  amount0: BigDecimal!
  amount1: BigDecimal!
  valueUsd: BigDecimal!
  tickLower: Int!
  tickUpper: Int!
  unclaimedFees0: BigDecimal!
  unclaimedFees1: BigDecimal!
  createdAt: DateTime!
}

type Swap {
  id: ID!
  pool: Pool!
  sender: String!
  recipient: String!
  amount0: BigDecimal!
  amount1: BigDecimal!
  amountUsd: BigDecimal!
  sqrtPriceX96: BigInt!
  tick: Int!
  timestamp: DateTime!
  transaction: String!
}

type Query {
  protocol(id: ID!): Protocol
  protocols(first: Int, skip: Int, orderBy: String): [Protocol!]!

  pool(id: ID!): Pool
  pools(
    first: Int
    skip: Int
    where: PoolFilter
    orderBy: String
  ): [Pool!]!

  token(id: ID!): Token
  tokens(first: Int, skip: Int): [Token!]!

  position(id: ID!): Position
  userPositions(owner: String!, first: Int, skip: Int): [Position!]!

  swaps(first: Int, skip: Int, where: SwapFilter): [Swap!]!
}

input PoolFilter {
  protocol: String
  token0: String
  token1: String
  tvl_gte: BigDecimal
}

input SwapFilter {
  pool: String
  sender: String
  amountUsd_gte: BigDecimal
  timestamp_gte: DateTime
}
```

### Query Examples

#### Get Top Pools by Volume

```graphql
query TopPoolsByVolume {
  pools(
    first: 10
    orderBy: "volume24h"
    orderDirection: desc
    where: { tvl_gte: "1000000" }
  ) {
    id
    token0 {
      symbol
      priceUsd
    }
    token1 {
      symbol
      priceUsd
    }
    tvl
    volume24h
    apr
    feeTier
  }
}
```

#### Get User Positions with Details

```graphql
query UserPositions($owner: String!) {
  userPositions(owner: $owner, first: 20) {
    id
    pool {
      token0 { symbol }
      token1 { symbol }
      apr
    }
    liquidity
    valueUsd
    unclaimedFees0
    unclaimedFees1
    tickLower
    tickUpper
    createdAt
  }
}
```

---

## WebSocket Streaming

### Connection

```javascript
const ws = new WebSocket('wss://stream.wia-defi.org/v1?apiKey=YOUR_API_KEY');

ws.onopen = () => {
  console.log('Connected');
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

### Subscribe to Pool Updates

```json
{
  "type": "subscribe",
  "channel": "pools",
  "params": {
    "poolIds": ["0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640"]
  }
}
```

**Stream Response:**
```json
{
  "type": "pool_update",
  "timestamp": "2026-01-12T10:30:45.123Z",
  "data": {
    "poolId": "0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640",
    "reserve0": "125001234.567890",
    "reserve1": "38501.234567890123456789",
    "sqrtPriceX96": "1987234567890123456789012345679",
    "tick": -276323
  }
}
```

### Subscribe to Swaps

```json
{
  "type": "subscribe",
  "channel": "swaps",
  "params": {
    "poolIds": ["0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640"],
    "minAmountUsd": "10000"
  }
}
```

---

## Data Models

### TypeScript Interfaces

```typescript
interface Protocol {
  protocolId: string;
  name: string;
  version: string;
  chainId: number;
  tvl: bigint;
  tvlChange24h: number;
  totalBorrowed: bigint;
  totalSupplied: bigint;
  utilization: number;
  markets: number;
  activeUsers24h: number;
  volume24h: bigint;
  deployedAt: Date;
  contracts: {
    pool: string;
    router?: string;
    factory?: string;
    oracle?: string;
  };
}

interface LiquidityPool {
  poolId: string;
  protocol: string;
  chainId: number;
  token0: Token;
  token1: Token;
  reserve0: bigint;
  reserve1: bigint;
  tvl: bigint;
  volume24h: bigint;
  fees24h: bigint;
  apr: number;
  feeTier: number;
  liquidity: bigint;
  sqrtPriceX96: bigint;
  tick: number;
  hooks?: string[];
  createdAt: Date;
}

interface Token {
  address: string;
  symbol: string;
  name: string;
  decimals: number;
  priceUsd: number;
  volume24h?: bigint;
  totalSupply?: bigint;
}

interface Position {
  positionId: string;
  type: 'liquidity' | 'lending' | 'staking';
  protocol: string;
  owner: string;
  valueUsd: number;
  createdAt: Date;
}

interface LiquidityPosition extends Position {
  type: 'liquidity';
  poolId: string;
  token0: string;
  token1: string;
  liquidity: bigint;
  amount0: bigint;
  amount1: bigint;
  tickLower: number;
  tickUpper: number;
  unclaimedFees0: bigint;
  unclaimedFees1: bigint;
  inRange: boolean;
}

interface LendingPosition extends Position {
  type: 'lending';
  asset: string;
  supplied: bigint;
  borrowed: bigint;
  apy: number;
  healthFactor?: number;
  rewards?: Reward[];
}

interface StakingPosition extends Position {
  type: 'staking';
  asset: string;
  staked: bigint;
  rewards: bigint;
  apr: number;
}

interface Reward {
  token: string;
  amount: bigint;
  valueUsd: number;
}

interface SwapQuote {
  quoteId: string;
  chainId: number;
  tokenIn: Token;
  tokenOut: Token;
  amountIn: bigint;
  amountOut: bigint;
  priceImpact: number;
  route: RouteStep[];
  estimatedGas: number;
  gasPriceGwei: number;
  gasCostUsd: number;
  expiresAt: Date;
  transaction: {
    to: string;
    data: string;
    value: string;
  };
}

interface RouteStep {
  protocol: string;
  poolId: string;
  percentage: number;
}
```

---

## Code Examples

### JavaScript/TypeScript SDK

```typescript
import { WIADeFiSDK } from '@wia/defi-sdk';

// Initialize SDK
const sdk = new WIADeFiSDK({
  apiKey: 'YOUR_API_KEY',
  network: 'mainnet', // or 'sepolia'
});

// Get protocol information
const aave = await sdk.protocols.get('aave-v4');
console.log(`Aave TVL: $${aave.tvl}`);

// Get top pools
const pools = await sdk.pools.list({
  protocol: 'uniswap-v4',
  sortBy: 'volume24h',
  order: 'desc',
  limit: 10,
});

pools.forEach(pool => {
  console.log(`${pool.token0.symbol}/${pool.token1.symbol}: $${pool.volume24h}`);
});

// Get user positions
const positions = await sdk.users.getPositions('0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb');
const totalValue = positions.reduce((sum, pos) => sum + pos.valueUsd, 0);
console.log(`Total Portfolio Value: $${totalValue}`);

// Get swap quote
const quote = await sdk.swap.quote({
  chainId: 1,
  tokenIn: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48', // USDC
  tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2', // WETH
  amountIn: '1000000000', // 1000 USDC
  slippageTolerance: 0.5,
});
console.log(`Expected output: ${quote.amountOut} WETH`);

// Stream pool updates
sdk.stream.subscribe('pools', {
  poolIds: ['0x88e6a0c2ddd26feeb64f039a2c41296fcb3f5640'],
  onUpdate: (data) => {
    console.log('Pool updated:', data);
  },
});
```

### Python SDK

```python
from wia_defi import WIADeFiSDK

# Initialize SDK
sdk = WIADeFiSDK(api_key='YOUR_API_KEY', network='mainnet')

# Get protocol
aave = sdk.protocols.get('aave-v4')
print(f"Aave TVL: ${aave.tvl}")

# Get pools
pools = sdk.pools.list(
    protocol='uniswap-v4',
    sort_by='volume24h',
    order='desc',
    limit=10
)

for pool in pools:
    print(f"{pool.token0.symbol}/{pool.token1.symbol}: ${pool.volume24h}")

# Get user positions
positions = sdk.users.get_positions('0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb')
total_value = sum(pos.value_usd for pos in positions)
print(f"Total Portfolio Value: ${total_value}")

# Get swap quote
quote = sdk.swap.quote(
    chain_id=1,
    token_in='0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48',
    token_out='0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
    amount_in='1000000000',
    slippage_tolerance=0.5
)
print(f"Expected output: {quote.amount_out} WETH")
```

---

## Error Handling

### HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 400 | Bad Request (invalid parameters) |
| 401 | Unauthorized (invalid API key) |
| 403 | Forbidden (rate limit exceeded) |
| 404 | Not Found |
| 429 | Too Many Requests |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

### Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETERS",
    "message": "Invalid token address format",
    "details": {
      "field": "tokenIn",
      "value": "0xinvalid",
      "expected": "Valid Ethereum address (0x + 40 hex characters)"
    }
  }
}
```

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (Hongik Ingan) - Benefit All Humanity

Licensed under MIT License
