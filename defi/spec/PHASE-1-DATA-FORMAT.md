# WIA DeFi Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Standard:** WIA-FIN-006
**Status:** Final
**Last Updated:** December 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Liquidity Pool Schema](#liquidity-pool-schema)
3. [Token Metadata](#token-metadata)
4. [AMM Data Structures](#amm-data-structures)
5. [Lending Protocol Format](#lending-protocol-format)
6. [Yield Farming Records](#yield-farming-records)
7. [Flash Loan Transactions](#flash-loan-transactions)
8. [Governance Data](#governance-data)
9. [Oracle Price Feeds](#oracle-price-feeds)
10. [Validation Rules](#validation-rules)
11. [Examples](#examples)

---

## Overview

The WIA DeFi Standard defines universal data formats for decentralized finance protocols, ensuring interoperability across AMMs, lending platforms, yield aggregators, and governance systems. This specification supports Ethereum, BSC, Polygon, Arbitrum, Optimism, and other EVM-compatible chains.

### Key Principles

- **Protocol Agnostic**: Works with Uniswap, Aave, Compound, Curve, Balancer, and others
- **Chain Neutral**: Support for multi-chain DeFi ecosystems
- **Composability**: Enable money lego strategies
- **Real-time**: Support for live TVL, APY, and price data
- **Security**: Built-in validation and audit trails

### Philosophy: 弘益人間 (Hongik Ingan)

> "Benefit All Humanity"

The WIA DeFi Standard democratizes access to decentralized finance by creating universal standards that enable seamless integration, reduce technical barriers, and promote financial inclusion globally.

---

## Liquidity Pool Schema

### Base Pool Structure

All liquidity pools conform to this universal format:

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "poolType": "AMM|Lending|Staking|Farming",
  "poolId": "string (unique identifier)",
  "protocol": "Uniswap|Aave|Compound|Curve|etc",
  "chain": {
    "name": "Ethereum|BSC|Polygon|Arbitrum|Optimism",
    "chainId": "number",
    "network": "mainnet|testnet"
  },
  "contractAddress": "0x...",
  "tokens": [],
  "metrics": {},
  "timestamp": "ISO-8601 timestamp",
  "metadata": {}
}
```

### Uniswap V2 Pool Example

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "poolType": "AMM",
  "poolId": "uniswap-v2-eth-usdc-001",
  "protocol": "Uniswap V2",
  "chain": {
    "name": "Ethereum",
    "chainId": 1,
    "network": "mainnet"
  },
  "contractAddress": "0xB4e16d0168e52d35CaCD2c6185b44281Ec28C9Dc",
  "tokens": [
    {
      "address": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "name": "Wrapped Ether",
      "decimals": 18,
      "reserve": "45234.567891234567891234",
      "weight": 0.5
    },
    {
      "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
      "symbol": "USDC",
      "name": "USD Coin",
      "decimals": 6,
      "reserve": "123456789.123456",
      "weight": 0.5
    }
  ],
  "metrics": {
    "tvl": 246913579.13,
    "tvlChange24h": 2.34,
    "volume24h": 45678912.34,
    "volume7d": 289456123.67,
    "fees24h": 137036.74,
    "feeRate": 0.003,
    "apy": 18.45,
    "apr": 17.23,
    "lpTokenSupply": "1234567.891234567891234567",
    "lpTokenPrice": 200.15
  },
  "timestamp": "2025-12-25T10:00:00Z",
  "metadata": {
    "createdAt": "2020-05-05T12:00:00Z",
    "createdBlock": 10000835,
    "lastUpdated": "2025-12-25T10:00:00Z",
    "audits": [
      {
        "auditor": "Trail of Bits",
        "date": "2020-04-15",
        "url": "https://example.com/audit"
      }
    ]
  }
}
```

### Uniswap V3 Concentrated Liquidity Pool

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "poolType": "AMM",
  "poolId": "uniswap-v3-eth-usdc-005",
  "protocol": "Uniswap V3",
  "chain": {
    "name": "Ethereum",
    "chainId": 1,
    "network": "mainnet"
  },
  "contractAddress": "0x88e6A0c2dDD26FEEb64F039a2c41296FcB3f5640",
  "feeRateParams": {
    "fee": 0.0005,
    "tickSpacing": 10
  },
  "tokens": [
    {
      "address": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "name": "Wrapped Ether",
      "decimals": 18,
      "reserve": "34567.891234567891234567"
    },
    {
      "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
      "symbol": "USDC",
      "name": "USD Coin",
      "decimals": 6,
      "reserve": "94567123.456789"
    }
  ],
  "liquidityDistribution": {
    "currentTick": 202345,
    "currentPrice": 2734.56,
    "tickLiquidity": [
      {
        "tickLower": 202300,
        "tickUpper": 202400,
        "liquidity": "12345678901234567890"
      }
    ]
  },
  "metrics": {
    "tvl": 189123456.78,
    "volume24h": 67891234.56,
    "fees24h": 33945.62,
    "feeRate": 0.0005,
    "apy": 24.67,
    "lpTokenSupply": "N/A (NFT positions)",
    "activePositions": 4567
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### Curve Stable Swap Pool

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "poolType": "AMM",
  "poolId": "curve-3pool-001",
  "protocol": "Curve",
  "chain": {
    "name": "Ethereum",
    "chainId": 1,
    "network": "mainnet"
  },
  "contractAddress": "0xbEbc44782C7dB0a1A60Cb6fe97d0b483032FF1C7",
  "ammType": "StableSwap",
  "tokens": [
    {
      "address": "0x6B175474E89094C44Da98b954EedeAC495271d0F",
      "symbol": "DAI",
      "decimals": 18,
      "reserve": "456789123.456789123456789123"
    },
    {
      "address": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
      "symbol": "USDC",
      "decimals": 6,
      "reserve": "512345678.901234"
    },
    {
      "address": "0xdAC17F958D2ee523a2206206994597C13D831ec7",
      "symbol": "USDT",
      "decimals": 6,
      "reserve": "489012345.678901"
    }
  ],
  "curveParameters": {
    "A": 2000,
    "fee": 0.0004,
    "adminFee": 0.5
  },
  "metrics": {
    "tvl": 1458147147.04,
    "volume24h": 123456789.12,
    "fees24h": 49382.72,
    "apy": 3.45,
    "virtualPrice": 1.023456
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Token Metadata

### ERC-20 Token Standard

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "tokenStandard": "ERC-20|ERC-721|ERC-1155",
  "address": "0x...",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "symbol": "string",
  "name": "string",
  "decimals": 18,
  "totalSupply": "string (wei representation)",
  "circulatingSupply": "string",
  "maxSupply": "string (optional)",
  "price": {
    "usd": 1234.56,
    "btc": 0.0234,
    "eth": 0.456,
    "change24h": 5.67,
    "change7d": -2.34,
    "marketCap": 12345678901.23,
    "volume24h": 2345678901.34
  },
  "holders": 123456,
  "transfers24h": 45678,
  "metadata": {
    "website": "https://example.com",
    "whitepaper": "https://example.com/whitepaper.pdf",
    "social": {
      "twitter": "@token",
      "discord": "https://discord.gg/token"
    },
    "coingeckoId": "token-name",
    "coinmarketcapId": "12345"
  }
}
```

---

## AMM Data Structures

### Constant Product Formula (x * y = k)

```json
{
  "ammType": "ConstantProduct",
  "formula": "x * y = k",
  "parameters": {
    "reserveX": "1000000.0",
    "reserveY": "2000000.0",
    "k": "2000000000000.0",
    "feeRate": 0.003
  },
  "swapCalculation": {
    "inputToken": "tokenX",
    "inputAmount": "1000.0",
    "outputToken": "tokenY",
    "outputAmount": "1994.017982",
    "priceImpact": 0.299,
    "minimumReceived": "1974.077942",
    "slippageTolerance": 0.01,
    "fee": "3.0"
  }
}
```

### Concentrated Liquidity (Uniswap V3)

```json
{
  "ammType": "ConcentratedLiquidity",
  "formula": "L = sqrt(x * y)",
  "position": {
    "positionId": "123456",
    "owner": "0x...",
    "tickLower": 200000,
    "tickUpper": 210000,
    "priceLower": 2500.0,
    "priceUpper": 3500.0,
    "liquidity": "12345678901234567890",
    "token0Amount": "10.5",
    "token1Amount": "28765.43",
    "inRange": true,
    "feesEarned0": "0.234",
    "feesEarned1": "641.23"
  }
}
```

### Stable Swap (Curve)

```json
{
  "ammType": "StableSwap",
  "formula": "A * n^n * sum(x_i) + D = A * D * n^n + D^(n+1) / (n^n * prod(x_i))",
  "parameters": {
    "A": 2000,
    "n": 3,
    "D": "1458147147.04",
    "reserves": [
      "456789123.456789",
      "512345678.901234",
      "489012345.678901"
    ]
  },
  "swapCalculation": {
    "inputToken": 0,
    "inputAmount": "10000.0",
    "outputToken": 1,
    "outputAmount": "9996.234567",
    "priceImpact": 0.004,
    "fee": "4.0"
  }
}
```

---

## Lending Protocol Format

### Aave V3 Market

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "protocolType": "Lending",
  "protocol": "Aave V3",
  "marketId": "aave-v3-ethereum-main",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "assets": [
    {
      "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "aToken": "0x4d5F47FA6A74757f35C14fD3a6Ef8E3C9BC514E8",
      "stableDebtToken": "0x...",
      "variableDebtToken": "0x...",
      "totalSupplied": "1234567.891234567891234567",
      "totalBorrowed": "987654.321098765432109876",
      "supplyAPY": 2.34,
      "variableBorrowAPY": 3.45,
      "stableBorrowAPY": 4.56,
      "utilizationRate": 0.80,
      "liquidityRate": 1.87,
      "availableLiquidity": "246913.570135802469135802",
      "ltv": 0.825,
      "liquidationThreshold": 0.86,
      "liquidationBonus": 0.05,
      "reserveFactor": 0.15,
      "isActive": true,
      "isFrozen": false,
      "borrowingEnabled": true,
      "stableBorrowingEnabled": false
    }
  ],
  "totalMarketSize": 45678912345.67,
  "totalBorrowed": 23456789123.45,
  "totalAvailable": 22222123222.22,
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### Compound V3 Market

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "protocolType": "Lending",
  "protocol": "Compound V3",
  "marketId": "compound-v3-usdc",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "baseAsset": {
    "asset": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
    "symbol": "USDC",
    "totalSupply": "1234567890.123456",
    "totalBorrow": "987654321.098765",
    "supplyAPR": 3.45,
    "borrowAPR": 5.67,
    "utilization": 0.80,
    "collateralFactor": 0.0
  },
  "collateralAssets": [
    {
      "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "collateralFactor": 0.83,
      "liquidationFactor": 0.88,
      "liquidationPenalty": 0.07,
      "supplyCap": "100000.0"
    }
  ],
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### User Position

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "positionType": "Lending",
  "user": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "protocol": "Aave V3",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "supplied": [
    {
      "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "amount": "100.5",
      "amountUSD": 274549.2,
      "apy": 2.34,
      "isCollateral": true
    }
  ],
  "borrowed": [
    {
      "asset": "0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48",
      "symbol": "USDC",
      "amount": "150000.0",
      "amountUSD": 150000.0,
      "apy": 3.45,
      "rateMode": "variable"
    }
  ],
  "healthFactor": 1.65,
  "totalCollateralUSD": 274549.2,
  "totalBorrowedUSD": 150000.0,
  "availableBorrowsUSD": 76822.41,
  "currentLTV": 0.5465,
  "liquidationThreshold": 0.86,
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Yield Farming Records

### Farm Pool

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "farmType": "LiquidityMining|SingleStaking|VaultStrategy",
  "farmId": "sushiswap-eth-usdc-farm",
  "protocol": "SushiSwap",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "stakingToken": {
    "address": "0xB4e16d0168e52d35CaCD2c6185b44281Ec28C9Dc",
    "symbol": "SLP",
    "name": "Sushi LP Token",
    "type": "LP"
  },
  "rewardTokens": [
    {
      "address": "0x6B3595068778DD592e39A122f4f5a5cF09C90fE2",
      "symbol": "SUSHI",
      "emissionRate": "100.0",
      "emissionPeriod": "per_block"
    }
  ],
  "metrics": {
    "totalStaked": "12345.678901234567891234",
    "totalStakedUSD": 2469135.79,
    "apy": 123.45,
    "apr": 89.67,
    "dailyRewards": "8640.0",
    "tvl": 2469135.79
  },
  "duration": {
    "startBlock": 12345678,
    "endBlock": 14567890,
    "startTime": "2025-01-01T00:00:00Z",
    "endTime": "2025-12-31T23:59:59Z"
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### User Farm Position

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "positionType": "YieldFarming",
  "user": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "farmId": "sushiswap-eth-usdc-farm",
  "protocol": "SushiSwap",
  "staked": {
    "amount": "100.5",
    "amountUSD": 20115.67,
    "stakedAt": "2025-06-15T10:00:00Z",
    "stakedBlock": 13456789
  },
  "rewards": {
    "pending": [
      {
        "token": "SUSHI",
        "amount": "234.567891234567891234",
        "amountUSD": 456.78
      }
    ],
    "claimed": [
      {
        "token": "SUSHI",
        "amount": "1234.567",
        "amountUSD": 2400.12,
        "claimedAt": "2025-12-01T10:00:00Z"
      }
    ]
  },
  "performance": {
    "apy": 123.45,
    "dailyReward": "5.67",
    "projectedYearlyReward": "2069.55"
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Flash Loan Transactions

### Flash Loan Request

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "transactionType": "FlashLoan",
  "protocol": "Aave V3",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "borrower": "0x...",
  "loans": [
    {
      "asset": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2",
      "symbol": "WETH",
      "amount": "1000.0",
      "fee": "0.9",
      "feeRate": 0.0009,
      "repayAmount": "1000.9"
    }
  ],
  "execution": {
    "mode": "0",
    "target": "0x...",
    "params": "0x...",
    "operations": [
      "Borrow 1000 WETH",
      "Swap on Uniswap",
      "Swap on SushiSwap",
      "Repay 1000.9 WETH",
      "Keep profit"
    ]
  },
  "result": {
    "success": true,
    "gasUsed": 345678,
    "gasPrice": "50000000000",
    "profit": {
      "asset": "WETH",
      "amount": "5.67",
      "amountUSD": 15487.92
    }
  },
  "transactionHash": "0x...",
  "blockNumber": 18456789,
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Governance Data

### Governance Proposal

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "governanceType": "DAO",
  "protocol": "Uniswap",
  "proposalId": "123",
  "title": "Reduce protocol fee to 0.05%",
  "description": "Proposal to reduce the protocol fee from 0.30% to 0.05% to increase competitiveness...",
  "proposer": "0x...",
  "status": "Active|Pending|Succeeded|Defeated|Queued|Executed|Canceled|Expired",
  "voting": {
    "startBlock": 18400000,
    "endBlock": 18500000,
    "startTime": "2025-12-20T00:00:00Z",
    "endTime": "2025-12-27T00:00:00Z",
    "quorum": "40000000.0",
    "votesFor": "45678901.234567",
    "votesAgainst": "12345678.901234",
    "votesAbstain": "1234567.890123",
    "totalVotes": "59259148.025924",
    "quorumReached": true,
    "currentResult": "For"
  },
  "execution": {
    "eta": "2025-12-29T00:00:00Z",
    "targets": ["0x..."],
    "values": ["0"],
    "signatures": ["setProtocolFee(uint256)"],
    "calldatas": ["0x..."]
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### User Vote

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "voteType": "GovernanceVote",
  "proposalId": "123",
  "voter": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "support": "For|Against|Abstain",
  "votingPower": "12345.678901234567891234",
  "reason": "I support this proposal because...",
  "transactionHash": "0x...",
  "blockNumber": 18456789,
  "timestamp": "2025-12-22T10:00:00Z"
}
```

---

## Oracle Price Feeds

### Chainlink Price Feed

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "oracleType": "Chainlink",
  "feedAddress": "0x5f4eC3Df9cbd43714FE2740f5E3616155c5b8419",
  "pair": "ETH/USD",
  "chain": {
    "name": "Ethereum",
    "chainId": 1
  },
  "latestRound": {
    "roundId": "18446744073709563891",
    "answer": "273456000000",
    "decimals": 8,
    "price": 2734.56,
    "updatedAt": "2025-12-25T09:59:45Z",
    "answeredInRound": "18446744073709563891"
  },
  "aggregator": "0x...",
  "heartbeat": 3600,
  "deviation": 0.5,
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### TWAP (Time-Weighted Average Price)

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "oracleType": "TWAP",
  "source": "Uniswap V3",
  "pair": "WETH/USDC",
  "poolAddress": "0x88e6A0c2dDD26FEEb64F039a2c41296FcB3f5640",
  "observations": [
    {
      "timestamp": "2025-12-25T10:00:00Z",
      "tickCumulative": "12345678901234567890",
      "secondsPerLiquidityCumulative": "98765432109876543210"
    }
  ],
  "twapPrice": {
    "period": 1800,
    "price": 2734.23,
    "token0": "WETH",
    "token1": "USDC"
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Validation Rules

### Pool Validation

```javascript
function validatePool(pool) {
  // Required fields
  if (!pool.version || !pool.standard || !pool.poolId) {
    throw new Error('Missing required fields');
  }

  // Standard version
  if (pool.standard !== 'WIA-FIN-006') {
    throw new Error('Invalid standard');
  }

  // Token validation
  if (!Array.isArray(pool.tokens) || pool.tokens.length < 2) {
    throw new Error('Pool must have at least 2 tokens');
  }

  // TVL must be positive
  if (pool.metrics.tvl <= 0) {
    throw new Error('TVL must be positive');
  }

  // Fee rate must be between 0 and 1
  if (pool.metrics.feeRate < 0 || pool.metrics.feeRate > 1) {
    throw new Error('Invalid fee rate');
  }

  return true;
}
```

### Health Factor Calculation

```javascript
function calculateHealthFactor(position) {
  const totalCollateralETH = position.totalCollateralUSD / ethPrice;
  const totalBorrowETH = position.totalBorrowedUSD / ethPrice;
  const avgLiquidationThreshold = position.liquidationThreshold;

  const healthFactor = (totalCollateralETH * avgLiquidationThreshold) / totalBorrowETH;

  return healthFactor;
}
```

### Impermanent Loss Calculation

```javascript
function calculateImpermanentLoss(priceRatio) {
  // IL = 2 * sqrt(priceRatio) / (1 + priceRatio) - 1
  const il = 2 * Math.sqrt(priceRatio) / (1 + priceRatio) - 1;
  return il * 100; // Return as percentage
}

// Example:
// If ETH price doubles (priceRatio = 2):
// IL = 2 * sqrt(2) / (1 + 2) - 1 = -5.72%
```

---

## Examples

### Complete DeFi Position

```json
{
  "version": "1.0.0",
  "standard": "WIA-FIN-006",
  "user": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "positions": [
    {
      "type": "AMM",
      "protocol": "Uniswap V3",
      "poolId": "uniswap-v3-eth-usdc-005",
      "liquidity": "12345678901234567890",
      "valueUSD": 50000.0,
      "feesEarnedUSD": 234.56
    },
    {
      "type": "Lending",
      "protocol": "Aave V3",
      "supplied": {
        "WETH": {
          "amount": "100.5",
          "valueUSD": 274549.2
        }
      },
      "borrowed": {
        "USDC": {
          "amount": "150000.0",
          "valueUSD": 150000.0
        }
      },
      "healthFactor": 1.65
    },
    {
      "type": "YieldFarming",
      "protocol": "Convex",
      "staked": {
        "cvxCRV": {
          "amount": "10000.0",
          "valueUSD": 8500.0
        }
      },
      "apy": 45.67,
      "pendingRewardsUSD": 123.45
    }
  ],
  "summary": {
    "totalValueLocked": 483048.2,
    "totalDebt": 150000.0,
    "netWorth": 333048.2,
    "yieldAPY": 28.34
  },
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## Appendix

### Supported Protocols

| Protocol | Type | Chains |
|----------|------|--------|
| Uniswap V2/V3 | AMM | Ethereum, Polygon, Arbitrum, Optimism |
| SushiSwap | AMM | Multi-chain |
| Curve | Stable AMM | Ethereum, Polygon, Arbitrum |
| Balancer | Weighted AMM | Ethereum, Polygon, Arbitrum |
| Aave V2/V3 | Lending | Multi-chain |
| Compound V2/V3 | Lending | Ethereum |
| MakerDAO | CDP | Ethereum |
| Convex | Yield Aggregator | Ethereum |
| Yearn | Vault | Multi-chain |

### Chain IDs

| Chain | Chain ID | RPC |
|-------|----------|-----|
| Ethereum Mainnet | 1 | https://eth.llamarpc.com |
| Polygon | 137 | https://polygon.llamarpc.com |
| Arbitrum One | 42161 | https://arb1.arbitrum.io/rpc |
| Optimism | 10 | https://mainnet.optimism.io |
| BSC | 56 | https://bsc-dataseed.binance.org |
| Avalanche | 43114 | https://api.avax.network/ext/bc/C/rpc |

---

**Version:** 1.0.0
**Last Updated:** December 2025
**Status:** Final

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
