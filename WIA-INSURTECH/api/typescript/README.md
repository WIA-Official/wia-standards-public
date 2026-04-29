# WIA-INSURTECH TypeScript SDK

**弘益人間** (Benefit All Humanity)

Comprehensive TypeScript SDK for interacting with major DeFi protocols including Uniswap v4, Aave V4, Lido v3, and more.

## Features

- **Multi-Protocol Support**: Uniswap v4, Aave V4, Lido v3, Compound, Curve, and 50+ protocols
- **AI-Powered Routing**: Optimal swap routes using AI aggregation
- **Real-time Streaming**: WebSocket support for live pool updates
- **Type-Safe**: Full TypeScript support with comprehensive type insurtechnitions
- **Cross-Chain**: Support for Ethereum, Arbitrum, Optimism, Base, and more
- **Production-Ready**: Built-in retry logic, error handling, and validation

## Installation

```bash
npm install @wia/insurtech-sdk
# or
yarn add @wia/insurtech-sdk
# or
pnpm add @wia/insurtech-sdk
```

## Quick Start

```typescript
import { WIADeFiSDK } from '@wia/insurtech-sdk';

// Initialize SDK
const sdk = new WIADeFiSDK({
  apiKey: 'your-api-key',
  network: 'mainnet'
});

// Get all protocols
const protocols = await sdk.getProtocols();
console.log(`Total TVL: ${protocols.reduce((sum, p) => sum + p.tvl, 0n)}`);

// Get swap quote
const quote = await sdk.getSwapQuote({
  chainId: 1,
  tokenIn: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48', // USDC
  tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2', // WETH
  amountIn: '1000000000', // 1000 USDC
  slippageTolerance: 0.5
});

console.log(`You will receive: ${quote.amountOut} WETH`);
console.log(`Price impact: ${quote.priceImpact}%`);
```

## Core Modules

### 1. Protocol Information

```typescript
// Get all protocols
const protocols = await sdk.getProtocols();

// Get specific protocol
const uniswap = await sdk.getProtocol('uniswap-v4');
console.log(`Uniswap v4 TVL: $${uniswap.tvl}`);
console.log(`24h Volume: $${uniswap.volume24h}`);
```

### 2. Liquidity Pools

```typescript
// Get pools with filtering
const pools = await sdk.getPools({
  protocol: 'uniswap-v4',
  minTvl: 1000000,
  sortBy: 'apr',
  order: 'desc',
  limit: 10
});

// Get specific pool
const pool = await sdk.getPool('pool-id');
console.log(`APR: ${pool.apr}%`);
console.log(`TVL: $${pool.tvl}`);
```

### 3. Swap Operations

```typescript
// Get swap quote
const quote = await sdk.getSwapQuote({
  chainId: 1,
  tokenIn: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48',
  tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
  amountIn: '1000000000',
  slippageTolerance: 0.5,
  deadline: Math.floor(Date.now() / 1000) + 1200 // 20 minutes
});

// The quote includes transaction data ready to be signed
console.log('Transaction data:', quote.transaction);

// Route details
quote.route.forEach(step => {
  console.log(`${step.protocol}: ${step.tokenIn} -> ${step.tokenOut} (${step.percentage}%)`);
});
```

### 4. User Positions

```typescript
// Get user positions across all protocols
const positions = await sdk.getUserPositions({
  address: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
  protocols: ['uniswap-v4', 'aave-v4', 'lido-v3'],
  minValueUsd: 100
});

positions.forEach(position => {
  console.log(`${position.protocol} - ${position.type}: $${position.valueUsd}`);
});
```

### 5. Lending Markets (Aave V4)

```typescript
// Get lending markets
const markets = await sdk.getLendingMarkets('aave-v4');

markets.forEach(market => {
  console.log(`${market.asset.symbol}`);
  console.log(`  Supply APY: ${market.supplyApy}%`);
  console.log(`  Borrow APY: ${market.borrowApy}%`);
  console.log(`  Utilization: ${market.utilizationRate}%`);
});

// Supply to lending protocol
const supplyTx = await sdk.supplyToLending({
  asset: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48',
  amount: 1000000000n // 1000 USDC
});

// Borrow from lending protocol
const borrowTx = await sdk.borrowFromLending({
  asset: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
  amount: 1000000000000000000n, // 1 WETH
  interestRateMode: 'variable'
});
```

### 6. Staking (Lido v3, Rocket Pool)

```typescript
// Get staking pools
const stakingPools = await sdk.getStakingPools();

stakingPools.forEach(pool => {
  console.log(`${pool.protocol} - ${pool.asset.symbol}`);
  console.log(`  APR: ${pool.apr}%`);
  console.log(`  Total Staked: ${pool.totalStaked}`);
});

// Stake ETH
const stakeTx = await sdk.stake('lido-v3-eth', 1000000000000000000n); // 1 ETH

// Unstake
const unstakeTx = await sdk.unstake('lido-v3-eth', 500000000000000000n); // 0.5 ETH
```

### 7. Real-time Streaming

```typescript
// Stream pool updates via WebSocket
const unsubscribe = sdk.streamPoolUpdates({
  poolIds: ['uniswap-v4-usdc-weth-3000'],
  minAmountUsd: 10000,
  onUpdate: (event) => {
    if (event.type === 'swap') {
      console.log('Large swap detected:', event.data);
    } else if (event.type === 'pool_update') {
      console.log('Pool reserves updated:', event.data);
    }
  },
  onError: (error) => {
    console.error('Stream error:', error);
  }
});

// Unsubscribe when done
// unsubscribe();
```

### 8. Token Information

```typescript
// Get token info
const usdc = await sdk.getToken('0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48', 1);
console.log(`${usdc.name} (${usdc.symbol})`);
console.log(`Price: $${usdc.priceUsd}`);

// Search tokens
const tokens = await sdk.searchTokens('USDC', 1);
tokens.forEach(token => {
  console.log(`${token.symbol}: ${token.address}`);
});
```

## Utility Functions

The SDK includes comprehensive utility functions:

```typescript
import {
  formatUnits,
  parseUnits,
  formatUsd,
  calculatePriceImpact,
  aprToApy,
  calculateHealthFactor
} from '@wia/insurtech-sdk';

// Format token amounts
const readable = formatUnits(1500000n, 6); // "1.5"

// Parse user input
const amount = parseUnits("1000.5", 18); // 1000500000000000000000n

// Format USD
const usd = formatUsd(1234567.89); // "$1,234,567.89"

// Calculate price impact
const impact = calculatePriceImpact(1000n, 998n, 1, 1000); // 0.2%

// Convert APR to APY
const apy = aprToApy(10); // 10.52%

// Calculate health factor
const health = calculateHealthFactor(10000, 5000, 75); // 1.5
```

## Validators

Built-in validation functions ensure data integrity:

```typescript
import {
  validateAddress,
  validateAmount,
  validateChainId
} from '@wia/insurtech-sdk';

try {
  validateAddress('0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb');
  validateAmount(1000000n);
  validateChainId(1);
} catch (error) {
  console.error('Validation failed:', error.message);
}
```

## Advanced Features

### Custom Configuration

```typescript
const sdk = new WIADeFiSDK({
  apiKey: 'your-api-key',
  network: 'mainnet',
  baseURL: 'https://custom-api.example.com/v1',
  timeout: 60000, // 60 seconds
  retries: 5
});
```

### Error Handling

```typescript
try {
  const quote = await sdk.getSwapQuote({
    chainId: 1,
    tokenIn: '0xInvalid',
    tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2',
    amountIn: '1000',
    slippageTolerance: 0.5
  });
} catch (error) {
  console.error('Error:', error.message);
  // Handle error appropriately
}
```

### Pagination

```typescript
// Get pools with pagination
const firstPage = await sdk.getPools({ limit: 50, offset: 0 });
const secondPage = await sdk.getPools({ limit: 50, offset: 50 });

console.log(`Total pools: ${firstPage.total}`);
```

## Supported Networks

- Ethereum Mainnet (chainId: 1)
- Sepolia Testnet (chainId: 11155111)
- Arbitrum One (chainId: 42161)
- Optimism (chainId: 10)
- Base (chainId: 8453)

## Supported Protocols (2026)

- **DEXs**: Uniswap v4, Curve v2, Balancer v3, SushiSwap
- **Lending**: Aave V4, Compound v3, Morpho Blue
- **Staking**: Lido v3, Rocket Pool, Frax
- **Aggregators**: 1inch v6, 0x Protocol, Paraswap
- **Derivatives**: GMX v2, Synthetix v3, dYdX v4

## API Key

Get your API key at: https://api.wia-insurtech.org/register

## Examples

See the `/examples` directory for complete examples:

- `swap.ts` - Token swapping
- `liquidity.ts` - Providing liquidity
- `lending.ts` - Lending and borrowing
- `staking.ts` - Staking operations
- `streaming.ts` - Real-time data streaming

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test

# Type checking
npm run typecheck

# Lint
npm run lint

# Format
npm run format
```

## Philosophy

**弘益人間** (Hongik Ingan) - "Benefit All Humanity"

This SDK embodies the WIA philosophy of making DeFi accessible, safe, and beneficial for everyone. We believe in:

- **Transparency**: Open standards and documentation
- **Security**: Rigorous validation and error handling
- **Accessibility**: Easy-to-use APIs for developers of all levels
- **Reliability**: Production-ready code with comprehensive testing

## Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests to our repository.

## License

MIT License - see LICENSE file for details

## Support

- Documentation: https://docs.wia.org/insurtech-sdk
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Discord: https://discord.gg/wia
- Email: dev@wia.org

---

© 2026 WIA Official | 弘益人間 (Benefit All Humanity)
