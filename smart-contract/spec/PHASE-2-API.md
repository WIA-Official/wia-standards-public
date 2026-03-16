# WIA-FIN-007 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-01-20

## Overview

Phase 2 defines standardized API interfaces for interacting with smart contracts across all supported blockchains. The WIA SDK provides type-safe, consistent methods for contract interaction.

## SDK Core Interface

### Installation

```bash
npm install @wia/smart-contract ethers
```

### Basic Usage

```typescript
import { WIASmartContract } from '@wia/smart-contract';

const contract = new WIASmartContract({
  address: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
  abi: contractABI,
  provider: ethersProvider,
  chain: 'ethereum'
});
```

## Standard Methods

### Read Operations

All read methods MUST be gas-free and return typed results:

```typescript
interface IWIAContract {
  // ERC-20 Standard
  name(): Promise<string>;
  symbol(): Promise<string>;
  decimals(): Promise<number>;
  totalSupply(): Promise<BigNumber>;
  balanceOf(address: string): Promise<BigNumber>;
  allowance(owner: string, spender: string): Promise<BigNumber>;

  // WIA Extensions
  getMetadata(): Promise<WIAContractMetadata>;
  getSecurityConfig(): Promise<SecurityConfig>;
  getDeploymentInfo(): Promise<DeploymentInfo>;
  
  // Batch operations
  multicall(calls: MulticallRequest[]): Promise<MulticallResponse[]>;
}
```

### Write Operations

```typescript
interface IWIAContractWrite {
  transfer(to: string, amount: BigNumber, options?: TxOptions): Promise<TransactionResponse>;
  approve(spender: string, amount: BigNumber, options?: TxOptions): Promise<TransactionResponse>;
  transferFrom(from: string, to: string, amount: BigNumber, options?: TxOptions): Promise<TransactionResponse>;
  
  // Gas optimization
  estimateGas(method: string, args: any[]): Promise<BigNumber>;
  sendOptimized(method: string, args: any[], strategy?: GasStrategy): Promise<TransactionResponse>;
}
```

## Event Handling

### Event Listeners

```typescript
interface IWIAEvents {
  on(event: string, listener: EventListener): void;
  once(event: string, listener: EventListener): void;
  off(event: string, listener: EventListener): void;
  removeAllListeners(event?: string): void;
  
  // Query historical events
  queryFilter(filter: EventFilter, fromBlock?: number, toBlock?: number): Promise<Event[]>;
  getPaginatedEvents(options: PaginationOptions): Promise<PaginatedEvents>;
  streamEvents(event: string, options?: StreamOptions): AsyncIterableIterator<Event>;
}
```

## Multi-Chain Support

### Chain Switching

```typescript
interface IWIAMultiChain {
  switchChain(chainId: number): Promise<void>;
  onChain(chainId: number): IWIAContract;
  getSupportedChains(): number[];
  getCurrentChain(): number;
}
```

## Error Handling

### Standard Error Types

```typescript
class WIAContractError extends Error {
  code: string;
  details?: any;
}

class WIAInsufficientBalanceError extends WIAContractError {
  available: BigNumber;
  required: BigNumber;
}

class WIAUnauthorizedError extends WIAContractError {
  caller: string;
  requiredRole: string;
}

class WIAContractPausedError extends WIAContractError {}
```

## Gas Management

### Gas Strategies

```typescript
type GasStrategy = 'slow' | 'standard' | 'fast' | 'instant';

interface GasEstimate {
  gasLimit: BigNumber;
  maxPriorityFeePerGas: BigNumber;
  maxFeePerGas: BigNumber;
  estimatedCost: BigNumber;
}
```

## Compliance Requirements

- [ ] All methods return Promise with typed results
- [ ] Error handling with WIA error types
- [ ] Event listeners for all standard events
- [ ] Multi-chain support with consistent interface
- [ ] Gas estimation and optimization
- [ ] Transaction retry logic
- [ ] TypeScript definitions included

**Passing Score:** 80/100

---

© 2025 SmileStory Inc. / WIA
