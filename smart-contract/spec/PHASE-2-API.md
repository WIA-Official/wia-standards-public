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

---

## A.1 SDK request lifecycle

Each SDK call follows the same lifecycle: validate input against the typed parameter shape, fetch the live nonce and gas tip estimate, build the transaction, sign with the wallet adapter, submit to the broadcast pool, and finally subscribe to the inclusion receipt. Read calls short-circuit the signing and broadcast steps; write calls run the full pipeline. The SDK exposes hooks at every boundary so wallets, hardware signers, and policy engines can intervene without forking the SDK.

## A.2 Multicall batching

`multicall` accepts an array of `MulticallRequest` items and dispatches them through a deployed multicall aggregator (Uniswap-style Multicall3 or equivalent) in a single eth_call. The SDK falls back to sequential reads if the aggregator is unavailable on the active chain. Failure of any sub-call is reported in the corresponding `MulticallResponse.success` flag; partial-result handling is the caller's responsibility per the Phase 3 protocol contract.

## A.3 Event listener semantics

`on(event, listener)` registers a long-lived listener that survives provider reconnects. The SDK keeps a high-water mark per `event × address` pair and replays any events missed during a reconnect. `once` resolves on the first matching log; `queryFilter` is bounded by the `fromBlock`/`toBlock` window the caller provides. `streamEvents` returns an async iterator that yields events in order and surfaces a sentinel value when the underlying provider closes the subscription.

## A.4 Multi-chain switching

`switchChain` updates the active provider, refreshes the nonce manager, and re-binds the wallet adapter. In-flight transactions on the previous chain continue to settle on that chain; the SDK does not migrate pending nonces across chains.

## A.5 Gas-strategy reference

| Strategy  | maxPriorityFeePerGas | Inclusion target | Use case                   |
|-----------|----------------------|------------------|----------------------------|
| slow      | 1 gwei               | next 8 blocks    | airdrops, batch settlement |
| standard  | 1.5 gwei             | next 4 blocks    | regular UX                 |
| fast      | 2.5 gwei             | next 2 blocks    | DEX trades                 |
| instant   | 4 gwei               | next block       | liquidations, MEV-sensitive|

The SDK rebases these tips against the live `eth_feeHistory` p25/p50/p75 values; the table is the floor.

## A.6 Error-handling envelope

Every SDK call resolves to either a typed result or a typed error. The error hierarchy mirrors the on-chain custom errors declared in Phase 1 §A.4 so callers receive `WIAUnauthorizedError`, `WIAInsufficientBalanceError`, `WIAContractPausedError`, `WIABlacklistedError`, etc., rather than free-form strings. Wallet-level errors (user rejection, hardware-signer timeout) are mapped onto a parallel hierarchy under `WIAWalletError` so app code can disambiguate without parsing strings.

## A.7 Receipt and confirmation discipline

Write calls return a transaction-response object that exposes the transaction hash immediately and a `wait(confirmations)` method that resolves once the configured confirmation depth is reached. Default confirmation depth is 6 on Ethereum mainnet, 32 on Arbitrum / Optimism, and 12 on BNB Chain. Hosts MAY override per chain but MUST document the override in the SDK initialisation block.

## A.8 ENS / DID resolution helpers

`resolveAddress(name)` accepts an ENS name (`*.eth`), a CCIP-Read offchain resolver, or a DID (`did:wia:*`, `did:ethr:*`, `did:web:*`). Resolution is cached for 5 minutes and invalidated on the next call rather than refreshed in the background. Resolution failures fall back to the literal address argument to defeat homoglyph attacks; the caller never sees a "did you mean" suggestion.

## A.9 Static-typing and code-generation policy

The SDK generates TypeScript types from the contract ABI plus the wiaExtensions block (Phase 1 §A.5) so a typo in a method name fails at compile time. The codegen tool is shipped at `cli/contract-codegen.sh` and emits both runtime client code and `.d.ts` declarations. Hosts that target other languages (Go, Python, Rust) get equivalent codegen via the same plugin interface.

## A.10 Logging, tracing, observability

SDK calls emit structured logs (one record per call) with the chain id, contract address, method, gas estimate, gas used, status, and optional correlation id. Tracing is exposed via the OpenTelemetry SDK; spans cover validate → estimate → sign → broadcast → confirm. Metrics emit in OpenMetrics format: call rate, gas spend per method, p50/p95/p99 confirmation latency, and revert reason histogram. Hosts wire these into their existing observability stack without an adapter layer.

## A.11 Provider failover

Production deployments configure ≥2 RPC providers (Alchemy, Infura, QuickNode, Ankr, or self-hosted Erigon / Reth / Geth). The SDK detects provider stalls via block-height monotonicity and request latency; on stall it routes to the next provider in the list. Failover is logged and emits an operations alert if the same provider stalls more than three times in five minutes.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-contract/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-contract-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-contract-host:1.0.0` ships every smart-contract envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-contract.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Smart-contract deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
