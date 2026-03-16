# WIA-FIN-007 Phase 4: Integration Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-01-20

## Overview

Phase 4 defines integration standards for wallets, oracles, DeFi protocols, and cross-chain bridges.

## Wallet Integration

### Supported Wallets

- MetaMask
- WalletConnect
- Coinbase Wallet
- Trust Wallet
- Ledger (Hardware)
- Trezor (Hardware)
- Safe (Multisig)

### Connection Interface

```typescript
interface IWIAWallet {
  connect(): Promise<WalletConnection>;
  disconnect(): void;
  switchChain(chainId: number): Promise<void>;
  signMessage(message: string): Promise<string>;
  signTypedData(data: TypedData): Promise<string>;
  sendTransaction(tx: TransactionRequest): Promise<TransactionResponse>;
}
```

## Oracle Integration

### Chainlink Price Feeds

```solidity
interface IWIAOracle {
    function getLatestPrice(address token) 
        external 
        view 
        returns (uint256 price, uint256 timestamp);
    
    function getPriceWithFallback(address token, address[] memory fallbacks) 
        external 
        view 
        returns (uint256 price);
}
```

### Multi-Oracle Aggregation

Minimum 3 oracle sources required for production.

## DeFi Protocol Integration

### Supported Protocols

- **Uniswap V3:** DEX integration
- **Aave V3:** Lending/borrowing
- **Curve:** Stablecoin swaps
- **Compound:** Money markets

### Standard Interfaces

```solidity
interface IWIADeFiIntegration {
    function swapExactInput(
        address tokenIn,
        address tokenOut,
        uint256 amountIn,
        uint256 minAmountOut
    ) external returns (uint256 amountOut);
    
    function addLiquidity(...) external returns (uint256 liquidity);
    function removeLiquidity(...) external returns (uint256 amount0, uint256 amount1);
}
```

## Cross-Chain Bridges

### LayerZero

```solidity
contract WIACrossChain is OFTV2 {
    function sendFrom(
        address _from,
        uint16 _dstChainId,
        bytes32 _toAddress,
        uint _amount
    ) public payable override;
}
```

### Wormhole

```solidity
interface IWIAWormholeBridge {
    function bridgeTokens(
        uint16 targetChain,
        bytes32 recipient,
        uint256 amount
    ) external payable;
}
```

## Subgraph Integration

### The Graph Protocol

```graphql
type WIAToken @entity {
  id: ID!
  name: String!
  symbol: String!
  totalSupply: BigInt!
  holders: [Holder!]! @derivedFrom(field: "token")
  transfers: [Transfer!]! @derivedFrom(field: "token")
}
```

## ENS Integration

```typescript
const address = await ensResolver.resolve('wia-token.eth');
await contract.transfer(await ensResolver.resolve('recipient.eth'), amount);
```

## IPFS Integration

```typescript
const cid = await ipfs.upload(metadata);
console.log(`IPFS: https://ipfs.io/ipfs/${cid}`);
```

## Compliance Requirements

- [ ] Universal wallet connector implemented
- [ ] Multi-oracle price feeds
- [ ] DeFi protocol integrations (minimum 2)
- [ ] Cross-chain bridge support
- [ ] Subgraph for historical data
- [ ] ENS resolution support
- [ ] IPFS metadata storage

**Passing Score:** 80/100

---

© 2025 SmileStory Inc. / WIA
