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

---

## A.1 Wallet integration matrix

| Wallet category        | Connection path     | Signing surface           |
|------------------------|---------------------|---------------------------|
| Browser injected       | EIP-1193 provider   | personal_sign + eth_sendTransaction |
| Hardware (Ledger/Trezor) | Bridge over USB/HID | EIP-712 typed data + raw tx |
| Mobile (WalletConnect) | WC v2 session       | eth_signTypedData_v4      |
| Multisig (Safe)        | Safe Apps SDK       | Tx proposal + co-sign     |
| Programmatic           | Server keystore     | Direct ECDSA signer       |

Every wallet category MUST surface the chain ID, account, and capability list via `eth_chainId`, `eth_accounts`, and `wallet_getCapabilities` so the SDK can adapt without fingerprinting the wallet.

## A.2 Oracle aggregation policy

Production deployments MUST consume at least three oracle sources and MUST reject any reading whose absolute deviation from the median exceeds 2%. Stale-price detection rejects readings older than 30 minutes (or chain-specific freshness threshold). The fallback path falls through Chainlink → Pyth → DIA → manual override; the override is a multisig action with its own timelock.

## A.3 Cross-chain bridge envelope

Bridges normalize to a unified envelope: source chain, target chain, source token address, target token address, sender, recipient, amount, nonce, and a 32-byte `bridgeId` derived from the originating bridge contract. LayerZero, Wormhole, Axelar, and Chainlink CCIP all expose adapters that emit this envelope on the inbound side; the recipient contract verifies the signature/proof before crediting funds.

## A.4 Subgraph data model

The subgraph indexes Transfer events, Approval events, and any contract-specific events declared in metadata. Entities are derived from the events and are exposed via GraphQL with cursor-based pagination. Subgraphs MUST handle re-orgs by versioning the entity store; the conformance suite includes a re-org-replay scenario that the subgraph deployment MUST pass.

## A.5 ENS / DID resolution

Wallet-facing flows resolve ENS names (`*.eth`) and DIDs (`did:wia:*`) via the DID resolver pinned to the registry at Phase 1 §A.1. The SDK caches resolved tuples for 5 minutes and refreshes lazily on the next call. Resolution failures fall back to the literal address, never to a typed-in alternative, to defeat homoglyph attacks.

## A.6 IPFS pinning policy

Off-chain artefacts (audit reports, NFT metadata, large struct payloads) are pinned to IPFS with a content-id that is also stored on chain in the metadata block. Pinning MUST run on at least three independent gateways (the canonical pin set is Pinata + web3.storage + nft.storage). Removal of any pin triggers a `PinDropped` operations alert that must be remediated within 24 hours.

## A.7 Reference cross-walk

- ISO/IEC 27001:2022 — information-security management
- ISO/IEC 27034 — application-security guidance
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- W3C DID Core 1.0 — decentralised identifiers
- W3C VC Data Model 2.0 — verifiable credentials
- EIP-165 — standard interface detection
- EIP-712 — typed structured data hashing and signing
- EIP-1271 — standard signature validation method for contracts
- EIP-2612 — permit (signed approvals)
- EIP-4337 — account abstraction
- EIP-1559 — fee market
- EIP-4844 — proto-danksharding
- IETF RFC 6979 — deterministic ECDSA
- IETF RFC 8017 — PKCS #1 RSA
- IETF RFC 6585 — additional HTTP status codes (429)

## A.7b Tokenisation cross-walk

- ERC-20 — fungible-token interface
- ERC-721 — non-fungible-token interface
- ERC-1155 — multi-token interface
- ERC-4626 — tokenised-vault interface
- ERC-2981 — NFT royalty signal
- ERC-5805 — voting-with-delegation token interface
- ERC-7281 — XERC20 cross-chain fungible token

WIA-conformant deployments declare the implemented interfaces in the metadata block (Phase 1 §A.1) so off-chain indexers can dispatch the correct ABI without re-resolving from source code.

## A.8 Audit lifecycle

Third-party audits cover initial deployment plus every major upgrade. The audit report is published at `audit/{version}/report.pdf` with the report hash anchored on chain. Findings at "high" or "critical" severity MUST be remediated before deployment; findings at "medium" or below MUST carry a documented justification or remediation plan in the deployment notes. The audit is itself a Phase 4 integration concern because the audit registry is part of the broader supply-chain provenance graph.

## A.8 Subgraph schema notes

The reference subgraph indexes the canonical event set declared in Phase 1 §A.3 plus any contract-specific events that an implementation chooses to surface. Each entity has a stable id (transaction hash + log index for events; address for contracts; address + token id for non-fungible holdings). Re-org handling versions every entity write, so the subgraph can roll back N blocks worth of mutations without losing history.

## A.9 Operational runbook

Production smart-contract deployments follow the runbook at `https://wiastandards.com/smart-contract/runbook/`: monitor, alert, contain, recover. Monitor covers state-of-chain dashboards (TVL, daily active addresses, gas spend); alert covers anomalous-event triggers; contain covers the circuit-breaker and pause flows; recover covers the upgrade timelock + multisig sequence. Incident drills are run quarterly; the runbook is reviewed after every drill.


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
