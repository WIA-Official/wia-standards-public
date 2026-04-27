# WIA-nft PHASE 3 — Protocol Specification

**Standard:** WIA-nft
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
creators, marketplace operators, indexers; on-chain call
discipline; reorg-handling; metadata-availability protocol;
EIP-712 signature handling; cross-chain bridge discipline;
audit-chain construction; failure modes.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7519 (JWT), RFC 8705 (mTLS-bound JWT)
- ERC-721, ERC-1155, ERC-2981, ERC-4906, ERC-5192
- EIP-712 — Typed structured-data hashing and signing
- EIP-191 — Signed Data Standard
- EIP-3770 — Chain-specific addresses
- W3C Decentralized Identifiers (DID) Core 1.0
- ISO/TC 307 — Blockchain and distributed-ledger reference

---

## §1 Authentication

Principals authenticate via JWS-signed JWTs alongside,
where appropriate, EIP-712 / EIP-191 wallet signatures
proving control of an Ethereum address:

| Claim          | Source                                                |
|----------------|-------------------------------------------------------|
| `iss`          | identity-provider URL                                 |
| `aud`          | the boundary URL                                      |
| `sub`          | creator / marketplace / indexer URN                   |
| `iat` / `exp`  | per RFC 7519                                          |
| `wia.role`     | `creator`, `marketplace`, `indexer`, `bridge-operator`, `analyst` |
| `wia.address`  | for wallet-bound roles, the EVM address                |
| `cnf`          | mTLS certificate-thumbprint binding                    |

Wallet-bound roles MUST present a fresh EIP-712 signature
proving control of `wia.address`; the signature has a
short window (≤ 5 minutes) to limit replay risk.

## §2 On-chain call discipline

The boundary verifies on-chain state by calling the
relevant standard methods on the contract:

- `ERC-721 ownerOf(tokenId)` — current owner
- `ERC-721 tokenURI(tokenId)` — metadata URI
- `ERC-1155 balanceOf(account, id)` — balance per
  account
- `ERC-1155 uri(id)` — metadata URI
- `ERC-2981 royaltyInfo(tokenId, salePrice)` — royalty
  receiver and amount

Calls go via the deployment's authorised RPC providers;
results are cross-checked against ≥ 2 independent
providers when available. Provider mismatch triggers
investigation and refusal of the cross-checked record.

## §3 Reorg-handling

For chain operations:

- The boundary maintains per-chain `finalityDepth`
  thresholds (12 blocks ETH mainnet, 64 blocks for
  L2s, chain-specific for non-EVM)
- Transfer events on blocks below finality are flagged
  `provisional`
- Reorgs detected via canonical chain-tip mismatch
  cause provisional records below the new fork point
  to be marked `superseded`
- Listings referencing superseded transfers auto-suspend
  until their referenced transfers re-finalise on the
  canonical chain

Chain reorgs are themselves audit events; the deployment's
quarterly compliance report tracks reorg frequency by
chain.

## §4 Metadata-availability protocol

For IPFS metadata:

- Deployment-controlled IPFS pinning service holds a
  copy of every published metadata file
- Public-gateway access is also offered for resilience
- Metadata is content-addressed by CID; pinning failures
  trigger re-pinning across redundant pinning providers

For HTTPS metadata:

- The boundary records a SHA-256 hash commitment at
  publish time
- Subsequent retrievals verify against the commitment
- Mismatch returns `urn:wia:nft:problem:metadata-mutation-detected`

## §5 EIP-712 signature handling

Listing and bid signatures follow EIP-712:

1. Construct the typed-data hash per the listing's domain
2. Recover the signer address via secp256k1 ecrecover
3. Verify the recovered address matches the declared
   seller / bidder
4. Cross-check the recovered address against the wallet-
   bound principal's `wia.address` claim

Signature replay protection: each EIP-712 message includes
a unique `nonce` field; the boundary tracks used nonces
per address with a 90-day window.

## §6 Cross-chain bridge discipline

For cross-chain token bridges:

- Boundary integrates with deployment-approved bridge
  protocols (Wormhole, LayerZero, etc.) only; arbitrary
  bridges are refused
- Source-chain token-lock event and destination-chain
  token-mint event are paired into a single
  `urn:wia:nft:bridge:transfer` record
- Bridge-side reorgs handled per the source chain's
  finality discipline; destination minting is held
  until source-side finality
- Bridge-protocol exploit detection: the deployment
  monitors known bridge-protocol oracle health and
  refuses cross-chain operations during oracle-paused
  states

## §7 Audit-chain construction

Every mint orchestration, transfer-event ingest,
listing intake, royalty mutation, and licence binding
emits an AuditEvent:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation
Scheme. Daily roots are signed by the deployment's
signing key (ES384 default).

For deployments operating across many chains, the audit
chain is partitioned by chain so single-chain audit
retrieves only that chain's entries.

## §8 Time discipline

Boundary clock: NTPv4 stratum-2. Block-time uses chain-
canonical block timestamps, not local clock. EIP-712
expiration windows reference UTC.

## §9 Key management

The deployment's signing key for audit-chain roots and
attestation responses lives in an HSM. Rotation cadence:

- Routine rotation every 180 days
- Emergency rotation on suspected key compromise
- Both signing keys (current + previous) appear in JWKS
  for ≥ 365 days post-rotation

For wallet-bound creator keys, the creator manages their
own keys; the deployment's role is verification only.

## §10 Failure modes

| failure                                  | behaviour                                                    |
|------------------------------------------|--------------------------------------------------------------|
| Identity-provider JWKS unreachable        | Cached keys honoured until cache expiry                     |
| RPC-provider mismatch                    | Record refused; investigation triggered                      |
| Chain reorg above finality depth          | Records below fork point marked superseded                  |
| Metadata-URI hash mismatch               | Mutation flagged; record marked tampered                    |
| EIP-712 nonce reuse                      | Refused with `signature-replay-detected`                    |
| Bridge-oracle paused                     | Cross-chain operations suspended                             |
| Audit-chain write failure                | Operation rejected (consistency w/ payment-system)          |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices

| Concern                       | Default                                | Notes                              |
|-------------------------------|----------------------------------------|------------------------------------|
| Token signing (boundary)      | ES256                                  | mTLS-bound (RFC 8705)              |
| Wallet signature verification | secp256k1 ECDSA                        | per EIP-191 / EIP-712              |
| Detached JWS body signature   | PS256 or ES256                         |                                    |
| Audit-chain root signing      | ES384                                  |                                    |
| Metadata hash                  | SHA-256                                | content-addressed where IPFS       |
| TLS                           | 1.3 (RFC 8446)                         | hybrid groups via WIA-pq-crypto    |

## Annex B — Conformance level

| Level     | Scope                                                                  |
|-----------|------------------------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                                    |
| Verified  | annual third-party audit (chain-state cross-check + pinning verification) |
| Anchored  | continuous evidence package + chain-anchored attestation roots          |

## Annex C — Worked sale lifecycle (informative)

```
1. Creator mints token (POST /mints)
2. Boundary verifies creator authority + EIP-712 signature
3. Mint tx confirms; tokenRef returned
4. Marketplace lists token (POST /listings)
5. Buyer places offer (POST /listings/{id}/bids)
6. Seller accepts; marketplace contract executes the swap
7. ERC-2981 royalty triggers; royalty paid via marketplace
8. Boundary records transfer event + royalty event
9. Audit chain captures every step
```

## Annex D — Negative-test vectors (informative)

| Stimulus                                                | Expected outcome                                |
|---------------------------------------------------------|-------------------------------------------------|
| Mint where caller is not contract minter                 | refused with `not-authorised-minter`           |
| Listing for soulbound (ERC-5192) token                   | refused with `transfer-restricted`              |
| Royalty policy update by non-creator                     | refused with `royalty-policy-update-forbidden`  |
| EIP-712 signature with reused nonce                      | refused with `signature-replay-detected`       |
| Bridge transfer with mismatched source/destination amts | refused with `bridge-amount-mismatch`           |

## Annex E — Provider-mismatch resolution (informative)

When two RPC providers disagree on a critical read
(`ownerOf`, `tokenURI`):

1. Boundary queries a third independent provider
2. If 2-of-3 agreement, accept the agreed result;
   flag the dissenting provider for investigation
3. If no agreement, refuse the record with
   `urn:wia:nft:problem:rpc-provider-disagreement`
4. Provider-quality scores degraded for repeated
   dissent

Provider-quality is published in the deployment's
quarterly compliance report (PHASE 4 §10).

## Annex F — Bridge-protocol risk classification (informative)

Bridges classified by security model:

| Class               | Examples                              | Boundary policy           |
|---------------------|---------------------------------------|---------------------------|
| Native (validated)  | rollup native bridges                  | Trusted by default         |
| Multi-sig committee | early bridges                          | Requires manual review     |
| MPC / threshold     | LayerZero, Wormhole core               | Trusted with ongoing audit |
| Optimistic          | Across-style                           | Trusted with delay         |
| Light-client        | IBC, Polymer                           | Trusted by design          |

Bridge classification is reviewed quarterly; class drift
(e.g., decommissioned multi-sig members) triggers re-
review.

## Annex G — Wallet-signature freshness (informative)

EIP-712 signatures from wallet-bound roles include:

```
{
  "wallet": "0xa1b2c3...",
  "intent": "mint",
  "tokenContract": "0xb47e3cd...",
  "validUntil": 1714287000,
  "nonce": "0x7f2c..."
}
```

Signatures past `validUntil` are refused. Per-address
nonce tracking prevents replay.

## Annex H — Per-chain finality discipline (informative)

| Chain      | Default `finalityDepth` | Notes                                           |
|------------|-------------------------|-------------------------------------------------|
| eth        | 12 blocks               | Beacon-chain finality at ~64 blocks (preferred) |
| pol        | 256 blocks              | Polygon PoS reorg-prone deeply                  |
| arb1       | 7 days (challenge)      | Practical use: 200 blocks                        |
| op         | 7 days (challenge)      | Practical use: 200 blocks                        |
| base       | 7 days (challenge)      | Practical use: 200 blocks                        |
| bsc        | 15 blocks               |                                                 |
| sol        | absolute slot finality  | At slot finalisation                             |

Deployments using the rollup chains (arb1/op/base) for
high-value transfers SHOULD wait for sequencer-batch
publication before treating as canonical.

## Annex I — Indexer-direct vs. RPC-direct read selection

For read paths:

| Operation                           | Recommended source     |
|-------------------------------------|-----------------------|
| Token-discovery list                | Indexer (subgraph)    |
| Single-token authoritative state    | RPC (cross-checked)   |
| Transfer-history aggregate          | Indexer               |
| Single-transfer verification         | RPC                  |
| Marketplace-listing aggregation      | Adapter API + RPC    |
