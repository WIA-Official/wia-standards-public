# WIA-nft PHASE 1 — Data Format Specification

**Standard:** WIA-nft
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for non-
fungible token operations: token records (ERC-721,
ERC-1155, equivalent multi-chain), token-metadata
records, ownership records, transfer-event records,
royalty records, marketplace-listing records, and the
cross-references binding tokens to off-chain content
and to the legal contracts that govern them. The shape
is interoperable with widely-fielded ERC token standards
and with W3C Decentralized Identifiers (DIDs) so existing
on-chain ecosystems adopt this PHASE without parallel
data models.

References (CITATION-POLICY ALLOW only):
- ERC-721 — Non-Fungible Token Standard (Ethereum Improvement Proposal EIP-721)
- ERC-1155 — Multi Token Standard (EIP-1155)
- ERC-2981 — NFT Royalty Standard (EIP-2981)
- ERC-4906 — EIP-721 Metadata Update Extension
- ERC-5192 — Minimal Soulbound NFTs
- EIP-712 — Typed structured-data hashing and signing
- EIP-191 — Signed Data Standard
- W3C Decentralized Identifiers (DID) Core 1.0
- W3C Verifiable Credentials Data Model 2.0
- IPFS / IPLD (InterPlanetary File System) — content addressing
- ISO/TC 307 — Blockchain and distributed ledger technologies (foundational and reference)
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 4122 (UUID), RFC 8785 (JSON Canonicalisation Scheme)

---

## §1 Scope

This PHASE applies to systems that mint, transfer, list,
or settle non-fungible tokens on EVM-compatible chains
(Ethereum, Polygon, Arbitrum, Optimism, Base, BSC, etc.)
and on chains operating under analogous token semantics
(Solana SPL non-fungible, Tezos FA2, Flow, Sui, Aptos).

The standard is chain-aware: each token record carries
the controlling chain identifier so multi-chain marketplaces,
cross-chain bridges, and legal-record systems honour the
correct on-chain provenance.

In scope: token, token-metadata, ownership, transfer,
royalty, listing-and-bid, and licence-binding records.
Out of scope: cryptocurrency token-fungibility (handled
by other domain standards), wallet operational primitives
(handled by ecosystem wallets), and validator-set governance
(handled by chain consensus).

## §2 Token record

Every minted token carries:

| Field             | Source / Binding                                        |
|-------------------|---------------------------------------------------------|
| `tokenRef`        | URN of form `urn:wia:nft:token:<chain>:<contract>:<tokenId>` |
| `chain`           | EIP-3770 chain shorthand or full chain-ID URN           |
| `contractAddress` | EIP-55 checksummed contract address                     |
| `tokenId`         | hex or decimal token ID per the contract's interface    |
| `tokenStandard`   | `erc-721`, `erc-1155`, `spl-non-fungible`, `fa2-non-fungible`, etc. |
| `metadataURI`     | IPFS CID, HTTPS URL, or DID URL                         |
| `mintedAt`        | block timestamp at mint                                 |
| `minterAddress`   | minter's address                                        |
| `currentOwners[]` | for ERC-1155 (multi-quantity), the address-and-balance set |
| `royaltyPolicyRef`| URN of the active royalty policy (PHASE 1 §6)           |

Token records mirror on-chain state; the boundary verifies
each record against on-chain calls (`ownerOf`, `tokenURI`,
`balanceOf`) before honouring it for downstream operations.

## §3 Token-metadata record

The token's metadata follows the published metadata schema
for its standard:

```json
{
  "name": "Example Artwork #001",
  "description": "...",
  "image": "ipfs://bafkreig.../image.png",
  "animation_url": "ipfs://bafkreig.../animation.mp4",
  "external_url": "https://creator.example/works/001",
  "attributes": [
    {"trait_type": "Background", "value": "Cobalt"},
    {"trait_type": "Edition", "value": 1, "max_value": 250}
  ],
  "properties": {
    "creators": [{"address": "0x...", "share": 100}],
    "category": "image",
    "files": [{"uri": "ipfs://...", "type": "image/png"}]
  }
}
```

Metadata referenced by an `ipfs://` URI is content-addressed
and immutable; metadata referenced by an `https://` URI
SHOULD declare a hash commitment in the on-chain record so
mutation is detectable. ERC-4906 metadata-update events
are recorded as separate metadata-version records so
historic state is preserved.

## §4 Ownership record

Per-chain ownership state:

- `ownershipId` — URN
- `tokenRef` — referenced token
- `ownerAddress` — current holder address
- `quantity` — for ERC-1155, the quantity held (1 for
  ERC-721)
- `acquisitionTransferRef` — URN of the transfer event
  that led to current ownership
- `acquiredAt` — block timestamp
- `transferRestrictions[]` — soulbound (ERC-5192),
  KYC-gated, jurisdictionally-restricted, time-locked

Soulbound or otherwise non-transferable tokens carry an
explicit `transferRestrictions` set so secondary-marketplace
listings can be refused at the marketplace boundary.

## §5 Transfer-event record

Each on-chain transfer:

- `transferId` — URN of form `urn:wia:nft:transfer:<chain>:<txHash>:<logIndex>`
- `tokenRef` — token transferred
- `fromAddress`, `toAddress` — counterparties
- `quantity` — for ERC-1155
- `transactionHash` — on-chain tx hash
- `blockNumber`, `blockTimestamp`
- `transferKind` — `mint`, `sale`, `gift`, `bridge`,
  `burn`, `restitution`
- `salePriceEthEquivalent` — for `sale` kind, the price
  at the chain's native or stablecoin equivalent
- `marketplaceRef` — URN of the marketplace where applicable

Transfer-kind classification (sale vs. gift vs. bridge)
draws on tx-trace inspection plus marketplace-platform
attestation; ambiguous cases are flagged and held for
review.

## §6 Royalty record

Per-token royalty policy (per ERC-2981 and equivalent):

- `royaltyPolicyId` — URN
- `tokenRef` — referenced token
- `creatorAddresses[]` — beneficiary addresses with shares
- `royaltyBps` — royalty rate in basis points (max 10,000)
- `enforcementMechanism` — closed enum:
  - `on-chain-erc2981` — contract-side enforcement
  - `marketplace-honoured` — marketplace-side honour
    (depends on marketplace policy)
  - `creator-tooling` — off-chain tooling for distribution
- `effectiveBlockNumber` — when the policy took effect
- `lastUpdatedAt` — RFC 3339

Royalty enforcement is contractually variable; the boundary
records the current policy but cannot guarantee enforcement
on every chain. The deployment's marketplace integration
(PHASE 4 §3) declares per-marketplace honour.

## §7 Marketplace-listing record

Off-chain listings on marketplaces:

- `listingId` — URN
- `tokenRef` — listed token
- `marketplaceRef` — URN of the marketplace
- `sellerAddress` — listing seller's address
- `priceCurrency` — listing currency (ETH, USD, USDC, etc.)
- `priceAmount` — listing price
- `listingType` — `fixed-price`, `english-auction`,
  `dutch-auction`, `offer-only`
- `expiresAt` — RFC 3339 expiry
- `signedOrder` — EIP-712 typed-data hash and signature

Listings are off-chain commitments; settlement happens
on-chain via the marketplace contract. The boundary tracks
listing state for analytics and for cross-marketplace
detection of duplicate listings (which can manifest as
race-condition double-sells if not coordinated).

## §8 Licence-binding record

For tokens that confer rights to underlying content:

- `licenceId` — URN
- `tokenRef` — referenced token
- `licenceType` — closed enum drawn from established
  licence catalogues (CC0, CC-BY, CC-BY-NC, custom-NFT-licence)
- `licenceURI` — URI of the licence text (often IPFS)
- `licenceHash` — SHA-256 of the licence text for
  immutability commitment
- `confersRights[]` — declared rights (display, derivative,
  commercial-use, exclusive-vs-non-exclusive)
- `revocationConditions` — when the licence may be revoked
  (typically on chain-of-custody loss for revocable licences)

Tokens conferring exclusive commercial licences (e.g., music
masters tokenised as exclusive licence) MUST attach the
underlying contract document URN; cross-domain references
to WIA-supply-chain or notary services apply where the
underlying asset is physical.

## §9 Cross-domain references (informative)

| Reference                  | Use site                                          |
|----------------------------|---------------------------------------------------|
| WIA-payment-system         | fiat settlement of marketplace sales              |
| WIA-identity-management    | KYC-gated transfer restrictions                   |
| WIA-network-security       | TLS cipher-suite floor for marketplace APIs       |
| WIA-pq-crypto              | post-quantum migration phase                      |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Conformance disclosure

Sections §2, §3, §4, §5 are mandatory. §6 (royalty) is
mandatory if the deployment supports royalty-bearing
products. §7 (marketplace) is mandatory for any deployment
with marketplace functions. §8 (licence) is mandatory
where the deployment offers licence-bearing tokens.

## Annex B — Chain-shorthand registry (informative)

Common EIP-3770 shorthands used in `tokenRef`:

| Shorthand | Chain                                |
|-----------|--------------------------------------|
| `eth`     | Ethereum mainnet                     |
| `pol`     | Polygon PoS                          |
| `arb1`    | Arbitrum One                         |
| `op`      | Optimism                             |
| `base`    | Base                                 |
| `bsc`     | BNB Smart Chain                      |
| `avax`    | Avalanche C-Chain                    |
| `sol`     | Solana mainnet (non-EIP-3770 native) |

Cross-chain bridges record a paired record on both source
and destination chains so provenance traceability survives
bridging.

## Annex C — Worked ERC-721 token record (informative)

```json
{
  "tokenRef": "urn:wia:nft:token:eth:0xb47e3cd837dDF8e4c57F05d70Ab865de6e193BBB:1234",
  "chain": "eth",
  "contractAddress": "0xb47e3cd837dDF8e4c57F05d70Ab865de6e193BBB",
  "tokenId": "0x4d2",
  "tokenStandard": "erc-721",
  "metadataURI": "ipfs://bafkreig.../1234.json",
  "mintedAt": "2026-04-01T12:34:00Z",
  "minterAddress": "0xa1b2c3...",
  "currentOwners": [
    {"address": "0xc4d5e6...", "balance": 1}
  ],
  "royaltyPolicyRef": "urn:wia:nft:royalty:eth:0xb47e3cd...:1234"
}
```

## Annex D — Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Major bumps
require ≥ 90 days overlap on every fielded reference
implementation. Deprecation enters a 12-month sunset
window with migration notes recorded in cross-marketplace
catalogues.

## Annex E — Conformance level

| Level     | Scope                                                          |
|-----------|----------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                            |
| Verified  | annual third-party audit                                        |
| Anchored  | continuous evidence package + chain-anchored attestation roots  |

## Annex F — Soulbound (ERC-5192) record (informative)

Soulbound tokens cannot transfer post-mint:

```json
{
  "tokenRef": "urn:wia:nft:token:eth:0xsoulbound...:101",
  "tokenStandard": "erc-5192",
  "currentOwners": [{"address": "0xrecipient...", "balance": 1}],
  "transferRestrictions": ["soulbound"],
  "metadataURI": "ipfs://bafkreig.../membership.json"
}
```

ERC-5192's `locked(tokenId)` returns `true` for these.
The boundary surfaces this at marketplace-listing time
so listings are refused before reaching settlement.

## Annex G — Verifiable-credential binding (informative)

For tokens carrying verifiable credentials (e.g.,
proof-of-attendance, certifications):

- W3C Verifiable Credential issued to the token's holder
- Credential references the tokenRef as `credentialSubject.id`
- Holder presents the credential via DIDComm to consumers
- Revocation status checked via the credential issuer's
  status registry

## Annex H — Multi-chain creator identity (informative)

A creator operating across chains uses a DID-bound
identity:

```
did:web:creator-x.example
  └── verificationMethod: [eth address, sol pubkey, btc address]
  └── service: [creator-dashboard, royalty-distribution]
```

Tokens minted on each chain reference the creator's DID
so cross-chain analytics can attribute correctly.

## Annex I — Royalty policy migration

When a creator migrates from on-chain ERC-2981 to a
splits-contract-based distribution:

1. Existing royalty policy frozen at migration block
2. New royalty policy declared with effective date
3. Migration audit record bound to the original token
4. Marketplaces honour the migrated policy from
   effective date forward
