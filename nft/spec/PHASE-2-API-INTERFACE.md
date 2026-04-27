# WIA-nft PHASE 2 — API Interface Specification

**Standard:** WIA-nft
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a deployment exposes
for token discovery, token-metadata retrieval, ownership
queries, transfer-event subscription, royalty queries,
marketplace-listing intake and lifecycle, licence-binding
queries, and creator-side mint orchestration. The shape is
HTTP/JSON for routine interactions; high-volume event
subscription uses streaming.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON), RFC 4122 (UUID)
- ERC-721, ERC-1155, ERC-2981, ERC-4906, ERC-5192
- EIP-712 — Typed structured-data hashing and signing
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers (DID) Core 1.0

---

## §1 Token discovery

```
GET /tokens?contract=<contractAddress>&chain=eth HTTP/1.1
GET /tokens/{tokenRef}
GET /tokens?owner=<address>&chain=eth
```

Returns token records (PHASE 1 §2). Token-discovery queries
typically resolve via index nodes that mirror chain state;
the boundary verifies discovered records by cross-checking
against on-chain calls before returning.

## §2 Token-metadata retrieval

```
GET /tokens/{tokenRef}/metadata HTTP/1.1
```

Returns the canonical token metadata (PHASE 1 §3). For
IPFS-stored metadata the boundary either pins or proxies
through a deployment-controlled gateway so metadata
availability is not solely dependent on third-party
gateways. ERC-4906 metadata-update events emit through
the subscription endpoint (§5).

## §3 Ownership queries

```
GET /tokens/{tokenRef}/owners HTTP/1.1
GET /accounts/{address}/holdings?chain=eth
```

For ERC-1155 multi-quantity tokens, the response
enumerates per-balance ownership. Soulbound (ERC-5192)
ownership returns a `transferRestrictions` field reflecting
the on-chain commitment.

## §4 Mint orchestration

For creators authorising mints through the deployment:

```
POST /mints HTTP/1.1
Authorization: Bearer <jws-creator-jwt>
Content-Type: application/json

{
  "contractAddress": "0xb47e3cd837dDF8e4c57F05d70Ab865de6e193BBB",
  "tokenStandard": "erc-721",
  "tokenIds": ["0x4d2", "0x4d3"],
  "recipientAddresses": ["0xa1b2c3...", "0xa1b2c3..."],
  "metadataURIs": ["ipfs://bafkreig.../...json", "ipfs://bafkreig.../...json"],
  "royaltyPolicy": {
    "creatorAddresses": ["0xa1b2c3..."],
    "royaltyBps": 750
  },
  "ledgerNonceCoordination": "<coordinator URN>"
}
```

The boundary verifies the creator's authority over the
target contract (via EIP-712 signature against a registered
public key), submits the mint transaction, captures the tx
hash, and returns the resulting token URNs once block
confirmation is obtained.

## §5 Transfer-event subscription

```
GET /transfers/stream?chain=eth&contract=<address> HTTP/1.1
Accept: text/event-stream
```

Live SSE stream of transfer events for the requested
chain/contract scope, drawing on chain index nodes and
mempool feeds. Each event is one PHASE 1 §5 transfer
record. Resume tokens permit reconnection without loss.

```
GET /tokens/{tokenRef}/transfers HTTP/1.1
```

Returns the historic transfer record for the token.

## §6 Royalty queries

```
GET /tokens/{tokenRef}/royalty HTTP/1.1
GET /tokens/{tokenRef}/royalty?at=blockNumber=12345678
```

Returns the royalty policy active at the requested block
(or current). Cross-marketplace royalty enforcement is
declared in capability documents (§9); the boundary
publishes per-marketplace honour status alongside the
royalty record.

## §7 Marketplace-listing intake

```
POST /listings HTTP/1.1
Authorization: Bearer <jws-marketplace-jwt>

{
  "tokenRef": "urn:wia:nft:token:eth:0xb47e3cd...:1234",
  "marketplaceRef": "urn:wia:nft:marketplace:opensea",
  "sellerAddress": "0xa1b2c3...",
  "priceCurrency": "ETH",
  "priceAmount": "1.5",
  "listingType": "fixed-price",
  "expiresAt": "2026-05-30T00:00:00Z",
  "signedOrder": "<EIP-712 typed-data signature>"
}
```

The boundary verifies the EIP-712 signature against the
seller's address, checks the token's transfer
restrictions (PHASE 1 §4), and records the listing.

```
GET /listings?tokenRef=...
PUT /listings/{listingId}/cancel
```

Listings can be cancelled by the seller; cancellation
emits a transfer-restriction-reset signal to listeners.

## §8 Bid lifecycle

For auction-listing types:

```
POST /listings/{listingId}/bids HTTP/1.1
{
  "bidderAddress": "0xc4d5e6...",
  "bidAmount": "1.6",
  "bidCurrency": "ETH",
  "bidExpiresAt": "2026-05-25T00:00:00Z",
  "signedBid": "<EIP-712>"
}
```

The boundary tracks bid lifecycle (open / outbid / accepted
/ expired / cancelled); auction settlement triggers the
on-chain marketplace contract to execute the transfer.

## §9 Capability discovery

```
GET /.well-known/wia/nft HTTP/1.1
```

Returns the deployment's capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "marketplace-x-prod-v3.2.1",
  "wia.chains": ["eth", "pol", "arb1", "base"],
  "wia.tokenStandards": ["erc-721", "erc-1155", "erc-2981"],
  "wia.royaltyHonour": "marketplace-honoured",
  "wia.metadataGateway": "https://ipfs.gateway.example",
  "wia.bridgeIntegrations": ["wormhole", "layerzero"],
  "wia.conformanceLevel": "Verified"
}
```

## §10 Errors and warnings

| problem URN                                       | meaning                                       |
|---------------------------------------------------|-----------------------------------------------|
| `urn:wia:nft:problem:token-not-found`             | tokenRef not visible to the boundary          |
| `urn:wia:nft:problem:transfer-restricted`         | token has soulbound or jurisdictional restriction |
| `urn:wia:nft:problem:invalid-signature`           | EIP-712 signature verification failed         |
| `urn:wia:nft:problem:metadata-unavailable`        | metadataURI unreachable beyond cache          |
| `urn:wia:nft:problem:listing-expired`             | listing past expiresAt                        |
| `urn:wia:nft:problem:auction-closed`              | bid attempted after auction closure           |
| `urn:wia:nft:problem:chain-reorg-detected`        | recent transfer affected by chain reorg       |
| `urn:wia:nft:problem:royalty-honour-not-supported`| target marketplace does not honour royalty    |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Pagination and rate limiting

List endpoints paginate at ≤ 1000 results per page.
Per-token rate limits default to 60 listing-intake
requests per minute per marketplace; high-volume index
queries use longer burst windows tuned to deployment
policy.

## Annex B — Idempotency

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. Replays return the original
response. Different body with same key returns
`urn:wia:nft:problem:idempotency-conflict` (409).

## Annex C — EIP-712 typed-data structure (informative)

Listing signature payload follows EIP-712 typed-data:

```
EIP-712 Domain:
  name: "WIA-NFT-Listing"
  version: "1"
  chainId: <chainId>
  verifyingContract: <marketplace contract>

Listing message:
  tokenContract: address
  tokenId: uint256
  seller: address
  price: uint256
  currency: address (zero for native)
  expiresAt: uint256
  nonce: uint256
```

Verification requires reconstructing the EIP-712 hash and
checking the signature against the seller's address.

## Annex D — Webhook delivery contract

```
POST /webhooks HTTP/1.1
{
  "endpoint": "https://creator-app.example/wia-webhook",
  "events": ["transfer.executed", "listing.expired", "royalty.paid"],
  "signingKid": "creator-app-2026"
}
```

Delivery uses TLS 1.3 with detached JWS in `Wia-Signature`.
Listeners acknowledge with 200 OK; un-acknowledged events
retry per the deployment's retry budget.

## Annex E — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion`. Standard-version mismatch is a
hard refusal; implementation-version mismatch is logged.

## Annex F — Reorg-handling protocol

When a chain reorg affects a recent transfer:

```
GET /tokens/{tokenRef}/reorg-status?lookbackBlocks=12 HTTP/1.1
```

Returns whether any of the lookback's transfers are
affected by reorg events. Listings referencing a
reorged transfer auto-suspend until block-finality
is re-established at the deployment-declared depth
(typically 12 blocks for ETH mainnet, 64 for L2s).

## Annex G — Cross-chain token-bridge tracking

```
POST /bridges/track HTTP/1.1
{
  "sourceTokenRef": "urn:wia:nft:token:eth:0x...:1234",
  "destinationChain": "pol",
  "bridgeProtocolRef": "urn:wia:nft:bridge:wormhole",
  "bridgeTxHash": "0x..."
}
```

The boundary tracks the cross-chain transfer state
(initiated / in-flight / completed / failed) and
records the destination tokenRef once minted at the
bridged-to chain.

## Annex H — Bulk-mint protocol (informative)

For high-volume mint events (10,000+ tokens):

```
POST /mints/batch HTTP/1.1
{
  "contractAddress": "0xb47e3cd...",
  "items": [/* per-token metadataURIs and recipients, 1–500 per batch */],
  "batchOrdering": "sequential",
  "blockBudget": 64
}
```

The boundary submits each batch as a separate transaction
with carefully managed nonce-coordination so concurrent
mints don't collide. Failed batches retry with the next
nonce window.

## Annex I — Marketplace-aggregator (informative)

For applications aggregating across marketplaces:

```
GET /aggregated-listings?tokenRef=...
```

Returns listings across all marketplaces the boundary
mirrors. Aggregated floor-price queries:

```
GET /collections/{contractAddress}/floor-price?currency=ETH
```

## Annex J — Verifiable-credential issuance flow

For credentials issued at mint:

```
POST /credentials HTTP/1.1
{
  "holderDid": "did:ethr:0xc4d5e6...",
  "subjectTokenRef": "urn:wia:nft:token:eth:0xsoulbound...:101",
  "credentialType": "MembershipCredential",
  "issuanceDate": "2026-04-28",
  "expirationDate": "2027-04-28"
}
```

The boundary signs the VC with the issuer's signing key
and registers it in the deployment's credential registry.

## Annex K — Per-marketplace adapter registry

The boundary maintains a registry of per-marketplace
adapters declaring the marketplace's specific API:

```
GET /marketplace-adapters HTTP/1.1
```

Returns per-marketplace adapter capabilities: supported
listing types, royalty-honour level, EIP-712 domain
parameters, fee structure, and capability disclosure URI.

Distribution to a new marketplace requires registering
its adapter with the deployment; unauthorised marketplaces
cannot route listings through the boundary.

## Annex L — Royalty-payment audit endpoint

For royalty recipients tracking actual receipt:

```
GET /royalty-payments?creator=<address>&period=2026-Q1
```

Returns per-payment records with token refs, marketplace
refs, sale prices, computed royalty amounts, and
delivered amounts. Where marketplace-honour falls below
the configured policy, the discrepancy is flagged for
creator action.

## Annex M — Per-collection statistics endpoint

For collection-level aggregates that downstream analytics
and price-discovery tools depend on:

```
GET /collections/{contractAddress}/stats?currency=ETH&period=30d
```

Returns:

- token-count (current)
- holder-count (current; deduplicated by address)
- floor-price (current; per the deployment's marketplace-aggregator)
- 24h / 7d / 30d transfer volume
- 24h / 7d / 30d sale volume (paid transfers only)
- royalty volume (paid via ERC-2981 + marketplace-honoured)
- listing-rate (listed / total ratio)

The boundary returns aggregate values only; per-token
detail is gated by the caller's authorisation. For
collections under monitoring, stats refresh at the
chain's block cadence; for non-monitored collections,
on-demand queries cause a one-time refresh.
