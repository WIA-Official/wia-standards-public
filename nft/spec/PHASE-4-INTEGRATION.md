# WIA-nft PHASE 4 — Integration Specification

**Standard:** WIA-nft
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how an NFT deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with broader
operational systems: chain RPC providers, indexers,
marketplaces, IPFS pinning services, fiat on/off-ramps,
KYC/AML compliance, royalty-distribution services, and
licence-rights verification systems.

References (CITATION-POLICY ALLOW only):
- ERC-721, ERC-1155, ERC-2981, ERC-4906, ERC-5192
- W3C Decentralized Identifiers (DID) Core 1.0, W3C Verifiable Credentials Data Model 2.0
- IPFS / IPLD — content addressing
- ISO/TC 307 — Blockchain reference framework
- WIA-payment-system, WIA-identity-management,
  WIA-network-security, WIA-pq-crypto

---

## §1 Chain RPC integration

The deployment subscribes to RPC providers per chain:

- **Primary:** dedicated infrastructure (own node where
  feasible) for high-volume chains
- **Secondary:** managed providers for redundancy
- **Tertiary:** public RPCs for last-resort failover

Provider rotation policy:

- Health-check every 60 seconds
- Auto-failover on consecutive failures
- Cross-check critical reads (ownerOf, tokenURI) across
  ≥ 2 providers before authoritative response

Bills, latency budgets, and slashing-events from any
self-hosted node feed into the deployment's RPC-quality
report (PHASE 4 §10).

## §2 Indexer integration

Indexers (The Graph, Goldsky, Alchemy, custom) maintain
materialised views of token state for fast retrieval:

- **Subgraph manifests:** for chains supported by The
  Graph, declared subgraph URLs serve query traffic
- **Custom indexers:** for chains lacking The Graph,
  the deployment runs its own indexer
- **Index lag tracking:** indexer freshness is monitored;
  excess lag triggers indexer restart

Cross-checked discrepancies between RPC-direct and
indexer-served reads are flagged as indexer-stale events.

## §3 Marketplace integration

Major marketplaces (OpenSea, Blur, Magic Eden, X2Y2,
Rarible) each have a published API:

- **Listing publication:** boundary forwards listings to
  per-marketplace endpoints
- **Listing retrieval:** boundary aggregates listings
  from multiple marketplaces
- **Royalty honour disclosure:** the boundary tracks
  per-marketplace royalty-honour status (full /
  optional / not-honoured)
- **Marketplace-specific signature verification:** each
  marketplace has its own EIP-712 domain; the boundary
  reconstructs the correct domain per marketplace

Cross-marketplace floor-price aggregation feeds creator
analytics and royalty-projection tools.

## §4 IPFS pinning integration

The deployment integrates with multiple pinning services
(Pinata, Filecoin, Web3.Storage, NFT.Storage, self-hosted
Filebase):

- Every published metadata is pinned across ≥ 3 pinning
  providers
- Pinning health monitored daily; failures trigger
  re-pinning
- Filecoin deals provide long-term economic guarantee
  for high-value content

Metadata referenced via centralised HTTPS URIs is
flagged in the capability disclosure as
`metadataAvailability=centralised`; consumers of the
boundary's data can apply preference filters.

## §5 Fiat on/off-ramp integration

For consumer-facing marketplaces accepting fiat payments:

- **On-ramp:** integrates with WIA-payment-system for
  fiat-to-crypto conversion at point of sale
- **Off-ramp:** for sellers cashing out, fiat
  disbursement via WIA-payment-system after on-chain
  settlement
- **Stablecoin settlement:** for partners preferring
  on-chain settlement, the boundary supports USDC, USDT,
  DAI, and other audited stablecoins

Currency-pair quotes flow from configured liquidity
providers; the boundary records the quote-at-execution
for dispute resolution.

## §6 KYC/AML compliance

For regulated marketplaces:

- Wallet-to-identity binding via WIA-identity-management
- Sanctions-list screening on every transfer of value
  exceeding deployment-declared thresholds
- PEP screening with EDD trigger for high-value
  transactions
- Travel-rule compliance for FATF-aligned transfers
  (where the regime adopts FATF recommendations)
- Suspicious-activity reporting per the regime's
  financial-intelligence-unit

Hits trigger investigations; transfers held until
cleared. Investigation logs are themselves audit-chained.

## §7 Royalty-distribution integration

For creators with multi-recipient royalty splits:

- Splits-contract integration (e.g., 0xSplits, Cobie's
  splits, per-deployment custom contracts)
- Off-chain split distribution where on-chain enforcement
  fails (marketplace-honour gap)
- Quarterly creator payment summary for tax-reporting

## §8 Licence-rights verification

For tokens conferring legal rights to underlying content:

- Licence text retrieved and SHA-256-verified against
  the chain commitment
- Owner's rights enumerated per the licence type
- Cross-domain references to WIA-supply-chain when the
  underlying asset is physical
- Notary-service integration for high-value licences
  (luxury items, real-estate-tokenisation)

## §9 Operational SLAs

| Concern                                        | Default SLA               |
|------------------------------------------------|---------------------------|
| Token discovery query p95                      | ≤ 500 ms                   |
| Mint orchestration → block confirmation        | per chain finality budget |
| Listing intake acknowledgement                 | ≤ 1 s                      |
| Reorg detection                                | ≤ 1 block depth            |
| Metadata pinning success                       | ≥ 99.9% across providers  |
| Audit-chain entry available                    | ≤ 10 s                     |

## §10 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Mint volume by chain and contract
- Transfer volume by chain and transfer-kind
- Royalty-policy distribution (BPS distribution)
- Marketplace-listing aggregate volumes
- Royalty-honour rate by marketplace
- Pinning-provider success rates
- RPC-provider quality metrics
- Reorg frequency by chain
- KYC/AML investigation rate
- Bridge transfer volumes and success rates
- Audit-chain integrity check results

## §11 Acceptance criteria

A deployment claims conformance when:

1. Every minted token has metadata pinned across ≥ 3
   pinning providers
2. Every transfer event is cross-checked across ≥ 2
   independent RPC providers
3. Royalty policy is queryable for every token-with-policy
4. Marketplace integrations honour the deployment's
   royalty-disclosure surface
5. KYC/AML screening is active for every transfer of
   value above threshold
6. Audit-chain entries match the on-chain state for the
   prior quarter
7. Quarterly compliance report has no integrity-check
   failures

## §12 Common pitfalls (informative)

- **Metadata centralisation drift** — many
  early-generation NFT projects shipped HTTPS metadata
  pointing to centralised servers that have since gone
  offline. The boundary's metadata-availability surface
  helps identify and migrate these to IPFS pinning
- **Royalty enforcement variance** — marketplace policies
  on royalty have shifted; deployments SHOULD periodically
  refresh per-marketplace honour status
- **Soulbound transfer attempts** — soulbound (ERC-5192)
  tokens cannot transfer; common error is listing them
  on marketplaces without honour-restriction; deployments
  SHOULD pre-filter
- **Bridge double-spend risk** — bridge protocols vary
  in security model; deployments SHOULD prefer
  audit-firm-reviewed bridges
- **Sanctions-list refresh latency** — sanctions lists
  update continuously; daily refresh is the floor

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table

| Reference                    | Use site                                           |
|------------------------------|----------------------------------------------------|
| WIA-payment-system           | fiat on/off-ramp settlement                        |
| WIA-identity-management      | KYC of marketplace participants                    |
| WIA-network-security         | TLS cipher-suite floor                             |
| WIA-pq-crypto                | post-quantum migration phase                       |

## Annex B — Decommissioning checklist

When an NFT marketplace or creator platform winds down:

- [ ] Active listings cancelled with seller notification
- [ ] In-flight bids refunded or matured
- [ ] Custodied funds released to creators per terms
- [ ] Metadata pinning continued via Filecoin contract
- [ ] Successor service identified for ongoing support
- [ ] Audit chain sealed and archive deposited

## Annex C — Conformance disclosure

Sections §1, §2, §4, §9, §11 are mandatory.
§3 is mandatory for deployments with marketplace functions.
§5 is mandatory for deployments accepting fiat.
§6 is mandatory for regulated jurisdictions.
§7 is mandatory for deployments with multi-recipient
royalties. §8 is mandatory for licence-bearing tokens.

## Annex D — Multi-chain creator workflow (informative)

A creator wishing to mint across multiple chains:

1. Single creative work; multiple per-chain deployments
2. Per-chain royalty policy declared
3. Cross-chain provenance via DID-bound creator identity
4. Bridge-tracking record per chain pair (PHASE 1 §9)
5. Aggregate analytics across chains in creator dashboard

## Annex E — Real-estate tokenisation (informative)

For deployments offering tokenised real-estate:

- Token represents fractional ownership in an underlying
  legal entity (LLC, SPV) holding the property
- Transfer restrictions for accredited-investor compliance
  per the regime
- Cross-domain references to WIA-supply-chain for
  property-condition reports
- Notary-service binding to legal contracts at mint and
  transfer
- Annual-report generation via the deployment's
  reporting pipeline

## Annex F — Music-rights tokenisation (informative)

For tokenised music-rights:

- Token represents a share in performance / mechanical /
  master-recording royalties
- Royalty policy includes split-by-rights-class
- Integration with PRO-equivalent royalty collection
  societies (ASCAP, BMI, KOMCA, JASRAC)
- Per-jurisdiction collection report

## Annex G — Carbon-credit tokenisation (informative)

For tokenised carbon credits:

- Token references the underlying credit registry
  (Verra, Gold Standard, KOC)
- Retirement burns the token (transfer to address(0)
  or custom retire function)
- Verifiable retirement evidence via VC-bound
  credential
- Cross-domain references to WIA-environmental-monitoring
  where the project has telemetry

## Annex H — Quarterly creator dashboard

Per-creator dashboard summarising:

- Tokens minted by chain and contract
- Transfer volume of creator's tokens
- Royalty paid (on-chain ERC-2981 + marketplace-honoured)
- Pinning-provider status across creator metadata
- Marketplace listing distribution
- Aggregate licence-rights status
- Audit-trail summary for the period

## Annex I — Capability-gap mitigation (informative)

For deployments where some integrations are partial:

- `metadataPinning=2-of-3` — two pinning providers
  available; mitigation: re-pin to a third
- `royaltyHonour=marketplace-mixed` — only some
  marketplaces honour; mitigation: surface
  per-marketplace status in listings UI
- `bridgeIntegration=incomplete` — bridge integration
  not deployed for a chain; mitigation: list bridges
  not yet supported in capability document
- `kycCoverage=high-value-only` — KYC active only
  above threshold; mitigation: declare threshold publicly

Each mitigation is recorded in the deployment policy and
in PHASE 4 §10 quarterly compliance report.
