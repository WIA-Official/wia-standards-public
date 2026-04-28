# WIA-BIO-021 — Phase 3: Protocol

> Synthetic-bio-registry canonical Phase 3: protocols (version control + access + federation).

# WIA-BIO-021: Synthetic Biology Registry Specification v1.0

> **Standard ID:** WIA-BIO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Part Classification System](#2-part-classification-system)
3. [Metadata Standards](#3-metadata-standards)
4. [Sequence Formats](#4-sequence-formats)
5. [SBOL Compliance](#5-sbol-compliance)
6. [Characterization Data](#6-characterization-data)
7. [Version Control](#7-version-control)
8. [Access Control and Licensing](#8-access-control-and-licensing)
9. [Registry Interoperability](#9-registry-interoperability)
10. [Implementation Guidelines](#10-implementation-guidelines)

---


## 7. Version Control

### 7.1 Semantic Versioning

Parts use semantic versioning: `MAJOR.MINOR.PATCH`

**Version Rules**:
- `MAJOR`: Incompatible sequence changes
- `MINOR`: Backward-compatible additions (new characterization data)
- `PATCH`: Bug fixes, documentation updates

**Examples**:
- `1.0.0` → `1.1.0`: Added characterization data
- `1.1.0` → `2.0.0`: Sequence optimization (incompatible)
- `1.1.0` → `1.1.1`: Fixed typo in description

### 7.2 Version History

```json
{
  "versionHistory": [
    {
      "version": "1.0.0",
      "date": "2024-06-15T10:30:00Z",
      "author": "initial-creator",
      "changes": "Initial part submission",
      "sequenceHash": "sha256:abc123..."
    },
    {
      "version": "1.1.0",
      "date": "2024-08-20T14:45:00Z",
      "author": "lab-member-2",
      "changes": "Added fluorescence characterization",
      "sequenceHash": "sha256:abc123..."
    },
    {
      "version": "2.0.0",
      "date": "2024-10-01T09:00:00Z",
      "author": "optimization-team",
      "changes": "Codon optimization for mammalian cells",
      "sequenceHash": "sha256:def456..."
    }
  ]
}
```

### 7.3 Deprecation Policy

When parts are deprecated:

```json
{
  "deprecation": {
    "deprecated": true,
    "date": "2024-11-15",
    "reason": "Superseded by BBa_K789012 with improved expression",
    "replacementPart": "BBa_K789012",
    "stillAvailable": true
  }
}
```

---



## 8. Access Control and Licensing

### 8.1 Licensing Options

Supported licenses:

| License | Code | Description |
|---------|------|-------------|
| CC-BY-4.0 | CC-BY | Attribution required |
| CC-BY-SA-4.0 | CC-BY-SA | Share-alike required |
| CC0-1.0 | CC0 | Public domain |
| MIT | MIT | Permissive software license |
| Proprietary | PROP | Custom terms required |
| OpenMTA | OMTA | Open Material Transfer Agreement |

### 8.2 Access Levels

```json
{
  "accessControl": {
    "visibility": "public|private|restricted",
    "permissions": {
      "view": ["public"],
      "download": ["registered-users"],
      "modify": ["part-owner", "admin"],
      "characterize": ["registered-users"]
    },
    "embargoUntil": "2025-01-01T00:00:00Z",
    "requiresAgreement": true,
    "agreementUrl": "https://example.com/mta.pdf"
  }
}
```

### 8.3 Material Transfer

```json
{
  "materialTransfer": {
    "available": true,
    "provider": "Addgene",
    "catalogNumber": "12345",
    "cost": 65.00,
    "currency": "USD",
    "shippingRestrictions": ["country:CN", "country:IR"],
    "requiresMTA": true
  }
}
```

---




---

## A.1 Version-control protocol

Part records are immutable once published. Updates create a new version with a new identifier; the prior version is preserved with a `replacedBy` link and a deprecation notice. The version history is a directed acyclic graph that consumers can walk to understand evolution. Forks (independent edits to a shared ancestor) are permitted; merges follow the explicit-acknowledgement model with the contributing institutions co-signing the merge.

## A.2 Access control and licensing

Access tiers: public (read-only, no sign-in), institutional (full read + contribution rights for affiliated members), private (per-project; default for in-progress designs). Licensing terms attached to a part: open use with attribution (e.g., CC-BY 4.0), open use under copyleft (CC-BY-SA 4.0 or OpenMTA), restricted use (academic only, no commercial). The protocol exposes the licence and the contributor declarations on every read; commercial users that intersect a restricted-licence part receive a typed `licensing.restricted` event.

## A.3 Federation between registries

Federation lets a query at one registry surface results from peer registries (iGEM Registry, JBEI Public Inventories, Addgene, IGEM Parts Registry, institutional repositories). The federation envelope carries the source registry, the source part identifier, the resolution policy (cache vs. live), and the trust class. Trust class drives whether a downstream consumer can rely on the federated record without independently verifying.

## A.4 Provenance graph

Every part keeps a provenance graph that names the design tool (Benchling, SnapGene, Geneious, j5, RAVEN, Cello), the design parameters, the assembly method (Phase 4 §A.4 cross-walk), the sequencing verification, and the lab that performed each step. The graph is signed by the responsible institution at every step so downstream consumers can verify the chain without having to ask.

## A.5 Conflict and curation

When two contributors submit overlapping characterisation data with conflicting headline values, the protocol layer flags the conflict and routes it to the registry curator. The curator may reconcile (one supersedes the other), retain both with annotations, or escalate to the contributing institutions' principal investigators. The conflict envelope is preserved in the audit log for downstream researchers to consult.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane calls. Bulk-import submissions are idempotent on the part-identifier set hash; duplicate submissions short-circuit to the cached result.

## A.7 Audit log retention

Audit-log retention follows institutional policy bounded by the lower of: 10 years (academic provenance norm) or the donor-consent window (where parts are derived from human or non-public material). The audit log carries: actor, action, resource_id, before-state hash, after-state hash, and a Ed25519 signature by the actor's institutional key. Tampering is detectable because the prior signature appears in the next entry's payload (hash chain).

## A.8 Curation-board governance

The curation board reviews disputed records, deprecation requests, and federation policy changes. Membership is rotating across contributing institutions, with a 2-year staggered term. Decisions are published with the dissent record so the community can see the trade-offs that drove a ruling. Curator decisions are appealable once to the full board with new evidence.

## A.9 Trust-class downgrade rules

Federated peers carry a trust class derived from: (a) declared support for the IGSC Harmonized Screening Protocol, (b) signing-key custody (HSM-backed, software-only, or shared), (c) audit-log retention (≥10 years, ≥5 years, ≥1 year, undeclared), (d) curation-board membership status. Peers with weaker classes have their records flagged at retrieval time so downstream consumers can decide whether to trust them for a given purpose.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-bio-registry/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-bio-registry-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-bio-registry-host:1.0.0` ships every synthetic-bio-registry envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-bio-registry.sh` ships sample envelope generators with no
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
ecosystem. Synthetic-bio-registry deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
