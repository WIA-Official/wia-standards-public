# WIA-COMP-016 — Phase 2: API Interface

> Software-license canonical Phase 2: API surface (declarations + SBOM + copyrights + commercial keys).

# WIA-COMP-016: Software License Specification v1.0

> **Standard ID:** WIA-COMP-016
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Computing Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [License Types and Categories](#2-license-types-and-categories)
3. [SPDX Integration](#3-spdx-integration)
4. [License Compatibility](#4-license-compatibility)
5. [Commercial Licensing](#5-commercial-licensing)
6. [Dependency Management](#6-dependency-management)
7. [Compliance and Auditing](#7-compliance-and-auditing)
8. [License Templates](#8-license-templates)
9. [Copyright Management](#9-copyright-management)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---


## 5. Commercial Licensing

### 5.1 Dual Licensing

**Model:**
```
Dual License Structure:
1. Open Source License (e.g., AGPL-3.0)
   - Free for open source use
   - Requires source disclosure

2. Commercial License
   - Paid license
   - No source disclosure required
   - Additional support and features
```

**Example: MySQL Model**
```
Options:
- GPL: Free, must open source your application
- Commercial: Paid, can keep your application proprietary
```

### 5.2 License Tiers

```
Common Licensing Tiers:

1. Community Edition
   - Open source (MIT, Apache, GPL)
   - Basic features
   - Community support

2. Professional Edition
   - Commercial license
   - Advanced features
   - Email support
   - $99-999/year

3. Enterprise Edition
   - Commercial license
   - All features
   - SLA, phone support
   - Custom pricing
```

### 5.3 Proprietary License Terms

```
Typical EULA Components:
1. Grant of License
   - Scope of use
   - Number of users/devices
   - Territorial restrictions

2. Restrictions
   - No reverse engineering
   - No redistribution
   - No modification

3. Ownership
   - Vendor retains all rights
   - Customer has usage rights only

4. Warranty and Liability
   - Limited or no warranty
   - Liability disclaimers

5. Term and Termination
   - License duration
   - Termination conditions
```

---



## 9. Copyright Management

### 9.1 Copyright Ownership

```
Copyright Holder Types:
1. Individual: "Copyright 2025 John Doe"
2. Company: "Copyright 2025 SmileStory Inc."
3. Multiple: "Copyright 2025 John Doe and SmileStory Inc."
4. Range: "Copyright 2020-2025 SmileStory Inc."
```

### 9.2 Copyright Transfer

```
Copyright Assignment:
- Contributor License Agreement (CLA)
- Assigns copyright to project owner
- Required by some projects (Apache, Google)

Example CLA:
"I hereby assign copyright in my contributions to {Project},
 to be licensed under {License}."
```

### 9.3 Copyright Notice

```
Placement:
1. LICENSE file: Full license text
2. README: License badge and summary
3. Source files: Header comment
4. NOTICE: Additional attribution
5. About dialog: In application UI
```

---




---

## A.1 Endpoint reference

```http
POST /license/v1/declarations              # declare outbound license
GET  /license/v1/declarations/{id}         # fetch declaration
POST /license/v1/sboms                     # submit SBOM
GET  /license/v1/sboms/{id}/components     # list components + licenses
POST /license/v1/copyrights                # register copyright assertion
GET  /license/v1/copyrights/{id}/chain     # view chain of title
POST /license/v1/keys                      # register signing key
```

Every endpoint follows the discovery convention at `/.well-known/wia-software-license`.

## A.2 Outbound-license declaration

`POST /declarations` accepts a project identifier, the SPDX expression (Phase 1 §A.1), the source path of the LICENSE file, and an optional list of contributor agreements. The endpoint returns a stable declaration identifier and the canonical hash of the declared license text. Subsequent SBOM submissions reference the declaration identifier.

## A.3 SBOM submission and analysis

`POST /sboms` accepts a CycloneDX or SPDX document and returns an analysis envelope: per-component license, per-component compatibility status, copyleft trigger flags, and a risk score per the Phase 3 compatibility model. Bulk submissions are paginated; the analysis runs asynchronously and the result becomes available at `GET /sboms/{id}/components` once complete.

## A.4 Copyright registration

`POST /copyrights` registers a copyright assertion: the work identifier, the asserting entity, the asserted year(s), the legal basis (original authorship, work-for-hire, assignment), and supporting evidence (DCO sign-off, CLA reference, assignment instrument). The chain-of-title endpoint returns the full lineage of assertions for a given work, signed with the registry's Ed25519 key.

## A.5 Commercial-license issuance

Commercial-license endpoints handle quote → order → license-key issuance → renewal. Each issued key carries the licensee identity, the licensed product version range, the seat count or usage cap, the expiry, and an Ed25519 signature over the entirety. Activation runs through the key endpoint with optional offline-activation challenges signed by the customer's installation.

## A.6 Rate-limit envelope

1000 req/h unauthenticated (read-only paths only), 5000 req/h authenticated, 10000 req/h premium tier. Bulk SBOM submissions count their components against an additional per-tenant quota.

## A.7 Webhook events

Webhooks fire on `declaration.created`, `sbom.analysed`, `compatibility.changed`, `copyright.registered`, `key.activated`, `key.revoked`, and `audit.completed`. Delivery is at-least-once with HMAC-SHA256 signing per the WIA family policy; receivers dedupe on `deliveryId`. Retry policy: 3 attempts at 1s/4s/16s, then dead-letter queue and operations page.

## A.8 Bulk operations

Bulk SBOM submissions accept up to 1,000 components per request. Bulk declaration imports accept up to 10,000 declarations per request. The response carries per-entry success/failure status; partial-success is the normal case so clients MUST inspect the per-entry status rather than assuming whole-batch atomicity. Idempotency keys (per IETF RFC 9221 / draft-idempotency-header) are honoured.

## A.9 Versioning and deprecation

API versioning is path-based (`/v1/`, `/v2/`). New optional fields are added without bumping the major version; field renames or removals require a major bump with a 12-month deprecation window per IETF RFC 8594 and 9745. The `Deprecation` and `Sunset` response headers advertise the deprecation timeline so clients can migrate ahead of removal.

## A.10 Idempotency and retry semantics

Mutating endpoints accept an `Idempotency-Key` header (UUID or random 128-bit token). The server caches the response for 24 hours keyed by the tuple `(tenant_id, endpoint, idempotency_key)`. Replay of the same key returns the cached response without re-executing; replay with a different request body returns `409 Conflict`. Read endpoints are inherently idempotent and ignore the header.

## A.11 Pagination, sort, filter

Cursor-based pagination via `?after=cursor&limit=N` with N capped at 100. Sort via `?sort=field&order=asc|desc`; multi-field sort via comma-separated list. Filtering uses `?field=value` for equality, `?field[op]=value` for `gte` / `lte` / `in` / `like`. The cursor is opaque; clients MUST NOT decode it. Responses carry `Link` headers per IETF RFC 8288.

## A.12 Authentication surfaces

OAuth 2.0 + OpenID Connect for human-driven sessions. Mutual TLS for system-to-system traffic. API tokens (per-tenant, scoped, rotated every 90 days) for the SDK auth surface. The auth surface advertises its capabilities at `/.well-known/openid-configuration` and `/.well-known/oauth-authorization-server`. Token introspection follows IETF RFC 7662; revocation follows RFC 7009.

## A.13 GraphQL surface (advisory)

A GraphQL surface mirrors the REST endpoints for tenants that prefer a single graph. Query types cover `licenseDeclaration`, `sbom`, `component`, `copyrightAssertion`, `commercialKey`; mutation types cover the create / update / revoke flows; subscription types cover the webhook event set. The schema is published at `https://wiastandards.com/software-license/graphql/schema.graphql`.

## A.14 Concrete example — declaring outbound license

```http
POST /license/v1/declarations HTTP/1.1
Authorization: Bearer eyJhbGc...
Content-Type: application/json
Idempotency-Key: 4f3d2c1b-9e8a-4b7c-8d6f-1a2b3c4d5e6f

{
  "projectId": "github.com/example/widget",
  "licenseExpression": "Apache-2.0",
  "licenseTextHash": "sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
  "sourcePath": "LICENSE",
  "declaringEntity": {
    "name": "Example Inc.",
    "contact": "compliance@example.com"
  },
  "rationale": "OSI-approved permissive license; compatible with downstream commercial deployment"
}
```

Response:

```json
{
  "declarationId": "decl_01HZX9...",
  "verifiedHash": "sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
  "status": "active",
  "createdAt": "2026-04-28T07:32:00Z"
}
```


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/software-license/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-software-license-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/software-license-host:1.0.0` ships every software-license envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/software-license.sh` ships sample envelope generators with no
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
ecosystem. Software-license deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
