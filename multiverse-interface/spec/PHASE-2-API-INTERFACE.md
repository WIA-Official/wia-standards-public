# WIA-multiverse-interface PHASE 2 — API Interface Specification

**Standard:** WIA-multiverse-interface (WIA-QUA-017)
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-
multiverse-interface participants expose so that
research consortia, simulation hubs, observation
arrays, ethics-review boards, and audit authorities
can coordinate hypotheses, scenarios, checkpoints,
observations, and provenance graphs through a
single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT)
- IETF RFC 9421 (HTTP Message Signatures)
- W3C PROV-O 1.0, W3C SHACL, W3C JSON-LD 1.1
- W3C VC 2.0, W3C DID 1.0
- HDF5 (The HDF Group, informative), Zarr v3 (informative)
- ISO 8601, ISO/IEC 27001:2022

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between research consortia, simulation hubs,
observation arrays, ethics-review boards, and audit
authorities for the multiverse-interface research
domain.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/protocols`     | research-protocol registry                       |
| `/v1/hypotheses`    | universe-hypothesis registry                     |
| `/v1/scenarios`     | scenario registry                                |
| `/v1/checkpoints`   | simulation checkpoint store                      |
| `/v1/observations`  | observation record store                         |
| `/v1/identities`    | identity-preservation registry                   |
| `/v1/timelines`     | timeline records                                 |
| `/v1/events`        | decoherence-event log                            |
| `/v1/provenance`    | PROV-O provenance graph                          |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints accept anonymous or JWT bearer
access per the consortium's policy. Write endpoints
require a JWT bound to a registered researcher
identity (DID) within an active research protocol.

## §4 Protocol operations

### 4.1 Register

```
POST /v1/protocols
```

Body: protocol record (PHASE-1 §2) signed by the
principal investigator. The registry verifies the
ethics-review reference resolves at the issuing
IRB / IEC / REC.

### 4.2 Lookup

```
GET /v1/protocols/{protocolRef}
```

Returns the canonical protocol record. Conditional
GET via `ETag`.

## §5 Hypothesis operations

### 5.1 Publish hypothesis

```
POST /v1/hypotheses
```

Body: hypothesis record (PHASE-1 §3) including
peer-reviewed paper references.

### 5.2 Falsifiability check

```
POST /v1/hypotheses/{hypothesisRef}/falsifiability-check
```

Body: an observation reference. Response: a
verdict on whether the observation reaches the
declared falsification criterion.

## §6 Scenario operations

### 6.1 Submit scenario

```
POST /v1/scenarios
```

Body: scenario record (PHASE-1 §4).

### 6.2 Run

```
POST /v1/scenarios/{scenarioRef}/run
```

Body: runtime profile reference and any input
overrides. Response: a job reference; checkpoints
are emitted to the registered storage.

## §7 Checkpoint operations

### 7.1 Submit

```
POST /v1/checkpoints
```

Body: checkpoint record (PHASE-1 §5) plus an
attached payload (HDF5 / Zarr / Parquet).

### 7.2 Resume

```
GET /v1/checkpoints/{checkpointRef}/resume
```

Returns the runtime profile and the storage URI
sufficient to resume the simulation from the
checkpoint.

## §8 Observation operations

### 8.1 Submit observation

```
POST /v1/observations
```

Body: observation record (PHASE-1 §6) with
calibration reference.

### 8.2 Lookup

```
GET /v1/observations/{observationRef}
```

Returns the observation record, signed by the
operator.

## §9 Identity-preservation operations

### 9.1 Register claim

```
POST /v1/identities
```

Body: identity-preservation record (PHASE-1 §7)
with consent reference.

### 9.2 Withdraw claim

```
DELETE /v1/identities/{identityRef}
```

Tombstones the claim and notifies all consortia
that referenced the identity.

## §10 Timeline operations

### 10.1 Submit timeline

```
POST /v1/timelines
```

Body: timeline record (PHASE-1 §8).

### 10.2 Branch graph

```
GET /v1/timelines/{timelineRef}/branches
```

Returns the branch and merge graph for inspection
by the consortium's reproducibility team.

## §11 Event log operations

### 11.1 Append event

```
POST /v1/events
```

Body: decoherence-event log entry (PHASE-1 §9).

### 11.2 Query

```
GET /v1/events?scenarioRef={ref}&since=<ISO8601>
```

Returns the events bound to the scenario in the
window.

## §12 Provenance operations

### 12.1 Publish PROV-O

```
POST /v1/provenance
```

Body: a PROV-O 1.0 graph. The registry validates
against the SHACL shape for the consortium.

### 12.2 Walk

```
POST /v1/provenance/walk
```

Body: starting URI and depth. Response: the
provenance graph rooted at the URI.

## §13 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/multiverse-interface/`.

## §14 Caching and rate limits

Hypothesis, scenario, and provenance read endpoints
are cached. Checkpoint and observation endpoints
are not (their content is immutable but may be
gated behind the consortium's storage policy).

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-multiverse-interface API, version: 1.0.0}
paths:
  /v1/scenarios:
    post:
      summary: Submit a scenario
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'ScenarioRecord.schema.json'}
      responses:
        '201': {description: Scenario submitted}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `protocol.registered`,
`hypothesis.published`, `scenario.run`,
`checkpoint.submitted`, `observation.signed`,
`identity.registered`, `event.logged`. Delivery is
signed with HMAC-SHA-256.

## Annex D — Federation

Federation between consortia follows the discovery
contract in PHASE-3. Cross-consortium queries
carry an `X-WIA-Federation-Path` header.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL to
a `tar.zst` of the deployment's records filtered
by protocol and date range.

## Annex F — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
protocols and ephemeral state.

## Annex G — Quotas

Per-consortium publish quotas default to 1,000
checkpoints per hour and 5,000 observations per
day.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>` returns
mutating-operation events.

## Annex I — Public introspection

`GET /v1/registry/stats` returns aggregate
counters.

## Annex J — Webhook payload shape

```json
{
  "event": "checkpoint.submitted",
  "checkpointRef": "f63f4f04-...",
  "scenarioRef": "https://muv.example.org/scenarios/cosmo-001",
  "wallClockTime": "2026-04-28T11:32:00+09:00"
}
```

## Annex K — Researcher access

```
POST /v1/registry/researcher-access
```

Body: a SHACL filter and a project identifier.
Response: scoped JWT for the requested data
extracts.

## Annex L — Bulk checkpoint upload

```
POST /v1/checkpoints/bulk
```

Body: a manifest enumerating checkpoints and their
storage URIs. The registry validates each digest
asynchronously and surfaces the verdict per
checkpoint via webhook.

## Annex M — Branch graph endpoint

```
POST /v1/timelines/{timelineRef}/branches/diff
```

Body: two timelines. Response: the structural diff
of branch and merge points so that consortia can
reconcile divergent timeline interpretations.

## Annex N — Provenance subscriptions

```
POST /v1/provenance/subscriptions
```

Body: a SHACL filter over the provenance graph.
Subscribers receive PROV-O fragment updates as
graph nodes are added.

## Annex O — Cohort enumeration

```
GET /v1/identities?cohort={cohortRef}
```

Returns the cohort's identity-preservation
records. Filter by `consentValid` to include only
identities with active consent.

## Annex P — Storage policy negotiation

```
GET /v1/checkpoints/{checkpointRef}/storage-policy
```

Returns the deployment's storage policy for the
checkpoint: retention window, replication count,
encryption-at-rest scheme, access policy.

## Annex Q — Job lifecycle webhooks

Simulation jobs emit lifecycle webhooks at
`scheduled`, `started`, `checkpointed`,
`completed`, `failed`. The webhook payload carries
the consortium signature and the runtime profile so
that downstream observers can verify job
provenance.

## Annex R — Audit-grade observation export

```
POST /v1/observations/audit-export
```

Body: a date range and instrument filter. Response:
a signed URL referencing a `tar.zst` of
observations and their calibration records,
suitable for end-to-end audit of an instrument's
output during the window.

## Annex S — Cross-protocol replication

```
POST /v1/protocols/{srcRef}/replicate
```

Body: target consortium and protocol shape filter.
Initiates a guarded replication of records into the
target protocol, applying the target consortium's
ethics-review and consent gates.

## Annex T — Hardware fingerprint surface

```
GET /v1/checkpoints/{checkpointRef}/hardware
```

Returns the hardware fingerprint that produced the
checkpoint: CPU/GPU/QPU model, count, interconnect
fabric, OS kernel and microcode version. Used by
reproducibility teams to locate matching compute
environments.

## Annex U — Open-access landing pages

```
GET /v1/registry/open-access?artefactRef={ref}
```

Returns the public landing page metadata for an
artefact under an open-access mandate. Includes
licence, embargo state, and DOI when minted.

## Annex V — Consent withdrawal cascade

When an identity-preservation consent is withdrawn,
the registry cascades the withdrawal to every
record referencing the identity: observations are
re-pseudonymised; aggregate counters update.

## Annex W — Federation peer trust matrix

```
GET /v1/registry/federation
```

Returns the federation peer list with each peer's
accepted operation groups, JWKS URL, and trust
state (`active`, `provisional`, `revoked`). Used by
clients to choose which peers to query when local
results are insufficient.

## Annex X — Long-running job audit

```
GET /v1/jobs/{jobRef}/audit
```

Returns the per-job audit trail: scheduling,
checkpoint emissions, errors, and the principal
investigator's annotations. The trail is signed by
the consortium so that downstream auditors can
verify it end-to-end.

## Annex Y — Researcher onboarding endpoint

```
POST /v1/registry/researchers
```

Body: a researcher record with affiliation
attestation, ethics-training certificate
identifier, and signing key set. Response: the
registered researcher reference.

## Annex Z — Reproducibility request

```
POST /v1/scenarios/{scenarioRef}/reproduce
```

Body: declared runtime profile, optional input
override set. Response: a job reference that
emits checkpoints; consumers compare reproduced
hashes against the originals.

## Annex AA — Cross-protocol comparison

```
POST /v1/protocols/compare
```

Body: two protocol references. Response: structural
diff at the hypothesis, scenario, and consent
level so that consortia can plan harmonisation
work.

弘益人間 (Hongik Ingan) — Benefit All Humanity
