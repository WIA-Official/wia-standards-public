# WIA-CITY-008: 3D Printing Construction — Specification Appendix

**Version:** 1.0
**Standard ID:** WIA-CITY-008
**Last Updated:** 2026-04-27

This appendix collects normative-supporting material that is shared across
PHASE-1 to PHASE-4 of the WIA-CITY-008 standard: error catalogue, conformance
profiles, reference checklists, and document maintenance procedure. It is
non-normative on its own; references from PHASE documents make individual
sections operative.

---

## A. Document Map

WIA-CITY-008 is organised as a master specification (`WIA-CITY-008-v1.0.md`)
with four PHASE documents and two cross-cutting companions:

| Document | Scope |
|---|---|
| `WIA-CITY-008-v1.0.md` | Master specification, scope, normative references |
| `PHASE-1-DATA-FORMAT.md` | Data schemas: building model, materials, jobs |
| `PHASE-2-API-INTERFACE.md` | REST API: projects, materials, jobs, errors |
| `PHASE-3-PROTOCOL.md` | Real-time WebSocket protocol, robot control |
| `PHASE-4-INTEGRATION.md` | Deployment, audit, lifecycle, governance |
| `SPEC-GLOSSARY.md` | Defined terms used across PHASE documents |
| `SPEC-APPENDIX.md` | This document |

Implementations claiming WIA-CITY-008 conformance MUST satisfy the requirements
of all four PHASE documents at the tier they declare.

---

## B. Error Code Catalogue

This section consolidates error codes used across the API and protocol surfaces
so implementers can wire a single error-handling spine. Codes are grouped by
domain. Each code MUST appear in error responses with the listed HTTP status
(for REST) or WebSocket close code where applicable.

### B.1 Authentication and Authorisation

| Code | HTTP | Meaning |
|---|---|---|
| `AUTH_REQUIRED` | 401 | Endpoint requires authentication |
| `AUTH_INVALID` | 401 | Token invalid or expired |
| `AUTH_FORBIDDEN` | 403 | Caller authenticated but not authorised |
| `AUTH_SCOPE_INSUFFICIENT` | 403 | Token lacks required OAuth scope |

### B.2 Project Lifecycle

| Code | HTTP | Meaning |
|---|---|---|
| `PROJECT_NOT_FOUND` | 404 | Project ID does not resolve |
| `PROJECT_LOCKED` | 409 | Project is in a state that disallows the requested mutation |
| `PROJECT_DUPLICATE_REGISTRATION` | 409 | A project with the same canonical key already exists |
| `PROJECT_INVALID_DIMENSIONS` | 422 | Provided dimensions fail validation rules |

### B.3 Material and Inventory

| Code | HTTP | Meaning |
|---|---|---|
| `MATERIAL_NOT_FOUND` | 404 | Material lot or class does not resolve |
| `MATERIAL_INSUFFICIENT` | 409 | Insufficient material for requested job |
| `MATERIAL_INCOMPATIBLE` | 422 | Material incompatible with declared printer / mix |

### B.4 Print Jobs

| Code | HTTP | Meaning |
|---|---|---|
| `JOB_NOT_FOUND` | 404 | Job ID does not resolve |
| `JOB_INVALID_GCODE` | 422 | G-code reference is malformed or unreachable |
| `JOB_PRINTER_OFFLINE` | 503 | Target printer is not currently reachable |
| `JOB_CURE_WINDOW_VIOLATION` | 409 | Operation conflicts with declared cure window |

### B.5 Generic

| Code | HTTP | Meaning |
|---|---|---|
| `RATE_LIMIT_EXCEEDED` | 429 | Caller exceeded the configured request budget |
| `INTERNAL_ERROR` | 500 | Unhandled server fault — operators must inspect logs |
| `MAINTENANCE` | 503 | Service intentionally offline for maintenance window |

Error response body MUST include at least the `code` field and SHOULD include a
human-readable `message`. Additional fields (`details`, `retryAfter`,
`correlationId`) MAY be included but MUST NOT be required by clients to
disambiguate the error.

---

## C. Conformance Profiles

WIA-CITY-008 defines three conformance profiles. An implementation declares the
profile under which it claims conformance; auditors verify only the requirements
mapped to that profile.

### C.1 Basic Profile

Targeted at small-scale deployments (single printer, single project pipeline).

- PHASE-1: building model, material lot, single print job
- PHASE-2: project register, material register, job submission, status query
- PHASE-3: WebSocket connect, status stream, single-channel sensor stream
- PHASE-4: lifecycle states, basic audit log

### C.2 Full Profile

Targeted at multi-printer integrators and contractors.

All Basic Profile requirements plus:
- PHASE-1: multi-segment job graphs, reinforcement schemas
- PHASE-2: webhook subscriptions, advanced filtering, pagination links
- PHASE-3: robot control protocol, batched sensor streams, QoS levels
- PHASE-4: signed audit packages, evidence retention, role-based audit access

### C.3 Advanced Profile

Targeted at certifiers, regulators, and platforms operating multi-tenant
infrastructure.

All Full Profile requirements plus:
- PHASE-1: cross-project material traceability schemas
- PHASE-2: tenancy headers, policy negotiation
- PHASE-3: priority preemption, deterministic interlocks (PTP)
- PHASE-4: cross-vendor crosswalk reporting, public conformance evidence

---

## D. Conformance Verification Procedure

The procedure below is recommended for parties seeking a third-party check of
their WIA-CITY-008 conformance claim. It is non-normative; auditors may use
equivalent methods that produce the same evidence types.

1. **Profile declaration** — implementation publishes the declared conformance
   profile and the deployment scope (single-tenant, multi-tenant, federated).
2. **Static review** — auditor reviews configuration, schemas, OpenAPI
   document, and lifecycle policies against the declared profile.
3. **Dynamic exercise** — auditor exercises representative endpoints and
   protocol messages against a running deployment using the published test
   vectors. Negative results are reported with the same fidelity as positive
   results.
4. **Evidence package** — auditor collects the SBOM, OpenAPI document,
   test-vector matrix, and signed manifest into a single tarball. Signatures
   use Sigstore (DSSE envelope, Rekor transparency log entry).
5. **Public summary** — implementation publishes a conformance summary citing
   the profile, the audit window, and the location of the evidence package.

The audit window SHOULD be no longer than 12 months; longer windows weaken the
public's ability to interpret the conformance claim against current behaviour.

---

## E. Maintenance and Versioning

The maintenance procedure for WIA-CITY-008 is governed by the WIA Standards
working group and applies to all PHASE documents simultaneously, except where a
PHASE-specific maintenance window is announced in writing.

- **Patch releases** (`v1.0.x`) clarify text without changing requirements;
  conforming implementations need no rework.
- **Minor releases** (`v1.x.0`) add optional capability or relax a requirement;
  existing conforming implementations remain conforming.
- **Major releases** (`v2.0.0`) may break compatibility; the working group
  publishes a migration guide and overlaps the prior major version on the
  reference implementation for at least 90 days.

Comments and proposed amendments are accepted on the
`WIA-Official/wia-standards` repository. Working group decisions are recorded
in the version-history table at the start of each PHASE document.

---

## F. Document Conventions

- **MUST / MUST NOT / SHOULD / SHOULD NOT / MAY** are interpreted per
  RFC 2119 and RFC 8174 when in uppercase.
- **Identifiers** in code blocks are illustrative unless the surrounding text
  declares them normative.
- **Examples** are non-normative even when they appear inside a section that
  contains normative text.
- **Numerical figures** in narrative passages are descriptive of typical field
  conditions and are NOT normative requirements; normative figures appear in
  tables or are explicitly labelled as MUST/SHOULD targets.

---

## G. References

The following references are cited from PHASE documents and the master
specification. Inclusion here is for navigational convenience; only references
explicitly cited from a normative section bear normative weight.

- ISO/IEC 27001 — Information security management systems
- ISO/IEC 17065:2012 — Conformity assessment requirements
- IEC 61508 — Functional safety of electrical/electronic systems
- ASTM C1583 — Standard test method for tensile strength of concrete surfaces
- RFC 2119 / RFC 8174 — Key words for use in RFCs
- RFC 6455 — The WebSocket Protocol
- RFC 7519 — JSON Web Token (JWT)
- W3C JSON Schema — Schema language for JSON
- OpenAPI 3.1 — API description language
- CycloneDX 1.5 / SPDX 2.3 — Software bill of materials formats
- Sigstore (DSSE / Rekor) — Software signing and transparency

---

**License:** CC BY 4.0
**弘益人間 (Benefit All Humanity)**
