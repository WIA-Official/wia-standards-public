# WIA-medical-data-privacy PHASE 2 — API Interface Specification

**Standard:** WIA-medical-data-privacy
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a privacy-aware system exposes to
clinical applications, research pipelines, regulators, and data subjects.
The shape is FHIR R5 RESTful, layered with this standard's privacy
extensions: every read or disclosure carries an explicit purpose, every
purpose is gated by an active consent or break-glass override, and every
event emits a hash-chained AuditEvent.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API (R5/http.html), Bundle, OperationOutcome
- HL7 SMART App Launch 2.2 (smarthealthit.org/specification) — for app authentication and scope strings
- IETF RFC 9457 (Problem Details), RFC 7807 (legacy Problem Details for FHIR-pre-R5 clients)
- IETF RFC 7515 (JWS), RFC 7519 (JWT) — receipt and audit signature
- IETF RFC 8615 (well-known URIs)
- ISO 8601 (timestamps)
- Open Banking-style mTLS profile is *not* required by this PHASE

---

## §1 Purpose-bearing requests

Every request that touches PHI carries a purpose. The purpose is
conveyed by the SMART scope string and a request header that reflects
the actual purpose for the call (a single SMART grant can authorise
multiple purposes; the call must declare which is in use).

```
GET /Patient/<id>
Authorization: Bearer <jwt>
WIA-Purpose-Of-Use: TREAT
WIA-Justification: routine encounter, scheduled 2026-04-27
Accept: application/fhir+json
```

The server validates that:
1. The JWT carries an active consent grant whose `provision.purpose`
   includes `TREAT`.
2. The header purpose matches one of the granted purposes.
3. The grantee in the consent matches the principal in the JWT.
4. The current time is inside the consent's `provision.period`.

If any check fails, the server returns `403 Forbidden` with an
RFC 9457 Problem Details body whose `type` URI is one of the
errors enumerated in §6.

## §2 Resource endpoints

The following FHIR R5 endpoints are exposed:

| Endpoint              | Methods               | Privacy gate                                      |
|-----------------------|-----------------------|---------------------------------------------------|
| `/Consent`            | GET, POST, PUT        | controller-only for write; subject-or-grantee-readable |
| `/AuditEvent`         | GET                   | controller, regulator, or subject (own events)    |
| `/Provenance`         | GET                   | piggybacks on the resource it traces              |
| `/Patient/<id>`       | GET, PATCH            | requires active consent for the declared purpose  |
| `/<R>/<id>`           | GET                   | requires active consent for the declared purpose  |
| `/<R>?<search>`       | GET                   | results filtered to consented subset (§3)         |
| `/Bundle/$transaction`| POST                  | each entry independently gated                    |

Resources outside this set inherit the `<R>/<id>` rules.

## §3 Search filtering (minimum necessary)

A search query returns *only* the subset of resources consented for
the declared purpose. Filtering is applied server-side, not client-side.
The server MUST NOT leak the existence of records the requester is
not authorised to see — the response body and the count both reflect
only the authorised subset.

If the search would return resources that exist but are filtered
out, the server adds a `link` of `relation: prohibited` whose `url`
is the URI of the consent record explaining why those records are
excluded. This is auditable but does not leak record content or count.

## §4 Consent management

```
POST /Consent
Content-Type: application/fhir+json
WIA-Purpose-Of-Use: HOPERAT
```

Body is a FHIR R5 Consent resource conforming to PHASE 1 §3.
The server signs the resource (JWS detached) and returns:

```
201 Created
Location: /Consent/<id>
WIA-Consent-Receipt-Sig: eyJhbGciOiJFUzI1NiJ9..<sig>
```

Updates to consent (`PUT /Consent/<id>`) preserve the prior version
in version history. Withdrawing consent is a `PUT` that flips
`status` to `inactive`; it does not delete the resource because
audit history must reference the prior consent that authorised the
disclosure.

## §5 Break-glass override

```
POST /Patient/<id>/$emergency-access
Content-Type: application/json
WIA-Purpose-Of-Use: ETREAT

{
  "reason": "patient unconscious, anaphylaxis suspected, need allergen list",
  "scope": ["AllergyIntolerance", "MedicationRequest", "Condition"],
  "expectedDuration": "PT4H"
}
```

The server accepts the override (`200 OK`) only if the asserting
principal has the `ETREAT` capability in their SMART grant. The
override creates an active short-lived consent record (with `status:
active`, `provision.purpose: ETREAT`, period bounded by
`expectedDuration`) and emits an AuditEvent with `severity: notice`
that is delivered to the controller's review queue.

## §6 Error model (Problem Details)

All non-2xx responses use RFC 9457 Problem Details. The following
`type` URIs are reserved:

| URI                                              | Status | Meaning                                              |
|--------------------------------------------------|-------:|------------------------------------------------------|
| `urn:wia:mdp:problem:no-active-consent`          | 403    | No consent matches the declared purpose              |
| `urn:wia:mdp:problem:wrong-jurisdiction`         | 403    | Record is in a jurisdiction the requester cannot reach |
| `urn:wia:mdp:problem:purpose-mismatch`           | 403    | Header purpose differs from JWT scope                |
| `urn:wia:mdp:problem:break-glass-not-authorised` | 403    | Principal lacks `ETREAT` capability                  |
| `urn:wia:mdp:problem:linkage-not-authorised`     | 403    | Cross-purpose linkage has no active authorisation    |
| `urn:wia:mdp:problem:retention-expired`          | 410    | Record retention has lapsed; cannot be served        |

Body example:

```json
{
  "type": "urn:wia:mdp:problem:no-active-consent",
  "title": "No active consent matches purpose TREAT for this patient",
  "status": 403,
  "detail": "Patient consent #c-91a7 expired 2025-12-31; renewal not on file.",
  "instance": "/Patient/abc-123"
}
```

## §7 Bulk export

The FHIR R5 Bulk Data Access (Flat FHIR) operations are supported
with privacy extensions:

- `GET /$export?_type=Patient,Observation&purpose=HRESCH&consentBundle=<id>`

The export is allowed only if the *consent bundle* listed names every
subject in the export's cohort. The server resolves the cohort against
the bundle and excludes any subject without an active consent for the
declared purpose. The exported NDJSON is signed and the manifest
references PHASE 1 §7 (de-identification job record) so that the
recipient can verify the dataset's residual-risk band.

## §8 Subject self-service

A data subject MAY query their own records:

- `GET /AuditEvent?agent-name=<self>` — returns events about the subject
- `GET /Consent?subject=<self>` — returns consents about the subject
- `POST /$rectification` — request correction of an erroneous record
- `POST /$erasure` — request erasure (subject to legal-hold rules)

Self-service endpoints authenticate the subject through the deployment's
identity provider. The server still emits AuditEvents for self-service
calls so that a subject's access pattern is itself observable to the
controller (this prevents impersonation from going unnoticed).

## §9 Conformance and discovery

The capabilities of this PHASE are advertised at the FHIR
`/metadata` endpoint, with WIA extensions describing the
purpose-of-use vocabulary, the consent profile, and the
break-glass policy. A privacy-aware client SHOULD read
`/metadata` once per session and cache the resulting capability
statement for the session lifetime.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Capability Statement (informative)

The FHIR `/metadata` endpoint advertises this PHASE's capabilities
through the standard CapabilityStatement resource with the following
WIA-specific extensions:

- `wia.purposeOfUse.valueSet` — URI of the closed PoU set (PHASE 1 §4)
- `wia.consent.profile` — URI of the Consent profile this PHASE binds
- `wia.breakGlass.policy` — URI of the deployment's break-glass policy
- `wia.audit.publicRoot` — URI of the daily-root transparency log if enabled
- `wia.jurisdiction` — declared jurisdiction code (PHASE 1 §1)

Clients SHOULD discover capabilities once per session; the cache
expiry SHOULD match the SMART access-token lifetime.

## Annex B — Common request patterns (informative)

The following patterns are typical clinical-application calls and
their expected privacy-gate behaviour:

- **Routine encounter open** — `GET /Patient/<id>?_revinclude=Encounter:patient`
  with `purpose:TREAT`; succeeds when the clinician is in the consent
  grantee list and the encounter is in the consent's period.
- **Refer-out** — `POST /Bundle/$transaction` with referral document;
  each Bundle entry is independently gated; the receiving organisation
  appears as a new grantee in a follow-on Consent created by the
  referral itself.
- **Research extract** — `GET /$export?_type=Observation&purpose=HRESCH`
  with consent bundle naming each subject; resource set returned
  intersected with consent set.
- **Subject access request** — `GET /AuditEvent?agent-name=<self>`;
  returns events about the authenticated subject only; emits its own
  AuditEvent recording the self-access.

## Annex C — Worked request/response example (informative)

The following exchange demonstrates a consent-gated read of an
Observation resource:

**Request:**

```
GET /Observation/obs-7e2-bp HTTP/1.1
Host: fhir.hospital-k.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
WIA-Purpose-Of-Use: TREAT
WIA-Justification: routine outpatient encounter, scheduled visit
Accept: application/fhir+json
```

**Response (success):**

```
HTTP/1.1 200 OK
Content-Type: application/fhir+json
WIA-Audit-Event-Id: urn:wia:mdp:audit:2026-04-27T09:31:14+09:00:7f2a
WIA-Consent-Receipt: urn:wia:mdp:consent:c-91a7

{ "resourceType": "Observation", "id": "obs-7e2-bp", ... }
```

**Response (no active consent):**

```
HTTP/1.1 403 Forbidden
Content-Type: application/problem+json
WIA-Audit-Event-Id: urn:wia:mdp:audit:2026-04-27T09:31:14+09:00:7f2b

{
  "type": "urn:wia:mdp:problem:no-active-consent",
  "title": "No active consent matches purpose TREAT for this patient",
  "status": 403,
  "detail": "Patient consent c-91a7 expired 2025-12-31; renewal not on file.",
  "instance": "/Observation/obs-7e2-bp"
}
```

Both the success and the failure emit AuditEvents into the chain.
The failure is auditable evidence that the consent gate worked, not
an unrecorded denial.

## Annex D — Negotiation of FHIR profile version

Clients SHOULD send `Accept: application/fhir+json; fhirVersion=5.0`.
Servers default to FHIR R5; prior FHIR versions are reachable only
via the `/R4` and `/STU3` legacy bases when the deployment policy
enables them. Cross-version translation is the responsibility of the
boundary, not of clients; clients see a single consistent profile
per session.
