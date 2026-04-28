# WIA-micro-credential PHASE 2 — API Interface Specification

**Standard:** WIA-micro-credential
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for micro-credential operations: issuer registration
and key publication, recipient binding and wallet
exchange, credential-class catalogue, issuance, evidence
storage, endorsement, framework-mapping retrieval,
revocation publication, presentation acceptance, and
verification. The API is shaped so a wallet, an issuer
platform, and a verifier (employer / regulator) can
exchange credentials without bilateral agreements
beyond DID resolution and trust-list selection.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IETF RFC 6749, RFC 7636 (PKCE), RFC 8414 (OAuth Authorization Server Metadata)
- W3C VC Data Model 2.0; W3C Data Integrity 1.0; W3C VC JOSE 1.0
- W3C DID Core 1.0; DID Resolution; DID Web; DID Key
- 1EdTech Open Badges 3.0 — Service / API specifications
- 1EdTech CLR 2.0 — Comprehensive Learner Record API
- W3C Status List 2021
- OpenID for Verifiable Credentials Issuance (OID4VCI), OpenID for Verifiable Presentations (OID4VP)

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Issuance endpoints participate
in the OID4VCI flow; presentation endpoints participate
in OID4VP. Wallet-to-issuer exchange tolerates push and
pre-authorised models.

## §2 Issuer endpoints

```
GET    /v1/issuers/.well-known/openid-credential-issuer  metadata (OID4VCI)
GET    /v1/issuers/.well-known/jwks.json                public keys
GET    /v1/issuers/{issuerRef}                          issuer profile
POST   /v1/issuers/{issuerRef}/keys                      key rotation
GET    /v1/issuers/{issuerRef}/policy                   issuance policy
```

Issuer metadata includes the supported credential
formats (W3C VC, OB 3.0, mDoc), the supported proof
types, the status-list URL, and the OID4VCI grant
types accepted (`pre-authorized_code`, `authorization_
code`).

## §3 Credential-class endpoints

```
GET    /v1/credential-classes                           list catalogue
POST   /v1/credential-classes                           publish new class
GET    /v1/credential-classes/{ref}                     retrieve
PATCH  /v1/credential-classes/{ref}                     amend (admin)
GET    /v1/credential-classes/{ref}/framework-mappings  EQF/NQF/ESCO mappings
```

Catalogue listings expose ESCO skill / occupation
references, EQF level, ISCED / ISCED-F codes, and
notional-learning hours so an external skills-matching
system can match a job posting to a credential class.

## §4 Issuance endpoints (OID4VCI)

```
POST   /v1/issuance/offer                               create credential offer
POST   /v1/issuance/credential                          issuer credential endpoint
POST   /v1/issuance/batch-credential                    batch issuance
POST   /v1/issuance/deferred-credential                 deferred issuance retrieval
```

`offer` returns a credential offer object (OID4VCI)
with the supported flows; the wallet completes either
a pre-authorised flow with a transaction code or an
authorisation_code flow with PKCE. Issuance endpoints
require a fresh proof of possession (DPoP / cnf claim)
binding the credential to the holder's controller key.

## §5 Recipient / wallet endpoints

```
GET    /v1/recipients/{recipientRef}                    public profile
POST   /v1/recipients/{recipientRef}/credentials        wallet ingestion (push)
GET    /v1/recipients/{recipientRef}/credentials        list recipient credentials
DELETE /v1/recipients/{recipientRef}/credentials/{c}    wallet removal (logical)
```

Push delivery is supported for issuers that pre-
authenticate the recipient (e.g. a learning-platform
account with a verified DID).

## §6 Evidence endpoints

```
POST   /v1/evidence                                     register evidence record
GET    /v1/evidence/{ref}                               retrieve (gated)
POST   /v1/evidence/{ref}/upload                        chunked upload (Range)
GET    /v1/evidence/{ref}/audit                         evidence audit trail
```

Evidence retrieval is gated by the issuance's
disclosure scopes; verifiers receive only the evidence
the recipient explicitly disclosed in the presentation.

## §7 Endorsement endpoints

```
POST   /v1/endorsements                                 publish endorsement
GET    /v1/endorsements/{ref}                           retrieve
GET    /v1/endorsements?issuer=&class=                  list per issuer / class
```

## §8 Revocation and status-list endpoints

```
GET    /v1/status-lists/{listRef}                       W3C Status List 2021 list
POST   /v1/issuance/{issuanceRef}/revoke                revocation action
POST   /v1/issuance/{issuanceRef}/suspend               suspension action
POST   /v1/issuance/{issuanceRef}/reinstate             un-suspension
```

Status-list responses are statically cached at the
edge with cache-control headers per the W3C Status
List recommendation.

## §9 Presentation endpoints (OID4VP)

```
POST   /v1/presentations/request                        verifier creates request
GET    /v1/presentations/{ref}                          retrieve (gated)
POST   /v1/presentations/{ref}/submit                   wallet submits VP
POST   /v1/presentations/{ref}/verify                   verifier-side verify
```

Presentation requests follow OID4VP `presentation_
definition`; the wallet matches credentials against the
input descriptors and constructs a VP with selective
disclosure.

## §10 Verification endpoints

```
POST   /v1/verifications                                run verification
GET    /v1/verifications/{ref}                          retrieve outcome
GET    /v1/verifications?subject=&since=                list / filter
```

Verification consumes a presentation, resolves the
issuer DID, fetches the active status list, evaluates
the issuance's revocation bit, validates proof, and
records the outcome.

## §11 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:mc:problem:credential-revoked",
  "title":  "Credential is revoked",
  "status": 410,
  "detail": "Issuance i-2026-04-12-007 is revoked per status list bit 3142",
  "instance": "/v1/verifications"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `credential-revoked`         | 410  | issuance has been revoked                     |
| `credential-suspended`       | 423  | issuance temporarily suspended                |
| `proof-invalid`              | 401  | proof verification failed                     |
| `issuer-untrusted`           | 401  | issuer DID not on verifier's trust list       |
| `did-resolution-failed`      | 502  | could not resolve issuer DID                  |
| `status-list-unavailable`    | 503  | status-list endpoint unreachable              |
| `presentation-mismatch`      | 422  | VP does not match presentation_definition     |

## §12 Bulk export

```
GET  /v1/$export?_type=Issuance,Endorsement,Verification
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

NDJSON output, one record per line. Exports support
issuer-side compliance reporting and recipient data-
portability rights.

## §13 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed                            |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `X-Trace-Id`            | W3C Trace Context                             |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |

## §14 Versioning

Resource paths are version-prefixed (`/v1/...`).

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked verification (informative)

```http
POST /v1/verifications HTTP/1.1
Authorization: Bearer ...
Content-Type: application/json

{
  "presentation": "eyJhbGciOiJFZERTQSIs...",
  "policy": "wia-mc://policy/employer-2026"
}
```

Response 200:

```json
{
  "verificationRef": "v-2026-04-12-001",
  "outcome": "valid",
  "issuerRef": "did:web:issuer.example",
  "credentialClassRef": "https://issuer.example/classes/sql-101",
  "claims": {"name": "Introduction to SQL", "level": "EQF-4"}
}
```

## Annex C — Webhook surface

Implementations expose webhooks for `issuance-
created`, `revocation-published`, `endorsement-
added`, and `verification-completed` events. Payloads
sign with RFC 7515 JWS; receivers verify against
`/.well-known/wia-mc-keys.json`. Delivery is at-least-
once; receivers are expected to be idempotent on
`eventId`.

## Annex D — Conformance disclosure

Implementations declare the OID4VCI / OID4VP profiles
supported, the W3C VC formats accepted (jwt_vc_json,
ldp_vc, mso_mdoc), the status-list scheme, and the
trust-framework bindings (eIDAS / EBSI / KISA / etc).

## Annex E — Issuer key-rotation flow

```
draft-key → published-to-DID → active → retired-key
                                  │
                                  └→ compromised → emergency-rotation
```

Active keys rotate on a sponsor-policy clock (annual
default). Compromised keys move directly to
emergency-rotation; the issuer re-signs outstanding
issuances with the replacement key and emits a key-
rotation event on the audit chain. Verifiers honour
the rotation through DID-Document key-history walks.

## Annex F — Trust-list publication

Issuers, regulators, and accreditation bodies publish
trust lists at content-addressed URIs:

| Authority           | Trust-list URI pattern                          |
|---------------------|-------------------------------------------------|
| EBSI                | EBSI Issuer Registry / Accreditation Registry   |
| EU member state     | per-state CSCA trust list                       |
| Sponsor / employer  | sponsor-issued list (signed)                    |
| KR DTAB             | KISA-published list                             |

Trust-list updates emit audit events; verifiers refresh
on a clock or on event-driven webhook delivery.

## Annex G — Operational headers for wallet flows

| Header                      | Meaning                                      |
|-----------------------------|----------------------------------------------|
| `WIA-MC-Wallet-Implementation`| wallet implementation identifier            |
| `WIA-MC-Holder-Binding-Type`| `dpop`, `cnf`, `did-binding`                  |
| `WIA-MC-Disclosure-Profile` | `sd-jwt`, `bbs+`, `salted-hash`               |
| `WIA-MC-Trust-List`         | trust-list URI consumed for this verification |
