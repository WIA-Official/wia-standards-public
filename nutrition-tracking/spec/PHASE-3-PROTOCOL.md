# WIA-nutrition-tracking PHASE 3 — Protocol Specification

**Standard:** WIA-nutrition-tracking
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
subjects, dietitians, clinicians, and app developers; the
food-database roster snapshotting protocol; allergen-
warning escalation; image-recognition feedback loop;
audit-chain construction; cryptographic signing; time
discipline; failure modes.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- USDA FoodData Central — REST/JSON conventions
- HL7 FHIR R5 — RESTful security conventions
- ISO/IEC 27001:2022 — information-security management

---

## §1 Authentication

Subjects, dietitians, clinicians, and developers
authenticate via JWS-signed JWTs issued by the deployment's
identity authority. Token claims:

| Claim         | Source                                                  |
|---------------|---------------------------------------------------------|
| `iss`         | identity-provider URL                                   |
| `aud`         | the boundary URL                                        |
| `sub`         | subject / dietitian / clinician / developer URN         |
| `iat` / `exp` | per RFC 7519                                            |
| `wia.role`    | `subject`, `dietitian`, `clinician`, `app-developer`, `auditor` |
| `wia.scope`   | operation-class scopes                                  |
| `wia.subjectRef` | for subject-bound tokens, the pseudonym               |
| `wia.consent` | for clinical tokens, active consent UID                 |
| `cnf`         | mTLS certificate-thumbprint binding (clinical tokens)   |

Subject self-service tokens are short-lived (15 minutes).
Dietitian write-tokens are short-lived (5 minutes).
App-developer tokens are scoped strictly to the deployment's
metadata operations and never carry subject-data scope.

## §2 Food-database roster snapshotting

The deployment's food-database roster is a versioned
snapshot:

- `rosterId` — URN
- `sources[]` — per-source snapshot version (USDA FDC
  publish timestamp, KR-RDA release version, OEFF date,
  etc.)
- `effectiveFrom` — RFC 3339
- `signature` — JWS by the deployment's roster-curator

Each intake event records the roster ID active at the time
of computation. Recomputation under a newer roster
references the original `rosterId` so the audit chain
preserves how the totals were computed.

Roster updates follow this discipline:

1. New roster version published with effective-from
   timestamp
2. Boundary continues serving the prior roster for the
   declared overlap window (typically 24h) so in-flight
   apps don't see split-roster inconsistency
3. After overlap, prior roster is archived but remains
   queryable for replay

## §3 Allergen-warning escalation

For active allergen records, the boundary's screening is
real-time at intake-publish time:

| Severity        | Action                                                            |
|-----------------|-------------------------------------------------------------------|
| `mild`          | warning surfaced; intake accepted                                 |
| `moderate`      | warning surfaced; intake accepted; 24h dietitian review queued    |
| `severe`        | intake refused; user override accepted with audit-chained ack     |
| `anaphylactic`  | intake refused; override requires explicit "I understand" + audit |

Persistent override patterns (e.g., the same severe-
allergen ingredient overridden weekly) escalate to
clinical-team review per the deployment's policy.

## §4 Image-recognition feedback loop

For deployments using image recognition:

1. Image submitted; recognition model returns candidates
   with confidence
2. User selects the correct match (or rejects all)
3. Selection is logged as feedback, tagged with consent
   for model-improvement use
4. Aggregated feedback feeds the model retraining pipeline
   per the deployment's consent policy

Per-image model-version IDs are recorded so an upgrade in
the model is traceable to the version that recognised any
given image.

## §5 Audit chain

Every intake-publish, prescription-state-change,
allergen-record-mutation, image-verification, and
consent-mutation emits an AuditEvent:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

`kind` enum:
- `intake-published` / `intake-corrected` / `intake-deleted`
- `hydration-published`
- `supplement-published`
- `prescription-active` / `prescription-superseded`
- `pattern-summary-generated`
- `allergen-record-published` / `allergen-record-mutated`
- `allergen-warning-fired` / `allergen-override-acknowledged`
- `image-evidence-published` / `image-verification-completed`
- `consent-mutated`
- `roster-snapshot-applied`

Anchored deployments mirror the audit chain to a regulator-
trusted witness on a declared cadence; clinical deployments
typically mirror to the EHR's audit surface.

## §6 Drug-nutrient interaction protocol

When the boundary detects a drug-nutrient interaction:

1. Cross-reference the subject's WIA-medication-adherence
   active prescriptions
2. Retrieve interaction profiles per drug from the
   deployment's interaction reference
3. For each active prescription, scan recent intake events
   for interacting nutrients (e.g., grapefruit juice with
   simvastatin, vitamin K with warfarin)
4. Surface high-significance interactions to the subject's
   care team via the medication-adherence webhook
5. Record the interaction-detection event in the audit
   chain

Interaction signal-to-noise is tuned per the deployment's
clinical workflow; a deployment may suppress repeat
interactions for the same drug-nutrient pair within a
window.

## §7 Time discipline

Boundary clock: NTPv4 stratum-2. Mobile-app clocks are
unverified; the boundary records the mobile-claimed
timestamp and the boundary-receipt timestamp. Discrepancies
above the deployment's tolerance flag `time-skew`.

For clinical deployments, dietitian terminals MUST be
NTPv4-disciplined; intake records logged from non-
disciplined terminals are flagged for review.

## §8 Pseudonym lifecycle

Subjects' pseudonymous identifiers follow:

- Issued at enrolment with a corresponding consent
- Stable across sessions within the deployment
- Bound to PII via a separate access-controlled service
- Re-identification (clinical-care necessity only)
  requires documented justification and is audit-chained

For consumer-facing apps with no clinical context, the
pseudonym is the user's account identifier; the deployment's
identity service holds the binding to email/phone/account.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 24 hours. Bulk endpoints accept per-line
idempotency keys for fine-grained replay protection during
migration.

## §10 Cryptographic-suite registry

| Concern              | Default                            | Notes                                |
|----------------------|------------------------------------|--------------------------------------|
| Token signing        | ES256                              | mTLS-bound for clinical              |
| TLS                  | 1.3 (RFC 8446)                     | hybrid groups via WIA-pq-crypto      |
| Image hash           | SHA-256                            |                                      |
| Audit-chain hash     | SHA-256                            |                                      |
| JWS body signature   | PS256 or ES256                     |                                      |

Post-quantum migration follows WIA-pq-crypto PHASE 3 phase
declarations. Hybrid TLS groups (X25519 + ML-KEM-768) are
deployable for clinical traffic where the deployment's
WIA-pq-crypto phase has reached B (hybrid).

## §11 Failure modes

| Failure                                  | Behaviour                                          |
|------------------------------------------|----------------------------------------------------|
| Identity-provider JWKS unreachable        | Cached keys honoured until cache expiry           |
| Food-database roster source unreachable   | Continue serving last-good roster                  |
| Image-recognition model timeout           | Image stored; recognition retried; UI fallback to text entry |
| Allergen-screening database unreachable   | Refuse intake until screening service recovers     |
| Audit-chain write failure                 | Operation rejected (consistency requirement)       |
| Drug-interaction reference unreachable    | Surface "interaction-screening-unavailable" warning |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked authentication sequence (informative)

1. Subject opens the app; app calls IdP `/authorize`
2. IdP authenticates the subject (password + TOTP / FIDO2)
3. IdP issues an access token with `wia.role=subject`
4. App calls boundary with the token
5. Boundary verifies signature, expiry, scope
6. Authorised operations proceed

For dietitian access:

1. Dietitian authenticates with FIDO2 + license-credential
2. IdP verifies dietitian's active license per the
   identity-management standard
3. Token issued with `wia.role=dietitian` and a list of
   assigned-subject pseudonyms in the scope
4. Boundary enforces per-call subject-list membership

## Annex B — Roster snapshot manifest (informative)

```json
{
  "rosterId": "urn:wia:nutr:roster:app-x:r-2026-04-28",
  "sources": [
    {"source": "usda-fdc", "version": "2024-04-01", "recordCount": 461283},
    {"source": "kr-rda", "version": "2026-01", "recordCount": 19234},
    {"source": "oeff", "version": "2026-04-15", "recordCount": 2891034}
  ],
  "effectiveFrom": "2026-04-28T00:00:00+09:00",
  "overlapUntil": "2026-04-29T00:00:00+09:00",
  "signature": "<jws-detached>"
}
```

## Annex C — Allergen-screening worked example (informative)

A subject has an `anaphylactic` peanut allergen record.
At intake POST:

1. Item references foodRef "Snickers bar"
2. Boundary retrieves the item's ingredient list
3. Ingredients include "peanuts"
4. Match against the subject's active allergen record
5. Severity = `anaphylactic` → 422 with
   `urn:wia:nutr:problem:allergen-conflict`
6. User confirms override with explicit acknowledgement
7. Override accepted; logged with audit-chain entry
   `allergen-override-acknowledged`

The clinical team's dashboard surfaces all overrides for
review, weighted by severity.

## Annex D — Conformance levels (informative)

| Level     | Scope                                                                  |
|-----------|------------------------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                                  |
| Verified  | annual third-party audit + roster signature verification              |
| Anchored  | continuous evidence package + regulator-witnessed audit chain          |

## Annex E — Cross-domain handshake protocol (informative)

For drug-nutrient interactions:

1. Subject grants consent to share medication list with
   nutrition-tracking
2. Nutrition boundary subscribes to medication-adherence
   webhook for the subject
3. New prescription event triggers interaction screening
   against recent intake events
4. Detected interactions flow back to medication-adherence
   for clinical review

The handshake preserves both standards' consent boundaries:
neither standard sees data outside its scope; only the
interaction-detection event itself crosses.

## Annex F — Pseudonym binding hash (informative)

For PII-binding service:

```
binding_record = {
  "pseudonym": "p-app-x-PR-014",
  "piiHash": SHA-256(canonicalised PII bundle),
  "boundAt": "2026-04-28T08:00:00+09:00",
  "consentRef": "urn:wia:nutr:consent:..."
}
```

The PII bundle itself is held in a separate access-
controlled vault; the binding service holds only the hash
plus consent-derived authorisation rules.
