# WIA Rust Intermediate — Phase 4: Integration

**Standard**: WIA Rust Intermediate
**Phase**: 4 of 4 — Integration with Existing Systems
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA Rust Intermediate composes with neighbouring
WIA standards and external systems:

1. **Sister tiers**: WIA Rust Learn (beginner), WIA Rust Advanced.
2. **WIA family standards**: WIA-OMNI-API, WIA-ACCESSIBILITY, WIA-AIR-SHIELD,
   WIA-INTENT, WIA-SOCIAL.
3. **External systems**: existing Learning Management Systems (LMS),
   employer credential verifiers, public certificate registries.

The aim is that a learner who attends a WIA Rust Intermediate course at
one academy can carry their credential to any other system that honours
the standard, without retyping or re-authentication.

---

## 2. Tier Integration

### 2.1 From WIA Rust Learn

A learner who has completed WIA Rust Learn presents their learner_record
to a WIA Rust Intermediate academy as a prerequisite. The academy:

1. Verifies the WIA Rust Learn record's signature against the issuing
   academy's published key.
2. Confirms the record's `level_attained` ≥ `Core` (default policy;
   academies MAY require `Full`).
3. Accepts the learner without re-administering Learn-tier exercises.

### 2.2 To WIA Rust Advanced

When a learner attains `Full` level on the Intermediate tier, the academy
MAY auto-issue an "Advanced eligibility" attestation that the Advanced
tier accepts as prerequisite proof.

### 2.3 Cross-Tier Auditing

Each tier's records reference the prior tier via `prerequisites`. A
verifier can walk the chain from Advanced → Intermediate → Learn and
confirm continuity of the learner's identity DID across tiers.

---

## 3. WIA Family Integration

### 3.1 WIA-OMNI-API

Learner credentials (transcripts, identity proofs, language certificates)
are stored in WIA-OMNI-API. Academies fetch them by DID rather than
holding raw documents.

```
GET https://omni.example/credential/did:wia:learner:01HZA…/transcript
Authorization: WIA-Sig …
→ 200 { "vendor": "credly", "url": "https://…", "expires_at": "…" }
```

### 3.2 WIA-ACCESSIBILITY

Learner accommodations profiles drive assessment format choices. The
academy MUST refuse to enrol a learner whose required accommodations
(extended time, screen-reader output, alternative input) cannot be met.

### 3.3 WIA-AIR-SHIELD

Transport hardening (TLS configuration, peer reputation). Academies MAY
refuse handshakes from peers whose AIR-SHIELD score is below an
academy-set threshold.

### 3.4 WIA-INTENT

A learner's "I want to attain Full Intermediate by autumn" intent is
lowered to a sequence of:

1. Choose academy (`GET /wri/manifest?level=Full&language=ko`).
2. Enrol (`PUT /wri/learner/{id}` with academy_id set).
3. Submit assessments per module (`POST /wri/assessment` × N).
4. Auto-issue Advanced eligibility on `level_attained=Full`.

### 3.5 WIA-SOCIAL

Academies MAY publish course completion announcements to learners' WIA-SOCIAL
feeds with audience `friends` by default. The learner's social federation
controls whether the post fans out further.

---

## 4. LMS Bridges

### 4.1 Bridge Architecture

A bridge translates WIA Rust Intermediate ↔ LMS native objects.

```
LMS (Moodle / Canvas / Blackboard)
        │
        ▼
   ┌────────────────────────┐
   │  WIA RI LMS Bridge     │
   └────────────────────────┘
        │
        ▼
WIA RI Academy API (Phase 2)
```

Bridges hold no learner signing keys; they translate vendor calls into
academy API calls under the academy's host signature.

### 4.2 Per-LMS Mapping

| LMS object | WIA RI object |
|------------|---------------|
| Course | curriculum_manifest |
| Module | module_id (Phase 1 §3.3) |
| Assignment | exercise |
| Submission | assessment_result (status `pending`) |
| Grade | assessment_result (status `graded`, band) |
| Student | learner_id (DID derived from LMS user id) |

Bridges MUST emit `delivery_receipt` for every LMS event consumed; missed
events MUST be retried with exponential backoff (initial 1 s, max 60 s).

---

## 5. Employer Credential Verification

An employer wishing to verify a candidate's WIA Rust Intermediate
credential follows:

1. Receive the candidate's DID from the candidate.
2. Resolve the DID to discover the issuing academy.
3. Call `GET /wri/learner/{did}` on the academy with the
   `Audience: employer` header.
4. Parse the redacted record: `level_attained`, `modules_passed[].band`.
5. Optionally verify the academy's host signature against a published
   trust anchor (see §7).

The academy MUST log the read for the learner's audit trail.

---

## 6. Public Certificate Registry Integration

Academies that publish credentials to public registries (Open Badges,
European Digital Credentials Infrastructure) MUST:

1. Sign the original WIA RI learner_record envelope.
2. Re-encode the data into the registry's native format.
3. Include the canonical WIA RI URL in the registry record so a verifier
   can always trace back to the source.

Re-encoding MUST NOT change semantic content. If a registry's schema
cannot represent a WIA RI field (e.g. `accommodations_profile`),
academies omit the field rather than approximate it.

---

## 7. Trust Anchors

Each academy publishes a trust anchor: the host signing key plus a
self-signed attestation of the academy's role and jurisdiction.
Verifiers fetch the trust anchor from
`https://academy.example/.well-known/wia-trust-anchor` and pin it for
their cache TTL (default 24 hours).

Trust anchor rotation follows the same chain semantics as identity
bundles in WIA-SOCIAL Phase 3 §7.

---

## 8. Migration Paths

### 8.1 From a Legacy LMS-only Programme

An academy currently running courses in an LMS without WIA RI migrates
by:

1. Deploying a WIA RI service (reference container available).
2. Importing existing learners as `medical.legacy=false` learner_records
   with a placeholder `modules_passed` rebuilt from LMS gradebook.
3. Running a 14-day shadow period where the LMS remains source of truth
   and the WIA RI service mirrors. After zero discrepancy days the WIA
   RI service becomes the system of record.
4. Switching the LMS bridge to read-from-WIA-RI mode.

### 8.2 From Another Curriculum Standard

Some academies use bespoke or vendor-specific curriculum standards. A
mapping document at `spec/profiles/<vendor>.md` describes the mapping for
each supported source. Vendors not in the registry can submit a profile
proposal via the WIA Standards governance process.

---

## 9. Observability

Bridges and adapters SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_ri_manifests_published_total` | counter | Lifetime publishes |
| `wia_ri_assessments_total{band}` | counter | Per-band assessment outcomes |
| `wia_ri_handshakes_total{role,outcome}` | counter | Federation handshakes |
| `wia_ri_learner_moves_total{outcome}` | counter | Learner record moves |

Labels MUST NOT include learner identifiers.

---

## 10. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | WIA-OMNI-API for credentials |
| **Core**    | Plus WIA-ACCESSIBILITY enforcement, WIA-AIR-SHIELD scoring |
| **Full**    | Plus WIA-INTENT lowering, WIA-SOCIAL outbound, LMS bridge for at least one platform |

Academies publish their level in `bridge_profile` of the discovery
document.

---

## 11. Worked Example — End-to-End

```
Learner   : did:wia:learner:01HZA…
Academy   : did:wia:academy:bonghwa
LMS       : Moodle 4.x via WIA RI bridge
Employer  : did:wia:employer:nogada
```

1. Learner enrols via Moodle → bridge creates learner_record.
2. Learner completes module 3 exercise; grader submits via Moodle.
3. Bridge publishes assessment_result to academy's `/wri/assessment`.
4. Academy updates learner_record; level_attained advances to `Core`.
5. Academy auto-publishes Open Badge to learner's profile (with consent).
6. Employer queries DID, fetches redacted record, verifies signature.
7. Optional: learner moves to silla academy via Phase 3 §4 chain.

---

## 12. Security Considerations

* LMS bridges hold OAuth tokens for the LMS vendor; operators MUST use
  WIA-OMNI-API for credential storage and HSM-backed refresh tokens.
* Trust anchors are high-value; loss requires re-establishing all
  federation receipts. Operators MUST keep the host signing key in an
  HSM or equivalent and MUST publish rotation events promptly.
* Public registry pushes are one-way; misissue is mitigated by appending
  a revocation envelope rather than attempting to recall the public
  registry entry (some registries do not support deletion).

---

## 13. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 6749 — OAuth 2.0
* W3C Verifiable Credentials Data Model
* Open Badges Specification 2.1
* European Digital Credentials Infrastructure
* WIA-OMNI-API standard
* WIA-ACCESSIBILITY standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-SOCIAL standard

---

## Appendix A — Bridge Configuration File

A reference Moodle bridge configuration uses TOML:

```toml
[bridge]
lms = "moodle"
lms_version = "4.x"
academy_id = "did:wia:academy:bonghwa"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"

[delivery]
poll_interval_seconds = 60
backoff = "exponential"
initial_delay_ms = 1000
max_delay_ms = 60000

[mapping]
course_to_manifest = "by-name"
module_to_module_id = { "Ownership" = 1, "Lifetimes" = 2, "Traits" = 3, "Generics" = 4, "Errors" = 5, "Concurrency" = 6, "Async" = 7, "Crates" = 8 }

[telemetry]
prometheus_port = 9091
```

Operators MAY embed vendor-specific sections; conformant bridges MUST
ignore unknown sections rather than refuse to start.

## Appendix B — Worked Cross-Tier Walk

A learner attains Full on Intermediate at academy α and proceeds to
Advanced at academy γ:

1. Learner publishes a delegation envelope to γ for `read_record`.
2. γ fetches the Intermediate record from α: signature verified, level
   confirmed `Full`.
3. γ fetches the Learn record from the academy that issued it: signature
   verified, level confirmed `Core` or higher.
4. γ enrols the learner with `prerequisites_satisfied=true`, no further
   re-administration of Intermediate exercises.
5. Each tier's record is preserved at its issuing academy; γ holds only
   pointers in the new Advanced record.

Verifiers querying γ's Advanced record can walk the chain through
Intermediate to Learn to confirm continuity of the same DID.

## Appendix C — Open Badges Mapping

Each Intermediate band maps to an Open Badges 2.1 assertion:

| WIA RI band | Open Badges criterion |
|-------------|------------------------|
| Pass | Mastery of stated learning outcomes |
| Merit | Pass plus exemplary code review |
| Distinction | Merit plus extension implementation |

The assertion's `evidence` field links to the assessment_result URL on
the issuing academy. Revocations on the WIA RI side are propagated to
Open Badges via the `revocationList` mechanism.

## Appendix D — Adapter State Machine: Moodle Bridge

```
  IDLE ─── academy publishes manifest ─►  POLLING
  POLLING ── new submission detected ───►  TRANSLATING
  TRANSLATING ── translation OK ────────►  POSTING
  POSTING ── academy 201 received ──────►  POLLING
  POSTING ── academy 4xx ───────────────►  ERROR
  ERROR ── operator clears ─────────────►  POLLING
  POSTING ── academy 5xx ───────────────►  RETRYING
  RETRYING ── max attempts exceeded ────►  ERROR
```

Bridges MUST log every state transition in append-only storage. Auditors
and post-incident reviewers depend on the trace for dispute resolution
when a learner's submission is missing from the academy record.

## Appendix E — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| WIA-OMNI-API credential fetch | ✓ | ✓ | ✓ |
| WIA-ACCESSIBILITY enforcement | — | ✓ | ✓ |
| WIA-AIR-SHIELD scoring | — | ✓ | ✓ |
| WIA-INTENT lowering | — | — | ✓ |
| WIA-SOCIAL outbound | — | — | ✓ |
| LMS bridge for ≥1 platform | — | — | ✓ |
| Open Badges export | optional | optional | ✓ |
| European DCI export | optional | optional | optional |
| Trust anchor publication | ✓ | ✓ | ✓ |

An academy publishing `bridge_profile=Full` MUST pass every line in the
Full column and SHOULD pass at least one optional line.

弘益人間 — Benefit All Humanity.
