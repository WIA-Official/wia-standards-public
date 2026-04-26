# WIA Rust Intermediate — Phase 1: Data Format

**Standard**: WIA Rust Intermediate
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — Benefit All Humanity

---

## 1. Scope

Phase 1 fixes the JSON shapes that every WIA Rust Intermediate participant
MUST be able to read and emit:

| Family | Purpose |
|--------|---------|
| **Curriculum Manifest** | Which modules a course covers, exercise list, assessment form |
| **Exercise** | A unit of practice with a difficulty band and rubric pointer |
| **Learner Record** | A learner's passed modules and attained level |
| **Assessment Result** | A graded submission for a single exercise |
| **Federation Receipt** | Trust evidence between two academies |

Out of scope: HTTP surface (Phase 2), federation protocol (Phase 3),
WIA-family integration (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Timestamps RFC 3339 in UTC, `Z` suffix.
* Identifiers URI-shaped per IETF RFC 3986. Learner identifiers SHOULD use
  `did:wia:learner:…` to avoid carrying personally identifying numbers.
* Binary fields base64url without padding per IETF RFC 4648 §5.
* Numbers fit in signed 64-bit; floats use IEEE 754 double precision.

### 2.1 Versioning

```json
"wia_rust_intermediate_version": "1.0.0"
```

A receiver MUST refuse a major version it does not implement.

---

## 3. Curriculum Manifest

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "curriculum_manifest",
  "manifest_id": "cm_01HZA…",
  "author": "did:wia:academy:bonghwa",
  "issued_at": "2026-04-01T00:00:00Z",
  "module_ids": [1, 2, 3, 4, 5, 6],
  "language": "ko",
  "delivery": "hybrid",
  "duration_weeks": 12,
  "prerequisites": ["WIA Rust Learn"],
  "exercises": [
    { "exercise_id": "ex-1-01", "module_id": 1, "topic": "move semantics", "difficulty": "intro", "rubric_id": "rb-1-intro" },
    { "exercise_id": "ex-1-02", "module_id": 1, "topic": "borrow tree",    "difficulty": "applied", "rubric_id": "rb-1-applied" }
  ],
  "assessment": {
    "form": "submission",
    "rubric_set_id": "rs-bonghwa-2026q2",
    "passing_band": "Pass"
  },
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

### 3.1 Required Fields

`wia_rust_intermediate_version`, `type`, `manifest_id`, `author`,
`issued_at`, `module_ids`, `assessment`, `signature`.

### 3.2 Optional Fields

`language` (BCP 47), `delivery` (`online` | `in_person` | `hybrid`),
`duration_weeks`, `prerequisites` (array of standard names), `exercises`.

### 3.3 Module IDs

Module ids are integers 1..8 mapping to the module map in the README.
Manifests SHOULD list modules in ascending order. Implementations MUST
treat unknown integers as `unrecognised` rather than fail.

---

## 4. Exercise

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "exercise",
  "exercise_id": "ex-3-04",
  "module_id": 3,
  "topic": "Trait objects vs static dispatch",
  "difficulty": "applied",
  "rubric_id": "rb-3-applied",
  "starter_code_url": "https://academy.example/exercises/ex-3-04/starter.tar.gz",
  "expected_outcome": "Compiles, all unit tests pass, no clippy warnings on lint-default config",
  "estimated_minutes": 90
}
```

### 4.1 Difficulty Bands

`intro` (≤30 min), `applied` (30–120 min), `extension` (>120 min).
Implementations use these bands to time-budget course delivery; they have
no normative effect on grading.

### 4.2 Starter Code

`starter_code_url` MUST resolve to a tar archive whose top-level directory
is a valid Cargo project. The archive MUST NOT contain `target/` or
`Cargo.lock` (lockfile is generated at first build).

---

## 5. Learner Record

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "learner_record",
  "learner_id": "did:wia:learner:01HZA…",
  "issued_at": "2026-04-15T00:00:00Z",
  "academy_id": "did:wia:academy:bonghwa",
  "modules_passed": [
    { "module_id": 1, "passed_at": "2026-02-15", "band": "Merit" },
    { "module_id": 2, "passed_at": "2026-02-28", "band": "Pass" },
    { "module_id": 3, "passed_at": "2026-03-12", "band": "Distinction" }
  ],
  "level_attained": null,
  "accommodations_profile": "did:wia:a11y:learner:01HZA…",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

### 5.1 Level Attained

Set to `Minimal`, `Core`, `Full`, or `null` if no level threshold has been
crossed. Implementations MUST recompute `level_attained` whenever a
`modules_passed` entry is added.

### 5.2 Accommodations Profile

`accommodations_profile` references a WIA-ACCESSIBILITY record describing
the learner's preferred assessment format (extended time, screen-reader
output, alternative input, etc.). Academies MUST honour the linked profile
or refuse to enrol the learner.

---

## 6. Assessment Result

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "assessment_result",
  "result_id": "res_01HZA…",
  "exercise_id": "ex-3-04",
  "learner_id": "did:wia:learner:01HZA…",
  "submitted_at": "2026-03-08T11:00:00Z",
  "graded_at": "2026-03-09T15:30:00Z",
  "grader_id": "did:wia:grader:09…",
  "band": "Distinction",
  "feedback": "Static dispatch chosen with sound justification; benchmark included.",
  "artefacts": [
    { "media_type": "application/x-tar", "url": "https://submissions.example/ex-3-04/01HZA.tar.gz" }
  ],
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

Bands: `Fail`, `Pass`, `Merit`, `Distinction`. Implementations MUST treat
`Fail` as a non-credit outcome that does not advance `modules_passed`.

---

## 7. Federation Receipt

When academy A accepts academy B's manifest, A emits a receipt that B
stores as proof of acceptance. The shape mirrors WIA-SOCIAL Phase 1 §5,
substituting `accepted_manifest_id` for `accepted_subject`.

---

## 8. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/wia-rust-intermediate/schemas/`.

---

## 9. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields.
3. Treat unknown optional fields as non-fatal.
4. Recompute `level_attained` on every `modules_passed` change.

---

## 10. References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 4648 — base64
* IETF RFC 8032 — EdDSA / Ed25519
* W3C DID 1.0
* JSON Schema Draft 2020-12

---

## Appendix A — Worked Manifest (Full level, Korean academy)

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "curriculum_manifest",
  "manifest_id": "cm_01HZB…",
  "author": "did:wia:academy:bonghwa",
  "issued_at": "2026-04-01T00:00:00Z",
  "module_ids": [1,2,3,4,5,6,7,8],
  "language": "ko",
  "delivery": "hybrid",
  "duration_weeks": 18,
  "prerequisites": ["WIA Rust Learn"],
  "exercises": [
    { "exercise_id": "ex-1-01", "module_id": 1, "topic":"move semantics",  "difficulty":"intro",     "rubric_id":"rb-1-intro" },
    { "exercise_id": "ex-1-02", "module_id": 1, "topic":"borrow tree",     "difficulty":"applied",   "rubric_id":"rb-1-applied" },
    { "exercise_id": "ex-2-01", "module_id": 2, "topic":"lifetime elision","difficulty":"intro",     "rubric_id":"rb-2-intro" },
    { "exercise_id": "ex-2-02", "module_id": 2, "topic":"HRTB basics",     "difficulty":"extension", "rubric_id":"rb-2-ext" },
    { "exercise_id": "ex-3-01", "module_id": 3, "topic":"trait objects",   "difficulty":"applied",   "rubric_id":"rb-3-applied" },
    { "exercise_id": "ex-4-01", "module_id": 4, "topic":"monomorphisation","difficulty":"applied",   "rubric_id":"rb-4-applied" },
    { "exercise_id": "ex-5-01", "module_id": 5, "topic":"thiserror compose","difficulty":"applied",  "rubric_id":"rb-5-applied" },
    { "exercise_id": "ex-6-01", "module_id": 6, "topic":"Send/Sync proof", "difficulty":"extension", "rubric_id":"rb-6-ext" },
    { "exercise_id": "ex-7-01", "module_id": 7, "topic":"Future poll loop","difficulty":"applied",   "rubric_id":"rb-7-applied" },
    { "exercise_id": "ex-8-01", "module_id": 8, "topic":"Cargo workspaces","difficulty":"intro",     "rubric_id":"rb-8-intro" }
  ],
  "assessment": {
    "form": "submission_with_oral",
    "rubric_set_id": "rs-bonghwa-2026q2",
    "passing_band": "Pass"
  },
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

## Appendix B — Worked Learner Record (Full level achieved)

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "learner_record",
  "learner_id": "did:wia:learner:01HZA…",
  "issued_at": "2026-09-15T00:00:00Z",
  "academy_id": "did:wia:academy:bonghwa",
  "modules_passed": [
    { "module_id": 1, "passed_at": "2026-02-15", "band": "Merit",       "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 2, "passed_at": "2026-02-28", "band": "Pass",        "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 3, "passed_at": "2026-03-12", "band": "Distinction", "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 4, "passed_at": "2026-03-28", "band": "Merit",       "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 5, "passed_at": "2026-05-10", "band": "Pass",        "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 6, "passed_at": "2026-06-21", "band": "Merit",       "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 7, "passed_at": "2026-07-30", "band": "Distinction", "academy_id": "did:wia:academy:bonghwa" },
    { "module_id": 8, "passed_at": "2026-09-12", "band": "Pass",        "academy_id": "did:wia:academy:bonghwa" }
  ],
  "level_attained": "Full",
  "accommodations_profile": "did:wia:a11y:learner:01HZA…",
  "history": [
    { "event": "issued",   "at": "2026-02-01T00:00:00Z" },
    { "event": "level_up", "at": "2026-05-10T16:30:00Z", "to": "Minimal" },
    { "event": "level_up", "at": "2026-06-21T17:00:00Z", "to": "Core" },
    { "event": "level_up", "at": "2026-09-12T11:45:00Z", "to": "Full" }
  ],
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

## Appendix C — Canonicalisation for Signatures

When a record's signature must be re-verified, implementations MUST:

1. Sort object keys lexicographically by Unicode code point.
2. Use compact JSON form (no insignificant whitespace).
3. Render numbers using shortest round-trip form for floats; integers
   without leading zeros.
4. Encode arrays preserving authored order.
5. UTF-8 encode the result with no BOM.
6. Hash with SHA-256 prefixed by the literal ASCII string
   `WIA-RUST-INTERMEDIATE/1.0\n` for domain separation.

## Appendix D — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `assessment.form` | `submission`, `submission_with_oral`, `live_review`, `live_pair_program` |
| `assessment.passing_band` | `Pass`, `Merit`, `Distinction` |
| `delivery` | `online`, `in_person`, `hybrid` |
| `difficulty` | `intro`, `applied`, `extension` |
| `band` | `Fail`, `Pass`, `Merit`, `Distinction` |
| `level_attained` | `null`, `Minimal`, `Core`, `Full` |

Future minor versions add tokens but never remove them.

## Appendix E — Worked Assessment Result (corrections chain)

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "assessment_result",
  "result_id": "res_01HZB…",
  "exercise_id": "ex-3-04",
  "learner_id": "did:wia:learner:01HZA…",
  "submitted_at": "2026-03-08T11:00:00Z",
  "graded_at": "2026-03-09T15:30:00Z",
  "grader_id": "did:wia:grader:09…",
  "band": "Merit",
  "feedback": "Good static dispatch; benchmark missing.",
  "artefacts": [
    { "media_type": "application/x-tar", "url": "https://submissions.example/ex-3-04/01HZA.tar.gz" }
  ],
  "supersedes": null,
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

A correction follows after the learner appeals; the original is preserved
and a fresh envelope references it via `supersedes`:

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "assessment_result",
  "result_id": "res_01HZC…",
  "exercise_id": "ex-3-04",
  "learner_id": "did:wia:learner:01HZA…",
  "submitted_at": "2026-03-15T11:00:00Z",
  "graded_at": "2026-03-16T10:00:00Z",
  "grader_id": "did:wia:grader:21…",
  "band": "Distinction",
  "feedback": "Reviewed appeal: benchmark in benches/dispatch.rs is sufficient. Upgrade to Distinction.",
  "supersedes": "res_01HZB…",
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

Implementations MUST treat the learner's effective band as the most
recent envelope by `graded_at` for that `(learner_id, exercise_id)` pair.

弘益人間 — Benefit All Humanity.
