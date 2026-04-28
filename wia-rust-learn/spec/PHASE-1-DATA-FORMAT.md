# WIA-RUST-LEARN — Phase 1: Data Format

> Rust-learn canonical Phase 1: curriculum-record + level-record + exercise + learner-progress + assessment envelopes per the Rust Book + Rust by Example + Rustlings.


## A.1 Curriculum-record envelope

The Phase 1 envelope groups WIA-RUST-LEARN curricula by delivery
class (instructor-led — synchronous classroom or live-online with
a credentialed instructor; self-paced — asynchronous with the per-
learner pace; cohort-based — asynchronous content with a cohort
+ discussion-channel envelope; hybrid — instructor-led foundation
+ self-paced extension). Each curriculum record carries: curriculum
identifier (UUID v7 per RFC 9562), curriculum-name + version per
SemVer per semver.org, target-audience envelope (beginner ·
intermediate · cross-trainer-from-other-language with the per-
language-of-origin reference — C/C++ programmers per ISO/IEC 9899
+ ISO/IEC 14882; Python programmers per Python Language Reference;
Java programmers per JLS; Go programmers per Go Specification;
JavaScript/TypeScript programmers per ECMAScript 2024), per-
curriculum prerequisite envelope (e.g., basic command-line
familiarity per POSIX.1-2024 IEEE Std 1003.1; basic git per
git-scm.com), per-curriculum target Rust edition (default 2024
per the Rust Edition Guide), per-curriculum estimated time-to-
completion envelope, and the per-curriculum audit envelope tied
to the delivery host.

## A.2 Level-record envelope

A level-record envelope MUST list: level identifier (Level 0
through Level 9 per the canonical 10-level WIA-RUST-LEARN catalogue),
per-level title + per-level learning-objective list, per-level
prerequisite-level chain envelope (Level 2 ⊂ Level 1 ⊂ Level 0;
Level 9 ⊂ all of Level 0-8), per-level scope (Level 0 — setup;
Level 1 — basics; Level 2 — ownership; Level 3 — structure;
Level 4 — advanced; Level 5 — concurrency; Level 6 — testing;
Level 7 — patterns; Level 8 — unsafe introduction; Level 9 —
capstone projects), per-level reference-material map (per-level
chapters of *The Rust Programming Language* + Rust by Example +
Rustlings exercise pack at `https://github.com/rust-lang/rustlings`),
per-level expected-duration envelope, and per-level mastery-
verification envelope per §A.5.

## A.3 Exercise-record envelope

Exercise-record envelopes catalogue the per-level practice surface:
per-exercise identifier (per-rustlings exercise name per the
upstream rustlings catalogue; per-RBE exercise name per Rust by
Example; per-curriculum-author custom-exercise name); per-exercise
class envelope (compile-fix — student fixes a non-compiling
snippet; output-match — student produces a program matching the
expected stdout; behaviour-test — student passes a `#[test]` suite;
property-test — student satisfies a property check per
proptest / quickcheck; benchmark — student meets a per-exercise
performance budget per Criterion.rs); per-exercise difficulty-tag
envelope (warmup · basic · standard · challenging · capstone);
per-exercise hint-tier envelope (per-exercise tiered-hint catalogue
per the per-curriculum learner-support policy); per-exercise canonical-
solution envelope (per-curriculum reference solution + per-curriculum
common-mistake catalogue).

## A.4 Learner-progress envelope

Learner-progress envelopes follow the xAPI 2.0 (Experience API per
ADL + IEEE 9274.1.1) + cmi5 conventions: per-learner pseudonymous
identifier per the per-LRS (Learning Record Store) envelope; per-
attempt timestamp per RFC 3339 ISO 8601; per-attempt outcome
envelope (PASSED / FAILED / IN-PROGRESS / ABANDONED); per-attempt
attempt-count envelope (per-exercise rolling-window attempt-count
+ per-exercise time-to-completion); per-attempt hint-consumption
envelope (per-attempt hint-tier reached); per-attempt error-class
envelope (per-attempt the dominant compiler-error category — borrow
checker, type mismatch, lifetime, trait bound, etc.); per-learner
mastery-state envelope (per-level NOT-STARTED / IN-PROGRESS /
COMPLETED / MASTERED) per the per-curriculum mastery-policy.

## A.5 Assessment-record envelope

Assessment-record envelopes describe per-level + per-track mastery
verification: per-assessment class envelope (per-level rustlings
completion — all per-level exercises pass; per-level capstone —
per-level capstone project rubric per the per-curriculum rubric;
per-track final — Level 9 capstone with peer + instructor review);
per-assessment rubric envelope (per-rubric criteria + per-criterion
performance levels + per-criterion weight per the per-track
assessment policy); per-assessment evidence envelope (per-evidence
artefact reference — git repository URL + per-evidence commit-hash);
per-assessment outcome envelope (per-outcome level-mastery flag +
per-outcome certification-eligibility flag); per-assessment audit
envelope (per-evaluator identifier + per-evaluation timestamp +
per-evaluator credential).

## A.6 Reference-material catalogue envelope

Reference-material envelopes pin the canonical learning materials
the curriculum cites: *The Rust Programming Language* book by
Klabnik + Nichols at `https://doc.rust-lang.org/book/` (per-edition
2024 / 2021 / 2018 mapping per the Rust Edition Guide); *Rust by
Example* at `https://doc.rust-lang.org/rust-by-example/` (per-
chapter cross-reference to *The Book* + Rustlings); *Rustlings*
exercises at `https://github.com/rust-lang/rustlings` (per-
exercise version-pin per the upstream tag); *The Rust Reference*
at `https://doc.rust-lang.org/reference/` (per-chapter reference
for advanced sections); *The Standard Library* at
`https://doc.rust-lang.org/std/` (per-module API reference); *The
Cargo Book* at `https://doc.rust-lang.org/cargo/` (per-section
build-system reference); per-curriculum extension catalogue (per-
curriculum supplemental references for cross-trainers per
Phase 1 §A.1).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wia-rust-learn/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wia-rust-learn-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wia-rust-learn-host:1.0.0` ships every wia-rust-learn envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wia-rust-learn.sh` ships sample envelope generators with no
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
ecosystem. Wia-rust-learn deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-wia-rust-learn-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

## Z.9 Disaster recovery and continuity-of-operations envelope

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-
exit envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity. The DR
envelope composes with WIA Secure Enclave for sealed-backup
envelopes and with WIA-AIR-SHIELD for runtime trust-list re-
hydration on the failover instance.

## Z.10 Supply-chain and software-bill-of-materials envelope

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per the operator's chosen specification: SPDX 2.3 / 3.0 per
ISO/IEC 5962 + Linux Foundation SPDX, or CycloneDX 1.6 per OWASP
Foundation. The SBOM enumerates every direct + transitive dependency
with the per-component name + version + licence + supplier + per-
component hash + per-component PURL (Package URL per package-url
spec) + per-component CPE (Common Platform Enumeration per NIST).
The host MUST publish per-release SBOM updates and MUST flag
breaking dependency-version migrations so downstream consumers
can plan ahead. Supply-chain attestation follows in-toto per
CNCF in-toto + SLSA (Supply-chain Levels for Software Artifacts)
per OpenSSF SLSA Framework — typically targeting SLSA Level 3 for
hosted production deployments.

弘益人間 — Benefit All Humanity.
