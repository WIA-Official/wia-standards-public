# WIA-RUST-LEARN — Phase 3: Protocol

> Rust-learn canonical Phase 3: protocols (per-level delivery + grading + ownership-borrow-check teaching + concurrency + testing + capstone) per Rust Reference + The Book.


## A.1 Per-level delivery protocol

Per-level delivery protocols cover: per-level lecture envelope (per-
level slide deck + per-level live-coding session + per-level Manim
animation per the per-curriculum animation catalogue at
`manim/episodes/level_*`); per-level reading envelope (per-level
mapping to *The Rust Programming Language* chapters per Klabnik +
Nichols at `https://doc.rust-lang.org/book/` + per-level mapping to
Rust by Example); per-level practice envelope (per-level rustlings
exercise pack + per-level Rust by Example exercises); per-level
assessment envelope (per-level rustlings completion + per-level
capstone exercise per the per-curriculum rubric); per-level
checkpoint envelope (per-checkpoint review session with the
instructor or with the cohort + per-checkpoint mastery-verification
trigger per Phase 1 §A.5).

## A.2 Grading + feedback protocol

Grading + feedback protocols cover: per-attempt automated-grader
envelope (per-attempt rustc compile + cargo test + cargo clippy +
cargo fmt --check pipeline; per-attempt diff-tracking against
canonical solution per the per-curriculum hint envelope); per-
attempt human-feedback envelope (per-attempt instructor or peer
review per the per-curriculum review-load policy; per-attempt
review-turnaround SLA per the per-curriculum response-time policy);
per-attempt feedback-routing envelope (per-attempt routing to the
appropriate reviewer based on per-reviewer per-level expertise);
per-attempt evolution envelope (per-attempt difficulty-adjustment
recommendation per the per-curriculum difficulty-progression
policy); per-attempt help-request envelope (per-learner office-
hours envelope per the per-curriculum support policy).

## A.3 Ownership + borrow-check teaching protocol

Ownership + borrow-check teaching protocols cover the most-difficult
conceptual hurdle for new Rust learners (per the *The Rust Programming
Language* Chapter 4 + the WIA-RUST-LEARN Level 2 curriculum): per-
concept incremental introduction (per-step ownership-without-borrow
introduction; per-step `&` shared-borrow introduction; per-step
`&mut` exclusive-borrow introduction; per-step lifetime-elision
introduction; per-step named-lifetime introduction); per-concept
visual aid envelope (per-concept Manim animation per the per-
curriculum NRT — No Runtime, no GC — visualisation envelope); per-
concept compiler-error walkthrough envelope (per-error E0382
"borrow of moved value" + E0499 "cannot borrow as mutable more
than once" + E0502 "cannot borrow as mutable because also borrowed
as immutable" per the rustc error-index at
`https://doc.rust-lang.org/error_codes/`); per-concept canonical-
mistake catalogue envelope per the per-curriculum common-mistake
record.

## A.4 Concurrency teaching protocol

Concurrency teaching protocols (Level 5) cover: per-concept thread-
spawn envelope per `std::thread` + POSIX.1-2024 IEEE Std 1003.1
(threads + mutexes + condition variables); per-concept channel
envelope per `std::sync::mpsc` + crossbeam-channel per
crossbeam-rs/crossbeam (single-producer single-consumer + multi-
producer single-consumer + multi-producer multi-consumer envelope);
per-concept shared-state envelope per `Arc<Mutex<T>>` +
`Arc<RwLock<T>>` + the `Send` + `Sync` auto-trait introduction;
per-concept data-race-prevention envelope (per-concept the borrow
checker prevents data races at compile time + the thread-safety
guarantee envelope per the Rust Reference §6.5); per-concept async
preview envelope (forward pointer to WIA-RUST-ADVANCED Phase 3 §A.1
for runtime composition depth).

## A.5 Testing teaching protocol

Testing teaching protocols (Level 6) cover: per-concept unit-test
envelope per `#[test]` + `cargo test` per the *The Rust Programming
Language* Chapter 11 (per-test setup + per-test assertion catalogue
+ per-test panic vs expected-fail envelope); per-concept integration-
test envelope (per-tests/ directory layout + per-integration test
shared-helper envelope); per-concept documentation-test envelope
(per-doc-comment example envelope + per-rustdoc test extraction);
per-concept test-organisation envelope (per-cargo-test filter
envelope + per-cargo-test --release vs --dev profile envelope);
per-concept code-coverage envelope (per-tarpaulin per
xd009642/tarpaulin + per-llvm-cov per the per-toolchain coverage
envelope); per-concept TDD envelope (per-curriculum red-green-
refactor practice envelope per Beck 2002).

## A.6 Capstone-project protocol

Capstone-project protocols (Level 9) cover: per-track CLI envelope
(per-CLI clap per clap-rs/clap + per-CLI argument-parsing exercise
+ per-CLI subcommand exercise + per-CLI integration-test exercise);
per-track web-service envelope (per-axum per tokio-rs/axum + per-
actix-web per actix/actix-web + per-track HTTP per RFC 9110 +
per-track JSON per RFC 8259 + per-track integration-test exercise);
per-track WebAssembly envelope (per-wasm-bindgen per the
WebAssembly Core Specification 2.0 (W3C) + per-WASM Component Model
preview); per-track per-capstone rubric envelope per Phase 1 §A.5;
per-track peer-review envelope (per-capstone cohort review session +
per-capstone instructor review session); per-track publication
envelope (per-capstone GitHub-repo public envelope + per-capstone
crates.io publication where applicable per the per-curriculum
publication-policy).


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
