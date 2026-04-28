# WIA-RUST-ADVANCED — Phase 2: API Interface

> Rust-advanced canonical Phase 2: API surface (crates + builds + benchmarks + ABI + soundness + telemetry + audit) per Cargo + cargo-build-system.


## A.1 Endpoint reference

```http
POST /wia-rust-advanced/v1/crates                    # register crate
GET  /wia-rust-advanced/v1/crates/{id}               # fetch crate record
POST /wia-rust-advanced/v1/builds                    # trigger build
GET  /wia-rust-advanced/v1/builds/{id}               # fetch build
POST /wia-rust-advanced/v1/benchmarks                # publish benchmark
GET  /wia-rust-advanced/v1/abi-checks/{id}           # ABI compat report
POST /wia-rust-advanced/v1/soundness-attestations    # publish soundness
WS   /wia-rust-advanced/v1/state/stream              # build stream
GET  /wia-rust-advanced/v1/audit/{id}                # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-rust-advanced`. Crate-mutation endpoints require
the requester's per-crate maintainer credential per the per-registry
trust envelope (crates.io API token; per-private-registry credential).
Cross-crate visibility is gated by per-crate explicit grant per the
operator's RBAC policy.

## A.2 Crate-registration API

`POST /crates` accepts the Phase 1 §A.1 envelope. The endpoint
validates the per-crate name + version uniqueness against the per-
registry catalogue (crates.io enforces yanking-not-deletion semantics
per the crates.io policy at `https://doc.rust-lang.org/cargo/
commands/cargo-yank.html`), validates the SPDX-license envelope per
the per-crate `license` field, materialises the per-crate Cargo.toml
into the per-registry index, and emits the registration event. Crate
records transition through `provisioning`, `published`, `yanked`,
`withdrawn`; per-state transitions emit audit events.

## A.3 Build API

`POST /builds` accepts a build-trigger envelope per the Cargo
build-system at `https://doc.rust-lang.org/cargo/reference/build-
scripts.html`: crate reference, target triple per Phase 1 §A.4,
build profile (`dev` / `release` / `test` / `bench` per the per-
profile `[profile.<name>]` Cargo.toml section), feature set per
Phase 1 §A.5, and the per-build cache envelope (per-build sccache
per Mozilla sccache + per-build cargo-cache per the operator's
build-cache vendor). The endpoint dispatches to the per-platform
builder and emits the build-success or build-failure event with
the per-failure structured diagnostics envelope per `cargo --message
-format=json`.

## A.4 Benchmark API

`POST /benchmarks` accepts a benchmark envelope per `cargo bench`
+ Criterion.rs per Bheisler/criterion: per-benchmark name + per-
benchmark group + per-benchmark statistical envelope (sample-size +
warmup-time + measurement-time + per-benchmark mean ± stdev per
Criterion.rs default); per-benchmark trend envelope (per-benchmark
rolling-window comparison + per-benchmark regression detection);
per-benchmark CI-environment envelope (per-CI dedicated benchmark
runner with the operator's noise-floor policy); per-benchmark
reproducibility envelope (per-benchmark git-revision + per-benchmark
toolchain-pin + per-benchmark hardware-fingerprint).

## A.5 ABI-compat-check API

`GET /abi-checks/{id}` returns per-crate ABI-compatibility checks:
per-major-version + per-minor-version SemVer compatibility per the
per-crate cargo-semver-checks per obi1kenobi/cargo-semver-checks;
per-crate public-API surface envelope per cargo-public-api per
Enselic/cargo-public-api; per-crate ABI-stability envelope for
cdylib + staticlib targets per the per-platform ABI definition; per-
crate breaking-change envelope (function signature change; trait-
method addition; struct field addition for non-`#[non_exhaustive]`
types; enum variant addition for non-`#[non_exhaustive]` enums); per-
crate `#[non_exhaustive]` annotation envelope per RFC 2008 + the
per-crate change policy.

## A.6 Soundness-attestation API

`POST /soundness-attestations` accepts a soundness-attestation
envelope per Phase 1 §A.6: per-`unsafe`-block file + line + column
+ SHA256 of the surrounding source-file; per-`unsafe`-block
`// SAFETY: ...` justification text; per-attestation cargo-miri test
result envelope (per-test pass / fail / panic / data-race / uninit-
mem / etc per Miri report); per-attestation cargo-careful + Rust
sanitizer envelope (`-Zsanitizer=address|leak|memory|thread` per the
nightly Rust sanitizer envelope); per-attestation reviewer-credential
envelope (per-reviewer-soundness-review trail per the operator's
review policy).

## A.7 Telemetry WebSocket

The state-stream WebSocket multiplexes per-crate events: per-build
start + completion events with the per-stage timing envelope; per-
build warning + error events; per-bench regression events; per-ABI
breaking-change events; per-soundness attestation events; per-crate
publish events; per-crate yank + withdraw events. Subscribers can
filter by crate-id, build-id, severity-class. Rate limits: 5000
req/h authenticated; 50000 req/h trusted-partner. WebSocket
subscriptions are bounded at 100 simultaneous per credential.

## A.8 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: crate
registration, every build event, every benchmark event, every ABI-
check event, every soundness-attestation event, every credential
change, every yank or withdraw event. The audit-trail integrity is
anchored into a Merkle tree per-crate; the root is committed to the
operator's archival record per the operator's per-crate retention
policy (typical 7 years for general crates; 10 years for security-
critical crates per the per-jurisdiction retention envelope).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wia-rust-advanced/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wia-rust-advanced-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wia-rust-advanced-host:1.0.0` ships every wia-rust-advanced envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wia-rust-advanced.sh` ships sample envelope generators with no
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
ecosystem. Wia-rust-advanced deployments that follow this layering
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
`/.well-known/wia-wia-rust-advanced-capabilities` that enumerates which
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
