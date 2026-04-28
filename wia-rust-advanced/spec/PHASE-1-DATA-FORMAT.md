# WIA-RUST-ADVANCED — Phase 1: Data Format

> Rust-advanced canonical Phase 1: crate-record + dependency + edition + target + feature-flag + safety-attestation envelopes per Rust Reference + Cargo Manifest Format.


## A.1 Crate-record envelope

The Phase 1 envelope groups Rust crates by class per the Cargo
Manifest Format (the `[package]` + `[lib]` + `[[bin]]` + `[[example]]`
+ `[[test]]` + `[[bench]]` sections defined at
`https://doc.rust-lang.org/cargo/reference/manifest.html`):
library — declared via `[lib]` with the per-crate `crate-type`
envelope (`rlib` default, `dylib`, `staticlib`, `cdylib` for FFI,
`bin` for executables, `proc-macro` for procedural macros);
binary — declared via `[[bin]]` with the per-binary entry point;
proc-macro — declared via `[lib] proc-macro = true` with the
hygiene + span tracking envelope per `proc-macro` crate; example +
test + bench — declared via `[[example]]` + `[[test]]` + `[[bench]]`
with the per-target compilation envelope. Each crate record carries:
crate identifier (per the package name + version per SemVer per
semver.org), edition (`edition = "2015" | "2018" | "2021" | "2024"`
per the Rust Edition Guide), MSRV (`rust-version = "1.83"` minimum
supported Rust version per RFC 2495 + the per-crate `rust-toolchain.toml`
envelope), per-crate license (SPDX identifier per `license = "MIT
OR Apache-2.0"` convention), per-crate metadata (`authors` +
`description` + `documentation` + `homepage` + `repository` +
`readme` + `keywords` + `categories`), and the per-crate audit
envelope tied to the crates.io publication record.

## A.2 Dependency-record envelope

Dependency-record envelopes follow the Cargo `[dependencies]` +
`[dev-dependencies]` + `[build-dependencies]` + `[target.'cfg(...)'.
dependencies]` sections: per-dependency name + version requirement
(per Cargo SemVer per `https://doc.rust-lang.org/cargo/reference/
specifying-dependencies.html`); per-dependency source envelope
(crates.io default + per-dependency `git = "https://..."` + per-
dependency `path = "..."` + per-dependency `registry = "..."` for
private registries); per-dependency feature envelope (`default-
features = false` + `features = ["..."]` for granular feature
selection); per-dependency `optional = true` envelope tied to the
parent crate's feature gate; per-dependency `package = "..."`
rename envelope; per-dependency Cargo.lock pinning per
content-hash + per-dependency reproducibility envelope per RFC 2706
+ the `--frozen` / `--locked` flag.

## A.3 Edition + MSRV envelope

Edition + MSRV envelopes track the per-crate language compatibility:
edition envelope (2015 — initial; 2018 — `dyn Trait` mandate +
module-system overhaul + `?` operator + non-lexical lifetimes;
2021 — disjoint capture + `IntoIterator` for arrays + Rust 2021
prelude; 2024 — async-fn-in-trait stabilisation + RPITIT +
Edition Guide 2024 envelope per the lang-team RFC catalogue);
MSRV envelope (per RFC 2495 + the `rust-version` field; per-crate
MSRV cannot exceed the latest stable; per-dependency MSRV envelope
must be compatible with the parent crate MSRV — `cargo msrv`
verifier per foresterre/cargo-msrv); per-crate edition migration
envelope (`cargo fix --edition` per the Edition Guide migration
catalogue); per-edition lint envelope (per-edition `#[deny(...)]`
+ per-edition `#[warn(...)]` per the per-edition RUSTC_BOOTSTRAP
envelope).

## A.4 Target + ABI envelope

Target + ABI envelopes describe per-target compilation: target
triple per LLVM target spec (per `arch-vendor-os-abi` per LLVM
`https://llvm.org/doxygen/classllvm_1_1Triple.html`);
per-target `cfg` predicate envelope (`cfg(target_arch = "x86_64")`
+ `cfg(target_os = "linux")` + `cfg(target_env = "gnu")` + per-
target `cfg(target_feature = "sse4.2")` per the Rust Reference §6.2
conditional compilation); per-target ABI envelope (`extern "C"` per
ISO/IEC 9899 platform ABI + `extern "system"` per Win32 stdcall +
`extern "Rust"` default + `extern "C-unwind"` for C panic
propagation per RFC 2945); per-target `#[repr(...)]` layout envelope
(`#[repr(C)]` per ISO/IEC 9899 layout; `#[repr(transparent)]` per
RFC 1758; `#[repr(packed(N))]` per RFC 1240; `#[repr(align(N))]` per
RFC 1358; `#[repr(Rust)]` default with non-stable layout); per-
target `target-cpu` + `target-feature` envelope per `rustc -C`
flags + the `RUSTFLAGS` environment variable.

## A.5 Feature-flag envelope

Feature-flag envelopes carry the Cargo `[features]` section: per-
feature name + per-feature dependency-list per the Cargo Reference
Features chapter at `https://doc.rust-lang.org/cargo/reference/
features.html`; per-feature default envelope (`default = ["std",
"derive"]`); per-feature mutual-exclusion envelope (per the per-
crate documentation since Cargo features are additive by default);
per-feature `dep:` envelope (Rust 2021 `dep:foo` syntax for
explicit optional-dependency reference per RFC 3143); per-feature
`?` envelope (Rust 2021 `foo?/feature` syntax for weak feature
activation per RFC 3143); per-feature `cfg(feature = "foo")`
envelope per the Rust Reference §6.2; per-crate `cargo build
--all-features` + `cargo build --no-default-features` verification
matrix.

## A.6 Safety-attestation envelope

Safety-attestation envelopes carry per-`unsafe`-block + per-`unsafe
fn` + per-`unsafe trait` + per-`unsafe impl` justification per the
Rustonomicon at `https://doc.rust-lang.org/nomicon/`: per-`unsafe`
soundness contract envelope (per-block `// SAFETY: ...` comment per
the standard library convention per RFC 2585); per-`unsafe` UB
taxonomy reference envelope (per the Reference §6.5 Behavior
considered undefined: data races, dereferencing dangling pointers,
mutating immutable data, calling functions with the wrong ABI,
misaligned access, accessing uninitialised memory, type confusion,
violating sound type invariants); per-crate `forbid(unsafe_code)`
envelope where the crate guarantees no unsafe; per-crate `deny(
unsafe_op_in_unsafe_fn)` envelope per RFC 2585; per-crate
`cargo miri test` verification envelope for unsafe-touching tests.


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
