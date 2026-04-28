# WIA-RUST-ADVANCED — Phase 3: Protocol

> Rust-advanced canonical Phase 3: protocols (async-runtime + unsafe-FFI + procedural-macro + perf-optimisation + WASM + edition-migration) per Rustonomicon + Edition Guide.


## A.1 Async-runtime composition protocol

Async-runtime composition protocols cover: per-runtime executor
envelope (tokio per tokio.rs + async-std per async.rs + smol per
async-rs/smol + glommio per DataDog/glommio + embassy per
embassy-rs/embassy + monoio per bytedance/monoio); per-runtime
task-spawn envelope (`tokio::spawn` + `async_std::task::spawn` +
`smol::spawn` + per-runtime equivalent); per-runtime reactor envelope
(per-runtime `mio` / `polling` / `io_uring` / per-runtime epoll
envelope); per-runtime `Future` + `Pin` + `Unpin` trait surface per
the `core::future` + `core::task` + `core::pin` modules per the
Rust Reference; per-runtime `Waker` envelope per RFC 2592 (Waker
v2 with `RawWaker` + `RawWakerVTable`); per-runtime `Send` + `Sync`
auto-trait envelope (per the per-task `Send`-vs-`!Send` envelope —
tokio defaults to `Send`-required spawn; `tokio::task::spawn_local`
opts in to `!Send` per the per-task LocalSet envelope).

## A.2 Unsafe + FFI protocol

Unsafe + FFI protocols cover: C interop per ISO/IEC 9899:2024 +
Rust Reference §6.7 `extern "C"` blocks (per-extern function
declaration + per-extern function ABI + per-extern function calling
convention envelope); C++ interop per ISO/IEC 14882:2024 + cxx per
dtolnay/cxx + autocxx per google/autocxx (per-binding generation
envelope per the per-tool annotation envelope); `bindgen` per
rust-lang/rust-bindgen for C-header → Rust-binding generation
envelope; `cbindgen` per mozilla/cbindgen for Rust → C-header
generation envelope; per-FFI `#[repr(C)]` layout envelope per the
per-platform calling convention; per-FFI ownership transfer envelope
(per-FFI `Box::into_raw` + `Box::from_raw` for owned-pointer
transfer; per-FFI `Vec::into_raw_parts` + `Vec::from_raw_parts`
for owned-vector transfer; per-FFI `CString::into_raw` +
`CString::from_raw` for owned-C-string transfer); per-FFI panic-
boundary envelope per RFC 2945 `extern "C-unwind"` for C-frame
panic propagation.

## A.3 Procedural-macro protocol

Procedural-macro protocols per the Rust Reference §3.1 macros
chapter cover: derive-macro envelope (`#[derive(MyMacro)]` per
the `proc_macro_derive` attribute + per-derive helper-attribute
envelope); attribute-macro envelope (`#[my_attribute(...)]` per
the `proc_macro_attribute` attribute); function-like macro
envelope (`my_macro!(...)` per the `proc_macro` attribute); per-
macro hygiene envelope per RFC 1561 (def-site vs call-site span
envelope); per-macro `syn` per dtolnay/syn for syntax-tree parsing;
per-macro `quote` per dtolnay/quote for quasi-quoting; per-macro
`proc-macro2` per dtolnay/proc-macro2 for the per-macro token-stream
envelope; per-macro `proc-macro-error` per dtolnay/proc-macro-error
for diagnostic envelope; per-macro `proc-macro-hack` legacy + the
stable Rust 2024 surface for procedural-macro authorship.

## A.4 Performance-optimisation protocol

Performance-optimisation protocols cover: per-crate profile envelope
(`[profile.release] opt-level = 3 + lto = "fat" + codegen-units = 1
+ panic = "abort"` per the per-crate Cargo.toml + the per-binary
size-vs-speed trade-off envelope); per-crate target-cpu envelope
(`RUSTFLAGS="-C target-cpu=native"` per the per-deployment hardware
profile + the per-crate compatibility envelope); per-crate target-
feature envelope (per-feature `+sse4.2` + `+avx2` + `+fma` + `+bmi2`
+ `+aes` + `+sha` per the per-platform feature catalogue); per-crate
LLVM IR + LLVM target spec envelope per
`https://llvm.org/docs/LangRef.html`; per-crate profile-guided
optimisation per RFC 2070 `cargo build --profile release-with-pgo`;
per-crate post-link optimisation per llvm-bolt per Facebook + per-
crate inlining envelope per `#[inline]` + `#[inline(always)]` +
`#[inline(never)]` annotations.

## A.5 WebAssembly target protocol

WebAssembly target protocols cover: per-target `wasm32-unknown-
unknown` envelope per the WebAssembly Core Specification 2.0 (W3C
Recommendation per `https://www.w3.org/TR/wasm-core-2/`); per-target
`wasm32-wasi` envelope per WASI Preview 2 + Component Model per the
Bytecode Alliance + W3C WebAssembly CG (per-target `wasi-libc` per
WebAssembly/wasi-libc + per-target `wasi-sdk` envelope); per-binding
`wasm-bindgen` per rustwasm/wasm-bindgen for JavaScript-interop
envelope; per-binding `js-sys` + `web-sys` per rustwasm/wasm-bindgen
for Web-API + JS-API surface; per-deployment `wasm-pack` per
rustwasm/wasm-pack for npm-package envelope; per-deployment `trunk`
per trunk-rs/trunk for full-stack web-app envelope; per-deployment
WASM-runtime envelope (Wasmtime per Bytecode Alliance + Wasmer per
wasmerio/wasmer + WAVM per WAVM/WAVM + WasmEdge per CNCF WasmEdge).

## A.6 Edition-migration protocol

Edition-migration protocols per the Rust Edition Guide at
`https://doc.rust-lang.org/edition-guide/` cover: per-crate per-
edition migration envelope (2015 → 2018: `dyn Trait` mandate +
module-system overhaul + `?` operator + non-lexical lifetimes;
2018 → 2021: disjoint capture + `IntoIterator` for arrays + Rust
2021 prelude + reserved-syntax envelope; 2021 → 2024: async-fn-
in-trait + RPITIT + Edition Guide 2024 envelope per the lang-team
RFC catalogue); per-crate `cargo fix --edition` per the per-edition
migration tool; per-crate `cargo fix --edition --idioms` per the
per-edition idiom-shift envelope (e.g., 2018 idiom = `dyn Trait`
explicit; 2021 idiom = `IntoIterator` for arrays); per-crate
`#![allow(rust_2021_compatibility)]` envelope as a per-crate
opt-out for legacy code; per-edition stable-vs-nightly feature
envelope per the per-edition RUSTC_BOOTSTRAP catalogue.


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
