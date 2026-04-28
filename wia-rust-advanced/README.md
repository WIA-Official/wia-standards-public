# WIA-RUST-ADVANCED — Advanced Rust Standard

> Open standard for advanced Rust: async runtime, unsafe code, FFI,
> performance optimisation, procedural macros, and the Edition Guide
> migration envelope.

**Standard ID**: WIA-RUST-ADVANCED
**Version**: 1.0.0
**Status**: Active
**Category**: Programming Languages
**Emoji**: 🦀
**License**: MIT

## One-line definition

WIA-RUST-ADVANCED defines the conformance envelope for advanced Rust
deployments: async-runtime composition (tokio + async-std + smol), unsafe
+ FFI per the Rustonomicon + ISO/IEC 9899 + ISO/IEC 14882, no-std + alloc
+ embedded targets, procedural-macro authorship, and the Edition Guide
2024 migration path.

## Scope

This standard covers:

1. **Async runtime composition** — tokio + async-std + smol +
   embassy + glommio, per-runtime executor model, per-runtime task
   primitives, the `Future` + `Pin` + `Unpin` trait surface, and the
   `Waker` + reactor envelope per the `core::task` module.

2. **Unsafe Rust** — per the Rustonomicon: raw pointers, unions,
   `unsafe` blocks + `unsafe fn` + `unsafe trait` + `unsafe impl`,
   the soundness contract (UB taxonomy), interior mutability through
   `UnsafeCell`, lifetime variance, the `Send` + `Sync` auto-trait
   envelope.

3. **FFI** — C interop per ISO/IEC 9899 + Rust Reference §6.7
   `extern` blocks; C++ interop per ISO/IEC 14882 + cxx + autocxx;
   ABI envelope (`extern "C"` / `extern "system"` / `extern
   "Rust"`); `#[repr(C)]` / `#[repr(transparent)]` / `#[repr(packed)]`
   layout control; `bindgen` + `cbindgen` toolchain.

4. **Performance optimisation** — LLVM IR Reference + the
   `cargo-asm` + `cargo-show-asm` + Compiler Explorer workflow;
   profile-guided optimisation per LLVM PGO; link-time optimisation
   per Cargo `lto` profile; `target-cpu` + `target-feature`; SIMD
   per `core::simd` + `std::arch`.

5. **WebAssembly target** — WebAssembly Core Specification 2.0 (W3C)
   + WASI Preview 2 + Component Model per Bytecode Alliance; the
   `wasm32-unknown-unknown` + `wasm32-wasi` target envelope; per-
   crate `wasm-bindgen` + `wasm-pack` + `trunk` + `leptos` + `yew`
   + `dioxus` toolchain.

6. **Procedural macros** — the `proc-macro` crate type; `syn` +
   `quote` + `proc-macro2` toolchain; derive macros + attribute
   macros + function-like macros; hygiene + span tracking; the
   `proc-macro-hack` legacy + the stable Rust 2024 procedural-macro
   surface.

7. **No-std + alloc + embedded** — the `core` + `alloc` + `std`
   layered crates; `#![no_std]` envelope; the `embedded-hal` ecosystem
   per the Rust Embedded Working Group; per-MCU runtime envelope
   (cortex-m + riscv + xtensa + msp430).

8. **Edition Guide migration** — Edition 2015 → 2018 → 2021 → 2024
   per the Rust Edition Guide; per-edition lints and the
   `cargo fix --edition` migration tool; per-edition idiom shifts
   (e.g., `dyn Trait` mandate, `?` operator, disjoint capture).

## Conformance §1 — Phase compliance

A WIA-RUST-ADVANCED conformant project MUST satisfy every Phase 1
through Phase 4 envelope (see `spec/PHASE-1-DATA-FORMAT.md` through
`spec/PHASE-4-INTEGRATION.md`).

## Conformance §2 — Toolchain envelope

A conformant project MUST pin the Rust toolchain via `rust-toolchain.toml`
naming a stable channel ≥1.83 (or per the Edition Guide the newest stable
that matches the project's declared edition). Nightly-only features (per
the `feature(...)` envelope) MUST be guarded behind a `#[cfg(...)]`
predicate and documented in the project's stability statement.

## Conformance §3 — Soundness envelope

Every `unsafe` block in a conformant project MUST carry a `// SAFETY:`
comment per the standard library convention. The justification MUST
reference the specific invariant the surrounding code maintains. A
conformant project SHOULD pass `cargo miri test` for any test reaching
unsafe code paths and SHOULD pass `cargo +nightly udeps` to detect
unused dependencies.

## Conformance §4 — Cross-standard composition

WIA-RUST-ADVANCED composes with WIA-RUST-LEARN (educational lifecycle
predecessor) and with WIA-OMNI-API for HTTP-surface conformance. A
conformant project SHOULD reuse the WIA Family identity envelope per
WIA-AIR-SHIELD where federated trust is needed.

## Reference implementations + CLI

The reference CLI at `cli/wia-rust-advanced.sh` ships sample
`Cargo.toml` + `src/lib.rs` skeletons demonstrating the conformance
envelope. The interactive simulator at `simulator/index.html` walks
the per-Phase contract with worked examples.

## Status + roadmap

v1.0.0 (this release) covers the eight scope areas above. v1.1
(planned) will add: async-trait stabilisation tracking per Rust 1.75+;
GAT (generic associated types) per Rust 1.65+; type-alias-impl-trait
(TAIT) per the lang-team RFC envelope.

## Related WIA standards

- **WIA-RUST-LEARN** — beginner-to-intermediate Rust learning standard
- **WIA-OMNI-API** — HTTP API conformance envelope
- **WIA-AIR-SHIELD** — runtime trust list
- **WIA-INTENT** — workload intent declaration

## License

MIT — see `LICENSE` at repo root.

弘益人間 (Benefit All Humanity) — © 2025 SmileStory Inc. / WIA
