# WIA-RUST-LEARN — Beginner-to-Intermediate Rust Learning Standard

> Open standard defining the curriculum scope, conformance bar, and
> learner-progression envelope for "Zero-to-Rust" learning paths.

**Standard ID**: WIA-RUST-LEARN
**Version**: 1.0.0
**Status**: Active
**Category**: Programming Languages — Education
**Emoji**: 🦀
**License**: MIT
**Companion advanced track**: WIA-RUST-ADVANCED

## One-line definition

WIA-RUST-LEARN defines a 10-level conformance envelope (Level 0 setup
through Level 9 capstone projects) covering the Rust language surface
that a beginner-to-intermediate learner needs to master before entering
the WIA-RUST-ADVANCED track.

## Scope

This standard covers:

1. **Level 0 — Setup**
   - Why Rust? (memory safety + concurrency without GC + zero-cost abstractions)
   - Installing Rust (rustup per https://rustup.rs)
   - Cargo deep-dive (Cargo Manifest Format + workspace + features)

2. **Level 1 — Basics**
   - Variables + types per the Rust Reference §6 + IEEE 754-2019 (f32/f64)
   - Functions + control flow + expression-vs-statement
   - Standard input/output + error envelope per `std::io`
   - String + `&str` + UTF-8 per Unicode 15.1

3. **Level 2 — Ownership ⭐ NRT (No Runtime, no GC)**
   - Move semantics + `Copy` trait
   - Borrowing + the borrow checker rules
   - Lifetimes (named + elided)
   - Slices + dynamically sized types

4. **Level 3 — Structure**
   - Structs + tuple structs + unit structs
   - Enums + pattern matching
   - Modules + crates + visibility envelope

5. **Level 4 — Advanced**
   - Generics + monomorphisation
   - Traits + default methods + trait objects (`dyn Trait`)
   - Iterators + closures + the Iterator combinator catalogue

6. **Level 5 — Concurrency**
   - Threads per `std::thread` + POSIX.1-2024 IEEE Std 1003.1
   - Channels per `std::sync::mpsc` + crossbeam-channel
   - Shared state per `Arc<Mutex<T>>` + `Arc<RwLock<T>>`

7. **Level 6 — Testing**
   - Unit tests per `#[test]` + `cargo test`
   - Integration tests in `tests/`
   - Documentation tests + `///` doc-comments + rustdoc

8. **Level 7 — Patterns**
   - Error handling per `Result<T, E>` + `?` operator + `thiserror` + `anyhow`
   - Common patterns (newtype, RAII, builder, typestate)

9. **Level 8 — Unsafe (introduction)**
   - When unsafe is needed (FFI, low-level memory, intrinsics)
   - The five superpowers per the Rust Book Ch.19
   - Forward pointer to WIA-RUST-ADVANCED Phase 3 §A.2 for depth

10. **Level 9 — Capstone projects**
    - CLI tool with clap + structopt-style argument parsing
    - Web service with axum or actix-web
    - WebAssembly module per WebAssembly Core Specification 2.0 (W3C)

## Conformance §1 — Curriculum delivery

A WIA-RUST-LEARN conformant curriculum MUST deliver every Level 0
through Level 9 envelope. Per-level mastery is verified through the
per-level rustlings exercise pack at
`https://github.com/rust-lang/rustlings`, the per-level Rust by Example
exercises at `https://doc.rust-lang.org/rust-by-example/`, and the
per-level capstone exercise per the per-curriculum host.

## Conformance §2 — Toolchain envelope

A conformant curriculum installs Rust via `rustup` per
`https://rustup.rs` on the stable channel ≥1.83 (matching the
WIA-RUST-ADVANCED MSRV baseline). Beginner exercises target the
2024 edition by default; legacy-edition exercises (2015 / 2018 / 2021)
are explicitly tagged.

## Conformance §3 — Reference materials

A conformant curriculum cross-references the canonical learning
materials:

- **The Book** — *The Rust Programming Language* (Klabnik + Nichols)
  at `https://doc.rust-lang.org/book/`
- **Rust by Example** at `https://doc.rust-lang.org/rust-by-example/`
- **Rustlings** exercises at `https://github.com/rust-lang/rustlings`
- **The Reference** — Rust Reference Manual at
  `https://doc.rust-lang.org/reference/`
- **The Standard Library** — `https://doc.rust-lang.org/std/`
- **Cargo Book** — `https://doc.rust-lang.org/cargo/`

## Conformance §4 — Cross-track composition

WIA-RUST-LEARN is the entry-level companion to WIA-RUST-ADVANCED.
A learner completing WIA-RUST-LEARN Level 9 SHOULD be able to read
WIA-RUST-ADVANCED Phase 1 (data format) without reaching for
external references on basic syntax. WIA-RUST-LEARN does NOT cover
async, FFI depth, procedural macros, or the unsafe-soundness contract
in detail; those are WIA-RUST-ADVANCED scope.

## Reference implementations

- **CLI** — `cli/wia-rust-learn.sh` ships per-level scaffolding +
  exercise-bootstrapping helpers.
- **Curriculum site** — `curriculum/index.html` walks the 10-level path.
- **Video catalogue** — `videos/index.html` (28 episodes).
- **Certification track** — `certification/index.html` (per-level
  badge + per-track certification envelope).
- **Manim animations** — `manim/episodes/` (28 animation scripts
  for the per-level video curriculum).
- **Interactive playground** — `playground/` (per-level exercise
  starter code).

## Status + roadmap

v1.0.0 (this release) covers Level 0 through Level 9. v1.1 (planned)
will add: Level 10 — published-crate authorship; expanded async
coverage as a bridge to WIA-RUST-ADVANCED.

## Related WIA standards

- **WIA-RUST-ADVANCED** — companion advanced-track standard
- **WIA-OMNI-API** — HTTP API conformance for capstone web services

## License

MIT — see `LICENSE` at repo root.

弘益人間 (Benefit All Humanity) — © 2025 SmileStory Inc. / WIA
