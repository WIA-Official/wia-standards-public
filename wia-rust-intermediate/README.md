# WIA Rust Intermediate

> **Standard**: WIA Rust Intermediate
> **Version**: 1.0.0
> **Status**: Draft
> **Philosophy**: 弘益人間 — Benefit All Humanity

WIA Rust Intermediate is the curriculum and conformance standard for the
"intermediate" tier of the WIA Rust Learning ladder. It sits between the
beginner standard (WIA Rust Learn) and the advanced standard
(WIA Rust Advanced), and it codifies the topics, depth, exercises and
assessment shape that an intermediate Rust learner SHOULD have mastered.

## Why this standard exists

* **Portability of credentials.** A learner who completes a WIA Rust
  Intermediate programme at one academy SHOULD be recognised at any other
  academy or employer that honours the standard.
* **Curriculum interoperability.** Course authors can publish a
  conformance manifest declaring "this course covers WIA Rust Intermediate
  modules 1–4 with assessment level Core." Other authors can compose
  modules without re-deriving prerequisites.
* **Assessment objectivity.** Phase 4 publishes a reference assessment
  bank so two academies grading the same submission converge on the
  same competency band.

## Module map

| # | Module | Outcome |
|---|--------|---------|
| 1 | Ownership in depth | Move semantics, borrow tree, partial moves |
| 2 | Lifetimes | Lifetime elision, explicit annotations, HRTB basics |
| 3 | Traits | Bounds, default methods, associated types, blanket impls |
| 4 | Generics | Monomorphisation, trait objects, type erasure trade-offs |
| 5 | Error handling | `Result`, `?`, error trait composition, `thiserror` patterns |
| 6 | Concurrency | Threads, channels, `Send`/`Sync` reasoning |
| 7 | Async basics | `Future`, `async`/`.await`, runtime selection |
| 8 | Crate ecosystem | Cargo workspaces, semver discipline, MSRV policy |

Phase 1–4 specifications fix the data shape of curriculum manifests,
academy APIs, federation between academies, and integration with the
broader WIA family.

## Contents

```
wia-rust-intermediate/
├── README.md           # this file
├── index.html          # public landing page
├── spec/               # PHASE-1 .. PHASE-4 specifications
├── api/                # TypeScript reference SDK
├── cli/                # CLI tool for validating manifests
├── ebook/{en,ko}/      # narrative learning text, 8 chapters per language
├── simulator/          # interactive walkthroughs
└── press/              # release notes, changelog
```

## How to use

* **Curriculum authors** read Phase 1 to learn the manifest shape, then
  publish a manifest under their own domain and submit it via Phase 2's
  API.
* **Academies** consume manifests, run assessments, and publish learner
  records via Phase 2's `/wri/learner` endpoint.
* **Employers** verify credentials via the federation handshake described
  in Phase 3.
* **Learners** read the ebook (`ebook/en/` or `ebook/ko/`) and use the
  simulator to practise.

## Conformance levels

| Level | Required modules | Assessment |
|-------|------------------|------------|
| Minimal | 1, 2, 5 | Submission of one working program per module |
| Core    | 1–6     | Plus rubric-graded code review |
| Full    | 1–8     | Plus oral defence and concurrency project |

## Companion standards

WIA Rust Intermediate composes with:

* **WIA Rust Learn** — beginner curriculum (prerequisite tier).
* **WIA Rust Advanced** — advanced curriculum (next tier).
* **WIA-OMNI-API** — credential storage for learner records.
* **WIA-ACCESSIBILITY** — accommodation profiles for learners.
* **WIA Standards** — overall standards governance.

弘益人間 — Benefit All Humanity.
