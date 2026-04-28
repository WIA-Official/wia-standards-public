#!/bin/bash

################################################################################
# WIA-RUST-ADVANCED — Reference CLI
#
# @version 1.0.0
# @license MIT
#
# 弘益人間 (Benefit All Humanity)
#
# CLI helpers for the WIA Rust Advanced standard. Generates skeleton
# Cargo.toml + src/lib.rs that exhibit the conformance envelope:
# safety attestation, edition pinning, MSRV, target features.
################################################################################

set -e

INDIGO='\033[0;94m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

VERSION="1.0.0"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║   🦀 WIA-RUST-ADVANCED Reference CLI                           ║"
    echo "║                      Version $VERSION                          ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() { echo -e "${GREEN}✓ $1${RESET}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${RESET}"; }
print_info()    { echo -e "${GRAY}  $1${RESET}"; }

# ----------------------------------------------------------------------
# scaffold — emit a conformant Cargo.toml + src/lib.rs skeleton
# ----------------------------------------------------------------------
scaffold() {
    local crate_name="${1:-my-crate}"
    local edition="${2:-2024}"
    local msrv="${3:-1.83}"
    local out_dir="${4:-./${crate_name}}"

    print_section "Scaffolding ${crate_name} (edition ${edition}, MSRV ${msrv})"

    mkdir -p "${out_dir}/src"

    cat > "${out_dir}/Cargo.toml" <<TOML
[package]
name = "${crate_name}"
version = "0.1.0"
edition = "${edition}"
rust-version = "${msrv}"
license = "MIT OR Apache-2.0"
description = "Conforms to WIA-RUST-ADVANCED v1.0"
documentation = "https://docs.rs/${crate_name}"
repository = "https://github.com/example/${crate_name}"
readme = "README.md"
keywords = ["wia", "wia-rust-advanced"]
categories = ["development-tools"]

[lib]
crate-type = ["rlib"]

[features]
default = ["std"]
std = []

[dependencies]

[profile.release]
opt-level = 3
lto = "fat"
codegen-units = 1
panic = "abort"
strip = true

[lints.rust]
unsafe_op_in_unsafe_fn = "deny"
unused_must_use = "deny"

[lints.clippy]
all = { level = "deny", priority = -1 }
pedantic = "warn"
TOML
    print_success "Wrote ${out_dir}/Cargo.toml"

    cat > "${out_dir}/src/lib.rs" <<'RS'
//! Conforms to WIA-RUST-ADVANCED v1.0
//!
//! Per the conformance envelope:
//! - Edition pinned via Cargo.toml `edition`
//! - MSRV pinned via Cargo.toml `rust-version`
//! - Every `unsafe` block carries a `// SAFETY:` justification
//! - `unsafe_op_in_unsafe_fn` denied at the lint level

#![deny(unsafe_op_in_unsafe_fn)]

/// Adds two `u32` values, saturating at `u32::MAX`.
#[must_use]
pub const fn saturating_add(a: u32, b: u32) -> u32 {
    a.saturating_add(b)
}

/// Demonstrates an attested `unsafe` block.
///
/// # Safety
///
/// The caller MUST guarantee that `ptr` is non-null, properly aligned,
/// and points to a valid `u32` for the duration of the call. The
/// pointer MUST NOT be mutated through any other path while this
/// function holds it.
#[must_use]
pub unsafe fn read_u32_attested(ptr: *const u32) -> u32 {
    // SAFETY: documented above; caller upholds non-null + alignment +
    // validity + no-aliasing-mutation invariants.
    unsafe { ptr.read() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saturating_add_overflows_to_max() {
        assert_eq!(saturating_add(u32::MAX, 1), u32::MAX);
    }

    #[test]
    fn read_u32_attested_reads_value() {
        let v: u32 = 42;
        // SAFETY: `&v as *const u32` is non-null, aligned, valid for
        // the duration of this expression, and not aliased mutably.
        let read = unsafe { read_u32_attested(&v) };
        assert_eq!(read, 42);
    }
}
RS
    print_success "Wrote ${out_dir}/src/lib.rs"

    cat > "${out_dir}/README.md" <<MD
# ${crate_name}

Conforms to WIA-RUST-ADVANCED v1.0.

## Build

\`\`\`
cargo build --release
\`\`\`

## Test

\`\`\`
cargo test
\`\`\`

弘益人間 — Benefit All Humanity.
MD
    print_success "Wrote ${out_dir}/README.md"

    print_info "Run inside ${out_dir}:"
    print_info "  cargo build --release"
    print_info "  cargo test"
}

# ----------------------------------------------------------------------
# attest — print a SAFETY attestation envelope per Phase 1 §A.6
# ----------------------------------------------------------------------
attest() {
    local file="${1:?usage: attest <file> <line> <invariant>}"
    local line="${2:?}"
    local invariant="${3:?}"

    print_section "Soundness attestation envelope"

    local digest
    if command -v sha256sum >/dev/null 2>&1; then
        digest=$(sha256sum "${file}" | awk '{print $1}')
    else
        digest="(sha256sum unavailable)"
    fi

    cat <<JSON
{
  "kind": "wia-rust-advanced/soundness-attestation",
  "file": "${file}",
  "line": ${line},
  "file_sha256": "${digest}",
  "invariant": "${invariant}",
  "miri_pass": null,
  "sanitizers_pass": null,
  "reviewer_credential": null,
  "timestamp_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
JSON

    print_success "Attestation envelope emitted (Phase 1 §A.6)."
}

# ----------------------------------------------------------------------
# build — wrap cargo build with the conformance flags
# ----------------------------------------------------------------------
build() {
    local profile="${1:-release}"

    print_section "cargo build --profile ${profile}"

    if ! command -v cargo >/dev/null 2>&1; then
        print_warning "cargo not found in PATH — install via https://rustup.rs"
        return 1
    fi

    if [ "${profile}" = "release" ]; then
        cargo build --release
    else
        cargo build
    fi
    print_success "Build complete."
}

# ----------------------------------------------------------------------
# check — run the conformance check suite (cargo + clippy + miri)
# ----------------------------------------------------------------------
check() {
    print_section "Conformance check suite"

    if ! command -v cargo >/dev/null 2>&1; then
        print_warning "cargo not found — install via https://rustup.rs"
        return 1
    fi

    print_info "→ cargo fmt --all -- --check"
    cargo fmt --all -- --check || print_warning "fmt drift detected"

    print_info "→ cargo clippy --all-targets -- -D warnings"
    cargo clippy --all-targets -- -D warnings || print_warning "clippy lints failed"

    print_info "→ cargo test --all-features"
    cargo test --all-features || print_warning "tests failed"

    if cargo miri --version >/dev/null 2>&1; then
        print_info "→ cargo miri test"
        cargo miri test || print_warning "miri reported UB"
    else
        print_info "miri not installed — skip (rustup +nightly component add miri)"
    fi
    print_success "Conformance check complete."
}

# ----------------------------------------------------------------------
# help
# ----------------------------------------------------------------------
show_help() {
    print_header
    cat <<HELP
Usage: wia-rust-advanced <command> [args]

Commands:
  scaffold <crate> [edition] [msrv] [out-dir]
      Emit a conformant Cargo.toml + src/lib.rs skeleton.
      Default edition=2024, msrv=1.83.

  attest <file> <line> <invariant>
      Emit a soundness-attestation envelope (Phase 1 §A.6).

  build [profile]
      cargo build with the conformance profile (default release).

  check
      Run cargo fmt + clippy + test + miri (if available).

  help
      Show this help.

References: Rust Reference + Rustonomicon + Cargo Manifest Format +
            Rust Edition Guide + WebAssembly Core 2.0 + WASI Preview 2 +
            tokio + async-std + ISO/IEC 9899 + ISO/IEC 14882.

弘益人間 (Benefit All Humanity)
HELP
}

case "${1:-help}" in
    scaffold)  shift; scaffold "$@" ;;
    attest)    shift; attest "$@" ;;
    build)     shift; build "$@" ;;
    check)     check ;;
    -h|--help|help|*) show_help ;;
esac
