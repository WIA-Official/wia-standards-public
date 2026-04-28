#!/bin/bash

################################################################################
# WIA-RUST-LEARN — Reference CLI
#
# @version 1.0.0
# @license MIT
#
# 弘益人間 (Benefit All Humanity)
#
# CLI helpers for the WIA Rust Learn (Zero-to-Rust) standard.
# Bootstraps per-level exercise environments, scaffolds learner workspaces,
# and emits xAPI 2.0 progress records.
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

LEVELS=(
    "0:Setup"
    "1:Basics"
    "2:Ownership"
    "3:Structure"
    "4:Advanced"
    "5:Concurrency"
    "6:Testing"
    "7:Patterns"
    "8:Unsafe"
    "9:Capstone"
)

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║   🦀 WIA-RUST-LEARN Reference CLI (Zero-to-Rust)               ║"
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
# levels — list the 10-level conformance envelope
# ----------------------------------------------------------------------
list_levels() {
    print_section "WIA-RUST-LEARN levels"
    for entry in "${LEVELS[@]}"; do
        local num="${entry%%:*}"
        local title="${entry#*:}"
        printf "  ${CYAN}Level %s${RESET}  %s\n" "${num}" "${title}"
    done
    echo ""
    print_info "Reference: README.md §Scope · spec/PHASE-1-DATA-FORMAT.md §A.2"
}

# ----------------------------------------------------------------------
# scaffold-level — set up a learner workspace for a target level
# ----------------------------------------------------------------------
scaffold_level() {
    local level="${1:?usage: scaffold-level <0..9> [out-dir]}"
    local out_dir="${2:-./level-${level}}"

    if [ "${level}" -lt 0 ] || [ "${level}" -gt 9 ]; then
        print_warning "Level must be 0..9"
        return 1
    fi

    print_section "Scaffolding Level ${level} workspace at ${out_dir}"
    mkdir -p "${out_dir}/src"

    cat > "${out_dir}/Cargo.toml" <<TOML
[package]
name = "level-${level}-workspace"
version = "0.1.0"
edition = "2024"
rust-version = "1.83"
license = "MIT OR Apache-2.0"
description = "WIA-RUST-LEARN Level ${level} learner workspace"

[dependencies]

[dev-dependencies]
TOML
    print_success "Cargo.toml emitted"

    case "${level}" in
        0)
            cat > "${out_dir}/src/main.rs" <<'RS'
fn main() {
    println!("Hello, Rust! — WIA-RUST-LEARN Level 0");
}
RS
            ;;
        1)
            cat > "${out_dir}/src/main.rs" <<'RS'
fn main() {
    let name = "Rustacean";
    let count: u32 = 42;
    println!("{name}, you have {count} crates to learn.");
}
RS
            ;;
        2)
            cat > "${out_dir}/src/main.rs" <<'RS'
fn main() {
    let s = String::from("ownership");
    let len = compute_length(&s);
    println!("'{s}' has length {len}");
}

fn compute_length(s: &str) -> usize {
    s.len()
}
RS
            ;;
        9)
            cat > "${out_dir}/src/main.rs" <<'RS'
//! Level 9 — Capstone scaffold (CLI starter)

use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    println!("Capstone CLI — args: {args:?}");
    println!("Replace this with your capstone project.");
}
RS
            ;;
        *)
            cat > "${out_dir}/src/main.rs" <<RS
fn main() {
    println!("WIA-RUST-LEARN Level ${level} — fill in your exercises here.");
}
RS
            ;;
    esac
    print_success "src/main.rs emitted"

    cat > "${out_dir}/README.md" <<MD
# Level ${level} Workspace — WIA-RUST-LEARN

Build:

\`\`\`
cargo build
\`\`\`

Run:

\`\`\`
cargo run
\`\`\`

Test:

\`\`\`
cargo test
\`\`\`

Forward to:
- The Rust Programming Language: https://doc.rust-lang.org/book/
- Rust by Example: https://doc.rust-lang.org/rust-by-example/
- Rustlings: https://github.com/rust-lang/rustlings

弘益人間 — Benefit All Humanity.
MD
    print_success "README.md emitted"
    print_info "Next: cd ${out_dir} && cargo run"
}

# ----------------------------------------------------------------------
# rustlings — set up the upstream rustlings exercise pack
# ----------------------------------------------------------------------
rustlings_setup() {
    print_section "Rustlings setup (per Phase 1 §A.6)"
    if ! command -v cargo >/dev/null 2>&1; then
        print_warning "cargo not in PATH — install Rust via https://rustup.rs"
        return 1
    fi
    print_info "Install rustlings:"
    print_info "  cargo install rustlings"
    print_info "Initialise local exercise directory:"
    print_info "  rustlings init"
    print_info "Run interactively:"
    print_info "  rustlings"
    print_info "Source: https://github.com/rust-lang/rustlings"
    print_success "Setup instructions printed."
}

# ----------------------------------------------------------------------
# progress — emit an xAPI 2.0 statement for a learner attempt
# ----------------------------------------------------------------------
progress() {
    local learner="${1:?usage: progress <learner-id> <exercise-id> <PASSED|FAILED>}"
    local exercise="${2:?}"
    local outcome="${3:?}"

    print_section "xAPI 2.0 statement (Phase 1 §A.4 / Phase 2 §A.4)"

    local verb_id verb_disp
    case "${outcome}" in
        PASSED)  verb_id="http://adlnet.gov/expapi/verbs/passed"; verb_disp="passed" ;;
        FAILED)  verb_id="http://adlnet.gov/expapi/verbs/failed"; verb_disp="failed" ;;
        *)       verb_id="http://adlnet.gov/expapi/verbs/attempted"; verb_disp="attempted" ;;
    esac

    cat <<JSON
{
  "actor": {
    "objectType": "Agent",
    "account": {
      "homePage": "https://wiastandards.com/wia-rust-learn",
      "name": "${learner}"
    }
  },
  "verb": {
    "id": "${verb_id}",
    "display": { "en-US": "${verb_disp}" }
  },
  "object": {
    "objectType": "Activity",
    "id": "https://wiastandards.com/wia-rust-learn/exercises/${exercise}",
    "definition": {
      "type": "http://adlnet.gov/expapi/activities/assessment",
      "name": { "en-US": "${exercise}" }
    }
  },
  "result": {
    "completion": true,
    "success": $([ "${outcome}" = "PASSED" ] && echo true || echo false)
  },
  "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "context": {
    "platform": "wia-rust-learn",
    "language": "en-US"
  }
}
JSON
    print_success "xAPI statement emitted (per IEEE 9274.1.1 + ADL xAPI 2.0)."
}

# ----------------------------------------------------------------------
# help
# ----------------------------------------------------------------------
show_help() {
    print_header
    cat <<HELP
Usage: wia-rust-learn <command> [args]

Commands:
  levels
      List the 10-level WIA-RUST-LEARN curriculum envelope.

  scaffold-level <0..9> [out-dir]
      Bootstrap a per-level Cargo workspace ready for cargo run / test.

  rustlings
      Print the rustlings exercise-pack setup instructions.

  progress <learner-id> <exercise-id> <PASSED|FAILED>
      Emit an xAPI 2.0 statement (Phase 1 §A.4 / Phase 2 §A.4).

  help
      Show this help.

References: The Rust Programming Language Book + Rust by Example +
            Rustlings + Rust Reference + Cargo Book + Edition Guide +
            ISO/IEC 9899 + ISO/IEC 14882 + POSIX.1-2024 +
            ECMAScript 2024 + IEEE 754-2019 + Unicode 15.1.

弘益人間 (Benefit All Humanity)
HELP
}

case "${1:-help}" in
    levels)              list_levels ;;
    scaffold-level)      shift; scaffold_level "$@" ;;
    rustlings)           rustlings_setup ;;
    progress)            shift; progress "$@" ;;
    -h|--help|help|*)    show_help ;;
esac
