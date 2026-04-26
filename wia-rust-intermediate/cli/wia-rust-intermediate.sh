#!/bin/bash
# WIA Rust Intermediate CLI Tool
# Standard: WIA Rust Intermediate v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="wia-rust-intermediate"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/wia-rust-intermediate/"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Rust Intermediate CLI v${VERSION}${NC}
${BLUE}Curriculum and assessment for the intermediate Rust tier.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <manifest>       Validate a curriculum manifest (JSON) per Phase 1
  manifest <module-id>      Print a starter manifest for the given module id (1..8)
  learner <id>              Print a starter learner record skeleton
  rubric <module-id>        Print the assessment rubric for the module
  level <manifest>          Report which conformance level the manifest meets
  info                      Show standard summary
  help                      This help

Examples:
  $(basename "$0") validate ./course.json
  $(basename "$0") manifest 3
  $(basename "$0") learner did:wia:learner:abc
  $(basename "$0") rubric 5
  $(basename "$0") level ./course.json

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Rust Intermediate v${VERSION}
${CYAN}Purpose${NC}: Curriculum + assessment for the intermediate Rust tier
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Manifest format     — curriculum manifest JSON shape
  2. API interface       — academy publish/read endpoints
  3. Federation protocol — credential trust between academies
  4. Integration         — bridges to WIA-OMNI-API and WIA-ACCESSIBILITY

Reference signature: Edwards-curve per IETF RFC 8032 (Ed25519).
EOF
}

require_jq() {
    command -v jq >/dev/null 2>&1 || { echo -e "${RED}jq required (https://jqlang.github.io/jq/)${NC}"; exit 2; }
}

validate() {
    local f=$1
    [ -z "$f" ] && { echo -e "${RED}path required${NC}"; exit 2; }
    [ -f "$f" ] || { echo -e "${RED}not found: $f${NC}"; exit 2; }
    require_jq

    for k in wia_rust_intermediate_version author module_ids assessment signature; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            echo -e "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_rust_intermediate_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) echo -e "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    n=$(jq '.module_ids | length' "$f")
    if [ "$n" -lt 1 ] || [ "$n" -gt 8 ]; then
        echo -e "${RED}module_ids must be 1..8 entries, got ${n}${NC}"; exit 1
    fi

    echo -e "${GREEN}OK — manifest structurally valid${NC}"
}

manifest() {
    local id=$1
    [ -z "$id" ] && { echo -e "${RED}module-id required (1..8)${NC}"; exit 2; }
    cat <<JSON
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "curriculum_manifest",
  "author": "did:wia:academy:example",
  "module_ids": [${id}],
  "assessment": {
    "form": "submission",
    "rubric_id": "wri-rubric-${id}-v1"
  },
  "exercises": [
    { "exercise_id": "ex-${id}-01", "topic": "Module ${id}, exercise 1", "difficulty": "intro" },
    { "exercise_id": "ex-${id}-02", "topic": "Module ${id}, exercise 2", "difficulty": "applied" }
  ],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

learner() {
    local lid=$1
    [ -z "$lid" ] && { echo -e "${RED}learner-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "learner_record",
  "learner_id": "${lid}",
  "issued_at": "$(date -u +%FT%TZ)",
  "modules_passed": [],
  "level_attained": null,
  "academy_id": "did:wia:academy:example",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

rubric() {
    local id=$1
    [ -z "$id" ] && { echo -e "${RED}module-id required${NC}"; exit 2; }
    cat <<EOF
Module ${id} rubric (Phase 4 §2):
  Pass:   working code, all examples covered, idiomatic Rust
  Merit:  pass + property-based tests for at least one example
  Distn:  merit + extension implementing one optional feature

Reviewer assigns one band; grader confirms; learner_record updated by academy.
EOF
}

level() {
    require_jq
    local f=$1
    [ -f "$f" ] || { echo -e "${RED}manifest not found${NC}"; exit 2; }
    mods=$(jq -r '.module_ids | sort | join(",")' "$f")
    case "$mods" in
        "1,2,5") echo "Minimal" ;;
        "1,2,3,4,5,6") echo "Core" ;;
        "1,2,3,4,5,6,7,8") echo "Full" ;;
        *) echo "Custom" ;;
    esac
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate) validate "$@" ;;
        manifest) manifest "$@" ;;
        learner)  learner "$@" ;;
        rubric)   rubric "$@" ;;
        level)    level "$@" ;;
        info)     show_info ;;
        help|-h|--help) show_help ;;
        *) echo -e "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
