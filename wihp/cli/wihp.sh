#!/bin/bash
# WIHP CLI — WIA Hangul Input Protocol
# Standard: WIHP v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/wihp/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<HELP
${CYAN}WIHP CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>           Validate a record (JSON) per Phase 1
  layout <name>               Print a keyboard_layout envelope
  gesture <code>              Print a gesture envelope
  sign-mapping <lang>         Print a sign_mapping envelope
  accessibility-binding <kind>  Print an accessibility_binding envelope
  info                        Show standard summary
  help                        This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
HELP
}

show_info() {
    cat <<INFO
${CYAN}Standard${NC}: WIHP v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Federation · 4 Integration
${CYAN}Reference${NC}: Unicode CLDR, WCAG 2.2, WAI-ARIA, W3C Web Speech API
INFO
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wihp_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

layout() {
    local n=$1; [ -z "$n" ] && { printf '%s\n' "${RED}name required${NC}"; exit 2; }
    cat <<JSON
{
  "wihp_version": "1.0.0",
  "type": "keyboard_layout",
  "layout_id": "kl_${n}",
  "name": "${n}",
  "language_tag": "ko-KR",
  "script": "Hang",
  "rows": [],
  "input_method_engine": "TODO",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

gesture() {
    local c=$1; [ -z "$c" ] && { printf '%s\n' "${RED}code required${NC}"; exit 2; }
    cat <<JSON
{
  "wihp_version": "1.0.0",
  "type": "gesture",
  "gesture_id": "g_${c}",
  "code": "${c}",
  "captured_at": "$(date -u +%FT%TZ)",
  "device_id": "did:wia:device:demo",
  "trajectory_points": [],
  "duration_ms": 0,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

sign_mapping() {
    local lang=$1; [ -z "$lang" ] && { printf '%s\n' "${RED}lang required${NC}"; exit 2; }
    cat <<JSON
{
  "wihp_version": "1.0.0",
  "type": "sign_mapping",
  "mapping_id": "sm_${lang}",
  "sign_language": "${lang}",
  "target_text_language": "TODO",
  "entries": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

accessibility_binding() {
    local k=$1; [ -z "$k" ] && { printf '%s\n' "${RED}kind required${NC}"; exit 2; }
    cat <<JSON
{
  "wihp_version": "1.0.0",
  "type": "accessibility_binding",
  "binding_id": "ab_${k}",
  "input_kind": "${k}",
  "wcag_baseline": "2.2-AA",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)              validate "$@" ;;
        layout)                layout "$@" ;;
        gesture)               gesture "$@" ;;
        sign-mapping)          sign_mapping "$@" ;;
        accessibility-binding) accessibility_binding "$@" ;;
        info)                  show_info ;;
        help|-h|--help)        show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
