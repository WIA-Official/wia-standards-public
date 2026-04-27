#!/bin/bash
# WIA Roadmap CLI
# Standard: WIA Roadmap v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/roadmap/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<HELP
${CYAN}WIA Roadmap CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>     Validate a record (JSON) per Phase 1
  milestone <name>      Print a milestone envelope
  dependency <ms>       Print a dependency declaration
  release <ms>          Print a release envelope
  progress <ms>         Print a progress_update envelope
  commitment-change     Print a commitment_change envelope
  info                  Show standard summary
  help                  This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
HELP
}

show_info() {
    cat <<INFO
${CYAN}Standard${NC}: WIA Roadmap v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Federation · 4 Integration
${CYAN}Reference${NC}: GitHub Projects, GitLab Epics, Linear, Jira, OKR
INFO
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wia_roadmap_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

milestone() {
    local n=$1; [ -z "$n" ] && { printf '%s\n' "${RED}name required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_roadmap_version": "1.0.0",
  "type": "milestone",
  "milestone_id": "ms_${n}",
  "name": "${n}",
  "project_id": "did:wia:project:demo",
  "planned_date": "$(date -u -d '+90 days' +%F 2>/dev/null || date -u +%F)",
  "confidence_pct": 70,
  "scope_summary": "TODO",
  "owner": "did:wia:org:demo",
  "tags": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

dependency() {
    local ms=$1; [ -z "$ms" ] && { printf '%s\n' "${RED}milestone required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_roadmap_version": "1.0.0",
  "type": "dependency",
  "dependency_id": "dep_${ms}_$(date +%s)",
  "milestone_id": "${ms}",
  "depends_on": "ms_external_TODO",
  "criticality": "soft",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

release() {
    local ms=$1; [ -z "$ms" ] && { printf '%s\n' "${RED}milestone required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_roadmap_version": "1.0.0",
  "type": "release",
  "release_id": "rel_${ms}_$(date +%s)",
  "milestone_id": "${ms}",
  "version": "1.0.0",
  "released_at": "$(date -u +%FT%TZ)",
  "evidence_refs": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

progress() {
    local ms=$1; [ -z "$ms" ] && { printf '%s\n' "${RED}milestone required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_roadmap_version": "1.0.0",
  "type": "progress_update",
  "update_id": "pu_${ms}_$(date +%s)",
  "milestone_id": "${ms}",
  "captured_at": "$(date -u +%FT%TZ)",
  "completion_pct": 0,
  "blockers": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

commitment_change() {
    cat <<JSON
{
  "wia_roadmap_version": "1.0.0",
  "type": "commitment_change",
  "change_id": "cc_$(date +%s)",
  "milestone_id": "ms_TODO",
  "kind": "slip",
  "from_date": "TODO",
  "to_date": "TODO",
  "rationale": "TODO",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)            validate "$@" ;;
        milestone)           milestone "$@" ;;
        dependency)          dependency "$@" ;;
        release)             release "$@" ;;
        progress)            progress "$@" ;;
        commitment-change)   commitment_change ;;
        info)                show_info ;;
        help|-h|--help)      show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
