#!/bin/bash
# WIA Sleep Monitoring CLI Tool
# Standard: WIA-MED-021 v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="sleep-monitoring"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/sleep-monitoring/"

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Sleep Monitoring CLI v${VERSION}${NC}
${BLUE}Open standard for sleep monitoring devices and clinical PSG.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>   Validate a record (JSON) per Phase 1
  session <patient>   Print a session envelope skeleton
  event <session>     Print a respiratory/arousal event envelope
  score <session>     Print a scoring metadata envelope
  consent <patient>   Print a patient consent envelope
  info                Show standard summary
  help                This help

Examples:
  $(basename "$0") validate ./session.json
  $(basename "$0") session did:wia:patient:01HXY
  $(basename "$0") event ses_01HXY
  $(basename "$0") score ses_01HXY

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA-MED-021 Sleep Monitoring v${VERSION}
${CYAN}Purpose${NC}: Open standard for sleep monitoring (clinical PSG + consumer wearables)
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Data format     — session, sleep stage, respiratory event, arousal, scoring
  2. API interface   — session ingest, query, stream, scoring re-computation
  3. Protocol        — federation, replay defence, patient consent
  4. Integration     — HL7 FHIR R5, AASM scoring, EDF / EDF+ raw recordings

Reference: AASM Manual current edition, EDF / EDF+, HL7 FHIR R5.
EOF
}

require_jq() {
    command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }
}

validate() {
    local f=$1
    [ -z "$f" ] && { printf '%s\n' "${RED}path required${NC}"; exit 2; }
    [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq

    for k in wia_sleep_monitoring_version type; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            printf '%s\n' "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_sleep_monitoring_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) printf '%s\n' "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    printf '%s\n' "${GREEN}OK — record structurally valid${NC}"
}

session() {
    local pid=$1
    [ -z "$pid" ] && { printf '%s\n' "${RED}patient-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "session",
  "session_id": "ses_$(date +%s)",
  "patient_id": "${pid}",
  "session_kind": "polysomnography",
  "started_at": "$(date -u +%FT%TZ)",
  "ended_at": "TODO",
  "device_ids": ["did:wia:device:psg-vendor-A"],
  "clinical_grade": true,
  "edf_url": "TODO (raw EDF+ recording reference)",
  "scoring_metadata_id": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

event() {
    local sid=$1
    [ -z "$sid" ] && { printf '%s\n' "${RED}session-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "respiratory_event",
  "session_id": "${sid}",
  "event_kind": "obstructive_apnoea",
  "started_at": "$(date -u +%FT%T.%3NZ)",
  "duration_seconds": 12,
  "associated_oxygen_desaturation_pct": 3.5,
  "associated_arousal": true,
  "scoring_algorithm": "AASM-2024-default",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

score() {
    local sid=$1
    [ -z "$sid" ] && { printf '%s\n' "${RED}session-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "scoring_metadata",
  "session_id": "${sid}",
  "scoring_algorithm": "AASM-2024-default",
  "scorer_id": "did:wia:scorer:tech-09",
  "scored_at": "$(date -u +%FT%TZ)",
  "summary": {
    "total_sleep_minutes": 0,
    "rem_minutes": 0,
    "n1_minutes": 0,
    "n2_minutes": 0,
    "n3_minutes": 0,
    "wake_minutes": 0,
    "ahi": 0.0,
    "rdi": 0.0,
    "min_spo2_pct": 0
  },
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

consent() {
    local pid=$1
    [ -z "$pid" ] && { printf '%s\n' "${RED}patient-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "patient_consent",
  "patient_id": "${pid}",
  "scopes": ["clinical_dashboard", "ehr_export", "research_de_identified"],
  "valid_from": "$(date -u +%FT%TZ)",
  "valid_until": "TODO (set explicit expiry)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate) validate "$@" ;;
        session)  session "$@" ;;
        event)    event "$@" ;;
        score)    score "$@" ;;
        consent)  consent "$@" ;;
        info)     show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
