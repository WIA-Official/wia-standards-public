#!/bin/bash
# WIA Telemedicine CLI
# Standard: WIA-MED-004 v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/telemedicine/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<HELP
${CYAN}WIA Telemedicine CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>      Validate a record (JSON) per Phase 1
  consult <patient>      Print a consultation_session envelope
  encounter <patient>    Print a clinical_encounter envelope
  prescription <encounter>  Print an e_prescription envelope
  schedule <provider>    Print an appointment_slot envelope
  consent <patient>      Print a patient_consent envelope
  info                   Show standard summary
  help                   This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
HELP
}

show_info() {
    cat <<INFO
${CYAN}Standard${NC}: WIA Telemedicine v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Federation · 4 Integration
${CYAN}Reference${NC}: HL7 FHIR R5, IHE ITI, ICD-11, SNOMED CT, RxNorm
INFO
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wia_telemedicine_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

consult() {
    local p=$1; [ -z "$p" ] && { printf '%s\n' "${RED}patient required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_telemedicine_version": "1.0.0",
  "type": "consultation_session",
  "session_id": "sess_${p}_$(date +%s)",
  "patient_id": "${p}",
  "provider_id": "did:wia:provider:demo",
  "modality": "video",
  "scheduled_start": "$(date -u +%FT%TZ)",
  "specialty": "general",
  "patient_consent_ref": "did:wia:consent:TODO",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

encounter() {
    local p=$1; [ -z "$p" ] && { printf '%s\n' "${RED}patient required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_telemedicine_version": "1.0.0",
  "type": "clinical_encounter",
  "encounter_id": "enc_${p}_$(date +%s)",
  "patient_id": "${p}",
  "session_id": "sess_TODO",
  "started_at": "$(date -u +%FT%TZ)",
  "icd11_codes": [],
  "snomed_findings": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

prescription() {
    local e=$1; [ -z "$e" ] && { printf '%s\n' "${RED}encounter required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_telemedicine_version": "1.0.0",
  "type": "e_prescription",
  "rx_id": "rx_${e}_$(date +%s)",
  "encounter_id": "${e}",
  "rxnorm_code": "TODO",
  "dose": "TODO",
  "frequency": "TODO",
  "duration_days": 0,
  "prescriber_id": "did:wia:provider:demo",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

schedule() {
    local pr=$1; [ -z "$pr" ] && { printf '%s\n' "${RED}provider required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_telemedicine_version": "1.0.0",
  "type": "appointment_slot",
  "slot_id": "slot_${pr}_$(date +%s)",
  "provider_id": "${pr}",
  "available_from": "$(date -u +%FT%TZ)",
  "duration_min": 30,
  "modality": "video",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

consent() {
    local p=$1; [ -z "$p" ] && { printf '%s\n' "${RED}patient required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_telemedicine_version": "1.0.0",
  "type": "patient_consent",
  "consent_id": "cons_${p}_$(date +%s)",
  "patient_id": "${p}",
  "scope": "telemedicine_consultation",
  "granted_at": "$(date -u +%FT%TZ)",
  "valid_until": "TODO",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)        validate "$@" ;;
        consult)         consult "$@" ;;
        encounter)       encounter "$@" ;;
        prescription)    prescription "$@" ;;
        schedule)        schedule "$@" ;;
        consent)         consent "$@" ;;
        info)            show_info ;;
        help|-h|--help)  show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
