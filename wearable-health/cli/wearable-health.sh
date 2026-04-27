#!/bin/bash
# WIA Wearable Health CLI Tool
# Standard: WIA-MED-020 v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="wearable-health"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/wearable-health/"

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Wearable Health CLI v${VERSION}${NC}
${BLUE}Open standard for wearable health monitoring devices.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>   Validate a record (JSON) per Phase 1
  device <id>         Print a device envelope skeleton
  measurement <type>  Print a measurement envelope skeleton
                      (heart_rate / step_count / sleep / ecg / glucose / spo2 / bp)
  alert <patient-id>  Print an alert envelope skeleton
  consent <patient-id> Print a patient consent envelope skeleton
  info                Show standard summary
  help                This help

Examples:
  $(basename "$0") validate ./measurement.json
  $(basename "$0") device dev-apple-watch-001
  $(basename "$0") measurement heart_rate
  $(basename "$0") alert did:wia:patient:01HXY

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA-MED-020 Wearable Health v${VERSION}
${CYAN}Purpose${NC}: Open standard for wearable health monitoring
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Data format     — device, measurement, session, alert, calibration
  2. API interface   — sync, stream, query, alert
  3. Protocol        — federation, replay defence, patient consent
  4. Integration     — HL7 FHIR R5, IEEE 11073, HealthKit, Google Fit

Reference: HL7 FHIR R5, IEEE 11073-104xx series, FDA 510(k) for clinical-grade.
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

    for k in wia_wearable_health_version type; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            printf '%s\n' "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_wearable_health_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) printf '%s\n' "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    printf '%s\n' "${GREEN}OK — record structurally valid${NC}"
}

device() {
    local id=$1
    [ -z "$id" ] && { printf '%s\n' "${RED}device-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_wearable_health_version": "1.0.0",
  "type": "device",
  "device_id": "${id}",
  "patient_id": "did:wia:patient:01HXY",
  "manufacturer": "TODO",
  "model": "TODO",
  "firmware_version": "TODO",
  "clinical_grade": false,
  "supported_measurements": ["heart_rate","step_count"],
  "registered_at": "$(date -u +%FT%TZ)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

measurement() {
    local mtype=$1
    [ -z "$mtype" ] && { printf '%s\n' "${RED}type required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_wearable_health_version": "1.0.0",
  "type": "measurement",
  "measurement_type": "${mtype}",
  "device_id": "dev-001",
  "patient_id": "did:wia:patient:01HXY",
  "captured_at": "$(date -u +%FT%TZ)",
  "value": 0,
  "unit": "TODO",
  "clinical_grade": false,
  "calibration_chain_id": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

alert() {
    local pid=$1
    [ -z "$pid" ] && { printf '%s\n' "${RED}patient-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_wearable_health_version": "1.0.0",
  "type": "alert",
  "patient_id": "${pid}",
  "alert_id": "alt_$(date +%s)",
  "captured_at": "$(date -u +%FT%TZ)",
  "severity": "warning",
  "rule": "heart_rate above 130 bpm at rest",
  "actions": ["notify_patient", "log_for_clinician"],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

consent() {
    local pid=$1
    [ -z "$pid" ] && { printf '%s\n' "${RED}patient-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_wearable_health_version": "1.0.0",
  "type": "patient_consent",
  "patient_id": "${pid}",
  "scopes": ["clinician_dashboard", "ehr_export", "research"],
  "valid_from": "$(date -u +%FT%TZ)",
  "valid_until": "TODO (set explicit expiry)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)    validate "$@" ;;
        device)      device "$@" ;;
        measurement) measurement "$@" ;;
        alert)       alert "$@" ;;
        consent)     consent "$@" ;;
        info)        show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
