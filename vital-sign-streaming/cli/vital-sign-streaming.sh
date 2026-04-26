#!/bin/bash
# WIA Vital Sign Streaming CLI Tool
# Standard: WIA Vital Sign Streaming v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="vital-sign-streaming"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/vital-sign-streaming/"

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Vital Sign Streaming CLI v${VERSION}${NC}
${BLUE}Real-time streaming of patient vital signs across devices and EHRs.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <frame>          Validate a vital-sign frame (JSON) per Phase 1
  frame <vital> <hz>        Print a sample frame for the given vital sign and rate
  subscribe <stream-id>     Print a Phase 2 subscribe payload (does not connect)
  alert <vital> <value>     Classify a single value into a clinical alert level
  fhir <frame>              Translate a frame into the HL7 FHIR Observation shape
  info                      Show standard summary
  help                      This help

Examples:
  $(basename "$0") validate ./frame.json
  $(basename "$0") frame ecg 250
  $(basename "$0") subscribe stream-001
  $(basename "$0") alert spo2 86
  $(basename "$0") fhir ./frame.json

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Vital Sign Streaming v${VERSION}
${CYAN}Purpose${NC}: Real-time streaming of patient vital signs
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Signal format       — Frame envelope, sample encoding, channel descriptors
  2. API interface       — Subscribe, replay, snapshot endpoints
  3. Protocol            — Federation, replay defence, encryption
  4. Integration         — HL7 FHIR R5, IEEE 11073, WIA-OMNI-API

Reference standards: IEEE 11073, HL7 FHIR R5, IETF RFC 8446 (TLS 1.3).
EOF
}

require_jq() {
    command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }
}

validate_frame() {
    local f=$1
    [ -z "$f" ] && { printf '%s\n' "${RED}path required${NC}"; exit 2; }
    [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq

    for k in wia_vital_sign_streaming_version stream_id patient_id captured_at samples; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            printf '%s\n' "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_vital_sign_streaming_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) printf '%s\n' "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    n=$(jq '.samples | length' "$f")
    if [ "$n" -lt 1 ]; then
        printf '%s\n' "${RED}samples array empty${NC}"; exit 1
    fi
    printf '%s\n' "${GREEN}OK — frame structurally valid (${n} sample(s))${NC}"
}

frame() {
    local vital=${1:-ecg}
    local hz=${2:-250}
    cat <<JSON
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "frame",
  "stream_id": "stream-001",
  "patient_id": "did:wia:patient:01HXY",
  "channel": "${vital}",
  "sample_rate_hz": ${hz},
  "captured_at": "$(date -u +%FT%TZ)",
  "samples": [0.012, 0.014, 0.013, 0.015, 0.017],
  "encoding": "float32",
  "calibration": { "lsb": 0.000244, "offset": 0.0, "unit": "mV" },
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

subscribe() {
    local sid=${1:-stream-001}
    cat <<JSON
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "subscribe",
  "stream_id": "${sid}",
  "subscriber_id": "did:wia:monitor:09",
  "transport": "websocket",
  "since": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

alert() {
    local vital=$1 value=$2
    [ -z "$vital" ] || [ -z "$value" ] && { printf '%s\n' "${RED}vital + value required${NC}"; exit 2; }
    case "$vital" in
        spo2)
            if   awk -v v=$value 'BEGIN{exit !(v < 90)}'; then echo "alert: critical (SpO2 < 90)";
            elif awk -v v=$value 'BEGIN{exit !(v < 94)}'; then echo "alert: warning (SpO2 < 94)";
            else echo "alert: normal"; fi ;;
        hr)
            if   awk -v v=$value 'BEGIN{exit !(v < 40 || v > 130)}'; then echo "alert: critical (HR out of 40-130)";
            elif awk -v v=$value 'BEGIN{exit !(v < 50 || v > 110)}'; then echo "alert: warning (HR out of 50-110)";
            else echo "alert: normal"; fi ;;
        sbp)
            if   awk -v v=$value 'BEGIN{exit !(v < 80 || v > 180)}'; then echo "alert: critical";
            elif awk -v v=$value 'BEGIN{exit !(v < 90 || v > 160)}'; then echo "alert: warning";
            else echo "alert: normal"; fi ;;
        *)
            echo "alert: unknown vital '${vital}' — pass-through"; exit 2 ;;
    esac
}

fhir() {
    require_jq
    local f=$1
    [ -f "$f" ] || { printf '%s\n' "${RED}frame not found${NC}"; exit 2; }
    local channel=$(jq -r '.channel' "$f")
    local patient=$(jq -r '.patient_id' "$f")
    local captured=$(jq -r '.captured_at' "$f")
    cat <<JSON
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{ "coding": [{ "system": "http://terminology.hl7.org/CodeSystem/observation-category", "code": "vital-signs" }] }],
  "code": { "text": "${channel}" },
  "subject": { "reference": "Patient/${patient}" },
  "effectiveDateTime": "${captured}",
  "valueQuantity": { "value": 0, "unit": "see frame.samples[]" },
  "derivedFrom": [{ "reference": "DocumentReference/wia-vital-sign-streaming/$(basename "$f")" }]
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)  validate_frame "$@" ;;
        frame)     frame "$@" ;;
        subscribe) subscribe "$@" ;;
        alert)     alert "$@" ;;
        fhir)      fhir "$@" ;;
        info)      show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
