#!/bin/bash
# WIA Smart Wheelchair CLI Tool
# Standard: WIA Smart Wheelchair v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="smart-wheelchair"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/smart-wheelchair/"

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Smart Wheelchair CLI v${VERSION}${NC}
${BLUE}Power and assistive wheelchair telemetry, navigation, accessibility.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>          Validate a wheelchair record (JSON) per Phase 1
  telemetry <wheelchair-id>  Print a telemetry frame skeleton
  route <wheelchair-id>      Print a route_goal envelope skeleton
  safety <wheelchair-id>     Print a safety_event envelope skeleton
  consent <user-id>          Print a user consent envelope skeleton
  info                       Show standard summary
  help                       This help

Examples:
  $(basename "$0") validate ./telemetry.json
  $(basename "$0") telemetry wheelchair-001
  $(basename "$0") route wheelchair-001
  $(basename "$0") safety wheelchair-001

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Smart Wheelchair v${VERSION}
${CYAN}Purpose${NC}: Power / assistive wheelchair interoperability
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Data format     — telemetry, route, navigation, user, safety
  2. API interface   — telemetry stream, route dispatch, safety event
  3. Protocol        — federation, replay defence, user consent
  4. Integration     — ROS 2, CANopen (CiA 301), WIA-OMNI-API, WIA-ACCESSIBILITY

Reference: ROS 2 Iron / Jazzy, CiA 301 v4.2.0, IETF RFC 8446 (TLS 1.3).
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

    for k in wia_smart_wheelchair_version type captured_at; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            printf '%s\n' "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_smart_wheelchair_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) printf '%s\n' "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    printf '%s\n' "${GREEN}OK — record structurally valid${NC}"
}

telemetry() {
    local id=$1
    [ -z "$id" ] && { printf '%s\n' "${RED}wheelchair-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_wheelchair_version": "1.0.0",
  "type": "telemetry",
  "wheelchair_id": "${id}",
  "user_id": "did:wia:user:01HXY",
  "captured_at": "$(date -u +%FT%TZ)",
  "pose": { "x": 0.0, "y": 0.0, "yaw_rad": 0.0, "frame": "map" },
  "speed_m_per_s": 0.0,
  "battery_pct": 78,
  "motor_temp_c": 38.4,
  "tilt_deg": 0.5,
  "obstacles": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

route() {
    local id=$1
    [ -z "$id" ] && { printf '%s\n' "${RED}wheelchair-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_wheelchair_version": "1.0.0",
  "type": "route_goal",
  "wheelchair_id": "${id}",
  "issued_at": "$(date -u +%FT%TZ)",
  "goal": { "x": 3.5, "y": -1.2, "yaw_rad": 0.0, "frame": "map" },
  "max_speed_m_per_s": 1.2,
  "comfort_profile_id": "comfort-default",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

safety() {
    local id=$1
    [ -z "$id" ] && { printf '%s\n' "${RED}wheelchair-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_wheelchair_version": "1.0.0",
  "type": "safety_event",
  "wheelchair_id": "${id}",
  "captured_at": "$(date -u +%FT%TZ)",
  "severity": "warning",
  "category": "obstacle_avoidance",
  "description": "Unexpected obstacle within 0.4 m, emergency stop triggered.",
  "auto_actions_taken": ["emergency_stop", "alert_user"],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

consent() {
    local uid=$1
    [ -z "$uid" ] && { printf '%s\n' "${RED}user-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_wheelchair_version": "1.0.0",
  "type": "user_consent",
  "user_id": "${uid}",
  "scopes": ["telemetry_share_caregiver", "route_log_retention", "safety_event_share_clinician"],
  "valid_from": "$(date -u +%FT%TZ)",
  "valid_until": "TODO (set explicit expiry)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)  validate "$@" ;;
        telemetry) telemetry "$@" ;;
        route)     route "$@" ;;
        safety)    safety "$@" ;;
        consent)   consent "$@" ;;
        info)      show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
