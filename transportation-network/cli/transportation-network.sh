#!/bin/bash
# WIA Transportation Network CLI
# Standard: WIA-UNI-008 v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/transportation-network/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Transportation Network CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>    Validate a record (JSON) per Phase 1
  route <agency>       Print a route definition envelope
  schedule <route>     Print a schedule envelope
  vehicle <agency>     Print a vehicle telemetry envelope
  trip <route>         Print a trip update envelope
  info                 Show standard summary
  help                 This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Transportation Network v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Protocol · 4 Integration
${CYAN}Reference${NC}: GTFS / GTFS-Realtime, NeTEx, SIRI, ISO 17572, OpenLR
EOF
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wia_transportation_network_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

route() {
    local agency=$1; [ -z "$agency" ] && { printf '%s\n' "${RED}agency required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_transportation_network_version": "1.0.0",
  "type": "route",
  "route_id": "rt_${agency}_001",
  "agency_id": "${agency}",
  "short_name": "TODO",
  "long_name": "TODO",
  "mode": "bus",
  "color": "#1f77b4",
  "stops": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

schedule() {
    local rt=$1; [ -z "$rt" ] && { printf '%s\n' "${RED}route required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_transportation_network_version": "1.0.0",
  "type": "schedule",
  "schedule_id": "sch_${rt}_$(date +%Y%m%d)",
  "route_id": "${rt}",
  "service_date": "$(date -u +%F)",
  "trips": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

vehicle() {
    local agency=$1; [ -z "$agency" ] && { printf '%s\n' "${RED}agency required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_transportation_network_version": "1.0.0",
  "type": "vehicle_position",
  "agency_id": "${agency}",
  "vehicle_id": "veh_001",
  "captured_at": "$(date -u +%FT%TZ)",
  "position": { "lat": 0, "lon": 0, "bearing_deg": 0, "speed_m_per_s": 0 },
  "trip_id": null,
  "occupancy_status": "many_seats_available",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

trip() {
    local rt=$1; [ -z "$rt" ] && { printf '%s\n' "${RED}route required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_transportation_network_version": "1.0.0",
  "type": "trip_update",
  "trip_id": "trip_${rt}_$(date +%s)",
  "route_id": "${rt}",
  "captured_at": "$(date -u +%FT%TZ)",
  "schedule_relationship": "scheduled",
  "stop_time_updates": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate) validate "$@" ;;
        route)    route "$@" ;;
        schedule) schedule "$@" ;;
        vehicle)  vehicle "$@" ;;
        trip)     trip "$@" ;;
        info)     show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
