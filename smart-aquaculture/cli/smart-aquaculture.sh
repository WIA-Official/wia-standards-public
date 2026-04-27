#!/bin/bash
# WIA Smart Aquaculture CLI
# Standard: WIA Smart Aquaculture v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/smart-aquaculture/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Smart Aquaculture CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>    Validate a record (JSON) per Phase 1
  farm <id>            Print a farm definition envelope
  water-quality <farm> Print a water quality reading envelope
  fish-health <farm>   Print a fish health observation envelope
  feeding <farm>       Print a feeding event envelope
  harvest <farm>       Print a harvest record envelope
  info                 Show standard summary
  help                 This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Smart Aquaculture v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Protocol · 4 Integration
${CYAN}Reference${NC}: FAO ALDFI, ASC, BAP, Global GAP
EOF
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wia_smart_aquaculture_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

farm() {
    local id=$1; [ -z "$id" ] && { printf '%s\n' "${RED}id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_aquaculture_version": "1.0.0",
  "type": "farm",
  "farm_id": "${id}",
  "operator_id": "did:wia:operator:demo",
  "boundary": { "type": "Polygon", "coordinates": [[]] },
  "system_kind": "marine_cage",
  "primary_species": "salmo_salar",
  "certifications": [],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

water_quality() {
    local farm=$1; [ -z "$farm" ] && { printf '%s\n' "${RED}farm required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_aquaculture_version": "1.0.0",
  "type": "water_quality_reading",
  "farm_id": "${farm}",
  "captured_at": "$(date -u +%FT%TZ)",
  "temperature_c": 0,
  "salinity_ppt": 0,
  "dissolved_oxygen_mg_per_l": 0,
  "ph": 0,
  "ammonia_mg_per_l": 0,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

fish_health() {
    local farm=$1; [ -z "$farm" ] && { printf '%s\n' "${RED}farm required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_aquaculture_version": "1.0.0",
  "type": "fish_health_observation",
  "farm_id": "${farm}",
  "observed_at": "$(date -u +%FT%TZ)",
  "cohort_id": "coh_001",
  "average_weight_g": 0,
  "mortality_count": 0,
  "disease_indicator": "none",
  "vaccination_record_id": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

feeding() {
    local farm=$1; [ -z "$farm" ] && { printf '%s\n' "${RED}farm required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_aquaculture_version": "1.0.0",
  "type": "feeding_event",
  "farm_id": "${farm}",
  "fed_at": "$(date -u +%FT%TZ)",
  "feed_type": "extruded_pellet",
  "feed_kg": 0,
  "cohort_id": "coh_001",
  "feed_conversion_ratio": 0,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

harvest() {
    local farm=$1; [ -z "$farm" ] && { printf '%s\n' "${RED}farm required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_aquaculture_version": "1.0.0",
  "type": "harvest_record",
  "farm_id": "${farm}",
  "harvested_at": "$(date -u +%FT%TZ)",
  "cohort_id": "coh_001",
  "species": "salmo_salar",
  "total_weight_kg": 0,
  "average_weight_g": 0,
  "destination": "processor_id_001",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)      validate "$@" ;;
        farm)          farm "$@" ;;
        water-quality) water_quality "$@" ;;
        fish-health)   fish_health "$@" ;;
        feeding)       feeding "$@" ;;
        harvest)       harvest "$@" ;;
        info)          show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
