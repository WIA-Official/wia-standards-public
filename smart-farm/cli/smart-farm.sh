#!/bin/bash
# WIA Smart Farm CLI
# Standard: WIA Smart Farm v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/smart-farm/"
RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; CYAN=$'\033[0;36m'; NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Smart Farm CLI v${VERSION}${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>    Validate a record (JSON) per Phase 1
  field <farm>         Print a field/parcel envelope skeleton
  sensor <field>       Print a sensor reading envelope skeleton
  irrigation <field>   Print an irrigation event envelope skeleton
  harvest <field>      Print a harvest record envelope skeleton
  info                 Show standard summary
  help                 This help

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Smart Farm v${VERSION}
${CYAN}Phases${NC}: 1 Data Format · 2 API · 3 Protocol · 4 Integration
${CYAN}Reference${NC}: ISO 19156 (O&M), AgGateway ADAPT, ISOBUS, FAO AgroVoc
EOF
}

require_jq() { command -v jq >/dev/null 2>&1 || { printf '%s\n' "${RED}jq required${NC}"; exit 2; }; }

validate() {
    local f=$1; [ -f "$f" ] || { printf '%s\n' "${RED}not found: $f${NC}"; exit 2; }
    require_jq
    for k in wia_smart_farm_version type; do
        jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1 || { printf '%s\n' "${RED}missing: ${k}${NC}"; exit 1; }
    done
    printf '%s\n' "${GREEN}OK${NC}"
}

field() {
    local farm=$1; [ -z "$farm" ] && { printf '%s\n' "${RED}farm required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_farm_version": "1.0.0",
  "type": "field",
  "field_id": "fld_${farm}_001",
  "farm_id": "${farm}",
  "boundary": { "type": "Polygon", "coordinates": [[]] },
  "area_hectares": 0,
  "current_crop": "TODO (FAO AgroVoc concept)",
  "soil_class": "TODO (USDA Soil Taxonomy)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

sensor() {
    local fld=$1; [ -z "$fld" ] && { printf '%s\n' "${RED}field required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_farm_version": "1.0.0",
  "type": "sensor_reading",
  "field_id": "${fld}",
  "sensor_id": "did:wia:sensor:soil-moisture-001",
  "captured_at": "$(date -u +%FT%TZ)",
  "measurement_kind": "soil_moisture_volumetric_pct",
  "value": 0,
  "unit": "%",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

irrigation() {
    local fld=$1; [ -z "$fld" ] && { printf '%s\n' "${RED}field required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_farm_version": "1.0.0",
  "type": "irrigation_event",
  "field_id": "${fld}",
  "started_at": "$(date -u +%FT%TZ)",
  "ended_at": "TODO",
  "method": "drip",
  "volume_m3": 0,
  "source": "well",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

harvest() {
    local fld=$1; [ -z "$fld" ] && { printf '%s\n' "${RED}field required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_smart_farm_version": "1.0.0",
  "type": "harvest_record",
  "field_id": "${fld}",
  "harvested_at": "$(date -u +%FT%TZ)",
  "crop": "TODO (FAO AgroVoc concept)",
  "yield_t_per_ha": 0,
  "moisture_pct": 0,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate)   validate "$@" ;;
        field)      field "$@" ;;
        sensor)     sensor "$@" ;;
        irrigation) irrigation "$@" ;;
        harvest)    harvest "$@" ;;
        info)       show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}
main "$@"
