#!/bin/bash
# WIA Time-Series Data CLI Tool
# Standard: WIA Time-Series Data v1.0.0
# Philosophy: 弘益人間 — Benefit All Humanity

set -e

STANDARD_NAME="time-series-data"
VERSION="1.0.0"
SPEC_URL="https://wiastandards.com/time-series-data/"

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

show_help() {
    cat <<EOF
${CYAN}WIA Time-Series Data CLI v${VERSION}${NC}
${BLUE}Open standard for time-series data exchange.${NC}

Usage: $(basename "$0") <command> [options]

Commands:
  validate <record>    Validate a time-series record (JSON) per Phase 1
  series <name>        Print a series definition envelope skeleton
  point <series>       Print a single data point envelope
  query <series>       Print a query envelope skeleton
  rollup <series>      Print a rollup / downsample request skeleton
  info                 Show standard summary
  help                 This help

Examples:
  $(basename "$0") validate ./series.json
  $(basename "$0") series temperature_kitchen_sensor_A
  $(basename "$0") point temperature_kitchen_sensor_A
  $(basename "$0") query temperature_kitchen_sensor_A

Reference: ${SPEC_URL}
弘益人間 — Benefit All Humanity
EOF
}

show_info() {
    cat <<EOF
${CYAN}Standard${NC}: WIA Time-Series Data v${VERSION}
${CYAN}Purpose${NC}: Open standard for time-series data exchange
${CYAN}Spec${NC}: ${SPEC_URL}
${CYAN}Phases${NC}:
  1. Data format     — series, point, label set, query, rollup envelopes
  2. API interface   — write, query, stream, rollup endpoints
  3. Protocol        — federation, replay defence, retention policy
  4. Integration     — OpenMetrics, Prometheus, InfluxDB, OpenTelemetry

Reference: OpenMetrics 1.0, Prometheus exposition format, IETF RFC 8259.
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

    for k in wia_time_series_data_version type; do
        if ! jq -e --arg k "$k" 'has($k)' "$f" >/dev/null 2>&1; then
            printf '%s\n' "${RED}MISSING required key: ${k}${NC}"; exit 1
        fi
    done

    ver=$(jq -r '.wia_time_series_data_version' "$f")
    case "$ver" in
        1.0|1.0.*) ;;
        *) printf '%s\n' "${RED}unsupported version: ${ver}${NC}"; exit 1 ;;
    esac

    printf '%s\n' "${GREEN}OK — record structurally valid${NC}"
}

series() {
    local name=$1
    [ -z "$name" ] && { printf '%s\n' "${RED}name required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_time_series_data_version": "1.0.0",
  "type": "series",
  "series_id": "ts_${name}",
  "name": "${name}",
  "unit": "TODO (UCUM unit)",
  "labels": { "env": "production", "region": "us-east-1" },
  "retention_seconds": 2592000,
  "downsampling_policy": [
    { "from_resolution_seconds": 1, "after_seconds": 3600,  "to_resolution_seconds": 60 },
    { "from_resolution_seconds": 60, "after_seconds": 604800, "to_resolution_seconds": 3600 }
  ],
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

point() {
    local s=$1
    [ -z "$s" ] && { printf '%s\n' "${RED}series-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_time_series_data_version": "1.0.0",
  "type": "point",
  "series_id": "ts_${s}",
  "captured_at": "$(date -u +%FT%T.%3NZ)",
  "value": 0,
  "value_type": "double",
  "labels": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

query() {
    local s=$1
    [ -z "$s" ] && { printf '%s\n' "${RED}series-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_time_series_data_version": "1.0.0",
  "type": "query",
  "series_id": "ts_${s}",
  "from": "$(date -u -d '1 hour ago' +%FT%T.%3NZ 2>/dev/null || date -u +%FT%T.%3NZ)",
  "to":   "$(date -u +%FT%T.%3NZ)",
  "aggregation": "avg",
  "resolution_seconds": 60,
  "label_filters": null,
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

rollup() {
    local s=$1
    [ -z "$s" ] && { printf '%s\n' "${RED}series-id required${NC}"; exit 2; }
    cat <<JSON
{
  "wia_time_series_data_version": "1.0.0",
  "type": "rollup_request",
  "series_id": "ts_${s}",
  "source_resolution_seconds": 1,
  "target_resolution_seconds": 60,
  "aggregations": ["avg", "min", "max", "count"],
  "from": "$(date -u -d '1 day ago' +%FT%TZ 2>/dev/null || date -u +%FT%TZ)",
  "to":   "$(date -u +%FT%TZ)",
  "signature": { "alg": "Ed25519", "value": "TODO" }
}
JSON
}

main() {
    local cmd=${1:-help}; shift || true
    case "$cmd" in
        validate) validate "$@" ;;
        series)   series "$@" ;;
        point)    point "$@" ;;
        query)    query "$@" ;;
        rollup)   rollup "$@" ;;
        info)     show_info ;;
        help|-h|--help) show_help ;;
        *) printf '%s\n' "${RED}unknown command: ${cmd}${NC}"; show_help; exit 2 ;;
    esac
}

main "$@"
