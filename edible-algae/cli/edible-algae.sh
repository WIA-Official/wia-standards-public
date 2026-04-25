#!/usr/bin/env bash
# WIA Edible Algae CLI (WIA-AGRI-033)
# Spirulina, chlorella, seaweed, and microalgae cultivation management.
#
# Subcommands:
#   list <resource>          List records of a resource type.
#   get <resource> <id>      Fetch a single record by id.
#   create <resource> <file> Create a new record from a JSON file.
#   validate <file>          Validate a JSON document against the WIA-AGRI-033 schema.
#   export <resource> <fmt>  Export data (json|csv).
#
# Resources: cultivars | batches | harvests | nutrient-profiles | facilities
#
# Authentication:
#   Set WIA_EDIBLE_ALGAE_API_KEY in your environment.
#   Override base URL via WIA_EDIBLE_ALGAE_URL (default: https://api.wia.live/edible-algae/v1).

set -euo pipefail

BASE_URL="${WIA_EDIBLE_ALGAE_URL:-https://api.wia.live/edible-algae/v1}"
API_KEY="${WIA_EDIBLE_ALGAE_API_KEY:-}"

usage() {
    cat <<EOF
WIA Edible Algae CLI — WIA-AGRI-033

Usage:
  edible-algae list <resource> [--limit N] [--offset N]
  edible-algae get <resource> <id>
  edible-algae create <resource> <file.json>
  edible-algae validate <file.json>
  edible-algae export <resource> <json|csv>

Resources:
  cultivars            Algae strain catalog (Spirulina, Chlorella, etc.)
  batches              Cultivation batch records
  harvests             Harvest yield and quality records
  nutrient-profiles    Nutritional composition entries
  facilities           Cultivation facility registrations

Examples:
  edible-algae list cultivars --limit 50
  edible-algae get batches batch-2026-04
  edible-algae create harvests harvest.json
  edible-algae validate batch.json
  edible-algae export harvests csv > harvests.csv

Authentication:
  export WIA_EDIBLE_ALGAE_API_KEY=...
  export WIA_EDIBLE_ALGAE_URL=https://api.wia.live/edible-algae/v1

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
EOF
}

require_jq() {
    command -v jq >/dev/null 2>&1 || { echo "Error: jq is required (https://jqlang.org)"; exit 2; }
}

curl_auth() {
    local args=("-fsS" "-H" "Accept: application/json")
    [ -n "$API_KEY" ] && args+=("-H" "X-API-Key: $API_KEY")
    curl "${args[@]}" "$@"
}

cmd_list() {
    local resource="${1:-}"; shift || true
    [ -z "$resource" ] && { usage; exit 2; }
    local limit=20 offset=0
    while [ $# -gt 0 ]; do
        case "$1" in
            --limit) limit="$2"; shift 2 ;;
            --offset) offset="$2"; shift 2 ;;
            *) echo "Unknown option: $1"; exit 2 ;;
        esac
    done
    curl_auth "$BASE_URL/$resource?limit=$limit&offset=$offset"
}

cmd_get() {
    local resource="${1:-}" id="${2:-}"
    [ -z "$resource" ] || [ -z "$id" ] && { usage; exit 2; }
    curl_auth "$BASE_URL/$resource/$id"
}

cmd_create() {
    local resource="${1:-}" file="${2:-}"
    [ -z "$resource" ] || [ -z "$file" ] && { usage; exit 2; }
    [ -f "$file" ] || { echo "Not found: $file"; exit 2; }
    curl_auth -X POST -H "Content-Type: application/json" --data-binary "@$file" "$BASE_URL/$resource"
}

cmd_validate() {
    local file="${1:-}"
    [ -z "$file" ] && { usage; exit 2; }
    [ -f "$file" ] || { echo "Not found: $file"; exit 2; }
    require_jq
    if jq -e . "$file" >/dev/null 2>&1; then
        echo "✅ Valid JSON: $file"
    else
        echo "❌ Invalid JSON: $file"; exit 1
    fi
}

cmd_export() {
    local resource="${1:-}" fmt="${2:-json}"
    [ -z "$resource" ] && { usage; exit 2; }
    case "$fmt" in
        json) curl_auth "$BASE_URL/$resource?limit=1000" ;;
        csv)
            require_jq
            curl_auth "$BASE_URL/$resource?limit=1000" \
                | jq -r '.items[] | [.id, .name // "", .status // ""] | @csv'
            ;;
        *) echo "Unknown format: $fmt (json|csv)"; exit 2 ;;
    esac
}

case "${1:-}" in
    list)     shift; cmd_list "$@" ;;
    get)      shift; cmd_get "$@" ;;
    create)   shift; cmd_create "$@" ;;
    validate) shift; cmd_validate "$@" ;;
    export)   shift; cmd_export "$@" ;;
    -h|--help|help|"") usage ;;
    *) echo "Unknown command: $1"; usage; exit 2 ;;
esac
