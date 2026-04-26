#!/usr/bin/env bash
# WIA-AGRI-025: Insect Protein Standard — POSIX shell client
#
# 弘益人間 (홍익인간) · Benefit All Humanity
#
# A minimal client demonstrating the PHASE-2 API contract.
# Designed for portability: depends only on `bash`, `curl`, and `jq`.
#
# Required environment:
#   WIA_API_BASE   — e.g. https://api.wia-standards.org/v1/insect-protein
#   WIA_API_KEY    — bearer token for authenticated calls
#
# Optional:
#   WIA_API_TIMEOUT (seconds, default 30)
#   WIA_API_DEBUG   (1 to echo curl invocations)
#
# Subcommands:
#   list-farms         [--country CC] [--species NAME]
#   get-farm           --farm-id FARM_ID
#   list-batches       [--species NAME] [--from ISO8601] [--to ISO8601]
#   get-batch          --batch-id BATCH_ID
#   submit-safety-test --batch-id BATCH_ID --file PATH/TO/REPORT.json
#   list-audit         [--since ISO8601] [--limit N]
#   whoami
#
# License: MIT

set -uo pipefail

API_BASE="${WIA_API_BASE:-https://api.wia-standards.org/v1/insect-protein}"
API_KEY="${WIA_API_KEY:-}"
TIMEOUT="${WIA_API_TIMEOUT:-30}"
DEBUG="${WIA_API_DEBUG:-0}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

require_jq() {
    if ! command -v jq >/dev/null 2>&1; then
        echo "error: jq is required (https://stedolan.github.io/jq/)" >&2
        exit 2
    fi
}

require_curl() {
    if ! command -v curl >/dev/null 2>&1; then
        echo "error: curl is required" >&2
        exit 2
    fi
}

require_key() {
    if [ -z "$API_KEY" ]; then
        echo "error: set WIA_API_KEY to authenticate" >&2
        exit 3
    fi
}

req() {
    # req METHOD PATH [JSON_BODY]
    local method="$1"
    local path="$2"
    local body="${3:-}"
    local url="${API_BASE%/}${path}"

    [ "$DEBUG" = "1" ] && echo "→ ${method} ${url}" >&2

    if [ -n "$body" ]; then
        curl -sS --max-time "$TIMEOUT" \
             -X "$method" \
             -H "Authorization: Bearer ${API_KEY}" \
             -H "Content-Type: application/json" \
             -H "Accept: application/json" \
             --data-binary "$body" \
             "$url"
    else
        curl -sS --max-time "$TIMEOUT" \
             -X "$method" \
             -H "Authorization: Bearer ${API_KEY}" \
             -H "Accept: application/json" \
             "$url"
    fi
}

usage() {
    grep -E '^#( |$)' "$0" | sed 's/^# \{0,1\}//'
    exit 2
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

cmd_list_farms() {
    local country="" species="" qs=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --country) country="$2"; shift 2 ;;
            --species) species="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -n "$country" ] && qs="${qs}country=$(printf '%s' "$country" | jq -sRr @uri)&"
    [ -n "$species" ] && qs="${qs}species=$(printf '%s' "$species" | jq -sRr @uri)&"
    require_key
    req GET "/farms${qs:+?${qs%&}}" | jq '.'
}

cmd_get_farm() {
    local farm_id=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --farm-id) farm_id="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -z "$farm_id" ] && { echo "error: --farm-id is required" >&2; exit 2; }
    require_key
    req GET "/farms/$(printf '%s' "$farm_id" | jq -sRr @uri)" | jq '.'
}

cmd_list_batches() {
    local species="" from="" to="" qs=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --species) species="$2"; shift 2 ;;
            --from)    from="$2"; shift 2 ;;
            --to)      to="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -n "$species" ] && qs="${qs}species=$(printf '%s' "$species" | jq -sRr @uri)&"
    [ -n "$from" ]    && qs="${qs}from=${from}&"
    [ -n "$to" ]      && qs="${qs}to=${to}&"
    require_key
    req GET "/batches${qs:+?${qs%&}}" | jq '.'
}

cmd_get_batch() {
    local batch_id=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --batch-id) batch_id="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -z "$batch_id" ] && { echo "error: --batch-id is required" >&2; exit 2; }
    require_key
    req GET "/batches/$(printf '%s' "$batch_id" | jq -sRr @uri)" | jq '.'
}

cmd_submit_safety_test() {
    local batch_id="" file=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --batch-id) batch_id="$2"; shift 2 ;;
            --file)     file="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -z "$batch_id" ] && { echo "error: --batch-id is required" >&2; exit 2; }
    [ -z "$file" ]     && { echo "error: --file is required" >&2; exit 2; }
    [ -f "$file" ]     || { echo "error: file not found: $file" >&2; exit 2; }

    # Validate that the file is well-formed JSON before submitting.
    if ! jq -e . "$file" >/dev/null 2>&1; then
        echo "error: $file is not valid JSON" >&2
        exit 4
    fi

    require_key
    local body
    body="$(cat "$file")"
    req POST "/batches/$(printf '%s' "$batch_id" | jq -sRr @uri)/safety-tests" "$body" | jq '.'
}

cmd_list_audit() {
    local since="" limit="" qs=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --since) since="$2"; shift 2 ;;
            --limit) limit="$2"; shift 2 ;;
            *) echo "unknown flag: $1" >&2; exit 2 ;;
        esac
    done
    [ -n "$since" ] && qs="${qs}since=${since}&"
    [ -n "$limit" ] && qs="${qs}limit=${limit}&"
    require_key
    req GET "/audit${qs:+?${qs%&}}" | jq '.'
}

cmd_whoami() {
    require_key
    req GET "/whoami" | jq '.'
}

# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------

require_curl
require_jq

[ $# -lt 1 ] && usage

sub="$1"; shift || true
case "$sub" in
    list-farms)         cmd_list_farms "$@" ;;
    get-farm)           cmd_get_farm "$@" ;;
    list-batches)       cmd_list_batches "$@" ;;
    get-batch)          cmd_get_batch "$@" ;;
    submit-safety-test) cmd_submit_safety_test "$@" ;;
    list-audit)         cmd_list_audit "$@" ;;
    whoami)             cmd_whoami ;;
    -h|--help|help)     usage ;;
    *) echo "unknown subcommand: $sub" >&2; usage ;;
esac
