#!/usr/bin/env bash
# WIA Digital ID CLI
# Decentralized identifier (DID) issuance, verification, and revocation.
#
# Subcommands:
#   list <resource>          List records.
#   get <resource> <id>      Fetch record by id.
#   create <resource> <file> Submit JSON record.
#   verify <jwt>             Verify a Verifiable Credential JWT.
#   revoke <credentialId>    Revoke a credential.
#
# Resources: identities | credentials | revocations | resolvers
#
# Auth: WIA_DIGITAL_ID_API_KEY env var.

set -euo pipefail

BASE_URL="${WIA_DIGITAL_ID_URL:-https://api.wia.live/digital-id/v1}"
API_KEY="${WIA_DIGITAL_ID_API_KEY:-}"

usage() {
    cat <<EOF
WIA Digital ID CLI — Decentralized identity / Verifiable Credentials

Usage:
  digital-id list <resource> [--limit N] [--offset N]
  digital-id get <resource> <id>
  digital-id create <resource> <file.json>
  digital-id verify <jwt>
  digital-id revoke <credentialId>

Resources:
  identities    DIDs and identity profiles
  credentials   Verifiable Credentials
  revocations   Revocation list entries
  resolvers     DID method resolver registrations

Examples:
  digital-id list credentials --limit 100
  digital-id verify eyJhbGciOiJFZERTQSIsImtpZCI6...
  digital-id revoke vc-2026-04-25-001

Auth:
  export WIA_DIGITAL_ID_API_KEY=...
  export WIA_DIGITAL_ID_URL=https://api.wia.live/digital-id/v1

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
EOF
}

require_jq() { command -v jq >/dev/null 2>&1 || { echo "Error: jq is required"; exit 2; }; }

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

cmd_verify() {
    local jwt="${1:-}"
    [ -z "$jwt" ] && { usage; exit 2; }
    curl_auth -X POST -H "Content-Type: application/json" --data-binary "{\"jwt\":\"$jwt\"}" "$BASE_URL/verify"
}

cmd_revoke() {
    local id="${1:-}"
    [ -z "$id" ] && { usage; exit 2; }
    curl_auth -X POST "$BASE_URL/revocations/$id"
}

case "${1:-}" in
    list)    shift; cmd_list "$@" ;;
    get)     shift; cmd_get "$@" ;;
    create)  shift; cmd_create "$@" ;;
    verify)  shift; cmd_verify "$@" ;;
    revoke)  shift; cmd_revoke "$@" ;;
    -h|--help|help|"") usage ;;
    *) echo "Unknown command: $1"; usage; exit 2 ;;
esac
