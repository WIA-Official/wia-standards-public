#!/usr/bin/env bash
# industrial-robot.sh — minimal POSIX shell client for the WIA industrial-robot standard.
# Demonstrates the request/response contract documented under spec/.
# Requires: curl, jq.
#
# Environment variables:
#   WIA_API_BASE   default https://industrial-robot.example.com/api/v1
#   WIA_API_TOKEN  bearer token (required for write operations)

set -euo pipefail

API_BASE="${WIA_API_BASE:-https://industrial-robot.example.com/api/v1}"
TOKEN="${WIA_API_TOKEN:-}"

usage() {
    cat <<USAGE
Usage: $0 <subcommand> [args]

Subcommands:
  list                       list resources
  get <id>                   read a single resource
  create <json-file>         POST a JSON payload
  validate <json-file>       schema-validate locally (jq only)
  export <id>                export a resource as JSON (pretty-printed)

Environment:
  WIA_API_BASE   = $API_BASE
  WIA_API_TOKEN  = (set: $([ -n "$TOKEN" ] && echo yes || echo no))
USAGE
}

require_token() {
    [ -n "$TOKEN" ] || { echo "WIA_API_TOKEN is required for this operation." >&2; exit 2; }
}

cmd="${1:-}"
shift || true

case "$cmd" in
    list)
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/resources" | jq .
        ;;
    get)
        id="${1:?missing id}"
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/resources/$id" | jq .
        ;;
    create)
        require_token
        f="${1:?missing json file}"
        curl -fsS -X POST -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" \
            --data-binary "@$f" "$API_BASE/resources" | jq .
        ;;
    validate)
        f="${1:?missing json file}"
        jq empty "$f" && echo "ok: $f is valid JSON"
        ;;
    export)
        id="${1:?missing id}"
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/resources/$id" | jq -S .
        ;;
    *)
        usage
        exit 2
        ;;
esac
