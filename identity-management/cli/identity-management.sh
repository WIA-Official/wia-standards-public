#!/usr/bin/env bash
# identity-management.sh — minimal POSIX shell client for the WIA Identity
# Management standard. Demonstrates the request/response contract documented
# under spec/PHASE-2-DATA.md. Requires: curl, jq.
#
# Environment:
#   WIA_API_BASE   default https://identity.example.com/api/v1
#   WIA_API_TOKEN  bearer token (required for write operations)

set -euo pipefail

API_BASE="${WIA_API_BASE:-https://identity.example.com/api/v1}"
TOKEN="${WIA_API_TOKEN:-}"

usage() {
    cat <<USAGE
Usage: $0 <subcommand> [args]

Subcommands:
  list-subjects
  get-subject <id>
  list-audit
  whoami

Environment:
  WIA_API_BASE  = $API_BASE
  WIA_API_TOKEN = (set: $([ -n "$TOKEN" ] && echo yes || echo no))
USAGE
}

cmd="${1:-}"
shift || true

case "$cmd" in
    list-subjects)
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/subjects" | jq .
        ;;
    get-subject)
        id="${1:?missing id}"
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/subjects/$id" | jq .
        ;;
    list-audit)
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/audit" | jq .
        ;;
    whoami)
        curl -fsS -H "Authorization: Bearer $TOKEN" "$API_BASE/whoami" | jq .
        ;;
    *)
        usage
        exit 2
        ;;
esac
