#!/bin/bash
BASE_URL="${WIA_FOODSAFETY_BASE_URL:-https://api.wia-foodsafety.io/v1}"
API_KEY="${WIA_FOODSAFETY_API_KEY:-}"

cmd_trace() {
  local qr_code="$1"
  echo "Tracing product: $qr_code"
  curl -H "X-API-Key: $API_KEY" "$BASE_URL/products/trace?qr_code=$qr_code"
}

cmd_recalls() {
  echo "Fetching active recalls..."
  curl -H "X-API-Key: $API_KEY" "$BASE_URL/recalls"
}

case "${1:-help}" in
  trace) cmd_trace "$2" ;;
  recalls) cmd_recalls ;;
  *) echo "Usage: $0 {trace|recalls} [args]" ;;
esac
