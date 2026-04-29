#!/bin/bash
BASE_URL="${WIA_FLOOD_BASE_URL:-https://api.wia-flood.io/v1}"
API_KEY="${WIA_FLOOD_API_KEY:-}"

cmd_predict() {
  local lat="$1"
  local lng="$2"
  echo "Getting flood prediction for: $lat, $lng"
  curl -H "X-API-Key: $API_KEY" "$BASE_URL/predictions?lat=$lat&lng=$lng"
}

cmd_gauges() {
  local region="$1"
  curl -H "X-API-Key: $API_KEY" "$BASE_URL/gauges?region=$region"
}

case "${1:-help}" in
  predict) cmd_predict "$2" "$3" ;;
  gauges) cmd_gauges "$2" ;;
  *) echo "Usage: $0 {predict|gauges} [args]" ;;
esac
