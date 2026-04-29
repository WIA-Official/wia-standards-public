#!/bin/bash
# WIA-FINTECH_INNOVATION CLI Tool
VERSION="1.0.0"
BASE_URL="${WIA_FINTECH_BASE_URL:-https://api.wia-fintech.io/v1}"
API_KEY="${WIA_FINTECH_API_KEY:-}"

cmd_payment() {
  local amount="$1"
  local currency="${2:-USD}"
  echo "Creating payment: $amount $currency"
  curl -X POST "$BASE_URL/payments" \
    -H "X-API-Key: $API_KEY" \
    -H "Content-Type: application/json" \
    -d "{\"amount\": $amount, \"currency\": \"$currency\", \"method\": \"card\"}"
}

cmd_bnpl() {
  local amount="$1"
  echo "Getting BNPL plans for: $amount"
  curl "$BASE_URL/bnpl/plans?amount=$amount" -H "X-API-Key: $API_KEY"
}

case "${1:-help}" in
  payment) cmd_payment "$2" "$3" ;;
  bnpl) cmd_bnpl "$2" ;;
  *) echo "Usage: $0 {payment|bnpl} [args]" ;;
esac
