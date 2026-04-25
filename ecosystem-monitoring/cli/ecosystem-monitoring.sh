#!/bin/bash

################################################################################
# WIA-ENE-031: Ecosystem Monitoring Standard - CLI Tool
#
# 弘益人間 (홍익人間) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-031
# Ecosystem Monitoring Standard API.
#
# Usage:
#   ecosystem-monitoring.sh <command> [options]
#
# Commands:
#   list-ecosystems  List ecosystems
#   get-ecosystem    Get ecosystem details
#   health           Get ecosystem health
#   biodiversity     Get biodiversity data
#   carbon           Get carbon data
#   water            Get water data  
#   submit-data      Submit monitoring data
#   satellite        Satellite imagery operations
#   sensors          IoT sensor operations
#   restoration      Restoration project operations
#   protected-area   Protected area operations
#   kpi              View KPI dashboard
#   report           Generate reports
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-031/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/ecosystem-monitoring.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║      🌲  WIA-ENE-031: Ecosystem Monitoring CLI v${VERSION}       ║"
  echo "║                                                                ║"
  echo "║              弘益人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
}

print_success() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1" >&2
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

# Check if jq is available
has_jq() {
  command -v jq &> /dev/null
}

# Pretty print JSON (uses jq if available)
pretty_json() {
  local json="$1"
  if has_jq; then
    echo "$json" | jq '.'
  else
    echo "$json"
  fi
}

# Load configuration
load_config() {
  if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    if [ -n "${WIA_API_KEY_CONF:-}" ]; then
      API_KEY="${WIA_API_KEY_CONF}"
    fi
    if [ -n "${WIA_API_ENDPOINT_CONF:-}" ]; then
      API_ENDPOINT="${WIA_API_ENDPOINT_CONF}"
    fi
  fi
}

# Save configuration
save_config() {
  mkdir -p "$(dirname "$CONFIG_FILE")"
  cat > "$CONFIG_FILE" <<EOF
# WIA-ENE-031 Ecosystem Monitoring CLI Configuration
WIA_API_KEY_CONF="${API_KEY}"
WIA_API_ENDPOINT_CONF="${API_ENDPOINT}"
EOF
  chmod 600 "$CONFIG_FILE"
  print_success "Configuration saved to $CONFIG_FILE"
}

# Make API request
api_request() {
  local method="$1"
  local path="$2"
  local data="${3:-}"

  if [ -z "$API_KEY" ]; then
    print_error "API key not set. Use: export WIA_API_KEY=your-key or run 'config' command"
    exit 1
  fi

  local url="${API_ENDPOINT}${path}"
  local response

  if [ "$method" = "GET" ]; then
    response=$(curl -s -X GET "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-031" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-031" \
      -H "X-WIA-Version: 1.0.0" \
      -d "$data")
  fi

  echo "$response"
}

################################################################################
# Commands
################################################################################

cmd_config() {
  print_header
  echo "Configuration Setup"
  echo ""

  read -p "API Endpoint [${API_ENDPOINT}]: " input_endpoint
  if [ -n "$input_endpoint" ]; then
    API_ENDPOINT="$input_endpoint"
  fi

  read -sp "API Key: " input_key
  echo ""
  if [ -n "$input_key" ]; then
    API_KEY="$input_key"
  fi

  save_config
  echo ""
  print_success "Configuration complete!"
}

cmd_list_ecosystems() {
  local type="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  local path="/api/v1/ecosystems?page=$page&limit=$limit"
  if [ -n "$type" ]; then
    path="$path&type=$type"
  fi

  print_info "Listing ecosystems..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_ecosystem() {
  local ecosystem_id="${1:-}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 get-ecosystem ECOSYSTEM_ID"
    exit 1
  fi

  print_info "Fetching ecosystem: $ecosystem_id"
  local response=$(api_request "GET" "/api/v1/ecosystems/$ecosystem_id")

  pretty_json "$response"
}

cmd_health() {
  local ecosystem_id="${1:-}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 health ECOSYSTEM_ID"
    exit 1
  fi

  print_info "Fetching ecosystem health: $ecosystem_id"
  local response=$(api_request "GET" "/api/v1/ecosystems/$ecosystem_id/health")

  pretty_json "$response"
}

cmd_biodiversity() {
  local ecosystem_id="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 biodiversity ECOSYSTEM_ID [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/ecosystems/$ecosystem_id/biodiversity"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path?startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching biodiversity data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_carbon() {
  local ecosystem_id="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 carbon ECOSYSTEM_ID [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/ecosystems/$ecosystem_id/carbon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path?startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching carbon sequestration data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_water() {
  local ecosystem_id="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 water ECOSYSTEM_ID [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/ecosystems/$ecosystem_id/water"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path?startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching water quality data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_kpi() {
  local ecosystem_id="${1:-}"
  local date="${2:-$(date -u +%Y-%m-%d)}"

  if [ -z "$ecosystem_id" ]; then
    echo "Usage: $0 kpi ECOSYSTEM_ID [DATE]"
    exit 1
  fi

  local path="/api/v1/analytics/kpi/$ecosystem_id?date=$date"

  print_info "Fetching KPI dashboard..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_report() {
  local ecosystem_id="${1:-}"
  local report_type="${2:-monthly}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$ecosystem_id" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 report ECOSYSTEM_ID --type TYPE --start-date DATE --end-date DATE"
    echo ""
    echo "Report Types: monthly, quarterly, annual"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "ecosystemId": "$ecosystem_id",
  "reportType": "$report_type",
  "period": {
    "startDate": "${start_date}T00:00:00Z",
    "endDate": "${end_date}T23:59:59Z"
  }
}
EOF
)

  print_info "Generating $report_type report..."
  local response=$(api_request "POST" "/api/v1/analytics/report" "$data")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}list-ecosystems${NC} [--type TYPE] [--page PAGE] [--limit LIMIT]
    List ecosystems with optional filtering

  ${GREEN}get-ecosystem${NC} ECOSYSTEM_ID
    Get ecosystem details

  ${GREEN}health${NC} ECOSYSTEM_ID
    Get ecosystem health indicators

  ${GREEN}biodiversity${NC} ECOSYSTEM_ID [START_DATE] [END_DATE]
    Get biodiversity data

  ${GREEN}carbon${NC} ECOSYSTEM_ID [START_DATE] [END_DATE]
    Get carbon sequestration data

  ${GREEN}water${NC} ECOSYSTEM_ID [START_DATE] [END_DATE]
    Get water quality and hydrology data

  ${GREEN}kpi${NC} ECOSYSTEM_ID [DATE]
    View KPI dashboard

  ${GREEN}report${NC} ECOSYSTEM_ID --type TYPE --start-date DATE --end-date DATE
    Generate performance report
    Types: monthly, quarterly, annual

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-031/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # List ecosystems
  $0 list-ecosystems --type temperate_forest

  # Get ecosystem health
  $0 health ECO-001

  # Get biodiversity data
  $0 biodiversity ECO-001 2025-01-01 2025-12-31

  # Get carbon data
  $0 carbon ECO-001 2025-01-01 2025-12-31

  # View KPI dashboard
  $0 kpi ECO-001

  # Generate report
  $0 report ECO-001 --type annual --start-date 2025-01-01 --end-date 2025-12-31

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-031
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-031

${BLUE}════════════════════════════════════════════════════════════════${NC}
${GREEN}         弘益人間 (홍익인간) · Benefit All Humanity${NC}
${BLUE}════════════════════════════════════════════════════════════════${NC}
EOF
}

################################################################################
# Main
################################################################################

# Load configuration
load_config

# Parse command
COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
  config)
    cmd_config
    ;;
  list-ecosystems)
    TYPE=""
    PAGE="1"
    LIMIT="10"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --page) PAGE="$2"; shift 2 ;;
        --limit) LIMIT="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_ecosystems "$TYPE" "$PAGE" "$LIMIT"
    ;;
  get-ecosystem)
    cmd_get_ecosystem "$@"
    ;;
  health)
    cmd_health "$@"
    ;;
  biodiversity)
    cmd_biodiversity "$@"
    ;;
  carbon)
    cmd_carbon "$@"
    ;;
  water)
    cmd_water "$@"
    ;;
  kpi)
    cmd_kpi "$@"
    ;;
  report)
    ECOSYSTEM_ID=""
    TYPE="monthly"
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) ECOSYSTEM_ID="$1"; shift ;;
      esac
    done
    cmd_report "$ECOSYSTEM_ID" "$TYPE" "$START_DATE" "$END_DATE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-031 Ecosystem Monitoring CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
