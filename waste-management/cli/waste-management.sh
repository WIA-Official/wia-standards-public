#!/bin/bash

################################################################################
# WIA-ENE-022: Waste Management Standard - CLI Tool
#
# 弘益人間 (홍익人間) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-022
# Waste Management Standard API.
#
# Usage:
#   waste-management.sh <command> [options]
#
# Commands:
#   create-event     Create waste generation event
#   get-event        Get waste event by ID
#   list-events      List waste events
#   schedule         Manage collection schedules
#   smartbin         Smart bin operations
#   facility         Facility operations
#   performance      Performance analytics
#   report           Generate reports
#   kpi              View KPI dashboard
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-022/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/waste-management.conf"

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
  echo "║         🗑️  WIA-ENE-022: Waste Management CLI v${VERSION}        ║"
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

# Parse JSON response (uses jq if available, otherwise basic parsing)
parse_json() {
  local json="$1"
  local key="$2"

  if has_jq; then
    echo "$json" | jq -r "$key"
  else
    # Basic grep/sed parsing (less reliable but works without jq)
    echo "$json" | grep -o "\"$key\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" | sed 's/.*"\([^"]*\)".*/\1/'
  fi
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
# WIA-ENE-022 Waste Management CLI Configuration
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
      -H "X-WIA-Standard: ENE-022" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-022" \
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

cmd_create_event() {
  local category="${1:-}"
  local quantity="${2:-}"
  local method="${3:-}"

  if [ -z "$category" ] || [ -z "$quantity" ] || [ -z "$method" ]; then
    echo "Usage: $0 create-event --category CATEGORY --quantity QUANTITY --method METHOD"
    echo ""
    echo "Categories: PAPER, PLASTIC, GLASS, METAL, FOOD, TEXTILE, WOOD, ELECTRONICS, HAZARDOUS, OTHER"
    echo "Methods: recycling, composting, incineration, landfill, energy_recovery, special_treatment"
    echo ""
    echo "Example:"
    echo "  $0 create-event --category PLASTIC --quantity 15.5 --method recycling"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "generatorId": "CLI-USER",
  "location": {
    "address": "Unknown",
    "coordinates": {
      "latitude": 0,
      "longitude": 0
    }
  },
  "waste": {
    "categoryCode": "WM-${category}",
    "name": "${category}",
    "quantity": ${quantity},
    "volume": 0,
    "hazardClass": 1,
    "composition": []
  },
  "treatment": {
    "plannedMethod": "${method}",
    "destinationId": "UNKNOWN",
    "scheduledDate": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  }
}
EOF
)

  print_info "Creating waste generation event..."
  local response=$(api_request "POST" "/api/v1/waste/generate" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local event_id=$(echo "$response" | jq -r '.data.eventId')
    print_success "Event created: $event_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create event"
    echo "$response"
    exit 1
  fi
}

cmd_get_event() {
  local event_id="${1:-}"

  if [ -z "$event_id" ]; then
    echo "Usage: $0 get-event EVENT_ID"
    exit 1
  fi

  print_info "Fetching event: $event_id"
  local response=$(api_request "GET" "/api/v1/waste/$event_id")

  pretty_json "$response"
}

cmd_list_events() {
  local category="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  local path="/api/v1/waste?page=$page&limit=$limit"
  if [ -n "$category" ]; then
    path="$path&categoryCode=WM-$category"
  fi

  print_info "Listing waste events..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_schedule() {
  local region="${1:-}"
  local category="${2:-}"

  if [ -z "$region" ]; then
    echo "Usage: $0 schedule --region REGION [--category CATEGORY]"
    exit 1
  fi

  local path="/api/v1/collection/schedules?region=$region"
  if [ -n "$category" ]; then
    path="$path&categoryCode=WM-$category"
  fi

  print_info "Fetching collection schedules for: $region"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_smartbin() {
  local action="${1:-}"
  local bin_id="${2:-}"

  case "$action" in
    get)
      if [ -z "$bin_id" ]; then
        echo "Usage: $0 smartbin get BIN_ID"
        exit 1
      fi
      print_info "Fetching smart bin: $bin_id"
      local response=$(api_request "GET" "/api/v1/smartbin/$bin_id")
      pretty_json "$response"
      ;;
    needs-collection)
      local region="${bin_id:-}"
      local path="/api/v1/smartbin/needs-collection"
      if [ -n "$region" ]; then
        path="$path?region=$region"
      fi
      print_info "Fetching bins needing collection..."
      local response=$(api_request "GET" "$path")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 smartbin <get|needs-collection> [options]"
      exit 1
      ;;
  esac
}

cmd_facility() {
  local action="${1:-}"
  local facility_id="${2:-}"

  case "$action" in
    status)
      if [ -z "$facility_id" ]; then
        echo "Usage: $0 facility status FACILITY_ID"
        exit 1
      fi
      print_info "Fetching facility status: $facility_id"
      local response=$(api_request "GET" "/api/v1/facility/$facility_id/status")
      pretty_json "$response"
      ;;
    performance)
      if [ -z "$facility_id" ]; then
        echo "Usage: $0 facility performance FACILITY_ID"
        exit 1
      fi
      print_info "Fetching facility performance: $facility_id"
      local response=$(api_request "GET" "/api/v1/facility/$facility_id/performance")
      pretty_json "$response"
      ;;
    list)
      print_info "Listing facilities..."
      local response=$(api_request "GET" "/api/v1/facilities")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 facility <status|performance|list> [FACILITY_ID]"
      exit 1
      ;;
  esac
}

cmd_performance() {
  local region="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 performance --region REGION --start-date YYYY-MM-DD --end-date YYYY-MM-DD"
    echo ""
    echo "Example:"
    echo "  $0 performance --region Seoul --start-date 2025-01-01 --end-date 2025-12-31"
    exit 1
  fi

  local path="/api/v1/analytics/performance?region=$region&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"

  print_info "Fetching recycling performance for: $region ($start_date to $end_date)"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_kpi() {
  local region="${1:-}"
  local date="${2:-$(date -u +%Y-%m-%d)}"

  local path="/api/v1/analytics/kpi"
  local params=""
  if [ -n "$region" ]; then
    params="region=$region"
  fi
  if [ -n "$date" ]; then
    if [ -n "$params" ]; then
      params="$params&date=$date"
    else
      params="date=$date"
    fi
  fi
  if [ -n "$params" ]; then
    path="$path?$params"
  fi

  print_info "Fetching KPI dashboard..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_report() {
  local report_type="${1:-monthly}"
  local region="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 report --type TYPE --region REGION --start-date YYYY-MM-DD --end-date YYYY-MM-DD"
    echo ""
    echo "Report Types: daily, weekly, monthly, quarterly, annual"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "reportType": "$report_type",
  "region": "$region",
  "period": {
    "start": "${start_date}T00:00:00Z",
    "end": "${end_date}T23:59:59Z"
  }
}
EOF
)

  print_info "Generating $report_type report for: $region"
  local response=$(api_request "POST" "/api/v1/analytics/report" "$data")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}create-event${NC} --category CATEGORY --quantity QUANTITY --method METHOD
    Create waste generation event
    Categories: PAPER, PLASTIC, GLASS, METAL, FOOD, TEXTILE, WOOD, ELECTRONICS, HAZARDOUS, OTHER
    Methods: recycling, composting, incineration, landfill, energy_recovery, special_treatment

  ${GREEN}get-event${NC} EVENT_ID
    Get waste event by ID

  ${GREEN}list-events${NC} [--category CATEGORY] [--page PAGE] [--limit LIMIT]
    List waste events with optional filtering

  ${GREEN}schedule${NC} --region REGION [--category CATEGORY]
    Get collection schedules for a region

  ${GREEN}smartbin${NC} <get|needs-collection> [BIN_ID|REGION]
    Smart bin operations
    - get BIN_ID: Get smart bin status
    - needs-collection [REGION]: List bins needing collection

  ${GREEN}facility${NC} <status|performance|list> [FACILITY_ID]
    Facility operations
    - status FACILITY_ID: Get facility status
    - performance FACILITY_ID: Get facility performance
    - list: List all facilities

  ${GREEN}performance${NC} --region REGION --start-date DATE --end-date DATE
    Get recycling performance analytics

  ${GREEN}kpi${NC} [--region REGION] [--date DATE]
    View KPI dashboard

  ${GREEN}report${NC} --type TYPE --region REGION --start-date DATE --end-date DATE
    Generate performance report
    Types: daily, weekly, monthly, quarterly, annual

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-022/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Create waste event
  $0 create-event --category PLASTIC --quantity 15.5 --method recycling

  # Get collection schedule
  $0 schedule --region "Seoul" --category PLASTIC

  # View performance
  $0 performance --region "Seoul" --start-date 2025-01-01 --end-date 2025-12-31

  # Generate report
  $0 report --type monthly --region "Seoul" --start-date 2025-12-01 --end-date 2025-12-31

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-022
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-022

${BLUE}════════════════════════════════════════════════════════════════${NC}
${GREEN}         弘익人間 (홍익인간) · Benefit All Humanity${NC}
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
  create-event)
    # Parse flags
    CATEGORY=""
    QUANTITY=""
    METHOD=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --category) CATEGORY="$2"; shift 2 ;;
        --quantity) QUANTITY="$2"; shift 2 ;;
        --method) METHOD="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_event "$CATEGORY" "$QUANTITY" "$METHOD"
    ;;
  get-event)
    cmd_get_event "$@"
    ;;
  list-events)
    CATEGORY=""
    PAGE="1"
    LIMIT="10"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --category) CATEGORY="$2"; shift 2 ;;
        --page) PAGE="$2"; shift 2 ;;
        --limit) LIMIT="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_events "$CATEGORY" "$PAGE" "$LIMIT"
    ;;
  schedule)
    REGION=""
    CATEGORY=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --category) CATEGORY="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_schedule "$REGION" "$CATEGORY"
    ;;
  smartbin)
    cmd_smartbin "$@"
    ;;
  facility)
    cmd_facility "$@"
    ;;
  performance)
    REGION=""
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_performance "$REGION" "$START_DATE" "$END_DATE"
    ;;
  kpi)
    REGION=""
    DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --date) DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_kpi "$REGION" "$DATE"
    ;;
  report)
    TYPE="monthly"
    REGION=""
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --region) REGION="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_report "$TYPE" "$REGION" "$START_DATE" "$END_DATE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-022 Waste Management CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
