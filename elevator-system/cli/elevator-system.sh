#!/bin/bash

################################################################################
# WIA-CITY-012: Elevator System Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-012
# Elevator System Standard API.
#
# Usage:
#   elevator-system.sh <command> [options]
#
# Commands:
#   list-elevators      List all elevators in a building
#   status              Get elevator status
#   call-elevator       Call elevator to a floor
#   destination         Create destination dispatch request
#   group-control       Manage group control system
#   traffic-analysis    Analyze elevator traffic patterns
#   energy-stats        View energy consumption and regeneration
#   maintenance         View maintenance schedule and alerts
#   kpi                 View performance KPIs
#   help                Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/city-012/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/elevator-system.conf"

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
  echo "║       🛗  WIA-CITY-012: Elevator System CLI v${VERSION}        ║"
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

# Parse JSON response
parse_json() {
  local json="$1"
  local key="$2"

  if has_jq; then
    echo "$json" | jq -r "$key"
  else
    echo "$json" | grep -o "\"$key\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" | sed 's/.*"\([^"]*\)".*/\1/'
  fi
}

# Pretty print JSON
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
# WIA-CITY-012 Elevator System CLI Configuration
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
      -H "X-WIA-Standard: CITY-012" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: CITY-012" \
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

cmd_list_elevators() {
  local building_id="${1:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 list-elevators --building-id BUILDING_ID"
    echo ""
    echo "Example:"
    echo "  $0 list-elevators --building-id \"BLDG-001\""
    exit 1
  fi

  print_info "Fetching elevators for building: $building_id"
  local response=$(api_request "GET" "/api/v1/elevators?buildingId=$building_id")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local count=$(echo "$response" | jq '.data.total')
    print_success "Found $count elevators"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch elevators"
    echo "$response"
    exit 1
  fi
}

cmd_status() {
  local elevator_id="${1:-}"

  if [ -z "$elevator_id" ]; then
    echo "Usage: $0 status --elevator-id ELEVATOR_ID"
    echo ""
    echo "Example:"
    echo "  $0 status --elevator-id \"BLDG-A-ELV-01\""
    exit 1
  fi

  print_info "Fetching status for elevator: $elevator_id"
  local response=$(api_request "GET" "/api/v1/elevators/$elevator_id")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local floor=$(echo "$response" | jq -r '.data.currentFloor')
    local direction=$(echo "$response" | jq -r '.data.direction')
    local status=$(echo "$response" | jq -r '.data.operational')
    local load=$(echo "$response" | jq -r '.data.load.percentage')

    echo ""
    echo -e "${CYAN}Elevator:${NC} $elevator_id"
    echo -e "${CYAN}Status:${NC} $status"
    echo -e "${CYAN}Current Floor:${NC} $floor"
    echo -e "${CYAN}Direction:${NC} $direction"
    echo -e "${CYAN}Load:${NC} ${load}%"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch elevator status"
    echo "$response"
    exit 1
  fi
}

cmd_call_elevator() {
  local elevator_id="${1:-}"
  local floor="${2:-}"
  local direction="${3:-}"

  if [ -z "$elevator_id" ] || [ -z "$floor" ]; then
    echo "Usage: $0 call-elevator --elevator-id ELEVATOR_ID --floor FLOOR [--direction up|down]"
    echo ""
    echo "Example:"
    echo "  $0 call-elevator --elevator-id \"BLDG-A-ELV-01\" --floor 10 --direction up"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "floor": $floor,
  "direction": "${direction:-}"
}
EOF
)

  print_info "Calling elevator $elevator_id to floor $floor"
  local response=$(api_request "POST" "/api/v1/elevators/$elevator_id/call" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local call_id=$(echo "$response" | jq -r '.data.callId')
    local eta=$(echo "$response" | jq -r '.data.estimatedArrival')
    print_success "Call created: $call_id"
    echo -e "${CYAN}Estimated Arrival:${NC} ${eta} seconds"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to call elevator"
    echo "$response"
    exit 1
  fi
}

cmd_destination() {
  local from_floor="${1:-}"
  local to_floor="${2:-}"
  local building_id="${3:-BLDG-001}"

  if [ -z "$from_floor" ] || [ -z "$to_floor" ]; then
    echo "Usage: $0 destination --from FLOOR --to FLOOR [--building-id ID]"
    echo ""
    echo "Example:"
    echo "  $0 destination --from 1 --to 25 --building-id \"BLDG-001\""
    exit 1
  fi

  local data=$(cat <<EOF
{
  "hall": {
    "floor": $from_floor,
    "location": "main"
  },
  "destination": $to_floor,
  "passengerCount": 1,
  "priority": "normal",
  "accessibility": {
    "wheelchair": false,
    "visuallyImpaired": false,
    "hearingImpaired": false
  }
}
EOF
)

  print_info "Creating destination request: Floor $from_floor → $to_floor"
  local response=$(api_request "POST" "/api/v1/destination/request" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local car=$(echo "$response" | jq -r '.data.carNumber')
    local eta=$(echo "$response" | jq -r '.data.estimatedArrival')
    print_success "Assigned to Car #$car"
    echo -e "${CYAN}Estimated Arrival:${NC} ${eta} seconds"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create destination request"
    echo "$response"
    exit 1
  fi
}

cmd_group_control() {
  local building_id="${1:-}"
  local action="${2:-status}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 group-control --building-id BUILDING_ID [--action status|metrics|optimize]"
    echo ""
    echo "Example:"
    echo "  $0 group-control --building-id \"BLDG-001\" --action status"
    exit 1
  fi

  local path=""
  case "$action" in
    status)
      path="/api/v1/group-control/$building_id/status"
      ;;
    metrics)
      path="/api/v1/group-control/$building_id/metrics"
      ;;
    optimize)
      path="/api/v1/group-control/$building_id/optimize"
      ;;
    *)
      print_error "Unknown action: $action"
      exit 1
      ;;
  esac

  print_info "Group control $action for building: $building_id"

  if [ "$action" = "optimize" ]; then
    local response=$(api_request "POST" "$path" "{}")
  else
    local response=$(api_request "GET" "$path")
  fi

  pretty_json "$response"
}

cmd_traffic_analysis() {
  local building_id="${1:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 traffic-analysis --building-id BUILDING_ID"
    echo ""
    echo "Example:"
    echo "  $0 traffic-analysis --building-id \"BLDG-001\""
    exit 1
  fi

  print_info "Analyzing traffic for building: $building_id"
  local response=$(api_request "GET" "/api/v1/traffic/$building_id/analysis")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local demand=$(echo "$response" | jq -r '.data.realTime.currentDemand')
    local direction=$(echo "$response" | jq -r '.data.realTime.trafficDirection')

    echo ""
    echo -e "${CYAN}Current Demand:${NC} $demand calls/min"
    echo -e "${CYAN}Traffic Direction:${NC} $direction"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch traffic analysis"
    echo "$response"
    exit 1
  fi
}

cmd_energy_stats() {
  local building_id="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 energy-stats --building-id BUILDING_ID [--start-date DATE] [--end-date DATE]"
    echo ""
    echo "Example:"
    echo "  $0 energy-stats --building-id \"BLDG-001\""
    exit 1
  fi

  local path="/api/v1/energy/$building_id/stats"
  local params=""

  if [ -n "$start_date" ]; then
    params="startDate=$start_date"
  fi
  if [ -n "$end_date" ]; then
    if [ -n "$params" ]; then
      params="$params&endDate=$end_date"
    else
      params="endDate=$end_date"
    fi
  fi

  if [ -n "$params" ]; then
    path="$path?$params"
  fi

  print_info "Fetching energy statistics for building: $building_id"
  local response=$(api_request "GET" "$path")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local consumed=$(echo "$response" | jq -r '.data.consumption.dailyTotal')
    local regenerated=$(echo "$response" | jq -r '.data.regeneration.dailyRegeneration')
    local rate=$(echo "$response" | jq -r '.data.regeneration.regenerationRate')

    echo ""
    echo -e "${CYAN}Daily Consumption:${NC} ${consumed} kWh"
    echo -e "${CYAN}Daily Regeneration:${NC} ${regenerated} kWh"
    echo -e "${CYAN}Regeneration Rate:${NC} ${rate}%"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch energy statistics"
    echo "$response"
    exit 1
  fi
}

cmd_maintenance() {
  local elevator_id="${1:-}"

  if [ -z "$elevator_id" ]; then
    echo "Usage: $0 maintenance --elevator-id ELEVATOR_ID"
    echo ""
    echo "Example:"
    echo "  $0 maintenance --elevator-id \"BLDG-A-ELV-01\""
    exit 1
  fi

  print_info "Fetching maintenance status for elevator: $elevator_id"
  local response=$(api_request "GET" "/api/v1/elevators/$elevator_id/maintenance")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local mtbf=$(echo "$response" | jq -r '.data.performance.mtbf')
    local mttr=$(echo "$response" | jq -r '.data.performance.mttr')
    local availability=$(echo "$response" | jq -r '.data.performance.availabilityRate')

    echo ""
    echo -e "${CYAN}MTBF:${NC} ${mtbf} hours"
    echo -e "${CYAN}MTTR:${NC} ${mttr} hours"
    echo -e "${CYAN}Availability:${NC} ${availability}%"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch maintenance status"
    echo "$response"
    exit 1
  fi
}

cmd_kpi() {
  local building_id="${1:-}"
  local kpi_type="${2:-operational}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 kpi --building-id BUILDING_ID [--type operational|energy|safety]"
    echo ""
    echo "Example:"
    echo "  $0 kpi --building-id \"BLDG-001\" --type operational"
    exit 1
  fi

  print_info "Fetching $kpi_type KPIs for building: $building_id"
  local response=$(api_request "GET" "/api/v1/analytics/$building_id/kpi/$kpi_type")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}list-elevators${NC} --building-id BUILDING_ID
    List all elevators in a building

  ${GREEN}status${NC} --elevator-id ELEVATOR_ID
    Get current elevator status

  ${GREEN}call-elevator${NC} --elevator-id ELEVATOR_ID --floor FLOOR [--direction up|down]
    Call elevator to a specific floor

  ${GREEN}destination${NC} --from FLOOR --to FLOOR [--building-id ID]
    Create destination dispatch request

  ${GREEN}group-control${NC} --building-id BUILDING_ID [--action status|metrics|optimize]
    Manage group control system

  ${GREEN}traffic-analysis${NC} --building-id BUILDING_ID
    Analyze elevator traffic patterns

  ${GREEN}energy-stats${NC} --building-id BUILDING_ID [--start-date DATE] [--end-date DATE]
    View energy consumption and regeneration statistics

  ${GREEN}maintenance${NC} --elevator-id ELEVATOR_ID
    View predictive maintenance status

  ${GREEN}kpi${NC} --building-id BUILDING_ID [--type operational|energy|safety]
    View performance KPIs

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/city-012/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # List elevators
  $0 list-elevators --building-id "BLDG-001"

  # Get elevator status
  $0 status --elevator-id "BLDG-A-ELV-01"

  # Call elevator
  $0 call-elevator --elevator-id "BLDG-A-ELV-01" --floor 10 --direction up

  # Destination dispatch
  $0 destination --from 1 --to 25 --building-id "BLDG-001"

  # View energy stats
  $0 energy-stats --building-id "BLDG-001"

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/city-012
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/city-012

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
  list-elevators)
    BUILDING_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_elevators "$BUILDING_ID"
    ;;
  status)
    ELEVATOR_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --elevator-id) ELEVATOR_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_status "$ELEVATOR_ID"
    ;;
  call-elevator)
    ELEVATOR_ID=""
    FLOOR=""
    DIRECTION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --elevator-id) ELEVATOR_ID="$2"; shift 2 ;;
        --floor) FLOOR="$2"; shift 2 ;;
        --direction) DIRECTION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_call_elevator "$ELEVATOR_ID" "$FLOOR" "$DIRECTION"
    ;;
  destination)
    FROM_FLOOR=""
    TO_FLOOR=""
    BUILDING_ID="BLDG-001"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --from) FROM_FLOOR="$2"; shift 2 ;;
        --to) TO_FLOOR="$2"; shift 2 ;;
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_destination "$FROM_FLOOR" "$TO_FLOOR" "$BUILDING_ID"
    ;;
  group-control)
    BUILDING_ID=""
    ACTION="status"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        --action) ACTION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_group_control "$BUILDING_ID" "$ACTION"
    ;;
  traffic-analysis)
    BUILDING_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_traffic_analysis "$BUILDING_ID"
    ;;
  energy-stats)
    BUILDING_ID=""
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_energy_stats "$BUILDING_ID" "$START_DATE" "$END_DATE"
    ;;
  maintenance)
    ELEVATOR_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --elevator-id) ELEVATOR_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_maintenance "$ELEVATOR_ID"
    ;;
  kpi)
    BUILDING_ID=""
    KPI_TYPE="operational"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --building-id) BUILDING_ID="$2"; shift 2 ;;
        --type) KPI_TYPE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_kpi "$BUILDING_ID" "$KPI_TYPE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-CITY-012 Elevator System CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
