#!/bin/bash

################################################################################
# WIA-ENE-028: Noise Pollution Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-028
# Noise Pollution Standard API.
#
# Usage:
#   noise-pollution.sh <command> [options]
#
# Commands:
#   measurement      Manage noise measurements
#   station          Manage monitoring stations
#   realtime         Get realtime noise data
#   source           Manage noise sources
#   time-period      Time period analysis
#   health           Health impact assessment
#   heatmap          Generate noise heatmap
#   alert            Manage noise alerts
#   complaint        Manage noise complaints
#   quiet-zone       Manage quiet zones
#   mitigation       Noise mitigation strategies
#   kpi              View KPI dashboard
#   compliance       Check compliance
#   certification    Certification management
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-028/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/noise-pollution.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║         🔊  WIA-ENE-028: Noise Pollution CLI v${VERSION}         ║"
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
# WIA-ENE-028 Noise Pollution CLI Configuration
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
      -H "X-WIA-Standard: ENE-028" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-028" \
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

cmd_measurement() {
  local action="${1:-}"
  shift || true

  case "$action" in
    get)
      local measurement_id="${1:-}"
      if [ -z "$measurement_id" ]; then
        echo "Usage: $0 measurement get MEASUREMENT_ID"
        exit 1
      fi
      print_info "Fetching measurement: $measurement_id"
      local response=$(api_request "GET" "/api/v1/noise/measurement/$measurement_id")
      pretty_json "$response"
      ;;
    list)
      local station_id=""
      local start_date=""
      local end_date=""
      while [[ $# -gt 0 ]]; do
        case $1 in
          --station) station_id="$2"; shift 2 ;;
          --start-date) start_date="$2"; shift 2 ;;
          --end-date) end_date="$2"; shift 2 ;;
          *) shift ;;
        esac
      done
      local path="/api/v1/noise/measurement?"
      [ -n "$station_id" ] && path="${path}stationId=${station_id}&"
      [ -n "$start_date" ] && path="${path}startDate=${start_date}&"
      [ -n "$end_date" ] && path="${path}endDate=${end_date}&"
      print_info "Listing measurements..."
      local response=$(api_request "GET" "${path%&}")
      pretty_json "$response"
      ;;
    stats)
      local station_id="${1:-}"
      local start_date="${2:-}"
      local end_date="${3:-}"
      if [ -z "$station_id" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
        echo "Usage: $0 measurement stats STATION_ID START_DATE END_DATE"
        exit 1
      fi
      print_info "Fetching statistics for station: $station_id"
      local response=$(api_request "GET" "/api/v1/noise/station/$station_id/statistics?startDate=$start_date&endDate=$end_date")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 measurement <get|list|stats> [options]"
      exit 1
      ;;
  esac
}

cmd_station() {
  local action="${1:-}"
  shift || true

  case "$action" in
    get)
      local station_id="${1:-}"
      if [ -z "$station_id" ]; then
        echo "Usage: $0 station get STATION_ID"
        exit 1
      fi
      print_info "Fetching station: $station_id"
      local response=$(api_request "GET" "/api/v1/noise/station/$station_id")
      pretty_json "$response"
      ;;
    list)
      local type=""
      local status=""
      local region=""
      while [[ $# -gt 0 ]]; do
        case $1 in
          --type) type="$2"; shift 2 ;;
          --status) status="$2"; shift 2 ;;
          --region) region="$2"; shift 2 ;;
          *) shift ;;
        esac
      done
      local path="/api/v1/noise/station?"
      [ -n "$type" ] && path="${path}type=${type}&"
      [ -n "$status" ] && path="${path}status=${status}&"
      [ -n "$region" ] && path="${path}region=${region}&"
      print_info "Listing stations..."
      local response=$(api_request "GET" "${path%&}")
      pretty_json "$response"
      ;;
    status)
      local station_id="${1:-}"
      if [ -z "$station_id" ]; then
        echo "Usage: $0 station status STATION_ID"
        exit 1
      fi
      print_info "Fetching station status: $station_id"
      local response=$(api_request "GET" "/api/v1/noise/station/$station_id/status")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 station <get|list|status> [options]"
      exit 1
      ;;
  esac
}

cmd_realtime() {
  local region="${1:-}"
  local path="/api/v1/noise/realtime"
  [ -n "$region" ] && path="${path}?region=${region}"

  print_info "Fetching realtime noise data..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_source() {
  local action="${1:-}"
  shift || true

  case "$action" in
    get)
      local source_id="${1:-}"
      if [ -z "$source_id" ]; then
        echo "Usage: $0 source get SOURCE_ID"
        exit 1
      fi
      print_info "Fetching noise source: $source_id"
      local response=$(api_request "GET" "/api/v1/noise/source/$source_id")
      pretty_json "$response"
      ;;
    list)
      local category=""
      local region=""
      while [[ $# -gt 0 ]]; do
        case $1 in
          --category) category="$2"; shift 2 ;;
          --region) region="$2"; shift 2 ;;
          *) shift ;;
        esac
      done
      local path="/api/v1/noise/source?"
      [ -n "$category" ] && path="${path}category=${category}&"
      [ -n "$region" ] && path="${path}region=${region}&"
      print_info "Listing noise sources..."
      local response=$(api_request "GET" "${path%&}")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 source <get|list> [options]"
      exit 1
      ;;
  esac
}

cmd_time_period() {
  local location="${1:-}"
  local date="${2:-}"

  if [ -z "$location" ] || [ -z "$date" ]; then
    echo "Usage: $0 time-period LOCATION DATE"
    echo ""
    echo "Example:"
    echo "  $0 time-period Seoul 2025-12-25"
    exit 1
  fi

  print_info "Fetching time period analysis for: $location ($date)"
  local response=$(api_request "GET" "/api/v1/noise/time-period?location=$location&date=$date")
  pretty_json "$response"
}

cmd_health() {
  local region="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 health REGION START_DATE END_DATE"
    echo ""
    echo "Example:"
    echo "  $0 health Seoul 2025-01-01 2025-12-31"
    exit 1
  fi

  print_info "Fetching health impact assessment for: $region"
  local response=$(api_request "GET" "/api/v1/noise/analytics/health?region=$region&startDate=$start_date&endDate=$end_date")
  pretty_json "$response"
}

cmd_heatmap() {
  local region="${1:-}"
  local date="${2:-}"
  local time_period="${3:-}"

  if [ -z "$region" ]; then
    echo "Usage: $0 heatmap REGION [DATE] [TIME_PERIOD]"
    echo ""
    echo "TIME_PERIOD: day, evening, night"
    echo ""
    echo "Example:"
    echo "  $0 heatmap Seoul 2025-12-25 night"
    exit 1
  fi

  local path="/api/v1/noise/heatmap?region=$region"
  [ -n "$date" ] && path="${path}&date=${date}"
  [ -n "$time_period" ] && path="${path}&timePeriod=${time_period}"

  print_info "Fetching noise heatmap for: $region"
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_alert() {
  local action="${1:-}"
  shift || true

  case "$action" in
    list)
      local severity="${1:-}"
      local path="/api/v1/noise/alerts"
      [ -n "$severity" ] && path="${path}?severity=${severity}"
      print_info "Fetching active alerts..."
      local response=$(api_request "GET" "$path")
      pretty_json "$response"
      ;;
    get)
      local alert_id="${1:-}"
      if [ -z "$alert_id" ]; then
        echo "Usage: $0 alert get ALERT_ID"
        exit 1
      fi
      print_info "Fetching alert: $alert_id"
      local response=$(api_request "GET" "/api/v1/noise/alerts/$alert_id")
      pretty_json "$response"
      ;;
    ack)
      local alert_id="${1:-}"
      if [ -z "$alert_id" ]; then
        echo "Usage: $0 alert ack ALERT_ID"
        exit 1
      fi
      print_info "Acknowledging alert: $alert_id"
      local response=$(api_request "POST" "/api/v1/noise/alerts/$alert_id/acknowledge" "{}")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 alert <list|get|ack> [options]"
      exit 1
      ;;
  esac
}

cmd_complaint() {
  local action="${1:-}"
  shift || true

  case "$action" in
    submit)
      local location="${1:-}"
      local noise_source="${2:-}"
      local description="${3:-}"
      if [ -z "$location" ] || [ -z "$noise_source" ] || [ -z "$description" ]; then
        echo "Usage: $0 complaint submit LOCATION NOISE_SOURCE DESCRIPTION"
        exit 1
      fi
      local data=$(cat <<EOF
{
  "location": {
    "address": "$location",
    "coordinates": {
      "latitude": 0,
      "longitude": 0
    }
  },
  "noiseSource": "$noise_source",
  "description": "$description"
}
EOF
)
      print_info "Submitting noise complaint..."
      local response=$(api_request "POST" "/api/v1/noise/complaint" "$data")
      pretty_json "$response"
      ;;
    get)
      local complaint_id="${1:-}"
      if [ -z "$complaint_id" ]; then
        echo "Usage: $0 complaint get COMPLAINT_ID"
        exit 1
      fi
      print_info "Fetching complaint: $complaint_id"
      local response=$(api_request "GET" "/api/v1/noise/complaint/$complaint_id")
      pretty_json "$response"
      ;;
    list)
      local status=""
      local region=""
      while [[ $# -gt 0 ]]; do
        case $1 in
          --status) status="$2"; shift 2 ;;
          --region) region="$2"; shift 2 ;;
          *) shift ;;
        esac
      done
      local path="/api/v1/noise/complaint?"
      [ -n "$status" ] && path="${path}status=${status}&"
      [ -n "$region" ] && path="${path}region=${region}&"
      print_info "Listing complaints..."
      local response=$(api_request "GET" "${path%&}")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 complaint <submit|get|list> [options]"
      exit 1
      ;;
  esac
}

cmd_quiet_zone() {
  local action="${1:-}"
  shift || true

  case "$action" in
    list)
      local region="${1:-}"
      local path="/api/v1/noise/quiet-zone"
      [ -n "$region" ] && path="${path}?region=${region}"
      print_info "Listing quiet zones..."
      local response=$(api_request "GET" "$path")
      pretty_json "$response"
      ;;
    get)
      local zone_id="${1:-}"
      if [ -z "$zone_id" ]; then
        echo "Usage: $0 quiet-zone get ZONE_ID"
        exit 1
      fi
      print_info "Fetching quiet zone: $zone_id"
      local response=$(api_request "GET" "/api/v1/noise/quiet-zone/$zone_id")
      pretty_json "$response"
      ;;
    check)
      local latitude="${1:-}"
      local longitude="${2:-}"
      if [ -z "$latitude" ] || [ -z "$longitude" ]; then
        echo "Usage: $0 quiet-zone check LATITUDE LONGITUDE"
        exit 1
      fi
      print_info "Checking quiet zone at: $latitude, $longitude"
      local response=$(api_request "GET" "/api/v1/noise/quiet-zone/check?latitude=$latitude&longitude=$longitude")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 quiet-zone <list|get|check> [options]"
      exit 1
      ;;
  esac
}

cmd_mitigation() {
  local region="${1:-}"
  local path="/api/v1/noise/mitigation"
  [ -n "$region" ] && path="${path}?region=${region}"

  print_info "Fetching mitigation strategies..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_kpi() {
  local region="${1:-}"
  local date="${2:-$(date -u +%Y-%m-%d)}"

  if [ -z "$region" ]; then
    echo "Usage: $0 kpi REGION [DATE]"
    exit 1
  fi

  print_info "Fetching KPI dashboard for: $region ($date)"
  local response=$(api_request "GET" "/api/v1/noise/analytics/kpi?region=$region&date=$date")
  pretty_json "$response"
}

cmd_compliance() {
  local region="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 compliance REGION START_DATE END_DATE"
    exit 1
  fi

  print_info "Fetching compliance report for: $region"
  local response=$(api_request "GET" "/api/v1/noise/compliance?region=$region&startDate=$start_date&endDate=$end_date")
  pretty_json "$response"
}

cmd_certification() {
  local action="${1:-}"
  shift || true

  case "$action" in
    get)
      local station_id="${1:-}"
      if [ -z "$station_id" ]; then
        echo "Usage: $0 certification get STATION_ID"
        exit 1
      fi
      print_info "Fetching certification: $station_id"
      local response=$(api_request "GET" "/api/v1/noise/certification/$station_id")
      pretty_json "$response"
      ;;
    apply)
      local station_id="${1:-}"
      local level="${2:-silver}"
      if [ -z "$station_id" ]; then
        echo "Usage: $0 certification apply STATION_ID [LEVEL]"
        echo "LEVEL: bronze, silver, gold, platinum (default: silver)"
        exit 1
      fi
      print_info "Applying for certification: $station_id ($level)"
      local response=$(api_request "POST" "/api/v1/noise/certification/apply" "{\"stationId\":\"$station_id\",\"level\":\"$level\"}")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 certification <get|apply> [options]"
      exit 1
      ;;
  esac
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}measurement${NC} <get|list|stats> [options]
    Manage noise measurements
    - get MEASUREMENT_ID: Get measurement by ID
    - list [--station STATION_ID] [--start-date DATE] [--end-date DATE]: List measurements
    - stats STATION_ID START_DATE END_DATE: Get statistical summary

  ${GREEN}station${NC} <get|list|status> [options]
    Manage monitoring stations
    - get STATION_ID: Get station information
    - list [--type TYPE] [--status STATUS] [--region REGION]: List stations
    - status STATION_ID: Get station operational status

  ${GREEN}realtime${NC} [REGION]
    Get realtime noise data for all active stations

  ${GREEN}source${NC} <get|list> [options]
    Manage noise sources
    - get SOURCE_ID: Get noise source information
    - list [--category CATEGORY] [--region REGION]: List noise sources

  ${GREEN}time-period${NC} LOCATION DATE
    Get time period analysis (day/evening/night)

  ${GREEN}health${NC} REGION START_DATE END_DATE
    Get health impact assessment

  ${GREEN}heatmap${NC} REGION [DATE] [TIME_PERIOD]
    Generate noise heatmap
    TIME_PERIOD: day, evening, night

  ${GREEN}alert${NC} <list|get|ack> [options]
    Manage noise alerts
    - list [SEVERITY]: List active alerts
    - get ALERT_ID: Get alert details
    - ack ALERT_ID: Acknowledge alert

  ${GREEN}complaint${NC} <submit|get|list> [options]
    Manage noise complaints
    - submit LOCATION NOISE_SOURCE DESCRIPTION: Submit complaint
    - get COMPLAINT_ID: Get complaint details
    - list [--status STATUS] [--region REGION]: List complaints

  ${GREEN}quiet-zone${NC} <list|get|check> [options]
    Manage quiet zones
    - list [REGION]: List quiet zones
    - get ZONE_ID: Get quiet zone details
    - check LATITUDE LONGITUDE: Check if location is in quiet zone

  ${GREEN}mitigation${NC} [REGION]
    Get noise mitigation strategies

  ${GREEN}kpi${NC} REGION [DATE]
    View KPI dashboard

  ${GREEN}compliance${NC} REGION START_DATE END_DATE
    Check compliance report

  ${GREEN}certification${NC} <get|apply> [options]
    Certification management
    - get STATION_ID: Get station certification
    - apply STATION_ID [LEVEL]: Apply for certification

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-028/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Get realtime noise levels
  $0 realtime Seoul

  # Get time period analysis
  $0 time-period "Seoul Station" 2025-12-25

  # View health impact
  $0 health Seoul 2025-01-01 2025-12-31

  # Generate noise heatmap
  $0 heatmap Seoul 2025-12-25 night

  # Submit noise complaint
  $0 complaint submit "123 Main St" "Construction" "Loud noise at night"

  # View KPI dashboard
  $0 kpi Seoul 2025-12-25

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-028
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-028

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
  measurement)
    cmd_measurement "$@"
    ;;
  station)
    cmd_station "$@"
    ;;
  realtime)
    cmd_realtime "$@"
    ;;
  source)
    cmd_source "$@"
    ;;
  time-period)
    cmd_time_period "$@"
    ;;
  health)
    cmd_health "$@"
    ;;
  heatmap)
    cmd_heatmap "$@"
    ;;
  alert)
    cmd_alert "$@"
    ;;
  complaint)
    cmd_complaint "$@"
    ;;
  quiet-zone)
    cmd_quiet_zone "$@"
    ;;
  mitigation)
    cmd_mitigation "$@"
    ;;
  kpi)
    cmd_kpi "$@"
    ;;
  compliance)
    cmd_compliance "$@"
    ;;
  certification)
    cmd_certification "$@"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-028 Noise Pollution CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
