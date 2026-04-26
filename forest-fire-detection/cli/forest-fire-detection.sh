#!/bin/bash

################################################################################
# WIA-ENE-032: Forest Fire Detection Standard - CLI Tool
#
# 弘益人間 (홍익人間) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-032
# Forest Fire Detection Standard API.
#
# Usage:
#   forest-fire-detection.sh <command> [options]
#
# Commands:
#   detect           Report fire detection event
#   list             List active fires
#   predict          Request fire spread prediction
#   danger           Get fire danger level
#   evacuate         Manage evacuation zones
#   resources        Manage firefighting resources
#   alert            Send fire alerts
#   status           Get certification status
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-032/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/forest-fire-detection.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
ORANGE='\033[0;33m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${RED}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║    🔥  WIA-ENE-032: Forest Fire Detection CLI v${VERSION}       ║"
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

print_fire() {
  echo -e "${RED}[🔥]${NC} $1"
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

# Check API key
check_api_key() {
  if [ -z "$API_KEY" ]; then
    if [ -f "$CONFIG_FILE" ]; then
      API_KEY=$(grep "^api_key=" "$CONFIG_FILE" | cut -d'=' -f2)
    fi

    if [ -z "$API_KEY" ]; then
      print_error "API key not found. Set WIA_API_KEY environment variable or create $CONFIG_FILE"
      exit 1
    fi
  fi
}

# Make API request
api_request() {
  local method="$1"
  local endpoint="$2"
  local data="$3"

  check_api_key

  local url="${API_ENDPOINT}${endpoint}"
  local response

  if [ "$method" = "GET" ]; then
    response=$(curl -s -X GET \
      -H "Authorization: Bearer ${API_KEY}" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-032" \
      -H "X-WIA-Version: 1.0.0" \
      "$url")
  else
    response=$(curl -s -X "$method" \
      -H "Authorization: Bearer ${API_KEY}" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-032" \
      -H "X-WIA-Version: 1.0.0" \
      -d "$data" \
      "$url")
  fi

  echo "$response"
}

################################################################################
# Command Functions
################################################################################

# Report fire detection
cmd_detect() {
  local latitude="$1"
  local longitude="$2"
  local method="${3:-VIIRS}"
  local confidence="${4:-95}"

  if [ -z "$latitude" ] || [ -z "$longitude" ]; then
    print_error "Usage: detect <latitude> <longitude> [method] [confidence]"
    exit 1
  fi

  print_info "Reporting fire detection at ($latitude, $longitude)..."

  local data=$(cat <<EOF
{
  "detectionMethod": "$method",
  "confidenceLevel": $confidence,
  "location": {
    "latitude": $latitude,
    "longitude": $longitude,
    "elevation": 0,
    "address": "",
    "forestType": "mixed",
    "administrativeArea": ""
  },
  "fireCharacteristics": {
    "frp": 0,
    "brightness": 0,
    "area": 0,
    "perimeter": 0,
    "fireLineIntensity": 0,
    "rateOfSpread": 0
  },
  "weatherConditions": {
    "temperature": 0,
    "humidity": 0,
    "windSpeed": 0,
    "windDirection": 0,
    "precipitation24h": 0
  },
  "riskAssessment": {
    "dangerLevel": 3,
    "fwi": 0,
    "threatToLife": "high",
    "threatToProperty": "high",
    "evacuationRequired": false
  },
  "metadata": {}
}
EOF
)

  local response=$(api_request "POST" "/api/v1/fire/detect" "$data")

  if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
    local event_id=$(echo "$response" | jq -r '.data.eventId')
    print_success "Fire detection reported successfully"
    print_fire "Event ID: $event_id"
    pretty_json "$response"
  else
    print_error "Failed to report fire detection"
    echo "$response"
    exit 1
  fi
}

# List active fires
cmd_list() {
  print_info "Fetching active fires..."

  local response=$(api_request "GET" "/api/v1/fire/active" "")

  if has_jq; then
    local count=$(echo "$response" | jq -r '.data | length')
    print_success "Found $count active fire(s)"

    echo ""
    echo -e "${CYAN}Active Fires:${NC}"
    echo "$response" | jq -r '.data[] | "🔥 \(.eventId) - \(.location.address) - Danger: \(.riskAssessment.dangerLevel)"'
    echo ""

    pretty_json "$response"
  else
    echo "$response"
  fi
}

# Request fire spread prediction
cmd_predict() {
  local fire_event_id="$1"
  local horizon="${2:-24}"

  if [ -z "$fire_event_id" ]; then
    print_error "Usage: predict <fire_event_id> [forecast_horizon_hours]"
    exit 1
  fi

  print_info "Requesting fire spread prediction for $fire_event_id (${horizon}h horizon)..."

  local data=$(cat <<EOF
{
  "fireEventId": "$fire_event_id",
  "forecastHorizon": $horizon
}
EOF
)

  local response=$(api_request "POST" "/api/v1/prediction/spread" "$data")

  if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
    print_success "Fire spread prediction generated"

    local pred_id=$(echo "$response" | jq -r '.data.predictionId')
    local current_area=$(echo "$response" | jq -r '.data.spreadPrediction.currentArea')
    local pred_24h=$(echo "$response" | jq -r '.data.spreadPrediction.predicted24h')
    local confidence=$(echo "$response" | jq -r '.data.spreadPrediction.confidence')

    echo ""
    print_fire "Prediction ID: $pred_id"
    echo -e "${YELLOW}Current Area:${NC} ${current_area} m²"
    echo -e "${ORANGE}Predicted 24h:${NC} ${pred_24h} m²"
    echo -e "${BLUE}Confidence:${NC} ${confidence}%"
    echo ""

    pretty_json "$response"
  else
    print_error "Failed to generate prediction"
    echo "$response"
    exit 1
  fi
}

# Get fire danger level
cmd_danger() {
  local region="$1"

  if [ -z "$region" ]; then
    print_error "Usage: danger <region>"
    exit 1
  fi

  print_info "Getting fire danger level for $region..."

  local response=$(api_request "GET" "/api/v1/danger/region/$region" "")

  if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
    local level=$(echo "$response" | jq -r '.data.dangerLevel')
    local fwi=$(echo "$response" | jq -r '.data.fwi')

    case $level in
      1)
        print_success "Fire Danger: 🟢 LOW (Level 1) - FWI: $fwi"
        ;;
      2)
        echo -e "${YELLOW}[⚠]${NC} Fire Danger: 🟡 MODERATE (Level 2) - FWI: $fwi"
        ;;
      3)
        echo -e "${ORANGE}[⚠]${NC} Fire Danger: 🟠 HIGH (Level 3) - FWI: $fwi"
        ;;
      4)
        echo -e "${RED}[⚠]${NC} Fire Danger: 🔴 VERY HIGH (Level 4) - FWI: $fwi"
        ;;
      5)
        echo -e "${PURPLE}[⚠]${NC} Fire Danger: 🟣 EXTREME (Level 5) - FWI: $fwi"
        ;;
    esac

    echo ""
    pretty_json "$response"
  else
    print_error "Failed to get fire danger level"
    echo "$response"
    exit 1
  fi
}

# Manage evacuation zones
cmd_evacuate() {
  local fire_event_id="$1"

  if [ -z "$fire_event_id" ]; then
    print_error "Usage: evacuate <fire_event_id>"
    exit 1
  fi

  print_info "Getting evacuation zones for fire event $fire_event_id..."

  local response=$(api_request "GET" "/api/v1/evacuation/zones/$fire_event_id" "")

  if has_jq; then
    local count=$(echo "$response" | jq -r '.data | length')
    print_success "Found $count evacuation zone(s)"

    echo ""
    echo -e "${CYAN}Evacuation Zones:${NC}"
    echo "$response" | jq -r '.data[] | "  \(.status | ascii_upcase): \(.name) - Population: \(.population)"'
    echo ""

    pretty_json "$response"
  else
    echo "$response"
  fi
}

# Manage firefighting resources
cmd_resources() {
  local action="${1:-list}"

  case $action in
    list)
      print_info "Fetching available firefighting resources..."
      local response=$(api_request "GET" "/api/v1/resources/available" "")

      if has_jq; then
        local count=$(echo "$response" | jq -r '.data | length')
        print_success "Found $count available resource(s)"

        echo ""
        echo -e "${CYAN}Available Resources:${NC}"
        echo "$response" | jq -r '.data[] | "  \(.type | ascii_upcase): \(.resourceId) - Status: \(.status)"'
        echo ""

        pretty_json "$response"
      else
        echo "$response"
      fi
      ;;

    track)
      local resource_id="$2"
      if [ -z "$resource_id" ]; then
        print_error "Usage: resources track <resource_id>"
        exit 1
      fi

      print_info "Tracking resource $resource_id..."
      local response=$(api_request "GET" "/api/v1/resources/track/$resource_id" "")

      if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
        local status=$(echo "$response" | jq -r '.data.status')
        local lat=$(echo "$response" | jq -r '.data.location.latitude')
        local lon=$(echo "$response" | jq -r '.data.location.longitude')

        print_success "Resource Status: $status"
        echo -e "${BLUE}Location:${NC} ($lat, $lon)"
        echo ""

        pretty_json "$response"
      else
        echo "$response"
      fi
      ;;

    *)
      print_error "Unknown action: $action"
      print_info "Available actions: list, track"
      exit 1
      ;;
  esac
}

# Send fire alert
cmd_alert() {
  local fire_event_id="$1"
  local level="${2:-3}"
  local message="${3:-Fire alert notification}"

  if [ -z "$fire_event_id" ]; then
    print_error "Usage: alert <fire_event_id> [level] [message]"
    exit 1
  fi

  print_info "Sending fire alert for event $fire_event_id..."

  local data=$(cat <<EOF
{
  "fireEventId": "$fire_event_id",
  "level": $level,
  "recipients": [],
  "message": "$message"
}
EOF
)

  local response=$(api_request "POST" "/api/v1/alert/send" "$data")

  if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
    local alert_id=$(echo "$response" | jq -r '.data.alertId')
    print_success "Fire alert sent successfully"
    print_fire "Alert ID: $alert_id"
    pretty_json "$response"
  else
    print_error "Failed to send fire alert"
    echo "$response"
    exit 1
  fi
}

# Get certification status
cmd_status() {
  print_info "Fetching certification status..."

  local response=$(api_request "GET" "/api/v1/certification/status" "")

  if has_jq && [ "$(echo "$response" | jq -r '.success')" = "true" ]; then
    local level=$(echo "$response" | jq -r '.data.level')
    local uptime=$(echo "$response" | jq -r '.data.uptime')
    local avg_detection=$(echo "$response" | jq -r '.data.avgDetectionTime')
    local false_positive=$(echo "$response" | jq -r '.data.falsePositiveRate')

    print_success "Certification Status"
    echo ""
    echo -e "${CYAN}Level:${NC} $level"
    echo -e "${CYAN}Uptime:${NC} $uptime%"
    echo -e "${CYAN}Avg Detection Time:${NC} $avg_detection minutes"
    echo -e "${CYAN}False Positive Rate:${NC} $false_positive%"
    echo ""

    pretty_json "$response"
  else
    print_error "Failed to get certification status"
    echo "$response"
    exit 1
  fi
}

# Show help
cmd_help() {
  print_header
  echo "Usage: forest-fire-detection.sh <command> [options]"
  echo ""
  echo "Commands:"
  echo "  detect <lat> <lon> [method] [confidence]    Report fire detection event"
  echo "  list                                         List active fires"
  echo "  predict <fire_event_id> [horizon]           Request fire spread prediction"
  echo "  danger <region>                              Get fire danger level"
  echo "  evacuate <fire_event_id>                     Get evacuation zones"
  echo "  resources [list|track] [resource_id]         Manage firefighting resources"
  echo "  alert <fire_event_id> [level] [message]      Send fire alert"
  echo "  status                                       Get certification status"
  echo "  help                                         Show this help"
  echo ""
  echo "Environment Variables:"
  echo "  WIA_API_KEY         API authentication key"
  echo "  WIA_API_ENDPOINT    API endpoint URL (default: https://api.wia.org/ene-032/v1)"
  echo ""
  echo "Configuration File:"
  echo "  ${CONFIG_FILE}"
  echo "  Format: api_key=YOUR_API_KEY"
  echo ""
  echo "Examples:"
  echo "  # Report fire detection"
  echo "  ./forest-fire-detection.sh detect 37.5665 126.9780 VIIRS 95"
  echo ""
  echo "  # List active fires"
  echo "  ./forest-fire-detection.sh list"
  echo ""
  echo "  # Request fire spread prediction"
  echo "  ./forest-fire-detection.sh predict FIRE-2025-KR-001234 24"
  echo ""
  echo "  # Get fire danger level"
  echo "  ./forest-fire-detection.sh danger \"강원도\""
  echo ""
  echo "  # Get evacuation zones"
  echo "  ./forest-fire-detection.sh evacuate FIRE-2025-KR-001234"
  echo ""
  echo "  # Track firefighting resource"
  echo "  ./forest-fire-detection.sh resources track HELI-02"
  echo ""
  echo "Version: $VERSION"
  echo "License: MIT"
  echo ""
  echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
  echo -e "${GREEN}         弘益人間 (홍익인간) · Benefit All Humanity${NC}"
  echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
  echo ""
}

################################################################################
# Main
################################################################################

# Parse command
COMMAND="${1:-help}"

case "$COMMAND" in
  detect)
    shift
    cmd_detect "$@"
    ;;
  list)
    cmd_list
    ;;
  predict)
    shift
    cmd_predict "$@"
    ;;
  danger)
    shift
    cmd_danger "$@"
    ;;
  evacuate)
    shift
    cmd_evacuate "$@"
    ;;
  resources)
    shift
    cmd_resources "$@"
    ;;
  alert)
    shift
    cmd_alert "$@"
    ;;
  status)
    cmd_status
    ;;
  help|--help|-h)
    cmd_help
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo "Run './forest-fire-detection.sh help' for usage information"
    exit 1
    ;;
esac
