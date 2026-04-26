#!/bin/bash

################################################################################
# WIA-ENE-033: Flood Prediction Standard - CLI Tool
#
# 弘益人間 (홍익人間) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-033
# Flood Prediction Standard API.
#
# Usage:
#   flood-prediction.sh <command> [options]
#
# Commands:
#   predict-risk     Predict flood risk for a location
#   monitor-river    Monitor river water levels
#   monitor-dam      Monitor dam/reservoir status
#   monitor-rainfall Monitor rainfall data
#   create-alert     Create flood alert
#   get-alerts       Get active flood alerts
#   evacuation-route Get evacuation routes
#   shelter-status   Get shelter capacity status
#   dashboard        View monitoring dashboard
#   kpi              View KPI metrics
#   report           Generate performance report
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-033/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/flood-prediction.conf"

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
  echo "║         🌊  WIA-ENE-033: Flood Prediction CLI v${VERSION}       ║"
  echo "║                                                                ║"
  echo "║              弘익人間 · Benefit All Humanity                   ║"
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
# WIA-ENE-033 Flood Prediction CLI Configuration
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
      -H "X-WIA-Standard: ENE-033" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-033" \
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

cmd_predict_risk() {
  local region="${1:-}"
  local horizon="${2:-24h}"

  if [ -z "$region" ]; then
    echo "Usage: $0 predict-risk --region REGION [--horizon HORIZON]"
    echo ""
    echo "Horizons: 1h, 6h, 24h, 72h (default: 24h)"
    echo ""
    echo "Example:"
    echo "  $0 predict-risk --region \"서울특별시 강남구\" --horizon 24h"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "location": {
    "regionId": "AUTO",
    "name": "${region}",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "timeHorizon": "${horizon}",
  "includePrecipitation": true,
  "includeRiverLevels": true,
  "includeDamStatus": true,
  "includeDrainage": true
}
EOF
)

  print_info "Predicting flood risk for: $region (${horizon})"
  local response=$(api_request "POST" "/api/v1/flood/predict" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local prediction_id=$(echo "$response" | jq -r '.data.predictionId')
    local risk_level=$(echo "$response" | jq -r '.data.riskLevel')
    local probability=$(echo "$response" | jq -r '.data.probability')

    print_success "Prediction ID: $prediction_id"
    echo ""
    echo -e "${CYAN}Risk Level:${NC} $risk_level"
    echo -e "${CYAN}Probability:${NC} ${probability}%"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to predict flood risk"
    echo "$response"
    exit 1
  fi
}

cmd_monitor_river() {
  local river_id="${1:-}"
  local threshold="${2:-}"

  if [ -z "$river_id" ]; then
    echo "Usage: $0 monitor-river --river-id RIVER_ID [--threshold LEVEL]"
    echo ""
    echo "Example:"
    echo "  $0 monitor-river --river-id \"HAN-001\" --threshold 5.0"
    exit 1
  fi

  print_info "Monitoring river: $river_id"
  local response=$(api_request "GET" "/api/v1/monitoring/rivers/$river_id")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local current=$(echo "$response" | jq -r '.data.current.waterLevel')
    local warning=$(echo "$response" | jq -r '.data.levels.warning')
    local danger=$(echo "$response" | jq -r '.data.levels.danger')
    local status=$(echo "$response" | jq -r '.data.status.condition')

    echo ""
    echo -e "${CYAN}Current Level:${NC} ${current}m"
    echo -e "${CYAN}Warning Level:${NC} ${warning}m"
    echo -e "${CYAN}Danger Level:${NC} ${danger}m"
    echo -e "${CYAN}Status:${NC} $status"
    echo ""

    if [ -n "$threshold" ] && (( $(echo "$current >= $threshold" | bc -l) )); then
      print_warning "⚠️  Water level (${current}m) exceeds threshold (${threshold}m)!"
    fi

    pretty_json "$response"
  else
    print_error "Failed to fetch river data"
    echo "$response"
    exit 1
  fi
}

cmd_monitor_dam() {
  local dam_id="${1:-}"

  if [ -z "$dam_id" ]; then
    echo "Usage: $0 monitor-dam --dam-id DAM_ID"
    echo ""
    echo "Example:"
    echo "  $0 monitor-dam --dam-id \"SOYANG-001\""
    exit 1
  fi

  print_info "Monitoring dam: $dam_id"
  local response=$(api_request "GET" "/api/v1/monitoring/dams/$dam_id")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local current=$(echo "$response" | jq -r '.data.waterLevel.current')
    local capacity=$(echo "$response" | jq -r '.data.storage.percentage')
    local discharge=$(echo "$response" | jq -r '.data.flow.discharge')

    echo ""
    echo -e "${CYAN}Water Level:${NC} ${current}m"
    echo -e "${CYAN}Capacity:${NC} ${capacity}%"
    echo -e "${CYAN}Discharge:${NC} ${discharge}m³/s"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch dam data"
    echo "$response"
    exit 1
  fi
}

cmd_create_alert() {
  local region="${1:-}"
  local risk_level="${2:-LEVEL-3}"
  local message="${3:-홍수 위험 경보}"

  if [ -z "$region" ]; then
    echo "Usage: $0 create-alert --region REGION [--level LEVEL] [--message MESSAGE]"
    echo ""
    echo "Levels: LEVEL-1, LEVEL-2, LEVEL-3, LEVEL-4"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "issuer": {
    "organizationId": "CLI-USER",
    "name": "CLI Tool",
    "authorityLevel": "local"
  },
  "alert": {
    "level": "${risk_level}",
    "severity": "severe",
    "urgency": "immediate",
    "certainty": "likely"
  },
  "affectedAreas": {
    "regionIds": ["${region}"],
    "geoJson": {},
    "population": 0,
    "buildings": 0
  },
  "event": {
    "type": "river-flood",
    "onset": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "expiry": "$(date -u -d '+6 hours' +%Y-%m-%dT%H:%M:%SZ)",
    "peakTime": "$(date -u -d '+3 hours' +%Y-%m-%dT%H:%M:%SZ)"
  },
  "instructions": {
    "ko": "${message}",
    "en": "Flood Alert",
    "actions": ["대피 준비"],
    "evacuationRequired": true,
    "evacuationZones": ["${region}"]
  },
  "contact": {
    "emergency": "119",
    "information": "1588-0000",
    "website": "https://disaster.go.kr"
  }
}
EOF
)

  print_info "Creating flood alert for: $region"
  local response=$(api_request "POST" "/api/v1/alert/create" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local alert_id=$(echo "$response" | jq -r '.data.alertId')
    print_success "Alert created: $alert_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create alert"
    echo "$response"
    exit 1
  fi
}

cmd_get_alerts() {
  local region="${1:-}"

  local path="/api/v1/alert/active"
  if [ -n "$region" ]; then
    path="$path?regionId=$region"
  fi

  print_info "Fetching active alerts..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_evacuation_route() {
  local location="${1:-}"
  local dest_type="${2:-shelter}"

  if [ -z "$location" ]; then
    echo "Usage: $0 evacuation-route --location LOCATION [--destination-type TYPE]"
    echo ""
    echo "Types: shelter, high-ground"
    echo ""
    echo "Example:"
    echo "  $0 evacuation-route --location \"서울특별시 서초구\" --destination-type shelter"
    exit 1
  fi

  local path="/api/v1/evacuation/routes?latitude=37.5665&longitude=126.9780&type=$dest_type"

  print_info "Finding evacuation routes from: $location"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_shelter_status() {
  local region="${1:-}"

  local path="/api/v1/evacuation/shelters"
  if [ -n "$region" ]; then
    path="$path?regionId=$region"
  fi

  print_info "Fetching shelter status..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_dashboard() {
  local region="${1:-}"

  local path="/api/v1/monitoring/dashboard"
  if [ -n "$region" ]; then
    path="$path?regionId=$region"
  fi

  print_info "Fetching monitoring dashboard..."
  local response=$(api_request "GET" "$path")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    echo ""
    echo -e "${CYAN}Active Alerts:${NC} $(echo "$response" | jq -r '.data.overview.activeAlerts')"
    echo -e "${CYAN}High Risk Areas:${NC} $(echo "$response" | jq -r '.data.overview.highRiskAreas')"
    echo -e "${CYAN}Evacuated Population:${NC} $(echo "$response" | jq -r '.data.overview.evacuatedPopulation')"
    echo -e "${CYAN}Operational Shelters:${NC} $(echo "$response" | jq -r '.data.overview.operationalShelters')"
    echo ""
  fi

  pretty_json "$response"
}

cmd_kpi() {
  local region="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  local path="/api/v1/analytics/kpi"
  local params=""

  if [ -n "$region" ]; then
    params="regionId=$region"
  fi
  if [ -n "$start_date" ]; then
    if [ -n "$params" ]; then
      params="$params&startDate=$start_date"
    else
      params="startDate=$start_date"
    fi
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

  print_info "Fetching KPI dashboard..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}predict-risk${NC} --region REGION [--horizon HORIZON]
    Predict flood risk for a location
    Horizons: 1h, 6h, 24h, 72h

  ${GREEN}monitor-river${NC} --river-id RIVER_ID [--threshold LEVEL]
    Monitor river water levels

  ${GREEN}monitor-dam${NC} --dam-id DAM_ID
    Monitor dam/reservoir status

  ${GREEN}create-alert${NC} --region REGION [--level LEVEL] [--message MESSAGE]
    Create flood alert
    Levels: LEVEL-1, LEVEL-2, LEVEL-3, LEVEL-4

  ${GREEN}get-alerts${NC} [--region REGION]
    Get active flood alerts

  ${GREEN}evacuation-route${NC} --location LOCATION [--destination-type TYPE]
    Get evacuation routes
    Types: shelter, high-ground

  ${GREEN}shelter-status${NC} [--region REGION]
    Get shelter capacity status

  ${GREEN}dashboard${NC} [--region REGION]
    View monitoring dashboard

  ${GREEN}kpi${NC} [--region REGION] [--start-date DATE] [--end-date DATE]
    View KPI metrics

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-033/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Predict flood risk
  $0 predict-risk --region "서울특별시 강남구" --horizon 24h

  # Monitor river
  $0 monitor-river --river-id "HAN-001" --threshold 5.0

  # Create alert
  $0 create-alert --region "서울특별시" --level LEVEL-3

  # Get evacuation routes
  $0 evacuation-route --location "서울특별시 서초구" --destination-type shelter

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-033
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-033

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
  predict-risk)
    REGION=""
    HORIZON="24h"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --horizon) HORIZON="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_predict_risk "$REGION" "$HORIZON"
    ;;
  monitor-river)
    RIVER_ID=""
    THRESHOLD=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --river-id) RIVER_ID="$2"; shift 2 ;;
        --threshold) THRESHOLD="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_monitor_river "$RIVER_ID" "$THRESHOLD"
    ;;
  monitor-dam)
    DAM_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --dam-id) DAM_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_monitor_dam "$DAM_ID"
    ;;
  create-alert)
    REGION=""
    LEVEL="LEVEL-3"
    MESSAGE="홍수 위험 경보"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --level) LEVEL="$2"; shift 2 ;;
        --message) MESSAGE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_alert "$REGION" "$LEVEL" "$MESSAGE"
    ;;
  get-alerts)
    REGION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_get_alerts "$REGION"
    ;;
  evacuation-route)
    LOCATION=""
    DEST_TYPE="shelter"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --location) LOCATION="$2"; shift 2 ;;
        --destination-type) DEST_TYPE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_evacuation_route "$LOCATION" "$DEST_TYPE"
    ;;
  shelter-status)
    REGION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_shelter_status "$REGION"
    ;;
  dashboard)
    REGION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_dashboard "$REGION"
    ;;
  kpi)
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
    cmd_kpi "$REGION" "$START_DATE" "$END_DATE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-033 Flood Prediction CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
