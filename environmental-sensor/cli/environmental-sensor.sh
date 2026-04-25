#!/bin/bash

################################################################################
# WIA-ENE-035: Environmental Sensor Network Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-035
# Environmental Sensor Network Standard API.
#
# Usage:
#   environmental-sensor.sh <command> [options]
#
# Commands:
#   register-sensor      Register new sensor device
#   get-sensor           Get sensor by ID
#   list-sensors         List all sensors
#   submit-reading       Submit sensor reading
#   get-reading          Get latest reading
#   calibrate            Submit calibration record
#   schedule-maintenance Schedule maintenance
#   configure-alert      Configure alert thresholds
#   list-alerts          List active alerts
#   create-network       Create sensor network
#   generate-report      Generate network report
#   kpi                  View KPI dashboard
#   help                 Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-035/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/environmental-sensor.conf"

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
  echo "║      📡  WIA-ENE-035: Environmental Sensor CLI v${VERSION}        ║"
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
# WIA-ENE-035 Environmental Sensor CLI Configuration
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
      -H "X-WIA-Standard: ENE-035" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-035" \
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

cmd_register_sensor() {
  local sensor_id="${1:-}"
  local sensor_type="${2:-AIR_QUALITY}"
  local location="${3:-Unknown}"
  local protocol="${4:-LORAWAN}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 register-sensor --id SENSOR_ID --type TYPE --location LOCATION --protocol PROTOCOL"
    echo ""
    echo "Sensor Types: AIR_QUALITY, WATER_QUALITY, SOIL_MOISTURE, NOISE_LEVEL, RADIATION_GAMMA, TEMPERATURE, etc."
    echo "Protocols: LORAWAN, NB_IOT, SIGFOX, WIFI, MQTT"
    echo ""
    echo "Example:"
    echo "  $0 register-sensor --id SENSOR-001 --type AIR_QUALITY --location '서울시청' --protocol LORAWAN"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sensorId": "${sensor_id}",
  "sensorType": "${sensor_type}",
  "location": {
    "sensorId": "${sensor_id}",
    "siteName": "${location}",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "specs": {
    "manufacturer": "WIA Sensors",
    "model": "WIA-${sensor_type}-001",
    "serialNumber": "${sensor_id}",
    "firmwareVersion": "1.0.0",
    "measuringRange": {
      "min": 0,
      "max": 1000,
      "unit": "μg/m³"
    },
    "accuracy": 5,
    "resolution": 1,
    "responseTime": 60,
    "operatingTemperature": {
      "min": -20,
      "max": 50
    }
  },
  "power": {
    "powerSource": "battery",
    "batteryType": "Li-ion",
    "batteryCapacity": 5000,
    "batteryLevel": 100,
    "powerConsumption": 50
  },
  "communication": {
    "protocol": "${protocol}",
    "frequency": 868,
    "transmitPower": 14
  },
  "sampling": {
    "samplingInterval": 300,
    "transmissionInterval": 900,
    "samplesPerTransmission": 3,
    "enabledParameters": ["pm2_5", "pm10", "temperature", "humidity"]
  }
}
EOF
)

  print_info "Registering sensor: $sensor_id"
  local response=$(api_request "POST" "/api/v1/sensor/register" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    print_success "Sensor registered successfully"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to register sensor"
    echo "$response"
    exit 1
  fi
}

cmd_get_sensor() {
  local sensor_id="${1:-}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 get-sensor SENSOR_ID"
    exit 1
  fi

  print_info "Fetching sensor: $sensor_id"
  local response=$(api_request "GET" "/api/v1/sensor/$sensor_id")
  pretty_json "$response"
}

cmd_list_sensors() {
  local page="${1:-1}"
  local limit="${2:-10}"

  local path="/api/v1/sensors?page=$page&limit=$limit"

  print_info "Listing sensors..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_submit_reading() {
  local sensor_id="${1:-}"
  local pm25="${2:-50}"
  local pm10="${3:-100}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 submit-reading --sensor-id SENSOR_ID --pm25 VALUE --pm10 VALUE"
    echo ""
    echo "Example:"
    echo "  $0 submit-reading --sensor-id SENSOR-001 --pm25 35.5 --pm10 55.0"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sensorId": "${sensor_id}",
  "sensorType": "AIR_QUALITY",
  "airQuality": {
    "pm2_5": ${pm25},
    "pm10": ${pm10},
    "temperature": 22.5,
    "humidity": 60
  },
  "dataQuality": "GOOD",
  "batteryLevel": 85,
  "signalStrength": -75
}
EOF
)

  print_info "Submitting sensor reading..."
  local response=$(api_request "POST" "/api/v1/reading/submit" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local reading_id=$(echo "$response" | jq -r '.data.readingId')
    print_success "Reading submitted: $reading_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to submit reading"
    echo "$response"
    exit 1
  fi
}

cmd_get_reading() {
  local sensor_id="${1:-}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 get-reading --sensor-id SENSOR_ID"
    exit 1
  fi

  print_info "Fetching latest reading for: $sensor_id"
  local response=$(api_request "GET" "/api/v1/sensor/$sensor_id/latest")
  pretty_json "$response"
}

cmd_calibrate() {
  local sensor_id="${1:-}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 calibrate --sensor-id SENSOR_ID"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sensorId": "${sensor_id}",
  "calibrationType": "FIELD",
  "performedBy": "CLI-USER",
  "calibrationPoints": [
    {
      "referenceValue": 0,
      "measuredValue": 0.5,
      "unit": "μg/m³",
      "deviation": 0.5
    },
    {
      "referenceValue": 100,
      "measuredValue": 102,
      "unit": "μg/m³",
      "deviation": 2
    }
  ],
  "postCalibrationAccuracy": 98,
  "passed": true,
  "nextCalibrationDue": "$(date -d '+6 months' -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
)

  print_info "Submitting calibration record..."
  local response=$(api_request "POST" "/api/v1/calibration/submit" "$data")
  pretty_json "$response"
}

cmd_schedule_maintenance() {
  local sensor_id="${1:-}"
  local type="${2:-INSPECTION}"
  local date="${3:-$(date -d '+7 days' +%Y-%m-%d)}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 schedule-maintenance --sensor-id SENSOR_ID --type TYPE --date DATE"
    echo ""
    echo "Types: INSPECTION, CLEANING, CALIBRATION, BATTERY_REPLACEMENT, FIRMWARE_UPDATE, REPAIR"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sensorId": "${sensor_id}",
  "maintenanceType": "${type}",
  "scheduledDate": "${date}T09:00:00Z",
  "description": "${type} for sensor ${sensor_id}"
}
EOF
)

  print_info "Scheduling maintenance..."
  local response=$(api_request "POST" "/api/v1/maintenance/schedule" "$data")
  pretty_json "$response"
}

cmd_configure_alert() {
  local sensor_id="${1:-}"
  local parameter="${2:-pm2_5}"
  local warning="${3:-50}"
  local critical="${4:-100}"

  if [ -z "$sensor_id" ]; then
    echo "Usage: $0 configure-alert --sensor-id SENSOR_ID --parameter PARAM --warning VALUE --critical VALUE"
    echo ""
    echo "Example:"
    echo "  $0 configure-alert --sensor-id SENSOR-001 --parameter pm2_5 --warning 50 --critical 100"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sensorId": "${sensor_id}",
  "enabled": true,
  "thresholds": [
    {
      "parameter": "${parameter}",
      "unit": "μg/m³",
      "warningMax": ${warning},
      "criticalMax": ${critical}
    }
  ],
  "notificationChannels": ["email"],
  "recipients": ["admin@example.com"]
}
EOF
)

  print_info "Configuring alert..."
  local response=$(api_request "POST" "/api/v1/alert/configure" "$data")
  pretty_json "$response"
}

cmd_list_alerts() {
  local sensor_id="${1:-}"

  local path="/api/v1/alerts/active"
  if [ -n "$sensor_id" ]; then
    path="$path?sensorId=$sensor_id"
  fi

  print_info "Listing active alerts..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_create_network() {
  local network_name="${1:-}"
  local protocol="${2:-LORAWAN}"

  if [ -z "$network_name" ]; then
    echo "Usage: $0 create-network --name NAME --protocol PROTOCOL"
    echo ""
    echo "Example:"
    echo "  $0 create-network --name '서울 대기질 네트워크' --protocol LORAWAN"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "networkName": "${network_name}",
  "description": "Environmental sensor network",
  "organization": "WIA",
  "protocol": "${protocol}",
  "sensors": [],
  "totalSensors": 0,
  "activeSensors": 0,
  "offlineSensors": 0,
  "networkHealth": {
    "uptime": 100,
    "avgSignalStrength": -70,
    "packetLossRate": 0,
    "avgLatency": 100
  }
}
EOF
)

  print_info "Creating sensor network..."
  local response=$(api_request "POST" "/api/v1/network/create" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local network_id=$(echo "$response" | jq -r '.data.networkId')
    print_success "Network created: $network_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create network"
    echo "$response"
    exit 1
  fi
}

cmd_generate_report() {
  local network_id="${1:-}"
  local type="${2:-weekly}"
  local start_date="${3:-$(date -d '-7 days' +%Y-%m-%d)}"
  local end_date="${4:-$(date +%Y-%m-%d)}"

  if [ -z "$network_id" ]; then
    echo "Usage: $0 generate-report --network-id ID --type TYPE --start DATE --end DATE"
    echo ""
    echo "Types: daily, weekly, monthly, annual"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "networkId": "${network_id}",
  "reportType": "${type}",
  "dateRange": {
    "startDate": "${start_date}T00:00:00Z",
    "endDate": "${end_date}T23:59:59Z"
  }
}
EOF
)

  print_info "Generating report..."
  local response=$(api_request "POST" "/api/v1/report/generate" "$data")
  pretty_json "$response"
}

cmd_kpi() {
  local network_id="${1:-}"
  local date="${2:-$(date +%Y-%m-%d)}"

  local path="/api/v1/analytics/kpi"
  local params=""
  if [ -n "$network_id" ]; then
    params="networkId=$network_id"
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

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}register-sensor${NC} --id ID --type TYPE --location LOCATION --protocol PROTOCOL
    Register new sensor device
    Example: register-sensor --id SENSOR-001 --type AIR_QUALITY --location '서울시청'

  ${GREEN}get-sensor${NC} SENSOR_ID
    Get sensor information by ID

  ${GREEN}list-sensors${NC} [--page PAGE] [--limit LIMIT]
    List all sensors

  ${GREEN}submit-reading${NC} --sensor-id ID --pm25 VALUE --pm10 VALUE
    Submit sensor reading
    Example: submit-reading --sensor-id SENSOR-001 --pm25 35.5 --pm10 55.0

  ${GREEN}get-reading${NC} --sensor-id SENSOR_ID
    Get latest reading for sensor

  ${GREEN}calibrate${NC} --sensor-id SENSOR_ID
    Submit calibration record

  ${GREEN}schedule-maintenance${NC} --sensor-id ID --type TYPE --date DATE
    Schedule maintenance task

  ${GREEN}configure-alert${NC} --sensor-id ID --parameter PARAM --warning VALUE --critical VALUE
    Configure alert thresholds

  ${GREEN}list-alerts${NC} [--sensor-id SENSOR_ID]
    List active alerts

  ${GREEN}create-network${NC} --name NAME --protocol PROTOCOL
    Create sensor network

  ${GREEN}generate-report${NC} --network-id ID --type TYPE --start DATE --end DATE
    Generate network report

  ${GREEN}kpi${NC} [--network-id ID] [--date DATE]
    View KPI dashboard

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-035/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Register sensor
  $0 register-sensor --id SENSOR-001 --type AIR_QUALITY --location '서울시청' --protocol LORAWAN

  # Submit reading
  $0 submit-reading --sensor-id SENSOR-001 --pm25 35.5 --pm10 55.0

  # Configure alert
  $0 configure-alert --sensor-id SENSOR-001 --parameter pm2_5 --warning 50 --critical 100

  # Create network
  $0 create-network --name '서울 대기질 네트워크' --protocol LORAWAN

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-035
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-035

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
  register-sensor)
    SENSOR_ID=""
    SENSOR_TYPE="AIR_QUALITY"
    LOCATION="Unknown"
    PROTOCOL="LORAWAN"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --id) SENSOR_ID="$2"; shift 2 ;;
        --type) SENSOR_TYPE="$2"; shift 2 ;;
        --location) LOCATION="$2"; shift 2 ;;
        --protocol) PROTOCOL="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_register_sensor "$SENSOR_ID" "$SENSOR_TYPE" "$LOCATION" "$PROTOCOL"
    ;;
  get-sensor)
    cmd_get_sensor "$@"
    ;;
  list-sensors)
    PAGE="1"
    LIMIT="10"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --page) PAGE="$2"; shift 2 ;;
        --limit) LIMIT="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_sensors "$PAGE" "$LIMIT"
    ;;
  submit-reading)
    SENSOR_ID=""
    PM25="50"
    PM10="100"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        --pm25) PM25="$2"; shift 2 ;;
        --pm10) PM10="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_submit_reading "$SENSOR_ID" "$PM25" "$PM10"
    ;;
  get-reading)
    SENSOR_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_get_reading "$SENSOR_ID"
    ;;
  calibrate)
    SENSOR_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_calibrate "$SENSOR_ID"
    ;;
  schedule-maintenance)
    SENSOR_ID=""
    TYPE="INSPECTION"
    DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        --type) TYPE="$2"; shift 2 ;;
        --date) DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_schedule_maintenance "$SENSOR_ID" "$TYPE" "$DATE"
    ;;
  configure-alert)
    SENSOR_ID=""
    PARAMETER="pm2_5"
    WARNING="50"
    CRITICAL="100"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        --parameter) PARAMETER="$2"; shift 2 ;;
        --warning) WARNING="$2"; shift 2 ;;
        --critical) CRITICAL="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_configure_alert "$SENSOR_ID" "$PARAMETER" "$WARNING" "$CRITICAL"
    ;;
  list-alerts)
    SENSOR_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sensor-id) SENSOR_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_alerts "$SENSOR_ID"
    ;;
  create-network)
    NETWORK_NAME=""
    PROTOCOL="LORAWAN"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --name) NETWORK_NAME="$2"; shift 2 ;;
        --protocol) PROTOCOL="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_network "$NETWORK_NAME" "$PROTOCOL"
    ;;
  generate-report)
    NETWORK_ID=""
    TYPE="weekly"
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --network-id) NETWORK_ID="$2"; shift 2 ;;
        --type) TYPE="$2"; shift 2 ;;
        --start) START_DATE="$2"; shift 2 ;;
        --end) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_generate_report "$NETWORK_ID" "$TYPE" "$START_DATE" "$END_DATE"
    ;;
  kpi)
    NETWORK_ID=""
    DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --network-id) NETWORK_ID="$2"; shift 2 ;;
        --date) DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_kpi "$NETWORK_ID" "$DATE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-035 Environmental Sensor Network CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
