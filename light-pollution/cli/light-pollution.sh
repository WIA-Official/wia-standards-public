#!/bin/bash

################################################################################
# WIA-ENE-029: Light Pollution Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-029
# Light Pollution Standard API.
#
# Usage:
#   light-pollution.sh <command> [options]
#
# Commands:
#   submit-measurement    Submit light pollution measurement
#   get-measurement       Get measurement by ID
#   get-sky-brightness    Get sky brightness for location
#   register-fixture      Register lighting fixture
#   get-fixture           Get fixture by ID
#   zone-limits           Get zone lighting limits
#   create-alert          Create pollution alert
#   list-alerts           List active alerts
#   report                Generate performance report
#   compliance            Check compliance dashboard
#   help                  Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-029/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/light-pollution.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║       💡  WIA-ENE-029: Light Pollution CLI v${VERSION}          ║"
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

print_stars() {
  echo -e "${PURPLE}[★]${NC} $1"
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
# WIA-ENE-029 Light Pollution CLI Configuration
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
      -H "X-WIA-Standard: ENE-029" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-029" \
      -H "X-WIA-Version: 1.0.0" \
      -d "$data")
  fi

  echo "$response"
}

# Calculate Bortle scale from sky brightness
calculate_bortle() {
  local brightness="$1"

  if awk "BEGIN {exit !($brightness >= 21.7)}"; then echo "1"
  elif awk "BEGIN {exit !($brightness >= 21.5)}"; then echo "2"
  elif awk "BEGIN {exit !($brightness >= 21.3)}"; then echo "3"
  elif awk "BEGIN {exit !($brightness >= 20.5)}"; then echo "4"
  elif awk "BEGIN {exit !($brightness >= 19.5)}"; then echo "5"
  elif awk "BEGIN {exit !($brightness >= 18.5)}"; then echo "6"
  elif awk "BEGIN {exit !($brightness >= 18.0)}"; then echo "7"
  elif awk "BEGIN {exit !($brightness >= 17.0)}"; then echo "8"
  else echo "9"
  fi
}

# Estimate visible stars from Bortle
estimate_stars() {
  local bortle="$1"

  case $bortle in
    1) echo "5000+" ;;
    2) echo "4000" ;;
    3) echo "2500" ;;
    4) echo "1000" ;;
    5) echo "500" ;;
    6) echo "250" ;;
    7) echo "100" ;;
    8) echo "50" ;;
    9) echo "10" ;;
    *) echo "Unknown" ;;
  esac
}

# Display Bortle scale info
display_bortle_info() {
  local bortle="$1"
  local brightness="$2"

  echo ""
  print_stars "Sky Quality Assessment"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "  Bortle Scale:        $bortle / 9"
  echo "  Sky Brightness:      $brightness mag/arcsec²"
  echo "  Visible Stars:       $(estimate_stars $bortle)"

  case $bortle in
    1) echo "  Classification:      Excellent Dark Sky ⭐⭐⭐⭐⭐" ;;
    2) echo "  Classification:      Typical Dark Sky ⭐⭐⭐⭐" ;;
    3) echo "  Classification:      Rural Sky ⭐⭐⭐" ;;
    4) echo "  Classification:      Rural-Suburban Transition ⭐⭐" ;;
    5) echo "  Classification:      Suburban Sky ⭐" ;;
    6) echo "  Classification:      Bright Suburban Sky 🌃" ;;
    7) echo "  Classification:      Suburban-Urban Transition 🌆" ;;
    8) echo "  Classification:      City Sky 🌇" ;;
    9) echo "  Classification:      Inner-City Sky 🏙️" ;;
  esac
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo ""
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

cmd_submit_measurement() {
  local lat="${1:-}"
  local lon="${2:-}"
  local sky_brightness="${3:-}"
  local illuminance="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 submit-measurement --lat LAT --lon LON --sky-brightness VALUE [--illuminance VALUE]"
    echo ""
    echo "Example:"
    echo "  $0 submit-measurement --lat 37.5665 --lon 126.9780 --sky-brightness 18.5 --illuminance 5.2"
    exit 1
  fi

  # Calculate Bortle scale
  local bortle=$(calculate_bortle "$sky_brightness")
  local visible_stars=$(estimate_stars "$bortle")

  local data=$(cat <<EOF
{
  "location": {
    "coordinates": {
      "latitude": ${lat},
      "longitude": ${lon}
    }
  },
  "skyBrightness": {
    "zenith": ${sky_brightness},
    "bortle": ${bortle},
    "sqm": ${sky_brightness},
    "visibleStars": ${visible_stars}
  },
EOF
)

  if [ -n "$illuminance" ]; then
    data+=$(cat <<EOF
  "illuminance": {
    "horizontal": ${illuminance},
    "unit": "lux"
  },
EOF
)
  fi

  data+=$(cat <<EOF
  "metadata": {
    "deviceId": "CLI",
    "deviceType": "Manual Entry",
    "operator": "CLI User",
    "weather": {
      "cloudCover": 0,
      "visibility": 20,
      "moonPhase": 0.15,
      "precipitation": false,
      "temperature": 20,
      "humidity": 50
    },
    "dataQuality": 80,
    "verified": false
  }
}
EOF
)

  print_info "Submitting measurement..."
  local response=$(api_request "POST" "/api/v1/measurements" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local measurement_id=$(echo "$response" | jq -r '.data.measurementId')
    print_success "Measurement submitted: $measurement_id"
    display_bortle_info "$bortle" "$sky_brightness"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to submit measurement"
    echo "$response"
    exit 1
  fi
}

cmd_get_measurement() {
  local measurement_id="${1:-}"

  if [ -z "$measurement_id" ]; then
    echo "Usage: $0 get-measurement MEASUREMENT_ID"
    exit 1
  fi

  print_info "Fetching measurement: $measurement_id"
  local response=$(api_request "GET" "/api/v1/measurements/$measurement_id")

  pretty_json "$response"
}

cmd_get_sky_brightness() {
  local lat="${1:-}"
  local lon="${2:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 get-sky-brightness --lat LAT --lon LON"
    exit 1
  fi

  print_info "Fetching sky brightness for location: $lat, $lon"
  local response=$(api_request "GET" "/api/v1/measurements/sky-brightness?lat=$lat&lon=$lon")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local brightness=$(echo "$response" | jq -r '.data.value')
    local bortle=$(calculate_bortle "$brightness")

    display_bortle_info "$bortle" "$brightness"
    pretty_json "$response"
  else
    print_error "Failed to fetch sky brightness"
    echo "$response"
  fi
}

cmd_register_fixture() {
  local type="${1:-}"
  local wattage="${2:-}"
  local color_temp="${3:-}"
  local zone="${4:-}"

  if [ -z "$type" ] || [ -z "$wattage" ] || [ -z "$color_temp" ] || [ -z "$zone" ]; then
    echo "Usage: $0 register-fixture --type TYPE --wattage WATTS --color-temp KELVIN --zone ZONE"
    echo ""
    echo "Types: street_light, area_light, wall_pack, flood_light, decorative, sports_field"
    echo "Zones: E1, E2, E3, E4, E5"
    echo ""
    echo "Example:"
    echo "  $0 register-fixture --type street_light --wattage 50 --color-temp 2700 --zone E4"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "type": "${type}",
  "location": {
    "address": "Unknown",
    "coordinates": {
      "latitude": 0,
      "longitude": 0
    },
    "zone": "${zone}"
  },
  "specifications": {
    "manufacturer": "Unknown",
    "model": "CLI Entry",
    "wattage": ${wattage},
    "lumens": $((wattage * 100)),
    "efficacy": 100,
    "colorTemperature": ${color_temp},
    "cri": 80,
    "beamAngle": 120,
    "cutoffAngle": 90,
    "shielding": "fully_cutoff",
    "upwardLightRatio": 2.5,
    "ratedLife": 50000,
    "l70": 50000
  },
  "controls": {
    "dimming": true,
    "motionSensor": false,
    "lightSensor": true,
    "timer": false,
    "centralControl": false,
    "smartControl": false
  }
}
EOF
)

  print_info "Registering lighting fixture..."
  local response=$(api_request "POST" "/api/v1/fixtures" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local fixture_id=$(echo "$response" | jq -r '.data.fixtureId')
    print_success "Fixture registered: $fixture_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to register fixture"
    echo "$response"
    exit 1
  fi
}

cmd_get_fixture() {
  local fixture_id="${1:-}"

  if [ -z "$fixture_id" ]; then
    echo "Usage: $0 get-fixture FIXTURE_ID"
    exit 1
  fi

  print_info "Fetching fixture: $fixture_id"
  local response=$(api_request "GET" "/api/v1/fixtures/$fixture_id")

  pretty_json "$response"
}

cmd_zone_limits() {
  local zone="${1:-}"

  if [ -z "$zone" ]; then
    echo "Usage: $0 zone-limits --zone ZONE"
    echo ""
    echo "Zones: E1, E2, E3, E4, E5"
    echo ""
    echo "  E1 - Intrinsically Dark (National Parks, Observatories)"
    echo "  E2 - Dark (Rural Areas)"
    echo "  E3 - Rural"
    echo "  E4 - Suburban"
    echo "  E5 - Urban"
    exit 1
  fi

  print_info "Fetching lighting limits for zone: $zone"
  local response=$(api_request "GET" "/api/v1/zones/$zone/limits")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    echo ""
    echo -e "${CYAN}Zone ${zone} Lighting Limits${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "$response" | jq -r '.data | "  Max Vertical Illuminance:  \(.maxVerticalIlluminance) lux
  Max Upward Light Ratio:    \(.maxUpwardLightRatio)%
  Max Color Temperature:     \(.maxColorTemperature)K
  Curfew Required:           \(.curfewRequired)
  Curfew Hours:              \(.curfewHours // "N/A")"'
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch zone limits"
    echo "$response"
  fi
}

cmd_create_alert() {
  local type="${1:-}"
  local severity="${2:-}"
  local lat="${3:-}"
  local lon="${4:-}"

  if [ -z "$type" ] || [ -z "$severity" ] || [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 create-alert --type TYPE --severity SEVERITY --lat LAT --lon LON"
    echo ""
    echo "Types: excessive_sky_glow, light_trespass_complaint, glare_hazard, wildlife_threat, curfew_violation"
    echo "Severity: minimal, low, moderate, high, severe, extreme"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "location": {
    "coordinates": {
      "latitude": ${lat},
      "longitude": ${lon}
    }
  },
  "alertType": "${type}",
  "severity": "${severity}",
  "measurement": {
    "measurementId": "CLI-ALERT",
    "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "location": {
      "coordinates": {
        "latitude": ${lat},
        "longitude": ${lon}
      }
    },
    "metadata": {
      "deviceId": "CLI",
      "deviceType": "Manual Report",
      "dataQuality": 80,
      "verified": false
    }
  },
  "recommendations": ["Investigate the source", "Implement mitigation measures"]
}
EOF
)

  print_info "Creating alert..."
  local response=$(api_request "POST" "/api/v1/alerts" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local alert_id=$(echo "$response" | jq -r '.data.alertId')
    print_warning "Alert created: $alert_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create alert"
    echo "$response"
    exit 1
  fi
}

cmd_list_alerts() {
  local type="${1:-}"
  local severity="${2:-}"

  local path="/api/v1/alerts?"
  [ -n "$type" ] && path+="type=$type&"
  [ -n "$severity" ] && path+="severity=$severity&"

  print_info "Fetching alerts..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_report() {
  local region="${1:-}"
  local start_date="${2:-}"
  local end_date="${3:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 report --region REGION --start-date YYYY-MM-DD --end-date YYYY-MM-DD"
    echo ""
    echo "Example:"
    echo "  $0 report --region \"Seoul\" --start-date 2025-01-01 --end-date 2025-12-31"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "reportType": "annual",
  "region": "${region}",
  "period": {
    "start": "${start_date}T00:00:00Z",
    "end": "${end_date}T23:59:59Z"
  }
}
EOF
)

  print_info "Generating performance report for: $region"
  local response=$(api_request "POST" "/api/v1/analytics/report" "$data")

  pretty_json "$response"
}

cmd_compliance() {
  local region="${1:-}"
  local zone="${2:-}"

  local path="/api/v1/analytics/compliance?region=$region"
  [ -n "$zone" ] && path+="&zone=$zone"

  print_info "Fetching compliance dashboard..."
  local response=$(api_request "GET" "$path")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    echo ""
    echo -e "${CYAN}Compliance Dashboard${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "$response" | jq -r '.data | "  Total Fixtures:        \(.totalFixtures)
  Compliant Fixtures:    \(.compliantFixtures)
  Violations:            \(.violations)
  Compliance Rate:       \(.complianceRate)%"'
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to fetch compliance dashboard"
    echo "$response"
  fi
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}submit-measurement${NC} --lat LAT --lon LON --sky-brightness VALUE [--illuminance VALUE]
    Submit light pollution measurement
    Sky Brightness: mag/arcsec² (15-22, higher = darker)
    Illuminance: lux (optional)

  ${GREEN}get-measurement${NC} MEASUREMENT_ID
    Get measurement by ID

  ${GREEN}get-sky-brightness${NC} --lat LAT --lon LON
    Get current sky brightness for location

  ${GREEN}register-fixture${NC} --type TYPE --wattage WATTS --color-temp KELVIN --zone ZONE
    Register a lighting fixture
    Types: street_light, area_light, wall_pack, flood_light, decorative, sports_field
    Zones: E1, E2, E3, E4, E5

  ${GREEN}get-fixture${NC} FIXTURE_ID
    Get fixture information

  ${GREEN}zone-limits${NC} --zone ZONE
    Get lighting limits for a Dark Sky Zone
    Zones: E1 (darkest) to E5 (urban)

  ${GREEN}create-alert${NC} --type TYPE --severity SEVERITY --lat LAT --lon LON
    Create a light pollution alert
    Types: excessive_sky_glow, light_trespass_complaint, glare_hazard
    Severity: minimal, low, moderate, high, severe, extreme

  ${GREEN}list-alerts${NC} [--type TYPE] [--severity SEVERITY]
    List active alerts

  ${GREEN}report${NC} --region REGION --start-date DATE --end-date DATE
    Generate performance report

  ${GREEN}compliance${NC} --region REGION [--zone ZONE]
    View compliance dashboard

  ${GREEN}help${NC}
    Show this help

${CYAN}Dark Sky Zones:${NC}

  ${PURPLE}E1${NC} - Intrinsically Dark (National Parks, Observatories)
       Max Illuminance: 0.1 lux | Max Color Temp: 2200K | Curfew: 22:00-06:00

  ${PURPLE}E2${NC} - Dark (Rural Areas)
       Max Illuminance: 1 lux | Max Color Temp: 2700K | Curfew: 23:00-06:00

  ${PURPLE}E3${NC} - Rural
       Max Illuminance: 2 lux | Max Color Temp: 3000K | No Curfew

  ${PURPLE}E4${NC} - Suburban
       Max Illuminance: 5 lux | Max Color Temp: 4000K | No Curfew

  ${PURPLE}E5${NC} - Urban
       Max Illuminance: 10 lux | Max Color Temp: 5000K | No Curfew

${CYAN}Bortle Dark Sky Scale:${NC}

  1-2: Excellent/Typical Dark Sky (5000+ stars) ⭐⭐⭐⭐⭐
  3-4: Rural Sky (1000-2500 stars) ⭐⭐⭐
  5-6: Suburban Sky (250-500 stars) ⭐⭐
  7-8: City Sky (50-100 stars) ⭐
  9:   Inner-City Sky (10 stars) 🏙️

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-029/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Submit sky brightness measurement
  $0 submit-measurement --lat 37.5665 --lon 126.9780 --sky-brightness 18.5 --illuminance 5.2

  # Check zone limits
  $0 zone-limits --zone E2

  # Register a street light
  $0 register-fixture --type street_light --wattage 50 --color-temp 2700 --zone E4

  # Generate annual report
  $0 report --region "Seoul" --start-date 2025-01-01 --end-date 2025-12-31

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-029
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-029
  IDA: https://darksky.org

${BLUE}════════════════════════════════════════════════════════════════${NC}
${PURPLE}         弘益人間 (홍익인간) · Benefit All Humanity${NC}
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
  submit-measurement)
    LAT=""
    LON=""
    SKY_BRIGHTNESS=""
    ILLUMINANCE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --lat) LAT="$2"; shift 2 ;;
        --lon) LON="$2"; shift 2 ;;
        --sky-brightness) SKY_BRIGHTNESS="$2"; shift 2 ;;
        --illuminance) ILLUMINANCE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_submit_measurement "$LAT" "$LON" "$SKY_BRIGHTNESS" "$ILLUMINANCE"
    ;;
  get-measurement)
    cmd_get_measurement "$@"
    ;;
  get-sky-brightness)
    LAT=""
    LON=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --lat) LAT="$2"; shift 2 ;;
        --lon) LON="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_get_sky_brightness "$LAT" "$LON"
    ;;
  register-fixture)
    TYPE=""
    WATTAGE=""
    COLOR_TEMP=""
    ZONE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --wattage) WATTAGE="$2"; shift 2 ;;
        --color-temp) COLOR_TEMP="$2"; shift 2 ;;
        --zone) ZONE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_register_fixture "$TYPE" "$WATTAGE" "$COLOR_TEMP" "$ZONE"
    ;;
  get-fixture)
    cmd_get_fixture "$@"
    ;;
  zone-limits)
    ZONE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --zone) ZONE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_zone_limits "$ZONE"
    ;;
  create-alert)
    TYPE=""
    SEVERITY=""
    LAT=""
    LON=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --severity) SEVERITY="$2"; shift 2 ;;
        --lat) LAT="$2"; shift 2 ;;
        --lon) LON="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_alert "$TYPE" "$SEVERITY" "$LAT" "$LON"
    ;;
  list-alerts)
    TYPE=""
    SEVERITY=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --severity) SEVERITY="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_alerts "$TYPE" "$SEVERITY"
    ;;
  report)
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
    cmd_report "$REGION" "$START_DATE" "$END_DATE"
    ;;
  compliance)
    REGION=""
    ZONE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --zone) ZONE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_compliance "$REGION" "$ZONE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-029 Light Pollution CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
