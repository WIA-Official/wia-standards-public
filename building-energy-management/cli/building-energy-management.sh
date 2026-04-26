#!/bin/bash

################################################################################
# WIA-CITY-011: Building Energy Management Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-011
# Building Energy Management Standard API.
#
# Usage:
#   building-energy-management.sh <command> [options]
#
# Commands:
#   dashboard           View real-time dashboard
#   energy              View energy consumption
#   hvac-status         Get HVAC system status
#   hvac-control        Control HVAC setpoint
#   lighting-control    Control lighting
#   solar               Get solar PV data
#   ess-status          Get ESS status
#   ess-control         Control ESS operation
#   peak-shaving        View/configure peak shaving
#   demand-response     View demand response status
#   carbon              View carbon emissions
#   benchmark           Compare with peer buildings
#   certification       Get certification status
#   forecast            Get energy forecast
#   alerts              View active alerts
#   help                Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/city-011/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/building-energy-management.conf"

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
  echo "║       ⚡  WIA-CITY-011: Building Energy Management CLI       ║"
  echo "║                         v${VERSION}                              ║"
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

print_metric() {
  local label="$1"
  local value="$2"
  local unit="$3"
  echo -e "${CYAN}${label}:${NC} ${value} ${unit}"
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
# WIA-CITY-011 Building Energy Management CLI Configuration
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
      -H "X-WIA-Standard: CITY-011" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: CITY-011" \
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

cmd_dashboard() {
  local building_id="${1:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 dashboard --building-id BUILDING_ID"
    echo ""
    echo "Example:"
    echo "  $0 dashboard --building-id BLD-SEOUL-001"
    exit 1
  fi

  print_header
  print_info "Fetching real-time dashboard for building: $building_id"
  echo ""

  response=$(api_request GET "/api/v1/dashboard/${building_id}/realtime")

  if has_jq; then
    current_demand=$(echo "$response" | jq -r '.data.energy.currentDemand')
    today_consumption=$(echo "$response" | jq -r '.data.energy.todayConsumption')
    today_cost=$(echo "$response" | jq -r '.data.cost.todayCost')
    today_carbon=$(echo "$response" | jq -r '.data.carbon.todayEmissions')
    solar_generation=$(echo "$response" | jq -r '.data.renewable.solarGeneration')
    avg_temp=$(echo "$response" | jq -r '.data.comfort.averageTemperature')

    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}ENERGY${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  Current Demand    " "$current_demand" "kW"
    print_metric "  Today Consumption " "$today_consumption" "kWh"
    print_metric "  Solar Generation  " "$solar_generation" "kW"
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}COST & CARBON${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  Today Cost        " "$today_cost" "KRW"
    print_metric "  Today CO₂         " "$today_carbon" "kg"
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}COMFORT${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  Average Temp      " "$avg_temp" "°C"
  else
    pretty_json "$response"
  fi
}

cmd_energy() {
  local building_id="${1:-}"
  local meter_type="${2:-electricity}"
  local period="${3:-today}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 energy --building-id BUILDING_ID [--type TYPE] [--period PERIOD]"
    echo ""
    echo "Types: electricity, gas, water, district-heating"
    echo "Periods: today, week, month, year"
    echo ""
    echo "Example:"
    echo "  $0 energy --building-id BLD-SEOUL-001 --type electricity --period today"
    exit 1
  fi

  print_header
  print_info "Energy consumption: $meter_type ($period)"
  echo ""

  # Calculate date range based on period
  case "$period" in
    today)
      start_date=$(date -u +"%Y-%m-%dT00:00:00Z")
      end_date=$(date -u +"%Y-%m-%dT23:59:59Z")
      ;;
    week)
      start_date=$(date -u -d "7 days ago" +"%Y-%m-%dT00:00:00Z")
      end_date=$(date -u +"%Y-%m-%dT23:59:59Z")
      ;;
    month)
      start_date=$(date -u -d "30 days ago" +"%Y-%m-%dT00:00:00Z")
      end_date=$(date -u +"%Y-%m-%dT23:59:59Z")
      ;;
    year)
      start_date=$(date -u -d "365 days ago" +"%Y-%m-%dT00:00:00Z")
      end_date=$(date -u +"%Y-%m-%dT23:59:59Z")
      ;;
  esac

  response=$(api_request GET "/api/v1/energy/consumption?buildingId=${building_id}&meterType=${meter_type}&startDate=${start_date}&endDate=${end_date}&aggregation=daily")

  pretty_json "$response"
}

cmd_hvac_status() {
  local system_id="${1:-}"

  if [ -z "$system_id" ]; then
    echo "Usage: $0 hvac-status --system-id SYSTEM_ID"
    echo ""
    echo "Example:"
    echo "  $0 hvac-status --system-id HVAC-AHU-001"
    exit 1
  fi

  print_header
  print_info "HVAC system status: $system_id"
  echo ""

  response=$(api_request GET "/api/v1/hvac/${system_id}/status")

  if has_jq; then
    status=$(echo "$response" | jq -r '.data.operationalStatus')
    mode=$(echo "$response" | jq -r '.data.controlMode')
    supply_temp=$(echo "$response" | jq -r '.data.temperature.supplyAir')
    return_temp=$(echo "$response" | jq -r '.data.temperature.returnAir')
    power=$(echo "$response" | jq -r '.data.energyConsumption.current')

    echo -e "${CYAN}Status:${NC} $status"
    echo -e "${CYAN}Control Mode:${NC} $mode"
    print_metric "Supply Air Temp" "$supply_temp" "°C"
    print_metric "Return Air Temp" "$return_temp" "°C"
    print_metric "Power Consumption" "$power" "kW"
  else
    pretty_json "$response"
  fi
}

cmd_hvac_control() {
  local building_id="${1:-}"
  local system_id="${2:-}"
  local setpoint="${3:-}"

  if [ -z "$building_id" ] || [ -z "$system_id" ] || [ -z "$setpoint" ]; then
    echo "Usage: $0 hvac-control --building-id BUILDING_ID --system-id SYSTEM_ID --setpoint TEMP"
    echo ""
    echo "Example:"
    echo "  $0 hvac-control --building-id BLD-001 --system-id HVAC-AHU-001 --setpoint 22.0"
    exit 1
  fi

  print_header
  print_info "Setting HVAC setpoint: $setpoint°C"
  echo ""

  data="{\"buildingId\":\"${building_id}\",\"systemId\":\"${system_id}\",\"mode\":\"auto\",\"setpoint\":${setpoint}}"
  response=$(api_request POST "/api/v1/hvac/setpoint" "$data")

  if [ "$(parse_json "$response" "success")" = "true" ]; then
    print_success "Setpoint updated successfully!"
  else
    print_error "Failed to update setpoint"
    pretty_json "$response"
  fi
}

cmd_solar() {
  local pv_system_id="${1:-}"

  if [ -z "$pv_system_id" ]; then
    echo "Usage: $0 solar --pv-id PV_SYSTEM_ID"
    echo ""
    echo "Example:"
    echo "  $0 solar --pv-id PV-ROOF-001"
    exit 1
  fi

  print_header
  print_info "Solar PV data: $pv_system_id"
  echo ""

  response=$(api_request GET "/api/v1/renewable/solar/${pv_system_id}")

  if has_jq; then
    current_gen=$(echo "$response" | jq -r '.data.generation.current')
    daily_gen=$(echo "$response" | jq -r '.data.generation.daily')
    irradiance=$(echo "$response" | jq -r '.data.irradiance')
    pr=$(echo "$response" | jq -r '.data.performanceRatio')

    print_metric "Current Generation" "$current_gen" "kW"
    print_metric "Today Generation  " "$daily_gen" "kWh"
    print_metric "Irradiance        " "$irradiance" "W/m²"
    print_metric "Performance Ratio " "$pr" "%"
  else
    pretty_json "$response"
  fi
}

cmd_ess_status() {
  local ess_id="${1:-}"

  if [ -z "$ess_id" ]; then
    echo "Usage: $0 ess-status --ess-id ESS_ID"
    echo ""
    echo "Example:"
    echo "  $0 ess-status --ess-id ESS-001"
    exit 1
  fi

  print_header
  print_info "ESS status: $ess_id"
  echo ""

  response=$(api_request GET "/api/v1/renewable/ess/${ess_id}")

  if has_jq; then
    mode=$(echo "$response" | jq -r '.data.mode')
    soc=$(echo "$response" | jq -r '.data.stateOfCharge')
    power=$(echo "$response" | jq -r '.data.power')
    soh=$(echo "$response" | jq -r '.data.health.stateOfHealth')

    echo -e "${CYAN}Mode:${NC} $mode"
    print_metric "State of Charge   " "$soc" "%"
    print_metric "Power             " "$power" "kW"
    print_metric "State of Health   " "$soh" "%"
  else
    pretty_json "$response"
  fi
}

cmd_carbon() {
  local building_id="${1:-}"
  local period="${2:-daily}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 carbon --building-id BUILDING_ID [--period PERIOD]"
    echo ""
    echo "Periods: daily, weekly, monthly, annual"
    echo ""
    echo "Example:"
    echo "  $0 carbon --building-id BLD-SEOUL-001 --period daily"
    exit 1
  fi

  print_header
  print_info "Carbon emissions report: $period"
  echo ""

  date=$(date +"%Y-%m-%d")
  response=$(api_request GET "/api/v1/carbon/${building_id}/report?period=${period}&date=${date}")

  if has_jq; then
    total=$(echo "$response" | jq -r '.data.emissions.total')
    elec=$(echo "$response" | jq -r '.data.emissions.electricity')
    offset=$(echo "$response" | jq -r '.data.offsetByRenewable.total')
    net=$(echo "$response" | jq -r '.data.netEmissions')

    print_metric "Total Emissions   " "$total" "kg CO₂"
    print_metric "  - Electricity   " "$elec" "kg CO₂"
    print_metric "Renewable Offset  " "$offset" "kg CO₂"
    print_metric "Net Emissions     " "$net" "kg CO₂"
  else
    pretty_json "$response"
  fi
}

cmd_benchmark() {
  local building_id="${1:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 benchmark --building-id BUILDING_ID"
    echo ""
    echo "Example:"
    echo "  $0 benchmark --building-id BLD-SEOUL-001"
    exit 1
  fi

  print_header
  print_info "Energy benchmarking comparison"
  echo ""

  response=$(api_request GET "/api/v1/benchmarking/${building_id}/compare?period=annual")

  if has_jq; then
    your_eui=$(echo "$response" | jq -r '.data.yourEUI')
    median_eui=$(echo "$response" | jq -r '.data.benchmarkEUI.median')
    rank=$(echo "$response" | jq -r '.data.rank')
    percentile=$(echo "$response" | jq -r '.data.percentile')

    print_metric "Your EUI          " "$your_eui" "kWh/m²/year"
    print_metric "Median EUI        " "$median_eui" "kWh/m²/year"
    echo -e "${CYAN}Rank:${NC} $rank (${percentile}th percentile)"
  else
    pretty_json "$response"
  fi
}

cmd_alerts() {
  local building_id="${1:-}"

  if [ -z "$building_id" ]; then
    echo "Usage: $0 alerts --building-id BUILDING_ID"
    echo ""
    echo "Example:"
    echo "  $0 alerts --building-id BLD-SEOUL-001"
    exit 1
  fi

  print_header
  print_info "Active alerts for building: $building_id"
  echo ""

  response=$(api_request GET "/api/v1/alerts/${building_id}/active")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat << EOF
Commands:
  dashboard           View real-time dashboard
  energy              View energy consumption
  hvac-status         Get HVAC system status
  hvac-control        Control HVAC setpoint
  solar               Get solar PV data
  ess-status          Get ESS status
  carbon              View carbon emissions
  benchmark           Compare with peer buildings
  alerts              View active alerts
  config              Configure API settings
  help                Show this help

Environment Variables:
  WIA_API_KEY         API key for authentication
  WIA_API_ENDPOINT    API endpoint URL

Examples:
  $0 dashboard --building-id BLD-SEOUL-001
  $0 energy --building-id BLD-SEOUL-001 --type electricity --period today
  $0 hvac-status --system-id HVAC-AHU-001
  $0 solar --pv-id PV-ROOF-001
  $0 carbon --building-id BLD-SEOUL-001 --period monthly

For more information, visit: https://wia.org/standards/city-011
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

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --building-id)
      BUILDING_ID="$2"
      shift 2
      ;;
    --system-id)
      SYSTEM_ID="$2"
      shift 2
      ;;
    --pv-id)
      PV_ID="$2"
      shift 2
      ;;
    --ess-id)
      ESS_ID="$2"
      shift 2
      ;;
    --type)
      TYPE="$2"
      shift 2
      ;;
    --period)
      PERIOD="$2"
      shift 2
      ;;
    --setpoint)
      SETPOINT="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

# Execute command
case $COMMAND in
  config)
    cmd_config
    ;;
  dashboard)
    cmd_dashboard "$BUILDING_ID"
    ;;
  energy)
    cmd_energy "$BUILDING_ID" "${TYPE:-electricity}" "${PERIOD:-today}"
    ;;
  hvac-status)
    cmd_hvac_status "$SYSTEM_ID"
    ;;
  hvac-control)
    cmd_hvac_control "$BUILDING_ID" "$SYSTEM_ID" "$SETPOINT"
    ;;
  solar)
    cmd_solar "$PV_ID"
    ;;
  ess-status)
    cmd_ess_status "$ESS_ID"
    ;;
  carbon)
    cmd_carbon "$BUILDING_ID" "${PERIOD:-daily}"
    ;;
  benchmark)
    cmd_benchmark "$BUILDING_ID"
    ;;
  alerts)
    cmd_alerts "$BUILDING_ID"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    cmd_help
    exit 1
    ;;
esac
