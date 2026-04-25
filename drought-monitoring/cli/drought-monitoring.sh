#!/bin/bash

################################################################################
# WIA-ENE-034: Drought Monitoring Standard - CLI Tool
#
# 弘益人間 (홍익人間) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-034
# Drought Monitoring Standard API.
#
# Usage:
#   drought-monitoring.sh <command> [options]
#
# Commands:
#   spi              Get SPI (Standardized Precipitation Index) data
#   pdsi             Get PDSI (Palmer Drought Severity Index) data
#   spei             Get SPEI (Standardized Precipitation Evapotranspiration Index) data
#   soil-moisture    Get soil moisture data
#   groundwater      Groundwater level operations
#   reservoir        Reservoir storage operations
#   precipitation    Get precipitation deficit analysis
#   vegetation       Get vegetation stress data
#   agriculture      Get agricultural drought impact
#   restrictions     Get water restrictions
#   declarations     Drought declaration operations
#   report           Get comprehensive drought monitoring report
#   forecast         Get drought forecast and early warning
#   trends           Get drought trend analysis
#   help             Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-034/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/drought-monitoring.conf"

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
  echo "║       🏜️  WIA-ENE-034: Drought Monitoring CLI v${VERSION}        ║"
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
# WIA-ENE-034 Drought Monitoring CLI Configuration
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
      -H "X-WIA-Standard: ENE-034" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-034" \
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

cmd_spi() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 spi LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/indices/spi?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching SPI data for coordinates: $lat, $lon"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_pdsi() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 pdsi LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/indices/pdsi?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching PDSI data for coordinates: $lat, $lon"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_spei() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 spei LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/indices/spei?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching SPEI data for coordinates: $lat, $lon"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_soil_moisture() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 soil-moisture LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/soil-moisture?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching soil moisture data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_groundwater() {
  local subcmd="${1:-list}"
  shift || true

  case "$subcmd" in
    list)
      print_info "Listing groundwater wells..."
      local response=$(api_request "GET" "/api/v1/groundwater/wells")
      pretty_json "$response"
      ;;
    get)
      local well_id="${1:-}"
      if [ -z "$well_id" ]; then
        echo "Usage: $0 groundwater get WELL_ID [START_DATE] [END_DATE]"
        exit 1
      fi
      local start_date="${2:-}"
      local end_date="${3:-}"
      local path="/api/v1/groundwater/wells/$well_id/levels"
      if [ -n "$start_date" ] && [ -n "$end_date" ]; then
        path="$path?startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
      fi
      print_info "Fetching groundwater level data for well: $well_id"
      local response=$(api_request "GET" "$path")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 groundwater {list|get} [options]"
      exit 1
      ;;
  esac
}

cmd_reservoir() {
  local subcmd="${1:-list}"
  shift || true

  case "$subcmd" in
    list)
      print_info "Listing reservoirs..."
      local response=$(api_request "GET" "/api/v1/reservoirs")
      pretty_json "$response"
      ;;
    get)
      local reservoir_id="${1:-}"
      if [ -z "$reservoir_id" ]; then
        echo "Usage: $0 reservoir get RESERVOIR_ID [START_DATE] [END_DATE]"
        exit 1
      fi
      local start_date="${2:-}"
      local end_date="${3:-}"
      local path="/api/v1/reservoirs/$reservoir_id/storage"
      if [ -n "$start_date" ] && [ -n "$end_date" ]; then
        path="$path?startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
      fi
      print_info "Fetching reservoir storage data for: $reservoir_id"
      local response=$(api_request "GET" "$path")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 reservoir {list|get} [options]"
      exit 1
      ;;
  esac
}

cmd_precipitation() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 precipitation LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/precipitation/deficit?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching precipitation deficit data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_vegetation() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 vegetation LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/vegetation/stress?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching vegetation stress data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_agriculture() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 agriculture LATITUDE LONGITUDE [START_DATE] [END_DATE]"
    exit 1
  fi

  local path="/api/v1/agriculture/impact?latitude=$lat&longitude=$lon"
  if [ -n "$start_date" ] && [ -n "$end_date" ]; then
    path="$path&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"
  fi

  print_info "Fetching agricultural drought impact data..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_restrictions() {
  local region="${1:-}"

  if [ -z "$region" ]; then
    echo "Usage: $0 restrictions REGION"
    exit 1
  fi

  print_info "Fetching water restrictions for: $region"
  local response=$(api_request "GET" "/api/v1/restrictions?region=$region")

  pretty_json "$response"
}

cmd_declarations() {
  local subcmd="${1:-list}"
  shift || true

  case "$subcmd" in
    list)
      print_info "Listing drought declarations..."
      local response=$(api_request "GET" "/api/v1/declarations")
      pretty_json "$response"
      ;;
    get)
      local declaration_id="${1:-}"
      if [ -z "$declaration_id" ]; then
        echo "Usage: $0 declarations get DECLARATION_ID"
        exit 1
      fi
      print_info "Fetching drought declaration: $declaration_id"
      local response=$(api_request "GET" "/api/v1/declarations/$declaration_id")
      pretty_json "$response"
      ;;
    *)
      echo "Usage: $0 declarations {list|get} [options]"
      exit 1
      ;;
  esac
}

cmd_report() {
  local lat="${1:-}"
  local lon="${2:-}"
  local date="${3:-$(date -u +%Y-%m-%d)}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 report LATITUDE LONGITUDE [DATE]"
    exit 1
  fi

  local path="/api/v1/monitoring/report?latitude=$lat&longitude=$lon&date=$date"

  print_info "Fetching comprehensive drought monitoring report..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_forecast() {
  local lat="${1:-}"
  local lon="${2:-}"

  if [ -z "$lat" ] || [ -z "$lon" ]; then
    echo "Usage: $0 forecast LATITUDE LONGITUDE"
    exit 1
  fi

  local path="/api/v1/forecast?latitude=$lat&longitude=$lon"

  print_info "Fetching drought forecast..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_trends() {
  local lat="${1:-}"
  local lon="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$lat" ] || [ -z "$lon" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 trends LATITUDE LONGITUDE START_DATE END_DATE"
    exit 1
  fi

  local path="/api/v1/analytics/trends?latitude=$lat&longitude=$lon&startDate=${start_date}T00:00:00Z&endDate=${end_date}T23:59:59Z"

  print_info "Fetching drought trend analysis..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}spi${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get Standardized Precipitation Index (SPI) data

  ${GREEN}pdsi${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get Palmer Drought Severity Index (PDSI) data

  ${GREEN}spei${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get Standardized Precipitation Evapotranspiration Index (SPEI) data

  ${GREEN}soil-moisture${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get soil moisture data

  ${GREEN}groundwater${NC} {list|get} [options]
    Groundwater level operations
    - list: List all groundwater wells
    - get WELL_ID [START_DATE] [END_DATE]: Get level data

  ${GREEN}reservoir${NC} {list|get} [options]
    Reservoir storage operations
    - list: List all reservoirs
    - get RESERVOIR_ID [START_DATE] [END_DATE]: Get storage data

  ${GREEN}precipitation${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get precipitation deficit analysis

  ${GREEN}vegetation${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get vegetation stress data (NDVI, VHI)

  ${GREEN}agriculture${NC} LATITUDE LONGITUDE [START_DATE] [END_DATE]
    Get agricultural drought impact assessment

  ${GREEN}restrictions${NC} REGION
    Get current water use restrictions

  ${GREEN}declarations${NC} {list|get} [options]
    Drought declaration operations
    - list: List all drought declarations
    - get DECLARATION_ID: Get declaration details

  ${GREEN}report${NC} LATITUDE LONGITUDE [DATE]
    Get comprehensive drought monitoring report

  ${GREEN}forecast${NC} LATITUDE LONGITUDE
    Get drought forecast and early warning

  ${GREEN}trends${NC} LATITUDE LONGITUDE START_DATE END_DATE
    Get drought trend analysis

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-034/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Get SPI data
  $0 spi 37.5665 126.9780 2025-01-01 2025-12-31

  # Get PDSI data
  $0 pdsi 37.5665 126.9780

  # Get soil moisture
  $0 soil-moisture 37.5665 126.9780

  # List reservoirs
  $0 reservoir list

  # Get reservoir storage
  $0 reservoir get RES-001 2025-01-01 2025-12-31

  # Get vegetation stress
  $0 vegetation 37.5665 126.9780

  # Get comprehensive report
  $0 report 37.5665 126.9780

  # Get drought forecast
  $0 forecast 37.5665 126.9780

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-034
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-034

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
  spi)
    cmd_spi "$@"
    ;;
  pdsi)
    cmd_pdsi "$@"
    ;;
  spei)
    cmd_spei "$@"
    ;;
  soil-moisture)
    cmd_soil_moisture "$@"
    ;;
  groundwater)
    cmd_groundwater "$@"
    ;;
  reservoir)
    cmd_reservoir "$@"
    ;;
  precipitation)
    cmd_precipitation "$@"
    ;;
  vegetation)
    cmd_vegetation "$@"
    ;;
  agriculture)
    cmd_agriculture "$@"
    ;;
  restrictions)
    cmd_restrictions "$@"
    ;;
  declarations)
    cmd_declarations "$@"
    ;;
  report)
    cmd_report "$@"
    ;;
  forecast)
    cmd_forecast "$@"
    ;;
  trends)
    cmd_trends "$@"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-034 Drought Monitoring CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
