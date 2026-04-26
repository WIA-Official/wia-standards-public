#!/bin/bash

###############################################################################
# WIA-ENE-036: Oil & Gas Drilling CLI Tool
#
# Description: Command-line interface for oil and gas well management,
#              production monitoring, and environmental compliance
#
# Version: 1.0.0
# License: CC BY 4.0
#
# 弘益人間 (홍익인간) - Benefit All Humanity
###############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_ENE036_ENDPOINT:-https://api.wia.org/ene-036/v1}"
API_KEY="${WIA_ENE036_API_KEY}"
OPERATOR_ID="${WIA_ENE036_OPERATOR_ID}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Emoji
OIL="🛢️ "
CHECK="✅"
CROSS="❌"
WARN="⚠️ "
INFO="ℹ️ "
ROCKET="🚀"
FIRE="🔥"
WATER="💧"
LEAF="🌿"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${BOLD}${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ${OIL}WIA-ENE-036: Oil & Gas Drilling CLI              ║"
    echo "║  弘益人間 (홍익인간) - Benefit All Humanity                 ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}${CHECK} $1${NC}"
}

print_error() {
    echo -e "${RED}${CROSS} Error: $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}${WARN} Warning: $1${NC}"
}

print_info() {
    echo -e "${CYAN}${INFO} $1${NC}"
}

check_dependencies() {
    if ! command -v curl &> /dev/null; then
        print_error "curl is required but not installed. Please install curl."
        exit 1
    fi

    if ! command -v jq &> /dev/null; then
        print_error "jq is required but not installed. Please install jq."
        exit 1
    fi
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_ENE036_API_KEY environment variable."
        echo -e "${INFO} Example: export WIA_ENE036_API_KEY=\"your-api-key\""
        exit 1
    fi
}

make_api_request() {
    local method=$1
    local endpoint=$2
    local data=$3

    local headers=(
        -H "Content-Type: application/json"
        -H "X-API-Key: $API_KEY"
        -H "User-Agent: WIA-ENE-036-CLI/1.0.0"
    )

    if [ -n "$OPERATOR_ID" ]; then
        headers+=(-H "X-Operator-ID: $OPERATOR_ID")
    fi

    if [ "$method" = "GET" ] || [ "$method" = "DELETE" ]; then
        curl -s -X "$method" "${headers[@]}" "$API_ENDPOINT$endpoint"
    else
        curl -s -X "$method" "${headers[@]}" -d "$data" "$API_ENDPOINT$endpoint"
    fi
}

###############################################################################
# Well Management Commands
###############################################################################

cmd_create_well() {
    print_info "Creating new well..."

    # Interactive input
    read -p "Well Type (PROD-1/PROD-2/PROD-3/EXP-1): " well_type
    read -p "Well Name: " well_name
    read -p "State/Province: " state
    read -p "Country (US/CA/etc): " country
    read -p "Latitude: " latitude
    read -p "Longitude: " longitude
    read -p "Total Depth (meters): " total_depth

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "wellType": "$well_type",
  "wellName": "$well_name",
  "operatorId": "$OPERATOR_ID",
  "location": {
    "address": {
      "state": "$state",
      "country": "$country"
    },
    "coordinates": {
      "latitude": $latitude,
      "longitude": $longitude,
      "datum": "WGS84"
    }
  },
  "wellboreGeometry": {
    "totalDepth": {
      "measuredDepth": $total_depth,
      "trueVerticalDepth": $total_depth,
      "unit": "meters"
    },
    "casingProgram": []
  }
}
EOF
)

    local response=$(make_api_request "POST" "/wells" "$payload")
    local well_id=$(echo "$response" | jq -r '.wellId')

    if [ -n "$well_id" ] && [ "$well_id" != "null" ]; then
        print_success "Well created successfully!"
        echo -e "${BOLD}Well ID:${NC} $well_id"
    else
        print_error "Failed to create well"
        echo "$response" | jq '.'
        exit 1
    fi
}

cmd_get_well() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 get-well <well-id>"
        exit 1
    fi

    print_info "Retrieving well: $well_id"

    local response=$(make_api_request "GET" "/wells/$well_id")
    echo "$response" | jq '.'
}

cmd_list_wells() {
    print_info "Listing wells..."

    local page=${1:-1}
    local limit=${2:-10}

    local response=$(make_api_request "GET" "/wells?page=$page&limit=$limit")

    echo -e "\n${BOLD}Wells:${NC}"
    echo "$response" | jq -r '.data[] | "\(.wellId) - \(.wellName) - Status: \(.status)"'

    echo -e "\n${BOLD}Pagination:${NC}"
    echo "$response" | jq '.pagination'
}

cmd_update_status() {
    local well_id=$1
    local new_status=$2

    if [ -z "$well_id" ] || [ -z "$new_status" ]; then
        print_error "Well ID and status are required"
        echo "Usage: $0 update-status <well-id> <status>"
        echo "Status: producing, shut_in, abandoned, etc."
        exit 1
    fi

    print_info "Updating well status to: $new_status"

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "status": "$new_status",
  "effectiveDate": "$(date -u +"%Y-%m-%d")"
}
EOF
)

    make_api_request "PUT" "/wells/$well_id/status" "$payload"
    print_success "Well status updated"
}

###############################################################################
# Production Commands
###############################################################################

cmd_submit_production() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 submit-production <well-id>"
        exit 1
    fi

    print_info "Submitting production data for: $well_id"

    read -p "${OIL}Oil production (bbl/day): " oil_bbl
    read -p "${FIRE}Gas production (scf/day): " gas_scf
    read -p "${WATER}Water production (bbl/day): " water_bbl

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "date": "$(date -u +"%Y-%m-%d")",
  "production": {
    "oil": {
      "net": {
        "value": $oil_bbl,
        "unit": "bbl/day"
      }
    },
    "gas": {
      "gross": {
        "value": $gas_scf,
        "unit": "scf/day"
      }
    },
    "water": {
      "produced": {
        "value": $water_bbl,
        "unit": "bbl/day"
      }
    }
  }
}
EOF
)

    make_api_request "POST" "/production/daily" "$payload"
    print_success "Production data submitted"
}

cmd_get_production() {
    local well_id=$1
    local days=${2:-30}

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 get-production <well-id> [days]"
        exit 1
    fi

    print_info "Retrieving production data for last $days days..."

    local end_date=$(date -u +"%Y-%m-%d")
    local start_date=$(date -u -d "$days days ago" +"%Y-%m-%d")

    local response=$(make_api_request "GET" "/production/daily?wellId=$well_id&startDate=$start_date&endDate=$end_date")

    echo -e "\n${BOLD}Production History:${NC}"
    echo "$response" | jq -r '.data[] | "\(.timestamp | split("T")[0]) - Oil: \(.dailyProduction.oil.net.value) bbl, Gas: \(.dailyProduction.gas.gross.value) scf"'
}

cmd_forecast() {
    local well_id=$1
    local months=${2:-12}

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 forecast <well-id> [months]"
        exit 1
    fi

    print_info "Generating production forecast for $months months..."

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "forecastPeriod": $months,
  "method": "decline_curve"
}
EOF
)

    local response=$(make_api_request "POST" "/production/forecast" "$payload")

    echo -e "\n${BOLD}Production Forecast:${NC}"
    echo "$response" | jq '.'
}

###############################################################################
# Environmental Commands
###############################################################################

cmd_report_emissions() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 report-emissions <well-id>"
        exit 1
    fi

    print_info "Reporting emissions data for: $well_id"

    read -p "${FIRE}Flared gas (scf/day): " flared
    read -p "Vented gas (scf/day): " vented
    read -p "Fugitive emissions (kg/day): " fugitive

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "reportingPeriod": {
    "startDate": "$(date -u -d '1 month ago' +"%Y-%m-%d")",
    "endDate": "$(date -u +"%Y-%m-%d")"
  },
  "emissionsData": {
    "methaneEmissions": {
      "flaring": {
        "value": $(echo "$flared * 30 * 0.0551 / 1000" | bc -l),
        "unit": "tonnes_CO2e/year"
      },
      "venting": {
        "value": $(echo "$vented * 30 * 0.0551 / 1000" | bc -l),
        "unit": "tonnes_CO2e/year"
      },
      "fugitiveEmissions": {
        "value": $(echo "$fugitive * 30 / 1000" | bc -l),
        "unit": "tonnes_CO2e/year"
      }
    },
    "flaringData": {
      "volumeFlared": {
        "value": $flared,
        "unit": "scf/day"
      }
    }
  }
}
EOF
)

    make_api_request "POST" "/environmental/emissions" "$payload"
    print_success "Emissions data submitted"
}

cmd_report_spill() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 report-spill <well-id>"
        exit 1
    fi

    print_warning "Reporting spill incident for: $well_id"

    read -p "Substance (crude_oil/produced_water/etc): " substance
    read -p "Volume spilled (bbl): " volume
    read -p "Location: " location
    read -p "Cause: " cause

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "incidentDate": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "substance": "$substance",
  "volumeSpilled": {
    "value": $volume,
    "unit": "bbl"
  },
  "location": "$location",
  "cause": "$cause",
  "immediateActions": "Containment initiated, authorities notified"
}
EOF
)

    make_api_request "POST" "/environmental/spills" "$payload"
    print_success "Spill incident reported"
}

cmd_get_emissions_alerts() {
    local severity=${1:-all}

    print_info "Retrieving emissions alerts (severity: $severity)..."

    local endpoint="/environmental/alerts"
    if [ "$severity" != "all" ]; then
        endpoint="$endpoint?severity=$severity"
    fi

    local response=$(make_api_request "GET" "$endpoint")

    echo -e "\n${BOLD}${FIRE}Emissions Alerts:${NC}"
    echo "$response" | jq -r '.data[] | "\(.severity | ascii_upcase) - \(.alertType) - \(.timestamp)"'
}

###############################################################################
# Safety Commands
###############################################################################

cmd_submit_bop_test() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 submit-bop-test <well-id>"
        exit 1
    fi

    print_info "Submitting BOP test results for: $well_id"

    read -p "Test pressure (bar): " test_pressure
    read -p "Test result (passed/failed): " test_result

    local payload=$(cat <<EOF
{
  "wellId": "$well_id",
  "testDate": "$(date -u +"%Y-%m-%d")",
  "testPressure": {
    "value": $test_pressure,
    "unit": "bar"
  },
  "testResult": "$test_result",
  "duration": {
    "value": 30,
    "unit": "minutes"
  }
}
EOF
)

    make_api_request "POST" "/safety/bop-tests" "$payload"
    print_success "BOP test results submitted"
}

cmd_report_incident() {
    local well_id=$1

    if [ -z "$well_id" ]; then
        print_error "Well ID is required"
        echo "Usage: $0 report-incident <well-id>"
        exit 1
    fi

    print_warning "Reporting safety incident for: $well_id"

    read -p "Incident type (kick/blowout/fire/injury/etc): " incident_type
    read -p "Severity (minor/moderate/serious/fatal): " severity
    read -p "Description: " description

    local payload=$(cat <<EOF
{
  "incidentId": "INC-$(date +%s)",
  "wellId": "$well_id",
  "incidentDate": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "incidentType": "$incident_type",
  "severity": "$severity",
  "description": "$description",
  "reportedToAuthorities": true,
  "investigationStatus": "pending"
}
EOF
)

    make_api_request "POST" "/safety/incidents" "$payload"
    print_success "Safety incident reported"
}

###############################################################################
# Regulatory Commands
###############################################################################

cmd_get_compliance() {
    local well_id=${1:-}

    if [ -z "$well_id" ]; then
        print_info "Retrieving overall compliance status..."
        local response=$(make_api_request "GET" "/regulatory/compliance?operatorId=$OPERATOR_ID")
    else
        print_info "Retrieving compliance status for: $well_id"
        local response=$(make_api_request "GET" "/regulatory/compliance?wellId=$well_id")
    fi

    echo -e "\n${BOLD}Compliance Status:${NC}"
    echo "$response" | jq '.'
}

cmd_list_permits() {
    print_info "Listing permits..."

    local response=$(make_api_request "GET" "/regulatory/permits?operatorId=$OPERATOR_ID")

    echo -e "\n${BOLD}Permits:${NC}"
    echo "$response" | jq -r '.data[] | "\(.permitType) - \(.permitNumber) - Expires: \(.expiryDate)"'
}

###############################################################################
# Dashboard Commands
###############################################################################

cmd_dashboard() {
    local operator_id=${1:-$OPERATOR_ID}

    if [ -z "$operator_id" ]; then
        print_error "Operator ID is required"
        exit 1
    fi

    print_info "Loading operator dashboard..."

    local response=$(make_api_request "GET" "/operators/$operator_id/dashboard")

    echo -e "\n${BOLD}${CYAN}Operator Dashboard${NC}"
    echo -e "${BOLD}════════════════════════════════════════${NC}"

    echo -e "\n${BOLD}Well Summary:${NC}"
    echo "$response" | jq -r '
        "  Total Wells: \(.summary.totalWells)",
        "  Producing: \(.summary.producingWells)",
        "  Shut In: \(.summary.shutInWells)",
        "  Abandoned: \(.summary.abandonedWells)"
    '

    echo -e "\n${BOLD}${OIL}Daily Production:${NC}"
    echo "$response" | jq -r '
        "  Oil: \(.production.daily.oil.value) \(.production.daily.oil.unit)",
        "  Gas: \(.production.daily.gas.value) \(.production.daily.gas.unit)"
    '

    echo -e "\n${BOLD}${LEAF}Environmental:${NC}"
    echo "$response" | jq -r '
        "  Total Emissions: \(.environmental.totalEmissions.value) \(.environmental.totalEmissions.unit)",
        "  Flaring Intensity: \(.environmental.flaringIntensity.value)\(.environmental.flaringIntensity.unit)",
        "  Spill Incidents: \(.environmental.spillIncidents)"
    '

    echo -e "\n${BOLD}Compliance:${NC}"
    echo "$response" | jq -r '
        "  Permit Compliance: \(.compliance.permitCompliance)%",
        "  Reporting Compliance: \(.compliance.reportingCompliance)%",
        "  Safety Score: \(.compliance.safetyScore)/100"
    '
}

###############################################################################
# Help & Version
###############################################################################

show_help() {
    print_header
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "${BOLD}Well Management:${NC}"
    echo "  create-well                 Create a new well (interactive)"
    echo "  get-well <id>               Get well details"
    echo "  list-wells [page] [limit]   List all wells"
    echo "  update-status <id> <status> Update well status"
    echo ""
    echo "${BOLD}Production:${NC}"
    echo "  submit-production <id>      Submit daily production data"
    echo "  get-production <id> [days]  Get production history"
    echo "  forecast <id> [months]      Generate production forecast"
    echo ""
    echo "${BOLD}Environmental:${NC}"
    echo "  report-emissions <id>       Report emissions data"
    echo "  report-spill <id>           Report spill incident"
    echo "  get-emissions-alerts [sev]  Get emissions alerts"
    echo ""
    echo "${BOLD}Safety:${NC}"
    echo "  submit-bop-test <id>        Submit BOP test results"
    echo "  report-incident <id>        Report safety incident"
    echo ""
    echo "${BOLD}Regulatory:${NC}"
    echo "  get-compliance [id]         Get compliance status"
    echo "  list-permits                List all permits"
    echo ""
    echo "${BOLD}Dashboard:${NC}"
    echo "  dashboard [operator-id]     Show operator dashboard"
    echo ""
    echo "${BOLD}General:${NC}"
    echo "  help                        Show this help message"
    echo "  version                     Show version information"
    echo ""
    echo "${BOLD}Environment Variables:${NC}"
    echo "  WIA_ENE036_API_KEY          API key (required)"
    echo "  WIA_ENE036_ENDPOINT         API endpoint (optional)"
    echo "  WIA_ENE036_OPERATOR_ID      Operator ID (optional)"
    echo ""
    echo "${BOLD}Examples:${NC}"
    echo "  $0 create-well"
    echo "  $0 submit-production WIA-OG-2025-TX-001234"
    echo "  $0 get-production WIA-OG-2025-TX-001234 30"
    echo "  $0 report-emissions WIA-OG-2025-TX-001234"
    echo "  $0 dashboard"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
}

show_version() {
    print_header
    echo "WIA-ENE-036 CLI Tool"
    echo "Version: 1.0.0"
    echo "License: CC BY 4.0"
    echo "© 2025 WIA - World Certification Industry Association"
    echo ""
    echo "弘익人間 (홍익인간) - Benefit All Humanity"
}

###############################################################################
# Main
###############################################################################

main() {
    check_dependencies

    local command=${1:-help}
    shift || true

    case "$command" in
        create-well)
            check_api_key
            cmd_create_well "$@"
            ;;
        get-well)
            check_api_key
            cmd_get_well "$@"
            ;;
        list-wells)
            check_api_key
            cmd_list_wells "$@"
            ;;
        update-status)
            check_api_key
            cmd_update_status "$@"
            ;;
        submit-production)
            check_api_key
            cmd_submit_production "$@"
            ;;
        get-production)
            check_api_key
            cmd_get_production "$@"
            ;;
        forecast)
            check_api_key
            cmd_forecast "$@"
            ;;
        report-emissions)
            check_api_key
            cmd_report_emissions "$@"
            ;;
        report-spill)
            check_api_key
            cmd_report_spill "$@"
            ;;
        get-emissions-alerts)
            check_api_key
            cmd_get_emissions_alerts "$@"
            ;;
        submit-bop-test)
            check_api_key
            cmd_submit_bop_test "$@"
            ;;
        report-incident)
            check_api_key
            cmd_report_incident "$@"
            ;;
        get-compliance)
            check_api_key
            cmd_get_compliance "$@"
            ;;
        list-permits)
            check_api_key
            cmd_list_permits "$@"
            ;;
        dashboard)
            check_api_key
            cmd_dashboard "$@"
            ;;
        help|--help|-h)
            show_help
            ;;
        version|--version|-v)
            show_version
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
