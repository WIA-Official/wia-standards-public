#!/bin/bash

###############################################################################
# WIA-ENE-038: Sustainable Mineral Mining CLI Tool
#
# Description: Command-line interface for mineral mining operations,
#              environmental management, and supply chain traceability
#
# Version: 1.0.0
# License: CC BY 4.0
#
# 弘益人間 (홍익인간) - Benefit All Humanity
###############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_ENE038_ENDPOINT:-https://api.wia.org/ene-038/v1}"
API_KEY="${WIA_ENE038_API_KEY}"
OPERATOR_ID="${WIA_ENE038_OPERATOR_ID}"

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
MINE="⛏️ "
CHECK="✅"
CROSS="❌"
WARN="⚠️ "
INFO="ℹ️ "
ROCKET="🚀"
LEAF="🌿"
WATER="💧"
GLOBE="🌍"
CHART="📊"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${BOLD}${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ${MINE}WIA-ENE-038: Sustainable Mineral Mining CLI     ║"
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
        print_error "API key not set. Please set WIA_ENE038_API_KEY environment variable."
        echo -e "${INFO} Example: export WIA_ENE038_API_KEY=\"your-api-key\""
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
        -H "User-Agent: WIA-ENE-038-CLI/1.0.0"
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
# Mine Management Commands
###############################################################################

cmd_create_mine() {
    print_info "Creating new mine..."

    # Interactive input
    read -p "Mine Type (OPEN_PIT/UNDERGROUND/SOLUTION): " mine_type
    read -p "Mine Name: " mine_name
    read -p "Primary Mineral (COPPER/LITHIUM/IRON/etc): " primary_mineral
    read -p "Region/Province: " region
    read -p "Country (CL/AU/ZA/etc): " country
    read -p "Latitude: " latitude
    read -p "Longitude: " longitude
    read -p "Estimated Reserves (tonnes): " reserves

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "mineType": "$mine_type",
  "mineName": "$mine_name",
  "operatorId": "$OPERATOR_ID",
  "location": {
    "address": {
      "region": "$region",
      "country": "$country"
    },
    "coordinates": {
      "latitude": $latitude,
      "longitude": $longitude,
      "datum": "WGS84"
    }
  },
  "mineralResources": {
    "primaryMineral": "$primary_mineral",
    "estimatedReserves": {
      "value": $reserves,
      "unit": "tonnes"
    }
  }
}
EOF
)

    local response=$(make_api_request "POST" "/mines" "$payload")
    local mine_id=$(echo "$response" | jq -r '.mineId')

    if [ -n "$mine_id" ] && [ "$mine_id" != "null" ]; then
        print_success "Mine created successfully!"
        echo -e "${BOLD}Mine ID:${NC} $mine_id"
    else
        print_error "Failed to create mine"
        echo "$response" | jq '.'
        exit 1
    fi
}

cmd_get_mine() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 get-mine <mine-id>"
        exit 1
    fi

    print_info "Retrieving mine: $mine_id"

    local response=$(make_api_request "GET" "/mines/$mine_id")
    echo "$response" | jq '.'
}

cmd_list_mines() {
    print_info "Listing mines..."

    local page=${1:-1}
    local limit=${2:-10}

    local response=$(make_api_request "GET" "/mines?page=$page&limit=$limit")

    echo -e "\n${BOLD}${MINE}Mines:${NC}"
    echo "$response" | jq -r '.data[] | "\(.mineId) - \(.mineName) - \(.mineralResources.primaryMineral) - Status: \(.status)"'

    echo -e "\n${BOLD}Pagination:${NC}"
    echo "$response" | jq '.pagination'
}

cmd_update_status() {
    local mine_id=$1
    local new_status=$2

    if [ -z "$mine_id" ] || [ -z "$new_status" ]; then
        print_error "Mine ID and status are required"
        echo "Usage: $0 update-status <mine-id> <status>"
        echo "Status: operating, closure, post_closure, etc."
        exit 1
    fi

    print_info "Updating mine status to: $new_status"

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "status": "$new_status",
  "effectiveDate": "$(date -u +"%Y-%m-%d")"
}
EOF
)

    make_api_request "PUT" "/mines/$mine_id/status" "$payload"
    print_success "Mine status updated"
}

###############################################################################
# Production Commands
###############################################################################

cmd_submit_production() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 submit-production <mine-id>"
        exit 1
    fi

    print_info "Submitting production data for: $mine_id"

    read -p "${MINE}Ore extracted (tonnes/day): " ore_extracted
    read -p "${MINE}Ore processed (tonnes/day): " ore_processed
    read -p "Mineral type (COPPER/LITHIUM/etc): " mineral_type
    read -p "Grade (percent): " grade
    read -p "Mineral produced (tonnes/day): " produced

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "date": "$(date -u +"%Y-%m-%d")",
  "production": {
    "ore": {
      "extracted": {
        "value": $ore_extracted,
        "unit": "tonnes/day"
      },
      "processed": {
        "value": $ore_processed,
        "unit": "tonnes/day"
      }
    },
    "minerals": [
      {
        "mineralType": "$mineral_type",
        "grade": {
          "value": $grade,
          "unit": "percent"
        },
        "produced": {
          "value": $produced,
          "unit": "tonnes/day"
        }
      }
    ]
  }
}
EOF
)

    make_api_request "POST" "/production/daily" "$payload"
    print_success "Production data submitted"
}

cmd_get_production() {
    local mine_id=$1
    local days=${2:-30}

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 get-production <mine-id> [days]"
        exit 1
    fi

    print_info "Retrieving production data for last $days days..."

    local end_date=$(date -u +"%Y-%m-%d")
    local start_date=$(date -u -d "$days days ago" +"%Y-%m-%d")

    local response=$(make_api_request "GET" "/production/daily?mineId=$mine_id&startDate=$start_date&endDate=$end_date")

    echo -e "\n${BOLD}${MINE}Production History:${NC}"
    echo "$response" | jq -r '.data[] | "\(.timestamp | split("T")[0]) - Ore: \(.dailyProduction.ore.processed.value) tonnes"'
}

###############################################################################
# Environmental Commands
###############################################################################

cmd_report_environmental() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 report-environmental <mine-id>"
        exit 1
    fi

    print_info "Reporting environmental data for: $mine_id"

    read -p "${WATER}Freshwater used (m3/day): " freshwater
    read -p "${WATER}Water recycled (m3/day): " recycled
    read -p "${GLOBE}GHG emissions Scope 1 (tonnes CO2e/year): " scope1
    read -p "${GLOBE}GHG emissions Scope 2 (tonnes CO2e/year): " scope2

    local recycling_rate=$(echo "scale=2; ($recycled / ($freshwater + $recycled)) * 100" | bc -l)

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "reportingPeriod": {
    "startDate": "$(date -u -d '1 month ago' +"%Y-%m-%d")",
    "endDate": "$(date -u +"%Y-%m-%d")"
  },
  "waterUsage": {
    "freshwater": {
      "value": $freshwater,
      "unit": "m3/day"
    },
    "recycled": {
      "value": $recycled,
      "unit": "m3/day"
    },
    "recyclingRate": {
      "value": $recycling_rate,
      "unit": "percent"
    }
  },
  "emissionsData": {
    "greenhouseGas": {
      "scope1": {
        "value": $scope1,
        "unit": "tonnes_CO2e/year"
      },
      "scope2": {
        "value": $scope2,
        "unit": "tonnes_CO2e/year"
      }
    }
  }
}
EOF
)

    make_api_request "POST" "/environmental/data" "$payload"
    print_success "Environmental data submitted"
}

cmd_monitor_tailings() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 monitor-tailings <mine-id>"
        exit 1
    fi

    print_info "Retrieving tailings monitoring data for: $mine_id"

    local response=$(make_api_request "GET" "/environmental/tailings/$mine_id")

    echo -e "\n${BOLD}${WARN}Tailings Monitoring:${NC}"
    echo "$response" | jq '.'
}

cmd_submit_reclamation() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 submit-reclamation <mine-id>"
        exit 1
    fi

    print_info "Submitting land reclamation progress for: $mine_id"

    read -p "${LEAF}Area reclaimed (hectares): " reclaimed_area
    read -p "Native species planted (yes/no): " native_species
    read -p "Survival rate (percent): " survival_rate

    local payload=$(cat <<EOF
{
  "reclaimedArea": {
    "value": $reclaimed_area,
    "unit": "hectares"
  },
  "revegetation": {
    "nativeSpecies": $([ "$native_species" = "yes" ] && echo "true" || echo "false"),
    "survivalRate": {
      "value": $survival_rate,
      "unit": "percent"
    }
  },
  "date": "$(date -u +"%Y-%m-%d")"
}
EOF
)

    make_api_request "POST" "/environmental/reclamation/$mine_id" "$payload"
    print_success "Reclamation progress submitted"
}

###############################################################################
# Community Relations Commands
###############################################################################

cmd_report_community() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 report-community <mine-id>"
        exit 1
    fi

    print_info "Submitting community engagement data for: $mine_id"

    read -p "Consultation date (YYYY-MM-DD): " consult_date
    read -p "Number of participants: " participants
    read -p "Topics discussed: " topics
    read -p "Local employment (number): " local_emp
    read -p "Community investment (USD): " investment

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "engagement": {
    "consultations": [
      {
        "date": "$consult_date",
        "participants": $participants,
        "topics": ["$topics"]
      }
    ]
  },
  "socialImpact": {
    "localEmployment": {
      "total": $local_emp
    },
    "communityInvestment": {
      "annual": $investment,
      "currency": "USD"
    }
  }
}
EOF
)

    make_api_request "POST" "/community/engagement" "$payload"
    print_success "Community engagement data submitted"
}

cmd_list_grievances() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 list-grievances <mine-id>"
        exit 1
    fi

    print_info "Listing grievances for: $mine_id"

    local response=$(make_api_request "GET" "/community/grievances?mineId=$mine_id")

    echo -e "\n${BOLD}Grievances:${NC}"
    echo "$response" | jq -r '.data[] | "\(.id) - Status: \(.status) - Date: \(.date)"'
}

###############################################################################
# Supply Chain Commands
###############################################################################

cmd_track_shipment() {
    local shipment_id=$1

    if [ -z "$shipment_id" ]; then
        print_error "Shipment ID is required"
        echo "Usage: $0 track-shipment <shipment-id>"
        exit 1
    fi

    print_info "Tracking shipment: $shipment_id"

    local response=$(make_api_request "GET" "/supply-chain/track/$shipment_id")

    echo -e "\n${BOLD}${GLOBE}Shipment Details:${NC}"
    echo "$response" | jq '.'
}

cmd_verify_conflict_free() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 verify-conflict-free <mine-id>"
        exit 1
    fi

    print_info "Verifying conflict-free status for: $mine_id"

    local response=$(make_api_request "GET" "/supply-chain/conflict-free?mineId=$mine_id")

    echo -e "\n${BOLD}${CHECK}Conflict-Free Verification:${NC}"
    echo "$response" | jq '.'
}

cmd_register_shipment() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 register-shipment <mine-id>"
        exit 1
    fi

    print_info "Registering new shipment for: $mine_id"

    read -p "Shipment ID: " shipment_id
    read -p "Mineral type: " mineral_type
    read -p "Quantity (tonnes): " quantity
    read -p "Destination country: " dest_country
    read -p "Destination facility: " dest_facility
    read -p "Conflict-free certified (yes/no): " conflict_free

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "shipmentId": "$shipment_id",
  "mineralType": "$mineral_type",
  "quantity": {
    "value": $quantity,
    "unit": "tonnes"
  },
  "destination": {
    "country": "$dest_country",
    "facility": "$dest_facility"
  },
  "certifications": ["IRMA", "RMI"],
  "conflictFree": $([ "$conflict_free" = "yes" ] && echo "true" || echo "false")
}
EOF
)

    make_api_request "POST" "/supply-chain/register" "$payload"
    print_success "Shipment registered"
}

###############################################################################
# ESG Commands
###############################################################################

cmd_esg_dashboard() {
    local operator_id=${1:-$OPERATOR_ID}

    if [ -z "$operator_id" ]; then
        print_error "Operator ID is required"
        exit 1
    fi

    print_info "Loading ESG dashboard..."

    local response=$(make_api_request "GET" "/esg/dashboard/$operator_id")

    echo -e "\n${BOLD}${CYAN}${CHART}ESG Dashboard${NC}"
    echo -e "${BOLD}════════════════════════════════════════${NC}"

    echo -e "\n${BOLD}${GLOBE}Environmental Performance:${NC}"
    echo "$response" | jq -r '
        "  Carbon Intensity: \(.environmental.carbonIntensity.value) \(.environmental.carbonIntensity.unit)",
        "  Water Intensity: \(.environmental.waterIntensity.value) \(.environmental.waterIntensity.unit)",
        "  Land Reclaimed: \(.environmental.landReclaimed.value) \(.environmental.landReclaimed.unit)"
    '

    echo -e "\n${BOLD}Social Performance:${NC}"
    echo "$response" | jq -r '
        "  LTIFR: \(.social.ltifr)",
        "  Fatalities: \(.social.fatalities)",
        "  Local Employment: \(.social.localEmploymentRate.value)%",
        "  Community Investment: $\(.social.communityInvestment.value) \(.social.communityInvestment.currency)"
    '

    echo -e "\n${BOLD}Governance:${NC}"
    echo "$response" | jq -r '
        "  Anti-Corruption Policy: \(.governance.antiCorruptionPolicy)",
        "  Transparency Reporting: \(.governance.transparencyReporting)",
        "  Certifications: \(.governance.certifications | join(", "))"
    '

    echo -e "\n${BOLD}Overall ESG Score:${NC}"
    echo "$response" | jq -r '"  Score: \(.overallScore)/100"'
}

cmd_submit_esg_report() {
    local mine_id=$1

    if [ -z "$mine_id" ]; then
        print_error "Mine ID is required"
        echo "Usage: $0 submit-esg-report <mine-id>"
        exit 1
    fi

    print_info "Submitting ESG report for: $mine_id"
    print_warning "This is a simplified submission. Full ESG reports require comprehensive data."

    read -p "Reporting year: " year
    read -p "Overall ESG score (0-100): " esg_score

    local payload=$(cat <<EOF
{
  "mineId": "$mine_id",
  "reportingYear": $year,
  "esgMetrics": {
    "reportingYear": $year,
    "overallESGScore": $esg_score
  }
}
EOF
)

    make_api_request "POST" "/esg/report" "$payload"
    print_success "ESG report submitted"
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

    echo -e "\n${BOLD}${CYAN}${MINE}Operator Dashboard${NC}"
    echo -e "${BOLD}════════════════════════════════════════${NC}"

    echo -e "\n${BOLD}Mine Summary:${NC}"
    echo "$response" | jq -r '
        "  Total Mines: \(.summary.totalMines)",
        "  Operating: \(.summary.operatingMines)",
        "  Closure: \(.summary.closureMines)"
    '

    echo -e "\n${BOLD}${MINE}Production (Monthly):${NC}"
    echo "$response" | jq -r '.production.monthly | to_entries[] | "  \(.key): \(.value.value) \(.value.unit)"'

    echo -e "\n${BOLD}${LEAF}Environmental:${NC}"
    echo "$response" | jq -r '
        "  Total Emissions: \(.environmental.totalEmissions.value) \(.environmental.totalEmissions.unit)",
        "  Water Recycling: \(.environmental.waterRecyclingRate.value)%",
        "  Land Reclaimed: \(.environmental.landReclaimed.value) \(.environmental.landReclaimed.unit)"
    '

    echo -e "\n${BOLD}Social:${NC}"
    echo "$response" | jq -r '
        "  LTIFR: \(.social.ltifr)",
        "  Local Employment: \(.social.localEmploymentRate.value)%",
        "  Community Investment: $\(.social.communityInvestment.value) \(.social.communityInvestment.currency)"
    '

    echo -e "\n${BOLD}${CHART}ESG:${NC}"
    echo "$response" | jq -r '
        "  Overall Score: \(.esg.overallScore)/100",
        "  Environmental: \(.esg.environmental)/100",
        "  Social: \(.esg.social)/100",
        "  Governance: \(.esg.governance)/100"
    '
}

###############################################################################
# Help & Version
###############################################################################

show_help() {
    print_header
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "${BOLD}Mine Management:${NC}"
    echo "  create-mine                    Create a new mine (interactive)"
    echo "  get-mine <id>                  Get mine details"
    echo "  list-mines [page] [limit]      List all mines"
    echo "  update-status <id> <status>    Update mine status"
    echo ""
    echo "${BOLD}Production:${NC}"
    echo "  submit-production <id>         Submit daily production data"
    echo "  get-production <id> [days]     Get production history"
    echo ""
    echo "${BOLD}Environmental:${NC}"
    echo "  report-environmental <id>      Report environmental data"
    echo "  monitor-tailings <id>          Monitor tailings facility"
    echo "  submit-reclamation <id>        Submit land reclamation progress"
    echo ""
    echo "${BOLD}Community Relations:${NC}"
    echo "  report-community <id>          Report community engagement"
    echo "  list-grievances <id>           List grievances"
    echo ""
    echo "${BOLD}Supply Chain:${NC}"
    echo "  register-shipment <id>         Register new shipment"
    echo "  track-shipment <shipment-id>   Track shipment"
    echo "  verify-conflict-free <id>      Verify conflict-free status"
    echo ""
    echo "${BOLD}ESG:${NC}"
    echo "  esg-dashboard [operator-id]    Show ESG dashboard"
    echo "  submit-esg-report <id>         Submit ESG report"
    echo ""
    echo "${BOLD}Dashboard:${NC}"
    echo "  dashboard [operator-id]        Show operator dashboard"
    echo ""
    echo "${BOLD}General:${NC}"
    echo "  help                           Show this help message"
    echo "  version                        Show version information"
    echo ""
    echo "${BOLD}Environment Variables:${NC}"
    echo "  WIA_ENE038_API_KEY             API key (required)"
    echo "  WIA_ENE038_ENDPOINT            API endpoint (optional)"
    echo "  WIA_ENE038_OPERATOR_ID         Operator ID (optional)"
    echo ""
    echo "${BOLD}Examples:${NC}"
    echo "  $0 create-mine"
    echo "  $0 submit-production WIA-MINE-2025-CL-001234"
    echo "  $0 track-shipment SHIP-2025-001234"
    echo "  $0 esg-dashboard"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
}

show_version() {
    print_header
    echo "WIA-ENE-038 CLI Tool"
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
        create-mine)
            check_api_key
            cmd_create_mine "$@"
            ;;
        get-mine)
            check_api_key
            cmd_get_mine "$@"
            ;;
        list-mines)
            check_api_key
            cmd_list_mines "$@"
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
        report-environmental)
            check_api_key
            cmd_report_environmental "$@"
            ;;
        monitor-tailings)
            check_api_key
            cmd_monitor_tailings "$@"
            ;;
        submit-reclamation)
            check_api_key
            cmd_submit_reclamation "$@"
            ;;
        report-community)
            check_api_key
            cmd_report_community "$@"
            ;;
        list-grievances)
            check_api_key
            cmd_list_grievances "$@"
            ;;
        register-shipment)
            check_api_key
            cmd_register_shipment "$@"
            ;;
        track-shipment)
            check_api_key
            cmd_track_shipment "$@"
            ;;
        verify-conflict-free)
            check_api_key
            cmd_verify_conflict_free "$@"
            ;;
        esg-dashboard)
            check_api_key
            cmd_esg_dashboard "$@"
            ;;
        submit-esg-report)
            check_api_key
            cmd_submit_esg_report "$@"
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
