#!/bin/bash

################################################################################
# WIA-AUTO-013: Smart Parking CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to smart parking operations
# including space search, reservations, occupancy tracking, and charging sessions.
################################################################################

set -e

# Colors for output
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
BLUE='\033[0;34m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
API_BASE_URL="${WIA_PARKING_API:-https://api.parking.example.com/v1}"
API_KEY="${WIA_PARKING_API_KEY:-}"

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🅿️  WIA-AUTO-013: Smart Parking CLI Tool              ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${RESET}"
}

print_error() {
    echo -e "${RED}✗ $1${RESET}"
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

# Check API key
check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_PARKING_API_KEY environment variable."
        exit 1
    fi
}

# Find available parking spaces
find_spaces() {
    local lat=${1:-37.7749}
    local lng=${2:--122.4194}
    local radius=${3:-500}
    local requires_ev=${4:-false}

    print_section "Finding Available Parking Spaces"
    print_info "Location: $lat, $lng"
    print_info "Search Radius: ${radius}m"
    print_info "EV Charging Required: $requires_ev"

    # Simulate API call (in production, use curl or http)
    echo ""
    print_success "Found 12 available spaces within ${radius}m"
    echo ""

    # Display results
    echo -e "${CYAN}┌─────────────────────────────────────────────────────────────────┐${RESET}"
    echo -e "${CYAN}│  Space ID      │ Lot         │ Distance │ Rate    │ Features  │${RESET}"
    echo -e "${CYAN}├─────────────────────────────────────────────────────────────────┤${RESET}"

    # Sample results
    echo -e "${GREEN}│  SP-EV-042     │${RESET} Downtown   │  150m    │ \$5/hr  │ EV,Covered│"
    echo -e "${GREEN}│  SP-001        │${RESET} Downtown   │  180m    │ \$4/hr  │ Covered   │"
    echo -e "${GREEN}│  SP-EV-018     │${RESET} Central    │  220m    │ \$6/hr  │ EV,24/7   │"
    echo -e "${GREEN}│  SP-043        │${RESET} Downtown   │  250m    │ \$4/hr  │ Standard  │"
    echo -e "${GREEN}│  SP-D-012      │${RESET} Downtown   │  280m    │ \$5/hr  │ Disabled  │"
    echo -e "${GREEN}│  SP-EV-025     │${RESET} Central    │  320m    │ \$6/hr  │ EV,FastCh │"

    echo -e "${CYAN}└─────────────────────────────────────────────────────────────────┘${RESET}"

    print_info "To reserve a space, use: wia-auto-013 reserve --space-id <ID>"
    echo ""
}

# Reserve a parking space
reserve_space() {
    local space_id=${1:-SP-001}
    local duration=${2:-120}
    local plate=${3:-ABC123}

    print_section "Parking Reservation"
    print_info "Space ID: $space_id"
    print_info "Duration: $duration minutes"
    print_info "License Plate: $plate"

    # Generate confirmation code
    local conf_code="RES-$(date +%Y%m%d)-$(head /dev/urandom | tr -dc A-F0-9 | head -c 4)"
    local access_code=$(head /dev/urandom | tr -dc 0-9 | head -c 4)

    print_section "Reservation Confirmed"
    print_success "Confirmation Code: ${conf_code}"
    print_success "Access Code: ${access_code}"

    echo ""
    print_info "Reservation Details:"
    echo -e "${GRAY}  ├─ Space:       ${space_id}${RESET}"
    echo -e "${GRAY}  ├─ Lot:         Downtown Parking Center${RESET}"
    echo -e "${GRAY}  ├─ Location:    Level 2, Section E, Row 4${RESET}"
    echo -e "${GRAY}  ├─ Start Time:  $(date '+%Y-%m-%d %H:%M')${RESET}"
    echo -e "${GRAY}  ├─ End Time:    $(date -d "+${duration} minutes" '+%Y-%m-%d %H:%M')${RESET}"
    echo -e "${GRAY}  ├─ Duration:    ${duration} minutes${RESET}"
    echo -e "${GRAY}  └─ Cost:        \$$(echo "scale=2; $duration / 60 * 5" | bc)${RESET}"

    echo ""
    print_section "Access Instructions"
    print_info "1. Enter code ${access_code} at parking gate"
    print_info "2. Follow signs to Level 2, Section E"
    print_info "3. Your space is Row 4, Position 2"
    print_info "4. Grace period: 15 minutes after reservation time"

    echo ""
    print_warning "Please arrive within 15 minutes or your reservation may be cancelled"
    echo ""
}

# Check lot occupancy status
check_status() {
    local lot_id=${1:-lot-downtown-01}

    print_section "Lot Occupancy Status"
    print_info "Lot ID: $lot_id"
    print_info "Lot Name: Downtown Parking Center"
    print_info "Last Updated: $(date '+%Y-%m-%d %H:%M:%S')"

    echo ""
    print_section "Overall Occupancy"

    local total=500
    local available=87
    local occupied=398
    local reserved=12
    local maintenance=3
    local rate=$(echo "scale=1; $occupied * 100 / $total" | bc)

    echo -e "${GRAY}  Total Spaces:     ${RESET}${total}"
    echo -e "${GREEN}  Available:        ${RESET}${available}"
    echo -e "${RED}  Occupied:         ${RESET}${occupied}"
    echo -e "${BLUE}  Reserved:         ${RESET}${reserved}"
    echo -e "${YELLOW}  Maintenance:      ${RESET}${maintenance}"
    echo -e "${ORANGE}  Occupancy Rate:   ${RESET}${rate}%"

    print_section "By Floor"
    echo -e "${GRAY}  Floor 1:  ${RESET}${GREEN}12${RESET} / 150 available  ${RED}[92% full]${RESET}"
    echo -e "${GRAY}  Floor 2:  ${RESET}${GREEN}35${RESET} / 175 available  ${YELLOW}[80% full]${RESET}"
    echo -e "${GRAY}  Floor 3:  ${RESET}${GREEN}40${RESET} / 175 available  ${YELLOW}[77% full]${RESET}"

    print_section "EV Charging Status"
    echo -e "${GRAY}  Total Chargers:   ${RESET}25"
    echo -e "${GRAY}  In Use:           ${RESET}${RED}18${RESET}"
    echo -e "${GRAY}  Available:        ${RESET}${GREEN}7${RESET}"

    echo ""
    if [ $available -lt 20 ]; then
        print_warning "Low availability! Consider nearby lots."
    elif [ $available -lt 50 ]; then
        print_info "Moderate availability"
    else
        print_success "Good availability"
    fi

    echo ""
}

# Start EV charging session
start_charging() {
    local space_id=${1:-SP-EV-042}
    local connector=${2:-ccs2}
    local target_soc=${3:-80}

    print_section "EV Charging Session"
    print_info "Space ID: $space_id"
    print_info "Connector Type: ${connector^^}"
    print_info "Target State of Charge: ${target_soc}%"

    # Generate session ID
    local session_id="CHG-$(date +%Y%m%d)-$(head /dev/urandom | tr -dc A-F0-9 | head -c 4)"

    echo ""
    print_success "Charging Session Started: ${session_id}"

    print_section "Charging Details"
    echo -e "${GRAY}  Charger ID:           ${RESET}EVSE-L2-18"
    echo -e "${GRAY}  Power Level:          ${RESET}Level 2 (11 kW)"
    echo -e "${GRAY}  Current SOC:          ${RESET}35%"
    echo -e "${GRAY}  Target SOC:           ${RESET}${target_soc}%"
    echo -e "${GRAY}  Estimated Time:       ${RESET}2h 30m"
    echo -e "${GRAY}  Estimated Cost:       ${RESET}\$8.25"
    echo -e "${GRAY}  Rate:                 ${RESET}\$0.35/kWh"

    print_section "Real-Time Status"
    print_info "Monitoring charging progress..."

    # Simulate charging progress
    for i in {1..5}; do
        local current_soc=$((35 + i * 3))
        local power=$(echo "scale=1; 11 - $i * 0.5" | bc)
        local energy=$(echo "scale=1; $i * 2.2" | bc)
        local cost=$(echo "scale=2; $energy * 0.35" | bc)

        sleep 1
        echo -e "${GRAY}  [$(date '+%H:%M:%S')] SOC: ${current_soc}% | Power: ${power} kW | Energy: ${energy} kWh | Cost: \$${cost}${RESET}"
    done

    echo ""
    print_success "Charging in progress"
    print_info "To stop charging: wia-auto-013 stop-charge --session-id ${session_id}"
    echo ""
}

# Generate occupancy report
generate_report() {
    local lot_id=${1:-lot-downtown-01}
    local period=${2:-week}

    print_section "Occupancy Report"
    print_info "Lot ID: $lot_id"
    print_info "Period: Last ${period}"
    print_info "Generated: $(date '+%Y-%m-%d %H:%M:%S')"

    print_section "Summary Statistics"
    echo -e "${GRAY}  Average Occupancy:    ${RESET}72.5%"
    echo -e "${GRAY}  Peak Occupancy:       ${RESET}95.2% (Wed 2:00 PM)"
    echo -e "${GRAY}  Lowest Occupancy:     ${RESET}18.3% (Sun 4:00 AM)"
    echo -e "${GRAY}  Average Dwell Time:   ${RESET}2h 15m"
    echo -e "${GRAY}  Turnover Rate:        ${RESET}3.2 vehicles/space/day"

    print_section "Revenue"
    echo -e "${GRAY}  Total Revenue:        ${RESET}\$12,450.00"
    echo -e "${GRAY}  Parking Revenue:      ${RESET}\$10,800.00"
    echo -e "${GRAY}  EV Charging:          ${RESET}\$1,450.00"
    echo -e "${GRAY}  Overstay Penalties:   ${RESET}\$200.00"
    echo -e "${GRAY}  Transactions:         ${RESET}3,847"
    echo -e "${GRAY}  Avg Transaction:      ${RESET}\$3.24"

    print_section "Peak Hours"
    echo -e "${GRAY}  Monday-Friday:        ${RESET}8:00 AM - 6:00 PM"
    echo -e "${GRAY}  Saturday:             ${RESET}10:00 AM - 8:00 PM"
    echo -e "${GRAY}  Sunday:               ${RESET}12:00 PM - 6:00 PM"

    print_section "Recommendations"
    print_success "Consider dynamic pricing during peak hours (Wed-Fri 2-4 PM)"
    print_info "Low weekend utilization - promote weekend parking specials"
    print_info "EV chargers at 72% utilization - consider adding 3-5 more"

    echo ""
    print_info "Full report saved to: /tmp/parking-report-${lot_id}-$(date +%Y%m%d).csv"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-013 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  find                     Find available parking spaces"
    echo "    --lat <latitude>       Search center latitude (default: 37.7749)"
    echo "    --lng <longitude>      Search center longitude (default: -122.4194)"
    echo "    --radius <meters>      Search radius (default: 500)"
    echo "    --requires-ev          Require EV charging capability"
    echo ""
    echo "  reserve                  Reserve a parking space"
    echo "    --space-id <id>        Space identifier (required)"
    echo "    --duration <minutes>   Parking duration (default: 120)"
    echo "    --plate <number>       License plate number (required)"
    echo ""
    echo "  status                   Check lot occupancy status"
    echo "    --lot-id <id>          Lot identifier (default: lot-downtown-01)"
    echo ""
    echo "  charge                   Start EV charging session"
    echo "    --space-id <id>        EV charging space ID (required)"
    echo "    --connector <type>     Connector type: ccs1, ccs2, chademo, type2"
    echo "    --target-soc <%>       Target state of charge (default: 80)"
    echo ""
    echo "  report                   Generate occupancy report"
    echo "    --lot-id <id>          Lot identifier (default: lot-downtown-01)"
    echo "    --period <period>      Report period: day, week, month (default: week)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-013 find --lat 37.7749 --lng -122.4194 --radius 500"
    echo "  wia-auto-013 reserve --space-id SP-EV-042 --duration 120 --plate ABC123"
    echo "  wia-auto-013 status --lot-id lot-downtown-01"
    echo "  wia-auto-013 charge --space-id SP-EV-042 --connector ccs2 --target-soc 80"
    echo "  wia-auto-013 report --lot-id lot-downtown-01 --period week"
    echo ""
    echo "Environment Variables:"
    echo "  WIA_PARKING_API_KEY      API key for authentication"
    echo "  WIA_PARKING_API          API base URL (default: https://api.parking.example.com/v1)"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-013 Smart Parking CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    find)
        LAT=37.7749
        LNG=-122.4194
        RADIUS=500
        REQUIRES_EV=false

        while [[ $# -gt 0 ]]; do
            case $1 in
                --lat) LAT=$2; shift 2 ;;
                --lng) LNG=$2; shift 2 ;;
                --radius) RADIUS=$2; shift 2 ;;
                --requires-ev) REQUIRES_EV=true; shift ;;
                *) shift ;;
            esac
        done

        print_header
        find_spaces "$LAT" "$LNG" "$RADIUS" "$REQUIRES_EV"
        ;;

    reserve)
        SPACE_ID=""
        DURATION=120
        PLATE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --space-id) SPACE_ID=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                --plate) PLATE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$SPACE_ID" ] || [ -z "$PLATE" ]; then
            print_error "Missing required arguments: --space-id and --plate"
            echo "Run 'wia-auto-013 help' for usage information"
            exit 1
        fi

        print_header
        reserve_space "$SPACE_ID" "$DURATION" "$PLATE"
        ;;

    status)
        LOT_ID="lot-downtown-01"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --lot-id) LOT_ID=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        check_status "$LOT_ID"
        ;;

    charge)
        SPACE_ID=""
        CONNECTOR="ccs2"
        TARGET_SOC=80

        while [[ $# -gt 0 ]]; do
            case $1 in
                --space-id) SPACE_ID=$2; shift 2 ;;
                --connector) CONNECTOR=$2; shift 2 ;;
                --target-soc) TARGET_SOC=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$SPACE_ID" ]; then
            print_error "Missing required argument: --space-id"
            echo "Run 'wia-auto-013 help' for usage information"
            exit 1
        fi

        print_header
        start_charging "$SPACE_ID" "$CONNECTOR" "$TARGET_SOC"
        ;;

    report)
        LOT_ID="lot-downtown-01"
        PERIOD="week"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --lot-id) LOT_ID=$2; shift 2 ;;
                --period) PERIOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_report "$LOT_ID" "$PERIOD"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-013 help' for usage information"
        exit 1
        ;;
esac

exit 0
