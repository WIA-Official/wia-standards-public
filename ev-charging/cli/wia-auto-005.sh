#!/bin/bash

################################################################################
# WIA-AUTO-005: EV Charging CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to EV charging calculations
# including charging time estimation, cost calculation, and session validation.
################################################################################

set -e

# Colors for output
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
DEFAULT_EFFICIENCY=0.9

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🔌 WIA-AUTO-005: EV Charging CLI Tool               ║"
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

# Calculate charging time
calc_time() {
    local battery=${1:-75}
    local power=${2:-150}
    local from=${3:-20}
    local to=${4:-80}

    print_section "Charging Time Calculation"
    print_info "Battery Capacity: $battery kWh"
    print_info "Charging Power: $power kW"
    print_info "Current SOC: ${from}%"
    print_info "Target SOC: ${to}%"

    # Convert percentages to decimal
    local from_dec=$(echo "scale=2; $from / 100" | bc -l)
    local to_dec=$(echo "scale=2; $to / 100" | bc -l)

    # Calculate energy needed
    local energy=$(echo "scale=2; $battery * ($to_dec - $from_dec)" | bc -l)
    print_info "Energy Needed: ${energy} kWh"

    # Calculate time (accounting for efficiency)
    local hours=$(echo "scale=2; $energy / ($power * $DEFAULT_EFFICIENCY)" | bc -l)
    local minutes=$(echo "scale=0; $hours * 60" | bc -l)

    print_section "Results"
    print_success "Charging Time: ${hours} hours (${minutes} minutes)"
    print_info "Average Power: $(echo "scale=1; $power * $DEFAULT_EFFICIENCY" | bc -l) kW (with ${DEFAULT_EFFICIENCY} efficiency)"

    # Range estimation (assuming 250 Wh/mile)
    local range_added=$(echo "scale=0; $energy * 1000 / 250" | bc -l)
    print_info "Range Added: ~${range_added} miles (at 250 Wh/mile)"

    # Cost estimation (if price provided)
    if [ ! -z "$5" ]; then
        local price=$5
        local cost=$(echo "scale=2; $energy * $price" | bc -l)
        print_info "Estimated Cost: \$${cost} (at \$${price}/kWh)"
    fi

    echo ""
}

# Calculate charging cost
calc_cost() {
    local energy=${1:-45}
    local price=${2:-0.35}
    local time=${3:-30}
    local session_fee=${4:-2.00}

    print_section "Charging Cost Calculation"
    print_info "Energy Delivered: $energy kWh"
    print_info "Energy Price: \$${price}/kWh"
    print_info "Session Duration: $time minutes"
    print_info "Session Fee: \$${session_fee}"

    # Energy cost
    local energy_cost=$(echo "scale=2; $energy * $price" | bc -l)

    # Total cost
    local total=$(echo "scale=2; $energy_cost + $session_fee" | bc -l)

    print_section "Cost Breakdown"
    print_info "Energy Cost: \$${energy_cost}"
    print_info "Session Fee: \$${session_fee}"
    print_success "Total Cost: \$${total}"

    # Cost per kWh (effective)
    local effective_rate=$(echo "scale=3; $total / $energy" | bc -l)
    print_info "Effective Rate: \$${effective_rate}/kWh"

    # Cost per mile (assuming 250 Wh/mile)
    local miles=$(echo "scale=0; $energy * 1000 / 250" | bc -l)
    local cost_per_mile=$(echo "scale=3; $total / $miles" | bc -l)
    print_info "Cost per Mile: \$${cost_per_mile} (for ${miles} miles range)"

    echo ""
}

# Validate charging session
validate_session() {
    local connector=${1:-CCS}
    local power=${2:-150}
    local vehicle=${3:-"BEV"}

    print_section "Session Validation"
    print_info "Connector Type: $connector"
    print_info "Requested Power: $power kW"
    print_info "Vehicle Type: $vehicle"

    # Connector compatibility
    case $connector in
        CCS|CCS1|CCS2)
            print_success "Connector: CCS (Combined Charging System)"
            print_info "Max Power: 350 kW"
            print_info "Max Voltage: 920V DC"
            print_info "Max Current: 500A"
            print_info "V2G Capable: Yes"
            ;;
        CHAdeMO)
            print_success "Connector: CHAdeMO"
            print_info "Max Power: 400 kW (v3.0)"
            print_info "Max Voltage: 1000V DC"
            print_info "Max Current: 400A"
            print_info "V2G Capable: Yes"
            ;;
        Tesla)
            print_success "Connector: Tesla Supercharger"
            print_info "Max Power: 250 kW"
            print_info "Max Voltage: 500V DC"
            print_info "Max Current: 500A"
            print_info "V2G Capable: No"
            ;;
        J1772)
            print_success "Connector: SAE J1772 (Level 1/2)"
            print_info "Max Power: 19.2 kW"
            print_info "Max Voltage: 240V AC"
            print_info "Max Current: 80A"
            print_info "V2G Capable: No"
            ;;
        Type2)
            print_success "Connector: IEC Type 2 (Mennekes)"
            print_info "Max Power: 43 kW"
            print_info "Max Voltage: 400V AC"
            print_info "Max Current: 63A"
            print_info "V2G Capable: No"
            ;;
        *)
            print_error "Unknown connector type: $connector"
            ;;
    esac

    print_section "Recommendations"

    if [ "$vehicle" == "BEV" ]; then
        print_info "For battery longevity, charge to 80% for daily use"
        print_info "Use 100% charge only for long trips"
    elif [ "$vehicle" == "PHEV" ]; then
        print_info "Fully charge PHEV battery for maximum electric range"
    fi

    if (( $(echo "$power > 150" | bc -l) )); then
        print_warning "High power charging (>150 kW) may impact battery health"
        print_info "Consider temperature preconditioning for optimal charging"
    fi

    echo ""
}

# Get connector information
connector_info() {
    local type=${1:-all}

    print_section "Connector Standards"

    if [ "$type" == "all" ]; then
        echo ""
        echo -e "${CYAN}CCS1 (North America)${RESET}"
        print_info "Type: DC + AC, Max: 350 kW, V2G: Yes"

        echo ""
        echo -e "${CYAN}CCS2 (Europe/Global)${RESET}"
        print_info "Type: DC + AC, Max: 350 kW, V2G: Yes"

        echo ""
        echo -e "${CYAN}CHAdeMO (Japan/Asia)${RESET}"
        print_info "Type: DC, Max: 400 kW, V2G: Yes"

        echo ""
        echo -e "${CYAN}Tesla Supercharger${RESET}"
        print_info "Type: DC, Max: 250 kW, V2G: No"

        echo ""
        echo -e "${CYAN}J1772 (Level 1/2)${RESET}"
        print_info "Type: AC, Max: 19.2 kW, V2G: No"

        echo ""
        echo -e "${CYAN}Type 2 (Mennekes)${RESET}"
        print_info "Type: AC, Max: 43 kW, V2G: No"

        echo ""
        echo -e "${CYAN}GB/T (China)${RESET}"
        print_info "Type: DC + AC, Max: 237.5 kW, V2G: Yes"
    else
        validate_session "$type" 0 ""
    fi

    echo ""
}

# Compare charging levels
compare_levels() {
    print_section "Charging Levels Comparison"

    echo ""
    echo -e "${CYAN}Level 1 (AC)${RESET}"
    print_info "Power: 1.4-1.9 kW"
    print_info "Voltage: 120V (NA) / 230V (EU)"
    print_info "Use: Home overnight charging"
    print_info "Time: 20-40 hours (0-80%)"

    echo ""
    echo -e "${CYAN}Level 2 (AC)${RESET}"
    print_info "Power: 3.3-19.2 kW"
    print_info "Voltage: 240V"
    print_info "Use: Home wallbox, public charging"
    print_info "Time: 4-8 hours (0-80%)"

    echo ""
    echo -e "${CYAN}Level 3 (DC Fast Charging)${RESET}"
    print_info "Power: 50-350 kW"
    print_info "Voltage: 400-800V DC"
    print_info "Use: Highway, commercial"
    print_info "Time: 15-45 minutes (0-80%)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-005 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-time                Calculate charging time"
    echo "    --battery <kWh>        Battery capacity (default: 75 kWh)"
    echo "    --power <kW>           Charging power (default: 150 kW)"
    echo "    --from <percent>       Current SOC (default: 20%)"
    echo "    --to <percent>         Target SOC (default: 80%)"
    echo "    --price <\$/kWh>        Energy price (optional)"
    echo ""
    echo "  calc-cost                Calculate charging cost"
    echo "    --energy <kWh>         Energy delivered (default: 45 kWh)"
    echo "    --price <\$/kWh>        Energy price (default: \$0.35/kWh)"
    echo "    --time <minutes>       Session duration (default: 30 min)"
    echo "    --session-fee <\$>      Session fee (default: \$2.00)"
    echo ""
    echo "  validate                 Validate charging session"
    echo "    --connector <type>     Connector type (CCS, CHAdeMO, Tesla, J1772, Type2)"
    echo "    --power <kW>           Requested power"
    echo "    --vehicle <type>       Vehicle type (BEV, PHEV)"
    echo ""
    echo "  connector-info           Show connector information"
    echo "    --type <connector>     Connector type (or 'all' for all)"
    echo ""
    echo "  compare-levels           Compare charging levels 1, 2, and 3"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-005 calc-time --battery 75 --power 150 --from 20 --to 80"
    echo "  wia-auto-005 calc-cost --energy 45 --price 0.35 --time 30"
    echo "  wia-auto-005 validate --connector CCS --power 150 --vehicle BEV"
    echo "  wia-auto-005 connector-info --type CCS"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-005 EV Charging CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    calc-time)
        BATTERY=75
        POWER=150
        FROM=20
        TO=80
        PRICE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --battery) BATTERY=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --price) PRICE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_time "$BATTERY" "$POWER" "$FROM" "$TO" "$PRICE"
        ;;

    calc-cost)
        ENERGY=45
        PRICE=0.35
        TIME=30
        SESSION_FEE=2.00

        while [[ $# -gt 0 ]]; do
            case $1 in
                --energy) ENERGY=$2; shift 2 ;;
                --price) PRICE=$2; shift 2 ;;
                --time) TIME=$2; shift 2 ;;
                --session-fee) SESSION_FEE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_cost "$ENERGY" "$PRICE" "$TIME" "$SESSION_FEE"
        ;;

    validate)
        CONNECTOR="CCS"
        POWER=150
        VEHICLE="BEV"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --connector) CONNECTOR=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                --vehicle) VEHICLE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_session "$CONNECTOR" "$POWER" "$VEHICLE"
        ;;

    connector-info)
        TYPE="all"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        connector_info "$TYPE"
        ;;

    compare-levels)
        print_header
        compare_levels
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-005 help' for usage information"
        exit 1
        ;;
esac

exit 0
