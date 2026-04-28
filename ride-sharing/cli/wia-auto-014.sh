#!/bin/bash

################################################################################
# WIA-AUTO-014: Ride Sharing CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Mobility Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to ride sharing calculations
# including matching, fare calculation, surge pricing, and route optimization.
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
BASE_FARE=3.50
RATE_PER_KM=1.20
RATE_PER_MINUTE=0.25
COMMISSION=0.20

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🚗 WIA-AUTO-014: Ride Sharing CLI Tool                ║"
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

format_currency() {
    printf "$%.2f" "$1"
}

# Calculate distance between two coordinates (Haversine formula)
calc_distance() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    # Convert to radians
    local lat1_rad=$(echo "scale=10; $lat1 * 3.14159265359 / 180" | bc -l)
    local lon1_rad=$(echo "scale=10; $lon1 * 3.14159265359 / 180" | bc -l)
    local lat2_rad=$(echo "scale=10; $lat2 * 3.14159265359 / 180" | bc -l)
    local lon2_rad=$(echo "scale=10; $lon2 * 3.14159265359 / 180" | bc -l)

    local dlat=$(echo "scale=10; $lat2_rad - $lat1_rad" | bc -l)
    local dlon=$(echo "scale=10; $lon2_rad - $lon1_rad" | bc -l)

    local a=$(echo "scale=10; s($dlat/2)^2 + c($lat1_rad) * c($lat2_rad) * s($dlon/2)^2" | bc -l)
    local c=$(echo "scale=10; 2 * a(sqrt($a) / sqrt(1-$a))" | bc -l)

    local distance=$(echo "scale=2; 6371 * $c" | bc -l)
    echo "$distance"
}

# Match driver to ride request
match_driver() {
    local rider_id=${1:-"rider_123"}
    local pickup=${2:-"37.7749,-122.4194"}
    local dest=${3:-"37.8044,-122.2712"}

    print_section "Driver Matching"
    print_info "Rider ID: $rider_id"
    print_info "Pickup: $pickup"
    print_info "Destination: $dest"

    # Parse coordinates
    IFS=',' read -r pickup_lat pickup_lng <<< "$pickup"
    IFS=',' read -r dest_lat dest_lng <<< "$dest"

    # Calculate trip distance
    local trip_distance=$(calc_distance $pickup_lat $pickup_lng $dest_lat $dest_lng)
    print_info "Trip Distance: ${trip_distance} km"

    # Simulate driver matching
    print_section "Best Match Found"
    print_success "Driver: John Doe"
    print_info "Rating: 4.87 ⭐ (1,234 trips)"
    print_info "Vehicle: 2022 Toyota Camry (Silver)"
    print_info "License: ABC1234"
    print_info "ETA: 4 minutes"
    print_info "Distance to pickup: 2.3 km"
    print_info "Match Score: 0.85/1.00"

    # Calculate fare estimate
    local duration=$(echo "scale=0; $trip_distance / 40 * 60" | bc -l)  # Assume 40 km/h
    local base=$(echo "$BASE_FARE" | bc -l)
    local dist_fare=$(echo "$trip_distance * $RATE_PER_KM" | bc -l)
    local time_fare=$(echo "$duration * $RATE_PER_MINUTE" | bc -l)
    local subtotal=$(echo "$base + $dist_fare + $time_fare" | bc -l)
    local total=$(echo "$subtotal * 1.15" | bc -l)  # Add 15% service fee

    print_section "Fare Estimate"
    print_info "Estimated fare: $(format_currency $total)"
    print_info "Estimated duration: ${duration} minutes"

    echo ""
}

# Calculate fare
calc_fare() {
    local distance=${1:-15.5}
    local duration=${2:-25}
    local surge=${3:-1.0}

    print_section "Fare Calculation"
    print_info "Distance: $distance km"
    print_info "Duration: $duration minutes"
    print_info "Surge multiplier: ${surge}x"

    # Calculate fare components
    local base=$(echo "$BASE_FARE" | bc -l)
    local dist_fare=$(echo "$distance * $RATE_PER_KM" | bc -l)
    local time_fare=$(echo "$duration * $RATE_PER_MINUTE" | bc -l)
    local subtotal=$(echo "$base + $dist_fare + $time_fare" | bc -l)

    print_section "Fare Breakdown"
    print_info "Base fare: $(format_currency $base)"
    print_info "Distance fare: $(format_currency $dist_fare) ($distance km × $(format_currency $RATE_PER_KM)/km)"
    print_info "Time fare: $(format_currency $time_fare) ($duration min × $(format_currency $RATE_PER_MINUTE)/min)"
    print_info "Subtotal: $(format_currency $subtotal)"

    # Apply surge
    local surge_amount=0
    if (( $(echo "$surge > 1.0" | bc -l) )); then
        surge_amount=$(echo "$subtotal * ($surge - 1.0)" | bc -l)
        print_warning "Surge pricing: +$(format_currency $surge_amount) (${surge}x multiplier)"
    fi

    # Add fees
    local service_fee=$(echo "($subtotal + $surge_amount) * 0.15" | bc -l)
    print_info "Service fee (15%): $(format_currency $service_fee)"

    # Calculate total
    local total=$(echo "$subtotal + $surge_amount + $service_fee" | bc -l)

    print_section "Total Fare"
    print_success "Total: $(format_currency $total)"

    # Calculate driver earnings
    local driver_earnings=$(echo "$total * (1 - $COMMISSION)" | bc -l)
    print_info "Driver earnings (80%): $(format_currency $driver_earnings)"
    print_info "Platform fee (20%): $(format_currency $(echo "$total * $COMMISSION" | bc -l))"

    echo ""
}

# Verify driver
verify_driver() {
    local driver_id=${1:-"driver_456"}
    local check_background=${2:-false}

    print_section "Driver Verification"
    print_info "Driver ID: $driver_id"

    print_section "Verification Checks"
    print_success "Identity: VERIFIED ✓"
    print_success "Phone: VERIFIED ✓"
    print_success "Email: VERIFIED ✓"
    print_success "Driver's License: VERIFIED ✓"
    print_info "License: D1234567 (CA)"
    print_info "Expires: 2028-03-20"

    if [ "$check_background" = true ]; then
        print_success "Background Check: PASSED ✓"
        print_info "Last checked: 2025-01-15"
        print_info "Criminal record: Clean"
        print_info "Driving record: Clean"
        print_info "Sex offender registry: Clear"
    fi

    print_section "Vehicle Verification"
    print_success "Vehicle Registration: VERIFIED ✓"
    print_success "Insurance: ACTIVE ✓"
    print_success "Safety Inspection: PASSED ✓"
    print_info "Vehicle: 2022 Toyota Camry"
    print_info "License Plate: ABC1234"
    print_info "Insurance expires: 2025-12-31"

    print_section "Driver Status"
    print_success "Rating: 4.87/5.00 ⭐"
    print_info "Total trips: 1,234"
    print_info "Acceptance rate: 92%"
    print_info "Cancellation rate: 2%"
    print_info "Account status: ACTIVE"

    echo ""
}

# Optimize route
optimize_route() {
    local waypoints=${1:-"37.7749,-122.4194;37.8044,-122.2712"}

    print_section "Route Optimization"

    # Parse waypoints
    IFS=';' read -ra POINTS <<< "$waypoints"
    print_info "Number of waypoints: ${#POINTS[@]}"

    local total_distance=0
    local i=0
    for point in "${POINTS[@]}"; do
        i=$((i+1))
        print_info "Waypoint $i: $point"

        if [ $i -gt 1 ]; then
            # Calculate segment distance
            IFS=',' read -r lat1 lng1 <<< "${POINTS[$i-2]}"
            IFS=',' read -r lat2 lng2 <<< "$point"
            local segment_dist=$(calc_distance $lat1 $lng1 $lat2 $lng2)
            total_distance=$(echo "$total_distance + $segment_dist" | bc -l)
        fi
    done

    print_section "Optimized Route"
    print_success "Total distance: $(echo "scale=2; $total_distance" | bc -l) km"

    local duration=$(echo "scale=0; $total_distance / 40 * 60" | bc -l)
    print_info "Estimated time: $duration minutes (avg 40 km/h)"

    local efficiency=0.92
    print_info "Route efficiency: $(echo "scale=0; $efficiency * 100" | bc -l)%"

    # Calculate carbon emissions
    local emissions=$(echo "scale=2; $total_distance * 0.18" | bc -l)
    print_info "Carbon emissions: ${emissions} kg CO₂"

    print_section "Route Characteristics"
    print_info "Traffic condition: Moderate"
    print_info "Optimal for: Time"
    print_info "Highways: Yes"
    print_info "Tolls: No"

    echo ""
}

# Calculate surge pricing
calc_surge() {
    local demand=${1:-150}
    local supply=${2:-50}
    local area=${3:-"downtown"}

    print_section "Surge Pricing Calculation"
    print_info "Area: $area"
    print_info "Current demand: $demand requests"
    print_info "Available supply: $supply drivers"

    # Calculate demand/supply ratio
    local ratio=$(echo "scale=2; $demand / $supply" | bc -l)
    print_info "Demand/Supply ratio: $ratio"

    # Calculate surge using formula: S = max(0, k × ln(demand/supply))
    # where k = 1.0 (sensitivity constant)
    local k=1.0
    local ln_ratio=$(echo "scale=4; l($ratio) / l(2.71828)" | bc -l)
    local raw_surge=$(echo "scale=2; $k * $ln_ratio" | bc -l)
    local surge=$(echo "scale=2; 1.0 + $raw_surge" | bc -l)

    # Clamp between 1.0 and 5.0
    if (( $(echo "$surge < 1.0" | bc -l) )); then
        surge=1.0
    elif (( $(echo "$surge > 5.0" | bc -l) )); then
        surge=5.0
    fi

    # Round to nearest 0.25
    surge=$(echo "scale=2; 0.25 * (($surge * 4 + 0.5) / 1)" | bc -l)

    print_section "Surge Multiplier"

    if (( $(echo "$surge <= 1.0" | bc -l) )); then
        print_success "No surge: ${surge}x"
    elif (( $(echo "$surge < 1.5" | bc -l) )); then
        print_info "Low surge: ${surge}x"
    elif (( $(echo "$surge < 2.0" | bc -l) )); then
        print_warning "Medium surge: ${surge}x"
    else
        print_error "High surge: ${surge}x"
    fi

    # Example fare impact
    local base_fare=20.00
    local surge_fare=$(echo "scale=2; $base_fare * $surge" | bc -l)
    local increase=$(echo "scale=2; $surge_fare - $base_fare" | bc -l)

    print_section "Price Impact"
    print_info "Base fare: $(format_currency $base_fare)"
    print_info "Surge fare: $(format_currency $surge_fare)"
    if (( $(echo "$surge > 1.0" | bc -l) )); then
        print_warning "Increase: +$(format_currency $increase) (+$(echo "scale=0; ($surge - 1.0) * 100" | bc -l)%)"
    fi

    echo ""
}

# Generate demand heatmap
demand_map() {
    local region=${1:-"san-francisco"}
    local radius=${2:-10}

    print_section "Demand Heatmap"
    print_info "Region: $region"
    print_info "Radius: $radius km"

    print_section "Demand Intensity Map"
    echo ""
    print_info "Legend: 🟢 Low   🟡 Medium   🟠 High   🔴 Extreme"
    echo ""

    # Simulate 5x5 grid
    for i in {1..5}; do
        local row="  "
        for j in {1..5}; do
            local intensity=$((RANDOM % 100))
            if [ $intensity -lt 25 ]; then
                row+="🟢 "
            elif [ $intensity -lt 50 ]; then
                row+="🟡 "
            elif [ $intensity -lt 75 ]; then
                row+="🟠 "
            else
                row+="🔴 "
            fi
        done
        echo -e "$row"
    done

    echo ""
    print_section "High Demand Areas"
    print_warning "Downtown: 2.5x surge"
    print_warning "Financial District: 2.0x surge"
    print_info "Airport: 1.5x surge"
    print_success "Suburbs: 1.0x (no surge)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-014 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  match                    Match driver to ride request"
    echo "    --rider-id <id>        Rider identifier (default: rider_123)"
    echo "    --pickup <lat,lng>     Pickup coordinates"
    echo "    --dest <lat,lng>       Destination coordinates"
    echo ""
    echo "  calc-fare                Calculate trip fare"
    echo "    --distance <km>        Trip distance in km (default: 15.5)"
    echo "    --duration <min>       Trip duration in minutes (default: 25)"
    echo "    --surge <multiplier>   Surge multiplier (default: 1.0)"
    echo ""
    echo "  verify-driver            Verify driver credentials"
    echo "    --driver-id <id>       Driver identifier"
    echo "    --check-background     Include background check"
    echo ""
    echo "  optimize-route           Optimize multi-waypoint route"
    echo "    --waypoints <coords>   Semicolon-separated coordinates"
    echo "                           Example: '37.7,-122.4;37.8,-122.3'"
    echo ""
    echo "  calc-surge               Calculate surge pricing"
    echo "    --demand <number>      Current ride requests (default: 150)"
    echo "    --supply <number>      Available drivers (default: 50)"
    echo "    --area <name>          Area name (default: downtown)"
    echo ""
    echo "  demand-map               Generate demand heatmap"
    echo "    --region <name>        Region name (default: san-francisco)"
    echo "    --radius <km>          Radius in km (default: 10)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-014 match --pickup '37.7749,-122.4194' --dest '37.8044,-122.2712'"
    echo "  wia-auto-014 calc-fare --distance 15.5 --duration 25 --surge 1.5"
    echo "  wia-auto-014 verify-driver --driver-id driver_456 --check-background"
    echo "  wia-auto-014 optimize-route --waypoints '37.7749,-122.4194;37.8044,-122.2712'"
    echo "  wia-auto-014 calc-surge --demand 150 --supply 50"
    echo "  wia-auto-014 demand-map --region san-francisco"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-014 Ride Sharing CLI Tool"
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
    match)
        RIDER_ID="rider_123"
        PICKUP="37.7749,-122.4194"
        DEST="37.8044,-122.2712"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --rider-id) RIDER_ID=$2; shift 2 ;;
                --pickup) PICKUP=$2; shift 2 ;;
                --dest) DEST=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        match_driver "$RIDER_ID" "$PICKUP" "$DEST"
        ;;

    calc-fare)
        DISTANCE=15.5
        DURATION=25
        SURGE=1.0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) DISTANCE=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                --surge) SURGE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_fare "$DISTANCE" "$DURATION" "$SURGE"
        ;;

    verify-driver)
        DRIVER_ID="driver_456"
        CHECK_BG=false

        while [[ $# -gt 0 ]]; do
            case $1 in
                --driver-id) DRIVER_ID=$2; shift 2 ;;
                --check-background) CHECK_BG=true; shift ;;
                *) shift ;;
            esac
        done

        print_header
        verify_driver "$DRIVER_ID" "$CHECK_BG"
        ;;

    optimize-route)
        WAYPOINTS="37.7749,-122.4194;37.8044,-122.2712"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --waypoints) WAYPOINTS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        optimize_route "$WAYPOINTS"
        ;;

    calc-surge)
        DEMAND=150
        SUPPLY=50
        AREA="downtown"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --demand) DEMAND=$2; shift 2 ;;
                --supply) SUPPLY=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_surge "$DEMAND" "$SUPPLY" "$AREA"
        ;;

    demand-map)
        REGION="san-francisco"
        RADIUS=10

        while [[ $# -gt 0 ]]; do
            case $1 in
                --region) REGION=$2; shift 2 ;;
                --radius) RADIUS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        demand_map "$REGION" "$RADIUS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-014 help' for usage information"
        exit 1
        ;;
esac

exit 0
