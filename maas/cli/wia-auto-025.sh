#!/bin/bash

################################################################################
# WIA-AUTO-025: Mobility-as-a-Service CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive & Mobility Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to MaaS journey planning,
# booking, real-time updates, and carbon tracking functionality.
################################################################################

set -e

# Colors for output
ORANGE='\033[38;5;208m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
API_BASE_URL="${WIA_MAAS_API_URL:-https://api.maas-platform.com/v1}"
API_KEY="${WIA_MAAS_API_KEY:-demo-key}"

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     🚗 WIA-AUTO-025: Mobility-as-a-Service CLI Tool          ║"
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

format_duration() {
    local seconds=$1
    local hours=$((seconds / 3600))
    local minutes=$(( (seconds % 3600) / 60 ))

    if [ $hours -gt 0 ]; then
        printf "%dh %dm" "$hours" "$minutes"
    else
        printf "%dm" "$minutes"
    fi
}

format_distance() {
    local meters=$1

    if [ $meters -lt 1000 ]; then
        printf "%d m" "$meters"
    else
        printf "%.1f km" "$(echo "scale=1; $meters / 1000" | bc)"
    fi
}

# Plan a journey
plan_journey() {
    local from_lat=${1}
    local from_lng=${2}
    local to_lat=${3}
    local to_lng=${4}
    local time=${5:-$(date -Iseconds)}
    local optimize=${6:-time}

    print_section "Journey Planning"
    print_info "From: ($from_lat, $from_lng)"
    print_info "To: ($to_lat, $to_lng)"
    print_info "Departure: $time"
    print_info "Optimize: $optimize"

    # Simulate API call (in real implementation, use curl/wget)
    echo ""
    print_section "Available Routes"

    # Route 1: Public Transit + Walking
    echo ""
    echo -e "${GREEN}Route 1: Public Transit${RESET}"
    print_info "Duration: 42 minutes"
    print_info "Cost: $8.50"
    print_info "Carbon: 0.82 kg CO2 (85% savings vs car)"
    print_info "Transfers: 1"
    echo ""
    print_info "Segments:"
    print_info "  1. Walk to Metro Station (400m, 5 min)"
    print_info "  2. Metro Line 1 → Downtown (12 stops, 18 min)"
    print_info "  3. Transfer to Bus 42 (3 min)"
    print_info "  4. Bus 42 → Tech Park (8 stops, 14 min)"
    print_info "  5. Walk to destination (200m, 2 min)"

    # Route 2: Bike-share + Metro
    echo ""
    echo -e "${CYAN}Route 2: Bike + Metro${RESET}"
    print_info "Duration: 38 minutes"
    print_info "Cost: $6.00 (subscription)"
    print_info "Carbon: 0.45 kg CO2 (92% savings vs car)"
    print_info "Transfers: 1"
    echo ""
    print_info "Segments:"
    print_info "  1. Bike-share to Metro (1.2km, 6 min)"
    print_info "  2. Metro Line 1 → Central (10 stops, 15 min)"
    print_info "  3. Transfer to Train (5 min)"
    print_info "  4. Train → Tech Park (3 stops, 8 min)"
    print_info "  5. Walk to destination (300m, 4 min)"

    # Route 3: Ride-hail
    echo ""
    echo -e "${YELLOW}Route 3: Ride-hail${RESET}"
    print_info "Duration: 32 minutes"
    print_info "Cost: $28.50"
    print_info "Carbon: 4.8 kg CO2"
    print_info "Transfers: 0"
    echo ""
    print_info "Segments:"
    print_info "  1. Ride-hail direct (15.2km, 32 min)"

    echo ""
    print_section "Recommendation"
    print_success "Route 2 (Bike + Metro) - Best balance of time, cost, and sustainability"
    echo ""
}

# Get mobility services near a location
get_services() {
    local lat=${1}
    local lng=${2}
    local radius=${3:-500}

    print_section "Nearby Mobility Services"
    print_info "Location: ($lat, $lng)"
    print_info "Search radius: ${radius}m"

    echo ""
    print_section "Available Services"

    # Bike-share
    echo ""
    echo -e "${GREEN}Bike-share: BayWheels${RESET}"
    print_info "Distance: 120m"
    print_info "Available: 5 bikes"
    print_info "Pricing: $2 unlock + $0.15/min"
    print_info "Battery: N/A (pedal bikes)"

    # E-scooter
    echo ""
    echo -e "${GREEN}E-scooter: Lime${RESET}"
    print_info "Distance: 85m"
    print_info "Available: 3 scooters"
    print_info "Pricing: $1 unlock + $0.25/min"
    print_info "Battery: 85%, 72%, 91%"

    # Car-share
    echo ""
    echo -e "${CYAN}Car-share: Zipcar${RESET}"
    print_info "Distance: 280m"
    print_info "Available: 2 vehicles"
    print_info "Pricing: $12/hour or $89/day"
    print_info "Types: Compact Electric, SUV Hybrid"

    # Metro Station
    echo ""
    echo -e "${ORANGE}Metro Station: Market St${RESET}"
    print_info "Distance: 450m"
    print_info "Lines: 1, 2, 5"
    print_info "Next arrivals: 3 min, 8 min, 15 min"
    print_info "Wheelchair accessible: Yes"

    echo ""
}

# View subscription plans
view_plans() {
    local region=${1:-default}

    print_section "Subscription Plans for $region"

    echo ""
    echo -e "${GRAY}Basic (Pay-per-use)${RESET}"
    print_info "Monthly fee: $0"
    print_info "Credits: Pay as you go"
    print_info "Benefits: None"
    print_info "Best for: Occasional users (<10 trips/month)"

    echo ""
    echo -e "${GREEN}Light - $49/month${RESET}"
    print_info "Credits: 20 trips included"
    print_info "Benefits:"
    print_info "  • Public transit unlimited"
    print_info "  • 50% off bike/scooter share"
    print_info "Best for: Regular commuters"

    echo ""
    echo -e "${CYAN}Standard - $99/month${RESET}"
    print_info "Credits: 60 mixed-mode trips"
    print_info "Benefits:"
    print_info "  • All public transit unlimited"
    print_info "  • Bike/scooter included (up to 30 min/trip)"
    print_info "  • 20% off ride-hail"
    print_info "Best for: Daily users without car"

    echo ""
    echo -e "${ORANGE}Premium - $199/month${RESET}"
    print_info "Credits: Unlimited trips (all modes)"
    print_info "Benefits:"
    print_info "  • Priority booking"
    print_info "  • Carbon offset included"
    print_info "  • 24/7 support"
    print_info "  • Free airport transfers"
    print_info "Best for: Heavy users, business travelers"

    echo ""
    echo -e "${YELLOW}Family - $299/month${RESET}"
    print_info "Credits: Unlimited for up to 4 members"
    print_info "Benefits:"
    print_info "  • Shared credit pool"
    print_info "  • Kid-friendly features"
    print_info "  • Family safety tracking"
    print_info "Best for: Families without car"

    echo ""
}

# Book a trip
book_trip() {
    local journey_id=${1}
    local payment=${2:-subscription}

    print_section "Booking Journey"
    print_info "Journey ID: $journey_id"
    print_info "Payment method: $payment"

    echo ""
    print_info "Processing booking..."
    sleep 1

    print_section "Booking Confirmation"
    print_success "Booking confirmed!"
    print_info "Booking ID: B-$(date +%s)-ABC123"
    print_info "Status: Confirmed"
    print_info "Total cost: $8.50"
    print_info "Payment: Deducted from subscription credits"

    echo ""
    print_section "Your Tickets"
    print_info "Ticket 1: Metro - Valid for 2 hours"
    print_info "  QR Code: [████████] Scan at gate"
    print_info ""
    print_info "Ticket 2: Bus 42 - Valid for 2 hours"
    print_info "  QR Code: [████████] Show to driver"

    echo ""
    print_section "Journey Details"
    print_info "Departure: $(date -d '+5 minutes' '+%H:%M')"
    print_info "Estimated arrival: $(date -d '+47 minutes' '+%H:%M')"
    print_info "Real-time updates: Enabled"

    echo ""
    print_success "Have a safe journey! 🚇"
    echo ""
}

# Track carbon footprint
track_carbon() {
    local user_id=${1}
    local period=${2:-month}

    print_section "Carbon Footprint Report"
    print_info "User: $user_id"
    print_info "Period: Last $period"

    echo ""
    print_section "Summary"
    print_info "Total trips: 42"
    print_info "Total distance: 387 km"
    print_info "Total emissions: 15.3 kg CO2"
    print_info "Baseline (car): 74.3 kg CO2"
    print_info "Carbon saved: 59.0 kg CO2 (79% reduction)"

    echo ""
    print_section "Emissions by Mode"
    print_info "Metro:       8.2 kg CO2  (210 km, 18 trips)"
    print_info "Bus:         4.1 kg CO2  (95 km, 12 trips)"
    print_info "Bike-share:  0.2 kg CO2  (42 km, 8 trips)"
    print_info "Walking:     0.0 kg CO2  (18 km, 22 segments)"
    print_info "E-scooter:   0.4 kg CO2  (12 km, 3 trips)"
    print_info "Ride-hail:   2.4 kg CO2  (10 km, 1 trip)"

    echo ""
    print_section "Carbon Offset"
    print_info "Offset purchased: 15.3 kg CO2"
    print_info "Cost: $0.31"
    print_info "Project: Wind Energy Farm, Texas"
    print_info "Certification: Gold Standard"

    echo ""
    print_section "Trends"
    print_success "8% reduction vs last month"
    print_info "Average per trip: 0.36 kg CO2"
    print_info "Monthly target: <20 kg CO2 (on track ✓)"

    echo ""
    print_section "Recommendations"
    print_info "• Replace remaining ride-hail trips with bike-share"
    print_info "• Consider premium tier for better carbon tracking"
    print_info "• You're in top 15% of sustainable travelers!"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-025 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  plan                     Plan a multimodal journey"
    echo "    --from <lat,lng>       Origin coordinates"
    echo "    --to <lat,lng>         Destination coordinates"
    echo "    --time <datetime>      Departure time (default: now)"
    echo "    --optimize <metric>    Optimize for time/cost/carbon (default: time)"
    echo ""
    echo "  services                 Find nearby mobility services"
    echo "    --location <lat,lng>   Location coordinates"
    echo "    --radius <meters>      Search radius (default: 500m)"
    echo ""
    echo "  plans                    View subscription plans"
    echo "    --region <name>        Region name (default: default)"
    echo ""
    echo "  book                     Book a journey"
    echo "    --journey-id <id>      Journey ID from planning"
    echo "    --payment <method>     Payment method (default: subscription)"
    echo ""
    echo "  carbon                   Track carbon footprint"
    echo "    --user-id <id>         User ID"
    echo "    --period <time>        Time period (day/week/month/year)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-025 plan --from \"37.7749,-122.4194\" --to \"37.3861,-122.0839\""
    echo "  wia-auto-025 services --location \"37.7749,-122.4194\" --radius 1000"
    echo "  wia-auto-025 plans --region sf-bay-area"
    echo "  wia-auto-025 book --journey-id J123456 --payment subscription"
    echo "  wia-auto-025 carbon --user-id user123 --period month"
    echo ""
    echo "Environment Variables:"
    echo "  WIA_MAAS_API_URL       API base URL (default: https://api.maas-platform.com/v1)"
    echo "  WIA_MAAS_API_KEY       API authentication key"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-025: Mobility-as-a-Service CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "API Endpoint: $API_BASE_URL"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    plan)
        FROM=""
        TO=""
        TIME=$(date -Iseconds)
        OPTIMIZE="time"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --time) TIME=$2; shift 2 ;;
                --optimize) OPTIMIZE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$FROM" ] || [ -z "$TO" ]; then
            print_error "Missing required arguments: --from and --to"
            echo "Run 'wia-auto-025 help' for usage information"
            exit 1
        fi

        # Parse coordinates
        FROM_LAT=$(echo "$FROM" | cut -d',' -f1)
        FROM_LNG=$(echo "$FROM" | cut -d',' -f2)
        TO_LAT=$(echo "$TO" | cut -d',' -f1)
        TO_LNG=$(echo "$TO" | cut -d',' -f2)

        print_header
        plan_journey "$FROM_LAT" "$FROM_LNG" "$TO_LAT" "$TO_LNG" "$TIME" "$OPTIMIZE"
        ;;

    services)
        LOCATION=""
        RADIUS=500

        while [[ $# -gt 0 ]]; do
            case $1 in
                --location) LOCATION=$2; shift 2 ;;
                --radius) RADIUS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$LOCATION" ]; then
            print_error "Missing required argument: --location"
            exit 1
        fi

        LAT=$(echo "$LOCATION" | cut -d',' -f1)
        LNG=$(echo "$LOCATION" | cut -d',' -f2)

        print_header
        get_services "$LAT" "$LNG" "$RADIUS"
        ;;

    plans)
        REGION="default"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --region) REGION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        view_plans "$REGION"
        ;;

    book)
        JOURNEY_ID=""
        PAYMENT="subscription"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --journey-id) JOURNEY_ID=$2; shift 2 ;;
                --payment) PAYMENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$JOURNEY_ID" ]; then
            print_error "Missing required argument: --journey-id"
            exit 1
        fi

        print_header
        book_trip "$JOURNEY_ID" "$PAYMENT"
        ;;

    carbon)
        USER_ID=""
        PERIOD="month"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --user-id) USER_ID=$2; shift 2 ;;
                --period) PERIOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$USER_ID" ]; then
            print_error "Missing required argument: --user-id"
            exit 1
        fi

        print_header
        track_carbon "$USER_ID" "$PERIOD"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-025 help' for usage information"
        exit 1
        ;;
esac

exit 0
