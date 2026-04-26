#!/bin/bash

################################################################################
# WIA-IND-016: Hotel Tech CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to hotel technology operations
# including reservations, room management, revenue management, and guest services.
################################################################################

set -e

# Colors for output
AMBER='\033[0;38;5;214m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
PROPERTY_ID="hotel-001"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║            🏨  WIA-IND-016: Hotel Tech CLI                    ║"
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

# Generate confirmation number
generate_confirmation() {
    echo $(cat /dev/urandom | tr -dc 'A-Z0-9' | fold -w 6 | head -n 1)
}

# Calculate nights between dates
calculate_nights() {
    local checkin=$1
    local checkout=$2
    local checkin_epoch=$(date -d "$checkin" +%s 2>/dev/null || date -j -f "%Y-%m-%d" "$checkin" +%s)
    local checkout_epoch=$(date -d "$checkout" +%s 2>/dev/null || date -j -f "%Y-%m-%d" "$checkout" +%s)
    echo $(( ($checkout_epoch - $checkin_epoch) / 86400 ))
}

# Create reservation
create_reservation() {
    local guest_name=""
    local email=""
    local phone=""
    local checkin=""
    local checkout=""
    local room_type="deluxe"
    local adults=2
    local children=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --guest) guest_name="$2"; shift 2 ;;
            --email) email="$2"; shift 2 ;;
            --phone) phone="$2"; shift 2 ;;
            --checkin) checkin="$2"; shift 2 ;;
            --checkout) checkout="$2"; shift 2 ;;
            --room-type) room_type="$2"; shift 2 ;;
            --adults) adults="$2"; shift 2 ;;
            --children) children="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Creating Reservation"

    if [ -z "$guest_name" ] || [ -z "$checkin" ] || [ -z "$checkout" ]; then
        print_error "Missing required parameters"
        print_info "Usage: wia-ind-016 reserve --guest \"Name\" --checkin YYYY-MM-DD --checkout YYYY-MM-DD"
        return 1
    fi

    local confirmation=$(generate_confirmation)
    local nights=$(calculate_nights "$checkin" "$checkout")

    # Calculate rates
    local room_rate=250
    case $room_type in
        standard) room_rate=150 ;;
        deluxe) room_rate=250 ;;
        suite) room_rate=450 ;;
        *) room_rate=250 ;;
    esac

    local subtotal=$((room_rate * nights))
    local taxes=$((subtotal * 15 / 100))
    local total=$((subtotal + taxes))

    print_section "Reservation Confirmed"
    print_success "Confirmation Number: $confirmation"
    echo ""
    print_info "Guest: $guest_name"
    [ -n "$email" ] && print_info "Email: $email"
    [ -n "$phone" ] && print_info "Phone: $phone"
    echo ""
    print_info "Check-in: $checkin (3:00 PM)"
    print_info "Check-out: $checkout (11:00 AM)"
    print_info "Nights: $nights"
    echo ""
    print_info "Room Type: $room_type"
    print_info "Guests: $adults adults, $children children"
    echo ""
    print_section "Rate Breakdown"
    print_info "Room Rate: \$${room_rate} × ${nights} nights = \$${subtotal}"
    print_info "Taxes & Fees (15%): \$${taxes}"
    echo ""
    print_success "Total Amount: \$${total}"
    echo ""
    print_info "Cancellation Policy: Free cancellation until 24 hours before check-in"
    echo ""
}

# Check availability
check_availability() {
    local date=""
    local nights=1
    local adults=2

    while [[ $# -gt 0 ]]; do
        case $1 in
            --date) date="$2"; shift 2 ;;
            --nights) nights="$2"; shift 2 ;;
            --adults) adults="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Room Availability Search"
    print_info "Date: $date"
    print_info "Nights: $nights"
    print_info "Guests: $adults"

    echo ""
    print_section "Available Rooms"

    # Simulate availability
    echo -e "${GREEN}[1]${RESET} Standard Double"
    print_info "  Available: 5 rooms"
    print_info "  Rate: \$150/night"
    print_info "  Occupancy: 2 adults"
    echo ""

    echo -e "${GREEN}[2]${RESET} Deluxe King"
    print_info "  Available: 8 rooms"
    print_info "  Rate: \$250/night"
    print_info "  Occupancy: 2 adults"
    echo ""

    echo -e "${GREEN}[3]${RESET} Junior Suite"
    print_info "  Available: 3 rooms"
    print_info "  Rate: \$380/night"
    print_info "  Occupancy: 4 adults"
    echo ""

    echo -e "${GREEN}[4]${RESET} Executive Suite"
    print_info "  Available: 1 room"
    print_info "  Rate: \$520/night"
    print_info "  Occupancy: 4 adults"
    echo ""
}

# Guest check-in
guest_checkin() {
    local confirmation=""
    local room=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --confirmation) confirmation="$2"; shift 2 ;;
            --room) room="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Guest Check-In"

    if [ -z "$confirmation" ]; then
        print_error "Confirmation number required"
        return 1
    fi

    if [ -z "$room" ]; then
        room="305"
        print_info "Assigning room: $room"
    fi

    echo ""
    print_success "Check-in Complete"
    print_info "Confirmation: $confirmation"
    print_info "Room: $room"
    echo ""
    print_section "Mobile Key"
    print_success "Mobile key sent to guest's phone"
    print_info "Access granted to:"
    print_info "  • Room $room"
    print_info "  • Elevator"
    print_info "  • Pool & Gym"
    print_info "  • Executive Lounge"
    echo ""
    print_info "WiFi Network: Hotel-Guest-5G"
    print_info "Password: Welcome2025"
    echo ""
}

# Guest check-out
guest_checkout() {
    local confirmation=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --confirmation) confirmation="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Guest Check-Out"

    if [ -z "$confirmation" ]; then
        print_error "Confirmation number required"
        return 1
    fi

    print_success "Check-out Complete"
    echo ""
    print_section "Final Folio"
    print_info "Room Charges (3 nights × \$250): \$750"
    print_info "Room Service: \$85"
    print_info "Mini Bar: \$35"
    print_info "Taxes & Fees (15%): \$130.50"
    echo ""
    print_success "Total Amount: \$1,000.50"
    print_info "Payment Method: Visa ending in 4242"
    echo ""
    print_info "Mobile key deactivated"
    print_info "Feedback survey sent to email"
    echo ""
}

# Update room status (housekeeping)
update_housekeeping() {
    local room=""
    local status="clean"
    local staff="housekeeper-01"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --room) room="$2"; shift 2 ;;
            --status) status="$2"; shift 2 ;;
            --staff) staff="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Housekeeping Update"

    if [ -z "$room" ]; then
        print_error "Room number required"
        return 1
    fi

    print_success "Room $room status updated to: $status"
    print_info "Staff: $staff"
    print_info "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"

    case $status in
        clean)
            print_info "Quality check: Passed"
            print_info "Ready for guest assignment"
            ;;
        dirty)
            print_info "Cleaning required"
            print_info "Estimated time: 30 minutes"
            ;;
        cleaning)
            print_info "Cleaning in progress"
            ;;
    esac
    echo ""
}

# Generate mobile key
generate_mobile_key() {
    local guest=""
    local room=""
    local valid_until=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --guest) guest="$2"; shift 2 ;;
            --room) room="$2"; shift 2 ;;
            --valid-until) valid_until="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Mobile Key Generation"

    if [ -z "$room" ]; then
        print_error "Room number required"
        return 1
    fi

    local key_id="KEY-$(cat /dev/urandom | tr -dc 'A-Z0-9' | fold -w 8 | head -n 1)"

    print_success "Mobile Key Generated"
    print_info "Key ID: $key_id"
    print_info "Room: $room"
    print_info "Valid Until: $valid_until"
    echo ""
    print_section "Access Permissions"
    print_info "✓ Room $room"
    print_info "✓ Elevator access"
    print_info "✓ Pool & Gym"
    print_info "✓ Parking garage"
    echo ""
    print_info "Key sent via push notification"
    echo ""
}

# Dynamic pricing analysis
analyze_pricing() {
    local room_type="deluxe"
    local date=""
    local occupancy=0.75

    while [[ $# -gt 0 ]]; do
        case $1 in
            --room-type) room_type="$2"; shift 2 ;;
            --date) date="$2"; shift 2 ;;
            --occupancy) occupancy="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Dynamic Pricing Analysis"
    print_info "Room Type: $room_type"
    print_info "Date: $date"
    print_info "Forecasted Occupancy: $(echo "$occupancy * 100" | bc)%"

    echo ""
    print_section "Pricing Recommendation"

    # Calculate dynamic rate
    local base_rate=250
    local demand_multiplier=1.0

    if (( $(echo "$occupancy > 0.85" | bc -l) )); then
        demand_multiplier=1.25
        print_info "Demand Level: Very High"
    elif (( $(echo "$occupancy > 0.70" | bc -l) )); then
        demand_multiplier=1.10
        print_info "Demand Level: High"
    elif (( $(echo "$occupancy > 0.50" | bc -l) )); then
        demand_multiplier=1.0
        print_info "Demand Level: Medium"
    else
        demand_multiplier=0.85
        print_info "Demand Level: Low"
    fi

    local recommended_rate=$(echo "$base_rate * $demand_multiplier" | bc | cut -d'.' -f1)

    echo ""
    print_info "Base Rate: \$${base_rate}"
    print_success "Recommended Rate: \$${recommended_rate}"

    echo ""
    print_section "Pricing Factors"
    print_info "• Occupancy forecast: $(echo "($recommended_rate - $base_rate) * 100 / $base_rate" | bc)%"
    print_info "• Day of week: Weekend (+10%)"
    print_info "• Local events: Conference in town (+15%)"

    echo ""
    print_section "Competitor Rates"
    print_info "• Hotel A: \$$(echo "$recommended_rate * 0.95" | bc | cut -d'.' -f1)"
    print_info "• Hotel B: \$$(echo "$recommended_rate * 1.05" | bc | cut -d'.' -f1)"
    print_info "• Hotel C: \$$(echo "$recommended_rate * 0.98" | bc | cut -d'.' -f1)"

    echo ""
    print_info "Confidence Score: 85%"
    echo ""
}

# Channel sync
sync_channels() {
    local update_rates=false
    local update_inventory=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --update-rates) update_rates=true; shift ;;
            --update-inventory) update_inventory=true; shift ;;
            *) shift ;;
        esac
    done

    print_section "Channel Manager Sync"
    print_info "Syncing with OTA channels..."

    sleep 1

    echo ""
    print_section "Sync Results"

    echo -e "${GREEN}[1]${RESET} Booking.com"
    print_success "Status: Connected"
    $update_rates && print_info "  Rates updated: 45 room types"
    $update_inventory && print_info "  Inventory updated: 120 rooms"
    print_info "  Last sync: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""

    echo -e "${GREEN}[2]${RESET} Expedia"
    print_success "Status: Connected"
    $update_rates && print_info "  Rates updated: 45 room types"
    $update_inventory && print_info "  Inventory updated: 120 rooms"
    print_info "  Last sync: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""

    echo -e "${GREEN}[3]${RESET} Airbnb"
    print_success "Status: Connected"
    $update_rates && print_info "  Rates updated: 15 listings"
    $update_inventory && print_info "  Inventory updated: 30 units"
    print_info "  Last sync: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""

    print_success "All channels synchronized successfully"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-016 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo -e "${CYAN}Reservations:${RESET}"
    echo "  reserve              Create a new reservation"
    echo "  availability         Check room availability"
    echo "  checkin              Guest check-in"
    echo "  checkout             Guest check-out"
    echo ""
    echo -e "${CYAN}Room Management:${RESET}"
    echo "  housekeeping         Update room status"
    echo "  mobile-key           Generate mobile key"
    echo ""
    echo -e "${CYAN}Revenue Management:${RESET}"
    echo "  pricing              Dynamic pricing analysis"
    echo ""
    echo -e "${CYAN}Channel Management:${RESET}"
    echo "  channel-sync         Sync with OTA channels"
    echo ""
    echo -e "${CYAN}System:${RESET}"
    echo "  version              Show version"
    echo "  help                 Show this help"
    echo ""
    echo "Examples:"
    echo ""
    echo "  # Create reservation"
    echo "  wia-ind-016 reserve --guest \"John Smith\" --checkin 2025-01-20 --checkout 2025-01-22"
    echo ""
    echo "  # Check availability"
    echo "  wia-ind-016 availability --date 2025-01-15 --nights 3"
    echo ""
    echo "  # Update room status"
    echo "  wia-ind-016 housekeeping --room 305 --status clean"
    echo ""
    echo "  # Dynamic pricing"
    echo "  wia-ind-016 pricing --room-type deluxe --date 2025-02-14"
    echo ""
    echo -e "${AMBER}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-IND-016 Hotel Tech CLI v${VERSION}"
    echo ""
    echo -e "${GRAY}Standard: WIA-IND-016${RESET}"
    echo -e "${GRAY}Category: Industry / Hospitality${RESET}"
    echo -e "${GRAY}License: MIT${RESET}"
    echo ""
}

# Main command router
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case $1 in
        reserve)
            shift
            create_reservation "$@"
            ;;
        availability)
            shift
            check_availability "$@"
            ;;
        checkin)
            shift
            guest_checkin "$@"
            ;;
        checkout)
            shift
            guest_checkout "$@"
            ;;
        housekeeping)
            shift
            update_housekeeping "$@"
            ;;
        mobile-key)
            shift
            generate_mobile_key "$@"
            ;;
        pricing)
            shift
            analyze_pricing "$@"
            ;;
        channel-sync)
            shift
            sync_channels "$@"
            ;;
        version)
            show_version
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main
main "$@"
