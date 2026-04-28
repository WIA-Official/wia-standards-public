#!/bin/bash

################################################################################
# WIA-IND-015: Travel Tech CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to travel technology features
# including flight search, hotel booking, itinerary management, document
# verification, and currency conversion.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
AMBER='\033[38;5;214m'
MAGENTA='\033[0;35m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
API_BASE_URL="${WIA_API_URL:-https://api.wiastandards.com/ind-015/v1}"
CONFIG_DIR="$HOME/.wia/ind-015"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          ✈️  WIA-IND-015: Travel Tech CLI                     ║"
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

print_data() {
    echo -e "${BLUE}  $1${RESET}"
}

# Configuration management
load_config() {
    if [ -f "$CONFIG_FILE" ]; then
        cat "$CONFIG_FILE"
    else
        echo "{}"
    fi
}

save_config() {
    mkdir -p "$CONFIG_DIR"
    echo "$1" > "$CONFIG_FILE"
    chmod 600 "$CONFIG_FILE"
}

get_api_key() {
    local config=$(load_config)
    if command -v jq &> /dev/null; then
        echo "$config" | jq -r '.apiKey // empty'
    else
        echo ""
    fi
}

# Currency formatting
format_currency() {
    local amount=$1
    local currency=${2:-USD}

    if (( $(echo "$amount < 1000" | bc -l) )); then
        printf "%s %.2f" "$currency" "$amount"
    elif (( $(echo "$amount < 1000000" | bc -l) )); then
        printf "%s %'d" "$currency" "$(printf "%.0f" "$amount")"
    else
        printf "%s %.2fM" "$currency" "$(echo "$amount / 1000000" | bc -l)"
    fi
}

# Date formatting
format_date() {
    local date=$1
    date -d "$date" "+%b %d, %Y" 2>/dev/null || echo "$date"
}

format_time() {
    local datetime=$1
    date -d "$datetime" "+%H:%M" 2>/dev/null || echo "$datetime"
}

# Search flights
search_flights() {
    local origin=""
    local destination=""
    local date=""
    local return_date=""
    local passengers=1
    local class="economy"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --from) origin="$2"; shift 2 ;;
            --to) destination="$2"; shift 2 ;;
            --date) date="$2"; shift 2 ;;
            --return) return_date="$2"; shift 2 ;;
            --passengers) passengers="$2"; shift 2 ;;
            --class) class="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$origin" ] || [ -z "$destination" ] || [ -z "$date" ]; then
        print_error "Missing required parameters: --from, --to, --date"
        echo "Usage: wia-ind-015 search-flights --from JFK --to LHR --date 2025-06-15 [--return 2025-06-22] [--passengers 2] [--class economy]"
        return 1
    fi

    print_section "Searching Flights"
    print_info "Route: $origin → $destination"
    print_info "Departure: $(format_date $date)"
    [ -n "$return_date" ] && print_info "Return: $(format_date $return_date)"
    print_info "Passengers: $passengers"
    print_info "Class: $class"

    # Simulated search results
    echo ""
    print_success "Found 5 flight options"
    echo ""

    print_data "Option 1: Direct Flight"
    print_info "  Airline: American Airlines (AA)"
    print_info "  Flight: AA100"
    print_info "  Departure: 08:00 from $origin Terminal 8"
    print_info "  Arrival: 20:15 at $destination Terminal 3"
    print_info "  Duration: 7h 15m"
    print_info "  Price: $(format_currency 850 USD)"
    echo ""

    print_data "Option 2: One Stop"
    print_info "  Airline: United Airlines (UA)"
    print_info "  Flight: UA456 → UA789"
    print_info "  Departure: 10:30 from $origin"
    print_info "  Stop: ORD (1h 15m layover)"
    print_info "  Arrival: 21:45 at $destination"
    print_info "  Duration: 9h 15m"
    print_info "  Price: $(format_currency 675 USD)"
    echo ""

    print_data "Option 3: Premium Direct"
    print_info "  Airline: British Airways (BA)"
    print_info "  Flight: BA177"
    print_info "  Departure: 19:00 from $origin Terminal 7"
    print_info "  Arrival: 07:05+1 at $destination Terminal 5"
    print_info "  Duration: 7h 5m"
    print_info "  Class: Premium Economy"
    print_info "  Price: $(format_currency 1250 USD)"
    echo ""
}

# Book hotel
book_hotel() {
    local city=""
    local checkin=""
    local checkout=""
    local rooms=1
    local guests=2
    local stars=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --city) city="$2"; shift 2 ;;
            --checkin) checkin="$2"; shift 2 ;;
            --checkout) checkout="$2"; shift 2 ;;
            --rooms) rooms="$2"; shift 2 ;;
            --guests) guests="$2"; shift 2 ;;
            --stars) stars="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$city" ] || [ -z "$checkin" ] || [ -z "$checkout" ]; then
        print_error "Missing required parameters: --city, --checkin, --checkout"
        echo "Usage: wia-ind-015 book-hotel --city London --checkin 2025-06-15 --checkout 2025-06-22 [--rooms 1] [--guests 2] [--stars 4]"
        return 1
    fi

    # Calculate nights
    local nights=$(( ($(date -d "$checkout" +%s) - $(date -d "$checkin" +%s)) / 86400 ))

    print_section "Searching Hotels in $city"
    print_info "Check-in: $(format_date $checkin)"
    print_info "Check-out: $(format_date $checkout)"
    print_info "Nights: $nights"
    print_info "Rooms: $rooms, Guests: $guests"
    [ -n "$stars" ] && print_info "Minimum stars: $stars"

    echo ""
    print_success "Found 8 hotel options"
    echo ""

    print_data "Hotel 1: The Royal Grand Hotel ⭐⭐⭐⭐⭐"
    print_info "  Location: Central $city"
    print_info "  Rating: 9.2/10 (1,234 reviews)"
    print_info "  Room: Deluxe King with City View"
    print_info "  Amenities: WiFi, Breakfast, Pool, Spa, Gym"
    print_info "  Accessibility: Wheelchair accessible, elevator"
    print_info "  Price per night: $(format_currency 320 USD)"
    print_info "  Total: $(format_currency $((320 * nights)) USD)"
    print_info "  Cancellation: Free until $(date -d "$checkin -3 days" "+%b %d")"
    echo ""

    print_data "Hotel 2: City Center Inn ⭐⭐⭐⭐"
    print_info "  Location: Downtown $city"
    print_info "  Rating: 8.5/10 (876 reviews)"
    print_info "  Room: Standard Double"
    print_info "  Amenities: WiFi, Breakfast, Gym"
    print_info "  Price per night: $(format_currency 180 USD)"
    print_info "  Total: $(format_currency $((180 * nights)) USD)"
    print_info "  Cancellation: Free until $(date -d "$checkin -1 days" "+%b %d")"
    echo ""
}

# Create itinerary
create_itinerary() {
    local file=""
    local title=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --file) file="$2"; shift 2 ;;
            --title) title="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$file" ]; then
        print_error "Missing required parameter: --file"
        echo "Usage: wia-ind-015 create-itinerary --file trip.json [--title \"My Trip\"]"
        return 1
    fi

    print_section "Creating Travel Itinerary"

    if [ ! -f "$file" ]; then
        print_warning "File not found, creating sample itinerary"

        cat > "$file" <<EOF
{
  "title": "${title:-My Travel Itinerary}",
  "travelers": [
    {
      "firstName": "John",
      "lastName": "Doe",
      "type": "adult",
      "contact": {
        "email": "john@example.com",
        "phone": "+1-555-0123"
      }
    }
  ],
  "items": [
    {
      "type": "flight",
      "dateTime": "2025-06-15T08:00:00Z",
      "title": "Flight to London",
      "location": "JFK → LHR",
      "bookingReference": "AA100-ABC123"
    },
    {
      "type": "hotel",
      "dateTime": "2025-06-15T15:00:00Z",
      "endDateTime": "2025-06-22T11:00:00Z",
      "title": "The Royal Grand Hotel",
      "location": "Central London",
      "confirmationNumber": "HTL-456789"
    },
    {
      "type": "activity",
      "dateTime": "2025-06-16T10:00:00Z",
      "title": "Tower of London Tour",
      "location": "Tower of London"
    }
  ]
}
EOF
        print_success "Created sample itinerary at $file"
    fi

    if command -v jq &> /dev/null; then
        print_success "Itinerary: $(jq -r '.title' "$file")"
        print_info "Travelers: $(jq -r '.travelers | length' "$file")"
        print_info "Items: $(jq -r '.items | length' "$file")"
        echo ""

        print_data "Itinerary Items:"
        jq -r '.items[] | "  " + .dateTime + " - " + .type + ": " + .title' "$file" | while read line; do
            print_info "$line"
        done
    else
        print_success "Created itinerary at $file"
        print_warning "Install 'jq' to view formatted itinerary"
    fi
}

# Check travel documents
check_documents() {
    local passport=""
    local destination=""
    local date=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --passport) passport="$2"; shift 2 ;;
            --destination) destination="$2"; shift 2 ;;
            --date) date="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$passport" ] || [ -z "$destination" ]; then
        print_error "Missing required parameters: --passport, --destination"
        echo "Usage: wia-ind-015 check-documents --passport US --destination GB [--date 2025-06-15]"
        return 1
    fi

    date=${date:-$(date +%Y-%m-%d)}

    print_section "Checking Travel Requirements"
    print_info "Passport: $passport"
    print_info "Destination: $destination"
    print_info "Travel Date: $(format_date $date)"
    echo ""

    # Simulated document check
    print_data "Passport Validity:"
    print_success "Valid for travel to $destination"
    print_info "  Expiry requirement: 6 months beyond travel"
    print_info "  Your passport: Valid"
    echo ""

    print_data "Visa Requirements:"
    if [ "$passport" = "US" ] && [ "$destination" = "GB" ]; then
        print_success "No visa required for tourism (up to 6 months)"
        print_info "  Entry requirements: Valid passport, return ticket"
        print_info "  Maximum stay: 180 days"
    else
        print_warning "Visa may be required - check with embassy"
    fi
    echo ""

    print_data "Health Requirements:"
    print_info "  COVID-19: Check current requirements"
    print_info "  Vaccines: None required for this route"
    print_info "  Travel Insurance: Recommended"
    echo ""

    print_data "Customs & Immigration:"
    print_info "  Currency limits: Declare amounts over $10,000 USD"
    print_info "  Prohibited items: Check customs guidelines"
    print_info "  Agricultural products: Restrictions apply"
    echo ""
}

# Convert currency
convert_currency() {
    local from=""
    local to=""
    local amount=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --from) from="$2"; shift 2 ;;
            --to) to="$2"; shift 2 ;;
            --amount) amount="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$from" ] || [ -z "$to" ] || [ -z "$amount" ]; then
        print_error "Missing required parameters: --from, --to, --amount"
        echo "Usage: wia-ind-015 convert --from USD --to GBP --amount 1000"
        return 1
    fi

    print_section "Currency Conversion"

    # Simulated exchange rates (would use real API in production)
    local rate=1.0
    case "$from-$to" in
        USD-GBP) rate=0.79 ;;
        USD-EUR) rate=0.92 ;;
        USD-JPY) rate=148.50 ;;
        GBP-USD) rate=1.27 ;;
        EUR-USD) rate=1.09 ;;
        JPY-USD) rate=0.0067 ;;
        *) rate=1.0 ;;
    esac

    local converted=$(echo "$amount * $rate" | bc -l)

    print_info "From: $(format_currency $amount $from)"
    print_info "To: $(format_currency $converted $to)"
    print_info "Exchange Rate: 1 $from = $rate $to"
    print_info "Rate Date: $(date +"%Y-%m-%d %H:%M")"
    echo ""

    print_data "Breakdown:"
    print_info "  Base Amount: $(format_currency $amount $from)"
    print_info "  Exchange Rate: $rate"
    print_info "  Converted: $(format_currency $converted $to)"
    local fee=$(echo "$converted * 0.02" | bc -l)
    print_info "  Service Fee (2%): $(format_currency $fee $to)"
    local total=$(echo "$converted - $fee" | bc -l)
    print_success "You receive: $(format_currency $total $to)"
}

# Find accessible options
find_accessible() {
    local type=""
    local destination=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) type="$2"; shift 2 ;;
            --destination) destination="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$type" ] || [ -z "$destination" ]; then
        print_error "Missing required parameters: --type, --destination"
        echo "Usage: wia-ind-015 accessible --type wheelchair --destination Paris"
        return 1
    fi

    print_section "Accessible Travel Options in $destination"
    print_info "Accessibility Type: $type"
    echo ""

    print_data "Wheelchair Accessible Hotels:"
    print_success "Found 12 fully accessible properties"
    print_info "  • Roll-in showers"
    print_info "  • Wide doorways (32\" minimum)"
    print_info "  • Grab bars in bathroom"
    print_info "  • Accessible parking"
    print_info "  • Elevator access"
    echo ""

    print_data "Transportation:"
    print_success "Public transport accessibility:"
    print_info "  • Metro: 90% stations wheelchair accessible"
    print_info "  • Buses: All equipped with ramps"
    print_info "  • Taxis: Wheelchair-accessible vehicles available"
    print_info "  • Airport: Full accessibility services"
    echo ""

    print_data "Attractions:"
    print_info "  • Museums: All major museums fully accessible"
    print_info "  • Monuments: Ramps and elevators available"
    print_info "  • Restaurants: Wide selection of accessible venues"
    echo ""

    print_data "Assistance Services:"
    print_info "  • Airport wheelchair service"
    print_info "  • Priority boarding available"
    print_info "  • Accessible tour guides"
    print_info "  • Medical equipment rental"
    echo ""
}

# Show version
show_version() {
    print_header
    echo -e "${GRAY}WIA-IND-015 Travel Tech CLI${RESET}"
    echo -e "${GRAY}Version: $VERSION${RESET}"
    echo -e "${GRAY}API: $API_BASE_URL${RESET}"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Travel technology command-line interface"
    echo ""
    echo "Usage: wia-ind-015 <command> [options]"
    echo ""
    echo "Commands:"
    echo -e "  ${CYAN}search-flights${RESET}      Search for flight options"
    echo -e "  ${CYAN}book-hotel${RESET}          Search and book hotels"
    echo -e "  ${CYAN}create-itinerary${RESET}    Create travel itinerary"
    echo -e "  ${CYAN}check-documents${RESET}     Verify travel documents and requirements"
    echo -e "  ${CYAN}convert${RESET}             Convert currency"
    echo -e "  ${CYAN}accessible${RESET}          Find accessible travel options"
    echo -e "  ${CYAN}version${RESET}             Show version information"
    echo -e "  ${CYAN}help${RESET}                Show this help message"
    echo ""
    echo "Examples:"
    echo -e "  ${GRAY}# Search for flights${RESET}"
    echo -e "  ${GREEN}wia-ind-015 search-flights --from JFK --to LHR --date 2025-06-15${RESET}"
    echo ""
    echo -e "  ${GRAY}# Book hotel${RESET}"
    echo -e "  ${GREEN}wia-ind-015 book-hotel --city London --checkin 2025-06-15 --checkout 2025-06-22${RESET}"
    echo ""
    echo -e "  ${GRAY}# Create itinerary${RESET}"
    echo -e "  ${GREEN}wia-ind-015 create-itinerary --file trip.json --title \"Summer Vacation\"${RESET}"
    echo ""
    echo -e "  ${GRAY}# Check travel documents${RESET}"
    echo -e "  ${GREEN}wia-ind-015 check-documents --passport US --destination GB${RESET}"
    echo ""
    echo -e "  ${GRAY}# Convert currency${RESET}"
    echo -e "  ${GREEN}wia-ind-015 convert --from USD --to GBP --amount 1000${RESET}"
    echo ""
    echo -e "  ${GRAY}# Find accessible options${RESET}"
    echo -e "  ${GREEN}wia-ind-015 accessible --type wheelchair --destination Paris${RESET}"
    echo ""
    echo -e "${AMBER}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main command dispatcher
main() {
    case "${1:-help}" in
        search-flights)
            shift
            search_flights "$@"
            ;;
        book-hotel)
            shift
            book_hotel "$@"
            ;;
        create-itinerary)
            shift
            create_itinerary "$@"
            ;;
        check-documents)
            shift
            check_documents "$@"
            ;;
        convert)
            shift
            convert_currency "$@"
            ;;
        accessible)
            shift
            find_accessible "$@"
            ;;
        version)
            show_version
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            echo "Try 'wia-ind-015 help' for more information."
            exit 1
            ;;
    esac
}

# Run main
main "$@"

**弘益人間 (홍익인간) · Benefit All Humanity**
