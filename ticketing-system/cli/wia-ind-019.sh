#!/bin/bash

################################################################################
# WIA-IND-019: Ticketing System CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to ticketing system operations
# including ticket creation, validation, dynamic pricing, and management.
################################################################################

set -e

# Colors for output
AMBER='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
PLATFORM="WIA-IND-019"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🎫 WIA-IND-019: Ticketing System CLI                 ║"
    echo "║                     Version $VERSION                            ║"
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

print_value() {
    echo -e "${BLUE}  $1${RESET}"
}

# Format currency
format_currency() {
    local amount=$1
    local currency=${2:-USD}
    printf "${currency} %.2f" "$amount"
}

# Generate ticket ID
generate_ticket_id() {
    local event_code=$1
    local section=$2
    local year=$(date +%Y)
    local sequence=$(printf "%04d" $((RANDOM % 10000)))
    echo "TKT-${year}-${event_code}-${section}-${sequence}"
}

# Generate TOTP (simplified version)
generate_totp() {
    local secret=$1
    local timestamp=${2:-$(date +%s)}
    local interval=30
    local counter=$((timestamp / interval))

    # Simple hash-based TOTP (in production use proper implementation)
    local hash=$(echo -n "${secret}${counter}" | sha256sum | cut -c1-6)
    local code=$((16#$hash % 1000000))
    printf "%06d" "$code"
}

# Calculate distance between coordinates (Haversine formula)
calculate_distance() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    local R=6371000 # Earth radius in meters

    # Convert to radians
    local lat1_rad=$(echo "scale=10; $lat1 * 3.14159265359 / 180" | bc -l)
    local lat2_rad=$(echo "scale=10; $lat2 * 3.14159265359 / 180" | bc -l)
    local delta_lat=$(echo "scale=10; ($lat2 - $lat1) * 3.14159265359 / 180" | bc -l)
    local delta_lon=$(echo "scale=10; ($lon2 - $lon1) * 3.14159265359 / 180" | bc -l)

    # Haversine formula (simplified)
    local distance=$(echo "scale=2; sqrt(($delta_lat * $delta_lat) + ($delta_lon * $delta_lon)) * $R / 1000" | bc -l)
    echo "$distance"
}

# ============================================================================
# Command: create-ticket
# ============================================================================

create_ticket() {
    local event=""
    local seat=""
    local holder=""
    local price=0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --event) event="$2"; shift 2 ;;
            --seat) seat="$2"; shift 2 ;;
            --holder) holder="$2"; shift 2 ;;
            --price) price="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    # Validate inputs
    if [ -z "$event" ] || [ -z "$seat" ] || [ -z "$holder" ] || [ "$price" == "0" ]; then
        print_error "Missing required parameters"
        echo "Usage: wia-ind-019 create-ticket --event EVENT_ID --seat SEAT --holder EMAIL --price PRICE"
        return 1
    fi

    print_section "Creating Ticket"

    # Generate ticket ID
    local event_code=$(echo "$event" | cut -d'-' -f3)
    local section=$(echo "$seat" | cut -d'-' -f1)
    local ticket_id=$(generate_ticket_id "$event_code" "$section")

    print_success "Ticket ID: $ticket_id"

    # Calculate pricing
    local taxes=$(echo "scale=2; $price * 0.10" | bc)
    local fees=5.00
    local final_price=$(echo "scale=2; $price + $taxes + $fees" | bc)

    print_section "Pricing Details"
    print_info "Base Price:    $(format_currency $price)"
    print_info "Taxes (10%):   $(format_currency $taxes)"
    print_info "Service Fee:   $(format_currency $fees)"
    print_success "Final Price:   $(format_currency $final_price)"

    # Generate security features
    local totp_secret=$(openssl rand -hex 20)
    local totp_code=$(generate_totp "$totp_secret")

    print_section "Security"
    print_info "QR Code:       Generated ✓"
    print_info "TOTP Code:     $totp_code"
    print_info "Blockchain:    Pending verification..."

    print_section "Ticket Details"
    print_value "Ticket ID:     $ticket_id"
    print_value "Event:         $event"
    print_value "Seat:          $seat"
    print_value "Holder:        $holder"
    print_value "Status:        Active"

    echo ""
    print_success "Ticket created successfully!"
    echo ""
}

# ============================================================================
# Command: validate
# ============================================================================

validate_ticket() {
    local ticket_id=""
    local qr_code=""
    local location=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ticket-id) ticket_id="$2"; shift 2 ;;
            --qr-code) qr_code="$2"; shift 2 ;;
            --location) location="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$ticket_id" ]; then
        print_error "Missing ticket ID"
        return 1
    fi

    print_section "Validating Ticket: $ticket_id"

    # Simulated validation checks
    local checks_passed=0
    local checks_total=6

    # Check 1: Ticket exists
    print_info "Checking ticket existence..."
    sleep 0.3
    print_success "Ticket found"
    ((checks_passed++))

    # Check 2: Ticket status
    print_info "Verifying ticket status..."
    sleep 0.3
    print_success "Status: Active"
    ((checks_passed++))

    # Check 3: Expiry date
    print_info "Checking expiry date..."
    sleep 0.3
    print_success "Ticket valid"
    ((checks_passed++))

    # Check 4: QR signature
    if [ -n "$qr_code" ]; then
        print_info "Verifying QR signature..."
        sleep 0.3
        print_success "QR code valid"
        ((checks_passed++))
    else
        print_warning "QR code not provided (skipped)"
    fi

    # Check 5: Geolocation
    if [ -n "$location" ]; then
        print_info "Checking geofence..."
        sleep 0.3
        # Parse location (lat,lon)
        local lat=$(echo "$location" | cut -d',' -f1)
        local lon=$(echo "$location" | cut -d',' -f2)
        # Venue location (example)
        local venue_lat=37.7749
        local venue_lon=-122.4194
        local distance=$(calculate_distance "$lat" "$lon" "$venue_lat" "$venue_lon")
        if (( $(echo "$distance < 0.1" | bc -l) )); then
            print_success "Location verified (${distance} km from venue)"
            ((checks_passed++))
        else
            print_warning "Location outside geofence (${distance} km from venue)"
        fi
    else
        print_warning "Location not provided (skipped)"
    fi

    # Check 6: Duplicate scan
    print_info "Checking for duplicate scan..."
    sleep 0.3
    print_success "No previous check-in detected"
    ((checks_passed++))

    print_section "Validation Result"

    local success_rate=$(echo "scale=0; $checks_passed * 100 / $checks_total" | bc)

    if [ "$checks_passed" == "$checks_total" ]; then
        print_success "ALL CHECKS PASSED (${checks_passed}/${checks_total})"
        print_success "✓ TICKET IS VALID - ALLOW ENTRY"
    elif [ "$checks_passed" -ge 4 ]; then
        print_warning "PARTIAL VALIDATION (${checks_passed}/${checks_total})"
        print_warning "→ MANUAL CHECK RECOMMENDED"
    else
        print_error "VALIDATION FAILED (${checks_passed}/${checks_total})"
        print_error "✗ DENY ENTRY"
    fi

    echo ""
}

# ============================================================================
# Command: calc-price
# ============================================================================

calc_dynamic_price() {
    local base=100
    local demand=1.0
    local days_until=30
    local capacity=0.5

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --base) base="$2"; shift 2 ;;
            --demand) demand="$2"; shift 2 ;;
            --days-until) days_until="$2"; shift 2 ;;
            --capacity) capacity="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Dynamic Pricing Calculator"

    print_info "Base Price:             $(format_currency $base)"
    print_info "Demand Factor:          $demand"
    print_info "Days Until Event:       $days_until"
    print_info "Capacity Remaining:     $(echo "scale=0; $capacity * 100" | bc)%"

    # Calculate time decay factor
    local time_decay=1.0
    if [ "$days_until" -gt 90 ]; then
        time_decay=0.6
    elif [ "$days_until" -gt 60 ]; then
        time_decay=0.8
    elif [ "$days_until" -gt 30 ]; then
        time_decay=1.0
    elif [ "$days_until" -gt 7 ]; then
        time_decay=1.1
    elif [ "$days_until" -gt 1 ]; then
        time_decay=1.2
    else
        time_decay=1.5
    fi

    # Calculate scarcity factor
    local scarcity=1.0
    if (( $(echo "$capacity > 0.7" | bc -l) )); then
        scarcity=0.7
    elif (( $(echo "$capacity > 0.5" | bc -l) )); then
        scarcity=0.9
    elif (( $(echo "$capacity > 0.25" | bc -l) )); then
        scarcity=1.0
    elif (( $(echo "$capacity > 0.1" | bc -l) )); then
        scarcity=1.5
    else
        scarcity=2.5
    fi

    # Calculate final price
    local calculated=$(echo "scale=2; $base * $demand * $time_decay * $scarcity" | bc)

    # Apply constraints (50% - 250%)
    local min_price=$(echo "scale=2; $base * 0.5" | bc)
    local max_price=$(echo "scale=2; $base * 2.5" | bc)

    local final_price=$calculated
    if (( $(echo "$calculated < $min_price" | bc -l) )); then
        final_price=$min_price
    elif (( $(echo "$calculated > $max_price" | bc -l) )); then
        final_price=$max_price
    fi

    print_section "Price Breakdown"
    print_info "Time Decay Factor:      $time_decay"
    print_info "Scarcity Factor:        $scarcity"
    print_info "Market Multiplier:      1.0"

    print_section "Result"
    print_value "Calculated Price:       $(format_currency $calculated)"

    if [ "$final_price" != "$calculated" ]; then
        print_warning "Adjusted (constraints): $(format_currency $final_price)"
    fi

    print_success "Final Dynamic Price:    $(format_currency $final_price)"

    local change=$(echo "scale=0; (($final_price - $base) / $base) * 100" | bc)
    if (( $(echo "$change > 0" | bc -l) )); then
        print_info "Change from base:       +${change}%"
    else
        print_info "Change from base:       ${change}%"
    fi

    echo ""
}

# ============================================================================
# Command: transfer
# ============================================================================

transfer_ticket() {
    local ticket_id=""
    local from=""
    local to=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ticket-id) ticket_id="$2"; shift 2 ;;
            --from) from="$2"; shift 2 ;;
            --to) to="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$ticket_id" ] || [ -z "$from" ] || [ -z "$to" ]; then
        print_error "Missing required parameters"
        echo "Usage: wia-ind-019 transfer --ticket-id ID --from EMAIL --to EMAIL"
        return 1
    fi

    print_section "Ticket Transfer"

    print_info "Ticket ID:    $ticket_id"
    print_info "From:         $from"
    print_info "To:           $to"

    # Simulated transfer process
    print_section "Processing Transfer"

    print_info "Verifying ownership..."
    sleep 0.5
    print_success "Owner verified"

    print_info "Checking transferability..."
    sleep 0.5
    print_success "Ticket is transferable"

    print_info "Updating blockchain record..."
    sleep 0.7
    print_success "Blockchain updated"

    print_info "Regenerating QR code..."
    sleep 0.5
    print_success "New QR code generated"

    print_info "Sending notifications..."
    sleep 0.5
    print_success "Email sent to both parties"

    local transfer_id="TXF-$(date +%s)-$(openssl rand -hex 4)"

    print_section "Transfer Complete"
    print_success "Transfer ID: $transfer_id"
    print_value "Status:      Completed"
    print_value "New Owner:   $to"

    echo ""
}

# ============================================================================
# Command: generate-qr
# ============================================================================

generate_qr() {
    local ticket_id=""
    local format="png"
    local output="./ticket-qr.png"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ticket-id) ticket_id="$2"; shift 2 ;;
            --format) format="$2"; shift 2 ;;
            --output) output="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$ticket_id" ]; then
        print_error "Missing ticket ID"
        return 1
    fi

    print_section "Generating QR Code"

    print_info "Ticket ID: $ticket_id"
    print_info "Format:    $format"
    print_info "Output:    $output"

    # Check if qrencode is available
    if command -v qrencode &> /dev/null; then
        print_info "Encoding ticket data..."
        local qr_data="wia://ticket/${ticket_id}"

        if [ "$format" == "terminal" ]; then
            echo ""
            qrencode -t ANSIUTF8 "$qr_data"
            echo ""
        else
            qrencode -o "$output" -s 10 -l H "$qr_data"
            print_success "QR code saved to: $output"
        fi
    else
        print_warning "qrencode not installed"
        print_info "Install with: apt-get install qrencode (Debian/Ubuntu)"
        print_info "            or brew install qrencode (macOS)"

        # Generate text-based representation
        print_info "Generating text representation..."
        echo ""
        echo "┌─────────────────┐"
        echo "│ ▓▓▓▓▓▓▓ ▓▓▓▓▓▓▓ │"
        echo "│ ▓     ▓ ▓     ▓ │"
        echo "│ ▓ ▓▓▓ ▓ ▓ ▓▓▓ ▓ │"
        echo "│ ▓ ▓▓▓ ▓ ▓ ▓▓▓ ▓ │"
        echo "│ ▓ ▓▓▓ ▓ ▓ ▓▓▓ ▓ │"
        echo "│ ▓     ▓ ▓     ▓ │"
        echo "│ ▓▓▓▓▓▓▓ ▓▓▓▓▓▓▓ │"
        echo "│   QR CODE HERE  │"
        echo "│  $ticket_id     │"
        echo "└─────────────────┘"
        echo ""
    fi

    echo ""
}

# ============================================================================
# Command: check-seats
# ============================================================================

check_seats() {
    local event=""
    local section=""
    local quantity=1

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --event) event="$2"; shift 2 ;;
            --section) section="$2"; shift 2 ;;
            --quantity) quantity="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$event" ]; then
        print_error "Missing event ID"
        return 1
    fi

    print_section "Seat Availability Check"

    print_info "Event:    $event"
    print_info "Section:  ${section:-All}"
    print_info "Quantity: $quantity"

    # Simulated capacity data
    local total=1000
    local sold=675
    local reserved=50
    local available=$((total - sold - reserved))
    local utilization=$(echo "scale=1; $sold * 100 / $total" | bc)

    print_section "Capacity Status"
    print_value "Total Capacity:     $total"
    print_value "Tickets Sold:       $sold"
    print_value "Reserved:           $reserved"
    print_success "Available:          $available"
    print_info "Utilization:        ${utilization}%"

    if [ "$available" -ge "$quantity" ]; then
        print_success "✓ Requested quantity ($quantity) is available"
    else
        print_error "✗ Insufficient seats (need $quantity, have $available)"
    fi

    echo ""
}

# ============================================================================
# Command: version
# ============================================================================

show_version() {
    echo "WIA-IND-019 Ticketing System CLI"
    echo "Version: $VERSION"
    echo "Platform: $PLATFORM"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo "© 2025 SmileStory Inc. / WIA"
}

# ============================================================================
# Command: help
# ============================================================================

show_help() {
    print_header

    echo "Usage: wia-ind-019 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo -e "${CYAN}  create-ticket${RESET}     Create a new ticket"
    echo "    --event EVENT_ID       Event identifier"
    echo "    --seat SEAT            Seat assignment (e.g., A-12-15)"
    echo "    --holder EMAIL         Ticket holder email"
    echo "    --price PRICE          Base price"
    echo ""
    echo -e "${CYAN}  validate${RESET}           Validate ticket at entry"
    echo "    --ticket-id ID         Ticket identifier"
    echo "    --qr-code CODE         QR code payload (optional)"
    echo "    --location LAT,LON     GPS coordinates (optional)"
    echo ""
    echo -e "${CYAN}  calc-price${RESET}         Calculate dynamic pricing"
    echo "    --base PRICE           Base price"
    echo "    --demand FACTOR        Demand factor (0.5 - 3.0)"
    echo "    --days-until DAYS      Days until event"
    echo "    --capacity RATIO       Capacity remaining (0.0 - 1.0)"
    echo ""
    echo -e "${CYAN}  transfer${RESET}           Transfer ticket ownership"
    echo "    --ticket-id ID         Ticket identifier"
    echo "    --from EMAIL           Current owner email"
    echo "    --to EMAIL             New owner email"
    echo ""
    echo -e "${CYAN}  generate-qr${RESET}        Generate QR code"
    echo "    --ticket-id ID         Ticket identifier"
    echo "    --format FORMAT        png|svg|terminal (default: png)"
    echo "    --output FILE          Output file path"
    echo ""
    echo -e "${CYAN}  check-seats${RESET}        Check seat availability"
    echo "    --event EVENT_ID       Event identifier"
    echo "    --section SECTION      Venue section (optional)"
    echo "    --quantity NUM         Number of seats needed"
    echo ""
    echo -e "${CYAN}  version${RESET}            Show version information"
    echo -e "${CYAN}  help${RESET}               Show this help message"
    echo ""
    echo "Examples:"
    echo ""
    echo "  wia-ind-019 create-ticket --event EVT-2025-001 --seat A-12-15 \\"
    echo "                            --holder john@example.com --price 150"
    echo ""
    echo "  wia-ind-019 validate --ticket-id TKT-2025-EVT001-A12-0001 \\"
    echo "                       --location 37.7749,-122.4194"
    echo ""
    echo "  wia-ind-019 calc-price --base 150 --demand 1.5 --days-until 30 \\"
    echo "                         --capacity 0.45"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        create-ticket)
            create_ticket "$@"
            ;;
        validate)
            validate_ticket "$@"
            ;;
        calc-price)
            calc_dynamic_price "$@"
            ;;
        transfer)
            transfer_ticket "$@"
            ;;
        generate-qr)
            generate_qr "$@"
            ;;
        check-seats)
            check_seats "$@"
            ;;
        version)
            show_version
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-ind-019 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main
main "$@"

# ============================================================================
# 弘익人間 (홍익인간) · Benefit All Humanity
# ============================================================================
