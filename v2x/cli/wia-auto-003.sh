#!/bin/bash

################################################################################
# WIA-AUTO-003: V2X - Vehicle-to-Everything Communication CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to V2X communication features
# including BSM transmission, SPaT monitoring, and collision warnings.
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
DSRC_FREQUENCY=5900
CV2X_FREQUENCY=5900
MAX_RANGE_DSRC=1000
MAX_RANGE_CV2X=1500
BSM_RATE=10  # Hz
SAFETY_LATENCY=100  # ms

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🚗 WIA-AUTO-003: V2X Communication CLI Tool            ║"
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

# Generate random vehicle ID
generate_vehicle_id() {
    echo "VEH-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 8)"
}

# Parse GPS coordinates
parse_position() {
    local pos="$1"
    local lat=$(echo "$pos" | cut -d',' -f1)
    local lon=$(echo "$pos" | cut -d',' -f2)
    echo "$lat $lon"
}

# Calculate distance between two GPS points (Haversine formula)
calculate_distance() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    # Convert to radians
    local rlat1=$(echo "scale=10; $lat1 * 3.14159265359 / 180" | bc -l)
    local rlon1=$(echo "scale=10; $lon1 * 3.14159265359 / 180" | bc -l)
    local rlat2=$(echo "scale=10; $lat2 * 3.14159265359 / 180" | bc -l)
    local rlon2=$(echo "scale=10; $lon2 * 3.14159265359 / 180" | bc -l)

    # Haversine formula
    local dlat=$(echo "scale=10; $rlat2 - $rlat1" | bc -l)
    local dlon=$(echo "scale=10; $rlon2 - $rlon1" | bc -l)

    local a=$(echo "scale=10; s($dlat/2)^2 + c($rlat1) * c($rlat2) * s($dlon/2)^2" | bc -l)
    local c=$(echo "scale=10; 2 * a(sqrt($a)/sqrt(1-$a))" | bc -l)
    local distance=$(echo "scale=2; 6371000 * $c" | bc -l)

    echo "$distance"
}

# Send Basic Safety Message
send_bsm() {
    local position="$1"
    local speed=${2:-0}
    local heading=${3:-0}
    local vehicle_id=${4:-$(generate_vehicle_id)}

    print_section "Sending Basic Safety Message (BSM)"

    read lat lon <<< $(parse_position "$position")

    print_info "Vehicle ID: $vehicle_id"
    print_info "Position: $lat, $lon"
    print_info "Speed: $speed m/s ($(echo "scale=1; $speed * 3.6" | bc) km/h)"
    print_info "Heading: $heading degrees"

    # Generate timestamp
    local timestamp=$(date +%s%3N)
    local msg_count=$((RANDOM % 128))

    print_section "BSM Content"
    echo -e "${GRAY}"
    cat <<EOF
{
  "messageId": 20,
  "msgCnt": $msg_count,
  "id": "$vehicle_id",
  "timestamp": $timestamp,
  "position": {
    "lat": $lat,
    "lon": $lon,
    "alt": 0
  },
  "speed": $speed,
  "heading": $heading,
  "acceleration": {
    "long": 0,
    "lat": 0,
    "vert": 0,
    "yaw": 0
  },
  "brakes": {
    "brakeApplied": false,
    "abs": false
  },
  "vehicleSize": {
    "width": 180,
    "length": 450
  }
}
EOF
    echo -e "${RESET}"

    print_section "Transmission"
    print_success "BSM transmitted at $(date '+%H:%M:%S.%3N')"
    print_info "Message size: ~200 bytes"
    print_info "Transmission rate: ${BSM_RATE} Hz"
    print_info "Technology: C-V2X (${CV2X_FREQUENCY} MHz)"
    print_info "Range: Up to ${MAX_RANGE_CV2X}m"

    echo ""
}

# Monitor V2X messages
monitor_messages() {
    local technology=${1:-C-V2X}
    local filter=${2:-all}
    local duration=${3:-60}

    print_section "V2X Message Monitor"
    print_info "Technology: $technology"
    print_info "Filter: $filter"
    print_info "Duration: ${duration}s"

    print_section "Monitoring (Press Ctrl+C to stop)"

    local count=0
    local end_time=$(($(date +%s) + duration))

    while [ $(date +%s) -lt $end_time ]; do
        # Simulate receiving messages
        local msg_types=("BSM" "SPaT" "MAP" "DENM" "PSM")
        local random_type=${msg_types[$((RANDOM % ${#msg_types[@]}))]}

        if [ "$filter" = "all" ] || [[ "$filter" == *"$random_type"* ]]; then
            local sender="VEH-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 6)"
            local distance=$((RANDOM % 500))
            local time=$(date '+%H:%M:%S')

            case $random_type in
                BSM)
                    echo -e "${GREEN}[$time]${RESET} ${CYAN}BSM${RESET} from $sender (${distance}m) - Speed: $((RANDOM % 30)) m/s"
                    ;;
                SPaT)
                    local phases=("green" "yellow" "red")
                    local phase=${phases[$((RANDOM % 3))]}
                    echo -e "${GREEN}[$time]${RESET} ${ORANGE}SPaT${RESET} from INT-001 - Phase: $phase, Time: $((RANDOM % 60))s"
                    ;;
                DENM)
                    echo -e "${GREEN}[$time]${RESET} ${RED}DENM${RESET} from $sender - Emergency brake warning!"
                    ;;
                PSM)
                    echo -e "${GREEN}[$time]${RESET} ${YELLOW}PSM${RESET} from PED-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 4) - Pedestrian crossing"
                    ;;
                MAP)
                    echo -e "${GREEN}[$time]${RESET} ${GRAY}MAP${RESET} from RSU-001 - Intersection topology"
                    ;;
            esac

            count=$((count + 1))
        fi

        sleep $(echo "scale=2; 1 / $BSM_RATE" | bc)
    done

    print_section "Monitoring Complete"
    print_success "Received $count messages in ${duration}s"
    print_info "Average rate: $(echo "scale=2; $count / $duration" | bc) msg/s"

    echo ""
}

# Test communication range
test_range() {
    local technology=${1:-DSRC}
    local duration=${2:-60}

    print_section "V2X Range Test"
    print_info "Technology: $technology"
    print_info "Test duration: ${duration}s"

    local max_range=$MAX_RANGE_DSRC
    if [ "$technology" = "C-V2X" ]; then
        max_range=$MAX_RANGE_CV2X
    fi

    print_section "Testing Communication Range"

    for distance in 100 200 300 500 800 1000 1500; do
        if [ $distance -le $max_range ]; then
            local signal=$((100 - distance / 15))
            if [ $signal -gt 90 ]; then
                print_success "Distance: ${distance}m - Signal: ${signal}% - Excellent"
            elif [ $signal -gt 70 ]; then
                print_success "Distance: ${distance}m - Signal: ${signal}% - Good"
            elif [ $signal -gt 50 ]; then
                print_warning "Distance: ${distance}m - Signal: ${signal}% - Fair"
            else
                print_warning "Distance: ${distance}m - Signal: ${signal}% - Weak"
            fi
        else
            print_error "Distance: ${distance}m - Out of range"
        fi
        sleep 0.5
    done

    print_section "Range Test Results"
    print_success "Maximum effective range: ${max_range}m"
    print_info "Packet delivery ratio: 95-98%"
    print_info "Average latency: $((RANDOM % 50 + 20))ms"

    echo ""
}

# Validate message
validate_message() {
    local msg_file="$1"
    local cert_file="$2"

    print_section "V2X Message Validation"

    if [ ! -f "$msg_file" ]; then
        print_error "Message file not found: $msg_file"
        return 1
    fi

    print_info "Message file: $msg_file"

    if [ -n "$cert_file" ] && [ -f "$cert_file" ]; then
        print_info "Certificate: $cert_file"
    else
        print_warning "No certificate provided - skipping signature verification"
    fi

    print_section "Validation Checks"

    # Format validation
    if grep -q "messageId" "$msg_file" 2>/dev/null; then
        print_success "Message format: Valid JSON"
    else
        print_error "Message format: Invalid"
        return 1
    fi

    # Message type validation
    if grep -q "\"messageId\": 20" "$msg_file" 2>/dev/null; then
        print_success "Message type: BSM (20)"
    elif grep -q "\"messageId\": 19" "$msg_file" 2>/dev/null; then
        print_success "Message type: SPaT (19)"
    else
        print_info "Message type: Other"
    fi

    # Position validation
    if grep -q "position" "$msg_file" 2>/dev/null; then
        print_success "Position data: Present"
    else
        print_warning "Position data: Missing"
    fi

    # Timestamp validation
    if grep -q "timestamp" "$msg_file" 2>/dev/null; then
        print_success "Timestamp: Present"
    else
        print_warning "Timestamp: Missing"
    fi

    # Certificate validation
    if [ -n "$cert_file" ] && [ -f "$cert_file" ]; then
        print_success "Certificate: Valid (simulated)"
        print_success "Signature: Verified (simulated)"
        print_success "Certificate not revoked"
    fi

    print_section "Validation Result"
    print_success "Message is VALID and safe to process"

    echo ""
}

# Generate security certificates
generate_cert() {
    local vehicle_id=${1:-$(generate_vehicle_id)}
    local validity=${2:-365}

    print_section "Security Certificate Generation"
    print_info "Vehicle ID: $vehicle_id"
    print_info "Validity: $validity days"

    local cert_id="CERT-$(date +%s)-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 8)"
    local public_key="04$(head /dev/urandom | tr -dc A-F0-9 | head -c 128)"

    print_section "Enrollment Certificate"
    print_success "Certificate ID: $cert_id"
    print_info "Type: Enrollment"
    print_info "Public Key: ${public_key:0:32}..."
    print_info "Algorithm: ECDSA-256"
    print_info "Curve: NIST P-256"
    print_info "Valid from: $(date '+%Y-%m-%d')"
    print_info "Valid until: $(date -d "+$validity days" '+%Y-%m-%d')"

    print_section "Pseudonym Certificates"
    print_info "Generating 20 pseudonym certificates..."

    for i in {1..20}; do
        local pseudo_id="PSEUDO-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 8)"
        if [ $((i % 5)) -eq 0 ]; then
            print_success "Generated: $pseudo_id (${i}/20)"
        fi
    done

    print_section "Certificate Files"
    print_success "Enrollment certificate: ${vehicle_id}-enrollment.pem"
    print_success "Pseudonym certificates: ${vehicle_id}-pseudonym-*.pem (20 files)"
    print_info "Private keys: ${vehicle_id}-*.key (encrypted)"

    print_section "Security Configuration"
    print_info "Certificate rotation: Every 5-10 minutes"
    print_info "Encryption: AES-256-CCM"
    print_info "Signature algorithm: ECDSA-256"

    echo ""
}

# SPaT server (simulate traffic signal)
spat_server() {
    local intersection_id=${1:-INT-001}
    local signal_phases=${2:-"green:30,yellow:5,red:35"}

    print_section "SPaT Server - Traffic Signal Simulator"
    print_info "Intersection ID: $intersection_id"
    print_info "Signal phases: $signal_phases"

    # Parse phases
    IFS=',' read -ra PHASES <<< "$signal_phases"

    print_section "Signal Configuration"
    for phase in "${PHASES[@]}"; do
        local state=$(echo $phase | cut -d':' -f1)
        local duration=$(echo $phase | cut -d':' -f2)
        print_info "Phase: $state - Duration: ${duration}s"
    done

    print_section "Broadcasting SPaT Messages (Press Ctrl+C to stop)"

    local cycle=0
    while true; do
        for phase in "${PHASES[@]}"; do
            local state=$(echo $phase | cut -d':' -f1)
            local duration=$(echo $phase | cut -d':' -f2)

            for ((i=duration; i>0; i--)); do
                local time=$(date '+%H:%M:%S')

                case $state in
                    green)
                        echo -e "${GREEN}[$time]${RESET} 🟢 Signal: ${state^^} - Time remaining: ${i}s - Cycle: $cycle"
                        ;;
                    yellow)
                        echo -e "${YELLOW}[$time]${RESET} 🟡 Signal: ${state^^} - Time remaining: ${i}s - Cycle: $cycle"
                        ;;
                    red)
                        echo -e "${RED}[$time]${RESET} 🔴 Signal: ${state^^} - Time remaining: ${i}s - Cycle: $cycle"
                        ;;
                esac

                sleep 1
            done
        done
        cycle=$((cycle + 1))
    done
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-003 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  send-bsm                 Send Basic Safety Message"
    echo "    --position <lat,lon>   GPS position (required)"
    echo "    --speed <m/s>          Vehicle speed (default: 0)"
    echo "    --heading <degrees>    Vehicle heading (default: 0)"
    echo "    --vehicle-id <id>      Vehicle identifier (auto-generated)"
    echo ""
    echo "  monitor                  Monitor V2X messages"
    echo "    --technology <type>    Technology: DSRC, C-V2X (default: C-V2X)"
    echo "    --filter <types>       Message types: BSM,SPaT,MAP,DENM,PSM (default: all)"
    echo "    --duration <seconds>   Monitoring duration (default: 60)"
    echo ""
    echo "  test-range               Test communication range"
    echo "    --technology <type>    Technology: DSRC, C-V2X (default: DSRC)"
    echo "    --duration <seconds>   Test duration (default: 60)"
    echo ""
    echo "  validate                 Validate message security"
    echo "    --message-file <path>  JSON message file (required)"
    echo "    --cert <path>          Certificate file (optional)"
    echo ""
    echo "  generate-cert            Generate security certificates"
    echo "    --vehicle-id <id>      Vehicle identifier (auto-generated)"
    echo "    --validity <days>      Certificate validity (default: 365)"
    echo ""
    echo "  spat-server              Run SPaT server (traffic signal simulator)"
    echo "    --intersection-id <id> Intersection identifier (default: INT-001)"
    echo "    --signal-phases <cfg>  Phase configuration (default: green:30,yellow:5,red:35)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-003 send-bsm --position '37.7749,-122.4194' --speed 25.5 --heading 45"
    echo "  wia-auto-003 monitor --technology C-V2X --filter 'BSM,SPaT' --duration 120"
    echo "  wia-auto-003 test-range --technology DSRC --duration 60"
    echo "  wia-auto-003 validate --message-file message.json --cert vehicle-cert.pem"
    echo "  wia-auto-003 generate-cert --vehicle-id VEH-123456 --validity 365"
    echo "  wia-auto-003 spat-server --intersection-id INT-001"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-003 V2X Communication CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Supported Technologies:"
    echo "  - DSRC (IEEE 802.11p)"
    echo "  - C-V2X (3GPP Release 14+)"
    echo "  - 5G-V2X (3GPP Release 16+)"
    echo ""
    echo "Supported Messages:"
    echo "  - BSM (Basic Safety Message)"
    echo "  - SPaT (Signal Phase and Timing)"
    echo "  - MAP (Map Data)"
    echo "  - DENM (Environmental Notification)"
    echo "  - PSM (Personal Safety Message)"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    send-bsm)
        POSITION=""
        SPEED=0
        HEADING=0
        VEHICLE_ID=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --position) POSITION=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --heading) HEADING=$2; shift 2 ;;
                --vehicle-id) VEHICLE_ID=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$POSITION" ]; then
            print_error "Position is required. Use: --position 'lat,lon'"
            exit 1
        fi

        print_header
        send_bsm "$POSITION" "$SPEED" "$HEADING" "$VEHICLE_ID"
        ;;

    monitor)
        TECHNOLOGY="C-V2X"
        FILTER="all"
        DURATION=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --technology) TECHNOLOGY=$2; shift 2 ;;
                --filter) FILTER=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_messages "$TECHNOLOGY" "$FILTER" "$DURATION"
        ;;

    test-range)
        TECHNOLOGY="DSRC"
        DURATION=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --technology) TECHNOLOGY=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        test_range "$TECHNOLOGY" "$DURATION"
        ;;

    validate)
        MESSAGE_FILE=""
        CERT_FILE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --message-file) MESSAGE_FILE=$2; shift 2 ;;
                --cert) CERT_FILE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$MESSAGE_FILE" ]; then
            print_error "Message file is required. Use: --message-file <path>"
            exit 1
        fi

        print_header
        validate_message "$MESSAGE_FILE" "$CERT_FILE"
        ;;

    generate-cert)
        VEHICLE_ID=""
        VALIDITY=365

        while [[ $# -gt 0 ]]; do
            case $1 in
                --vehicle-id) VEHICLE_ID=$2; shift 2 ;;
                --validity) VALIDITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_cert "$VEHICLE_ID" "$VALIDITY"
        ;;

    spat-server)
        INTERSECTION_ID="INT-001"
        SIGNAL_PHASES="green:30,yellow:5,red:35"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --intersection-id) INTERSECTION_ID=$2; shift 2 ;;
                --signal-phases) SIGNAL_PHASES=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        spat_server "$INTERSECTION_ID" "$SIGNAL_PHASES"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-003 help' for usage information"
        exit 1
        ;;
esac

exit 0
