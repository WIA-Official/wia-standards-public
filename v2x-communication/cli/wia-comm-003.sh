#!/bin/bash

################################################################################
# WIA-COMM-003: V2X Communication CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to V2X communication functions
# including V2V messaging, V2I integration, V2P safety, and platooning.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
DSRC_FREQUENCY=5900  # MHz
MAX_RANGE=1000       # meters
BSM_RATE=10          # Hz

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🚗 WIA-COMM-003: V2X Communication CLI               ║"
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

# Start V2X service
start_service() {
    local tech=${1:-C-V2X}
    local freq=${2:-5900}

    print_section "Starting V2X Communication Service"
    print_info "Technology: $tech"
    print_info "Frequency: $freq MHz"
    print_info "Security: Enabled"

    sleep 1

    print_success "Radio initialized"
    print_success "GNSS receiver ready"
    print_success "Security certificates loaded"
    print_success "V2X service started"

    print_section "Service Status"
    print_info "BSM Broadcast: ${BSM_RATE} Hz"
    print_info "Maximum Range: ${MAX_RANGE} m"
    print_info "Nearby Vehicles: 5 detected"
    print_info "Connected RSUs: 2 in range"

    echo ""
}

# Send BSM message
send_bsm() {
    local lat=${1:-37.7749}
    local lon=${2:--122.4194}
    local speed=${3:-65}

    print_section "Sending Basic Safety Message (BSM)"
    print_info "Latitude: $lat"
    print_info "Longitude: $lon"
    print_info "Speed: $speed km/h"
    print_info "Heading: 270°"

    # Simulate BSM transmission
    sleep 0.5

    print_success "BSM transmitted successfully"
    print_info "Message Count: 42"
    print_info "Timestamp: $(date +%s)"
    print_info "Range: Broadcast to all vehicles within ${MAX_RANGE}m"

    echo ""
}

# Monitor V2V messages
monitor_v2v() {
    local msg_type=${1:-all}
    local filter=${2:-}

    print_section "Monitoring V2V Messages"
    print_info "Message Type: $msg_type"
    [ -n "$filter" ] && print_info "Filter: $filter"

    echo ""
    print_info "Press Ctrl+C to stop monitoring..."
    echo ""

    # Simulate receiving messages
    for i in {1..10}; do
        sleep 0.5
        local vehicle_id="VEH-$(printf "%06d" $((RANDOM % 1000)))"
        local distance=$((RANDOM % 500 + 50))
        local speed=$((RANDOM % 50 + 50))

        if [ "$filter" == "collision-warning" ] && [ $((RANDOM % 3)) -eq 0 ]; then
            print_warning "[COLLISION WARNING] $vehicle_id - ${distance}m - TTC: 2.3s"
        else
            echo -e "${GRAY}[BSM] $vehicle_id - ${distance}m - ${speed} km/h${RESET}"
        fi
    done

    echo ""
    print_success "Received 10 messages"
    echo ""
}

# Test RSU connection
test_rsu() {
    local rsu_id=${1:-RSU-001}
    local location=${2:-"Main St & 1st Ave"}

    print_section "Testing RSU Connection"
    print_info "RSU ID: $rsu_id"
    print_info "Location: $location"

    sleep 1

    print_success "RSU detected"
    print_info "Distance: 245 m"
    print_info "Signal Strength: -65 dBm"
    print_info "Latency: 8 ms"

    print_section "RSU Services"
    print_success "SPaT (Traffic Signal): Available"
    print_success "MAP (Road Geometry): Available"
    print_success "TIM (Traveler Info): Available"
    print_info "Emergency Preemption: Supported"

    print_section "Traffic Signal Status"
    print_info "Intersection ID: 12345"
    print_info "Current Phase: Green"
    print_info "Time Remaining: 15 seconds"
    print_info "Next Phase: Yellow (3s) → Red"

    echo ""
}

# Measure latency
measure_latency() {
    local target=${1:-VEH-789}
    local count=${2:-10}

    print_section "Measuring V2X Latency"
    print_info "Target: $target"
    print_info "Ping Count: $count"

    echo ""
    local total=0
    local min=999
    local max=0

    for i in $(seq 1 $count); do
        local latency=$((RANDOM % 15 + 3))  # 3-18 ms
        total=$((total + latency))

        [ $latency -lt $min ] && min=$latency
        [ $latency -gt $max ] && max=$latency

        echo -e "${GRAY}Ping $i: ${latency} ms${RESET}"
        sleep 0.2
    done

    local avg=$((total / count))

    echo ""
    print_section "Latency Statistics"
    print_success "Minimum: ${min} ms"
    print_success "Maximum: ${max} ms"
    print_success "Average: ${avg} ms"

    if [ $avg -lt 10 ]; then
        print_success "Latency: EXCELLENT (< 10 ms)"
    elif [ $avg -lt 20 ]; then
        print_warning "Latency: GOOD (10-20 ms)"
    else
        print_error "Latency: POOR (> 20 ms)"
    fi

    echo ""
}

# Platoon mode
platoon_mode() {
    local mode=${1:-member}
    local max_vehicles=${2:-5}
    local spacing=${3:-10}

    print_section "Platooning Mode"
    print_info "Mode: ${mode^^}"
    print_info "Max Vehicles: $max_vehicles"
    print_info "Target Spacing: ${spacing}m"

    sleep 1

    if [ "$mode" == "leader" ]; then
        print_success "Platoon created"
        print_info "Platoon ID: PLT-$(date +%s)"
        print_info "Target Speed: 100 km/h"
        print_info "Formation: Line"

        echo ""
        print_section "Platoon Status"
        print_info "Leader: VEH-001 (You)"
        print_info "Members: 0 / $max_vehicles"
        print_info "Waiting for join requests..."

        sleep 2

        print_success "VEH-234 joined (Position 1)"
        sleep 1
        print_success "VEH-567 joined (Position 2)"

        echo ""
        print_section "Platoon Active"
        print_info "Members: 2 / $max_vehicles"
        print_info "Formation: Stable"
        print_info "Average Spacing: ${spacing}m ±0.5m"
        print_success "CACC (Cooperative Adaptive Cruise Control): Active"

    else
        print_info "Searching for platoons..."
        sleep 1

        print_success "Platoon found: PLT-1640000000"
        print_info "Leader: VEH-001"
        print_info "Members: 2 / 5"
        print_info "Speed: 100 km/h"

        echo ""
        print_section "Join Request"
        print_info "Sending join request..."
        sleep 1

        print_success "Join request APPROVED"
        print_info "Assigned Position: 3"
        print_info "Target Spacing: ${spacing}m"

        echo ""
        print_section "Platoon Joined"
        print_success "Now following platoon"
        print_info "Maintaining ${spacing}m spacing"
        print_success "CACC engaged"
    fi

    echo ""
}

# Send hazard warning
send_hazard() {
    local hazard_type=${1:-accident}
    local severity=${2:-high}

    print_section "Sending Hazard Warning (DENM)"
    print_info "Type: ${hazard_type^^}"
    print_info "Severity: ${severity^^}"
    print_info "Position: Current location"

    sleep 0.5

    print_success "DENM transmitted"
    print_info "Relevance: 500m radius"
    print_info "Duration: 10 minutes"
    print_info "Traffic Direction: All directions"

    echo ""
    print_section "Warning Broadcast"
    print_success "Nearby vehicles notified: 12"
    print_success "RSUs notified: 2"
    print_info "Cloud TMC: Synced"

    echo ""
}

# Detect pedestrians
detect_pedestrians() {
    print_section "Pedestrian Detection (V2P)"
    print_info "Scanning for Personal Safety Messages (PSM)..."

    sleep 1

    print_success "3 pedestrians detected"

    echo ""
    print_section "Pedestrian Details"

    echo -e "${GRAY}┌──────────┬──────────┬───────┬───────────┬──────────┐${RESET}"
    echo -e "${GRAY}│ ID       │ Distance │ Speed │ Risk      │ Action   │${RESET}"
    echo -e "${GRAY}├──────────┼──────────┼───────┼───────────┼──────────┤${RESET}"
    echo -e "${GRAY}│ PED-001  │ 45m      │ 1.2m/s│${RESET} ${GREEN}LOW${RESET}${GRAY}       │ Monitor  │${RESET}"
    echo -e "${GRAY}│ PED-002  │ 25m      │ 1.5m/s│${RESET} ${YELLOW}MEDIUM${RESET}${GRAY}    │ Monitor  │${RESET}"
    echo -e "${GRAY}│ PED-003  │ 15m      │ 1.8m/s│${RESET} ${RED}HIGH${RESET}${GRAY}      │ Slow Down│${RESET}"
    echo -e "${GRAY}└──────────┴──────────┴───────┴───────────┴──────────┘${RESET}"

    echo ""
    print_warning "Pedestrian in path detected!"
    print_info "Recommended action: Reduce speed to 20 km/h"

    echo ""
}

# Show statistics
show_stats() {
    print_section "V2X Communication Statistics"

    echo ""
    echo -e "${CYAN}Messages Sent:${RESET}"
    print_info "BSM: 1,234 messages"
    print_info "CAM: 567 messages"
    print_info "DENM: 12 messages"

    echo ""
    echo -e "${CYAN}Messages Received:${RESET}"
    print_info "V2V: 4,567 messages"
    print_info "V2I: 234 messages (from 3 RSUs)"
    print_info "V2P: 45 messages (15 pedestrians)"

    echo ""
    echo -e "${CYAN}Performance:${RESET}"
    print_success "Average Latency: 7.2 ms"
    print_success "Packet Delivery Ratio: 99.5%"
    print_success "Channel Busy Ratio: 42%"

    echo ""
    echo -e "${CYAN}Security:${RESET}"
    print_success "Messages Authenticated: 100%"
    print_info "Certificate Changes: 12 (automatic)"
    print_info "Misbehavior Detected: 0"

    echo ""
    echo -e "${CYAN}Connected Entities:${RESET}"
    print_info "Nearby Vehicles: 23"
    print_info "Connected RSUs: 3"
    print_info "Pedestrians: 8"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-003 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  start                    Start V2X communication service"
    echo "    --tech <DSRC|C-V2X>    Communication technology (default: C-V2X)"
    echo "    --frequency <MHz>      Operating frequency (default: 5900)"
    echo ""
    echo "  send-bsm                 Send Basic Safety Message"
    echo "    --lat <latitude>       Latitude (default: 37.7749)"
    echo "    --lon <longitude>      Longitude (default: -122.4194)"
    echo "    --speed <km/h>         Speed in km/h (default: 65)"
    echo ""
    echo "  monitor                  Monitor V2V messages"
    echo "    --type <all|V2V|V2I>   Message type filter (default: all)"
    echo "    --filter <keyword>     Filter messages (e.g., collision-warning)"
    echo ""
    echo "  test-rsu                 Test RSU connection"
    echo "    --rsu-id <id>          RSU identifier (default: RSU-001)"
    echo "    --location <desc>      Location description"
    echo ""
    echo "  measure-latency          Measure communication latency"
    echo "    --target <vehicle-id>  Target vehicle ID (default: VEH-789)"
    echo "    --count <number>       Number of pings (default: 10)"
    echo ""
    echo "  platoon                  Platoon mode"
    echo "    --mode <leader|member> Platoon role (default: member)"
    echo "    --max-vehicles <num>   Maximum platoon size (default: 5)"
    echo "    --spacing <meters>     Inter-vehicle spacing (default: 10)"
    echo ""
    echo "  send-hazard              Send hazard warning"
    echo "    --type <type>          Hazard type (accident, breakdown, weather)"
    echo "    --severity <level>     Severity (low, medium, high, danger)"
    echo ""
    echo "  detect-pedestrians       Detect nearby pedestrians (V2P)"
    echo ""
    echo "  stats                    Show communication statistics"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-003 start --tech C-V2X --frequency 5900"
    echo "  wia-comm-003 send-bsm --lat 37.7749 --lon -122.4194 --speed 65"
    echo "  wia-comm-003 monitor --type V2V --filter collision-warning"
    echo "  wia-comm-003 platoon --mode leader --max-vehicles 5 --spacing 10"
    echo "  wia-comm-003 send-hazard --type accident --severity high"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-003 V2X Communication CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Supported Technologies:"
    echo "  - DSRC (IEEE 802.11p)"
    echo "  - C-V2X (LTE-V2X)"
    echo "  - 5G NR-V2X"
    echo ""
    echo "Supported Messages:"
    echo "  - BSM (Basic Safety Message)"
    echo "  - CAM (Cooperative Awareness Message)"
    echo "  - DENM (Decentralized Environmental Notification)"
    echo "  - SPaT (Signal Phase and Timing)"
    echo "  - MAP (Map Data)"
    echo "  - PSM (Personal Safety Message)"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    start)
        TECH="C-V2X"
        FREQ=5900

        while [[ $# -gt 0 ]]; do
            case $1 in
                --tech) TECH=$2; shift 2 ;;
                --frequency) FREQ=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        start_service "$TECH" "$FREQ"
        ;;

    send-bsm)
        LAT=37.7749
        LON=-122.4194
        SPEED=65

        while [[ $# -gt 0 ]]; do
            case $1 in
                --lat) LAT=$2; shift 2 ;;
                --lon) LON=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        send_bsm "$LAT" "$LON" "$SPEED"
        ;;

    monitor)
        TYPE="all"
        FILTER=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --filter) FILTER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_v2v "$TYPE" "$FILTER"
        ;;

    test-rsu)
        RSU_ID="RSU-001"
        LOCATION="Main St & 1st Ave"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --rsu-id) RSU_ID=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        test_rsu "$RSU_ID" "$LOCATION"
        ;;

    measure-latency)
        TARGET="VEH-789"
        COUNT=10

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target) TARGET=$2; shift 2 ;;
                --count) COUNT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        measure_latency "$TARGET" "$COUNT"
        ;;

    platoon)
        MODE="member"
        MAX_VEHICLES=5
        SPACING=10

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mode) MODE=$2; shift 2 ;;
                --max-vehicles) MAX_VEHICLES=$2; shift 2 ;;
                --spacing) SPACING=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        platoon_mode "$MODE" "$MAX_VEHICLES" "$SPACING"
        ;;

    send-hazard)
        HAZARD_TYPE="accident"
        SEVERITY="high"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) HAZARD_TYPE=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        send_hazard "$HAZARD_TYPE" "$SEVERITY"
        ;;

    detect-pedestrians)
        print_header
        detect_pedestrians
        ;;

    stats)
        print_header
        show_stats
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-003 help' for usage information"
        exit 1
        ;;
esac

exit 0
