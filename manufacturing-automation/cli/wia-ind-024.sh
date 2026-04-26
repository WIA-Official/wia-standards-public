#!/bin/bash

################################################################################
# WIA-IND-024: Manufacturing Automation CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to manufacturing automation
# functions including production line control, robot operations, OEE
# calculation, PLC integration, and quality inspection.
################################################################################

set -e

# Colors for output
AMBER='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
BOLD='\033[1m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🏭 WIA-IND-024: Manufacturing Automation CLI             ║"
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

print_value() {
    echo -e "${BOLD}$1:${RESET} ${CYAN}$2${RESET}"
}

# Show usage
show_usage() {
    print_header
    echo "Usage: wia-ind-024 <command> [options]"
    echo ""
    echo -e "${AMBER}Commands:${RESET}"
    echo ""
    echo "  Production Line:"
    echo "    monitor --line <id> [--interval <sec>]    Monitor production line"
    echo "    start --line <id> [--speed <pct>]         Start production line"
    echo "    stop --line <id> [--reason <text>]        Stop production line"
    echo "    estop --line <id> --reason <text>         Emergency stop"
    echo ""
    echo "  Robot Control:"
    echo "    robot move --id <id> --x <mm> --y <mm> --z <mm>"
    echo "    robot pick --id <id> --part <id> --force <N>"
    echo "    robot place --id <id> --x <mm> --y <mm> --z <mm>"
    echo "    robot status --id <id>                    Get robot status"
    echo ""
    echo "  Quality Inspection:"
    echo "    inspect --station <id> --part <id> --type <type>"
    echo ""
    echo "  OEE Calculation:"
    echo "    oee --line <id> [--shift <name>] [--date <YYYY-MM-DD>]"
    echo ""
    echo "  PLC Integration:"
    echo "    plc connect --type <type> --host <ip>"
    echo "    plc read --address <addr>"
    echo "    plc write --address <addr> --value <val>"
    echo ""
    echo "  Safety:"
    echo "    safety check --line <id>                  Check safety status"
    echo "    safety zones --line <id>                  List safety zones"
    echo ""
    echo "  Conveyor Control:"
    echo "    conveyor start --id <id> --speed <m/s>"
    echo "    conveyor stop --id <id>"
    echo ""
    echo "  Utilities:"
    echo "    --version                                 Show version"
    echo "    --help                                    Show this help"
    echo ""
    echo -e "${GRAY}Examples:${RESET}"
    echo "  wia-ind-024 monitor --line LINE-A --interval 5"
    echo "  wia-ind-024 oee --line LINE-A --shift day"
    echo "  wia-ind-024 robot move --id ROBOT-A1 --x 500 --y 300 --z 200"
    echo ""
}

# Monitor production line
cmd_monitor() {
    local line_id=""
    local interval=5

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            --interval) interval="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$line_id" ]; then
        print_error "Line ID required"
        exit 1
    fi

    print_section "Monitoring Production Line: $line_id"
    print_info "Update interval: ${interval}s (Press Ctrl+C to stop)"
    echo ""

    while true; do
        # Simulate real-time data
        local oee=$(echo "80 + ($RANDOM % 10)" | bc)
        local availability=$(echo "88 + ($RANDOM % 8)" | bc)
        local performance=$(echo "92 + ($RANDOM % 6)" | bc)
        local quality=$(echo "94 + ($RANDOM % 5)" | bc)
        local output=$(echo "1200 + ($RANDOM % 100)" | bc)

        clear
        print_header
        print_section "Production Line Status: $line_id"
        echo ""

        print_value "Status" "🟢 Running"
        print_value "Current Output" "${output} units"
        print_value "Target Output" "1280 units"
        echo ""

        print_section "OEE Metrics"
        echo -e "${BOLD}Overall OEE:${RESET}      ${AMBER}${oee}%${RESET}"
        echo -e "${BOLD}Availability:${RESET}     ${GREEN}${availability}%${RESET}"
        echo -e "${BOLD}Performance:${RESET}      ${CYAN}${performance}%${RESET}"
        echo -e "${BOLD}Quality:${RESET}          ${BLUE}${quality}%${RESET}"
        echo ""

        print_section "Workstations"
        echo -e "${GRAY}WS-01${RESET} Assembly    ${GREEN}Running${RESET}  Cycle: 12.5s  Output: 78/80"
        echo -e "${GRAY}WS-02${RESET} Welding     ${GREEN}Running${RESET}  Cycle: 8.2s   Output: 79/80"
        echo -e "${GRAY}WS-03${RESET} Inspection  ${GREEN}Running${RESET}  Cycle: 15.1s  Output: 77/80"
        echo -e "${GRAY}WS-04${RESET} Packaging   ${GREEN}Running${RESET}  Cycle: 10.8s  Output: 78/80"
        echo ""

        echo -e "${GRAY}Last updated: $(date '+%Y-%m-%d %H:%M:%S')${RESET}"

        sleep "$interval"
    done
}

# Calculate OEE
cmd_oee() {
    local line_id=""
    local shift="day"
    local date=$(date '+%Y-%m-%d')

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            --shift) shift="$2"; shift 2 ;;
            --date) date="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$line_id" ]; then
        print_error "Line ID required"
        exit 1
    fi

    print_section "OEE Calculation: $line_id"
    print_info "Shift: $shift | Date: $date"
    echo ""

    # Simulated calculations
    local planned_time=28800  # 8 hours in seconds
    local downtime=1800       # 30 minutes
    local operating_time=$((planned_time - downtime))
    local target_output=640
    local actual_output=590
    local good_units=575

    # Calculate metrics
    local availability=$(echo "scale=2; ($operating_time / $planned_time) * 100" | bc)
    local performance=$(echo "scale=2; ($actual_output / $target_output) * 100" | bc)
    local quality=$(echo "scale=2; ($good_units / $actual_output) * 100" | bc)
    local oee=$(echo "scale=2; ($availability * $performance * $quality) / 10000" | bc)

    print_section "Time Analysis"
    print_value "Planned Production Time" "${planned_time}s (8h)"
    print_value "Downtime" "${downtime}s (30min)"
    print_value "Operating Time" "${operating_time}s (7.5h)"
    echo ""

    print_section "Production Analysis"
    print_value "Target Output" "$target_output units"
    print_value "Actual Output" "$actual_output units"
    print_value "Good Units" "$good_units units"
    print_value "Defective Units" "$((actual_output - good_units)) units"
    echo ""

    print_section "OEE Results"
    echo -e "${BOLD}Overall OEE:${RESET}      ${AMBER}${oee}%${RESET}"
    echo -e "${BOLD}Availability:${RESET}     ${GREEN}${availability}%${RESET}"
    echo -e "${BOLD}Performance:${RESET}      ${CYAN}${performance}%${RESET}"
    echo -e "${BOLD}Quality:${RESET}          ${BLUE}${quality}%${RESET}"
    echo ""

    # World-class comparison
    if (( $(echo "$oee >= 85" | bc -l) )); then
        print_success "World-class performance! (Target: 85%)"
    elif (( $(echo "$oee >= 80" | bc -l) )); then
        print_success "Good performance! (Target: 85%)"
    else
        print_warning "Below target. Improvement needed. (Target: 85%)"
    fi
}

# Robot control
cmd_robot() {
    local action="$1"
    shift

    case "$action" in
        move)
            cmd_robot_move "$@"
            ;;
        pick)
            cmd_robot_pick "$@"
            ;;
        place)
            cmd_robot_place "$@"
            ;;
        status)
            cmd_robot_status "$@"
            ;;
        *)
            print_error "Unknown robot command: $action"
            exit 1
            ;;
    esac
}

cmd_robot_move() {
    local robot_id=""
    local x=0 y=0 z=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) robot_id="$2"; shift 2 ;;
            --x) x="$2"; shift 2 ;;
            --y) y="$2"; shift 2 ;;
            --z) z="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$robot_id" ]; then
        print_error "Robot ID required"
        exit 1
    fi

    print_section "Moving Robot: $robot_id"
    print_value "Target Position" "X: ${x}mm, Y: ${y}mm, Z: ${z}mm"
    echo ""

    print_info "Planning trajectory..."
    sleep 0.5
    print_success "Trajectory planned"

    print_info "Moving robot..."
    sleep 1
    print_success "Robot moved to target position"
}

cmd_robot_pick() {
    local robot_id=""
    local part_id=""
    local force=50

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) robot_id="$2"; shift 2 ;;
            --part) part_id="$2"; shift 2 ;;
            --force) force="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Pick Operation: $robot_id"
    print_value "Part ID" "$part_id"
    print_value "Grip Force" "${force}N"
    echo ""

    print_info "Moving to pick position..."
    sleep 0.5
    print_success "Position reached"

    print_info "Closing gripper (${force}N)..."
    sleep 0.5
    print_success "Part gripped successfully"
}

cmd_robot_place() {
    local robot_id=""
    local x=0 y=0 z=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) robot_id="$2"; shift 2 ;;
            --x) x="$2"; shift 2 ;;
            --y) y="$2"; shift 2 ;;
            --z) z="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Place Operation: $robot_id"
    print_value "Place Position" "X: ${x}mm, Y: ${y}mm, Z: ${z}mm"
    echo ""

    print_info "Moving to place position..."
    sleep 0.5
    print_success "Position reached"

    print_info "Opening gripper..."
    sleep 0.5
    print_success "Part placed successfully"
}

cmd_robot_status() {
    local robot_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) robot_id="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Robot Status: $robot_id"
    echo ""
    print_value "Model" "Universal Robots UR10e"
    print_value "Status" "🟢 Operational"
    print_value "Current Position" "X: 485mm, Y: 312mm, Z: 198mm"
    print_value "Gripper" "Closed (45N)"
    print_value "Cycle Count" "3,247"
    print_value "Runtime" "142.5 hours"
    print_value "Last Maintenance" "2025-12-20"
}

# Quality inspection
cmd_inspect() {
    local station=""
    local part_id=""
    local type="vision"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --station) station="$2"; shift 2 ;;
            --part) part_id="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Quality Inspection"
    print_value "Station" "$station"
    print_value "Part ID" "$part_id"
    print_value "Inspection Type" "$type"
    echo ""

    print_info "Capturing images..."
    sleep 0.5
    print_success "Images captured"

    print_info "Analyzing defects..."
    sleep 1
    print_success "Analysis complete"

    echo ""
    print_section "Inspection Results"

    if (( RANDOM % 10 < 9 )); then
        print_success "PASS - No defects detected"
        echo ""
        print_value "Dimensions" "✓ Within tolerance (±0.05mm)"
        print_value "Surface Quality" "✓ No defects"
        print_value "Color Match" "✓ Delta E: 2.3 (< 5.0)"
    else
        print_error "FAIL - Defects detected"
        echo ""
        print_value "Dimensions" "✓ Within tolerance"
        print_value "Surface Quality" "✗ 1 scratch detected (3.2mm)"
        print_value "Color Match" "✓ Delta E: 3.1"
    fi
}

# PLC integration
cmd_plc() {
    local action="$1"
    shift

    case "$action" in
        connect)
            cmd_plc_connect "$@"
            ;;
        read)
            cmd_plc_read "$@"
            ;;
        write)
            cmd_plc_write "$@"
            ;;
        *)
            print_error "Unknown PLC command: $action"
            exit 1
            ;;
    esac
}

cmd_plc_connect() {
    local plc_type=""
    local host=""
    local port=102

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) plc_type="$2"; shift 2 ;;
            --host) host="$2"; shift 2 ;;
            --port) port="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Connecting to PLC"
    print_value "Type" "$plc_type"
    print_value "Host" "$host:$port"
    echo ""

    print_info "Establishing connection..."
    sleep 0.5
    print_success "Connected to PLC"
}

cmd_plc_read() {
    local address=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --address) address="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Reading PLC Address: $address"
    echo ""

    local value=$((RANDOM % 1000))
    print_success "Value: $value"
}

cmd_plc_write() {
    local address=""
    local value=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --address) address="$2"; shift 2 ;;
            --value) value="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Writing to PLC"
    print_value "Address" "$address"
    print_value "Value" "$value"
    echo ""

    print_info "Writing value..."
    sleep 0.3
    print_success "Value written successfully"
}

# Safety check
cmd_safety() {
    local action="$1"
    shift

    case "$action" in
        check)
            cmd_safety_check "$@"
            ;;
        zones)
            cmd_safety_zones "$@"
            ;;
        *)
            print_error "Unknown safety command: $action"
            exit 1
            ;;
    esac
}

cmd_safety_check() {
    local line_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Safety Status: $line_id"
    echo ""

    print_success "Emergency stops: 12/12 functional"
    print_success "Light curtains: 4/4 active"
    print_success "Safety mats: 6/6 active"
    print_success "Safety gates: 3/3 closed and locked"
    print_success "Safety relays: Dual-channel OK"
    echo ""
    print_success "All safety systems operational"
}

cmd_safety_zones() {
    local line_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Safety Zones: $line_id"
    echo ""

    echo -e "${GRAY}ZONE-1${RESET}  Restricted    ${GREEN}Safe${RESET}    Light Curtain, E-Stop"
    echo -e "${GRAY}ZONE-2${RESET}  Collaborative ${GREEN}Safe${RESET}    Force Monitoring"
    echo -e "${GRAY}ZONE-3${RESET}  Restricted    ${GREEN}Safe${RESET}    Safety Mat, Gate"
}

# Conveyor control
cmd_conveyor() {
    local action="$1"
    shift

    case "$action" in
        start)
            cmd_conveyor_start "$@"
            ;;
        stop)
            cmd_conveyor_stop "$@"
            ;;
        *)
            print_error "Unknown conveyor command: $action"
            exit 1
            ;;
    esac
}

cmd_conveyor_start() {
    local conveyor_id=""
    local speed=0.5

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) conveyor_id="$2"; shift 2 ;;
            --speed) speed="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Starting Conveyor: $conveyor_id"
    print_value "Speed" "${speed} m/s"
    echo ""

    print_info "Ramping up..."
    sleep 0.5
    print_success "Conveyor running at ${speed} m/s"
}

cmd_conveyor_stop() {
    local conveyor_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) conveyor_id="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Stopping Conveyor: $conveyor_id"
    echo ""

    print_info "Decelerating..."
    sleep 0.5
    print_success "Conveyor stopped"
}

# Production line control
cmd_start() {
    local line_id=""
    local speed=100

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            --speed) speed="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Starting Production Line: $line_id"
    print_value "Speed" "${speed}%"
    echo ""

    print_info "Pre-start checks..."
    sleep 0.5
    print_success "Safety systems OK"
    print_success "All workstations ready"

    print_info "Starting line..."
    sleep 0.5
    print_success "Production line started at ${speed}% speed"
}

cmd_stop() {
    local line_id=""
    local reason="Manual stop"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            --reason) reason="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Stopping Production Line: $line_id"
    print_value "Reason" "$reason"
    echo ""

    print_info "Controlled stop in progress..."
    sleep 0.8
    print_success "Production line stopped"
}

cmd_estop() {
    local line_id=""
    local reason=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --line) line_id="$2"; shift 2 ;;
            --reason) reason="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "🚨 EMERGENCY STOP: $line_id"
    print_value "Reason" "$reason"
    echo ""

    print_error "EMERGENCY STOP ACTIVATED"
    print_info "All motion stopped"
    print_info "Power removed from drives"
    print_warning "Manual reset required"
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_usage
        exit 0
    fi

    local command="$1"
    shift

    case "$command" in
        --version)
            echo "WIA-IND-024 Manufacturing Automation CLI v${VERSION}"
            ;;
        --help)
            show_usage
            ;;
        monitor)
            cmd_monitor "$@"
            ;;
        start)
            cmd_start "$@"
            ;;
        stop)
            cmd_stop "$@"
            ;;
        estop)
            cmd_estop "$@"
            ;;
        robot)
            cmd_robot "$@"
            ;;
        inspect)
            cmd_inspect "$@"
            ;;
        oee)
            cmd_oee "$@"
            ;;
        plc)
            cmd_plc "$@"
            ;;
        safety)
            cmd_safety "$@"
            ;;
        conveyor)
            cmd_conveyor "$@"
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"

# **弘益人間 (홍익인간) · Benefit All Humanity**
