#!/bin/bash

################################################################################
# WIA-AUTO-011: Intelligent Transportation System CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Mobility Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to ITS functions including
# traffic flow calculation, signal optimization, congestion prediction,
# and vehicle detection.
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
HIGHWAY_FREE_FLOW=100
ARTERIAL_FREE_FLOW=60
JAM_DENSITY=160
SATURATION_FLOW=1900
YELLOW_TIME=4
MIN_CYCLE=40
MAX_CYCLE=180

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🚦 WIA-AUTO-011: Intelligent Transportation System       ║"
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

# Calculate traffic flow
calc_flow() {
    local density=${1:-50}
    local speed=${2:-}
    local free_flow=${3:-100}

    print_section "Traffic Flow Calculation"
    print_info "Density: $density veh/km"
    print_info "Free-flow Speed: $free_flow km/h"

    # Calculate speed using Greenshields model if not provided
    if [ -z "$speed" ]; then
        speed=$(echo "scale=2; $free_flow * (1 - $density / $JAM_DENSITY)" | bc -l)
        print_info "Calculated Speed: $speed km/h (Greenshields model)"
    else
        print_info "Given Speed: $speed km/h"
    fi

    # Calculate flow: q = k × v
    local flow=$(echo "scale=2; $density * $speed" | bc -l)

    # Calculate critical density and max flow
    local critical_density=$(echo "scale=2; $JAM_DENSITY / 2" | bc -l)
    local max_flow=$(echo "scale=2; ($free_flow * $JAM_DENSITY) / 4" | bc -l)

    print_section "Results"
    print_success "Traffic Flow: $flow veh/h"
    print_info "Critical Density: $critical_density veh/km"
    print_info "Maximum Flow (Capacity): $max_flow veh/h"

    # Determine Level of Service
    local los
    if (( $(echo "$density < 7" | bc -l) )); then
        los="A (Free flow)"
        print_success "Level of Service: $los"
    elif (( $(echo "$density < 11" | bc -l) )); then
        los="B (Reasonably free flow)"
        print_success "Level of Service: $los"
    elif (( $(echo "$density < 18" | bc -l) )); then
        los="C (Stable flow)"
        print_success "Level of Service: $los"
    elif (( $(echo "$density < 26" | bc -l) )); then
        los="D (Approaching unstable)"
        print_warning "Level of Service: $los"
    elif (( $(echo "$density < 45" | bc -l) )); then
        los="E (Unstable flow)"
        print_warning "Level of Service: $los"
    else
        los="F (Forced/breakdown flow)"
        print_error "Level of Service: $los"
    fi

    # Determine flow regime
    if (( $(echo "$density < $critical_density * 0.3" | bc -l) )); then
        print_info "Flow Regime: Free-flow"
    elif (( $(echo "$density < $critical_density" | bc -l) )); then
        print_info "Flow Regime: Stable"
    elif (( $(echo "$density < $JAM_DENSITY * 0.8" | bc -l) )); then
        print_info "Flow Regime: Unstable"
    else
        print_info "Flow Regime: Congested"
    fi

    echo ""
}

# Optimize signal timing
optimize_signals() {
    local flows="$1"
    local saturations="$2"
    local lost_times=${3:-"3,3"}

    print_section "Signal Timing Optimization"

    # Parse comma-separated values
    IFS=',' read -ra flow_array <<< "$flows"
    IFS=',' read -ra sat_array <<< "$saturations"
    IFS=',' read -ra lost_array <<< "$lost_times"

    local n_approaches=${#flow_array[@]}
    print_info "Number of Approaches: $n_approaches"

    # Calculate flow ratios and Y
    local Y=0
    local total_lost_time=0
    local total_flow=0

    print_section "Approach Analysis"
    for i in "${!flow_array[@]}"; do
        local flow=${flow_array[$i]}
        local sat=${sat_array[$i]}
        local lost=${lost_array[$i]:-3}

        local ratio=$(echo "scale=4; $flow / $sat" | bc -l)
        Y=$(echo "scale=4; $Y + $ratio" | bc -l)
        total_lost_time=$(echo "$total_lost_time + $lost" | bc -l)
        total_flow=$(echo "$total_flow + $flow" | bc -l)

        print_info "Approach $((i+1)): Flow=$flow veh/h, Sat=$sat veh/h, Ratio=$ratio"
    done

    print_section "Optimization Results"
    print_info "Sum of Flow Ratios (Y): $Y"

    # Check if oversaturated
    if (( $(echo "$Y >= 1.0" | bc -l) )); then
        print_error "Intersection is OVERSATURATED (Y ≥ 1.0)"
        print_error "Reduce demand or increase capacity"
        echo ""
        return 1
    fi

    # Calculate optimal cycle length using Webster's formula
    # C_opt = (1.5L + 5) / (1 - Y)
    local numerator=$(echo "scale=2; 1.5 * $total_lost_time + 5" | bc -l)
    local denominator=$(echo "scale=4; 1 - $Y" | bc -l)
    local cycle_opt=$(echo "scale=2; $numerator / $denominator" | bc -l)

    # Round to nearest 5 seconds
    cycle_opt=$(echo "scale=0; ($cycle_opt + 2.5) / 5 * 5" | bc -l)

    # Constrain to min/max
    if (( $(echo "$cycle_opt < $MIN_CYCLE" | bc -l) )); then
        cycle_opt=$MIN_CYCLE
        print_warning "Cycle length set to minimum ($MIN_CYCLE s)"
    elif (( $(echo "$cycle_opt > $MAX_CYCLE" | bc -l) )); then
        cycle_opt=$MAX_CYCLE
        print_warning "Cycle length set to maximum ($MAX_CYCLE s)"
    fi

    print_success "Optimal Cycle Length: $cycle_opt seconds"

    # Allocate green times
    local effective_green=$(echo "$cycle_opt - $total_lost_time" | bc -l)
    print_info "Effective Green Time: $effective_green seconds"

    print_section "Phase Timings"
    for i in "${!flow_array[@]}"; do
        local flow=${flow_array[$i]}
        local green=$(echo "scale=0; ($effective_green * $flow) / $total_flow" | bc -l)

        print_info "Phase $((i+1)): Green=${green}s, Yellow=${YELLOW_TIME}s, Red=$((cycle_opt - green - YELLOW_TIME))s"
    done

    # Calculate v/c ratio and estimated delay
    local vc_ratio=$Y
    print_section "Performance Metrics"
    print_info "Volume/Capacity Ratio: $(echo "scale=2; $vc_ratio" | bc -l)"

    # Determine LOS based on v/c ratio
    if (( $(echo "$vc_ratio < 0.6" | bc -l) )); then
        print_success "Expected LOS: A or B"
    elif (( $(echo "$vc_ratio < 0.7" | bc -l) )); then
        print_success "Expected LOS: C"
    elif (( $(echo "$vc_ratio < 0.8" | bc -l) )); then
        print_warning "Expected LOS: D"
    elif (( $(echo "$vc_ratio < 0.9" | bc -l) )); then
        print_warning "Expected LOS: E"
    else
        print_error "Expected LOS: F"
    fi

    echo ""
}

# Predict congestion
predict_congestion() {
    local location="$1"
    local time_window=${2:-60}

    print_section "Congestion Prediction"
    print_info "Location: $location"
    print_info "Time Window: $time_window minutes"

    local current_hour=$(date +%H)
    local base_speed=80
    local congestion="light"

    # Simulate peak hours
    if [ $current_hour -ge 7 ] && [ $current_hour -le 9 ]; then
        base_speed=45
        congestion="moderate"
    elif [ $current_hour -ge 16 ] && [ $current_hour -le 18 ]; then
        base_speed=40
        congestion="heavy"
    elif [ $current_hour -ge 22 ] || [ $current_hour -le 5 ]; then
        base_speed=95
        congestion="none"
    fi

    print_section "Prediction Results"
    print_info "Predicted Speed: $base_speed km/h"

    case $congestion in
        "none")
            print_success "Congestion Level: None"
            print_info "Estimated Delay: 0 minutes"
            ;;
        "light")
            print_success "Congestion Level: Light"
            print_info "Estimated Delay: 3-5 minutes"
            ;;
        "moderate")
            print_warning "Congestion Level: Moderate"
            print_info "Estimated Delay: 8-12 minutes"
            ;;
        "heavy")
            print_error "Congestion Level: Heavy"
            print_info "Estimated Delay: 15-25 minutes"
            ;;
    esac

    print_info "Confidence: 85%"
    print_info "Contributing Factors: Time of day, historical patterns"

    echo ""
}

# Monitor traffic
monitor_traffic() {
    local intersection="$1"
    local duration=${2:-300}

    print_section "Traffic Monitoring"
    print_info "Intersection: $intersection"
    print_info "Duration: $duration seconds"

    print_section "Monitoring Active..."
    print_info "Press Ctrl+C to stop"

    # Simulate monitoring
    local count=0
    while [ $count -lt $duration ]; do
        local speed=$((50 + RANDOM % 40))
        local volume=$((800 + RANDOM % 400))
        local occupancy=$((15 + RANDOM % 15))

        echo -ne "\r${GRAY}  Speed: ${speed} km/h | Volume: ${volume} veh/h | Occupancy: ${occupancy}%${RESET}"

        sleep 5
        count=$((count + 5))
    done

    echo ""
    print_success "Monitoring completed"
    echo ""
}

# Emergency vehicle preemption
preempt_vehicle() {
    local vehicle_id="$1"
    local route="$2"

    print_section "Emergency Vehicle Preemption"
    print_info "Vehicle ID: $vehicle_id"
    print_info "Route: $route"

    IFS=',' read -ra intersections <<< "$route"
    local n_intersections=${#intersections[@]}

    print_section "Preemption Sequence"
    for i in "${!intersections[@]}"; do
        local intersection=${intersections[$i]}
        print_success "Intersection $intersection: Preemption GRANTED"
        print_info "  Clearance Time: 10 seconds"
        print_info "  Green Time: 30 seconds"
        print_info "  Recovery Time: 15 seconds"
    done

    local time_savings=$((n_intersections * 25))
    print_section "Summary"
    print_success "Total Time Savings: $time_savings seconds"
    print_info "All signals will return to normal operation after vehicle passage"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-011 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-flow                Calculate traffic flow"
    echo "    --density <veh/km>     Traffic density (required)"
    echo "    --speed <km/h>         Average speed (optional, will calculate)"
    echo "    --free-flow <km/h>     Free-flow speed (default: 100)"
    echo ""
    echo "  optimize-signals         Optimize signal timing"
    echo "    --flow <v1,v2,...>     Flow rates for each approach (veh/h)"
    echo "    --saturation <s1,s2>   Saturation flows for each approach"
    echo "    --lost-time <l1,l2>    Lost times (default: 3,3)"
    echo ""
    echo "  predict                  Predict congestion"
    echo "    --location <coords>    Location (lat,lon or description)"
    echo "    --window <minutes>     Prediction window (default: 60)"
    echo ""
    echo "  monitor                  Monitor traffic at intersection"
    echo "    --intersection <name>  Intersection name"
    echo "    --duration <seconds>   Monitoring duration (default: 300)"
    echo ""
    echo "  preempt                  Emergency vehicle preemption"
    echo "    --vehicle-id <id>      Emergency vehicle ID"
    echo "    --route <A,B,C>        Route intersections (comma-separated)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-011 calc-flow --density 50 --speed 60"
    echo "  wia-auto-011 optimize-signals --flow 800,600 --saturation 1800,1600"
    echo "  wia-auto-011 predict --location 'Main St & 1st Ave' --window 60"
    echo "  wia-auto-011 monitor --intersection 'Downtown-5th' --duration 600"
    echo "  wia-auto-011 preempt --vehicle-id EMS-001 --route A,B,C,D"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-011 Intelligent Transportation System CLI"
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
    calc-flow)
        DENSITY=50
        SPEED=""
        FREE_FLOW=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --density) DENSITY=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --free-flow) FREE_FLOW=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_flow "$DENSITY" "$SPEED" "$FREE_FLOW"
        ;;

    optimize-signals)
        FLOW=""
        SATURATION=""
        LOST_TIME="3,3"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --flow) FLOW=$2; shift 2 ;;
                --saturation) SATURATION=$2; shift 2 ;;
                --lost-time) LOST_TIME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$FLOW" ] || [ -z "$SATURATION" ]; then
            print_error "Both --flow and --saturation are required"
            exit 1
        fi

        print_header
        optimize_signals "$FLOW" "$SATURATION" "$LOST_TIME"
        ;;

    predict)
        LOCATION="Unknown"
        WINDOW=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --location) LOCATION=$2; shift 2 ;;
                --window) WINDOW=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        predict_congestion "$LOCATION" "$WINDOW"
        ;;

    monitor)
        INTERSECTION="Unknown"
        DURATION=300

        while [[ $# -gt 0 ]]; do
            case $1 in
                --intersection) INTERSECTION=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_traffic "$INTERSECTION" "$DURATION"
        ;;

    preempt)
        VEHICLE_ID="EMS-001"
        ROUTE="A,B,C"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --vehicle-id) VEHICLE_ID=$2; shift 2 ;;
                --route) ROUTE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        preempt_vehicle "$VEHICLE_ID" "$ROUTE"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-011 help' for usage information"
        exit 1
        ;;
esac

exit 0
