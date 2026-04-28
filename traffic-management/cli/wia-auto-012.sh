#!/bin/bash

################################################################################
# WIA-AUTO-012: Traffic Management CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive & Mobility Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to traffic management calculations
# including flow analysis, signal optimization, congestion detection, and
# traffic prediction.
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
FREE_FLOW_SPEED=100  # km/h
JAM_DENSITY=180      # veh/km
SATURATION_FLOW=1900 # veh/h/lane

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🚦 WIA-AUTO-012: Traffic Management CLI              ║"
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
    local density=${1:-30}
    local speed=${2:-80}
    local lanes=${3:-3}

    print_section "Traffic Flow Calculation"
    print_info "Density: $density veh/km"
    print_info "Speed: $speed km/h"
    print_info "Lanes: $lanes"

    # Calculate flow: q = k × v
    local flow=$(echo "$density * $speed" | bc -l)
    print_info "Flow (per lane): $(printf "%.0f" $flow) veh/h"

    # Total flow
    local total_flow=$(echo "$flow * $lanes" | bc -l)

    # Calculate critical density and capacity
    local kc=$(echo "$JAM_DENSITY / 2" | bc -l)
    local capacity=$(echo "$FREE_FLOW_SPEED * $JAM_DENSITY / 4 * $lanes" | bc -l)

    print_section "Results"
    print_success "Total Flow: $(printf "%.0f" $total_flow) veh/h"
    print_info "Capacity: $(printf "%.0f" $capacity) veh/h"

    # V/C ratio
    local vc_ratio=$(echo "scale=3; $total_flow / $capacity" | bc -l)
    print_info "V/C Ratio: $vc_ratio"

    # Determine regime
    if (( $(echo "$density < $kc * 0.5" | bc -l) )); then
        print_success "Regime: FREE FLOW"
        print_success "Level of Service: A or B"
    elif (( $(echo "$density >= $kc * 0.5 && $density <= $kc * 1.2" | bc -l) )); then
        print_warning "Regime: CAPACITY"
        print_warning "Level of Service: C or D"
    else
        print_error "Regime: CONGESTED"
        print_error "Level of Service: E or F"
    fi

    # Check for congestion
    local congestion_threshold=$(echo "$FREE_FLOW_SPEED * 0.3" | bc -l)
    if (( $(echo "$speed < $congestion_threshold" | bc -l) )); then
        print_error "Status: CONGESTED (Speed < 30% of free-flow)"
    else
        print_success "Status: FLOWING"
    fi

    echo ""
}

# Optimize signal timing
optimize_signal() {
    local phase_volumes="$1"
    local lost_time=${2:-4}

    print_section "Signal Timing Optimization"

    # Parse phase volumes
    IFS=',' read -ra VOLUMES <<< "$phase_volumes"
    local num_phases=${#VOLUMES[@]}

    print_info "Number of phases: $num_phases"
    print_info "Lost time per phase: ${lost_time}s"

    # Calculate flow ratios
    local Y=0
    for vol in "${VOLUMES[@]}"; do
        local y=$(echo "scale=4; $vol / $SATURATION_FLOW" | bc -l)
        Y=$(echo "$Y + $y" | bc -l)
        print_info "Phase volume: $vol veh/h (y = $y)"
    done

    print_section "Analysis"
    print_info "Critical Flow Ratio (Y): $Y"

    # Check for over-saturation
    if (( $(echo "$Y >= 1.0" | bc -l) )); then
        print_error "OVER-SATURATED: Y >= 1.0"
        print_error "Intersection cannot handle this demand"
        echo ""
        print_section "Recommendations"
        print_info "• Add more lanes"
        print_info "• Prohibit some turning movements"
        print_info "• Implement one-way system"
        print_info "• Provide alternative routes"
        echo ""
        return 1
    fi

    # Calculate optimal cycle time using Webster's formula
    local L=$(echo "$lost_time * $num_phases" | bc -l)
    local Co=$(echo "scale=2; (1.5 * $L + 5) / (1 - $Y)" | bc -l)

    # Constrain cycle time
    if (( $(echo "$Co < 60" | bc -l) )); then
        Co=60
        print_warning "Cycle time adjusted to minimum: 60s"
    elif (( $(echo "$Co > 120" | bc -l) )); then
        Co=120
        print_warning "Cycle time adjusted to maximum: 120s"
    fi

    print_section "Optimal Signal Timing"
    print_success "Cycle Time: ${Co}s"

    # Allocate green times
    local effective_green=$(echo "$Co - $L" | bc -l)
    local phase_num=1

    for vol in "${VOLUMES[@]}"; do
        local y=$(echo "scale=4; $vol / $SATURATION_FLOW" | bc -l)
        local green=$(echo "scale=1; $effective_green * $y / $Y" | bc -l)

        # Calculate delay
        local lambda=$(echo "scale=4; $green / $Co" | bc -l)
        local x=$(echo "scale=4; $vol / ($SATURATION_FLOW * $lambda)" | bc -l)

        if (( $(echo "$x < 1" | bc -l) )); then
            local delay=$(echo "scale=1; ($Co * (1-$lambda)^2) / (2*(1-$lambda*$x))" | bc -l)
        else
            local delay=999
        fi

        print_info "Phase $phase_num: Green=${green}s, Delay=${delay}s"
        phase_num=$((phase_num + 1))
    done

    # Determine LOS
    print_section "Performance"
    if (( $(echo "$Y < 0.85" | bc -l) )); then
        print_success "Performance: GOOD (Y < 0.85)"
    else
        print_warning "Performance: FAIR (Y approaching 1.0)"
    fi

    echo ""
}

# Detect congestion
detect_congestion() {
    local speed=${1:-25}
    local density=${2:-120}
    local free_flow=${3:-$FREE_FLOW_SPEED}

    print_section "Congestion Detection"
    print_info "Current Speed: $speed km/h"
    print_info "Current Density: $density veh/km"
    print_info "Free-flow Speed: $free_flow km/h"

    # Calculate travel time index
    local tti=$(echo "scale=2; $free_flow / $speed" | bc -l)

    # Speed reduction
    local speed_reduction=$(echo "scale=1; ($free_flow - $speed) / $free_flow * 100" | bc -l)

    print_section "Metrics"
    print_info "Travel Time Index: $tti"
    print_info "Speed Reduction: ${speed_reduction}%"

    # Determine congestion severity
    print_section "Congestion Status"

    local congestion_threshold=$(echo "$free_flow * 0.3" | bc -l)
    if (( $(echo "$speed >= $congestion_threshold" | bc -l) )); then
        print_success "Severity: NONE - Traffic flowing normally"
        print_info "Score: 0/100"
    elif (( $(echo "$tti < 1.3" | bc -l) )); then
        print_warning "Severity: MILD - Minor delays"
        print_info "Score: 25/100"
        print_section "Recommendations"
        print_info "• Monitor situation"
        print_info "• Prepare variable message signs"
    elif (( $(echo "$tti < 1.8" | bc -l) )); then
        print_warning "Severity: MODERATE - Noticeable delays"
        print_info "Score: 50/100"
        print_section "Recommendations"
        print_info "• Activate variable message signs"
        print_info "• Adjust signal timing"
        print_info "• Consider ramp metering"
    elif (( $(echo "$tti < 2.5" | bc -l) )); then
        print_error "Severity: SEVERE - Significant delays"
        print_info "Score: 75/100"
        print_section "Recommendations"
        print_info "• Implement traffic diversion"
        print_info "• Activate incident management team"
        print_info "• Alert navigation systems"
    else
        print_error "Severity: GRIDLOCK - Critical congestion"
        print_info "Score: 100/100"
        print_section "Recommendations"
        print_info "• EMERGENCY RESPONSE REQUIRED"
        print_info "• Full traffic diversion"
        print_info "• Public transportation alerts"
        print_info "• Coordinate with police"
    fi

    echo ""
}

# Predict traffic
predict_traffic() {
    local hour=${1:-8}
    local day=${2:-monday}
    local weather=${3:-clear}

    print_section "Traffic Prediction"
    print_info "Hour: ${hour}:00"
    print_info "Day: $day"
    print_info "Weather: $weather"

    # Base flow
    local base_flow=800

    # Time-of-day factor
    if [ $hour -ge 7 ] && [ $hour -le 9 ]; then
        local tod_factor=1.5
        local period="AM Peak"
    elif [ $hour -ge 16 ] && [ $hour -le 18 ]; then
        local tod_factor=1.6
        local period="PM Peak"
    elif [ $hour -ge 11 ] && [ $hour -le 13 ]; then
        local tod_factor=1.1
        local period="Lunch"
    elif [ $hour -ge 0 ] && [ $hour -le 5 ]; then
        local tod_factor=0.3
        local period="Overnight"
    else
        local tod_factor=1.0
        local period="Off-Peak"
    fi

    # Day-of-week factor
    case $day in
        monday|tuesday|wednesday|thursday)
            local dow_factor=1.0
            ;;
        friday)
            local dow_factor=1.1
            ;;
        saturday)
            local dow_factor=0.7
            ;;
        sunday)
            local dow_factor=0.5
            ;;
        *)
            local dow_factor=1.0
            ;;
    esac

    # Weather factor
    case $weather in
        clear)
            local weather_factor=1.0
            ;;
        rain)
            local weather_factor=0.9
            ;;
        snow)
            local weather_factor=0.7
            ;;
        fog)
            local weather_factor=0.8
            ;;
        *)
            local weather_factor=1.0
            ;;
    esac

    # Calculate predicted flow
    local predicted_flow=$(echo "scale=0; $base_flow * $tod_factor * $dow_factor * $weather_factor" | bc -l)

    # Estimate speed
    local predicted_speed=95
    if (( $(echo "$predicted_flow > 1400" | bc -l) )); then
        predicted_speed=65
    elif (( $(echo "$predicted_flow > 1000" | bc -l) )); then
        predicted_speed=80
    fi

    print_section "Prediction Results"
    print_success "Period: $period"
    print_success "Predicted Flow: $predicted_flow veh/h"
    print_success "Predicted Speed: $predicted_speed km/h"
    print_info "Confidence: 85%"

    print_section "Factors Applied"
    print_info "Time-of-day: ${tod_factor}x"
    print_info "Day-of-week: ${dow_factor}x"
    print_info "Weather: ${weather_factor}x"

    # Determine expected conditions
    print_section "Expected Conditions"
    if (( $(echo "$predicted_flow < 1000" | bc -l) )); then
        print_success "Light traffic - Free flow expected"
    elif (( $(echo "$predicted_flow < 1400" | bc -l) )); then
        print_warning "Moderate traffic - Minor delays possible"
    else
        print_error "Heavy traffic - Congestion likely"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-012 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-flow                Calculate traffic flow metrics"
    echo "    --density <veh/km>     Traffic density (default: 30)"
    echo "    --speed <km/h>         Average speed (default: 80)"
    echo "    --lanes <count>        Number of lanes (default: 3)"
    echo ""
    echo "  optimize-signal          Optimize signal timing"
    echo "    --phases <volumes>     Comma-separated phase volumes (e.g., \"1200,900\")"
    echo "    --lost-time <sec>      Lost time per phase (default: 4)"
    echo ""
    echo "  detect-congestion        Detect and assess congestion"
    echo "    --speed <km/h>         Current speed (default: 25)"
    echo "    --density <veh/km>     Current density (default: 120)"
    echo "    --free-flow <km/h>     Free-flow speed (default: 100)"
    echo ""
    echo "  predict                  Predict traffic conditions"
    echo "    --hour <0-23>          Hour of day (default: 8)"
    echo "    --day <name>           Day of week (default: monday)"
    echo "    --weather <condition>  Weather (clear/rain/snow/fog)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-012 calc-flow --density 45 --speed 60 --lanes 3"
    echo "  wia-auto-012 optimize-signal --phases \"1200,900\" --lost-time 4"
    echo "  wia-auto-012 detect-congestion --speed 25 --density 120"
    echo "  wia-auto-012 predict --hour 8 --day monday --weather clear"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-012 Traffic Management CLI Tool"
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
    calc-flow)
        DENSITY=30
        SPEED=80
        LANES=3

        while [[ $# -gt 0 ]]; do
            case $1 in
                --density) DENSITY=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --lanes) LANES=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_flow "$DENSITY" "$SPEED" "$LANES"
        ;;

    optimize-signal)
        PHASES="1200,900"
        LOST_TIME=4

        while [[ $# -gt 0 ]]; do
            case $1 in
                --phases) PHASES=$2; shift 2 ;;
                --lost-time) LOST_TIME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        optimize_signal "$PHASES" "$LOST_TIME"
        ;;

    detect-congestion)
        SPEED=25
        DENSITY=120
        FREE_FLOW=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --speed) SPEED=$2; shift 2 ;;
                --density) DENSITY=$2; shift 2 ;;
                --free-flow) FREE_FLOW=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        detect_congestion "$SPEED" "$DENSITY" "$FREE_FLOW"
        ;;

    predict)
        HOUR=8
        DAY="monday"
        WEATHER="clear"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --hour) HOUR=$2; shift 2 ;;
                --day) DAY=$2; shift 2 ;;
                --weather) WEATHER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        predict_traffic "$HOUR" "$DAY" "$WEATHER"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-012 help' for usage information"
        exit 1
        ;;
esac

exit 0
