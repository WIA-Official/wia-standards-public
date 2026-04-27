#!/bin/bash

################################################################################
# WIA-AUTO-027: Universal Fluid for All Mobility CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Fluids Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to fluid condition analysis,
# replacement scheduling, compatibility checking, and real-time monitoring.
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
MAX_TAN=4.0
ACTION_TAN=2.5
MAX_WATER=0.2
ACTION_WATER=0.1
MAX_VISCOSITY_CHANGE=0.15

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🚗 WIA-AUTO-027: Universal Fluid Management CLI          ║"
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

# Calculate health score from parameters
calculate_health_score() {
    local viscosity=$1
    local tan=$2
    local water=$3
    local ref_viscosity=${4:-45}

    # Viscosity score (25% weight)
    local visc_change=$(echo "scale=4; ($viscosity - $ref_viscosity) / $ref_viscosity" | bc -l)
    local visc_change_abs=$(echo "scale=4; sqrt($visc_change * $visc_change)" | bc -l)
    local visc_score=$(echo "scale=2; (1 - ($visc_change_abs / $MAX_VISCOSITY_CHANGE)) * 25" | bc -l)
    if (( $(echo "$visc_score < 0" | bc -l) )); then visc_score=0; fi

    # TAN score (30% weight)
    local tan_score=$(echo "scale=2; (1 - ($tan / $MAX_TAN)) * 30" | bc -l)
    if (( $(echo "$tan_score < 0" | bc -l) )); then tan_score=0; fi

    # Water score (20% weight)
    local water_score=$(echo "scale=2; (1 - ($water / $MAX_WATER)) * 20" | bc -l)
    if (( $(echo "$water_score < 0" | bc -l) )); then water_score=0; fi

    # Particle score (15% weight) - assume good for now
    local particle_score=15

    # Oxidation score (10% weight) - assume good for now
    local oxidation_score=10

    # Total health score
    local health=$(echo "scale=2; $visc_score + $tan_score + $water_score + $particle_score + $oxidation_score" | bc -l)

    echo "$health"
}

# Get condition status from health score
get_condition_status() {
    local health=$1

    if (( $(echo "$health >= 80" | bc -l) )); then
        echo "excellent"
    elif (( $(echo "$health >= 60" | bc -l) )); then
        echo "good"
    elif (( $(echo "$health >= 40" | bc -l) )); then
        echo "acceptable"
    elif (( $(echo "$health >= 20" | bc -l) )); then
        echo "marginal"
    else
        echo "critical"
    fi
}

# Get recommendation from health score
get_recommendation() {
    local health=$1

    if (( $(echo "$health >= 80" | bc -l) )); then
        echo "continue_monitoring"
    elif (( $(echo "$health >= 60" | bc -l) )); then
        echo "increase_monitoring"
    elif (( $(echo "$health >= 40" | bc -l) )); then
        echo "plan_replacement"
    else
        echo "immediate_replacement"
    fi
}

# Analyze fluid condition
analyze_fluid() {
    local viscosity=${1:-45.0}
    local tan=${2:-1.0}
    local water=${3:-0.05}
    local particles=${4:-15}
    local temperature=${5:-40}
    local ref_viscosity=${6:-45}

    print_section "Fluid Condition Analysis"
    print_info "Viscosity: $viscosity cSt @ ${temperature}°C (Reference: $ref_viscosity cSt)"
    print_info "Total Acid Number (TAN): $tan mg KOH/g"
    print_info "Water Content: $(echo "scale=2; $water * 100" | bc -l)%"
    print_info "Particle Count: $particles particles/mL (≥25μm)"

    # Calculate health score
    local health=$(calculate_health_score $viscosity $tan $water $ref_viscosity)
    local condition=$(get_condition_status $health)
    local recommendation=$(get_recommendation $health)

    print_section "Analysis Results"

    # Display health score with color
    if (( $(echo "$health >= 80" | bc -l) )); then
        print_success "Health Score: $health/100 (Excellent)"
    elif (( $(echo "$health >= 60" | bc -l) )); then
        print_success "Health Score: $health/100 (Good)"
    elif (( $(echo "$health >= 40" | bc -l) )); then
        print_warning "Health Score: $health/100 (Acceptable)"
    elif (( $(echo "$health >= 20" | bc -l) )); then
        print_warning "Health Score: $health/100 (Marginal)"
    else
        print_error "Health Score: $health/100 (Critical)"
    fi

    print_info "Condition Status: $condition"

    # Individual parameter checks
    print_section "Parameter Status"

    # Viscosity check
    local visc_change=$(echo "scale=4; ($viscosity - $ref_viscosity) / $ref_viscosity" | bc -l)
    local visc_change_abs=$(echo "scale=4; sqrt($visc_change * $visc_change)" | bc -l)
    local visc_pct=$(echo "scale=1; $visc_change_abs * 100" | bc -l)

    if (( $(echo "$visc_change_abs < 0.10" | bc -l) )); then
        print_success "Viscosity Change: ${visc_pct}% (Normal)"
    elif (( $(echo "$visc_change_abs < $MAX_VISCOSITY_CHANGE" | bc -l) )); then
        print_warning "Viscosity Change: ${visc_pct}% (Caution - Action limit 10%)"
    else
        print_error "Viscosity Change: ${visc_pct}% (Critical - Condemning limit 15%)"
    fi

    # TAN check
    if (( $(echo "$tan < $ACTION_TAN" | bc -l) )); then
        print_success "TAN: $tan mg KOH/g (Normal)"
    elif (( $(echo "$tan < $MAX_TAN" | bc -l) )); then
        print_warning "TAN: $tan mg KOH/g (Warning - Action limit $ACTION_TAN)"
    else
        print_error "TAN: $tan mg KOH/g (Critical - Condemning limit $MAX_TAN)"
    fi

    # Water check
    if (( $(echo "$water < $ACTION_WATER" | bc -l) )); then
        print_success "Water Content: $(echo "scale=2; $water * 100" | bc -l)% (Normal)"
    elif (( $(echo "$water < $MAX_WATER" | bc -l) )); then
        print_warning "Water Content: $(echo "scale=2; $water * 100" | bc -l)% (Warning - Action limit $(echo "scale=1; $ACTION_WATER * 100" | bc -l)%)"
    else
        print_error "Water Content: $(echo "scale=2; $water * 100" | bc -l)% (Critical - Condemning limit $(echo "scale=1; $MAX_WATER * 100" | bc -l)%)"
    fi

    # Particle check
    if (( $particles <= 15 )); then
        print_success "Particle Count: $particles particles/mL (Normal - ISO 16/14/11)"
    elif (( $particles <= 25 )); then
        print_warning "Particle Count: $particles particles/mL (Caution - ISO 18/16/13)"
    else
        print_error "Particle Count: $particles particles/mL (Critical - ISO 20/18/15)"
    fi

    # Recommendation
    print_section "Recommendation"

    case $recommendation in
        "continue_monitoring")
            print_success "Action: Continue normal monitoring schedule"
            print_info "Next test: 1 month or 500 operating hours"
            ;;
        "increase_monitoring")
            print_warning "Action: Increase monitoring frequency"
            print_info "Next test: 2 weeks or 250 operating hours"
            ;;
        "plan_replacement")
            print_warning "Action: Plan fluid replacement within 1 month"
            print_info "Estimated remaining life: 200-500 hours"
            ;;
        "immediate_replacement")
            print_error "Action: IMMEDIATE REPLACEMENT REQUIRED"
            print_error "Fluid has reached condemning limits"
            ;;
    esac

    echo ""
}

# Check fluid compatibility
check_compatibility() {
    local fluid_a="$1"
    local fluid_b="$2"
    local ratio=${3:-0.5}

    print_section "Fluid Compatibility Check"
    print_info "Fluid A: $fluid_a"
    print_info "Fluid B: $fluid_b"
    print_info "Mixture Ratio: $(echo "scale=0; $ratio * 100" | bc -l)% Fluid B"

    print_section "Compatibility Analysis"

    # Simple compatibility logic based on fluid types
    local compatible=true
    local score=95

    case "$fluid_a-$fluid_b" in
        *synthetic*-*synthetic*)
            print_success "Base Type Compatibility: Excellent (Both synthetic)"
            ;;
        *bio*-*bio*)
            print_success "Base Type Compatibility: Excellent (Both bio-based)"
            ;;
        *mineral*-*mineral*)
            print_success "Base Type Compatibility: Good (Both mineral)"
            ;;
        *synthetic*-*mineral*|*mineral*-*synthetic*)
            print_warning "Base Type Compatibility: Fair (Synthetic-Mineral mix)"
            score=75
            ;;
        *bio*-*mineral*|*mineral*-*bio*)
            print_warning "Base Type Compatibility: Fair (Bio-Mineral mix)"
            score=70
            ;;
        *bio*-*synthetic*|*synthetic*-*bio*)
            print_success "Base Type Compatibility: Good (Bio-Synthetic mix)"
            score=85
            ;;
    esac

    print_section "Results"
    if [ "$score" -ge 90 ]; then
        print_success "Compatibility Score: $score/100 (Highly Compatible)"
        print_success "Mixing Approved: Yes"
    elif [ "$score" -ge 70 ]; then
        print_warning "Compatibility Score: $score/100 (Moderately Compatible)"
        print_warning "Mixing Approved: Yes, with testing recommended"
    else
        print_error "Compatibility Score: $score/100 (Poor Compatibility)"
        print_error "Mixing Approved: Not recommended"
        compatible=false
    fi

    print_section "Recommendations"
    if [ "$compatible" = true ]; then
        print_info "• Perform compatibility test before full implementation"
        print_info "• Monitor viscosity after mixing"
        print_info "• Check for phase separation after 24 hours"
        print_info "• Verify additive compatibility"
    else
        print_info "• Do not mix these fluids"
        print_info "• Drain system completely before changing fluid type"
        print_info "• Flush system if necessary"
    fi

    echo ""
}

# Calculate replacement schedule
calculate_schedule() {
    local fluid_type="$1"
    local hours=${2:-5000}
    local condition=${3:-"good"}
    local vehicle_type=${4:-"electric-vehicle"}

    print_section "Replacement Schedule Calculation"
    print_info "Fluid Type: $fluid_type"
    print_info "Current Operating Hours: $hours"
    print_info "Current Condition: $condition"
    print_info "Vehicle Type: $vehicle_type"

    # Base service intervals by fluid type
    local base_interval
    case "$fluid_type" in
        *synthetic*|*bio*)
            base_interval=8000
            ;;
        *mineral*)
            base_interval=5000
            ;;
        *)
            base_interval=6000
            ;;
    esac

    # Adjust for vehicle type
    case "$vehicle_type" in
        *electric*|*ev*)
            base_interval=$(echo "$base_interval * 1.2" | bc)
            ;;
        *hybrid*)
            base_interval=$(echo "$base_interval * 1.1" | bc)
            ;;
        *heavy*|*industrial*)
            base_interval=$(echo "$base_interval * 0.8" | bc)
            ;;
    esac

    # Adjust for current condition
    local remaining
    case "$condition" in
        excellent)
            remaining=$(echo "$base_interval - $hours" | bc)
            ;;
        good)
            remaining=$(echo "$base_interval * 0.9 - $hours" | bc)
            ;;
        acceptable)
            remaining=$(echo "$base_interval * 0.7 - $hours" | bc)
            ;;
        marginal)
            remaining=500
            ;;
        critical)
            remaining=0
            ;;
    esac

    print_section "Schedule Results"
    print_info "Base Service Interval: $(printf "%.0f" $base_interval) hours"
    print_info "Adjusted for Vehicle Type: $(printf "%.0f" $(echo "$base_interval" | bc)) hours"

    if (( $(echo "$remaining > 2000" | bc -l) )); then
        print_success "Remaining Service Life: $(printf "%.0f" $remaining) hours"
        print_success "Next Replacement: In $(printf "%.0f" $(echo "$remaining / 24" | bc)) days (estimated)"
    elif (( $(echo "$remaining > 500" | bc -l) )); then
        print_warning "Remaining Service Life: $(printf "%.0f" $remaining) hours"
        print_warning "Next Replacement: Plan within 1 month"
    elif (( $(echo "$remaining > 0" | bc -l) )); then
        print_error "Remaining Service Life: $(printf "%.0f" $remaining) hours"
        print_error "Next Replacement: Within 2 weeks (urgent)"
    else
        print_error "Remaining Service Life: 0 hours"
        print_error "Next Replacement: IMMEDIATE"
    fi

    print_section "Cost Estimate"
    local fluid_cost=180
    local labor_cost=70
    local total=$(echo "$fluid_cost + $labor_cost" | bc)

    print_info "Fluid Cost: \$$fluid_cost"
    print_info "Labor Cost: \$$labor_cost"
    print_success "Total Estimated Cost: \$$total"

    echo ""
}

# Monitor fluid (simulated real-time)
monitor_fluid() {
    local sensor_id=${1:-"UFAM-001"}
    local interval=${2:-5}

    print_section "Real-Time Fluid Monitoring"
    print_info "Sensor ID: $sensor_id"
    print_info "Update Interval: $interval seconds"
    print_warning "Press Ctrl+C to stop monitoring"
    echo ""

    local count=0
    while true; do
        count=$((count + 1))

        # Simulate sensor readings with slight variations
        local base_visc=45.0
        local base_tan=1.5
        local base_temp=85

        local variation=$(echo "scale=2; ($RANDOM % 100) / 100 - 0.5" | bc -l)
        local visc=$(echo "scale=2; $base_visc + $variation" | bc -l)
        local tan=$(echo "scale=3; $base_tan + ($variation / 10)" | bc -l)
        local temp=$(echo "scale=1; $base_temp + ($variation * 5)" | bc -l)
        local water=$(echo "scale=3; 0.05 + ($variation / 100)" | bc -l)

        local health=$(calculate_health_score $visc $tan $water)

        echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')] Reading #$count${RESET}"
        echo -e "${GRAY}  Viscosity: $visc cSt @ ${temp}°C${RESET}"
        echo -e "${GRAY}  TAN: $tan mg KOH/g${RESET}"
        echo -e "${GRAY}  Water: $(echo "scale=2; $water * 100" | bc -l)%${RESET}"

        if (( $(echo "$health >= 80" | bc -l) )); then
            echo -e "${GREEN}  Health: $health/100 ✓${RESET}"
        elif (( $(echo "$health >= 60" | bc -l) )); then
            echo -e "${YELLOW}  Health: $health/100 ⚠${RESET}"
        else
            echo -e "${RED}  Health: $health/100 ✗${RESET}"
        fi
        echo ""

        sleep $interval
    done
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-027 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  analyze                  Analyze fluid condition from measurements"
    echo "    --viscosity <cSt>      Kinematic viscosity (default: 45.0)"
    echo "    --tan <mg KOH/g>       Total Acid Number (default: 1.0)"
    echo "    --water <percent>      Water content (default: 0.05)"
    echo "    --particles <count>    Particle count per mL (default: 15)"
    echo "    --temperature <°C>     Measurement temperature (default: 40)"
    echo "    --reference <cSt>      Reference viscosity (default: 45)"
    echo ""
    echo "  compatibility            Check fluid mixing compatibility"
    echo "    --fluid-a <type>       First fluid type"
    echo "    --fluid-b <type>       Second fluid type"
    echo "    --ratio <0-1>          Mixture ratio (default: 0.5)"
    echo ""
    echo "  schedule                 Calculate replacement schedule"
    echo "    --type <fluid>         Fluid type (synthetic/mineral/bio)"
    echo "    --hours <hours>        Current operating hours (default: 5000)"
    echo "    --condition <status>   Current condition (default: good)"
    echo "    --vehicle <type>       Vehicle type (default: electric-vehicle)"
    echo ""
    echo "  monitor                  Real-time fluid monitoring (simulated)"
    echo "    --sensor-id <id>       Sensor identifier (default: UFAM-001)"
    echo "    --interval <seconds>   Update interval (default: 5)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-027 analyze --viscosity 46.5 --tan 2.1 --water 0.08"
    echo "  wia-auto-027 compatibility --fluid-a synthetic --fluid-b mineral"
    echo "  wia-auto-027 schedule --type bio-based --hours 6000 --condition good"
    echo "  wia-auto-027 monitor --sensor-id UFAM-001 --interval 10"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-027 Universal Fluid Management CLI"
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
    analyze)
        VISCOSITY=45.0
        TAN=1.0
        WATER=0.05
        PARTICLES=15
        TEMPERATURE=40
        REFERENCE=45

        while [[ $# -gt 0 ]]; do
            case $1 in
                --viscosity) VISCOSITY=$2; shift 2 ;;
                --tan) TAN=$2; shift 2 ;;
                --water) WATER=$2; shift 2 ;;
                --particles) PARTICLES=$2; shift 2 ;;
                --temperature) TEMPERATURE=$2; shift 2 ;;
                --reference) REFERENCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_fluid "$VISCOSITY" "$TAN" "$WATER" "$PARTICLES" "$TEMPERATURE" "$REFERENCE"
        ;;

    compatibility)
        FLUID_A="synthetic"
        FLUID_B="mineral"
        RATIO=0.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --fluid-a) FLUID_A=$2; shift 2 ;;
                --fluid-b) FLUID_B=$2; shift 2 ;;
                --ratio) RATIO=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        check_compatibility "$FLUID_A" "$FLUID_B" "$RATIO"
        ;;

    schedule)
        TYPE="synthetic"
        HOURS=5000
        CONDITION="good"
        VEHICLE="electric-vehicle"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --hours) HOURS=$2; shift 2 ;;
                --condition) CONDITION=$2; shift 2 ;;
                --vehicle) VEHICLE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calculate_schedule "$TYPE" "$HOURS" "$CONDITION" "$VEHICLE"
        ;;

    monitor)
        SENSOR_ID="UFAM-001"
        INTERVAL=5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --sensor-id) SENSOR_ID=$2; shift 2 ;;
                --interval) INTERVAL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_fluid "$SENSOR_ID" "$INTERVAL"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-027 help' for usage information"
        exit 1
        ;;
esac

exit 0
