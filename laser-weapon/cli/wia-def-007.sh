#!/bin/bash

################################################################################
# WIA-DEF-007: Laser Weapon Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Systems Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to laser weapon system calculations
# including beam propagation, engagement planning, and C-RAM simulations.
################################################################################

set -e

# Colors for output
SLATE='\033[38;5;109m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SPEED_OF_LIGHT=299792458
PI=3.14159265359

# Helper functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🔴 WIA-DEF-007: Laser Weapon Standard CLI            ║"
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

format_power() {
    local power=$1

    if (( $(echo "$power < 1000" | bc -l) )); then
        printf "%.1f W" "$power"
    elif (( $(echo "$power < 1000000" | bc -l) )); then
        printf "%.1f kW" "$(echo "$power / 1000" | bc -l)"
    else
        printf "%.1f MW" "$(echo "$power / 1000000" | bc -l)"
    fi
}

format_energy() {
    local energy=$1

    if (( $(echo "$energy < 1000" | bc -l) )); then
        printf "%.1f J" "$energy"
    elif (( $(echo "$energy < 1000000" | bc -l) )); then
        printf "%.1f kJ" "$(echo "$energy / 1000" | bc -l)"
    elif (( $(echo "$energy < 1000000000" | bc -l) )); then
        printf "%.1f MJ" "$(echo "$energy / 1000000" | bc -l)"
    else
        printf "%.1f GJ" "$(echo "$energy / 1000000000" | bc -l)"
    fi
}

# Calculate beam parameters
calc_beam() {
    local power=${1:-100000}
    local range=${2:-2000}
    local diameter=${3:-0.2}

    print_section "Beam Calculation"
    print_info "Laser Power: $(format_power $power)"
    print_info "Target Range: ${range} m"
    print_info "Beam Diameter: ${diameter} m"

    # Beam radius
    local radius=$(echo "$diameter / 2" | bc -l)

    # Beam area (m²)
    local area=$(echo "$PI * $radius * $radius" | bc -l)

    # Intensity (W/m²)
    local intensity=$(echo "$power / $area" | bc -l)

    # Divergence (assume M² = 1.5, λ = 1.064 μm)
    local wavelength=0.000001064
    local m2=1.5
    local waist=$(echo "$diameter / 4" | bc -l)
    local divergence=$(echo "$m2 * $wavelength / ($PI * $waist)" | bc -l)

    # Beam radius at range
    local ratio=$(echo "$range * $divergence / $waist" | bc -l)
    local radius_at_range=$(echo "$waist * sqrt(1 + $ratio * $ratio)" | bc -l)

    print_section "Results"
    print_success "Beam Radius at Target: $(printf "%.3f" $radius_at_range) m"
    print_success "Intensity: $(printf "%.2e" $intensity) W/m²"
    print_info "Divergence: $(printf "%.1f" $(echo "$divergence * 1000000" | bc -l)) μrad"
    print_info "Spot Size: $(printf "%.3f" $(echo "$radius_at_range * 2" | bc -l)) m"

    echo ""
}

# Validate engagement scenario
validate_engagement() {
    local target_type=${1:-mortar}
    local range=${2:-2000}
    local weather=${3:-clear}

    print_section "Engagement Validation"
    print_info "Target Type: $target_type"
    print_info "Range: ${range} m"
    print_info "Weather: $weather"

    # Set atmospheric transmission based on weather
    local transmission=0.9
    case $weather in
        clear)
            transmission=0.9
            ;;
        haze)
            transmission=0.6
            ;;
        rain)
            transmission=0.3
            ;;
        fog)
            transmission=0.1
            ;;
    esac

    print_section "Safety Checks"

    # Range check
    if (( $(echo "$range <= 5000" | bc -l) )); then
        print_success "Range Check: PASS (within 5 km limit)"
    else
        print_warning "Range Check: WARNING (exceeds 5 km)"
    fi

    # Atmospheric check
    if (( $(echo "$transmission >= 0.5" | bc -l) )); then
        print_success "Atmospheric Transmission: PASS (${transmission})"
    else
        print_warning "Atmospheric Transmission: WARNING (${transmission})"
    fi

    # Power check (assume 100 kW system)
    local power=100000
    local required_power=50000

    if (( $(echo "$power >= $required_power" | bc -l) )); then
        print_success "Power Check: PASS ($(format_power $power) available)"
    else
        print_error "Power Check: FAIL (insufficient power)"
    fi

    # Time to kill estimate
    local ttk=3
    case $target_type in
        mortar)
            ttk=3
            ;;
        rocket)
            ttk=5
            ;;
        artillery)
            ttk=8
            ;;
        uav)
            ttk=2
            ;;
    esac

    # Adjust for range and weather
    local range_factor=$(echo "$range / 2000" | bc -l)
    local weather_factor=$(echo "1 / $transmission" | bc -l)
    ttk=$(echo "$ttk * $range_factor * $weather_factor" | bc -l)

    print_section "Engagement Analysis"
    print_info "Estimated Time to Kill: $(printf "%.1f" $ttk) seconds"

    if (( $(echo "$ttk <= 10" | bc -l) )); then
        print_success "Engagement Feasibility: FEASIBLE"
        print_info "Kill Probability: $(echo "scale=0; (100 - $ttk * 5)" | bc)%"
    else
        print_warning "Engagement Feasibility: MARGINAL"
        print_info "Kill Probability: < 50%"
    fi

    echo ""
}

# Simulate atmospheric effects
simulate_atmosphere() {
    local range=${1:-5000}
    local humidity=${2:-60}
    local temp=${3:-25}

    print_section "Atmospheric Simulation"
    print_info "Range: ${range} m ($(echo "scale=1; $range / 1000" | bc) km)"
    print_info "Humidity: ${humidity}%"
    print_info "Temperature: ${temp}°C"

    # Calculate visibility-based attenuation
    # α ≈ 3.91 / V (clear: 20km, haze: 5km)
    local visibility=15000  # 15 km default

    # Adjust visibility based on humidity
    if (( humidity > 80 )); then
        visibility=5000
    elif (( humidity > 60 )); then
        visibility=10000
    fi

    local alpha=$(echo "scale=4; 3.91 / ($visibility / 1000)" | bc -l)

    # Transmission T = exp(-α × L)
    local range_km=$(echo "scale=3; $range / 1000" | bc -l)
    # exp approximation using e^x ≈ 1 + x + x²/2 for small x
    local exponent=$(echo "scale=6; -1 * $alpha * $range_km" | bc -l)
    local transmission=$(echo "scale=4; e($exponent)" | bc -l)

    print_section "Results"
    print_info "Visibility: $(echo "$visibility / 1000" | bc) km"
    print_info "Attenuation Coefficient: $(printf "%.3f" $alpha) km⁻¹"
    print_success "Atmospheric Transmission: $(printf "%.1f" $(echo "$transmission * 100" | bc -l))%"

    # Power delivery example
    local input_power=100000
    local delivered=$(echo "$input_power * $transmission" | bc -l)
    print_info "Input Power: $(format_power $input_power)"
    print_info "Delivered Power: $(format_power $delivered)"

    # Assessment
    if (( $(echo "$transmission >= 0.7" | bc -l) )); then
        print_success "Atmospheric Conditions: EXCELLENT"
    elif (( $(echo "$transmission >= 0.5" | bc -l) )); then
        print_success "Atmospheric Conditions: GOOD"
    elif (( $(echo "$transmission >= 0.3" | bc -l) )); then
        print_warning "Atmospheric Conditions: MARGINAL"
    else
        print_error "Atmospheric Conditions: POOR"
    fi

    echo ""
}

# Generate thermal management plan
thermal_plan() {
    local power=${1:-150000}
    local duty_cycle=${2:-0.5}

    print_section "Thermal Management Plan"
    print_info "Operating Power: $(format_power $power)"
    print_info "Duty Cycle: $(echo "$duty_cycle * 100" | bc -l)%"

    # Efficiency (assume 30%)
    local efficiency=0.3
    local waste_heat=$(echo "$power * (1 - $efficiency)" | bc -l)
    local avg_waste=$(echo "$waste_heat * $duty_cycle" | bc -l)

    print_section "Thermal Load"
    print_info "Wall-Plug Efficiency: $(echo "$efficiency * 100" | bc)%"
    print_info "Waste Heat (Peak): $(format_power $waste_heat)"
    print_info "Waste Heat (Average): $(format_power $avg_waste)"

    # Cooling requirement (1.5× safety margin)
    local required_cooling=$(echo "$avg_waste * 1.5" | bc -l)

    print_section "Cooling Requirements"
    print_success "Required Cooling Capacity: $(format_power $required_cooling)"

    # Coolant flow (assume water, 4.186 kJ/kg·K, ΔT=15K)
    local specific_heat=4186
    local temp_rise=15
    local flow_rate=$(echo "$avg_waste / ($specific_heat * $temp_rise)" | bc -l)
    local flow_lpm=$(echo "$flow_rate * 60" | bc -l)

    print_info "Coolant Flow Rate: $(printf "%.1f" $flow_lpm) L/min"
    print_info "Temperature Rise: ${temp_rise}°C"

    # Operating limits
    local max_continuous=$(echo "3600 / (1 - $duty_cycle + 0.01)" | bc -l)

    print_section "Operating Limits"
    print_info "Max Continuous Operation: $(echo "scale=0; $max_continuous / 60" | bc) minutes"

    if (( $(echo "$duty_cycle > 0.7" | bc -l) )); then
        print_warning "High duty cycle - recommend cool-down periods every 30 minutes"
    else
        print_success "Duty cycle within normal operating range"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-def-007 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-beam                Calculate beam parameters at target"
    echo "    --power <watts>        Laser power (default: 100000 W / 100 kW)"
    echo "    --range <meters>       Target range (default: 2000 m)"
    echo "    --diameter <meters>    Initial beam diameter (default: 0.2 m)"
    echo ""
    echo "  validate                 Validate engagement scenario"
    echo "    --target <type>        Target type: mortar, rocket, artillery, uav"
    echo "    --range <meters>       Target range (default: 2000 m)"
    echo "    --weather <condition>  Weather: clear, haze, rain, fog"
    echo ""
    echo "  simulate-atmosphere      Simulate atmospheric propagation"
    echo "    --range <meters>       Propagation distance (default: 5000 m)"
    echo "    --humidity <percent>   Relative humidity (default: 60%)"
    echo "    --temp <celsius>       Temperature (default: 25°C)"
    echo ""
    echo "  thermal-plan             Generate thermal management plan"
    echo "    --power <watts>        Operating power (default: 150000 W)"
    echo "    --duty-cycle <0-1>     Duty cycle fraction (default: 0.5)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-def-007 calc-beam --power 100000 --range 2000 --diameter 0.1"
    echo "  wia-def-007 validate --target mortar --range 2000 --weather clear"
    echo "  wia-def-007 simulate-atmosphere --range 5000 --humidity 70"
    echo "  wia-def-007 thermal-plan --power 150000 --duty-cycle 0.3"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-DEF-007 Laser Weapon Standard CLI"
    echo "Version: $VERSION"
    echo "Category: DEF (Defense & Security)"
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
    calc-beam)
        POWER=100000
        RANGE=2000
        DIAMETER=0.2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) POWER=$2; shift 2 ;;
                --range) RANGE=$2; shift 2 ;;
                --diameter) DIAMETER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_beam "$POWER" "$RANGE" "$DIAMETER"
        ;;

    validate)
        TARGET="mortar"
        RANGE=2000
        WEATHER="clear"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target) TARGET=$2; shift 2 ;;
                --range) RANGE=$2; shift 2 ;;
                --weather) WEATHER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_engagement "$TARGET" "$RANGE" "$WEATHER"
        ;;

    simulate-atmosphere)
        RANGE=5000
        HUMIDITY=60
        TEMP=25

        while [[ $# -gt 0 ]]; do
            case $1 in
                --range) RANGE=$2; shift 2 ;;
                --humidity) HUMIDITY=$2; shift 2 ;;
                --temp) TEMP=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate_atmosphere "$RANGE" "$HUMIDITY" "$TEMP"
        ;;

    thermal-plan)
        POWER=150000
        DUTY=0.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) POWER=$2; shift 2 ;;
                --duty-cycle) DUTY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        thermal_plan "$POWER" "$DUTY"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-def-007 help' for usage information"
        exit 1
        ;;
esac

exit 0
