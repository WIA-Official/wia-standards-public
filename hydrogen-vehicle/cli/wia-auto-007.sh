#!/bin/bash

################################################################################
# WIA-AUTO-007: Hydrogen Vehicle CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to hydrogen vehicle calculations
# including fuel cell efficiency, range estimation, tank validation, and
# refueling optimization.
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
LHV_H2=120          # MJ/kg
HHV_H2=142          # MJ/kg
H35_PRESSURE=350    # bar
H70_PRESSURE=700    # bar

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🚗 WIA-AUTO-007: Hydrogen Vehicle CLI Tool            ║"
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

format_energy() {
    local energy=$1

    if (( $(echo "$energy < 1" | bc -l) )); then
        printf "%.3f MJ" "$energy"
    elif (( $(echo "$energy < 1000" | bc -l) )); then
        printf "%.2f MJ" "$energy"
    else
        printf "%.2f GJ" "$(echo "$energy / 1000" | bc -l)"
    fi
}

# Calculate fuel cell efficiency
calc_efficiency() {
    local power=${1:-100}
    local h2_flow=${2:-0.9}
    local voltage=${3:-400}
    local current=${4:-250}

    print_section "Fuel Cell Efficiency Calculation"
    print_info "Power Output: $power kW"
    print_info "H2 Flow Rate: $h2_flow kg/h"
    print_info "Stack Voltage: $voltage V"
    print_info "Stack Current: $current A"

    # Calculate hydrogen energy input (kW)
    local h2_energy_rate=$(echo "$h2_flow * $LHV_H2 / 3.6" | bc -l)
    print_info "H2 Energy Input: $(printf '%.2f' $h2_energy_rate) kW"

    # Calculate efficiency
    local efficiency=$(echo "scale=4; $power / $h2_energy_rate" | bc -l)
    local efficiency_pct=$(echo "$efficiency * 100" | bc -l)

    # Calculate stack power
    local stack_power=$(echo "scale=2; $voltage * $current / 1000" | bc -l)

    # Calculate current density (assume 500 cm² cell area)
    local current_density=$(echo "scale=3; $current / 500" | bc -l)

    # Calculate power density (assume 4 kW/L)
    local power_density=$(echo "scale=2; $power / ($power / 4)" | bc -l)

    print_section "Results"
    print_success "Fuel Cell Efficiency: $(printf '%.1f%%' $efficiency_pct)"
    print_info "Stack Power: $(printf '%.2f' $stack_power) kW"
    print_info "Current Density: $(printf '%.3f' $current_density) A/cm²"
    print_info "Power Density: $(printf '%.2f' $power_density) kW/L"

    # Feasibility assessment
    if (( $(echo "$efficiency >= 0.55" | bc -l) )); then
        print_success "Assessment: OPTIMAL (≥55%)"
    elif (( $(echo "$efficiency >= 0.45" | bc -l) )); then
        print_success "Assessment: ACCEPTABLE (45-55%)"
    elif (( $(echo "$efficiency >= 0.30" | bc -l) )); then
        print_warning "Assessment: SUBOPTIMAL (30-45%)"
    else
        print_error "Assessment: CRITICAL (<30%)"
    fi

    echo ""
}

# Calculate vehicle range
calc_range() {
    local h2_capacity=${1:-5.6}
    local fc_efficiency=${2:-0.60}
    local sys_efficiency=${3:-0.90}
    local consumption=${4:-0.95}

    print_section "Vehicle Range Calculation"
    print_info "H2 Capacity: $h2_capacity kg"
    print_info "FC Efficiency: $(echo "$fc_efficiency * 100" | bc -l)%"
    print_info "System Efficiency: $(echo "$sys_efficiency * 100" | bc -l)%"
    print_info "Energy Consumption: $consumption MJ/km"

    # Calculate total hydrogen energy
    local h2_energy=$(echo "$h2_capacity * $LHV_H2" | bc -l)
    print_info "Total H2 Energy: $(format_energy $h2_energy)"

    # Calculate fuel cell energy
    local fc_energy=$(echo "$h2_energy * $fc_efficiency" | bc -l)
    print_info "FC Output Energy: $(format_energy $fc_energy)"

    # Calculate usable energy
    local usable_energy=$(echo "$fc_energy * $sys_efficiency" | bc -l)
    print_info "Usable Energy: $(format_energy $usable_energy)"

    # Calculate range
    local range=$(echo "$usable_energy / $consumption" | bc -l)

    # Apply 10% buffer
    local range_buffer=$(echo "$range * 0.10" | bc -l)
    local range_actual=$(echo "$range - $range_buffer" | bc -l)

    print_section "Range Results"
    print_success "Estimated Range: $(printf '%.0f' $range_actual) km"
    print_info "Range Buffer (10%): $(printf '%.0f' $range_buffer) km"
    print_info "Total Theoretical Range: $(printf '%.0f' $range) km"

    # Energy in kWh
    local energy_kwh=$(echo "$usable_energy / 3.6" | bc -l)
    print_info "Usable Energy: $(printf '%.1f' $energy_kwh) kWh"

    # Comparison to known vehicles
    print_section "Comparison"
    if (( $(echo "$range_actual >= 400" | bc -l) )); then
        print_success "Range comparable to: Toyota Mirai (650 km)"
    elif (( $(echo "$range_actual >= 350" | bc -l) )); then
        print_success "Range comparable to: Hyundai NEXO (600 km)"
    elif (( $(echo "$range_actual >= 300" | bc -l) )); then
        print_info "Range comparable to: Honda Clarity (589 km)"
    else
        print_warning "Range below typical commercial FCEVs"
    fi

    echo ""
}

# Validate tank pressure
validate_tank() {
    local pressure=${1:-700}
    local temperature=${2:-20}
    local tank_type=${3:-"Type IV"}
    local standard=${4:-"H70"}

    print_section "Tank Pressure Validation"
    print_info "Current Pressure: $pressure bar"
    print_info "Current Temperature: $temperature °C"
    print_info "Tank Type: $tank_type"
    print_info "Standard: $standard"

    # Determine working pressure
    local working_pressure
    if [ "$standard" == "H70" ]; then
        working_pressure=$H70_PRESSURE
    else
        working_pressure=$H35_PRESSURE
    fi

    print_info "Working Pressure: $working_pressure bar"

    # Safety checks
    print_section "Safety Checks"

    # Pressure check
    if (( $(echo "$pressure > $working_pressure" | bc -l) )); then
        print_error "Pressure Check: FAIL (Exceeds working pressure)"
    elif (( $(echo "$pressure > $working_pressure * 0.95" | bc -l) )); then
        print_warning "Pressure Check: WARNING (Near maximum)"
    else
        print_success "Pressure Check: PASS"
    fi

    # Temperature check
    if (( $(echo "$temperature > 85" | bc -l) )); then
        print_error "Temperature Check: FAIL (Exceeds 85°C limit)"
    elif (( $(echo "$temperature > 75" | bc -l) )); then
        print_warning "Temperature Check: WARNING (Elevated temperature)"
    elif (( $(echo "$temperature < -40" | bc -l) )); then
        print_error "Temperature Check: FAIL (Below -40°C limit)"
    else
        print_success "Temperature Check: PASS"
    fi

    # Safety margin
    local safety_margin=$(echo "scale=4; ($working_pressure - $pressure) / $working_pressure" | bc -l)
    local margin_pct=$(echo "$safety_margin * 100" | bc -l)

    print_info "Safety Margin: $(printf '%.1f%%' $margin_pct)"

    if (( $(echo "$safety_margin < 0.10" | bc -l) )); then
        print_warning "Safety Margin: LOW (<10%)"
    else
        print_success "Safety Margin: ADEQUATE (≥10%)"
    fi

    # Test pressure
    local test_pressure=$(echo "$working_pressure * 1.5" | bc -l)
    print_info "Test Pressure (1.5×): $(printf '%.0f' $test_pressure) bar"

    print_section "Validation Result"
    if (( $(echo "$pressure <= $working_pressure && $temperature <= 85 && $temperature >= -40" | bc -l) )); then
        print_success "Tank state is VALID and SAFE"
    else
        print_error "Tank state is INVALID - safety concern detected"
    fi

    echo ""
}

# Optimize refueling
optimize_refuel() {
    local target_pressure=${1:-700}
    local ambient_temp=${2:-15}
    local current_pressure=${3:-150}
    local current_temp=${4:-20}
    local tank_volume=${5:-122.4}
    local standard=${6:-"H70"}

    print_section "Refueling Optimization"
    print_info "Target Pressure: $target_pressure bar"
    print_info "Ambient Temperature: $ambient_temp °C"
    print_info "Current Pressure: $current_pressure bar"
    print_info "Current Temperature: $current_temp °C"
    print_info "Tank Volume: $tank_volume liters"
    print_info "Standard: $standard"

    # Determine pre-cooling temperature
    local precool_temp
    if [ "$standard" == "H70" ]; then
        # H70: -40 to -33°C
        precool_temp=$(echo "scale=1; -40 + ($ambient_temp + 10) / 5" | bc -l)
        if (( $(echo "$precool_temp < -40" | bc -l) )); then
            precool_temp=-40
        elif (( $(echo "$precool_temp > -33" | bc -l) )); then
            precool_temp=-33
        fi
    else
        # H35: -20 to -10°C
        precool_temp=$(echo "scale=1; -20 + ($ambient_temp + 5) / 3" | bc -l)
        if (( $(echo "$precool_temp < -20" | bc -l) )); then
            precool_temp=-20
        elif (( $(echo "$precool_temp > -10" | bc -l) )); then
            precool_temp=-10
        fi
    fi

    # Calculate hydrogen mass
    # Simplified: m ≈ P × V / (R × T), where R×T ≈ 2400 for this case
    local current_mass=$(echo "scale=3; $current_pressure * $tank_volume / 2400" | bc -l)
    local target_mass=$(echo "scale=3; $target_pressure * $tank_volume / 2400" | bc -l)
    local h2_mass=$(echo "scale=3; $target_mass - $current_mass" | bc -l)

    # Flow rate (kg/min)
    local flow_rate=0.060  # 60 g/s in kg/s
    local flow_rate_min=$(echo "$flow_rate * 60" | bc -l)

    # Estimated time
    local est_time=$(echo "scale=2; $h2_mass / $flow_rate" | bc -l)
    local est_time_min=$(echo "scale=2; $est_time / 60" | bc -l)

    # Pressure ramp rate
    local pressure_diff=$(echo "$target_pressure - $current_pressure" | bc -l)
    local ramp_rate=$(echo "scale=2; $pressure_diff / $est_time" | bc -l)

    # Estimate final temperature
    local temp_rise=$(echo "scale=1; $pressure_diff / 10 + $ambient_temp / 2" | bc -l)
    if (( $(echo "$temp_rise > 85" | bc -l) )); then
        temp_rise=85
    fi
    local final_temp=$(echo "$current_temp + $temp_rise" | bc -l)

    print_section "Refueling Plan"
    print_success "Pre-cooling Temperature: $(printf '%.1f' $precool_temp) °C"
    print_info "Flow Rate: $(printf '%.2f' $flow_rate) kg/s ($(printf '%.2f' $flow_rate_min) kg/min)"
    print_info "H2 Mass to Dispense: $(printf '%.2f' $h2_mass) kg"
    print_success "Estimated Fill Time: $(printf '%.1f' $est_time_min) minutes"
    print_info "Pressure Ramp Rate: $(printf '%.2f' $ramp_rate) bar/s"
    print_info "Estimated Final Temperature: $(printf '%.1f' $final_temp) °C"

    print_section "Safety Checks"

    # Pre-cooling check
    if [ "$standard" == "H70" ]; then
        if (( $(echo "$precool_temp >= -40 && $precool_temp <= -33" | bc -l) )); then
            print_success "Pre-cooling: PASS (H70 range: -40 to -33°C)"
        else
            print_error "Pre-cooling: FAIL (Outside H70 range)"
        fi
    else
        if (( $(echo "$precool_temp >= -20 && $precool_temp <= -10" | bc -l) )); then
            print_success "Pre-cooling: PASS (H35 range: -20 to -10°C)"
        else
            print_error "Pre-cooling: FAIL (Outside H35 range)"
        fi
    fi

    # Final temperature check
    if (( $(echo "$final_temp <= 85" | bc -l) )); then
        print_success "Final Temperature: PASS (≤85°C)"
    else
        print_error "Final Temperature: FAIL (>85°C) - Reduce flow rate"
    fi

    # Ramp rate check
    if (( $(echo "$ramp_rate <= 5.0" | bc -l) )); then
        print_success "Pressure Ramp Rate: PASS (≤5.0 bar/s)"
    else
        print_warning "Pressure Ramp Rate: WARNING (>5.0 bar/s)"
    fi

    # Target pressure check
    local max_pressure
    if [ "$standard" == "H70" ]; then
        max_pressure=700
    else
        max_pressure=350
    fi

    if (( $(echo "$target_pressure <= $max_pressure" | bc -l) )); then
        print_success "Target Pressure: PASS (≤$max_pressure bar for $standard)"
    else
        print_error "Target Pressure: FAIL (Exceeds $max_pressure bar for $standard)"
    fi

    print_section "Recommendation"
    if (( $(echo "$final_temp <= 85 && $ramp_rate <= 5.0 && $target_pressure <= $max_pressure" | bc -l) )); then
        print_success "Refueling plan is SAFE to execute"
        print_info "Protocol: SAE J2601"
    else
        print_error "Refueling plan needs adjustment"
        print_info "Suggested: Reduce flow rate or improve pre-cooling"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-007 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-efficiency          Calculate fuel cell efficiency"
    echo "    --power <kW>           Power output (default: 100 kW)"
    echo "    --h2-flow <kg/h>       H2 flow rate (default: 0.9 kg/h)"
    echo "    --voltage <V>          Stack voltage (default: 400 V)"
    echo "    --current <A>          Stack current (default: 250 A)"
    echo ""
    echo "  calc-range               Calculate vehicle range"
    echo "    --h2-capacity <kg>     Hydrogen capacity (default: 5.6 kg)"
    echo "    --fc-eff <0-1>         Fuel cell efficiency (default: 0.60)"
    echo "    --sys-eff <0-1>        System efficiency (default: 0.90)"
    echo "    --consumption <MJ/km>  Energy consumption (default: 0.95 MJ/km)"
    echo ""
    echo "  validate-tank            Validate tank pressure and temperature"
    echo "    --pressure <bar>       Current pressure (default: 700 bar)"
    echo "    --temperature <°C>     Current temperature (default: 20°C)"
    echo "    --tank-type <type>     Tank type (default: Type IV)"
    echo "    --standard <std>       H35 or H70 (default: H70)"
    echo ""
    echo "  optimize-refuel          Optimize refueling parameters"
    echo "    --target-pressure <bar>  Target pressure (default: 700 bar)"
    echo "    --ambient-temp <°C>      Ambient temperature (default: 15°C)"
    echo "    --current-pressure <bar> Current pressure (default: 150 bar)"
    echo "    --current-temp <°C>      Current temperature (default: 20°C)"
    echo "    --tank-volume <L>        Tank volume (default: 122.4 L)"
    echo "    --standard <std>         H35 or H70 (default: H70)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-007 calc-efficiency --power 100 --h2-flow 0.8"
    echo "  wia-auto-007 calc-range --h2-capacity 5.6 --fc-eff 0.60"
    echo "  wia-auto-007 validate-tank --pressure 700 --temperature 20"
    echo "  wia-auto-007 optimize-refuel --target-pressure 700 --ambient-temp 15"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-007 CLI Tool"
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
    calc-efficiency)
        POWER=100
        H2_FLOW=0.9
        VOLTAGE=400
        CURRENT=250

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) POWER=$2; shift 2 ;;
                --h2-flow) H2_FLOW=$2; shift 2 ;;
                --voltage) VOLTAGE=$2; shift 2 ;;
                --current) CURRENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_efficiency "$POWER" "$H2_FLOW" "$VOLTAGE" "$CURRENT"
        ;;

    calc-range)
        H2_CAPACITY=5.6
        FC_EFF=0.60
        SYS_EFF=0.90
        CONSUMPTION=0.95

        while [[ $# -gt 0 ]]; do
            case $1 in
                --h2-capacity) H2_CAPACITY=$2; shift 2 ;;
                --fc-eff) FC_EFF=$2; shift 2 ;;
                --sys-eff) SYS_EFF=$2; shift 2 ;;
                --consumption) CONSUMPTION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_range "$H2_CAPACITY" "$FC_EFF" "$SYS_EFF" "$CONSUMPTION"
        ;;

    validate-tank)
        PRESSURE=700
        TEMPERATURE=20
        TANK_TYPE="Type IV"
        STANDARD="H70"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --pressure) PRESSURE=$2; shift 2 ;;
                --temperature) TEMPERATURE=$2; shift 2 ;;
                --tank-type) TANK_TYPE=$2; shift 2 ;;
                --standard) STANDARD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_tank "$PRESSURE" "$TEMPERATURE" "$TANK_TYPE" "$STANDARD"
        ;;

    optimize-refuel)
        TARGET_PRESSURE=700
        AMBIENT_TEMP=15
        CURRENT_PRESSURE=150
        CURRENT_TEMP=20
        TANK_VOLUME=122.4
        STANDARD="H70"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target-pressure) TARGET_PRESSURE=$2; shift 2 ;;
                --ambient-temp) AMBIENT_TEMP=$2; shift 2 ;;
                --current-pressure) CURRENT_PRESSURE=$2; shift 2 ;;
                --current-temp) CURRENT_TEMP=$2; shift 2 ;;
                --tank-volume) TANK_VOLUME=$2; shift 2 ;;
                --standard) STANDARD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        optimize_refuel "$TARGET_PRESSURE" "$AMBIENT_TEMP" "$CURRENT_PRESSURE" "$CURRENT_TEMP" "$TANK_VOLUME" "$STANDARD"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-007 help' for usage information"
        exit 1
        ;;
esac

exit 0
