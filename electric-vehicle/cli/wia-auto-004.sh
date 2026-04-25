#!/bin/bash

################################################################################
# WIA-AUTO-004: Electric Vehicle Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to electric vehicle calculations
# including range estimation, energy consumption, charging time, and regenerative
# braking analysis.
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
GRAVITY=9.81
AIR_DENSITY=1.225
KMH_TO_MS=0.277778

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🔋 WIA-AUTO-004: Electric Vehicle CLI               ║"
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

# Calculate range
calc_range() {
    local battery=${1:-75}
    local soc=${2:-80}
    local consumption=${3:-0.18}
    local dod=${4:-0.9}

    print_section "Range Calculation"
    print_info "Battery Capacity: $battery kWh"
    print_info "State of Charge: $soc %"
    print_info "Energy Consumption: $consumption kWh/km"
    print_info "Usable DoD: $dod"

    # Available energy = Capacity × SoC × DoD
    local available=$(echo "scale=2; $battery * $soc / 100 * $dod" | bc -l)
    print_info "Available Energy: ${available} kWh"

    # Range = Available Energy / Consumption
    local range=$(echo "scale=0; $available / $consumption" | bc -l)

    print_section "Results"
    print_success "Estimated Range: $range km"

    # Min/max with ±15% uncertainty
    local range_min=$(echo "scale=0; $range * 0.85" | bc -l)
    local range_max=$(echo "scale=0; $range * 1.15" | bc -l)
    print_info "Range (min): $range_min km"
    print_info "Range (max): $range_max km"

    echo ""
}

# Calculate energy consumption
calc_energy() {
    local distance=${1:-100}
    local speed=${2:-100}
    local mass=${3:-1800}
    local cd=${4:-0.24}
    local area=${5:-2.3}
    local crr=${6:-0.008}

    print_section "Energy Consumption Calculation"
    print_info "Distance: $distance km"
    print_info "Average Speed: $speed km/h"
    print_info "Vehicle Mass: $mass kg"
    print_info "Drag Coefficient (Cd): $cd"
    print_info "Frontal Area: $area m²"
    print_info "Rolling Resistance: $crr"

    # Convert speed to m/s
    local speed_ms=$(echo "scale=3; $speed * $KMH_TO_MS" | bc -l)

    # Time in hours
    local time_h=$(echo "scale=4; $distance / $speed" | bc -l)

    # Aerodynamic drag force: F = 0.5 × ρ × Cd × A × v²
    local f_aero=$(echo "scale=3; 0.5 * $AIR_DENSITY * $cd * $area * $speed_ms * $speed_ms" | bc -l)

    # Rolling resistance force: F = Crr × m × g
    local f_roll=$(echo "scale=3; $crr * $mass * $GRAVITY" | bc -l)

    # Total force
    local f_total=$(echo "scale=3; $f_aero + $f_roll" | bc -l)

    # Power at wheels (W) = F × v
    local power_wheel=$(echo "scale=0; $f_total * $speed_ms" | bc -l)

    print_section "Forces and Power"
    print_info "Aerodynamic Drag: ${f_aero} N"
    print_info "Rolling Resistance: ${f_roll} N"
    print_info "Total Resistance: ${f_total} N"
    print_info "Power at Wheels: $(echo "scale=1; $power_wheel / 1000" | bc -l) kW"

    # Energy at wheels (kWh) = Power (kW) × Time (h)
    local energy_wheel=$(echo "scale=3; $power_wheel / 1000 * $time_h" | bc -l)

    # Account for drivetrain efficiency (assume 90%)
    local efficiency=0.90
    local energy_battery=$(echo "scale=3; $energy_wheel / $efficiency" | bc -l)

    # Add accessories (0.3 kW baseline)
    local accessories=$(echo "scale=3; 0.3 * $time_h" | bc -l)
    local total_energy=$(echo "scale=3; $energy_battery + $accessories" | bc -l)

    print_section "Results"
    print_success "Total Energy: ${total_energy} kWh"

    local consumption_100=$(echo "scale=2; $total_energy / $distance * 100" | bc -l)
    print_success "Consumption: ${consumption_100} kWh/100km"

    echo ""
}

# Calculate charging time
calc_charging() {
    local battery=${1:-75}
    local from_soc=${2:-20}
    local to_soc=${3:-80}
    local power=${4:-150}

    print_section "Charging Time Calculation"
    print_info "Battery Capacity: $battery kWh"
    print_info "Charging: $from_soc% → $to_soc%"
    print_info "Charger Power: $power kW"

    # Energy to deliver
    local energy_needed=$(echo "scale=2; $battery * ($to_soc - $from_soc) / 100" | bc -l)
    print_info "Energy to Deliver: ${energy_needed} kWh"

    # Charging efficiency (95% for DC fast)
    local efficiency=0.95
    local energy_from_charger=$(echo "scale=2; $energy_needed / $efficiency" | bc -l)

    # Simple calculation: time = energy / power
    local time_h=$(echo "scale=3; $energy_from_charger / $power" | bc -l)
    local time_min=$(echo "scale=0; $time_h * 60" | bc -l)

    print_section "Results"
    print_success "Charging Time: $time_min minutes ($(echo "scale=1; $time_h" | bc -l) hours)"

    local avg_power=$(echo "scale=1; $energy_from_charger / $time_h" | bc -l)
    print_info "Average Power: ${avg_power} kW"

    # Warnings
    if (( $(echo "$to_soc > 80" | bc -l) )); then
        print_warning "Charging slows significantly above 80% SoC"
    fi

    echo ""
}

# Simulate drive profile
simulate_drive() {
    local profile=${1:-city}
    local distance=${2:-50}
    local battery=${3:-60}

    print_section "Drive Profile Simulation"
    print_info "Profile: $profile"
    print_info "Distance: $distance km"
    print_info "Battery: $battery kWh"

    # Different consumption for different profiles
    local consumption
    case "$profile" in
        city|urban)
            consumption=0.16
            print_info "Urban Profile: Frequent stops, moderate speed"
            ;;
        highway)
            consumption=0.20
            print_info "Highway Profile: High speed, aerodynamic drag dominant"
            ;;
        mixed)
            consumption=0.18
            print_info "Mixed Profile: Combination of urban and highway"
            ;;
        sport)
            consumption=0.24
            print_info "Sport Profile: Aggressive driving, high acceleration"
            ;;
        eco)
            consumption=0.14
            print_info "Eco Profile: Efficient driving, smooth acceleration"
            ;;
        *)
            consumption=0.18
            print_warning "Unknown profile, using mixed"
            ;;
    esac

    # Energy consumed
    local energy=$(echo "scale=2; $distance * $consumption" | bc -l)

    # SoC change
    local soc_delta=$(echo "scale=1; $energy / $battery * 100" | bc -l)

    # Assume 15% regen recovery for city, 5% for highway
    local regen
    if [ "$profile" = "city" ] || [ "$profile" = "urban" ]; then
        regen=0.15
    else
        regen=0.05
    fi

    local energy_recovered=$(echo "scale=2; $energy * $regen" | bc -l)
    local net_energy=$(echo "scale=2; $energy - $energy_recovered" | bc -l)
    local net_soc_delta=$(echo "scale=1; $net_energy / $battery * 100" | bc -l)

    print_section "Results"
    print_success "Energy Consumed: ${energy} kWh"
    print_success "Energy Recovered (Regen): ${energy_recovered} kWh"
    print_success "Net Energy: ${net_energy} kWh"
    print_info "SoC Change: -${net_soc_delta}%"

    local consumption_100=$(echo "scale=2; $net_energy / $distance * 100" | bc -l)
    print_success "Effective Consumption: ${consumption_100} kWh/100km"

    echo ""
}

# Calculate regenerative braking
calc_regen() {
    local mass=${1:-1800}
    local speed=${2:-100}

    print_section "Regenerative Braking Calculation"
    print_info "Vehicle Mass: $mass kg"
    print_info "Initial Speed: $speed km/h"
    print_info "Final Speed: 0 km/h (full stop)"

    # Convert to m/s
    local v_ms=$(echo "scale=3; $speed * $KMH_TO_MS" | bc -l)

    # Kinetic energy: KE = 0.5 × m × v²
    local ke_j=$(echo "scale=0; 0.5 * $mass * $v_ms * $v_ms" | bc -l)
    local ke_kwh=$(echo "scale=4; $ke_j / 3600000" | bc -l)

    print_info "Kinetic Energy: ${ke_j} J (${ke_kwh} kWh)"

    # Typical regen efficiency: 70% (motor 88% × inverter 96% × battery 96%)
    local efficiency=0.70
    local recovered_kwh=$(echo "scale=4; $ke_kwh * $efficiency" | bc -l)

    print_section "Results"
    print_success "Recoverable Energy: ${recovered_kwh} kWh"
    print_info "Recovery Efficiency: 70%"

    # Range extension (assume 18 kWh/100km consumption)
    local range_ext=$(echo "scale=2; $recovered_kwh / 0.18" | bc -l)
    print_info "Range Extension: ${range_ext} km"

    # Average power during 5-second braking
    local braking_time=5
    local avg_power=$(echo "scale=1; $recovered_kwh * 3600 / $braking_time" | bc -l)
    print_info "Average Regen Power (5s braking): ${avg_power} kW"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-004 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-range               Calculate vehicle range"
    echo "    --battery <kWh>        Battery capacity (default: 75 kWh)"
    echo "    --soc <percent>        State of Charge (default: 80%)"
    echo "    --consumption <kWh/km> Energy consumption (default: 0.18)"
    echo ""
    echo "  calc-energy              Calculate energy consumption"
    echo "    --distance <km>        Distance to travel (default: 100 km)"
    echo "    --speed <km/h>         Average speed (default: 100 km/h)"
    echo "    --mass <kg>            Vehicle mass (default: 1800 kg)"
    echo "    --cd <value>           Drag coefficient (default: 0.24)"
    echo ""
    echo "  calc-charging            Calculate charging time"
    echo "    --battery <kWh>        Battery capacity (default: 75 kWh)"
    echo "    --from <percent>       Starting SoC (default: 20%)"
    echo "    --to <percent>         Target SoC (default: 80%)"
    echo "    --power <kW>           Charger power (default: 150 kW)"
    echo ""
    echo "  simulate                 Simulate drive profile"
    echo "    --profile <type>       city, highway, mixed, sport, eco (default: city)"
    echo "    --distance <km>        Distance (default: 50 km)"
    echo "    --battery <kWh>        Battery capacity (default: 60 kWh)"
    echo ""
    echo "  calc-regen               Calculate regenerative braking"
    echo "    --mass <kg>            Vehicle mass (default: 1800 kg)"
    echo "    --speed <km/h>         Initial speed (default: 100 km/h)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-004 calc-range --battery 75 --soc 80 --consumption 0.18"
    echo "  wia-auto-004 calc-energy --distance 100 --speed 120 --mass 1800"
    echo "  wia-auto-004 calc-charging --battery 75 --power 150 --from 20 --to 80"
    echo "  wia-auto-004 simulate --profile highway --distance 100 --battery 75"
    echo "  wia-auto-004 calc-regen --mass 1800 --speed 100"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-004 CLI Tool"
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
    calc-range)
        BATTERY=75
        SOC=80
        CONSUMPTION=0.18
        DOD=0.9

        while [[ $# -gt 0 ]]; do
            case $1 in
                --battery) BATTERY=$2; shift 2 ;;
                --soc) SOC=$2; shift 2 ;;
                --consumption) CONSUMPTION=$2; shift 2 ;;
                --dod) DOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_range "$BATTERY" "$SOC" "$CONSUMPTION" "$DOD"
        ;;

    calc-energy)
        DISTANCE=100
        SPEED=100
        MASS=1800
        CD=0.24
        AREA=2.3
        CRR=0.008

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) DISTANCE=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --mass) MASS=$2; shift 2 ;;
                --cd) CD=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                --crr) CRR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_energy "$DISTANCE" "$SPEED" "$MASS" "$CD" "$AREA" "$CRR"
        ;;

    calc-charging)
        BATTERY=75
        FROM=20
        TO=80
        POWER=150

        while [[ $# -gt 0 ]]; do
            case $1 in
                --battery) BATTERY=$2; shift 2 ;;
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_charging "$BATTERY" "$FROM" "$TO" "$POWER"
        ;;

    simulate)
        PROFILE="city"
        DISTANCE=50
        BATTERY=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --profile) PROFILE=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                --battery) BATTERY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate_drive "$PROFILE" "$DISTANCE" "$BATTERY"
        ;;

    calc-regen)
        MASS=1800
        SPEED=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mass) MASS=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_regen "$MASS" "$SPEED"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-004 help' for usage information"
        exit 1
        ;;
esac

exit 0
