#!/bin/bash

################################################################################
# WIA-AUTO-019: Hyperloop Transportation CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Mobility Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to hyperloop calculations
# including drag force, levitation, linear motors, and journey simulation.
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
ATMOSPHERIC_PRESSURE=101325
TARGET_PRESSURE=100
TUBE_DIAMETER=3.3
POD_DIAMETER=2.7
DRAG_COEFFICIENT=0.15
GRAVITY=9.81
SPEED_OF_LIGHT=340
MAGNETIC_PERMEABILITY=0.0000012566370614  # 4π × 10⁻⁷
GAS_CONSTANT=8.314
MOLAR_MASS_AIR=0.029
STANDARD_TEMP=293

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     🚄 WIA-AUTO-019: Hyperloop Transportation CLI             ║"
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
        printf "%.0f Wh" "$(echo "$energy * 1000" | bc -l)"
    elif (( $(echo "$energy < 1000" | bc -l) )); then
        printf "%.2f kWh" "$energy"
    elif (( $(echo "$energy < 1000000" | bc -l) )); then
        printf "%.2f MWh" "$(echo "$energy / 1000" | bc -l)"
    else
        printf "%.2f GWh" "$(echo "$energy / 1000000" | bc -l)"
    fi
}

# Calculate drag force
calc_drag() {
    local pressure=${1:-100}
    local velocity=${2:-300}
    local area=${3:-5.72}
    local drag_coef=${4:-0.15}

    print_section "Drag Force Calculation"
    print_info "Pressure: $pressure Pa ($(echo "scale=4; $pressure / $ATMOSPHERIC_PRESSURE * 100" | bc)% atmospheric)"
    print_info "Velocity: $velocity m/s ($(echo "scale=0; $velocity * 3.6" | bc) km/h)"
    print_info "Cross-sectional Area: $area m²"
    print_info "Drag Coefficient: $drag_coef"

    # Calculate air density: ρ = (P × M) / (R × T)
    local air_density=$(echo "scale=8; ($pressure * $MOLAR_MASS_AIR) / ($GAS_CONSTANT * $STANDARD_TEMP)" | bc -l)
    print_info "Air Density: $air_density kg/m³"

    # Calculate drag force: F = 0.5 × ρ × v² × Cd × A
    local v_squared=$(echo "$velocity * $velocity" | bc -l)
    local drag_force=$(echo "scale=2; 0.5 * $air_density * $v_squared * $drag_coef * $area" | bc -l)

    # Calculate power: P = F × v
    local power=$(echo "scale=2; $drag_force * $velocity" | bc -l)
    local power_kw=$(echo "scale=2; $power / 1000" | bc -l)

    # Calculate energy per 100 km
    local distance_100km=100000
    local energy_j=$(echo "$drag_force * $distance_100km" | bc -l)
    local energy_kwh=$(echo "scale=2; $energy_j / 3600000" | bc -l)

    print_section "Results"
    print_success "Drag Force: $drag_force N"
    print_success "Power Required: $power_kw kW"
    print_success "Energy per 100 km: $energy_kwh kWh"

    # Comparison to atmospheric pressure
    local atm_density=1.225
    local atm_force=$(echo "scale=2; 0.5 * $atm_density * $v_squared * $drag_coef * $area" | bc -l)
    local reduction=$(echo "scale=0; ($atm_force - $drag_force) / $atm_force * 100" | bc -l)

    print_section "Atmospheric Comparison"
    print_info "Drag at Atmospheric Pressure: $atm_force N"
    print_info "Drag Reduction: ${reduction}%"

    echo ""
}

# Calculate levitation force
calc_levitation() {
    local mass=${1:-15000}
    local field_strength=${2:-1.2}
    local area=${3:-0.4}
    local num_magnets=${4:-8}

    print_section "Magnetic Levitation Calculation"
    print_info "Pod Mass: $mass kg"
    print_info "Magnetic Field Strength: $field_strength T"
    print_info "Effective Area per Magnet: $area m²"
    print_info "Number of Magnets: $num_magnets"

    # Required force: F = m × g
    local required_force=$(echo "$mass * $GRAVITY" | bc -l)
    print_info "Required Force: $required_force N"

    # Levitation force per magnet: F = (B² × A) / (2μ₀)
    local b_squared=$(echo "$field_strength * $field_strength" | bc -l)
    local force_per_magnet=$(echo "scale=2; ($b_squared * $area) / (2 * $MAGNETIC_PERMEABILITY)" | bc -l)

    # Total force
    local total_force=$(echo "$force_per_magnet * $num_magnets" | bc -l)

    # Safety margin
    local safety_margin=$(echo "scale=2; $total_force / $required_force" | bc -l)

    # Power consumption (typical: 7.5 kW per magnet)
    local power_per_magnet=7.5
    local total_power=$(echo "$power_per_magnet * $num_magnets" | bc -l)

    print_section "Results"
    print_success "Force per Magnet: $force_per_magnet N"
    print_success "Total Levitation Force: $total_force N"
    print_success "Safety Margin: ${safety_margin}× required"
    print_success "Power Consumption: $total_power kW"

    # Stability check
    if (( $(echo "$safety_margin >= 1.5" | bc -l) )); then
        print_success "Stability: STABLE (safety margin ≥ 1.5)"
    elif (( $(echo "$safety_margin >= 1.0" | bc -l) )); then
        print_warning "Stability: MARGINAL (safety margin < 1.5)"
    else
        print_error "Stability: UNSTABLE (insufficient force)"
    fi

    echo ""
}

# Calculate energy consumption
calc_energy() {
    local distance=${1:-600000}
    local mass=${2:-15000}
    local speed=${3:-300}
    local passengers=${4:-28}

    print_section "Energy Consumption Calculation"
    print_info "Distance: $(echo "scale=0; $distance / 1000" | bc) km"
    print_info "Pod Mass: $mass kg"
    print_info "Max Speed: $speed m/s ($(echo "scale=0; $speed * 3.6" | bc) km/h)"
    print_info "Passengers: $passengers"

    # Total mass (pod + passengers at 75 kg each)
    local total_mass=$(echo "$mass + ($passengers * 75)" | bc -l)
    print_info "Total Mass: $total_mass kg"

    # 1. Acceleration energy: E = 0.5 × m × v²
    local kinetic=$(echo "0.5 * $total_mass * $speed * $speed" | bc -l)
    local accel_energy=$(echo "scale=2; $kinetic / (3600000 * 0.85)" | bc -l)  # kWh with motor efficiency

    # 2. Cruising energy (drag)
    # Air density at 100 Pa
    local rho=$(echo "scale=8; (100 * 0.029) / (8.314 * 293)" | bc -l)
    local area=5.72
    local drag=$(echo "scale=2; 0.5 * $rho * $speed * $speed * 0.15 * $area" | bc -l)
    local cruise_energy=$(echo "scale=2; ($drag * $distance) / 3600000" | bc -l)

    # 3. Levitation energy
    local travel_time=$(echo "$distance / $speed" | bc -l)
    local lev_power=75
    local lev_energy=$(echo "scale=2; ($lev_power * $travel_time) / 3600" | bc -l)

    # 4. Auxiliary systems
    local aux_power=25
    local aux_energy=$(echo "scale=2; ($aux_power * $travel_time) / 3600" | bc -l)

    # 5. Regenerative braking (75% recovery)
    local regen_energy=$(echo "scale=2; $accel_energy * -0.75" | bc -l)

    # Total energy
    local total=$(echo "$accel_energy + $cruise_energy + $lev_energy + $aux_energy + $regen_energy" | bc -l)
    local per_passenger=$(echo "scale=2; $total / $passengers" | bc -l)

    print_section "Energy Breakdown"
    print_info "Acceleration: $(format_energy $accel_energy)"
    print_info "Cruising (drag): $(format_energy $cruise_energy)"
    print_info "Levitation: $(format_energy $lev_energy)"
    print_info "Auxiliary: $(format_energy $aux_energy)"
    print_info "Regenerative Braking: $(format_energy $regen_energy) (recovery)"

    print_section "Results"
    print_success "Total Energy: $(format_energy $total)"
    print_success "Energy per Passenger: $(format_energy $per_passenger)"

    # Travel time
    local time_minutes=$(echo "scale=0; $travel_time / 60" | bc -l)
    print_success "Travel Time: $time_minutes minutes"

    # Comparisons
    print_section "Comparison to Other Modes"
    local airplane_energy=$(echo "$passengers * 900" | bc -l)
    local car_energy=$(echo "$passengers * 200" | bc -l)
    local train_energy=$(echo "$passengers * 20" | bc -l)

    print_info "Airplane: $(format_energy $airplane_energy) ($(echo "scale=0; $airplane_energy / $total" | bc)× more)"
    print_info "Car: $(format_energy $car_energy) ($(echo "scale=0; $car_energy / $total" | bc)× more)"
    print_info "High-Speed Rail: $(format_energy $train_energy) ($(echo "scale=1; $train_energy / $total" | bc)× more)"

    echo ""
}

# Validate pod design
validate_pod() {
    local mass=${1:-15000}
    local area=${2:-5.72}
    local passengers=${3:-28}
    local speed=${4:-333.33}

    print_section "Pod Design Validation"
    print_info "Mass: $mass kg"
    print_info "Cross-sectional Area: $area m²"
    print_info "Passenger Capacity: $passengers"
    print_info "Max Speed: $speed m/s ($(echo "scale=0; $speed * 3.6" | bc) km/h)"

    # Tube area
    local tube_area=$(echo "scale=6; 3.14159 * ($TUBE_DIAMETER / 2) * ($TUBE_DIAMETER / 2)" | bc -l)
    print_info "Tube Area: $tube_area m²"

    # Area ratio
    local area_ratio=$(echo "scale=4; $area / $tube_area" | bc -l)

    print_section "Validation Checks"

    # Check 1: Tube area ratio
    if (( $(echo "$area_ratio < 0.6" | bc -l) )); then
        print_success "Tube Area Ratio: $area_ratio (PASS - good clearance)"
    elif (( $(echo "$area_ratio < 0.7" | bc -l) )); then
        print_warning "Tube Area Ratio: $area_ratio (WARNING - approaching Kantrowitz limit)"
    else
        print_error "Tube Area Ratio: $area_ratio (FAIL - exceeds 0.7, choking risk)"
    fi

    # Check 2: Kantrowitz limit
    local kantrowitz=$(echo "scale=2; 340 * sqrt(1 / $area_ratio - 1)" | bc -l)
    print_info "Kantrowitz Limit: $kantrowitz m/s ($(echo "scale=0; $kantrowitz * 3.6" | bc) km/h)"

    if (( $(echo "$speed <= $kantrowitz" | bc -l) )); then
        print_success "Speed Check: PASS (within Kantrowitz limit)"
    else
        print_error "Speed Check: FAIL (exceeds Kantrowitz limit)"
        print_info "Recommendation: Add compressor fan or increase tube diameter"
    fi

    # Check 3: Mass efficiency
    local payload=$(echo "$passengers * 75" | bc -l)
    local mass_eff=$(echo "scale=4; $payload / $mass" | bc -l)

    if (( $(echo "$mass_eff >= 0.3" | bc -l) )); then
        print_success "Mass Efficiency: $(echo "scale=1; $mass_eff * 100" | bc)% (GOOD)"
    else
        print_warning "Mass Efficiency: $(echo "scale=1; $mass_eff * 100" | bc)% (consider lightweight materials)"
    fi

    # Check 4: Power to weight
    local total_power=105  # kW (typical)
    local power_to_weight=$(echo "scale=2; ($total_power * 1000) / $mass" | bc -l)
    print_info "Power to Weight: $power_to_weight W/kg"

    print_section "Validation Result"
    if (( $(echo "$area_ratio < 0.7 && $speed <= $kantrowitz" | bc -l) )); then
        print_success "Pod Design: VALID"
    else
        print_error "Pod Design: REQUIRES MODIFICATIONS"
    fi

    echo ""
}

# Simulate journey
simulate() {
    local distance=${1:-600000}
    local max_speed=${2:-333.33}
    local passengers=${3:-28}

    print_section "Journey Simulation"
    print_info "Route Distance: $(echo "scale=0; $distance / 1000" | bc) km"
    print_info "Maximum Speed: $max_speed m/s ($(echo "scale=0; $max_speed * 3.6" | bc) km/h)"
    print_info "Passengers: $passengers"

    # Acceleration phase (0.5g = 4.9 m/s²)
    local accel=4.9
    local accel_time=$(echo "scale=2; $max_speed / $accel" | bc -l)
    local accel_dist=$(echo "scale=2; 0.5 * $accel * $accel_time * $accel_time" | bc -l)

    # Deceleration phase (1.0g = 9.81 m/s²)
    local decel=9.81
    local decel_time=$(echo "scale=2; $max_speed / $decel" | bc -l)
    local decel_dist=$(echo "scale=2; 0.5 * $decel * $decel_time * $decel_time" | bc -l)

    # Cruising phase
    local cruise_dist=$(echo "$distance - $accel_dist - $decel_dist" | bc -l)
    local cruise_time=$(echo "scale=2; $cruise_dist / $max_speed" | bc -l)

    # Total travel time (in tube)
    local travel_time=$(echo "$accel_time + $cruise_time + $decel_time" | bc -l)

    # Station time (boarding + airlock)
    local station_time=420  # 7 minutes (2 min board + 4 min evacuate + 1 min pre-departure)

    # Total time
    local total_time=$(echo "$travel_time + $station_time" | bc -l)

    print_section "Journey Phases"
    print_info "1. Boarding & Airlock: 7 minutes"
    print_info "2. Acceleration: $(echo "scale=0; $accel_time" | bc) seconds ($(echo "scale=1; $accel_dist / 1000" | bc) km)"
    print_info "3. Cruising: $(echo "scale=0; $cruise_time" | bc) seconds ($(echo "scale=1; $cruise_dist / 1000" | bc) km)"
    print_info "4. Deceleration: $(echo "scale=0; $decel_time" | bc) seconds ($(echo "scale=1; $decel_dist / 1000" | bc) km)"
    print_info "5. Airlock & Deboarding: 5 minutes"

    print_section "Results"
    print_success "Total Time: $(echo "scale=0; $total_time / 60" | bc) minutes"
    print_success "Travel Time (in tube): $(echo "scale=0; $travel_time / 60" | bc) minutes"
    print_success "Average Speed: $(echo "scale=0; $distance / $travel_time * 3.6" | bc) km/h"
    print_success "Maximum Speed: $(echo "scale=0; $max_speed * 3.6" | bc) km/h"

    # Calculate energy
    local mass=15000
    local total_mass=$(echo "$mass + ($passengers * 75)" | bc -l)
    local kinetic=$(echo "0.5 * $total_mass * $max_speed * $max_speed / (3600000 * 0.85)" | bc -l)

    local rho=$(echo "scale=8; (100 * 0.029) / (8.314 * 293)" | bc -l)
    local drag=$(echo "scale=2; 0.5 * $rho * $max_speed * $max_speed * 0.15 * 5.72" | bc -l)
    local cruise_energy=$(echo "scale=2; ($drag * $distance) / 3600000" | bc -l)

    local lev_energy=$(echo "scale=2; (75 * $travel_time) / 3600" | bc -l)
    local aux_energy=$(echo "scale=2; (25 * $travel_time) / 3600" | bc -l)

    local total_energy=$(echo "$kinetic + $cruise_energy + $lev_energy + $aux_energy - ($kinetic * 0.75)" | bc -l)
    local per_pass=$(echo "scale=2; $total_energy / $passengers" | bc -l)

    print_section "Energy & Environmental"
    print_success "Total Energy: $(format_energy $total_energy)"
    print_success "Per Passenger: $(format_energy $per_pass)"

    # CO2 savings vs airplane
    local airplane_co2=$(echo "scale=0; $passengers * ($distance / 1000) * 0.15" | bc -l)
    print_success "CO₂ Savings vs Airplane: $airplane_co2 kg"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-019 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-drag                Calculate drag force at operating conditions"
    echo "    --pressure <Pa>        Air pressure (default: 100 Pa)"
    echo "    --velocity <m/s>       Pod velocity (default: 300 m/s)"
    echo "    --area <m²>            Cross-sectional area (default: 5.72 m²)"
    echo "    --drag-coef <num>      Drag coefficient (default: 0.15)"
    echo ""
    echo "  calc-levitation          Calculate magnetic levitation force"
    echo "    --mass <kg>            Pod mass (default: 15000 kg)"
    echo "    --field-strength <T>   Magnetic field (default: 1.2 T)"
    echo "    --area <m²>            Pole area per magnet (default: 0.4 m²)"
    echo "    --magnets <num>        Number of magnets (default: 8)"
    echo ""
    echo "  calc-energy              Calculate journey energy consumption"
    echo "    --distance <m>         Distance (default: 600000 m)"
    echo "    --mass <kg>            Pod mass (default: 15000 kg)"
    echo "    --speed <m/s>          Max speed (default: 300 m/s)"
    echo "    --passengers <num>     Number of passengers (default: 28)"
    echo ""
    echo "  validate-pod             Validate pod design"
    echo "    --mass <kg>            Pod mass (default: 15000 kg)"
    echo "    --area <m²>            Cross-sectional area (default: 5.72 m²)"
    echo "    --passengers <num>     Capacity (default: 28)"
    echo "    --speed <m/s>          Max speed (default: 333.33 m/s)"
    echo ""
    echo "  simulate                 Simulate complete journey"
    echo "    --distance <m>         Distance (default: 600000 m)"
    echo "    --max-speed <m/s>      Maximum speed (default: 333.33 m/s)"
    echo "    --passengers <num>     Number of passengers (default: 28)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-019 calc-drag --pressure 100 --velocity 300 --area 5.72"
    echo "  wia-auto-019 calc-levitation --mass 15000 --field-strength 1.2"
    echo "  wia-auto-019 calc-energy --distance 600000 --passengers 28"
    echo "  wia-auto-019 validate-pod --mass 15000 --area 5.72 --passengers 28"
    echo "  wia-auto-019 simulate --distance 600000 --max-speed 333.33"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-019 Hyperloop CLI Tool"
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
    calc-drag)
        PRESSURE=100
        VELOCITY=300
        AREA=5.72
        DRAG_COEF=0.15

        while [[ $# -gt 0 ]]; do
            case $1 in
                --pressure) PRESSURE=$2; shift 2 ;;
                --velocity) VELOCITY=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                --drag-coef) DRAG_COEF=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_drag "$PRESSURE" "$VELOCITY" "$AREA" "$DRAG_COEF"
        ;;

    calc-levitation)
        MASS=15000
        FIELD=1.2
        AREA=0.4
        MAGNETS=8

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mass) MASS=$2; shift 2 ;;
                --field-strength) FIELD=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                --magnets) MAGNETS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_levitation "$MASS" "$FIELD" "$AREA" "$MAGNETS"
        ;;

    calc-energy)
        DISTANCE=600000
        MASS=15000
        SPEED=300
        PASSENGERS=28

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) DISTANCE=$2; shift 2 ;;
                --mass) MASS=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --passengers) PASSENGERS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_energy "$DISTANCE" "$MASS" "$SPEED" "$PASSENGERS"
        ;;

    validate-pod)
        MASS=15000
        AREA=5.72
        PASSENGERS=28
        SPEED=333.33

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mass) MASS=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                --passengers) PASSENGERS=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_pod "$MASS" "$AREA" "$PASSENGERS" "$SPEED"
        ;;

    simulate)
        DISTANCE=600000
        MAX_SPEED=333.33
        PASSENGERS=28

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) DISTANCE=$2; shift 2 ;;
                --max-speed) MAX_SPEED=$2; shift 2 ;;
                --passengers) PASSENGERS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate "$DISTANCE" "$MAX_SPEED" "$PASSENGERS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-019 help' for usage information"
        exit 1
        ;;
esac

exit 0
