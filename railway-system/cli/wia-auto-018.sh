#!/bin/bash

################################################################################
# WIA-AUTO-018: Railway System CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Railway Engineering Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to railway system calculations
# including braking distance, headway, signaling validation, and safety checks.
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
SERVICE_DECELERATION=0.7
EMERGENCY_DECELERATION=1.5
STANDARD_GAUGE=1435
PLATFORM_HEIGHT=1100

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🚄 WIA-AUTO-018: Railway System CLI                 ║"
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

# Calculate braking distance
calc_braking() {
    local speed_kmh=${1:-200}
    local deceleration=${2:-0.7}
    local reaction_time=${3:-3}
    local safety_margin=${4:-50}

    print_section "Braking Distance Calculation"
    print_info "Initial Speed: $speed_kmh km/h"
    print_info "Deceleration: $deceleration m/s²"
    print_info "Reaction Time: $reaction_time seconds"
    print_info "Safety Margin: $safety_margin meters"

    # Convert km/h to m/s
    local speed_ms=$(echo "scale=2; $speed_kmh / 3.6" | bc -l)
    print_info "Speed in m/s: $speed_ms m/s"

    # Reaction distance: d = v × t
    local reaction_dist=$(echo "scale=2; $speed_ms * $reaction_time" | bc -l)
    print_info "Reaction Distance: $reaction_dist meters"

    # Braking distance: d = v² / (2a)
    local braking_dist=$(echo "scale=2; ($speed_ms * $speed_ms) / (2 * $deceleration)" | bc -l)
    print_info "Braking Distance: $braking_dist meters"

    # Total distance
    local total_dist=$(echo "scale=2; $reaction_dist + $braking_dist + $safety_margin" | bc -l)

    # Time to stop
    local time_to_stop=$(echo "scale=2; $speed_ms / $deceleration" | bc -l)

    print_section "Results"
    print_success "Total Braking Distance: ${total_dist} meters"
    print_info "Time to Stop: $time_to_stop seconds"

    # Safety assessment
    if (( $(echo "$speed_kmh <= 80" | bc -l) )); then
        print_success "Train Type: Metro/Urban (safe for short headway)"
    elif (( $(echo "$speed_kmh <= 160" | bc -l) )); then
        print_success "Train Type: Regional (moderate headway required)"
    elif (( $(echo "$speed_kmh <= 250" | bc -l) )); then
        print_warning "Train Type: Intercity (longer headway required)"
    else
        print_warning "Train Type: High-Speed (maximum headway required)"
    fi

    echo ""
}

# Validate train headway
validate_headway() {
    local speed_kmh=${1:-200}
    local braking_dist=${2:-3000}
    local train_length=${3:-200}
    local safety_dist=${4:-100}

    print_section "Headway Validation"
    print_info "Train Speed: $speed_kmh km/h"
    print_info "Braking Distance: $braking_dist meters"
    print_info "Train Length: $train_length meters"
    print_info "Safety Distance: $safety_dist meters"

    # Minimum headway: H = d_b + L_t + d_s
    local headway=$(echo "scale=2; $braking_dist + $train_length + $safety_dist" | bc -l)

    # Convert speed to m/s
    local speed_ms=$(echo "scale=2; $speed_kmh / 3.6" | bc -l)

    # Headway in seconds
    local headway_sec=$(echo "scale=2; $headway / $speed_ms" | bc -l)

    # Line capacity (trains per hour)
    local capacity=$(echo "scale=0; 3600 / $headway_sec" | bc -l)

    print_section "Results"
    print_success "Minimum Headway: ${headway} meters"
    print_info "Headway Time: $headway_sec seconds"
    print_success "Line Capacity: $capacity trains/hour"

    # Capacity assessment
    if (( $(echo "$capacity >= 30" | bc -l) )); then
        print_success "Capacity: HIGH (suitable for metro operations)"
    elif (( $(echo "$capacity >= 20" | bc -l) )); then
        print_success "Capacity: GOOD (suitable for urban rail)"
    elif (( $(echo "$capacity >= 10" | bc -l) )); then
        print_warning "Capacity: MODERATE (suitable for regional rail)"
    else
        print_warning "Capacity: LOW (suitable for mainline only)"
    fi

    echo ""
}

# Generate signal configuration
generate_signal() {
    local signal_type=${1:-ETCS}
    local level=${2:-2}

    print_section "Signal Configuration Generator"
    print_info "Signal Type: $signal_type"
    print_info "Level: $level"

    case "$signal_type" in
        ETCS)
            print_section "ETCS Level $level Configuration"

            case "$level" in
                1)
                    print_info "Architecture: Track-side Eurobalises"
                    print_info "Communication: Point-based transmission"
                    print_info "Signals: Track-side signals required"
                    print_success "Components:"
                    print_info "  - Eurobalises: Point information transmission"
                    print_info "  - LEU: Lineside Electronic Unit"
                    print_info "  - DMI: Driver Machine Interface"
                    print_info "  - BTM: Balise Transmission Module"
                    ;;
                2)
                    print_info "Architecture: Continuous radio communication"
                    print_info "Communication: GSM-R (876-880 / 921-925 MHz)"
                    print_info "Signals: Cab signaling only (no track-side signals)"
                    print_success "Components:"
                    print_info "  - RBC: Radio Block Center"
                    print_info "  - GSM-R Network: Continuous communication"
                    print_info "  - Eurobalises: Location reference only"
                    print_info "  - DMI: Driver Machine Interface"
                    print_info "  - Odometry: Precise positioning"
                    ;;
                3)
                    print_info "Architecture: Moving block operation"
                    print_info "Communication: GSM-R / LTE-R / 5G-R"
                    print_info "Signals: No track circuits (train integrity on-board)"
                    print_success "Components:"
                    print_info "  - RBC: Radio Block Center (enhanced)"
                    print_info "  - GNSS: Satellite positioning"
                    print_info "  - Train Integrity: On-board detection"
                    print_info "  - DMI: Driver Machine Interface"
                    print_warning "Status: Under development (not fully deployed)"
                    ;;
                *)
                    print_error "Invalid ETCS level. Use 1, 2, or 3."
                    ;;
            esac
            ;;

        CBTC)
            print_section "CBTC Configuration"
            print_info "Type: Communications-Based Train Control"
            print_info "Application: Metro and urban rail systems"
            print_success "Components:"
            print_info "  - Zone Controller: Train supervision"
            print_info "  - Radio Network: Continuous bidirectional (2.4 GHz / LTE)"
            print_info "  - ATO: Automatic Train Operation"
            print_info "  - Wayside Equipment: Track circuits / axle counters"
            print_success "Automation Levels (GoA):"
            print_info "  - GoA 1: ATP only (driver controls)"
            print_info "  - GoA 2: ATO with driver supervision"
            print_info "  - GoA 3: Driverless (attendant on board)"
            print_info "  - GoA 4: Unattended (fully autonomous)"
            print_success "Typical Performance:"
            print_info "  - Headway: 90-120 seconds"
            print_info "  - Capacity: 30-40 trains/hour"
            print_info "  - Speed: Up to 100 km/h"
            ;;

        PTC)
            print_section "PTC Configuration"
            print_info "Type: Positive Train Control (North America)"
            print_info "Application: Freight and passenger railways"
            print_success "Components:"
            print_info "  - GPS: Positioning"
            print_info "  - Radio: 220 MHz communication"
            print_info "  - Back Office Server: Movement authority"
            print_info "  - Wayside Interface Units: Signal/switch monitoring"
            print_success "Core Functions:"
            print_info "  - Prevent train-to-train collisions"
            print_info "  - Enforce speed restrictions"
            print_info "  - Protect roadway workers"
            print_info "  - Prevent movement through misaligned switches"
            ;;

        *)
            print_error "Unknown signal type. Use ETCS, CBTC, or PTC."
            exit 1
            ;;
    esac

    echo ""
}

# Monitor platform safety
monitor_platform() {
    local station=${1:-"Central Station"}
    local platform=${2:-"1"}

    print_section "Platform Safety Monitor"
    print_info "Station: $station"
    print_info "Platform: $platform"

    # Simulate platform monitoring
    print_section "Platform Status"
    print_success "Platform Screen Doors: OPERATIONAL"
    print_info "Door Status: CLOSED & LOCKED"
    print_info "Last Operation: $(date +'%Y-%m-%d %H:%M:%S')"

    print_section "Safety Systems"
    print_success "Edge Detection: ACTIVE"
    print_success "CCTV Monitoring: ACTIVE"
    print_success "Emergency Stop Buttons: ARMED"
    print_success "Fire Detection: NORMAL"
    print_success "Ventilation: NORMAL"

    print_section "Train Alignment"
    print_info "Stopping Position: MARKER A"
    print_info "Alignment Tolerance: ±300 mm"
    print_success "Last Train Accuracy: +15 cm (EXCELLENT)"

    print_section "Passenger Flow"
    print_info "Waiting Passengers: $(shuf -i 20-150 -n 1)"
    print_info "Platform Occupancy: $(shuf -i 30-80 -n 1)%"
    print_success "Overcrowding Risk: LOW"

    print_section "Environmental Conditions"
    print_info "Temperature: $(shuf -i 18-24 -n 1)°C"
    print_info "Air Quality: GOOD"
    print_info "Lighting Level: 100%"
    print_success "Overall Status: ALL SYSTEMS NORMAL"

    echo ""
}

# Calculate energy consumption
calc_energy() {
    local mass=${1:-400}
    local speed_kmh=${2:-200}
    local distance=${3:-100}

    print_section "Energy Consumption Calculation"
    print_info "Train Mass: $mass tonnes"
    print_info "Average Speed: $speed_kmh km/h"
    print_info "Distance: $distance km"

    # Convert to SI units
    local mass_kg=$(echo "$mass * 1000" | bc -l)
    local speed_ms=$(echo "scale=2; $speed_kmh / 3.6" | bc -l)

    # Kinetic energy: E = 0.5 × m × v²
    local kinetic_energy=$(echo "scale=2; 0.5 * $mass_kg * $speed_ms * $speed_ms" | bc -l)
    local kinetic_kwh=$(echo "scale=2; $kinetic_energy / 3600000" | bc -l)

    # Approximate resistance energy (80 kWh/km at 200 km/h)
    local energy_per_km=$(echo "scale=2; 0.4 * $speed_kmh" | bc -l)
    local resistance_energy=$(echo "scale=2; $energy_per_km * $distance" | bc -l)

    # Total energy consumption
    local total_energy=$(echo "scale=2; $kinetic_kwh + $resistance_energy" | bc -l)

    # With regenerative braking (assume 30% recovery)
    local energy_recovered=$(echo "scale=2; $total_energy * 0.3" | bc -l)
    local net_energy=$(echo "scale=2; $total_energy - $energy_recovered" | bc -l)

    print_section "Results"
    print_info "Kinetic Energy: $kinetic_kwh kWh"
    print_info "Resistance Energy: $resistance_energy kWh"
    print_success "Gross Energy Consumption: $total_energy kWh"
    print_success "Energy Recovered (30%): $energy_recovered kWh"
    print_success "Net Energy Consumption: $net_energy kWh"

    # Energy per km
    local energy_per_km_net=$(echo "scale=2; $net_energy / $distance" | bc -l)
    print_info "Energy Efficiency: $energy_per_km_net kWh/km"

    # CO2 comparison
    local co2_saved=$(echo "scale=2; $net_energy * 0.15" | bc -l)  # vs 150g CO2/kWh for cars
    print_section "Environmental Impact"
    print_success "CO₂ Savings vs. Car: ~$co2_saved kg"
    print_info "Passenger Capacity: 800-1200 passengers"
    print_success "Energy per Passenger: Very efficient!"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-018 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-braking             Calculate braking distance"
    echo "    --speed <km/h>         Initial speed (default: 200 km/h)"
    echo "    --deceleration <m/s²>  Deceleration rate (default: 0.7 m/s²)"
    echo "    --reaction <seconds>   Reaction time (default: 3 seconds)"
    echo "    --safety <meters>      Safety margin (default: 50 meters)"
    echo ""
    echo "  validate-headway         Validate train separation"
    echo "    --speed <km/h>         Train speed (default: 200 km/h)"
    echo "    --braking <meters>     Braking distance (default: 3000 m)"
    echo "    --length <meters>      Train length (default: 200 m)"
    echo "    --safety <meters>      Safety distance (default: 100 m)"
    echo ""
    echo "  generate-signal          Generate signaling configuration"
    echo "    --type <ETCS|CBTC|PTC> Signal system type (default: ETCS)"
    echo "    --level <1|2|3>        ETCS level (default: 2)"
    echo ""
    echo "  monitor-platform         Monitor platform safety systems"
    echo "    --station <name>       Station name (default: 'Central Station')"
    echo "    --platform <number>    Platform number (default: '1')"
    echo ""
    echo "  calc-energy              Calculate energy consumption"
    echo "    --mass <tonnes>        Train mass (default: 400 tonnes)"
    echo "    --speed <km/h>         Average speed (default: 200 km/h)"
    echo "    --distance <km>        Distance (default: 100 km)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-018 calc-braking --speed 300 --deceleration 0.8"
    echo "  wia-auto-018 validate-headway --speed 200 --braking-distance 3000"
    echo "  wia-auto-018 generate-signal --type ETCS --level 2"
    echo "  wia-auto-018 monitor-platform --station 'Central' --platform 3"
    echo "  wia-auto-018 calc-energy --mass 400 --speed 250 --distance 500"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-018 Railway System CLI Tool"
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
    calc-braking)
        SPEED=200
        DECELERATION=0.7
        REACTION=3
        SAFETY=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --speed) SPEED=$2; shift 2 ;;
                --deceleration) DECELERATION=$2; shift 2 ;;
                --reaction) REACTION=$2; shift 2 ;;
                --safety) SAFETY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_braking "$SPEED" "$DECELERATION" "$REACTION" "$SAFETY"
        ;;

    validate-headway)
        SPEED=200
        BRAKING=3000
        LENGTH=200
        SAFETY=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --speed) SPEED=$2; shift 2 ;;
                --braking-distance) BRAKING=$2; shift 2 ;;
                --length) LENGTH=$2; shift 2 ;;
                --safety) SAFETY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_headway "$SPEED" "$BRAKING" "$LENGTH" "$SAFETY"
        ;;

    generate-signal)
        TYPE="ETCS"
        LEVEL=2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --level) LEVEL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_signal "$TYPE" "$LEVEL"
        ;;

    monitor-platform)
        STATION="Central Station"
        PLATFORM="1"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --station) STATION=$2; shift 2 ;;
                --platform) PLATFORM=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_platform "$STATION" "$PLATFORM"
        ;;

    calc-energy)
        MASS=400
        SPEED=200
        DISTANCE=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mass) MASS=$2; shift 2 ;;
                --speed) SPEED=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_energy "$MASS" "$SPEED" "$DISTANCE"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-018 help' for usage information"
        exit 1
        ;;
esac

exit 0
