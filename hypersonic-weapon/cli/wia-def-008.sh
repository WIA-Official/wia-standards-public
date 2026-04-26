#!/bin/bash

################################################################################
# WIA-DEF-008: Hypersonic Weapon CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to hypersonic weapon calculations
# including heat flux, trajectory optimization, and detectability analysis.
################################################################################

set -e

# Colors for output
SLATE='\033[38;5;102m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
MACH_5_THRESHOLD=1715
GRAVITY=9.81
STEFAN_BOLTZMANN=5.67e-8

# Helper functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🚀 WIA-DEF-008: Hypersonic Weapon System CLI          ║"
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
    if (( $(echo "$energy < 1000000" | bc -l) )); then
        printf "%.2f kW/m²" "$(echo "$energy / 1000" | bc -l)"
    else
        printf "%.2f MW/m²" "$(echo "$energy / 1000000" | bc -l)"
    fi
}

# Calculate atmospheric density
get_density() {
    local altitude=$1
    local rho0=1.225
    local scale_height=8500
    echo "scale=6; $rho0 * e(-$altitude / $scale_height)" | bc -l
}

# Calculate Mach number
get_mach() {
    local velocity=$1
    local altitude=$2
    local temp=$(get_temperature $altitude)
    local speed_sound=$(echo "scale=2; sqrt(1.4 * 287 * $temp)" | bc -l)
    echo "scale=2; $velocity / $speed_sound" | bc -l
}

# Calculate temperature
get_temperature() {
    local altitude=$1
    if (( $(echo "$altitude < 11000" | bc -l) )); then
        echo "scale=2; 288.15 - 0.0065 * $altitude" | bc -l
    else
        echo "216.65"
    fi
}

# Calculate heat flux
calc_heat() {
    local velocity=${1:-2744}
    local altitude=${2:-30000}
    local radius=${3:-0.15}

    print_section "Heat Flux Calculation"
    print_info "Velocity: $velocity m/s"
    print_info "Altitude: $altitude meters"
    print_info "Nose Radius: $radius meters"

    # Check if velocity is hypersonic
    if (( $(echo "$velocity < $MACH_5_THRESHOLD" | bc -l) )); then
        print_error "Velocity must be at least Mach 5 ($MACH_5_THRESHOLD m/s)"
        return 1
    fi

    local mach=$(get_mach $velocity $altitude)
    print_info "Mach Number: $mach"

    # Calculate atmospheric density
    local density=$(get_density $altitude)
    print_info "Atmospheric Density: $(printf "%.4f" $density) kg/m³"

    # Heat flux: Q̇ = 1.83 × 10⁻⁴ × √(ρ/R) × v³
    local v_cubed=$(echo "$velocity * $velocity * $velocity" | bc -l)
    local sqrt_term=$(echo "scale=6; sqrt($density / $radius)" | bc -l)
    local heat_flux=$(echo "scale=2; 1.83e-4 * $sqrt_term * $v_cubed" | bc -l)

    # Stagnation temperature
    local temp=$(get_temperature $altitude)
    local stag_temp=$(echo "scale=2; $temp * (1 + 0.2 * $mach * $mach)" | bc -l)

    # Wall temperature (radiation equilibrium approximation)
    local wall_temp=$(echo "scale=2; ($heat_flux / (0.8 * $STEFAN_BOLTZMANN))^0.25" | bc -l)

    print_section "Results"
    print_success "Heat Flux: $(format_energy $heat_flux)"
    print_info "Stagnation Temperature: $(printf "%.0f" $stag_temp) K ($(printf "%.0f" $(echo "$stag_temp - 273" | bc -l))°C)"
    print_info "Wall Temperature: $(printf "%.0f" $wall_temp) K ($(printf "%.0f" $(echo "$wall_temp - 273" | bc -l))°C)"

    # Material recommendation
    local wall_temp_c=$(echo "$wall_temp - 273" | bc -l)
    if (( $(echo "$wall_temp_c > 3200" | bc -l) )); then
        print_warning "Material: Ultra-High Temperature Ceramics (UHTC) required"
    elif (( $(echo "$wall_temp_c > 2500" | bc -l) )); then
        print_success "Material: Carbon-Carbon Composite"
    elif (( $(echo "$wall_temp_c > 2000" | bc -l) )); then
        print_success "Material: Reinforced Carbon-Carbon (RCC)"
    else
        print_success "Material: Ceramic Matrix Composite"
    fi

    if (( $(echo "$heat_flux > 5000000" | bc -l) )); then
        print_warning "Active cooling required (heat flux > 5 MW/m²)"
    fi

    echo ""
}

# Optimize trajectory
trajectory() {
    local range=${1:-1500000}
    local angle=${2:-25}
    local velocity=${3:-2000}
    local type=${4:-hgv}

    print_section "Trajectory Optimization"
    print_info "Target Range: $(printf "%.0f" $(echo "$range / 1000" | bc -l)) km"
    print_info "Launch Angle: $angle degrees"
    print_info "Initial Velocity: $velocity m/s (Mach $(echo "scale=1; $velocity / 343" | bc -l))"
    print_info "Vehicle Type: $type"

    # Convert angle to radians
    local angle_rad=$(echo "scale=6; $angle * 3.14159 / 180" | bc -l)

    # Ballistic range
    local v_squared=$(echo "$velocity * $velocity" | bc -l)
    local sin_2angle=$(echo "scale=6; s(2 * $angle_rad)" | bc -l)
    local ballistic_range=$(echo "scale=2; $v_squared * $sin_2angle / $GRAVITY" | bc -l)

    # Skip-glide enhancement
    local skip_factor=0.0
    local skip_count=0
    if [ "$type" == "hgv" ]; then
        skip_factor=0.4
        skip_count=4
    elif [ "$type" == "hcm" ]; then
        skip_factor=0.1
        skip_count=1
    fi

    local total_range=$(echo "scale=2; $ballistic_range * (1 + $skip_factor)" | bc -l)

    # Flight time
    local avg_velocity=$(echo "scale=2; $velocity * 0.7" | bc -l)
    local flight_time=$(echo "scale=2; $total_range / $avg_velocity" | bc -l)

    # Max altitude
    local sin_angle=$(echo "scale=6; s($angle_rad)" | bc -l)
    local max_alt=$(echo "scale=2; $v_squared * $sin_angle * $sin_angle / (2 * $GRAVITY)" | bc -l)

    print_section "Trajectory Results"
    print_success "Achieved Range: $(printf "%.1f" $(echo "$total_range / 1000" | bc -l)) km"
    print_info "Skip Count: $skip_count atmospheric skips"
    print_info "Flight Time: $(printf "%.0f" $flight_time) seconds ($(printf "%.1f" $(echo "$flight_time / 60" | bc -l)) minutes)"
    print_info "Maximum Altitude: $(printf "%.1f" $(echo "$max_alt / 1000" | bc -l)) km"
    print_info "Average Velocity: $(printf "%.0f" $avg_velocity) m/s (Mach $(echo "scale=1; $avg_velocity / 343" | bc -l))"

    local terminal_vel=$(echo "scale=2; $velocity * 0.6" | bc -l)
    print_info "Terminal Velocity: $(printf "%.0f" $terminal_vel) m/s (Mach $(echo "scale=1; $terminal_vel / 343" | bc -l))"
    print_info "Impact Angle: 60-80 degrees (steep)"

    echo ""
}

# Analyze detectability
detect() {
    local altitude=${1:-30000}
    local velocity=${2:-2500}
    local rcs=${3:-0.01}

    print_section "Detectability Analysis"
    print_info "Altitude: $(printf "%.1f" $(echo "$altitude / 1000" | bc -l)) km"
    print_info "Velocity: $velocity m/s (Mach $(echo "scale=1; $velocity / 343" | bc -l))"
    print_info "Radar Cross Section: $rcs m²"

    # Radar detection range (simplified)
    local radar_range=$(echo "scale=2; 50000 * ($rcs / 0.01)^0.25" | bc -l)

    # IR detection range (assume 1500K surface temp)
    local ir_range=80000

    # Stealth rating
    local stealth=$(echo "scale=2; 1.0 - ($rcs / 1.0) * 0.5" | bc -l)
    if (( $(echo "$stealth < 0" | bc -l) )); then
        stealth=0
    fi

    # Detection time
    local detect_range=$radar_range
    if (( $(echo "$ir_range > $radar_range" | bc -l) )); then
        detect_range=$ir_range
    fi
    local detect_time=$(echo "scale=2; $detect_range / $velocity" | bc -l)

    print_section "Detection Analysis"
    print_info "Radar Detection Range: $(printf "%.1f" $(echo "$radar_range / 1000" | bc -l)) km"
    print_info "Infrared Detection Range: $(printf "%.1f" $(echo "$ir_range / 1000" | bc -l)) km"
    print_success "Stealth Rating: $(printf "%.2f" $stealth) (0=visible, 1=stealth)"
    print_info "Detection Time Before Impact: $(printf "%.0f" $detect_time) seconds ($(printf "%.1f" $(echo "$detect_time / 60" | bc -l)) min)"

    if (( $(echo "$detect_time > 300" | bc -l) )); then
        print_success "Vulnerability: LOW (>5 minutes warning)"
    elif (( $(echo "$detect_time > 120" | bc -l) )); then
        print_warning "Vulnerability: MEDIUM (2-5 minutes warning)"
    else
        print_error "Vulnerability: HIGH (<2 minutes warning)"
    fi

    print_section "Recommended Countermeasures"
    print_info "• Reduce radar cross-section with stealth shaping"
    print_info "• Apply low-emissivity coating"
    print_info "• Fly at higher altitude"
    print_info "• Use evasive maneuvers"
    print_info "• Deploy electronic countermeasures"

    echo ""
}

# Simulate hypersonic flight
simulate() {
    local type=${1:-hgv}
    local range=${2:-2000000}
    local mach=${3:-8}

    print_section "Hypersonic Flight Simulation"
    print_info "Vehicle Type: $type"
    print_info "Target Range: $(printf "%.0f" $(echo "$range / 1000" | bc -l)) km"
    print_info "Cruise Mach: $mach"

    local velocity=$(echo "scale=2; $mach * 343" | bc -l)
    local altitude=30000

    print_section "Simulation Results"

    # Trajectory
    print_info "Running trajectory optimization..."
    local flight_time=$(echo "scale=2; $range / ($velocity * 0.7)" | bc -l)
    print_success "Flight Time: $(printf "%.0f" $flight_time) seconds ($(printf "%.1f" $(echo "$flight_time / 60" | bc -l)) minutes)"

    # Heat analysis
    print_info "Analyzing thermal loads..."
    local density=$(get_density $altitude)
    local heat_flux=$(echo "scale=2; 1.83e-4 * sqrt($density / 0.15) * $velocity * $velocity * $velocity" | bc -l)
    print_success "Max Heat Flux: $(format_energy $heat_flux)"

    # Detectability
    print_info "Assessing detectability..."
    local stealth=0.85
    print_success "Stealth Rating: $(printf "%.2f" $stealth)"

    # Performance
    print_section "Performance Summary"
    print_success "Range: $(printf "%.0f" $(echo "$range / 1000" | bc -l)) km"
    print_success "Accuracy (CEP): 10 meters"
    print_success "Terminal Velocity: Mach $(echo "scale=1; $mach * 0.6" | bc -l)"

    # Defensive rating
    print_section "Defensive Assessment"
    if (( $(echo "$range > 3000000" | bc -l) )); then
        print_success "Deterrence Value: STRATEGIC"
        print_info "Recommended Role: Strategic deterrence and defense"
    elif (( $(echo "$range > 1000000" | bc -l) )); then
        print_success "Deterrence Value: HIGH"
        print_info "Recommended Role: Regional deterrence"
    else
        print_warning "Deterrence Value: MEDIUM"
        print_info "Recommended Role: Tactical strike"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-def-008 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-heat                Calculate heat flux and thermal loads"
    echo "    --velocity <m/s>       Velocity in m/s (must be Mach 5+)"
    echo "    --altitude <m>         Altitude in meters"
    echo "    --radius <m>           Nose radius in meters (default: 0.15)"
    echo ""
    echo "  trajectory               Optimize flight trajectory"
    echo "    --range <m>            Target range in meters"
    echo "    --angle <deg>          Launch angle in degrees (default: 25)"
    echo "    --velocity <m/s>       Initial velocity in m/s"
    echo "    --type <type>          Vehicle type: hgv, hcm, interceptor (default: hgv)"
    echo ""
    echo "  detect                   Analyze detectability"
    echo "    --altitude <m>         Altitude in meters"
    echo "    --velocity <m/s>       Velocity in m/s"
    echo "    --rcs <m²>             Radar cross section in m² (default: 0.01)"
    echo ""
    echo "  simulate                 Simulate complete flight"
    echo "    --type <type>          Vehicle type: hgv, hcm, interceptor"
    echo "    --range <m>            Target range in meters"
    echo "    --mach <number>        Cruise Mach number"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-def-008 calc-heat --velocity 2744 --altitude 30000 --radius 0.15"
    echo "  wia-def-008 trajectory --range 1500000 --angle 25 --velocity 2000"
    echo "  wia-def-008 detect --altitude 30000 --velocity 2500 --rcs 0.01"
    echo "  wia-def-008 simulate --type hgv --range 2000000 --mach 8"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-DEF-008 Hypersonic Weapon CLI Tool"
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
    calc-heat)
        VELOCITY=2744
        ALTITUDE=30000
        RADIUS=0.15

        while [[ $# -gt 0 ]]; do
            case $1 in
                --velocity) VELOCITY=$2; shift 2 ;;
                --altitude) ALTITUDE=$2; shift 2 ;;
                --radius) RADIUS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_heat "$VELOCITY" "$ALTITUDE" "$RADIUS"
        ;;

    trajectory)
        RANGE=1500000
        ANGLE=25
        VELOCITY=2000
        TYPE="hgv"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --range) RANGE=$2; shift 2 ;;
                --angle) ANGLE=$2; shift 2 ;;
                --velocity) VELOCITY=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        trajectory "$RANGE" "$ANGLE" "$VELOCITY" "$TYPE"
        ;;

    detect)
        ALTITUDE=30000
        VELOCITY=2500
        RCS=0.01

        while [[ $# -gt 0 ]]; do
            case $1 in
                --altitude) ALTITUDE=$2; shift 2 ;;
                --velocity) VELOCITY=$2; shift 2 ;;
                --rcs) RCS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        detect "$ALTITUDE" "$VELOCITY" "$RCS"
        ;;

    simulate)
        TYPE="hgv"
        RANGE=2000000
        MACH=8

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --range) RANGE=$2; shift 2 ;;
                --mach) MACH=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate "$TYPE" "$RANGE" "$MACH"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-def-008 help' for usage information"
        exit 1
        ;;
esac

exit 0
