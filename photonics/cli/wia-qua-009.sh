#!/bin/bash

################################################################################
# WIA-QUA-009: Photonics CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Quantum Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to photonics calculations
# including photon energy, fiber design, laser analysis, and LiDAR simulation.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;94m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SPEED_OF_LIGHT=299792458
PLANCK_CONSTANT=6.62607015e-34
ELEMENTARY_CHARGE=1.602176634e-19

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║              💡 WIA-QUA-009: Photonics CLI                    ║"
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

# Parse wavelength with unit
parse_wavelength() {
    local input=$1
    local value
    local unit

    # Extract numeric value and unit
    if [[ $input =~ ^([0-9.]+)([a-zA-Z]+)$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]}
    else
        value=$input
        unit="m"
    fi

    # Convert to meters
    case $unit in
        nm) echo "scale=20; $value / 1000000000" | bc -l ;;
        um|μm) echo "scale=20; $value / 1000000" | bc -l ;;
        mm) echo "scale=20; $value / 1000" | bc -l ;;
        m) echo "$value" ;;
        *) echo "$value" ;;
    esac
}

# Format wavelength
format_wavelength() {
    local meters=$1
    local nm=$(echo "$meters * 1000000000" | bc -l)

    if (( $(echo "$nm < 1000" | bc -l) )); then
        printf "%.1f nm" "$nm"
    elif (( $(echo "$nm < 1000000" | bc -l) )); then
        local um=$(echo "$meters * 1000000" | bc -l)
        printf "%.2f μm" "$um"
    else
        printf "%.2e m" "$meters"
    fi
}

# Format power
format_power() {
    local power=$1

    if (( $(echo "$power < 0.000000001" | bc -l) )); then
        printf "%.2f pW" "$(echo "$power * 1000000000000" | bc -l)"
    elif (( $(echo "$power < 0.000001" | bc -l) )); then
        printf "%.2f nW" "$(echo "$power * 1000000000" | bc -l)"
    elif (( $(echo "$power < 0.001" | bc -l) )); then
        printf "%.2f μW" "$(echo "$power * 1000000" | bc -l)"
    elif (( $(echo "$power < 1" | bc -l) )); then
        printf "%.2f mW" "$(echo "$power * 1000" | bc -l)"
    elif (( $(echo "$power < 1000" | bc -l) )); then
        printf "%.2f W" "$power"
    else
        printf "%.2f kW" "$(echo "$power / 1000" | bc -l)"
    fi
}

# Calculate photon energy
photon_energy() {
    local wavelength_input=${1:-1550nm}
    local unit=${2:-eV}

    print_section "Photon Energy Calculation"

    # Parse wavelength
    local wavelength=$(parse_wavelength "$wavelength_input")

    print_info "Wavelength: $(format_wavelength $wavelength)"

    # Calculate frequency: ν = c/λ
    local frequency=$(echo "scale=10; $SPEED_OF_LIGHT / $wavelength" | bc -l)
    print_info "Frequency: $(printf "%.3e" $frequency) Hz"

    # Calculate energy: E = hν
    local energy=$(echo "scale=30; $PLANCK_CONSTANT * $frequency" | bc -l)

    print_section "Results"

    # Energy in joules
    print_info "Energy (J): $(printf "%.3e" $energy) J"

    # Energy in electronvolts
    local energy_ev=$(echo "scale=10; $energy / $ELEMENTARY_CHARGE" | bc -l)
    print_success "Energy (eV): $(printf "%.3f" $energy_ev) eV"

    # Determine color
    local nm=$(echo "$wavelength * 1000000000" | bc -l)
    local color="Non-visible"

    if (( $(echo "$nm >= 380 && $nm <= 450" | bc -l) )); then
        color="Violet 💜"
    elif (( $(echo "$nm > 450 && $nm <= 495" | bc -l) )); then
        color="Blue 💙"
    elif (( $(echo "$nm > 495 && $nm <= 570" | bc -l) )); then
        color="Green 💚"
    elif (( $(echo "$nm > 570 && $nm <= 590" | bc -l) )); then
        color="Yellow 💛"
    elif (( $(echo "$nm > 590 && $nm <= 620" | bc -l) )); then
        color="Orange 🧡"
    elif (( $(echo "$nm > 620 && $nm <= 750" | bc -l) )); then
        color="Red ❤️"
    elif (( $(echo "$nm < 380" | bc -l) )); then
        color="Ultraviolet (UV)"
    else
        color="Infrared (IR)"
    fi

    print_info "Color: $color"

    echo ""
}

# Design optical fiber
design_fiber() {
    local core_dia_input=${1:-9um}
    local wavelength_input=${2:-1550nm}

    print_section "Optical Fiber Design"

    # Parse inputs
    local core_radius=$(echo "$(parse_wavelength "$core_dia_input") / 2" | bc -l)
    local wavelength=$(parse_wavelength "$wavelength_input")

    # Standard fiber parameters
    local cladding_radius=$(echo "$core_radius * 13.889" | bc -l)  # 125 μm cladding for 9 μm core
    local core_index=1.4681
    local cladding_index=1.4628

    print_info "Core radius: $(echo "$core_radius * 1000000" | bc -l) μm"
    print_info "Cladding radius: $(echo "$cladding_radius * 1000000" | bc -l) μm"
    print_info "Core index: $core_index"
    print_info "Cladding index: $cladding_index"
    print_info "Wavelength: $(format_wavelength $wavelength)"

    # Calculate numerical aperture: NA = √(n₁² - n₂²)
    local n1_sq=$(echo "$core_index * $core_index" | bc -l)
    local n2_sq=$(echo "$cladding_index * $cladding_index" | bc -l)
    local na=$(echo "scale=6; sqrt($n1_sq - $n2_sq)" | bc -l)

    # Calculate V-number: V = (2π/λ) × a × NA
    local pi=3.14159265359
    local v_number=$(echo "scale=6; (2 * $pi * $core_radius * $na) / $wavelength" | bc -l)

    print_section "Fiber Properties"
    print_success "Numerical Aperture (NA): $(printf "%.4f" $na)"
    print_info "V-number: $(printf "%.2f" $v_number)"

    # Determine fiber type
    if (( $(echo "$v_number < 2.405" | bc -l) )); then
        print_success "Fiber Type: Single-Mode"

        # Calculate mode field diameter
        local mfd_factor=$(echo "scale=6; 0.65 + 1.619 / ($v_number ^ 1.5) + 2.879 / ($v_number ^ 6)" | bc -l)
        local mfd=$(echo "scale=6; 2 * $core_radius * $mfd_factor" | bc -l)
        print_info "Mode Field Diameter: $(echo "$mfd * 1000000" | bc -l) μm"
    else
        print_warning "Fiber Type: Multi-Mode"
        local num_modes=$(echo "scale=0; ($v_number * $v_number) / 2" | bc -l)
        print_info "Approximate number of modes: $num_modes"
    fi

    # Typical attenuation at different wavelengths
    local attenuation
    local nm=$(echo "$wavelength * 1000000000" | bc -l)

    if (( $(echo "$nm >= 1520 && $nm <= 1570" | bc -l) )); then
        attenuation=0.2
    elif (( $(echo "$nm >= 1280 && $nm <= 1340" | bc -l) )); then
        attenuation=0.35
    else
        attenuation=0.5
    fi

    print_info "Typical Attenuation: $attenuation dB/km"

    echo ""
}

# Analyze laser system
analyze_laser() {
    local type=${1:-diode}
    local wavelength_input=${2:-850nm}
    local power_input=${3:-10mW}

    print_section "Laser System Analysis"

    # Parse inputs
    local wavelength=$(parse_wavelength "$wavelength_input")

    # Parse power
    local power
    if [[ $power_input =~ ^([0-9.]+)(mW|W|kW)$ ]]; then
        local pval=${BASH_REMATCH[1]}
        local punit=${BASH_REMATCH[2]}

        case $punit in
            mW) power=$(echo "$pval / 1000" | bc -l) ;;
            W) power=$pval ;;
            kW) power=$(echo "$pval * 1000" | bc -l) ;;
        esac
    else
        power=$power_input
    fi

    print_info "Laser Type: $type"
    print_info "Wavelength: $(format_wavelength $wavelength)"
    print_info "Output Power: $(format_power $power)"

    # Typical efficiency by type
    local efficiency
    case $type in
        diode) efficiency=0.35 ;;
        fiber) efficiency=0.30 ;;
        solid-state) efficiency=0.20 ;;
        gas) efficiency=0.10 ;;
        *) efficiency=0.25 ;;
    esac

    # Calculate electrical power
    local electrical=$(echo "scale=6; $power / $efficiency" | bc -l)
    local heat=$(echo "scale=6; $electrical - $power" | bc -l)

    print_section "Performance Metrics"
    print_success "Wall-plug Efficiency: $(echo "$efficiency * 100" | bc -l)%"
    print_info "Electrical Input: $(format_power $electrical)"
    print_info "Heat Dissipation: $(format_power $heat)"

    # Calculate photon flux
    local photon_energy=$(echo "scale=30; $PLANCK_CONSTANT * $SPEED_OF_LIGHT / $wavelength" | bc -l)
    local photon_flux=$(echo "scale=10; $power / $photon_energy" | bc -l)

    print_info "Photon Flux: $(printf "%.3e" $photon_flux) photons/s"

    # Typical beam parameters
    print_section "Beam Parameters"
    print_info "Beam Quality (M²): 1.1 (typical for $type)"
    print_info "Beam Diameter: 1.0 mm (typical)"
    print_info "Divergence: 1.0 mrad (typical)"

    echo ""
}

# Simulate LiDAR
simulate_lidar() {
    local range_input=${1:-100m}
    local resolution_input=${2:-0.1m}

    print_section "LiDAR System Simulation"

    # Parse range
    local range
    if [[ $range_input =~ ^([0-9.]+)m$ ]]; then
        range=${BASH_REMATCH[1]}
    else
        range=$range_input
    fi

    # Parse resolution
    local resolution
    if [[ $resolution_input =~ ^([0-9.]+)m$ ]]; then
        resolution=${BASH_REMATCH[1]}
    else
        resolution=$resolution_input
    fi

    print_info "Maximum Range: $range m"
    print_info "Range Resolution: $resolution m"

    # Typical LiDAR parameters
    local wavelength=905e-9  # 905 nm (common for automotive LiDAR)
    local pulse_energy=1e-6  # 1 μJ
    local pulse_rate=100000  # 100 kHz

    print_info "Wavelength: $(format_wavelength $wavelength)"
    print_info "Pulse Energy: $(printf "%.1f" $(echo "$pulse_energy * 1000000" | bc -l)) μJ"
    print_info "Pulse Rate: $(echo "$pulse_rate / 1000" | bc -l) kHz"

    print_section "Performance Analysis"

    # Calculate time of flight
    local tof=$(echo "scale=10; 2 * $range / $SPEED_OF_LIGHT" | bc -l)
    print_info "Time of Flight: $(printf "%.3f" $(echo "$tof * 1000000000" | bc -l)) ns"

    # Points per second
    local point_rate=$(echo "scale=0; $pulse_rate / 1" | bc -l)
    print_success "Point Rate: $(printf "%.0f" $point_rate) points/s"

    # Calculate angular resolution (typical)
    local h_fov=120  # degrees
    local v_fov=25   # degrees
    local h_points=1024
    local v_points=64

    local h_res=$(echo "scale=3; $h_fov / $h_points" | bc -l)
    local v_res=$(echo "scale=3; $v_fov / $v_points" | bc -l)

    print_info "Horizontal FOV: ${h_fov}°"
    print_info "Vertical FOV: ${v_fov}°"
    print_info "Angular Resolution: ${h_res}° × ${v_res}°"

    # Accuracy estimate
    local accuracy=$(echo "scale=3; $resolution / 2" | bc -l)
    print_success "Typical Accuracy: ±${accuracy} m"

    print_section "Applications"
    print_info "- Autonomous vehicles"
    print_info "- 3D mapping and surveying"
    print_info "- Obstacle detection"
    print_info "- Environmental monitoring"

    echo ""
}

# Calculate LED efficiency
led_efficiency() {
    local input_power_input=${1:-1W}
    local lumens=${2:-100}

    print_section "LED Luminous Efficacy"

    # Parse power
    local power
    if [[ $input_power_input =~ ^([0-9.]+)W$ ]]; then
        power=${BASH_REMATCH[1]}
    else
        power=$input_power_input
    fi

    print_info "Input Power: ${power} W"
    print_info "Luminous Flux: ${lumens} lm"

    # Calculate efficacy
    local efficacy=$(echo "scale=2; $lumens / $power" | bc -l)

    print_section "Results"
    print_success "Luminous Efficacy: ${efficacy} lm/W"

    # Compare to standards
    if (( $(echo "$efficacy < 50" | bc -l) )); then
        print_warning "Below standard LED efficiency"
    elif (( $(echo "$efficacy < 100" | bc -l) )); then
        print_success "Good LED efficiency"
    elif (( $(echo "$efficacy < 150" | bc -l) )); then
        print_success "High-efficiency LED"
    else
        print_success "Excellent efficiency (state-of-the-art)"
    fi

    # Typical LED efficacies
    print_section "Reference Values"
    print_info "Incandescent bulb: 10-20 lm/W"
    print_info "Fluorescent: 50-100 lm/W"
    print_info "Standard LED: 80-120 lm/W"
    print_info "High-efficiency LED: 150-200 lm/W"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-qua-009 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  photon-energy            Calculate photon energy from wavelength"
    echo "    --wavelength <value>   Wavelength (e.g., 1550nm, 850nm, 532nm)"
    echo "    --unit <eV|J>          Output unit (default: eV)"
    echo ""
    echo "  design-fiber             Design optical fiber"
    echo "    --core-dia <value>     Core diameter (e.g., 9um, 50um)"
    echo "    --wavelength <value>   Operating wavelength (default: 1550nm)"
    echo ""
    echo "  analyze-laser            Analyze laser system"
    echo "    --type <type>          Laser type (diode, fiber, solid-state, gas)"
    echo "    --wavelength <value>   Wavelength (default: 850nm)"
    echo "    --power <value>        Output power (e.g., 10mW, 1W)"
    echo ""
    echo "  simulate-lidar           Simulate LiDAR system"
    echo "    --range <value>        Maximum range (default: 100m)"
    echo "    --resolution <value>   Range resolution (default: 0.1m)"
    echo ""
    echo "  led-efficiency           Calculate LED luminous efficacy"
    echo "    --input-power <value>  Input power (default: 1W)"
    echo "    --output-lumens <val>  Output lumens (default: 100)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-qua-009 photon-energy --wavelength 1550nm"
    echo "  wia-qua-009 design-fiber --core-dia 9um --wavelength 1550nm"
    echo "  wia-qua-009 analyze-laser --type diode --wavelength 850nm --power 10mW"
    echo "  wia-qua-009 simulate-lidar --range 100m --resolution 0.1m"
    echo "  wia-qua-009 led-efficiency --input-power 5W --output-lumens 500"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-QUA-009 Photonics CLI Tool"
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
    photon-energy)
        WAVELENGTH="1550nm"
        UNIT="eV"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --wavelength) WAVELENGTH=$2; shift 2 ;;
                --unit) UNIT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        photon_energy "$WAVELENGTH" "$UNIT"
        ;;

    design-fiber)
        CORE_DIA="9um"
        WAVELENGTH="1550nm"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --core-dia) CORE_DIA=$2; shift 2 ;;
                --wavelength) WAVELENGTH=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_fiber "$CORE_DIA" "$WAVELENGTH"
        ;;

    analyze-laser)
        TYPE="diode"
        WAVELENGTH="850nm"
        POWER="10mW"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --wavelength) WAVELENGTH=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_laser "$TYPE" "$WAVELENGTH" "$POWER"
        ;;

    simulate-lidar)
        RANGE="100m"
        RESOLUTION="0.1m"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --range) RANGE=$2; shift 2 ;;
                --resolution) RESOLUTION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate_lidar "$RANGE" "$RESOLUTION"
        ;;

    led-efficiency)
        INPUT_POWER="1W"
        OUTPUT_LUMENS="100"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --input-power) INPUT_POWER=$2; shift 2 ;;
                --output-lumens) OUTPUT_LUMENS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        led_efficiency "$INPUT_POWER" "$OUTPUT_LUMENS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-qua-009 help' for usage information"
        exit 1
        ;;
esac

exit 0

# 弘益人間 (홍익인간) · Benefit All Humanity
