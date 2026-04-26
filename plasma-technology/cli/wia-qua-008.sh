#!/bin/bash

################################################################################
# WIA-QUA-008: Plasma Technology CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Plasma Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to plasma technology calculations
# including Debye length, plasma generation, fusion power, and thruster performance.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
ELEMENTARY_CHARGE=1.602176634e-19
ELECTRON_MASS=9.1093837015e-31
BOLTZMANN=1.380649e-23
EPSILON_0=8.8541878128e-12
AMU=1.66053906660e-27
STANDARD_GRAVITY=9.80665

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🔮 WIA-QUA-008: Plasma Technology CLI               ║"
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

format_scientific() {
    printf "%.2e" "$1"
}

# Calculate Debye length
calc_debye_length() {
    local density=${1:-1e18}
    local temperature=${2:-10000}

    print_section "Debye Length Calculation"
    print_info "Electron Density: $(format_scientific $density) m⁻³"
    print_info "Electron Temperature: $temperature K"

    # λD = √(ε₀kT / ne²)
    local numerator=$(echo "$EPSILON_0 * $BOLTZMANN * $temperature" | bc -l)
    local denominator=$(echo "$density * $ELEMENTARY_CHARGE * $ELEMENTARY_CHARGE" | bc -l)
    local debye_sq=$(echo "$numerator / $denominator" | bc -l)
    local debye=$(echo "sqrt($debye_sq)" | bc -l)

    print_section "Results"
    print_success "Debye Length: $(format_scientific $debye) m"
    print_info "           = $(echo "scale=2; $debye * 1e6" | bc) µm"
    print_info "           = $(echo "scale=2; $debye * 1e9" | bc) nm"

    # Calculate plasma parameter
    local volume=$(echo "4/3 * 3.14159 * $debye_sq * $debye" | bc -l)
    local plasma_param=$(echo "$density * $volume" | bc -l)

    print_section "Plasma Criterion"
    print_info "Plasma Parameter (ΛD): $(format_scientific $plasma_param)"

    if (( $(echo "$plasma_param > 1" | bc -l) )); then
        print_success "ΛD >> 1: True plasma condition satisfied"
    else
        print_warning "ΛD < 1: Weakly ionized or non-plasma regime"
    fi

    echo ""
}

# Calculate plasma frequency
calc_plasma_frequency() {
    local density=${1:-1e18}

    print_section "Plasma Frequency Calculation"
    print_info "Electron Density: $(format_scientific $density) m⁻³"

    # ωpe = √(ne² / ε₀me)
    local numerator=$(echo "$density * $ELEMENTARY_CHARGE * $ELEMENTARY_CHARGE" | bc -l)
    local denominator=$(echo "$EPSILON_0 * $ELECTRON_MASS" | bc -l)
    local omega_sq=$(echo "$numerator / $denominator" | bc -l)
    local omega=$(echo "sqrt($omega_sq)" | bc -l)
    local freq=$(echo "$omega / (2 * 3.14159)" | bc -l)

    print_section "Results"
    print_success "Plasma Frequency (ωpe): $(format_scientific $omega) rad/s"
    print_success "Plasma Frequency (fpe): $(format_scientific $freq) Hz"
    print_info "                      = $(echo "scale=2; $freq / 1e9" | bc) GHz"

    echo ""
}

# Simulate plasma generation
generate_plasma() {
    local method=${1:-rf}
    local power=${2:-1000}
    local frequency=${3:-13.56e6}

    print_section "Plasma Generation Simulation"
    print_info "Method: $method"
    print_info "Power: $power W"
    print_info "Frequency: $(format_scientific $frequency) Hz"

    # Estimate plasma density
    local density_base=1e16
    if [ "$method" = "icp" ]; then
        density_base=1e17
    fi

    local density=$(echo "$density_base * sqrt($power / 1000)" | bc -l)

    # Estimate electron temperature
    local temp=$(echo "30000 * sqrt($power / 1000)" | bc -l)

    print_section "Estimated Plasma Parameters"
    print_success "Electron Density: $(format_scientific $density) m⁻³"
    print_success "Electron Temperature: $(printf "%.0f" $temp) K ($(echo "scale=2; $temp / 11604" | bc) eV)"
    print_info "Absorbed Power: $(echo "$power * 0.7" | bc) W (70% efficiency)"
    print_info "Uniformity: 85%"

    if [ "$method" = "rf" ] || [ "$method" = "icp" ]; then
        print_success "Plasma Status: STABLE (RF discharge)"
    else
        print_warning "Plasma Status: Check stability"
    fi

    echo ""
}

# Calculate fusion power
calc_fusion_power() {
    local reactor=${1:-tokamak}
    local density=${2:-1e20}
    local temp=${3:-150e6}

    print_section "Fusion Power Calculation"
    print_info "Reactor Type: $reactor"
    print_info "Plasma Density: $(format_scientific $density) m⁻³"
    print_info "Ion Temperature: $(format_scientific $temp) K ($(echo "scale=0; $temp * $BOLTZMANN / ($ELEMENTARY_CHARGE * 1000)" | bc -l) keV)"

    # D-T reaction parameters
    local T_keV=$(echo "scale=2; $temp * $BOLTZMANN / ($ELEMENTARY_CHARGE * 1000)" | bc -l)

    # Simplified <σv> calculation
    local sigma_v
    if (( $(echo "$T_keV < 10" | bc -l) )); then
        sigma_v=$(echo "1e-28 * $T_keV * $T_keV" | bc -l)
    elif (( $(echo "$T_keV > 100" | bc -l) )); then
        sigma_v=1e-22
    else
        sigma_v=$(echo "1e-24 * $T_keV * $T_keV / 100" | bc -l)
    fi

    # Fusion power density
    local n_D=$(echo "$density / 2" | bc -l)
    local n_T=$(echo "$density / 2" | bc -l)
    local E_fusion=$(echo "17.6e6 * $ELEMENTARY_CHARGE" | bc -l)
    local reaction_rate=$(echo "$n_D * $n_T * $sigma_v" | bc -l)

    # Volume estimate (ITER-scale)
    local volume
    if [ "$reactor" = "tokamak" ]; then
        volume=830  # ITER volume ~830 m³
    else
        volume=500  # Stellarator
    fi

    local fusion_power=$(echo "$reaction_rate * $E_fusion * $volume / 4" | bc -l)

    # Energy confinement time (simplified)
    local tau_E=3.5  # seconds (ITER target)

    # Triple product
    local triple_product=$(echo "$density * $T_keV * $tau_E" | bc -l)

    print_section "Fusion Performance"
    print_success "Fusion Power: $(format_scientific $fusion_power) W"
    print_info "            = $(echo "scale=0; $fusion_power / 1e6" | bc) MW"
    print_success "Reaction Rate: $(format_scientific $reaction_rate) reactions/m³/s"
    print_success "Confinement Time: $tau_E s"
    print_success "Triple Product: $(format_scientific $triple_product) keV·s/m³"

    # Check Lawson criterion
    if (( $(echo "$triple_product > 3e21" | bc -l) )); then
        print_success "Lawson Criterion: SATISFIED ✓"
    else
        print_warning "Lawson Criterion: NOT SATISFIED (need > 3×10²¹)"
    fi

    # Q factor
    local heating_power=50e6
    local q_factor=$(echo "$fusion_power / $heating_power" | bc -l)
    print_info "Q Factor: $(echo "scale=2; $q_factor" | bc)"

    if (( $(echo "$q_factor > 5" | bc -l) )); then
        print_success "Ignition: ACHIEVED 🔥"
    elif (( $(echo "$q_factor > 1" | bc -l) )); then
        print_warning "Ignition: Approaching (Q > 1)"
    else
        print_warning "Ignition: Not achieved (Q < 1)"
    fi

    echo ""
}

# Plasma diagnostics
run_diagnostics() {
    local probe=${1:-langmuir}

    print_section "Plasma Diagnostics"
    print_info "Diagnostic Type: $probe"

    if [ "$probe" = "langmuir" ]; then
        print_section "Langmuir Probe Analysis"
        print_info "Voltage Sweep: -100 to +100 V"
        print_info "Probe Area: 1 mm²"

        # Simulated results
        print_section "Results"
        print_success "Electron Density: 5.2×10¹⁷ m⁻³"
        print_success "Electron Temperature: 2.3 eV (26,700 K)"
        print_success "Plasma Potential: +12.5 V"
        print_success "Floating Potential: +6.8 V"
        print_info "Ion Saturation Current: -2.5 mA"
        print_info "Electron Saturation Current: +15.3 mA"

    elif [ "$probe" = "oes" ]; then
        print_section "Optical Emission Spectroscopy"
        print_info "Wavelength Range: 200-900 nm"
        print_info "Resolution: 0.5 nm"

        print_section "Detected Lines"
        print_success "Ar I: 750.4 nm (strong)"
        print_success "Ar I: 811.5 nm (medium)"
        print_success "Ar II: 434.8 nm (weak)"
        print_info "Estimated Te: 2.1 eV"
        print_info "Gas Temperature: 450 K"
    fi

    echo ""
}

# Ion thruster performance
calc_propulsion() {
    local type=${1:-ion}
    local power=${2:-10000}
    local propellant=${3:-xenon}

    print_section "Plasma Propulsion Calculation"
    print_info "Thruster Type: $type"
    print_info "Power: $power W"
    print_info "Propellant: $propellant"

    # Xenon properties
    local xenon_mass=$(echo "131.29 * $AMU" | bc -l)

    if [ "$type" = "ion" ]; then
        local voltage=1500
        local mass_flow=5  # mg/s

        # ve = √(2eV/m)
        local ve_sq=$(echo "2 * $ELEMENTARY_CHARGE * $voltage / $xenon_mass" | bc -l)
        local ve=$(echo "sqrt($ve_sq)" | bc -l)

        # F = ṁ × ve
        local mass_flow_kg=$(echo "$mass_flow / 1e6" | bc -l)
        local thrust=$(echo "$mass_flow_kg * $ve" | bc -l)

        # Isp = ve / g0
        local isp=$(echo "$ve / $STANDARD_GRAVITY" | bc -l)

        print_section "Ion Thruster Performance"
        print_success "Thrust: $(echo "scale=1; $thrust * 1000" | bc) mN"
        print_success "Specific Impulse: $(echo "scale=0; $isp" | bc) s"
        print_success "Exhaust Velocity: $(echo "scale=0; $ve" | bc) m/s"
        print_info "Efficiency: 65%"
        print_info "Power-to-Thrust: $(echo "scale=0; $power / ($thrust * 1000)" | bc) W/mN"

    elif [ "$type" = "hall" ]; then
        local voltage=300
        local mass_flow=8  # mg/s

        local ve_sq=$(echo "2 * $ELEMENTARY_CHARGE * $voltage * 0.8 / $xenon_mass" | bc -l)
        local ve=$(echo "sqrt($ve_sq)" | bc -l)
        local mass_flow_kg=$(echo "$mass_flow / 1e6" | bc -l)
        local thrust=$(echo "$mass_flow_kg * $ve" | bc -l)
        local isp=$(echo "$ve / $STANDARD_GRAVITY" | bc -l)

        print_section "Hall Thruster Performance"
        print_success "Thrust: $(echo "scale=1; $thrust * 1000" | bc) mN"
        print_success "Specific Impulse: $(echo "scale=0; $isp" | bc) s"
        print_success "Exhaust Velocity: $(echo "scale=0; $ve" | bc) m/s"
        print_info "Efficiency: 55%"
        print_info "Power-to-Thrust: $(echo "scale=0; $power / ($thrust * 1000)" | bc) W/mN"
    fi

    print_section "Mission Parameters"
    local consumption=$(echo "scale=3; $mass_flow * 86.4" | bc)  # g/day
    print_info "Propellant Consumption: $consumption g/day"
    print_info "100 kg propellant lifetime: $(echo "scale=0; 100000 / $mass_flow / 86400" | bc) days"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-qua-008 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  debye-length             Calculate Debye length"
    echo "    --density <m⁻³>        Electron density (default: 1e18)"
    echo "    --temperature <K>      Electron temperature (default: 10000)"
    echo ""
    echo "  plasma-frequency         Calculate plasma frequency"
    echo "    --density <m⁻³>        Electron density (default: 1e18)"
    echo ""
    echo "  generate-plasma          Simulate plasma generation"
    echo "    --method <type>        Method: rf, icp, dc (default: rf)"
    echo "    --power <W>            Power in watts (default: 1000)"
    echo "    --frequency <Hz>       RF frequency (default: 13.56e6)"
    echo ""
    echo "  fusion-power             Calculate fusion reactor power"
    echo "    --reactor <type>       Type: tokamak, stellarator (default: tokamak)"
    echo "    --density <m⁻³>        Plasma density (default: 1e20)"
    echo "    --temp <K>             Ion temperature (default: 150e6)"
    echo ""
    echo "  diagnostics              Run plasma diagnostics"
    echo "    --probe <type>         Probe: langmuir, oes (default: langmuir)"
    echo ""
    echo "  propulsion               Calculate thruster performance"
    echo "    --type <thruster>      Type: ion, hall (default: ion)"
    echo "    --power <W>            Power in watts (default: 10000)"
    echo "    --propellant <gas>     Propellant: xenon, krypton (default: xenon)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-qua-008 debye-length --density 1e18 --temperature 10000"
    echo "  wia-qua-008 generate-plasma --method icp --power 5000"
    echo "  wia-qua-008 fusion-power --reactor tokamak --density 1e20 --temp 150e6"
    echo "  wia-qua-008 propulsion --type ion --power 10000"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-QUA-008 Plasma Technology CLI Tool"
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
    debye-length)
        DENSITY=1e18
        TEMPERATURE=10000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --density) DENSITY=$2; shift 2 ;;
                --temperature) TEMPERATURE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_debye_length "$DENSITY" "$TEMPERATURE"
        ;;

    plasma-frequency)
        DENSITY=1e18

        while [[ $# -gt 0 ]]; do
            case $1 in
                --density) DENSITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_plasma_frequency "$DENSITY"
        ;;

    generate-plasma)
        METHOD="rf"
        POWER=1000
        FREQUENCY=13.56e6

        while [[ $# -gt 0 ]]; do
            case $1 in
                --method) METHOD=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                --frequency) FREQUENCY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_plasma "$METHOD" "$POWER" "$FREQUENCY"
        ;;

    fusion-power)
        REACTOR="tokamak"
        DENSITY=1e20
        TEMP=150e6

        while [[ $# -gt 0 ]]; do
            case $1 in
                --reactor) REACTOR=$2; shift 2 ;;
                --density) DENSITY=$2; shift 2 ;;
                --temp) TEMP=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_fusion_power "$REACTOR" "$DENSITY" "$TEMP"
        ;;

    diagnostics)
        PROBE="langmuir"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --probe) PROBE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        run_diagnostics "$PROBE"
        ;;

    propulsion)
        TYPE="ion"
        POWER=10000
        PROPELLANT="xenon"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                --propellant) PROPELLANT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_propulsion "$TYPE" "$POWER" "$PROPELLANT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-qua-008 help' for usage information"
        exit 1
        ;;
esac

exit 0
