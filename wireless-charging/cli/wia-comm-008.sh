#!/bin/bash

################################################################################
# WIA-COMM-008: Wireless Charging CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to wireless charging calculations
# including efficiency, coil design, FOD, alignment optimization, and EMF safety.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
MU_0=0.0000012566370614  # 4π × 10⁻⁷ H/m
PI=3.14159265359

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🔋 WIA-COMM-008: Wireless Charging CLI               ║"
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

print_value() {
    echo -e "  ${CYAN}$1:${RESET} $2"
}

# Parse power with unit
parse_power() {
    local input=$1
    local value
    local unit

    if [[ $input =~ ^([0-9.]+)([a-zA-Z]+)$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]}
    else
        value=$input
        unit="W"
    fi

    # Convert to watts
    case $unit in
        mW) echo "scale=10; $value / 1000" | bc -l ;;
        kW) echo "scale=10; $value * 1000" | bc -l ;;
        MW) echo "scale=10; $value * 1000000" | bc -l ;;
        W) echo "$value" ;;
        *) echo "$value" ;;
    esac
}

# Parse frequency with unit
parse_frequency() {
    local input=$1
    local value
    local unit

    if [[ $input =~ ^([0-9.]+)([a-zA-Z]+)$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]}
    else
        value=$input
        unit="Hz"
    fi

    # Convert to Hz
    case $unit in
        kHz) echo "scale=10; $value * 1000" | bc -l ;;
        MHz) echo "scale=10; $value * 1000000" | bc -l ;;
        GHz) echo "scale=10; $value * 1000000000" | bc -l ;;
        Hz) echo "$value" ;;
        *) echo "$value" ;;
    esac
}

# Parse distance with unit
parse_distance() {
    local input=$1
    local value
    local unit

    if [[ $input =~ ^([0-9.]+)([a-zA-Z]+)$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]}
    else
        value=$input
        unit="mm"
    fi

    # Convert to mm
    case $unit in
        cm) echo "scale=10; $value * 10" | bc -l ;;
        m) echo "scale=10; $value * 1000" | bc -l ;;
        mm) echo "$value" ;;
        *) echo "$value" ;;
    esac
}

# Calculate efficiency
calc_efficiency() {
    local input_power=""
    local output_power=""
    local distance=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --input) input_power="$2"; shift 2 ;;
            --output) output_power="$2"; shift 2 ;;
            --distance) distance="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$input_power" ] || [ -z "$output_power" ]; then
        print_error "Usage: wia-comm-008 calc-efficiency --input <power> --output <power> --distance <dist>"
        return 1
    fi

    print_section "Calculating Charging Efficiency"

    local p_in=$(parse_power "$input_power")
    local p_out=$(parse_power "$output_power")
    local dist=$(parse_distance "${distance:-5mm}")

    local efficiency=$(echo "scale=2; ($p_out / $p_in) * 100" | bc -l)
    local losses=$(echo "scale=2; $p_in - $p_out" | bc -l)

    print_value "Input Power" "${p_in} W"
    print_value "Output Power" "${p_out} W"
    print_value "Distance" "${dist} mm"
    print_value "Efficiency" "${efficiency}%"
    print_value "Power Loss" "${losses} W"

    if (( $(echo "$efficiency >= 85" | bc -l) )); then
        print_success "Rating: Excellent"
    elif (( $(echo "$efficiency >= 75" | bc -l) )); then
        print_success "Rating: Good"
    elif (( $(echo "$efficiency >= 65" | bc -l) )); then
        print_warning "Rating: Fair"
    else
        print_error "Rating: Poor"
    fi
}

# Design transmitter coil
design_coil() {
    local power=""
    local freq=""
    local diameter=""
    local turns=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --power) power="$2"; shift 2 ;;
            --freq|--frequency) freq="$2"; shift 2 ;;
            --diameter) diameter="$2"; shift 2 ;;
            --turns) turns="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$power" ] || [ -z "$freq" ]; then
        print_error "Usage: wia-comm-008 design-coil --power <power> --freq <freq> --diameter <dia> --turns <n>"
        return 1
    fi

    print_section "Designing Transmitter Coil"

    local p=$(parse_power "$power")
    local f=$(parse_frequency "$freq")
    local d=$(parse_distance "${diameter:-50mm}")
    local n="${turns:-20}"

    # Calculate inductance (simplified flat spiral coil)
    # L ≈ (μ₀ × N² × r²) / (8r + 11w)
    local radius=$(echo "scale=10; $d / 2000" | bc -l)  # Convert mm to m
    local inner_radius=$(echo "scale=10; $radius * 0.5" | bc -l)
    local width=$(echo "scale=10; $radius - $inner_radius" | bc -l)

    local inductance=$(echo "scale=6; ($MU_0 * $n * $n * $radius * $radius) / (8 * $radius + 11 * $width) * 1000000" | bc -l)

    # Estimate resistance (AWG 20 wire)
    local wire_length=$(echo "scale=6; 2 * $PI * $radius * $n" | bc -l)
    local resistance_per_m=0.033  # Ω/m for AWG 20
    local resistance=$(echo "scale=6; $wire_length * $resistance_per_m" | bc -l)

    # Calculate quality factor
    local q_factor=$(echo "scale=2; (2 * $PI * $f * $inductance / 1000000) / $resistance" | bc -l)

    # Max current
    local max_current=$(echo "scale=3; sqrt($p / $resistance)" | bc -l)

    print_value "Coil Diameter" "${d} mm"
    print_value "Coil Turns" "$n"
    print_value "Wire Gauge" "AWG 20 (Litz)"
    print_value "Inductance" "${inductance} μH"
    print_value "Resistance" "${resistance} Ω"
    print_value "Quality Factor (Q)" "$q_factor"
    print_value "Max Current" "${max_current} A"
    print_value "Operating Frequency" "$(echo "scale=0; $f / 1000" | bc -l) kHz"

    print_info ""
    print_info "Recommendations:"
    if (( $(echo "$q_factor < 100" | bc -l) )); then
        print_warning "  • Q factor is low. Consider using higher quality litz wire."
    else
        print_success "  • Q factor is good for efficient power transfer."
    fi
    print_info "  • Use ferrite shielding (0.5mm thick) to improve performance."
    print_info "  • Target coupling coefficient: k > 0.3 for good efficiency."
}

# Detect foreign objects
detect_fod() {
    local threshold=""
    local baseline=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --threshold) threshold="$2"; shift 2 ;;
            --baseline-impedance) baseline="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$threshold" ] || [ -z "$baseline" ]; then
        print_error "Usage: wia-comm-008 detect-fod --threshold <val> --baseline-impedance <ohms>"
        return 1
    fi

    print_section "Foreign Object Detection (FOD)"

    # Simulate FOD measurement
    local baseline_q="$baseline"
    local measured_q=$(echo "scale=2; $baseline_q * (1 - $threshold * 1.5)" | bc -l)
    local q_change=$(echo "scale=2; ($baseline_q - $measured_q) / $baseline_q * 100" | bc -l)

    print_value "Baseline Q Factor" "$baseline_q"
    print_value "Measured Q Factor" "$measured_q"
    print_value "Q Change" "${q_change}%"
    print_value "Detection Threshold" "$(echo "scale=0; $threshold * 100" | bc -l)%"

    if (( $(echo "$q_change > ($threshold * 100)" | bc -l) )); then
        print_error "Foreign Object Detected!"
        print_warning "Recommended Action: Stop charging immediately"
        print_info "  • Check charging pad for metal objects (coins, keys, etc.)"
        print_info "  • Remove any foreign objects before resuming charging"
    else
        print_success "No Foreign Object Detected"
        print_info "  • Charging surface is clear"
        print_info "  • Safe to continue power transfer"
    fi
}

# Optimize alignment
optimize_align() {
    local offset_x=""
    local offset_y=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --offset-x) offset_x="$2"; shift 2 ;;
            --offset-y) offset_y="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$offset_x" ] || [ -z "$offset_y" ]; then
        print_error "Usage: wia-comm-008 optimize-align --offset-x <mm> --offset-y <mm>"
        return 1
    fi

    print_section "Alignment Optimization"

    local x=$(parse_distance "$offset_x")
    local y=$(parse_distance "$offset_y")

    # Calculate offset magnitude
    local offset=$(echo "scale=3; sqrt($x * $x + $y * $y)" | bc -l)

    # Calculate alignment score (0-100)
    local max_offset=20  # mm
    local score=$(echo "scale=1; 100 - ($offset / $max_offset * 100)" | bc -l)
    if (( $(echo "$score < 0" | bc -l) )); then
        score=0
    fi

    print_value "Lateral Offset X" "${x} mm"
    print_value "Lateral Offset Y" "${y} mm"
    print_value "Total Offset" "${offset} mm"
    print_value "Alignment Score" "${score}/100"

    if (( $(echo "$score >= 90" | bc -l) )); then
        print_success "Alignment: Excellent"
    elif (( $(echo "$score >= 70" | bc -l) )); then
        print_success "Alignment: Good"
    elif (( $(echo "$score >= 50" | bc -l) )); then
        print_warning "Alignment: Fair"
    else
        print_error "Alignment: Poor"
    fi

    print_info ""
    print_info "Recommendations:"
    if (( $(echo "$x > $y" | bc -l) )); then
        if (( $(echo "$x > 0" | bc -l) )); then
            print_info "  • Move device LEFT by ${x} mm"
        else
            print_info "  • Move device RIGHT by $(echo "scale=1; -$x" | bc -l) mm"
        fi
    else
        if (( $(echo "$y > 0" | bc -l) )); then
            print_info "  • Move device DOWN by ${y} mm"
        else
            print_info "  • Move device UP by $(echo "scale=1; -$y" | bc -l) mm"
        fi
    fi
}

# Calculate EMF exposure
calc_emf() {
    local power=""
    local distance=""
    local freq=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --power) power="$2"; shift 2 ;;
            --distance) distance="$2"; shift 2 ;;
            --freq|--frequency) freq="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$power" ] || [ -z "$distance" ]; then
        print_error "Usage: wia-comm-008 calc-emf --power <power> --distance <dist> --freq <freq>"
        return 1
    fi

    print_section "EMF Exposure Calculation"

    local p=$(parse_power "$power")
    local d=$(parse_distance "$distance")
    local f=$(parse_frequency "${freq:-110kHz}")

    # Simplified magnetic field calculation
    # B ≈ (μ₀ * N * I * r²) / [2 * (r² + d²)^(3/2)]
    # Assume: N=20 turns, r=25mm, I=sqrt(P/R), R=1Ω

    local turns=20
    local radius=0.025  # meters
    local current=$(echo "scale=6; sqrt($p / 1)" | bc -l)
    local d_m=$(echo "scale=6; $d / 1000" | bc -l)

    local numerator=$(echo "scale=12; $MU_0 * $turns * $current * $radius * $radius" | bc -l)
    local denominator=$(echo "scale=12; 2 * (($radius * $radius + $d_m * $d_m)^1.5)" | bc -l)
    local b_field=$(echo "scale=3; ($numerator / $denominator) * 1000000" | bc -l)  # Convert to μT

    # ICNIRP limit for 110 kHz
    local b_limit=6.25  # μT

    print_value "Power" "${p} W"
    print_value "Distance" "${d} mm"
    print_value "Frequency" "$(echo "scale=0; $f / 1000" | bc -l) kHz"
    print_value "Magnetic Field (B)" "${b_field} μT"
    print_value "ICNIRP Limit" "${b_limit} μT"

    if (( $(echo "$b_field <= $b_limit" | bc -l) )); then
        print_success "EMF Compliant"
        local margin=$(echo "scale=1; (($b_limit - $b_field) / $b_limit) * 100" | bc -l)
        print_value "Safety Margin" "${margin}%"
    else
        print_error "EMF Exceeds Limit"
        print_warning "Recommended Actions:"
        print_info "  • Increase distance from charging pad"
        print_info "  • Reduce charging power"
        print_info "  • Add magnetic shielding (ferrite)"
    fi
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-008 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-efficiency       Calculate charging efficiency"
    echo "  design-coil           Design transmitter coil"
    echo "  detect-fod            Detect foreign objects"
    echo "  optimize-align        Optimize device alignment"
    echo "  calc-emf              Calculate EMF exposure"
    echo "  version               Show version"
    echo "  help                  Show this help message"
    echo ""
    echo "Examples:"
    echo "  ${CYAN}wia-comm-008 calc-efficiency --input 15W --output 12W --distance 5mm${RESET}"
    echo "  ${CYAN}wia-comm-008 design-coil --power 15W --freq 110kHz --diameter 50mm${RESET}"
    echo "  ${CYAN}wia-comm-008 detect-fod --threshold 0.1 --baseline-impedance 50${RESET}"
    echo "  ${CYAN}wia-comm-008 optimize-align --offset-x 2mm --offset-y 3mm${RESET}"
    echo "  ${CYAN}wia-comm-008 calc-emf --power 15W --distance 10mm --freq 110kHz${RESET}"
    echo ""
    echo "For detailed documentation, visit:"
    echo "  https://wiastandards.com/standards/wireless-charging"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    echo "WIA-COMM-008: Wireless Charging CLI v$VERSION"
    echo "弘益人間 (Benefit All Humanity)"
}

# Main command router
main() {
    if [ $# -eq 0 ]; then
        show_help
        return 0
    fi

    local command=$1
    shift

    case $command in
        calc-efficiency)
            calc_efficiency "$@"
            ;;
        design-coil)
            design_coil "$@"
            ;;
        detect-fod)
            detect_fod "$@"
            ;;
        optimize-align)
            optimize_align "$@"
            ;;
        calc-emf)
            calc_emf "$@"
            ;;
        version|--version|-v)
            show_version
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            return 1
            ;;
    esac
}

# Run main
main "$@"

# 弘益人間 (홍익인간) · Benefit All Humanity
