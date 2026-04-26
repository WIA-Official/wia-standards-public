#!/bin/bash

################################################################################
# WIA-COMM-007: Optical Communication CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to optical communication calculations
# including link budget, DWDM design, dispersion analysis, and transceiver modeling.
################################################################################

set -e

# Colors for output
BLUE='\033[0;94m'
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

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       💡 WIA-COMM-007: Optical Communication CLI             ║"
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

# Parse power with unit
parse_power() {
    local input=$1
    local value
    local unit

    if [[ $input =~ ^(-?[0-9.]+)(dBm|mW|W)?$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]:-dBm}
    else
        value=$input
        unit="dBm"
    fi

    # Convert to dBm
    case $unit in
        dBm) echo "$value" ;;
        mW) echo "scale=3; 10 * l($value) / l(10)" | bc -l ;;
        W) echo "scale=3; 10 * l($value * 1000) / l(10)" | bc -l ;;
        *) echo "$value" ;;
    esac
}

# Parse distance with unit
parse_distance() {
    local input=$1
    local value
    local unit

    if [[ $input =~ ^([0-9.]+)(m|km)?$ ]]; then
        value=${BASH_REMATCH[1]}
        unit=${BASH_REMATCH[2]:-km}
    else
        value=$input
        unit="km"
    fi

    # Convert to km
    case $unit in
        km) echo "$value" ;;
        m) echo "scale=6; $value / 1000" | bc -l ;;
        *) echo "$value" ;;
    esac
}

# Calculate link budget
link_budget() {
    local tx_power=${1:-0}
    local fiber_length=${2:-80}
    local fiber_loss=${3:-0.2}

    print_section "Optical Link Budget Calculation"

    # Parse inputs
    tx_power=$(parse_power "$tx_power")
    fiber_length=$(parse_distance "$fiber_length")

    print_info "TX Power: ${tx_power} dBm"
    print_info "Fiber Length: ${fiber_length} km"
    print_info "Fiber Loss: ${fiber_loss} dB/km"

    # Calculate losses
    local total_fiber_loss=$(echo "scale=3; $fiber_loss * $fiber_length" | bc -l)
    local connector_loss=1.0  # 2 connectors @ 0.5 dB each
    local total_loss=$(echo "scale=3; $total_fiber_loss + $connector_loss" | bc -l)

    # Calculate received power
    local rx_power=$(echo "scale=3; $tx_power - $total_loss" | bc -l)

    # Typical receiver sensitivity
    local rx_sensitivity=-28

    # Calculate margin
    local margin=$(echo "scale=3; $rx_power - $rx_sensitivity" | bc -l)

    print_section "Link Budget Results"
    print_info "Total Fiber Loss: ${total_fiber_loss} dB"
    print_info "Connector Loss: ${connector_loss} dB"
    print_info "Total Loss: ${total_loss} dB"
    print_success "Received Power: ${rx_power} dBm"
    print_info "Receiver Sensitivity: ${rx_sensitivity} dBm"

    if (( $(echo "$margin >= 3" | bc -l) )); then
        print_success "Link Margin: ${margin} dB (PASS)"
    elif (( $(echo "$margin >= 0" | bc -l) )); then
        print_warning "Link Margin: ${margin} dB (MARGINAL)"
    else
        print_error "Link Margin: ${margin} dB (FAIL)"
    fi

    echo ""
}

# Design DWDM system
design_dwdm() {
    local channels=${1:-96}
    local spacing=${2:-50GHz}
    local data_rate=${3:-100G}

    print_section "DWDM System Design"

    print_info "Number of Channels: $channels"
    print_info "Channel Spacing: $spacing"
    print_info "Data Rate per Channel: $data_rate"

    # Parse data rate
    local rate_value
    if [[ $data_rate =~ ^([0-9.]+)G$ ]]; then
        rate_value=${BASH_REMATCH[1]}
    else
        rate_value=$data_rate
    fi

    # Calculate total capacity
    local total_capacity=$(echo "scale=2; $channels * $rate_value" | bc -l)

    # ITU-T grid reference
    local ref_freq=193.1  # THz

    print_section "System Specifications"
    print_success "Total Capacity: ${total_capacity} Tbps"
    print_info "Reference Frequency: ${ref_freq} THz (1552.52 nm)"
    print_info "Wavelength Band: C-band (1530-1565 nm)"

    # Calculate channel frequencies
    local spacing_thz
    case $spacing in
        12.5GHz) spacing_thz=0.0125 ;;
        25GHz) spacing_thz=0.025 ;;
        50GHz) spacing_thz=0.05 ;;
        100GHz) spacing_thz=0.1 ;;
        *) spacing_thz=0.05 ;;
    esac

    print_section "Sample Channels"
    for i in 1 5 10 50 96; do
        if [ $i -le $channels ]; then
            local freq=$(echo "scale=4; $ref_freq + ($i - 1) * $spacing_thz" | bc -l)
            local wl=$(echo "scale=2; 299792.458 / $freq" | bc -l)
            print_info "Channel $i: ${freq} THz (${wl} nm)"
        fi
    done

    print_section "Applications"
    print_info "- Long-haul telecommunications"
    print_info "- Metro networks"
    print_info "- Submarine cable systems"
    print_info "- Data center interconnect"

    echo ""
}

# Analyze transceiver
analyze_transceiver() {
    local type=${1:-QSFP28}
    local wavelength=${2:-1310nm}
    local data_rate=${3:-100G}

    print_section "Optical Transceiver Analysis"

    print_info "Form Factor: $type"
    print_info "Wavelength: $wavelength"
    print_info "Data Rate: $data_rate"

    # Determine specifications based on type
    local tx_power
    local rx_sensitivity
    local max_reach
    local lanes

    case $type in
        SFP+)
            tx_power=-5
            rx_sensitivity=-14
            max_reach=10
            lanes=1
            ;;
        SFP28)
            tx_power=-3
            rx_sensitivity=-12
            max_reach=10
            lanes=1
            ;;
        QSFP28)
            tx_power=0
            rx_sensitivity=-10
            max_reach=10
            lanes=4
            ;;
        QSFP-DD)
            tx_power=0
            rx_sensitivity=-8
            max_reach=2
            lanes=8
            ;;
        *)
            tx_power=0
            rx_sensitivity=-12
            max_reach=10
            lanes=4
            ;;
    esac

    print_section "Transceiver Specifications"
    print_success "TX Power: ${tx_power} dBm"
    print_info "RX Sensitivity: ${rx_sensitivity} dBm"
    print_info "Maximum Reach: ${max_reach} km"
    print_info "Number of Lanes: $lanes"

    # Calculate per-lane data rate
    local rate_value
    if [[ $data_rate =~ ^([0-9.]+)G$ ]]; then
        rate_value=${BASH_REMATCH[1]}
    else
        rate_value=$data_rate
    fi

    local lane_rate=$(echo "scale=2; $rate_value / $lanes" | bc -l)
    print_info "Per-Lane Rate: ${lane_rate} Gbps"

    # Determine modulation
    local modulation
    if (( $(echo "$lane_rate <= 25" | bc -l) )); then
        modulation="NRZ"
    else
        modulation="PAM4"
    fi

    print_info "Modulation: $modulation"

    print_section "Standards Compliance"
    print_info "- IEEE 802.3 Ethernet"
    print_info "- MSA (Multi-Source Agreement)"
    print_info "- SFF specifications"

    echo ""
}

# Calculate dispersion
calculate_dispersion() {
    local fiber_length=${1:-100km}
    local wavelength=${2:-1550nm}
    local data_rate=${3:-10G}

    print_section "Chromatic Dispersion Calculation"

    # Parse inputs
    fiber_length=$(parse_distance "$fiber_length")

    # Standard SMF dispersion @ 1550 nm
    local dispersion_coeff=17  # ps/(nm·km)

    # Parse wavelength
    local wl_nm
    if [[ $wavelength =~ ^([0-9.]+)nm$ ]]; then
        wl_nm=${BASH_REMATCH[1]}
    else
        wl_nm=$wavelength
    fi

    # Adjust dispersion coefficient based on wavelength
    if (( $(echo "$wl_nm >= 1280 && $wl_nm <= 1340" | bc -l) )); then
        dispersion_coeff=0  # Zero dispersion @ O-band
    fi

    print_info "Fiber Length: ${fiber_length} km"
    print_info "Wavelength: ${wl_nm} nm"
    print_info "Dispersion Coefficient: ${dispersion_coeff} ps/(nm·km)"

    # Calculate total dispersion
    local total_dispersion=$(echo "scale=2; $dispersion_coeff * $fiber_length" | bc -l)

    # Typical spectral width
    local spectral_width=0.1  # nm

    # Calculate pulse broadening
    local pulse_broadening=$(echo "scale=2; $total_dispersion * $spectral_width" | bc -l)

    # Parse data rate
    local rate_value
    if [[ $data_rate =~ ^([0-9.]+)G$ ]]; then
        rate_value=${BASH_REMATCH[1]}
    else
        rate_value=$data_rate
    fi

    # Bit period in ps
    local bit_period=$(echo "scale=2; 1000 / $rate_value" | bc -l)

    print_section "Dispersion Analysis"
    print_success "Total Dispersion: ${total_dispersion} ps/nm"
    print_info "Pulse Broadening: ${pulse_broadening} ps"
    print_info "Bit Period: ${bit_period} ps"

    # Check if compensation needed
    local tolerance=$(echo "scale=2; $bit_period * 0.1" | bc -l)
    if (( $(echo "$pulse_broadening > $tolerance" | bc -l) )); then
        print_warning "Dispersion compensation recommended"

        # Calculate DCF length
        local dcf_dispersion=-80  # ps/(nm·km)
        local dcf_length=$(echo "scale=2; $total_dispersion / (-1 * $dcf_dispersion)" | bc -l)
        print_info "Required DCF: ${dcf_length} km"
    else
        print_success "Dispersion within tolerance"
    fi

    echo ""
}

# Coherent transmission simulation
coherent_transmission() {
    local modulation=${1:-DP-QPSK}
    local symbol_rate=${2:-32GBaud}
    local osnr=${3:-15dB}

    print_section "Coherent Optical Transmission"

    print_info "Modulation: $modulation"
    print_info "Symbol Rate: $symbol_rate"
    print_info "OSNR: $osnr"

    # Parse symbol rate
    local rate_value
    if [[ $symbol_rate =~ ^([0-9.]+)GBaud$ ]]; then
        rate_value=${BASH_REMATCH[1]}
    else
        rate_value=$symbol_rate
    fi

    # Parse OSNR
    local osnr_value
    if [[ $osnr =~ ^([0-9.]+)dB$ ]]; then
        osnr_value=${BASH_REMATCH[1]}
    else
        osnr_value=$osnr
    fi

    # Determine bits per symbol and spectral efficiency
    local bits_per_symbol
    local spectral_eff
    local required_osnr

    case $modulation in
        DP-QPSK)
            bits_per_symbol=4
            spectral_eff=2.0
            required_osnr=11
            ;;
        DP-16QAM)
            bits_per_symbol=8
            spectral_eff=4.0
            required_osnr=18
            ;;
        DP-64QAM)
            bits_per_symbol=12
            spectral_eff=6.0
            required_osnr=25
            ;;
        *)
            bits_per_symbol=4
            spectral_eff=2.0
            required_osnr=11
            ;;
    esac

    # Calculate data rate
    local data_rate=$(echo "scale=2; $bits_per_symbol * $rate_value" | bc -l)

    print_section "System Parameters"
    print_success "Bits per Symbol: $bits_per_symbol"
    print_info "Spectral Efficiency: ${spectral_eff} bit/s/Hz"
    print_info "Data Rate: ${data_rate} Gbps"

    # Check OSNR requirement
    print_section "OSNR Analysis"
    print_info "Required OSNR: ${required_osnr} dB (for BER = 10⁻³)"
    print_info "Available OSNR: ${osnr_value} dB"

    local osnr_margin=$(echo "scale=2; $osnr_value - $required_osnr" | bc -l)
    if (( $(echo "$osnr_margin >= 2" | bc -l) )); then
        print_success "OSNR Margin: ${osnr_margin} dB (PASS)"
    elif (( $(echo "$osnr_margin >= 0" | bc -l) )); then
        print_warning "OSNR Margin: ${osnr_margin} dB (MARGINAL)"
    else
        print_error "OSNR Margin: ${osnr_margin} dB (FAIL)"
    fi

    print_section "DSP Functions"
    print_info "- Chromatic dispersion compensation"
    print_info "- Polarization demultiplexing"
    print_info "- Carrier phase recovery"
    print_info "- Forward error correction (FEC)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-007 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  link-budget              Calculate optical link budget"
    echo "    --tx-power <value>     TX power (default: 0dBm)"
    echo "    --fiber-length <value> Fiber length (default: 80km)"
    echo "    --fiber-loss <value>   Attenuation (default: 0.2dB/km)"
    echo ""
    echo "  design-dwdm              Design DWDM system"
    echo "    --channels <value>     Number of channels (default: 96)"
    echo "    --spacing <value>      Channel spacing (default: 50GHz)"
    echo "    --data-rate <value>    Per-channel rate (default: 100G)"
    echo ""
    echo "  analyze-transceiver      Analyze optical transceiver"
    echo "    --type <value>         Form factor (default: QSFP28)"
    echo "    --wavelength <value>   Wavelength (default: 1310nm)"
    echo "    --data-rate <value>    Data rate (default: 100G)"
    echo ""
    echo "  dispersion               Calculate chromatic dispersion"
    echo "    --fiber-length <value> Fiber length (default: 100km)"
    echo "    --wavelength <value>   Wavelength (default: 1550nm)"
    echo "    --data-rate <value>    Data rate (default: 10G)"
    echo ""
    echo "  coherent                 Simulate coherent transmission"
    echo "    --modulation <value>   Format (default: DP-QPSK)"
    echo "    --symbol-rate <value>  Symbol rate (default: 32GBaud)"
    echo "    --osnr <value>         OSNR (default: 15dB)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-007 link-budget --tx-power 0dBm --fiber-length 80km"
    echo "  wia-comm-007 design-dwdm --channels 96 --spacing 50GHz"
    echo "  wia-comm-007 analyze-transceiver --type QSFP-DD --data-rate 400G"
    echo "  wia-comm-007 dispersion --fiber-length 100km --wavelength 1550nm"
    echo "  wia-comm-007 coherent --modulation DP-16QAM --symbol-rate 32GBaud"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-007 Optical Communication CLI Tool"
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
    link-budget)
        TX_POWER="0dBm"
        FIBER_LENGTH="80km"
        FIBER_LOSS="0.2"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --tx-power) TX_POWER=$2; shift 2 ;;
                --fiber-length) FIBER_LENGTH=$2; shift 2 ;;
                --fiber-loss) FIBER_LOSS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        link_budget "$TX_POWER" "$FIBER_LENGTH" "$FIBER_LOSS"
        ;;

    design-dwdm)
        CHANNELS="96"
        SPACING="50GHz"
        DATA_RATE="100G"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --channels) CHANNELS=$2; shift 2 ;;
                --spacing) SPACING=$2; shift 2 ;;
                --data-rate) DATA_RATE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_dwdm "$CHANNELS" "$SPACING" "$DATA_RATE"
        ;;

    analyze-transceiver)
        TYPE="QSFP28"
        WAVELENGTH="1310nm"
        DATA_RATE="100G"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --wavelength) WAVELENGTH=$2; shift 2 ;;
                --data-rate) DATA_RATE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_transceiver "$TYPE" "$WAVELENGTH" "$DATA_RATE"
        ;;

    dispersion)
        FIBER_LENGTH="100km"
        WAVELENGTH="1550nm"
        DATA_RATE="10G"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --fiber-length) FIBER_LENGTH=$2; shift 2 ;;
                --wavelength) WAVELENGTH=$2; shift 2 ;;
                --data-rate) DATA_RATE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calculate_dispersion "$FIBER_LENGTH" "$WAVELENGTH" "$DATA_RATE"
        ;;

    coherent)
        MODULATION="DP-QPSK"
        SYMBOL_RATE="32GBaud"
        OSNR="15dB"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --modulation) MODULATION=$2; shift 2 ;;
                --symbol-rate) SYMBOL_RATE=$2; shift 2 ;;
                --osnr) OSNR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        coherent_transmission "$MODULATION" "$SYMBOL_RATE" "$OSNR"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-007 help' for usage information"
        exit 1
        ;;
esac

exit 0

# 弘益人間 (홍익인간) · Benefit All Humanity
