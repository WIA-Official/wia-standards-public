#!/bin/bash

################################################################################
# WIA-DEF-006: Electronic Warfare CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Defense & Security Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to electronic warfare calculations
# including jamming power, spectrum analysis, and signal intelligence.
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
SPEED_OF_LIGHT=299792458
MIN_JS_NOISE=10
MIN_JS_DECEPTION=20

# Helper functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        📡 WIA-DEF-006: Electronic Warfare CLI Tool           ║"
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

    if (( $(echo "$power < 1" | bc -l) )); then
        printf "%.2f mW" "$(echo "$power * 1000" | bc -l)"
    elif (( $(echo "$power < 1000" | bc -l) )); then
        printf "%.2f W" "$power"
    else
        printf "%.2f kW" "$(echo "$power / 1000" | bc -l)"
    fi
}

# Calculate jamming power
calc_jamming() {
    local freq=${1:-2400}
    local distance=${2:-10000}
    local target_power=${3:-100}
    local js_ratio=${4:-20}
    local jammer_gain=${5:-10}
    local target_gain=${6:-3}

    print_section "Jamming Power Calculation"
    print_info "Target Frequency: $freq MHz"
    print_info "Target Distance: $distance meters"
    print_info "Target Power: $(format_power $target_power)"
    print_info "Required J/S Ratio: $js_ratio dB"

    # Convert dBi to linear
    local jammer_gain_linear=$(echo "e(l(10) * $jammer_gain / 10)" | bc -l)
    local target_gain_linear=$(echo "e(l(10) * $target_gain / 10)" | bc -l)
    local js_ratio_linear=$(echo "e(l(10) * $js_ratio / 10)" | bc -l)

    # Assume jammer at half distance (self-screening)
    local jammer_distance=$(echo "$distance / 2" | bc -l)

    # Calculate required jammer power
    # Pj = J/S × Ps × Gs × (Rj/Rs)² / Gj
    local dist_ratio=$(echo "$jammer_distance / $distance" | bc -l)
    local dist_ratio_sq=$(echo "$dist_ratio * $dist_ratio" | bc -l)
    local jammer_power=$(echo "$js_ratio_linear * $target_power * $target_gain_linear * $dist_ratio_sq / $jammer_gain_linear" | bc -l)

    # Calculate effective range
    local min_js=$(echo "$MIN_JS_NOISE" | bc -l)
    local min_js_linear=$(echo "e(l(10) * $min_js / 10)" | bc -l)
    local eff_range=$(echo "sqrt(($jammer_power * $jammer_gain_linear) / ($target_power * $target_gain_linear * $min_js_linear)) * $distance" | bc -l)

    print_section "Results"
    print_success "Required Jammer Power: $(format_power $jammer_power)"
    print_info "Effective Range: $(printf "%.0f" $eff_range) meters"
    print_info "Power Consumption: $(format_power $(echo "$jammer_power / 0.5" | bc -l))"

    # Feasibility
    if (( $(echo "$jammer_power < 1000" | bc -l) )); then
        print_success "Feasibility: POSSIBLE"
    elif (( $(echo "$jammer_power < 10000" | bc -l) )); then
        print_warning "Feasibility: DIFFICULT"
    else
        print_error "Feasibility: IMPRACTICAL"
    fi

    echo ""
}

# Analyze signal
analyze_signal() {
    local freq=${1:-5800}
    local bandwidth=${2:-20}
    local signal_type=${3:-radar}

    print_section "Signal Analysis"
    print_info "Frequency: $freq MHz"
    print_info "Bandwidth: $bandwidth MHz"
    print_info "Expected Type: $signal_type"

    # Classify signal based on parameters
    local modulation="Unknown"
    local classification="Unknown"
    local confidence=0

    if (( $(echo "$bandwidth < 0.01" | bc -l) )); then
        modulation="FM"
        classification="Communication"
        confidence=70
    elif (( $(echo "$bandwidth < 1" | bc -l) )); then
        modulation="OFDM"
        classification="Communication"
        confidence=80
    elif (( $(echo "$bandwidth > 10" | bc -l) )); then
        modulation="Chirp"
        classification="Radar"
        confidence=85
    else
        modulation="QAM"
        classification="Datalink"
        confidence=60
    fi

    print_section "Analysis Results"
    print_success "Modulation: $modulation"
    print_success "Classification: $classification"
    print_info "Confidence: $confidence%"

    # Determine frequency band
    local band="Unknown"
    if (( $(echo "$freq >= 3 && $freq < 30" | bc -l) )); then
        band="HF (3-30 MHz)"
    elif (( $(echo "$freq >= 30 && $freq < 300" | bc -l) )); then
        band="VHF (30-300 MHz)"
    elif (( $(echo "$freq >= 300 && $freq < 3000" | bc -l) )); then
        band="UHF (300-3000 MHz)"
    elif (( $(echo "$freq >= 3000 && $freq < 30000" | bc -l) )); then
        band="SHF (3-30 GHz)"
    elif (( $(echo "$freq >= 30000" | bc -l) )); then
        band="EHF (30-300 GHz)"
    fi

    print_info "Frequency Band: $band"

    echo ""
}

# Generate frequency hopping pattern
generate_fhss() {
    local channels=${1:-50}
    local dwell_time=${2:-100}
    local base_freq=${3:-2400}

    print_section "Frequency Hopping Pattern Generation"
    print_info "Number of Channels: $channels"
    print_info "Dwell Time: $dwell_time ms"
    print_info "Base Frequency: $base_freq MHz"

    # Calculate hop rate
    local hop_rate=$(echo "1000 / $dwell_time" | bc -l)
    print_info "Hop Rate: $(printf "%.1f" $hop_rate) hops/second"

    # Calculate processing gain
    local channel_spacing=1  # MHz
    local spread_bw=$(echo "$channels * $channel_spacing" | bc -l)
    local info_bw=$channel_spacing
    local pg=$(echo "10 * l($spread_bw / $info_bw) / l(10)" | bc -l)

    print_section "Results"
    print_success "Processing Gain: $(printf "%.1f" $pg) dB"
    print_info "Total Bandwidth: $(printf "%.0f" $spread_bw) MHz"

    # Generate sample sequence (first 10 channels)
    print_section "Sample Hopping Sequence (first 10 hops)"
    for i in {1..10}; do
        local channel=$((RANDOM % channels))
        local freq=$(echo "$base_freq + $channel * $channel_spacing" | bc -l)
        printf "${GRAY}  %2d. Channel %3d: %.1f MHz${RESET}\n" "$i" "$channel" "$freq"
    done

    echo ""
}

# Spectrum scan
spectrum_scan() {
    local start_freq=${1:-2400}
    local end_freq=${2:-2500}
    local rbw=${3:-1}

    print_section "Spectrum Scan"
    print_info "Start Frequency: $start_freq MHz"
    print_info "End Frequency: $end_freq MHz"
    print_info "Resolution Bandwidth: $rbw MHz"

    local total_bw=$(echo "$end_freq - $start_freq" | bc -l)
    local num_scans=$(echo "scale=0; $total_bw / $rbw" | bc)

    print_info "Total Bandwidth: $(printf "%.0f" $total_bw) MHz"
    print_info "Number of Scans: $num_scans"

    # Simulate signal detection
    local num_signals=$((RANDOM % 5 + 1))

    print_section "Detected Signals"
    for i in $(seq 1 $num_signals); do
        local freq=$(echo "$start_freq + $RANDOM * ($end_freq - $start_freq) / 32767" | bc -l)
        local power=$((RANDOM % 50 - 100))
        local bw=$(echo "$rbw * (0.1 + $RANDOM * 0.5 / 32767)" | bc -l)
        printf "${GREEN}  Signal %d: %.2f MHz, %d dBm, BW: %.2f MHz${RESET}\n" "$i" "$freq" "$power" "$bw"
    done

    # Calculate occupancy
    local occupancy=$((RANDOM % 30 + 10))
    print_section "Summary"
    print_info "Spectrum Occupancy: $occupancy%"

    echo ""
}

# Calculate path loss
calc_path_loss() {
    local distance=${1:-1000}
    local freq=${2:-2400}

    print_section "Path Loss Calculation"
    print_info "Distance: $distance meters ($(echo "scale=2; $distance / 1000" | bc) km)"
    print_info "Frequency: $freq MHz"

    # Path loss formula: PL = 20log(d) + 20log(f) + 32.45
    local dist_km=$(echo "scale=6; $distance / 1000" | bc -l)
    local pl=$(echo "20 * l($dist_km) / l(10) + 20 * l($freq) / l(10) + 32.45" | bc -l)

    print_section "Results"
    print_success "Free Space Path Loss: $(printf "%.2f" $pl) dB"

    # Calculate received power for example transmitter
    local tx_power=30  # dBm
    local tx_gain=3    # dBi
    local rx_gain=3    # dBi
    local rx_power=$(echo "$tx_power + $tx_gain + $rx_gain - $pl" | bc -l)

    print_info "Example Link Budget:"
    print_info "  Tx Power: $tx_power dBm"
    print_info "  Tx Gain: $tx_gain dBi"
    print_info "  Rx Gain: $rx_gain dBi"
    print_success "  Rx Power: $(printf "%.2f" $rx_power) dBm"

    echo ""
}

# RF Safety calculation
calc_rf_safety() {
    local freq=${1:-2400}
    local eirp=${2:-1000}

    print_section "RF Safety Calculation"
    print_info "Frequency: $freq MHz"
    print_info "EIRP: $(format_power $eirp)"

    # Calculate MPE
    local mpe
    if (( $(echo "$freq >= 0.3 && $freq <= 3" | bc -l) )); then
        mpe=$(echo "180 / ($freq * $freq)" | bc -l)
    elif (( $(echo "$freq > 3 && $freq <= 15" | bc -l) )); then
        mpe=1.0
    elif (( $(echo "$freq > 15 && $freq <= 300" | bc -l) )); then
        mpe=$(echo "$freq / 15" | bc -l)
    else
        mpe=1.0
    fi

    print_info "MPE: $(printf "%.2f" $mpe) mW/cm²"

    # Calculate safe distance
    # S = EIRP / (4πR²), R = √(EIRP / (4π × MPE))
    local mpe_wm2=$(echo "$mpe * 10" | bc -l)
    local pi=3.14159265359
    local safe_dist=$(echo "sqrt($eirp / (4 * $pi * $mpe_wm2))" | bc -l)

    print_section "Safety Parameters"
    print_success "Safe Distance: $(printf "%.1f" $safe_dist) meters"
    print_warning "Safety Zone (20% margin): $(printf "%.1f" $(echo "$safe_dist * 1.2" | bc -l)) meters"

    if (( $(echo "$mpe <= 5" | bc -l) )); then
        print_info "Exposure Time Limit: 6 minutes"
    else
        print_info "Exposure Time Limit: None (low power)"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-def-006 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-jamming             Calculate jamming power requirements"
    echo "    --freq <MHz>           Target frequency (default: 2400)"
    echo "    --distance <m>         Target distance (default: 10000)"
    echo "    --target-power <W>     Target power (default: 100)"
    echo "    --js-ratio <dB>        Required J/S ratio (default: 20)"
    echo ""
    echo "  analyze-signal           Analyze signal parameters"
    echo "    --freq <MHz>           Signal frequency (default: 5800)"
    echo "    --bandwidth <MHz>      Signal bandwidth (default: 20)"
    echo "    --type <type>          Expected signal type (default: radar)"
    echo ""
    echo "  generate-fhss            Generate frequency hopping pattern"
    echo "    --channels <n>         Number of channels (default: 50)"
    echo "    --dwell-time <ms>      Dwell time per channel (default: 100)"
    echo "    --base-freq <MHz>      Base frequency (default: 2400)"
    echo ""
    echo "  spectrum-scan            Perform spectrum scan"
    echo "    --start <MHz>          Start frequency (default: 2400)"
    echo "    --end <MHz>            End frequency (default: 2500)"
    echo "    --resolution <MHz>     Resolution bandwidth (default: 1)"
    echo ""
    echo "  calc-path-loss           Calculate free space path loss"
    echo "    --distance <m>         Distance (default: 1000)"
    echo "    --freq <MHz>           Frequency (default: 2400)"
    echo ""
    echo "  calc-rf-safety           Calculate RF safety parameters"
    echo "    --freq <MHz>           Frequency (default: 2400)"
    echo "    --eirp <W>             EIRP (default: 1000)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-def-006 calc-jamming --freq 2400 --distance 10000 --target-power 100"
    echo "  wia-def-006 analyze-signal --freq 5800 --bandwidth 20 --type radar"
    echo "  wia-def-006 generate-fhss --channels 50 --dwell-time 100"
    echo "  wia-def-006 spectrum-scan --start 2400 --end 2500 --resolution 1"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-DEF-006 Electronic Warfare CLI Tool"
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
    calc-jamming)
        FREQ=2400
        DISTANCE=10000
        TARGET_POWER=100
        JS_RATIO=20
        JAMMER_GAIN=10
        TARGET_GAIN=3

        while [[ $# -gt 0 ]]; do
            case $1 in
                --freq) FREQ=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                --target-power) TARGET_POWER=$2; shift 2 ;;
                --js-ratio) JS_RATIO=$2; shift 2 ;;
                --jammer-gain) JAMMER_GAIN=$2; shift 2 ;;
                --target-gain) TARGET_GAIN=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_jamming "$FREQ" "$DISTANCE" "$TARGET_POWER" "$JS_RATIO" "$JAMMER_GAIN" "$TARGET_GAIN"
        ;;

    analyze-signal)
        FREQ=5800
        BANDWIDTH=20
        TYPE="radar"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --freq) FREQ=$2; shift 2 ;;
                --bandwidth) BANDWIDTH=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_signal "$FREQ" "$BANDWIDTH" "$TYPE"
        ;;

    generate-fhss)
        CHANNELS=50
        DWELL_TIME=100
        BASE_FREQ=2400

        while [[ $# -gt 0 ]]; do
            case $1 in
                --channels) CHANNELS=$2; shift 2 ;;
                --dwell-time) DWELL_TIME=$2; shift 2 ;;
                --base-freq) BASE_FREQ=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_fhss "$CHANNELS" "$DWELL_TIME" "$BASE_FREQ"
        ;;

    spectrum-scan)
        START=2400
        END=2500
        RBW=1

        while [[ $# -gt 0 ]]; do
            case $1 in
                --start) START=$2; shift 2 ;;
                --end) END=$2; shift 2 ;;
                --resolution) RBW=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        spectrum_scan "$START" "$END" "$RBW"
        ;;

    calc-path-loss)
        DISTANCE=1000
        FREQ=2400

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) DISTANCE=$2; shift 2 ;;
                --freq) FREQ=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_path_loss "$DISTANCE" "$FREQ"
        ;;

    calc-rf-safety)
        FREQ=2400
        EIRP=1000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --freq) FREQ=$2; shift 2 ;;
                --eirp) EIRP=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_rf_safety "$FREQ" "$EIRP"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-def-006 help' for usage information"
        exit 1
        ;;
esac

exit 0
