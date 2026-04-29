#!/bin/bash

################################################################################
# WIA-COMM-004: 5G/6G Spectrum CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Standards Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to 5G/6G spectrum management
# operations including spectrum queries, link budgets, interference analysis,
# and regulatory compliance checks.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
MAGENTA='\033[0;35m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SPEED_OF_LIGHT=299792458
THERMAL_NOISE=-174  # dBm/Hz

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         📡 WIA-COMM-004: 5G/6G Spectrum CLI                   ║"
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

print_spectrum() {
    echo -e "${MAGENTA}📡 $1${RESET}"
}

# Query spectrum availability
query_spectrum() {
    local region=${1:-US}
    local band=${2:-n77}
    local location=${3:-"37.7749,-122.4194"}

    print_section "Spectrum Availability Query"
    print_info "Region: $region"
    print_info "Band: $band"
    print_info "Location: $location"

    print_spectrum "Querying spectrum database..."
    sleep 0.5

    # Simulate spectrum availability
    local avail_bandwidth=$((100 + RANDOM % 200))
    local interference_score=$((RANDOM % 40 + 10))
    local interference_level=$(echo "scale=1; $interference_score / 10" | bc)

    print_section "Available Spectrum"
    print_success "Band: $band"
    print_info "Frequency Range: 3300-4200 MHz (FR1)"
    print_info "Available Bandwidth: $avail_bandwidth MHz"
    print_info "Interference Score: $interference_level/10"

    if (( $(echo "$interference_level < 3" | bc -l) )); then
        print_success "Low interference - excellent conditions"
    elif (( $(echo "$interference_level < 6" | bc -l) )); then
        print_warning "Moderate interference - acceptable"
    else
        print_warning "High interference - mitigation recommended"
    fi

    local rec_power=$((43 + RANDOM % 6))
    print_info "Recommended Power: $rec_power dBm"

    echo ""
}

# Calculate link budget
calculate_link_budget() {
    local frequency=${1:-3600}
    local distance=${2:-1000}
    local power=${3:-46}

    print_section "Link Budget Calculation"
    print_info "Frequency: $frequency MHz"
    print_info "Distance: $distance meters"
    print_info "Transmit Power: $power dBm"

    # Transmitter
    print_section "Transmitter"
    local antenna_gain=17
    local cable_loss=3
    local eirp=$((power + antenna_gain - cable_loss))
    print_info "Antenna Gain: +$antenna_gain dBi"
    print_info "Cable Loss: -$cable_loss dB"
    print_success "EIRP: $eirp dBm"

    # Path Loss (UMa model)
    print_section "Path Loss"
    local fc=$(echo "scale=4; $frequency / 1000" | bc)
    local pl_fs=$(echo "scale=2; 28 + 22 * l($distance)/l(10) + 20 * l($fc)/l(10)" | bc -l)
    local shadow_fading=8
    local building_pen=15
    local total_pl=$(echo "scale=2; $pl_fs + $shadow_fading + $building_pen" | bc)

    print_info "Free Space Loss: ${pl_fs} dB"
    print_info "Shadow Fading: +$shadow_fading dB"
    print_info "Building Penetration: +$building_pen dB"
    print_success "Total Path Loss: ${total_pl} dB"

    # Receiver
    print_section "Receiver"
    local rx_gain=0
    local noise_fig=7
    local bandwidth=100  # MHz
    local bw_dbhz=$(echo "scale=2; 10 * l($bandwidth * 1000000)/l(10)" | bc -l)
    local noise_power=$(echo "scale=2; $THERMAL_NOISE + $bw_dbhz + $noise_fig" | bc)
    local sensitivity=$(echo "scale=2; $noise_power + 3" | bc)  # +3 dB SINR

    print_info "Antenna Gain: $rx_gain dBi"
    print_info "Noise Figure: $noise_fig dB"
    print_info "Noise Power: ${noise_power} dBm"
    print_info "Sensitivity (QPSK): ${sensitivity} dBm"

    # Link Budget
    print_section "Link Budget Analysis"
    local rx_power=$(echo "scale=2; $eirp - $total_pl + $rx_gain" | bc)
    local margin=$(echo "scale=2; $rx_power - $sensitivity - 11" | bc)  # 11 dB total margin

    print_info "Received Power: ${rx_power} dBm"
    print_info "Required Margin: 11 dB (SINR + Interference + Fade)"

    if (( $(echo "$margin >= 0" | bc -l) )); then
        print_success "Link Budget: ${margin} dB margin - LINK CLOSES ✓"
    else
        print_error "Link Budget: ${margin} dB margin - LINK FAILS ✗"
    fi

    echo ""
}

# Analyze spectrum usage
analyze_spectrum() {
    local band=${1:-n77}
    local bandwidth=${2:-100}
    local mimo=${3:-4x4}

    print_section "Spectrum Analysis"
    print_info "Band: $band"
    print_info "Bandwidth: $bandwidth MHz"
    print_info "MIMO Configuration: $mimo"

    print_spectrum "Analyzing spectrum utilization..."
    sleep 0.5

    # Parse MIMO
    local mimo_layers=${mimo%%x*}

    # Calculate PRBs
    local scs=30  # kHz
    local prb_bw=$(echo "scale=4; 12 * $scs / 1000" | bc)
    local num_prbs=$(echo "scale=0; $bandwidth / $prb_bw" | bc)

    print_section "Physical Resource Blocks"
    print_info "Subcarrier Spacing: $scs kHz"
    print_info "PRB Bandwidth: ${prb_bw} MHz"
    print_success "Number of PRBs: $num_prbs"

    # Spectral efficiency
    print_section "Spectral Efficiency"
    local modulation="256-QAM"
    local spec_eff=$(echo "scale=2; 5.5 * $mimo_layers * 0.75" | bc)  # Overhead factor
    local throughput=$(echo "scale=0; $bandwidth * $spec_eff" | bc)

    print_info "Modulation: $modulation"
    print_info "MIMO Layers: $mimo_layers"
    print_info "Spectral Efficiency: ${spec_eff} bps/Hz"
    print_success "Peak Throughput: $throughput Mbps"

    echo ""
}

# Simulate carrier aggregation
carrier_aggregation() {
    local bands=${1:-n77,n78,n257}
    local bandwidths=${2:-100,80,200}

    print_section "Carrier Aggregation"
    print_info "Bands: $bands"
    print_info "Bandwidths: $bandwidths MHz"

    IFS=',' read -ra BAND_ARRAY <<< "$bands"
    IFS=',' read -ra BW_ARRAY <<< "$bandwidths"

    print_spectrum "Configuring carriers..."
    sleep 0.3

    local total_bw=0
    local total_throughput=0

    print_section "Component Carriers"
    for i in "${!BAND_ARRAY[@]}"; do
        local band="${BAND_ARRAY[$i]}"
        local bw="${BW_ARRAY[$i]}"
        local throughput=$(echo "scale=0; $bw * 5.5 * 4 * 0.75" | bc)

        print_info "CC$((i+1)): $band @ $bw MHz → $throughput Mbps"

        total_bw=$((total_bw + bw))
        total_throughput=$(echo "scale=0; $total_throughput + $throughput" | bc)
    done

    print_section "Aggregation Results"
    print_success "Total Bandwidth: $total_bw MHz"
    print_success "Aggregated Throughput: $total_throughput Mbps"
    print_info "Spectral Efficiency: 5.5 bps/Hz (256-QAM, 4×4 MIMO)"

    local ca_type="inter-band"
    if [ ${#BAND_ARRAY[@]} -eq 1 ]; then
        ca_type="intra-band"
    fi
    print_info "CA Type: $ca_type"

    echo ""
}

# Check regulatory compliance
check_compliance() {
    local region=${1:-US}
    local band=${2:-n77}
    local power=${3:-46}

    print_section "Regulatory Compliance Check"
    print_info "Region: $region"
    print_info "Band: $band"
    print_info "Transmit Power: $power dBm"

    print_spectrum "Checking regulations..."
    sleep 0.5

    local max_power=68
    local regulations="3GPP TS 38.104"

    if [ "$region" == "US" ]; then
        if [[ "$band" == "n71" ]]; then
            max_power=68
            regulations="FCC Part 27"
        elif [[ "$band" =~ ^n2 ]]; then
            max_power=75
            regulations="FCC Part 30 (mmWave)"
        fi
    elif [ "$region" == "EU" ]; then
        max_power=68
        regulations="ECC Decision (18)06"
    fi

    print_section "Compliance Results"
    print_info "Applicable Regulation: $regulations"
    print_info "Maximum Allowed Power: $max_power dBm"
    print_info "Your Power Setting: $power dBm"

    if [ $power -le $max_power ]; then
        print_success "COMPLIANT ✓"
    else
        print_error "NON-COMPLIANT ✗"
        print_warning "Reduce power to $max_power dBm or below"
    fi

    echo ""
}

# Calculate spectrum efficiency
spectrum_efficiency() {
    local modulation=${1:-256QAM}
    local mimo=${2:-4x4}
    local bandwidth=${3:-100}

    print_section "Spectrum Efficiency Calculation"
    print_info "Modulation: $modulation"
    print_info "MIMO: $mimo"
    print_info "Bandwidth: $bandwidth MHz"

    # Map modulation to bits/symbol
    local bits_per_symbol=8  # 256-QAM default
    case $modulation in
        QPSK) bits_per_symbol=2 ;;
        16QAM) bits_per_symbol=4 ;;
        64QAM) bits_per_symbol=6 ;;
        256QAM) bits_per_symbol=8 ;;
        1024QAM) bits_per_symbol=10 ;;
    esac

    local mimo_layers=${mimo%%x*}
    local code_rate=0.75
    local overhead=0.75

    print_section "Calculation"
    print_info "Bits per Symbol: $bits_per_symbol"
    print_info "MIMO Layers: $mimo_layers"
    print_info "Code Rate: $code_rate"
    print_info "Overhead Factor: $overhead"

    local theoretical_eff=$(echo "scale=2; $bits_per_symbol * $code_rate * $mimo_layers" | bc)
    local practical_eff=$(echo "scale=2; $theoretical_eff * $overhead" | bc)
    local throughput=$(echo "scale=0; $bandwidth * $practical_eff" | bc)

    print_section "Results"
    print_info "Theoretical Efficiency: ${theoretical_eff} bps/Hz"
    print_success "Practical Efficiency: ${practical_eff} bps/Hz"
    print_success "Throughput: $throughput Mbps"

    # Required SINR
    local sinr=22  # for 256-QAM
    case $modulation in
        QPSK) sinr=5 ;;
        16QAM) sinr=12 ;;
        64QAM) sinr=18 ;;
        256QAM) sinr=22 ;;
        1024QAM) sinr=27 ;;
    esac
    print_info "Required SINR: $sinr dB"

    echo ""
}

# DSA spectrum request (CBRS)
dsa_request() {
    local mode=${1:-CBRS}
    local tier=${2:-GAA}
    local bandwidth=${3:-20}

    print_section "Dynamic Spectrum Access Request"
    print_info "Mode: $mode"
    print_info "Tier: $tier"
    print_info "Requested Bandwidth: $bandwidth MHz"

    print_spectrum "Contacting Spectrum Access System (SAS)..."
    sleep 0.5

    # Power limits
    local max_power=30  # GAA outdoor
    if [ "$tier" == "PAL" ]; then
        max_power=47
    fi

    print_section "SAS Response"

    local freq_start=$((3550 + RANDOM % 100))
    local freq_end=$((freq_start + bandwidth))
    local grant_id="GRANT-$(date +%s)-$(head /dev/urandom | tr -dc 'a-z0-9' | head -c 8)"

    print_success "Spectrum Grant Approved"
    print_info "Grant ID: $grant_id"
    print_info "Frequency: $freq_start-$freq_end MHz"
    print_info "Maximum EIRP: $max_power dBm"
    print_info "Channel Type: $tier"
    print_info "Heartbeat Interval: 240 seconds"
    print_info "Grant Expires: $(date -d '+1 hour' '+%Y-%m-%d %H:%M:%S')"

    echo ""
}

# Interference analysis
analyze_interference() {
    local band=${1:-n77}
    local location=${2:-"40.7128,-74.0060"}

    print_section "Interference Analysis"
    print_info "Band: $band"
    print_info "Location: $location"

    print_spectrum "Analyzing interference environment..."
    sleep 0.5

    local frequency=3600
    local co_channel=$((RANDOM % 30 + 50))
    local adjacent=$((co_channel - 10))
    local aggregate=$((RANDOM % 15 + 65))

    print_section "Interference Measurements"
    print_info "Co-Channel Interference: -$co_channel dB"
    print_info "Adjacent Channel Interference: -$adjacent dB"
    print_info "Aggregate Interference: -$aggregate dBm"

    local protection=$((aggregate - 10))
    print_section "Protection Analysis"
    print_info "Protection Ratio: $protection dB"

    if [ $protection -ge 10 ]; then
        print_success "Meets protection criteria (≥ 10 dB)"
    else
        print_warning "Below protection threshold"
        print_info "Mitigation: Reduce power, use beamforming, increase antenna height"
    fi

    echo ""
}

# Beamforming pattern analysis
beamforming_analysis() {
    local elements=${1:-64}
    local frequency=${2:-28000}
    local steering=${3:-30}

    print_section "Beamforming Analysis"
    print_info "Array Elements: $elements"
    print_info "Frequency: $frequency MHz"
    print_info "Steering Angle: $steering degrees"

    print_spectrum "Calculating beam pattern..."
    sleep 0.5

    local array_gain=$(echo "scale=1; 10 * l($elements)/l(10)" | bc -l)
    local beam_width=$(echo "scale=1; 120 / sqrt($elements)" | bc)
    local side_lobe=-20

    print_section "Beam Characteristics"
    print_success "Array Gain: ${array_gain} dB"
    print_info "Main Beam Width: ${beam_width}°"
    print_info "Side Lobe Level: $side_lobe dB"
    print_info "Front-to-Back Ratio: 30 dB"

    local scan_range=60
    print_section "Beam Steering"
    print_info "Scan Range: ±$scan_range°"
    print_info "Current Steering: $steering°"
    print_success "Beam formed successfully"

    echo ""
}

# TDD configuration
tdd_config() {
    local pattern=${1:-DDDSU}
    local periodicity=${2:-5}

    print_section "TDD Configuration"
    print_info "Pattern: $pattern"
    print_info "Periodicity: ${periodicity}ms"

    print_spectrum "Configuring TDD frame structure..."
    sleep 0.3

    local dl_slots=0
    local ul_slots=0
    local special=0

    for ((i=0; i<${#pattern}; i++)); do
        case ${pattern:$i:1} in
            D) dl_slots=$((dl_slots + 1)) ;;
            U) ul_slots=$((ul_slots + 1)) ;;
            S) special=$((special + 1)) ;;
        esac
    done

    local dl_ratio=$((dl_slots * 100 / ${#pattern}))
    local ul_ratio=$((ul_slots * 100 / ${#pattern}))

    print_section "Frame Analysis"
    print_info "Total Slots: ${#pattern}"
    print_info "Downlink Slots: $dl_slots (${dl_ratio}%)"
    print_info "Uplink Slots: $ul_slots (${ul_ratio}%)"
    print_info "Special Slots: $special"

    print_section "Configuration"
    print_success "TDD pattern configured"
    print_info "DL/UL Ratio: ${dl_ratio}:${ul_ratio}"
    print_info "Suitable for: "
    if [ $dl_ratio -gt 60 ]; then
        print_info "  - High downlink traffic (video streaming, downloads)"
    else
        print_info "  - Balanced traffic (web browsing, VoIP)"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-004 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  query                    Query spectrum availability"
    echo "    --region <code>        Region code (US, EU, CN, etc.)"
    echo "    --band <band>          5G NR band (n77, n78, n257, etc.)"
    echo "    --location <lat,lon>   Geographic location"
    echo ""
    echo "  link-budget              Calculate link budget"
    echo "    --frequency <MHz>      Carrier frequency"
    echo "    --distance <meters>    Link distance"
    echo "    --power <dBm>          Transmit power"
    echo ""
    echo "  analyze                  Analyze spectrum usage"
    echo "    --band <band>          5G NR band"
    echo "    --bandwidth <MHz>      Channel bandwidth"
    echo "    --mimo <config>        MIMO configuration (e.g., 4x4)"
    echo ""
    echo "  carrier-agg              Simulate carrier aggregation"
    echo "    --bands <list>         Comma-separated band list"
    echo "    --bw <list>            Comma-separated bandwidth list (MHz)"
    echo ""
    echo "  compliance               Check regulatory compliance"
    echo "    --region <code>        Region code"
    echo "    --band <band>          5G NR band"
    echo "    --power <dBm>          Transmit power"
    echo ""
    echo "  efficiency               Calculate spectrum efficiency"
    echo "    --modulation <type>    QPSK, 16QAM, 64QAM, 256QAM, 1024QAM"
    echo "    --mimo <config>        MIMO configuration"
    echo "    --bandwidth <MHz>      Channel bandwidth"
    echo ""
    echo "  dsa                      Dynamic spectrum access request"
    echo "    request                Request spectrum allocation"
    echo "      --mode <type>        DSA mode (CBRS, LSA, LAA)"
    echo "      --tier <level>       CBRS tier (PAL, GAA)"
    echo "      --bandwidth <MHz>    Requested bandwidth"
    echo ""
    echo "  interference             Analyze interference"
    echo "    --band <band>          5G NR band"
    echo "    --location <lat,lon>   Geographic location"
    echo ""
    echo "  beamforming              Analyze beam pattern"
    echo "    --elements <count>     Number of antenna elements"
    echo "    --frequency <MHz>      Carrier frequency"
    echo "    --steering <degrees>   Steering angle"
    echo ""
    echo "  tdd-config               Configure TDD pattern"
    echo "    --pattern <pattern>    TDD pattern (e.g., DDDSU)"
    echo "    --periodicity <ms>     Frame periodicity"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-004 query --region US --band n77 --location '37.7749,-122.4194'"
    echo "  wia-comm-004 link-budget --frequency 3600 --distance 1000 --power 46"
    echo "  wia-comm-004 analyze --band n77 --bandwidth 100 --mimo 4x4"
    echo "  wia-comm-004 carrier-agg --bands n77,n78,n257 --bw 100,80,200"
    echo "  wia-comm-004 compliance --region US --band n77 --power 46"
    echo "  wia-comm-004 efficiency --modulation 256QAM --mimo 4x4 --bandwidth 100"
    echo "  wia-comm-004 dsa request --mode CBRS --tier GAA --bandwidth 20"
    echo "  wia-comm-004 interference --band n77 --location '40.7128,-74.0060'"
    echo "  wia-comm-004 beamforming --elements 64 --frequency 28000 --steering 30"
    echo "  wia-comm-004 tdd-config --pattern DDDSU --periodicity 5"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-004 5G/6G Spectrum Management CLI Tool"
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
    query)
        REGION="US"
        BAND="n77"
        LOCATION="37.7749,-122.4194"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --region) REGION=$2; shift 2 ;;
                --band) BAND=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        query_spectrum "$REGION" "$BAND" "$LOCATION"
        ;;

    link-budget)
        FREQUENCY=3600
        DISTANCE=1000
        POWER=46

        while [[ $# -gt 0 ]]; do
            case $1 in
                --frequency) FREQUENCY=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calculate_link_budget "$FREQUENCY" "$DISTANCE" "$POWER"
        ;;

    analyze)
        BAND="n77"
        BANDWIDTH=100
        MIMO="4x4"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --band) BAND=$2; shift 2 ;;
                --bandwidth) BANDWIDTH=$2; shift 2 ;;
                --mimo) MIMO=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_spectrum "$BAND" "$BANDWIDTH" "$MIMO"
        ;;

    carrier-agg)
        BANDS="n77,n78,n257"
        BANDWIDTHS="100,80,200"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --bands) BANDS=$2; shift 2 ;;
                --bw) BANDWIDTHS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        carrier_aggregation "$BANDS" "$BANDWIDTHS"
        ;;

    compliance)
        REGION="US"
        BAND="n77"
        POWER=46

        while [[ $# -gt 0 ]]; do
            case $1 in
                --region) REGION=$2; shift 2 ;;
                --band) BAND=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        check_compliance "$REGION" "$BAND" "$POWER"
        ;;

    efficiency)
        MODULATION="256QAM"
        MIMO="4x4"
        BANDWIDTH=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --modulation) MODULATION=$2; shift 2 ;;
                --mimo) MIMO=$2; shift 2 ;;
                --bandwidth) BANDWIDTH=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        spectrum_efficiency "$MODULATION" "$MIMO" "$BANDWIDTH"
        ;;

    dsa)
        SUBCMD=${1:-request}
        shift || true

        MODE="CBRS"
        TIER="GAA"
        BANDWIDTH=20

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mode) MODE=$2; shift 2 ;;
                --tier) TIER=$2; shift 2 ;;
                --bandwidth) BANDWIDTH=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        dsa_request "$MODE" "$TIER" "$BANDWIDTH"
        ;;

    interference)
        BAND="n77"
        LOCATION="40.7128,-74.0060"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --band) BAND=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_interference "$BAND" "$LOCATION"
        ;;

    beamforming)
        ELEMENTS=64
        FREQUENCY=28000
        STEERING=30

        while [[ $# -gt 0 ]]; do
            case $1 in
                --elements) ELEMENTS=$2; shift 2 ;;
                --frequency) FREQUENCY=$2; shift 2 ;;
                --steering) STEERING=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        beamforming_analysis "$ELEMENTS" "$FREQUENCY" "$STEERING"
        ;;

    tdd-config)
        PATTERN="DDDSU"
        PERIODICITY=5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --pattern) PATTERN=$2; shift 2 ;;
                --periodicity) PERIODICITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        tdd_config "$PATTERN" "$PERIODICITY"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-004 help' for usage information"
        exit 1
        ;;
esac

exit 0
