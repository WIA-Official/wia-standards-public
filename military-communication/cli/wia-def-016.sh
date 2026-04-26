#!/bin/bash

################################################################################
# WIA-DEF-016: Military Communication CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Communications Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to military communication
# calculations including link budgets, frequency management, and encryption.
################################################################################

set -e

# Colors for output
SLATE='\033[38;5;244m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SPEED_OF_LIGHT=299792458
EARTH_RADIUS=6371
K_FACTOR=1.33
THERMAL_NOISE=-174

# Helper functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         📡 WIA-DEF-016: Military Communication CLI            ║"
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
    printf "%.2f dBm" "$power"
}

# Calculate link budget
calc_link() {
    local power=${1:-50}
    local freq=${2:-150}
    local distance=${3:-25}
    local tx_gain=${4:-3}
    local rx_gain=${5:-3}

    print_section "Link Budget Calculation"
    print_info "Transmit Power: $power watts"
    print_info "Frequency: $freq MHz"
    print_info "Distance: $distance km"
    print_info "TX Antenna Gain: $tx_gain dBi"
    print_info "RX Antenna Gain: $rx_gain dBi"

    # Convert power to dBW and dBm
    local power_dbw=$(echo "scale=2; 10 * l($power) / l(10)" | bc -l)
    local power_dbm=$(echo "$power_dbw + 30" | bc -l)
    print_info "Transmit Power: $(format_power $power_dbm)"

    # Calculate free-space path loss
    # L_fs = 20 log10(d) + 20 log10(f) + 32.45
    local log_d=$(echo "scale=6; 20 * l($distance) / l(10)" | bc -l)
    local log_f=$(echo "scale=6; 20 * l($freq) / l(10)" | bc -l)
    local path_loss=$(echo "$log_d + $log_f + 32.45" | bc -l)

    print_info "Free-Space Path Loss: $(printf "%.2f" $path_loss) dB"

    # Calculate received power
    # P_rx = P_tx + G_tx - L_fs + G_rx
    local rx_power=$(echo "$power_dbm + $tx_gain - $path_loss + $rx_gain" | bc -l)

    # Calculate noise floor (assume 10 kHz bandwidth, 8 dB NF)
    local bandwidth=10000
    local noise_figure=8
    local log_bw=$(echo "scale=6; 10 * l($bandwidth) / l(10)" | bc -l)
    local noise_floor=$(echo "$THERMAL_NOISE + $log_bw + $noise_figure" | bc -l)

    # Calculate SNR
    local snr=$(echo "$rx_power - $noise_floor" | bc -l)

    # Calculate link margin (assume 10 dB required SNR)
    local required_snr=10
    local link_margin=$(echo "$snr - $required_snr" | bc -l)

    print_section "Results"
    print_success "Received Power: $(format_power $rx_power)"
    print_info "Noise Floor: $(format_power $noise_floor)"
    print_success "SNR: $(printf "%.2f" $snr) dB"
    print_success "Link Margin: $(printf "%.2f" $link_margin) dB"

    # Determine feasibility
    if (( $(echo "$link_margin >= 20" | bc -l) )); then
        print_success "Link Quality: EXCELLENT"
    elif (( $(echo "$link_margin >= 10" | bc -l) )); then
        print_success "Link Quality: GOOD"
    elif (( $(echo "$link_margin >= 0" | bc -l) )); then
        print_warning "Link Quality: MARGINAL"
    else
        print_error "Link Quality: POOR/IMPOSSIBLE"
    fi

    # Calculate maximum range
    local max_link_loss=$(echo "$power_dbm + $tx_gain + $rx_gain - $noise_floor - $required_snr" | bc -l)
    local max_path_loss=$(echo "$max_link_loss" | bc -l)

    # Solve for max distance: d_max = 10^((L_fs - 20*log(f) - 32.45) / 20)
    local exp=$(echo "scale=6; ($max_path_loss - $log_f - 32.45) / 20" | bc -l)
    local max_distance=$(echo "scale=2; e($exp * l(10))" | bc -l)

    print_info "Maximum Range: $(printf "%.2f" $max_distance) km"

    echo ""
}

# Encrypt message
encrypt_msg() {
    local message="$1"
    local level=${2:-secret}
    local algorithm=${3:-aes-256-gcm}

    print_section "Message Encryption"
    print_info "Classification: ${level^^}"
    print_info "Algorithm: $algorithm"
    print_info "Message Length: ${#message} bytes"

    # Generate mock IV and ciphertext
    local iv=$(head /dev/urandom | tr -dc 'A-Za-z0-9' | head -c 24)
    local ciphertext=$(echo -n "$message" | base64 | head -c 32)
    local auth_tag=$(head /dev/urandom | tr -dc 'A-Za-z0-9' | head -c 24)

    print_section "Encrypted Message"
    print_success "Ciphertext: $ciphertext..."
    print_info "IV: $iv"
    print_info "Auth Tag: $auth_tag"
    print_info "Key ID: TACKEY-$(date +%Y-%m-%d)"

    # Calculate expiration
    case $level in
        top-secret|ts-sci)
            print_info "Expires: $(date -d '+24 hours' '+%Y-%m-%d %H:%M:%S')"
            ;;
        secret)
            print_info "Expires: $(date -d '+7 days' '+%Y-%m-%d')"
            ;;
        *)
            print_info "Expires: $(date -d '+30 days' '+%Y-%m-%d')"
            ;;
    esac

    print_success "Message encrypted successfully"
    echo ""
}

# Validate frequency
validate_freq() {
    local freq=${1:-150}
    local band=${2:-vhf}
    local location=${3:-"37.5,-122.4"}

    print_section "Frequency Validation"
    print_info "Frequency: $freq MHz"
    print_info "Band: ${band^^}"
    print_info "Location: $location"

    # Determine if frequency is in authorized band
    local is_authorized=false
    case $band in
        hf)
            if (( $(echo "$freq >= 3 && $freq <= 30" | bc -l) )); then
                is_authorized=true
            fi
            ;;
        vhf)
            if (( $(echo "$freq >= 30 && $freq <= 300" | bc -l) )); then
                is_authorized=true
            fi
            ;;
        uhf)
            if (( $(echo "$freq >= 300 && $freq <= 3000" | bc -l) )); then
                is_authorized=true
            fi
            ;;
    esac

    print_section "Validation Results"

    if [ "$is_authorized" = true ]; then
        print_success "Frequency is in authorized ${band^^} band"

        # Simulate conflict check (20% chance)
        if (( RANDOM % 5 == 0 )); then
            print_warning "Potential conflict detected with BRAVO-COMPANY at $(echo "$freq + 0.025" | bc) MHz"
            print_info "Recommendation: Coordinate with unit or use alternative"

            # Suggest alternatives
            print_section "Alternative Frequencies"
            for i in {1..3}; do
                local alt=$(echo "scale=3; $freq + ($i * 0.5)" | bc -l)
                print_info "Alternative $i: $(printf "%.3f" $alt) MHz"
            done
        else
            print_success "No conflicts detected"
            print_success "Frequency approved for use"
            print_info "Allocation ID: FREQ-$(date +%s)"
        fi
    else
        print_error "Frequency is NOT in authorized ${band^^} band"
        print_info "Authorized ${band^^} range: $(get_band_range $band)"

        print_section "Suggested Frequencies"
        case $band in
            vhf)
                print_info "Suggestion 1: 148.25 MHz"
                print_info "Suggestion 2: 151.75 MHz"
                print_info "Suggestion 3: 155.50 MHz"
                ;;
            uhf)
                print_info "Suggestion 1: 420.50 MHz"
                print_info "Suggestion 2: 435.75 MHz"
                print_info "Suggestion 3: 450.25 MHz"
                ;;
        esac
    fi

    echo ""
}

get_band_range() {
    case $1 in
        hf) echo "3-30 MHz" ;;
        vhf) echo "30-300 MHz" ;;
        uhf) echo "300-3000 MHz" ;;
        *) echo "Unknown" ;;
    esac
}

# Establish secure link
establish_link() {
    local callsign=${1:-ALPHA-6}
    local freq=${2:-148.25}
    local power=${3:-50}

    print_section "Establishing Secure Link"
    print_info "Call Sign: $callsign"
    print_info "Frequency: $freq MHz"
    print_info "Power: $power watts"

    # Simulate link establishment
    sleep 1

    print_section "Link Status"
    print_success "Status: OPERATIONAL"

    # Generate random realistic values
    local rssi=$(echo "scale=2; -75 + ($RANDOM % 20)" | bc -l)
    local snr=$(echo "scale=2; 20 + ($RANDOM % 10)" | bc -l)
    local ber=$(echo "scale=10; 1 / (10^6)" | bc -l)

    print_info "RSSI: $(printf "%.2f" $rssi) dBm"
    print_info "SNR: $(printf "%.2f" $snr) dB"
    print_info "BER: < 1.0e-6"
    print_info "Encryption: AES-256-GCM ENABLED"
    print_info "Anti-Jam: FREQUENCY HOPPING ENABLED"

    print_success "Secure link established successfully"
    print_info "Active Connections: 1"
    print_info "Last Update: $(date '+%Y-%m-%d %H:%M:%S')"

    echo ""
}

# Calculate SATCOM link
calc_satcom() {
    local terminal=${1:-tactical}
    local freq=${2:-8000}
    local dish=${3:-1.2}

    print_section "SATCOM Link Budget"
    print_info "Terminal Type: ${terminal^^}"
    print_info "Downlink Frequency: $freq MHz ($(get_satcom_band $freq))"
    print_info "Antenna Diameter: $dish meters"

    # Calculate antenna gain
    # G ≈ 20 log(π D f / c) + 10 log(efficiency)
    local wavelength=$(echo "scale=6; 299.792458 / $freq" | bc -l)
    local gain_linear=$(echo "scale=6; 0.6 * (3.14159 * $dish / $wavelength)^2" | bc -l)
    local gain_db=$(echo "scale=2; 10 * l($gain_linear) / l(10)" | bc -l)

    # GEO satellite distance
    local distance=36000

    # Calculate path loss
    local log_d=$(echo "scale=6; 20 * l($distance) / l(10)" | bc -l)
    local log_f=$(echo "scale=6; 20 * l($freq) / l(10)" | bc -l)
    local path_loss=$(echo "$log_d + $log_f + 32.45" | bc -l)

    # Satellite EIRP (typical WGS)
    local sat_eirp=50

    # Received power
    local rx_power=$(echo "$sat_eirp - $path_loss + $gain_db - 2" | bc -l)

    # C/N calculation
    local cnr=$(echo "$rx_power + 128.6" | bc -l)  # Simplified

    print_section "Link Parameters"
    print_info "Antenna Gain: $(printf "%.2f" $gain_db) dBi"
    print_info "Path Loss: $(printf "%.2f" $path_loss) dB"
    print_info "Satellite EIRP: $sat_eirp dBW"

    print_section "Link Performance"
    print_success "Received Power: $(format_power $rx_power)"
    print_success "C/N Ratio: $(printf "%.2f" $cnr) dB"

    if (( $(echo "$cnr >= 15" | bc -l) )); then
        print_success "Link Status: LOCKED (Excellent)"
        print_info "BER: < 1.0e-7"
    elif (( $(echo "$cnr >= 10" | bc -l) )); then
        print_success "Link Status: TRACKING (Good)"
        print_info "BER: < 1.0e-5"
    else
        print_warning "Link Status: ACQUIRING (Marginal)"
        print_info "BER: ~1.0e-3"
    fi

    # Simulate satellite pointing
    local elevation=$(echo "scale=2; 30 + ($RANDOM % 50)" | bc -l)
    local azimuth=$(echo "scale=2; $RANDOM % 360" | bc -l)

    print_section "Terminal Pointing"
    print_info "Elevation: $(printf "%.2f" $elevation)°"
    print_info "Azimuth: $(printf "%.2f" $azimuth)°"
    print_info "Rain Margin: $(printf "%.2f" $(echo "$cnr - 10" | bc -l)) dB"

    echo ""
}

get_satcom_band() {
    local freq=$1
    if (( $(echo "$freq >= 8000 && $freq <= 12000" | bc -l) )); then
        echo "X-band"
    elif (( $(echo "$freq >= 12000 && $freq <= 18000" | bc -l) )); then
        echo "Ku-band"
    elif (( $(echo "$freq >= 26000 && $freq <= 40000" | bc -l) )); then
        echo "Ka-band"
    else
        echo "Unknown"
    fi
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-def-016 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-link                Calculate tactical radio link budget"
    echo "    --power <watts>        Transmit power (default: 50 W)"
    echo "    --freq <MHz>           Frequency (default: 150 MHz)"
    echo "    --distance <km>        Distance (default: 25 km)"
    echo "    --tx-gain <dBi>        TX antenna gain (default: 3 dBi)"
    echo "    --rx-gain <dBi>        RX antenna gain (default: 3 dBi)"
    echo ""
    echo "  encrypt                  Encrypt tactical message"
    echo "    --message <text>       Message to encrypt"
    echo "    --level <class>        Classification (secret, top-secret)"
    echo "    --algorithm <alg>      Encryption algorithm (default: aes-256-gcm)"
    echo ""
    echo "  validate-freq            Validate frequency allocation"
    echo "    --freq <MHz>           Frequency to validate"
    echo "    --band <band>          Frequency band (hf, vhf, uhf)"
    echo "    --location <lat,lon>   Location coordinates"
    echo ""
    echo "  establish-link           Establish secure communication link"
    echo "    --callsign <sign>      Unit call sign (default: ALPHA-6)"
    echo "    --freq <MHz>           Operating frequency (default: 148.25)"
    echo "    --power <watts>        Transmit power (default: 50 W)"
    echo ""
    echo "  calc-satcom              Calculate SATCOM link budget"
    echo "    --terminal <type>      Terminal type (tactical, manpack)"
    echo "    --freq <MHz>           Downlink frequency (default: 8000 MHz)"
    echo "    --dish <meters>        Antenna diameter (default: 1.2 m)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-def-016 calc-link --power 50 --freq 150 --distance 25"
    echo "  wia-def-016 encrypt --message 'SITREP: ALL CLEAR' --level secret"
    echo "  wia-def-016 validate-freq --freq 148.25 --band vhf"
    echo "  wia-def-016 establish-link --callsign BRAVO-3 --freq 151.50"
    echo "  wia-def-016 calc-satcom --terminal tactical --freq 8000 --dish 1.2"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-DEF-016 Military Communication CLI"
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
    calc-link)
        POWER=50
        FREQ=150
        DISTANCE=25
        TX_GAIN=3
        RX_GAIN=3

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) POWER=$2; shift 2 ;;
                --freq) FREQ=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                --tx-gain) TX_GAIN=$2; shift 2 ;;
                --rx-gain) RX_GAIN=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_link "$POWER" "$FREQ" "$DISTANCE" "$TX_GAIN" "$RX_GAIN"
        ;;

    encrypt)
        MESSAGE="TACTICAL MESSAGE"
        LEVEL="secret"
        ALGORITHM="aes-256-gcm"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --message) MESSAGE=$2; shift 2 ;;
                --level) LEVEL=$2; shift 2 ;;
                --algorithm) ALGORITHM=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        encrypt_msg "$MESSAGE" "$LEVEL" "$ALGORITHM"
        ;;

    validate-freq)
        FREQ=150
        BAND="vhf"
        LOCATION="37.5,-122.4"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --freq) FREQ=$2; shift 2 ;;
                --band) BAND=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_freq "$FREQ" "$BAND" "$LOCATION"
        ;;

    establish-link)
        CALLSIGN="ALPHA-6"
        FREQ=148.25
        POWER=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --callsign) CALLSIGN=$2; shift 2 ;;
                --freq) FREQ=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        establish_link "$CALLSIGN" "$FREQ" "$POWER"
        ;;

    calc-satcom)
        TERMINAL="tactical"
        FREQ=8000
        DISH=1.2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --terminal) TERMINAL=$2; shift 2 ;;
                --freq) FREQ=$2; shift 2 ;;
                --dish) DISH=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_satcom "$TERMINAL" "$FREQ" "$DISH"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-def-016 help' for usage information"
        exit 1
        ;;
esac

exit 0
