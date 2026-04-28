#!/bin/bash

################################################################################
# WIA-COMM-005: Satellite Internet CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communications Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to satellite internet calculations
# including link budgets, Doppler shift, handover prediction, and orbital mechanics.
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
SPEED_OF_LIGHT=299792458  # m/s
SPEED_OF_LIGHT_KM=299792.458  # km/s
EARTH_RADIUS=6371  # km
MU_EARTH=398600.4418  # km³/s² (Standard gravitational parameter)
BOLTZMANN=1.380649e-23  # J/K
PI=3.14159265359

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🛰️ WIA-COMM-005: Satellite Internet CLI             ║"
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

format_frequency() {
    local freq=$1

    if (( $(echo "$freq < 1" | bc -l) )); then
        printf "%.2f MHz" "$(echo "$freq * 1000" | bc -l)"
    elif (( $(echo "$freq < 1000" | bc -l) )); then
        printf "%.2f GHz" "$freq"
    else
        printf "%.2f THz" "$(echo "$freq / 1000" | bc -l)"
    fi
}

# Calculate link budget
calc_link_budget() {
    local distance=${1:-600}
    local frequency=${2:-28}
    local tx_power=${3:-10}
    local tx_gain=${4:-35}
    local rx_gain=${5:-35}
    local bandwidth=${6:-250}

    print_section "Link Budget Calculation"
    print_info "Distance: $distance km"
    print_info "Frequency: $(format_frequency $frequency)"
    print_info "TX Power: $tx_power W"
    print_info "TX Gain: $tx_gain dBi"
    print_info "RX Gain: $rx_gain dBi"
    print_info "Bandwidth: $bandwidth MHz"

    # Convert TX power to dBW
    local tx_power_dbw=$(echo "scale=2; 10 * l($tx_power) / l(10)" | bc -l)
    print_info "TX Power (dBW): $tx_power_dbw dBW"

    # Calculate EIRP
    local eirp=$(echo "scale=2; $tx_power_dbw + $tx_gain" | bc -l)
    print_success "EIRP: $eirp dBW"

    # Calculate free-space path loss
    # L = 20 log₁₀(d) + 20 log₁₀(f) + 92.45
    local path_loss=$(echo "scale=2; 20 * l($distance) / l(10) + 20 * l($frequency) / l(10) + 92.45" | bc -l)
    print_info "Free-Space Path Loss: $path_loss dB"

    # Total losses (including atmospheric, rain fade, etc.)
    local atm_loss=1
    local rain_fade=3
    local pol_loss=0.5
    local point_loss=0.5
    local total_loss=$(echo "scale=2; $path_loss + $atm_loss + $rain_fade + $pol_loss + $point_loss" | bc -l)
    print_info "Total Losses: $total_loss dB"

    # Received power (dBW)
    local rx_power_dbw=$(echo "scale=2; $eirp + $rx_gain - $total_loss" | bc -l)
    local rx_power_dbm=$(echo "scale=2; $rx_power_dbw + 30" | bc -l)
    print_success "Received Power: $rx_power_dbm dBm"

    # Calculate noise power
    # N = k × T × B
    local noise_temp=250  # K (typical user terminal)
    local bandwidth_hz=$(echo "$bandwidth * 1e6" | bc -l)
    local noise_power_watts=$(echo "scale=10; $BOLTZMANN * $noise_temp * $bandwidth_hz" | bc -l)
    local noise_power_dbm=$(echo "scale=2; 10 * l($noise_power_watts) / l(10) + 30" | bc -l)
    print_info "Noise Power: $noise_power_dbm dBm"

    # Calculate SNR
    local snr=$(echo "scale=2; $rx_power_dbm - $noise_power_dbm" | bc -l)
    print_success "SNR: $snr dB"

    # Calculate capacity (Shannon)
    # C = B × log₂(1 + SNR)
    local snr_linear=$(echo "e($snr * l(10) / 10)" | bc -l)
    local capacity_bps=$(echo "scale=2; $bandwidth_hz * l(1 + $snr_linear) / l(2)" | bc -l)
    local capacity_mbps=$(echo "scale=2; $capacity_bps / 1e6" | bc -l)
    print_success "Theoretical Capacity: $capacity_mbps Mbps"

    # Link margin
    local required_snr=10
    local link_margin=$(echo "scale=2; $snr - $required_snr" | bc -l)

    if (( $(echo "$link_margin > 0" | bc -l) )); then
        print_success "Link Margin: $link_margin dB (FEASIBLE)"
    else
        print_warning "Link Margin: $link_margin dB (INSUFFICIENT)"
    fi
}

# Calculate Doppler shift
calc_doppler() {
    local velocity=${1:-7.5}  # km/s (default LEO velocity)
    local frequency=${2:-28}  # GHz
    local angle=${3:-0}  # degrees (0 = overhead, 90 = horizon)

    print_section "Doppler Shift Calculation"
    print_info "Satellite Velocity: $velocity km/s"
    print_info "Carrier Frequency: $(format_frequency $frequency)"
    print_info "Angle from Zenith: $angle degrees"

    # Convert to radians
    local angle_rad=$(echo "scale=10; $angle * $PI / 180" | bc -l)
    local cos_angle=$(echo "scale=10; c($angle_rad)" | bc -l)

    # Δf = (v/c) × f₀ × cos(θ)
    local frequency_hz=$(echo "$frequency * 1e9" | bc -l)
    local velocity_ms=$(echo "$velocity * 1000" | bc -l)
    local doppler_shift=$(echo "scale=2; ($velocity_ms / $SPEED_OF_LIGHT) * $frequency_hz * $cos_angle" | bc -l)
    local doppler_shift_khz=$(echo "scale=2; $doppler_shift / 1000" | bc -l)

    print_success "Doppler Shift: $doppler_shift_khz kHz"

    # Corrected frequency
    local corrected_freq=$(echo "scale=6; $frequency_hz - $doppler_shift" | bc -l)
    local corrected_freq_ghz=$(echo "scale=6; $corrected_freq / 1e9" | bc -l)
    print_info "Corrected Frequency: $corrected_freq_ghz GHz"

    # Maximum Doppler (at horizon, angle = 90°)
    if [ "$angle" = "0" ]; then
        local max_doppler=$(echo "scale=2; ($velocity_ms / $SPEED_OF_LIGHT) * $frequency_hz / 1000" | bc -l)
        print_info "Maximum Doppler (overhead): ±$max_doppler kHz"
    fi
}

# Predict satellite visibility
predict_visibility() {
    local lat=${1:-37.7749}
    local lon=${2:-122.4194}
    local altitude=${3:-550}
    local duration=${4:-3600}

    print_section "Satellite Visibility Prediction"
    print_info "Location: $lat°N, $lon°E"
    print_info "Satellite Altitude: $altitude km"
    print_info "Prediction Duration: $duration seconds"

    # Calculate orbital period
    local r=$(echo "$EARTH_RADIUS + $altitude" | bc -l)
    local period=$(echo "scale=2; 2 * $PI * sqrt($r^3 / $MU_EARTH) / 60" | bc -l)
    print_info "Orbital Period: $period minutes"

    # Calculate visibility duration per pass
    local min_elevation=25  # degrees
    local min_elev_rad=$(echo "scale=10; $min_elevation * $PI / 180" | bc -l)
    local cos_min_elev=$(echo "scale=10; c($min_elev_rad)" | bc -l)
    local visibility_angle=$(echo "scale=10; a($EARTH_RADIUS * $cos_min_elev / ($EARTH_RADIUS + $altitude))" | bc -l)
    local visibility_duration=$(echo "scale=2; 2 * $visibility_angle * $period / (2 * $PI)" | bc -l)

    print_success "Visibility per Pass: $visibility_duration minutes"

    # Number of passes in duration
    local period_seconds=$(echo "$period * 60" | bc -l)
    local num_passes=$(echo "scale=0; $duration / $period_seconds + 1" | bc -l)

    print_info "Number of Passes: $num_passes"

    # Calculate orbital velocity
    local velocity=$(echo "scale=2; sqrt($MU_EARTH / $r)" | bc -l)
    print_info "Orbital Velocity: $velocity km/s"
}

# Calculate handover time
calc_handover() {
    local altitude=${1:-550}
    local elevation=${2:-45}
    local min_elevation=${3:-25}

    print_section "Handover Prediction"
    print_info "Satellite Altitude: $altitude km"
    print_info "Current Elevation: $elevation degrees"
    print_info "Minimum Elevation: $min_elevation degrees"

    # Calculate angular velocity
    local r=$(echo "$EARTH_RADIUS + $altitude" | bc -l)
    local velocity=$(echo "scale=6; sqrt($MU_EARTH / $r)" | bc -l)
    local angular_velocity=$(echo "scale=10; $velocity / $r" | bc -l)

    # Convert elevations to radians
    local elev_rad=$(echo "scale=10; $elevation * $PI / 180" | bc -l)
    local min_elev_rad=$(echo "scale=10; $min_elevation * $PI / 180" | bc -l)

    # Time until handover (simplified)
    local angle_diff=$(echo "scale=10; $elev_rad - $min_elev_rad" | bc -l)
    if (( $(echo "$angle_diff < 0" | bc -l) )); then
        angle_diff=0
    fi
    local time_remaining=$(echo "scale=2; $angle_diff / $angular_velocity" | bc -l)

    print_success "Time Until Handover: $time_remaining seconds"
    print_info "Handover Type: Intra-plane"
    print_info "Expected Interruption: ~50 ms"
}

# Optimize beam steering
optimize_beams() {
    local num_users=${1:-1000}
    local coverage_area=${2:-500}
    local beam_width=${3:-2.5}
    local num_beams=${4:-64}

    print_section "Beam Steering Optimization"
    print_info "Number of Users: $num_users"
    print_info "Coverage Area: $coverage_area km²"
    print_info "Beam Width: $beam_width degrees"
    print_info "Number of Beams: $num_beams"

    # Calculate beam coverage area
    local altitude=550
    local beam_radius=$(echo "scale=4; $altitude * s($beam_width / 2 * $PI / 180) / c($beam_width / 2 * $PI / 180)" | bc -l)
    local beam_area=$(echo "scale=2; $PI * $beam_radius * $beam_radius" | bc -l)

    print_info "Beam Coverage Area: $beam_area km²"

    # Users per beam
    local users_per_beam=$(echo "scale=0; ($num_users + $num_beams - 1) / $num_beams" | bc)
    print_success "Users per Beam: $users_per_beam"

    # Coverage efficiency
    local total_beam_coverage=$(echo "scale=4; $num_beams * $beam_area" | bc -l)
    local efficiency=$(echo "scale=4; $total_beam_coverage / $coverage_area" | bc -l)
    if (( $(echo "$efficiency > 1" | bc -l) )); then
        efficiency=1.0
    fi
    local efficiency_pct=$(echo "scale=2; $efficiency * 100" | bc -l)

    print_success "Coverage Efficiency: $efficiency_pct%"
}

# Check orbital debris compliance
debris_check() {
    local altitude=${1:-550}
    local inclination=${2:-53}

    print_section "Orbital Debris Compliance Check"
    print_info "Altitude: $altitude km"
    print_info "Inclination: $inclination degrees"

    # 25-year rule check
    if (( $(echo "$altitude < 600" | bc -l) )); then
        print_success "25-Year Rule: COMPLIANT (passive deorbit < 5 years)"
        print_info "Deorbit Strategy: Passive atmospheric drag"
    elif (( $(echo "$altitude < 2000" | bc -l) )); then
        print_warning "25-Year Rule: Requires active deorbit"
        print_info "Deorbit Strategy: Active propulsion required"
    else
        print_error "25-Year Rule: HIGH RISK - Requires active deorbit within 25 years"
        print_info "Deorbit Strategy: Active propulsion + precise control"
    fi

    # Debris density check (simplified)
    if (( $(echo "$altitude > 700 && $altitude < 900" | bc -l) )); then
        print_warning "Debris Density: HIGH (700-900 km is a congested zone)"
    else
        print_success "Debris Density: ACCEPTABLE"
    fi

    # Inclination risk
    if (( $(echo "$inclination > 70" | bc -l) )); then
        print_warning "Inclination Risk: ELEVATED (high-inclination orbits more congested)"
    else
        print_success "Inclination Risk: NOMINAL"
    fi
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-005 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  link-budget       Calculate RF link budget"
    echo "  doppler           Calculate Doppler shift"
    echo "  predict           Predict satellite visibility"
    echo "  handover          Calculate handover timing"
    echo "  beam-steer        Optimize beam steering"
    echo "  debris-check      Check orbital debris compliance"
    echo "  version           Show version"
    echo "  help              Show this help"
    echo ""
    echo "Examples:"
    echo "  wia-comm-005 link-budget --distance 600 --frequency 28 --power 10"
    echo "  wia-comm-005 doppler --velocity 7.5 --frequency 28 --angle 0"
    echo "  wia-comm-005 predict --lat 37.7749 --lon -122.4194 --altitude 550"
    echo "  wia-comm-005 handover --altitude 550 --elevation 45"
    echo "  wia-comm-005 beam-steer --users 1000 --coverage-area 500"
    echo "  wia-comm-005 debris-check --altitude 550 --inclination 53"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
}

# Main command dispatcher
main() {
    case "${1:-help}" in
        link-budget)
            print_header
            calc_link_budget "${2}" "${3}" "${4}" "${5}" "${6}" "${7}"
            ;;
        doppler)
            print_header
            calc_doppler "${2}" "${3}" "${4}"
            ;;
        predict)
            print_header
            predict_visibility "${2}" "${3}" "${4}" "${5}"
            ;;
        handover)
            print_header
            calc_handover "${2}" "${3}" "${4}"
            ;;
        beam-steer)
            print_header
            optimize_beams "${2}" "${3}" "${4}" "${5}"
            ;;
        debris-check)
            print_header
            debris_check "${2}" "${3}"
            ;;
        version)
            echo "WIA-COMM-005 Satellite Internet CLI v$VERSION"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac

    echo ""
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${GRAY}弘益人間 (Benefit All Humanity) · WIA-COMM-005 v$VERSION${RESET}"
    echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
    echo ""
}

# Run main function
main "$@"
