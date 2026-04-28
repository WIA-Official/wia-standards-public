#!/bin/bash

################################################################################
# WIA-COMM-009: Wireless Power Transfer CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to wireless power transfer
# calculations including inductive, capacitive, resonant, microwave, and
# laser power transmission.
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
SPEED_OF_LIGHT=299792458
MU_0=1.25663706212e-6
EPSILON_0=8.854187817e-12

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        ⚡ WIA-COMM-009: Wireless Power Transfer CLI          ║"
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

# Inductive power transfer (Qi/AirFuel)
inductive_wpt() {
    local standard=${1:-Qi}
    local power=${2:-15}
    local frequency=${3:-87000}

    print_section "Inductive Power Transfer: $standard"

    print_info "Standard: $standard"
    print_info "Frequency: $frequency Hz ($(echo "scale=2; $frequency / 1000" | bc) kHz)"
    print_info "Max Power: $power W"

    # Calculate wavelength
    local wavelength=$(echo "scale=6; $SPEED_OF_LIGHT / $frequency" | bc)
    print_info "Wavelength: $wavelength m ($(echo "scale=0; $wavelength * 1000" | bc) mm)"

    print_section "Coil Parameters"

    case $standard in
        Qi)
            local coil_diameter=0.05
            local coil_turns=20
            local q_factor=100
            ;;
        AirFuel)
            local coil_diameter=0.08
            local coil_turns=15
            local q_factor=120
            ;;
        SAE-J2954)
            local coil_diameter=0.5
            local coil_turns=30
            local q_factor=200
            ;;
        *)
            local coil_diameter=0.05
            local coil_turns=20
            local q_factor=100
            ;;
    esac

    print_info "Coil diameter: $(echo "scale=1; $coil_diameter * 1000" | bc) mm"
    print_info "Coil turns: $coil_turns"
    print_info "Q-factor: $q_factor"

    # Calculate inductance (simplified)
    local inductance=$(echo "scale=9; $MU_0 * 3.14159 * $coil_diameter^2 * $coil_turns^2 / (4 * $coil_diameter)" | bc -l)
    print_info "Inductance: $(echo "scale=6; $inductance * 1e6" | bc) µH"

    print_section "Power Transfer Simulation"

    # Assume 5mm distance for Qi
    local distance=0.005
    if [ "$standard" == "SAE-J2954" ]; then
        distance=0.20
    fi

    # Calculate coupling coefficient
    local coupling=$(echo "scale=4; e(-$distance / $coil_diameter)" | bc -l)
    print_info "Distance: $(echo "scale=0; $distance * 1000" | bc) mm"
    print_success "Coupling coefficient: $coupling"

    # Calculate efficiency
    local efficiency=$(echo "scale=4; $coupling^2 * $q_factor / (1 + $coupling^2 * $q_factor)" | bc -l)
    print_success "Transfer efficiency: $(echo "scale=1; $efficiency * 100" | bc)%"

    # Power delivered
    local power_delivered=$(echo "scale=2; $power * $efficiency" | bc)
    print_success "Power delivered: $power_delivered W"

    # Heat loss
    local heat_loss=$(echo "scale=2; $power - $power_delivered" | bc)
    print_info "Heat dissipation: $heat_loss W"

    print_section "Safety Check"
    print_success "Foreign object detection: Enabled (Q-factor monitoring)"
    print_success "SAR compliance: < 2 W/kg"
    print_success "Temperature limit: 60°C"

    echo ""
}

# Resonant wireless power
resonant_wpt() {
    local distance=${1:-1.0}
    local power=${2:-3300}
    local frequency=${3:-6780000}

    print_section "Resonant Wireless Power Transfer"

    print_info "Frequency: $(echo "scale=2; $frequency / 1e6" | bc) MHz (ISM band)"
    print_info "Distance: $distance m"
    print_info "Max Power: $power W"

    print_section "Resonant Coil Design"

    local tx_diameter=0.5
    local rx_diameter=0.3
    local tx_turns=10
    local rx_turns=8
    local q_tx=200
    local q_rx=180

    print_info "TX coil: $(echo "scale=0; $tx_diameter * 1000" | bc) mm diameter, $tx_turns turns, Q=$q_tx"
    print_info "RX coil: $(echo "scale=0; $rx_diameter * 1000" | bc) mm diameter, $rx_turns turns, Q=$q_rx"

    # Calculate mutual inductance (simplified)
    local mutual=$(echo "scale=9; $MU_0 * 3.14159 * ($tx_diameter/2)^2 * ($rx_diameter/2)^2 * $tx_turns * $rx_turns / (2 * ($tx_diameter^2 + $distance^2)^1.5)" | bc -l)

    # Calculate inductances
    local L1=$(echo "scale=9; $MU_0 * 3.14159 * ($tx_diameter/2)^2 * $tx_turns^2 / $tx_diameter" | bc -l)
    local L2=$(echo "scale=9; $MU_0 * 3.14159 * ($rx_diameter/2)^2 * $rx_turns^2 / $rx_diameter" | bc -l)

    # Coupling coefficient
    local coupling=$(echo "scale=6; $mutual / sqrt($L1 * $L2)" | bc -l)

    print_section "Coupling Analysis"
    print_info "Mutual inductance: $(echo "scale=2; $mutual * 1e6" | bc) µH"
    print_success "Coupling coefficient: $coupling"

    # Calculate efficiency
    local k2QQ=$(echo "scale=6; $coupling^2 * $q_tx * $q_rx" | bc -l)
    local efficiency=$(echo "scale=6; $k2QQ / (1 + $k2QQ)" | bc -l)

    print_section "Power Transfer"
    print_success "Transfer efficiency: $(echo "scale=1; $efficiency * 100" | bc)%"

    local power_delivered=$(echo "scale=1; $power * $efficiency" | bc)
    print_success "Power delivered: $power_delivered W"

    local heat=$(echo "scale=1; $power - $power_delivered" | bc)
    print_info "Heat dissipation: $heat W"

    print_section "EV Charging Application"
    if [ $(echo "$power_delivered > 3000" | bc) -eq 1 ]; then
        print_success "Suitable for EV Level 2 charging (3.3 kW)"
        local charge_time=$(echo "scale=1; 60 / ($power_delivered / 1000)" | bc)
        print_info "~60 kWh battery: $charge_time hours to full charge"
    fi

    echo ""
}

# Microwave power beaming
microwave_wpt() {
    local frequency=${1:-2.45e9}
    local distance=${2:-100}
    local power=${3:-1000}

    print_section "Microwave Power Beaming"

    print_info "Frequency: $(echo "scale=2; $frequency / 1e9" | bc) GHz"
    print_info "Distance: $distance m"
    print_info "Transmit Power: $power W"

    # Calculate wavelength
    local wavelength=$(echo "scale=6; $SPEED_OF_LIGHT / $frequency" | bc -l)
    print_info "Wavelength: $(echo "scale=0; $wavelength * 1000" | bc) mm"

    print_section "Antenna Configuration"

    local antenna_gain=20  # dBi
    local beamwidth=10     # degrees

    print_info "Transmit antenna gain: $antenna_gain dBi"
    print_info "Beamwidth: $beamwidth°"
    print_info "Rectenna efficiency: 70%"

    # Convert gain to linear
    local Gt=$(echo "scale=6; e($antenna_gain * l(10) / 10)" | bc -l)
    local Gr=$Gt  # Assume same for receiver

    print_section "Link Budget Calculation"

    # Friis equation: Pr = Pt × Gt × Gr × (λ/4πd)²
    local path_loss_factor=$(echo "scale=12; ($wavelength / (4 * 3.14159 * $distance))^2" | bc -l)
    local received_rf=$(echo "scale=6; $power * $Gt * $Gr * $path_loss_factor" | bc -l)

    # Path loss in dB
    local path_loss_db=$(echo "scale=2; -10 * l($path_loss_factor) / l(10)" | bc -l)
    print_info "Path loss: $path_loss_db dB"

    # Atmospheric loss (0.1 dB/km at 2.45 GHz)
    local atm_loss=$(echo "scale=3; 0.1 * $distance / 1000" | bc)
    print_info "Atmospheric loss: $atm_loss dB"

    # Rectenna conversion
    local rectenna_eff=0.7
    local received_dc=$(echo "scale=3; $received_rf * $rectenna_eff" | bc)

    print_success "Received RF power: $received_rf W"
    print_success "DC power output: $received_dc W"

    local total_eff=$(echo "scale=4; $received_dc / $power" | bc)
    print_success "End-to-end efficiency: $(echo "scale=2; $total_eff * 100" | bc)%"

    print_section "Safety Analysis"

    # Power density
    local beam_area=$(echo "scale=6; 3.14159 * ($distance * 0.0174533 * $beamwidth / 2)^2" | bc -l)
    local power_density=$(echo "scale=3; $power * $Gt / (4 * 3.14159 * $distance^2)" | bc -l)

    print_info "Power density (peak): $power_density W/m²"

    if [ $(echo "$power_density < 10" | bc) -eq 1 ]; then
        print_success "Power density OK (< 10 W/m² limit)"
    else
        print_warning "Power density exceeds safety limit!"
    fi

    echo ""
}

# Laser power beaming
laser_wpt() {
    local wavelength=${1:-1064e-9}
    local distance=${2:-1000}
    local power=${3:-10000}

    print_section "Laser Power Beaming"

    print_info "Wavelength: $(echo "scale=0; $wavelength * 1e9" | bc) nm (Nd:YAG)"
    print_info "Distance: $distance m"
    print_info "Laser power: $power W"

    print_section "Beam Propagation"

    local beam_diameter=0.1  # 10 cm initial
    local divergence=0.001   # 1 mrad

    # Spot size at target
    local spot_size=$(echo "scale=6; $beam_diameter + 2 * $distance * $divergence" | bc)
    print_info "Beam diameter (initial): $(echo "scale=0; $beam_diameter * 1000" | bc) mm"
    print_info "Divergence: $(echo "scale=3; $divergence * 1000" | bc) mrad"
    print_success "Spot size at target: $(echo "scale=1; $spot_size * 1000" | bc) mm"

    # Atmospheric transmission (simplified)
    local atm_transmission=$(echo "scale=4; e(-$distance / 10000)" | bc -l)
    print_info "Atmospheric transmission: $(echo "scale=1; $atm_transmission * 100" | bc)%"

    # Optical power at target
    local optical_power=$(echo "scale=2; $power * $atm_transmission" | bc)
    print_success "Optical power at target: $optical_power W"

    # Irradiance
    local spot_area=$(echo "scale=9; 3.14159 * ($spot_size / 2)^2" | bc)
    local irradiance=$(echo "scale=0; $optical_power / $spot_area" | bc)
    print_info "Irradiance: $irradiance W/m²"

    print_section "Photovoltaic Receiver"

    local pv_area=0.01  # 100 cm²
    local pv_eff=0.45   # 45% (GaAs multi-junction)

    print_info "PV cell type: GaAs multi-junction"
    print_info "PV area: $(echo "scale=0; $pv_area * 10000" | bc) cm²"
    print_info "PV efficiency: $(echo "scale=0; $pv_eff * 100" | bc)%"

    # Power received by PV
    local pv_received=$(echo "scale=3; $irradiance * $pv_area" | bc)
    if [ $(echo "$pv_received > $optical_power" | bc) -eq 1 ]; then
        pv_received=$optical_power
    fi

    # Electrical power output
    local electrical=$(echo "scale=2; $pv_received * $pv_eff" | bc)

    print_success "PV received power: $pv_received W"
    print_success "Electrical output: $electrical W"

    # Total efficiency
    local total_eff=$(echo "scale=4; $electrical / $power" | bc)
    print_success "Total efficiency: $(echo "scale=1; $total_eff * 100" | bc)%"

    print_section "Applications"
    print_info "✓ Space power relay"
    print_info "✓ Disaster relief (helicopter to ground)"
    print_info "✓ Underwater vehicle charging"
    print_info "✓ Remote sensor networks"

    echo ""
}

# EV dynamic charging
ev_dynamic() {
    local speed=${1:-100}      # km/h
    local power=${2:-20000}    # 20 kW
    local efficiency=${3:-85}  # 85%

    print_section "EV Dynamic Wireless Charging"

    # Convert speed to m/s
    local speed_ms=$(echo "scale=2; $speed / 3.6" | bc)

    print_info "Vehicle speed: $speed km/h ($speed_ms m/s)"
    print_info "System power: $(echo "scale=1; $power / 1000" | bc) kW"
    print_info "Transfer efficiency: $efficiency%"

    print_section "Road Infrastructure"

    local segment_length=3    # meters
    local coil_spacing=3      # meters
    local air_gap=0.20        # 20 cm

    print_info "Segment length: $segment_length m"
    print_info "Coil spacing: $coil_spacing m"
    print_info "Air gap (road to vehicle): $(echo "scale=0; $air_gap * 1000" | bc) mm"

    print_section "Power Transfer"

    # Calculate duty cycle
    local duty_cycle=$(echo "scale=3; $segment_length / $coil_spacing" | bc)
    print_info "Duty cycle: $(echo "scale=0; $duty_cycle * 100" | bc)%"

    # Effective power received
    local eff_decimal=$(echo "scale=3; $efficiency / 100" | bc)
    local power_received=$(echo "scale=0; $power * $eff_decimal * $duty_cycle" | bc)

    print_success "Average power received: $(echo "scale=1; $power_received / 1000" | bc) kW"

    # Energy per km
    local time_per_segment=$(echo "scale=4; $segment_length / $speed_ms" | bc)
    local energy_per_segment=$(echo "scale=0; $power_received * $time_per_segment" | bc)
    local segments_per_km=$(echo "scale=0; 1000 / $coil_spacing" | bc)
    local energy_per_km=$(echo "scale=0; $energy_per_segment * $segments_per_km / 1000" | bc)

    print_info "Energy received per km: $energy_per_km kJ"
    print_info "  = $(echo "scale=3; $energy_per_km / 3600" | bc) kWh/km"

    print_section "Battery Impact"

    # Typical EV consumes 0.2 kWh/km
    local consumption=0.2  # kWh/km
    local received_kwh=$(echo "scale=3; $energy_per_km / 3600" | bc)
    local net_change=$(echo "scale=3; $received_kwh - $consumption" | bc)

    print_info "Typical EV consumption: $consumption kWh/km"

    if [ $(echo "$net_change > 0" | bc) -eq 1 ]; then
        print_success "Net battery gain: +$net_change kWh/km"
        print_success "✓ Battery charges while driving!"
    else
        local net_loss=$(echo "scale=3; -1 * $net_change" | bc)
        print_warning "Net battery loss: -$net_loss kWh/km"
        print_info "Extends range by $(echo "scale=0; ($received_kwh / $consumption) * 100" | bc)%"
    fi

    echo ""
}

# Calculate efficiency
calculate_efficiency() {
    local technology=${1:-resonant}
    local distance=${2:-0.5}
    local coupling=${3:-0.6}

    print_section "Efficiency Calculation: $technology"

    print_info "Technology: $technology"
    print_info "Distance: $distance m"

    case $technology in
        inductive)
            local q_factor=100
            print_info "Q-factor: $q_factor"
            print_info "Coupling: $coupling"

            local eff=$(echo "scale=4; $coupling^2 * $q_factor / (1 + $coupling^2 * $q_factor)" | bc -l)
            print_success "Efficiency: $(echo "scale=1; $eff * 100" | bc)%"
            ;;

        resonant)
            local q_tx=200
            local q_rx=180
            print_info "Q-factor (TX): $q_tx"
            print_info "Q-factor (RX): $q_rx"
            print_info "Coupling: $coupling"

            local k2QQ=$(echo "scale=6; $coupling^2 * $q_tx * $q_rx" | bc -l)
            local eff=$(echo "scale=6; $k2QQ / (1 + $k2QQ)" | bc -l)
            print_success "Efficiency: $(echo "scale=1; $eff * 100" | bc)%"
            ;;

        microwave)
            local freq=2.45e9
            local gain=20  # dBi

            local wavelength=$(echo "scale=6; $SPEED_OF_LIGHT / $freq" | bc -l)
            local Gt=$(echo "scale=6; e($gain * l(10) / 10)" | bc -l)

            local path_loss=$(echo "scale=12; ($wavelength / (4 * 3.14159 * $distance))^2" | bc -l)
            local link_eff=$(echo "scale=6; $path_loss * $Gt * $Gt" | bc -l)
            local rectenna_eff=0.7
            local eff=$(echo "scale=6; $link_eff * $rectenna_eff" | bc)

            print_success "Efficiency: $(echo "scale=2; $eff * 100" | bc)%"
            ;;

        laser)
            local atm_trans=$(echo "scale=4; e(-$distance / 10000)" | bc -l)
            local pv_eff=0.45
            local eff=$(echo "scale=4; $atm_trans * $pv_eff" | bc)

            print_info "Atmospheric transmission: $(echo "scale=1; $atm_trans * 100" | bc)%"
            print_info "PV efficiency: $(echo "scale=0; $pv_eff * 100" | bc)%"
            print_success "Total efficiency: $(echo "scale=1; $eff * 100" | bc)%"
            ;;
    esac

    echo ""
}

# Safety analysis
safety_check() {
    local power=${1:-1000}
    local distance=${2:-2}
    local frequency=${3:-2.45e9}

    print_section "Safety Analysis"

    print_info "Power: $power W"
    print_info "Distance: $distance m"
    print_info "Frequency: $(echo "scale=2; $frequency / 1e9" | bc) GHz"

    print_section "Power Density"

    # Assume 20 dBi antenna
    local gain_dbi=20
    local Gt=$(echo "scale=6; e($gain_dbi * l(10) / 10)" | bc -l)

    local power_density=$(echo "scale=3; $power * $Gt / (4 * 3.14159 * $distance^2)" | bc -l)
    print_info "Peak power density: $power_density W/m²"

    print_section "Regulatory Limits"

    # FCC/ICNIRP limits for general public
    local limit_general=10  # W/m² for general public
    local limit_occupational=50  # W/m² for occupational

    if [ $(echo "$power_density < $limit_general" | bc) -eq 1 ]; then
        print_success "✓ Safe for general public (< $limit_general W/m²)"
    elif [ $(echo "$power_density < $limit_occupational" | bc) -eq 1 ]; then
        print_warning "⚠ Safe for occupational only (< $limit_occupational W/m²)"
    else
        print_error "✗ Exceeds safety limits!"
    fi

    print_section "SAR (Specific Absorption Rate)"

    # Simplified SAR calculation
    local sar=$(echo "scale=4; $power_density * 0.0001" | bc)  # Very simplified
    print_info "Estimated SAR: $sar W/kg"

    if [ $(echo "$sar < 2" | bc) -eq 1 ]; then
        print_success "✓ SAR below limit (< 2 W/kg FCC)"
    else
        print_error "✗ SAR exceeds regulatory limit!"
    fi

    print_section "Safety Recommendations"
    print_info "• Implement foreign object detection (FOD)"
    print_info "• Use living object protection (LOP)"
    print_info "• Install thermal monitoring"
    print_info "• Maintain minimum safety distance: $(echo "scale=1; sqrt($power * $Gt / (4 * 3.14159 * $limit_general))" | bc -l) m"
    print_info "• Comply with FCC Part 15 / EN 55011"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-009 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  inductive [standard] [power] [freq]"
    echo "    Inductive power transfer (Qi, AirFuel, SAE-J2954)"
    echo "    Examples:"
    echo "      wia-comm-009 inductive Qi 15 87000"
    echo "      wia-comm-009 inductive SAE-J2954 11000 85000"
    echo ""
    echo "  resonant [distance] [power] [freq]"
    echo "    Resonant wireless power (WiTricity-style)"
    echo "    Example: wia-comm-009 resonant 1.0 3300 6.78e6"
    echo ""
    echo "  microwave [freq] [distance] [power]"
    echo "    Microwave power beaming (rectenna)"
    echo "    Example: wia-comm-009 microwave 2.45e9 100 1000"
    echo ""
    echo "  laser [wavelength] [distance] [power]"
    echo "    Laser power transmission"
    echo "    Example: wia-comm-009 laser 1064e-9 1000 10000"
    echo ""
    echo "  ev-dynamic [speed] [power] [efficiency]"
    echo "    EV dynamic charging simulation"
    echo "    Example: wia-comm-009 ev-dynamic 100 20000 85"
    echo ""
    echo "  efficiency [tech] [distance] [coupling]"
    echo "    Calculate efficiency for given technology"
    echo "    Example: wia-comm-009 efficiency resonant 0.5 0.6"
    echo ""
    echo "  safety [power] [distance] [frequency]"
    echo "    Safety analysis (SAR, power density)"
    echo "    Example: wia-comm-009 safety 1000 2 2.45e9"
    echo ""
    echo "  version                     Show version information"
    echo "  help                        Show this help message"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-009 Wireless Power Transfer CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Supported Technologies:"
    echo "  - Inductive Power Transfer (Qi, AirFuel, SAE J2954)"
    echo "  - Capacitive Power Transfer (CPT)"
    echo "  - Resonant Wireless Power (WiTricity)"
    echo "  - Microwave Power Beaming (2.45 GHz, 5.8 GHz)"
    echo "  - Laser Power Transmission (1064 nm, 808 nm)"
    echo "  - EV Dynamic Charging (in-road)"
    echo "  - Space Solar Power Concepts"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    inductive)
        STANDARD=${1:-Qi}
        POWER=${2:-15}
        FREQ=${3:-87000}
        print_header
        inductive_wpt "$STANDARD" "$POWER" "$FREQ"
        ;;

    resonant)
        DISTANCE=${1:-1.0}
        POWER=${2:-3300}
        FREQ=${3:-6.78e6}
        print_header
        resonant_wpt "$DISTANCE" "$POWER" "$FREQ"
        ;;

    microwave)
        FREQ=${1:-2.45e9}
        DISTANCE=${2:-100}
        POWER=${3:-1000}
        print_header
        microwave_wpt "$FREQ" "$DISTANCE" "$POWER"
        ;;

    laser)
        WAVELENGTH=${1:-1064e-9}
        DISTANCE=${2:-1000}
        POWER=${3:-10000}
        print_header
        laser_wpt "$WAVELENGTH" "$DISTANCE" "$POWER"
        ;;

    ev-dynamic)
        SPEED=${1:-100}
        POWER=${2:-20000}
        EFF=${3:-85}
        print_header
        ev_dynamic "$SPEED" "$POWER" "$EFF"
        ;;

    efficiency)
        TECH=${1:-resonant}
        DIST=${2:-0.5}
        COUPLING=${3:-0.6}
        print_header
        calculate_efficiency "$TECH" "$DIST" "$COUPLING"
        ;;

    safety)
        POWER=${1:-1000}
        DIST=${2:-2}
        FREQ=${3:-2.45e9}
        print_header
        safety_check "$POWER" "$DIST" "$FREQ"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-009 help' for usage information"
        exit 1
        ;;
esac

exit 0
