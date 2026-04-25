#!/bin/bash

################################################################################
# WIA-IND-002: Smart Textile Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industrial Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to smart textile calculations
# including conductivity, sensor performance, temperature regulation, and
# energy harvesting analysis.
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
PI=3.141592653589793
STEFAN_BOLTZMANN=0.0000000567  # W/m²·K⁴

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         👕 WIA-IND-002: Smart Textile CLI                     ║"
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

# Calculate fabric conductivity
calc_conductivity() {
    local material=${1:-silver}
    local fraction=${2:-0.05}
    local alignment=${3:-0.7}
    local contact=${4:-0.3}

    print_section "Fabric Conductivity Calculation"

    # Base conductivity values (S/m)
    local base_conductivity
    case $material in
        silver)
            base_conductivity=63000000  # 6.3×10⁷ S/m
            print_info "Material: Silver-plated fiber"
            ;;
        copper)
            base_conductivity=59600000  # 5.96×10⁷ S/m
            print_info "Material: Copper wire"
            ;;
        graphene)
            base_conductivity=100000  # 1×10⁵ S/m
            print_info "Material: Graphene fiber"
            ;;
        cnt)
            base_conductivity=500000  # 5×10⁵ S/m (aligned CNT)
            print_info "Material: Carbon nanotube"
            ;;
        steel)
            base_conductivity=1400000  # 1.4×10⁶ S/m
            print_info "Material: Stainless steel"
            ;;
        *)
            base_conductivity=10000  # Default: 10⁴ S/m
            print_info "Material: Generic conductive fiber"
            ;;
    esac

    print_info "Fiber fraction: $fraction"
    print_info "Alignment factor: $alignment"
    print_info "Contact factor: $contact"

    # Effective conductivity = base × fraction × alignment × contact
    local effective=$(echo "scale=2; $base_conductivity * $fraction * $alignment * $contact" | bc -l)

    print_section "Results"
    print_success "Effective Conductivity: $effective S/m"

    # Calculate resistance for 1m length, 1mm² cross-section
    local resistance=$(echo "scale=4; 1 / ($effective / 1000000)" | bc -l)
    print_info "Resistance (1m, 1mm²): $resistance Ω/m"

    # Conductivity class
    local class
    if (( $(echo "$effective >= 10000" | bc -l) )); then
        class="Excellent (> 10⁴ S/m)"
    elif (( $(echo "$effective >= 1000" | bc -l) )); then
        class="Good (10³ - 10⁴ S/m)"
    elif (( $(echo "$effective >= 100" | bc -l) )); then
        class="Moderate (10² - 10³ S/m)"
    else
        class="Low (< 10² S/m)"
    fi
    print_info "Conductivity class: $class"

    echo ""
}

# Calculate sensor sensitivity
calc_sensor() {
    local type=${1:-strain}
    local gauge_factor=${2:-20}
    local strain=${3:-2.5}

    print_section "Sensor Performance Calculation"
    print_info "Sensor type: $type"

    case $type in
        strain)
            print_info "Gauge factor (GF): $gauge_factor"
            print_info "Applied strain: $strain %"

            # Sensitivity = GF × strain
            local sensitivity=$(echo "scale=4; $gauge_factor * $strain / 100" | bc -l)

            # Relative resistance change
            local delta_r=$(echo "scale=2; $sensitivity * 100" | bc -l)

            print_section "Results"
            print_success "Sensitivity (ΔR/R₀): $sensitivity"
            print_info "Resistance change: $delta_r %"

            # For 1kΩ base resistance
            local base_r=1000
            local new_r=$(echo "scale=2; $base_r * (1 + $sensitivity)" | bc -l)
            print_info "New resistance (1kΩ base): $new_r Ω"
            ;;

        pressure)
            local sensitivity_kpa=${gauge_factor:-0.5}  # kPa⁻¹
            local pressure=${strain:-10}  # kPa

            print_info "Sensitivity: $sensitivity_kpa kPa⁻¹"
            print_info "Applied pressure: $pressure kPa"

            local response=$(echo "scale=4; $sensitivity_kpa * $pressure" | bc -l)
            local delta_r=$(echo "scale=2; $response * 100" | bc -l)

            print_section "Results"
            print_success "Response (ΔR/R₀): $response"
            print_info "Resistance change: $delta_r %"
            ;;

        temperature)
            local tcr=${gauge_factor:-3500}  # ppm/°C
            local delta_t=${strain:-5}  # °C

            print_info "TCR: $tcr ppm/°C"
            print_info "Temperature change: $delta_t °C"

            local delta_r=$(echo "scale=4; $tcr * $delta_t / 1000000" | bc -l)
            local delta_r_pct=$(echo "scale=2; $delta_r * 100" | bc -l)

            print_section "Results"
            print_success "Resistance change (ΔR/R₀): $delta_r"
            print_info "Resistance change: $delta_r_pct %"
            ;;

        *)
            print_error "Unknown sensor type: $type"
            print_info "Supported types: strain, pressure, temperature"
            return 1
            ;;
    esac

    echo ""
}

# Calculate temperature regulation
calc_thermal() {
    local skin_temp=${1:-34}
    local ambient=${2:-25}
    local emissivity=${3:-0.85}
    local h_coeff=${4:-15}  # Heat transfer coefficient W/m²·K

    print_section "Temperature Regulation Calculation"
    print_info "Skin temperature: $skin_temp °C"
    print_info "Ambient temperature: $ambient °C"
    print_info "Emissivity: $emissivity"
    print_info "Heat transfer coefficient: $h_coeff W/m²·K"

    # Convert to Kelvin
    local skin_k=$(echo "$skin_temp + 273.15" | bc -l)
    local ambient_k=$(echo "$ambient + 273.15" | bc -l)

    # Convective heat transfer: Q_conv = h × (Tskin - Tamb)
    local q_conv=$(echo "scale=2; $h_coeff * ($skin_temp - $ambient)" | bc -l)

    # Radiative heat transfer: Q_rad = ε × σ × (Tskin⁴ - Tamb⁴)
    local skin_k4=$(echo "scale=6; $skin_k * $skin_k * $skin_k * $skin_k" | bc -l)
    local ambient_k4=$(echo "scale=6; $ambient_k * $ambient_k * $ambient_k * $ambient_k" | bc -l)
    local q_rad=$(echo "scale=2; $emissivity * $STEFAN_BOLTZMANN * ($skin_k4 - $ambient_k4)" | bc -l)

    # Total cooling power
    local q_total=$(echo "scale=2; $q_conv + $q_rad" | bc -l)

    print_section "Results"
    print_success "Total cooling power: $q_total W/m²"
    print_info "Convective component: $q_conv W/m²"
    print_info "Radiative component: $q_rad W/m²"

    # Cooling efficiency
    local conv_pct=$(echo "scale=1; $q_conv * 100 / $q_total" | bc -l)
    local rad_pct=$(echo "scale=1; $q_rad * 100 / $q_total" | bc -l)
    print_info "Convection: $conv_pct %"
    print_info "Radiation: $rad_pct %"

    # PCM energy storage example (200g paraffin, L=200 J/g)
    local pcm_mass=200
    local latent_heat=200
    local energy=$(echo "scale=0; $pcm_mass * $latent_heat" | bc -l)
    local buffering_time=$(echo "scale=1; $energy / ($q_total * 0.05)" | bc -l)  # 0.05 m² area

    print_section "PCM Thermal Buffering (Example)"
    print_info "PCM mass: $pcm_mass g (paraffin)"
    print_info "Latent heat: $latent_heat J/g"
    print_info "Energy storage: $energy J"
    print_info "Buffering time (0.05 m² area): $buffering_time minutes"

    echo ""
}

# Calculate energy harvesting
calc_energy() {
    local type=${1:-piezo}
    local area=${2:-100}  # cm²
    local freq=${3:-2.5}  # Hz

    print_section "Energy Harvesting Calculation"
    print_info "Harvesting type: $type"

    case $type in
        piezo|piezoelectric)
            local d33=${4:-25}  # pC/N
            local force=${5:-50}  # N

            print_info "Active area: $area cm²"
            print_info "Frequency: $freq Hz"
            print_info "Piezoelectric coefficient d₃₃: $d33 pC/N"
            print_info "Applied force: $force N"

            # Simplified power calculation: P ≈ 0.5 × k × A × f × d²
            # Using d33 and force to estimate displacement
            local displacement=$(echo "scale=4; $d33 * $force / 10000" | bc -l)  # mm
            local power=$(echo "scale=2; 0.5 * 20 * $area / 100 * $freq * $displacement * $displacement" | bc -l)

            print_section "Results"
            print_success "Estimated power output: $power μW"

            # Daily energy
            local daily_energy=$(echo "scale=2; $power * 24 / 1000" | bc -l)
            print_info "Daily energy (continuous): $daily_energy mWh"

            # Walking: 8 hours/day
            local walking_energy=$(echo "scale=2; $power * 8 / 1000" | bc -l)
            print_info "Daily energy (8h walking): $walking_energy mWh"
            ;;

        thermo|thermoelectric)
            local delta_t=${4:-10}  # K
            local seebeck=${5:-200}  # μV/K

            print_info "Active area: $area cm²"
            print_info "Temperature difference: $delta_t K"
            print_info "Seebeck coefficient: $seebeck μV/K"

            # Voltage: V = α × ΔT
            local voltage=$(echo "scale=4; $seebeck * $delta_t / 1000000" | bc -l)  # V

            # Assume internal resistance 10Ω/cm²
            local r_internal=$(echo "scale=2; 10 * $area / 100" | bc -l)

            # Maximum power: P = V² / (4R)
            local power=$(echo "scale=2; $voltage * $voltage * 1000000 / (4 * $r_internal)" | bc -l)  # μW

            print_section "Results"
            print_success "Estimated power output: $power μW"
            print_info "Open circuit voltage: $voltage V"
            print_info "Internal resistance: $r_internal Ω"

            # Daily energy
            local daily_energy=$(echo "scale=2; $power * 24 / 1000" | bc -l)
            print_info "Daily energy (continuous): $daily_energy mWh"
            ;;

        solar)
            local efficiency=${4:-12}  # %
            local irradiance=${5:-1000}  # W/m²

            print_info "Active area: $area cm²"
            print_info "Efficiency: $efficiency %"
            print_info "Irradiance: $irradiance W/m²"

            # Power = Area × Irradiance × Efficiency
            local area_m2=$(echo "scale=6; $area / 10000" | bc -l)
            local power=$(echo "scale=2; $area_m2 * $irradiance * $efficiency / 100 * 1000000" | bc -l)  # μW

            print_section "Results"
            print_success "Peak power output: $power μW"
            print_info "Peak power: $(echo "scale=2; $power / 1000" | bc -l) mW"

            # Daily energy (assuming 6 hours effective sunlight)
            local daily_energy=$(echo "scale=2; $power * 6 / 1000" | bc -l)
            print_info "Daily energy (6h sun): $daily_energy mWh"
            ;;

        tribo|triboelectric)
            local v_oc=${4:-100}  # V (open circuit)
            local i_sc=${5:-10}  # μA (short circuit)

            print_info "Active area: $area cm²"
            print_info "Contact frequency: $freq Hz"
            print_info "Open circuit voltage: $v_oc V"
            print_info "Short circuit current: $i_sc μA"

            # Average power ≈ (V_oc × I_sc) / 4 (impedance matching)
            local peak_power=$(echo "scale=2; $v_oc * $i_sc / 4" | bc -l)

            # Duty cycle consideration (contact time ~10% of period)
            local avg_power=$(echo "scale=2; $peak_power * 0.1" | bc -l)

            print_section "Results"
            print_success "Average power output: $avg_power μW"
            print_info "Peak power: $peak_power μW"

            # Daily energy (8 hours of motion)
            local daily_energy=$(echo "scale=2; $avg_power * 8 / 1000" | bc -l)
            print_info "Daily energy (8h motion): $daily_energy mWh"
            ;;

        *)
            print_error "Unknown energy harvesting type: $type"
            print_info "Supported types: piezo, thermo, solar, tribo"
            return 1
            ;;
    esac

    echo ""
}

# Simulate washability
simulate_wash() {
    local cycles=${1:-50}
    local retention_target=${2:-80}
    local initial_r=${3:-100}  # Ω

    print_section "Washability Simulation"
    print_info "Wash cycles: $cycles"
    print_info "Target retention: $retention_target %"
    print_info "Initial resistance: $initial_r Ω"

    # Degradation model: R(n) = R₀ × (1 + k × n)
    # For 80% retention at 50 cycles: 1 + k × 50 = 1.25 → k = 0.005
    local k=$(echo "scale=6; (1 / ($retention_target / 100) - 1) / $cycles" | bc -l)

    print_section "Simulation Results"

    # Show resistance at key cycles
    for cycle in 0 10 20 30 40 50; do
        local r_current=$(echo "scale=2; $initial_r * (1 + $k * $cycle)" | bc -l)
        local retention=$(echo "scale=1; 100 / (1 + $k * $cycle)" | bc -l)

        if (( cycle == 0 )); then
            print_info "Cycle $cycle: $r_current Ω (baseline)"
        else
            local pass_fail
            if (( $(echo "$retention >= $retention_target" | bc -l) )); then
                pass_fail="${GREEN}✓ PASS${GRAY}"
            else
                pass_fail="${RED}✗ FAIL${GRAY}"
            fi
            echo -e "${GRAY}  Cycle $cycle: $r_current Ω ($retention% retention) $pass_fail${RESET}"
        fi
    done

    print_section "Assessment"
    local final_r=$(echo "scale=2; $initial_r * (1 + $k * $cycles)" | bc -l)
    local final_retention=$(echo "scale=1; 100 / (1 + $k * $cycles)" | bc -l)

    if (( $(echo "$final_retention >= $retention_target" | bc -l) )); then
        print_success "PASS: Retention $final_retention% ≥ target $retention_target%"
    else
        print_warning "FAIL: Retention $final_retention% < target $retention_target%"
    fi

    print_info "Final resistance: $final_r Ω"
    print_info "Degradation rate: $(echo "scale=4; $k * 100" | bc -l)% per cycle"

    echo ""
}

# Calculate power budget
calc_power() {
    local mcu=${1:-5}  # mA
    local sensor=${2:-2}  # mA
    local ble=${3:-10}  # mA
    local voltage=${4:-3.7}  # V
    local battery=${5:-500}  # mAh

    print_section "Power Budget Calculation"
    print_info "MCU current: $mcu mA"
    print_info "Sensor current: $sensor mA"
    print_info "BLE current: $ble mA"
    print_info "Operating voltage: $voltage V"
    print_info "Battery capacity: $battery mAh"

    # Total current
    local total_current=$(echo "scale=2; $mcu + $sensor + $ble" | bc -l)

    # Total power
    local total_power=$(echo "scale=2; $total_current * $voltage" | bc -l)

    # Battery life
    local battery_life=$(echo "scale=1; $battery / $total_current" | bc -l)

    print_section "Results"
    print_success "Total current: $total_current mA"
    print_success "Total power: $total_power mW"
    print_success "Battery life: $battery_life hours"

    # Days of operation
    local days=$(echo "scale=1; $battery_life / 24" | bc -l)
    print_info "Operating days: $days days"

    # Power breakdown
    local mcu_pct=$(echo "scale=1; $mcu * 100 / $total_current" | bc -l)
    local sensor_pct=$(echo "scale=1; $sensor * 100 / $total_current" | bc -l)
    local ble_pct=$(echo "scale=1; $ble * 100 / $total_current" | bc -l)

    print_section "Power Distribution"
    print_info "MCU: $mcu_pct %"
    print_info "Sensors: $sensor_pct %"
    print_info "BLE: $ble_pct %"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-002 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-conductivity   Calculate fabric electrical conductivity"
    echo "  calc-sensor         Calculate sensor sensitivity and performance"
    echo "  calc-thermal        Calculate temperature regulation performance"
    echo "  calc-energy         Calculate energy harvesting output"
    echo "  simulate-wash       Simulate washability durability"
    echo "  calc-power          Calculate power budget and battery life"
    echo "  --help, -h          Show this help message"
    echo "  --version, -v       Show version information"
    echo ""
    echo "Examples:"
    echo "  wia-ind-002 calc-conductivity --material silver --fraction 0.05 --alignment 0.8"
    echo "  wia-ind-002 calc-sensor --type strain --gauge-factor 20 --strain 3.0"
    echo "  wia-ind-002 calc-thermal --skin-temp 34 --ambient 25 --emissivity 0.85"
    echo "  wia-ind-002 calc-energy --type piezo --area 100 --freq 2.5"
    echo "  wia-ind-002 simulate-wash --cycles 50 --retention-target 80"
    echo "  wia-ind-002 calc-power --mcu 5 --sensor 2 --ble 10 --battery 500"
    echo ""
    echo "弘益人間 (Benefit All Humanity) - WIA Industrial Research Group"
    echo ""
}

# Main command router
main() {
    if [[ $# -eq 0 ]]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        calc-conductivity)
            local material="silver"
            local fraction=0.05
            local alignment=0.7
            local contact=0.3

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --material) material=$2; shift 2 ;;
                    --fraction) fraction=$2; shift 2 ;;
                    --alignment) alignment=$2; shift 2 ;;
                    --contact) contact=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_conductivity "$material" "$fraction" "$alignment" "$contact"
            ;;

        calc-sensor)
            local type="strain"
            local gauge_factor=20
            local strain=2.5

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --type) type=$2; shift 2 ;;
                    --gauge-factor) gauge_factor=$2; shift 2 ;;
                    --strain) strain=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_sensor "$type" "$gauge_factor" "$strain"
            ;;

        calc-thermal)
            local skin_temp=34
            local ambient=25
            local emissivity=0.85
            local h_coeff=15

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --skin-temp) skin_temp=$2; shift 2 ;;
                    --ambient) ambient=$2; shift 2 ;;
                    --emissivity) emissivity=$2; shift 2 ;;
                    --h-coeff) h_coeff=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_thermal "$skin_temp" "$ambient" "$emissivity" "$h_coeff"
            ;;

        calc-energy)
            local type="piezo"
            local area=100
            local freq=2.5
            local param1=""
            local param2=""

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --type) type=$2; shift 2 ;;
                    --area) area=$2; shift 2 ;;
                    --freq) freq=$2; shift 2 ;;
                    --d33) param1=$2; shift 2 ;;
                    --force) param2=$2; shift 2 ;;
                    --delta-t) param1=$2; shift 2 ;;
                    --seebeck) param2=$2; shift 2 ;;
                    --efficiency) param1=$2; shift 2 ;;
                    --irradiance) param2=$2; shift 2 ;;
                    --v-oc) param1=$2; shift 2 ;;
                    --i-sc) param2=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_energy "$type" "$area" "$freq" "$param1" "$param2"
            ;;

        simulate-wash)
            local cycles=50
            local retention_target=80
            local initial_r=100

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --cycles) cycles=$2; shift 2 ;;
                    --retention-target) retention_target=$2; shift 2 ;;
                    --initial-r) initial_r=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            simulate_wash "$cycles" "$retention_target" "$initial_r"
            ;;

        calc-power)
            local mcu=5
            local sensor=2
            local ble=10
            local voltage=3.7
            local battery=500

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --mcu) mcu=$2; shift 2 ;;
                    --sensor) sensor=$2; shift 2 ;;
                    --ble) ble=$2; shift 2 ;;
                    --voltage) voltage=$2; shift 2 ;;
                    --battery) battery=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_power "$mcu" "$sensor" "$ble" "$voltage" "$battery"
            ;;

        --help|-h)
            show_help
            ;;

        --version|-v)
            echo "WIA-IND-002 Smart Textile CLI v$VERSION"
            echo "弘益人間 (Benefit All Humanity)"
            ;;

        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main
main "$@"
