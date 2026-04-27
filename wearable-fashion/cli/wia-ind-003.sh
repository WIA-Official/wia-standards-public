#!/bin/bash

################################################################################
# WIA-IND-003: Wearable Fashion Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Fashion Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to wearable fashion calculations
# including LED power, battery life, thermal management, and energy harvesting.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
LED_VOLTAGE=3.3
DEFAULT_EFFICIENCY=0.9

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        👔 WIA-IND-003: Wearable Fashion CLI                  ║"
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

# Calculate LED power consumption
calc_led_power() {
    local count=${1:-100}
    local current=${2:-20}      # mA per LED
    local voltage=${3:-3.3}     # V
    local brightness=${4:-100}  # %
    local duty_cycle=${5:-100}  # %

    print_section "LED Power Calculation"
    print_info "LED Count: $count"
    print_info "Current per LED: ${current}mA"
    print_info "Voltage: ${voltage}V"
    print_info "Brightness: ${brightness}%"
    print_info "Duty Cycle: ${duty_cycle}%"

    # Power per LED (W) = Voltage × Current / 1000
    local power_per_led=$(echo "scale=4; $voltage * $current / 1000" | bc -l)
    print_info "Power per LED: ${power_per_led}W"

    # Total max power
    local max_power=$(echo "scale=2; $power_per_led * $count" | bc -l)
    print_info "Maximum Power: ${max_power}W"

    # Actual power with brightness and duty cycle
    local brightness_factor=$(echo "scale=4; $brightness / 100" | bc -l)
    local duty_factor=$(echo "scale=4; $duty_cycle / 100" | bc -l)
    local actual_power=$(echo "scale=3; $max_power * $brightness_factor * $brightness_factor * $duty_factor" | bc -l)

    print_section "Results"
    print_success "Actual Power Consumption: ${actual_power}W"

    # Current draw
    local current_ma=$(echo "scale=1; $actual_power * 1000 / $voltage" | bc -l)
    print_info "Current Draw: ${current_ma}mA"

    # Energy per hour
    local energy_wh=$(echo "scale=2; $actual_power * 1" | bc -l)
    print_info "Energy per Hour: ${energy_wh}Wh"

    echo ""
}

# Calculate battery life
calc_battery() {
    local capacity=${1:-1000}    # mAh
    local current=${2:-50}       # mA
    local voltage=${3:-3.7}      # V
    local dod=${4:-0.9}          # Depth of discharge
    local efficiency=${5:-0.9}   # Power efficiency

    print_section "Battery Life Calculation"
    print_info "Battery Capacity: ${capacity}mAh"
    print_info "Average Current: ${current}mA"
    print_info "Battery Voltage: ${voltage}V"
    print_info "Usable DoD: $dod"
    print_info "Efficiency: $efficiency"

    # Usable capacity
    local usable=$(echo "scale=2; $capacity * $dod * $efficiency" | bc -l)
    print_info "Usable Capacity: ${usable}mAh"

    # Battery life in hours
    local life_hours=$(echo "scale=2; $usable / $current" | bc -l)

    # Convert to hours and minutes
    local hours=$(echo "$life_hours" | cut -d. -f1)
    local decimal=$(echo "scale=2; $life_hours - $hours" | bc -l)
    local minutes=$(echo "scale=0; $decimal * 60" | bc -l)

    print_section "Results"
    print_success "Battery Life: ${life_hours} hours (${hours}h ${minutes}m)"

    # Energy capacity
    local energy_wh=$(echo "scale=2; $capacity * $voltage / 1000" | bc -l)
    print_info "Battery Energy: ${energy_wh}Wh"

    # Power consumption
    local power_w=$(echo "scale=3; $current * $voltage / 1000" | bc -l)
    print_info "Average Power: ${power_w}W"

    echo ""
}

# Design smart jewelry
design_jewelry() {
    local type=${1:-bracelet}
    local features=${2:-heartRate,ledDisplay}

    print_section "Smart Jewelry Designer: $type"

    # Base specifications
    case $type in
        ring)
            local led_count=8
            local battery_capacity=50
            local size="16-22mm diameter"
            ;;
        bracelet)
            local led_count=24
            local battery_capacity=150
            local size="18-22cm circumference"
            ;;
        necklace)
            local led_count=40
            local battery_capacity=300
            local size="40-50cm length"
            ;;
        *)
            local led_count=16
            local battery_capacity=100
            local size="custom"
            ;;
    esac

    print_info "Type: $type"
    print_info "Size: $size"
    print_info "LED Count: $led_count"
    print_info "Battery: ${battery_capacity}mAh"

    # Calculate power consumption based on features
    local led_current=0
    local sensor_current=0
    local base_current=2  # MCU baseline

    if [[ $features == *"led"* ]]; then
        led_current=$(echo "scale=1; $led_count * 20 * 0.3 * 0.5" | bc -l)  # 30% brightness, 50% duty
        print_info "LEDs: ${led_count} RGB LEDs"
    fi

    if [[ $features == *"heartRate"* ]]; then
        sensor_current=$(echo "scale=1; $sensor_current + 10" | bc -l)
        print_info "Heart Rate: PPG sensor (10mA)"
    fi

    if [[ $features == *"temperature"* ]]; then
        sensor_current=$(echo "scale=1; $sensor_current + 0.5" | bc -l)
        print_info "Temperature: NTC sensor (0.5mA)"
    fi

    if [[ $features == *"haptic"* ]]; then
        local haptic_current=5  # Average with duty cycle
        print_info "Haptic: Vibration motor (avg 5mA)"
    else
        local haptic_current=0
    fi

    # Total current
    local total_current=$(echo "scale=1; $led_current + $sensor_current + $base_current + $haptic_current" | bc -l)

    # Battery life
    local battery_life=$(echo "scale=1; $battery_capacity * 0.9 / $total_current" | bc -l)

    print_section "Power Analysis"
    print_success "Total Current: ${total_current}mA"
    print_success "Battery Life: ${battery_life} hours"

    # Recommendations
    print_section "Recommendations"
    if (( $(echo "$battery_life < 8" | bc -l) )); then
        print_warning "Battery life is low. Consider:"
        print_info "  • Reduce LED brightness or count"
        print_info "  • Use motion activation"
        print_info "  • Increase battery capacity"
    else
        print_success "Battery life is adequate for daily use"
    fi

    echo ""
}

# Calculate thermal comfort
calc_thermal() {
    local power=${1:-15}        # W
    local area=${2:-500}        # cm²
    local ambient=${3:-20}      # °C
    local target=${4:-35}       # °C

    print_section "Thermal Comfort Calculation"
    print_info "Heating Power: ${power}W"
    print_info "Heated Area: ${area}cm²"
    print_info "Ambient Temp: ${ambient}°C"
    print_info "Target Temp: ${target}°C"

    # Power density (W/cm²)
    local power_density=$(echo "scale=4; $power / $area" | bc -l)
    print_info "Power Density: ${power_density}W/cm²"

    # Temperature rise needed
    local temp_rise=$(echo "scale=1; $target - $ambient" | bc -l)
    print_info "Required Rise: ${temp_rise}°C"

    # Heating time estimate (simplified)
    # Assumes fabric mass ~0.5kg, specific heat ~1500 J/kg·K
    local mass=0.5
    local specific_heat=1500
    local energy_needed=$(echo "scale=2; $mass * $specific_heat * $temp_rise / 1000" | bc -l)  # kJ
    local time_seconds=$(echo "scale=0; $energy_needed * 1000 / $power" | bc -l)
    local time_minutes=$(echo "scale=1; $time_seconds / 60" | bc -l)

    print_section "Results"
    print_success "Heating Time: ${time_minutes} minutes"

    # Battery consumption
    local energy_wh=$(echo "scale=2; $power * $time_minutes / 60" | bc -l)
    print_info "Energy to Warm: ${energy_wh}Wh"

    # Continuous operation current (at 12V)
    local voltage=12
    local current_ma=$(echo "scale=0; $power * 1000 / $voltage" | bc -l)
    print_info "Continuous Current: ${current_ma}mA at ${voltage}V"

    # Safety check
    if (( $(echo "$power_density > 0.02" | bc -l) )); then
        print_warning "Power density is high (>${power_density}W/cm²)"
        print_info "  • Ensure thermal protection at 45°C"
        print_info "  • Add temperature sensor"
    fi

    echo ""
}

# Calculate energy harvesting
calc_harvest() {
    local type=${1:-solar}
    local area=${2:-200}        # cm² for solar
    local time=${3:-8}          # hours
    local efficiency=${4:-0.20} # 20% for solar

    print_section "Energy Harvesting: $type"

    case $type in
        solar)
            print_info "Solar Panel Area: ${area}cm²"
            print_info "Exposure Time: ${time} hours"
            print_info "Panel Efficiency: $(echo "scale=0; $efficiency * 100" | bc -l)%"

            # Average solar irradiance (indoor: 10, outdoor sun: 100 mW/cm²)
            local irradiance_outdoor=100
            local irradiance_indoor=10

            # Outdoor energy
            local power_outdoor=$(echo "scale=2; $area * $irradiance_outdoor * $efficiency / 1000" | bc -l)
            local energy_outdoor=$(echo "scale=2; $power_outdoor * $time" | bc -l)

            # Indoor energy
            local power_indoor=$(echo "scale=2; $area * $irradiance_indoor * $efficiency / 1000" | bc -l)
            local energy_indoor=$(echo "scale=2; $power_indoor * $time" | bc -l)

            print_section "Results"
            print_success "Outdoor Power: ${power_outdoor}W"
            print_info "Outdoor Energy (${time}h): ${energy_outdoor}Wh"
            print_success "Indoor Power: ${power_indoor}W"
            print_info "Indoor Energy (${time}h): ${energy_indoor}Wh"
            ;;

        kinetic)
            print_info "Activity Level: Normal walking"
            print_info "Duration: ${time} hours"

            # Piezoelectric harvesting (typical: 1-10mW per element)
            local power_mw=5
            local power_w=$(echo "scale=4; $power_mw / 1000" | bc -l)
            local energy_wh=$(echo "scale=3; $power_w * $time" | bc -l)

            print_section "Results"
            print_success "Average Power: ${power_mw}mW"
            print_info "Energy (${time}h): ${energy_wh}Wh"
            ;;

        thermoelectric)
            local temp_diff=${area:-10}  # Reuse param as temp diff
            print_info "Temperature Differential: ${temp_diff}°C"
            print_info "TEG Efficiency: 5%"

            # Power per cm² of TEG (very rough estimate)
            local power_per_cm2=0.01  # W/cm²
            local teg_area=20  # cm²
            local power_w=$(echo "scale=3; $power_per_cm2 * $teg_area * $temp_diff / 10" | bc -l)
            local energy_wh=$(echo "scale=3; $power_w * $time" | bc -l)

            print_section "Results"
            print_success "Average Power: ${power_w}W"
            print_info "Energy (${time}h): ${energy_wh}Wh"
            ;;
    esac

    echo ""
}

# Display usage
usage() {
    print_header
    echo "Usage: wia-ind-003 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-led-power    Calculate LED power consumption"
    echo "  calc-battery      Calculate battery life"
    echo "  design-jewelry    Design smart jewelry"
    echo "  calc-thermal      Calculate thermal comfort"
    echo "  calc-harvest      Estimate energy harvesting"
    echo "  --version         Show version"
    echo "  --help            Show this help"
    echo ""
    echo "Examples:"
    echo "  wia-ind-003 calc-led-power --count 100 --current 20 --voltage 3.3"
    echo "  wia-ind-003 calc-battery --capacity 1000 --current 50"
    echo "  wia-ind-003 design-jewelry --type bracelet --features heartRate,ledDisplay"
    echo "  wia-ind-003 calc-thermal --power 15 --area 500 --ambient 10"
    echo "  wia-ind-003 calc-harvest --type solar --area 200 --time 8"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo "© 2025 SmileStory Inc. / WIA"
    echo ""
}

# Parse arguments
if [ $# -eq 0 ]; then
    usage
    exit 0
fi

case "$1" in
    calc-led-power)
        shift
        count=100
        current=20
        voltage=3.3
        brightness=100
        duty_cycle=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --count) count="$2"; shift 2 ;;
                --current) current="$2"; shift 2 ;;
                --voltage) voltage="$2"; shift 2 ;;
                --brightness) brightness="$2"; shift 2 ;;
                --duty-cycle) duty_cycle="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_led_power "$count" "$current" "$voltage" "$brightness" "$duty_cycle"
        ;;

    calc-battery)
        shift
        capacity=1000
        current=50
        voltage=3.7
        dod=0.9
        efficiency=0.9

        while [[ $# -gt 0 ]]; do
            case $1 in
                --capacity) capacity="$2"; shift 2 ;;
                --current) current="$2"; shift 2 ;;
                --voltage) voltage="$2"; shift 2 ;;
                --dod) dod="$2"; shift 2 ;;
                --efficiency) efficiency="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_battery "$capacity" "$current" "$voltage" "$dod" "$efficiency"
        ;;

    design-jewelry)
        shift
        type="bracelet"
        features="heartRate,ledDisplay"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) type="$2"; shift 2 ;;
                --features) features="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_jewelry "$type" "$features"
        ;;

    calc-thermal)
        shift
        power=15
        area=500
        ambient=20
        target=35

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) power="$2"; shift 2 ;;
                --area) area="$2"; shift 2 ;;
                --ambient) ambient="$2"; shift 2 ;;
                --target) target="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_thermal "$power" "$area" "$ambient" "$target"
        ;;

    calc-harvest)
        shift
        type="solar"
        area=200
        time=8

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) type="$2"; shift 2 ;;
                --area) area="$2"; shift 2 ;;
                --time) time="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_harvest "$type" "$area" "$time"
        ;;

    --version)
        echo "WIA-IND-003 CLI v${VERSION}"
        ;;

    --help|-h)
        usage
        ;;

    *)
        print_error "Unknown command: $1"
        usage
        exit 1
        ;;
esac
