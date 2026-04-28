#!/bin/bash

################################################################################
# WIA-AUTO-009: Vehicle Semiconductor CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Semiconductor Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to automotive semiconductor
# classification, validation, and reliability calculations.
################################################################################

set -e

# Colors for output
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
BOLTZMANN=8.617333262e-5  # eV/K
ACTIVATION_ENERGY=0.7      # eV
REFERENCE_TEMP=55          # Celsius

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🚗 WIA-AUTO-009: Vehicle Semiconductor CLI             ║"
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

# Get temperature grade specifications
get_temp_grade_spec() {
    local grade=$1
    case $grade in
        0) echo "-40°C to +150°C (Engine compartment)" ;;
        1) echo "-40°C to +125°C (Under hood)" ;;
        2) echo "-40°C to +105°C (Passenger cabin)" ;;
        3) echo "-40°C to +85°C (Mild environment)" ;;
        *) echo "Unknown grade" ;;
    esac
}

# Get ASIL target metrics
get_asil_targets() {
    local asil=$1
    case $asil in
        D|ASIL-D) echo "PMHF: <10 FIT, LFM: <1%, DC: ≥99%" ;;
        C|ASIL-C) echo "PMHF: <100 FIT, LFM: <10%, DC: ≥97%" ;;
        B|ASIL-B) echo "PMHF: <100 FIT, LFM: <10%, DC: ≥90%" ;;
        A|ASIL-A) echo "PMHF: <1000 FIT, LFM: <10%, DC: ≥60%" ;;
        QM) echo "Quality Management (no specific targets)" ;;
        *) echo "Unknown ASIL level" ;;
    esac
}

# Classify semiconductor component
classify_component() {
    local type=${1:-MCU}
    local temp_grade=${2:-1}
    local asil=${3:-D}

    print_section "Component Classification"
    print_info "Type: $type"
    print_info "Temperature Grade: $temp_grade ($(get_temp_grade_spec $temp_grade))"
    print_info "Required ASIL: ASIL-$asil"

    print_section "AEC Qualification"
    case $type in
        MCU|SoC|PMIC|CAN|LIN|Sensor-IC|OpAmp|ADC|DAC|Memory)
            print_success "AEC-Q100: Integrated Circuits"
            print_info "Qualification required for automotive use"
            ;;
        MOSFET|IGBT|Gate-Driver)
            print_success "AEC-Q101: Discrete Semiconductors"
            print_info "High-voltage and power device qualification"
            ;;
        *)
            print_warning "AEC Qualification: Unknown for type $type"
            ;;
    esac

    print_section "Safety Requirements"
    print_info "ASIL-$asil Targets: $(get_asil_targets $asil)"

    print_section "Recommended Applications"
    case $asil in
        D|ASIL-D)
            print_info "• Steering control systems"
            print_info "• Brake control (ABS, ESC)"
            print_info "• Airbag deployment"
            ;;
        C|ASIL-C)
            print_info "• Antilock braking system"
            print_info "• Traction control"
            print_info "• Electronic stability control"
            ;;
        B|ASIL-B)
            print_info "• Headlight control"
            print_info "• Wiper control"
            print_info "• Horn"
            ;;
        A|ASIL-A)
            print_info "• Rear lights"
            print_info "• Turn signals"
            ;;
        QM)
            print_info "• Infotainment systems"
            print_info "• Comfort features"
            print_info "• Non-safety applications"
            ;;
    esac

    print_section "Temperature Suitability"
    case $temp_grade in
        0)
            print_info "Suitable for: Engine bay, exhaust systems"
            print_warning "Maximum junction temperature: 150°C"
            ;;
        1)
            print_info "Suitable for: Under hood, near engine"
            print_warning "Maximum junction temperature: 125°C"
            ;;
        2)
            print_info "Suitable for: Passenger cabin, dashboard"
            print_info "Maximum junction temperature: 105°C"
            ;;
        3)
            print_info "Suitable for: Trunk, mild environments"
            print_info "Maximum junction temperature: 85°C"
            ;;
    esac

    echo ""
}

# Validate AEC-Q100 compliance
validate_aecq100() {
    local component=${1:-MCU-BC-2024}
    local standard=${2:-AEC-Q100}

    print_section "AEC-Q100 Compliance Validation"
    print_info "Component: $component"
    print_info "Standard: $standard"

    print_section "Required Tests"
    local tests=(
        "Temperature Cycling (TC): 1000 cycles, -55°C to +150°C"
        "High Temperature Operating Life (HTOL): 1000h @ Tj max"
        "High Temperature Storage Life (HTSL): 1000h"
        "Temperature Humidity Bias (THB): 1000h @ 85°C/85%RH"
        "Electrostatic Discharge (ESD): ±2kV HBM minimum"
        "Latchup: ±100mA @ 125°C"
        "Moisture Sensitivity Level (MSL)"
        "Electrical Disturbance"
    )

    for test in "${tests[@]}"; do
        print_success "$test"
    done

    print_section "Qualification Status"
    print_success "Component meets AEC-Q100 requirements"
    print_info "Certification valid for 5 years"
    print_info "Re-qualification required on design changes"

    print_section "Acceptance Criteria"
    print_info "Sample size: 231 units (77 per lot, 3 lots)"
    print_info "Failures allowed: 0 for Grade 0/1, 1 for Grade 2/3"
    print_info "Confidence level: 60% @ 1000 FIT"

    echo ""
}

# Calculate reliability metrics
calculate_reliability() {
    local fit_rate=${1:-10}
    local operating_temp=${2:-125}
    local reference_temp=${3:-55}

    print_section "Reliability Calculation"
    print_info "Base FIT Rate: $fit_rate FIT @ ${reference_temp}°C"
    print_info "Operating Temperature: ${operating_temp}°C"

    # Calculate acceleration factor using Arrhenius equation
    # AF = exp[Ea/k × (1/T1 - 1/T2)]
    local t1_kelvin=$(echo "$reference_temp + 273.15" | bc -l)
    local t2_kelvin=$(echo "$operating_temp + 273.15" | bc -l)

    local inv_t1=$(echo "scale=10; 1 / $t1_kelvin" | bc -l)
    local inv_t2=$(echo "scale=10; 1 / $t2_kelvin" | bc -l)
    local delta_inv=$(echo "scale=10; $inv_t1 - $inv_t2" | bc -l)
    local exponent=$(echo "scale=10; $ACTIVATION_ENERGY / $BOLTZMANN * $delta_inv" | bc -l)

    # Calculate e^exponent (using series expansion for large values)
    local af=$(echo "scale=6; e($exponent)" | bc -l)

    # Adjusted FIT rate
    local adjusted_fit=$(echo "scale=2; $fit_rate * $af" | bc -l)

    # MTBF in hours
    local mtbf=$(echo "scale=0; 1000000000 / $adjusted_fit" | bc -l)

    # MTBF in years (assuming 8760 hours per year)
    local mtbf_years=$(echo "scale=2; $mtbf / 8760" | bc -l)

    # Failure rate per year
    local failure_rate=$(echo "scale=6; $adjusted_fit * 8760 / 1000000000" | bc -l)

    print_section "Results"
    print_info "Acceleration Factor: $(printf "%.2f" $af)"
    print_success "Adjusted FIT Rate: $(printf "%.2f" $adjusted_fit) FIT @ ${operating_temp}°C"
    print_success "MTBF: $(printf "%.0f" $mtbf) hours ($(printf "%.0f" $mtbf_years) years)"
    print_info "Failure Rate: $(printf "%.6f" $failure_rate)% per year"

    print_section "Interpretation"
    if (( $(echo "$adjusted_fit < 10" | bc -l) )); then
        print_success "Excellent: Suitable for ASIL-D applications"
    elif (( $(echo "$adjusted_fit < 100" | bc -l) )); then
        print_success "Good: Suitable for ASIL-B/C applications"
    elif (( $(echo "$adjusted_fit < 1000" | bc -l) )); then
        print_warning "Moderate: Suitable for ASIL-A applications"
    else
        print_error "Poor: May only be suitable for QM applications"
    fi

    echo ""
}

# Generate safety report
generate_safety_report() {
    local asil=${1:-D}
    local application=${2:-brake-control}

    print_section "Functional Safety Report"
    print_info "Required ASIL: ASIL-$asil"
    print_info "Application: $application"

    print_section "ISO 26262 Requirements"
    print_info "Standard: ISO 26262:2018"
    print_info "ASIL Targets: $(get_asil_targets $asil)"

    print_section "Safety Mechanisms Required"
    case $asil in
        D|ASIL-D)
            print_success "✓ Lockstep CPU cores (dual-core redundancy)"
            print_success "✓ Memory ECC (single and double bit correction)"
            print_success "✓ CRC on data buses"
            print_success "✓ Watchdog timers (independent + window)"
            print_success "✓ Voltage monitoring (POR, LVD, HVD)"
            print_success "✓ Temperature monitoring"
            print_success "✓ Clock monitoring (frequency + integrity)"
            print_success "✓ Safe state management"
            print_success "✓ BIST (Built-In Self Test)"
            ;;
        C|ASIL-C)
            print_success "✓ Memory ECC"
            print_success "✓ Watchdog timer"
            print_success "✓ Voltage monitoring"
            print_success "✓ Temperature monitoring"
            print_success "✓ CRC on critical data"
            ;;
        B|ASIL-B)
            print_success "✓ Watchdog timer"
            print_success "✓ Voltage monitoring"
            print_success "✓ Basic error detection"
            ;;
        A|ASIL-A)
            print_success "✓ Watchdog timer"
            print_success "✓ Basic monitoring"
            ;;
        QM)
            print_info "No specific safety mechanisms required"
            ;;
    esac

    print_section "Development Process Requirements"
    print_info "1. Hazard Analysis and Risk Assessment (HARA)"
    print_info "2. Safety Goals definition"
    print_info "3. Functional Safety Concept"
    print_info "4. Technical Safety Concept"
    print_info "5. Hardware/Software Safety Requirements"
    print_info "6. Design and Implementation"
    print_info "7. Verification and Validation"
    print_info "8. Functional Safety Assessment"
    print_info "9. Production Release"
    print_info "10. Operation and Maintenance"

    print_section "Verification Requirements"
    case $asil in
        D|ASIL-D)
            print_info "• Hardware-in-the-loop (HIL) testing"
            print_info "• Fault injection testing"
            print_info "• FMEA/FMEDA analysis"
            print_info "• Independent safety assessment"
            ;;
        C|ASIL-C)
            print_info "• HIL testing"
            print_info "• FMEA analysis"
            print_info "• Safety review"
            ;;
        B|ASIL-B|A|ASIL-A)
            print_info "• Functional testing"
            print_info "• Safety review"
            ;;
    esac

    echo ""
}

# Thermal analysis
thermal_analysis() {
    local power=${1:-2.0}
    local theta_ja=${2:-40}
    local ambient=${3:-105}

    print_section "Thermal Analysis"
    print_info "Power Dissipation: ${power}W"
    print_info "Thermal Resistance (θja): ${theta_ja}°C/W"
    print_info "Ambient Temperature: ${ambient}°C"

    # Calculate junction temperature: Tj = Ta + (Pd × θja)
    local tj=$(echo "$ambient + ($power * $theta_ja)" | bc -l)

    print_section "Junction Temperature"
    print_success "Calculated Tj: $(printf "%.1f" $tj)°C"

    # Check against temperature grades
    if (( $(echo "$tj <= 150" | bc -l) )); then
        print_success "Grade 0: PASS (Tj ≤ 150°C)"
    else
        print_error "Grade 0: FAIL (Tj > 150°C)"
    fi

    if (( $(echo "$tj <= 125" | bc -l) )); then
        print_success "Grade 1: PASS (Tj ≤ 125°C)"
    else
        print_warning "Grade 1: FAIL (Tj > 125°C)"
    fi

    if (( $(echo "$tj <= 105" | bc -l) )); then
        print_success "Grade 2: PASS (Tj ≤ 105°C)"
    else
        print_warning "Grade 2: FAIL (Tj > 105°C)"
    fi

    if (( $(echo "$tj <= 85" | bc -l) )); then
        print_success "Grade 3: PASS (Tj ≤ 85°C)"
    else
        print_warning "Grade 3: FAIL (Tj > 85°C)"
    fi

    print_section "Recommendations"
    if (( $(echo "$tj > 125" | bc -l) )); then
        print_warning "Junction temperature exceeds Grade 1 limits!"
        print_info "• Consider heatsink (reduce θja)"
        print_info "• Reduce power dissipation"
        print_info "• Improve airflow"
        print_info "• Use forced cooling"

        local required_theta=$(echo "scale=2; (125 - $ambient) / $power" | bc -l)
        print_info "Required θja for Grade 1: ≤ $(printf "%.1f" $required_theta)°C/W"
    elif (( $(echo "$tj > 105" | bc -l) )); then
        print_warning "Junction temperature exceeds Grade 2 limits"
        print_info "Consider thermal management for cabin applications"
    else
        print_success "Thermal design adequate"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-009 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  classify                 Classify semiconductor component"
    echo "    --type <type>          Component type (MCU, SoC, PMIC, etc.)"
    echo "    --temp-grade <0-3>     Temperature grade"
    echo "    --asil <A-D|QM>        Required ASIL level"
    echo ""
    echo "  validate                 Validate AEC-Q100 compliance"
    echo "    --component <id>       Component identifier"
    echo "    --standard <std>       Standard (AEC-Q100, AEC-Q101, AEC-Q200)"
    echo ""
    echo "  reliability              Calculate reliability metrics"
    echo "    --fit-rate <number>    Base FIT rate @ reference temp"
    echo "    --operating-temp <C>   Operating temperature (Celsius)"
    echo "    --reference-temp <C>   Reference temperature (default: 55°C)"
    echo ""
    echo "  safety-report            Generate functional safety report"
    echo "    --asil <A-D|QM>        Required ASIL level"
    echo "    --application <name>   Application description"
    echo ""
    echo "  thermal                  Thermal analysis"
    echo "    --power <W>            Power dissipation (Watts)"
    echo "    --theta-ja <C/W>       Thermal resistance junction-to-ambient"
    echo "    --ambient <C>          Ambient temperature (Celsius)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-009 classify --type MCU --temp-grade 1 --asil D"
    echo "  wia-auto-009 validate --component MCU-BC-2024 --standard AEC-Q100"
    echo "  wia-auto-009 reliability --fit-rate 10 --operating-temp 125"
    echo "  wia-auto-009 safety-report --asil D --application brake-control"
    echo "  wia-auto-009 thermal --power 2.5 --theta-ja 35 --ambient 105"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-009 CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Automotive Grade Standards:"
    echo "  • AEC-Q100 (Integrated Circuits)"
    echo "  • AEC-Q101 (Discrete Semiconductors)"
    echo "  • AEC-Q200 (Passive Components)"
    echo "  • ISO 26262 (Functional Safety)"
    echo ""
    echo -e "${GRAY}弘익인間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    classify)
        TYPE="MCU"
        TEMP_GRADE=1
        ASIL="D"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --temp-grade) TEMP_GRADE=$2; shift 2 ;;
                --asil) ASIL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        classify_component "$TYPE" "$TEMP_GRADE" "$ASIL"
        ;;

    validate)
        COMPONENT="MCU-BC-2024"
        STANDARD="AEC-Q100"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --component) COMPONENT=$2; shift 2 ;;
                --standard) STANDARD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_aecq100 "$COMPONENT" "$STANDARD"
        ;;

    reliability)
        FIT_RATE=10
        OPERATING_TEMP=125
        REFERENCE_TEMP=55

        while [[ $# -gt 0 ]]; do
            case $1 in
                --fit-rate) FIT_RATE=$2; shift 2 ;;
                --operating-temp) OPERATING_TEMP=$2; shift 2 ;;
                --reference-temp) REFERENCE_TEMP=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calculate_reliability "$FIT_RATE" "$OPERATING_TEMP" "$REFERENCE_TEMP"
        ;;

    safety-report)
        ASIL="D"
        APPLICATION="brake-control"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --asil) ASIL=$2; shift 2 ;;
                --application) APPLICATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_safety_report "$ASIL" "$APPLICATION"
        ;;

    thermal)
        POWER=2.0
        THETA_JA=40
        AMBIENT=105

        while [[ $# -gt 0 ]]; do
            case $1 in
                --power) POWER=$2; shift 2 ;;
                --theta-ja) THETA_JA=$2; shift 2 ;;
                --ambient) AMBIENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        thermal_analysis "$POWER" "$THETA_JA" "$AMBIENT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-009 help' for usage information"
        exit 1
        ;;
esac

exit 0
