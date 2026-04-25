#!/bin/bash

################################################################################
# WIA-BIO-005: Cellular Therapy CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biotechnology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to cellular therapy calculations
# including dose calculations, quality assessment, and safety monitoring.
################################################################################

set -e

# Colors for output
TEAL='\033[0;36m'
CYAN='\033[0;96m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
MIN_VIABILITY=85
MIN_POTENCY=70
MAX_ENDOTOXIN=5

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🔬 WIA-BIO-005: Cellular Therapy CLI                ║"
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

format_scientific() {
    local number=$1
    if command -v bc &> /dev/null; then
        printf "%.2e" "$number"
    else
        echo "$number"
    fi
}

# Calculate cell dose
calc_dose() {
    local total_cells=${1:-5e8}
    local weight=${2:-70}
    local viability=${3:-92.5}
    local car_percent=${4:-0}

    print_section "Dose Calculation"
    print_info "Total Cells: $(format_scientific $total_cells)"
    print_info "Patient Weight: $weight kg"
    print_info "Viability: ${viability}%"
    if [ "$car_percent" != "0" ]; then
        print_info "CAR+ Percentage: ${car_percent}%"
    fi

    # Calculate viable cells
    local viable_cells=$(echo "$total_cells * $viability / 100" | bc -l)
    print_info "Viable Cells: $(format_scientific $viable_cells)"

    # Calculate CAR+ cells if applicable
    if [ "$car_percent" != "0" ]; then
        local car_cells=$(echo "$viable_cells * $car_percent / 100" | bc -l)
        print_info "CAR+ Cells: $(format_scientific $car_cells)"
        viable_cells=$car_cells
    fi

    # Calculate dose
    local dose=$(echo "$viable_cells / $weight" | bc -l)

    print_section "Results"
    print_success "Dose: $(format_scientific $dose) cells/kg"

    # Check dose range for CAR-T (1e6 - 5e6 cells/kg)
    local min_dose=1000000
    local max_dose=5000000

    if (( $(echo "$dose >= $min_dose && $dose <= $max_dose" | bc -l) )); then
        print_success "Dose is within recommended range (1e6 - 5e6 cells/kg)"
    elif (( $(echo "$dose < $min_dose" | bc -l) )); then
        print_warning "Dose below recommended minimum (1e6 cells/kg)"
        print_info "Consider collecting more cells or adjusting manufacturing"
    else
        print_warning "Dose exceeds recommended maximum (5e6 cells/kg)"
        print_info "Consider dose reduction to minimize toxicity risk"
    fi

    echo ""
}

# Assess quality control
assess_quality() {
    local viability=${1:-92.5}
    local potency=${2:-85}
    local endotoxin=${3:-0.3}
    local sterility=${4:-true}
    local mycoplasma=${5:-false}

    print_section "Quality Control Assessment"
    print_info "Viability: ${viability}%"
    print_info "Potency: ${potency}%"
    print_info "Endotoxin: ${endotoxin} EU/mL"
    print_info "Sterility: $sterility"
    print_info "Mycoplasma: $mycoplasma"

    print_section "Release Criteria"

    local pass_count=0
    local fail_count=0

    # Check viability
    if (( $(echo "$viability >= $MIN_VIABILITY" | bc -l) )); then
        print_success "Viability: PASS (≥${MIN_VIABILITY}%)"
        ((pass_count++))
    else
        print_error "Viability: FAIL (${viability}% < ${MIN_VIABILITY}%)"
        ((fail_count++))
    fi

    # Check potency
    if (( $(echo "$potency >= $MIN_POTENCY" | bc -l) )); then
        print_success "Potency: PASS (≥${MIN_POTENCY}%)"
        ((pass_count++))
    else
        print_error "Potency: FAIL (${potency}% < ${MIN_POTENCY}%)"
        ((fail_count++))
    fi

    # Check endotoxin
    if (( $(echo "$endotoxin < $MAX_ENDOTOXIN" | bc -l) )); then
        print_success "Endotoxin: PASS (<${MAX_ENDOTOXIN} EU/mL)"
        ((pass_count++))
    else
        print_error "Endotoxin: FAIL (${endotoxin} ≥ ${MAX_ENDOTOXIN} EU/mL)"
        ((fail_count++))
    fi

    # Check sterility
    if [ "$sterility" = "true" ]; then
        print_success "Sterility: PASS (No growth)"
        ((pass_count++))
    else
        print_error "Sterility: FAIL (Contamination detected)"
        ((fail_count++))
    fi

    # Check mycoplasma
    if [ "$mycoplasma" = "false" ]; then
        print_success "Mycoplasma: PASS (Negative)"
        ((pass_count++))
    else
        print_error "Mycoplasma: FAIL (Positive)"
        ((fail_count++))
    fi

    print_section "Overall Assessment"

    if [ $fail_count -eq 0 ]; then
        if (( $(echo "$viability >= 95 && $potency >= 85" | bc -l) )); then
            print_success "Grade: A (Excellent)"
        elif (( $(echo "$viability >= 90 && $potency >= 80" | bc -l) )); then
            print_success "Grade: B (Good)"
        else
            print_success "Grade: C (Acceptable)"
        fi
        print_success "Recommendation: RELEASE"
    else
        print_error "Grade: F (Failed)"
        if [ "$sterility" = "false" ] || [ "$mycoplasma" = "true" ]; then
            print_error "Recommendation: REJECT (Contamination)"
        else
            print_warning "Recommendation: RETEST"
        fi
    fi

    echo ""
}

# Track manufacturing batch
track_batch() {
    local batch_id=${1:-"CT-2024-001"}
    local stage=${2:-"expansion"}
    local day=${3:-7}
    local cell_count=${4:-5e7}
    local viability=${5:-92}

    print_section "Manufacturing Batch Tracking"
    print_info "Batch ID: $batch_id"
    print_info "Current Stage: $stage"
    print_info "Day in Culture: $day"
    print_info "Cell Count: $(format_scientific $cell_count)"
    print_info "Viability: ${viability}%"

    print_section "Process Status"

    # Check stage progress
    case $stage in
        "collection")
            print_info "Status: Collection complete"
            print_info "Next: Cell activation (24-48 hours)"
            ;;
        "activation")
            print_info "Status: T cell activation in progress"
            print_info "Next: Transduction (48-72 hours)"
            ;;
        "transduction")
            print_info "Status: Viral transduction in progress"
            print_info "Next: Expansion (7-10 days)"
            ;;
        "expansion")
            print_info "Status: Cell expansion in progress"
            local expected_completion=$((day + 3))
            print_info "Expected Completion: Day $expected_completion"
            ;;
        "harvest")
            print_info "Status: Harvest and formulation"
            print_info "Next: Cryopreservation (24 hours)"
            ;;
        "cryopreservation")
            print_info "Status: Cryopreservation complete"
            print_success "Manufacturing complete"
            ;;
        *)
            print_warning "Unknown stage"
            ;;
    esac

    # Check viability
    if (( $(echo "$viability >= 90" | bc -l) )); then
        print_success "Viability: Excellent (${viability}%)"
    elif (( $(echo "$viability >= 85" | bc -l) )); then
        print_success "Viability: Acceptable (${viability}%)"
    else
        print_warning "Viability: Below threshold (${viability}% < 85%)"
    fi

    # Check expansion (if in expansion stage)
    if [ "$stage" = "expansion" ]; then
        local starting_cells=5e6
        local expansion=$(echo "$cell_count / $starting_cells" | bc -l)
        print_info "Expansion Ratio: $(printf "%.1f" $expansion)×"

        if (( $(echo "$expansion >= 100" | bc -l) )); then
            print_success "Expansion: On target (≥100×)"
        else
            print_warning "Expansion: Below target (<100×)"
        fi
    fi

    echo ""
}

# Monitor patient safety
monitor_safety() {
    local patient_id=${1:-"PT-001"}
    local days_post=${2:-7}
    local crs_grade=${3:-0}
    local icans_grade=${4:-0}

    print_section "Patient Safety Monitoring"
    print_info "Patient ID: $patient_id"
    print_info "Days Post-Infusion: $days_post"

    print_section "Toxicity Assessment"

    # CRS Assessment
    print_info "CRS Grade: $crs_grade"
    case $crs_grade in
        0)
            print_success "CRS: None"
            ;;
        1)
            print_warning "CRS Grade 1: Fever only"
            print_info "Treatment: Supportive care, antipyretics"
            ;;
        2)
            print_warning "CRS Grade 2: Hypotension and/or hypoxia"
            print_info "Treatment: Tocilizumab 8 mg/kg IV"
            ;;
        3|4)
            print_error "CRS Grade $crs_grade: Severe"
            print_info "Treatment: Tocilizumab + Steroids, ICU care"
            ;;
    esac

    # ICANS Assessment
    print_info "ICANS Grade: $icans_grade"
    case $icans_grade in
        0)
            print_success "ICANS: None"
            ;;
        1)
            print_warning "ICANS Grade 1: Mild neurotoxicity"
            print_info "Treatment: Monitor ICE score q4-6h"
            ;;
        2)
            print_warning "ICANS Grade 2: Moderate neurotoxicity"
            print_info "Treatment: Dexamethasone 10 mg q6h"
            ;;
        3|4)
            print_error "ICANS Grade $icans_grade: Severe"
            print_info "Treatment: High-dose steroids, ICU care"
            ;;
    esac

    print_section "Monitoring Recommendations"

    if [ $days_post -le 10 ]; then
        print_info "Inpatient monitoring required"
        print_info "Vital signs and neuro checks q4-6h"
        print_info "Tocilizumab should be readily available"
    elif [ $days_post -le 30 ]; then
        print_info "Patient should remain within 2 hours of center"
        print_info "Weekly clinic visits required"
    else
        print_success "High-risk period passed"
        print_info "Continue routine follow-up"
    fi

    if [ $crs_grade -ge 2 ] || [ $icans_grade -ge 2 ]; then
        print_warning "Active Grade ≥2 toxicity - urgent intervention required"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-005 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-dose                Calculate patient dose"
    echo "    --cells <number>       Total cell count (default: 5e8)"
    echo "    --weight <kg>          Patient weight (default: 70 kg)"
    echo "    --viability <percent>  Cell viability (default: 92.5%)"
    echo "    --car <percent>        CAR+ percentage (optional)"
    echo ""
    echo "  assess-quality           Assess quality control metrics"
    echo "    --viability <percent>  Cell viability (default: 92.5%)"
    echo "    --potency <percent>    Potency (default: 85%)"
    echo "    --endotoxin <EU/mL>    Endotoxin level (default: 0.3)"
    echo "    --sterility <bool>     Sterility (default: true)"
    echo "    --mycoplasma <bool>    Mycoplasma (default: false)"
    echo ""
    echo "  track-batch              Track manufacturing batch"
    echo "    --batch <id>           Batch identifier"
    echo "    --stage <stage>        Manufacturing stage"
    echo "    --day <number>         Day in culture"
    echo "    --cells <number>       Current cell count"
    echo "    --viability <percent>  Current viability"
    echo ""
    echo "  monitor-safety           Monitor patient safety"
    echo "    --patient <id>         Patient identifier"
    echo "    --days <number>        Days post-infusion"
    echo "    --crs <grade>          CRS grade (0-4)"
    echo "    --icans <grade>        ICANS grade (0-4)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-005 calc-dose --cells 5e8 --weight 70 --viability 92.5"
    echo "  wia-bio-005 assess-quality --viability 92.5 --potency 85 --endotoxin 0.3"
    echo "  wia-bio-005 track-batch --batch CT-2024-001 --stage expansion"
    echo "  wia-bio-005 monitor-safety --patient PT-001 --days 7 --crs 1 --icans 0"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-005 Cellular Therapy CLI Tool"
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
    calc-dose)
        CELLS=5e8
        WEIGHT=70
        VIABILITY=92.5
        CAR_PERCENT=0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --cells) CELLS=$2; shift 2 ;;
                --weight) WEIGHT=$2; shift 2 ;;
                --viability) VIABILITY=$2; shift 2 ;;
                --car) CAR_PERCENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_dose "$CELLS" "$WEIGHT" "$VIABILITY" "$CAR_PERCENT"
        ;;

    assess-quality)
        VIABILITY=92.5
        POTENCY=85
        ENDOTOXIN=0.3
        STERILITY=true
        MYCOPLASMA=false

        while [[ $# -gt 0 ]]; do
            case $1 in
                --viability) VIABILITY=$2; shift 2 ;;
                --potency) POTENCY=$2; shift 2 ;;
                --endotoxin) ENDOTOXIN=$2; shift 2 ;;
                --sterility) STERILITY=$2; shift 2 ;;
                --mycoplasma) MYCOPLASMA=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_quality "$VIABILITY" "$POTENCY" "$ENDOTOXIN" "$STERILITY" "$MYCOPLASMA"
        ;;

    track-batch)
        BATCH="CT-2024-001"
        STAGE="expansion"
        DAY=7
        CELLS=5e7
        VIABILITY=92

        while [[ $# -gt 0 ]]; do
            case $1 in
                --batch) BATCH=$2; shift 2 ;;
                --stage) STAGE=$2; shift 2 ;;
                --day) DAY=$2; shift 2 ;;
                --cells) CELLS=$2; shift 2 ;;
                --viability) VIABILITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        track_batch "$BATCH" "$STAGE" "$DAY" "$CELLS" "$VIABILITY"
        ;;

    monitor-safety)
        PATIENT="PT-001"
        DAYS=7
        CRS=0
        ICANS=0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT=$2; shift 2 ;;
                --days) DAYS=$2; shift 2 ;;
                --crs) CRS=$2; shift 2 ;;
                --icans) ICANS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_safety "$PATIENT" "$DAYS" "$CRS" "$ICANS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-005 help' for usage information"
        exit 1
        ;;
esac

exit 0
