#!/bin/bash

################################################################################
# WIA-BIO-003: Gene Therapy CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biotechnology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to gene therapy calculations
# including dosage optimization, safety assessment, and protocol generation.
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
AAV_MAX_DOSE=2e14
STANDARD_DOSE=1e13
LOW_DOSE=1e12

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🧬 WIA-BIO-003: Gene Therapy CLI Tool              ║"
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

format_number() {
    local num=$1
    if command -v numfmt &> /dev/null; then
        numfmt --to=si "$num"
    else
        printf "%.2e" "$num"
    fi
}

# Calculate dosage
calc_dosage() {
    local weight=${1:-70}
    local tissue=${2:-liver}
    local vector=${3:-AAV9}

    print_section "Dosage Calculation"
    print_info "Patient Weight: $weight kg"
    print_info "Target Tissue: $tissue"
    print_info "Vector Type: $vector"

    # Tissue-specific parameters
    local cell_count=0
    local vg_per_cell=0

    case $tissue in
        liver)
            cell_count=200000000000  # 2×10^11
            vg_per_cell=100000       # 1×10^5
            ;;
        muscle)
            cell_count=500000000000  # 5×10^11
            vg_per_cell=10000        # 1×10^4
            ;;
        CNS)
            cell_count=100000000000  # 1×10^11
            vg_per_cell=100000       # 1×10^5
            ;;
        *)
            print_error "Unknown tissue type"
            return 1
            ;;
    esac

    # Calculate total viral genomes
    local total_vg=$(echo "$cell_count * $vg_per_cell" | bc -l)
    local vg_per_kg=$(echo "$total_vg / $weight" | bc -l)

    # Calculate volume (assuming titer of 1×10^13 vg/mL)
    local titer=10000000000000  # 1×10^13
    local volume_ml=$(echo "$total_vg / $titer" | bc -l)

    print_section "Results"
    print_success "Total Viral Genomes: $(printf '%.2e' $total_vg) vg"
    print_success "Dose per kg: $(printf '%.2e' $vg_per_kg) vg/kg"
    print_info "Injection Volume: $(printf '%.1f' $volume_ml) mL"

    # Assess feasibility
    if (( $(echo "$vg_per_kg < $STANDARD_DOSE" | bc -l) )); then
        print_success "Dose Level: STANDARD (Safe for clinical use)"
    elif (( $(echo "$vg_per_kg < $AAV_MAX_DOSE" | bc -l) )); then
        print_warning "Dose Level: HIGH (Enhanced monitoring required)"
    else
        print_error "Dose Level: EXCEEDS MTD (Not recommended)"
    fi

    # Expected efficiency
    local efficiency=70
    case $vector in
        AAV8) efficiency=80 ;;
        AAV9) efficiency=70 ;;
        AAV1) efficiency=60 ;;
    esac

    print_info "Expected Transduction: $efficiency%"

    echo ""
}

# Assess safety
assess_safety() {
    local dose=${1:-1e13}
    local immune_status=${2:-normal}
    local alt=${3:-25}
    local ast=${4:-30}

    print_section "Safety Assessment"
    print_info "Dose: $(printf '%.2e' $dose) vg/kg"
    print_info "Immune Status: $immune_status"
    print_info "ALT: $alt U/L"
    print_info "AST: $ast U/L"

    local score=100
    local warnings=()

    # Check liver function
    if (( $(echo "$alt > 60" | bc -l) )) || (( $(echo "$ast > 60" | bc -l) )); then
        score=$((score - 25))
        warnings+=("Elevated liver enzymes detected")
    fi

    # Check dose level
    if (( $(echo "$dose > $AAV_MAX_DOSE" | bc -l) )); then
        score=$((score - 30))
        warnings+=("Dose exceeds maximum tolerated dose")
    elif (( $(echo "$dose > $STANDARD_DOSE" | bc -l) )); then
        score=$((score - 15))
        warnings+=("High-dose regimen increases risk")
    fi

    # Check immune status
    if [ "$immune_status" = "autoimmune" ]; then
        score=$((score - 20))
        warnings+=("Autoimmune condition increases immune response risk")
    fi

    print_section "Safety Score"

    if [ $score -ge 80 ]; then
        print_success "Safety Score: $score/100 (LOW RISK)"
        print_info "Patient eligible for treatment"
    elif [ $score -ge 60 ]; then
        print_warning "Safety Score: $score/100 (MEDIUM RISK)"
        print_info "Enhanced monitoring recommended"
    elif [ $score -ge 40 ]; then
        print_error "Safety Score: $score/100 (HIGH RISK)"
        print_info "Proceed with caution, risk mitigation required"
    else
        print_error "Safety Score: $score/100 (EXTREME RISK)"
        print_info "Treatment not recommended"
    fi

    if [ ${#warnings[@]} -gt 0 ]; then
        print_section "Warnings"
        for warning in "${warnings[@]}"; do
            print_warning "$warning"
        done
    fi

    print_section "Recommendations"
    if [ $score -lt 80 ]; then
        print_info "• Weekly ALT/AST monitoring for 12 weeks"
        print_info "• Consider prophylactic corticosteroids"
    fi
    if (( $(echo "$dose > $STANDARD_DOSE" | bc -l) )); then
        print_info "• Daily platelet monitoring for TMA"
        print_info "• Nephrology consult on standby"
    fi

    echo ""
}

# Predict efficiency
predict_efficiency() {
    local vector=${1:-AAV9}
    local tissue=${2:-liver}

    print_section "Transduction Efficiency Prediction"
    print_info "Vector: $vector"
    print_info "Target Tissue: $tissue"

    local efficiency=50

    case "$tissue-$vector" in
        liver-AAV8) efficiency=80 ;;
        liver-AAV9) efficiency=70 ;;
        muscle-AAV1) efficiency=60 ;;
        muscle-AAV9) efficiency=50 ;;
        CNS-AAV9) efficiency=40 ;;
        retina-AAV2) efficiency=70 ;;
        *) efficiency=50 ;;
    esac

    print_section "Results"
    print_success "Expected Efficiency: $efficiency%"

    if [ $efficiency -ge 70 ]; then
        print_success "Highly efficient transduction expected"
    elif [ $efficiency -ge 50 ]; then
        print_info "Moderate efficiency expected"
    else
        print_warning "Lower efficiency - consider dose adjustment"
    fi

    local optimized_dose=$(echo "scale=2; 100 / $efficiency" | bc)
    print_info "Dose adjustment factor: ${optimized_dose}x for 100% coverage"

    echo ""
}

# Monitor expression
monitor_expression() {
    local gene=${1:-F8}
    local timepoints=${2:-7,14,28,90}

    print_section "Gene Expression Monitoring"
    print_info "Therapeutic Gene: $gene"
    print_info "Time Points: $timepoints days"

    print_section "Expected Expression Timeline"

    IFS=',' read -ra POINTS <<< "$timepoints"
    for day in "${POINTS[@]}"; do
        # Simulate expression kinetics
        local expression=0
        if [ $day -lt 7 ]; then
            expression=10
        elif [ $day -lt 28 ]; then
            expression=$(echo "scale=1; 10 + (90 * ($day - 7) / 21)" | bc)
        elif [ $day -lt 84 ]; then
            expression=$(echo "scale=1; 100 - (10 * ($day - 28) / 56)" | bc)
        else
            expression=90
        fi

        if (( $(echo "$expression >= 80" | bc -l) )); then
            print_success "Day $day: ${expression}% of normal (Therapeutic range)"
        elif (( $(echo "$expression >= 40" | bc -l) )); then
            print_info "Day $day: ${expression}% of normal (Rising)"
        else
            print_warning "Day $day: ${expression}% of normal (Subtherapeutic)"
        fi
    done

    print_section "Recommendations"
    print_info "• Peak expression expected at Day 28"
    print_info "• Plateau phase begins around Day 84"
    print_info "• Monitor with qPCR and functional assays"
    print_info "• Consider dose adjustment if <40% at Week 12"

    echo ""
}

# Generate clinical protocol
generate_protocol() {
    local condition=${1:-hemophilia}
    local vector=${2:-AAV9}

    print_section "Clinical Trial Protocol Generator"
    print_info "Condition: $condition"
    print_info "Vector: $vector"

    local gene="F8"
    case $condition in
        hemophilia*A|hemophilia) gene="F8" ;;
        hemophilia*B) gene="F9" ;;
        SMA) gene="SMN1" ;;
        DMD) gene="DMD" ;;
    esac

    print_section "Protocol Overview"
    print_success "Protocol ID: WIA-BIO-003-$(date +%s)-$vector"
    print_info "Phase: I/II"
    print_info "Therapeutic Gene: $gene"
    print_info "Delivery Vector: $vector"

    print_section "Inclusion Criteria"
    print_info "• Confirmed diagnosis of $condition"
    print_info "• Age ≥18 years"
    print_info "• AAV neutralizing antibody titer <1:5"
    print_info "• Normal liver function (ALT/AST <1.5× ULN)"
    print_info "• Willing to use contraception for 6 months"

    print_section "Exclusion Criteria"
    print_info "• Active hepatitis B or C"
    print_info "• HIV positive"
    print_info "• Active malignancy"
    print_info "• Pregnancy or nursing"
    print_info "• Prior gene therapy with same serotype"

    print_section "Dose Escalation (3+3 Design)"
    print_info "Cohort 1: 1×10¹² vg/kg (n=3)"
    print_info "Cohort 2: 3×10¹² vg/kg (n=3)"
    print_info "Cohort 3: 1×10¹³ vg/kg (n=3)"
    print_info "Cohort 4: 3×10¹³ vg/kg (n=3)"
    print_info "Cohort 5: 1×10¹⁴ vg/kg (n=3)"

    print_section "Primary Endpoints"
    print_success "• Safety and tolerability"
    print_success "• Incidence of dose-limiting toxicities"
    print_success "• Maximum tolerated dose"

    print_section "Secondary Endpoints"
    print_info "• Transgene expression levels"
    print_info "• Clinical improvement in $condition phenotype"
    print_info "• Quality of life scores"
    print_info "• Immunogenicity assessment"

    print_section "Monitoring Schedule"
    print_info "Day 0: Administration + 24h observation"
    print_info "Week 1: Daily labs (CBC, CMP, LFTs)"
    print_info "Week 2-4: Weekly labs + transgene expression"
    print_info "Month 2-6: Monthly follow-up + functional tests"
    print_info "Year 1-5: Quarterly then annual long-term monitoring"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-003 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-dosage              Calculate optimal viral vector dosage"
    echo "    --weight <kg>          Patient weight (default: 70 kg)"
    echo "    --tissue <type>        Target tissue: liver, muscle, CNS (default: liver)"
    echo "    --vector <type>        Vector type: AAV9, AAV8, AAV1 (default: AAV9)"
    echo ""
    echo "  assess-safety            Assess patient safety profile"
    echo "    --dose <vg/kg>         Dose in vg/kg (default: 1e13)"
    echo "    --immune-status <str>  Immune status: normal, autoimmune (default: normal)"
    echo "    --alt <U/L>            ALT level (default: 25)"
    echo "    --ast <U/L>            AST level (default: 30)"
    echo ""
    echo "  predict-efficiency       Predict transduction efficiency"
    echo "    --vector <type>        Vector type (default: AAV9)"
    echo "    --tissue <type>        Target tissue (default: liver)"
    echo ""
    echo "  monitor-expression       Monitor gene expression timeline"
    echo "    --gene <symbol>        Gene symbol (default: F8)"
    echo "    --time-points <days>   Comma-separated days (default: 7,14,28,90)"
    echo ""
    echo "  generate-protocol        Generate clinical trial protocol"
    echo "    --condition <disease>  Disease name (default: hemophilia)"
    echo "    --vector <type>        Vector type (default: AAV9)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-003 calc-dosage --weight 70 --tissue liver --vector AAV9"
    echo "  wia-bio-003 assess-safety --dose 1e13 --immune-status normal"
    echo "  wia-bio-003 predict-efficiency --vector AAV8 --tissue liver"
    echo "  wia-bio-003 monitor-expression --gene F8 --time-points 7,14,28,90"
    echo "  wia-bio-003 generate-protocol --condition hemophilia --vector AAV9"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-003 Gene Therapy CLI Tool"
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
    calc-dosage)
        WEIGHT=70
        TISSUE="liver"
        VECTOR="AAV9"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --weight) WEIGHT=$2; shift 2 ;;
                --tissue) TISSUE=$2; shift 2 ;;
                --vector) VECTOR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_dosage "$WEIGHT" "$TISSUE" "$VECTOR"
        ;;

    assess-safety)
        DOSE=1e13
        IMMUNE="normal"
        ALT=25
        AST=30

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dose) DOSE=$2; shift 2 ;;
                --immune-status) IMMUNE=$2; shift 2 ;;
                --alt) ALT=$2; shift 2 ;;
                --ast) AST=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_safety "$DOSE" "$IMMUNE" "$ALT" "$AST"
        ;;

    predict-efficiency)
        VECTOR="AAV9"
        TISSUE="liver"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --vector) VECTOR=$2; shift 2 ;;
                --tissue) TISSUE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        predict_efficiency "$VECTOR" "$TISSUE"
        ;;

    monitor-expression)
        GENE="F8"
        TIMEPOINTS="7,14,28,90"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --gene) GENE=$2; shift 2 ;;
                --time-points) TIMEPOINTS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_expression "$GENE" "$TIMEPOINTS"
        ;;

    generate-protocol)
        CONDITION="hemophilia"
        VECTOR="AAV9"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --condition) CONDITION=$2; shift 2 ;;
                --vector) VECTOR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_protocol "$CONDITION" "$VECTOR"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-003 help' for usage information"
        exit 1
        ;;
esac

exit 0
