#!/bin/bash

################################################################################
# WIA-BIO-016: Biopharma CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biopharmaceutical Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to biopharmaceutical calculations
# including binding affinity, PK/PD modeling, and immunogenicity assessment.
################################################################################

set -e

# Colors for output
TEAL='\033[0;36m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
AVOGADRO=6.022e23
IGG_MW=150000
IGG_HALF_LIFE=21
MIN_THERAPEUTIC_KD=10e-9

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║             💉 WIA-BIO-016: Biopharma CLI Tool                ║"
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
    local value=$1
    printf "%.2e" "$value"
}

# Calculate binding affinity
calc_affinity() {
    local ab_conc=${1:-1e-9}
    local ag_conc=${2:-1e-8}
    local complex_conc=${3:-0.9e-9}

    print_section "Binding Affinity Calculation"
    print_info "Antibody concentration: $(format_scientific $ab_conc) M"
    print_info "Antigen concentration: $(format_scientific $ag_conc) M"
    print_info "Complex concentration: $(format_scientific $complex_conc) M"

    # Calculate free concentrations
    local free_ab=$(echo "$ab_conc - $complex_conc" | bc -l)
    local free_ag=$(echo "$ag_conc - $complex_conc" | bc -l)

    print_info "Free antibody: $(format_scientific $free_ab) M"
    print_info "Free antigen: $(format_scientific $free_ag) M"

    # Calculate Kd: Kd = [Ab][Ag] / [Ab-Ag]
    local kd=$(echo "$free_ab * $free_ag / $complex_conc" | bc -l)

    # Calculate fraction bound
    local fraction_bound=$(echo "$complex_conc / $ab_conc" | bc -l)

    print_section "Results"
    print_success "Dissociation constant (Kd): $(format_scientific $kd) M"
    print_info "Fraction bound: $(printf "%.2f" $fraction_bound) ($(printf "%.1f" $(echo "$fraction_bound * 100" | bc -l))%)"

    # Classify binding strength
    if (( $(echo "$kd < 1e-12" | bc -l) )); then
        print_success "Binding strength: ULTRA-HIGH"
    elif (( $(echo "$kd < 100e-12" | bc -l) )); then
        print_success "Binding strength: VERY HIGH"
    elif (( $(echo "$kd < 1e-9" | bc -l) )); then
        print_success "Binding strength: HIGH"
    elif (( $(echo "$kd < 10e-9" | bc -l) )); then
        print_warning "Binding strength: MODERATE"
    else
        print_error "Binding strength: LOW"
    fi

    # Therapeutic suitability
    if (( $(echo "$kd <= $MIN_THERAPEUTIC_KD" | bc -l) )); then
        print_success "Therapeutic suitability: SUITABLE"
    else
        print_warning "Therapeutic suitability: BORDERLINE/UNSUITABLE"
    fi

    echo ""
}

# Calculate pharmacokinetics
calc_pk() {
    local dose=${1:-100}
    local bioavailability=${2:-1.0}
    local clearance=${3:-0.2}
    local volume=${4:-5}

    print_section "Pharmacokinetic Calculation"
    print_info "Dose: $dose mg"
    print_info "Bioavailability: $bioavailability ($(echo "$bioavailability * 100" | bc -l)%)"
    print_info "Clearance: $clearance L/h"
    print_info "Volume of distribution: $volume L"

    # Calculate AUC: AUC = (Dose × F) / CL
    local auc=$(echo "$dose * $bioavailability / $clearance" | bc -l)
    print_info "AUC: $(printf "%.2f" $auc) mg·h/L"

    # Calculate elimination rate constant: ke = CL / Vd
    local ke=$(echo "$clearance / $volume" | bc -l)

    # Calculate half-life: t½ = 0.693 / ke
    local half_life=$(echo "0.693 / $ke" | bc -l)

    # Calculate initial concentration: C0 = Dose / Vd
    local c0=$(echo "$dose / $volume" | bc -l)

    print_section "Results"
    print_success "AUC: $(printf "%.2f" $auc) mg·h/L"
    print_success "Half-life: $(printf "%.2f" $half_life) hours"
    print_success "Initial concentration (C0): $(printf "%.2f" $c0) mg/L"
    print_info "Elimination rate constant (ke): $(printf "%.4f" $ke) h⁻¹"

    # Recommend dosing interval
    if (( $(echo "$half_life < 12" | bc -l) )); then
        local interval=$(echo "$half_life / 2" | bc -l)
        if (( $(echo "$interval < 4" | bc -l) )); then
            interval=4
        fi
        print_info "Recommended dosing interval: $(printf "%.0f" $interval) hours (every $(printf "%.0f" $interval)h)"
    elif (( $(echo "$half_life < 48" | bc -l) )); then
        print_info "Recommended dosing interval: 24 hours (once daily)"
    else
        print_info "Recommended dosing interval: $(printf "%.0f" $half_life) hours"
    fi

    echo ""
}

# Assess immunogenicity
assess_immunogenicity() {
    local ada_rate=${1:-0.15}
    local severity=${2:-5}
    local duration=${3:-2}
    local tolerance=${4:-0.5}

    print_section "Immunogenicity Risk Assessment"
    print_info "ADA incidence rate: $ada_rate ($(echo "$ada_rate * 100" | bc -l)%)"
    print_info "Clinical severity: $severity (1-10)"
    print_info "Duration factor: $duration"
    print_info "Tolerance factor: $tolerance"

    # Calculate immunogenicity risk score
    # IRS = (ADA_rate × Severity × Duration) / Tolerance
    local irs=$(echo "$ada_rate * $severity * $duration / $tolerance" | bc -l)

    print_section "Results"
    print_success "Immunogenicity Risk Score (IRS): $(printf "%.2f" $irs)"

    # Classify risk level
    if (( $(echo "$irs < 10" | bc -l) )); then
        print_success "Risk Level: LOW"
        print_info "Monitoring: Standard"
    elif (( $(echo "$irs < 30" | bc -l) )); then
        print_warning "Risk Level: MODERATE"
        print_info "Monitoring: Frequent"
    elif (( $(echo "$irs < 60" | bc -l) )); then
        print_error "Risk Level: HIGH"
        print_info "Monitoring: Intensive"
    else
        print_error "Risk Level: VERY HIGH"
        print_info "Monitoring: Intensive + Consider alternatives"
    fi

    # Predicted ADA incidence
    local predicted_ada=$(echo "$ada_rate * (1 + $irs / 100)" | bc -l)
    if (( $(echo "$predicted_ada > 1" | bc -l) )); then
        predicted_ada=1
    fi
    print_info "Predicted ADA incidence: $(printf "%.1f" $(echo "$predicted_ada * 100" | bc -l))%"

    # Recommendations
    print_section "Recommendations"
    print_info "• Monitor ADA levels during treatment"
    if (( $(echo "$irs > 20" | bc -l) )); then
        print_info "• Consider sequence optimization/humanization"
        print_info "• Deimmunize T-cell epitopes"
    fi
    if (( $(echo "$irs > 40" | bc -l) )); then
        print_info "• Consider alternative format (Fab, scFv)"
        print_info "• Immunosuppression protocol may be needed"
    fi

    echo ""
}

# Validate drug candidate
validate_candidate() {
    local drug_type=${1:-monoclonal-antibody}
    local affinity=${2:-1e-9}
    local stability=${3:-0.95}
    local immuno_risk=${4:-low}

    print_section "Drug Candidate Validation"
    print_info "Drug type: $drug_type"
    print_info "Target affinity (Kd): $(format_scientific $affinity) M"
    print_info "Stability: $stability ($(echo "$stability * 100" | bc -l)%)"
    print_info "Immunogenicity risk: $immuno_risk"

    local errors=0
    local warnings=0

    print_section "Validation Checks"

    # Affinity check
    if (( $(echo "$affinity > $MIN_THERAPEUTIC_KD" | bc -l) )); then
        print_warning "Affinity: Weaker than typical therapeutic threshold"
        warnings=$((warnings + 1))
    else
        print_success "Affinity: Within therapeutic range"
    fi

    if (( $(echo "$affinity > 100e-9" | bc -l) )); then
        print_error "Affinity: Too weak for therapeutic application"
        errors=$((errors + 1))
    fi

    # Stability check
    if (( $(echo "$stability < 0.9" | bc -l) )); then
        print_warning "Stability: Below optimal threshold (90%)"
        warnings=$((warnings + 1))
    else
        print_success "Stability: Acceptable"
    fi

    if (( $(echo "$stability < 0.7" | bc -l) )); then
        print_error "Stability: Insufficient for development"
        errors=$((errors + 1))
    fi

    # Immunogenicity check
    if [ "$immuno_risk" = "high" ]; then
        print_warning "Immunogenicity: High risk detected"
        warnings=$((warnings + 1))
    else
        print_success "Immunogenicity: Acceptable risk"
    fi

    print_section "Validation Result"

    if [ $errors -eq 0 ]; then
        print_success "Candidate is VALID for development"

        if [ $warnings -eq 0 ]; then
            print_success "Feasibility: HIGH"
            print_info "Estimated timeline: 8-10 years"
            print_info "Estimated cost: \$1,000-1,500M"
        else
            print_warning "Feasibility: MEDIUM"
            print_info "Estimated timeline: 9-11 years"
            print_info "Estimated cost: \$1,200-1,800M"
        fi
    else
        print_error "Candidate has CRITICAL ISSUES"
        print_error "Feasibility: LOW"
        print_info "Recommendation: Address issues or return to discovery"
    fi

    echo ""
}

# Simulate PK/PD
simulate_pkpd() {
    local dose=${1:-100}
    local clearance=${2:-0.2}
    local volume=${3:-5}
    local ec50=${4:-1}

    print_section "PK/PD Simulation"
    print_info "Dose: $dose mg"
    print_info "Clearance: $clearance L/h"
    print_info "Volume: $volume L"
    print_info "EC50: $ec50 mg/L"

    # Calculate PK
    local c0=$(echo "$dose / $volume" | bc -l)
    local ke=$(echo "$clearance / $volume" | bc -l)
    local half_life=$(echo "0.693 / $ke" | bc -l)

    print_section "Pharmacokinetics"
    print_success "Initial concentration: $(printf "%.2f" $c0) mg/L"
    print_success "Half-life: $(printf "%.2f" $half_life) hours"

    # Calculate PD at different time points
    print_section "Pharmacodynamics"

    for time in 0 6 12 24 48; do
        # C(t) = C0 × e^(-ke×t)
        local conc=$(echo "$c0 * e(-$ke * $time)" | bc -l)

        # Receptor occupancy = C / (EC50 + C)
        local occupancy=$(echo "$conc / ($ec50 + $conc)" | bc -l)
        local occupancy_pct=$(echo "$occupancy * 100" | bc -l)

        printf "  t=${time}h: C=%.2f mg/L, Occupancy=%.1f%%\n" $conc $occupancy_pct
    done

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-016 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-affinity            Calculate binding affinity (Kd)"
    echo "    --ab-conc <M>          Antibody concentration (default: 1e-9 M)"
    echo "    --ag-conc <M>          Antigen concentration (default: 1e-8 M)"
    echo "    --complex-conc <M>     Complex concentration (default: 0.9e-9 M)"
    echo ""
    echo "  calc-pk                  Calculate pharmacokinetic parameters"
    echo "    --dose <mg>            Dose in mg (default: 100)"
    echo "    --bioavail <0-1>       Bioavailability (default: 1.0)"
    echo "    --clearance <L/h>      Clearance (default: 0.2)"
    echo "    --volume <L>           Volume of distribution (default: 5)"
    echo ""
    echo "  immunogenicity           Assess immunogenicity risk"
    echo "    --ada-rate <0-1>       ADA incidence rate (default: 0.15)"
    echo "    --severity <1-10>      Clinical severity (default: 5)"
    echo "    --duration <factor>    Duration factor (default: 2)"
    echo "    --tolerance <factor>   Tolerance factor (default: 0.5)"
    echo ""
    echo "  validate                 Validate drug candidate"
    echo "    --type <type>          Drug type (default: monoclonal-antibody)"
    echo "    --affinity <M>         Target affinity Kd (default: 1e-9)"
    echo "    --stability <0-1>      Stability score (default: 0.95)"
    echo "    --immuno-risk <level>  Immunogenicity risk (low/moderate/high)"
    echo ""
    echo "  simulate-pk              Simulate PK parameters"
    echo "    --dose <mg>            Dose (default: 100)"
    echo "    --clearance <L/h>      Clearance (default: 0.2)"
    echo "    --volume <L>           Volume (default: 5)"
    echo ""
    echo "  simulate-pkpd            Simulate PK/PD over time"
    echo "    --dose <mg>            Dose (default: 100)"
    echo "    --clearance <L/h>      Clearance (default: 0.2)"
    echo "    --volume <L>           Volume (default: 5)"
    echo "    --ec50 <mg/L>          EC50 value (default: 1)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-016 calc-affinity --ab-conc 1e-9 --ag-conc 1e-8"
    echo "  wia-bio-016 calc-pk --dose 100 --clearance 0.2"
    echo "  wia-bio-016 immunogenicity --ada-rate 0.15 --severity 5"
    echo "  wia-bio-016 validate --type mab --affinity 1e-9 --stability 0.95"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-016 Biopharma CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    calc-affinity)
        AB_CONC=1e-9
        AG_CONC=1e-8
        COMPLEX_CONC=0.9e-9

        while [[ $# -gt 0 ]]; do
            case $1 in
                --ab-conc) AB_CONC=$2; shift 2 ;;
                --ag-conc) AG_CONC=$2; shift 2 ;;
                --complex-conc) COMPLEX_CONC=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_affinity "$AB_CONC" "$AG_CONC" "$COMPLEX_CONC"
        ;;

    calc-pk)
        DOSE=100
        BIOAVAIL=1.0
        CLEARANCE=0.2
        VOLUME=5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dose) DOSE=$2; shift 2 ;;
                --bioavail) BIOAVAIL=$2; shift 2 ;;
                --clearance) CLEARANCE=$2; shift 2 ;;
                --volume) VOLUME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_pk "$DOSE" "$BIOAVAIL" "$CLEARANCE" "$VOLUME"
        ;;

    immunogenicity)
        ADA_RATE=0.15
        SEVERITY=5
        DURATION=2
        TOLERANCE=0.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --ada-rate) ADA_RATE=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                --tolerance) TOLERANCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_immunogenicity "$ADA_RATE" "$SEVERITY" "$DURATION" "$TOLERANCE"
        ;;

    validate)
        DRUG_TYPE="monoclonal-antibody"
        AFFINITY=1e-9
        STABILITY=0.95
        IMMUNO_RISK="low"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) DRUG_TYPE=$2; shift 2 ;;
                --affinity) AFFINITY=$2; shift 2 ;;
                --stability) STABILITY=$2; shift 2 ;;
                --immuno-risk) IMMUNO_RISK=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_candidate "$DRUG_TYPE" "$AFFINITY" "$STABILITY" "$IMMUNO_RISK"
        ;;

    simulate-pk)
        DOSE=100
        CLEARANCE=0.2
        VOLUME=5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dose) DOSE=$2; shift 2 ;;
                --clearance) CLEARANCE=$2; shift 2 ;;
                --volume) VOLUME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_pk "$DOSE" 1.0 "$CLEARANCE" "$VOLUME"
        ;;

    simulate-pkpd)
        DOSE=100
        CLEARANCE=0.2
        VOLUME=5
        EC50=1

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dose) DOSE=$2; shift 2 ;;
                --clearance) CLEARANCE=$2; shift 2 ;;
                --volume) VOLUME=$2; shift 2 ;;
                --ec50) EC50=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate_pkpd "$DOSE" "$CLEARANCE" "$VOLUME" "$EC50"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-016 help' for usage information"
        exit 1
        ;;
esac

exit 0
