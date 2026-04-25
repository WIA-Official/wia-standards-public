#!/bin/bash

################################################################################
# WIA-BIO-017: Bio-Safety CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biosafety Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to biosafety calculations including
# risk assessment, BSL validation, PPE requirements, and exposure assessment.
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
MIN_LOG_REDUCTION=6
BLEACH_CONCENTRATION=5000
SPILL_CONTACT_TIME=20

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║              ☣️  WIA-BIO-017: Bio-Safety CLI                  ║"
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

# Calculate risk score
calc_risk() {
    local hazard=${1:-2}
    local probability=${2:-0.1}
    local impact=${3:-5}
    local containment=${4:-0.7}

    print_section "Risk Assessment"
    print_info "Hazard Level: $hazard (Risk Group)"
    print_info "Exposure Probability: $probability"
    print_info "Impact Extent: $impact"
    print_info "Containment Effectiveness: $containment"

    # Calculate risk score: R = (H × P × E) / C
    local risk_score=$(echo "scale=2; ($hazard * $probability * $impact) / $containment" | bc -l)

    print_section "Results"
    print_success "Risk Score: $risk_score"

    # Determine risk level
    if (( $(echo "$risk_score < 10" | bc -l) )); then
        print_success "Risk Level: LOW"
        print_info "Recommended BSL: BSL-1"
    elif (( $(echo "$risk_score < 30" | bc -l) )); then
        print_warning "Risk Level: MEDIUM"
        print_info "Recommended BSL: BSL-2"
    elif (( $(echo "$risk_score < 60" | bc -l) )); then
        print_error "Risk Level: HIGH"
        print_info "Recommended BSL: BSL-3"
    else
        print_error "Risk Level: EXTREME"
        print_info "Recommended BSL: BSL-4"
    fi

    print_section "Recommendations"
    if (( $(echo "$risk_score >= 60" | bc -l) )); then
        print_error "CRITICAL: Immediate review of safety procedures required"
        print_error "Consider upgrading to BSL-4 or abandoning procedure"
    elif (( $(echo "$risk_score >= 30" | bc -l) )); then
        print_warning "Enhanced safety measures required"
        print_warning "Upgrade to BSL-3 or implement additional controls"
    elif (( $(echo "$risk_score >= 10" | bc -l) )); then
        print_info "Standard BSL-2 practices should be followed"
        print_info "Ensure proper PPE and biosafety cabinet use"
    else
        print_success "Standard microbiological practices sufficient"
    fi

    echo ""
}

# Validate BSL level
validate_bsl() {
    local pathogen="$1"
    local facility="$2"
    local procedure="$3"

    print_section "BSL Compliance Validation"
    print_info "Pathogen Class: $pathogen"
    print_info "Facility Level: $facility"
    print_info "Procedure Type: $procedure"

    local violations=0
    local warnings=0

    print_section "Compliance Checks"

    # BSL level mapping
    declare -A bsl_levels=( ["BSL-1"]=1 ["BSL-2"]=2 ["BSL-3"]=3 ["BSL-4"]=4 )

    # Check facility level
    if [ "${bsl_levels[$facility]}" -lt "${bsl_levels[$pathogen]}" ]; then
        print_error "Facility level insufficient for pathogen class"
        ((violations++))
    else
        print_success "Facility level adequate"
    fi

    # BSL-specific checks
    case "$pathogen" in
        BSL-4)
            print_info "Checking BSL-4 requirements..."
            print_warning "⚠ Positive pressure suit or Class III BSC required"
            print_warning "⚠ Isolated zone required"
            print_warning "⚠ Double-door airlock required"
            print_warning "⚠ Effluent decontamination system required"
            ((warnings+=4))
            ;;
        BSL-3)
            print_info "Checking BSL-3 requirements..."
            print_success "Respiratory protection (N95/PAPR) required"
            print_success "Solid-front gown recommended"
            print_success "BSC required for all procedures"
            print_success "Directional inward airflow required"
            print_warning "⚠ Autoclave within laboratory recommended"
            ((warnings++))
            ;;
        BSL-2)
            print_info "Checking BSL-2 requirements..."
            print_success "Gloves required"
            print_success "Lab coat or gown required"
            if [[ "$procedure" == *"aerosol"* ]]; then
                print_warning "⚠ BSC required for aerosol-generating procedures"
                ((warnings++))
            fi
            print_success "Autoclave availability required"
            ;;
        BSL-1)
            print_info "Checking BSL-1 requirements..."
            print_success "Standard microbiological practices"
            print_success "Lab coat recommended"
            ;;
    esac

    print_section "Validation Result"

    if [ $violations -eq 0 ]; then
        print_success "Setup is COMPLIANT"
        local score=$((100 - warnings * 5))
        print_info "Compliance Score: $score/100"
    else
        print_error "Setup is NON-COMPLIANT ($violations violations)"
        print_error "Upgrade facility or modify procedure before proceeding"
    fi

    echo ""
}

# Generate PPE requirements
generate_ppe() {
    local bsl=${1:-2}
    local procedure="$2"

    print_section "PPE Requirements Generator"
    print_info "BSL Level: BSL-$bsl"
    print_info "Procedure: $procedure"

    print_section "Required PPE"

    case $bsl in
        4)
            print_success "Gloves: Double-layer nitrile with extended cuff"
            print_success "Respiratory: Supplied air (positive pressure suit)"
            print_success "Eye Protection: Full face shield (integrated)"
            print_success "Body Protection: Positive pressure suit"
            print_success "Footwear: Dedicated boots"
            print_info "Additional: Chemical shower for decontamination"
            ;;
        3)
            print_success "Gloves: Double-layer nitrile"
            print_success "Respiratory: Fit-tested N95 or PAPR"
            print_success "Eye Protection: Face shield with safety glasses"
            print_success "Body Protection: Solid-front gown"
            print_success "Footwear: Dedicated laboratory shoes"
            print_warning "⚠ Annual respiratory fit testing required"
            ;;
        2)
            print_success "Gloves: Nitrile (single or double layer)"
            print_success "Respiratory: None (N95 if aerosols)"
            print_success "Eye Protection: Safety glasses or goggles"
            print_success "Body Protection: Lab coat or gown"
            print_success "Footwear: Closed-toe shoes"
            if [[ "$procedure" == *"aerosol"* ]]; then
                print_warning "⚠ N95 required for aerosol-generating procedures"
            fi
            ;;
        1)
            print_success "Gloves: Optional (latex or nitrile)"
            print_success "Respiratory: None"
            print_success "Eye Protection: Safety glasses if splash risk"
            print_success "Body Protection: Lab coat recommended"
            print_success "Footwear: Closed-toe shoes"
            ;;
    esac

    print_section "Donning Sequence"
    print_info "1. Wash hands"
    print_info "2. Put on first pair of gloves"
    print_info "3. Put on gown/suit"
    if [ $bsl -ge 3 ]; then
        print_info "4. Put on second pair of gloves"
    fi
    print_info "5. Put on respiratory protection"
    print_info "6. Put on eye protection"
    print_info "7. Verify all PPE properly fitted"

    echo ""
}

# Assess exposure
assess_exposure() {
    local dose=${1:-1000000}
    local frequency=${2:-1}
    local duration=${3:-30}
    local safety_factor=${4:-0.1}

    print_section "Exposure Assessment"
    print_info "Dose Concentration: $dose CFU/ml"
    print_info "Frequency: $frequency events"
    print_info "Duration: $duration minutes"
    print_info "Safety Factor: $safety_factor"

    # Calculate exposure: EA = (D × F × T) × SF
    local exposure=$(echo "scale=2; ($dose * $frequency * $duration) * $safety_factor" | bc -l)

    print_section "Results"
    print_success "Exposure Assessment Value: $exposure"

    # Estimate infection risk (simplified)
    local risk=$(echo "scale=4; $exposure / 1000000" | bc -l)
    print_info "Estimated Infection Risk: $risk"

    print_section "Risk Classification"
    if (( $(echo "$risk > 0.5" | bc -l) )); then
        print_error "HIGH RISK: Immediate medical evaluation required"
        print_error "Consider post-exposure prophylaxis"
        print_error "Implement strict quarantine protocols"
    elif (( $(echo "$risk > 0.1" | bc -l) )); then
        print_warning "MODERATE RISK: Medical evaluation recommended"
        print_warning "Monitor for symptoms"
        print_warning "Follow-up testing as appropriate"
    elif (( $(echo "$risk > 0.01" | bc -l) )); then
        print_warning "LOW RISK: Document exposure and monitor"
        print_info "Be alert for symptoms"
    else
        print_success "MINIMAL RISK: Standard precautions sufficient"
        print_info "Document incident for records"
    fi

    print_section "Recommendations"
    print_info "1. Decontaminate affected area immediately"
    print_info "2. Notify supervisor and biosafety officer"
    print_info "3. Complete incident report"
    print_info "4. Seek medical evaluation if indicated"
    print_info "5. Monitor for symptoms (14-21 days)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-017 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-risk                Calculate biosafety risk score"
    echo "    --hazard <1-4>         Hazard level / Risk Group (default: 2)"
    echo "    --probability <0-1>    Exposure probability (default: 0.1)"
    echo "    --impact <1-10>        Impact extent (default: 5)"
    echo "    --containment <0.1-1>  Containment effectiveness (default: 0.7)"
    echo ""
    echo "  validate-bsl             Validate BSL compliance"
    echo "    --pathogen <BSL-X>     Required BSL for pathogen (e.g., BSL-3)"
    echo "    --facility <BSL-X>     Actual facility BSL level"
    echo "    --procedure <type>     Procedure type (e.g., 'viral culture')"
    echo ""
    echo "  generate-ppe             Generate PPE requirements"
    echo "    --bsl <1-4>            Biosafety level (default: 2)"
    echo "    --procedure <type>     Procedure description"
    echo ""
    echo "  assess-exposure          Assess exposure incident"
    echo "    --dose <number>        Dose concentration (CFU/ml, default: 1e6)"
    echo "    --frequency <number>   Frequency of events (default: 1)"
    echo "    --duration <minutes>   Exposure duration (default: 30)"
    echo "    --safety <0-1>         Safety factor (default: 0.1)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-017 calc-risk --hazard 3 --probability 0.15 --impact 8"
    echo "  wia-bio-017 validate-bsl --pathogen BSL-3 --facility BSL-3 --procedure 'aerosol'"
    echo "  wia-bio-017 generate-ppe --bsl 3 --procedure 'viral culture'"
    echo "  wia-bio-017 assess-exposure --dose 1000000 --frequency 1 --duration 30"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-017 Bio-Safety CLI Tool"
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
    calc-risk)
        HAZARD=2
        PROBABILITY=0.1
        IMPACT=5
        CONTAINMENT=0.7

        while [[ $# -gt 0 ]]; do
            case $1 in
                --hazard) HAZARD=$2; shift 2 ;;
                --probability) PROBABILITY=$2; shift 2 ;;
                --impact) IMPACT=$2; shift 2 ;;
                --containment) CONTAINMENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_risk "$HAZARD" "$PROBABILITY" "$IMPACT" "$CONTAINMENT"
        ;;

    validate-bsl)
        PATHOGEN="BSL-2"
        FACILITY="BSL-2"
        PROCEDURE="standard"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --pathogen) PATHOGEN=$2; shift 2 ;;
                --facility) FACILITY=$2; shift 2 ;;
                --procedure) PROCEDURE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_bsl "$PATHOGEN" "$FACILITY" "$PROCEDURE"
        ;;

    generate-ppe)
        BSL=2
        PROCEDURE="standard"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --bsl) BSL=$2; shift 2 ;;
                --procedure) PROCEDURE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_ppe "$BSL" "$PROCEDURE"
        ;;

    assess-exposure)
        DOSE=1000000
        FREQUENCY=1
        DURATION=30
        SAFETY=0.1

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dose) DOSE=$2; shift 2 ;;
                --frequency) FREQUENCY=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                --safety) SAFETY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_exposure "$DOSE" "$FREQUENCY" "$DURATION" "$SAFETY"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-017 help' for usage information"
        exit 1
        ;;
esac

exit 0
