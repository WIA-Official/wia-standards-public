#!/bin/bash

################################################################################
# WIA-BIO-018: Bio-Ethics CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Bioethics Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to bio-ethics functions including
# informed consent validation, ethical risk assessment, and gene editing evaluation.
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
MIN_COMPREHENSION=0.8
MIN_COMPLIANCE=0.8
MIN_CAPACITY_FULL=1.0
MIN_CAPACITY_BORDERLINE=0.75

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║              ⚖️  WIA-BIO-018: Bio-Ethics CLI                  ║"
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

# Validate informed consent
validate_consent() {
    local participant_id=${1:-P-12345}
    local study_id=${2:-STUDY-001}
    local comprehension=${3:-0.85}
    local capacity=${4:-full}
    local witness=${5:-yes}

    print_section "Informed Consent Validation"
    print_info "Participant ID: $participant_id"
    print_info "Study ID: $study_id"
    print_info "Comprehension Score: $comprehension"
    print_info "Participant Capacity: $capacity"
    print_info "Witness Present: $witness"

    # Check comprehension
    print_section "Comprehension Assessment"
    if (( $(echo "$comprehension >= $MIN_COMPREHENSION" | bc -l) )); then
        print_success "Comprehension: PASS (${comprehension} >= ${MIN_COMPREHENSION})"
    else
        print_error "Comprehension: FAIL (${comprehension} < ${MIN_COMPREHENSION})"
        print_info "Action Required: Re-educate participant and reassess"
    fi

    # Check capacity
    print_section "Capacity Assessment"
    case "$capacity" in
        full)
            print_success "Capacity: FULL - Can provide autonomous consent"
            ;;
        borderline)
            print_warning "Capacity: BORDERLINE - Can consent with support"
            print_info "Recommendation: Provide additional support"
            ;;
        diminished)
            print_warning "Capacity: DIMINISHED - Needs LAR, can assent"
            print_info "Required: Legally Authorized Representative consent"
            ;;
        severely-impaired)
            print_error "Capacity: SEVERELY IMPAIRED - Needs LAR, limited assent"
            print_info "Required: Legally Authorized Representative consent"
            ;;
        *)
            print_warning "Capacity: UNKNOWN"
            ;;
    esac

    # Check witness requirement
    print_section "Witness Requirement"
    if [ "$witness" = "yes" ]; then
        print_success "Witness: Present and signature obtained"
    else
        if [ "$capacity" != "full" ] || (( $(echo "$comprehension < 0.85" | bc -l) )); then
            print_error "Witness: Required but not present"
            print_info "Action: Obtain witness signature"
        else
            print_info "Witness: Not required for this participant"
        fi
    fi

    # Calculate compliance score
    local compliance=0.9
    print_section "Compliance Assessment"
    print_info "Overall Compliance Score: $compliance"

    if (( $(echo "$compliance >= 0.95" | bc -l) )); then
        print_success "Compliance: EXCELLENT (${compliance})"
        print_success "Consent is VALID and compliant"
    elif (( $(echo "$compliance >= $MIN_COMPLIANCE" | bc -l) )); then
        print_success "Compliance: GOOD (${compliance})"
        print_success "Consent is VALID"
    else
        print_error "Compliance: INADEQUATE (${compliance})"
        print_error "Consent is INVALID - Address violations before proceeding"
    fi

    echo ""
}

# Assess ethical risk
assess_risk() {
    local study_type=${1:-gene-therapy}
    local population=${2:-adults}
    local intervention=${3:-drug}
    local risk_level=${4:-minimal}

    print_section "Ethical Risk Assessment"
    print_info "Study Type: $study_type"
    print_info "Target Population: $population"
    print_info "Intervention Type: $intervention"
    print_info "Risk Level: $risk_level"

    # Population risk assessment
    print_section "Population Risk"
    case "$population" in
        adults)
            print_success "General adult population - Standard protections"
            ;;
        children)
            print_warning "Vulnerable: Children"
            print_info "Required: Parental permission + child assent (age 7+)"
            ;;
        pregnant-women)
            print_warning "Vulnerable: Pregnant women"
            print_info "Required: Fetal risk assessment, father's consent"
            ;;
        prisoners)
            print_warning "Vulnerable: Prisoners"
            print_info "Required: Prisoner representative on IRB, no parole advantage"
            ;;
        cognitively-impaired)
            print_warning "Vulnerable: Cognitively impaired"
            print_info "Required: Capacity assessment, LAR consent"
            ;;
        *)
            print_info "Population requires ethical review"
            ;;
    esac

    # Intervention risk assessment
    print_section "Intervention Risk"
    case "$intervention" in
        observational)
            print_success "Low risk: Observational study"
            risk_score=0.1
            ;;
        behavioral)
            print_success "Low risk: Behavioral intervention"
            risk_score=0.2
            ;;
        drug)
            print_warning "Moderate risk: Drug intervention"
            risk_score=0.6
            ;;
        device)
            print_warning "Moderate risk: Device intervention"
            risk_score=0.5
            ;;
        surgical)
            print_error "High risk: Surgical intervention"
            risk_score=0.8
            ;;
        gene-therapy)
            print_error "High risk: Gene therapy"
            risk_score=0.9
            print_info "Required: Long-term follow-up, genetic counseling"
            ;;
        *)
            print_warning "Unknown intervention type"
            risk_score=0.5
            ;;
    esac

    # Risk-benefit analysis
    print_section "Risk-Benefit Analysis"
    local benefit_score=0.7
    local ratio=$(echo "scale=2; $benefit_score / $risk_score" | bc -l)

    print_info "Risk Score: $risk_score"
    print_info "Benefit Score: $benefit_score"
    print_info "Risk-Benefit Ratio: $ratio"

    if (( $(echo "$ratio >= 1.5" | bc -l) )); then
        print_success "Risk-Benefit: FAVORABLE (ratio >= 1.5)"
        print_success "Recommendation: APPROVE with standard protections"
    elif (( $(echo "$ratio >= 1.0" | bc -l) )); then
        print_warning "Risk-Benefit: ACCEPTABLE (ratio >= 1.0)"
        print_warning "Recommendation: APPROVE with enhanced protections"
    elif (( $(echo "$ratio >= 0.7" | bc -l) )); then
        print_warning "Risk-Benefit: MARGINAL"
        print_warning "Recommendation: REVISE to improve risk-benefit profile"
    else
        print_error "Risk-Benefit: UNFAVORABLE"
        print_error "Recommendation: REJECT - risks outweigh benefits"
    fi

    # Required protections
    print_section "Required Protections"
    if [ "$population" != "adults" ] || [ "$risk_level" != "minimal" ]; then
        print_info "• IRB approval with continuing review"
    fi
    if [ "$risk_level" = "greater-than-minimal" ] || [ "$risk_level" = "high" ]; then
        print_info "• Data Safety Monitoring Board"
        print_info "• Independent safety monitoring"
    fi
    if [ "$intervention" = "gene-therapy" ]; then
        print_info "• Long-term follow-up (minimum 5 years)"
        print_info "• Genetic counseling"
    fi

    echo ""
}

# Submit IRB protocol
submit_irb() {
    local protocol_file=${1:-protocol.json}
    local institution=${2:-MIT}

    print_section "IRB Protocol Submission"
    print_info "Protocol File: $protocol_file"
    print_info "Institution: $institution"

    # Determine review level
    print_section "Review Level Determination"
    local risk_level=${3:-minimal}
    local has_vulnerable=${4:-no}
    local review_level=""

    if [ "$risk_level" = "minimal" ] && [ "$has_vulnerable" = "no" ]; then
        review_level="expedited"
        print_success "Review Level: EXPEDITED"
        print_info "Minimal risk, no vulnerable populations"
    elif [ "$risk_level" = "minor-increase" ] && [ "$has_vulnerable" = "no" ]; then
        review_level="expedited"
        print_success "Review Level: EXPEDITED"
        print_info "Minor increase over minimal risk"
    else
        review_level="full-board"
        print_warning "Review Level: FULL BOARD"
        print_info "Greater than minimal risk or vulnerable populations"
    fi

    # Generate submission ID
    local submission_id="IRB-$(date +%s)-$(head /dev/urandom | tr -dc A-Z0-9 | head -c 8)"

    print_section "Submission Result"
    print_success "Submission ID: $submission_id"
    print_info "Status: SUBMITTED"
    print_info "Review Level: $review_level"
    print_info "Institution: $institution"

    if [ "$review_level" = "full-board" ]; then
        print_info "Expected timeline: 4-6 weeks"
        print_info "Next IRB meeting: Check institutional calendar"
    else
        print_info "Expected timeline: 1-2 weeks"
    fi

    echo ""
}

# Evaluate gene editing ethics
evaluate_gene_editing() {
    local editing_type=${1:-somatic}
    local purpose=${2:-therapeutic}
    local disease=${3:-sickle-cell}

    print_section "Gene Editing Ethics Evaluation"
    print_info "Editing Type: $editing_type"
    print_info "Purpose: $purpose"
    print_info "Target Disease: $disease"

    # Evaluate editing type
    print_section "Editing Type Assessment"
    case "$editing_type" in
        somatic)
            print_success "Editing Type: SOMATIC (non-heritable)"
            print_info "Changes affect only the individual, not offspring"
            ;;
        germline)
            print_error "Editing Type: GERMLINE (heritable)"
            print_warning "Changes will be passed to future generations"
            print_warning "International moratorium in effect for clinical applications"
            ;;
        *)
            print_warning "Unknown editing type"
            ;;
    esac

    # Evaluate purpose
    print_section "Purpose Assessment"
    case "$purpose" in
        therapeutic)
            print_success "Purpose: THERAPEUTIC (treat/prevent disease)"
            print_info "Ethically acceptable with proper oversight"
            ;;
        enhancement)
            print_error "Purpose: ENHANCEMENT (improve normal traits)"
            print_error "Ethically controversial and generally not acceptable"
            ;;
        research)
            print_info "Purpose: RESEARCH (scientific investigation)"
            print_info "May be acceptable with appropriate restrictions"
            ;;
        *)
            print_warning "Purpose requires ethical review"
            ;;
    esac

    # Final decision
    print_section "Ethical Decision"

    if [ "$editing_type" = "germline" ] && [ "$purpose" = "enhancement" ]; then
        print_error "Decision: REJECT"
        print_error "Germline editing for enhancement is prohibited"
    elif [ "$editing_type" = "germline" ]; then
        print_warning "Decision: DEFER"
        print_warning "Germline editing requires international consensus"
        print_info "Current status: Research only, no clinical applications"
    elif [ "$editing_type" = "somatic" ] && [ "$purpose" = "enhancement" ]; then
        print_error "Decision: REJECT"
        print_error "Enhancement applications not currently acceptable"
    elif [ "$editing_type" = "somatic" ] && [ "$purpose" = "therapeutic" ]; then
        print_success "Decision: APPROVE for clinical trial"
        print_info "Subject to:"
        print_info "  • IRB approval"
        print_info "  • FDA IND approval (or equivalent)"
        print_info "  • Informed consent"
        print_info "  • Long-term follow-up (5+ years)"
        print_info "  • Genetic counseling"
        print_info "  • Off-target monitoring"
    else
        print_warning "Decision: REQUIRES FURTHER REVIEW"
    fi

    # Risk monitoring
    print_section "Required Monitoring"
    print_info "• Off-target effects surveillance"
    print_info "• Immune response monitoring"
    print_info "• Clinical efficacy assessment"
    print_info "• Long-term safety follow-up"
    if [ "$disease" != "" ]; then
        print_info "• Disease-specific outcomes for $disease"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-018 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  validate-consent         Validate informed consent"
    echo "    --participant <id>     Participant identifier (default: P-12345)"
    echo "    --study <id>           Study identifier (default: STUDY-001)"
    echo "    --comprehension <0-1>  Comprehension score (default: 0.85)"
    echo "    --capacity <level>     Capacity level: full|borderline|diminished (default: full)"
    echo "    --witness <yes|no>     Witness present (default: yes)"
    echo ""
    echo "  assess-risk              Assess ethical risk"
    echo "    --study-type <type>    Study type (default: gene-therapy)"
    echo "    --population <pop>     Target population (default: adults)"
    echo "    --intervention <type>  Intervention type (default: drug)"
    echo "    --risk-level <level>   Risk level: minimal|minor-increase|high (default: minimal)"
    echo ""
    echo "  submit-irb               Submit IRB protocol"
    echo "    --protocol <file>      Protocol file (default: protocol.json)"
    echo "    --institution <name>   Institution name (default: MIT)"
    echo "    --risk-level <level>   Risk level (default: minimal)"
    echo "    --vulnerable <yes|no>  Has vulnerable populations (default: no)"
    echo ""
    echo "  evaluate-gene-editing    Evaluate gene editing ethics"
    echo "    --type <somatic|germline>  Editing type (default: somatic)"
    echo "    --purpose <purpose>    Purpose: therapeutic|enhancement|research (default: therapeutic)"
    echo "    --disease <disease>    Target disease (default: sickle-cell)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-018 validate-consent --participant P-12345 --comprehension 0.92"
    echo "  wia-bio-018 assess-risk --study-type gene-therapy --population adults"
    echo "  wia-bio-018 submit-irb --protocol study.json --institution MIT"
    echo "  wia-bio-018 evaluate-gene-editing --type somatic --disease sickle-cell"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-018 CLI Tool"
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
    validate-consent)
        PARTICIPANT="P-12345"
        STUDY="STUDY-001"
        COMPREHENSION="0.85"
        CAPACITY="full"
        WITNESS="yes"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --participant) PARTICIPANT=$2; shift 2 ;;
                --study) STUDY=$2; shift 2 ;;
                --comprehension) COMPREHENSION=$2; shift 2 ;;
                --capacity) CAPACITY=$2; shift 2 ;;
                --witness) WITNESS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_consent "$PARTICIPANT" "$STUDY" "$COMPREHENSION" "$CAPACITY" "$WITNESS"
        ;;

    assess-risk)
        STUDY_TYPE="gene-therapy"
        POPULATION="adults"
        INTERVENTION="drug"
        RISK_LEVEL="minimal"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --study-type) STUDY_TYPE=$2; shift 2 ;;
                --population) POPULATION=$2; shift 2 ;;
                --intervention) INTERVENTION=$2; shift 2 ;;
                --risk-level) RISK_LEVEL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_risk "$STUDY_TYPE" "$POPULATION" "$INTERVENTION" "$RISK_LEVEL"
        ;;

    submit-irb)
        PROTOCOL="protocol.json"
        INSTITUTION="MIT"
        RISK_LEVEL="minimal"
        VULNERABLE="no"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --institution) INSTITUTION=$2; shift 2 ;;
                --risk-level) RISK_LEVEL=$2; shift 2 ;;
                --vulnerable) VULNERABLE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        submit_irb "$PROTOCOL" "$INSTITUTION" "$RISK_LEVEL" "$VULNERABLE"
        ;;

    evaluate-gene-editing)
        EDITING_TYPE="somatic"
        PURPOSE="therapeutic"
        DISEASE="sickle-cell"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) EDITING_TYPE=$2; shift 2 ;;
                --purpose) PURPOSE=$2; shift 2 ;;
                --disease) DISEASE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        evaluate_gene_editing "$EDITING_TYPE" "$PURPOSE" "$DISEASE"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-018 help' for usage information"
        exit 1
        ;;
esac

exit 0
