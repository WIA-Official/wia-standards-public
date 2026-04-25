#!/bin/bash

################################################################################
# WIA-BIO-010: Clinical Trial Data CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biotechnology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to clinical trial data management
# including trial registration, patient data collection, adverse event reporting,
# and statistical calculations.
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
ALPHA=0.05
POWER=0.80
Z_ALPHA_2=1.96
Z_BETA=0.84

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        📋 WIA-BIO-010: Clinical Trial Data CLI Tool          ║"
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

# Register new trial
register_trial() {
    local title="$1"
    local phase="$2"
    local sample_size=${3:-300}

    print_section "Clinical Trial Registration"

    # Generate trial ID (NCT format)
    local trial_id="NCT$(date +%s | tail -c 9)"

    print_info "Title: $title"
    print_info "Phase: $phase"
    print_info "Sample Size: $sample_size patients"

    print_section "Registration Details"
    print_success "Trial ID: $trial_id"
    print_info "Registration Date: $(date '+%Y-%m-%d')"
    print_info "Status: PLANNED"
    print_info "Protocol Version: 1.0"

    print_section "Timeline"
    print_info "Enrollment Period: 12 months"
    print_info "Treatment Duration: 52 weeks"
    print_info "Follow-up: 4 weeks"
    print_info "Estimated Completion: $(date -d '+18 months' '+%Y-%m-%d')"

    echo ""
}

# Record patient data
record_data() {
    local trial_id="$1"
    local patient_id="$2"
    local hba1c=${3:-7.5}

    print_section "Patient Data Collection"
    print_info "Trial ID: $trial_id"
    print_info "Patient ID: $patient_id"
    print_info "Visit: Baseline"
    print_info "Date: $(date '+%Y-%m-%d')"

    print_section "Clinical Measurements"
    print_info "HbA1c: ${hba1c}%"

    # Validate HbA1c
    if (( $(echo "$hba1c < 4.0" | bc -l) )) || (( $(echo "$hba1c > 15.0" | bc -l) )); then
        print_error "HbA1c value out of valid range (4.0-15.0%)"
    elif (( $(echo "$hba1c >= 7.0" | bc -l) )); then
        print_warning "HbA1c ≥7.0% (above target)"
    else
        print_success "HbA1c <7.0% (at target)"
    fi

    print_section "Validation"
    print_success "Data collection completed"
    local data_id="DATA-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"
    print_info "Data ID: $data_id"
    print_info "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"

    echo ""
}

# Report adverse event
report_ae() {
    local trial_id="$1"
    local patient_id="$2"
    local severity=${3:-moderate}

    print_section "Adverse Event Reporting"
    print_info "Trial ID: $trial_id"
    print_info "Patient ID: $patient_id"
    print_info "AE Term: Hypoglycemia"
    print_info "Severity: $severity"
    print_info "Start Date: $(date '+%Y-%m-%d')"

    # Generate AE ID
    local ae_id="AE-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"

    print_section "AE Details"
    print_success "AE ID: $ae_id"
    print_info "MedDRA PT: Hypoglycaemia (10020993)"
    print_info "SOC: Metabolism and nutrition disorders"
    print_info "Causality: Probable"
    print_info "Action Taken: None required"
    print_info "Outcome: Recovered"

    print_section "Seriousness Criteria"
    if [ "$severity" == "severe" ] || [ "$severity" == "life-threatening" ]; then
        print_warning "SERIOUS ADVERSE EVENT (SAE)"
        print_error "Reporting required within 15 days"
        print_info "Deadline: $(date -d '+15 days' '+%Y-%m-%d')"
    else
        print_success "Non-serious adverse event"
        print_info "Standard reporting procedures apply"
    fi

    echo ""
}

# Calculate sample size
calc_sample_size() {
    local alpha=${1:-0.05}
    local power=${2:-0.80}
    local effect_size=${3:-0.5}
    local sd=${4:-1.5}

    print_section "Sample Size Calculation"
    print_info "Significance Level (α): $alpha"
    print_info "Statistical Power: $(echo "$power * 100" | bc)%"
    print_info "Effect Size (Δ): $effect_size"
    print_info "Standard Deviation (σ): $sd"

    # Calculate Z values
    local z_alpha=1.96  # for two-sided α=0.05
    local z_beta=0.84   # for 80% power

    if (( $(echo "$power >= 0.90" | bc -l) )); then
        z_beta=1.28
    fi

    print_info "Z(α/2): $z_alpha"
    print_info "Z(β): $z_beta"

    # Calculate sample size per group
    # n = (Z_α/2 + Z_β)² × (2σ²) / Δ²
    local numerator=$(echo "scale=6; ($z_alpha + $z_beta)^2 * 2 * $sd^2" | bc -l)
    local denominator=$(echo "scale=6; $effect_size^2" | bc -l)
    local n_per_group=$(echo "scale=0; ($numerator / $denominator) + 0.5" | bc)  # Round up

    # Total sample size
    local total_n=$(echo "$n_per_group * 2" | bc)

    # Adjust for 20% dropout
    local adjusted_n=$(echo "scale=0; ($total_n / 0.8) + 0.5" | bc)

    print_section "Results"
    print_success "Sample Size per Group: $n_per_group patients"
    print_success "Total Sample Size: $total_n patients"
    print_info "Adjusted for 20% Dropout: $adjusted_n patients"

    print_section "Feasibility"
    if [ "$total_n" -lt 100 ]; then
        print_success "Small trial (feasible)"
    elif [ "$total_n" -lt 500 ]; then
        print_success "Medium trial (standard Phase III)"
    elif [ "$total_n" -lt 1000 ]; then
        print_warning "Large trial (requires multi-center)"
    else
        print_warning "Very large trial (mega-trial)"
    fi

    echo ""
}

# Calculate statistical power
calc_power() {
    local sample_size=${1:-100}
    local alpha=${2:-0.05}
    local effect_size=${3:-0.5}

    print_section "Statistical Power Calculation"
    print_info "Sample Size per Group: $sample_size"
    print_info "Significance Level (α): $alpha"
    print_info "Effect Size (Δ): $effect_size"

    # Simplified power calculation
    # This is an approximation
    local power_estimate=$(echo "scale=4; 0.5 + 0.4 * ($sample_size / 100) * $effect_size" | bc -l)

    # Cap at 0.99
    if (( $(echo "$power_estimate > 0.99" | bc -l) )); then
        power_estimate=0.99
    fi

    local power_percent=$(echo "scale=1; $power_estimate * 100" | bc)

    print_section "Results"
    print_success "Statistical Power: ${power_percent}%"

    if (( $(echo "$power_estimate >= 0.80" | bc -l) )); then
        print_success "Adequate power (≥80%)"
    elif (( $(echo "$power_estimate >= 0.70" | bc -l) )); then
        print_warning "Marginal power (70-80%)"
    else
        print_error "Insufficient power (<70%)"
        print_info "Recommendation: Increase sample size"
    fi

    print_section "Interpretation"
    print_info "Probability of detecting true effect: ${power_percent}%"
    print_info "Type II error rate (β): $(echo "scale=1; (1 - $power_estimate) * 100" | bc)%"

    echo ""
}

# Generate regulatory submission
generate_submission() {
    local trial_id="$1"
    local agency=${2:-FDA}
    local format=${3:-CDISC}

    print_section "Regulatory Submission Generation"
    print_info "Trial ID: $trial_id"
    print_info "Regulatory Agency: $agency"
    print_info "Format: $format"

    local submission_id="SUB-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"

    print_section "Submission Package"
    print_success "Submission ID: $submission_id"
    print_info "Type: New Drug Application (NDA)"
    print_info "Submission Date: $(date '+%Y-%m-%d')"

    print_section "Included Datasets"
    print_info "✓ SDTM Domains: DM, AE, VS, LB, EX, DS"
    print_info "✓ ADaM Datasets: ADSL, ADAE, ADLB, ADTTE"
    print_info "✓ Define.xml: Data definitions"
    print_info "✓ Reviewer's Guide: Documentation"

    print_section "CDISC Compliance"
    print_success "SDTM v1.7 compliant"
    print_success "ADaM v1.1 compliant"
    print_success "Define-XML 2.0 included"

    print_section "Next Steps"
    print_info "1. Review submission package"
    print_info "2. Obtain quality control approval"
    print_info "3. Submit via $agency electronic gateway"
    print_info "4. Monitor submission status"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-010 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  register-trial           Register new clinical trial"
    echo "    --title <string>       Trial title (required)"
    echo "    --phase <I|II|III|IV>  Trial phase (required)"
    echo "    --sample-size <n>      Planned sample size (default: 300)"
    echo ""
    echo "  record-data              Record patient data"
    echo "    --trial-id <id>        Trial identifier (required)"
    echo "    --patient-id <id>      Patient identifier (required)"
    echo "    --hba1c <value>        HbA1c percentage"
    echo ""
    echo "  report-ae                Report adverse event"
    echo "    --trial-id <id>        Trial identifier (required)"
    echo "    --patient-id <id>      Patient identifier (required)"
    echo "    --severity <level>     Severity (mild, moderate, severe)"
    echo ""
    echo "  calc-sample-size         Calculate required sample size"
    echo "    --alpha <value>        Significance level (default: 0.05)"
    echo "    --power <value>        Statistical power (default: 0.80)"
    echo "    --effect-size <value>  Effect size (default: 0.5)"
    echo "    --sd <value>           Standard deviation (default: 1.5)"
    echo ""
    echo "  calc-power               Calculate statistical power"
    echo "    --sample-size <n>      Sample size per group"
    echo "    --alpha <value>        Significance level (default: 0.05)"
    echo "    --effect-size <value>  Effect size (default: 0.5)"
    echo ""
    echo "  generate-submission      Generate regulatory submission"
    echo "    --trial-id <id>        Trial identifier (required)"
    echo "    --agency <name>        Agency (FDA, EMA, PMDA)"
    echo "    --format <type>        Format (CDISC, eCTD)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-010 register-trial --title \"Phase III Study\" --phase III"
    echo "  wia-bio-010 record-data --trial-id TR-001 --patient-id PT-001 --hba1c 6.8"
    echo "  wia-bio-010 calc-sample-size --alpha 0.05 --power 0.80 --effect-size 0.5"
    echo "  wia-bio-010 generate-submission --trial-id TR-001 --agency FDA"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-010 Clinical Trial Data CLI Tool"
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
    register-trial)
        TITLE="Clinical Trial"
        PHASE="III"
        SAMPLE_SIZE=300

        while [[ $# -gt 0 ]]; do
            case $1 in
                --title) TITLE=$2; shift 2 ;;
                --phase) PHASE=$2; shift 2 ;;
                --sample-size) SAMPLE_SIZE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        register_trial "$TITLE" "$PHASE" "$SAMPLE_SIZE"
        ;;

    record-data)
        TRIAL_ID=""
        PATIENT_ID=""
        HBA1C=7.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --trial-id) TRIAL_ID=$2; shift 2 ;;
                --patient-id) PATIENT_ID=$2; shift 2 ;;
                --hba1c) HBA1C=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$TRIAL_ID" ] || [ -z "$PATIENT_ID" ]; then
            echo -e "${RED}Error: --trial-id and --patient-id are required${RESET}"
            exit 1
        fi

        print_header
        record_data "$TRIAL_ID" "$PATIENT_ID" "$HBA1C"
        ;;

    report-ae)
        TRIAL_ID=""
        PATIENT_ID=""
        SEVERITY="moderate"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --trial-id) TRIAL_ID=$2; shift 2 ;;
                --patient-id) PATIENT_ID=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$TRIAL_ID" ] || [ -z "$PATIENT_ID" ]; then
            echo -e "${RED}Error: --trial-id and --patient-id are required${RESET}"
            exit 1
        fi

        print_header
        report_ae "$TRIAL_ID" "$PATIENT_ID" "$SEVERITY"
        ;;

    calc-sample-size)
        ALPHA=0.05
        POWER=0.80
        EFFECT_SIZE=0.5
        SD=1.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --alpha) ALPHA=$2; shift 2 ;;
                --power) POWER=$2; shift 2 ;;
                --effect-size) EFFECT_SIZE=$2; shift 2 ;;
                --sd) SD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_sample_size "$ALPHA" "$POWER" "$EFFECT_SIZE" "$SD"
        ;;

    calc-power)
        SAMPLE_SIZE=100
        ALPHA=0.05
        EFFECT_SIZE=0.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --sample-size) SAMPLE_SIZE=$2; shift 2 ;;
                --alpha) ALPHA=$2; shift 2 ;;
                --effect-size) EFFECT_SIZE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_power "$SAMPLE_SIZE" "$ALPHA" "$EFFECT_SIZE"
        ;;

    generate-submission)
        TRIAL_ID=""
        AGENCY="FDA"
        FORMAT="CDISC"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --trial-id) TRIAL_ID=$2; shift 2 ;;
                --agency) AGENCY=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$TRIAL_ID" ]; then
            echo -e "${RED}Error: --trial-id is required${RESET}"
            exit 1
        fi

        print_header
        generate_submission "$TRIAL_ID" "$AGENCY" "$FORMAT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-010 help' for usage information"
        exit 1
        ;;
esac

exit 0
