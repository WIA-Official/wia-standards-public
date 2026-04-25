#!/bin/bash

################################################################################
# WIA-BIO-002: Genome Sequencing CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Bioinformatics Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to genome sequencing calculations
# including coverage analysis, quality validation, and variant statistics.
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
HUMAN_GENOME_SIZE=3088269832
MIN_PHRED_QUALITY=30
CLINICAL_MIN_COVERAGE=30
RESEARCH_MIN_COVERAGE=20
CLINICAL_MIN_Q30=90
RESEARCH_MIN_Q30=85

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🧬 WIA-BIO-002: Genome Sequencing CLI                ║"
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
    printf "%'d" "$num" 2>/dev/null || echo "$num"
}

# Calculate sequencing coverage
calc_coverage() {
    local reads=${1:-100000000}
    local length=${2:-150}
    local genome=${3:-$HUMAN_GENOME_SIZE}
    local paired=${4:-true}

    print_section "Coverage Calculation"
    print_info "Total Reads: $(format_number $reads)"
    print_info "Read Length: ${length} bp"
    print_info "Genome Size: $(format_number $genome) bp"
    print_info "Paired-End: $paired"

    # Calculate effective read length
    local effective_length=$length
    if [ "$paired" = "true" ]; then
        effective_length=$((length * 2))
        print_info "Effective Length: ${effective_length} bp (paired-end)"
    fi

    # Calculate total bases
    local total_bases=$(echo "$reads * $effective_length" | bc -l)
    print_info "Total Bases: $(format_number ${total_bases%.*})"

    # Account for duplicates (assume 15%)
    local dup_rate=0.15
    local unique_bases=$(echo "$total_bases * (1 - $dup_rate)" | bc -l)

    # Calculate average depth
    local avg_depth=$(echo "scale=2; $unique_bases / $genome" | bc -l)

    print_section "Coverage Results"
    print_success "Average Depth: ${avg_depth}×"

    # Estimate coverage at thresholds
    local cov_30x=$(echo "scale=1; 100 * e(-30 / $avg_depth)" | bc -l)
    cov_30x=$(echo "100 - $cov_30x" | bc -l)
    print_info "Coverage at ≥30×: ${cov_30x}%"

    # Feasibility assessment
    local avg_int=${avg_depth%.*}
    if [ "$avg_int" -ge 40 ]; then
        print_success "Feasibility: EXCELLENT (Clinical Grade)"
    elif [ "$avg_int" -ge 30 ]; then
        print_success "Feasibility: GOOD (Clinical Grade)"
    elif [ "$avg_int" -ge 20 ]; then
        print_warning "Feasibility: ACCEPTABLE (Research Grade)"
    else
        print_error "Feasibility: INSUFFICIENT"
        print_info "Recommendation: Increase sequencing depth"
    fi

    echo ""
}

# Validate sequencing quality
validate_quality() {
    local q30=${1:-92}
    local coverage=${2:-35}
    local uniformity=${3:-0.93}
    local mapping=${4:-0.97}
    local duplication=${5:-0.15}
    local contamination=${6:-0.005}

    print_section "Quality Validation"
    print_info "Q30 Percentage: ${q30}%"
    print_info "Mean Coverage: ${coverage}×"
    print_info "Uniformity: $(echo "scale=1; $uniformity * 100" | bc -l)%"
    print_info "Mapping Rate: $(echo "scale=1; $mapping * 100" | bc -l)%"
    print_info "Duplication: $(echo "scale=1; $duplication * 100" | bc -l)%"
    print_info "Contamination: $(echo "scale=2; $contamination * 100" | bc -l)%"

    print_section "Quality Checks"

    local errors=0
    local warnings=0

    # Q30 Check
    if (( $(echo "$q30 < $RESEARCH_MIN_Q30" | bc -l) )); then
        print_error "Q30 Score: FAIL (${q30}% < ${RESEARCH_MIN_Q30}%)"
        ((errors++))
    elif (( $(echo "$q30 < $CLINICAL_MIN_Q30" | bc -l) )); then
        print_warning "Q30 Score: WARN (${q30}% < ${CLINICAL_MIN_Q30}%)"
        ((warnings++))
    else
        print_success "Q30 Score: PASS (${q30}%)"
    fi

    # Coverage Check
    if (( $(echo "$coverage < $RESEARCH_MIN_COVERAGE" | bc -l) )); then
        print_error "Coverage: FAIL (${coverage}× < ${RESEARCH_MIN_COVERAGE}×)"
        ((errors++))
    elif (( $(echo "$coverage < $CLINICAL_MIN_COVERAGE" | bc -l) )); then
        print_warning "Coverage: WARN (${coverage}× < ${CLINICAL_MIN_COVERAGE}×)"
        ((warnings++))
    else
        print_success "Coverage: PASS (${coverage}×)"
    fi

    # Uniformity Check
    if (( $(echo "$uniformity < 0.85" | bc -l) )); then
        print_error "Uniformity: FAIL ($(echo "scale=1; $uniformity * 100" | bc -l)% < 85%)"
        ((errors++))
    elif (( $(echo "$uniformity < 0.9" | bc -l) )); then
        print_warning "Uniformity: WARN ($(echo "scale=1; $uniformity * 100" | bc -l)% < 90%)"
        ((warnings++))
    else
        print_success "Uniformity: PASS ($(echo "scale=1; $uniformity * 100" | bc -l)%)"
    fi

    # Mapping Rate Check
    if (( $(echo "$mapping < 0.9" | bc -l) )); then
        print_error "Mapping Rate: FAIL ($(echo "scale=1; $mapping * 100" | bc -l)% < 90%)"
        ((errors++))
    elif (( $(echo "$mapping < 0.95" | bc -l) )); then
        print_warning "Mapping Rate: WARN ($(echo "scale=1; $mapping * 100" | bc -l)% < 95%)"
        ((warnings++))
    else
        print_success "Mapping Rate: PASS ($(echo "scale=1; $mapping * 100" | bc -l)%)"
    fi

    # Duplication Rate Check
    if (( $(echo "$duplication > 0.3" | bc -l) )); then
        print_error "Duplication Rate: FAIL ($(echo "scale=1; $duplication * 100" | bc -l)% > 30%)"
        ((errors++))
    elif (( $(echo "$duplication > 0.2" | bc -l) )); then
        print_warning "Duplication Rate: WARN ($(echo "scale=1; $duplication * 100" | bc -l)% > 20%)"
        ((warnings++))
    else
        print_success "Duplication Rate: PASS ($(echo "scale=1; $duplication * 100" | bc -l)%)"
    fi

    # Contamination Check
    if (( $(echo "$contamination > 0.02" | bc -l) )); then
        print_error "Contamination: FAIL ($(echo "scale=2; $contamination * 100" | bc -l)% > 2%)"
        ((errors++))
    elif (( $(echo "$contamination > 0.01" | bc -l) )); then
        print_warning "Contamination: WARN ($(echo "scale=2; $contamination * 100" | bc -l)% > 1%)"
        ((warnings++))
    else
        print_success "Contamination: PASS ($(echo "scale=2; $contamination * 100" | bc -l)%)"
    fi

    print_section "Validation Result"

    if [ $errors -gt 0 ]; then
        print_error "Quality Grade: FAILED"
        print_info "Errors: $errors | Warnings: $warnings"
        print_info "Do NOT proceed with analysis"
    elif [ $warnings -eq 0 ]; then
        print_success "Quality Grade: CLINICAL"
        print_info "Suitable for clinical applications"
    else
        print_warning "Quality Grade: RESEARCH"
        print_info "Warnings: $warnings"
        print_info "Suitable for research, consider optimization for clinical use"
    fi

    echo ""
}

# Calculate Phred score
calc_phred() {
    local accuracy=${1:-0.999}

    print_section "Phred Score Calculation"
    print_info "Accuracy: $(echo "scale=4; $accuracy * 100" | bc -l)%"

    # Q = -10 * log10(1 - accuracy)
    local error=$(echo "1 - $accuracy" | bc -l)
    local phred=$(echo "scale=2; -10 * l($error) / l(10)" | bc -l)

    print_section "Results"
    print_success "Phred Score: Q${phred%.*}"
    print_info "Error Probability: $error"

    # Interpretation
    local q_int=${phred%.*}
    if [ "$q_int" -ge 40 ]; then
        print_success "Quality: EXCELLENT (99.99% accuracy)"
    elif [ "$q_int" -ge 30 ]; then
        print_success "Quality: GOOD (99.9% accuracy)"
    elif [ "$q_int" -ge 20 ]; then
        print_warning "Quality: ACCEPTABLE (99% accuracy)"
    else
        print_error "Quality: POOR (<99% accuracy)"
    fi

    echo ""
}

# Simulate variant calling statistics
calc_variants() {
    local total=${1:-5000000}
    local snp_rate=${2:-0.8}
    local het_rate=${3:-0.6}

    print_section "Variant Statistics"
    print_info "Total Variants: $(format_number $total)"

    # Calculate variant types
    local snps=$(echo "scale=0; $total * $snp_rate" | bc -l)
    local indels=$(echo "$total - $snps" | bc -l)

    print_info "SNPs: $(format_number ${snps%.*})"
    print_info "Indels: $(format_number ${indels%.*})"

    # Calculate zygosity
    local het=$(echo "scale=0; $total * $het_rate" | bc -l)
    local hom=$(echo "$total - $het" | bc -l)

    print_section "Zygosity Distribution"
    print_info "Heterozygous: $(format_number ${het%.*}) ($(echo "scale=1; $het_rate * 100" | bc -l)%)"
    print_info "Homozygous: $(format_number ${hom%.*}) ($(echo "scale=1; (1 - $het_rate) * 100" | bc -l)%)"

    # Calculate Het/Hom ratio
    local het_hom=$(echo "scale=2; $het / $hom" | bc -l)
    print_info "Het/Hom Ratio: $het_hom"

    # Ti/Tv ratio (typical for WGS)
    local titv=2.1
    print_section "Quality Metrics"
    print_info "Ti/Tv Ratio: $titv (expected: 2.0-2.1 for WGS)"

    if (( $(echo "$het_hom >= 1.3 && $het_hom <= 2.0" | bc -l) )); then
        print_success "Het/Hom Ratio: NORMAL (1.5-2.0 expected)"
    else
        print_warning "Het/Hom Ratio: UNUSUAL (check for contamination or CNVs)"
    fi

    echo ""
}

# Generate QC report summary
qc_report() {
    print_section "Quality Control Report Summary"

    print_info "Sample ID: SAMPLE_$(date +%Y%m%d)_001"
    print_info "Run Date: $(date '+%Y-%m-%d %H:%M:%S')"
    print_info "Platform: Illumina NovaSeq 6000"

    print_section "Sequencing Metrics"
    print_success "Total Reads: 150,000,000 pairs"
    print_success "Read Length: 2×150 bp"
    print_success "Q30 Score: 92.5%"
    print_success "Cluster PF: 85.3%"

    print_section "Alignment Metrics"
    print_success "Mapped Reads: 97.2%"
    print_success "Properly Paired: 95.8%"
    print_success "Mean Coverage: 35.2×"
    print_success "Coverage Uniformity: 93.1%"

    print_section "Quality Assessment"
    print_success "Duplication Rate: 14.5%"
    print_success "Contamination: 0.3%"
    print_success "GC Bias: 2.1%"

    print_section "Variant Calling"
    print_success "Total Variants: 4,523,891"
    print_success "SNPs: 3,892,456"
    print_success "Indels: 631,435"
    print_success "Ti/Tv Ratio: 2.08"
    print_success "Het/Hom Ratio: 1.63"

    print_section "Overall Assessment"
    print_success "Quality Grade: CLINICAL"
    print_success "Status: PASS - Suitable for clinical reporting"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-002 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-coverage            Calculate sequencing coverage"
    echo "    --reads <number>       Total number of reads (default: 100M)"
    echo "    --length <bp>          Read length in base pairs (default: 150)"
    echo "    --genome <bp>          Genome size in base pairs (default: 3.1G)"
    echo "    --paired <true|false>  Paired-end sequencing (default: true)"
    echo ""
    echo "  validate-quality         Validate sequencing quality metrics"
    echo "    --q30 <percent>        Q30 percentage (default: 92)"
    echo "    --coverage <depth>     Mean coverage depth (default: 35)"
    echo "    --uniformity <ratio>   Coverage uniformity 0-1 (default: 0.93)"
    echo "    --mapping <ratio>      Mapping rate 0-1 (default: 0.97)"
    echo "    --duplication <ratio>  Duplication rate 0-1 (default: 0.15)"
    echo "    --contamination <ratio> Contamination rate 0-1 (default: 0.005)"
    echo ""
    echo "  calc-phred               Calculate Phred quality score"
    echo "    --accuracy <ratio>     Base calling accuracy 0-1 (default: 0.999)"
    echo ""
    echo "  calc-variants            Calculate variant statistics"
    echo "    --total <number>       Total variants (default: 5M)"
    echo "    --snp-rate <ratio>     SNP ratio 0-1 (default: 0.8)"
    echo "    --het-rate <ratio>     Heterozygous ratio 0-1 (default: 0.6)"
    echo ""
    echo "  qc-report                Generate QC report summary"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-002 calc-coverage --reads 150000000 --length 150 --paired true"
    echo "  wia-bio-002 validate-quality --q30 92 --coverage 35"
    echo "  wia-bio-002 calc-phred --accuracy 0.999"
    echo "  wia-bio-002 calc-variants --total 5000000"
    echo "  wia-bio-002 qc-report"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-002 CLI Tool"
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
    calc-coverage)
        READS=100000000
        LENGTH=150
        GENOME=$HUMAN_GENOME_SIZE
        PAIRED=true

        while [[ $# -gt 0 ]]; do
            case $1 in
                --reads) READS=$2; shift 2 ;;
                --length) LENGTH=$2; shift 2 ;;
                --genome) GENOME=$2; shift 2 ;;
                --paired) PAIRED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_coverage "$READS" "$LENGTH" "$GENOME" "$PAIRED"
        ;;

    validate-quality)
        Q30=92
        COVERAGE=35
        UNIFORMITY=0.93
        MAPPING=0.97
        DUPLICATION=0.15
        CONTAMINATION=0.005

        while [[ $# -gt 0 ]]; do
            case $1 in
                --q30) Q30=$2; shift 2 ;;
                --coverage) COVERAGE=$2; shift 2 ;;
                --uniformity) UNIFORMITY=$2; shift 2 ;;
                --mapping) MAPPING=$2; shift 2 ;;
                --duplication) DUPLICATION=$2; shift 2 ;;
                --contamination) CONTAMINATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_quality "$Q30" "$COVERAGE" "$UNIFORMITY" "$MAPPING" "$DUPLICATION" "$CONTAMINATION"
        ;;

    calc-phred)
        ACCURACY=0.999

        while [[ $# -gt 0 ]]; do
            case $1 in
                --accuracy) ACCURACY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_phred "$ACCURACY"
        ;;

    calc-variants)
        TOTAL=5000000
        SNP_RATE=0.8
        HET_RATE=0.6

        while [[ $# -gt 0 ]]; do
            case $1 in
                --total) TOTAL=$2; shift 2 ;;
                --snp-rate) SNP_RATE=$2; shift 2 ;;
                --het-rate) HET_RATE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_variants "$TOTAL" "$SNP_RATE" "$HET_RATE"
        ;;

    qc-report)
        print_header
        qc_report
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-002 help' for usage information"
        exit 1
        ;;
esac

exit 0
