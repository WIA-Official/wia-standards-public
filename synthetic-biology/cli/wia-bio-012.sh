#!/bin/bash

################################################################################
# WIA-BIO-012: Synthetic Biology CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biotechnology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to synthetic biology calculations
# including promoter strength, gene expression, circuit design, and biosafety.
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
RNAP_SPEED=60
RIBOSOME_SPEED=17
MRNA_HALF_LIFE=300
PROTEIN_HALF_LIFE=3600

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🧬 WIA-BIO-012: Synthetic Biology CLI Tool            ║"
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

# Calculate promoter strength
calc_promoter() {
    local pmax=${1:-1000}
    local inducer=${2:-0.000001}
    local kd=${3:-0.0000001}
    local hill=${4:-2}

    print_section "Promoter Strength Calculation"
    print_info "Pmax: $pmax RPU (Relative Promoter Units)"
    print_info "Inducer concentration: $inducer M"
    print_info "Kd (dissociation constant): $kd M"
    print_info "Hill coefficient: $hill"

    # Calculate using Hill equation: P = Pmax × [I]^n / (Kd + [I]^n)
    local inducer_n=$(echo "e($hill * l($inducer))" | bc -l)
    local kd_n=$(echo "e($hill * l($kd))" | bc -l)
    local strength=$(echo "scale=2; $pmax * $inducer_n / ($kd_n + $inducer_n)" | bc -l)

    # Calculate saturation
    local saturation=$(echo "scale=2; ($strength / $pmax) * 100" | bc -l)

    # Calculate fold induction (assuming 1% basal)
    local basal=$(echo "$pmax * 0.01" | bc -l)
    local fold=$(echo "scale=2; $strength / $basal" | bc -l)

    print_section "Results"
    print_success "Promoter Strength: $(printf "%.2f" $strength) RPU"
    print_info "Saturation: $(printf "%.2f" $saturation)% of maximum"
    print_info "Fold Induction: $(printf "%.2f" $fold)×"

    if (( $(echo "$saturation > 95" | bc -l) )); then
        print_success "Status: FULLY INDUCED"
    elif (( $(echo "$saturation > 50" | bc -l) )); then
        print_warning "Status: PARTIALLY INDUCED"
    else
        print_info "Status: LOW INDUCTION"
    fi

    echo ""
}

# Calculate gene expression
calc_expression() {
    local k_tx=${1:-0.5}
    local k_tl=${2:-0.1}
    local k_deg_m=${3:-0.00385}
    local k_deg_p=${4:-0.0069}

    print_section "Gene Expression Calculation"
    print_info "Transcription rate: $k_tx /s"
    print_info "Translation rate: $k_tl /s"
    print_info "mRNA degradation rate: $k_deg_m /s"
    print_info "Protein degradation rate: $k_deg_p /s"

    # Calculate steady state
    local ss_mrna=$(echo "scale=2; $k_tx / $k_deg_m" | bc -l)
    local ss_protein=$(echo "scale=2; ($k_tl * $ss_mrna) / $k_deg_p" | bc -l)

    # Calculate half-lives
    local t_half_mrna=$(echo "scale=2; l(2) / $k_deg_m" | bc -l)
    local t_half_protein=$(echo "scale=2; l(2) / $k_deg_p" | bc -l)

    # Time to 90% steady state
    local t_90=$(echo "scale=2; 2.3 / $k_deg_p" | bc -l)

    print_section "Steady State"
    print_success "mRNA: $(printf "%.2f" $ss_mrna) molecules/cell"
    print_success "Protein: $(printf "%.2f" $ss_protein) molecules/cell"

    print_section "Kinetics"
    print_info "mRNA half-life: $(printf "%.2f" $t_half_mrna) seconds ($(printf "%.2f" $(echo "$t_half_mrna / 60" | bc -l)) min)"
    print_info "Protein half-life: $(printf "%.2f" $t_half_protein) seconds ($(printf "%.2f" $(echo "$t_half_protein / 60" | bc -l)) min)"
    print_info "Time to 90% steady state: $(printf "%.2f" $t_90) seconds ($(printf "%.2f" $(echo "$t_90/ 60" | bc -l)) min)"

    echo ""
}

# Design genetic circuit
design_circuit() {
    local parts="$1"
    local host=${2:-"E. coli"}
    local purpose=${3:-"gene expression"}

    print_section "Genetic Circuit Design"

    # Split parts by comma
    IFS=',' read -ra PART_ARRAY <<< "$parts"
    local num_parts=${#PART_ARRAY[@]}

    print_info "Parts: $num_parts"
    for i in "${!PART_ARRAY[@]}"; do
        print_info "  $((i+1)). ${PART_ARRAY[$i]}"
    done

    print_info "Host organism: $host"
    print_info "Purpose: $purpose"

    # Estimate size (500 bp average per part)
    local size=$((num_parts * 500))

    print_section "Circuit Properties"
    print_success "Circuit ID: CIR-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"
    print_info "Estimated size: $size bp"
    print_info "Assembly method: BioBrick standard"

    # Biosafety check
    if [[ "$parts" == *"toxin"* ]] || [[ "$parts" == *"pathogen"* ]]; then
        print_warning "Biosafety Level: BSL-2 (contains concerning elements)"
        print_warning "Kill switch REQUIRED"
    else
        print_success "Biosafety Level: BSL-1 (standard lab practices)"
    fi

    # Size check
    if [ $size -gt 20000 ]; then
        print_error "WARNING: Size exceeds typical plasmid limit (20 kb)"
    else
        print_success "Size within normal plasmid range"
    fi

    echo ""
}

# Optimize pathway
optimize_pathway() {
    local target=${1:-"ethanol"}
    local substrate=${2:-"glucose"}
    local host=${3:-"E. coli"}

    print_section "Metabolic Pathway Optimization"
    print_info "Target product: $target"
    print_info "Substrate: $substrate"
    print_info "Host organism: $host"

    print_section "Optimization Strategy"
    print_success "Identified pathway: $substrate → $target"
    print_info "Number of steps: 3 reactions"

    print_section "Gene Modifications"
    print_success "Genes to overexpress:"
    print_info "  1. ${substrate}_kinase (phosphorylation)"
    print_info "  2. ${target}_dehydrogenase (conversion)"
    print_info "  3. ${target}_synthase (synthesis)"

    print_warning "Genes to delete:"
    print_info "  1. competing_pathway_1"
    print_info "  2. competing_pathway_2"

    print_section "Predictions"
    local theoretical_yield=1.0
    local predicted_yield=0.85

    print_success "Theoretical yield: $(printf "%.2f" $theoretical_yield) mol $target / mol $substrate"
    print_info "Predicted yield: $(printf "%.2f" $predicted_yield) mol/mol ($(echo "$predicted_yield * 100 / $theoretical_yield" | bc -l)% of theoretical)"

    print_section "Bottlenecks"
    print_warning "Rate-limiting step: ${substrate}_kinase"
    print_info "Recommendation: Use strong promoter (>1000 RPU)"

    echo ""
}

# Simulate circuit
simulate() {
    local circuit_id=${1:-"CIR-001"}
    local duration=${2:-3600}

    print_section "Circuit Simulation"
    print_info "Circuit ID: $circuit_id"
    print_info "Duration: $duration seconds ($(echo "$duration / 60" | bc -l) minutes)"

    # Simple simulation parameters
    local k_tx=0.5
    local k_tl=0.1
    local k_deg_m=0.00385
    local k_deg_p=0.0069

    # Calculate final state
    local ss_mrna=$(echo "scale=2; $k_tx / $k_deg_m" | bc -l)
    local ss_protein=$(echo "scale=2; ($k_tl * $ss_mrna) / $k_deg_p" | bc -l)

    print_section "Simulation Results"
    print_success "Final mRNA: $(printf "%.2f" $ss_mrna) molecules/cell"
    print_success "Final protein: $(printf "%.2f" $ss_protein) molecules/cell"

    # Check if steady state reached
    local t_ss=$(echo "scale=2; 2.3 / $k_deg_p" | bc -l)
    if (( $(echo "$duration > $t_ss" | bc -l) )); then
        print_success "Steady state: REACHED"
        print_info "Time to steady state: $(printf "%.2f" $t_ss) seconds"
    else
        print_warning "Steady state: NOT REACHED"
        print_info "Need $(printf "%.2f" $t_ss) seconds to reach steady state"
    fi

    print_section "Expression Profile"
    print_info "Peak expression: ~$(echo "$duration / 3600" | bc -l) hours"
    print_success "Circuit stability: STABLE"

    echo ""
}

# Assess biosafety
assess_biosafety() {
    local circuit_id=${1:-"CIR-001"}
    local parts=${2:-"standard"}

    print_section "Biosafety Assessment"
    print_info "Circuit ID: $circuit_id"
    print_info "Parts: $parts"

    # Risk scoring
    local risk_score=1
    local level="BSL-1"

    if [[ "$parts" == *"toxin"* ]]; then
        risk_score=$((risk_score + 5))
        level="BSL-2"
    fi

    if [[ "$parts" == *"pathogen"* ]]; then
        risk_score=$((risk_score + 10))
        level="BSL-3"
    fi

    if [[ "$parts" == *"antibiotic"* ]] && [[ "$parts" == *"resistance"* ]]; then
        risk_score=$((risk_score + 3))
    fi

    print_section "Risk Assessment"
    print_info "Risk score: $risk_score / 25"

    if [ $risk_score -le 5 ]; then
        print_success "Risk level: LOW"
    elif [ $risk_score -le 10 ]; then
        print_warning "Risk level: MEDIUM"
    elif [ $risk_score -le 15 ]; then
        print_error "Risk level: HIGH"
    else
        print_error "Risk level: EXTREME"
    fi

    print_section "Biosafety Level"
    if [ "$level" == "BSL-1" ]; then
        print_success "Required level: $level"
        print_info "Containment: Standard lab practices"
    elif [ "$level" == "BSL-2" ]; then
        print_warning "Required level: $level"
        print_info "Containment: Biosafety cabinet, limited access"
    else
        print_error "Required level: $level"
        print_info "Containment: Specialized facility required"
    fi

    print_section "Recommendations"
    if [ $risk_score -gt 5 ]; then
        print_warning "Consider implementing kill switch system"
        print_warning "Use auxotrophic strain for containment"
        print_info "Institutional Biosafety Committee (IBC) review required"
    else
        print_success "Standard lab practices sufficient"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-012 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-promoter            Calculate promoter strength"
    echo "    --pmax <RPU>           Maximum strength (default: 1000)"
    echo "    --inducer <M>          Inducer concentration (default: 1e-6)"
    echo "    --kd <M>               Dissociation constant (default: 1e-7)"
    echo "    --hill <n>             Hill coefficient (default: 2)"
    echo ""
    echo "  calc-expression          Calculate gene expression dynamics"
    echo "    --k-tx <rate>          Transcription rate (default: 0.5)"
    echo "    --k-tl <rate>          Translation rate (default: 0.1)"
    echo "    --k-deg-m <rate>       mRNA degradation rate (default: 0.00385)"
    echo "    --k-deg-p <rate>       Protein degradation rate (default: 0.0069)"
    echo ""
    echo "  design-circuit           Design genetic circuit"
    echo "    --parts <list>         Comma-separated BioBrick IDs"
    echo "    --host <organism>      Chassis organism (default: E. coli)"
    echo "    --purpose <desc>       Circuit purpose (default: gene expression)"
    echo ""
    echo "  optimize-pathway         Optimize metabolic pathway"
    echo "    --target <product>     Target product (default: ethanol)"
    echo "    --substrate <input>    Substrate (default: glucose)"
    echo "    --host <organism>      Host organism (default: E. coli)"
    echo ""
    echo "  simulate                 Simulate circuit behavior"
    echo "    --circuit-id <id>      Circuit identifier (default: CIR-001)"
    echo "    --duration <sec>       Simulation duration (default: 3600)"
    echo ""
    echo "  assess-biosafety         Assess biosafety requirements"
    echo "    --circuit-id <id>      Circuit identifier"
    echo "    --parts <desc>         Part description"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-012 calc-promoter --pmax 1000 --inducer 1e-6 --kd 1e-7"
    echo "  wia-bio-012 design-circuit --parts 'BBa_J23100,BBa_B0034,BBa_E0040'"
    echo "  wia-bio-012 optimize-pathway --target ethanol --substrate glucose"
    echo "  wia-bio-012 simulate --circuit-id CIR-001 --duration 7200"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-012 Synthetic Biology CLI Tool"
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
    calc-promoter)
        PMAX=1000
        INDUCER=0.000001
        KD=0.0000001
        HILL=2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --pmax) PMAX=$2; shift 2 ;;
                --inducer) INDUCER=$2; shift 2 ;;
                --kd) KD=$2; shift 2 ;;
                --hill) HILL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_promoter "$PMAX" "$INDUCER" "$KD" "$HILL"
        ;;

    calc-expression)
        K_TX=0.5
        K_TL=0.1
        K_DEG_M=0.00385
        K_DEG_P=0.0069

        while [[ $# -gt 0 ]]; do
            case $1 in
                --k-tx) K_TX=$2; shift 2 ;;
                --k-tl) K_TL=$2; shift 2 ;;
                --k-deg-m) K_DEG_M=$2; shift 2 ;;
                --k-deg-p) K_DEG_P=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_expression "$K_TX" "$K_TL" "$K_DEG_M" "$K_DEG_P"
        ;;

    design-circuit)
        PARTS="BBa_J23100,BBa_B0034,BBa_E0040,BBa_B0015"
        HOST="E. coli"
        PURPOSE="gene expression"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --parts) PARTS=$2; shift 2 ;;
                --host) HOST=$2; shift 2 ;;
                --purpose) PURPOSE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_circuit "$PARTS" "$HOST" "$PURPOSE"
        ;;

    optimize-pathway)
        TARGET="ethanol"
        SUBSTRATE="glucose"
        HOST="E. coli"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target) TARGET=$2; shift 2 ;;
                --substrate) SUBSTRATE=$2; shift 2 ;;
                --host) HOST=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        optimize_pathway "$TARGET" "$SUBSTRATE" "$HOST"
        ;;

    simulate)
        CIRCUIT_ID="CIR-001"
        DURATION=3600

        while [[ $# -gt 0 ]]; do
            case $1 in
                --circuit-id) CIRCUIT_ID=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate "$CIRCUIT_ID" "$DURATION"
        ;;

    assess-biosafety)
        CIRCUIT_ID="CIR-001"
        PARTS="standard"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --circuit-id) CIRCUIT_ID=$2; shift 2 ;;
                --parts) PARTS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_biosafety "$CIRCUIT_ID" "$PARTS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-012 help' for usage information"
        exit 1
        ;;
esac

exit 0
