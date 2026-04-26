#!/bin/bash

################################################################################
# WIA-BIO-015: Bio-Manufacturing CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Bio-Manufacturing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to bio-manufacturing calculations
# including yield, productivity, bioreactor monitoring, and purification design.
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
MIN_VIABILITY=70
OPTIMAL_PH_MIN=6.9
OPTIMAL_PH_MAX=7.4
OPTIMAL_TEMP_MIN=36
OPTIMAL_TEMP_MAX=37
OPTIMAL_DO_MIN=30
OPTIMAL_DO_MAX=50

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🏭 WIA-BIO-015: Bio-Manufacturing CLI                 ║"
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

# Calculate product yield
calc_yield() {
    local product_conc=${1:-5.2}
    local final_vol=${2:-1000}
    local substrate_conc=${3:-50}
    local initial_vol=${4:-1000}

    print_section "Product Yield Calculation"
    print_info "Product Concentration: $product_conc g/L"
    print_info "Final Volume: $final_vol L"
    print_info "Substrate Concentration: $substrate_conc g/L"
    print_info "Initial Volume: $initial_vol L"

    # Calculate total product
    local total_product=$(echo "$product_conc * $final_vol" | bc -l)
    print_info "Total Product: $(printf "%.2f" $total_product) g"

    # Calculate total substrate
    local total_substrate=$(echo "$substrate_conc * $initial_vol" | bc -l)
    print_info "Total Substrate: $(printf "%.2f" $total_substrate) g"

    # Calculate yield
    local yield=$(echo "scale=6; $total_product / $total_substrate" | bc -l)
    local efficiency=$(echo "$yield * 100" | bc -l)

    print_section "Results"
    print_success "Yield: $(printf "%.4f" $yield) ($(printf "%.2f" $efficiency)%)"

    # Performance assessment
    if (( $(echo "$yield > 0.15" | bc -l) )); then
        print_success "Performance: EXCELLENT"
    elif (( $(echo "$yield > 0.10" | bc -l) )); then
        print_success "Performance: GOOD"
    elif (( $(echo "$yield > 0.05" | bc -l) )); then
        print_warning "Performance: FAIR"
    else
        print_error "Performance: POOR"
    fi

    echo ""
}

# Calculate volumetric productivity
calc_productivity() {
    local final_conc=${1:-5.2}
    local initial_conc=${2:-0}
    local culture_time=${3:-240}

    print_section "Volumetric Productivity Calculation"
    print_info "Final Concentration: $final_conc g/L"
    print_info "Initial Concentration: $initial_conc g/L"
    print_info "Culture Time: $culture_time hours ($(echo "scale=1; $culture_time / 24" | bc -l) days)"

    # Calculate productivity (g/L/h)
    local productivity=$(echo "scale=6; ($final_conc - $initial_conc) / $culture_time" | bc -l)
    local per_day=$(echo "$productivity * 24" | bc -l)

    print_section "Results"
    print_success "Volumetric Productivity: $(printf "%.4f" $productivity) g/L/h"
    print_success "Daily Productivity: $(printf "%.3f" $per_day) g/L/day"

    # Performance assessment for mAb
    if (( $(echo "$per_day > 0.8" | bc -l) )); then
        print_success "Performance: EXCELLENT (>0.8 g/L/day)"
    elif (( $(echo "$per_day > 0.5" | bc -l) )); then
        print_success "Performance: GOOD (>0.5 g/L/day)"
    elif (( $(echo "$per_day > 0.3" | bc -l) )); then
        print_warning "Performance: FAIR (>0.3 g/L/day)"
    else
        print_error "Performance: POOR (<0.3 g/L/day)"
    fi

    echo ""
}

# Monitor bioreactor
monitor_reactor() {
    local reactor_id=${1:-BR-001}
    local cell_density=${2:-2.5e6}
    local viability=${3:-95}
    local ph=${4:-7.2}
    local temp=${5:-37}
    local do=${6:-40}

    print_section "Bioreactor Monitoring"
    print_info "Reactor ID: $reactor_id"
    print_info "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"

    print_section "Culture Metrics"
    print_info "Viable Cell Density: $cell_density cells/mL"
    print_info "Viability: $viability%"
    print_info "pH: $ph"
    print_info "Temperature: $temp°C"
    print_info "Dissolved Oxygen: $do%"

    print_section "Status Checks"

    # Viability check
    if (( $(echo "$viability < $MIN_VIABILITY" | bc -l) )); then
        print_error "Viability: CRITICAL (<$MIN_VIABILITY%)"
    elif (( $(echo "$viability < 85" | bc -l) )); then
        print_warning "Viability: WARNING (<85%)"
    else
        print_success "Viability: OPTIMAL (>85%)"
    fi

    # pH check
    if (( $(echo "$ph < $OPTIMAL_PH_MIN" | bc -l) )) || (( $(echo "$ph > $OPTIMAL_PH_MAX" | bc -l) )); then
        print_error "pH: OUT OF RANGE ($OPTIMAL_PH_MIN-$OPTIMAL_PH_MAX)"
    else
        print_success "pH: OPTIMAL"
    fi

    # Temperature check
    if (( $(echo "$temp < $OPTIMAL_TEMP_MIN" | bc -l) )) || (( $(echo "$temp > $OPTIMAL_TEMP_MAX" | bc -l) )); then
        print_warning "Temperature: SUBOPTIMAL ($OPTIMAL_TEMP_MIN-$OPTIMAL_TEMP_MAX°C)"
    else
        print_success "Temperature: OPTIMAL"
    fi

    # DO check
    if (( $(echo "$do < 20" | bc -l) )) || (( $(echo "$do > 60" | bc -l) )); then
        print_error "Dissolved Oxygen: OUT OF RANGE (20-60%)"
    elif (( $(echo "$do < $OPTIMAL_DO_MIN" | bc -l) )) || (( $(echo "$do > $OPTIMAL_DO_MAX" | bc -l) )); then
        print_warning "Dissolved Oxygen: SUBOPTIMAL ($OPTIMAL_DO_MIN-$OPTIMAL_DO_MAX%)"
    else
        print_success "Dissolved Oxygen: OPTIMAL"
    fi

    print_section "Overall Status"
    if (( $(echo "$viability < $MIN_VIABILITY" | bc -l) )); then
        print_error "Status: CRITICAL - Immediate action required"
    elif (( $(echo "$viability < 85" | bc -l) )) || (( $(echo "$ph < $OPTIMAL_PH_MIN" | bc -l) )) || (( $(echo "$ph > $OPTIMAL_PH_MAX" | bc -l) )); then
        print_warning "Status: WARNING - Monitor closely"
    else
        print_success "Status: OPTIMAL - Culture performing well"
    fi

    echo ""
}

# Design purification protocol
design_purification() {
    local product=${1:-antibody}
    local scale=${2:-1000}

    print_section "Purification Protocol Design"
    print_info "Product Type: $product"
    print_info "Scale: $scale L"

    print_section "Chromatography Steps"

    if [ "$product" = "antibody" ]; then
        print_info "Step 1: Protein A Affinity Chromatography"
        print_info "  - Type: Capture"
        print_info "  - Binding Capacity: 40 g/L resin"
        print_info "  - Expected Recovery: 95%"
        print_info "  - Expected Purity: 95%"
        print_info "  - Residence Time: 6 minutes"
        print_info ""

        print_info "Step 2: Anion Exchange Chromatography"
        print_info "  - Type: Polishing (flow-through)"
        print_info "  - Binding Capacity: 60 g/L resin"
        print_info "  - Expected Recovery: 92%"
        print_info "  - Expected Purity: 98%"
        print_info "  - Residence Time: 4 minutes"
        print_info ""

        print_info "Step 3: Hydrophobic Interaction Chromatography"
        print_info "  - Type: Polishing"
        print_info "  - Binding Capacity: 40 g/L resin"
        print_info "  - Expected Recovery: 90%"
        print_info "  - Expected Purity: 99%"
        print_info "  - Residence Time: 5 minutes"
        print_info ""

        local overall_recovery=$(echo "0.95 * 0.92 * 0.90 * 100" | bc -l)
        print_section "Overall Performance"
        print_success "Overall Recovery: $(printf "%.1f" $overall_recovery)%"
        print_success "Final Purity: >99%"
        print_info "Estimated Duration: 26 hours"
        print_info "Estimated Cost: ~\$90 per gram"
    else
        print_info "Step 1: Cation Exchange Chromatography (Capture)"
        print_info "Step 2: Hydrophobic Interaction Chromatography"
        print_info "Step 3: Size Exclusion Chromatography (Polishing)"
        print_info ""
        print_success "Overall Recovery: ~75%"
        print_success "Final Purity: >95%"
    fi

    print_section "Additional Steps"
    print_info "✓ Viral Inactivation (Low pH hold)"
    print_info "✓ Ultrafiltration/Diafiltration"
    print_info "✓ Formulation and Fill"

    echo ""
}

# Optimize culture conditions
optimize() {
    local cell_line=${1:-CHO}
    local product=${2:-mAb}
    local target=${3:-8.0}

    print_section "Fermentation Optimization"
    print_info "Cell Line: $cell_line"
    print_info "Product: $product"
    print_info "Target Titer: $target g/L"

    print_section "Recommended Parameters"
    print_success "Initial Cell Density: 3 × 10⁵ cells/mL"
    print_success "Glucose: 6.0 g/L (with fed-batch feeding)"
    print_success "Glutamine: 4.0 mM"
    print_success "Temperature: 37°C (growth), 33°C (production)"
    print_success "pH: 7.1"
    print_success "Dissolved Oxygen: 40%"

    print_section "Feeding Strategy"
    print_info "Mode: Fed-batch with exponential feeding"
    print_info "Feed Start: Day 3"
    print_info "Feed Composition: Concentrated glucose + amino acids"
    print_info "Temperature Shift: Day 7 (37°C → 33°C)"

    print_section "Expected Performance"
    local days=$(echo "scale=1; $target / 0.6" | bc -l)
    print_success "Predicted Titer: $(printf "%.1f" $target) g/L"
    print_info "Culture Duration: ~$(printf "%.0f" $days) days"
    print_info "Productivity: ~0.6 g/L/day"

    print_section "Recommendations"
    print_info "✓ Implement temperature shift for improved product quality"
    print_info "✓ Monitor lactate levels (<30 mM)"
    print_info "✓ Use PAT for real-time glucose monitoring"
    print_info "✓ Validate feeding strategy with 3 batches"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-015 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-yield               Calculate product yield"
    echo "    --product <g/L>        Product concentration (default: 5.2)"
    echo "    --substrate <g/L>      Substrate concentration (default: 50)"
    echo "    --volume <L>           Culture volume (default: 1000)"
    echo ""
    echo "  calc-productivity        Calculate volumetric productivity"
    echo "    --final <g/L>          Final concentration (default: 5.2)"
    echo "    --initial <g/L>        Initial concentration (default: 0)"
    echo "    --time <hours>         Culture time (default: 240)"
    echo ""
    echo "  monitor                  Monitor bioreactor status"
    echo "    --reactor <ID>         Reactor ID (default: BR-001)"
    echo "    --cells <cells/mL>     Cell density (default: 2.5e6)"
    echo "    --viability <%>        Viability (default: 95)"
    echo "    --ph <value>           pH (default: 7.2)"
    echo "    --temp <°C>            Temperature (default: 37)"
    echo "    --do <%>               Dissolved oxygen (default: 40)"
    echo ""
    echo "  design-purification      Design purification protocol"
    echo "    --product <type>       Product type (antibody/protein, default: antibody)"
    echo "    --scale <L>            Scale in liters (default: 1000)"
    echo ""
    echo "  optimize                 Optimize culture conditions"
    echo "    --cell-line <name>     Cell line (default: CHO)"
    echo "    --product <type>       Product type (default: mAb)"
    echo "    --target <g/L>         Target titer (default: 8.0)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-015 calc-yield --product 5.2 --substrate 50 --volume 1000"
    echo "  wia-bio-015 monitor --reactor BR-001 --cells 2.5e6 --viability 95"
    echo "  wia-bio-015 design-purification --product antibody --scale 1000"
    echo "  wia-bio-015 optimize --cell-line CHO --product mAb --target 8.0"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-015 Bio-Manufacturing CLI Tool"
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
    calc-yield)
        PRODUCT=5.2
        SUBSTRATE=50
        VOLUME=1000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --product) PRODUCT=$2; shift 2 ;;
                --substrate) SUBSTRATE=$2; shift 2 ;;
                --volume) VOLUME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_yield "$PRODUCT" "$VOLUME" "$SUBSTRATE" "$VOLUME"
        ;;

    calc-productivity)
        FINAL=5.2
        INITIAL=0
        TIME=240

        while [[ $# -gt 0 ]]; do
            case $1 in
                --final) FINAL=$2; shift 2 ;;
                --initial) INITIAL=$2; shift 2 ;;
                --time) TIME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_productivity "$FINAL" "$INITIAL" "$TIME"
        ;;

    monitor)
        REACTOR="BR-001"
        CELLS="2.5e6"
        VIABILITY=95
        PH=7.2
        TEMP=37
        DO=40

        while [[ $# -gt 0 ]]; do
            case $1 in
                --reactor) REACTOR=$2; shift 2 ;;
                --cells) CELLS=$2; shift 2 ;;
                --viability) VIABILITY=$2; shift 2 ;;
                --ph) PH=$2; shift 2 ;;
                --temp) TEMP=$2; shift 2 ;;
                --do) DO=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        monitor_reactor "$REACTOR" "$CELLS" "$VIABILITY" "$PH" "$TEMP" "$DO"
        ;;

    design-purification)
        PRODUCT="antibody"
        SCALE=1000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --product) PRODUCT=$2; shift 2 ;;
                --scale) SCALE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_purification "$PRODUCT" "$SCALE"
        ;;

    optimize)
        CELL_LINE="CHO"
        PRODUCT="mAb"
        TARGET=8.0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --cell-line) CELL_LINE=$2; shift 2 ;;
                --product) PRODUCT=$2; shift 2 ;;
                --target) TARGET=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        optimize "$CELL_LINE" "$PRODUCT" "$TARGET"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-015 help' for usage information"
        exit 1
        ;;
esac

exit 0
