#!/bin/bash

################################################################################
# WIA-IND-005: Cosmetics Data Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to cosmetics data operations
# including ingredient validation, safety scoring, regulatory compliance,
# formulation analysis, and batch tracking.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🧴 WIA-IND-005: Cosmetics Data CLI                    ║"
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

# Validate ingredient
validate_ingredient() {
    local inci=${1:-""}
    local concentration=${2:-1.0}

    if [ -z "$inci" ]; then
        print_error "INCI name is required"
        return 1
    fi

    print_section "Ingredient Validation"
    print_info "INCI Name: $inci"
    print_info "Concentration: $concentration%"

    # Simulate ingredient database lookup
    case "$inci" in
        "Aqua"|"Water")
            print_success "Valid ingredient: Aqua (Water)"
            print_info "Function: Solvent"
            print_info "Safety Score: 100/100"
            print_info "Regulatory: Approved worldwide"
            print_info "Typical Range: 30-95%"
            ;;
        "Glycerin")
            print_success "Valid ingredient: Glycerin"
            print_info "CAS: 56-81-5"
            print_info "Function: Humectant"
            print_info "Safety Score: 95/100"
            print_info "Typical Range: 1-10%"
            if (( $(echo "$concentration > 10" | bc -l) )); then
                print_warning "Concentration above typical range (may be sticky)"
            fi
            ;;
        "Retinol")
            print_success "Valid ingredient: Retinol"
            print_info "CAS: 68-26-8"
            print_info "Function: Active (Anti-aging)"
            print_info "Safety Score: 75/100"
            print_info "Typical Range: 0.01-1.0%"
            print_warning "Light-sensitive, requires proper packaging"
            print_warning "Not recommended for pregnant women"
            if (( $(echo "$concentration > 1.0" | bc -l) )); then
                print_error "Concentration too high - may cause irritation"
            fi
            ;;
        "Sodium Hyaluronate"|"Hyaluronic Acid")
            print_success "Valid ingredient: Sodium Hyaluronate"
            print_info "CAS: 9067-32-7"
            print_info "Function: Humectant, Skin Conditioning"
            print_info "Safety Score: 98/100"
            print_info "Typical Range: 0.05-2.0%"
            ;;
        "Niacinamide")
            print_success "Valid ingredient: Niacinamide"
            print_info "CAS: 98-92-0"
            print_info "Function: Active (Brightening, Anti-aging)"
            print_info "Safety Score: 92/100"
            print_info "Typical Range: 2-10%"
            ;;
        "Phenoxyethanol")
            print_success "Valid ingredient: Phenoxyethanol"
            print_info "CAS: 122-99-6"
            print_info "Function: Preservative"
            print_info "Safety Score: 85/100"
            print_info "EU Max: 1.0%"
            if (( $(echo "$concentration > 1.0" | bc -l) )); then
                print_error "Exceeds EU maximum concentration (1.0%)"
            fi
            ;;
        *)
            print_warning "Ingredient not found in local database"
            print_info "Please verify INCI name spelling"
            ;;
    esac

    echo ""
}

# Calculate safety score
calc_safety() {
    local toxicity=${1:-90}
    local allergenicity=${2:-90}
    local irritation=${3:-85}
    local regulatory=${4:-95}

    print_section "Safety Score Calculation"
    print_info "Toxicity Score: $toxicity/100 (weight: 30%)"
    print_info "Allergenicity Score: $allergenicity/100 (weight: 30%)"
    print_info "Irritation Score: $irritation/100 (weight: 20%)"
    print_info "Regulatory Score: $regulatory/100 (weight: 20%)"

    # Safety Score = (Toxicity × 0.3) + (Allergenicity × 0.3) + (Irritation × 0.2) + (Regulatory × 0.2)
    local score=$(echo "scale=1; ($toxicity * 0.3) + ($allergenicity * 0.3) + ($irritation * 0.2) + ($regulatory * 0.2)" | bc -l)

    print_section "Results"
    print_success "Overall Safety Score: $score/100"

    if (( $(echo "$score >= 90" | bc -l) )); then
        print_info "Rating: Excellent - Very safe for use"
    elif (( $(echo "$score >= 80" | bc -l) )); then
        print_info "Rating: Good - Safe for general use"
    elif (( $(echo "$score >= 70" | bc -l) )); then
        print_info "Rating: Moderate - Use with caution"
    else
        print_warning "Rating: Low - Review formulation"
    fi

    echo ""
}

# Check regulatory compliance
check_regulatory() {
    local product_type=${1:-"serum"}
    local market=${2:-"US"}

    print_section "Regulatory Compliance Check"
    print_info "Product Type: $product_type"
    print_info "Target Market: $market"

    case "$market" in
        US)
            print_success "Market: United States (FDA)"
            print_info "Classification: Cosmetic"
            print_info "Pre-market approval: Not required"
            print_info "Facility registration: Required (MoCRA 2022)"
            print_info "Product listing: Required"
            print_info "Adverse event reporting: Required for serious events"
            print_info "GMP: ISO 22716 recommended"
            ;;
        EU)
            print_success "Market: European Union"
            print_info "Regulation: EC 1223/2009"
            print_info "CPSR: Required (Cosmetic Product Safety Report)"
            print_info "CPNP: Required before marketing"
            print_info "Responsible Person: Must be in EU"
            print_info "GMP: ISO 22716 required"
            print_warning "Check Annexes II, III, IV, V, VI for restrictions"
            ;;
        KR|Korea)
            print_success "Market: South Korea (MFDS)"
            print_info "Category: General Cosmetic (notification)"
            print_info "Functional claims require pre-approval"
            print_info "Korean labeling required"
            print_info "Some animal testing may be required"
            ;;
        JP|Japan)
            print_success "Market: Japan (PMDA)"
            print_info "Category: Cosmetic (notification system)"
            print_info "Quasi-drug requires approval"
            print_info "JSCI: Japanese Standards of Cosmetic Ingredients"
            print_info "Manufacturing facility registration required"
            ;;
        CN|China)
            print_success "Market: China (NMPA)"
            print_info "Category: General Cosmetic (notification)"
            print_info "Special cosmetics require registration"
            print_info "IECIC: Ingredient inventory check required"
            print_warning "Animal testing may be required for imports"
            ;;
        *)
            print_warning "Market not recognized: $market"
            print_info "Supported markets: US, EU, KR, JP, CN"
            ;;
    esac

    echo ""
}

# Analyze formulation
analyze_formula() {
    local total_percent=${1:-100}
    local ph=${2:-5.8}
    local ingredient_count=${3:-12}

    print_section "Formulation Analysis"
    print_info "Total Percentage: $total_percent%"
    print_info "pH: $ph"
    print_info "Ingredient Count: $ingredient_count"

    # Check total percentage
    if [ "$total_percent" = "100" ]; then
        print_success "Total percentage check: PASSED"
    else
        local deviation=$(echo "scale=2; $total_percent - 100" | bc -l)
        if (( $(echo "${deviation#-} <= 0.5" | bc -l) )); then
            print_warning "Total percentage deviation: ${deviation}% (within tolerance)"
        else
            print_error "Total percentage deviation: ${deviation}% (exceeds tolerance)"
        fi
    fi

    # Check pH
    if (( $(echo "$ph >= 3.5" | bc -l) )) && (( $(echo "$ph <= 8.5" | bc -l) )); then
        print_success "pH range check: PASSED (safe range 3.5-8.5)"
        if (( $(echo "$ph >= 4.5" | bc -l) )) && (( $(echo "$ph <= 6.5" | bc -l) )); then
            print_info "pH is in optimal range for skin (4.5-6.5)"
        fi
    else
        print_warning "pH outside typical cosmetic range (3.5-8.5)"
    fi

    # Check ingredient count
    if [ "$ingredient_count" -le 30 ]; then
        print_success "Ingredient count: Reasonable ($ingredient_count ingredients)"
    else
        print_warning "High ingredient count ($ingredient_count) - may increase sensitization risk"
    fi

    # Stability prediction
    print_section "Stability Prediction"
    local stability_score=$(echo "scale=0; 100 - ($ingredient_count * 0.5)" | bc -l)
    if [ "$stability_score" -gt 90 ]; then
        print_success "Predicted Stability: Excellent"
    elif [ "$stability_score" -gt 75 ]; then
        print_success "Predicted Stability: Good"
    else
        print_warning "Predicted Stability: Moderate - stability testing recommended"
    fi

    echo ""
}

# Calculate Margin of Safety
calc_mos() {
    local noael=${1:-100}  # mg/kg bw/day
    local sed=${2:-0.5}    # mg/kg bw/day

    print_section "Margin of Safety (MoS) Calculation"
    print_info "NOAEL: $noael mg/kg bw/day"
    print_info "SED: $sed mg/kg bw/day"

    # MoS = NOAEL / SED
    local mos=$(echo "scale=1; $noael / $sed" | bc -l)

    print_section "Results"
    print_success "Margin of Safety: $mos"

    if (( $(echo "$mos >= 100" | bc -l) )); then
        print_success "Assessment: Generally safe (MoS ≥ 100)"
    elif (( $(echo "$mos >= 10" | bc -l) )); then
        print_warning "Assessment: Marginal (10 ≤ MoS < 100) - requires justification"
    else
        print_error "Assessment: Unacceptable risk (MoS < 10)"
    fi

    echo ""
}

# Calculate exposure dose
calc_exposure() {
    local amount=${1:-2.0}     # g per application
    local concentration=${2:-1.0}  # %
    local absorption=${3:-10}  # %
    local bodyweight=${4:-60}  # kg

    print_section "Systemic Exposure Dose (SED) Calculation"
    print_info "Amount Applied: $amount g"
    print_info "Ingredient Concentration: $concentration%"
    print_info "Dermal Absorption: $absorption%"
    print_info "Body Weight: $bodyweight kg"

    # SED = (Amount × Concentration × Absorption) / Body Weight
    local sed=$(echo "scale=4; ($amount * $concentration * $absorption) / (100 * 100 * $bodyweight)" | bc -l)

    print_section "Results"
    print_success "Systemic Exposure Dose: $sed mg/kg bw/day"

    # Convert to µg/kg bw/day for easier reading
    local sed_ug=$(echo "scale=2; $sed * 1000" | bc -l)
    print_info "SED: $sed_ug µg/kg bw/day"

    echo ""
}

# Track batch
track_batch() {
    local batch_id=${1:-"B2025001"}
    local product_id=${2:-"PRD-001"}

    print_section "Batch Tracking"
    print_info "Batch ID: $batch_id"
    print_info "Product ID: $product_id"

    # Simulate batch lookup
    print_section "Batch Information"
    print_success "Manufacturing Date: 2025-01-15"
    print_success "Expiry Date: 2027-01-15"
    print_success "Batch Size: 500 kg"
    print_success "Formula Version: v2.1"

    print_section "Quality Control Status"
    print_success "Microbiological Test: PASSED"
    print_success "pH: 5.8 (spec: 5.6-6.0)"
    print_success "Viscosity: 4500 cP (spec: 4000-5000)"
    print_success "Release Status: RELEASED"
    print_info "Released By: QA-Manager-01"
    print_info "Release Date: 2025-01-16"

    print_section "Distribution"
    print_info "Units Produced: 10,000"
    print_info "Units Distributed: 8,500"
    print_info "Units in Stock: 1,500"

    echo ""
}

# Calculate concentration
calc_concentration() {
    local ingredient_mass=${1:-1.5}
    local total_mass=${2:-100}

    print_section "Concentration Calculation"
    print_info "Ingredient Mass: $ingredient_mass g"
    print_info "Total Formula Mass: $total_mass g"

    # Concentration (%) = (Ingredient Mass / Total Mass) × 100
    local concentration=$(echo "scale=2; ($ingredient_mass / $total_mass) * 100" | bc -l)

    print_section "Results"
    print_success "Concentration: $concentration%"

    # Also show in ppm and mg/g
    local ppm=$(echo "scale=0; $concentration * 10000" | bc -l)
    local mg_g=$(echo "scale=1; $concentration * 10" | bc -l)

    print_info "In ppm: $ppm ppm"
    print_info "In mg/g: $mg_g mg/g"

    echo ""
}

# Allergen threshold check
check_allergen() {
    local concentration=${1:-0.001}  # %
    local product_type=${2:-"leave-on"}

    print_section "EU Allergen Threshold Check"
    print_info "Concentration: $concentration%"
    print_info "Product Type: $product_type"

    case "$product_type" in
        leave-on|leave_on)
            local threshold=0.001
            print_info "Threshold (Leave-on): > $threshold%"
            if (( $(echo "$concentration > $threshold" | bc -l) )); then
                print_warning "MUST be declared on label (EU 26 allergens)"
                print_info "Concentration ($concentration%) exceeds threshold ($threshold%)"
            else
                print_success "Below declaration threshold"
            fi
            ;;
        rinse-off|rinse_off)
            local threshold=0.01
            print_info "Threshold (Rinse-off): > $threshold%"
            if (( $(echo "$concentration > $threshold" | bc -l) )); then
                print_warning "MUST be declared on label (EU 26 allergens)"
                print_info "Concentration ($concentration%) exceeds threshold ($threshold%)"
            else
                print_success "Below declaration threshold"
            fi
            ;;
        *)
            print_warning "Unknown product type. Use: leave-on or rinse-off"
            ;;
    esac

    echo ""
}

# Calculate preservative efficacy
calc_preservative() {
    local initial_count=${1:-1000000}  # CFU/g
    local day7_count=${2:-100}         # CFU/g

    print_section "Preservative Efficacy Test (PET)"
    print_info "Initial Count (Day 0): $initial_count CFU/g"
    print_info "Day 7 Count: $day7_count CFU/g"

    # Log reduction = log10(Initial / Final)
    local log_reduction=$(echo "scale=2; l($initial_count / $day7_count) / l(10)" | bc -l)

    print_section "Results"
    print_success "Log Reduction: $log_reduction"

    # USP <51> Criteria A (strictest)
    if (( $(echo "$log_reduction >= 2" | bc -l) )); then
        print_success "Day 7 Criteria: PASSED (≥2 log reduction)"
        print_info "Meets USP <51> Criteria A for Day 7"
    else
        print_error "Day 7 Criteria: FAILED (requires ≥2 log reduction)"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-005 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  validate-ingredient      Validate ingredient by INCI name"
    echo "    --inci <name>          INCI name (required)"
    echo "    --concentration <val>  Concentration in % (default: 1.0)"
    echo ""
    echo "  calc-safety              Calculate safety score"
    echo "    --toxicity <score>     Toxicity score 0-100 (default: 90)"
    echo "    --allergen <score>     Allergenicity score 0-100 (default: 90)"
    echo "    --irritation <score>   Irritation score 0-100 (default: 85)"
    echo "    --regulatory <score>   Regulatory score 0-100 (default: 95)"
    echo ""
    echo "  check-regulatory         Check regulatory compliance"
    echo "    --product <type>       Product type (default: serum)"
    echo "    --market <region>      US, EU, KR, JP, CN (default: US)"
    echo ""
    echo "  analyze-formula          Analyze formulation"
    echo "    --total <percent>      Total percentage (default: 100)"
    echo "    --ph <value>           pH value (default: 5.8)"
    echo "    --ingredients <count>  Ingredient count (default: 12)"
    echo ""
    echo "  calc-mos                 Calculate Margin of Safety"
    echo "    --noael <value>        NOAEL in mg/kg bw/day (default: 100)"
    echo "    --sed <value>          SED in mg/kg bw/day (default: 0.5)"
    echo ""
    echo "  calc-exposure            Calculate Systemic Exposure Dose"
    echo "    --amount <g>           Amount applied (default: 2.0 g)"
    echo "    --concentration <%>    Ingredient concentration (default: 1.0%)"
    echo "    --absorption <%>       Dermal absorption (default: 10%)"
    echo "    --bodyweight <kg>      Body weight (default: 60 kg)"
    echo ""
    echo "  track-batch              Track batch information"
    echo "    --batch-id <id>        Batch ID (default: B2025001)"
    echo "    --product-id <id>      Product ID (default: PRD-001)"
    echo ""
    echo "  calc-concentration       Calculate ingredient concentration"
    echo "    --mass <g>             Ingredient mass (default: 1.5 g)"
    echo "    --total <g>            Total formula mass (default: 100 g)"
    echo ""
    echo "  check-allergen           Check EU allergen threshold"
    echo "    --concentration <%>    Concentration (default: 0.001%)"
    echo "    --type <product>       leave-on or rinse-off (default: leave-on)"
    echo ""
    echo "  calc-preservative        Calculate preservative efficacy"
    echo "    --initial <CFU>        Initial count (default: 1000000)"
    echo "    --day7 <CFU>           Day 7 count (default: 100)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-ind-005 validate-ingredient --inci \"Retinol\" --concentration 0.5"
    echo "  wia-ind-005 calc-safety --toxicity 85 --allergen 90"
    echo "  wia-ind-005 check-regulatory --product sunscreen --market EU"
    echo "  wia-ind-005 analyze-formula --total 100 --ph 5.5 --ingredients 15"
    echo "  wia-ind-005 calc-mos --noael 100 --sed 0.5"
    echo "  wia-ind-005 track-batch --batch-id B2025001 --product-id PRD-001"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-IND-005 Cosmetics Data CLI Tool"
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
    validate-ingredient)
        INCI=""
        CONCENTRATION=1.0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --inci) INCI=$2; shift 2 ;;
                --concentration) CONCENTRATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_ingredient "$INCI" "$CONCENTRATION"
        ;;

    calc-safety)
        TOXICITY=90
        ALLERGEN=90
        IRRITATION=85
        REGULATORY=95

        while [[ $# -gt 0 ]]; do
            case $1 in
                --toxicity) TOXICITY=$2; shift 2 ;;
                --allergen) ALLERGEN=$2; shift 2 ;;
                --irritation) IRRITATION=$2; shift 2 ;;
                --regulatory) REGULATORY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_safety "$TOXICITY" "$ALLERGEN" "$IRRITATION" "$REGULATORY"
        ;;

    check-regulatory)
        PRODUCT="serum"
        MARKET="US"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --product) PRODUCT=$2; shift 2 ;;
                --market) MARKET=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        check_regulatory "$PRODUCT" "$MARKET"
        ;;

    analyze-formula)
        TOTAL=100
        PH=5.8
        INGREDIENTS=12

        while [[ $# -gt 0 ]]; do
            case $1 in
                --total) TOTAL=$2; shift 2 ;;
                --ph) PH=$2; shift 2 ;;
                --ingredients) INGREDIENTS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_formula "$TOTAL" "$PH" "$INGREDIENTS"
        ;;

    calc-mos)
        NOAEL=100
        SED=0.5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --noael) NOAEL=$2; shift 2 ;;
                --sed) SED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_mos "$NOAEL" "$SED"
        ;;

    calc-exposure)
        AMOUNT=2.0
        CONCENTRATION=1.0
        ABSORPTION=10
        BODYWEIGHT=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --amount) AMOUNT=$2; shift 2 ;;
                --concentration) CONCENTRATION=$2; shift 2 ;;
                --absorption) ABSORPTION=$2; shift 2 ;;
                --bodyweight) BODYWEIGHT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_exposure "$AMOUNT" "$CONCENTRATION" "$ABSORPTION" "$BODYWEIGHT"
        ;;

    track-batch)
        BATCH_ID="B2025001"
        PRODUCT_ID="PRD-001"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --batch-id) BATCH_ID=$2; shift 2 ;;
                --product-id) PRODUCT_ID=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        track_batch "$BATCH_ID" "$PRODUCT_ID"
        ;;

    calc-concentration)
        MASS=1.5
        TOTAL=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mass) MASS=$2; shift 2 ;;
                --total) TOTAL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_concentration "$MASS" "$TOTAL"
        ;;

    check-allergen)
        CONCENTRATION=0.001
        TYPE="leave-on"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --concentration) CONCENTRATION=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        check_allergen "$CONCENTRATION" "$TYPE"
        ;;

    calc-preservative)
        INITIAL=1000000
        DAY7=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --initial) INITIAL=$2; shift 2 ;;
                --day7) DAY7=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_preservative "$INITIAL" "$DAY7"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-ind-005 help' for usage information"
        exit 1
        ;;
esac

exit 0
