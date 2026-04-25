#!/bin/bash

################################################################################
# WIA-IND-006: Personalized Cosmetics Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry & Biotech Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to personalized cosmetics
# calculations including skin analysis, custom formulation, genetic skin age
# prediction, ingredient optimization, and efficacy tracking.
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
    echo "║        💅 WIA-IND-006: Personalized Cosmetics CLI            ║"
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

# Calculate skin health score
calc_skin_score() {
    local hydration=${1:-50}
    local elasticity=${2:-70}
    local oil_balance=${3:-50}
    local sensitivity=${4:-30}
    local pigmentation=${5:-75}

    print_section "Skin Health Score Calculation"
    print_info "Hydration Level: $hydration / 100"
    print_info "Elasticity Score: $elasticity / 100"
    print_info "Oil Balance: $oil_balance / 100 (50 = optimal)"
    print_info "Sensitivity Index: $sensitivity / 100 (lower is better)"
    print_info "Pigmentation Uniformity: $pigmentation / 100"

    # Oil balance deviation from optimal (50)
    local oil_deviation=$(echo "scale=2; ($oil_balance - 50)" | bc -l)
    if [ $(echo "$oil_deviation < 0" | bc -l) -eq 1 ]; then
        oil_deviation=$(echo "scale=2; $oil_deviation * -1" | bc -l)
    fi
    local oil_score=$(echo "scale=2; 100 - ($oil_deviation * 2)" | bc -l)
    if [ $(echo "$oil_score < 0" | bc -l) -eq 1 ]; then
        oil_score=0
    fi

    # Sensitivity is inverse (lower is better)
    local sensitivity_score=$(echo "scale=2; 100 - $sensitivity" | bc -l)

    # Weighted score calculation
    # Score = (H × 0.25) + (E × 0.25) + (O × 0.20) + (S × 0.15) + (P × 0.15)
    local score=$(echo "scale=1; ($hydration * 0.25) + ($elasticity * 0.25) + ($oil_score * 0.20) + ($sensitivity_score * 0.15) + ($pigmentation * 0.15)" | bc -l)

    print_section "Results"
    print_info "Oil Balance Score: $oil_score / 100"
    print_info "Sensitivity Score: $sensitivity_score / 100"
    print_success "Overall Skin Health Score: $score / 100"

    # Classification
    if [ $(echo "$score >= 80" | bc -l) -eq 1 ]; then
        print_success "Classification: Excellent skin health"
    elif [ $(echo "$score >= 65" | bc -l) -eq 1 ]; then
        print_success "Classification: Good skin health"
    elif [ $(echo "$score >= 50" | bc -l) -eq 1 ]; then
        print_warning "Classification: Fair skin health (improvement recommended)"
    else
        print_warning "Classification: Poor skin health (intensive care needed)"
    fi

    echo ""
}

# Calculate ingredient concentration adjustment
calc_ingredient_concentration() {
    local base_conc=${1:-1.0}
    local skin_factor=${2:-0}
    local max_adjustment=${3:-0.3}

    print_section "Ingredient Concentration Optimization"
    print_info "Base Concentration: $base_conc %"
    print_info "Skin Factor: $skin_factor (-1.0 to +1.0)"
    print_info "Maximum Adjustment: ±$max_adjustment (±${max_adjustment}%)"

    # Adjusted Concentration = Base × (1 + Skin Factor × Max Adjustment)
    local adjustment=$(echo "scale=4; 1 + ($skin_factor * $max_adjustment)" | bc -l)
    local adjusted=$(echo "scale=3; $base_conc * $adjustment" | bc -l)

    print_section "Results"
    print_info "Adjustment Multiplier: ${adjustment}x"
    print_success "Adjusted Concentration: $adjusted %"

    # Example interpretation
    if [ $(echo "$skin_factor > 0" | bc -l) -eq 1 ]; then
        print_info "Interpretation: Increasing concentration due to skin needs"
    elif [ $(echo "$skin_factor < 0" | bc -l) -eq 1 ]; then
        print_info "Interpretation: Decreasing concentration to avoid over-treatment"
    else
        print_info "Interpretation: Maintaining base concentration"
    fi

    echo ""
}

# Calculate genetic skin age
calc_genetic_age() {
    local chrono_age=${1:-30}
    local genetic_risk=${2:-5}
    local care_score=${3:-50}

    print_section "Genetic Skin Age Prediction"
    print_info "Chronological Age: $chrono_age years"
    print_info "Genetic Risk Score: $genetic_risk / 10 (0=protected, 10=high risk)"
    print_info "Skincare Quality Score: $care_score / 100"

    # Biological Age = Chronological + (Genetic Risk × 5) - (Care Score × 0.3)
    local genetic_impact=$(echo "scale=1; $genetic_risk * 5" | bc -l)
    local care_impact=$(echo "scale=1; $care_score * 0.3" | bc -l)
    local bio_age=$(echo "scale=1; $chrono_age + $genetic_impact - $care_impact" | bc -l)

    # Ensure biological age is not negative
    if [ $(echo "$bio_age < 0" | bc -l) -eq 1 ]; then
        bio_age=0
    fi

    local age_diff=$(echo "scale=1; $bio_age - $chrono_age" | bc -l)

    print_section "Results"
    print_info "Genetic Aging Impact: +$genetic_impact years"
    print_info "Skincare Benefit: -$care_impact years"
    print_success "Biological Skin Age: $bio_age years"

    if [ $(echo "$age_diff > 0" | bc -l) -eq 1 ]; then
        print_warning "Skin is aging $age_diff years faster than chronological age"
        print_info "Recommendation: Intensive anti-aging regimen with genetic-targeted actives"
    elif [ $(echo "$age_diff < 0" | bc -l) -eq 1 ]; then
        age_diff=$(echo "scale=1; $age_diff * -1" | bc -l)
        print_success "Skin is $age_diff years younger than chronological age"
        print_info "Recommendation: Maintain current skincare routine"
    else
        print_info "Biological age matches chronological age"
        print_info "Recommendation: Standard preventive care"
    fi

    echo ""
}

# Calculate product efficacy score
calc_efficacy() {
    local target_achievement=${1:-0.25}
    local baseline_improvement=${2:-0.20}
    local compliance=${3:-85}

    print_section "Product Efficacy Score Calculation"
    print_info "Target Achievement: $target_achievement (measured improvement)"
    print_info "Baseline Improvement: $baseline_improvement (expected with standard products)"
    print_info "Compliance Rate: $compliance %"

    # Convert compliance to factor (0-1)
    local compliance_factor=$(echo "scale=2; $compliance / 100" | bc -l)

    # Efficacy = (Target / Baseline) × 100 × Compliance
    local efficacy_raw=$(echo "scale=2; ($target_achievement / $baseline_improvement) * 100" | bc -l)
    local efficacy=$(echo "scale=1; $efficacy_raw * $compliance_factor" | bc -l)

    print_section "Results"
    print_info "Raw Efficacy (100% compliance): $efficacy_raw %"
    print_success "Adjusted Efficacy: $efficacy %"

    # Interpretation
    if [ $(echo "$efficacy >= 150" | bc -l) -eq 1 ]; then
        print_success "Outstanding efficacy (>150%)"
    elif [ $(echo "$efficacy >= 120" | bc -l) -eq 1 ]; then
        print_success "Excellent efficacy (120-150%)"
    elif [ $(echo "$efficacy >= 100" | bc -l) -eq 1 ]; then
        print_success "Good efficacy (100-120%)"
    elif [ $(echo "$efficacy >= 80" | bc -l) -eq 1 ]; then
        print_warning "Moderate efficacy (80-100%)"
    else
        print_warning "Below expected efficacy (<80%) - consider reformulation"
    fi

    if [ $(echo "$compliance < 90" | bc -l) -eq 1 ]; then
        print_warning "Low compliance detected - results may improve with consistent use"
    fi

    echo ""
}

# Calculate ingredient compatibility
calc_compatibility() {
    local interaction_penalty=${1:-10}
    local sensitivity_risk=${2:-5}
    local stability_issues=${3:-0}

    print_section "Ingredient Compatibility Index"
    print_info "Interaction Penalty: $interaction_penalty points"
    print_info "Sensitivity Risk: $sensitivity_risk points"
    print_info "Stability Issues: $stability_issues points"

    # Compatibility = 100 - (Interaction + Sensitivity + Stability)
    local total_penalty=$(echo "scale=0; $interaction_penalty + $sensitivity_risk + $stability_issues" | bc -l)
    local compatibility=$(echo "scale=0; 100 - $total_penalty" | bc -l)

    if [ $(echo "$compatibility < 0" | bc -l) -eq 1 ]; then
        compatibility=0
    fi

    print_section "Results"
    print_info "Total Penalty: $total_penalty points"
    print_success "Compatibility Score: $compatibility / 100"

    # Approval threshold
    if [ $(echo "$compatibility >= 80" | bc -l) -eq 1 ]; then
        print_success "Status: Excellent compatibility - highly recommended"
    elif [ $(echo "$compatibility >= 60" | bc -l) -eq 1 ]; then
        print_success "Status: Good compatibility - approved for use"
    elif [ $(echo "$compatibility >= 40" | bc -l) -eq 1 ]; then
        print_warning "Status: Marginal compatibility - use with caution"
    else
        print_error "Status: Poor compatibility - NOT recommended"
    fi

    echo ""
}

# Analyze Baumann skin type from scores
analyze_baumann() {
    local sebum=${1:-100}
    local erythema=${2:-250}
    local pigmentation_risk=${3:-50}
    local elasticity=${4:-75}

    print_section "Baumann Skin Type Analysis"
    print_info "Sebum Level: $sebum μg/cm²"
    print_info "Erythema Index: $erythema EI"
    print_info "Pigmentation Risk: $pigmentation_risk"
    print_info "Elasticity Score: $elasticity"

    # Determine Oiliness (O/D)
    local oil_type
    if [ $(echo "$sebum > 150" | bc -l) -eq 1 ]; then
        oil_type="O"
        oil_desc="Oily"
    else
        oil_type="D"
        oil_desc="Dry"
    fi

    # Determine Sensitivity (S/R)
    local sens_type
    if [ $(echo "$erythema > 250" | bc -l) -eq 1 ]; then
        sens_type="S"
        sens_desc="Sensitive"
    else
        sens_type="R"
        sens_desc="Resistant"
    fi

    # Determine Pigmentation (P/N)
    local pig_type
    if [ $(echo "$pigmentation_risk > 75" | bc -l) -eq 1 ]; then
        pig_type="P"
        pig_desc="Pigmented"
    else
        pig_type="N"
        pig_desc="Non-Pigmented"
    fi

    # Determine Wrinkles (W/T)
    local wrinkle_type
    if [ $(echo "$elasticity < 70" | bc -l) -eq 1 ]; then
        wrinkle_type="W"
        wrinkle_desc="Wrinkle-Prone"
    else
        wrinkle_type="T"
        wrinkle_desc="Tight"
    fi

    local baumann_type="${oil_type}${sens_type}${pig_type}${wrinkle_type}"

    print_section "Results"
    print_success "Baumann Type: $baumann_type"
    print_info "  Oiliness: $oil_type ($oil_desc)"
    print_info "  Sensitivity: $sens_type ($sens_desc)"
    print_info "  Pigmentation: $pig_type ($pig_desc)"
    print_info "  Wrinkles: $wrinkle_type ($wrinkle_desc)"

    # Recommendations based on type
    print_section "Skincare Recommendations"

    if [ "$oil_type" = "O" ]; then
        print_info "• Oil control: Niacinamide (5-10%), Salicylic Acid (1-2%)"
    else
        print_info "• Hydration: Hyaluronic Acid, Ceramides, Rich moisturizers"
    fi

    if [ "$sens_type" = "S" ]; then
        print_info "• Soothing: Centella, Allantoin, avoid harsh actives"
    else
        print_info "• Can tolerate: Retinoids, AHAs, stronger actives"
    fi

    if [ "$pig_type" = "P" ]; then
        print_info "• Brightening: Vitamin C (15%), Niacinamide, Tranexamic Acid"
        print_info "• Sun protection: SPF 50+ daily, avoid prolonged sun"
    fi

    if [ "$wrinkle_type" = "W" ]; then
        print_info "• Anti-aging: Retinol (0.25-1%), Peptides, Vitamin C"
    fi

    echo ""
}

# Optimize formulation for climate
optimize_climate() {
    local climate=${1:-temperate}
    local base_hydration=${2:-5}
    local base_oil=${3:-10}

    print_section "Climate-Based Formulation Optimization"
    print_info "Climate Zone: $climate"
    print_info "Base Hydration %: $base_hydration"
    print_info "Base Oil Phase %: $base_oil"

    local hydration_mod=0
    local oil_mod=0
    local recommendations=""

    case $climate in
        tropical-humid)
            hydration_mod=-0.2
            oil_mod=-0.15
            recommendations="Light textures, oil control (+40%), antioxidants (+25%)"
            ;;
        desert-arid)
            hydration_mod=0.4
            oil_mod=0.35
            recommendations="Rich textures, occlusives (+40%), barrier repair (+30%)"
            ;;
        arctic-cold)
            hydration_mod=0.3
            oil_mod=0.4
            recommendations="Balms, ceramides (+35%), soothing (+25%)"
            ;;
        temperate)
            hydration_mod=0
            oil_mod=0
            recommendations="Seasonal adjustment recommended (light in summer, rich in winter)"
            ;;
        *)
            recommendations="Standard formulation"
            ;;
    esac

    # Calculate adjusted concentrations
    local adj_hydration=$(echo "scale=2; $base_hydration * (1 + $hydration_mod)" | bc -l)
    local adj_oil=$(echo "scale=2; $base_oil * (1 + $oil_mod)" | bc -l)

    print_section "Results"
    print_info "Hydration Modifier: $hydration_mod ($(echo "scale=0; $hydration_mod * 100" | bc -l)%)"
    print_info "Oil Phase Modifier: $oil_mod ($(echo "scale=0; $oil_mod * 100" | bc -l)%)"
    print_success "Adjusted Hydration: $adj_hydration %"
    print_success "Adjusted Oil Phase: $adj_oil %"
    print_info "Recommendations: $recommendations"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-006 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-skin-score              Calculate overall skin health score"
    echo "    --hydration <0-100>        Hydration level (default: 50)"
    echo "    --elasticity <0-100>       Elasticity score (default: 70)"
    echo "    --oil <0-100>              Oil balance (50=optimal, default: 50)"
    echo "    --sensitivity <0-100>      Sensitivity index (lower better, default: 30)"
    echo "    --pigmentation <0-100>     Pigmentation uniformity (default: 75)"
    echo ""
    echo "  calc-ingredient-conc         Calculate adjusted ingredient concentration"
    echo "    --base <float>             Base concentration % (default: 1.0)"
    echo "    --factor <-1.0 to 1.0>     Skin factor (default: 0)"
    echo "    --max-adj <float>          Max adjustment (default: 0.3)"
    echo ""
    echo "  calc-genetic-age             Calculate biological skin age from genetics"
    echo "    --age <number>             Chronological age (default: 30)"
    echo "    --risk <0-10>              Genetic risk score (default: 5)"
    echo "    --care <0-100>             Skincare quality score (default: 50)"
    echo ""
    echo "  calc-efficacy                Calculate product efficacy score"
    echo "    --achieved <float>         Measured improvement (default: 0.25)"
    echo "    --baseline <float>         Expected baseline (default: 0.20)"
    echo "    --compliance <0-100>       Usage compliance % (default: 85)"
    echo ""
    echo "  calc-compatibility           Calculate ingredient compatibility"
    echo "    --interaction <0-100>      Interaction penalty (default: 10)"
    echo "    --sensitivity <0-100>      Sensitivity risk (default: 5)"
    echo "    --stability <0-100>        Stability issues (default: 0)"
    echo ""
    echo "  analyze-baumann              Analyze Baumann skin type"
    echo "    --sebum <number>           Sebum level μg/cm² (default: 100)"
    echo "    --erythema <number>        Erythema index (default: 250)"
    echo "    --pig-risk <0-100>         Pigmentation risk (default: 50)"
    echo "    --elasticity <0-100>       Elasticity score (default: 75)"
    echo ""
    echo "  optimize-climate             Optimize formulation for climate"
    echo "    --climate <zone>           Climate zone (tropical-humid|desert-arid|arctic-cold|temperate)"
    echo "    --hydration <float>        Base hydration % (default: 5)"
    echo "    --oil <float>              Base oil phase % (default: 10)"
    echo ""
    echo "  --version                    Show version information"
    echo "  --help                       Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-ind-006 calc-skin-score --hydration 45 --elasticity 68 --oil 35"
    echo "  wia-ind-006 calc-genetic-age --age 32 --risk 8 --care 70"
    echo "  wia-ind-006 analyze-baumann --sebum 180 --erythema 320"
    echo "  wia-ind-006 optimize-climate --climate tropical-humid --hydration 5 --oil 12"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-IND-006: Personalized Cosmetics Standard"
    echo "CLI Version: $VERSION"
    echo ""
    echo "License: MIT"
    echo "Author: WIA Industry & Biotech Research Group"
    echo "© 2025 SmileStory Inc. / WIA"
    echo ""
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        calc-skin-score)
            shift
            hydration=50
            elasticity=70
            oil=50
            sensitivity=30
            pigmentation=75

            while [ $# -gt 0 ]; do
                case "$1" in
                    --hydration) hydration=$2; shift 2 ;;
                    --elasticity) elasticity=$2; shift 2 ;;
                    --oil) oil=$2; shift 2 ;;
                    --sensitivity) sensitivity=$2; shift 2 ;;
                    --pigmentation) pigmentation=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_skin_score $hydration $elasticity $oil $sensitivity $pigmentation
            ;;

        calc-ingredient-conc)
            shift
            base=1.0
            factor=0
            max_adj=0.3

            while [ $# -gt 0 ]; do
                case "$1" in
                    --base) base=$2; shift 2 ;;
                    --factor) factor=$2; shift 2 ;;
                    --max-adj) max_adj=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_ingredient_concentration $base $factor $max_adj
            ;;

        calc-genetic-age)
            shift
            age=30
            risk=5
            care=50

            while [ $# -gt 0 ]; do
                case "$1" in
                    --age) age=$2; shift 2 ;;
                    --risk) risk=$2; shift 2 ;;
                    --care) care=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_genetic_age $age $risk $care
            ;;

        calc-efficacy)
            shift
            achieved=0.25
            baseline=0.20
            compliance=85

            while [ $# -gt 0 ]; do
                case "$1" in
                    --achieved) achieved=$2; shift 2 ;;
                    --baseline) baseline=$2; shift 2 ;;
                    --compliance) compliance=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_efficacy $achieved $baseline $compliance
            ;;

        calc-compatibility)
            shift
            interaction=10
            sensitivity=5
            stability=0

            while [ $# -gt 0 ]; do
                case "$1" in
                    --interaction) interaction=$2; shift 2 ;;
                    --sensitivity) sensitivity=$2; shift 2 ;;
                    --stability) stability=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            calc_compatibility $interaction $sensitivity $stability
            ;;

        analyze-baumann)
            shift
            sebum=100
            erythema=250
            pig_risk=50
            elasticity=75

            while [ $# -gt 0 ]; do
                case "$1" in
                    --sebum) sebum=$2; shift 2 ;;
                    --erythema) erythema=$2; shift 2 ;;
                    --pig-risk) pig_risk=$2; shift 2 ;;
                    --elasticity) elasticity=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            analyze_baumann $sebum $erythema $pig_risk $elasticity
            ;;

        optimize-climate)
            shift
            climate="temperate"
            hydration=5
            oil=10

            while [ $# -gt 0 ]; do
                case "$1" in
                    --climate) climate=$2; shift 2 ;;
                    --hydration) hydration=$2; shift 2 ;;
                    --oil) oil=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            optimize_climate $climate $hydration $oil
            ;;

        --version)
            show_version
            ;;

        --help|help)
            show_help
            ;;

        *)
            print_error "Unknown command: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
