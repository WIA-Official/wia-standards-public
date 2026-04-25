#!/bin/bash

################################################################################
# WIA-IND-007: Food Tech Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Food Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to food technology calculations
# including protein efficiency, nutritional analysis, food safety scoring,
# shelf life prediction, fermentation optimization, and carbon footprint.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
AVOGADRO=6.022e23

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🍽️  WIA-IND-007: Food Tech CLI                      ║"
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

# Calculate protein efficiency (PER and FCR)
calc_protein() {
    local protein_type=${1:-cultured-beef}
    local feed_input=${2:-2.5}
    local protein_output=${3:-1.2}

    print_section "Protein Efficiency Calculation"
    print_info "Protein Type: $protein_type"
    print_info "Feed Input: $feed_input kg"
    print_info "Protein Output: $protein_output kg"

    # Feed Conversion Ratio (FCR)
    local fcr=$(echo "scale=2; $feed_input / $protein_output" | bc -l)
    print_info "Feed Conversion Ratio (FCR): $fcr"

    # Protein Efficiency Ratio (PER) - simplified calculation
    # Assumes weight gain proportional to protein output
    local per=$(echo "scale=2; $protein_output / ($feed_input * 0.15)" | bc -l)

    # Efficiency vs traditional beef (FCR ~25)
    local efficiency_gain=$(echo "scale=1; (25 - $fcr) / 25 * 100" | bc -l)

    print_section "Results"
    print_success "Feed Conversion Ratio: $fcr (lower is better)"
    print_success "Protein Efficiency Ratio: $per"
    print_success "Efficiency vs. Beef: +${efficiency_gain}%"

    # Environmental impact estimation
    local water_saved=$(echo "scale=0; (15400 - 367) * $protein_output" | bc -l)
    local carbon_saved=$(echo "scale=1; (50 - 4) * $protein_output" | bc -l)

    print_section "Environmental Impact (per kg protein)"
    print_info "Water Saved: ${water_saved} liters (vs. traditional beef)"
    print_info "GHG Reduced: ${carbon_saved} kg CO2e (vs. traditional beef)"

    echo ""
}

# Analyze nutritional density
analyze_nutrition() {
    local food=${1:-tofu}
    local serving=${2:-100}

    print_section "Nutritional Analysis"
    print_info "Food: $food"
    print_info "Serving Size: $serving g"

    # Sample nutritional data for common foods (per 100g)
    case $food in
        tofu)
            local calories=76
            local protein=8
            local fiber=0.3
            local iron=5.4
            local calcium=350
            ;;
        spinach)
            local calories=23
            local protein=2.9
            local fiber=2.2
            local iron=2.7
            local calcium=99
            ;;
        beef)
            local calories=250
            local protein=26
            local fiber=0
            local iron=2.6
            local calcium=18
            ;;
        *)
            print_error "Food not in database. Using default values."
            local calories=100
            local protein=5
            local fiber=1
            local iron=1
            local calcium=50
            ;;
    esac

    # Calculate Nutritional Density Score (NDS)
    # NDS = (sum of nutrient %RDI) / calories × 100
    local protein_rdi=$(echo "scale=1; $protein / 50 * 100" | bc -l)
    local iron_rdi=$(echo "scale=1; $iron / 8 * 100" | bc -l)
    local calcium_rdi=$(echo "scale=1; $calcium / 1000 * 100" | bc -l)
    local fiber_score=$(echo "scale=1; $fiber / 25 * 100" | bc -l)

    local total_rdi=$(echo "scale=1; $protein_rdi + $iron_rdi + $calcium_rdi + $fiber_score" | bc -l)
    local nds=$(echo "scale=0; $total_rdi / $calories * 100" | bc -l)

    print_section "Nutritional Profile (per 100g)"
    print_info "Calories: $calories kcal"
    print_info "Protein: ${protein}g (${protein_rdi}% RDI)"
    print_info "Fiber: ${fiber}g (${fiber_score}% RDI)"
    print_info "Iron: ${iron}mg (${iron_rdi}% RDI)"
    print_info "Calcium: ${calcium}mg (${calcium_rdi}% RDI)"

    print_section "Results"
    print_success "Nutritional Density Score (NDS): $nds"

    # Classification
    if [ $(echo "$nds > 500" | bc -l) -eq 1 ]; then
        print_success "Classification: SUPERFOOD ⭐⭐⭐"
    elif [ $(echo "$nds > 200" | bc -l) -eq 1 ]; then
        print_success "Classification: Nutrient-Dense ⭐⭐"
    elif [ $(echo "$nds > 100" | bc -l) -eq 1 ]; then
        print_info "Classification: Moderate ⭐"
    else
        print_warning "Classification: Low nutrient density"
    fi

    echo ""
}

# Assess food safety
assess_safety() {
    local microbial=${1:-95}
    local chemical=${2:-98}
    local physical=${3:-92}

    print_section "Food Safety Assessment"
    print_info "Microbial Score: $microbial / 100"
    print_info "Chemical Score: $chemical / 100"
    print_info "Physical Score: $physical / 100"

    # Calculate Food Safety Index (FSI)
    # FSI = (Microbial × 0.4) + (Chemical × 0.3) + (Physical × 0.3)
    local fsi=$(echo "scale=1; ($microbial * 0.4) + ($chemical * 0.3) + ($physical * 0.3)" | bc -l)

    print_section "Results"
    print_success "Food Safety Index (FSI): $fsi / 100"

    # Classification
    if [ $(echo "$fsi >= 90" | bc -l) -eq 1 ]; then
        print_success "Classification: SAFE ✓"
        print_success "Risk Level: LOW"
    elif [ $(echo "$fsi >= 70" | bc -l) -eq 1 ]; then
        print_info "Classification: Acceptable"
        print_info "Risk Level: MEDIUM"
    elif [ $(echo "$fsi >= 50" | bc -l) -eq 1 ]; then
        print_warning "Classification: Marginal"
        print_warning "Risk Level: HIGH"
    else
        print_error "Classification: UNSAFE"
        print_error "Risk Level: CRITICAL - Do not consume"
    fi

    echo ""
}

# Predict shelf life
predict_shelf() {
    local product=${1:-yogurt}
    local temp=${2:-4}
    local ref_days=${3:-14}

    print_section "Shelf Life Prediction"
    print_info "Product: $product"
    print_info "Storage Temperature: ${temp}°C"
    print_info "Reference Shelf Life: $ref_days days at 4°C"

    # Q10 temperature coefficient (typically 2-3 for food)
    local q10=2.5
    local ref_temp=4

    # Shelf Life = Q10^((T_ref - T) / 10) × SL_ref
    local temp_diff=$(echo "scale=2; ($ref_temp - $temp) / 10" | bc -l)
    local factor=$(echo "scale=4; e(l($q10) * $temp_diff)" | bc -l)
    local shelf_life=$(echo "scale=0; $ref_days * $factor" | bc -l)

    print_section "Results"
    print_success "Predicted Shelf Life: $shelf_life days"

    # Calculate for different temperatures
    print_section "Temperature Impact"
    for storage_temp in -18 0 4 10 20 25; do
        local diff=$(echo "scale=2; ($ref_temp - $storage_temp) / 10" | bc -l)
        local f=$(echo "scale=4; e(l($q10) * $diff)" | bc -l)
        local sl=$(echo "scale=0; $ref_days * $f" | bc -l)

        if [ $storage_temp -eq $temp ]; then
            print_success "At ${storage_temp}°C: $sl days ← Current"
        else
            print_info "At ${storage_temp}°C: $sl days"
        fi
    done

    echo ""
}

# Optimize fermentation
optimize_ferment() {
    local organism=${1:-kombucha}
    local temp=${2:-25}
    local ph=${3:-3.5}

    print_section "Fermentation Optimization"
    print_info "Organism: $organism"
    print_info "Temperature: ${temp}°C"
    print_info "pH: $ph"

    # Calculate specific growth rate using simplified Arrhenius equation
    # μ = μ_max × exp(-Ea/RT) × pH_factor
    local optimal_temp=30
    local r=8.314  # Gas constant J/mol/K
    local ea=50000  # Activation energy J/mol (typical)

    local temp_k=$(echo "scale=2; $temp + 273.15" | bc -l)
    local optimal_k=$(echo "scale=2; $optimal_temp + 273.15" | bc -l)

    # Temperature factor (simplified)
    local temp_factor=$(echo "scale=3; e(-$ea / ($r * $temp_k) + $ea / ($r * $optimal_k))" | bc -l)

    # pH factor (bell curve around optimal pH 4.5 for kombucha)
    local optimal_ph=4.5
    local ph_diff=$(echo "scale=2; ($ph - $optimal_ph) ^ 2" | bc -l)
    local ph_factor=$(echo "scale=3; e(-$ph_diff)" | bc -l)

    # Maximum growth rate (typical for kombucha)
    local mu_max=0.15  # h^-1
    local mu=$(echo "scale=4; $mu_max * $temp_factor * $ph_factor" | bc -l)

    # Doubling time = ln(2) / μ
    local doubling_time=$(echo "scale=1; 0.693 / $mu" | bc -l)

    # Fermentation time estimate (7 doublings typical)
    local ferment_time=$(echo "scale=0; $doubling_time * 7" | bc -l)

    print_section "Results"
    print_success "Specific Growth Rate (μ): ${mu} h⁻¹"
    print_success "Doubling Time: ${doubling_time} hours"
    print_success "Estimated Fermentation Time: ${ferment_time} hours"

    # Optimization recommendations
    print_section "Optimization Recommendations"

    if [ $(echo "$temp < 20" | bc -l) -eq 1 ]; then
        print_warning "Temperature is low. Consider increasing to 25-30°C"
    elif [ $(echo "$temp > 35" | bc -l) -eq 1 ]; then
        print_warning "Temperature is high. Consider decreasing to 25-30°C"
    else
        print_success "Temperature is optimal"
    fi

    if [ $(echo "$ph < 3.0" | bc -l) -eq 1 ]; then
        print_warning "pH is very acidic. May inhibit growth"
    elif [ $(echo "$ph > 5.0" | bc -l) -eq 1 ]; then
        print_warning "pH is high. Risk of contamination"
    else
        print_success "pH is in acceptable range"
    fi

    # Calculate expected yield (simplified)
    local yield_factor=$(echo "scale=2; $temp_factor * $ph_factor" | bc -l)
    local expected_yield=$(echo "scale=0; 85 * $yield_factor" | bc -l)
    print_info "Expected Yield: ${expected_yield}% of theoretical maximum"

    echo ""
}

# Calculate carbon footprint
calc_carbon() {
    local product=${1:-plant-burger}
    local lifecycle=${2:-full}

    print_section "Carbon Footprint Analysis"
    print_info "Product: $product"
    print_info "Lifecycle Scope: $lifecycle"

    # Sample carbon footprint data (kg CO2e per kg product)
    case $product in
        plant-burger)
            local agriculture=0.5
            local processing=0.8
            local packaging=0.3
            local transport=0.2
            local retail=0.1
            local conventional_product="beef burger"
            local conventional_emissions=50
            ;;
        cultured-meat)
            local agriculture=0.2
            local processing=2.5
            local packaging=0.3
            local transport=0.2
            local retail=0.1
            local conventional_product="conventional meat"
            local conventional_emissions=50
            ;;
        tofu)
            local agriculture=0.3
            local processing=0.5
            local packaging=0.2
            local transport=0.15
            local retail=0.05
            local conventional_product="chicken"
            local conventional_emissions=6
            ;;
        *)
            print_error "Product not in database"
            return 1
            ;;
    esac

    # Calculate total
    local total=$(echo "scale=2; $agriculture + $processing + $packaging + $transport + $retail" | bc -l)

    print_section "Emissions Breakdown (kg CO2e per kg product)"
    print_info "Agriculture: $agriculture"
    print_info "Processing: $processing"
    print_info "Packaging: $packaging"
    print_info "Transportation: $transport"
    print_info "Retail: $retail"

    print_section "Results"
    print_success "Total Carbon Footprint: ${total} kg CO2e per kg"

    # Comparison
    local reduction=$(echo "scale=1; ($conventional_emissions - $total) / $conventional_emissions * 100" | bc -l)
    print_section "Comparison to $conventional_product"
    print_info "Conventional Product: ${conventional_emissions} kg CO2e per kg"
    print_success "Reduction: ${reduction}%"

    # Environmental impact
    local trees_offset=$(echo "scale=1; $total / 20" | bc -l)  # 1 tree absorbs ~20kg CO2/year
    print_info "Equivalent to emissions absorbed by ${trees_offset} trees/year"

    echo ""
}

# Calculate personalized nutrition
calc_macros() {
    local age=${1:-35}
    local gender=${2:-male}
    local weight=${3:-75}
    local height=${4:-175}
    local activity=${5:-moderate}

    print_section "Personalized Nutrition Calculator"
    print_info "Age: $age years"
    print_info "Gender: $gender"
    print_info "Weight: $weight kg"
    print_info "Height: $height cm"
    print_info "Activity Level: $activity"

    # Calculate BMR using Mifflin-St Jeor Equation
    if [ "$gender" = "male" ]; then
        local bmr=$(echo "scale=0; 10 * $weight + 6.25 * $height - 5 * $age + 5" | bc -l)
    else
        local bmr=$(echo "scale=0; 10 * $weight + 6.25 * $height - 5 * $age - 161" | bc -l)
    fi

    # Activity factor
    case $activity in
        sedentary)
            local activity_factor=1.2
            ;;
        light)
            local activity_factor=1.375
            ;;
        moderate)
            local activity_factor=1.55
            ;;
        active)
            local activity_factor=1.725
            ;;
        very_active)
            local activity_factor=1.9
            ;;
        *)
            local activity_factor=1.55
            ;;
    esac

    # Calculate TDEE
    local tdee=$(echo "scale=0; $bmr * $activity_factor" | bc -l)

    # Calculate macros (balanced approach: 30% protein, 40% carbs, 30% fat)
    local protein_cal=$(echo "scale=0; $tdee * 0.30" | bc -l)
    local carb_cal=$(echo "scale=0; $tdee * 0.40" | bc -l)
    local fat_cal=$(echo "scale=0; $tdee * 0.30" | bc -l)

    local protein_g=$(echo "scale=0; $protein_cal / 4" | bc -l)
    local carb_g=$(echo "scale=0; $carb_cal / 4" | bc -l)
    local fat_g=$(echo "scale=0; $fat_cal / 9" | bc -l)

    print_section "Results"
    print_success "Basal Metabolic Rate (BMR): $bmr kcal/day"
    print_success "Total Daily Energy Expenditure (TDEE): $tdee kcal/day"

    print_section "Recommended Macronutrients (Maintenance)"
    print_info "Protein: ${protein_g}g (${protein_cal} kcal, 30%)"
    print_info "Carbohydrates: ${carb_g}g (${carb_cal} kcal, 40%)"
    print_info "Fat: ${fat_g}g (${fat_cal} kcal, 30%)"

    # Calculate protein per kg bodyweight
    local protein_per_kg=$(echo "scale=2; $protein_g / $weight" | bc -l)
    print_info "Protein per kg bodyweight: ${protein_per_kg}g/kg"

    # Weight goals
    print_section "Calorie Targets for Different Goals"
    local loss=$(echo "scale=0; $tdee - 500" | bc -l)
    local gain=$(echo "scale=0; $tdee + 300" | bc -l)
    print_info "Weight Loss (-0.5kg/week): $loss kcal/day"
    print_info "Maintenance: $tdee kcal/day"
    print_info "Weight Gain (+0.25kg/week): $gain kcal/day"

    echo ""
}

# Show usage
usage() {
    print_header
    echo "Usage: wia-ind-007 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-protein [type] [input] [output]  - Calculate protein efficiency"
    echo "  analyze-nutrition [food] [serving]     - Analyze nutritional density"
    echo "  assess-safety [micro] [chem] [phys]    - Assess food safety score"
    echo "  predict-shelf [product] [temp] [days]  - Predict shelf life"
    echo "  optimize-ferment [org] [temp] [ph]     - Optimize fermentation"
    echo "  calc-carbon [product] [lifecycle]      - Calculate carbon footprint"
    echo "  calc-macros [age] [gender] [kg] [cm] [activity] - Personalized nutrition"
    echo "  --version                               - Show version"
    echo "  --help                                  - Show this help"
    echo ""
    echo "Examples:"
    echo "  wia-ind-007 calc-protein cultured-beef 2.5 1.2"
    echo "  wia-ind-007 analyze-nutrition tofu 100"
    echo "  wia-ind-007 assess-safety 95 98 92"
    echo "  wia-ind-007 predict-shelf yogurt 4 14"
    echo "  wia-ind-007 optimize-ferment kombucha 25 3.5"
    echo "  wia-ind-007 calc-carbon plant-burger full"
    echo "  wia-ind-007 calc-macros 35 male 75 175 moderate"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo ""
}

# Main command router
main() {
    if [ $# -eq 0 ]; then
        usage
        exit 0
    fi

    case "$1" in
        calc-protein)
            shift
            calc_protein "$@"
            ;;
        analyze-nutrition)
            shift
            analyze_nutrition "$@"
            ;;
        assess-safety)
            shift
            assess_safety "$@"
            ;;
        predict-shelf)
            shift
            predict_shelf "$@"
            ;;
        optimize-ferment)
            shift
            optimize_ferment "$@"
            ;;
        calc-carbon)
            shift
            calc_carbon "$@"
            ;;
        calc-macros)
            shift
            calc_macros "$@"
            ;;
        --version|-v)
            echo "WIA-IND-007 Food Tech CLI v${VERSION}"
            echo "弘益人間 (Benefit All Humanity)"
            ;;
        --help|-h)
            usage
            ;;
        *)
            print_error "Unknown command: $1"
            echo ""
            usage
            exit 1
            ;;
    esac
}

# Run main
main "$@"
