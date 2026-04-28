#!/bin/bash

################################################################################
# WIA-IND-008: Smart Kitchen Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry 4.0 Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to smart kitchen calculations
# including energy consumption, recipe scaling, inventory management, and
# nutritional analysis.
################################################################################

set -e

# Colors for output (Indigo theme)
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
WATER_SPECIFIC_HEAT=4.186  # kJ/kg·°C
KWH_TO_KJ=3600

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🍳 WIA-IND-008: Smart Kitchen CLI (스마트 주방)          ║"
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
    echo -e "${GRAY}ℹ $1${RESET}"
}

# Usage/Help function
show_usage() {
    print_header
    cat << EOF
${CYAN}USAGE:${RESET}
    wia-ind-008 <command> [options]

${CYAN}COMMANDS:${RESET}
    ${GREEN}calc-energy${RESET}       Calculate cooking energy consumption
    ${GREEN}scale-recipe${RESET}      Scale recipe for different servings
    ${GREEN}calc-nutrition${RESET}    Calculate nutritional information
    ${GREEN}inventory${RESET}         Manage food inventory
    ${GREEN}expiry-check${RESET}      Check for expiring items
    ${GREEN}shopping-list${RESET}     Generate shopping list
    ${GREEN}meal-plan${RESET}         Create meal plan
    ${GREEN}optimize-schedule${RESET} Optimize cooking schedule for energy
    ${GREEN}version${RESET}           Show version information
    ${GREEN}help${RESET}              Show this help message

${CYAN}EXAMPLES:${RESET}
    # Calculate oven energy usage
    wia-ind-008 calc-energy --appliance oven --power 3.5 --time 1.5

    # Scale recipe from 4 to 6 servings
    wia-ind-008 scale-recipe --from 4 --to 6 --ingredients "flour:200g,sugar:100g"

    # Check inventory expiring in 3 days
    wia-ind-008 expiry-check --days 3

    # Calculate meal nutrition
    wia-ind-008 calc-nutrition --calories 450 --protein 25 --carbs 50 --fat 15

${CYAN}PHILOSOPHY:${RESET}
    ${INDIGO}弘益人間 (홍익인간)${RESET} - Benefit All Humanity
    Making healthy, efficient cooking accessible to everyone.

${GRAY}For more information: https://wiastandards.com/standards/smart-kitchen${RESET}

EOF
}

# Calculate cooking energy consumption
calc_energy() {
    local appliance=""
    local power_kw=0
    local time_hours=0
    local efficiency=1.0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --appliance)
                appliance="$2"
                shift 2
                ;;
            --power)
                power_kw="$2"
                shift 2
                ;;
            --time)
                time_hours="$2"
                shift 2
                ;;
            --efficiency)
                efficiency="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    # Validation
    if [[ -z "$appliance" ]]; then
        print_error "Appliance type required (--appliance)"
        return 1
    fi

    # Set default efficiency based on appliance type
    case "$appliance" in
        oven)
            [[ "$efficiency" == "1.0" ]] && efficiency=0.85
            ;;
        induction-cooktop|induction)
            [[ "$efficiency" == "1.0" ]] && efficiency=0.90
            ;;
        gas-cooktop|gas)
            [[ "$efficiency" == "1.0" ]] && efficiency=0.65
            ;;
        microwave)
            [[ "$efficiency" == "1.0" ]] && efficiency=0.75
            ;;
        dishwasher)
            [[ "$efficiency" == "1.0" ]] && efficiency=0.80
            ;;
    esac

    print_section "Energy Calculation for ${appliance}"

    # Calculate energy
    local energy=$(echo "scale=3; $power_kw * $time_hours * $efficiency" | bc)
    local energy_no_eff=$(echo "scale=3; $power_kw * $time_hours" | bc)
    local waste=$(echo "scale=3; $energy_no_eff - $energy" | bc)
    local waste_percent=$(echo "scale=1; ($waste / $energy_no_eff) * 100" | bc)

    # Estimate cost (assuming 0.15 USD/kWh average)
    local cost=$(echo "scale=2; $energy * 0.15" | bc)

    echo "Appliance:           ${appliance}"
    echo "Power Rating:        ${power_kw} kW"
    echo "Operating Time:      ${time_hours} hours"
    echo "Efficiency Factor:   ${efficiency}"
    echo ""
    print_success "Energy Consumed:     ${energy} kWh"
    echo "Theoretical Max:     ${energy_no_eff} kWh"
    echo "Energy Wasted:       ${waste} kWh (${waste_percent}%)"
    echo "Estimated Cost:      \$${cost} USD"
    echo ""

    # Provide context
    print_info "Context: This is equivalent to:"
    local led_hours=$(echo "scale=0; $energy * 1000 / 10" | bc)
    local phone_charges=$(echo "scale=0; $energy * 1000 / 15" | bc)
    echo "  - Running a 10W LED bulb for ${led_hours} hours"
    echo "  - Charging a smartphone ${phone_charges} times"
}

# Scale recipe for different servings
scale_recipe() {
    local from_servings=0
    local to_servings=0
    local ingredients=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --from)
                from_servings="$2"
                shift 2
                ;;
            --to)
                to_servings="$2"
                shift 2
                ;;
            --ingredients)
                ingredients="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    # Validation
    if [[ $from_servings -eq 0 || $to_servings -eq 0 ]]; then
        print_error "Both --from and --to servings required"
        return 1
    fi

    local scale_factor=$(echo "scale=4; $to_servings / $from_servings" | bc)

    print_section "Recipe Scaling"
    echo "Original Servings:   ${from_servings}"
    echo "Target Servings:     ${to_servings}"
    echo "Scaling Factor:      ${scale_factor}x"
    echo ""

    if [[ -n "$ingredients" ]]; then
        print_section "Scaled Ingredients"
        IFS=',' read -ra ITEMS <<< "$ingredients"
        for item in "${ITEMS[@]}"; do
            IFS=':' read -ra PARTS <<< "$item"
            local name="${PARTS[0]}"
            local amount="${PARTS[1]}"

            # Extract number and unit
            local num=$(echo "$amount" | grep -o '[0-9.]*')
            local unit=$(echo "$amount" | sed 's/[0-9.]*//')

            # Scale the amount
            local scaled=$(echo "scale=1; $num * $scale_factor" | bc)

            echo "  ${name}: ${num}${unit} → ${scaled}${unit}"
        done
    fi

    echo ""

    # Cooking time adjustment (cube root rule for volume-based)
    local time_factor=$(echo "scale=3; e(l($scale_factor)/3)" | bc -l)
    print_section "Time Adjustment Estimate"
    echo "Volume-based (baking/roasting): ${time_factor}x of original time"
    print_info "Note: Surface-area cooking (pan-fry) uses different scaling"
}

# Calculate nutritional information
calc_nutrition() {
    local calories=0
    local protein=0
    local carbs=0
    local fat=0
    local servings=1

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --calories) calories="$2"; shift 2 ;;
            --protein) protein="$2"; shift 2 ;;
            --carbs) carbs="$2"; shift 2 ;;
            --fat) fat="$2"; shift 2 ;;
            --servings) servings="$2"; shift 2 ;;
            *) print_error "Unknown option: $1"; return 1 ;;
        esac
    done

    print_section "Nutritional Analysis"

    # Calculate calories from macros
    local cal_from_protein=$(echo "scale=0; $protein * 4" | bc)
    local cal_from_carbs=$(echo "scale=0; $carbs * 4" | bc)
    local cal_from_fat=$(echo "scale=0; $fat * 9" | bc)
    local total_cal=$(echo "$cal_from_protein + $cal_from_carbs + $cal_from_fat" | bc)

    # Calculate percentages
    local protein_pct=$(echo "scale=1; ($cal_from_protein / $total_cal) * 100" | bc)
    local carbs_pct=$(echo "scale=1; ($cal_from_carbs / $total_cal) * 100" | bc)
    local fat_pct=$(echo "scale=1; ($cal_from_fat / $total_cal) * 100" | bc)

    echo "Per Serving:"
    echo "  Calories:    ${calories} kcal"
    echo "  Protein:     ${protein}g (${cal_from_protein} kcal, ${protein_pct}%)"
    echo "  Carbs:       ${carbs}g (${cal_from_carbs} kcal, ${carbs_pct}%)"
    echo "  Fat:         ${fat}g (${cal_from_fat} kcal, ${fat_pct}%)"
    echo ""
    echo "Total Servings: ${servings}"

    local total_calories=$(echo "$calories * $servings" | bc)
    echo "Total Calories: ${total_calories} kcal"
    echo ""

    # Macro ratio classification
    print_section "Macro Ratio Analysis"
    if (( $(echo "$protein_pct >= 30" | bc -l) )); then
        echo "  ✓ High-protein meal"
    fi
    if (( $(echo "$carbs_pct <= 20" | bc -l) )); then
        echo "  ✓ Low-carb / Keto-friendly"
    fi
    if (( $(echo "$fat_pct >= 60" | bc -l) )); then
        echo "  ✓ High-fat / Ketogenic"
    fi
    if (( $(echo "$carbs_pct >= 45 && $carbs_pct <= 65" | bc -l) )); then
        echo "  ✓ Balanced carbohydrate"
    fi
}

# Check expiring inventory items
expiry_check() {
    local days=7
    local location="all"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --days) days="$2"; shift 2 ;;
            --location) location="$2"; shift 2 ;;
            *) print_error "Unknown option: $1"; return 1 ;;
        esac
    done

    print_section "Expiring Items Check (${days} days)"

    # This is a demo - in real implementation, would query database
    print_warning "Items expiring within ${days} days:"
    echo ""
    echo "  Location: Refrigerator"
    echo "    • Milk (2L) - expires in 2 days"
    echo "    • Lettuce (1 head) - expires in 3 days"
    echo "    • Chicken breast (500g) - expires in 1 day"
    echo ""
    echo "  Location: Pantry"
    echo "    • Bread (1 loaf) - expires in 4 days"
    echo ""

    print_section "Recommendations"
    echo "  1. Use chicken breast for tonight's dinner"
    echo "  2. Make smoothies with milk before expiration"
    echo "  3. Prepare salad with lettuce"
    echo "  4. Freeze bread if not using soon"
}

# Generate shopping list
generate_shopping_list() {
    local meals=7
    local people=2

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --meals) meals="$2"; shift 2 ;;
            --people) people="$2"; shift 2 ;;
            *) print_error "Unknown option: $1"; return 1 ;;
        esac
    done

    print_section "Shopping List Generator"
    echo "Meal Plan Duration: ${meals} days"
    echo "Number of People:   ${people}"
    echo ""

    # Demo shopping list
    print_section "📦 Produce"
    echo "  ☐ Lettuce - 2 heads"
    echo "  ☐ Tomatoes - 1 kg"
    echo "  ☐ Onions - 500g"
    echo "  ☐ Garlic - 1 bulb"
    echo "  ☐ Carrots - 500g"
    echo ""

    print_section "🥩 Protein"
    echo "  ☐ Chicken breast - 1 kg"
    echo "  ☐ Ground beef - 500g"
    echo "  ☐ Eggs - 1 dozen"
    echo "  ☐ Tofu - 400g"
    echo ""

    print_section "🥛 Dairy"
    echo "  ☐ Milk - 2L"
    echo "  ☐ Cheese - 200g"
    echo "  ☐ Yogurt - 500g"
    echo ""

    print_section "🌾 Grains & Pantry"
    echo "  ☐ Rice - 2kg"
    echo "  ☐ Pasta - 500g"
    echo "  ☐ Bread - 1 loaf"
    echo "  ☐ Cooking oil - 500ml"
    echo ""

    local estimated_cost=$(echo "$meals * $people * 8" | bc)
    print_info "Estimated total: \$${estimated_cost} USD"
}

# Optimize cooking schedule
optimize_schedule() {
    local meal_plan="default"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --week-plan) meal_plan="$2"; shift 2 ;;
            *) print_error "Unknown option: $1"; return 1 ;;
        esac
    done

    print_section "Energy-Optimized Cooking Schedule"

    echo "Time-of-Use Pricing:"
    echo "  Off-Peak:  23:00-07:00 (\$0.08/kWh)"
    echo "  Mid-Peak:  07:00-17:00 (\$0.12/kWh)"
    echo "  Peak:      17:00-23:00 (\$0.18/kWh)"
    echo ""

    print_section "Recommended Schedule"
    echo ""
    echo "Monday:"
    echo "  08:00 - Slow cooker beef stew (6h, off-peak start)"
    echo "  18:30 - Quick stir-fry (15min, minimize peak usage)"
    echo ""
    echo "Tuesday:"
    echo "  07:00 - Rice cooker for day (mid-peak start)"
    echo "  23:00 - Dishwasher eco cycle (off-peak)"
    echo ""
    echo "Sunday (Batch Cooking Day):"
    echo "  09:00 - Oven roast chicken × 2 (2h)"
    echo "  09:00 - Rice cooker large batch"
    echo "  10:00 - Steam vegetables (20min)"
    echo ""

    print_section "Energy Savings"
    local normal_cost=45
    local optimized_cost=32
    local savings=$(echo "$normal_cost - $optimized_cost" | bc)
    local savings_pct=$(echo "scale=1; ($savings / $normal_cost) * 100" | bc)

    echo "Normal schedule:     \$${normal_cost}/week"
    echo "Optimized schedule:  \$${optimized_cost}/week"
    print_success "Savings:             \$${savings}/week (${savings_pct}%)"
}

# Meal planning
create_meal_plan() {
    local days=7
    local people=2
    local diet="balanced"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --days) days="$2"; shift 2 ;;
            --people) people="$2"; shift 2 ;;
            --diet) diet="$2"; shift 2 ;;
            *) print_error "Unknown option: $1"; return 1 ;;
        esac
    done

    print_section "Meal Plan (${days} days, ${people} people)"
    echo "Dietary Preference: ${diet}"
    echo ""

    # Sample meal plan
    echo "Monday:"
    echo "  Breakfast: Oatmeal with berries (300 kcal)"
    echo "  Lunch:     Grilled chicken salad (450 kcal)"
    echo "  Dinner:    Kimchi jjigae with rice (550 kcal)"
    echo ""
    echo "Tuesday:"
    echo "  Breakfast: Scrambled eggs & toast (350 kcal)"
    echo "  Lunch:     Pasta carbonara (520 kcal)"
    echo "  Dinner:    Teriyaki salmon with vegetables (480 kcal)"
    echo ""
    echo "..."
    echo ""

    local daily_cal=$(echo "1300 * $people" | bc)
    local weekly_cal=$(echo "$daily_cal * $days" | bc)

    print_section "Nutritional Summary"
    echo "Daily calories:  ${daily_cal} kcal (avg per person: 1300)"
    echo "Weekly total:    ${weekly_cal} kcal"
    echo "Macro balance:   Protein 25%, Carbs 50%, Fat 25%"
}

# Version information
show_version() {
    print_header
    echo "${INDIGO}WIA-IND-008: Smart Kitchen Standard${RESET}"
    echo "Version: ${VERSION}"
    echo "License: MIT"
    echo "Author:  WIA Industry 4.0 Research Group"
    echo ""
    echo "${INDIGO}弘益人間 (홍익인간)${RESET} - Benefit All Humanity"
    echo ""
    echo "Website: https://wiastandards.com"
    echo "GitHub:  https://github.com/WIA-Official/wia-standards"
}

# Main command dispatcher
main() {
    if [[ $# -eq 0 ]]; then
        show_usage
        exit 0
    fi

    local command=$1
    shift

    case "$command" in
        calc-energy)
            calc_energy "$@"
            ;;
        scale-recipe)
            scale_recipe "$@"
            ;;
        calc-nutrition)
            calc_nutrition "$@"
            ;;
        expiry-check)
            expiry_check "$@"
            ;;
        inventory)
            print_info "Inventory management - Feature in development"
            ;;
        shopping-list)
            generate_shopping_list "$@"
            ;;
        meal-plan)
            create_meal_plan "$@"
            ;;
        optimize-schedule)
            optimize_schedule "$@"
            ;;
        version|--version|-v)
            show_version
            ;;
        help|--help|-h)
            show_usage
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
