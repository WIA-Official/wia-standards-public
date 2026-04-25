#!/bin/bash

################################################################################
# WIA-AUTO-021: Vehicle Lightweight Material CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Materials Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to lightweight material calculations
# including weight reduction, material properties, and cost-benefit analysis.
################################################################################

set -e

# Colors for output
ORANGE='\033[38;5;208m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
GRAVITY=9.81
FUEL_EFFICIENCY_COEF=0.65
CO2_PER_LITER_GASOLINE=2392  # grams

# Material database (simplified - density in kg/m³, tensile strength in MPa, cost factor)
declare -A MATERIALS
MATERIALS=(
    ["steel,density"]=7850
    ["steel,strength"]=400
    ["steel,modulus"]=200
    ["steel,cost"]=1.0

    ["hss,density"]=7850
    ["hss,strength"]=550
    ["hss,modulus"]=200
    ["hss,cost"]=1.2

    ["ahss,density"]=7850
    ["ahss,strength"]=780
    ["ahss,modulus"]=200
    ["ahss,cost"]=1.5

    ["uhss,density"]=7850
    ["uhss,strength"]=1500
    ["uhss,modulus"]=200
    ["uhss,cost"]=1.8

    ["aluminum,density"]=2700
    ["aluminum,strength"]=310
    ["aluminum,modulus"]=69
    ["aluminum,cost"]=4.0

    ["al-7075,density"]=2810
    ["al-7075,strength"]=572
    ["al-7075,modulus"]=72
    ["al-7075,cost"]=6.0

    ["magnesium,density"]=1770
    ["magnesium,strength"]=260
    ["magnesium,modulus"]=45
    ["magnesium,cost"]=5.0

    ["cfrp,density"]=1600
    ["cfrp,strength"]=800
    ["cfrp,modulus"]=150
    ["cfrp,cost"]=25.0

    ["gfrp,density"]=2000
    ["gfrp,strength"]=480
    ["gfrp,modulus"]=35
    ["gfrp,cost"]=8.0

    ["titanium,density"]=4430
    ["titanium,strength"]=950
    ["titanium,modulus"]=114
    ["titanium,cost"]=40.0
)

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🚗 WIA-AUTO-021: Vehicle Lightweight Material CLI        ║"
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

get_material_property() {
    local material=$1
    local property=$2
    local key="${material},${property}"
    echo "${MATERIALS[$key]:-0}"
}

# Calculate specific strength
calc_specific_strength() {
    local material=$1
    local density=$(get_material_property "$material" "density")
    local strength=$(get_material_property "$material" "strength")

    if [ "$density" == "0" ]; then
        echo "0"
        return
    fi

    # Specific strength in kN·m/kg = (MPa) / (kg/m³) * 1000
    echo "scale=2; $strength / $density * 1000" | bc -l
}

# Calculate weight reduction
calc_weight() {
    local component=${1:-hood}
    local from_material=${2:-steel}
    local to_material=${3:-aluminum}
    local area=${4:-1.8}  # m²
    local thickness_from=${5:-0.8}  # mm

    print_section "Weight Reduction Calculation"
    print_info "Component: $component"
    print_info "From: $from_material → To: $to_material"
    print_info "Surface area: $area m²"

    # Get material properties
    local density_from=$(get_material_property "$from_material" "density")
    local density_to=$(get_material_property "$to_material" "density")
    local modulus_from=$(get_material_property "$from_material" "modulus")
    local modulus_to=$(get_material_property "$to_material" "modulus")

    if [ "$density_from" == "0" ] || [ "$density_to" == "0" ]; then
        print_error "Material not found in database"
        return 1
    fi

    print_info "Densities: $from_material=$density_from kg/m³, $to_material=$density_to kg/m³"

    # Calculate equivalent thickness for same stiffness
    # For bending: D ∝ E × t³
    # t_new = t_old × (E_old / E_new)^(1/3)
    local thickness_to=$(echo "scale=4; $thickness_from * e(l($modulus_from / $modulus_to) / 3)" | bc -l)

    print_info "Thickness: $from_material=${thickness_from}mm, $to_material=${thickness_to}mm (equivalent stiffness)"

    # Calculate volumes (area × thickness, convert mm to m)
    local volume_from=$(echo "scale=8; $area * $thickness_from / 1000" | bc -l)
    local volume_to=$(echo "scale=8; $area * $thickness_to / 1000" | bc -l)

    # Calculate masses
    local mass_from=$(echo "scale=4; $volume_from * $density_from" | bc -l)
    local mass_to=$(echo "scale=4; $volume_to * $density_to" | bc -l)

    # Weight saved
    local weight_saved=$(echo "scale=4; $mass_from - $mass_to" | bc -l)
    local percent_reduction=$(echo "scale=2; 100 * $weight_saved / $mass_from" | bc -l)

    print_section "Results"
    print_success "Original mass: $(printf "%.2f" $mass_from) kg"
    print_success "New mass: $(printf "%.2f" $mass_to) kg"
    print_success "Weight saved: $(printf "%.2f" $weight_saved) kg ($(printf "%.1f" $percent_reduction)%)"

    # Fuel efficiency impact (assuming 1500 kg vehicle)
    local vehicle_weight=1500
    local weight_reduction_pct=$(echo "scale=6; 100 * $weight_saved / $vehicle_weight" | bc -l)
    local fuel_reduction=$(echo "scale=4; $FUEL_EFFICIENCY_COEF * $weight_reduction_pct" | bc -l)

    print_section "Fuel Efficiency Impact"
    print_info "Vehicle weight: $vehicle_weight kg"
    print_info "Weight reduction: $(printf "%.2f" $weight_reduction_pct)%"
    print_success "Fuel consumption reduction: $(printf "%.2f" $fuel_reduction)%"

    # CO₂ reduction (assuming 8 L/100km baseline)
    local baseline_consumption=8  # L/100km
    local fuel_saved=$(echo "scale=4; $baseline_consumption * $fuel_reduction / 100" | bc -l)
    local co2_per_km=$(echo "scale=2; $fuel_saved * $CO2_PER_LITER_GASOLINE / 100" | bc -l)

    print_info "Fuel saved: $(printf "%.2f" $fuel_saved) L/100km"
    print_info "CO₂ reduction: $(printf "%.1f" $co2_per_km) g/km"

    # Cost comparison
    local cost_from=$(get_material_property "$from_material" "cost")
    local cost_to=$(get_material_property "$to_material" "cost")
    local cost_ratio=$(echo "scale=2; $cost_to / $cost_from" | bc -l)

    print_section "Cost Analysis"
    print_info "Material cost ratio: ${cost_ratio}× (relative to $from_material)"

    if (( $(echo "$cost_ratio < 2.0" | bc -l) )); then
        print_success "Cost impact: MODERATE"
    elif (( $(echo "$cost_ratio < 5.0" | bc -l) )); then
        print_warning "Cost impact: HIGH"
    else
        print_error "Cost impact: VERY HIGH"
    fi

    echo ""
}

# Show material properties
show_material() {
    local material=${1:-aluminum}
    local property=${2:-all}

    print_section "Material Properties: $material"

    # Check if material exists
    local density=$(get_material_property "$material" "density")
    if [ "$density" == "0" ]; then
        print_error "Material '$material' not found in database"
        echo ""
        return 1
    fi

    if [ "$property" == "all" ] || [ "$property" == "mechanical" ]; then
        local strength=$(get_material_property "$material" "strength")
        local modulus=$(get_material_property "$material" "modulus")
        local specific_strength=$(calc_specific_strength "$material")
        local specific_modulus=$(echo "scale=2; $modulus / $density * 1000000" | bc -l)

        print_info "Mechanical Properties:"
        print_success "  Density: $density kg/m³"
        print_success "  Tensile Strength: $strength MPa"
        print_success "  Young's Modulus: $modulus GPa"
        print_success "  Specific Strength: $specific_strength kN·m/kg"
        print_success "  Specific Modulus: $(printf "%.2f" $specific_modulus) MN·m/kg"
    fi

    if [ "$property" == "all" ] || [ "$property" == "cost" ]; then
        local cost=$(get_material_property "$material" "cost")
        print_info "Cost Properties:"
        print_success "  Relative Cost: ${cost}× (vs. mild steel)"
    fi

    # Applications
    if [ "$property" == "all" ]; then
        print_info "Typical Applications:"
        case $material in
            steel)
                print_info "  • Body structure, chassis, general parts"
                ;;
            hss|ahss)
                print_info "  • Reinforcements, door beams, B-pillars"
                ;;
            uhss)
                print_info "  • Safety cage, intrusion beams, hot-stamped parts"
                ;;
            aluminum|al-7075)
                print_info "  • Body panels, hoods, doors, chassis, wheels"
                ;;
            magnesium)
                print_info "  • Instrument panels, seat frames, transmission cases"
                ;;
            cfrp)
                print_info "  • High-performance structures, body panels, monocoque"
                ;;
            gfrp)
                print_info "  • Underbody panels, battery enclosures, interior"
                ;;
            titanium)
                print_info "  • Exhaust systems, valve springs, fasteners"
                ;;
        esac
    fi

    echo ""
}

# Compare materials
compare_materials() {
    local materials=("$@")

    if [ ${#materials[@]} -lt 2 ]; then
        print_error "Please specify at least 2 materials to compare"
        echo ""
        return 1
    fi

    print_section "Material Comparison"

    # Print header
    printf "${CYAN}%-15s${RESET} | ${GRAY}%-10s %-12s %-12s %-18s${RESET}\n" \
        "Material" "Density" "Strength" "Modulus" "Specific Strength"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # Print each material
    for material in "${materials[@]}"; do
        local density=$(get_material_property "$material" "density")
        if [ "$density" == "0" ]; then
            print_warning "Material '$material' not found - skipping"
            continue
        fi

        local strength=$(get_material_property "$material" "strength")
        local modulus=$(get_material_property "$material" "modulus")
        local specific_strength=$(calc_specific_strength "$material")

        printf "%-15s | %7s %9s %9s %15s\n" \
            "$material" \
            "$density kg/m³" \
            "$strength MPa" \
            "$modulus GPa" \
            "$(printf "%.1f" $specific_strength) kN·m/kg"
    done

    echo ""
}

# Crash test simulation (simplified)
crash_test() {
    local material=${1:-aluminum}
    local impact_speed=${2:-50}  # km/h
    local angle=${3:-30}  # degrees

    print_section "Crash Test Simulation"
    print_info "Material: $material"
    print_info "Impact speed: $impact_speed km/h"
    print_info "Impact angle: $angle degrees"

    # Get material properties
    local density=$(get_material_property "$material" "density")
    local strength=$(get_material_property "$material" "strength")

    if [ "$density" == "0" ]; then
        print_error "Material not found"
        return 1
    fi

    # Convert speed to m/s
    local velocity=$(echo "scale=4; $impact_speed / 3.6" | bc -l)

    # Simplified energy calculation (assuming 1500 kg vehicle)
    local mass=1500
    local kinetic_energy=$(echo "scale=2; 0.5 * $mass * $velocity * $velocity / 1000" | bc -l)

    print_info "Kinetic energy: $(printf "%.1f" $kinetic_energy) kJ"

    # Simplified SEA calculation (actual values depend on geometry)
    # Typical SEA values: Steel 20-40, Aluminum 25-50, CFRP 60-100 kJ/kg
    local sea=30
    case $material in
        steel|hss)
            sea=30
            ;;
        ahss|uhss)
            sea=35
            ;;
        aluminum|al-7075)
            sea=40
            ;;
        magnesium)
            sea=35
            ;;
        cfrp)
            sea=80
            ;;
        gfrp)
            sea=45
            ;;
    esac

    print_section "Results"
    print_info "Estimated Specific Energy Absorption: $sea kJ/kg"

    # Check if SEA meets minimum requirement (50 kJ/kg)
    if (( $(echo "$sea >= 50" | bc -l) )); then
        print_success "SEA Check: PASS (meets minimum 50 kJ/kg)"
    else
        print_warning "SEA Check: WARNING (below recommended 50 kJ/kg)"
        print_info "Consider: Structural optimization, hybrid design, or CFRP"
    fi

    # Material-specific recommendations
    print_section "Recommendations"
    case $material in
        steel|hss)
            print_info "• Consider AHSS or UHSS for better energy absorption"
            print_info "• Use progressive crush zones"
            ;;
        ahss|uhss)
            print_success "• Excellent for crash structures"
            print_info "• Use hot-stamping for complex shapes"
            ;;
        aluminum*)
            print_info "• Good energy absorption with lower weight"
            print_info "• Consider extrusions for load-bearing members"
            ;;
        cfrp)
            print_success "• Excellent specific energy absorption"
            print_warning "• Ensure proper design to avoid brittle failure"
            ;;
    esac

    echo ""
}

# Cost-benefit analysis
cost_benefit() {
    local material=${1:-cfrp}
    local quantity=${2:-1000}
    local lifespan=${3:-10}  # years

    print_section "Cost-Benefit Analysis"
    print_info "Material: $material"
    print_info "Production quantity: $quantity units"
    print_info "Vehicle lifespan: $lifespan years"

    # Get cost factor
    local cost=$(get_material_property "$material" "cost")
    if [ "$cost" == "0" ]; then
        print_error "Material not found"
        return 1
    fi

    # Assume baseline steel component costs $100
    local baseline_cost=100
    local material_cost=$(echo "scale=2; $baseline_cost * $cost" | bc -l)

    print_section "Cost Analysis"
    print_info "Material cost per component: \$$(printf "%.2f" $material_cost)"
    print_info "vs. Steel baseline: \$$baseline_cost"

    local additional_cost=$(echo "scale=2; $material_cost - $baseline_cost" | bc -l)
    print_info "Additional cost: \$$(printf "%.2f" $additional_cost) per unit"

    local total_additional=$(echo "scale=2; $additional_cost * $quantity" | bc -l)
    print_info "Total additional investment: \$$(printf "%.0f" $total_additional)"

    # Fuel savings calculation (simplified)
    # Assume 10 kg weight reduction, 8 L/100km, 20,000 km/year, $1.50/L
    local weight_reduction=10  # kg
    local fuel_price=1.50
    local km_per_year=20000
    local baseline_consumption=8

    local weight_pct=$(echo "scale=6; 100 * $weight_reduction / 1500" | bc -l)
    local fuel_reduction_pct=$(echo "scale=4; $FUEL_EFFICIENCY_COEF * $weight_pct" | bc -l)
    local liters_saved_per_year=$(echo "scale=2; $km_per_year * $baseline_consumption * $fuel_reduction_pct / 10000" | bc -l)
    local annual_savings=$(echo "scale=2; $liters_saved_per_year * $fuel_price" | bc -l)
    local lifetime_savings=$(echo "scale=2; $annual_savings * $lifespan" | bc -l)

    print_section "Fuel Savings (per vehicle)"
    print_info "Estimated weight reduction: $weight_reduction kg"
    print_info "Fuel reduction: $(printf "%.2f" $fuel_reduction_pct)%"
    print_info "Liters saved per year: $(printf "%.1f" $liters_saved_per_year) L"
    print_success "Annual fuel cost savings: \$$(printf "%.2f" $annual_savings)"
    print_success "Lifetime savings ($lifespan years): \$$(printf "%.2f" $lifetime_savings)"

    # Payback period
    if (( $(echo "$additional_cost > 0" | bc -l) )); then
        local payback=$(echo "scale=2; $additional_cost / $annual_savings" | bc -l)
        print_section "Payback Analysis"
        print_info "Payback period: $(printf "%.1f" $payback) years"

        if (( $(echo "$payback <= 3" | bc -l) )); then
            print_success "ROI: EXCELLENT (≤ 3 years)"
        elif (( $(echo "$payback <= 5" | bc -l) )); then
            print_success "ROI: GOOD (≤ 5 years)"
        elif (( $(echo "$payback <= $lifespan" | bc -l) )); then
            print_warning "ROI: MODERATE (within vehicle lifetime)"
        else
            print_error "ROI: POOR (exceeds vehicle lifetime)"
        fi
    fi

    echo ""
}

# List available materials
list_materials() {
    print_section "Available Materials"

    echo ""
    echo "${CYAN}Ferrous Materials:${RESET}"
    echo "  • steel      - Mild Steel"
    echo "  • hss        - High Strength Steel"
    echo "  • ahss       - Advanced High Strength Steel"
    echo "  • uhss       - Ultra High Strength Steel"

    echo ""
    echo "${CYAN}Non-Ferrous Metals:${RESET}"
    echo "  • aluminum   - Aluminum 6061-T6"
    echo "  • al-7075    - Aluminum 7075-T6"
    echo "  • magnesium  - Magnesium AZ31B"
    echo "  • titanium   - Titanium Ti-6Al-4V"

    echo ""
    echo "${CYAN}Composites:${RESET}"
    echo "  • cfrp       - Carbon Fiber Reinforced Plastic"
    echo "  • gfrp       - Glass Fiber Reinforced Plastic"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-021 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-weight              Calculate weight reduction"
    echo "    --component <name>     Component name (default: hood)"
    echo "    --from <material>      Original material (default: steel)"
    echo "    --to <material>        New material (default: aluminum)"
    echo "    --area <m²>            Surface area (default: 1.8)"
    echo "    --thickness <mm>       Original thickness (default: 0.8)"
    echo ""
    echo "  material                 Show material properties"
    echo "    --name <material>      Material name"
    echo "    --property <type>      Property type: all, mechanical, cost (default: all)"
    echo ""
    echo "  compare                  Compare multiple materials"
    echo "    <material1> <material2> ... <materialN>"
    echo ""
    echo "  crash-test               Simulate crash test"
    echo "    --material <name>      Material name"
    echo "    --impact-speed <km/h>  Impact speed (default: 50)"
    echo "    --angle <degrees>      Impact angle (default: 30)"
    echo ""
    echo "  cost-benefit             Perform cost-benefit analysis"
    echo "    --material <name>      Material name"
    echo "    --quantity <units>     Production quantity (default: 1000)"
    echo "    --lifespan <years>     Vehicle lifespan (default: 10)"
    echo ""
    echo "  list                     List available materials"
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-021 calc-weight --component hood --from steel --to aluminum"
    echo "  wia-auto-021 material --name cfrp"
    echo "  wia-auto-021 compare steel aluminum cfrp"
    echo "  wia-auto-021 crash-test --material aluminum --impact-speed 60"
    echo "  wia-auto-021 cost-benefit --material cfrp --quantity 10000"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-021 CLI Tool"
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
    calc-weight)
        COMPONENT="hood"
        FROM="steel"
        TO="aluminum"
        AREA=1.8
        THICKNESS=0.8

        while [[ $# -gt 0 ]]; do
            case $1 in
                --component) COMPONENT=$2; shift 2 ;;
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                --thickness) THICKNESS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_weight "$COMPONENT" "$FROM" "$TO" "$AREA" "$THICKNESS"
        ;;

    material)
        MATERIAL="aluminum"
        PROPERTY="all"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --name) MATERIAL=$2; shift 2 ;;
                --property) PROPERTY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        show_material "$MATERIAL" "$PROPERTY"
        ;;

    compare)
        if [ $# -eq 0 ]; then
            print_header
            print_error "Please specify materials to compare"
            echo "Example: wia-auto-021 compare steel aluminum cfrp"
            echo ""
            exit 1
        fi

        print_header
        compare_materials "$@"
        ;;

    crash-test)
        MATERIAL="aluminum"
        SPEED=50
        ANGLE=30

        while [[ $# -gt 0 ]]; do
            case $1 in
                --material) MATERIAL=$2; shift 2 ;;
                --impact-speed) SPEED=$2; shift 2 ;;
                --angle) ANGLE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        crash_test "$MATERIAL" "$SPEED" "$ANGLE"
        ;;

    cost-benefit)
        MATERIAL="cfrp"
        QUANTITY=1000
        LIFESPAN=10

        while [[ $# -gt 0 ]]; do
            case $1 in
                --material) MATERIAL=$2; shift 2 ;;
                --quantity) QUANTITY=$2; shift 2 ;;
                --lifespan) LIFESPAN=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        cost_benefit "$MATERIAL" "$QUANTITY" "$LIFESPAN"
        ;;

    list)
        print_header
        list_materials
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-021 help' for usage information"
        exit 1
        ;;
esac

exit 0
