#!/bin/bash

################################################################################
# WIA-COMM-013: Data Center CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Data Center Infrastructure Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to data center calculations
# including PUE, tier validation, cooling, power, and cost estimation.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
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
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🏢 WIA-COMM-013: Data Center CLI                      ║"
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

# Calculate PUE
calc_pue() {
    local it_load=${1:-1000}
    local cooling=${2:-150}
    local lighting=${3:-20}
    local ups_losses=${4:-50}

    print_section "Power Usage Effectiveness (PUE) Calculation"
    print_info "IT Load: $it_load kW"
    print_info "Cooling: $cooling kW"
    print_info "Lighting: $lighting kW"
    print_info "UPS Losses: $ups_losses kW"

    # Calculate total facility power
    local total=$(echo "$it_load + $cooling + $lighting + $ups_losses" | bc -l)

    # Calculate PUE
    local pue=$(echo "scale=3; $total / $it_load" | bc -l)

    # Calculate DCiE (Data Center infrastructure Efficiency)
    local dcie=$(echo "scale=1; 100 / $pue" | bc -l)

    print_section "Results"
    print_success "Total Facility Power: $(printf "%.1f" $total) kW"
    print_success "PUE: $(printf "%.3f" $pue)"
    print_success "DCiE: $(printf "%.1f" $dcie)%"

    # Efficiency rating
    if (( $(echo "$pue < 1.2" | bc -l) )); then
        print_success "Rating: EXCELLENT (Hyperscale-level efficiency)"
    elif (( $(echo "$pue < 1.5" | bc -l) )); then
        print_success "Rating: GOOD (Modern efficient design)"
    elif (( $(echo "$pue < 1.8" | bc -l) )); then
        print_warning "Rating: FAIR (Room for improvement)"
    else
        print_error "Rating: POOR (Significant optimization needed)"
    fi

    # Recommendations
    print_section "Recommendations"
    if (( $(echo "$pue >= 1.5" | bc -l) )); then
        print_info "• Consider hot/cold aisle containment"
        print_info "• Optimize cooling setpoints (raise inlet temp to 24°C)"
        print_info "• Implement free cooling/economizers"
        print_info "• Upgrade to high-efficiency UPS (>96%)"
    fi

    echo ""
}

# Validate tier compliance
validate_tier() {
    local tier=${1:-III}
    local power_paths=${2:-2}
    local redundancy=${3:-N+1}
    local concurrent_maint=${4:-yes}
    local fault_tolerant=${5:-no}

    print_section "Tier $tier Compliance Validation"
    print_info "Power Paths: $power_paths"
    print_info "Redundancy: $redundancy"
    print_info "Concurrent Maintenance: $concurrent_maint"
    print_info "Fault Tolerant: $fault_tolerant"

    local compliant=true
    local checks=0
    local passed=0

    print_section "Validation Checks"

    # Tier I requirements
    if [ "$tier" = "I" ]; then
        checks=$((checks + 1))
        if [ $power_paths -ge 1 ]; then
            print_success "Power paths: PASS (≥1)"
            passed=$((passed + 1))
        else
            print_error "Power paths: FAIL (requires ≥1)"
            compliant=false
        fi

        print_info "Expected Availability: 99.671%"
        print_info "Expected Downtime: 28.8 hours/year"
    fi

    # Tier II requirements
    if [ "$tier" = "II" ]; then
        checks=$((checks + 2))
        if [ $power_paths -ge 1 ]; then
            print_success "Power paths: PASS (≥1)"
            passed=$((passed + 1))
        else
            print_error "Power paths: FAIL (requires ≥1)"
            compliant=false
        fi

        if [ "$redundancy" = "N+1" ] || [ "$redundancy" = "2N" ] || [ "$redundancy" = "2N+1" ]; then
            print_success "Redundancy: PASS ($redundancy)"
            passed=$((passed + 1))
        else
            print_error "Redundancy: FAIL (requires N+1 minimum)"
            compliant=false
        fi

        print_info "Expected Availability: 99.741%"
        print_info "Expected Downtime: 22.0 hours/year"
    fi

    # Tier III requirements
    if [ "$tier" = "III" ]; then
        checks=$((checks + 3))
        if [ $power_paths -ge 2 ]; then
            print_success "Power paths: PASS (≥2)"
            passed=$((passed + 1))
        else
            print_error "Power paths: FAIL (requires ≥2)"
            compliant=false
        fi

        if [ "$redundancy" = "N+1" ] || [ "$redundancy" = "2N" ] || [ "$redundancy" = "2N+1" ]; then
            print_success "Redundancy: PASS ($redundancy)"
            passed=$((passed + 1))
        else
            print_error "Redundancy: FAIL (requires N+1 minimum)"
            compliant=false
        fi

        if [ "$concurrent_maint" = "yes" ] || [ "$concurrent_maint" = "true" ]; then
            print_success "Concurrent Maintenance: PASS"
            passed=$((passed + 1))
        else
            print_error "Concurrent Maintenance: FAIL (required)"
            compliant=false
        fi

        print_info "Expected Availability: 99.982%"
        print_info "Expected Downtime: 1.6 hours/year"
    fi

    # Tier IV requirements
    if [ "$tier" = "IV" ]; then
        checks=$((checks + 4))
        if [ $power_paths -ge 2 ]; then
            print_success "Power paths: PASS (≥2 active)"
            passed=$((passed + 1))
        else
            print_error "Power paths: FAIL (requires ≥2 active)"
            compliant=false
        fi

        if [ "$redundancy" = "2N" ] || [ "$redundancy" = "2N+1" ]; then
            print_success "Redundancy: PASS ($redundancy)"
            passed=$((passed + 1))
        else
            print_error "Redundancy: FAIL (requires 2N or 2N+1)"
            compliant=false
        fi

        if [ "$concurrent_maint" = "yes" ] || [ "$concurrent_maint" = "true" ]; then
            print_success "Concurrent Maintenance: PASS"
            passed=$((passed + 1))
        else
            print_error "Concurrent Maintenance: FAIL (required)"
            compliant=false
        fi

        if [ "$fault_tolerant" = "yes" ] || [ "$fault_tolerant" = "true" ]; then
            print_success "Fault Tolerant: PASS"
            passed=$((passed + 1))
        else
            print_error "Fault Tolerant: FAIL (required)"
            compliant=false
        fi

        print_info "Expected Availability: 99.995%"
        print_info "Expected Downtime: 0.4 hours/year (26.3 minutes)"
    fi

    print_section "Summary"
    print_info "Checks Passed: $passed/$checks"
    if [ "$compliant" = true ]; then
        print_success "Status: COMPLIANT with Tier $tier"
    else
        print_error "Status: NON-COMPLIANT with Tier $tier"
    fi

    echo ""
}

# Design rack layout
design_rack() {
    local racks=${1:-40}
    local density=${2:-10}
    local floor_area=${3:-10000}

    print_section "Rack Layout Design"
    print_info "Number of Racks: $racks"
    print_info "Average Power per Rack: $density kW"
    print_info "Floor Area: $floor_area sq ft"

    # Calculate total IT load
    local total_it=$(echo "$racks * $density" | bc -l)

    # Calculate optimal layout (rows x racks per row)
    local rows=$(echo "sqrt($racks / 2)" | bc -l | awk '{print int($1+0.5)}')
    local racks_per_row=$(echo "$racks / $rows" | bc -l | awk '{print int($1+0.5)}')

    # Rack footprint
    local rack_width=2
    local rack_depth=4
    local rack_footprint=$(echo "$rack_width * $rack_depth" | bc -l)

    # Aisle dimensions
    local cold_aisle=4
    local hot_aisle=5

    # Calculate total area needed
    local total_rack_area=$(echo "$racks * $rack_footprint" | bc -l)
    local utilization=$(echo "scale=1; $total_rack_area / $floor_area * 100" | bc -l)

    print_section "Layout Configuration"
    print_success "Layout: $rows rows × $racks_per_row racks/row"
    print_info "Rack Footprint: ${rack_width}ft × ${rack_depth}ft = $rack_footprint sq ft"
    print_info "Cold Aisle Width: $cold_aisle ft"
    print_info "Hot Aisle Width: $hot_aisle ft"

    print_section "Capacity Analysis"
    print_success "Total IT Load: $(printf "%.0f" $total_it) kW"
    print_info "Total Rack Area: $(printf "%.0f" $total_rack_area) sq ft"
    print_info "Floor Utilization: $(printf "%.1f" $utilization)%"

    # Power density
    local power_density=$(echo "$total_it / ($floor_area / 1000)" | bc -l)
    print_info "Power Density: $(printf "%.1f" $power_density) kW/1000 sq ft"

    print_section "Recommendations"
    if (( $(echo "$density <= 5" | bc -l) )); then
        print_info "• Low density: Standard air cooling sufficient"
    elif (( $(echo "$density <= 15" | bc -l) )); then
        print_info "• Medium density: Hot/cold aisle containment recommended"
    elif (( $(echo "$density <= 25" | bc -l) )); then
        print_info "• High density: In-row cooling required"
    else
        print_info "• Very high density: Consider liquid cooling"
    fi

    echo ""
}

# Calculate cooling capacity
calc_cooling() {
    local it_load=${1:-1000}
    local ambient_temp=${2:-25}
    local target_pue=${3:-1.2}

    print_section "Cooling Capacity Calculation"
    print_info "IT Load: $it_load kW"
    print_info "Ambient Temperature: ${ambient_temp}°C"
    print_info "Target PUE: $target_pue"

    # Calculate cooling load
    local cooling_load=$(echo "($target_pue - 1) * $it_load" | bc -l)

    # Convert to tons (1 ton = 3.517 kW)
    local cooling_tons=$(echo "$cooling_load / 3.517" | bc -l)

    # Calculate CFM requirement (180 CFM/kW is typical)
    local cfm=$(echo "$it_load * 180" | bc -l)

    print_section "Cooling Requirements"
    print_success "Cooling Load: $(printf "%.0f" $cooling_load) kW"
    print_success "Cooling Capacity: $(printf "%.0f" $cooling_tons) tons"
    print_info "Airflow Required: $(printf "%.0f" $cfm) CFM"

    # CRAC/CRAH units needed (assume 30 tons per unit)
    local units=$(echo "$cooling_tons / 30" | bc -l)
    local units_n1=$(echo "$units + 1" | bc -l | awk '{print int($1+0.5)}')

    print_section "Equipment Sizing"
    print_info "CRAC/CRAH Units (N): $(printf "%.1f" $units) units"
    print_success "With N+1 Redundancy: $units_n1 units × 30 tons"

    # Temperature setpoints
    print_section "Recommended Setpoints"
    print_info "Cold Aisle Inlet: 20-24°C (68-75°F)"
    print_info "Hot Aisle Exhaust: <38°C (100°F)"
    print_info "Humidity: 45-55% RH"

    echo ""
}

# Estimate costs
estimate_cost() {
    local it_capacity=${1:-1000}
    local tier=${2:-III}

    print_section "Data Center Cost Estimation"
    print_info "IT Capacity: $it_capacity kW"
    print_info "Tier Level: $tier"

    # Cost per kW varies by tier
    local cost_per_kw
    case $tier in
        I) cost_per_kw=2000 ;;
        II) cost_per_kw=3000 ;;
        III) cost_per_kw=4000 ;;
        IV) cost_per_kw=5000 ;;
        *) cost_per_kw=3500 ;;
    esac

    # CapEx calculation
    local total_capex=$(echo "$it_capacity * $cost_per_kw" | bc -l)
    local electrical=$(echo "$total_capex * 0.33" | bc -l)
    local mechanical=$(echo "$total_capex * 0.23" | bc -l)
    local construction=$(echo "$total_capex * 0.35" | bc -l)
    local network=$(echo "$total_capex * 0.06" | bc -l)
    local security=$(echo "$total_capex * 0.03" | bc -l)

    print_section "CapEx (Capital Expenditure)"
    print_success "Total CapEx: \$$(printf "%'.0f" $total_capex)"
    print_info "  Electrical (33%): \$$(printf "%'.0f" $electrical)"
    print_info "  Mechanical (23%): \$$(printf "%'.0f" $mechanical)"
    print_info "  Construction (35%): \$$(printf "%'.0f" $construction)"
    print_info "  Network (6%): \$$(printf "%'.0f" $network)"
    print_info "  Security (3%): \$$(printf "%'.0f" $security)"

    # OpEx calculation (annual)
    local opex_per_kw=1200
    local total_opex=$(echo "$it_capacity * $opex_per_kw" | bc -l)
    local electricity=$(echo "$total_opex * 0.70" | bc -l)
    local staffing=$(echo "$total_opex * 0.20" | bc -l)
    local maintenance=$(echo "$total_opex * 0.10" | bc -l)

    print_section "OpEx (Operating Expenditure - Annual)"
    print_success "Total Annual OpEx: \$$(printf "%'.0f" $total_opex)"
    print_info "  Electricity (70%): \$$(printf "%'.0f" $electricity)"
    print_info "  Staffing (20%): \$$(printf "%'.0f" $staffing)"
    print_info "  Maintenance (10%): \$$(printf "%'.0f" $maintenance)"

    # 5-year TCO
    local tco_5yr=$(echo "$total_capex + ($total_opex * 5)" | bc -l)

    print_section "Total Cost of Ownership"
    print_success "5-Year TCO: \$$(printf "%'.0f" $tco_5yr)"
    print_info "Cost per kW (CapEx): \$$(printf "%'.0f" $cost_per_kw)"
    print_info "Cost per kW/year (OpEx): \$$(printf "%'.0f" $opex_per_kw)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-013 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-pue                 Calculate Power Usage Effectiveness"
    echo "    --it-load <kW>         IT equipment load (default: 1000)"
    echo "    --cooling <kW>         Cooling load (default: 150)"
    echo "    --lighting <kW>        Lighting load (default: 20)"
    echo "    --ups-losses <kW>      UPS losses (default: 50)"
    echo ""
    echo "  validate-tier            Validate tier compliance"
    echo "    --tier <I|II|III|IV>   Tier level (default: III)"
    echo "    --power-paths <n>      Number of power paths (default: 2)"
    echo "    --redundancy <type>    N, N+1, 2N, 2N+1 (default: N+1)"
    echo "    --concurrent-maint     Concurrent maintenance (yes/no)"
    echo "    --fault-tolerant       Fault tolerant (yes/no)"
    echo ""
    echo "  design-rack              Design rack layout"
    echo "    --racks <n>            Number of racks (default: 40)"
    echo "    --density <kW>         Power per rack (default: 10)"
    echo "    --floor-area <sqft>    Floor area (default: 10000)"
    echo ""
    echo "  calc-cooling             Calculate cooling capacity"
    echo "    --it-load <kW>         IT load (default: 1000)"
    echo "    --ambient-temp <°C>    Ambient temperature (default: 25)"
    echo "    --target-pue <n>       Target PUE (default: 1.2)"
    echo ""
    echo "  estimate-cost            Estimate data center costs"
    echo "    --capacity <kW>        IT capacity (default: 1000)"
    echo "    --tier <I|II|III|IV>   Tier level (default: III)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-013 calc-pue --it-load 1000 --cooling 150"
    echo "  wia-comm-013 validate-tier --tier III --power-paths 2 --redundancy N+1"
    echo "  wia-comm-013 design-rack --racks 40 --density 10"
    echo "  wia-comm-013 estimate-cost --capacity 1000 --tier III"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-013 Data Center CLI Tool"
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
    calc-pue)
        IT_LOAD=1000
        COOLING=150
        LIGHTING=20
        UPS_LOSSES=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --it-load) IT_LOAD=$2; shift 2 ;;
                --cooling) COOLING=$2; shift 2 ;;
                --lighting) LIGHTING=$2; shift 2 ;;
                --ups-losses) UPS_LOSSES=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_pue "$IT_LOAD" "$COOLING" "$LIGHTING" "$UPS_LOSSES"
        ;;

    validate-tier)
        TIER="III"
        POWER_PATHS=2
        REDUNDANCY="N+1"
        CONCURRENT="yes"
        FAULT_TOL="no"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --tier) TIER=$2; shift 2 ;;
                --power-paths) POWER_PATHS=$2; shift 2 ;;
                --redundancy) REDUNDANCY=$2; shift 2 ;;
                --concurrent-maint) CONCURRENT=$2; shift 2 ;;
                --fault-tolerant) FAULT_TOL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_tier "$TIER" "$POWER_PATHS" "$REDUNDANCY" "$CONCURRENT" "$FAULT_TOL"
        ;;

    design-rack)
        RACKS=40
        DENSITY=10
        FLOOR_AREA=10000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --racks) RACKS=$2; shift 2 ;;
                --density) DENSITY=$2; shift 2 ;;
                --floor-area) FLOOR_AREA=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        design_rack "$RACKS" "$DENSITY" "$FLOOR_AREA"
        ;;

    calc-cooling)
        IT_LOAD=1000
        AMBIENT=25
        TARGET_PUE=1.2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --it-load) IT_LOAD=$2; shift 2 ;;
                --ambient-temp) AMBIENT=$2; shift 2 ;;
                --target-pue) TARGET_PUE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_cooling "$IT_LOAD" "$AMBIENT" "$TARGET_PUE"
        ;;

    estimate-cost)
        CAPACITY=1000
        TIER="III"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --capacity) CAPACITY=$2; shift 2 ;;
                --tier) TIER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        estimate_cost "$CAPACITY" "$TIER"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-013 help' for usage information"
        exit 1
        ;;
esac

exit 0
