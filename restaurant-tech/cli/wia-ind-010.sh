#!/bin/bash

################################################################################
# WIA-IND-010: Restaurant Tech Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to restaurant technology calculations
# including POS totals, table turnover, RevPASH, labor costs, food costs, and more.
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
    echo "║         🍴 WIA-IND-010: Restaurant Tech CLI                   ║"
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

# POS: Calculate order total
calc_order_total() {
    local subtotal=${1:-45.50}
    local tax_rate=${2:-8}
    local tip_percent=${3:-18}
    local discount=${4:-0}

    print_section "Order Total Calculation"
    print_info "Subtotal: \$$subtotal"
    print_info "Tax Rate: $tax_rate%"
    print_info "Tip: $tip_percent%"
    print_info "Discount: \$$discount"

    # Calculate discount amount
    local discount_amount=$(echo "scale=2; $discount" | bc -l)
    local taxable_amount=$(echo "scale=2; $subtotal - $discount_amount" | bc -l)

    # Calculate tax
    local tax=$(echo "scale=2; $taxable_amount * $tax_rate / 100" | bc -l)

    # Calculate tip (on subtotal before discount)
    local tip=$(echo "scale=2; $subtotal * $tip_percent / 100" | bc -l)

    # Calculate total
    local total=$(echo "scale=2; $taxable_amount + $tax + $tip" | bc -l)

    print_section "Results"
    print_success "Subtotal: \$$subtotal"
    print_info "Discount: -\$$discount_amount"
    print_info "Taxable Amount: \$$taxable_amount"
    print_info "Tax ($tax_rate%): \$$tax"
    print_info "Tip ($tip_percent%): \$$tip"
    print_success "Total: \$$total"

    echo ""
}

# Calculate table turnover rate
calc_turnover() {
    local parties=${1:-85}
    local tables=${2:-25}
    local hours=${3:-8}

    print_section "Table Turnover Calculation"
    print_info "Parties Served: $parties"
    print_info "Number of Tables: $tables"
    print_info "Hours Open: $hours"

    # Turnover = Parties / (Tables × Hours)
    local turnover=$(echo "scale=3; $parties / ($tables * $hours)" | bc -l)

    # Average time per party
    local avg_time=$(echo "scale=0; 60 / $turnover" | bc -l)

    print_section "Results"
    print_success "Turnover Rate: $turnover parties/table/hour"
    print_info "Average Table Time: $avg_time minutes per party"

    # Performance assessment
    if (( $(echo "$turnover > 0.5" | bc -l) )); then
        print_success "Excellent turnover (> 0.5)"
    elif (( $(echo "$turnover > 0.3" | bc -l) )); then
        print_warning "Good turnover (0.3-0.5)"
    else
        print_warning "Low turnover (< 0.3) - consider optimization"
    fi

    echo ""
}

# Calculate Revenue Per Available Seat Hour (RevPASH)
calc_revpash() {
    local revenue=${1:-12500}
    local seats=${2:-100}
    local hours=${3:-8}

    print_section "RevPASH Calculation"
    print_info "Total Revenue: \$$revenue"
    print_info "Number of Seats: $seats"
    print_info "Hours Open: $hours"

    # RevPASH = Revenue / (Seats × Hours)
    local revpash=$(echo "scale=2; $revenue / ($seats * $hours)" | bc -l)

    print_section "Results"
    print_success "RevPASH: \$$revpash per seat per hour"

    # Benchmark comparison
    if (( $(echo "$revpash > 20" | bc -l) )); then
        print_success "Excellent RevPASH (> \$20)"
    elif (( $(echo "$revpash > 15" | bc -l) )); then
        print_success "Good RevPASH (\$15-20)"
    elif (( $(echo "$revpash > 10" | bc -l) )); then
        print_warning "Average RevPASH (\$10-15)"
    else
        print_warning "Low RevPASH (< \$10) - pricing/efficiency review needed"
    fi

    echo ""
}

# Calculate food cost percentage
calc_food_cost() {
    local cogs=${1:-3500}
    local revenue=${2:-10000}

    print_section "Food Cost Percentage"
    print_info "Cost of Goods Sold: \$$cogs"
    print_info "Food Sales Revenue: \$$revenue"

    # Food Cost % = (COGS / Revenue) × 100
    local food_cost_pct=$(echo "scale=2; ($cogs / $revenue) * 100" | bc -l)
    local profit=$(echo "scale=2; $revenue - $cogs" | bc -l)
    local profit_margin=$(echo "scale=2; ($profit / $revenue) * 100" | bc -l)

    print_section "Results"
    print_success "Food Cost: $food_cost_pct%"
    print_info "Gross Profit: \$$profit"
    print_info "Profit Margin: $profit_margin%"

    # Target range: 28-35%
    if (( $(echo "$food_cost_pct < 28" | bc -l) )); then
        print_success "Excellent food cost (< 28%)"
    elif (( $(echo "$food_cost_pct < 35" | bc -l) )); then
        print_success "Target range (28-35%)"
    elif (( $(echo "$food_cost_pct < 40" | bc -l) )); then
        print_warning "Above target (35-40%) - review pricing/portions"
    else
        print_error "Too high (> 40%) - immediate action needed"
    fi

    echo ""
}

# Calculate labor cost percentage
calc_labor_cost() {
    local labor=${1:-7000}
    local revenue=${2:-25000}

    print_section "Labor Cost Percentage"
    print_info "Total Labor Cost: \$$labor"
    print_info "Total Revenue: \$$revenue"

    # Labor Cost % = (Labor / Revenue) × 100
    local labor_pct=$(echo "scale=2; ($labor / $revenue) * 100" | bc -l)

    print_section "Results"
    print_success "Labor Cost: $labor_pct%"

    # Target range: 25-35%
    if (( $(echo "$labor_pct < 25" | bc -l) )); then
        print_success "Excellent labor cost (< 25%)"
    elif (( $(echo "$labor_pct < 35" | bc -l) )); then
        print_success "Target range (25-35%)"
    elif (( $(echo "$labor_pct < 40" | bc -l) )); then
        print_warning "Above target (35-40%) - review scheduling"
    else
        print_error "Too high (> 40%) - optimization needed"
    fi

    echo ""
}

# Calculate prime cost (food + labor)
calc_prime_cost() {
    local food_cost=${1:-3500}
    local labor_cost=${2:-7000}
    local revenue=${3:-25000}

    print_section "Prime Cost Calculation"
    print_info "Food Cost: \$$food_cost"
    print_info "Labor Cost: \$$labor_cost"
    print_info "Total Revenue: \$$revenue"

    # Prime Cost = Food Cost + Labor Cost
    local prime_cost=$(echo "scale=2; $food_cost + $labor_cost" | bc -l)
    local prime_cost_pct=$(echo "scale=2; ($prime_cost / $revenue) * 100" | bc -l)

    # Food cost %
    local food_pct=$(echo "scale=2; ($food_cost / $revenue) * 100" | bc -l)

    # Labor cost %
    local labor_pct=$(echo "scale=2; ($labor_cost / $revenue) * 100" | bc -l)

    print_section "Results"
    print_info "Food Cost: $food_pct%"
    print_info "Labor Cost: $labor_pct%"
    print_success "Prime Cost: \$$prime_cost ($prime_cost_pct%)"

    # Target prime cost: < 60%
    if (( $(echo "$prime_cost_pct < 55" | bc -l) )); then
        print_success "Excellent prime cost (< 55%)"
    elif (( $(echo "$prime_cost_pct < 60" | bc -l) )); then
        print_success "Good prime cost (55-60%)"
    elif (( $(echo "$prime_cost_pct < 65" | bc -l) )); then
        print_warning "Above target (60-65%) - review costs"
    else
        print_error "Too high (> 65%) - profitability at risk"
    fi

    echo ""
}

# Calculate menu item profitability
calc_menu_profit() {
    local cost=${1:-4.50}
    local price=${2:-15.99}

    print_section "Menu Item Profitability"
    print_info "Item Cost: \$$cost"
    print_info "Item Price: \$$price"

    # Profit = Price - Cost
    local profit=$(echo "scale=2; $price - $cost" | bc -l)

    # Profit Margin % = (Profit / Price) × 100
    local margin=$(echo "scale=2; ($profit / $price) * 100" | bc -l)

    # Markup % = (Profit / Cost) × 100
    local markup=$(echo "scale=2; ($profit / $cost) * 100" | bc -l)

    # Food cost %
    local food_cost_pct=$(echo "scale=2; ($cost / $price) * 100" | bc -l)

    print_section "Results"
    print_success "Profit per Item: \$$profit"
    print_info "Profit Margin: $margin%"
    print_info "Markup: $markup%"
    print_info "Food Cost %: $food_cost_pct%"

    # Typical restaurant markup: 200-400%
    if (( $(echo "$markup > 300" | bc -l) )); then
        print_success "Excellent markup (> 300%)"
    elif (( $(echo "$markup > 200" | bc -l) )); then
        print_success "Good markup (200-300%)"
    elif (( $(echo "$markup > 150" | bc -l) )); then
        print_warning "Low markup (150-200%)"
    else
        print_error "Too low (< 150%) - consider price increase"
    fi

    echo ""
}

# Calculate average check size
calc_avg_check() {
    local revenue=${1:-12500}
    local covers=${2:-180}

    print_section "Average Check Calculation"
    print_info "Total Revenue: \$$revenue"
    print_info "Number of Covers: $covers"

    # Average Check = Revenue / Covers
    local avg_check=$(echo "scale=2; $revenue / $covers" | bc -l)

    print_section "Results"
    print_success "Average Check: \$$avg_check per person"

    # Projections
    local daily_target_covers=200
    local projected_revenue=$(echo "scale=2; $avg_check * $daily_target_covers" | bc -l)

    print_info "If you serve $daily_target_covers covers:"
    print_info "Projected Revenue: \$$projected_revenue"

    echo ""
}

# Calculate kitchen ticket time
calc_ticket_time() {
    local total_time=${1:-1440}  # Total minutes
    local tickets=${2:-72}

    print_section "Kitchen Ticket Time"
    print_info "Total Time: $total_time minutes"
    print_info "Number of Tickets: $tickets"

    # Average = Total Time / Tickets
    local avg_time=$(echo "scale=1; $total_time / $tickets" | bc -l)

    print_section "Results"
    print_success "Average Ticket Time: $avg_time minutes"

    # Target: 12-20 minutes for casual dining
    if (( $(echo "$avg_time < 12" | bc -l) )); then
        print_success "Very fast service (< 12 min)"
    elif (( $(echo "$avg_time < 20" | bc -l) )); then
        print_success "Target range (12-20 min)"
    elif (( $(echo "$avg_time < 30" | bc -l) )); then
        print_warning "Slower than ideal (20-30 min)"
    else
        print_error "Too slow (> 30 min) - kitchen optimization needed"
    fi

    echo ""
}

# Calculate staff schedule optimization
calc_schedule() {
    local expected_covers=${1:-180}
    local labor_target=${2:-28}
    local avg_check=${3:-25.00}

    print_section "Staff Schedule Optimization"
    print_info "Expected Covers: $expected_covers"
    print_info "Labor Target: $labor_target%"
    print_info "Average Check: \$$avg_check"

    # Projected revenue
    local projected_revenue=$(echo "scale=2; $expected_covers * $avg_check" | bc -l)
    print_info "Projected Revenue: \$$projected_revenue"

    # Labor budget
    local labor_budget=$(echo "scale=2; $projected_revenue * $labor_target / 100" | bc -l)

    # Assume average hourly rate $15
    local avg_hourly=15
    local labor_hours=$(echo "scale=1; $labor_budget / $avg_hourly" | bc -l)

    # Covers per labor hour
    local covers_per_hour=$(echo "scale=1; $expected_covers / $labor_hours" | bc -l)

    # Sales per labor hour
    local sales_per_hour=$(echo "scale=2; $projected_revenue / $labor_hours" | bc -l)

    print_section "Results"
    print_success "Labor Budget: \$$labor_budget ($labor_target%)"
    print_info "Total Labor Hours: $labor_hours hours"
    print_info "Covers per Labor Hour: $covers_per_hour"
    print_info "Sales per Labor Hour: \$$sales_per_hour"

    # Recommended staffing (assuming 8-hour shifts)
    local staff_count=$(echo "scale=0; ($labor_hours + 7) / 8" | bc -l)
    print_info "Recommended Staff: $staff_count employees (8-hour shifts)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-010 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo "  ${CYAN}pos total${RESET}          Calculate order total with tax and tip"
    echo "                      --subtotal <amount> --tax <rate> --tip <percent> --discount <amount>"
    echo ""
    echo "  ${CYAN}turnover${RESET}           Calculate table turnover rate"
    echo "                      --parties <count> --tables <count> --hours <count>"
    echo ""
    echo "  ${CYAN}revpash${RESET}            Calculate Revenue Per Available Seat Hour"
    echo "                      --revenue <amount> --seats <count> --hours <count>"
    echo ""
    echo "  ${CYAN}food-cost${RESET}          Calculate food cost percentage"
    echo "                      --cogs <amount> --revenue <amount>"
    echo ""
    echo "  ${CYAN}labor-cost${RESET}         Calculate labor cost percentage"
    echo "                      --labor <amount> --revenue <amount>"
    echo ""
    echo "  ${CYAN}prime-cost${RESET}         Calculate prime cost (food + labor)"
    echo "                      --food <amount> --labor <amount> --revenue <amount>"
    echo ""
    echo "  ${CYAN}menu-profit${RESET}        Calculate menu item profitability"
    echo "                      --cost <amount> --price <amount>"
    echo ""
    echo "  ${CYAN}avg-check${RESET}          Calculate average check size"
    echo "                      --revenue <amount> --covers <count>"
    echo ""
    echo "  ${CYAN}ticket-time${RESET}        Calculate average kitchen ticket time"
    echo "                      --total-time <minutes> --tickets <count>"
    echo ""
    echo "  ${CYAN}schedule${RESET}           Optimize staff schedule"
    echo "                      --covers <count> --labor-target <percent> --avg-check <amount>"
    echo ""
    echo "  ${CYAN}version${RESET}            Show version information"
    echo "  ${CYAN}help${RESET}               Show this help message"
    echo ""
    echo "Examples:"
    echo ""
    echo "  ${GRAY}# Calculate order total${RESET}"
    echo "  wia-ind-010 pos total --subtotal 45.50 --tax 8 --tip 18"
    echo ""
    echo "  ${GRAY}# Calculate table turnover${RESET}"
    echo "  wia-ind-010 turnover --parties 85 --tables 25 --hours 8"
    echo ""
    echo "  ${GRAY}# Calculate RevPASH${RESET}"
    echo "  wia-ind-010 revpash --revenue 12500 --seats 100 --hours 8"
    echo ""
    echo "  ${GRAY}# Calculate food cost${RESET}"
    echo "  wia-ind-010 food-cost --cogs 3500 --revenue 10000"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo -e "${CYAN}WIA-IND-010: Restaurant Tech Standard${RESET}"
    echo -e "${GRAY}Version: $VERSION${RESET}"
    echo -e "${GRAY}Category: IND (Industry)${RESET}"
    echo -e "${GRAY}Color: Indigo (#6366F1)${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Parse arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --subtotal) SUBTOTAL="$2"; shift 2 ;;
            --tax) TAX="$2"; shift 2 ;;
            --tip) TIP="$2"; shift 2 ;;
            --discount) DISCOUNT="$2"; shift 2 ;;
            --parties) PARTIES="$2"; shift 2 ;;
            --tables) TABLES="$2"; shift 2 ;;
            --hours) HOURS="$2"; shift 2 ;;
            --revenue) REVENUE="$2"; shift 2 ;;
            --seats) SEATS="$2"; shift 2 ;;
            --cogs) COGS="$2"; shift 2 ;;
            --labor) LABOR="$2"; shift 2 ;;
            --food) FOOD="$2"; shift 2 ;;
            --cost) COST="$2"; shift 2 ;;
            --price) PRICE="$2"; shift 2 ;;
            --covers) COVERS="$2"; shift 2 ;;
            --total-time) TOTAL_TIME="$2"; shift 2 ;;
            --tickets) TICKETS="$2"; shift 2 ;;
            --labor-target) LABOR_TARGET="$2"; shift 2 ;;
            --avg-check) AVG_CHECK="$2"; shift 2 ;;
            *) shift ;;
        esac
    done
}

# Main command router
main() {
    case "${1:-help}" in
        pos)
            case "${2:-help}" in
                total)
                    parse_args "${@:3}"
                    calc_order_total "${SUBTOTAL:-45.50}" "${TAX:-8}" "${TIP:-18}" "${DISCOUNT:-0}"
                    ;;
                *)
                    show_help
                    ;;
            esac
            ;;
        turnover)
            parse_args "${@:2}"
            calc_turnover "${PARTIES:-85}" "${TABLES:-25}" "${HOURS:-8}"
            ;;
        revpash)
            parse_args "${@:2}"
            calc_revpash "${REVENUE:-12500}" "${SEATS:-100}" "${HOURS:-8}"
            ;;
        food-cost)
            parse_args "${@:2}"
            calc_food_cost "${COGS:-3500}" "${REVENUE:-10000}"
            ;;
        labor-cost)
            parse_args "${@:2}"
            calc_labor_cost "${LABOR:-7000}" "${REVENUE:-25000}"
            ;;
        prime-cost)
            parse_args "${@:2}"
            calc_prime_cost "${FOOD:-3500}" "${LABOR:-7000}" "${REVENUE:-25000}"
            ;;
        menu-profit)
            parse_args "${@:2}"
            calc_menu_profit "${COST:-4.50}" "${PRICE:-15.99}"
            ;;
        avg-check)
            parse_args "${@:2}"
            calc_avg_check "${REVENUE:-12500}" "${COVERS:-180}"
            ;;
        ticket-time)
            parse_args "${@:2}"
            calc_ticket_time "${TOTAL_TIME:-1440}" "${TICKETS:-72}"
            ;;
        schedule)
            parse_args "${@:2}"
            calc_schedule "${COVERS:-180}" "${LABOR_TARGET:-28}" "${AVG_CHECK:-25.00}"
            ;;
        version)
            show_version
            ;;
        help|--help|-h)
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

# Run main
main "$@"
