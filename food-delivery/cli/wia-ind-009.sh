#!/bin/bash

################################################################################
# WIA-IND-009: Food Delivery Standard CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Food Delivery Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to food delivery calculations
# including delivery time estimation, cost calculation, route optimization,
# temperature monitoring, and driver performance metrics.
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
PI=3.14159265359
EARTH_RADIUS=6371  # km

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🚚 WIA-IND-009: Food Delivery CLI                    ║"
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

# Calculate haversine distance between two coordinates
# 두 좌표 사이의 하버사인 거리 계산
haversine_distance() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    # Convert degrees to radians
    local lat1_rad=$(echo "scale=10; $lat1 * $PI / 180" | bc -l)
    local lon1_rad=$(echo "scale=10; $lon1 * $PI / 180" | bc -l)
    local lat2_rad=$(echo "scale=10; $lat2 * $PI / 180" | bc -l)
    local lon2_rad=$(echo "scale=10; $lon2 * $PI / 180" | bc -l)

    # Differences
    local dlat=$(echo "scale=10; $lat2_rad - $lat1_rad" | bc -l)
    local dlon=$(echo "scale=10; $lon2_rad - $lon1_rad" | bc -l)

    # Haversine formula
    local a=$(echo "scale=10; s($dlat/2)^2 + c($lat1_rad) * c($lat2_rad) * s($dlon/2)^2" | bc -l)
    local c=$(echo "scale=10; 2 * a($a / sqrt(1-$a)) / 1" | bc -l)
    local distance=$(echo "scale=2; $EARTH_RADIUS * $c" | bc -l)

    echo $distance
}

# Calculate delivery time estimation
# 배달 시간 예측 계산
calc_time() {
    local distance=${1:-5.0}
    local prep_time=${2:-15}
    local traffic=${3:-1.2}
    local vehicle=${4:-"bike"}

    print_section "Delivery Time Calculation"
    print_info "Distance: $distance km"
    print_info "Prep Time: $prep_time minutes"
    print_info "Traffic Factor: ${traffic}x"
    print_info "Vehicle Type: $vehicle"

    # Average speeds by vehicle type (km/h)
    local speed=15
    case $vehicle in
        bike)       speed=15 ;;
        ebike)      speed=20 ;;
        scooter)    speed=25 ;;
        motorcycle) speed=35 ;;
        car)        speed=30 ;;
        *)          speed=20 ;;
    esac

    print_info "Base Speed: $speed km/h"

    # Calculate transit time
    local transit_time=$(echo "scale=1; ($distance / $speed * 60) * $traffic" | bc -l)
    print_info "Transit Time: $transit_time minutes"

    # Other time components
    local assign_time=2
    local pickup_time=7
    local dropoff_time=3

    # Total time
    local total=$(echo "scale=1; $prep_time + $assign_time + $pickup_time + $transit_time + $dropoff_time" | bc -l)

    print_section "Results"
    print_success "Preparation Time: $prep_time min"
    print_success "Assignment Time: $assign_time min"
    print_success "Pickup Time: $pickup_time min"
    print_success "Transit Time: $transit_time min"
    print_success "Dropoff Time: $dropoff_time min"
    echo ""
    print_success "Total Delivery Time: $total minutes"

    # ETA with confidence intervals
    local eta_min=$(echo "scale=0; $total * 0.85" | bc -l)
    local eta_max=$(echo "scale=0; $total * 1.20" | bc -l)
    print_info "ETA Range: $eta_min - $eta_max minutes (with traffic variability)"

    echo ""
}

# Calculate delivery cost
# 배달 비용 계산
calc_cost() {
    local distance=${1:-5.0}
    local time=${2:-25}
    local surge=${3:-1.0}
    local subtotal=${4:-25.00}

    print_section "Delivery Cost Calculation"
    print_info "Distance: $distance km"
    print_info "Estimated Time: $time minutes"
    print_info "Surge Multiplier: ${surge}x"
    print_info "Order Subtotal: \$$subtotal"

    # Base fee based on distance
    local base_fee
    if (( $(echo "$distance <= 2" | bc -l) )); then
        base_fee=2.99
    elif (( $(echo "$distance <= 5" | bc -l) )); then
        base_fee=3.99
    elif (( $(echo "$distance <= 10" | bc -l) )); then
        base_fee=5.99
    else
        local extra=$(echo "scale=2; ($distance - 10) * 0.50" | bc -l)
        base_fee=$(echo "scale=2; 5.99 + $extra" | bc -l)
    fi

    print_info "Base Fee: \$$base_fee"

    # Distance fee
    local distance_fee=$(echo "scale=2; $distance * 0.40" | bc -l)
    print_info "Distance Fee: \$$distance_fee"

    # Time fee
    local time_fee=$(echo "scale=2; $time * 0.15" | bc -l)
    print_info "Time Fee: \$$time_fee"

    # Delivery fee (before surge)
    local delivery_fee=$(echo "scale=2; $base_fee + $distance_fee + $time_fee" | bc -l)
    print_info "Delivery Fee (before surge): \$$delivery_fee"

    # Apply surge
    local delivery_fee_surge=$(echo "scale=2; $delivery_fee * $surge" | bc -l)

    # Service fee (15% of subtotal)
    local service_fee=$(echo "scale=2; $subtotal * 0.15" | bc -l)

    # Small order fee
    local small_order_fee=0
    if (( $(echo "$subtotal < 12" | bc -l) )); then
        small_order_fee=$(echo "scale=2; 12 - $subtotal" | bc -l)
    fi

    # Tax (9%)
    local tax=$(echo "scale=2; ($subtotal + $delivery_fee_surge + $service_fee + $small_order_fee) * 0.09" | bc -l)

    # Total
    local total=$(echo "scale=2; $subtotal + $delivery_fee_surge + $service_fee + $small_order_fee + $tax" | bc -l)

    print_section "Cost Breakdown"
    print_success "Order Subtotal: \$$subtotal"
    print_success "Delivery Fee: \$$delivery_fee_surge"
    print_success "Service Fee: \$$service_fee"
    if (( $(echo "$small_order_fee > 0" | bc -l) )); then
        print_warning "Small Order Fee: \$$small_order_fee"
    fi
    print_success "Tax (9%): \$$tax"
    echo ""
    print_success "Total: \$$total"

    # Breakdown for stakeholders
    print_section "Revenue Distribution"
    local driver_pay=$(echo "scale=2; $delivery_fee_surge * 0.80" | bc -l)
    local platform_rev=$(echo "scale=2; $service_fee + ($delivery_fee_surge * 0.20)" | bc -l)
    print_info "To Driver: \$$driver_pay"
    print_info "To Restaurant: \$$subtotal"
    print_info "To Platform: \$$platform_rev"

    echo ""
}

# Calculate route optimization savings
# 경로 최적화 절감 계산
optimize_route() {
    local num_orders=${1:-3}
    local avg_distance=${2:-4.0}

    print_section "Route Optimization Analysis"
    print_info "Number of Orders: $num_orders"
    print_info "Average Distance per Order: $avg_distance km"

    # Without optimization (separate trips)
    local unoptimized_distance=$(echo "scale=2; $num_orders * $avg_distance * 2" | bc -l)
    local unoptimized_time=$(echo "scale=0; $unoptimized_distance / 20 * 60" | bc -l)

    print_info "Unoptimized Distance: $unoptimized_distance km"
    print_info "Unoptimized Time: $unoptimized_time minutes"

    # With optimization (TSP routing, ~35% savings)
    local optimization_rate=0.35
    local optimized_distance=$(echo "scale=2; $unoptimized_distance * (1 - $optimization_rate)" | bc -l)
    local optimized_time=$(echo "scale=0; $optimized_distance / 20 * 60" | bc -l)

    # Savings
    local distance_saved=$(echo "scale=2; $unoptimized_distance - $optimized_distance" | bc -l)
    local time_saved=$(echo "scale=0; $unoptimized_time - $optimized_time" | bc -l)
    local fuel_saved=$(echo "scale=2; $distance_saved * 0.08" | bc -l)  # 0.08 L/km
    local co2_saved=$(echo "scale=2; $distance_saved * 0.12" | bc -l)   # 0.12 kg/km

    print_section "Optimization Results"
    print_success "Optimized Distance: $optimized_distance km"
    print_success "Optimized Time: $optimized_time minutes"
    echo ""
    print_success "Distance Saved: $distance_saved km (${optimization_rate}00%)"
    print_success "Time Saved: $time_saved minutes"
    print_success "Fuel Saved: $fuel_saved liters"
    print_success "CO2 Reduced: $co2_saved kg"

    # Efficiency metric
    local efficiency=$(echo "scale=2; $num_orders / $optimized_distance" | bc -l)
    print_info "Efficiency: $efficiency orders/km"

    echo ""
}

# Monitor temperature compliance
# 온도 준수 모니터링
monitor_temp() {
    local food_type=${1:-"hot"}
    local duration=${2:-30}
    local actual_temp=${3:-55}

    print_section "Temperature Compliance Monitoring"
    print_info "Food Type: $food_type"
    print_info "Transit Duration: $duration minutes"
    print_info "Actual Temperature: ${actual_temp}°C"

    # Define safe ranges
    local min_temp
    local max_temp
    local max_duration

    case $food_type in
        hot)
            min_temp=60
            max_temp=100
            max_duration=45
            print_info "Safe Range: ≥${min_temp}°C"
            ;;
        cold)
            min_temp=-10
            max_temp=4
            max_duration=60
            print_info "Safe Range: ≤${max_temp}°C"
            ;;
        frozen)
            min_temp=-40
            max_temp=-15
            max_duration=30
            print_info "Safe Range: ≤${max_temp}°C"
            ;;
        ambient)
            min_temp=15
            max_temp=25
            max_duration=90
            print_info "Safe Range: ${min_temp}-${max_temp}°C"
            ;;
    esac

    print_info "Maximum Safe Duration: $max_duration minutes"

    # Check compliance
    print_section "Compliance Check"

    local temp_ok=0
    local time_ok=0

    # Temperature check
    if [ "$food_type" = "hot" ]; then
        if (( $(echo "$actual_temp >= $min_temp" | bc -l) )); then
            temp_ok=1
            print_success "Temperature: PASS (${actual_temp}°C ≥ ${min_temp}°C)"
        else
            print_error "Temperature: FAIL (${actual_temp}°C < ${min_temp}°C)"
        fi
    elif [ "$food_type" = "cold" ] || [ "$food_type" = "frozen" ]; then
        if (( $(echo "$actual_temp <= $max_temp" | bc -l) )); then
            temp_ok=1
            print_success "Temperature: PASS (${actual_temp}°C ≤ ${max_temp}°C)"
        else
            print_error "Temperature: FAIL (${actual_temp}°C > ${max_temp}°C)"
        fi
    else
        if (( $(echo "$actual_temp >= $min_temp" | bc -l) )) && (( $(echo "$actual_temp <= $max_temp" | bc -l) )); then
            temp_ok=1
            print_success "Temperature: PASS (${min_temp}°C ≤ ${actual_temp}°C ≤ ${max_temp}°C)"
        else
            print_error "Temperature: FAIL (out of ${min_temp}-${max_temp}°C range)"
        fi
    fi

    # Time check
    if (( $(echo "$duration <= $max_duration" | bc -l) )); then
        time_ok=1
        print_success "Duration: PASS (${duration} min ≤ ${max_duration} min)"
    else
        print_warning "Duration: WARNING (${duration} min > ${max_duration} min recommended)"
    fi

    # Overall compliance
    echo ""
    if [ $temp_ok -eq 1 ] && [ $time_ok -eq 1 ]; then
        print_success "Overall: COMPLIANT ✓"
    elif [ $temp_ok -eq 1 ]; then
        print_warning "Overall: PARTIAL COMPLIANCE (temperature OK, time exceeded)"
    else
        print_error "Overall: NON-COMPLIANT ✗"
        print_warning "Action Required: Issue refund or credit"
    fi

    echo ""
}

# Calculate driver performance metrics
# 배달원 성과 지표 계산
driver_stats() {
    local orders=${1:-45}
    local hours=${2:-8}
    local on_time=${3:-42}
    local rating=${4:-4.7}

    print_section "Driver Performance Metrics"
    print_info "Total Orders: $orders"
    print_info "Active Hours: $hours"
    print_info "On-Time Deliveries: $on_time"
    print_info "Average Rating: $rating/5.0"

    # Calculate metrics
    local orders_per_hour=$(echo "scale=2; $orders / $hours" | bc -l)
    local on_time_rate=$(echo "scale=1; $on_time / $orders * 100" | bc -l)

    # Earnings estimation
    local avg_delivery_fee=8.50
    local avg_tip=4.20
    local total_earnings=$(echo "scale=2; $orders * ($avg_delivery_fee + $avg_tip)" | bc -l)
    local earnings_per_hour=$(echo "scale=2; $total_earnings / $hours" | bc -l)

    print_section "Efficiency Metrics"
    print_success "Orders per Hour: $orders_per_hour"
    print_success "On-Time Rate: $on_time_rate%"

    # Performance tier
    local tier="Bronze"
    if (( $(echo "$orders >= 2000 && $rating >= 4.8 && $on_time_rate >= 95" | bc -l) )); then
        tier="Platinum"
    elif (( $(echo "$orders >= 500 && $rating >= 4.6 && $on_time_rate >= 90" | bc -l) )); then
        tier="Gold"
    elif (( $(echo "$orders >= 100 && $rating >= 4.3 && $on_time_rate >= 85" | bc -l) )); then
        tier="Silver"
    fi

    print_info "Performance Tier: $tier"

    print_section "Earnings Estimate"
    print_success "Total Earnings: \$$total_earnings"
    print_success "Hourly Rate: \$$earnings_per_hour/hour"
    print_info "Average per Delivery: \$$(echo "scale=2; $total_earnings / $orders" | bc -l)"

    # Quality assessment
    print_section "Quality Assessment"
    if (( $(echo "$rating >= 4.5" | bc -l) )); then
        print_success "Customer Rating: Excellent ($rating/5.0)"
    elif (( $(echo "$rating >= 4.0" | bc -l) )); then
        print_success "Customer Rating: Good ($rating/5.0)"
    else
        print_warning "Customer Rating: Needs Improvement ($rating/5.0)"
    fi

    if (( $(echo "$on_time_rate >= 90" | bc -l) )); then
        print_success "On-Time Performance: Excellent ($on_time_rate%)"
    elif (( $(echo "$on_time_rate >= 80" | bc -l) )); then
        print_success "On-Time Performance: Good ($on_time_rate%)"
    else
        print_warning "On-Time Performance: Needs Improvement ($on_time_rate%)"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "USAGE:"
    echo "  wia-ind-009 <command> [options]"
    echo ""
    echo "COMMANDS:"
    echo "  calc-time       Calculate delivery time estimation"
    echo "  calc-cost       Calculate delivery cost breakdown"
    echo "  optimize-route  Analyze route optimization savings"
    echo "  monitor-temp    Check temperature compliance"
    echo "  driver-stats    Calculate driver performance metrics"
    echo "  version         Show version information"
    echo "  help            Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  wia-ind-009 calc-time --distance 5.2 --prep-time 15 --traffic 1.2 --vehicle ebike"
    echo "  wia-ind-009 calc-cost --distance 8.5 --time 25 --surge 1.5 --subtotal 30.00"
    echo "  wia-ind-009 optimize-route --orders 4 --avg-distance 3.5"
    echo "  wia-ind-009 monitor-temp --type hot --duration 30 --temp 55"
    echo "  wia-ind-009 driver-stats --orders 45 --hours 8 --on-time 42 --rating 4.7"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo ""
}

# Main command router
main() {
    case ${1:-help} in
        calc-time)
            shift
            local distance=5.0 prep=15 traffic=1.2 vehicle="bike"
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --distance) distance="$2"; shift 2 ;;
                    --prep-time) prep="$2"; shift 2 ;;
                    --traffic) traffic="$2"; shift 2 ;;
                    --vehicle) vehicle="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            calc_time "$distance" "$prep" "$traffic" "$vehicle"
            ;;
        calc-cost)
            shift
            local distance=5.0 time=25 surge=1.0 subtotal=25.00
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --distance) distance="$2"; shift 2 ;;
                    --time) time="$2"; shift 2 ;;
                    --surge) surge="$2"; shift 2 ;;
                    --subtotal) subtotal="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            calc_cost "$distance" "$time" "$surge" "$subtotal"
            ;;
        optimize-route)
            shift
            local orders=3 avg_dist=4.0
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --orders) orders="$2"; shift 2 ;;
                    --avg-distance) avg_dist="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            optimize_route "$orders" "$avg_dist"
            ;;
        monitor-temp)
            shift
            local type="hot" duration=30 temp=55
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --type) type="$2"; shift 2 ;;
                    --duration) duration="$2"; shift 2 ;;
                    --temp) temp="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            monitor_temp "$type" "$duration" "$temp"
            ;;
        driver-stats)
            shift
            local orders=45 hours=8 on_time=42 rating=4.7
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --orders) orders="$2"; shift 2 ;;
                    --hours) hours="$2"; shift 2 ;;
                    --on-time) on_time="$2"; shift 2 ;;
                    --rating) rating="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            driver_stats "$orders" "$hours" "$on_time" "$rating"
            ;;
        version)
            print_header
            echo "Version: $VERSION"
            echo "Standard: WIA-IND-009"
            echo "Category: Industry (Indigo)"
            echo ""
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            echo "Run 'wia-ind-009 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
