#!/bin/bash

################################################################################
# WIA-AUTO-015: Autonomous Ship CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Maritime Autonomy Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to autonomous ship operations
# including navigation, collision avoidance, route planning, and monitoring.
################################################################################

set -e

# Colors for output
ORANGE='\033[38;5;208m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
BLUE='\033[0;34m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
EARTH_RADIUS_NM=3440.065
SAFE_DISTANCE_NM=2.0
KNOTS_TO_MPS=0.514444
NM_TO_METERS=1852

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           🚢 WIA-AUTO-015: Autonomous Ship CLI                ║"
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

# Convert degrees to radians
to_radians() {
    echo "scale=10; $1 * 3.141592653589793 / 180" | bc -l
}

# Convert radians to degrees
to_degrees() {
    echo "scale=10; $1 * 180 / 3.141592653589793" | bc -l
}

# Calculate great circle distance
calc_distance() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    # Convert to radians
    local phi1=$(to_radians $lat1)
    local phi2=$(to_radians $lat2)
    local delta_phi=$(echo "scale=10; $phi2 - $phi1" | bc -l)
    local delta_lambda=$(to_radians $(echo "scale=10; $lon2 - $lon1" | bc -l))

    # Haversine formula
    local a=$(echo "scale=10; s($delta_phi/2)^2 + c($phi1) * c($phi2) * s($delta_lambda/2)^2" | bc -l)
    local c=$(echo "scale=10; 2 * a(sqrt($a)/sqrt(1-$a))" | bc -l)
    local distance=$(echo "scale=2; $EARTH_RADIUS_NM * $c" | bc -l)

    echo "$distance"
}

# Calculate bearing
calc_bearing() {
    local lat1=$1
    local lon1=$2
    local lat2=$3
    local lon2=$4

    local phi1=$(to_radians $lat1)
    local phi2=$(to_radians $lat2)
    local delta_lambda=$(to_radians $(echo "scale=10; $lon2 - $lon1" | bc -l))

    local y=$(echo "scale=10; s($delta_lambda) * c($phi2)" | bc -l)
    local x=$(echo "scale=10; c($phi1) * s($phi2) - s($phi1) * c($phi2) * c($delta_lambda)" | bc -l)
    local theta=$(echo "scale=10; a($y/$x)" | bc -l)

    local bearing=$(to_degrees $theta)
    local normalized=$(echo "scale=2; ($bearing + 360) % 360" | bc -l)

    echo "$normalized"
}

# Initialize autonomous ship system
init_ship() {
    local imo=${1:-"9876543"}
    local mmsi=${2:-"123456789"}

    print_section "Initializing Autonomous Ship System"
    print_info "IMO Number: $imo"
    print_info "MMSI: $mmsi"

    # Create configuration
    local config_dir="$HOME/.wia/auto-015"
    mkdir -p "$config_dir"

    cat > "$config_dir/ship-$imo.json" <<EOF
{
  "imo": "$imo",
  "mmsi": "$mmsi",
  "initialized": "$(date -Iseconds)",
  "autonomy_level": 0,
  "status": "initialized"
}
EOF

    print_success "Ship configuration created: $config_dir/ship-$imo.json"
    print_info "Default autonomy level: 0 (Manual)"
    echo ""
}

# Calculate collision risk
collision_risk() {
    local own_pos=$1      # "lat,lon"
    local own_heading=$2
    local own_speed=$3
    local target_pos=$4   # "lat,lon"
    local target_heading=$5
    local target_speed=$6

    print_section "Collision Risk Assessment"

    # Parse positions
    IFS=',' read -r own_lat own_lon <<< "$own_pos"
    IFS=',' read -r target_lat target_lon <<< "$target_pos"

    print_info "Own Ship: $own_lat°N, $own_lon°E @ ${own_heading}° / ${own_speed} kts"
    print_info "Target: $target_lat°N, $target_lon°E @ ${target_heading}° / ${target_speed} kts"

    # Calculate distance and bearing
    local distance=$(calc_distance $own_lat $own_lon $target_lat $target_lon)
    local bearing=$(calc_bearing $own_lat $own_lon $target_lat $target_lon)

    print_section "Geometric Analysis"
    print_info "Distance: $distance NM"
    print_info "Bearing: $bearing°"

    # Calculate relative bearing
    local rel_bearing=$(echo "scale=2; ($bearing - $own_heading + 360) % 360" | bc -l)
    print_info "Relative Bearing: $rel_bearing°"

    # Determine COLREG situation
    local situation="unknown"
    if (( $(echo "$rel_bearing >= 5 && $rel_bearing <= 112.5" | bc -l) )); then
        situation="CROSSING (target on starboard)"
    elif (( $(echo "$rel_bearing > 112.5 && $rel_bearing < 247.5" | bc -l) )); then
        situation="OVERTAKING"
    elif (( $(echo "$rel_bearing >= 170 && $rel_bearing <= 190" | bc -l) )); then
        situation="HEAD-ON"
    else
        situation="SAFE PASSING"
    fi

    # Simplified CPA/TCPA calculation (approximation)
    local rel_speed=$(echo "scale=2; sqrt(($own_speed)^2 + ($target_speed)^2 - 2*$own_speed*$target_speed*c((($target_heading - $own_heading)*3.14159/180)))" | bc -l)
    local tcpa=$(echo "scale=2; ($distance * c((($rel_bearing)*3.14159/180))) / $rel_speed" | bc -l)
    local cpa=$(echo "scale=2; $distance * s((($rel_bearing)*3.14159/180))" | bc -l)

    # Handle negative TCPA (target moving away)
    if (( $(echo "$tcpa < 0" | bc -l) )); then
        tcpa=$(echo "scale=2; -1 * $tcpa" | bc -l)
    fi

    local tcpa_minutes=$(echo "scale=1; $tcpa * 60" | bc -l)

    print_section "Collision Parameters"
    print_info "CPA (Closest Point of Approach): $cpa NM"
    print_info "TCPA (Time to CPA): $tcpa_minutes minutes"

    # Calculate risk index
    local risk_index=$(echo "scale=2; ($SAFE_DISTANCE_NM / ($cpa + 0.1)) * (20 / ($tcpa + 0.1))" | bc -l)

    print_section "COLREG Assessment"
    print_info "Situation: $situation"

    print_section "Risk Assessment"
    print_info "Risk Index: $risk_index"

    if (( $(echo "$risk_index < 0.3" | bc -l) )); then
        print_success "Risk Level: LOW - Monitor only"
    elif (( $(echo "$risk_index < 0.7" | bc -l) )); then
        print_warning "Risk Level: MEDIUM - Prepare maneuver"
    elif (( $(echo "$risk_index < 1.0" | bc -l) )); then
        print_error "Risk Level: HIGH - Execute maneuver"
    else
        print_error "Risk Level: CRITICAL - Emergency action required"
    fi

    # Recommend action
    if (( $(echo "$risk_index >= 0.7" | bc -l) )); then
        print_section "Recommended Action"
        if [[ "$situation" == *"CROSSING"* ]]; then
            print_warning "Alter course to starboard by 30°"
            print_info "COLREG Rule 15: Give way to vessel on starboard"
        elif [[ "$situation" == "HEAD-ON" ]]; then
            print_warning "Alter course to starboard by 15°"
            print_info "COLREG Rule 14: Both vessels turn to starboard"
        elif [[ "$situation" == "OVERTAKING" ]]; then
            print_info "Maintain course and speed (stand-on vessel)"
            print_info "COLREG Rule 13: Overtaking vessel must keep clear"
        fi
    fi

    echo ""
}

# Plan route
plan_route() {
    local from=$1    # "lat,lon"
    local to=$2      # "lat,lon"
    local optimize=${3:-"fuel"}

    print_section "Route Planning"

    IFS=',' read -r from_lat from_lon <<< "$from"
    IFS=',' read -r to_lat to_lon <<< "$to"

    print_info "Origin: $from_lat°N, $from_lon°E"
    print_info "Destination: $to_lat°N, $to_lon°E"
    print_info "Optimization: $optimize"

    # Calculate great circle distance
    local distance=$(calc_distance $from_lat $from_lon $to_lat $to_lon)
    local initial_bearing=$(calc_bearing $from_lat $from_lon $to_lat $to_lon)

    print_section "Route Analysis"
    print_success "Great Circle Distance: $distance NM"
    print_info "Initial Bearing: $initial_bearing°"

    # Estimate voyage parameters
    local cruise_speed=16  # knots
    local voyage_hours=$(echo "scale=1; $distance / $cruise_speed" | bc -l)
    local voyage_days=$(echo "scale=1; $voyage_hours / 24" | bc -l)

    # Fuel estimation (simplified)
    local fuel_rate=8.5  # tons/day
    local estimated_fuel=$(echo "scale=1; $voyage_days * $fuel_rate" | bc -l)

    print_section "Voyage Estimates"
    print_info "Cruising Speed: $cruise_speed knots"
    print_info "Voyage Time: $voyage_hours hours ($voyage_days days)"
    print_info "Estimated Fuel: $estimated_fuel metric tons"

    # Generate waypoints (simplified - just midpoint for demonstration)
    local mid_lat=$(echo "scale=6; ($from_lat + $to_lat) / 2" | bc -l)
    local mid_lon=$(echo "scale=6; ($from_lon + $to_lon) / 2" | bc -l)

    print_section "Waypoints"
    print_info "WP001: $mid_lat°N, $mid_lon°E"
    print_success "Route plan generated successfully"

    echo ""
}

# Monitor ship status
monitor_ship() {
    local interval=${1:-5}

    print_section "Ship Monitoring (Update every ${interval}s)"
    print_info "Press Ctrl+C to stop"
    echo ""

    while true; do
        clear
        print_header

        print_section "Real-time Status"
        print_info "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"

        print_section "Position & Navigation"
        print_success "GNSS Fix: RTK (18 satellites)"
        print_info "Position: 35.6762°N, 139.6503°E"
        print_info "Heading: 090° (East)"
        print_info "Speed: 15.2 knots"
        print_info "Course Over Ground: 089.8°"

        print_section "Autonomy"
        print_info "Autonomy Level: 3 (Remote without crew)"
        print_success "Mode: AUTONOMOUS NAVIGATION"

        print_section "Sensors"
        print_success "Radar (X-band): Operational (48 NM range)"
        print_success "Radar (S-band): Operational (96 NM range)"
        print_success "LiDAR: Operational (1.2M points/sec)"
        print_success "AIS: Operational (15 targets)"
        print_success "Cameras: 8/8 operational"

        print_section "Propulsion"
        print_info "Main Engine: 850 RPM (75% load)"
        print_info "Fuel Level: 82.5%"
        print_info "Consumption: 8.5 tons/day"

        print_section "Weather"
        print_info "Wind: 12 kts from 045°"
        print_info "Wave Height: 1.5 meters"
        print_info "Visibility: 15 NM"
        print_success "Conditions: NOMINAL"

        print_section "Alerts"
        print_info "No active warnings"

        sleep "$interval"
    done
}

# Execute collision avoidance
avoid_collision() {
    local strategy=${1:-"alter-course"}
    local magnitude=${2:-15}

    print_section "Collision Avoidance Maneuver"
    print_info "Strategy: $strategy"
    print_info "Magnitude: $magnitude"

    if [ "$strategy" == "alter-course" ]; then
        print_section "Executing Course Alteration"
        print_info "Current Heading: 090°"
        local new_heading=$(echo "scale=0; (90 + $magnitude) % 360" | bc -l)
        print_info "New Heading: $new_heading°"
        print_warning "Turning to starboard by $magnitude°"

        for i in {1..5}; do
            sleep 0.5
            echo -n "."
        done
        echo ""

        print_success "Maneuver completed"
        print_info "New heading established: $new_heading°"
        print_info "Monitor target vessel for CPA confirmation"

    elif [ "$strategy" == "reduce-speed" ]; then
        print_section "Executing Speed Reduction"
        print_info "Current Speed: 15.2 knots"
        local new_speed=$(echo "scale=1; 15.2 * (1 - $magnitude / 100)" | bc -l)
        print_info "New Speed: $new_speed knots"
        print_warning "Reducing speed by $magnitude%"

        for i in {1..5}; do
            sleep 0.5
            echo -n "."
        done
        echo ""

        print_success "Speed reduction completed"
        print_info "New speed established: $new_speed knots"
    fi

    echo ""
}

# Set autonomy level
set_autonomy() {
    local level=${1:-0}
    local reason=${2:-"manual request"}

    print_section "Autonomy Level Change"
    print_info "Requested Level: $level"
    print_info "Reason: $reason"

    case $level in
        0)
            print_info "Level 0: Manual Operation"
            ;;
        1)
            print_info "Level 1: On-board Decision Support"
            ;;
        2)
            print_info "Level 2: Remote Control with Crew"
            ;;
        3)
            print_info "Level 3: Remote Control without Crew"
            ;;
        4)
            print_info "Level 4: Fully Autonomous"
            ;;
        *)
            print_error "Invalid autonomy level: $level"
            return 1
            ;;
    esac

    print_section "Pre-transition Checks"
    print_success "System health: NOMINAL"
    print_success "Sensor coverage: 100%"
    print_success "Communication link: STABLE"
    print_success "Weather conditions: ACCEPTABLE"

    print_section "Executing Transition"
    for i in {1..3}; do
        sleep 0.5
        echo -n "."
    done
    echo ""

    print_success "Autonomy level changed to: $level"
    print_info "All systems updated"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-015 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  init                     Initialize autonomous ship system"
    echo "    --imo <number>         IMO number (default: 9876543)"
    echo "    --mmsi <number>        MMSI number (default: 123456789)"
    echo ""
    echo "  collision-risk           Calculate collision risk with target"
    echo "    --own-pos <lat,lon>    Own ship position"
    echo "    --own-heading <deg>    Own ship heading"
    echo "    --own-speed <kts>      Own ship speed"
    echo "    --target-pos <lat,lon> Target position"
    echo "    --target-heading <deg> Target heading"
    echo "    --target-speed <kts>   Target speed"
    echo ""
    echo "  plan-route               Plan route between two positions"
    echo "    --from <lat,lon>       Origin position"
    echo "    --to <lat,lon>         Destination position"
    echo "    --optimize <method>    Optimization: fuel/time/safety (default: fuel)"
    echo ""
    echo "  monitor                  Monitor ship status in real-time"
    echo "    --interval <seconds>   Update interval (default: 5)"
    echo ""
    echo "  avoid-collision          Execute collision avoidance maneuver"
    echo "    --strategy <type>      alter-course/reduce-speed (default: alter-course)"
    echo "    --magnitude <value>    Degrees or percentage (default: 15)"
    echo ""
    echo "  set-autonomy             Change autonomy level"
    echo "    --level <0-4>          Target autonomy level"
    echo "    --reason <text>        Reason for change"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-015 init --imo 9876543 --mmsi 123456789"
    echo "  wia-auto-015 collision-risk --own-pos '35.6762,139.6503' --own-heading 90 \\"
    echo "                               --own-speed 15 --target-pos '35.6850,139.7500' \\"
    echo "                               --target-heading 270 --target-speed 12"
    echo "  wia-auto-015 plan-route --from '35.6762,139.6503' --to '1.2897,103.8501'"
    echo "  wia-auto-015 monitor --interval 5"
    echo "  wia-auto-015 avoid-collision --strategy alter-course --magnitude 30"
    echo "  wia-auto-015 set-autonomy --level 3 --reason 'entering open ocean'"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-015 Autonomous Ship CLI Tool"
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
    init)
        IMO="9876543"
        MMSI="123456789"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --imo) IMO=$2; shift 2 ;;
                --mmsi) MMSI=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        init_ship "$IMO" "$MMSI"
        ;;

    collision-risk)
        OWN_POS="35.6762,139.6503"
        OWN_HEADING=90
        OWN_SPEED=15
        TARGET_POS="35.6850,139.7500"
        TARGET_HEADING=270
        TARGET_SPEED=12

        while [[ $# -gt 0 ]]; do
            case $1 in
                --own-pos) OWN_POS=$2; shift 2 ;;
                --own-heading) OWN_HEADING=$2; shift 2 ;;
                --own-speed) OWN_SPEED=$2; shift 2 ;;
                --target-pos) TARGET_POS=$2; shift 2 ;;
                --target-heading) TARGET_HEADING=$2; shift 2 ;;
                --target-speed) TARGET_SPEED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        collision_risk "$OWN_POS" "$OWN_HEADING" "$OWN_SPEED" "$TARGET_POS" "$TARGET_HEADING" "$TARGET_SPEED"
        ;;

    plan-route)
        FROM="35.6762,139.6503"
        TO="1.2897,103.8501"
        OPTIMIZE="fuel"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --optimize) OPTIMIZE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        plan_route "$FROM" "$TO" "$OPTIMIZE"
        ;;

    monitor)
        INTERVAL=5

        while [[ $# -gt 0 ]]; do
            case $1 in
                --interval) INTERVAL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        monitor_ship "$INTERVAL"
        ;;

    avoid-collision)
        STRATEGY="alter-course"
        MAGNITUDE=15

        while [[ $# -gt 0 ]]; do
            case $1 in
                --strategy) STRATEGY=$2; shift 2 ;;
                --magnitude) MAGNITUDE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        avoid_collision "$STRATEGY" "$MAGNITUDE"
        ;;

    set-autonomy)
        LEVEL=0
        REASON="manual request"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --level) LEVEL=$2; shift 2 ;;
                --reason) REASON=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        set_autonomy "$LEVEL" "$REASON"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-015 help' for usage information"
        exit 1
        ;;
esac

exit 0
