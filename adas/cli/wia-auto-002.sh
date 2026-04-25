#!/bin/bash

################################################################################
# WIA-AUTO-002: ADAS - Advanced Driver Assistance System CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Safety Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to ADAS calculations including
# safe distance, TTC, lane detection, and collision avoidance.
################################################################################

set -e

# Colors for output
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
GRAVITY=9.81
REACTION_TIME=1.5

# Friction coefficients
FRICTION_DRY=0.85
FRICTION_WET=0.6
FRICTION_SNOW=0.25
FRICTION_ICE=0.12

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🚗 WIA-AUTO-002: ADAS CLI Tool                         ║"
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

# Calculate safe following distance
calc_distance() {
    local speed=${1:-100}  # km/h
    local road_condition=${2:-dry}

    print_section "Safe Following Distance Calculation"
    print_info "Speed: $speed km/h"
    print_info "Road Condition: $road_condition"

    # Convert km/h to m/s
    local velocity=$(echo "scale=2; $speed / 3.6" | bc -l)
    print_info "Velocity: $velocity m/s"

    # Get friction coefficient
    local friction
    case $road_condition in
        dry)    friction=$FRICTION_DRY ;;
        wet)    friction=$FRICTION_WET ;;
        snow)   friction=$FRICTION_SNOW ;;
        ice)    friction=$FRICTION_ICE ;;
        *)      friction=$FRICTION_WET ;;
    esac
    print_info "Friction Coefficient: $friction"

    # Calculate reaction distance
    local reaction_dist=$(echo "scale=2; $velocity * $REACTION_TIME" | bc -l)
    print_info "Reaction Distance: ${reaction_dist}m"

    # Calculate braking distance
    local max_decel=$(echo "scale=2; $friction * $GRAVITY" | bc -l)
    local braking_dist=$(echo "scale=2; ($velocity * $velocity) / (2 * $max_decel)" | bc -l)
    print_info "Braking Distance: ${braking_dist}m"

    # Total safe distance (with 20% safety margin)
    local total_dist=$(echo "scale=2; ($reaction_dist + $braking_dist) * 1.2 + 2.0" | bc -l)

    print_section "Results"
    print_success "Total Safe Distance: ${total_dist}m"

    # Time gap
    local time_gap=$(echo "scale=2; $total_dist / $velocity" | bc -l)
    print_info "Equivalent Time Gap: ${time_gap}s"

    # Provide recommendations
    if (( $(echo "$time_gap < 2.0" | bc -l) )); then
        print_warning "Time gap < 2.0s: Reduce speed or increase distance"
    elif (( $(echo "$time_gap < 3.0" | bc -l) )); then
        print_info "Recommendation: Maintain at least 3-second gap"
    else
        print_success "Safe following distance achieved"
    fi

    echo ""
}

# Calculate Time-to-Collision
calc_ttc() {
    local ego_speed=${1:-90}      # km/h
    local target_speed=${2:-50}   # km/h
    local distance=${3:-50}       # meters

    print_section "Time-to-Collision Calculation"
    print_info "Ego Vehicle Speed: $ego_speed km/h"
    print_info "Target Vehicle Speed: $target_speed km/h"
    print_info "Current Distance: ${distance}m"

    # Convert to m/s
    local v_ego=$(echo "scale=2; $ego_speed / 3.6" | bc -l)
    local v_target=$(echo "scale=2; $target_speed / 3.6" | bc -l)

    # Calculate relative velocity
    local v_rel=$(echo "scale=2; $v_ego - $v_target" | bc -l)
    print_info "Relative Velocity: ${v_rel} m/s"

    # Calculate safe distance at current speed
    local friction=$FRICTION_DRY
    local reaction_dist=$(echo "scale=2; $v_ego * $REACTION_TIME" | bc -l)
    local max_decel=$(echo "scale=2; $friction * $GRAVITY" | bc -l)
    local braking_dist=$(echo "scale=2; ($v_ego * $v_ego) / (2 * $max_decel)" | bc -l)
    local safe_dist=$(echo "scale=2; ($reaction_dist + $braking_dist) * 1.2 + 2.0" | bc -l)

    # Calculate TTC
    local ttc
    if (( $(echo "$v_rel <= 0" | bc -l) )); then
        ttc="∞"
        print_info "TTC: ∞ (not approaching)"
    else
        ttc=$(echo "scale=2; ($distance - $safe_dist) / $v_rel" | bc -l)
        print_info "TTC: ${ttc}s"
    fi

    print_section "Risk Assessment"

    # Determine risk level
    if [ "$ttc" = "∞" ]; then
        print_success "Risk: NONE"
    elif (( $(echo "$ttc < 0" | bc -l) )) || (( $(echo "$distance < $safe_dist" | bc -l) )); then
        print_error "Risk: CRITICAL - EMERGENCY BRAKING REQUIRED"
    elif (( $(echo "$ttc < 0.8" | bc -l) )); then
        print_error "Risk: CRITICAL - Activate Emergency Braking"
    elif (( $(echo "$ttc < 1.5" | bc -l) )); then
        print_error "Risk: HIGH - Alert driver + partial braking"
    elif (( $(echo "$ttc < 2.5" | bc -l) )); then
        print_warning "Risk: MEDIUM - Alert driver"
    elif (( $(echo "$ttc < 4.0" | bc -l) )); then
        print_warning "Risk: LOW - Visual warning"
    else
        print_success "Risk: NONE - Monitor only"
    fi

    print_info "Safe Distance: ${safe_dist}m"
    print_info "Distance Margin: $(echo "scale=2; $distance - $safe_dist" | bc -l)m"

    echo ""
}

# Check lane position
lane_check() {
    local position=${1:-0.0}   # meters from center
    local angle=${2:-0.0}      # degrees

    print_section "Lane Departure Check"
    print_info "Lateral Position: ${position}m from center"
    print_info "Heading Angle: ${angle}°"

    # Convert angle to radians
    local angle_rad=$(echo "scale=4; $angle * 3.14159 / 180" | bc -l)

    # Assume lane width of 3.5m
    local lane_width=3.5
    local half_width=$(echo "scale=2; $lane_width / 2" | bc -l)

    # Calculate distance to lane edges
    local dist_left=$(echo "scale=2; $half_width - $position" | bc -l)
    local dist_right=$(echo "scale=2; $half_width + $position" | bc -l)

    print_info "Distance to Left Edge: ${dist_left}m"
    print_info "Distance to Right Edge: ${dist_right}m"

    # Estimate vehicle speed (assuming 100 km/h)
    local velocity=27.78  # m/s

    # Calculate Time-to-Lane-Crossing
    local lateral_vel=$(echo "scale=4; $velocity * s($angle_rad)" | bc -l)
    local lateral_vel_abs=$(echo "scale=4; sqrt($lateral_vel * $lateral_vel)" | bc -l)

    local tlc
    if (( $(echo "$lateral_vel_abs < 0.01" | bc -l) )); then
        tlc="∞"
    elif (( $(echo "$angle > 0" | bc -l) )); then
        # Drifting left
        tlc=$(echo "scale=2; $dist_left / $lateral_vel_abs" | bc -l)
    else
        # Drifting right
        tlc=$(echo "scale=2; $dist_right / $lateral_vel_abs" | bc -l)
    fi

    print_section "Lane Departure Status"

    if [ "$tlc" = "∞" ]; then
        print_success "Status: Centered - No departure detected"
    elif (( $(echo "$tlc < 0.5" | bc -l) )); then
        print_error "Status: IMMINENT DEPARTURE - Audible warning + corrective steering"
    elif (( $(echo "$tlc < 1.0" | bc -l) )); then
        print_warning "Status: Departing - Haptic warning (steering vibration)"
    elif (( $(echo "$tlc < 2.0" | bc -l) )); then
        print_warning "Status: Drifting - Visual warning"
    else
        print_success "Status: Normal - Within lane"
    fi

    if [ "$tlc" != "∞" ]; then
        print_info "Time to Lane Crossing: ${tlc}s"
    fi

    echo ""
}

# Simulate collision scenario
simulate_collision() {
    local ego_speed=${1:-90}      # km/h
    local target_speed=${2:-50}   # km/h
    local distance=${3:-30}       # meters

    print_section "Collision Scenario Simulation"
    print_info "Ego Speed: $ego_speed km/h"
    print_info "Target Speed: $target_speed km/h"
    print_info "Initial Distance: ${distance}m"

    # Convert to m/s
    local v_ego=$(echo "scale=2; $ego_speed / 3.6" | bc -l)
    local v_target=$(echo "scale=2; $target_speed / 3.6" | bc -l)

    # Calculate relative velocity
    local v_rel=$(echo "scale=2; $v_ego - $v_target" | bc -l)

    # Simulate time steps
    print_section "Time Evolution"

    local t=0
    local d=$distance
    local step=0.5  # 0.5 second steps

    while (( $(echo "$d > 0 && $t < 10" | bc -l) )); do
        # Calculate TTC at this moment
        local friction=$FRICTION_DRY
        local max_decel=$(echo "scale=2; $friction * $GRAVITY" | bc -l)
        local safe_dist=$(echo "scale=2; $v_ego * $REACTION_TIME + ($v_ego * $v_ego) / (2 * $max_decel)" | bc -l)

        local ttc=$(echo "scale=2; ($d - $safe_dist) / $v_rel" | bc -l)

        printf "  t=%.1fs: Distance=%.1fm, TTC=%.2fs" $t $d $ttc

        # Determine action
        if (( $(echo "$ttc < 0.8" | bc -l) )); then
            echo -e " ${RED}[EMERGENCY BRAKE]${RESET}"
            # Apply emergency braking
            v_ego=$(echo "scale=2; $v_ego - $max_decel * $step" | bc -l)
            if (( $(echo "$v_ego < 0" | bc -l) )); then
                v_ego=0
            fi
        elif (( $(echo "$ttc < 1.5" | bc -l) )); then
            echo -e " ${YELLOW}[PARTIAL BRAKE]${RESET}"
            # Apply partial braking
            local partial_decel=$(echo "scale=2; $max_decel * 0.6" | bc -l)
            v_ego=$(echo "scale=2; $v_ego - $partial_decel * $step" | bc -l)
        else
            echo ""
        fi

        # Update distance
        d=$(echo "scale=2; $d - $v_rel * $step" | bc -l)
        t=$(echo "scale=2; $t + $step" | bc -l)
    done

    print_section "Simulation Result"

    if (( $(echo "$d <= 0" | bc -l) )); then
        print_error "COLLISION OCCURRED"
        print_info "Consider: Increase following distance or enable AEB"
    elif (( $(echo "$v_ego == 0" | bc -l) )); then
        print_success "COLLISION AVOIDED - Vehicle stopped"
        print_info "Final Distance: ${d}m"
    else
        print_success "SAFE - Distance maintained"
        print_info "Final Distance: ${d}m"
    fi

    echo ""
}

# Generate ADAS report
generate_report() {
    local format=${1:-text}
    local output=${2:-}

    print_section "ADAS System Report"

    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    if [ "$format" = "json" ]; then
        # JSON format
        cat > ${output:-/dev/stdout} <<EOF
{
  "standard": "WIA-AUTO-002",
  "version": "1.0.0",
  "timestamp": "$timestamp",
  "system": {
    "sensors": {
      "lidar": { "enabled": true, "range": 200, "status": "healthy" },
      "radar": { "enabled": true, "range": 150, "status": "healthy" },
      "camera": { "enabled": true, "resolution": "1920x1080", "status": "healthy" },
      "ultrasonic": { "enabled": true, "range": 5, "status": "healthy" }
    },
    "features": {
      "acc": { "enabled": true, "status": "active" },
      "lka": { "enabled": true, "status": "active" },
      "aeb": { "enabled": true, "status": "armed" },
      "fcw": { "enabled": true, "status": "monitoring" },
      "ldw": { "enabled": true, "status": "monitoring" }
    },
    "safety_level": "SAE_LEVEL_2",
    "health": "healthy"
  },
  "constants": {
    "gravity": $GRAVITY,
    "reaction_time": $REACTION_TIME,
    "friction": {
      "dry": $FRICTION_DRY,
      "wet": $FRICTION_WET,
      "snow": $FRICTION_SNOW,
      "ice": $FRICTION_ICE
    }
  }
}
EOF
    else
        # Text format
        print_info "Standard: WIA-AUTO-002 v1.0.0"
        print_info "Timestamp: $timestamp"
        echo ""
        print_success "System Status: HEALTHY"
        print_success "Sensors: LiDAR, Radar, Camera, Ultrasonic - All Operational"
        print_success "Features: ACC, LKA, AEB, FCW, LDW - All Active"
        print_success "Safety Level: SAE Level 2 (Partial Automation)"
    fi

    if [ -n "$output" ] && [ "$format" = "json" ]; then
        print_success "Report saved to: $output"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-auto-002 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-distance            Calculate safe following distance"
    echo "    --speed <km/h>         Vehicle speed (default: 100 km/h)"
    echo "    --road-condition <str> Road condition: dry, wet, snow, ice (default: dry)"
    echo ""
    echo "  calc-ttc                 Calculate Time-to-Collision"
    echo "    --ego-speed <km/h>     Ego vehicle speed (default: 90 km/h)"
    echo "    --target-speed <km/h>  Target vehicle speed (default: 50 km/h)"
    echo "    --distance <m>         Current distance (default: 50 m)"
    echo ""
    echo "  lane-check               Check lane departure status"
    echo "    --position <m>         Lateral position from center (default: 0.0 m)"
    echo "    --angle <deg>          Heading angle (default: 0.0°)"
    echo ""
    echo "  simulate-collision       Simulate collision scenario"
    echo "    --ego-speed <km/h>     Ego vehicle speed (default: 90 km/h)"
    echo "    --target-speed <km/h>  Target vehicle speed (default: 50 km/h)"
    echo "    --distance <m>         Initial distance (default: 30 m)"
    echo ""
    echo "  report                   Generate ADAS system report"
    echo "    --format <str>         Output format: text, json (default: text)"
    echo "    --output <file>        Output file (default: stdout)"
    echo ""
    echo "  detect                   Detect objects from sensor data (placeholder)"
    echo "    --sensor <type>        Sensor type: lidar, radar, camera"
    echo "    --data <file>          Sensor data file (JSON)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-auto-002 calc-distance --speed 120 --road-condition wet"
    echo "  wia-auto-002 calc-ttc --ego-speed 100 --target-speed 60 --distance 40"
    echo "  wia-auto-002 lane-check --position 0.5 --angle 2.5"
    echo "  wia-auto-002 simulate-collision --ego-speed 90 --target-speed 50 --distance 30"
    echo "  wia-auto-002 report --format json --output adas-report.json"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-AUTO-002 ADAS CLI Tool"
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
    calc-distance)
        SPEED=100
        ROAD_CONDITION="dry"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --speed) SPEED=$2; shift 2 ;;
                --road-condition) ROAD_CONDITION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_distance "$SPEED" "$ROAD_CONDITION"
        ;;

    calc-ttc)
        EGO_SPEED=90
        TARGET_SPEED=50
        DISTANCE=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --ego-speed) EGO_SPEED=$2; shift 2 ;;
                --target-speed) TARGET_SPEED=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_ttc "$EGO_SPEED" "$TARGET_SPEED" "$DISTANCE"
        ;;

    lane-check)
        POSITION=0.0
        ANGLE=0.0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --position) POSITION=$2; shift 2 ;;
                --angle) ANGLE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        lane_check "$POSITION" "$ANGLE"
        ;;

    simulate-collision)
        EGO_SPEED=90
        TARGET_SPEED=50
        DISTANCE=30

        while [[ $# -gt 0 ]]; do
            case $1 in
                --ego-speed) EGO_SPEED=$2; shift 2 ;;
                --target-speed) TARGET_SPEED=$2; shift 2 ;;
                --distance) DISTANCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        simulate_collision "$EGO_SPEED" "$TARGET_SPEED" "$DISTANCE"
        ;;

    report)
        FORMAT="text"
        OUTPUT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --format) FORMAT=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_report "$FORMAT" "$OUTPUT"
        ;;

    detect)
        print_header
        print_warning "Object detection from sensor data: Not yet implemented"
        print_info "This feature requires integration with actual sensor hardware/simulators"
        echo ""
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-auto-002 help' for usage information"
        exit 1
        ;;
esac

exit 0
