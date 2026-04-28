#!/bin/bash

################################################################################
# WIA-IND-014: Smart Gym CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Fitness Technology Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to smart gym features including
# member management, equipment tracking, workout planning, and analytics.
################################################################################

set -e

# Colors for output
INDIGO='\033[38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.smart-gym.wia.com}"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🏋️  WIA-IND-014: Smart Gym CLI Tool                  ║"
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

# Calculate One-Rep Max using multiple formulas
calc_one_rep_max() {
    local weight=$1
    local reps=$2

    print_section "One-Rep Max Calculation"
    print_info "Weight: ${weight} kg, Reps: ${reps}"
    echo ""

    # Epley Formula
    local epley=$(echo "scale=2; $weight * (1 + $reps / 30)" | bc)
    echo -e "${CYAN}Epley Formula:${RESET}     ${epley} kg"

    # Brzycki Formula
    local brzycki=$(echo "scale=2; $weight * (36 / (37 - $reps))" | bc)
    echo -e "${CYAN}Brzycki Formula:${RESET}   ${brzycki} kg"

    # Lombardi Formula
    local lombardi=$(echo "scale=2; $weight * (e(l($reps) * 0.1))" | bc -l)
    echo -e "${CYAN}Lombardi Formula:${RESET}  ${lombardi} kg"

    # Average
    local average=$(echo "scale=2; ($epley + $brzycki + $lombardi) / 3" | bc)
    echo -e "\n${GREEN}Average 1RM:${RESET}       ${average} kg"
}

# Calculate training volume
calc_training_volume() {
    local weight=$1
    local reps=$2
    local sets=$3

    local volume=$(echo "scale=2; $weight * $reps * $sets" | bc)

    print_section "Training Volume Calculation"
    echo -e "${CYAN}Weight:${RESET}   ${weight} kg"
    echo -e "${CYAN}Reps:${RESET}     ${reps}"
    echo -e "${CYAN}Sets:${RESET}     ${sets}"
    echo -e "\n${GREEN}Total Volume:${RESET} ${volume} kg"
}

# Calculate calorie burn for workout
calc_calorie_burn() {
    local activity=$1
    local duration=$2  # minutes
    local weight=$3    # kg

    print_section "Calorie Burn Calculation"

    # MET values for different activities
    local met=0
    case $activity in
        "weights"|"strength")
            met=3.5
            ;;
        "cardio-light")
            met=5.0
            ;;
        "cardio-moderate")
            met=7.0
            ;;
        "cardio-vigorous")
            met=10.0
            ;;
        "hiit")
            met=12.0
            ;;
        "yoga")
            met=2.5
            ;;
        *)
            print_error "Unknown activity type"
            return 1
            ;;
    esac

    # Calories = MET × Weight(kg) × Duration(hours)
    local duration_hours=$(echo "scale=4; $duration / 60" | bc)
    local calories=$(echo "scale=1; $met * $weight * $duration_hours" | bc)

    echo -e "${CYAN}Activity:${RESET}       ${activity}"
    echo -e "${CYAN}Duration:${RESET}       ${duration} minutes"
    echo -e "${CYAN}Weight:${RESET}         ${weight} kg"
    echo -e "${CYAN}MET Value:${RESET}      ${met}"
    echo -e "\n${GREEN}Calories Burned:${RESET} ${calories} kcal"
}

# Member check-in
member_checkin() {
    local member_id=$1
    local method=${2:-rfid}

    print_section "Member Check-in"
    print_info "Member ID: $member_id"
    print_info "Method: $method"

    # Simulate API call
    sleep 0.5

    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    echo ""
    print_success "Check-in successful!"
    echo -e "${CYAN}Timestamp:${RESET}      ${timestamp}"
    echo -e "${CYAN}Location:${RESET}       Main Entrance"

    # Simulate member info
    echo -e "\n${INDIGO}Member Information:${RESET}"
    echo -e "${GRAY}  Name: John Doe${RESET}"
    echo -e "${GRAY}  Membership: Premium (45 days remaining)${RESET}"
    echo -e "${GRAY}  Current Streak: 7 days${RESET}"

    echo -e "\n${INDIGO}Today's Recommended Workout:${RESET}"
    echo -e "${GRAY}  • Upper Body Strength Training${RESET}"
    echo -e "${GRAY}  • Duration: 60 minutes${RESET}"
    echo -e "${GRAY}  • Focus: Chest & Back${RESET}"
}

# Start workout session
workout_start() {
    local member_id=$1
    local equipment_id=${2:-MANUAL}
    local duration=${3:-60}

    print_section "Starting Workout Session"
    print_info "Member: $member_id"
    print_info "Equipment: $equipment_id"
    print_info "Planned Duration: ${duration} minutes"

    # Generate session ID
    local session_id="SESSION-$(date +%Y%m%d)-$(printf '%04d' $RANDOM)"

    echo ""
    print_success "Workout session started!"
    echo -e "${CYAN}Session ID:${RESET}     ${session_id}"
    echo -e "${CYAN}Start Time:${RESET}     $(date '+%Y-%m-%d %H:%M:%S')"

    # Save session info to temp file for monitoring
    echo "$session_id|$member_id|$(date +%s)|$duration" > /tmp/wia-ind-014-session.txt

    echo -e "\n${GRAY}Tip: Use 'wia-ind-014 workout monitor --session $session_id' to track progress${RESET}"
}

# Monitor live workout
workout_monitor() {
    local session_id=$1

    print_section "Live Workout Monitor"
    print_info "Session: $session_id"

    # Check if session file exists
    if [ ! -f /tmp/wia-ind-014-session.txt ]; then
        print_error "No active session found"
        return 1
    fi

    # Read session info
    IFS='|' read -r sid member_id start_time duration < /tmp/wia-ind-014-session.txt

    if [ "$sid" != "$session_id" ]; then
        print_error "Session ID mismatch"
        return 1
    fi

    local current_time=$(date +%s)
    local elapsed=$((current_time - start_time))
    local elapsed_min=$((elapsed / 60))

    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${CYAN}║                    LIVE WORKOUT METRICS                        ║${RESET}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${RESET}"

    # Simulate live data
    local hr=$((140 + RANDOM % 30))
    local calories=$((elapsed_min * 8))
    local exercises=$((RANDOM % 5 + 3))
    local sets=$((RANDOM % 10 + 8))

    echo ""
    echo -e "${INDIGO}Time:${RESET}           ${elapsed_min} / ${duration} minutes"
    echo -e "${INDIGO}Heart Rate:${RESET}     ${hr} bpm"
    echo -e "${INDIGO}Calories:${RESET}       ${calories} kcal"
    echo -e "${INDIGO}Exercises:${RESET}      ${exercises}"
    echo -e "${INDIGO}Sets:${RESET}           ${sets}"

    # Progress bar
    local progress=$((elapsed_min * 100 / duration))
    local filled=$((progress / 2))
    local empty=$((50 - filled))

    echo -e "\n${CYAN}Progress:${RESET}"
    printf "["
    printf "%${filled}s" | tr ' ' '▓'
    printf "%${empty}s" | tr ' ' '░'
    printf "] %d%%\n" "$progress"
}

# Generate workout plan
plan_create() {
    local member_id=$1
    local goal=${2:-strength}
    local weeks=${3:-8}

    print_section "Generating AI Workout Plan"
    print_info "Member: $member_id"
    print_info "Goal: $goal"
    print_info "Duration: $weeks weeks"

    # Simulate AI processing
    echo ""
    echo -e "${GRAY}Analyzing member profile...${RESET}"
    sleep 0.5
    echo -e "${GRAY}Calculating optimal progression...${RESET}"
    sleep 0.5
    echo -e "${GRAY}Generating personalized plan...${RESET}"
    sleep 0.5

    print_success "Workout plan generated!"

    local plan_id="PLAN-$(date +%Y%m%d)-$(printf '%04d' $RANDOM)"
    echo -e "\n${CYAN}Plan ID:${RESET}        ${plan_id}"
    echo -e "${CYAN}Goal:${RESET}           ${goal}"
    echo -e "${CYAN}Duration:${RESET}       ${weeks} weeks"
    echo -e "${CYAN}Frequency:${RESET}      4 days/week"

    echo -e "\n${INDIGO}Week 1 Preview:${RESET}"
    echo -e "${GRAY}  Monday:    Upper Body Strength (60 min)${RESET}"
    echo -e "${GRAY}  Tuesday:   Lower Body Power (55 min)${RESET}"
    echo -e "${GRAY}  Thursday:  Upper Body Hypertrophy (65 min)${RESET}"
    echo -e "${GRAY}  Saturday:  Full Body Conditioning (50 min)${RESET}"

    echo -e "\n${GRAY}Plan includes:${RESET}"
    echo -e "${GRAY}  • Progressive overload protocol${RESET}"
    echo -e "${GRAY}  • Deload week (week 4)${RESET}"
    echo -e "${GRAY}  • Form analysis checkpoints${RESET}"
    echo -e "${GRAY}  • Injury prevention exercises${RESET}"
}

# Facility status
facility_status() {
    local live=${1:-false}

    print_section "Facility Status"

    if [ "$live" = "true" ] || [ "$live" = "--live" ]; then
        echo -e "${GRAY}Live monitoring mode (Ctrl+C to exit)...${RESET}\n"

        while true; do
            # Clear screen and reposition cursor
            tput clear

            print_header

            # Simulate live data
            local occupancy=$((50 + RANDOM % 50))
            local temp=$((20 + RANDOM % 4))
            local humidity=$((45 + RANDOM % 20))
            local co2=$((400 + RANDOM % 400))

            echo -e "${INDIGO}Current Occupancy:${RESET}  ${occupancy}/150 members (${occupancy}%)"
            echo ""

            # Occupancy bar
            local filled=$((occupancy / 2))
            local empty=$((50 - filled))
            printf "["
            if [ $occupancy -lt 60 ]; then
                printf "${GREEN}"
            elif [ $occupancy -lt 85 ]; then
                printf "${YELLOW}"
            else
                printf "${RED}"
            fi
            printf "%${filled}s" | tr ' ' '▓'
            printf "${RESET}"
            printf "%${empty}s" | tr ' ' '░'
            printf "]\n\n"

            echo -e "${INDIGO}Environmental Conditions:${RESET}"
            echo -e "  Temperature:  ${temp}°C"
            echo -e "  Humidity:     ${humidity}%"
            echo -e "  CO2 Level:    ${co2} ppm"

            echo -e "\n${INDIGO}Equipment Availability:${RESET}"
            echo -e "  Treadmills:   $((12 + RANDOM % 8))/20 available"
            echo -e "  Bikes:        $((8 + RANDOM % 7))/15 available"
            echo -e "  Rowers:       $((6 + RANDOM % 4))/10 available"

            echo -e "\n${GRAY}Updated: $(date '+%Y-%m-%d %H:%M:%S')${RESET}"

            sleep 2
        done
    else
        # Single snapshot
        echo -e "${INDIGO}Current Occupancy:${RESET}  65/150 members (43%)"
        echo -e "\n${INDIGO}Environmental Conditions:${RESET}"
        echo -e "  Temperature:  21°C"
        echo -e "  Humidity:     52%"
        echo -e "  CO2 Level:    650 ppm"
        echo -e "  Air Quality:  Good"

        echo -e "\n${INDIGO}Equipment Availability:${RESET}"
        echo -e "  Treadmills:   15/20 available"
        echo -e "  Bikes:        10/15 available"
        echo -e "  Rowers:       8/10 available"
        echo -e "  Ellipticals:  6/8 available"

        echo -e "\n${INDIGO}Peak Hours Today:${RESET}"
        echo -e "  Morning:      06:00-08:00 (avg 85 members)"
        echo -e "  Evening:      17:00-20:00 (avg 120 members)"
    fi
}

# Equipment maintenance check
equipment_maintenance() {
    local check_all=${1:-false}

    print_section "Equipment Maintenance Check"

    if [ "$check_all" = "--check-all" ]; then
        echo -e "${GRAY}Scanning all equipment...${RESET}\n"

        echo -e "${CYAN}Equipment ID    Type          Hours    Status      Next Service${RESET}"
        echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"

        # Simulate equipment list
        echo -e "TREAD-001       Treadmill     2,450    ${GREEN}Good${RESET}        In 50 hours"
        echo -e "TREAD-002       Treadmill     3,890    ${YELLOW}Fair${RESET}        ${YELLOW}In 10 hours${RESET}"
        echo -e "TREAD-003       Treadmill     5,120    ${RED}Poor${RESET}        ${RED}Overdue!${RESET}"
        echo -e "BIKE-001        Spin Bike     1,820    ${GREEN}Excellent${RESET}   In 180 hours"
        echo -e "BIKE-002        Spin Bike     2,340    ${GREEN}Good${RESET}        In 160 hours"
        echo -e "ROW-001         Rower         980      ${GREEN}Excellent${RESET}   In 520 hours"
        echo -e "CABLE-001       Cable Mach    4,200    ${YELLOW}Fair${RESET}        ${YELLOW}In 30 hours${RESET}"

        echo -e "\n${RED}⚠ Action Required:${RESET}"
        echo -e "  • TREAD-003: Belt replacement needed"
        echo -e "  • TREAD-002: General service recommended within 10 hours"
        echo -e "  • CABLE-001: Cable inspection needed within 30 hours"
    else
        # Single equipment check
        local equipment_id=${2:-TREAD-001}

        echo -e "${CYAN}Equipment:${RESET}      $equipment_id"
        echo -e "${CYAN}Type:${RESET}           Treadmill"
        echo -e "${CYAN}Status:${RESET}         ${GREEN}Good${RESET}"
        echo -e "${CYAN}Total Hours:${RESET}    2,450 hours"
        echo -e "${CYAN}Last Service:${RESET}   2025-11-15"
        echo -e "${CYAN}Next Service:${RESET}   2026-01-15 (in 50 hours)"
        echo -e "${CYAN}Health Score:${RESET}   ${GREEN}85/100${RESET}"

        echo -e "\n${INDIGO}Recent Maintenance History:${RESET}"
        echo -e "${GRAY}  2025-11-15: Belt alignment, motor inspection${RESET}"
        echo -e "${GRAY}  2025-08-20: Belt replacement, calibration${RESET}"
        echo -e "${GRAY}  2025-05-10: General service, lubrication${RESET}"
    fi
}

# Analytics report
analytics_member() {
    local member_id=$1
    local period=${2:-30days}

    print_section "Member Analytics Report"
    print_info "Member: $member_id"
    print_info "Period: $period"

    echo ""
    echo -e "${INDIGO}Workout Statistics:${RESET}"
    echo -e "  Total Workouts:        24"
    echo -e "  Total Time:            1,440 minutes (24 hours)"
    echo -e "  Avg Duration:          60 minutes"
    echo -e "  Workouts/Week:         6"

    echo -e "\n${INDIGO}Performance Metrics:${RESET}"
    echo -e "  Total Calories:        12,500 kcal"
    echo -e "  Total Volume:          125,000 kg"
    echo -e "  Avg Heart Rate:        145 bpm"
    echo -e "  Strength Progress:     ${GREEN}+12%${RESET}"
    echo -e "  Cardio Progress:       ${GREEN}+8%${RESET}"

    echo -e "\n${INDIGO}Attendance:${RESET}"
    echo -e "  Days Attended:         24/30 days"
    echo -e "  Current Streak:        7 days"
    echo -e "  Longest Streak:        14 days"
    echo -e "  Consistency Score:     ${GREEN}92/100${RESET}"

    echo -e "\n${INDIGO}Personal Records (Last 30 days):${RESET}"
    echo -e "${GREEN}  • Squat:       120 kg → 130 kg (+8%)${RESET}"
    echo -e "${GREEN}  • Bench Press: 85 kg → 90 kg (+6%)${RESET}"
    echo -e "${GREEN}  • Deadlift:    140 kg → 150 kg (+7%)${RESET}"

    echo -e "\n${INDIGO}Body Composition:${RESET}"
    echo -e "  Weight:          75.5 kg (${GREEN}-1.2 kg${RESET})"
    echo -e "  Body Fat:        15.2% (${GREEN}-1.5%${RESET})"
    echo -e "  Muscle Mass:     38.5 kg (${GREEN}+0.8 kg${RESET})"
}

# Virtual class list
virtual_list_classes() {
    local date=${1:-today}

    print_section "Virtual Classes Schedule"
    print_info "Date: $date"

    echo ""
    echo -e "${CYAN}Time   Class Name              Instructor    Type    Spots${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "06:00  Morning Yoga            Sarah Lee     Yoga    12/25"
    echo -e "07:00  HIIT Cardio Blast       Mike Chen     HIIT    18/20"
    echo -e "09:00  Strength Fundamentals   John Smith    Strength 10/15"
    echo -e "12:00  Lunch Break Stretch     Emma Wilson   Yoga     8/20"
    echo -e "17:00  Power Cycling           Tom Brown     Spin    ${YELLOW}20/20${RESET}"
    echo -e "18:00  Evening Pilates         Lisa Park     Pilates 14/25"
    echo -e "19:00  Total Body Strength     Alex Kim      Strength 16/20"

    echo -e "\n${GRAY}Tip: Use 'wia-ind-014 virtual join --class CLASS-ID --member MEMBER-ID'${RESET}"
}

# Show help
show_help() {
    print_header

    echo "Usage: wia-ind-014 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo -e "${CYAN}Member Management:${RESET}"
    echo "  member checkin --id <ID> [--method rfid|app|biometric]"
    echo "  member checkout --id <ID>"
    echo ""
    echo -e "${CYAN}Workout Sessions:${RESET}"
    echo "  workout start --member <ID> [--equipment <ID>] [--duration <min>]"
    echo "  workout monitor --session <ID>"
    echo "  workout complete --session <ID>"
    echo ""
    echo -e "${CYAN}Workout Planning:${RESET}"
    echo "  plan create --member <ID> [--goal strength|cardio|weight-loss] [--weeks 8]"
    echo "  plan view --id <PLAN-ID>"
    echo ""
    echo -e "${CYAN}Facility Management:${RESET}"
    echo "  facility status [--live]"
    echo "  facility register --name <NAME> --capacity <NUM>"
    echo ""
    echo -e "${CYAN}Equipment:${RESET}"
    echo "  equipment add --type <TYPE> --id <ID>"
    echo "  equipment maintenance [--check-all]"
    echo "  equipment status --id <ID>"
    echo ""
    echo -e "${CYAN}Analytics:${RESET}"
    echo "  analytics member --id <ID> [--period 7days|30days|90days]"
    echo "  analytics facility [--period week|month]"
    echo ""
    echo -e "${CYAN}Virtual Training:${RESET}"
    echo "  virtual list-classes [--date today|tomorrow|YYYY-MM-DD]"
    echo "  virtual join --class <ID> --member <ID>"
    echo ""
    echo -e "${CYAN}Calculations:${RESET}"
    echo "  calc --metric one-rep-max --weight <KG> --reps <NUM>"
    echo "  calc --metric training-volume --weight <KG> --reps <NUM> --sets <NUM>"
    echo "  calc --metric calorie-burn --activity <TYPE> --duration <MIN> --weight <KG>"
    echo ""
    echo -e "${CYAN}Other:${RESET}"
    echo "  version"
    echo "  help"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
}

# Main command router
main() {
    case "$1" in
        version)
            echo "WIA-IND-014 Smart Gym CLI v$VERSION"
            ;;
        help|--help|-h)
            show_help
            ;;
        member)
            case "$2" in
                checkin)
                    shift 2
                    member_id=""
                    method="rfid"
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --id) member_id="$2"; shift 2 ;;
                            --method) method="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    member_checkin "$member_id" "$method"
                    ;;
                *) print_error "Unknown member command: $2" ;;
            esac
            ;;
        workout)
            case "$2" in
                start)
                    shift 2
                    member_id=""
                    equipment_id="MANUAL"
                    duration=60
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --member) member_id="$2"; shift 2 ;;
                            --equipment) equipment_id="$2"; shift 2 ;;
                            --duration) duration="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    workout_start "$member_id" "$equipment_id" "$duration"
                    ;;
                monitor)
                    shift 2
                    session_id=""
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --session) session_id="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    workout_monitor "$session_id"
                    ;;
                *) print_error "Unknown workout command: $2" ;;
            esac
            ;;
        plan)
            case "$2" in
                create)
                    shift 2
                    member_id=""
                    goal="strength"
                    weeks=8
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --member) member_id="$2"; shift 2 ;;
                            --goal) goal="$2"; shift 2 ;;
                            --weeks) weeks="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    plan_create "$member_id" "$goal" "$weeks"
                    ;;
                *) print_error "Unknown plan command: $2" ;;
            esac
            ;;
        facility)
            case "$2" in
                status)
                    facility_status "$3"
                    ;;
                *) print_error "Unknown facility command: $2" ;;
            esac
            ;;
        equipment)
            case "$2" in
                maintenance)
                    equipment_maintenance "$3"
                    ;;
                *) print_error "Unknown equipment command: $2" ;;
            esac
            ;;
        analytics)
            case "$2" in
                member)
                    shift 2
                    member_id=""
                    period="30days"
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --id) member_id="$2"; shift 2 ;;
                            --period) period="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    analytics_member "$member_id" "$period"
                    ;;
                *) print_error "Unknown analytics command: $2" ;;
            esac
            ;;
        virtual)
            case "$2" in
                list-classes)
                    virtual_list_classes "$3"
                    ;;
                *) print_error "Unknown virtual command: $2" ;;
            esac
            ;;
        calc)
            shift
            metric=""
            weight=0
            reps=0
            sets=0
            activity=""
            duration=0
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --metric) metric="$2"; shift 2 ;;
                    --weight) weight="$2"; shift 2 ;;
                    --reps) reps="$2"; shift 2 ;;
                    --sets) sets="$2"; shift 2 ;;
                    --activity) activity="$2"; shift 2 ;;
                    --duration) duration="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done

            case "$metric" in
                one-rep-max)
                    calc_one_rep_max "$weight" "$reps"
                    ;;
                training-volume)
                    calc_training_volume "$weight" "$reps" "$sets"
                    ;;
                calorie-burn)
                    calc_calorie_burn "$activity" "$duration" "$weight"
                    ;;
                *) print_error "Unknown metric: $metric" ;;
            esac
            ;;
        *)
            print_error "Unknown command: $1"
            echo "Run 'wia-ind-014 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
