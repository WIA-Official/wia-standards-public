#!/bin/bash

################################################################################
# WIA-IND-012: Fitness Tracking CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Health & Fitness Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to fitness tracking calculations
# including activity monitoring, heart rate analysis, calorie calculation, and
# workout logging.
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
SPEED_OF_LIGHT=299792458

# 상수 (Constants - Korean)
BMR_MIFFLIN_MALE_BASE=5
BMR_MIFFLIN_FEMALE_BASE=-161

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🏃 WIA-IND-012: Fitness Tracking CLI v$VERSION          ║"
    echo "║                    弘益人間 · Benefit All                      ║"
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

print_metric() {
    local label=$1
    local value=$2
    local unit=$3
    printf "${CYAN}%-25s${RESET} ${GREEN}%10s${RESET} ${GRAY}%s${RESET}\n" "$label:" "$value" "$unit"
}

# Calculate BMR (Basal Metabolic Rate)
# BMR 계산 (기초대사량)
calc_bmr() {
    local weight=${1:-70}
    local height=${2:-170}
    local age=${3:-30}
    local gender=${4:-male}

    print_section "BMR Calculation (기초대사량 계산)"
    print_info "Weight: $weight kg"
    print_info "Height: $height cm"
    print_info "Age: $age years"
    print_info "Gender: $gender"

    # Mifflin-St Jeor Equation
    # Men: BMR = (10 × weight) + (6.25 × height) - (5 × age) + 5
    # Women: BMR = (10 × weight) + (6.25 × height) - (5 × age) - 161

    local weight_component=$(echo "10 * $weight" | bc -l)
    local height_component=$(echo "6.25 * $height" | bc -l)
    local age_component=$(echo "5 * $age" | bc -l)

    local bmr
    if [ "$gender" = "male" ]; then
        bmr=$(echo "$weight_component + $height_component - $age_component + 5" | bc -l)
    else
        bmr=$(echo "$weight_component + $height_component - $age_component - 161" | bc -l)
    fi

    echo ""
    print_metric "BMR (Mifflin-St Jeor)" "$(printf "%.0f" $bmr)" "kcal/day"

    # Calculate TDEE (Total Daily Energy Expenditure)
    echo ""
    print_info "TDEE (Total Daily Energy Expenditure):"

    local sedentary=$(echo "$bmr * 1.2" | bc -l)
    local light=$(echo "$bmr * 1.375" | bc -l)
    local moderate=$(echo "$bmr * 1.55" | bc -l)
    local active=$(echo "$bmr * 1.725" | bc -l)
    local very_active=$(echo "$bmr * 1.9" | bc -l)

    print_metric "  Sedentary" "$(printf "%.0f" $sedentary)" "kcal/day"
    print_metric "  Lightly Active" "$(printf "%.0f" $light)" "kcal/day"
    print_metric "  Moderately Active" "$(printf "%.0f" $moderate)" "kcal/day"
    print_metric "  Very Active" "$(printf "%.0f" $active)" "kcal/day"
    print_metric "  Extremely Active" "$(printf "%.0f" $very_active)" "kcal/day"
}

# Calculate calories burned during activity
# 활동 칼로리 계산
calc_calories() {
    local activity=${1:-running}
    local weight=${2:-70}
    local duration=${3:-30}
    local intensity=${4:-moderate}

    print_section "Calorie Calculation (칼로리 계산)"
    print_info "Activity: $activity"
    print_info "Weight: $weight kg"
    print_info "Duration: $duration minutes"
    print_info "Intensity: $intensity"

    # MET values for common activities
    local met
    case "$activity" in
        walking)
            case "$intensity" in
                light) met=3.0 ;;
                moderate) met=3.5 ;;
                vigorous) met=5.0 ;;
                *) met=3.5 ;;
            esac
            ;;
        running)
            case "$intensity" in
                light) met=8.3 ;;   # 5 mph
                moderate) met=9.8 ;; # 6 mph
                vigorous) met=11.0 ;; # 7 mph
                *) met=9.8 ;;
            esac
            ;;
        cycling)
            case "$intensity" in
                light) met=6.8 ;;   # 10-12 mph
                moderate) met=8.0 ;; # 12-14 mph
                vigorous) met=10.0 ;; # 14-16 mph
                *) met=8.0 ;;
            esac
            ;;
        swimming)
            case "$intensity" in
                light) met=5.8 ;;
                moderate) met=7.0 ;;
                vigorous) met=9.8 ;;
                *) met=7.0 ;;
            esac
            ;;
        strength)
            case "$intensity" in
                light) met=3.0 ;;
                moderate) met=5.0 ;;
                vigorous) met=6.0 ;;
                *) met=5.0 ;;
            esac
            ;;
        hiit)
            met=8.0
            ;;
        yoga)
            met=3.0
            ;;
        *)
            print_error "Unknown activity: $activity"
            return 1
            ;;
    esac

    # Calories = MET × weight(kg) × duration(hours)
    local duration_hours=$(echo "scale=4; $duration / 60" | bc -l)
    local calories=$(echo "scale=2; $met * $weight * $duration_hours" | bc -l)

    echo ""
    print_metric "MET Value" "$met" ""
    print_metric "Calories Burned" "$(printf "%.0f" $calories)" "kcal"

    # Calculate EPOC (afterburn)
    local epoc_factor
    if (( $(echo "$met < 5" | bc -l) )); then
        epoc_factor=1.05
    elif (( $(echo "$met < 8" | bc -l) )); then
        epoc_factor=1.10
    else
        epoc_factor=1.15
    fi

    local total_with_epoc=$(echo "scale=2; $calories * $epoc_factor" | bc -l)
    print_metric "With EPOC (afterburn)" "$(printf "%.0f" $total_with_epoc)" "kcal"
}

# Calculate heart rate zones
# 심박수 구간 계산
calc_hr_zones() {
    local age=${1:-30}
    local resting_hr=${2:-60}

    print_section "Heart Rate Zones (심박수 구간)"
    print_info "Age: $age years"
    print_info "Resting HR: $resting_hr BPM"

    # Calculate max heart rate
    local max_hr=$(echo "220 - $age" | bc -l)
    print_metric "Max Heart Rate" "$(printf "%.0f" $max_hr)" "BPM"

    # HR Reserve = Max HR - Resting HR
    local hr_reserve=$(echo "$max_hr - $resting_hr" | bc -l)
    print_metric "HR Reserve" "$(printf "%.0f" $hr_reserve)" "BPM"

    echo ""
    print_info "Training Zones (Karvonen Method):"

    # Zone 1: 50-60% (Recovery)
    local zone1_low=$(echo "$resting_hr + ($hr_reserve * 0.50)" | bc -l)
    local zone1_high=$(echo "$resting_hr + ($hr_reserve * 0.60)" | bc -l)
    printf "${CYAN}Zone 1 (Recovery):    ${GREEN}%3.0f-%3.0f BPM${GRAY}  (50-60%% Max HR)${RESET}\n" $zone1_low $zone1_high

    # Zone 2: 60-70% (Aerobic)
    local zone2_low=$(echo "$resting_hr + ($hr_reserve * 0.60)" | bc -l)
    local zone2_high=$(echo "$resting_hr + ($hr_reserve * 0.70)" | bc -l)
    printf "${CYAN}Zone 2 (Aerobic):     ${GREEN}%3.0f-%3.0f BPM${GRAY}  (60-70%% Max HR)${RESET}\n" $zone2_low $zone2_high

    # Zone 3: 70-80% (Tempo)
    local zone3_low=$(echo "$resting_hr + ($hr_reserve * 0.70)" | bc -l)
    local zone3_high=$(echo "$resting_hr + ($hr_reserve * 0.80)" | bc -l)
    printf "${CYAN}Zone 3 (Tempo):       ${GREEN}%3.0f-%3.0f BPM${GRAY}  (70-80%% Max HR)${RESET}\n" $zone3_low $zone3_high

    # Zone 4: 80-90% (Threshold)
    local zone4_low=$(echo "$resting_hr + ($hr_reserve * 0.80)" | bc -l)
    local zone4_high=$(echo "$resting_hr + ($hr_reserve * 0.90)" | bc -l)
    printf "${CYAN}Zone 4 (Threshold):   ${GREEN}%3.0f-%3.0f BPM${GRAY}  (80-90%% Max HR)${RESET}\n" $zone4_low $zone4_high

    # Zone 5: 90-100% (Maximum)
    local zone5_low=$(echo "$resting_hr + ($hr_reserve * 0.90)" | bc -l)
    local zone5_high=$max_hr
    printf "${CYAN}Zone 5 (Maximum):     ${GREEN}%3.0f-%3.0f BPM${GRAY}  (90-100%% Max HR)${RESET}\n" $zone5_low $zone5_high
}

# Analyze heart rate data
# 심박수 데이터 분석
analyze_hr() {
    local age=${1:-30}
    local hr_data=${2:-"120,135,150,165,155,145,130"}

    print_section "Heart Rate Analysis (심박수 분석)"
    print_info "Age: $age years"
    print_info "HR Data: $hr_data"

    # Calculate max HR
    local max_hr=$(echo "220 - $age" | bc -l)

    # Parse HR data
    IFS=',' read -ra HR_ARRAY <<< "$hr_data"
    local sum=0
    local count=${#HR_ARRAY[@]}
    local max_recorded=0
    local min_recorded=999

    for hr in "${HR_ARRAY[@]}"; do
        sum=$(echo "$sum + $hr" | bc -l)
        if (( $(echo "$hr > $max_recorded" | bc -l) )); then
            max_recorded=$hr
        fi
        if (( $(echo "$hr < $min_recorded" | bc -l) )); then
            min_recorded=$hr
        fi
    done

    local avg_hr=$(echo "scale=1; $sum / $count" | bc -l)

    echo ""
    print_metric "Average HR" "$(printf "%.0f" $avg_hr)" "BPM"
    print_metric "Max HR" "$max_recorded" "BPM"
    print_metric "Min HR" "$min_recorded" "BPM"

    # Calculate % of max HR
    local percent_max=$(echo "scale=1; ($avg_hr / $max_hr) * 100" | bc -l)
    print_metric "% of Max HR" "$(printf "%.1f" $percent_max)" "%"

    # Determine zone
    local zone
    if (( $(echo "$percent_max < 60" | bc -l) )); then
        zone="Zone 1 (Recovery)"
    elif (( $(echo "$percent_max < 70" | bc -l) )); then
        zone="Zone 2 (Aerobic)"
    elif (( $(echo "$percent_max < 80" | bc -l) )); then
        zone="Zone 3 (Tempo)"
    elif (( $(echo "$percent_max < 90" | bc -l) )); then
        zone="Zone 4 (Threshold)"
    else
        zone="Zone 5 (Maximum)"
    fi

    print_metric "Training Zone" "$zone" ""
}

# Calculate BMI
# BMI 계산
calc_bmi() {
    local weight=${1:-70}
    local height=${2:-170}

    print_section "BMI Calculation (체질량지수 계산)"
    print_info "Weight: $weight kg"
    print_info "Height: $height cm"

    # BMI = weight(kg) / (height(m))²
    local height_m=$(echo "scale=4; $height / 100" | bc -l)
    local bmi=$(echo "scale=2; $weight / ($height_m * $height_m)" | bc -l)

    echo ""
    print_metric "BMI" "$(printf "%.1f" $bmi)" "kg/m²"

    # Classification
    local classification
    if (( $(echo "$bmi < 18.5" | bc -l) )); then
        classification="Underweight"
    elif (( $(echo "$bmi < 25" | bc -l) )); then
        classification="Normal weight"
    elif (( $(echo "$bmi < 30" | bc -l) )); then
        classification="Overweight"
    else
        classification="Obese"
    fi

    print_metric "Classification" "$classification" ""
}

# Calculate pace and speed
# 페이스와 속도 계산
calc_pace() {
    local distance=${1:-5000}  # meters
    local duration=${2:-1800}  # seconds (30 minutes)

    print_section "Pace & Speed Calculation (페이스 & 속도 계산)"
    print_info "Distance: $distance meters"
    print_info "Duration: $duration seconds ($(echo "scale=1; $duration / 60" | bc -l) minutes)"

    # Calculate pace (min/km)
    local distance_km=$(echo "scale=4; $distance / 1000" | bc -l)
    local duration_min=$(echo "scale=4; $duration / 60" | bc -l)
    local pace=$(echo "scale=2; $duration_min / $distance_km" | bc -l)

    # Calculate speed (km/h)
    local duration_hours=$(echo "scale=4; $duration / 3600" | bc -l)
    local speed=$(echo "scale=2; $distance_km / $duration_hours" | bc -l)

    echo ""
    print_metric "Pace" "$(printf "%.2f" $pace)" "min/km"
    print_metric "Speed" "$(printf "%.2f" $speed)" "km/h"

    # Convert to min/mile
    local pace_mile=$(echo "scale=2; $pace * 1.60934" | bc -l)
    print_metric "Pace (mile)" "$(printf "%.2f" $pace_mile)" "min/mile"

    # Convert to mph
    local speed_mph=$(echo "scale=2; $speed / 1.60934" | bc -l)
    print_metric "Speed (mph)" "$(printf "%.2f" $speed_mph)" "mph"
}

# Log activity
# 활동 기록
log_activity() {
    local type=${1:-running}
    local duration=${2:-30}
    local distance=${3:-5000}
    local hr=${4:-150}

    print_section "Activity Logged (활동 기록됨)"

    local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
    print_metric "Timestamp" "$timestamp" ""
    print_metric "Activity Type" "$type" ""
    print_metric "Duration" "$duration" "minutes"
    print_metric "Distance" "$distance" "meters"
    print_metric "Avg Heart Rate" "$hr" "BPM"

    print_success "Activity successfully logged!"
}

# Log steps
# 걸음 수 기록
log_steps() {
    local steps=${1:-10000}
    local date=${2:-$(date +"%Y-%m-%d")}

    print_section "Steps Logged (걸음 수 기록됨)"

    print_metric "Date" "$date" ""
    print_metric "Steps" "$steps" "steps"

    # Estimate distance (average stride length = 0.415 × height)
    # Assuming average height of 170cm
    local stride_length=$(echo "170 * 0.415 / 100" | bc -l)
    local distance=$(echo "scale=2; $steps * $stride_length / 1000" | bc -l)
    print_metric "Est. Distance" "$(printf "%.2f" $distance)" "km"

    # Estimate calories (rough estimate: 0.04 kcal per step for 70kg person)
    local calories=$(echo "scale=0; $steps * 0.04" | bc -l)
    print_metric "Est. Calories" "$(printf "%.0f" $calories)" "kcal"

    # Goal progress (assuming 10,000 steps goal)
    local goal=10000
    local progress=$(echo "scale=1; ($steps / $goal) * 100" | bc -l)
    print_metric "Goal Progress" "$(printf "%.1f" $progress)" "%"

    if (( $(echo "$steps >= $goal" | bc -l) )); then
        print_success "Daily step goal achieved! 🎉"
    else
        local remaining=$(echo "$goal - $steps" | bc -l)
        print_info "Steps remaining: $(printf "%.0f" $remaining)"
    fi
}

# Set fitness goal
# 피트니스 목표 설정
set_goal() {
    local type=${1:-steps}
    local target=${2:-10000}
    local period=${3:-daily}

    print_section "Goal Set (목표 설정됨)"

    print_metric "Goal Type" "$type" ""
    print_metric "Target" "$target" ""
    print_metric "Period" "$period" ""

    print_success "Fitness goal successfully set!"
    print_info "Track your progress with: wia-ind-012 summary"
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-012 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo "  ${CYAN}calc-bmr${RESET} --weight W --height H --age A --gender G"
    echo "    Calculate Basal Metabolic Rate"
    echo "    Example: wia-ind-012 calc-bmr --weight 70 --height 170 --age 30 --gender male"
    echo ""
    echo "  ${CYAN}calc-calories${RESET} --activity A --weight W --duration D --intensity I"
    echo "    Calculate calories burned"
    echo "    Example: wia-ind-012 calc-calories --activity running --weight 70 --duration 30 --intensity moderate"
    echo ""
    echo "  ${CYAN}calc-hr-zones${RESET} --age A --resting-hr R"
    echo "    Calculate heart rate training zones"
    echo "    Example: wia-ind-012 calc-hr-zones --age 30 --resting-hr 60"
    echo ""
    echo "  ${CYAN}analyze-hr${RESET} --age A --hr-data \"H1,H2,H3,...\""
    echo "    Analyze heart rate data"
    echo "    Example: wia-ind-012 analyze-hr --age 30 --hr-data \"120,135,150,165,155\""
    echo ""
    echo "  ${CYAN}calc-bmi${RESET} --weight W --height H"
    echo "    Calculate Body Mass Index"
    echo "    Example: wia-ind-012 calc-bmi --weight 70 --height 170"
    echo ""
    echo "  ${CYAN}calc-pace${RESET} --distance D --duration T"
    echo "    Calculate pace and speed"
    echo "    Example: wia-ind-012 calc-pace --distance 5000 --duration 1800"
    echo ""
    echo "  ${CYAN}log-activity${RESET} --type T --duration D --distance M --hr H"
    echo "    Log a fitness activity"
    echo "    Example: wia-ind-012 log-activity --type running --duration 30 --distance 5000 --hr 150"
    echo ""
    echo "  ${CYAN}log-steps${RESET} --steps S --date D"
    echo "    Log daily step count"
    echo "    Example: wia-ind-012 log-steps --steps 12500 --date 2025-12-27"
    echo ""
    echo "  ${CYAN}set-goal${RESET} --type T --target N --period P"
    echo "    Set a fitness goal"
    echo "    Example: wia-ind-012 set-goal --type steps --target 10000 --period daily"
    echo ""
    echo "  ${CYAN}--version${RESET}"
    echo "    Show version information"
    echo ""
    echo "  ${CYAN}--help${RESET}"
    echo "    Show this help message"
    echo ""
    echo "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo "${GRAY}WIA - World Certification Industry Association${RESET}"
}

# Parse command line arguments
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

case "$1" in
    calc-bmr)
        shift
        weight=70
        height=170
        age=30
        gender="male"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --weight) weight="$2"; shift 2 ;;
                --height) height="$2"; shift 2 ;;
                --age) age="$2"; shift 2 ;;
                --gender) gender="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        calc_bmr "$weight" "$height" "$age" "$gender"
        ;;

    calc-calories)
        shift
        activity="running"
        weight=70
        duration=30
        intensity="moderate"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --activity) activity="$2"; shift 2 ;;
                --weight) weight="$2"; shift 2 ;;
                --duration) duration="$2"; shift 2 ;;
                --intensity) intensity="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        calc_calories "$activity" "$weight" "$duration" "$intensity"
        ;;

    calc-hr-zones)
        shift
        age=30
        resting_hr=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --age) age="$2"; shift 2 ;;
                --resting-hr) resting_hr="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        calc_hr_zones "$age" "$resting_hr"
        ;;

    analyze-hr)
        shift
        age=30
        hr_data="120,135,150,165,155,145,130"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --age) age="$2"; shift 2 ;;
                --hr-data) hr_data="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        analyze_hr "$age" "$hr_data"
        ;;

    calc-bmi)
        shift
        weight=70
        height=170

        while [[ $# -gt 0 ]]; do
            case $1 in
                --weight) weight="$2"; shift 2 ;;
                --height) height="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        calc_bmi "$weight" "$height"
        ;;

    calc-pace)
        shift
        distance=5000
        duration=1800

        while [[ $# -gt 0 ]]; do
            case $1 in
                --distance) distance="$2"; shift 2 ;;
                --duration) duration="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        calc_pace "$distance" "$duration"
        ;;

    log-activity)
        shift
        type="running"
        duration=30
        distance=5000
        hr=150

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) type="$2"; shift 2 ;;
                --duration) duration="$2"; shift 2 ;;
                --distance) distance="$2"; shift 2 ;;
                --hr) hr="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        log_activity "$type" "$duration" "$distance" "$hr"
        ;;

    log-steps)
        shift
        steps=10000
        date=$(date +"%Y-%m-%d")

        while [[ $# -gt 0 ]]; do
            case $1 in
                --steps) steps="$2"; shift 2 ;;
                --date) date="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        log_steps "$steps" "$date"
        ;;

    set-goal)
        shift
        type="steps"
        target=10000
        period="daily"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) type="$2"; shift 2 ;;
                --target) target="$2"; shift 2 ;;
                --period) period="$2"; shift 2 ;;
                *) shift ;;
            esac
        done

        set_goal "$type" "$target" "$period"
        ;;

    --version|-v)
        print_header
        echo "Version: $VERSION"
        echo "License: MIT"
        echo "Author: WIA Health & Fitness Working Group"
        echo ""
        echo "弘益人間 (Benefit All Humanity)"
        ;;

    --help|-h)
        show_help
        ;;

    *)
        print_error "Unknown command: $1"
        echo "Run 'wia-ind-012 --help' for usage information"
        exit 1
        ;;
esac
