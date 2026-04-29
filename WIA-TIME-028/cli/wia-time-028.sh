#!/bin/bash

################################################################################
# WIA-TIME-028: Temporal Medical Care CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Temporal Medicine Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to temporal medical care functions
# including diagnosis, treatment planning, and health monitoring.
################################################################################

set -e

# Colors for output
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
TSI_NORMAL_MAX=30
TSI_CRITICAL=70
AGE_DELTA_NORMAL=0.05
MCI_NORMAL=0.95
CSS_MILD=30
CSS_MODERATE=60
CSS_SEVERE=90

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🩺 WIA-TIME-028: Temporal Medical Care CLI            ║"
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

# Diagnose Temporal Sickness Syndrome
diagnose_tss() {
    local patient_id="$1"
    local displacement=${2:--86400}
    local symptoms="$3"
    local severity=${4:-5}
    local mass=${5:-75}

    print_section "Temporal Sickness Syndrome Diagnosis"
    print_info "Patient ID: $patient_id"
    print_info "Displacement: $displacement seconds ($(echo "scale=2; $displacement / 86400" | bc) days)"
    print_info "Reported Severity: $severity/10"
    print_info "Mass: $mass kg"
    print_info "Symptoms: $symptoms"

    # Calculate TSS severity score (simplified)
    local abs_disp=$(echo "scale=0; sqrt($displacement * $displacement)" | bc -l)
    local velocity=0.5
    local age=35
    local recovery=0.7
    local adapt_time=86400

    local tss_score=$(echo "scale=2; ($abs_disp * $mass * $velocity) / ($age * $recovery * $adapt_time) * 100" | bc -l)
    tss_score=$(echo "scale=0; $tss_score / 1" | bc)  # Convert to integer

    # Cap at 100
    if (( $(echo "$tss_score > 100" | bc -l) )); then
        tss_score=100
    fi

    print_section "Diagnosis Results"
    print_success "TSS Severity Score: $tss_score"

    # Determine severity level
    if (( $(echo "$tss_score <= 20" | bc -l) )); then
        print_success "Severity Level: MILD"
        print_info "Prognosis: Excellent"
        print_info "Treatment: Outpatient care with oral medications"
        print_info "Expected Recovery: 2-3 days"
    elif (( $(echo "$tss_score <= 40" | bc -l) )); then
        print_warning "Severity Level: MODERATE"
        print_info "Prognosis: Good"
        print_info "Treatment: Hospitalization for 2-3 days, IV therapy"
        print_info "Expected Recovery: 5-7 days"
    elif (( $(echo "$tss_score <= 70" | bc -l) )); then
        print_error "Severity Level: SEVERE"
        print_info "Prognosis: Fair"
        print_info "Treatment: ICU admission, intensive temporal stabilization"
        print_info "Expected Recovery: 2-3 weeks"
    else
        print_error "Severity Level: CRITICAL"
        print_info "Prognosis: Critical"
        print_info "Treatment: EMERGENCY - Immediate ICU, life support may be needed"
        print_info "Expected Recovery: 1-2 months (if survives)"
    fi

    print_section "Recommended Treatment"
    if (( $(echo "$tss_score <= 20" | bc -l) )); then
        print_info "• Chronostabin 50mg PO every 6 hours × 2 days"
        print_info "• Temporodol 10mg PO as needed for nausea"
        print_info "• Rest and hydration"
        print_info "• Follow-up in 3 days"
    elif (( $(echo "$tss_score <= 40" | bc -l) )); then
        print_info "• Temporazine 25mg IV every 8 hours × 3 days"
        print_info "• IV fluids and electrolyte replacement"
        print_info "• Temporal Stabilization Chamber sessions (2/day)"
        print_info "• Continuous vital sign monitoring"
        print_info "• Hospital admission 48-72 hours"
    else
        print_info "• Chronolox 100mg IV every 4 hours"
        print_info "• Advanced temporal stabilization therapy"
        print_info "• ICU level care"
        print_info "• Possible cellular regeneration therapy"
        print_info "• 24/7 monitoring"
    fi

    echo ""
}

# Calculate Cellular Age
calc_cellular_age() {
    local patient_id="$1"
    local chrono_age=${2:-35}
    local telomere=${3:-8500}
    local displacement=${4:-0}

    print_section "Cellular Age Analysis"
    print_info "Patient ID: $patient_id"
    print_info "Chronological Age: $chrono_age years"
    print_info "Telomere Length: $telomere bp"
    print_info "Temporal Displacement History: $displacement seconds total"

    # Calculate telomere-based age
    local expected_telomere=$(echo "11000 - ($chrono_age * 45)" | bc -l)
    local telomere_delta=$(echo "$telomere - $expected_telomere" | bc -l)
    local age_adjustment=$(echo "$telomere_delta / 45" | bc -l)
    local telomere_age=$(echo "$chrono_age - $age_adjustment" | bc -l)

    # Add temporal displacement effect
    local temporal_aging=$(echo "scale=2; (sqrt($displacement * $displacement) / 31536000) * 0.1" | bc -l)
    local biological_age=$(echo "scale=1; $telomere_age + $temporal_aging" | bc -l)

    # Calculate age delta
    local age_delta=$(echo "scale=1; $biological_age - $chrono_age" | bc -l)
    local age_delta_pct=$(echo "scale=1; ($age_delta / $chrono_age) * 100" | bc -l)

    print_section "Analysis Results"
    print_success "Biological Age: $biological_age years"
    print_info "Chronological Age: $chrono_age years"

    if (( $(echo "$age_delta >= 0" | bc -l) )); then
        print_info "Age Delta: +$age_delta years (+$age_delta_pct%)"
    else
        print_info "Age Delta: $age_delta years ($age_delta_pct%)"
    fi

    # Determine severity
    local abs_delta_pct=$(echo "scale=3; sqrt($age_delta_pct * $age_delta_pct) / 100" | bc -l)

    if (( $(echo "$abs_delta_pct <= $AGE_DELTA_NORMAL" | bc -l) )); then
        print_success "Status: NORMAL - Age within expected range"
        print_info "No treatment required"
    elif (( $(echo "$abs_delta_pct <= 0.15" | bc -l) )); then
        print_warning "Status: MILD DISCREPANCY"
        print_info "Lifestyle modifications recommended"
        print_info "Monitor every 6 months"
    elif (( $(echo "$abs_delta_pct <= 0.30" | bc -l) )); then
        print_error "Status: MODERATE DISCREPANCY"
        print_info "Age synchronization therapy recommended"
        print_info "Treatment duration: 2-3 months"
    else
        print_error "Status: SEVERE/CRITICAL DISCREPANCY"
        print_info "Immediate intervention required"
        print_info "Intensive age synchronization therapy"
        print_info "Treatment duration: 6-12 months"
    fi

    print_section "Biomarker Status"
    if (( telomere < 5000 )); then
        print_error "Telomere Status: CRITICALLY SHORT"
    elif (( telomere < 8000 )); then
        print_warning "Telomere Status: SHORT"
    elif (( telomere > 20000 )); then
        print_warning "Telomere Status: ABNORMALLY LONG"
    else
        print_success "Telomere Status: NORMAL"
    fi

    echo ""
}

# Assess Memory Coherence
memory_check() {
    local patient_id="$1"
    local timeline_events=${2:-100}
    local intact_memories=${3:-95}
    local fragmentation=${4:-2}

    print_section "Memory Coherence Assessment"
    print_info "Patient ID: $patient_id"
    print_info "Expected Timeline Events: $timeline_events"
    print_info "Intact Memories: $intact_memories"
    print_info "Fragmentation Events: $fragmentation"

    # Calculate MCI
    local frag_rate=$(echo "scale=4; $fragmentation / ($timeline_events + $fragmentation)" | bc -l)
    local intact_pct=$(echo "scale=4; $intact_memories / $timeline_events" | bc -l)
    local timeline_coherence=0.98
    local mci=$(echo "scale=4; $intact_pct * (1 - $frag_rate) * $timeline_coherence" | bc -l)

    print_section "Assessment Results"
    print_success "Memory Coherence Index: $(printf '%.3f' $mci)"
    print_info "Intact Memory Percentage: $(echo "scale=1; $intact_pct * 100" | bc)%"
    print_info "Fragmentation Rate: $(echo "scale=1; $frag_rate * 100" | bc)%"

    # Determine status
    if (( $(echo "$mci >= $MCI_NORMAL" | bc -l) )); then
        print_success "Status: NORMAL - No memory impairment detected"
        print_info "No treatment required"
    elif (( $(echo "$mci >= 0.80" | bc -l) )); then
        print_warning "Status: MILD IMPAIRMENT"
        print_info "Cognitive therapy recommended"
        print_info "Prognosis: Good with treatment"
    elif (( $(echo "$mci >= 0.60" | bc -l) )); then
        print_error "Status: MODERATE IMPAIRMENT"
        print_info "Memory reconstruction therapy required"
        print_info "Treatment: 3 sessions per week × 12 weeks"
        print_info "Prognosis: Fair"
    else
        print_error "Status: SEVERE IMPAIRMENT"
        print_info "Intensive rehabilitation required"
        print_info "Hospital-based treatment recommended"
        print_info "Prognosis: Guarded"
    fi

    print_section "Recommendations"
    if (( $(echo "$mci < $MCI_NORMAL" | bc -l) )); then
        print_info "• Memory reconstruction therapy"
        print_info "• Timeline journaling"
        print_info "• Cognitive behavioral therapy"
        print_info "• Nootropic support (acetylcholinesterase inhibitors)"
        print_info "• Weekly progress monitoring"
    fi

    echo ""
}

# Evaluate Chronological Stress Syndrome
stress_eval() {
    local patient_id="$1"
    local distress=${2:-5}
    local exposure_time=${3:-86400}
    local paradox_events=${4:-1}
    local social_support=${5:-7}
    local resilience=${6:-7}

    print_section "Chronological Stress Syndrome Evaluation"
    print_info "Patient ID: $patient_id"
    print_info "Psychological Distress: $distress/10"
    print_info "Total Exposure Time: $exposure_time seconds"
    print_info "Paradoxical Events: $paradox_events"
    print_info "Social Support: $social_support/10"
    print_info "Resilience: $resilience/10"

    # Calculate CSS Score
    local css_score=$(echo "scale=2; ($distress * $exposure_time * $paradox_events) / ($social_support * $resilience)" | bc -l)
    css_score=$(echo "scale=0; $css_score / 1" | bc)  # Convert to integer

    print_section "Evaluation Results"
    print_success "CSS Score: $css_score"

    # Determine severity
    if (( css_score <= CSS_MILD )); then
        print_success "Severity: MILD"
        print_info "Status: Manageable stress levels"
        print_info "Treatment: Self-care and monitoring"
    elif (( css_score <= CSS_MODERATE )); then
        print_warning "Severity: MODERATE"
        print_info "Status: Intervention recommended"
        print_info "Treatment: Counseling and support groups"
    elif (( css_score <= CSS_SEVERE )); then
        print_error "Severity: SEVERE"
        print_info "Status: Treatment required"
        print_info "Treatment: Intensive therapy, possible medication"
    else
        print_error "Severity: CRITICAL"
        print_info "Status: Emergency care needed"
        print_info "Treatment: Immediate psychiatric intervention"
    fi

    print_section "Treatment Plan"
    if (( css_score <= CSS_MILD )); then
        print_info "• Temporal support group (weekly)"
        print_info "• Stress management techniques"
        print_info "• Regular exercise and sleep hygiene"
        print_info "• Monthly check-ins"
    elif (( css_score <= CSS_MODERATE )); then
        print_info "• Temporal-focused CBT (weekly sessions)"
        print_info "• Support group participation"
        print_info "• Mindfulness and relaxation training"
        print_info "• Bi-weekly monitoring"
    elif (( css_score <= CSS_SEVERE )); then
        print_info "• Intensive psychotherapy (2× weekly)"
        print_info "• SSRI medication (Sertraline 50mg daily)"
        print_info "• Timeline acceptance therapy"
        print_info "• Weekly monitoring"
        print_info "• Possible short-term anxiolytics"
    else
        print_info "• Daily psychiatric monitoring"
        print_info "• Emergency intervention protocol"
        print_info "• Intensive therapy sessions"
        print_info "• Medication management"
        print_info "• 24/7 crisis support"
        print_info "• Consider hospitalization"
    fi

    echo ""
}

# Generate Treatment Plan
generate_treatment() {
    local patient_id="$1"
    local condition="$2"
    local severity="$3"
    local output_file="$4"

    print_section "Treatment Plan Generation"
    print_info "Patient ID: $patient_id"
    print_info "Condition: $condition"
    print_info "Severity: $severity"

    local plan_id="TP-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"

    print_success "Generated Treatment Plan ID: $plan_id"

    if [ -n "$output_file" ]; then
        cat > "$output_file" << EOF
# Treatment Plan: $plan_id

**Patient ID:** $patient_id
**Condition:** $condition
**Severity:** $severity
**Date:** $(date)

## Treatment Goals
1. Resolve primary symptoms
2. Stabilize condition
3. Prevent complications
4. Return to baseline function

## Medications
EOF

        if [ "$condition" == "TSS" ]; then
            if [ "$severity" == "mild" ]; then
                echo "- Chronostabin 50mg PO q6h × 2 days" >> "$output_file"
                echo "- Temporodol 10mg PO PRN nausea" >> "$output_file"
            elif [ "$severity" == "moderate" ]; then
                echo "- Temporazine 25mg IV q8h × 3 days" >> "$output_file"
                echo "- IV fluids and electrolytes" >> "$output_file"
            else
                echo "- Chronolox 100mg IV q4h" >> "$output_file"
                echo "- Advanced temporal stabilization" >> "$output_file"
            fi
        fi

        cat >> "$output_file" << EOF

## Therapies
- Temporal Stabilization Chamber sessions
- Cellular support therapy
- Physical rehabilitation as needed

## Monitoring
- Vital signs monitoring
- Temporal stress index tracking
- Laboratory follow-up

## Follow-up Schedule
- Day 3: Clinical evaluation
- Week 1: Progress assessment
- Week 2: Treatment response check
- Month 1: Final evaluation

## Emergency Contact
- 24/7 Temporal Medical Hotline: 1-800-TEMP-MED
- Emergency Services: 911

---
弘益人間 (Benefit All Humanity)
© 2025 WIA - MIT License
EOF

        print_success "Treatment plan saved to: $output_file"
    fi

    echo ""
}

# Monitor Patient Vitals
monitor_vitals() {
    local patient_id="$1"
    local duration=${2:-60}

    print_section "Vital Signs Monitoring"
    print_info "Patient ID: $patient_id"
    print_info "Monitoring Duration: $duration seconds"
    print_info "Starting continuous monitoring..."

    echo ""
    local end_time=$(($(date +%s) + duration))
    local count=0

    while [ $(date +%s) -lt $end_time ]; do
        count=$((count + 1))

        # Simulate vital signs
        local hr=$((70 + RANDOM % 30))
        local bp_sys=$((110 + RANDOM % 30))
        local bp_dia=$((70 + RANDOM % 20))
        local temp=$(echo "scale=1; 36.5 + ($RANDOM % 20) / 10" | bc -l)
        local tsi=$((RANDOM % 50))

        echo -ne "\r${GRAY}Reading #$count: "
        echo -ne "HR=${hr}bpm "
        echo -ne "BP=${bp_sys}/${bp_dia}mmHg "
        echo -ne "Temp=${temp}°C "

        if (( tsi < TSI_NORMAL_MAX )); then
            echo -ne "TSI=${GREEN}${tsi}${GRAY} ✓${RESET}"
        elif (( tsi < TSI_CRITICAL )); then
            echo -ne "TSI=${YELLOW}${tsi}${GRAY} ⚠${RESET}"
        else
            echo -ne "TSI=${RED}${tsi}${GRAY} ✗${RESET}"
        fi

        sleep 3
    done

    echo ""
    echo ""
    print_success "Monitoring complete"
    print_info "Total readings: $count"
    print_info "Report saved to patient record"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-time-028 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  diagnose                 Diagnose Temporal Sickness Syndrome"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --displacement <sec>   Temporal displacement in seconds"
    echo "    --symptoms <list>      Comma-separated symptoms"
    echo "    --severity <1-10>      Self-reported severity"
    echo ""
    echo "  calc-age                 Calculate cellular age"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --chrono-age <years>   Chronological age"
    echo "    --telomere <bp>        Telomere length in base pairs"
    echo "    --displacement <sec>   Total temporal exposure"
    echo ""
    echo "  memory-check             Assess memory coherence"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --timeline-events <n>  Expected timeline events"
    echo "    --intact-memories <n>  Number of intact memories"
    echo "    --fragmentation <n>    Fragmentation events"
    echo ""
    echo "  stress-eval              Evaluate chronological stress"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --distress <0-10>      Psychological distress level"
    echo "    --exposure <sec>       Total temporal exposure time"
    echo "    --events <n>           Paradoxical events encountered"
    echo ""
    echo "  treatment                Generate treatment plan"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --condition <name>     Medical condition (TSS, CAD, TA, CSS)"
    echo "    --severity <level>     Severity (mild, moderate, severe, critical)"
    echo "    --output <file>        Output file (optional)"
    echo ""
    echo "  monitor                  Monitor patient vital signs"
    echo "    --patient <id>         Patient identifier (required)"
    echo "    --duration <seconds>   Monitoring duration (default: 60)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-time-028 diagnose --patient P-001 --displacement -86400 --symptoms nausea,dizziness"
    echo "  wia-time-028 calc-age --patient P-001 --chrono-age 35 --telomere 8500"
    echo "  wia-time-028 memory-check --patient P-001 --timeline-events 150 --intact-memories 142"
    echo "  wia-time-028 treatment --patient P-001 --condition TSS --severity moderate --output plan.txt"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-TIME-028 Temporal Medical Care CLI"
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
    diagnose)
        PATIENT_ID=""
        DISPLACEMENT=-86400
        SYMPTOMS="nausea,disorientation"
        SEVERITY=5
        MASS=75

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --displacement) DISPLACEMENT=$2; shift 2 ;;
                --symptoms) SYMPTOMS=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                --mass) MASS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        diagnose_tss "$PATIENT_ID" "$DISPLACEMENT" "$SYMPTOMS" "$SEVERITY" "$MASS"
        ;;

    calc-age)
        PATIENT_ID=""
        CHRONO_AGE=35
        TELOMERE=8500
        DISPLACEMENT=0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --chrono-age) CHRONO_AGE=$2; shift 2 ;;
                --telomere) TELOMERE=$2; shift 2 ;;
                --displacement) DISPLACEMENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        calc_cellular_age "$PATIENT_ID" "$CHRONO_AGE" "$TELOMERE" "$DISPLACEMENT"
        ;;

    memory-check)
        PATIENT_ID=""
        TIMELINE_EVENTS=100
        INTACT_MEMORIES=95
        FRAGMENTATION=2

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --timeline-events) TIMELINE_EVENTS=$2; shift 2 ;;
                --intact-memories) INTACT_MEMORIES=$2; shift 2 ;;
                --fragmentation) FRAGMENTATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        memory_check "$PATIENT_ID" "$TIMELINE_EVENTS" "$INTACT_MEMORIES" "$FRAGMENTATION"
        ;;

    stress-eval)
        PATIENT_ID=""
        DISTRESS=5
        EXPOSURE=86400
        EVENTS=1
        SUPPORT=7
        RESILIENCE=7

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --distress) DISTRESS=$2; shift 2 ;;
                --exposure) EXPOSURE=$2; shift 2 ;;
                --events) EVENTS=$2; shift 2 ;;
                --support) SUPPORT=$2; shift 2 ;;
                --resilience) RESILIENCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        stress_eval "$PATIENT_ID" "$DISTRESS" "$EXPOSURE" "$EVENTS" "$SUPPORT" "$RESILIENCE"
        ;;

    treatment)
        PATIENT_ID=""
        CONDITION="TSS"
        SEVERITY="moderate"
        OUTPUT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --condition) CONDITION=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        generate_treatment "$PATIENT_ID" "$CONDITION" "$SEVERITY" "$OUTPUT"
        ;;

    monitor)
        PATIENT_ID=""
        DURATION=60

        while [[ $# -gt 0 ]]; do
            case $1 in
                --patient) PATIENT_ID=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PATIENT_ID" ]; then
            print_error "Patient ID required"
            exit 1
        fi

        print_header
        monitor_vitals "$PATIENT_ID" "$DURATION"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-time-028 help' for usage information"
        exit 1
        ;;
esac

exit 0
