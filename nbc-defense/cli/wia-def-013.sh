#!/bin/bash

################################################################################
# WIA-DEF-013: NBC Defense CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to NBC/CBRN defense operations
# including threat detection, protection calculations, decontamination planning,
# and medical countermeasure management.
################################################################################

set -e

# Colors for output
SLATE='\033[38;5;102m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
ORANGE='\033[0;33m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           ☢️  WIA-DEF-013: NBC Defense CLI                    ║"
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

print_critical() {
    echo -e "${RED}${ORANGE}🚨 CRITICAL: $1${RESET}"
}

# Detect CBRN agent
detect_agent() {
    local agent_type=${1:-chemical}
    local concentration=${2:-0.05}
    local location=${3:-"Unknown"}

    print_section "CBRN Agent Detection"
    print_info "Agent Type: $agent_type"
    print_info "Concentration: $concentration mg/m³"
    print_info "Location: $location"

    # Determine threat level
    local threat_level
    if (( $(echo "$concentration > 0.1" | bc -l) )); then
        threat_level=5
        print_critical "THREAT LEVEL 5 - CATASTROPHIC"
    elif (( $(echo "$concentration > 0.01" | bc -l) )); then
        threat_level=4
        print_error "THREAT LEVEL 4 - SEVERE"
    elif (( $(echo "$concentration > 0.001" | bc -l) )); then
        threat_level=3
        print_warning "THREAT LEVEL 3 - SUBSTANTIAL"
    elif (( $(echo "$concentration > 0.0001" | bc -l) )); then
        threat_level=2
        print_warning "THREAT LEVEL 2 - MODERATE"
    else
        threat_level=1
        print_success "THREAT LEVEL 1 - LOW"
    fi

    print_section "Recommended Actions"

    if [ $threat_level -ge 4 ]; then
        print_critical "IMMEDIATE EVACUATION REQUIRED"
        print_error "Don MOPP 4 immediately"
        print_error "Activate emergency response plan"
    elif [ $threat_level -ge 3 ]; then
        print_warning "Don MOPP 3 protective equipment"
        print_warning "Begin decontamination procedures"
        print_warning "Request medical support"
    elif [ $threat_level -ge 2 ]; then
        print_info "Increase monitoring frequency"
        print_info "Prepare MOPP 2 equipment"
        print_info "Alert response teams"
    else
        print_success "Continue monitoring"
        print_success "Maintain readiness MOPP 1"
    fi

    echo ""
}

# Assess threat level
assess_threat() {
    local agent=${1:-nerve-agent}
    local concentration=${2:-0.05}
    local population=${3:-10000}
    local wind_speed=${4:-15}

    print_section "Threat Assessment"
    print_info "Agent: $agent"
    print_info "Concentration: $concentration mg/m³"
    print_info "Population at Risk: $population"
    print_info "Wind Speed: $wind_speed km/h"

    # Calculate affected area (simplified)
    local affected_area=$(echo "scale=2; $concentration * $wind_speed * 0.1" | bc -l)
    print_info "Estimated Affected Area: $affected_area km²"

    # Estimate casualties
    local casualty_rate
    if (( $(echo "$concentration > 0.1" | bc -l) )); then
        casualty_rate=0.8
    elif (( $(echo "$concentration > 0.01" | bc -l) )); then
        casualty_rate=0.5
    else
        casualty_rate=0.2
    fi

    local casualties=$(echo "scale=0; $population * $casualty_rate * 0.1" | bc -l)
    local fatalities=$(echo "scale=0; $casualties * 0.3" | bc -l)

    print_section "Impact Estimates"
    print_error "Estimated Casualties: $casualties"
    print_error "Estimated Fatalities: $fatalities"

    # Time to impact
    local time_to_impact=$(echo "scale=0; 3600 * $affected_area / $wind_speed" | bc -l)
    print_warning "Time to Impact: $time_to_impact seconds ($(echo "scale=1; $time_to_impact / 60" | bc -l) minutes)"

    print_section "Required Response"
    print_error "- Evacuate all personnel within affected area"
    print_error "- Don MOPP 4 protective equipment"
    print_warning "- Establish decontamination corridors"
    print_warning "- Dispense medical countermeasures"

    echo ""
}

# Calculate protection requirements
calc_protection() {
    local agent=${1:-nerve-agent}
    local concentration=${2:-0.05}
    local duration=${3:-3600}

    print_section "Protection Requirements"
    print_info "Agent: $agent"
    print_info "Concentration: $concentration mg/m³"
    print_info "Duration: $duration seconds ($(echo "scale=1; $duration / 3600" | bc -l) hours)"

    # Determine MOPP level
    local mopp_level
    if (( $(echo "$concentration > 0.1" | bc -l) )); then
        mopp_level=4
    elif (( $(echo "$concentration > 0.01" | bc -l) )); then
        mopp_level=3
    elif (( $(echo "$concentration > 0.001" | bc -l) )); then
        mopp_level=2
    else
        mopp_level=1
    fi

    print_section "Required MOPP Level"
    print_success "MOPP $mopp_level"

    print_section "Required PPE"
    if [ $mopp_level -ge 1 ]; then
        print_success "✓ JSLIST Chemical Protective Suit"
    fi
    if [ $mopp_level -ge 2 ]; then
        print_success "✓ Chemical Protective Overboots"
    fi
    if [ $mopp_level -ge 3 ]; then
        print_success "✓ M50 Protective Mask with C2A1 filters"
    fi
    if [ $mopp_level -ge 4 ]; then
        print_success "✓ Butyl Rubber Gloves"
        print_success "✓ Full encapsulation (hood up, suit sealed)"
    fi

    print_section "Work/Rest Cycle (at 30°C)"
    if [ $mopp_level -eq 4 ]; then
        print_warning "Work: 30 minutes"
        print_warning "Rest: 30 minutes"
        print_info "Fluid Intake: 2 liters/hour"
        print_error "Heat Stress Risk: HIGH"
    else
        print_success "Work: 60 minutes"
        print_success "Rest: 15 minutes"
        print_info "Fluid Intake: 1 liter/hour"
        print_success "Heat Stress Risk: MODERATE"
    fi

    print_section "Precautions"
    print_info "- Monitor for heat stress symptoms"
    print_info "- Maintain hydration"
    print_info "- Use buddy system"
    print_info "- Limit exposure time"

    echo ""
}

# Plan decontamination
plan_decon() {
    local personnel=${1:-150}
    local agent=${2:-sarin}
    local area=${3:-urban}

    print_section "Decontamination Planning"
    print_info "Affected Personnel: $personnel"
    print_info "Contaminant: $agent"
    print_info "Area Type: $area"

    # Calculate throughput and time
    local throughput=8  # people per hour
    local time=$(echo "scale=0; $personnel / $throughput * 60" | bc -l)

    print_section "Decontamination Estimates"
    print_success "Type: Operational Decontamination"
    print_info "Throughput: $throughput people/hour"
    print_warning "Estimated Time: $time minutes ($(echo "scale=1; $time / 60" | bc -l) hours)"

    print_section "Required Supplies"
    local water=$(echo "$personnel * 50" | bc -l)
    local bleach=$(echo "$personnel * 5" | bc -l)
    local soap=$(echo "scale=0; $personnel / 10 + 1" | bc -l)

    print_info "Water: $water liters"
    print_info "0.5% Bleach Solution: $bleach liters"
    print_info "Soap/Detergent: $soap bottles"
    print_info "Towels: $personnel each"
    print_info "Plastic Bags (waste): $(echo "$personnel * 2" | bc -l) each"

    local staff=$(echo "scale=0; $personnel / 10 + 5" | bc -l)
    print_info "Personnel Required: $staff"

    print_section "Decontamination Corridor"
    print_info "Station 1: Triage (30 sec/person, 2 staff)"
    print_info "Station 2: Disrobe (60 sec/person, 3 staff)"
    print_info "Station 3: Wash (180 sec/person, 4 staff)"
    print_info "Station 4: Rinse (120 sec/person, 2 staff)"
    print_info "Station 5: Dress (60 sec/person, 2 staff)"
    print_info "Station 6: Medical (300 sec/person, 3 staff)"

    print_section "Special Considerations"
    if [[ $agent == *"nerve"* ]]; then
        print_warning "Administer Mark I kits as needed"
        print_warning "Monitor for cholinergic symptoms"
    fi

    if [[ $area == "urban" ]]; then
        print_info "Coordinate with local authorities"
        print_info "Manage public information"
    fi

    echo ""
}

# Dispense countermeasures
dispense_countermeasures() {
    local agent=${1:-vx}
    local population=${2:-1000}
    local severity=${3:-moderate}

    print_section "Medical Countermeasure Distribution"
    print_info "Agent: $agent"
    print_info "Population: $population"
    print_info "Severity: $severity"

    print_section "Primary Countermeasure"

    if [[ $agent == *"nerve"* ]] || [[ $agent == "vx" ]] || [[ $agent == "sarin" ]]; then
        print_success "MARK I Kit (Atropine + 2-PAM Chloride)"
        print_info "Dosage: 2mg atropine + 600mg 2-PAM"
        print_info "Route: IM auto-injector"

        if [ "$severity" == "severe" ]; then
            print_warning "Frequency: Every 5-10 minutes (max 3 kits)"
            local required=$(echo "$population * 3" | bc -l)
        else
            print_info "Frequency: Once"
            local required=$population
        fi

        print_info "Required Quantity: $required kits"

        print_section "Alternative Countermeasure"
        print_info "CANA (Diazepam 10mg) - For seizures"

    elif [[ $agent == "anthrax" ]]; then
        print_success "Ciprofloxacin 500mg"
        print_info "Dosage: 500mg twice daily"
        print_info "Route: Oral"
        print_info "Duration: 60 days"
        local required=$(echo "$population * 120" | bc -l)  # 2 tablets per day for 60 days
        print_info "Required Quantity: $required tablets"

    elif [[ $agent == *"cesium"* ]] || [[ $agent == *"iodine"* ]]; then
        print_success "Potassium Iodide (KI) 130mg"
        print_info "Dosage: 130mg once daily"
        print_info "Route: Oral"
        print_info "Duration: Until exposure risk passes"
        local required=$population
        print_info "Required Quantity: $required tablets/day"
    else
        print_warning "Supportive Care"
        print_info "No specific antidote available"
        print_info "Provide symptomatic treatment"
    fi

    print_section "Administration Protocol"
    print_info "1. Verify agent exposure"
    print_info "2. Assess severity of symptoms"
    print_info "3. Administer appropriate dose"
    print_info "4. Monitor vital signs"
    print_info "5. Document administration"

    print_section "Monitoring Requirements"
    print_info "- Heart rate"
    print_info "- Blood pressure"
    print_info "- Respiratory rate"
    print_info "- Pupil size"
    print_info "- Level of consciousness"

    echo ""
}

# Generate emergency response plan
emergency_response() {
    local scenario=${1:-chemical-attack}
    local location=${2:-city-center}
    local population=${3:-50000}

    print_section "Emergency Response Plan"
    print_info "Scenario: $scenario"
    print_info "Location: $location"
    print_info "Population at Risk: $population"

    print_section "Incident Command Structure"
    print_success "✓ Incident Commander: On-Scene IC"
    print_success "✓ Safety Officer"
    print_success "✓ Public Information Officer"
    print_success "✓ Operations Section Chief"
    print_success "✓ Planning Section Chief"
    print_success "✓ Logistics Section Chief"
    print_success "✓ Finance/Admin Section Chief"

    print_section "Response Phases"

    echo -e "${YELLOW}Phase 1: Recognition and Notification (0-5 min)${RESET}"
    print_info "- Detect CBRN incident"
    print_info "- Sound alarms"
    print_info "- Notify emergency services"
    print_info "- Activate emergency response plan"

    echo -e "\n${YELLOW}Phase 2: Initial Response (5-30 min)${RESET}"
    print_info "- Establish incident command"
    print_info "- Define hot/warm/cold zones"
    print_info "- Don PPE (MOPP 4)"
    print_info "- Rescue exposed personnel"
    print_info "- Begin immediate decontamination"

    echo -e "\n${YELLOW}Phase 3: Extended Response (30 min - 12 hrs)${RESET}"
    print_info "- Laboratory agent identification"
    print_info "- Establish mass decon corridors"
    print_info "- Dispense medical countermeasures"
    print_info "- Transport casualties to hospitals"
    print_info "- Evidence collection"

    echo -e "\n${YELLOW}Phase 4: Recovery (12 hrs - 7 days)${RESET}"
    print_info "- Area decontamination"
    print_info "- Environmental sampling"
    print_info "- Victim follow-up"
    print_info "- Investigation"
    print_info "- After-action review"

    print_section "Critical Resources"
    print_info "- CBRN Detection Equipment: 5 units"
    print_info "- MOPP 4 Suits: $(echo "scale=0; $population / 100" | bc -l)"
    print_info "- Decon Tents: $(echo "scale=0; $population / 500 + 1" | bc -l)"
    print_info "- Medical Countermeasures: $population doses"
    print_info "- Ambulances: $(echo "scale=0; $population * 0.1 / 2" | bc -l)"

    print_section "Success Criteria"
    print_success "✓ All exposed personnel decontaminated"
    print_success "✓ Zero secondary contamination"
    print_success "✓ All casualties receive medical care"
    print_success "✓ Area cleared for reoccupation"
    print_success "✓ Public confidence restored"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-def-013 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  detect-agent                 Detect and identify CBRN agent"
    echo "    --type <type>              Agent type (chemical/biological/radiological)"
    echo "    --concentration <value>    Concentration (mg/m³ or Bq/m³)"
    echo "    --location <location>      Detection location"
    echo ""
    echo "  assess-threat                Assess threat level and impact"
    echo "    --agent <agent>            Agent name (e.g., nerve-agent, anthrax)"
    echo "    --concentration <value>    Concentration (mg/m³)"
    echo "    --population <number>      Population at risk"
    echo "    --wind-speed <km/h>        Wind speed (default: 15)"
    echo ""
    echo "  calc-protection              Calculate protection requirements"
    echo "    --agent <agent>            Agent name"
    echo "    --concentration <value>    Concentration (mg/m³)"
    echo "    --duration <seconds>       Exposure duration (default: 3600)"
    echo ""
    echo "  plan-decon                   Plan decontamination operation"
    echo "    --personnel <number>       Number of affected personnel"
    echo "    --agent <agent>            Contaminant agent"
    echo "    --area <type>              Area type (urban/rural/facility)"
    echo ""
    echo "  dispense-countermeasures     Plan medical countermeasure distribution"
    echo "    --agent <agent>            CBRN agent"
    echo "    --population <number>      Population requiring treatment"
    echo "    --severity <level>         Exposure severity (mild/moderate/severe)"
    echo ""
    echo "  emergency-response           Generate emergency response plan"
    echo "    --scenario <scenario>      Incident scenario"
    echo "    --location <location>      Incident location"
    echo "    --population <number>      Population at risk (default: 50000)"
    echo ""
    echo "  version                      Show version information"
    echo "  help                         Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-def-013 detect-agent --type chemical --concentration 0.05"
    echo "  wia-def-013 assess-threat --agent sarin --concentration 0.08 --population 10000"
    echo "  wia-def-013 calc-protection --agent vx --concentration 0.1 --duration 1800"
    echo "  wia-def-013 plan-decon --personnel 150 --agent sarin --area urban"
    echo "  wia-def-013 dispense-countermeasures --agent anthrax --population 5000"
    echo "  wia-def-013 emergency-response --scenario chemical-attack --location downtown"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-DEF-013 NBC Defense CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    detect-agent)
        TYPE="chemical"
        CONCENTRATION=0.05
        LOCATION="Unknown"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --concentration) CONCENTRATION=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        detect_agent "$TYPE" "$CONCENTRATION" "$LOCATION"
        ;;

    assess-threat)
        AGENT="nerve-agent"
        CONCENTRATION=0.05
        POPULATION=10000
        WIND_SPEED=15

        while [[ $# -gt 0 ]]; do
            case $1 in
                --agent) AGENT=$2; shift 2 ;;
                --concentration) CONCENTRATION=$2; shift 2 ;;
                --population) POPULATION=$2; shift 2 ;;
                --wind-speed) WIND_SPEED=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        assess_threat "$AGENT" "$CONCENTRATION" "$POPULATION" "$WIND_SPEED"
        ;;

    calc-protection)
        AGENT="nerve-agent"
        CONCENTRATION=0.05
        DURATION=3600

        while [[ $# -gt 0 ]]; do
            case $1 in
                --agent) AGENT=$2; shift 2 ;;
                --concentration) CONCENTRATION=$2; shift 2 ;;
                --duration) DURATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_protection "$AGENT" "$CONCENTRATION" "$DURATION"
        ;;

    plan-decon)
        PERSONNEL=150
        AGENT="sarin"
        AREA="urban"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --personnel) PERSONNEL=$2; shift 2 ;;
                --agent) AGENT=$2; shift 2 ;;
                --area) AREA=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        plan_decon "$PERSONNEL" "$AGENT" "$AREA"
        ;;

    dispense-countermeasures)
        AGENT="vx"
        POPULATION=1000
        SEVERITY="moderate"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --agent) AGENT=$2; shift 2 ;;
                --population) POPULATION=$2; shift 2 ;;
                --severity) SEVERITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        dispense_countermeasures "$AGENT" "$POPULATION" "$SEVERITY"
        ;;

    emergency-response)
        SCENARIO="chemical-attack"
        LOCATION="city-center"
        POPULATION=50000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --scenario) SCENARIO=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                --population) POPULATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        emergency_response "$SCENARIO" "$LOCATION" "$POPULATION"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-def-013 help' for usage information"
        exit 1
        ;;
esac

exit 0
