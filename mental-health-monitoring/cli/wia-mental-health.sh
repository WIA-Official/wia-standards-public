#!/bin/bash
# WIA-MENTAL-HEALTH CLI Tool
# Mental Health Monitoring Standard
#
# 弘益人間 (Benefit All Humanity)
# © 2025 WIA - World Certification Industry Association
# MIT License

VERSION="1.0.0"
STANDARD="WIA-MENTAL-HEALTH"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Print banner
print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                  🧠 WIA-MENTAL-HEALTH                     ║"
    echo "║          Mental Health Monitoring Standard v${VERSION}        ║"
    echo "║                                                           ║"
    echo "║              弘益人間 · Benefit All Humanity              ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# Print help
print_help() {
    echo -e "${GREEN}Usage:${NC} wia-mental-health <command> [options]"
    echo ""
    echo -e "${GREEN}Commands:${NC}"
    echo "  assess        Run mental health assessment (PHQ-9, GAD-7)"
    echo "  score         Calculate scores from raw responses"
    echo "  crisis        Perform crisis screening"
    echo "  biomarker     Analyze biomarker data"
    echo "  validate      Validate data against WIA schema"
    echo "  export        Export data in WIA format"
    echo "  version       Show version information"
    echo "  help          Show this help message"
    echo ""
    echo -e "${GREEN}Examples:${NC}"
    echo "  wia-mental-health assess --instrument phq9"
    echo "  wia-mental-health score --phq9 '2,1,2,1,0,1,0,1,0'"
    echo "  wia-mental-health crisis --suicide 0 --selfharm 0"
    echo "  wia-mental-health biomarker --cortisol 15 --hrv 35"
    echo ""
}

# PHQ-9 severity calculation
calculate_phq9_severity() {
    local score=$1
    if [ $score -le 4 ]; then
        echo "none"
    elif [ $score -le 9 ]; then
        echo "mild"
    elif [ $score -le 14 ]; then
        echo "moderate"
    elif [ $score -le 19 ]; then
        echo "moderately-severe"
    else
        echo "severe"
    fi
}

# GAD-7 severity calculation
calculate_gad7_severity() {
    local score=$1
    if [ $score -le 4 ]; then
        echo "none"
    elif [ $score -le 9 ]; then
        echo "mild"
    elif [ $score -le 14 ]; then
        echo "moderate"
    else
        echo "severe"
    fi
}

# Normalize score to 0.0-1.0
normalize_score() {
    local raw=$1
    local max=$2
    echo "scale=2; $raw / $max" | bc
}

# Calculate scores from responses
cmd_score() {
    local phq9_responses=""
    local gad7_responses=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --phq9)
                phq9_responses="$2"
                shift 2
                ;;
            --gad7)
                gad7_responses="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Mental Health Index Calculation${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    if [ -n "$phq9_responses" ]; then
        # Parse comma-separated values and sum
        IFS=',' read -ra ITEMS <<< "$phq9_responses"
        phq9_sum=0
        for item in "${ITEMS[@]}"; do
            phq9_sum=$((phq9_sum + item))
        done

        phq9_norm=$(normalize_score $phq9_sum 27)
        phq9_severity=$(calculate_phq9_severity $phq9_sum)

        echo ""
        echo -e "${BLUE}PHQ-9 (Depression):${NC}"
        echo "  Raw Score:        $phq9_sum / 27"
        echo "  Normalized Score: $phq9_norm"
        echo "  Severity:         $phq9_severity"
    fi

    if [ -n "$gad7_responses" ]; then
        IFS=',' read -ra ITEMS <<< "$gad7_responses"
        gad7_sum=0
        for item in "${ITEMS[@]}"; do
            gad7_sum=$((gad7_sum + item))
        done

        gad7_norm=$(normalize_score $gad7_sum 21)
        gad7_severity=$(calculate_gad7_severity $gad7_sum)

        echo ""
        echo -e "${BLUE}GAD-7 (Anxiety):${NC}"
        echo "  Raw Score:        $gad7_sum / 21"
        echo "  Normalized Score: $gad7_norm"
        echo "  Severity:         $gad7_severity"
    fi

    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Crisis screening
cmd_crisis() {
    local suicide=0
    local selfharm=0
    local intoxication="false"
    local psychosis="false"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --suicide)
                suicide="$2"
                shift 2
                ;;
            --selfharm)
                selfharm="$2"
                shift 2
                ;;
            --intoxication)
                intoxication="true"
                shift
                ;;
            --psychosis)
                psychosis="true"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    # Determine crisis level
    local level="none"
    local action="Continue standard care"
    local color=$GREEN

    if [ $suicide -ge 3 ] || [ "$psychosis" = "true" ]; then
        level="IMMINENT"
        action="Call emergency services (911) immediately"
        color=$RED
    elif [ $suicide -ge 2 ] || ([ $selfharm -ge 2 ] && [ "$intoxication" = "true" ]); then
        level="HIGH"
        action="Immediate clinical contact required"
        color=$RED
    elif [ $suicide -ge 1 ] || [ $selfharm -ge 2 ]; then
        level="MODERATE"
        action="Activate safety plan, schedule urgent appointment"
        color=$YELLOW
    elif [ $selfharm -ge 1 ] || [ "$intoxication" = "true" ]; then
        level="LOW"
        action="Continue current care, monitor closely"
        color=$GREEN
    fi

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${RED}🚨 Crisis Screening Result${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "  Crisis Level:      ${color}${level}${NC}"
    echo -e "  Recommended Action: ${action}"
    echo ""

    if [ "$level" = "IMMINENT" ] || [ "$level" = "HIGH" ]; then
        echo -e "  ${YELLOW}📞 988 Suicide & Crisis Lifeline - Available 24/7${NC}"
    fi

    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Biomarker analysis
cmd_biomarker() {
    local cortisol=""
    local crp=""
    local hrv=""
    local sleep=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --cortisol)
                cortisol="$2"
                shift 2
                ;;
            --crp)
                crp="$2"
                shift 2
                ;;
            --hrv)
                hrv="$2"
                shift 2
                ;;
            --sleep)
                sleep="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}🧬 Biomarker Analysis${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""

    if [ -n "$cortisol" ]; then
        local status="Normal"
        local color=$GREEN
        if (( $(echo "$cortisol > 25" | bc -l) )); then
            status="Elevated"
            color=$YELLOW
        elif (( $(echo "$cortisol < 5" | bc -l) )); then
            status="Low"
            color=$YELLOW
        fi
        echo -e "  Cortisol:  ${cortisol} ng/mL  [${color}${status}${NC}]"
    fi

    if [ -n "$crp" ]; then
        local status="Normal"
        local color=$GREEN
        if (( $(echo "$crp > 3" | bc -l) )); then
            status="Elevated (inflammation)"
            color=$YELLOW
        fi
        echo -e "  CRP:       ${crp} mg/L    [${color}${status}${NC}]"
    fi

    if [ -n "$hrv" ]; then
        local status="Normal"
        local color=$GREEN
        if [ $hrv -lt 20 ]; then
            status="Low (poor autonomic balance)"
            color=$YELLOW
        elif [ $hrv -gt 50 ]; then
            status="Excellent"
            color=$GREEN
        fi
        echo -e "  HRV RMSSD: ${hrv} ms      [${color}${status}${NC}]"
    fi

    if [ -n "$sleep" ]; then
        local status="Good"
        local color=$GREEN
        if [ $sleep -lt 75 ]; then
            status="Poor"
            color=$RED
        elif [ $sleep -lt 85 ]; then
            status="Fair"
            color=$YELLOW
        fi
        echo -e "  Sleep:     ${sleep}%        [${color}${status}${NC}]"
    fi

    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Validate data
cmd_validate() {
    local file="$1"

    if [ -z "$file" ]; then
        echo -e "${RED}Error: No file specified${NC}"
        echo "Usage: wia-mental-health validate <file.json>"
        exit 1
    fi

    if [ ! -f "$file" ]; then
        echo -e "${RED}Error: File not found: $file${NC}"
        exit 1
    fi

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Validating: ${file}${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""

    # Check if jq is available
    if ! command -v jq &> /dev/null; then
        echo -e "${YELLOW}Warning: jq not installed. Basic validation only.${NC}"
        # Basic JSON validation
        if python3 -c "import json; json.load(open('$file'))" 2>/dev/null; then
            echo -e "  ✅ Valid JSON structure"
        else
            echo -e "  ❌ Invalid JSON structure"
            exit 1
        fi
    else
        # Check for required fields
        local valid=true

        # Check mental_health_index
        if jq -e '.mental_health_index' "$file" > /dev/null 2>&1; then
            echo -e "  ✅ mental_health_index present"

            # Check score ranges
            local dep_score=$(jq -r '.mental_health_index.depression_score.value // empty' "$file")
            if [ -n "$dep_score" ]; then
                if (( $(echo "$dep_score >= 0 && $dep_score <= 1" | bc -l) )); then
                    echo -e "  ✅ depression_score in valid range (0-1)"
                else
                    echo -e "  ❌ depression_score out of range: $dep_score"
                    valid=false
                fi
            fi
        else
            echo -e "  ⚠️  mental_health_index not found"
        fi

        # Check consent
        if jq -e '.consent' "$file" > /dev/null 2>&1; then
            echo -e "  ✅ consent record present"
        else
            echo -e "  ⚠️  consent record recommended"
        fi

        # Check encryption markers
        if jq -e '.security' "$file" > /dev/null 2>&1; then
            echo -e "  ✅ security configuration present"
        else
            echo -e "  ⚠️  security configuration recommended for PHI"
        fi

        echo ""
        if [ "$valid" = true ]; then
            echo -e "  ${GREEN}✅ Validation PASSED${NC}"
        else
            echo -e "  ${RED}❌ Validation FAILED${NC}"
        fi
    fi

    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Version info
cmd_version() {
    echo -e "${GREEN}${STANDARD}${NC} version ${VERSION}"
    echo "Standard: Mental Health Monitoring"
    echo "Category: MED (Healthcare)"
    echo "License: MIT"
    echo ""
    echo "弘益人間 · Benefit All Humanity"
    echo "© 2025 WIA - World Certification Industry Association"
}

# Main entry point
main() {
    if [ $# -eq 0 ]; then
        print_banner
        print_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        assess)
            print_banner
            echo -e "${YELLOW}Interactive assessment mode coming soon.${NC}"
            echo "Use 'wia-mental-health score' for score calculation."
            ;;
        score)
            cmd_score "$@"
            ;;
        crisis)
            cmd_crisis "$@"
            ;;
        biomarker)
            cmd_biomarker "$@"
            ;;
        validate)
            cmd_validate "$@"
            ;;
        export)
            echo -e "${YELLOW}Export functionality coming soon.${NC}"
            ;;
        version|--version|-v)
            cmd_version
            ;;
        help|--help|-h)
            print_banner
            print_help
            ;;
        *)
            echo -e "${RED}Unknown command: $command${NC}"
            echo "Run 'wia-mental-health help' for usage."
            exit 1
            ;;
    esac
}

main "$@"
