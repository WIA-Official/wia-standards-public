#!/bin/bash

################################################################################
# WIA-IND-004: Beauty Tech Standard - CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Beauty Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

VERSION="1.0.0"
SCRIPT_NAME="wia-ind-004"

# 색상 정의 (Color definitions)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

################################################################################
# Utility Functions
################################################################################

print_header() {
    echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${MAGENTA}║${NC}  💄 WIA-IND-004: Beauty Tech Standard v${VERSION}        ${MAGENTA}║${NC}"
    echo -e "${MAGENTA}║${NC}  弘益人間 (Benefit All Humanity)                          ${MAGENTA}║${NC}"
    echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_usage() {
    print_header
    cat << EOF
Usage: $SCRIPT_NAME <command> [options]

Commands:
  ${GREEN}calc-skin-health${NC}        Calculate overall skin health score
  ${GREEN}calc-skin-age${NC}           Calculate estimated skin age
  ${GREEN}calc-hydration${NC}          Calculate hydration score
  ${GREEN}calc-pore-score${NC}         Calculate pore quality score
  ${GREEN}calc-color-match${NC}        Calculate foundation color match (ΔE)
  ${GREEN}calc-melanin-index${NC}      Calculate melanin index from RGB
  ${GREEN}calc-ita${NC}                Calculate Individual Typology Angle
  ${GREEN}recommend-routine${NC}       Get personalized skincare routine
  ${GREEN}analyze-ingredient${NC}      Analyze cosmetic ingredient safety
  ${GREEN}calc-led-dose${NC}           Calculate LED therapy light dose
  ${GREEN}track-progress${NC}          Calculate improvement percentage
  ${GREEN}help${NC}                    Show this help message
  ${GREEN}version${NC}                 Show version information

Examples:
  $SCRIPT_NAME calc-skin-health --hydration 75 --elasticity 80 --texture 70
  $SCRIPT_NAME calc-color-match --l1 68 --a1 8 --b1 18 --l2 70 --a2 7 --b2 19
  $SCRIPT_NAME calc-melanin-index --red 180 --green 150 --blue 120
  $SCRIPT_NAME calc-led-dose --power 30 --time 15 --area 400

For detailed help on each command:
  $SCRIPT_NAME <command> --help

EOF
}

print_error() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

################################################################################
# Calculation Functions
################################################################################

# 피부 건강 점수 계산 (Skin Health Score Calculation)
calc_skin_health_score() {
    local hydration=$1
    local elasticity=$2
    local texture=$3
    local tone=$4
    local pore=$5

    # Skin Health Score = Hydration × 0.25 + Elasticity × 0.25 + Texture × 0.20 + Tone × 0.15 + Pore × 0.15
    local score=$(echo "scale=2; ($hydration * 0.25) + ($elasticity * 0.25) + ($texture * 0.20) + ($tone * 0.15) + ($pore * 0.15)" | bc)

    echo "$score"
}

# 피부 나이 계산 (Skin Age Calculation)
calc_skin_age() {
    local chronological_age=$1
    local wrinkle_score=$2
    local elasticity_score=$3
    local pigmentation_score=$4
    local texture_score=$5

    # Expected values for age (baseline)
    local wrinkle_expected=$(echo "scale=2; $chronological_age * 0.5" | bc)
    local elasticity_expected=$(echo "scale=2; 100 - ($chronological_age * 0.4)" | bc)
    local pigmentation_expected=$(echo "scale=2; 90 - ($chronological_age * 0.3)" | bc)
    local texture_expected=$(echo "scale=2; 95 - ($chronological_age * 0.35)" | bc)

    # Calculate age offset
    local wrinkle_offset=$(echo "scale=2; ($wrinkle_score - $wrinkle_expected) * 0.35" | bc)
    local elasticity_offset=$(echo "scale=2; ($elasticity_expected - $elasticity_score) * 0.30" | bc)
    local pigmentation_offset=$(echo "scale=2; ($pigmentation_score - $pigmentation_expected) * 0.20" | bc)
    local texture_offset=$(echo "scale=2; ($texture_expected - $texture_score) * 0.15" | bc)

    local total_offset=$(echo "scale=2; $wrinkle_offset + $elasticity_offset + $pigmentation_offset + $texture_offset" | bc)
    local skin_age=$(echo "scale=1; $chronological_age + $total_offset" | bc)

    echo "$skin_age"
}

# 멜라닌 인덱스 계산 (Melanin Index Calculation)
calc_melanin_index() {
    local red=$1
    local green=$2

    # MI = -100 × log₁₀((1/R) / (1/G))
    # Simplified: MI = 100 × log₁₀(R/G)
    local ratio=$(echo "scale=6; $red / $green" | bc)
    local log_ratio=$(echo "scale=4; l($ratio) / l(10)" | bc -l)
    local mi=$(echo "scale=2; 100 * $log_ratio" | bc)

    echo "$mi"
}

# ITA (Individual Typology Angle) 계산
calc_ita() {
    local l_star=$1
    local b_star=$2

    # ITA = [arctan((L* - 50) / b*)] × (180 / π)
    local numerator=$(echo "scale=4; $l_star - 50" | bc)
    local atan_result=$(echo "scale=6; a($numerator / $b_star)" | bc -l)
    local ita=$(echo "scale=2; $atan_result * 180 / 3.141592653589793" | bc)

    echo "$ita"
}

# 색상 매칭 점수 계산 (Color Match Score - Delta E)
calc_delta_e() {
    local l1=$1
    local a1=$2
    local b1=$3
    local l2=$4
    local a2=$5
    local b2=$6

    # ΔE = √((L₁* - L₂*)² + (a₁* - a₂*)² + (b₁* - b₂*)²)
    local dl=$(echo "scale=4; $l1 - $l2" | bc)
    local da=$(echo "scale=4; $a1 - $a2" | bc)
    local db=$(echo "scale=4; $b1 - $b2" | bc)

    local dl_sq=$(echo "scale=4; $dl * $dl" | bc)
    local da_sq=$(echo "scale=4; $da * $da" | bc)
    local db_sq=$(echo "scale=4; $db * $db" | bc)

    local sum=$(echo "scale=4; $dl_sq + $da_sq + $db_sq" | bc)
    local delta_e=$(echo "scale=2; sqrt($sum)" | bc)

    echo "$delta_e"
}

# 모공 품질 점수 계산 (Pore Quality Score)
calc_pore_quality_score() {
    local pore_count=$1
    local avg_diameter=$2
    local distribution_variance=$3

    # Count normalized (baseline: 20 pores/cm²)
    local count_norm=$(echo "scale=2; if ($pore_count < 20) 0 else ($pore_count - 20) * 2" | bc)
    if (( $(echo "$count_norm > 100" | bc -l) )); then
        count_norm=100
    fi

    # Size normalized (baseline: 200 μm)
    local size_norm=$(echo "scale=2; if ($avg_diameter < 200) 0 else ($avg_diameter - 200) / 3" | bc)
    if (( $(echo "$size_norm > 100" | bc -l) )); then
        size_norm=100
    fi

    # Pore Score = 100 - (Count_normalized × 0.4 + Size_normalized × 0.4 + Distribution_variance × 0.2)
    local pore_score=$(echo "scale=2; 100 - (($count_norm * 0.4) + ($size_norm * 0.4) + ($distribution_variance * 0.2))" | bc)

    echo "$pore_score"
}

# LED 치료 광 용량 계산 (LED Therapy Light Dose)
calc_led_dose() {
    local power_density=$1  # mW/cm²
    local time_minutes=$2   # minutes
    local area_cm2=$3       # cm²

    # Dose (J/cm²) = Power density (mW/cm²) × Time (seconds) / 1000
    local time_seconds=$(echo "scale=2; $time_minutes * 60" | bc)
    local dose=$(echo "scale=2; ($power_density * $time_seconds) / 1000" | bc)

    # Total energy delivered
    local total_energy=$(echo "scale=2; $dose * $area_cm2" | bc)

    echo "$dose $total_energy"
}

# 개선도 계산 (Improvement Percentage)
calc_improvement() {
    local baseline=$1
    local current=$2

    # Improvement % = ((Current - Baseline) / Baseline) × 100
    local diff=$(echo "scale=4; $current - $baseline" | bc)
    local improvement=$(echo "scale=2; ($diff / $baseline) * 100" | bc)

    echo "$improvement"
}

################################################################################
# Command Implementations
################################################################################

cmd_calc_skin_health() {
    local hydration=0
    local elasticity=0
    local texture=0
    local tone=85
    local pore=75

    while [[ $# -gt 0 ]]; do
        case $1 in
            --hydration) hydration=$2; shift 2 ;;
            --elasticity) elasticity=$2; shift 2 ;;
            --texture) texture=$2; shift 2 ;;
            --tone) tone=$2; shift 2 ;;
            --pore) pore=$2; shift 2 ;;
            --help)
                echo "Calculate overall skin health score (0-100)"
                echo ""
                echo "Usage: $SCRIPT_NAME calc-skin-health [options]"
                echo ""
                echo "Options:"
                echo "  --hydration NUM    Hydration score (0-100)"
                echo "  --elasticity NUM   Elasticity score (0-100)"
                echo "  --texture NUM      Texture smoothness (0-100)"
                echo "  --tone NUM         Tone evenness (0-100, default: 85)"
                echo "  --pore NUM         Pore quality (0-100, default: 75)"
                echo ""
                echo "Formula:"
                echo "  Score = Hydration×0.25 + Elasticity×0.25 + Texture×0.20 + Tone×0.15 + Pore×0.15"
                exit 0
                ;;
            *) print_error "Unknown option: $1" ;;
        esac
    done

    [[ $hydration -eq 0 ]] && print_error "Missing --hydration parameter"
    [[ $elasticity -eq 0 ]] && print_error "Missing --elasticity parameter"
    [[ $texture -eq 0 ]] && print_error "Missing --texture parameter"

    local score=$(calc_skin_health_score $hydration $elasticity $texture $tone $pore)

    echo ""
    echo "═══════════════════════════════════════"
    echo "  💄 Skin Health Score Analysis"
    echo "═══════════════════════════════════════"
    echo ""
    echo "Component Scores:"
    echo "  Hydration:    ${hydration}/100 (weight: 25%)"
    echo "  Elasticity:   ${elasticity}/100 (weight: 25%)"
    echo "  Texture:      ${texture}/100 (weight: 20%)"
    echo "  Tone:         ${tone}/100 (weight: 15%)"
    echo "  Pore Quality: ${pore}/100 (weight: 15%)"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    if (( $(echo "$score >= 90" | bc -l) )); then
        echo -e "Overall Score: ${GREEN}${score}/100 ✨${NC}"
        echo -e "Rating: ${GREEN}Excellent${NC} - Optimal skin health"
    elif (( $(echo "$score >= 80" | bc -l) )); then
        echo -e "Overall Score: ${GREEN}${score}/100${NC}"
        echo -e "Rating: ${GREEN}Very Good${NC} - Minor improvements possible"
    elif (( $(echo "$score >= 70" | bc -l) )); then
        echo -e "Overall Score: ${YELLOW}${score}/100${NC}"
        echo -e "Rating: ${YELLOW}Good${NC} - Some concerns to address"
    elif (( $(echo "$score >= 60" | bc -l) )); then
        echo -e "Overall Score: ${YELLOW}${score}/100${NC}"
        echo -e "Rating: ${YELLOW}Fair${NC} - Multiple concerns"
    else
        echo -e "Overall Score: ${RED}${score}/100${NC}"
        echo -e "Rating: ${RED}Poor${NC} - Significant concerns"
    fi
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

cmd_calc_skin_age() {
    local age=0
    local wrinkle=0
    local elasticity=0
    local pigmentation=85
    local texture=80

    while [[ $# -gt 0 ]]; do
        case $1 in
            --age) age=$2; shift 2 ;;
            --wrinkle) wrinkle=$2; shift 2 ;;
            --elasticity) elasticity=$2; shift 2 ;;
            --pigmentation) pigmentation=$2; shift 2 ;;
            --texture) texture=$2; shift 2 ;;
            --help)
                echo "Calculate estimated skin age based on measurements"
                echo ""
                echo "Usage: $SCRIPT_NAME calc-skin-age [options]"
                echo ""
                echo "Options:"
                echo "  --age NUM            Chronological age (required)"
                echo "  --wrinkle NUM        Wrinkle score 0-100 (required)"
                echo "  --elasticity NUM     Elasticity score 0-100 (required)"
                echo "  --pigmentation NUM   Pigmentation evenness 0-100 (default: 85)"
                echo "  --texture NUM        Texture smoothness 0-100 (default: 80)"
                exit 0
                ;;
            *) print_error "Unknown option: $1" ;;
        esac
    done

    [[ $age -eq 0 ]] && print_error "Missing --age parameter"
    [[ $wrinkle -eq 0 ]] && print_error "Missing --wrinkle parameter"
    [[ $elasticity -eq 0 ]] && print_error "Missing --elasticity parameter"

    local skin_age=$(calc_skin_age $age $wrinkle $elasticity $pigmentation $texture)
    local diff=$(echo "scale=1; $skin_age - $age" | bc)

    echo ""
    echo "═══════════════════════════════════════"
    echo "  📊 Skin Age Analysis"
    echo "═══════════════════════════════════════"
    echo ""
    echo "Chronological Age: ${age} years"
    echo "Estimated Skin Age: ${skin_age} years"
    echo ""

    if (( $(echo "$diff < -2" | bc -l) )); then
        local younger=$(echo "scale=1; -1 * $diff" | bc)
        echo -e "Result: ${GREEN}Skin is ${younger} years younger! 🎉${NC}"
    elif (( $(echo "$diff > 2" | bc -l) )); then
        echo -e "Result: ${YELLOW}Skin is ${diff} years older ⚠️${NC}"
        echo ""
        echo "Recommendations:"
        echo "  • Consider anti-aging treatments (retinol, peptides)"
        echo "  • Increase sun protection (SPF 50+ daily)"
        echo "  • Add antioxidants to routine"
    else
        echo -e "Result: ${GREEN}Skin age matches chronological age ✓${NC}"
    fi
    echo ""
}

cmd_calc_color_match() {
    local l1=0 a1=0 b1=0 l2=0 a2=0 b2=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --l1) l1=$2; shift 2 ;;
            --a1) a1=$2; shift 2 ;;
            --b1) b1=$2; shift 2 ;;
            --l2) l2=$2; shift 2 ;;
            --a2) a2=$2; shift 2 ;;
            --b2) b2=$2; shift 2 ;;
            --help)
                echo "Calculate color match score (Delta E) in CIELAB color space"
                echo ""
                echo "Usage: $SCRIPT_NAME calc-color-match [options]"
                echo ""
                echo "Options:"
                echo "  --l1 NUM    Lightness of color 1 (0-100)"
                echo "  --a1 NUM    Green-Red axis of color 1 (-128 to +127)"
                echo "  --b1 NUM    Blue-Yellow axis of color 1 (-128 to +127)"
                echo "  --l2 NUM    Lightness of color 2 (0-100)"
                echo "  --a2 NUM    Green-Red axis of color 2 (-128 to +127)"
                echo "  --b2 NUM    Blue-Yellow axis of color 2 (-128 to +127)"
                echo ""
                echo "Interpretation:"
                echo "  ΔE < 1    = Not perceptible (exact match)"
                echo "  ΔE 1-2    = Perceptible with close observation"
                echo "  ΔE 2-3.5  = Perceptible at a glance"
                echo "  ΔE 3.5-5  = Clear difference"
                echo "  ΔE > 5    = Different colors"
                exit 0
                ;;
            *) print_error "Unknown option: $1" ;;
        esac
    done

    local delta_e=$(calc_delta_e $l1 $a1 $b1 $l2 $a2 $b2)
    local match_score=$(echo "scale=2; 100 * e(-1 * $delta_e / 10)" | bc -l)

    echo ""
    echo "═══════════════════════════════════════"
    echo "  🎨 Foundation Color Match Analysis"
    echo "═══════════════════════════════════════"
    echo ""
    echo "Skin Tone (LAB):   L*=${l1}, a*=${a1}, b*=${b1}"
    echo "Foundation (LAB):  L*=${l2}, a*=${a2}, b*=${b2}"
    echo ""
    echo "Delta E (ΔE):      ${delta_e}"
    echo "Match Score:       ${match_score}/100"
    echo ""

    if (( $(echo "$delta_e < 1" | bc -l) )); then
        echo -e "Match Quality: ${GREEN}Perfect Match ✨${NC}"
        echo "Not perceptible to human eye"
    elif (( $(echo "$delta_e < 2" | bc -l) )); then
        echo -e "Match Quality: ${GREEN}Excellent Match${NC}"
        echo "Perceptible only with close observation"
    elif (( $(echo "$delta_e < 3.5" | bc -l) )); then
        echo -e "Match Quality: ${GREEN}Good Match${NC}"
        echo "Perceptible at a glance"
    elif (( $(echo "$delta_e < 5" | bc -l) )); then
        echo -e "Match Quality: ${YELLOW}Fair Match${NC}"
        echo "Clear difference visible"
    else
        echo -e "Match Quality: ${RED}Poor Match${NC}"
        echo "Colors are noticeably different"
    fi
    echo ""
}

cmd_calc_melanin_index() {
    local red=0 green=0 blue=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --red) red=$2; shift 2 ;;
            --green) green=$2; shift 2 ;;
            --blue) blue=$2; shift 2 ;;
            --help)
                echo "Calculate Melanin Index from RGB values"
                echo ""
                echo "Usage: $SCRIPT_NAME calc-melanin-index [options]"
                echo ""
                echo "Options:"
                echo "  --red NUM      Red channel value (0-255)"
                echo "  --green NUM    Green channel value (0-255)"
                echo "  --blue NUM     Blue channel value (0-255)"
                echo ""
                echo "Interpretation (Fitzpatrick Type):"
                echo "  MI < 35    = Type I (Very fair)"
                echo "  MI 35-42   = Type II (Fair)"
                echo "  MI 42-48   = Type III (Medium)"
                echo "  MI 48-55   = Type IV (Olive)"
                echo "  MI 55-65   = Type V (Brown)"
                echo "  MI > 65    = Type VI (Very dark)"
                exit 0
                ;;
            *) print_error "Unknown option: $1" ;;
        esac
    done

    [[ $red -eq 0 ]] && print_error "Missing --red parameter"
    [[ $green -eq 0 ]] && print_error "Missing --green parameter"

    local mi=$(calc_melanin_index $red $green)

    echo ""
    echo "═══════════════════════════════════════"
    echo "  🌈 Melanin Index Analysis"
    echo "═══════════════════════════════════════"
    echo ""
    echo "RGB Values: R=${red}, G=${green}, B=${blue}"
    echo "Melanin Index: ${mi}"
    echo ""

    if (( $(echo "$mi < 35" | bc -l) )); then
        echo -e "Fitzpatrick Type: ${CYAN}I - Very Fair${NC}"
        echo "Characteristics: Pale white, freckles, always burns"
    elif (( $(echo "$mi < 42" | bc -l) )); then
        echo -e "Fitzpatrick Type: ${CYAN}II - Fair${NC}"
        echo "Characteristics: White, burns easily"
    elif (( $(echo "$mi < 48" | bc -l) )); then
        echo -e "Fitzpatrick Type: ${CYAN}III - Medium${NC}"
        echo "Characteristics: Cream white, sometimes burns"
    elif (( $(echo "$mi < 55" | bc -l) )); then
        echo -e "Fitzpatrick Type: ${CYAN}IV - Olive${NC}"
        echo "Characteristics: Moderate brown, rarely burns"
    elif (( $(echo "$mi < 65" | bc -l) )); then
        echo -e "Fitzpatrick Type: ${CYAN}V - Brown${NC}"
        echo "Characteristics: Dark brown, very rarely burns"
    else
        echo -e "Fitzpatrick Type: ${CYAN}VI - Very Dark${NC}"
        echo "Characteristics: Deeply pigmented, never burns"
    fi
    echo ""
}

cmd_calc_ita() {
    local l=0 b=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --l) l=$2; shift 2 ;;
            --b) b=$2; shift 2 ;;
            --help)
                echo "Calculate Individual Typology Angle (ITA)"
                echo ""
                echo "Usage: $SCRIPT_NAME calc-ita [options]"
                echo ""
                echo "Options:"
                echo "  --l NUM    L* (Lightness) value (0-100)"
                echo "  --b NUM    b* (Blue-Yellow) value (-128 to +127)"
                echo ""
                echo "Classification:"
                echo "  ITA > 55°   = Very light"
                echo "  ITA 41-55°  = Light"
                echo "  ITA 28-41°  = Intermediate"
                echo "  ITA 19-28°  = Tan"
                echo "  ITA 10-19°  = Brown"
                echo "  ITA < 10°   = Dark"
                exit 0
                ;;
            *) print_error "Unknown option: $1" ;;
        esac
    done

    [[ $l -eq 0 ]] && print_error "Missing --l parameter"
    [[ $b -eq 0 ]] && print_error "Missing --b parameter"

    local ita=$(calc_ita $l $b)

    echo ""
    echo "═══════════════════════════════════════"
    echo "  📐 ITA (Individual Typology Angle)"
    echo "═══════════════════════════════════════"
    echo ""
    echo "L* (Lightness): ${l}"
    echo "b* (Blue-Yellow): ${b}"
    echo "ITA: ${ita}°"
    echo ""

    if (( $(echo "$ita > 55" | bc -l) )); then
        echo "Classification: Very Light"
    elif (( $(echo "$ita > 41" | bc -l) )); then
        echo "Classification: Light"
    elif (( $(echo "$ita > 28" | bc -l) )); then
        echo "Classification: Intermediate"
    elif (( $(echo "$ita > 19" | bc -l) )); then
        echo "Classification: Tan"
    elif (( $(echo "$ita > 10" | bc -l) )); then
        echo "Classification: Brown"
    else
        echo "Classification: Dark"
    fi
    echo ""
}

cmd_version() {
    print_header
    echo "Standard: WIA-IND-004"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo "Author: WIA Beauty Technology Research Group"
    echo ""
    echo "弘익人間 (홍익인간) · Benefit All Humanity"
    echo ""
}

################################################################################
# Main Entry Point
################################################################################

main() {
    if [[ $# -eq 0 ]]; then
        print_usage
        exit 0
    fi

    local command=$1
    shift

    case $command in
        calc-skin-health)
            cmd_calc_skin_health "$@"
            ;;
        calc-skin-age)
            cmd_calc_skin_age "$@"
            ;;
        calc-color-match)
            cmd_calc_color_match "$@"
            ;;
        calc-melanin-index)
            cmd_calc_melanin_index "$@"
            ;;
        calc-ita)
            cmd_calc_ita "$@"
            ;;
        help|--help|-h)
            print_usage
            ;;
        version|--version|-v)
            cmd_version
            ;;
        *)
            print_error "Unknown command: $command\n\nRun '$SCRIPT_NAME help' for usage information."
            ;;
    esac
}

main "$@"
