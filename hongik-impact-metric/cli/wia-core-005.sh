#!/bin/bash

################################################################################
# WIA-CORE-005: Hongik Impact Metric CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to Hongik Impact assessment:
# - Multi-dimensional impact scoring
# - Project comparison and ranking
# - Accessibility evaluation
# - Impact report generation
################################################################################

set -e

# Colors for output
INDIGO='\033[0;35m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
GLOBAL_POPULATION=8000000000

# Dimension weights
WEIGHT_SG=0.20  # Social Good
WEIGHT_AC=0.20  # Accessibility
WEIGHT_SU=0.15  # Sustainability
WEIGHT_HW=0.15  # Health & Wellbeing
WEIGHT_EE=0.10  # Economic Equity
WEIGHT_ED=0.10  # Education
WEIGHT_IP=0.10  # Innovation

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🌟 WIA-CORE-005: Hongik Impact Metric CLI Tool          ║"
    echo "║                      Version $VERSION                            ║"
    echo "║                  弘益人間 · Benefit All Humanity                ║"
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
    echo -e "${PURPLE}  $1${RESET}"
}

# Calculate logarithm (base 10)
log10() {
    local x=$1
    echo "l($x)/l(10)" | bc -l
}

# Calculate stakeholder reach factor
calc_reach_factor() {
    local beneficiaries=$1

    if (( $(echo "$beneficiaries <= 0" | bc -l) )); then
        echo "1.0"
        return
    fi

    local log_b=$(log10 $beneficiaries)
    local log_global=$(log10 $GLOBAL_POPULATION)
    local srf=$(echo "scale=4; 1 + 9 * ($log_b / $log_global)" | bc -l)

    # Clamp between 1 and 10
    if (( $(echo "$srf < 1" | bc -l) )); then
        echo "1.0"
    elif (( $(echo "$srf > 10" | bc -l) )); then
        echo "10.0"
    else
        echo "$srf"
    fi
}

# Calculate temporal factor
calc_temporal_factor() {
    local duration=$1

    case $duration in
        temporary|temp)
            echo "0.55"
            ;;
        short|short-term)
            echo "0.75"
            ;;
        medium|medium-term)
            echo "0.95"
            ;;
        long|long-term)
            echo "1.20"
            ;;
        permanent|perm)
            echo "1.45"
            ;;
        *)
            echo "0.95"  # Default to medium-term
            ;;
    esac
}

# Classify score
classify_score() {
    local score=$1

    if (( $(echo "$score >= 900" | bc -l) )); then
        echo "🌟 Exceptional"
    elif (( $(echo "$score >= 800" | bc -l) )); then
        echo "⭐⭐⭐ Elite"
    elif (( $(echo "$score >= 700" | bc -l) )); then
        echo "⭐⭐ High"
    elif (( $(echo "$score >= 600" | bc -l) )); then
        echo "⭐ Good"
    elif (( $(echo "$score >= 500" | bc -l) )); then
        echo "✓✓ Moderate"
    elif (( $(echo "$score >= 400" | bc -l) )); then
        echo "✓ Fair"
    elif (( $(echo "$score >= 300" | bc -l) )); then
        echo "~ Limited"
    elif (( $(echo "$score >= 200" | bc -l) )); then
        echo "⚠ Minimal"
    elif (( $(echo "$score >= 100" | bc -l) )); then
        echo "⚠⚠ Poor"
    else
        echo "❌ Harmful"
    fi
}

# Determine certification level
get_certification() {
    local score=$1

    if (( $(echo "$score >= 900" | bc -l) )); then
        echo "💎 Diamond"
    elif (( $(echo "$score >= 800" | bc -l) )); then
        echo "🏆 Platinum"
    elif (( $(echo "$score >= 700" | bc -l) )); then
        echo "🥇 Gold"
    elif (( $(echo "$score >= 600" | bc -l) )); then
        echo "🥈 Silver"
    elif (( $(echo "$score >= 400" | bc -l) )); then
        echo "🥉 Bronze"
    else
        echo "None"
    fi
}

# Assess project impact
assess_impact() {
    local name="${1:-Unknown Project}"
    local sg=${2:-0.5}
    local ac=${3:-0.5}
    local su=${4:-0.5}
    local hw=${5:-0.5}
    local ee=${6:-0.5}
    local ed=${7:-0.5}
    local ip=${8:-0.5}
    local beneficiaries=${9:-1000}
    local duration=${10:-medium}

    print_section "Hongik Impact Assessment: $name"

    # Validate scores (0-1)
    for score in $sg $ac $su $hw $ee $ed $ip; do
        if (( $(echo "$score < 0 || $score > 1" | bc -l) )); then
            print_error "All dimension scores must be between 0 and 1"
            return 1
        fi
    done

    print_info "Dimension Scores (0-1):"
    print_metric "  Social Good:       $sg"
    print_metric "  Accessibility:     $ac"
    print_metric "  Sustainability:    $su"
    print_metric "  Health/Wellbeing:  $hw"
    print_metric "  Economic Equity:   $ee"
    print_metric "  Education:         $ed"
    print_metric "  Innovation:        $ip"
    echo ""
    print_info "Stakeholders:"
    print_metric "  Beneficiaries:     $(printf "%'d" $beneficiaries)"
    print_metric "  Duration:          $duration"

    # Calculate weighted scores
    local ws_sg=$(echo "scale=4; $sg * $WEIGHT_SG" | bc -l)
    local ws_ac=$(echo "scale=4; $ac * $WEIGHT_AC" | bc -l)
    local ws_su=$(echo "scale=4; $su * $WEIGHT_SU" | bc -l)
    local ws_hw=$(echo "scale=4; $hw * $WEIGHT_HW" | bc -l)
    local ws_ee=$(echo "scale=4; $ee * $WEIGHT_EE" | bc -l)
    local ws_ed=$(echo "scale=4; $ed * $WEIGHT_ED" | bc -l)
    local ws_ip=$(echo "scale=4; $ip * $WEIGHT_IP" | bc -l)

    # Calculate base score
    local base_score=$(echo "scale=4; $ws_sg + $ws_ac + $ws_su + $ws_hw + $ws_ee + $ws_ed + $ws_ip" | bc -l)

    # Calculate reach factor
    local reach_factor=$(calc_reach_factor $beneficiaries)

    # Calculate temporal factor
    local temporal_factor=$(calc_temporal_factor $duration)

    # Calculate Hongik Impact Score
    local his=$(echo "scale=1; $base_score * $reach_factor * $temporal_factor * 1000" | bc -l)

    # Calculate per-capita impact
    local per_capita=$(echo "scale=6; $his / $beneficiaries" | bc -l)

    # Classify
    local classification=$(classify_score $his)
    local certification=$(get_certification $his)

    print_section "Results"
    echo ""
    print_metric "  Base Score:        $(printf "%.4f" $base_score)"
    print_metric "  Reach Factor:      $(printf "%.2f" $reach_factor) / 10"
    print_metric "  Temporal Factor:   $(printf "%.2f" $temporal_factor)"
    echo ""
    print_success "Hongik Impact Score (HIS): $(printf "%.1f" $his) / 1000"
    echo ""
    print_info "Classification:    $classification"
    print_info "Certification:     $certification"
    print_info "Per-Capita Impact: $(printf "%.6f" $per_capita)"
    echo ""

    # Recommendations
    print_section "Impact Summary"
    if (( $(echo "$his >= 800" | bc -l) )); then
        print_success "Exceptional impact! This project significantly benefits humanity."
        print_info "Continue maintaining high standards and lead by example."
    elif (( $(echo "$his >= 600" | bc -l) )); then
        print_success "High impact! This project creates meaningful benefit."
        print_info "Consider expanding reach or improving weaker dimensions."
    elif (( $(echo "$his >= 400" | bc -l) )); then
        print_warning "Moderate impact. Room for significant improvement."
        print_info "Focus on accessibility, sustainability, and stakeholder reach."
    else
        print_warning "Limited impact. Major improvements needed."
        print_info "Reassess strategy and align with 弘益人間 principles."
    fi
    echo ""
}

# Calculate accessibility score
calc_accessibility() {
    local wcag="${1:-aa-partial}"
    local languages=${2:-1}
    local offline=${3:-false}
    local low_bandwidth=${4:-false}
    local economic=${5:-0.5}

    print_section "Accessibility Assessment"

    # WCAG score (25%)
    local wcag_score=0
    case $wcag in
        none)           wcag_score=0.0 ;;
        a-partial)      wcag_score=0.3 ;;
        a-full)         wcag_score=0.5 ;;
        aa-partial)     wcag_score=0.7 ;;
        aa-full|AA)     wcag_score=0.85 ;;
        aaa-partial)    wcag_score=0.95 ;;
        aaa-full|AAA)   wcag_score=1.0 ;;
        *)              wcag_score=0.7 ;;
    esac
    local digital=$(echo "scale=4; $wcag_score * 0.25" | bc -l)

    # Language score (15%)
    local lang_coverage=$(echo "scale=4; ($languages / 50)" | bc -l)
    if (( $(echo "$lang_coverage > 1" | bc -l) )); then
        lang_coverage=1.0
    fi
    local language=$(echo "scale=4; $lang_coverage * 0.15" | bc -l)

    # Economic accessibility (20%)
    local econ=$(echo "scale=4; $economic * 0.20" | bc -l)

    # Geographic accessibility (20%)
    local geo_base=0.5
    if [ "$low_bandwidth" = "true" ]; then
        geo_base=$(echo "scale=2; $geo_base + 0.25" | bc -l)
    fi
    if [ "$offline" = "true" ]; then
        geo_base=$(echo "scale=2; $geo_base + 0.25" | bc -l)
    fi
    local geographic=$(echo "scale=4; $geo_base * 0.20" | bc -l)

    # Physical accessibility (20% - assumed average)
    local physical=$(echo "scale=4; 0.7 * 0.20" | bc -l)

    # Total accessibility score
    local total=$(echo "scale=4; $digital + $language + $econ + $geographic + $physical" | bc -l)

    print_info "Component Scores:"
    print_metric "  Digital (WCAG):    $(printf "%.4f" $digital) (WCAG: $wcag)"
    print_metric "  Language:          $(printf "%.4f" $language) ($languages languages)"
    print_metric "  Economic:          $(printf "%.4f" $econ)"
    print_metric "  Geographic:        $(printf "%.4f" $geographic) (Offline: $offline, Low-BW: $low_bandwidth)"
    print_metric "  Physical:          $(printf "%.4f" $physical) (assumed)"
    echo ""
    print_success "Total Accessibility Score: $(printf "%.4f" $total)"

    if (( $(echo "$total >= 0.9" | bc -l) )); then
        print_info "Excellent! Highly accessible to all populations."
    elif (( $(echo "$total >= 0.7" | bc -l) )); then
        print_info "Good accessibility. Minor barriers remain."
    elif (( $(echo "$total >= 0.5" | bc -l) )); then
        print_info "Moderate accessibility. Significant improvements needed."
    else
        print_warning "Poor accessibility. Major barriers exist."
    fi
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-core-005 <command> [options]"
    echo ""
    echo "Commands:"
    echo ""
    echo "  assess              - Assess project impact"
    echo "  accessibility       - Calculate accessibility score"
    echo "  reach               - Calculate stakeholder reach factor"
    echo "  classify            - Classify impact score"
    echo "  version             - Show version"
    echo "  help                - Show this help"
    echo ""
    echo "Examples:"
    echo ""
    echo "  # Full assessment"
    echo "  wia-core-005 assess --name \"Healthcare App\" \\"
    echo "    --social-good 0.90 --accessibility 0.85 --sustainability 0.75 \\"
    echo "    --health 0.95 --economic-equity 0.70 --education 0.80 \\"
    echo "    --innovation 0.85 --beneficiaries 10000000"
    echo ""
    echo "  # Quick assessment with positional args"
    echo "  wia-core-005 assess \"My Project\" 0.8 0.9 0.7 0.85 0.75 0.8 0.85 1000000"
    echo ""
    echo "  # Accessibility check"
    echo "  wia-core-005 accessibility --wcag AA --languages 20 \\"
    echo "    --offline true --low-bandwidth true --economic 0.8"
    echo ""
    echo "  # Calculate reach factor"
    echo "  wia-core-005 reach 50000000"
    echo ""
    echo "  # Classify score"
    echo "  wia-core-005 classify 742.5"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo "© 2025 SmileStory Inc. / WIA - MIT License"
    echo ""
}

# Show version
show_version() {
    echo "WIA-CORE-005 Hongik Impact Metric CLI v${VERSION}"
    echo "弘益人間 · Benefit All Humanity"
}

# Parse named arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --name) PROJECT_NAME="$2"; shift 2 ;;
            --social-good) SOCIAL_GOOD="$2"; shift 2 ;;
            --accessibility) ACCESSIBILITY="$2"; shift 2 ;;
            --sustainability) SUSTAINABILITY="$2"; shift 2 ;;
            --health|--health-wellbeing) HEALTH="$2"; shift 2 ;;
            --economic-equity) ECONOMIC="$2"; shift 2 ;;
            --education) EDUCATION="$2"; shift 2 ;;
            --innovation) INNOVATION="$2"; shift 2 ;;
            --beneficiaries) BENEFICIARIES="$2"; shift 2 ;;
            --duration) DURATION="$2"; shift 2 ;;
            --wcag) WCAG="$2"; shift 2 ;;
            --languages) LANGUAGES="$2"; shift 2 ;;
            --offline) OFFLINE="$2"; shift 2 ;;
            --low-bandwidth) LOW_BANDWIDTH="$2"; shift 2 ;;
            --economic) ECONOMIC_ACC="$2"; shift 2 ;;
            *) shift ;;
        esac
    done
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        assess)
            print_header
            if [[ $1 =~ ^-- ]]; then
                # Named arguments
                parse_args "$@"
                assess_impact "${PROJECT_NAME:-Unknown}" \
                    "${SOCIAL_GOOD:-0.5}" "${ACCESSIBILITY:-0.5}" \
                    "${SUSTAINABILITY:-0.5}" "${HEALTH:-0.5}" \
                    "${ECONOMIC:-0.5}" "${EDUCATION:-0.5}" \
                    "${INNOVATION:-0.5}" "${BENEFICIARIES:-1000}" \
                    "${DURATION:-medium}"
            else
                # Positional arguments
                assess_impact "$@"
            fi
            ;;

        accessibility)
            print_header
            parse_args "$@"
            calc_accessibility "${WCAG:-aa-partial}" "${LANGUAGES:-1}" \
                "${OFFLINE:-false}" "${LOW_BANDWIDTH:-false}" \
                "${ECONOMIC_ACC:-0.5}"
            ;;

        reach)
            local beneficiaries=${1:-1000}
            local rf=$(calc_reach_factor $beneficiaries)
            echo "Beneficiaries: $(printf "%'d" $beneficiaries)"
            echo "Reach Factor: $(printf "%.2f" $rf) / 10"
            ;;

        classify)
            local score=${1:-500}
            echo "Score: $score / 1000"
            echo "Classification: $(classify_score $score)"
            echo "Certification: $(get_certification $score)"
            ;;

        version)
            show_version
            ;;

        help|--help|-h)
            show_help
            ;;

        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main
main "$@"
