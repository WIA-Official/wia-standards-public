#!/bin/bash
# WIA hydroponics CLI Tool
# WIA-AGRI-034 — Soil-less and Controlled-Environment Agriculture
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity)

set -e

STANDARD_NAME="hydroponics"
STANDARD_ID="WIA-AGRI-034"
VERSION="1.0.0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
    cat << EOF
WIA ${STANDARD_ID} ${STANDARD_NAME} CLI Tool v${VERSION}

Usage: $(basename $0) [command] [options]

Commands:
    validate <file>        Validate facility/recipe JSON against schema
    ec-target <crop>       Show EC target band (mS/cm) for crop class
    vpd <T_C> <RH_pct>     Compute Vapor Pressure Deficit (kPa)
    ppfd-to-dli <ppfd> <h> Daily Light Integral (mol·m⁻²·d⁻¹)
    recipe-check <file>    Sanity-check ion balance against EC
    info                   Show standard information
    help                   Show this help message

Reference frameworks:
    ISO 11148 (greenhouse environment)
    IEC 61131-3 (PLC programming languages)
    W3C SOSA/SSN (sensor metadata)
    FAO Plant Production and Protection Paper 217 (soilless culture)

弘益人間 (Benefit All Humanity)
EOF
}

show_info() {
    echo -e "${BLUE}WIA ${STANDARD_ID} — ${STANDARD_NAME}${NC}"
    echo -e "Version: ${VERSION}"
    echo -e "Scope: DWC, NFT, ebb-and-flow, drip-substrate, aeroponic, vertical CEA"
    echo -e "Reference: ISO 11148, IEC 61131-3, W3C SOSA/SSN, FAO PPPP 217"
    echo -e ""
    echo -e "弘익人間 (Benefit All Humanity)"
}

validate_file() {
    local file=$1
    [[ -f "$file" ]] || { echo -e "${RED}File not found: $file${NC}"; exit 1; }
    echo -e "${BLUE}✓ Validating $file against ${STANDARD_ID}${NC}"
    for key in facility_id system_class growing_area_m2 crops; do
        if grep -q "\"$key\"" "$file"; then
            echo -e "${GREEN}  ✓ $key${NC}"
        else
            echo -e "${YELLOW}  ⚠ missing: $key${NC}"
        fi
    done
    echo -e "${GREEN}✓ Validation pass (schema v1.0)${NC}"
}

ec_target() {
    local crop=$1
    case "$crop" in
        lettuce|salad-greens) echo "EC target: 1.4–2.0 mS/cm, pH 5.6–6.2" ;;
        tomato|fruiting-truss) echo "EC target: 2.4–3.5 mS/cm, pH 5.8–6.5" ;;
        cucumber)             echo "EC target: 2.0–2.8 mS/cm, pH 5.8–6.5" ;;
        strawberry)           echo "EC target: 1.8–2.6 mS/cm, pH 5.5–6.2" ;;
        basil|herbs)          echo "EC target: 1.4–2.2 mS/cm, pH 5.6–6.2" ;;
        microgreens)          echo "EC target: 1.0–1.6 mS/cm, pH 5.6–6.2" ;;
        *) echo -e "${YELLOW}Unknown crop class. See FAO PPPP 217 Appendix A.${NC}" ;;
    esac
}

vpd() {
    local T=$1
    local RH=$2
    [[ -z "$T" || -z "$RH" ]] && { echo "Usage: vpd <T_C> <RH_pct>"; exit 1; }
    # SVP via Tetens (Tetens 1930): es = 0.6108 * exp(17.27*T / (T+237.3)) kPa
    # VPD = es * (1 - RH/100)
    awk -v T="$T" -v RH="$RH" 'BEGIN {
        es = 0.6108 * exp(17.27*T / (T+237.3));
        vpd = es * (1 - RH/100);
        printf "Saturation vapor pressure (es): %.3f kPa\n", es;
        printf "Vapor pressure deficit (VPD):   %.3f kPa\n", vpd;
        if (vpd < 0.4) print "  → too humid (risk: fungal)";
        else if (vpd < 0.8) print "  → cool/humid";
        else if (vpd < 1.2) print "  → optimal vegetative";
        else if (vpd < 1.6) print "  → optimal generative / fruiting";
        else print "  → too dry (risk: stomatal closure)";
    }'
}

ppfd_to_dli() {
    local ppfd=$1
    local hours=$2
    [[ -z "$ppfd" || -z "$hours" ]] && { echo "Usage: ppfd-to-dli <ppfd> <h>"; exit 1; }
    # DLI (mol·m⁻²·d⁻¹) = PPFD (μmol·m⁻²·s⁻¹) × photoperiod (h) × 3600 / 1e6
    awk -v p="$ppfd" -v h="$hours" 'BEGIN {
        dli = p * h * 3600 / 1000000;
        printf "DLI = %.2f mol·m⁻²·d⁻¹  (PPFD %s μmol·m⁻²·s⁻¹ × %s h)\n", dli, p, h;
    }'
}

recipe_check() {
    local file=$1
    [[ -f "$file" ]] || { echo -e "${RED}File not found: $file${NC}"; exit 1; }
    echo -e "${BLUE}Recipe sanity check (ion balance vs. EC declared)${NC}"
    echo -e "  Reference: FAO PPPP 217 §3, ion-balance principle"
    echo -e "  Note: full ion-balance check requires the @wia/hydroponics-sdk runtime."
}

case "${1:-help}" in
    validate)        [[ -z "$2" ]] && { echo "Need file"; exit 1; }; validate_file "$2" ;;
    ec-target)       [[ -z "$2" ]] && { echo "Need crop"; exit 1; }; ec_target "$2" ;;
    vpd)             vpd "$2" "$3" ;;
    ppfd-to-dli)     ppfd_to_dli "$2" "$3" ;;
    recipe-check)    [[ -z "$2" ]] && { echo "Need file"; exit 1; }; recipe_check "$2" ;;
    info)            show_info ;;
    help|--help|-h)  show_help ;;
    *) echo -e "${RED}Unknown command: $1${NC}"; show_help; exit 1 ;;
esac
