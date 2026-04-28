#!/bin/bash

################################################################################
# WIA-SPACE-009: Space Radiation CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Space Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool surfaces space-radiation envelope operations: dose
# bookkeeping, mission radiation budget, SPE alert ingest, dosimeter
# state, and the ICRP/NCRP/NASA-STD-3001 risk-criterion check.
################################################################################

set -e

# Colors
INDIGO='\033[0;94m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

VERSION="1.0.0"
ENDPOINT_BASE="${WIA_SPACE_009_API:-https://api.wiastandards.com/space-radiation/v1}"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        ☢️  WIA-SPACE-009: Space Radiation CLI                  ║"
    echo "║                      Version $VERSION                          ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() { echo -e "${GREEN}✓ $1${RESET}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${RESET}"; }
print_error()   { echo -e "${RED}✗ $1${RESET}"; }
print_info()    { echo -e "${GRAY}  $1${RESET}"; }

# ----------------------------------------------------------------------
# Dose-record envelope generator
# ----------------------------------------------------------------------
gen_dose_record() {
    local crew_id="${1:-CREW-001}"
    local absorbed_gy="${2:-0.0023}"
    local equiv_sv="${3:-0.0029}"
    local effective_sv="${4:-0.0027}"

    print_section "Dose-record envelope (Phase 1 §A.2)"

    cat <<JSON
{
  "kind": "wia-space-009/dose-record",
  "id": "$(uuidgen 2>/dev/null || echo 00000000-0000-7000-8000-000000000000)",
  "crew_member_id": "${crew_id}",
  "absorbed_dose_gy": ${absorbed_gy},
  "equivalent_dose_sv": ${equiv_sv},
  "effective_dose_sv": ${effective_sv},
  "weighting_factors": {
    "model": "ICRP-103",
    "w_R_proton": 2,
    "w_R_alpha": 20,
    "w_R_HZE": 20
  },
  "instrument_ref": "DOSIM-TLD-001",
  "uncertainty_k2": 0.15,
  "timestamp_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
JSON

    print_success "Dose record envelope emitted (per ICRP 103 + ISO/IEC Guide 98-3)."
}

# ----------------------------------------------------------------------
# Mission-radiation-budget computation (toy model)
# ----------------------------------------------------------------------
compute_mission_budget() {
    local trajectory="${1:-LEO-400km}"
    local duration_days="${2:-180}"
    local shielding_g_cm2="${3:-20}"

    print_section "Mission-radiation-budget envelope (Phase 1 §A.6)"

    print_info "Trajectory:           ${trajectory}"
    print_info "Duration (days):      ${duration_days}"
    print_info "Shielding (g/cm²):    ${shielding_g_cm2}"

    # Toy GCR dose rate (mSv/day) per shielding
    local gcr_dose_rate
    case "${trajectory}" in
        LEO-400km)        gcr_dose_rate=0.30 ;;
        cislunar)         gcr_dose_rate=1.20 ;;
        lunar-surface)    gcr_dose_rate=0.60 ;;
        mars-transit)     gcr_dose_rate=1.85 ;;
        mars-surface)     gcr_dose_rate=0.65 ;;
        *)                gcr_dose_rate=0.50 ;;
    esac

    # Linear toy model: shielding scaling
    local total_msv
    total_msv=$(awk -v r="${gcr_dose_rate}" -v d="${duration_days}" -v s="${shielding_g_cm2}" \
        'BEGIN{ printf "%.2f", r*d*(20.0/(s+20.0)) }')

    print_success "GCR cumulative (toy model): ${total_msv} mSv"
    print_warning "SPE add-on (worst-case 1972/1989/2003): +200 mSv unsheltered"
    print_info  "Rigorous projection requires HZETRN 2015 + AE-9/AP-9 (Phase 4 §A.1)."
}

# ----------------------------------------------------------------------
# Risk-criterion check vs NASA-STD-3001 + NCRP 132
# ----------------------------------------------------------------------
risk_criterion_check() {
    local cum_sv="${1:-0.30}"
    local age="${2:-40}"
    local sex="${3:-M}"

    print_section "Risk-criterion check (Phase 1 §A.3)"

    # NASA-STD-3001 Vol 1 career-limit (3% REID)
    local limit_sv
    if [[ "${sex}" = "F" ]]; then
        # Female limits (lower)
        if [[ "${age}" -lt 30 ]]; then limit_sv=0.6
        elif [[ "${age}" -lt 40 ]]; then limit_sv=0.9
        elif [[ "${age}" -lt 50 ]]; then limit_sv=1.7
        else limit_sv=3.0
        fi
    else
        # Male limits
        if [[ "${age}" -lt 30 ]]; then limit_sv=0.7
        elif [[ "${age}" -lt 40 ]]; then limit_sv=1.0
        elif [[ "${age}" -lt 50 ]]; then limit_sv=2.0
        else limit_sv=4.0
        fi
    fi

    local fraction
    fraction=$(awk -v c="${cum_sv}" -v l="${limit_sv}" 'BEGIN{ printf "%.1f", 100*c/l }')

    print_info "Cumulative effective dose: ${cum_sv} Sv"
    print_info "NASA career limit:         ${limit_sv} Sv (age ${age}, sex ${sex})"
    print_info "Utilisation:               ${fraction}%"

    awk -v c="${cum_sv}" -v l="${limit_sv}" 'BEGIN{ exit !(c < 0.5*l) }' \
        && print_success "Within green band (<50% of NASA-STD-3001 career limit)." \
        || awk -v c="${cum_sv}" -v l="${limit_sv}" 'BEGIN{ exit !(c < 0.75*l) }' \
        && print_warning "Yellow band (50-75%) — schedule medical review per NCRP 142." \
        || print_error  "Red band (>75%) — flight-rules deviation review required."
}

# ----------------------------------------------------------------------
# SPE alert format (Phase 2 §A.5)
# ----------------------------------------------------------------------
spe_alert_envelope() {
    local severity="${1:-S2}"
    local source="${2:-NOAA-SWPC}"

    print_section "SPE-alert envelope (Phase 2 §A.5)"

    cat <<JSON
{
  "kind": "wia-space-009/spe-alert",
  "id": "$(uuidgen 2>/dev/null || echo 00000000-0000-7000-8000-000000000000)",
  "scale": "NOAA-S",
  "severity": "${severity}",
  "source": "${source}",
  "onset_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "expected_duration_h": 36,
  "shelter_recommendation": "evaluate against per-mission flight-rules"
}
JSON

    print_success "SPE alert envelope emitted (downstream Phase 3 §A.2 shelter protocol)."
}

# ----------------------------------------------------------------------
# Dosimeter state probe
# ----------------------------------------------------------------------
dosimeter_state() {
    local id="${1:-DOSIM-TLD-001}"

    print_section "Dosimeter-state envelope (Phase 1 §A.5)"

    cat <<JSON
{
  "kind": "wia-space-009/dosimeter-state",
  "id": "${id}",
  "class": "TLD-LiF:Mg,Cu,P",
  "calibration": {
    "cert_id": "NIST-PT-DOSIM-2026Q1",
    "last_calib_utc": "2026-01-15T08:30:00Z",
    "next_due_utc":   "2026-07-15T00:00:00Z",
    "traceable_to":   "NIST + IAEA-TECDOC-1126"
  },
  "deployed_position": "intra-vehicular-port-3",
  "cumulative_mGy_eq": 12.4,
  "rolling_30d_mSv": 8.6
}
JSON

    print_success "Dosimeter state envelope emitted (per ISO/IEC 17025 + ANSI N13.11)."
}

# ----------------------------------------------------------------------
# Help
# ----------------------------------------------------------------------
show_help() {
    print_header
    cat <<HELP
Usage: wia-space-009 <command> [args]

Commands:
  dose <crew-id> <absorbed-Gy> <equiv-Sv> <effective-Sv>
      Emit a dose-record envelope (Phase 1 §A.2).

  budget <trajectory> <duration-days> <shielding-g-cm2>
      Toy mission-radiation-budget computation (Phase 1 §A.6).
      Trajectory ∈ {LEO-400km, cislunar, lunar-surface, mars-transit,
                    mars-surface}.

  risk <cumulative-Sv> <age> <sex M|F>
      Risk-criterion check vs NASA-STD-3001 Vol 1 + NCRP 132.

  spe <severity S1..S5> <source>
      SPE-alert envelope (Phase 2 §A.5).

  dosimeter <dosimeter-id>
      Dosimeter-state envelope (Phase 1 §A.5).

  help
      Show this help.

Environment:
  WIA_SPACE_009_API   API endpoint base (default: ${ENDPOINT_BASE})

References: NCRP 132 / 142 / 153 / 167 · ICRP 103 · NASA-STD-3001 ·
            HZETRN 2015 · IAEA-TECDOC-1126 · ISO/IEC 17025 · ANSI N13.11.

弘益人間 (Benefit All Humanity)
HELP
}

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
case "${1:-help}" in
    dose)         shift; gen_dose_record "$@" ;;
    budget)       shift; compute_mission_budget "$@" ;;
    risk)         shift; risk_criterion_check "$@" ;;
    spe)          shift; spe_alert_envelope "$@" ;;
    dosimeter)    shift; dosimeter_state "$@" ;;
    -h|--help|help|*) show_help ;;
esac
