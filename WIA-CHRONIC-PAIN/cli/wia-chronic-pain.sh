#!/bin/bash
#
# WIA-CHRONIC-PAIN CLI
# Neuroplasticity Reversal Chronic Pain Standard
#
# Version: 1.0.0
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

# Configuration
WIA_API_URL="${WIA_API_URL:-https://api.wia.live/chronic-pain/v1}"
WIA_API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/chronic-pain.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# ============================================================================
# Helper Functions
# ============================================================================

print_banner() {
    echo -e "${MAGENTA}"
    echo "╔═══════════════════════════════════════════════════════════════════════════╗"
    echo "║                        WIA-CHRONIC-PAIN CLI                               ║"
    echo "║              Neuroplasticity Reversal Pain Standard                       ║"
    echo "║                                                                           ║"
    echo "║                    弘益人間 - Benefit All Humanity                         ║"
    echo "╚═══════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_help() {
    print_banner
    echo "Usage: wia-chronic-pain <command> [options]"
    echo ""
    echo "Commands:"
    echo "  assess <patient_id>          Create comprehensive pain assessment"
    echo "  profile <patient_id>         Get patient chronic pain profile"
    echo "  sensitization <patient_id>   Get central sensitization status"
    echo "  nri <patient_id>             Calculate Neuroplasticity Reversal Index"
    echo "  neuromod <patient_id>        Generate neuromodulation plan"
    echo "  recommend <patient_id>       Get treatment recommendations"
    echo "  opioid-risk <patient_id>     Get opioid risk assessment"
    echo "  taper <patient_id>           Create opioid tapering plan"
    echo "  chronify <patient_id>        Predict chronification risk"
    echo "  validate <file>              Validate WIA-CHRONIC-PAIN JSON file"
    echo "  config                       Configure API credentials"
    echo "  help                         Show this help message"
    echo ""
    echo "Options:"
    echo "  --pain-type <type>    Pain type (nociceptive|neuropathic|nociplastic|mixed)"
    echo "  --format <fmt>        Output format (json|table|summary)"
    echo "  --output <file>       Write output to file"
    echo "  --verbose             Verbose output"
    echo ""
    echo "Examples:"
    echo "  wia-chronic-pain assess patient-123 --pain-type nociplastic"
    echo "  wia-chronic-pain nri patient-123 --format summary"
    echo "  wia-chronic-pain neuromod patient-123 --output plan.json"
    echo ""
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_api_key() {
    if [ -z "$WIA_API_KEY" ]; then
        if [ -f "$CONFIG_FILE" ]; then
            source "$CONFIG_FILE"
        fi
    fi

    if [ -z "$WIA_API_KEY" ]; then
        log_error "API key not configured. Run 'wia-chronic-pain config' first."
        exit 1
    fi
}

api_request() {
    local method="$1"
    local endpoint="$2"
    local data="$3"

    local curl_opts=(-s -X "$method")
    curl_opts+=(-H "Authorization: Bearer $WIA_API_KEY")
    curl_opts+=(-H "Content-Type: application/json")
    curl_opts+=(-H "Accept: application/json")

    if [ -n "$data" ]; then
        curl_opts+=(-d "$data")
    fi

    local response
    response=$(curl "${curl_opts[@]}" "${WIA_API_URL}${endpoint}" 2>/dev/null)

    if [ $? -ne 0 ]; then
        log_error "API request failed"
        exit 1
    fi

    echo "$response"
}

format_output() {
    local data="$1"
    local format="$2"

    case "$format" in
        json)
            echo "$data" | jq .
            ;;
        table)
            echo "$data" | jq -r 'to_entries | .[] | "\(.key): \(.value)"'
            ;;
        summary)
            format_summary "$data"
            ;;
        *)
            echo "$data" | jq .
            ;;
    esac
}

format_nri_summary() {
    local data="$1"

    local index=$(echo "$data" | jq -r '.current_index // 0')
    local trend=$(echo "$data" | jq -r '.trend // "unknown"')
    local reversal=$(echo "$data" | jq -r '.reversal_potential // 0')

    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║        NEUROPLASTICITY REVERSAL INDEX (NRI)               ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Color code based on NRI value
    local nri_color="${RED}"
    local interpretation="Challenging"
    if (( $(echo "$index >= 80" | bc -l) )); then
        nri_color="${GREEN}"
        interpretation="Excellent reversal potential"
    elif (( $(echo "$index >= 60" | bc -l) )); then
        nri_color="${GREEN}"
        interpretation="Good reversal potential"
    elif (( $(echo "$index >= 40" | bc -l) )); then
        nri_color="${YELLOW}"
        interpretation="Moderate potential"
    elif (( $(echo "$index >= 20" | bc -l) )); then
        nri_color="${YELLOW}"
        interpretation="Limited potential"
    fi

    echo -e "  NRI Score: ${nri_color}${index}/100${NC}"
    echo "  Interpretation: $interpretation"
    echo "  Trend: $trend"
    echo "  Reversal Potential: ${reversal}%"
    echo ""
    echo "  Components:"
    echo "    Central Sensitization: $(echo "$data" | jq -r '.components.central_sensitization // "N/A"')"
    echo "    Gray Matter Changes: $(echo "$data" | jq -r '.components.gray_matter_changes // "N/A"')"
    echo "    Functional Connectivity: $(echo "$data" | jq -r '.components.functional_connectivity // "N/A"')"
    echo "    Psychosocial: $(echo "$data" | jq -r '.components.psychosocial_factors // "N/A"')"
    echo ""

    # Brain plasticity visualization
    echo "  Brain Plasticity State:"
    local bar_length=$((index / 5))
    printf "  ["
    for ((i=0; i<20; i++)); do
        if [ $i -lt $bar_length ]; then
            printf "${nri_color}█${NC}"
        else
            printf "░"
        fi
    done
    printf "] ${index}%%\n"
    echo ""
}

# ============================================================================
# Command Implementations
# ============================================================================

cmd_config() {
    print_banner

    mkdir -p "$(dirname "$CONFIG_FILE")"

    echo "WIA-CHRONIC-PAIN Configuration"
    echo ""
    read -p "API Key: " api_key
    read -p "API URL [${WIA_API_URL}]: " api_url

    if [ -z "$api_url" ]; then
        api_url="$WIA_API_URL"
    fi

    cat > "$CONFIG_FILE" << EOF
WIA_API_KEY="$api_key"
WIA_API_URL="$api_url"
EOF

    chmod 600 "$CONFIG_FILE"
    log_success "Configuration saved to $CONFIG_FILE"
}

cmd_assess() {
    local patient_id="$1"
    local pain_type="${PAIN_TYPE:-nociplastic}"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Creating pain assessment for patient: $patient_id"

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id",
  "pain_type": "$pain_type"
}
EOF
)

    local response=$(api_request "POST" "/assess" "$payload")
    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_profile() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Fetching profile for patient: $patient_id"

    local response=$(api_request "GET" "/profiles/$patient_id?include=sensitization,psychosocial,treatments")
    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_sensitization() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Fetching central sensitization status for patient: $patient_id"

    local response=$(api_request "GET" "/sensitization/$patient_id")

    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║              CENTRAL SENSITIZATION STATUS                  ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    local csi=$(echo "$response" | jq -r '.csi_score.score // 0')
    local level=$(echo "$response" | jq -r '.sensitization_level // "unknown"')

    # Color based on severity
    local csi_color="${GREEN}"
    if [ "$level" = "severe" ]; then
        csi_color="${RED}"
    elif [ "$level" = "moderate" ]; then
        csi_color="${YELLOW}"
    fi

    echo -e "  CSI Score: ${csi_color}${csi}/100${NC}"
    echo "  Sensitization Level: $level"
    echo "  Temporal Summation: $(echo "$response" | jq -r '.temporal_summation.present // "N/A"')"
    echo "  CPM Status: $(echo "$response" | jq -r '.cpm.status // "N/A"')"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_nri() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Calculating NRI for patient: $patient_id"

    local response=$(api_request "GET" "/neuroplasticity/index/$patient_id")

    if [ "$OUTPUT_FORMAT" = "summary" ] || [ -z "$OUTPUT_FORMAT" ]; then
        format_nri_summary "$response"
    else
        format_output "$response" "$OUTPUT_FORMAT"
    fi
}

cmd_neuromod() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Generating neuromodulation plan for patient: $patient_id"

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id",
  "target_symptoms": ["pain_intensity", "central_sensitization"],
  "preferences": {
    "clinic_based": true
  }
}
EOF
)

    local response=$(api_request "POST" "/neuromodulation/plan" "$payload")

    echo ""
    echo -e "${MAGENTA}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${MAGENTA}║               NEUROMODULATION TREATMENT PLAN               ║${NC}"
    echo -e "${MAGENTA}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_recommend() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Getting treatment recommendations for patient: $patient_id"

    local response=$(api_request "POST" "/treatment/recommend" "{\"patient_id\": \"$patient_id\"}")

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║               TREATMENT RECOMMENDATIONS                    ║${NC}"
    echo -e "${GREEN}║            (Neuroplasticity Reversal Focus)               ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_opioid_risk() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Assessing opioid risk for patient: $patient_id"

    local response=$(api_request "GET" "/opioid-risk/$patient_id")

    local risk_level=$(echo "$response" | jq -r '.risk_level // "unknown"')
    local mme=$(echo "$response" | jq -r '.current_mme // 0')

    echo ""
    case "$risk_level" in
        low)
            echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${GREEN}║                  OPIOID RISK ASSESSMENT                    ║${NC}"
            echo -e "${GREEN}║                    Risk Level: LOW                         ║${NC}"
            echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
        moderate)
            echo -e "${YELLOW}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${YELLOW}║                  OPIOID RISK ASSESSMENT                    ║${NC}"
            echo -e "${YELLOW}║                  Risk Level: MODERATE                      ║${NC}"
            echo -e "${YELLOW}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
        high)
            echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${RED}║                  OPIOID RISK ASSESSMENT                    ║${NC}"
            echo -e "${RED}║                    Risk Level: HIGH                        ║${NC}"
            echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
    esac
    echo ""
    echo "  Current MME: ${mme} mg/day"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_taper() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Creating opioid taper plan for patient: $patient_id"

    local response=$(api_request "POST" "/opioid-taper/plan" "{\"patient_id\": \"$patient_id\"}")

    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                   OPIOID TAPER PLAN                        ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_chronify() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Predicting chronification risk for patient: $patient_id"

    local response=$(api_request "POST" "/predict/chronification" "{\"patient_id\": \"$patient_id\"}")

    local probability=$(echo "$response" | jq -r '.chronification_probability // 0')
    local risk_level=$(echo "$response" | jq -r '.risk_level // "unknown"')

    echo ""
    echo -e "${YELLOW}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║              CHRONIFICATION RISK PREDICTION                ║${NC}"
    echo -e "${YELLOW}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "  Chronification Probability: ${probability}%"
    echo "  Risk Level: ${risk_level}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_validate() {
    local file="$1"

    if [ -z "$file" ]; then
        log_error "File path required"
        exit 1
    fi

    if [ ! -f "$file" ]; then
        log_error "File not found: $file"
        exit 1
    fi

    log_info "Validating WIA-CHRONIC-PAIN file: $file"

    # Basic JSON validation
    if ! jq . "$file" > /dev/null 2>&1; then
        log_error "Invalid JSON format"
        exit 1
    fi

    # Check required fields
    local required_fields=("patient_id" "pain_type")
    for field in "${required_fields[@]}"; do
        if ! jq -e ".$field" "$file" > /dev/null 2>&1; then
            log_error "Missing required field: $field"
            exit 1
        fi
    done

    # Validate pain type
    local pain_type=$(jq -r '.pain_type' "$file")
    local valid_types="nociceptive neuropathic nociplastic mixed"
    if [[ ! " $valid_types " =~ " $pain_type " ]]; then
        log_error "Invalid pain_type: $pain_type"
        log_info "Valid types: $valid_types"
        exit 1
    fi

    # Validate duration
    local duration=$(jq -r '.pain_assessment.duration_months // 0' "$file")
    if (( $(echo "$duration < 3" | bc -l) )); then
        log_warning "Duration <3 months may not qualify as chronic pain"
    fi

    log_success "File is valid WIA-CHRONIC-PAIN format"
}

# ============================================================================
# Main
# ============================================================================

# Parse global options
OUTPUT_FORMAT="json"
PAIN_TYPE=""
OUTPUT_FILE=""
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --pain-type)
            PAIN_TYPE="$2"
            shift 2
            ;;
        --format)
            OUTPUT_FORMAT="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        *)
            break
            ;;
    esac
done

# Get command
COMMAND="${1:-help}"
shift || true

# Execute command
case "$COMMAND" in
    assess)
        cmd_assess "$@"
        ;;
    profile)
        cmd_profile "$@"
        ;;
    sensitization)
        cmd_sensitization "$@"
        ;;
    nri)
        cmd_nri "$@"
        ;;
    neuromod)
        cmd_neuromod "$@"
        ;;
    recommend)
        cmd_recommend "$@"
        ;;
    opioid-risk)
        cmd_opioid_risk "$@"
        ;;
    taper)
        cmd_taper "$@"
        ;;
    chronify)
        cmd_chronify "$@"
        ;;
    validate)
        cmd_validate "$@"
        ;;
    config)
        cmd_config
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        log_error "Unknown command: $COMMAND"
        echo "Run 'wia-chronic-pain help' for usage"
        exit 1
        ;;
esac
