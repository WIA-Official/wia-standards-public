#!/bin/bash
#
# WIA-AUTOIMMUNE CLI
# Treg-Microbiome Axis Autoimmune Disease Standard
#
# Version: 1.0.0
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

# Configuration
WIA_API_URL="${WIA_API_URL:-https://api.wia.live/autoimmune/v1}"
WIA_API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/autoimmune.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ============================================================================
# Helper Functions
# ============================================================================

print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════════════════════╗"
    echo "║                           WIA-AUTOIMMUNE CLI                              ║"
    echo "║               Treg-Microbiome Axis Autoimmune Standard                    ║"
    echo "║                                                                           ║"
    echo "║                    弘益人間 - Benefit All Humanity                         ║"
    echo "╚═══════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_help() {
    print_banner
    echo "Usage: wia-autoimmune <command> [options]"
    echo ""
    echo "Commands:"
    echo "  assess <patient_id>        Create comprehensive autoimmune assessment"
    echo "  profile <patient_id>       Get patient autoimmune profile"
    echo "  treg <patient_id>          Get Treg functional status"
    echo "  microbiome <patient_id>    Get microbiome analysis"
    echo "  recommend <patient_id>     Get treatment recommendations"
    echo "  activity <patient_id>      Get disease activity scores"
    echo "  flare <patient_id>         Predict flare risk"
    echo "  remission <patient_id>     Assess remission probability"
    echo "  tmas <patient_id>          Calculate TMAS score"
    echo "  validate <file>            Validate WIA-AUTOIMMUNE JSON file"
    echo "  config                     Configure API credentials"
    echo "  help                       Show this help message"
    echo ""
    echo "Options:"
    echo "  --disease <type>      Disease type (RA|SLE|MS|T1D|IBD|PSO|HT|GD)"
    echo "  --format <fmt>        Output format (json|table|summary)"
    echo "  --output <file>       Write output to file"
    echo "  --verbose             Verbose output"
    echo ""
    echo "Environment Variables:"
    echo "  WIA_API_URL           API base URL (default: https://api.wia.live/autoimmune/v1)"
    echo "  WIA_API_KEY           API authentication key"
    echo ""
    echo "Examples:"
    echo "  wia-autoimmune assess patient-123 --disease RA"
    echo "  wia-autoimmune treg patient-123 --format json"
    echo "  wia-autoimmune flare patient-123 --output prediction.json"
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
        log_error "API key not configured. Run 'wia-autoimmune config' first."
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

format_summary() {
    local data="$1"

    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}                    ASSESSMENT SUMMARY                      ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo ""

    # Extract key metrics
    local treg_count=$(echo "$data" | jq -r '.immune_markers.treg.count.value // "N/A"')
    local foxp3=$(echo "$data" | jq -r '.immune_markers.treg.foxp3_expression // "N/A"')
    local suppression=$(echo "$data" | jq -r '.immune_markers.treg.suppressive_function // "N/A"')
    local dysbiosis=$(echo "$data" | jq -r '.microbiome.dysbiosis_score // "N/A"')
    local activity=$(echo "$data" | jq -r '.disease_activity.activity_level // "N/A"')

    echo -e "  ${YELLOW}Treg Status:${NC}"
    echo "    Count: $treg_count cells/μL"
    echo "    FOXP3 Expression: $foxp3%"
    echo "    Suppressive Function: $suppression%"
    echo ""
    echo -e "  ${YELLOW}Microbiome:${NC}"
    echo "    Dysbiosis Score: $dysbiosis/100"
    echo ""
    echo -e "  ${YELLOW}Disease Activity:${NC}"
    echo "    Level: $activity"
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
}

# ============================================================================
# Command Implementations
# ============================================================================

cmd_config() {
    print_banner

    mkdir -p "$(dirname "$CONFIG_FILE")"

    echo "WIA-AUTOIMMUNE Configuration"
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
    local disease_type="${DISEASE_TYPE:-RA}"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Creating assessment for patient: $patient_id"

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id",
  "disease_type": "$disease_type"
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

    local response=$(api_request "GET" "/profiles/$patient_id?include=treg,microbiome")
    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_treg() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Fetching Treg status for patient: $patient_id"

    local response=$(api_request "GET" "/treg-status/$patient_id")

    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                    TREG STATUS REPORT                      ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_microbiome() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Fetching microbiome analysis for patient: $patient_id"

    local response=$(api_request "GET" "/microbiome/$patient_id?scfa_focus=true")
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

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id"
}
EOF
)

    local response=$(api_request "POST" "/treatment/recommend" "$payload")

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                TREATMENT RECOMMENDATIONS                   ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_activity() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Fetching disease activity for patient: $patient_id"

    local endpoint="/disease-activity/$patient_id"
    if [ -n "$DISEASE_TYPE" ]; then
        endpoint="${endpoint}?disease_type=$DISEASE_TYPE"
    fi

    local response=$(api_request "GET" "$endpoint")
    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_flare() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Predicting flare risk for patient: $patient_id"

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id",
  "include_microbiome": true,
  "prediction_horizon_days": 30
}
EOF
)

    local response=$(api_request "POST" "/flare/predict" "$payload")

    # Extract risk level for color coding
    local risk_level=$(echo "$response" | jq -r '.risk_level // "unknown"')
    local probability=$(echo "$response" | jq -r '.flare_probability // 0')

    echo ""
    case "$risk_level" in
        low)
            echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${GREEN}║                    FLARE PREDICTION                        ║${NC}"
            echo -e "${GREEN}║                   Risk Level: LOW                          ║${NC}"
            echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
        moderate)
            echo -e "${YELLOW}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${YELLOW}║                    FLARE PREDICTION                        ║${NC}"
            echo -e "${YELLOW}║                 Risk Level: MODERATE                       ║${NC}"
            echo -e "${YELLOW}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
        high|very_high)
            echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${RED}║                    FLARE PREDICTION                        ║${NC}"
            echo -e "${RED}║                  Risk Level: HIGH                          ║${NC}"
            echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
            ;;
    esac
    echo ""
    echo "  Flare Probability (30 days): ${probability}%"
    echo ""

    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_remission() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Assessing remission probability for patient: $patient_id"

    local payload=$(cat <<EOF
{
  "patient_id": "$patient_id"
}
EOF
)

    local response=$(api_request "POST" "/remission/assess" "$payload")
    format_output "$response" "$OUTPUT_FORMAT"
}

cmd_tmas() {
    local patient_id="$1"

    if [ -z "$patient_id" ]; then
        log_error "Patient ID required"
        exit 1
    fi

    check_api_key

    log_info "Calculating TMAS score for patient: $patient_id"

    local response=$(api_request "GET" "/profiles/$patient_id/tmas")

    local score=$(echo "$response" | jq -r '.total // 0')
    local interpretation=$(echo "$response" | jq -r '.interpretation // "unknown"')

    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║           TREG-MICROBIOME AXIS SCORE (TMAS)               ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "  Total Score: $score / 100"
    echo "  Interpretation: $interpretation"
    echo ""
    echo "  Components:"
    echo "    Treg Component: $(echo "$response" | jq -r '.treg_component // 0')"
    echo "    Microbiome Component: $(echo "$response" | jq -r '.microbiome_component // 0')"
    echo "    Activity Component: $(echo "$response" | jq -r '.activity_component // 0')"
    echo ""
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

    log_info "Validating WIA-AUTOIMMUNE file: $file"

    # Basic JSON validation
    if ! jq . "$file" > /dev/null 2>&1; then
        log_error "Invalid JSON format"
        exit 1
    fi

    # Check required fields
    local required_fields=("patient_id" "disease_type")
    for field in "${required_fields[@]}"; do
        if ! jq -e ".$field" "$file" > /dev/null 2>&1; then
            log_error "Missing required field: $field"
            exit 1
        fi
    done

    # Validate disease type
    local disease_type=$(jq -r '.disease_type' "$file")
    local valid_types="RA SLE MS T1D IBD PSO HT GD"
    if [[ ! " $valid_types " =~ " $disease_type " ]]; then
        log_error "Invalid disease_type: $disease_type"
        log_info "Valid types: $valid_types"
        exit 1
    fi

    log_success "File is valid WIA-AUTOIMMUNE format"
}

# ============================================================================
# Main
# ============================================================================

# Parse global options
OUTPUT_FORMAT="json"
DISEASE_TYPE=""
OUTPUT_FILE=""
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --disease)
            DISEASE_TYPE="$2"
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
    treg)
        cmd_treg "$@"
        ;;
    microbiome)
        cmd_microbiome "$@"
        ;;
    recommend)
        cmd_recommend "$@"
        ;;
    activity)
        cmd_activity "$@"
        ;;
    flare)
        cmd_flare "$@"
        ;;
    remission)
        cmd_remission "$@"
        ;;
    tmas)
        cmd_tmas "$@"
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
        echo "Run 'wia-autoimmune help' for usage"
        exit 1
        ;;
esac
