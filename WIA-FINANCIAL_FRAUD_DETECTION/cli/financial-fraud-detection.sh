#!/bin/bash
################################################################################
# WIA-FINANCIAL_FRAUD_DETECTION CLI Tool
#
# Command-line interface for WIA Financial Fraud Detection API
#
# @version 1.0.0
# @standard WIA-FINANCIAL_FRAUD_DETECTION
# @organization WIA (World Certification Industry Association)
# @philosophy 弘益人間 (Hongik Ingan) - Benefit All Humanity
#
# Usage:
#   ./financial-fraud-detection.sh <command> [options]
#
# Commands:
#   analyze     - Analyze a transaction for fraud
#   feedback    - Submit feedback on a fraud decision
#   report      - Get fraud report for a transaction
#   stats       - Get fraud statistics
#   health      - Check API health status
#   config      - Configure API credentials
#   help        - Show help information
#
# Environment Variables:
#   WIA_FRAUD_API_KEY     - API key for authentication
#   WIA_FRAUD_BASE_URL    - Base URL for API (default: https://api.wia-fraud.io/v1)
#
# Examples:
#   # Configure API key
#   ./financial-fraud-detection.sh config --api-key EXAMPLE_API_KEY_REPLACE_ME
#
#   # Analyze transaction from file
#   ./financial-fraud-detection.sh analyze --file transaction.json
#
#   # Get statistics for last 30 days
#   ./financial-fraud-detection.sh stats --days 30
#
################################################################################

set -euo pipefail

# Script configuration
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${HOME}/.wia-fraud-detection.conf"
BASE_URL="${WIA_FRAUD_BASE_URL:-https://api.wia-fraud.io/v1}"
API_KEY="${WIA_FRAUD_API_KEY:-}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

# Print colored message
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

# Load configuration
load_config() {
    if [[ -f "$CONFIG_FILE" ]]; then
        # shellcheck disable=SC1090
        source "$CONFIG_FILE"
        if [[ -n "${WIA_FRAUD_API_KEY_STORED:-}" ]]; then
            API_KEY="$WIA_FRAUD_API_KEY_STORED"
        fi
    fi
}

# Save configuration
save_config() {
    cat > "$CONFIG_FILE" <<EOF
# WIA-FINANCIAL_FRAUD_DETECTION Configuration
# Generated: $(date)
WIA_FRAUD_API_KEY_STORED="$API_KEY"
WIA_FRAUD_BASE_URL="$BASE_URL"
EOF
    chmod 600 "$CONFIG_FILE"
    print_success "Configuration saved to $CONFIG_FILE"
}

# Check if API key is set
check_api_key() {
    if [[ -z "$API_KEY" ]]; then
        print_error "API key not set. Please configure with: $0 config --api-key YOUR_KEY"
        exit 1
    fi
}

# Make API request
api_request() {
    local method="$1"
    local endpoint="$2"
    local data="${3:-}"

    local url="${BASE_URL}${endpoint}"
    local curl_args=(
        -X "$method"
        -H "Content-Type: application/json"
        -H "X-API-Key: $API_KEY"
        -H "User-Agent: wia-fraud-detection-cli/$VERSION"
        -s
        -w "\n%{http_code}"
    )

    if [[ -n "$data" ]]; then
        curl_args+=(-d "$data")
    fi

    local response
    response=$(curl "${curl_args[@]}" "$url")

    local http_code
    http_code=$(echo "$response" | tail -n1)
    local body
    body=$(echo "$response" | sed '$d')

    if [[ "$http_code" -ge 200 && "$http_code" -lt 300 ]]; then
        echo "$body"
    else
        print_error "API request failed (HTTP $http_code)"
        echo "$body" | jq -r '.errors[]?.message // "Unknown error"' 2>/dev/null || echo "$body"
        exit 1
    fi
}

# Pretty print JSON
pretty_json() {
    if command -v jq &> /dev/null; then
        jq -C '.'
    else
        cat
    fi
}

# Format timestamp
format_timestamp() {
    local timestamp="$1"
    if command -v date &> /dev/null; then
        date -d "$timestamp" "+%Y-%m-%d %H:%M:%S" 2>/dev/null || echo "$timestamp"
    else
        echo "$timestamp"
    fi
}

################################################################################
# Command Implementations
################################################################################

# Command: config
cmd_config() {
    local api_key_arg=""
    local base_url_arg=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --api-key)
                api_key_arg="$2"
                shift 2
                ;;
            --base-url)
                base_url_arg="$2"
                shift 2
                ;;
            --show)
                load_config
                echo "Configuration:"
                echo "  API Key: ${API_KEY:0:12}...${API_KEY: -4}"
                echo "  Base URL: $BASE_URL"
                echo "  Config File: $CONFIG_FILE"
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -n "$api_key_arg" ]]; then
        API_KEY="$api_key_arg"
    fi

    if [[ -n "$base_url_arg" ]]; then
        BASE_URL="$base_url_arg"
    fi

    if [[ -z "$API_KEY" ]]; then
        print_error "API key is required. Use: $0 config --api-key YOUR_KEY"
        exit 1
    fi

    save_config
}

# Command: analyze
cmd_analyze() {
    check_api_key

    local file=""
    local transaction_data=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --file)
                file="$2"
                shift 2
                ;;
            --data)
                transaction_data="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "$file" && -z "$transaction_data" ]]; then
        print_error "Transaction data required. Use --file or --data"
        exit 1
    fi

    if [[ -n "$file" ]]; then
        if [[ ! -f "$file" ]]; then
            print_error "File not found: $file"
            exit 1
        fi
        transaction_data=$(cat "$file")
    fi

    print_info "Analyzing transaction..."

    local response
    response=$(api_request POST "/fraud/analyze" "{\"transaction\": $transaction_data}")

    # Extract key fields
    local risk_score
    risk_score=$(echo "$response" | jq -r '.data.fraud_assessment.risk_score')
    local decision
    decision=$(echo "$response" | jq -r '.data.fraud_assessment.decision')
    local risk_level
    risk_level=$(echo "$response" | jq -r '.data.fraud_assessment.risk_level')
    local transaction_id
    transaction_id=$(echo "$response" | jq -r '.data.fraud_assessment.transaction_id')

    echo
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  FRAUD ASSESSMENT RESULT"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo
    echo "  Transaction ID: $transaction_id"
    echo "  Risk Score:     $risk_score"
    echo "  Risk Level:     ${risk_level^^}"
    echo "  Decision:       ${decision^^}"
    echo

    # Display decision icon
    case $decision in
        approve)
            print_success "Transaction APPROVED"
            ;;
        challenge)
            print_warning "Transaction CHALLENGED - Additional authentication required"
            ;;
        review)
            print_warning "Transaction FLAGGED FOR REVIEW"
            ;;
        block)
            print_error "Transaction BLOCKED"
            ;;
    esac

    echo
    echo "  Top Reasons:"
    echo "$response" | jq -r '.data.fraud_assessment.decision_reasons[]' | sed 's/^/  - /'

    echo
    echo "  Model Scores:"
    echo "$response" | jq -r '.data.fraud_assessment.model_scores | to_entries[] | "  - \(.key): \(.value)"'

    echo
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo

    # Save full response to file
    local output_file="fraud_result_${transaction_id}.json"
    echo "$response" > "$output_file"
    print_success "Full result saved to: $output_file"
}

# Command: feedback
cmd_feedback() {
    check_api_key

    local transaction_id=""
    local feedback_type=""
    local fraud_type=""
    local notes=""
    local submitted_by="${USER}"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --transaction-id)
                transaction_id="$2"
                shift 2
                ;;
            --type)
                feedback_type="$2"
                shift 2
                ;;
            --fraud-type)
                fraud_type="$2"
                shift 2
                ;;
            --notes)
                notes="$2"
                shift 2
                ;;
            --submitted-by)
                submitted_by="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "$transaction_id" || -z "$feedback_type" ]]; then
        print_error "Transaction ID and feedback type required"
        echo "Usage: $0 feedback --transaction-id txn_123 --type confirmed_fraud [--fraud-type stolen_card] [--notes 'text']"
        exit 1
    fi

    local data
    data=$(jq -n \
        --arg tid "$transaction_id" \
        --arg ft "$feedback_type" \
        --arg frt "$fraud_type" \
        --arg n "$notes" \
        --arg sb "$submitted_by" \
        '{transaction_id: $tid, feedback_type: $ft, fraud_type: $frt, notes: $n, submitted_by: $sb} | if .fraud_type == "" then del(.fraud_type) else . end | if .notes == "" then del(.notes) else . end')

    print_info "Submitting feedback..."

    local response
    response=$(api_request POST "/fraud/feedback" "$data")

    print_success "Feedback submitted successfully"
    echo "$response" | pretty_json
}

# Command: report
cmd_report() {
    check_api_key

    local transaction_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --transaction-id)
                transaction_id="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "$transaction_id" ]]; then
        print_error "Transaction ID required"
        echo "Usage: $0 report --transaction-id txn_123"
        exit 1
    fi

    print_info "Fetching fraud report for transaction: $transaction_id"

    local response
    response=$(api_request GET "/fraud/reports/$transaction_id")

    echo "$response" | pretty_json
}

# Command: stats
cmd_stats() {
    check_api_key

    local days=30
    local start_date=""
    local end_date=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --days)
                days="$2"
                shift 2
                ;;
            --start-date)
                start_date="$2"
                shift 2
                ;;
            --end-date)
                end_date="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "$start_date" ]]; then
        start_date=$(date -u -d "$days days ago" +"%Y-%m-%d" 2>/dev/null || date -u -v-${days}d +"%Y-%m-%d")
    fi

    if [[ -z "$end_date" ]]; then
        end_date=$(date -u +"%Y-%m-%d")
    fi

    print_info "Fetching fraud statistics from $start_date to $end_date"

    local response
    response=$(api_request GET "/fraud/statistics?start_date=$start_date&end_date=$end_date")

    # Extract statistics
    local total_txns
    total_txns=$(echo "$response" | jq -r '.data.statistics.total_transactions')
    local fraud_detected
    fraud_detected=$(echo "$response" | jq -r '.data.statistics.fraud_detected')
    local fraud_rate
    fraud_rate=$(echo "$response" | jq -r '.data.statistics.fraud_rate')
    local amount_saved
    amount_saved=$(echo "$response" | jq -r '.data.statistics.amount_saved_usd')

    echo
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  FRAUD STATISTICS"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo
    echo "  Period:              $start_date to $end_date"
    echo "  Total Transactions:  $(printf "%'d" "$total_txns")"
    echo "  Fraud Detected:      $(printf "%'d" "$fraud_detected")"
    echo "  Fraud Rate:          $(awk "BEGIN {printf \"%.2f%%\", $fraud_rate * 100}")"
    echo "  Amount Saved:        \$$(printf "%'.2f" "$amount_saved")"
    echo
    echo "  Decisions:"
    echo "$response" | jq -r '.data.statistics.by_decision | to_entries[] | "  - \(.key): \(.value)"'
    echo
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo
}

# Command: health
cmd_health() {
    load_config

    print_info "Checking API health..."

    local response
    response=$(curl -s -w "\n%{http_code}" "${BASE_URL}/health")

    local http_code
    http_code=$(echo "$response" | tail -n1)
    local body
    body=$(echo "$response" | sed '$d')

    if [[ "$http_code" == "200" ]]; then
        print_success "API is healthy"
        echo "$body" | pretty_json
    else
        print_error "API is unhealthy (HTTP $http_code)"
        echo "$body"
        exit 1
    fi
}

# Command: help
cmd_help() {
    cat <<EOF
WIA-FINANCIAL_FRAUD_DETECTION CLI Tool v$VERSION

Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity

USAGE:
    $(basename "$0") <command> [options]

COMMANDS:
    config      Configure API credentials
    analyze     Analyze a transaction for fraud
    feedback    Submit feedback on a fraud decision
    report      Get fraud report for a transaction
    stats       Get fraud statistics
    health      Check API health status
    help        Show this help message
    version     Show version information

OPTIONS:
    config:
        --api-key KEY       Set API key
        --base-url URL      Set base URL (default: https://api.wia-fraud.io/v1)
        --show              Show current configuration

    analyze:
        --file FILE         Path to transaction JSON file
        --data JSON         Transaction data as JSON string

    feedback:
        --transaction-id ID         Transaction identifier
        --type TYPE                 Feedback type (confirmed_fraud, false_positive, etc.)
        --fraud-type TYPE           Fraud type (stolen_card, account_takeover, etc.)
        --notes TEXT                Additional notes
        --submitted-by USER         User who submitted feedback

    report:
        --transaction-id ID         Transaction identifier

    stats:
        --days N                    Number of days to include (default: 30)
        --start-date YYYY-MM-DD     Start date
        --end-date YYYY-MM-DD       End date

EXAMPLES:
    # Configure API key
    $(basename "$0") config --api-key EXAMPLE_API_KEY_REPLACE_ME

    # Analyze transaction from file
    $(basename "$0") analyze --file transaction.json

    # Submit fraud feedback
    $(basename "$0") feedback --transaction-id txn_123 --type confirmed_fraud --fraud-type stolen_card

    # Get fraud statistics for last 7 days
    $(basename "$0") stats --days 7

    # Check API health
    $(basename "$0") health

ENVIRONMENT VARIABLES:
    WIA_FRAUD_API_KEY       API key for authentication
    WIA_FRAUD_BASE_URL      Base URL for API

MORE INFO:
    Documentation: https://docs.wia-fraud.io
    Support:       fraud-detection@wia-official.org

© 2026 WIA (World Certification Industry Association)
EOF
}

# Command: version
cmd_version() {
    echo "WIA-FINANCIAL_FRAUD_DETECTION CLI v$VERSION"
    echo "© 2026 WIA (World Certification Industry Association)"
}

################################################################################
# Main
################################################################################

main() {
    # Load configuration
    load_config

    # Check for command
    if [[ $# -eq 0 ]]; then
        cmd_help
        exit 0
    fi

    local command="$1"
    shift

    # Dispatch command
    case $command in
        config)
            cmd_config "$@"
            ;;
        analyze)
            cmd_analyze "$@"
            ;;
        feedback)
            cmd_feedback "$@"
            ;;
        report)
            cmd_report "$@"
            ;;
        stats)
            cmd_stats "$@"
            ;;
        health)
            cmd_health "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        version|--version|-v)
            cmd_version
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run '$(basename "$0") help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
