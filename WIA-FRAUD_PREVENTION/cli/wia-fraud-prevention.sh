#!/usr/bin/env bash
#
# WIA-FRAUD_PREVENTION CLI v1.0.0
# Command-line interface for WIA Fraud Prevention Standard
#
# Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
#
# Usage: wia-fraud-prevention.sh [command] [options]
#
# Copyright 2026 WIA (World Industry Association)
# Licensed under MIT License

set -euo pipefail

# Configuration
VERSION="1.0.0"
API_BASE_URL="${WIA_FP_API_URL:-https://api.fraud-prevention.wia.org/v1}"
API_KEY="${WIA_FP_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia-fraud-prevention/config.json"
LOG_FILE="${HOME}/.wia-fraud-prevention/fraud-prevention.log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging function
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    echo "[${timestamp}] [${level}] ${message}" >> "${LOG_FILE}"

    case "${level}" in
        ERROR)
            echo -e "${RED}[ERROR]${NC} ${message}" >&2
            ;;
        WARN)
            echo -e "${YELLOW}[WARN]${NC} ${message}" >&2
            ;;
        INFO)
            echo -e "${GREEN}[INFO]${NC} ${message}"
            ;;
        DEBUG)
            if [[ "${DEBUG:-false}" == "true" ]]; then
                echo -e "${CYAN}[DEBUG]${NC} ${message}"
            fi
            ;;
    esac
}

# Create config directory if not exists
init_config() {
    mkdir -p "$(dirname "${CONFIG_FILE}")"
    mkdir -p "$(dirname "${LOG_FILE}")"

    if [[ ! -f "${CONFIG_FILE}" ]]; then
        cat > "${CONFIG_FILE}" <<EOF
{
  "api_base_url": "${API_BASE_URL}",
  "api_key": "${API_KEY}",
  "default_threshold": 70,
  "enable_logging": true,
  "log_level": "INFO"
}
EOF
        log INFO "Configuration file created at ${CONFIG_FILE}"
    fi
}

# Load configuration
load_config() {
    if [[ -f "${CONFIG_FILE}" ]]; then
        API_BASE_URL=$(jq -r '.api_base_url // "https://api.fraud-prevention.wia.org/v1"' "${CONFIG_FILE}")
        API_KEY=$(jq -r '.api_key // ""' "${CONFIG_FILE}")
    fi
}

# API request helper
api_request() {
    local method="$1"
    local endpoint="$2"
    local data="${3:-}"

    if [[ -z "${API_KEY}" ]]; then
        log ERROR "API key not configured. Run 'wia-fraud-prevention.sh config --api-key YOUR_KEY'"
        exit 1
    fi

    local curl_opts=(
        -X "${method}"
        -H "Content-Type: application/json"
        -H "X-API-Key: ${API_KEY}"
        -H "User-Agent: WIA-FraudPrevention-CLI/${VERSION}"
        -s
        -w "\n%{http_code}"
    )

    if [[ -n "${data}" ]]; then
        curl_opts+=(-d "${data}")
    fi

    local response=$(curl "${curl_opts[@]}" "${API_BASE_URL}${endpoint}")
    local http_code=$(echo "${response}" | tail -n1)
    local body=$(echo "${response}" | sed '$d')

    log DEBUG "API Request: ${method} ${endpoint}"
    log DEBUG "HTTP Status: ${http_code}"
    log DEBUG "Response: ${body}"

    if [[ "${http_code}" -ge 200 && "${http_code}" -lt 300 ]]; then
        echo "${body}"
    else
        log ERROR "API request failed with status ${http_code}"
        echo "${body}" | jq -r '.error.message // "Unknown error"' >&2
        exit 1
    fi
}

# Command: fraud-detect
cmd_fraud_detect() {
    local transaction_id=""
    local amount=""
    local currency="USD"
    local user_id=""
    local merchant_id=""
    local ip_address=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --transaction-id)
                transaction_id="$2"
                shift 2
                ;;
            --amount)
                amount="$2"
                shift 2
                ;;
            --currency)
                currency="$2"
                shift 2
                ;;
            --user-id)
                user_id="$2"
                shift 2
                ;;
            --merchant-id)
                merchant_id="$2"
                shift 2
                ;;
            --ip)
                ip_address="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${transaction_id}" || -z "${amount}" || -z "${user_id}" ]]; then
        log ERROR "Required parameters: --transaction-id, --amount, --user-id"
        exit 1
    fi

    log INFO "Analyzing transaction ${transaction_id}..."

    local request_data=$(jq -n \
        --arg tid "${transaction_id}" \
        --arg amt "${amount}" \
        --arg cur "${currency}" \
        --arg uid "${user_id}" \
        --arg mid "${merchant_id}" \
        --arg ip "${ip_address}" \
        '{
            transaction: {
                transactionId: $tid,
                amount: ($amt | tonumber),
                currency: $cur,
                merchantId: $mid,
                timestamp: (now | strftime("%Y-%m-%dT%H:%M:%SZ"))
            },
            user: {
                userId: $uid
            },
            options: {
                includeExplanation: true,
                includeRecommendations: true
            }
        }')

    local response=$(api_request POST "/detect/transaction" "${request_data}")

    # Parse and display results
    local is_fraudulent=$(echo "${response}" | jq -r '.fraudAnalysis.verdict.isFraudulent')
    local risk_score=$(echo "${response}" | jq -r '.fraudAnalysis.verdict.riskScore')
    local confidence=$(echo "${response}" | jq -r '.fraudAnalysis.verdict.confidence')
    local action=$(echo "${response}" | jq -r '.fraudAnalysis.verdict.recommendedAction')

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}                 FRAUD DETECTION RESULT${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Transaction ID: ${transaction_id}"
    echo "Amount: ${amount} ${currency}"
    echo ""

    if [[ "${is_fraudulent}" == "true" ]]; then
        echo -e "Status: ${RED}FRAUDULENT${NC}"
    else
        echo -e "Status: ${GREEN}LEGITIMATE${NC}"
    fi

    echo "Risk Score: ${risk_score}/100"
    echo "Confidence: $(echo "${confidence} * 100" | bc)%"
    echo "Recommended Action: ${action}"
    echo ""

    # Display risk factors
    echo "Risk Factors:"
    echo "${response}" | jq -r '.fraudAnalysis.riskFactors[] | "  • \(.description) (severity: \(.severity))"'
    echo ""

    # Display explanation
    echo "Analysis Summary:"
    echo "${response}" | jq -r '.fraudAnalysis.explanation.summary' | fold -s -w 70 | sed 's/^/  /'
    echo ""

    log INFO "Fraud detection completed for transaction ${transaction_id}"
}

# Command: analyze-transaction
cmd_analyze_transaction() {
    local transaction_file=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --file)
                transaction_file="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${transaction_file}" ]]; then
        log ERROR "Required parameter: --file"
        exit 1
    fi

    if [[ ! -f "${transaction_file}" ]]; then
        log ERROR "File not found: ${transaction_file}"
        exit 1
    fi

    log INFO "Analyzing transactions from ${transaction_file}..."

    local request_data=$(cat "${transaction_file}")
    local response=$(api_request POST "/detect/batch" "${request_data}")

    # Display results
    local total=$(echo "${response}" | jq -r '.batchAnalysis.totalTransactions')
    local fraudulent=$(echo "${response}" | jq -r '.batchAnalysis.fraudulentCount')
    local processing_time=$(echo "${response}" | jq -r '.batchAnalysis.processingTime')

    echo ""
    echo "Batch Analysis Results:"
    echo "  Total Transactions: ${total}"
    echo "  Fraudulent: ${fraudulent}"
    echo "  Processing Time: ${processing_time}ms"
    echo ""

    # Display detailed results
    echo "Detailed Results:"
    echo "${response}" | jq -r '.batchAnalysis.results[] | "\(.transactionId): Risk Score \(.riskScore) - \(if .isFraudulent then "FRAUD" else "OK" end)"'

    log INFO "Batch analysis completed: ${total} transactions, ${fraudulent} fraudulent"
}

# Command: train-model
cmd_train_model() {
    local model_name=""
    local model_type="gradient_boosting"
    local dataset=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --name)
                model_name="$2"
                shift 2
                ;;
            --type)
                model_type="$2"
                shift 2
                ;;
            --dataset)
                dataset="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${model_name}" || -z "${dataset}" ]]; then
        log ERROR "Required parameters: --name, --dataset"
        exit 1
    fi

    log INFO "Starting model training: ${model_name}..."

    local request_data=$(jq -n \
        --arg name "${model_name}" \
        --arg type "${model_type}" \
        --arg dataset "${dataset}" \
        '{
            modelName: $name,
            modelType: $type,
            trainingDataset: $dataset,
            hyperparameters: {
                learningRate: 0.001,
                epochs: 100
            }
        }')

    local response=$(api_request POST "/models/train" "${request_data}")

    local job_id=$(echo "${response}" | jq -r '.trainingJob.jobId')
    local status=$(echo "${response}" | jq -r '.trainingJob.status')
    local estimated_duration=$(echo "${response}" | jq -r '.trainingJob.estimatedDuration')

    echo ""
    echo "Training Job Created:"
    echo "  Job ID: ${job_id}"
    echo "  Status: ${status}"
    echo "  Estimated Duration: $(( estimated_duration / 60 )) minutes"
    echo ""

    log INFO "Model training job created: ${job_id}"
}

# Command: alert-setup
cmd_alert_setup() {
    local rule_name=""
    local threshold=70
    local action="alert"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --name)
                rule_name="$2"
                shift 2
                ;;
            --threshold)
                threshold="$2"
                shift 2
                ;;
            --action)
                action="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${rule_name}" ]]; then
        log ERROR "Required parameter: --name"
        exit 1
    fi

    log INFO "Creating alert rule: ${rule_name}..."

    local request_data=$(jq -n \
        --arg name "${rule_name}" \
        --arg threshold "${threshold}" \
        --arg action "${action}" \
        '{
            ruleName: $name,
            enabled: true,
            priority: "high",
            conditions: {
                operator: "AND",
                rules: [{
                    field: "riskScore",
                    operator: ">",
                    value: ($threshold | tonumber)
                }]
            },
            actions: {
                alert: true,
                block: ($action == "block"),
                notify: ["fraud_team"]
            }
        }')

    local response=$(api_request POST "/alerts/rules" "${request_data}")

    local rule_id=$(echo "${response}" | jq -r '.rule.ruleId')

    echo ""
    echo "Alert Rule Created:"
    echo "  Rule ID: ${rule_id}"
    echo "  Name: ${rule_name}"
    echo "  Threshold: ${threshold}"
    echo "  Action: ${action}"
    echo ""

    log INFO "Alert rule created: ${rule_id}"
}

# Command: monitor-start
cmd_monitor_start() {
    local user_id=""
    local duration=3600

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --user-id)
                user_id="$2"
                shift 2
                ;;
            --duration)
                duration="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${user_id}" ]]; then
        log ERROR "Required parameter: --user-id"
        exit 1
    fi

    log INFO "Starting monitoring session for user ${user_id}..."

    local request_data=$(jq -n \
        --arg uid "${user_id}" \
        --arg dur "${duration}" \
        '{
            userId: $uid,
            monitoringLevel: "enhanced",
            duration: ($dur | tonumber),
            alertThreshold: {
                riskScore: 60,
                confidence: 0.85
            }
        }')

    local response=$(api_request POST "/monitor/session" "${request_data}")

    local session_id=$(echo "${response}" | jq -r '.session.sessionId')
    local expires_at=$(echo "${response}" | jq -r '.session.expiresAt')

    echo ""
    echo "Monitoring Session Started:"
    echo "  Session ID: ${session_id}"
    echo "  User ID: ${user_id}"
    echo "  Duration: $(( duration / 60 )) minutes"
    echo "  Expires: ${expires_at}"
    echo ""

    log INFO "Monitoring session started: ${session_id}"
}

# Command: report
cmd_report() {
    local report_type="fraud_summary"
    local start_date=""
    local end_date=""
    local format="json"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --type)
                report_type="$2"
                shift 2
                ;;
            --start)
                start_date="$2"
                shift 2
                ;;
            --end)
                end_date="$2"
                shift 2
                ;;
            --format)
                format="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ -z "${start_date}" ]]; then
        start_date=$(date -u -d "7 days ago" +"%Y-%m-%dT%H:%M:%SZ")
    fi

    if [[ -z "${end_date}" ]]; then
        end_date=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    fi

    log INFO "Generating ${report_type} report..."

    local request_data=$(jq -n \
        --arg type "${report_type}" \
        --arg start "${start_date}" \
        --arg end "${end_date}" \
        --arg fmt "${format}" \
        '{
            reportType: $type,
            period: {
                start: $start,
                end: $end
            },
            includeCharts: true,
            includeRecommendations: true,
            format: $fmt
        }')

    local response=$(api_request POST "/reports/fraud" "${request_data}")

    local report_id=$(echo "${response}" | jq -r '.report.reportId')
    local status=$(echo "${response}" | jq -r '.report.status')
    local download_url=$(echo "${response}" | jq -r '.report.downloadUrl')

    echo ""
    echo "Report Generation Initiated:"
    echo "  Report ID: ${report_id}"
    echo "  Status: ${status}"
    echo "  Download URL: ${download_url}"
    echo ""

    log INFO "Report generated: ${report_id}"
}

# Command: config
cmd_config() {
    local show_config=false
    local set_api_key=""
    local set_threshold=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --show)
                show_config=true
                shift
                ;;
            --api-key)
                set_api_key="$2"
                shift 2
                ;;
            --threshold)
                set_threshold="$2"
                shift 2
                ;;
            *)
                log ERROR "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ "${show_config}" == "true" ]]; then
        echo ""
        echo "Current Configuration:"
        cat "${CONFIG_FILE}" | jq '.'
        echo ""
        echo "Config File: ${CONFIG_FILE}"
        echo "Log File: ${LOG_FILE}"
        return
    fi

    if [[ -n "${set_api_key}" ]]; then
        jq --arg key "${set_api_key}" '.api_key = $key' "${CONFIG_FILE}" > "${CONFIG_FILE}.tmp"
        mv "${CONFIG_FILE}.tmp" "${CONFIG_FILE}"
        log INFO "API key updated"
    fi

    if [[ -n "${set_threshold}" ]]; then
        jq --arg threshold "${set_threshold}" '.default_threshold = ($threshold | tonumber)' "${CONFIG_FILE}" > "${CONFIG_FILE}.tmp"
        mv "${CONFIG_FILE}.tmp" "${CONFIG_FILE}"
        log INFO "Default threshold updated"
    fi
}

# Command: help
cmd_help() {
    cat <<EOF

${BLUE}WIA-FRAUD_PREVENTION CLI v${VERSION}${NC}
${CYAN}弘익人間 (홍익인간) - Benefit All Humanity${NC}

${YELLOW}USAGE:${NC}
  wia-fraud-prevention.sh <command> [options]

${YELLOW}COMMANDS:${NC}
  ${GREEN}fraud-detect${NC}           Detect fraud in a single transaction
  ${GREEN}analyze-transaction${NC}    Analyze batch transactions from file
  ${GREEN}train-model${NC}            Train a new ML fraud detection model
  ${GREEN}alert-setup${NC}            Set up fraud alert rules
  ${GREEN}monitor-start${NC}          Start enhanced monitoring session
  ${GREEN}report${NC}                 Generate fraud prevention reports
  ${GREEN}config${NC}                 Configure CLI settings
  ${GREEN}help${NC}                   Show this help message

${YELLOW}EXAMPLES:${NC}
  # Detect fraud in a transaction
  wia-fraud-prevention.sh fraud-detect \\
    --transaction-id txn_001 \\
    --amount 150.00 \\
    --user-id usr_123 \\
    --merchant-id mch_456 \\
    --ip 192.168.1.100

  # Analyze batch transactions
  wia-fraud-prevention.sh analyze-transaction --file transactions.json

  # Train a new model
  wia-fraud-prevention.sh train-model \\
    --name "Fraud Model v2" \\
    --type gradient_boosting \\
    --dataset dataset_2026_q1

  # Set up alert rule
  wia-fraud-prevention.sh alert-setup \\
    --name "High Risk Alert" \\
    --threshold 80 \\
    --action block

  # Start monitoring session
  wia-fraud-prevention.sh monitor-start \\
    --user-id usr_123 \\
    --duration 3600

  # Generate report
  wia-fraud-prevention.sh report \\
    --type fraud_summary \\
    --start 2026-01-01T00:00:00Z \\
    --end 2026-01-12T23:59:59Z \\
    --format pdf

  # Configure API key
  wia-fraud-prevention.sh config --api-key wia_fp_your_api_key

${YELLOW}ENVIRONMENT VARIABLES:${NC}
  WIA_FP_API_URL        API base URL (default: https://api.fraud-prevention.wia.org/v1)
  WIA_FP_API_KEY        API authentication key
  DEBUG                 Enable debug logging (true/false)

${YELLOW}DOCUMENTATION:${NC}
  https://docs.fraud-prevention.wia.org

${YELLOW}SUPPORT:${NC}
  Email: support@wia.org
  GitHub: https://github.com/wia-official/wia-standards

© 2026 WIA (World Industry Association)

EOF
}

# Main entry point
main() {
    init_config
    load_config

    if [[ $# -eq 0 ]]; then
        cmd_help
        exit 0
    fi

    local command="$1"
    shift

    case "${command}" in
        fraud-detect)
            cmd_fraud_detect "$@"
            ;;
        analyze-transaction)
            cmd_analyze_transaction "$@"
            ;;
        train-model)
            cmd_train_model "$@"
            ;;
        alert-setup)
            cmd_alert_setup "$@"
            ;;
        monitor-start)
            cmd_monitor_start "$@"
            ;;
        report)
            cmd_report "$@"
            ;;
        config)
            cmd_config "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            log ERROR "Unknown command: ${command}"
            echo ""
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
