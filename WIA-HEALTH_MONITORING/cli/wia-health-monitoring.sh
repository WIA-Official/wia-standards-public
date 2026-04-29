#!/usr/bin/env bash
# WIA-HEALTH_MONITORING CLI Tool
# Version: 1.0.0
# 弘益人間 (홍익인간) - Benefit All Humanity

set -euo pipefail

# Configuration
WIA_API_BASE="${WIA_API_BASE:-https://api.wia-health.org/v1}"
WIA_CONFIG_FILE="${HOME}/.wia-health-monitoring/config.json"
WIA_API_KEY="${WIA_API_KEY:-}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Helper functions
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $*"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# Load configuration
load_config() {
    if [[ -f "$WIA_CONFIG_FILE" ]]; then
        WIA_API_KEY=$(jq -r '.api_key // ""' "$WIA_CONFIG_FILE")
        WIA_API_BASE=$(jq -r '.api_base // "https://api.wia-health.org/v1"' "$WIA_CONFIG_FILE")
    fi
}

# Save configuration
save_config() {
    local api_key="$1"
    local api_base="${2:-https://api.wia-health.org/v1}"

    mkdir -p "$(dirname "$WIA_CONFIG_FILE")"
    cat > "$WIA_CONFIG_FILE" <<EOF
{
  "api_key": "$api_key",
  "api_base": "$api_base",
  "created_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
    chmod 600 "$WIA_CONFIG_FILE"
    log_success "Configuration saved to $WIA_CONFIG_FILE"
}

# API request wrapper
api_request() {
    local method="$1"
    local endpoint="$2"
    local data="${3:-}"

    if [[ -z "$WIA_API_KEY" ]]; then
        log_error "API key not configured. Run: wia-health-monitoring config --api-key YOUR_KEY"
        exit 1
    fi

    local curl_args=(
        -s -X "$method"
        -H "Authorization: Bearer $WIA_API_KEY"
        -H "Content-Type: application/json"
        -H "User-Agent: WIA-HealthMonitoring-CLI/1.0"
    )

    if [[ -n "$data" ]]; then
        curl_args+=(-d "$data")
    fi

    curl "${curl_args[@]}" "${WIA_API_BASE}${endpoint}"
}

# Command: config
cmd_config() {
    local api_key=""
    local api_base=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --api-key) api_key="$2"; shift 2 ;;
            --api-base) api_base="$2"; shift 2 ;;
            --show)
                if [[ -f "$WIA_CONFIG_FILE" ]]; then
                    log_info "Current configuration:"
                    jq '.' "$WIA_CONFIG_FILE"
                else
                    log_warning "No configuration found"
                fi
                return 0
                ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    if [[ -n "$api_key" ]]; then
        save_config "$api_key" "$api_base"
    else
        log_error "Missing --api-key parameter"
        exit 1
    fi
}

# Command: device-sync
cmd_device_sync() {
    local device_id=""
    local force=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --device-id) device_id="$2"; shift 2 ;;
            --force) force=true; shift ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    log_info "Synchronizing devices..."

    if [[ -n "$device_id" ]]; then
        log_info "Syncing device: $device_id"
        response=$(api_request POST "/devices/$device_id/sync" "{\"force\": $force}")
    else
        log_info "Syncing all devices"
        response=$(api_request POST "/devices/sync" "{\"force\": $force}")
    fi

    echo "$response" | jq '.'

    if echo "$response" | jq -e '.status == "success"' >/dev/null 2>&1; then
        log_success "Device sync completed successfully"
    else
        log_error "Device sync failed"
        exit 1
    fi
}

# Command: health-report
cmd_health_report() {
    local period="daily"
    local date="$(date +%Y-%m-%d)"
    local format="json"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --period) period="$2"; shift 2 ;;
            --date) date="$2"; shift 2 ;;
            --format) format="$2"; shift 2 ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    log_info "Fetching health report for $date ($period)..."

    local endpoint
    case $period in
        daily) endpoint="/summaries/daily/$date" ;;
        weekly)
            local year=$(date -d "$date" +%Y)
            local week=$(date -d "$date" +%V)
            endpoint="/summaries/weekly/$year/$week"
            ;;
        monthly)
            local year=$(date -d "$date" +%Y)
            local month=$(date -d "$date" +%m)
            endpoint="/summaries/monthly/$year/$month"
            ;;
        *) log_error "Invalid period: $period"; exit 1 ;;
    esac

    response=$(api_request GET "$endpoint")

    if [[ "$format" == "json" ]]; then
        echo "$response" | jq '.'
    elif [[ "$format" == "table" ]]; then
        echo -e "\n${CYAN}=== Health Summary ===${NC}"
        echo "$response" | jq -r '
            "Date: \(.date // "N/A")",
            "",
            "Heart Rate:",
            "  Average: \(.metrics.heart_rate.avg // "N/A") bpm",
            "  Min: \(.metrics.heart_rate.min // "N/A") bpm",
            "  Max: \(.metrics.heart_rate.max // "N/A") bpm",
            "  Resting: \(.metrics.heart_rate.resting // "N/A") bpm",
            "",
            "Activity:",
            "  Steps: \(.metrics.activity.steps // "N/A")",
            "  Distance: \(.metrics.activity.distance_km // "N/A") km",
            "  Calories: \(.metrics.activity.calories // "N/A")",
            "",
            "Sleep:",
            "  Duration: \(.metrics.sleep.total_minutes // "N/A") minutes",
            "  Efficiency: \(.metrics.sleep.efficiency // "N/A")%",
            "  Score: \(.metrics.sleep.sleep_score // "N/A")/100"
        '
    else
        log_error "Invalid format: $format"
        exit 1
    fi
}

# Command: metric-query
cmd_metric_query() {
    local metric_type=""
    local start=""
    local end=""
    local limit=50

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) metric_type="$2"; shift 2 ;;
            --start) start="$2"; shift 2 ;;
            --end) end="$2"; shift 2 ;;
            --limit) limit="$2"; shift 2 ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    if [[ -z "$metric_type" ]]; then
        log_error "Missing --type parameter"
        exit 1
    fi

    log_info "Querying $metric_type metrics..."

    local query="metric_type=$metric_type&limit=$limit"
    [[ -n "$start" ]] && query+="&start=$start"
    [[ -n "$end" ]] && query+="&end=$end"

    response=$(api_request GET "/metrics?$query")
    echo "$response" | jq '.'

    local count=$(echo "$response" | jq '.metrics | length')
    log_success "Retrieved $count metrics"
}

# Command: alert-setup
cmd_alert_setup() {
    local metric_type=""
    local condition=""
    local threshold=""
    local severity="WARNING"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) metric_type="$2"; shift 2 ;;
            --condition) condition="$2"; shift 2 ;;
            --threshold) threshold="$2"; shift 2 ;;
            --severity) severity="$2"; shift 2 ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    if [[ -z "$metric_type" || -z "$condition" || -z "$threshold" ]]; then
        log_error "Missing required parameters: --type, --condition, --threshold"
        exit 1
    fi

    log_info "Setting up alert for $metric_type..."

    local payload=$(cat <<EOF
{
  "alert_rules": [{
    "metric_type": "$metric_type",
    "condition": "$condition",
    "threshold": $threshold,
    "severity": "$severity",
    "notification_channels": ["PUSH", "EMAIL"]
  }]
}
EOF
    )

    response=$(api_request POST "/alerts/config" "$payload")
    echo "$response" | jq '.'
    log_success "Alert configured successfully"
}

# Command: threshold-set
cmd_threshold_set() {
    local metric_type=""
    local low=""
    local high=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) metric_type="$2"; shift 2 ;;
            --low) low="$2"; shift 2 ;;
            --high) high="$2"; shift 2 ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    if [[ -z "$metric_type" ]]; then
        log_error "Missing --type parameter"
        exit 1
    fi

    log_info "Setting thresholds for $metric_type..."

    local rules="[]"
    if [[ -n "$low" ]]; then
        rules=$(echo "$rules" | jq --arg mt "$metric_type" --argjson val "$low" \
            '. + [{"metric_type": $mt, "condition": "LESS_THAN", "threshold": $val, "severity": "WARNING"}]')
    fi
    if [[ -n "$high" ]]; then
        rules=$(echo "$rules" | jq --arg mt "$metric_type" --argjson val "$high" \
            '. + [{"metric_type": $mt, "condition": "GREATER_THAN", "threshold": $val, "severity": "WARNING"}]')
    fi

    local payload=$(jq -n --argjson r "$rules" '{"alert_rules": $r}')

    response=$(api_request POST "/alerts/config" "$payload")
    echo "$response" | jq '.'
    log_success "Thresholds configured"
}

# Command: export-data
cmd_export_data() {
    local format="JSON"
    local start=""
    local end=""
    local metric_types=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --format) format="$2"; shift 2 ;;
            --start) start="$2"; shift 2 ;;
            --end) end="$2"; shift 2 ;;
            --types) metric_types="$2"; shift 2 ;;
            *) log_error "Unknown option: $1"; exit 1 ;;
        esac
    done

    log_info "Requesting data export ($format format)..."

    local query="format=$format"
    [[ -n "$start" ]] && query+="&start=$start"
    [[ -n "$end" ]] && query+="&end=$end"
    [[ -n "$metric_types" ]] && query+="&metric_types=$metric_types"

    response=$(api_request GET "/export/metrics?$query")
    export_id=$(echo "$response" | jq -r '.export_id')

    log_info "Export initiated. ID: $export_id"
    log_info "Waiting for export to complete..."

    # Poll for completion
    local max_attempts=30
    local attempt=0
    while [[ $attempt -lt $max_attempts ]]; do
        sleep 2
        status_response=$(api_request GET "/export/$export_id")
        status=$(echo "$status_response" | jq -r '.status')

        if [[ "$status" == "COMPLETED" ]]; then
            download_url=$(echo "$status_response" | jq -r '.download_url')
            log_success "Export completed!"
            log_info "Download URL: $download_url"

            # Auto-download if wget/curl available
            local filename="wia-health-export-$(date +%Y%m%d-%H%M%S).$format"
            if command -v wget &> /dev/null; then
                wget -O "$filename" "$download_url"
                log_success "Downloaded to: $filename"
            elif command -v curl &> /dev/null; then
                curl -o "$filename" "$download_url"
                log_success "Downloaded to: $filename"
            fi
            return 0
        elif [[ "$status" == "FAILED" ]]; then
            log_error "Export failed"
            exit 1
        fi

        ((attempt++))
        echo -n "."
    done

    log_error "Export timeout"
    exit 1
}

# Command: help
cmd_help() {
    cat <<EOF
${CYAN}WIA-HEALTH_MONITORING CLI Tool${NC}
Version: 1.0.0

${YELLOW}USAGE:${NC}
    wia-health-monitoring <command> [options]

${YELLOW}COMMANDS:${NC}
    ${GREEN}config${NC}          Configure API credentials
    ${GREEN}device-sync${NC}     Synchronize device data
    ${GREEN}health-report${NC}   Generate health reports
    ${GREEN}metric-query${NC}    Query specific health metrics
    ${GREEN}alert-setup${NC}     Configure health alerts
    ${GREEN}threshold-set${NC}   Set metric thresholds
    ${GREEN}export-data${NC}     Export health data
    ${GREEN}help${NC}            Show this help message

${YELLOW}EXAMPLES:${NC}
    # Configure API key
    wia-health-monitoring config --api-key YOUR_API_KEY

    # Sync all devices
    wia-health-monitoring device-sync

    # Get daily health report
    wia-health-monitoring health-report --period daily --format table

    # Query heart rate data
    wia-health-monitoring metric-query --type HEART_RATE --start 2026-01-01 --limit 100

    # Setup glucose alert
    wia-health-monitoring alert-setup --type BLOOD_GLUCOSE --condition LESS_THAN --threshold 70 --severity CRITICAL

    # Set heart rate thresholds
    wia-health-monitoring threshold-set --type HEART_RATE --low 40 --high 140

    # Export data to CSV
    wia-health-monitoring export-data --format CSV --start 2026-01-01 --end 2026-01-31

${YELLOW}ENVIRONMENT VARIABLES:${NC}
    WIA_API_KEY          API authentication key
    WIA_API_BASE         API base URL (default: https://api.wia-health.org/v1)

${YELLOW}CONFIGURATION FILE:${NC}
    ${HOME}/.wia-health-monitoring/config.json

${MAGENTA}弘益人間 (홍익인간) - Benefit All Humanity${NC}
© 2026 WIA (World Certification Industry Association)

For more information, visit: https://wia.org/standards/health-monitoring
EOF
}

# Main execution
main() {
    if [[ $# -eq 0 ]]; then
        cmd_help
        exit 0
    fi

    load_config

    local command="$1"
    shift

    case "$command" in
        config) cmd_config "$@" ;;
        device-sync) cmd_device_sync "$@" ;;
        health-report) cmd_health_report "$@" ;;
        metric-query) cmd_metric_query "$@" ;;
        alert-setup) cmd_alert_setup "$@" ;;
        threshold-set) cmd_threshold_set "$@" ;;
        export-data) cmd_export_data "$@" ;;
        help|--help|-h) cmd_help ;;
        *)
            log_error "Unknown command: $command"
            echo ""
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
