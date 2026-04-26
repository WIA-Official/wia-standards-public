#!/bin/bash

################################################################################
# WIA-IND-027: Industrial IoT CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to industrial IoT operations
# including OPC-UA, MQTT, time-series data, alerts, and device management.
################################################################################

set -e

# Colors for output
AMBER='\033[0;38;5;214m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
CONFIG_DIR="$HOME/.wia/ind-027"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🔌  WIA-IND-027: Industrial IoT CLI                  ║"
    echo "║                      Version $VERSION                            ║"
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

# Generate random ID
generate_id() {
    echo $(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
}

# Format timestamp
format_timestamp() {
    date -u +"%Y-%m-%dT%H:%M:%SZ"
}

# Load configuration
load_config() {
    if [ -f "$CONFIG_FILE" ]; then
        cat "$CONFIG_FILE"
    else
        echo "{}"
    fi
}

# Save configuration
save_config() {
    mkdir -p "$CONFIG_DIR"
    echo "$1" > "$CONFIG_FILE"
}

# ============================================================================
# OPC-UA Commands
# ============================================================================

opcua_connect() {
    local endpoint=""
    local username=""
    local password=""
    local security="None"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --endpoint) endpoint="$2"; shift 2 ;;
            --username) username="$2"; shift 2 ;;
            --password) password="$2"; shift 2 ;;
            --security) security="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$endpoint" ]; then
        print_error "Endpoint is required"
        print_info "Usage: wia-ind-027 opcua connect --endpoint opc.tcp://192.168.1.100:4840"
        return 1
    fi

    print_section "Connecting to OPC-UA Server"
    print_info "Endpoint: $endpoint"
    print_info "Security: $security"
    [ -n "$username" ] && print_info "Username: $username"

    # Simulate connection
    sleep 1

    print_success "Connected to OPC-UA server"
    print_info "Session ID: session-$(generate_id)"
    echo ""
    print_info "Available commands:"
    print_info "  wia-ind-027 opcua read --node \"ns=2;s=Temperature_Sensor_01\""
    print_info "  wia-ind-027 opcua write --node \"ns=2;s=Setpoint\" --value 50.0"
    print_info "  wia-ind-027 opcua subscribe --node \"ns=2;s=Machine_Status\""
    echo ""
}

opcua_read() {
    local node=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --node) node="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$node" ]; then
        print_error "Node ID is required"
        return 1
    fi

    print_section "Reading OPC-UA Node"
    print_info "Node: $node"

    # Simulate read
    sleep 0.5

    local value=$(awk -v min=40 -v max=60 'BEGIN{srand(); print min+rand()*(max-min)}')
    local timestamp=$(format_timestamp)

    echo ""
    print_success "Read successful"
    echo -e "${CYAN}┌─────────────────────────────────────────────┐${RESET}"
    echo -e "${CYAN}│${RESET} ${GREEN}Value:${RESET}      ${value}°C"
    echo -e "${CYAN}│${RESET} ${GREEN}Quality:${RESET}    Good"
    echo -e "${CYAN}│${RESET} ${GREEN}Timestamp:${RESET}  ${timestamp}"
    echo -e "${CYAN}└─────────────────────────────────────────────┘${RESET}"
    echo ""
}

opcua_write() {
    local node=""
    local value=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --node) node="$2"; shift 2 ;;
            --value) value="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$node" ] || [ -z "$value" ]; then
        print_error "Node ID and value are required"
        return 1
    fi

    print_section "Writing to OPC-UA Node"
    print_info "Node: $node"
    print_info "Value: $value"

    # Simulate write
    sleep 0.5

    print_success "Write successful"
    print_info "Status: Good"
    echo ""
}

opcua_subscribe() {
    local node=""
    local interval=1000

    while [[ $# -gt 0 ]]; do
        case $1 in
            --node) node="$2"; shift 2 ;;
            --interval) interval="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$node" ]; then
        print_error "Node ID is required"
        return 1
    fi

    print_section "Subscribing to OPC-UA Node"
    print_info "Node: $node"
    print_info "Interval: ${interval}ms"
    echo ""
    print_success "Subscription created"
    print_info "Press Ctrl+C to stop..."
    echo ""

    # Simulate subscription
    for i in {1..10}; do
        local value=$(awk -v min=40 -v max=60 'BEGIN{srand(); print min+rand()*(max-min)}')
        local timestamp=$(date +"%H:%M:%S")
        echo -e "${GRAY}[${timestamp}]${RESET} ${GREEN}${value}${RESET}"
        sleep $(echo "scale=2; $interval/1000" | bc)
    done

    echo ""
    print_info "Subscription ended"
}

# ============================================================================
# MQTT Commands
# ============================================================================

mqtt_publish() {
    local topic=""
    local message=""
    local qos=0
    local retain=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --topic) topic="$2"; shift 2 ;;
            --message) message="$2"; shift 2 ;;
            --qos) qos="$2"; shift 2 ;;
            --retain) retain=true; shift ;;
            *) shift ;;
        esac
    done

    if [ -z "$topic" ] || [ -z "$message" ]; then
        print_error "Topic and message are required"
        return 1
    fi

    print_section "Publishing MQTT Message"
    print_info "Topic: $topic"
    print_info "QoS: $qos"
    print_info "Retain: $retain"
    echo ""
    echo -e "${CYAN}Message:${RESET}"
    echo "$message" | jq '.' 2>/dev/null || echo "$message"
    echo ""

    # Simulate publish
    sleep 0.3

    print_success "Message published successfully"
    print_info "Message ID: msg-$(generate_id)"
    echo ""
}

mqtt_subscribe() {
    local topic=""
    local qos=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --topic) topic="$2"; shift 2 ;;
            --qos) qos="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$topic" ]; then
        print_error "Topic is required"
        return 1
    fi

    print_section "Subscribing to MQTT Topic"
    print_info "Topic: $topic"
    print_info "QoS: $qos"
    echo ""
    print_success "Subscribed successfully"
    print_info "Press Ctrl+C to stop..."
    echo ""

    # Simulate incoming messages
    for i in {1..5}; do
        local timestamp=$(date +"%H:%M:%S")
        local value=$(awk -v min=40 -v max=60 'BEGIN{srand(); print min+rand()*(max-min)}')
        echo -e "${GRAY}[${timestamp}]${RESET} ${CYAN}$topic${RESET}"
        echo -e "${GREEN}{\"value\": $value, \"unit\": \"celsius\", \"timestamp\": \"$(format_timestamp)\"}${RESET}"
        echo ""
        sleep 2
    done

    print_info "Subscription ended"
}

# ============================================================================
# Time-Series Commands
# ============================================================================

timeseries_write() {
    local measurement=""
    local sensor=""
    local value=""
    local unit=""
    local tags=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --measurement) measurement="$2"; shift 2 ;;
            --sensor) sensor="$2"; shift 2 ;;
            --value) value="$2"; shift 2 ;;
            --unit) unit="$2"; shift 2 ;;
            --tags) tags="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$measurement" ] || [ -z "$value" ]; then
        print_error "Measurement and value are required"
        return 1
    fi

    print_section "Writing Time-Series Data"
    print_info "Measurement: $measurement"
    [ -n "$sensor" ] && print_info "Sensor: $sensor"
    print_info "Value: $value ${unit}"
    print_info "Timestamp: $(format_timestamp)"

    # Simulate write
    sleep 0.3

    print_success "Data written successfully"
    echo ""
}

timeseries_query() {
    local measurement=""
    local start=""
    local end=""
    local agg="mean"
    local interval="5m"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --measurement) measurement="$2"; shift 2 ;;
            --start) start="$2"; shift 2 ;;
            --end) end="$2"; shift 2 ;;
            --agg) agg="$2"; shift 2 ;;
            --interval) interval="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$measurement" ]; then
        print_error "Measurement is required"
        return 1
    fi

    [ -z "$start" ] && start=$(date -d '1 day ago' +%Y-%m-%d)
    [ -z "$end" ] && end=$(date +%Y-%m-%d)

    print_section "Querying Time-Series Data"
    print_info "Measurement: $measurement"
    print_info "Time Range: $start to $end"
    print_info "Aggregation: $agg($interval)"
    echo ""

    # Simulate query
    sleep 0.5

    print_success "Query completed"
    echo ""
    echo -e "${CYAN}Results:${RESET}"
    echo -e "${GRAY}┌────────────────────┬──────────┐${RESET}"
    echo -e "${GRAY}│ Timestamp          │ Value    │${RESET}"
    echo -e "${GRAY}├────────────────────┼──────────┤${RESET}"

    for i in {1..5}; do
        local ts=$(date -d "$i hours ago" +"%Y-%m-%d %H:00")
        local val=$(awk -v min=40 -v max=60 'BEGIN{srand(); print min+rand()*(max-min)}')
        printf "${GRAY}│${RESET} %-18s ${GRAY}│${RESET} %8.2f ${GRAY}│${RESET}\n" "$ts" "$val"
    done

    echo -e "${GRAY}└────────────────────┴──────────┘${RESET}"
    print_info "Total rows: 5"
    echo ""
}

# ============================================================================
# Device Management Commands
# ============================================================================

devices_list() {
    local protocol=""
    local status=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --protocol) protocol="$2"; shift 2 ;;
            --status) status="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    print_section "Listing Devices"
    [ -n "$protocol" ] && print_info "Filter - Protocol: $protocol"
    [ -n "$status" ] && print_info "Filter - Status: $status"
    echo ""

    # Simulate device list
    sleep 0.5

    echo -e "${CYAN}Devices:${RESET}"
    echo -e "${GRAY}┌──────────────┬──────────┬───────────┬──────────┬────────────────┐${RESET}"
    echo -e "${GRAY}│ Device ID    │ Type     │ Protocol  │ Status   │ Last Seen      │${RESET}"
    echo -e "${GRAY}├──────────────┼──────────┼───────────┼──────────┼────────────────┤${RESET}"

    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} %-8s ${GRAY}│${RESET} %-9s ${GRAY}│${RESET} ${GREEN}%-8s${RESET} ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "plc-001" "PLC" "OPC-UA" "online" "2 min ago"
    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} %-8s ${GRAY}│${RESET} %-9s ${GRAY}│${RESET} ${GREEN}%-8s${RESET} ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "temp-001" "Sensor" "MQTT" "online" "30 sec ago"
    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} %-8s ${GRAY}│${RESET} %-9s ${GRAY}│${RESET} ${YELLOW}%-8s${RESET} ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "robot-001" "Robot" "PROFINET" "idle" "1 min ago"
    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} %-8s ${GRAY}│${RESET} %-9s ${GRAY}│${RESET} ${RED}%-8s${RESET} ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "cnc-001" "CNC" "Modbus" "error" "5 min ago"

    echo -e "${GRAY}└──────────────┴──────────┴───────────┴──────────┴────────────────┘${RESET}"
    print_info "Total devices: 4"
    echo ""
}

devices_update() {
    local device=""
    local firmware=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --device) device="$2"; shift 2 ;;
            --firmware) firmware="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$device" ] || [ -z "$firmware" ]; then
        print_error "Device ID and firmware version are required"
        return 1
    fi

    print_section "Updating Device Firmware"
    print_info "Device: $device"
    print_info "Target Version: $firmware"
    echo ""

    print_info "Downloading firmware..."
    sleep 1
    print_success "Download complete"

    print_info "Uploading to device..."
    sleep 2
    print_success "Upload complete"

    print_info "Installing firmware..."
    sleep 2
    print_success "Installation complete"

    print_info "Verifying..."
    sleep 1
    print_success "Firmware update successful"
    echo ""
}

# ============================================================================
# Alert Commands
# ============================================================================

alerts_create() {
    local metric=""
    local threshold=""
    local severity="warning"
    local operator=">"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --metric) metric="$2"; shift 2 ;;
            --threshold) threshold="$2"; shift 2 ;;
            --severity) severity="$2"; shift 2 ;;
            --operator) operator="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$metric" ] || [ -z "$threshold" ]; then
        print_error "Metric and threshold are required"
        return 1
    fi

    print_section "Creating Alert Rule"
    print_info "Metric: $metric"
    print_info "Condition: $metric $operator $threshold"
    print_info "Severity: $severity"

    # Simulate creation
    sleep 0.5

    local rule_id="rule-$(generate_id)"

    print_success "Alert rule created"
    print_info "Rule ID: $rule_id"
    echo ""
}

alerts_list() {
    print_section "Active Alerts"
    echo ""

    # Simulate alerts
    sleep 0.5

    echo -e "${CYAN}Alerts:${RESET}"
    echo -e "${GRAY}┌──────────────┬──────────┬─────────────────────────────────┬────────────────┐${RESET}"
    echo -e "${GRAY}│ Alert ID     │ Severity │ Message                         │ Timestamp      │${RESET}"
    echo -e "${GRAY}├──────────────┼──────────┼─────────────────────────────────┼────────────────┤${RESET}"

    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} ${RED}%-8s${RESET} ${GRAY}│${RESET} %-31s ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "alert-001" "critical" "Temperature > 85°C" "2 min ago"
    printf "${GRAY}│${RESET} %-12s ${GRAY}│${RESET} ${YELLOW}%-8s${RESET} ${GRAY}│${RESET} %-31s ${GRAY}│${RESET} %-14s ${GRAY}│${RESET}\n" \
        "alert-002" "warning" "Vibration spike detected" "15 min ago"

    echo -e "${GRAY}└──────────────┴──────────┴─────────────────────────────────┴────────────────┘${RESET}"
    print_info "Total alerts: 2"
    echo ""
}

# ============================================================================
# Dashboard Command
# ============================================================================

dashboard() {
    local factory="factory-001"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --factory) factory="$2"; shift 2 ;;
            --live) shift ;;
            *) shift ;;
        esac
    done

    print_section "Production Dashboard"
    print_info "Factory: $factory"
    print_info "Updated: $(date +"%Y-%m-%d %H:%M:%S")"
    echo ""

    # OEE Metrics
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${CYAN}║                     OEE Metrics                           ║${RESET}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${RESET}"
    echo ""
    echo -e "${GREEN}  Availability:${RESET}  92.5%  ${GRAY}[████████████████████░░]${RESET}"
    echo -e "${GREEN}  Performance:${RESET}   87.3%  ${GRAY}[█████████████████░░░░]${RESET}"
    echo -e "${GREEN}  Quality:${RESET}       95.8%  ${GRAY}[███████████████████░░]${RESET}"
    echo -e "${AMBER}  Overall OEE:${RESET}  77.4%  ${GRAY}[███████████████░░░░░░]${RESET}"
    echo ""

    # Production Metrics
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${CYAN}║                 Production Metrics                        ║${RESET}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${RESET}"
    echo ""
    echo -e "${GREEN}  Units Produced:${RESET}      1,245 units"
    echo -e "${GREEN}  Production Rate:${RESET}     156 units/hour"
    echo -e "${GREEN}  Cycle Time:${RESET}          23 seconds"
    echo -e "${GREEN}  Scrap Rate:${RESET}          2.1%"
    echo ""

    # Equipment Status
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${CYAN}║                 Equipment Status                          ║${RESET}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${RESET}"
    echo ""
    echo -e "${GREEN}  ● Line A${RESET}      Running    ${GRAY}(OEE: 82.1%)${RESET}"
    echo -e "${GREEN}  ● Line B${RESET}      Running    ${GRAY}(OEE: 75.3%)${RESET}"
    echo -e "${YELLOW}  ● Line C${RESET}      Idle       ${GRAY}(Changeover)${RESET}"
    echo -e "${RED}  ● Line D${RESET}      Error      ${GRAY}(Motor fault)${RESET}"
    echo ""

    print_info "Press Ctrl+C to exit"
}

# ============================================================================
# Digital Twin Commands
# ============================================================================

twin_create() {
    local asset=""
    local type=""
    local model=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --asset) asset="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --model) model="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$asset" ] || [ -z "$type" ]; then
        print_error "Asset ID and type are required"
        return 1
    fi

    print_section "Creating Digital Twin"
    print_info "Asset: $asset"
    print_info "Type: $type"
    [ -n "$model" ] && print_info "Model: $model"

    # Simulate creation
    sleep 1

    print_success "Digital twin created"
    print_info "Twin ID: twin-$(generate_id)"
    echo ""
}

twin_sync() {
    local asset=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --asset) asset="$2"; shift 2 ;;
            --live) shift ;;
            *) shift ;;
        esac
    done

    if [ -z "$asset" ]; then
        print_error "Asset ID is required"
        return 1
    fi

    print_section "Syncing Digital Twin"
    print_info "Asset: $asset"
    echo ""

    print_info "Syncing sensor data..."
    sleep 1
    print_success "Sensors synchronized"

    print_info "Updating simulation model..."
    sleep 1
    print_success "Model updated"

    print_info "Running analytics..."
    sleep 1
    print_success "Analytics complete"

    echo ""
    print_success "Digital twin synchronized"
    echo ""
}

twin_predict() {
    local asset=""
    local metric="rul"
    local horizon="30d"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --asset) asset="$2"; shift 2 ;;
            --metric) metric="$2"; shift 2 ;;
            --horizon) horizon="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$asset" ]; then
        print_error "Asset ID is required"
        return 1
    fi

    print_section "Predictive Analysis"
    print_info "Asset: $asset"
    print_info "Metric: $metric"
    print_info "Horizon: $horizon"
    echo ""

    # Simulate prediction
    sleep 2

    print_success "Prediction complete"
    echo ""
    echo -e "${CYAN}Results:${RESET}"
    echo -e "${GREEN}  Remaining Useful Life:${RESET}  45 days"
    echo -e "${GREEN}  Confidence:${RESET}             85%"
    echo -e "${GREEN}  Failure Probability:${RESET}    12%"
    echo ""
    echo -e "${YELLOW}Recommendations:${RESET}"
    echo -e "${GRAY}  • Schedule maintenance in 30 days${RESET}"
    echo -e "${GRAY}  • Monitor vibration levels closely${RESET}"
    echo -e "${GRAY}  • Order replacement parts${RESET}"
    echo ""
}

# ============================================================================
# MES Integration Commands
# ============================================================================

mes_sync() {
    local order=""
    local status=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --production-order) order="$2"; shift 2 ;;
            --status) status="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$order" ]; then
        print_error "Production order is required"
        return 1
    fi

    print_section "Syncing with MES"
    print_info "Production Order: $order"
    [ -n "$status" ] && print_info "Status: $status"

    # Simulate sync
    sleep 1

    print_success "MES sync complete"
    print_info "Updated at: $(format_timestamp)"
    echo ""
}

# ============================================================================
# Main Command Router
# ============================================================================

show_help() {
    print_header
    echo "Usage: wia-ind-027 <command> [options]"
    echo ""
    echo -e "${CYAN}OPC-UA Commands:${RESET}"
    echo "  opcua connect     Connect to OPC-UA server"
    echo "  opcua read        Read node value"
    echo "  opcua write       Write node value"
    echo "  opcua subscribe   Subscribe to node changes"
    echo ""
    echo -e "${CYAN}MQTT Commands:${RESET}"
    echo "  mqtt publish      Publish message to topic"
    echo "  mqtt subscribe    Subscribe to topic"
    echo ""
    echo -e "${CYAN}Time-Series Commands:${RESET}"
    echo "  timeseries write  Write data point"
    echo "  timeseries query  Query historical data"
    echo ""
    echo -e "${CYAN}Device Management:${RESET}"
    echo "  devices list      List connected devices"
    echo "  devices update    Update device firmware"
    echo ""
    echo -e "${CYAN}Alert Management:${RESET}"
    echo "  alerts create     Create alert rule"
    echo "  alerts list       List active alerts"
    echo ""
    echo -e "${CYAN}Monitoring:${RESET}"
    echo "  dashboard         View production dashboard"
    echo ""
    echo -e "${CYAN}Digital Twin:${RESET}"
    echo "  twin create       Create digital twin"
    echo "  twin sync         Sync digital twin"
    echo "  twin predict      Run predictive analysis"
    echo ""
    echo -e "${CYAN}MES Integration:${RESET}"
    echo "  mes sync          Sync with MES system"
    echo ""
    echo -e "${CYAN}Other:${RESET}"
    echo "  version           Show version"
    echo "  help              Show this help"
    echo ""
    echo -e "${GRAY}For command-specific help: wia-ind-027 <command> --help${RESET}"
    echo ""
    echo -e "${AMBER}弘益人間 (Benefit All Humanity)${RESET}"
    echo ""
}

show_version() {
    print_header
    echo -e "${CYAN}Version:${RESET} $VERSION"
    echo -e "${CYAN}Standard:${RESET} WIA-IND-027"
    echo -e "${CYAN}License:${RESET} MIT"
    echo ""
}

# Main router
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        opcua)
            shift
            case "$1" in
                connect) shift; opcua_connect "$@" ;;
                read) shift; opcua_read "$@" ;;
                write) shift; opcua_write "$@" ;;
                subscribe) shift; opcua_subscribe "$@" ;;
                *) print_error "Unknown opcua command: $1"; exit 1 ;;
            esac
            ;;
        mqtt)
            shift
            case "$1" in
                publish) shift; mqtt_publish "$@" ;;
                subscribe) shift; mqtt_subscribe "$@" ;;
                *) print_error "Unknown mqtt command: $1"; exit 1 ;;
            esac
            ;;
        timeseries)
            shift
            case "$1" in
                write) shift; timeseries_write "$@" ;;
                query) shift; timeseries_query "$@" ;;
                *) print_error "Unknown timeseries command: $1"; exit 1 ;;
            esac
            ;;
        devices)
            shift
            case "$1" in
                list) shift; devices_list "$@" ;;
                update) shift; devices_update "$@" ;;
                *) print_error "Unknown devices command: $1"; exit 1 ;;
            esac
            ;;
        alerts)
            shift
            case "$1" in
                create) shift; alerts_create "$@" ;;
                list) shift; alerts_list "$@" ;;
                *) print_error "Unknown alerts command: $1"; exit 1 ;;
            esac
            ;;
        dashboard)
            shift; dashboard "$@" ;;
        twin)
            shift
            case "$1" in
                create) shift; twin_create "$@" ;;
                sync) shift; twin_sync "$@" ;;
                predict) shift; twin_predict "$@" ;;
                *) print_error "Unknown twin command: $1"; exit 1 ;;
            esac
            ;;
        mes)
            shift
            case "$1" in
                sync) shift; mes_sync "$@" ;;
                *) print_error "Unknown mes command: $1"; exit 1 ;;
            esac
            ;;
        version)
            show_version ;;
        help|--help|-h)
            show_help ;;
        *)
            print_error "Unknown command: $1"
            echo "Run 'wia-ind-027 help' for usage"
            exit 1
            ;;
    esac
}

main "$@"

# 弘益人間 (Benefit All Humanity)
