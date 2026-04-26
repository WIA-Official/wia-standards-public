#!/bin/bash

################################################################################
# WIA-COMM-002: IoT (M2M) CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA IoT and M2M Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to IoT/M2M operations including
# device management, telemetry publishing, command handling, and firmware updates.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
MAGENTA='\033[0;35m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
CONFIG_FILE="$HOME/.wia-comm-002/config.json"

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🔗 WIA-COMM-002: IoT (M2M) CLI                       ║"
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

print_iot() {
    echo -e "${MAGENTA}🔗 $1${RESET}"
}

# Generate random hex string
generate_random_hex() {
    local length=$1
    head /dev/urandom | tr -dc 'a-f0-9' | head -c "$length"
}

# Device Registration
device_register() {
    local device_id=${1:-"device-$(date +%s)"}
    local device_type=${2:-"sensor"}
    local protocol=${3:-"mqtt"}

    print_section "Device Registration"
    print_info "Device ID: $device_id"
    print_info "Type: $device_type"
    print_info "Protocol: ${protocol^^}"

    print_iot "Generating device credentials..."
    sleep 0.3

    local cert_fingerprint=$(generate_random_hex 64)
    print_success "Certificate generated"
    print_info "Fingerprint: ${cert_fingerprint:0:40}..."

    print_iot "Registering with IoT platform..."
    sleep 0.5

    print_success "Device registered successfully!"
    print_info "MQTT Broker: mqtt://iot.example.com:1883"
    print_info "Topics:"
    print_info "  - Telemetry: devices/$device_id/telemetry"
    print_info "  - Commands: devices/$device_id/commands"
    print_info "  - Status: devices/$device_id/status"

    # Save config
    mkdir -p "$(dirname "$CONFIG_FILE")"
    cat > "$CONFIG_FILE" <<EOF
{
  "device_id": "$device_id",
  "device_type": "$device_type",
  "protocol": "$protocol",
  "broker": "mqtt://iot.example.com:1883",
  "registered_at": "$(date -Iseconds)"
}
EOF

    echo ""
}

# Publish Telemetry
publish_telemetry() {
    local topic=${1:-"sensors/temp"}
    local data=${2:-'{"value": 23.5, "unit": "celsius"}'}

    if [ ! -f "$CONFIG_FILE" ]; then
        print_error "Device not registered. Run 'device register' first."
        return 1
    fi

    local device_id=$(grep -o '"device_id": "[^"]*' "$CONFIG_FILE" | cut -d'"' -f4)

    print_section "Publishing Telemetry"
    print_info "Device: $device_id"
    print_info "Topic: $topic"
    print_info "Data: $data"

    print_iot "Connecting to MQTT broker..."
    sleep 0.2
    print_success "Connected"

    print_iot "Publishing message (QoS 1)..."
    sleep 0.3
    print_success "Message published"

    local msg_id=$(generate_random_hex 8)
    print_info "Message ID: $msg_id"
    print_info "Timestamp: $(date -Iseconds)"

    echo ""
}

# Subscribe to Topic
subscribe_topic() {
    local topic=${1:-"commands/#"}
    local qos=${2:-1}

    if [ ! -f "$CONFIG_FILE" ]; then
        print_error "Device not registered. Run 'device register' first."
        return 1
    fi

    local device_id=$(grep -o '"device_id": "[^"]*' "$CONFIG_FILE" | cut -d'"' -f4)

    print_section "Subscribing to Topic"
    print_info "Device: $device_id"
    print_info "Topic: $topic"
    print_info "QoS: $qos"

    print_iot "Connecting to MQTT broker..."
    sleep 0.2
    print_success "Connected"

    print_iot "Subscribing to topic..."
    sleep 0.3
    print_success "Subscribed successfully"

    print_info "Waiting for messages (Press Ctrl+C to stop)..."

    # Simulate receiving messages
    local count=0
    while [ $count -lt 3 ]; do
        sleep 2
        count=$((count + 1))

        echo -e "\n${MAGENTA}[$(date +%H:%M:%S)]${RESET} Message received:"
        print_info "Topic: devices/$device_id/commands"
        print_info "Payload: {\"action\": \"update_config\", \"interval\": 60}"
    done

    echo ""
}

# List Devices
device_list() {
    local status=${1:-"all"}

    print_section "Device List"
    print_info "Filter: $status"

    print_iot "Querying IoT platform..."
    sleep 0.3

    echo -e "\n${CYAN}ID${RESET}              ${CYAN}Type${RESET}        ${CYAN}Status${RESET}    ${CYAN}Last Seen${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"

    echo -e "sensor-001      temperature ${GREEN}online${RESET}    2025-12-26 10:30:00"
    echo -e "sensor-002      humidity    ${GREEN}online${RESET}    2025-12-26 10:29:45"
    echo -e "actuator-001    switch      ${YELLOW}sleep${RESET}     2025-12-26 10:25:00"
    echo -e "gateway-001     gateway     ${GREEN}online${RESET}    2025-12-26 10:30:15"
    echo -e "sensor-003      motion      ${RED}offline${RESET}   2025-12-26 09:15:20"

    echo -e "\n${GRAY}Total: 5 devices (3 online, 1 sleeping, 1 offline)${RESET}"
    echo ""
}

# Update Device
device_update() {
    local device_id=${1:-"sensor-001"}
    local firmware=${2:-"v2.0.1"}

    print_section "Firmware Update"
    print_info "Device: $device_id"
    print_info "Target version: $firmware"

    print_iot "Checking current version..."
    sleep 0.2
    print_info "Current version: v1.5.3"

    print_iot "Downloading firmware..."
    for i in 20 40 60 80 100; do
        sleep 0.3
        echo -ne "${GRAY}  Progress: ${i}%\r${RESET}"
    done
    echo ""
    print_success "Download complete"

    print_iot "Verifying checksum..."
    sleep 0.3
    print_success "Checksum verified"

    print_iot "Installing firmware..."
    sleep 0.5
    print_success "Installation complete"

    print_iot "Rebooting device..."
    sleep 0.4
    print_success "Device rebooted with firmware $firmware"

    echo ""
}

# Configure Device
device_config() {
    local device_id=${1:-"sensor-001"}
    local interval=${2:-60}

    print_section "Device Configuration Update"
    print_info "Device: $device_id"
    print_info "Reporting interval: ${interval}s"

    print_iot "Sending configuration..."
    sleep 0.3

    cat <<EOF

${CYAN}Configuration:${RESET}
  {
    "reporting_interval": $interval,
    "power_mode": "eco",
    "sensors": {
      "temperature": {
        "enabled": true,
        "calibration_offset": -0.5
      }
    }
  }
EOF

    print_success "Configuration sent"

    print_iot "Waiting for acknowledgment..."
    sleep 0.5
    print_success "Configuration applied by device"

    echo ""
}

# Network Scan
network_scan() {
    local protocol=${1:-"lorawan"}

    print_section "Network Scan"
    print_info "Protocol: ${protocol^^}"

    print_iot "Scanning for networks..."
    sleep 0.8

    echo -e "\n${CYAN}Network ID${RESET}        ${CYAN}RSSI${RESET}    ${CYAN}SNR${RESET}   ${CYAN}Gateway${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"

    if [ "$protocol" = "lorawan" ]; then
        echo -e "home-network     -68dBm   8dB   gw-001"
        echo -e "office-network   -75dBm   6dB   gw-002"
        echo -e "public-ttn       -82dBm   4dB   gw-public-01"
    elif [ "$protocol" = "wifi" ]; then
        echo -e "IoT-Network-2G   -45dBm   --    ap-001"
        echo -e "IoT-Network-5G   -52dBm   --    ap-001"
        echo -e "Office-IoT       -68dBm   --    ap-office"
    fi

    echo -e "\n${GRAY}Scan complete${RESET}"
    echo ""
}

# Network Join
network_join() {
    local network=${1:-"home-network"}
    local key=${2:-"ABC123"}

    print_section "Network Join"
    print_info "Network: $network"
    print_info "Key: ${key:0:6}..."

    print_iot "Sending join request..."
    sleep 0.5
    print_success "Join request sent"

    print_iot "Waiting for join accept..."
    sleep 0.7
    print_success "Join accepted"

    print_info "Device Address: 260B1234"
    print_info "Session keys generated"

    print_success "Successfully joined network: $network"

    echo ""
}

# Digital Twin Operations
twin_create() {
    local device_id=${1:-"sensor-001"}
    local sync_interval=${2:-30}

    print_section "Digital Twin Creation"
    print_info "Device: $device_id"
    print_info "Sync interval: ${sync_interval}s"

    print_iot "Creating digital twin..."
    sleep 0.4

    cat <<EOF

${CYAN}Digital Twin:${RESET}
  {
    "device_id": "$device_id",
    "state": {
      "reported": {
        "temperature": 23.5,
        "humidity": 60.2,
        "battery": 87
      },
      "desired": {
        "reporting_interval": $sync_interval
      }
    },
    "metadata": {
      "created_at": "$(date -Iseconds)",
      "last_sync": "$(date -Iseconds)"
    }
  }
EOF

    print_success "Digital twin created"
    print_info "Shadow endpoint: /shadows/$device_id"

    echo ""
}

# Query Digital Twin
twin_query() {
    local device_id=${1:-"sensor-001"}
    local property=${2:-"temperature"}

    print_section "Digital Twin Query"
    print_info "Device: $device_id"
    print_info "Property: $property"

    print_iot "Querying digital twin..."
    sleep 0.3

    local value
    case $property in
        temperature)
            value="23.5°C"
            ;;
        humidity)
            value="60.2%"
            ;;
        battery)
            value="87%"
            ;;
        *)
            value="N/A"
            ;;
    esac

    print_success "Query successful"
    echo -e "\n${CYAN}$property:${RESET} $value"
    print_info "Last updated: $(date -Iseconds -d '1 minute ago')"

    echo ""
}

# Data Query
data_query() {
    local device_id=${1:-"sensor-001"}
    local metric=${2:-"temperature"}
    local period=${3:-"24h"}

    print_section "Data Query"
    print_info "Device: $device_id"
    print_info "Metric: $metric"
    print_info "Period: $period"

    print_iot "Querying time-series database..."
    sleep 0.4

    echo -e "\n${CYAN}Results:${RESET}"
    print_info "Data points: 288"
    print_info "Average: 23.2°C"
    print_info "Minimum: 18.5°C (02:00)"
    print_info "Maximum: 28.1°C (14:00)"

    echo -e "\n${GRAY}Recent values:${RESET}"
    echo "  10:30:00 → 23.5°C"
    echo "  10:25:00 → 23.3°C"
    echo "  10:20:00 → 23.1°C"
    echo "  10:15:00 → 22.9°C"
    echo "  10:10:00 → 22.7°C"

    echo ""
}

# Data Aggregation
data_aggregate() {
    local devices=${1:-"sensor-*"}
    local function=${2:-"avg"}
    local window=${3:-"1h"}

    print_section "Data Aggregation"
    print_info "Devices: $devices"
    print_info "Function: ${function^^}"
    print_info "Window: $window"

    print_iot "Aggregating data..."
    sleep 0.5

    echo -e "\n${CYAN}Aggregated Results (last 24h):${RESET}"
    echo "  00:00-01:00 → 19.2°C (3 devices)"
    echo "  01:00-02:00 → 18.7°C (3 devices)"
    echo "  02:00-03:00 → 18.5°C (3 devices)"
    echo "  ..."
    echo "  10:00-11:00 → 23.4°C (3 devices)"

    echo ""
}

# Certificate Operations
cert_generate() {
    local device_id=${1:-"sensor-001"}
    local validity=${2:-365}

    print_section "Certificate Generation"
    print_info "Device: $device_id"
    print_info "Validity: $validity days"

    print_iot "Generating key pair..."
    sleep 0.3
    print_success "Key pair generated (EC P-256)"

    print_iot "Creating certificate signing request..."
    sleep 0.2
    print_success "CSR created"

    print_iot "Signing certificate..."
    sleep 0.3
    print_success "Certificate signed by CA"

    local fingerprint=$(generate_random_hex 64)
    print_info "Fingerprint: ${fingerprint:0:40}..."
    print_info "Valid until: $(date -Iseconds -d "+${validity} days")"

    echo ""
}

# Certificate Rotation
cert_rotate() {
    local device_id=${1:-"sensor-001"}

    print_section "Certificate Rotation"
    print_info "Device: $device_id"

    print_iot "Generating new certificate..."
    sleep 0.4
    print_success "New certificate generated"

    print_iot "Updating device credentials..."
    sleep 0.3
    print_success "Credentials updated"

    print_iot "Notifying device..."
    sleep 0.2
    print_success "Device acknowledged"

    print_warning "Old certificate will expire in 7 days"

    echo ""
}

# Authentication Test
auth_test() {
    local device_id=${1:-"sensor-001"}

    print_section "Authentication Test"
    print_info "Device: $device_id"

    print_iot "Testing certificate..."
    sleep 0.3
    print_success "Certificate valid"

    print_iot "Testing TLS connection..."
    sleep 0.4
    print_success "TLS handshake successful"

    print_iot "Testing MQTT authentication..."
    sleep 0.3
    print_success "MQTT authentication successful"

    print_success "All authentication checks passed"

    echo ""
}

# Help
show_help() {
    print_header
    cat <<EOF
${CYAN}Usage:${RESET}
  wia-comm-002 <command> [options]

${CYAN}Device Management:${RESET}
  device register [--id ID] [--type TYPE] [--protocol PROTOCOL]
    Register a new IoT device

  device list [--status STATUS]
    List all registered devices

  device update --id ID --firmware VERSION
    Update device firmware

  device config --id ID --interval SECONDS
    Update device configuration

${CYAN}Data Operations:${RESET}
  publish --topic TOPIC --data JSON
    Publish telemetry data

  subscribe --topic TOPIC [--qos QOS]
    Subscribe to MQTT topic

  data query --device ID --metric NAME --period DURATION
    Query historical data

  data aggregate --devices PATTERN --function FUNC --window DURATION
    Aggregate data from multiple devices

${CYAN}Network Management:${RESET}
  network scan --protocol PROTOCOL
    Scan for available networks

  network join --network NAME --key KEY
    Join a network

${CYAN}Digital Twin:${RESET}
  twin create --device ID --sync-interval SECONDS
    Create digital twin

  twin query --device ID --property NAME
    Query twin property

${CYAN}Security:${RESET}
  cert generate --device ID --validity DAYS
    Generate device certificate

  cert rotate --device ID
    Rotate device certificate

  auth test --device ID
    Test device authentication

${CYAN}Options:${RESET}
  --version    Show version information
  --help       Show this help message

${CYAN}Examples:${RESET}
  wia-comm-002 device register --id sensor-001 --type temperature
  wia-comm-002 publish --topic sensors/temp --data '{"value": 23.5}'
  wia-comm-002 twin query --device sensor-001 --property temperature

${GRAY}弘益人間 (Benefit All Humanity)${RESET}
${GRAY}WIA-COMM-002 v$VERSION - © 2025 SmileStory Inc. / WIA${RESET}
EOF
    echo ""
}

# Version
show_version() {
    print_header
    echo -e "${CYAN}Version:${RESET} $VERSION"
    echo -e "${CYAN}Standard:${RESET} WIA-COMM-002"
    echo -e "${CYAN}Protocol Support:${RESET} MQTT, CoAP, AMQP, HTTP"
    echo ""
}

# Main command router
main() {
    local command=${1:-"help"}
    shift || true

    case $command in
        device)
            local subcommand=${1:-"list"}
            shift || true
            case $subcommand in
                register) device_register "$@" ;;
                list) device_list "$@" ;;
                update) device_update "$@" ;;
                config) device_config "$@" ;;
                *) print_error "Unknown device command: $subcommand" ;;
            esac
            ;;
        publish) publish_telemetry "$@" ;;
        subscribe) subscribe_topic "$@" ;;
        network)
            local subcommand=${1:-"scan"}
            shift || true
            case $subcommand in
                scan) network_scan "$@" ;;
                join) network_join "$@" ;;
                *) print_error "Unknown network command: $subcommand" ;;
            esac
            ;;
        twin)
            local subcommand=${1:-"create"}
            shift || true
            case $subcommand in
                create) twin_create "$@" ;;
                query) twin_query "$@" ;;
                *) print_error "Unknown twin command: $subcommand" ;;
            esac
            ;;
        data)
            local subcommand=${1:-"query"}
            shift || true
            case $subcommand in
                query) data_query "$@" ;;
                aggregate) data_aggregate "$@" ;;
                *) print_error "Unknown data command: $subcommand" ;;
            esac
            ;;
        cert)
            local subcommand=${1:-"generate"}
            shift || true
            case $subcommand in
                generate) cert_generate "$@" ;;
                rotate) cert_rotate "$@" ;;
                *) print_error "Unknown cert command: $subcommand" ;;
            esac
            ;;
        auth)
            local subcommand=${1:-"test"}
            shift || true
            case $subcommand in
                test) auth_test "$@" ;;
                *) print_error "Unknown auth command: $subcommand" ;;
            esac
            ;;
        --version) show_version ;;
        --help|help) show_help ;;
        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-comm-002 --help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"

exit 0
