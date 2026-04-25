#!/bin/bash

################################################################################
# WIA-CORE-007: Universal Protocol CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to the Universal Protocol for
# cross-platform message exchange and RPC operations.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        📡 WIA-CORE-007: Universal Protocol CLI                ║"
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

# Generate unique message ID
generate_id() {
    local prefix=$1
    local timestamp=$(date +%s%N | cut -b1-13)
    local random=$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)
    echo "${prefix}_${timestamp}${random}"
}

# Send RPC call
rpc_call() {
    local method="$1"
    local payload="$2"
    local endpoint="${3:-http://localhost:8080}"
    local headers="${4:-}"

    print_section "RPC Call"
    print_info "Method: $method"
    print_info "Endpoint: $endpoint"
    print_info "Payload: $payload"

    # Generate message ID
    local msg_id=$(generate_id "req")
    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    # Create JSON message
    local message=$(cat <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "request",
  "method": "$method",
  "headers": {
    "content-type": "application/json"
  },
  "payload": $payload,
  "metadata": {
    "timestamp": "$timestamp",
    "source": "cli"
  }
}
EOF
)

    print_section "Request Message"
    echo "$message" | jq '.' 2>/dev/null || echo "$message"

    # Send HTTP request
    print_section "Sending Request"

    if command -v curl &> /dev/null; then
        local response=$(curl -s -X POST "$endpoint/rpc" \
            -H "Content-Type: application/json" \
            $headers \
            -d "$message")

        print_section "Response"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"

        # Parse status
        local status=$(echo "$response" | jq -r '.status' 2>/dev/null || echo "unknown")

        if [ "$status" == "success" ]; then
            print_success "RPC call successful"
        else
            print_error "RPC call failed"
        fi
    else
        print_error "curl not found. Please install curl to make HTTP requests."
        exit 1
    fi

    echo ""
}

# Subscribe to events
subscribe_events() {
    local channel="$1"
    local endpoint="${2:-ws://localhost:8080}"

    print_section "Event Subscription"
    print_info "Channel: $channel"
    print_info "Endpoint: $endpoint"

    # Generate subscription message
    local msg_id=$(generate_id "sub")
    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    local message=$(cat <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "request",
  "method": "events.subscribe",
  "payload": {
    "channels": ["$channel"]
  },
  "metadata": {
    "timestamp": "$timestamp"
  }
}
EOF
)

    print_section "Subscription Message"
    echo "$message" | jq '.' 2>/dev/null || echo "$message"

    print_warning "WebSocket subscription requires a WebSocket client"
    print_info "Use websocat or wscat to connect:"
    print_info "  wscat -c $endpoint"
    print_info "  Then send: $message"

    echo ""
}

# Send custom message
send_message() {
    local type="$1"
    local method="$2"
    local payload="$3"
    local endpoint="${4:-http://localhost:8080}"

    print_section "Sending Message"
    print_info "Type: $type"
    print_info "Method: $method"
    print_info "Endpoint: $endpoint"

    # Generate message
    local msg_id=$(generate_id "$type")
    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    local message=$(cat <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "$type",
  "method": "$method",
  "payload": $payload,
  "metadata": {
    "timestamp": "$timestamp",
    "source": "cli"
  }
}
EOF
)

    print_section "Message"
    echo "$message" | jq '.' 2>/dev/null || echo "$message"

    # Send via HTTP POST
    if command -v curl &> /dev/null; then
        print_section "Sending"
        curl -s -X POST "$endpoint/rpc" \
            -H "Content-Type: application/json" \
            -d "$message" | jq '.' 2>/dev/null || cat
    else
        print_warning "curl not found. Message formatted but not sent."
    fi

    echo ""
}

# Start protocol server
start_server() {
    local port="${1:-8080}"
    local transport="${2:-http}"

    print_section "Starting Protocol Server"
    print_info "Port: $port"
    print_info "Transport: $transport"

    print_warning "Server functionality requires Node.js implementation"
    print_info "To start a server:"
    print_info "  1. Install @wia/core-007: npm install @wia/core-007"
    print_info "  2. Create server script:"

    cat <<'EOF'

const { createRPCServer } = require('@wia/core-007');

const server = createRPCServer({
  port: 8080,
  transport: 'http'
});

server.register('echo', async (payload) => {
  return { echo: payload };
});

server.start();
console.log('Server started on port 8080');

EOF

    echo ""
}

# Validate message
validate_message() {
    local message_file="$1"

    print_section "Validating Message"
    print_info "File: $message_file"

    if [ ! -f "$message_file" ]; then
        print_error "File not found: $message_file"
        exit 1
    fi

    # Read and parse JSON
    if ! jq empty "$message_file" 2>/dev/null; then
        print_error "Invalid JSON format"
        exit 1
    fi

    # Check required fields
    local has_id=$(jq -e '.id' "$message_file" &>/dev/null && echo "yes" || echo "no")
    local has_version=$(jq -e '.version' "$message_file" &>/dev/null && echo "yes" || echo "no")
    local has_type=$(jq -e '.type' "$message_file" &>/dev/null && echo "yes" || echo "no")

    print_section "Validation Results"

    if [ "$has_id" == "yes" ]; then
        print_success "ID field: present"
    else
        print_error "ID field: missing"
    fi

    if [ "$has_version" == "yes" ]; then
        print_success "Version field: present"
    else
        print_error "Version field: missing"
    fi

    if [ "$has_type" == "yes" ]; then
        local type=$(jq -r '.type' "$message_file")
        if [[ "$type" == "request" || "$type" == "response" || "$type" == "event" || "$type" == "stream" ]]; then
            print_success "Type field: valid ($type)"
        else
            print_error "Type field: invalid value ($type)"
        fi
    else
        print_error "Type field: missing"
    fi

    # Show full message
    print_section "Message Content"
    jq '.' "$message_file"

    if [ "$has_id" == "yes" ] && [ "$has_version" == "yes" ] && [ "$has_type" == "yes" ]; then
        echo ""
        print_success "Message is valid"
    else
        echo ""
        print_error "Message is invalid"
        exit 1
    fi

    echo ""
}

# Create message template
create_template() {
    local type="${1:-request}"
    local output="${2:-message.json}"

    print_section "Creating Message Template"
    print_info "Type: $type"
    print_info "Output: $output"

    local msg_id=$(generate_id "$type")
    local timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    case $type in
        request)
            cat > "$output" <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "request",
  "method": "service.method",
  "headers": {
    "content-type": "application/json"
  },
  "payload": {
    "key": "value"
  },
  "metadata": {
    "timestamp": "$timestamp",
    "source": "client"
  }
}
EOF
            ;;
        response)
            cat > "$output" <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "response",
  "status": "success",
  "payload": {
    "result": "value"
  },
  "metadata": {
    "timestamp": "$timestamp"
  }
}
EOF
            ;;
        event)
            cat > "$output" <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "event",
  "method": "event.name",
  "payload": {
    "data": "value"
  },
  "metadata": {
    "timestamp": "$timestamp"
  }
}
EOF
            ;;
        stream)
            cat > "$output" <<EOF
{
  "id": "$msg_id",
  "version": "1.0.0",
  "type": "stream",
  "method": "stream.data",
  "payload": {
    "chunk": "data"
  },
  "metadata": {
    "timestamp": "$timestamp",
    "sequence": 1,
    "final": false
  }
}
EOF
            ;;
        *)
            print_error "Unknown message type: $type"
            exit 1
            ;;
    esac

    print_success "Template created: $output"
    cat "$output" | jq '.'

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-core-007 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  call <method> <payload>      Send RPC request"
    echo "    --endpoint <url>           Server endpoint (default: http://localhost:8080)"
    echo "    --headers <headers>        Custom HTTP headers"
    echo ""
    echo "  subscribe <channel>          Subscribe to event channel"
    echo "    --endpoint <url>           WebSocket endpoint (default: ws://localhost:8080)"
    echo ""
    echo "  send                         Send custom message"
    echo "    --type <type>              Message type (request|response|event|stream)"
    echo "    --method <method>          Method name"
    echo "    --payload <json>           Message payload"
    echo "    --endpoint <url>           Server endpoint"
    echo ""
    echo "  server                       Start protocol server"
    echo "    --port <port>              Server port (default: 8080)"
    echo "    --transport <type>         Transport type (default: http)"
    echo ""
    echo "  validate <file>              Validate message format"
    echo ""
    echo "  template                     Create message template"
    echo "    --type <type>              Message type (default: request)"
    echo "    --output <file>            Output file (default: message.json)"
    echo ""
    echo "  version                      Show version information"
    echo "  help                         Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-core-007 call user.getProfile '{\"userId\":\"12345\"}'"
    echo "  wia-core-007 subscribe notifications --endpoint ws://localhost:8080"
    echo "  wia-core-007 send --type event --method user.created --payload '{\"id\":\"123\"}'"
    echo "  wia-core-007 server --port 8080 --transport http"
    echo "  wia-core-007 validate message.json"
    echo "  wia-core-007 template --type request --output req.json"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-CORE-007 CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    call)
        METHOD="$1"
        PAYLOAD="$2"
        ENDPOINT="http://localhost:8080"
        HEADERS=""

        shift 2 || true

        while [[ $# -gt 0 ]]; do
            case $1 in
                --endpoint) ENDPOINT=$2; shift 2 ;;
                --headers) HEADERS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        rpc_call "$METHOD" "$PAYLOAD" "$ENDPOINT" "$HEADERS"
        ;;

    subscribe)
        CHANNEL="$1"
        ENDPOINT="ws://localhost:8080"

        shift || true

        while [[ $# -gt 0 ]]; do
            case $1 in
                --endpoint) ENDPOINT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        subscribe_events "$CHANNEL" "$ENDPOINT"
        ;;

    send)
        TYPE="request"
        METHOD=""
        PAYLOAD="{}"
        ENDPOINT="http://localhost:8080"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --method) METHOD=$2; shift 2 ;;
                --payload) PAYLOAD=$2; shift 2 ;;
                --endpoint) ENDPOINT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        send_message "$TYPE" "$METHOD" "$PAYLOAD" "$ENDPOINT"
        ;;

    server)
        PORT=8080
        TRANSPORT="http"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --port) PORT=$2; shift 2 ;;
                --transport) TRANSPORT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        start_server "$PORT" "$TRANSPORT"
        ;;

    validate)
        FILE="$1"

        if [ -z "$FILE" ]; then
            echo -e "${RED}Error: File path required${RESET}"
            echo "Usage: wia-core-007 validate <file>"
            exit 1
        fi

        print_header
        validate_message "$FILE"
        ;;

    template)
        TYPE="request"
        OUTPUT="message.json"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        create_template "$TYPE" "$OUTPUT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-core-007 help' for usage information"
        exit 1
        ;;
esac

exit 0
