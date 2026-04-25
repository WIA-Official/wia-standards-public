#!/bin/bash

################################################################################
# WIA-TIME-014: Data Time Transport CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to data time transport features
# including time capsule management, temporal messaging, and data encoding.
################################################################################

set -e

# Colors for output
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
CONFIG_DIR="$HOME/.wia/time-014"
CAPSULES_DIR="$CONFIG_DIR/capsules"
MESSAGES_DIR="$CONFIG_DIR/messages"
DATA_DIR="$CONFIG_DIR/data"

# Create directories if they don't exist
mkdir -p "$CONFIG_DIR" "$CAPSULES_DIR" "$MESSAGES_DIR" "$DATA_DIR"

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       💾 WIA-TIME-014: Data Time Transport CLI               ║"
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
    echo -e "${BLUE}ℹ $1${RESET}"
}

generate_uuid() {
    if command -v uuidgen &> /dev/null; then
        uuidgen | tr '[:upper:]' '[:lower:]'
    else
        cat /proc/sys/kernel/random/uuid
    fi
}

# Time Capsule Commands

cmd_create_capsule() {
    local data_file="$1"
    local delivery_time="$2"
    local title="${3:-Untitled Capsule}"

    if [[ -z "$data_file" || -z "$delivery_time" ]]; then
        print_error "Usage: create-capsule <data-file> <delivery-time> [title]"
        echo "  Example: create-capsule data.json '2030-01-01T00:00:00Z' 'My Time Capsule'"
        exit 1
    fi

    if [[ ! -f "$data_file" ]]; then
        print_error "File not found: $data_file"
        exit 1
    fi

    print_section "Creating Time Capsule"

    local capsule_id=$(generate_uuid)
    local created=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    # Create capsule metadata
    cat > "$CAPSULES_DIR/$capsule_id.json" <<EOF
{
  "id": "$capsule_id",
  "version": "1.0.0",
  "created": "$created",
  "deliveryTime": "$delivery_time",
  "timeline": "TL-01-PRIME-ALPHA-0000000001",
  "state": "created",
  "metadata": {
    "title": "$title",
    "size": $(stat -f%z "$data_file" 2>/dev/null || stat -c%s "$data_file"),
    "sourceFile": "$data_file"
  }
}
EOF

    # Copy data file
    cp "$data_file" "$CAPSULES_DIR/$capsule_id.data"

    print_success "Time capsule created successfully"
    echo -e "  ${GRAY}Capsule ID:${RESET}    $capsule_id"
    echo -e "  ${GRAY}Title:${RESET}         $title"
    echo -e "  ${GRAY}Delivery:${RESET}      $delivery_time"
    echo -e "  ${GRAY}Timeline:${RESET}      TL-01-PRIME-ALPHA-0000000001"
    echo -e "  ${GRAY}State:${RESET}         created"

    print_info "Capsule stored in: $CAPSULES_DIR/$capsule_id.json"
}

cmd_list_capsules() {
    local state_filter="$1"

    print_section "Time Capsules"

    local count=0
    for capsule_file in "$CAPSULES_DIR"/*.json; do
        if [[ ! -f "$capsule_file" ]]; then
            continue
        fi

        local capsule_id=$(basename "$capsule_file" .json)
        local title=$(jq -r '.metadata.title' "$capsule_file")
        local delivery=$(jq -r '.deliveryTime' "$capsule_file")
        local state=$(jq -r '.state' "$capsule_file")
        local size=$(jq -r '.metadata.size' "$capsule_file")

        # Apply state filter if provided
        if [[ -n "$state_filter" && "$state" != "$state_filter" ]]; then
            continue
        fi

        # Determine if capsule is ready
        local current_time=$(date -u +%s)
        local delivery_timestamp=$(date -d "$delivery" +%s 2>/dev/null || date -j -f "%Y-%m-%dT%H:%M:%SZ" "$delivery" +%s)

        if [[ $current_time -ge $delivery_timestamp ]]; then
            state="ready"
        fi

        echo -e "\n${VIOLET}📦 Capsule: $capsule_id${RESET}"
        echo -e "  ${GRAY}Title:${RESET}         $title"
        echo -e "  ${GRAY}Delivery:${RESET}      $delivery"
        echo -e "  ${GRAY}State:${RESET}         $state"
        echo -e "  ${GRAY}Size:${RESET}          $(numfmt --to=iec-i --suffix=B $size 2>/dev/null || echo "$size bytes")"

        ((count++))
    done

    if [[ $count -eq 0 ]]; then
        print_info "No time capsules found"
    else
        echo ""
        print_success "Total: $count capsule(s)"
    fi
}

cmd_open_capsule() {
    local capsule_id="$1"
    local output_file="${2:-}"

    if [[ -z "$capsule_id" ]]; then
        print_error "Usage: open-capsule <capsule-id> [output-file]"
        exit 1
    fi

    local capsule_file="$CAPSULES_DIR/$capsule_id.json"
    local data_file="$CAPSULES_DIR/$capsule_id.data"

    if [[ ! -f "$capsule_file" ]]; then
        print_error "Capsule not found: $capsule_id"
        exit 1
    fi

    print_section "Opening Time Capsule"

    # Check delivery time
    local delivery=$(jq -r '.deliveryTime' "$capsule_file")
    local current_time=$(date -u +%s)
    local delivery_timestamp=$(date -d "$delivery" +%s 2>/dev/null || date -j -f "%Y-%m-%dT%H:%M:%SZ" "$delivery" +%s)

    if [[ $current_time -lt $delivery_timestamp ]]; then
        print_warning "Capsule is time-locked until: $delivery"
        local time_remaining=$((delivery_timestamp - current_time))
        local days=$((time_remaining / 86400))
        local hours=$(((time_remaining % 86400) / 3600))
        echo -e "  ${GRAY}Time remaining:${RESET} $days days, $hours hours"
        exit 1
    fi

    # Extract data
    if [[ -n "$output_file" ]]; then
        cp "$data_file" "$output_file"
        print_success "Capsule data extracted to: $output_file"
    else
        cat "$data_file"
    fi

    # Update state
    jq '.state = "opened"' "$capsule_file" > "$capsule_file.tmp" && mv "$capsule_file.tmp" "$capsule_file"

    print_success "Time capsule opened successfully"
}

cmd_delete_capsule() {
    local capsule_id="$1"

    if [[ -z "$capsule_id" ]]; then
        print_error "Usage: delete-capsule <capsule-id>"
        exit 1
    fi

    local capsule_file="$CAPSULES_DIR/$capsule_id.json"
    local data_file="$CAPSULES_DIR/$capsule_id.data"

    if [[ ! -f "$capsule_file" ]]; then
        print_error "Capsule not found: $capsule_id"
        exit 1
    fi

    print_section "Deleting Time Capsule"

    # Show capsule info before deletion
    local title=$(jq -r '.metadata.title' "$capsule_file")
    echo -e "  ${GRAY}Capsule ID:${RESET} $capsule_id"
    echo -e "  ${GRAY}Title:${RESET}      $title"

    # Confirm deletion
    read -p "Are you sure you want to delete this capsule? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Deletion cancelled"
        exit 0
    fi

    # Delete files
    rm -f "$capsule_file" "$data_file"

    print_success "Time capsule deleted successfully"
}

# Messaging Commands

cmd_send_message() {
    local message="$1"
    local target_time="$2"
    local target_timeline="${3:-TL-01-PRIME-ALPHA-0000000001}"

    if [[ -z "$message" || -z "$target_time" ]]; then
        print_error "Usage: send <message> <target-time> [target-timeline]"
        echo "  Example: send 'Hello future!' '2026-01-01T00:00:00Z'"
        exit 1
    fi

    print_section "Sending Temporal Message"

    local message_id=$(generate_uuid)
    local sent_time=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    # Create message
    cat > "$MESSAGES_DIR/$message_id.json" <<EOF
{
  "id": "$message_id",
  "protocol": "TMP/1.0",
  "from": {
    "timeline": "TL-01-PRIME-ALPHA-0000000001",
    "time": "$sent_time",
    "sender": "$(whoami)"
  },
  "to": {
    "timeline": "$target_timeline",
    "time": "$target_time",
    "recipient": "broadcast"
  },
  "content": {
    "type": "text/plain",
    "data": "$(echo -n "$message" | base64)",
    "size": ${#message}
  },
  "priority": "normal",
  "status": "sent"
}
EOF

    print_success "Message sent successfully"
    echo -e "  ${GRAY}Message ID:${RESET}       $message_id"
    echo -e "  ${GRAY}Target Timeline:${RESET}  $target_timeline"
    echo -e "  ${GRAY}Target Time:${RESET}      $target_time"
    echo -e "  ${GRAY}Size:${RESET}             ${#message} bytes"

    print_info "Message stored in: $MESSAGES_DIR/$message_id.json"
}

cmd_receive_messages() {
    local limit="${1:-10}"

    print_section "Temporal Messages"

    local count=0
    for message_file in "$MESSAGES_DIR"/*.json; do
        if [[ ! -f "$message_file" ]]; then
            continue
        fi

        if [[ $count -ge $limit ]]; then
            break
        fi

        local message_id=$(basename "$message_file" .json)
        local sender=$(jq -r '.from.sender' "$message_file")
        local time=$(jq -r '.from.time' "$message_file")
        local content_b64=$(jq -r '.content.data' "$message_file")
        local content=$(echo "$content_b64" | base64 -d)
        local priority=$(jq -r '.priority' "$message_file")

        echo -e "\n${VIOLET}📨 Message: $message_id${RESET}"
        echo -e "  ${GRAY}From:${RESET}      $sender"
        echo -e "  ${GRAY}Time:${RESET}      $time"
        echo -e "  ${GRAY}Priority:${RESET}  $priority"
        echo -e "  ${GRAY}Content:${RESET}   $content"

        ((count++))
    done

    if [[ $count -eq 0 ]]; then
        print_info "No messages found"
    else
        echo ""
        print_success "Showing $count message(s)"
    fi
}

# Data Encoding Commands

cmd_encode() {
    local input_file="$1"
    local output_file="${2:-$input_file.tdp}"
    local compression="${3:-zstd}"

    if [[ -z "$input_file" ]]; then
        print_error "Usage: encode <input-file> [output-file] [compression]"
        echo "  Compression: none, lz4, zstd, brotli"
        exit 1
    fi

    if [[ ! -f "$input_file" ]]; then
        print_error "File not found: $input_file"
        exit 1
    fi

    print_section "Encoding Temporal Data"

    local input_size=$(stat -f%z "$input_file" 2>/dev/null || stat -c%s "$input_file")

    # Create TDP header
    local tdp_id=$(generate_uuid)
    local created=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    # Calculate hash
    local hash=$(sha512sum "$input_file" | cut -d' ' -f1)

    # Create metadata file
    cat > "$output_file.meta" <<EOF
{
  "id": "$tdp_id",
  "version": "1.0.0",
  "created": "$created",
  "compression": "$compression",
  "originalSize": $input_size,
  "integrity": {
    "algorithm": "sha512",
    "hash": "$hash"
  }
}
EOF

    # Copy data (in real implementation, would apply compression)
    cp "$input_file" "$output_file"

    local output_size=$(stat -f%z "$output_file" 2>/dev/null || stat -c%s "$output_file")
    local ratio=$(awk "BEGIN {printf \"%.2f\", $input_size / $output_size}")

    print_success "Data encoded successfully"
    echo -e "  ${GRAY}TDP ID:${RESET}          $tdp_id"
    echo -e "  ${GRAY}Input Size:${RESET}      $(numfmt --to=iec-i --suffix=B $input_size 2>/dev/null || echo "$input_size bytes")"
    echo -e "  ${GRAY}Output Size:${RESET}     $(numfmt --to=iec-i --suffix=B $output_size 2>/dev/null || echo "$output_size bytes")"
    echo -e "  ${GRAY}Compression:${RESET}     $compression (${ratio}x)"
    echo -e "  ${GRAY}Hash:${RESET}            ${hash:0:16}..."

    print_info "Encoded file: $output_file"
    print_info "Metadata: $output_file.meta"
}

cmd_decode() {
    local input_file="$1"
    local output_file="${2:-${input_file%.tdp}}"

    if [[ -z "$input_file" ]]; then
        print_error "Usage: decode <input-file> [output-file]"
        exit 1
    fi

    if [[ ! -f "$input_file" ]]; then
        print_error "File not found: $input_file"
        exit 1
    fi

    if [[ ! -f "$input_file.meta" ]]; then
        print_error "Metadata file not found: $input_file.meta"
        exit 1
    fi

    print_section "Decoding Temporal Data"

    # Read metadata
    local tdp_id=$(jq -r '.id' "$input_file.meta")
    local compression=$(jq -r '.compression' "$input_file.meta")
    local original_hash=$(jq -r '.integrity.hash' "$input_file.meta")

    # Copy data (in real implementation, would apply decompression)
    cp "$input_file" "$output_file"

    # Verify hash
    local current_hash=$(sha512sum "$output_file" | cut -d' ' -f1)

    if [[ "$current_hash" == "$original_hash" ]]; then
        print_success "Data decoded successfully"
        print_success "Integrity verified ✓"
    else
        print_warning "Data decoded but integrity check failed"
        echo -e "  ${GRAY}Expected:${RESET} ${original_hash:0:16}..."
        echo -e "  ${GRAY}Got:${RESET}      ${current_hash:0:16}..."
    fi

    echo -e "  ${GRAY}TDP ID:${RESET}       $tdp_id"
    echo -e "  ${GRAY}Compression:${RESET}  $compression"

    print_info "Decoded file: $output_file"
}

cmd_validate() {
    local input_file="$1"

    if [[ -z "$input_file" ]]; then
        print_error "Usage: validate <file>"
        exit 1
    fi

    if [[ ! -f "$input_file" ]]; then
        print_error "File not found: $input_file"
        exit 1
    fi

    print_section "Validating Data Integrity"

    if [[ -f "$input_file.meta" ]]; then
        # TDP file
        local original_hash=$(jq -r '.integrity.hash' "$input_file.meta")
        local current_hash=$(sha512sum "$input_file" | cut -d' ' -f1)

        if [[ "$current_hash" == "$original_hash" ]]; then
            print_success "Integrity check passed ✓"
            echo -e "  ${GRAY}Hash:${RESET} ${current_hash:0:32}..."
        else
            print_error "Integrity check failed ✗"
            echo -e "  ${GRAY}Expected:${RESET} ${original_hash:0:32}..."
            echo -e "  ${GRAY}Got:${RESET}      ${current_hash:0:32}..."
            exit 1
        fi
    else
        # Regular file
        local hash=$(sha512sum "$input_file" | cut -d' ' -f1)
        print_success "File hash calculated"
        echo -e "  ${GRAY}SHA-512:${RESET} $hash"
    fi
}

# Bandwidth Commands

cmd_bandwidth() {
    local action="${1:-status}"

    print_section "Temporal Bandwidth"

    if [[ "$action" == "status" || "$action" == "monitor" ]]; then
        # Simulated bandwidth status
        echo -e "${GRAY}Utilization:${RESET}    45%"
        echo -e "${GRAY}Available:${RESET}      500 MB/s²"
        echo -e "${GRAY}Capacity:${RESET}       1 GB/s²"
        echo -e "${GRAY}Queue Size:${RESET}     10 MB"
        echo -e "${GRAY}Delay:${RESET}          5 seconds"
        echo -e "${GRAY}Congestion:${RESET}     ${GREEN}Low${RESET}"

        echo -e "\n${CYAN}Priority Allocation:${RESET}"
        echo -e "  ${GRAY}Critical:${RESET}     1000 MB/s²"
        echo -e "  ${GRAY}High:${RESET}         750 MB/s²"
        echo -e "  ${GRAY}Normal:${RESET}       500 MB/s²"
        echo -e "  ${GRAY}Low:${RESET}          250 MB/s²"
        echo -e "  ${GRAY}Background:${RESET}   100 MB/s²"
    fi
}

cmd_reserve_bandwidth() {
    local amount="$1"
    local duration="$2"

    if [[ -z "$amount" || -z "$duration" ]]; then
        print_error "Usage: reserve <amount-in-MB> <duration-in-seconds>"
        exit 1
    fi

    print_section "Reserving Bandwidth"

    local reservation_id=$(generate_uuid)
    local start_time=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    local end_time=$(date -u -d "+${duration} seconds" +"%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u -v+${duration}S +"%Y-%m-%dT%H:%M:%SZ")

    print_success "Bandwidth reserved successfully"
    echo -e "  ${GRAY}Reservation ID:${RESET} $reservation_id"
    echo -e "  ${GRAY}Amount:${RESET}         $amount MB/s²"
    echo -e "  ${GRAY}Duration:${RESET}       $duration seconds"
    echo -e "  ${GRAY}Start:${RESET}          $start_time"
    echo -e "  ${GRAY}End:${RESET}            $end_time"
}

# Timeline Commands

cmd_timeline_info() {
    local timeline_id="${1:-TL-01-PRIME-ALPHA-0000000001}"

    print_section "Timeline Information"

    echo -e "${GRAY}Timeline ID:${RESET}  $timeline_id"
    echo -e "${GRAY}Type:${RESET}         PRIME"
    echo -e "${GRAY}Branch:${RESET}       ALPHA"
    echo -e "${GRAY}Sequence:${RESET}     1"
    echo -e "${GRAY}Status:${RESET}       ${GREEN}Active${RESET}"
    echo -e "${GRAY}Current Time:${RESET} $(date -u +"%Y-%m-%dT%H:%M:%SZ")"

    echo -e "\n${CYAN}Synchronization:${RESET}"
    echo -e "  ${GRAY}Last Sync:${RESET}    $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
    echo -e "  ${GRAY}Offset:${RESET}       0.5 ms"
    echo -e "  ${GRAY}Drift:${RESET}        0.001 ms/day"
}

cmd_timeline_list() {
    print_section "Available Timelines"

    echo -e "\n${VIOLET}1. TL-01-PRIME-ALPHA-0000000001${RESET}"
    echo -e "   ${GRAY}Type:${RESET}   PRIME"
    echo -e "   ${GRAY}Status:${RESET} ${GREEN}Active${RESET}"

    echo -e "\n${VIOLET}2. TL-01-BRANCH-BETA-0000000042${RESET}"
    echo -e "   ${GRAY}Type:${RESET}   BRANCH"
    echo -e "   ${GRAY}Status:${RESET} ${GREEN}Active${RESET}"

    print_success "Total: 2 timeline(s)"
}

cmd_timeline_sync() {
    local timeline_id="${1:-TL-01-PRIME-ALPHA-0000000001}"

    print_section "Synchronizing Timeline"

    echo -e "${GRAY}Timeline:${RESET} $timeline_id"
    echo -n "Syncing..."
    sleep 1
    echo " done"

    print_success "Timeline synchronized successfully"
    echo -e "  ${GRAY}Offset:${RESET}       0.5 ms"
    echo -e "  ${GRAY}Delay:${RESET}        100 ms"
    echo -e "  ${GRAY}Drift:${RESET}        0.001 ms/day"
    echo -e "  ${GRAY}Next Sync:${RESET}    $(date -u -d "+60 seconds" +"%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u -v+60S +"%Y-%m-%dT%H:%M:%SZ")"
}

# Main command dispatcher

cmd_help() {
    print_header

    cat <<EOF
${CYAN}Time Capsule Commands:${RESET}
  create-capsule <file> <delivery-time> [title]
      Create a new time capsule

  list-capsules [state]
      List all time capsules (filter by state: created, sealed, ready, opened)

  open-capsule <capsule-id> [output-file]
      Open and extract a time capsule

  delete-capsule <capsule-id>
      Delete a time capsule

${CYAN}Messaging Commands:${RESET}
  send <message> <target-time> [timeline]
      Send a temporal message

  receive [limit]
      Receive temporal messages

${CYAN}Data Commands:${RESET}
  encode <input-file> [output-file] [compression]
      Encode data for temporal transport

  decode <input-file> [output-file]
      Decode temporal data packet

  validate <file>
      Validate data integrity

${CYAN}Bandwidth Commands:${RESET}
  bandwidth [status|monitor]
      Check bandwidth status

  reserve <amount-MB> <duration-seconds>
      Reserve temporal bandwidth

${CYAN}Timeline Commands:${RESET}
  timeline-info [timeline-id]
      Get timeline information

  timeline-list
      List available timelines

  timeline-sync [timeline-id]
      Synchronize with timeline

${CYAN}Utility Commands:${RESET}
  version
      Show version information

  help
      Show this help message

  config
      Show configuration

${GRAY}Examples:${RESET}
  wia-time-014 create-capsule data.json '2030-01-01T00:00:00Z' 'Future Archive'
  wia-time-014 send 'Hello future!' '2026-01-01T00:00:00Z'
  wia-time-014 encode myfile.txt myfile.tdp zstd
  wia-time-014 bandwidth status

${GRAY}For more information, visit: https://wiastandards.com/standards/WIA-TIME-014${RESET}
EOF
}

cmd_version() {
    print_header
    echo -e "${GRAY}Version:${RESET}      $VERSION"
    echo -e "${GRAY}Standard:${RESET}     WIA-TIME-014"
    echo -e "${GRAY}Protocol:${RESET}     TMP/1.0, TDP/1.0"
    echo -e "${GRAY}License:${RESET}      MIT"
    echo ""
    echo -e "${VIOLET}弘益人間 (Benefit All Humanity)${RESET}"
}

cmd_config() {
    print_section "Configuration"

    echo -e "${GRAY}Config Dir:${RESET}     $CONFIG_DIR"
    echo -e "${GRAY}Capsules Dir:${RESET}   $CAPSULES_DIR"
    echo -e "${GRAY}Messages Dir:${RESET}   $MESSAGES_DIR"
    echo -e "${GRAY}Data Dir:${RESET}       $DATA_DIR"

    local capsule_count=$(ls -1 "$CAPSULES_DIR"/*.json 2>/dev/null | wc -l)
    local message_count=$(ls -1 "$MESSAGES_DIR"/*.json 2>/dev/null | wc -l)

    echo ""
    echo -e "${GRAY}Capsules:${RESET}       $capsule_count"
    echo -e "${GRAY}Messages:${RESET}       $message_count"
}

# Main entry point

main() {
    local command="${1:-help}"
    shift || true

    case "$command" in
        # Time Capsule
        create-capsule)
            cmd_create_capsule "$@"
            ;;
        list-capsules)
            cmd_list_capsules "$@"
            ;;
        open-capsule)
            cmd_open_capsule "$@"
            ;;
        delete-capsule)
            cmd_delete_capsule "$@"
            ;;

        # Messaging
        send)
            cmd_send_message "$@"
            ;;
        receive)
            cmd_receive_messages "$@"
            ;;

        # Data Encoding
        encode)
            cmd_encode "$@"
            ;;
        decode)
            cmd_decode "$@"
            ;;
        validate)
            cmd_validate "$@"
            ;;

        # Bandwidth
        bandwidth)
            cmd_bandwidth "$@"
            ;;
        reserve)
            cmd_reserve_bandwidth "$@"
            ;;

        # Timeline
        timeline-info)
            cmd_timeline_info "$@"
            ;;
        timeline-list)
            cmd_timeline_list "$@"
            ;;
        timeline-sync)
            cmd_timeline_sync "$@"
            ;;

        # Utility
        version|--version|-v)
            cmd_version
            ;;
        help|--help|-h)
            cmd_help
            ;;
        config)
            cmd_config
            ;;

        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-time-014 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
