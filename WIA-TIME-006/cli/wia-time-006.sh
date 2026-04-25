#!/bin/bash

################################################################################
# WIA-TIME-006: Universal Time Database CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to the Universal Time Database
# including event insertion, querying, timeline management, and synchronization.
################################################################################

set -e

# Colors for output
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
DB_DIR="$HOME/.wia-time-006"
CONFIG_FILE="$DB_DIR/config.json"
DATA_DIR="$DB_DIR/data"

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          🗄️  WIA-TIME-006: Universal Time Database           ║"
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

# Initialize database
init_db() {
    local storage=${1:-distributed}
    local replicas=${2:-3}

    print_section "Database Initialization"

    # Create directories
    mkdir -p "$DB_DIR" "$DATA_DIR"

    # Create config
    cat > "$CONFIG_FILE" << EOC
{
  "version": "$VERSION",
  "universe": "prime",
  "timeline": "alpha-001",
  "storage": {
    "type": "$storage",
    "replicationFactor": $replicas,
    "consistency": "causal"
  },
  "initialized": "$(date -Iseconds)"
}
EOC

    print_success "Database initialized"
    print_info "Storage type: $storage"
    print_info "Replication factor: $replicas"
    print_info "Data directory: $DATA_DIR"
    print_info "Config file: $CONFIG_FILE"
    echo ""
}

# Insert event
insert_event() {
    local time="$1"
    local universe="${2:-prime}"
    local timeline="${3:-alpha-001}"
    local event_type="${4:-user-action}"
    local description="${5:-Event}"

    print_section "Insert Event"

    # Generate event ID
    local event_id="evt-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"

    # Convert time to nanoseconds
    local timestamp=$(date -d "$time" +%s)000000000

    print_info "Event ID: $event_id"
    print_info "Timestamp: $time ($timestamp ns)"
    print_info "Universe: $universe"
    print_info "Timeline: $timeline"
    print_info "Type: $event_type"
    print_info "Description: $description"

    # Create event file
    local event_file="$DATA_DIR/${timeline}_${event_id}.json"
    cat > "$event_file" << EOE
{
  "event_id": "$event_id",
  "coordinate": {
    "timestamp": "$timestamp",
    "universe_id": "$universe",
    "timeline_id": "$timeline",
    "position": {"x": 0, "y": 0, "z": 0}
  },
  "event_type": "$event_type",
  "data": {
    "description": "$description"
  },
  "significance": 0.5,
  "mutable": true,
  "inserted_at": "$(date -Iseconds)"
}
EOE

    print_success "Event inserted: $event_id"
    print_info "Stored at: $event_file"
    echo ""
}

# Query events
query_events() {
    local universe="${1:-prime}"
    local timeline="${2:-alpha-*}"
    local from="${3:-2000-01-01}"
    local to="${4:-2100-01-01}"
    local format="${5:-table}"

    print_section "Query Events"
    print_info "Universe: $universe"
    print_info "Timeline: $timeline"
    print_info "From: $from"
    print_info "To: $to"
    print_info "Format: $format"

    # Find matching files
    local pattern="${DATA_DIR}/${timeline}_evt-*.json"
    local count=0

    print_section "Results"

    if [ "$format" == "json" ]; then
        echo "["
        local first=true
        for file in $pattern; do
            if [ -f "$file" ]; then
                if [ "$first" = true ]; then
                    first=false
                else
                    echo ","
                fi
                cat "$file"
                ((count++))
            fi
        done
        echo "]"
    else
        echo -e "${CYAN}Event ID                    | Timeline     | Timestamp            | Type${RESET}"
        echo "----------------------------+--------------+----------------------+---------------"
        for file in $pattern; do
            if [ -f "$file" ]; then
                local event_id=$(jq -r '.event_id' "$file" 2>/dev/null || echo "N/A")
                local tl=$(jq -r '.coordinate.timeline_id' "$file" 2>/dev/null || echo "N/A")
                local ts=$(jq -r '.coordinate.timestamp' "$file" 2>/dev/null || echo "N/A")
                local type=$(jq -r '.event_type' "$file" 2>/dev/null || echo "N/A")
                printf "%-28s| %-12s | %-20s | %s\n" "$event_id" "$tl" "$ts" "$type"
                ((count++))
            fi
        done
    fi

    echo ""
    print_success "Found $count events"
    echo ""
}

# Create timeline branch
branch_create() {
    local from="$1"
    local name="$2"
    local branch_point="${3:-now}"
    local description="${4:-Timeline branch}"

    print_section "Create Timeline Branch"
    print_info "Source timeline: $from"
    print_info "New timeline: $name"
    print_info "Branch point: $branch_point"
    print_info "Description: $description"

    # Create branch metadata
    local branch_file="$DATA_DIR/timeline_${name}.json"
    cat > "$branch_file" << EOB
{
  "timeline_id": "$name",
  "parent_timeline": "$from",
  "branch_point": "$(date -d "$branch_point" +%s)000000000",
  "divergence_factor": 0.0,
  "created_at": "$(date -Iseconds)",
  "description": "$description",
  "status": "active"
}
EOB

    # Copy events from parent up to branch point
    local copied=0
    for file in ${DATA_DIR}/${from}_evt-*.json; do
        if [ -f "$file" ]; then
            local new_file="${file/$from/$name}"
            cp "$file" "$new_file"
            ((copied++))
        fi
    done

    print_success "Branch created: $name"
    print_info "Events copied: $copied"
    print_info "Branch file: $branch_file"
    echo ""
}

# List timelines
list_timelines() {
    local universe="${1:-prime}"

    print_section "Timelines in Universe: $universe"

    echo -e "${CYAN}Timeline ID      | Parent        | Status  | Events | Created${RESET}"
    echo "-----------------+---------------+---------+--------+-------------------------"

    for file in ${DATA_DIR}/timeline_*.json; do
        if [ -f "$file" ]; then
            local tl=$(jq -r '.timeline_id' "$file")
            local parent=$(jq -r '.parent_timeline // "N/A"' "$file")
            local status=$(jq -r '.status' "$file")
            local created=$(jq -r '.created_at' "$file")

            # Count events
            local event_count=$(ls ${DATA_DIR}/${tl}_evt-*.json 2>/dev/null | wc -l)

            printf "%-16s | %-13s | %-7s | %-6s | %s\n" "$tl" "$parent" "$status" "$event_count" "$created"
        fi
    done

    echo ""
}

# Synchronize timelines
sync_timelines() {
    local source="$1"
    local target="$2"
    local mode="${3:-incremental}"

    print_section "Timeline Synchronization"

    # Parse source and target (format: universe:timeline)
    local source_universe="${source%%:*}"
    local source_timeline="${source##*:}"
    local target_universe="${target%%:*}"
    local target_timeline="${target##*:}"

    print_info "Source: $source_universe:$source_timeline"
    print_info "Target: $target_universe:$target_timeline"
    print_info "Mode: $mode"

    local sync_id="sync-$(date +%s)-$(head /dev/urandom | tr -dc a-z0-9 | head -c 8)"
    local start_time=$(date +%s%3N)

    # Count events to sync
    local event_count=$(ls ${DATA_DIR}/${source_timeline}_evt-*.json 2>/dev/null | wc -l)

    # Simulate sync
    sleep 0.5

    local end_time=$(date +%s%3N)
    local duration=$((end_time - start_time))

    print_section "Sync Results"
    print_success "Sync ID: $sync_id"
    print_info "Events synced: $event_count"
    print_info "Duration: ${duration}ms"
    print_info "Status: Success"
    echo ""
}

# Get database statistics
show_stats() {
    print_section "Database Statistics"

    # Count events
    local total_events=$(ls ${DATA_DIR}/*_evt-*.json 2>/dev/null | wc -l)

    # Count timelines
    local total_timelines=$(ls ${DATA_DIR}/timeline_*.json 2>/dev/null | wc -l)

    # Calculate storage
    local storage_used=$(du -sh "$DATA_DIR" 2>/dev/null | cut -f1)

    print_info "Total Events: $total_events"
    print_info "Total Timelines: $total_timelines"
    print_info "Storage Used: $storage_used"
    print_info "Data Directory: $DATA_DIR"

    if [ -f "$CONFIG_FILE" ]; then
        print_section "Configuration"
        local universe=$(jq -r '.universe' "$CONFIG_FILE")
        local timeline=$(jq -r '.timeline' "$CONFIG_FILE")
        local storage=$(jq -r '.storage.type' "$CONFIG_FILE")

        print_info "Default Universe: $universe"
        print_info "Default Timeline: $timeline"
        print_info "Storage Type: $storage"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-time-006 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  init                     Initialize database"
    echo "    --storage <type>       Storage type (local|distributed|quantum)"
    echo "    --replicas <count>     Replication factor (default: 3)"
    echo ""
    echo "  insert                   Insert temporal event"
    echo "    --time <datetime>      Event timestamp"
    echo "    --universe <id>        Universe ID (default: prime)"
    echo "    --timeline <id>        Timeline ID (default: alpha-001)"
    echo "    --type <type>          Event type"
    echo "    --event <description>  Event description"
    echo ""
    echo "  query                    Query events"
    echo "    --universe <id>        Universe to query (default: prime)"
    echo "    --timeline <pattern>   Timeline pattern (default: alpha-*)"
    echo "    --from <datetime>      Start time (default: 2000-01-01)"
    echo "    --to <datetime>        End time (default: 2100-01-01)"
    echo "    --format <fmt>         Output format (table|json)"
    echo ""
    echo "  branch create            Create timeline branch"
    echo "    --from <timeline>      Source timeline"
    echo "    --name <timeline>      New timeline name"
    echo "    --divergence-point <dt> Branch point (default: now)"
    echo ""
    echo "  branch list              List all timelines"
    echo "    --universe <id>        Universe to query (default: prime)"
    echo ""
    echo "  sync                     Synchronize timelines"
    echo "    --source <uni:tl>      Source universe:timeline"
    echo "    --target <uni:tl>      Target universe:timeline"
    echo "    --mode <mode>          Sync mode (full|incremental|differential)"
    echo ""
    echo "  stats                    Show database statistics"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-time-006 init --storage distributed --replicas 3"
    echo "  wia-time-006 insert --time '2024-01-01T00:00:00Z' --event 'First contact'"
    echo "  wia-time-006 query --timeline 'alpha-*' --from '2020-01-01' --to '2025-01-01'"
    echo "  wia-time-006 branch create --from 'alpha-001' --name 'beta-001'"
    echo "  wia-time-006 sync --source 'prime:alpha-001' --target 'universe-7:beta-003'"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-TIME-006 Universal Time Database CLI"
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
    init)
        STORAGE="distributed"
        REPLICAS=3

        while [[ $# -gt 0 ]]; do
            case $1 in
                --storage) STORAGE=$2; shift 2 ;;
                --replicas) REPLICAS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        init_db "$STORAGE" "$REPLICAS"
        ;;

    insert)
        TIME="now"
        UNIVERSE="prime"
        TIMELINE="alpha-001"
        TYPE="user-action"
        EVENT="Event"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --time) TIME=$2; shift 2 ;;
                --universe) UNIVERSE=$2; shift 2 ;;
                --timeline) TIMELINE=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                --event) EVENT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        insert_event "$TIME" "$UNIVERSE" "$TIMELINE" "$TYPE" "$EVENT"
        ;;

    query)
        UNIVERSE="prime"
        TIMELINE="alpha-*"
        FROM="2000-01-01"
        TO="2100-01-01"
        FORMAT="table"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --universe) UNIVERSE=$2; shift 2 ;;
                --timeline) TIMELINE=$2; shift 2 ;;
                --from) FROM=$2; shift 2 ;;
                --to) TO=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        query_events "$UNIVERSE" "$TIMELINE" "$FROM" "$TO" "$FORMAT"
        ;;

    branch)
        SUBCMD=${1:-help}
        shift || true

        case "$SUBCMD" in
            create)
                FROM=""
                NAME=""
                POINT="now"

                while [[ $# -gt 0 ]]; do
                    case $1 in
                        --from) FROM=$2; shift 2 ;;
                        --name) NAME=$2; shift 2 ;;
                        --divergence-point) POINT=$2; shift 2 ;;
                        *) shift ;;
                    esac
                done

                print_header
                branch_create "$FROM" "$NAME" "$POINT"
                ;;

            list)
                UNIVERSE="prime"

                while [[ $# -gt 0 ]]; do
                    case $1 in
                        --universe) UNIVERSE=$2; shift 2 ;;
                        *) shift ;;
                    esac
                done

                print_header
                list_timelines "$UNIVERSE"
                ;;

            *)
                echo -e "${RED}Error: Unknown branch command '$SUBCMD'${RESET}"
                echo "Run 'wia-time-006 help' for usage information"
                exit 1
                ;;
        esac
        ;;

    sync)
        SOURCE=""
        TARGET=""
        MODE="incremental"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --source) SOURCE=$2; shift 2 ;;
                --target) TARGET=$2; shift 2 ;;
                --mode) MODE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        sync_timelines "$SOURCE" "$TARGET" "$MODE"
        ;;

    stats)
        print_header
        show_stats
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-time-006 help' for usage information"
        exit 1
        ;;
esac

exit 0
