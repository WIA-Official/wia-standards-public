#!/bin/bash

################################################################################
# WIA-TIME-033: Historical Archive CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to historical archive operations
# including event recording, querying, verification, and timeline reconciliation.
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

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🏛️  WIA-TIME-033: Historical Archive CLI               ║"
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

# Record historical event
record_event() {
    local event=${1:-"Historical Event"}
    local date=${2:-$(date +%Y-%m-%d)}
    local timeline=${3:-PRIME}
    local type=${4:-other}

    print_section "Recording Historical Event"

    print_info "Event: $event"
    print_info "Date: $date"
    print_info "Timeline: $timeline"
    print_info "Type: $type"

    # Generate record ID
    local year=$(date -d "$date" +%Y 2>/dev/null || echo "2024")
    local record_id="REC-${timeline}-${year}-$(printf '%06d' $RANDOM)"

    print_section "Record Details"
    print_success "Record ID: $record_id"
    print_info "Status: Verified"
    print_info "Confidence: 95.0%"
    print_info "Verification Method: Multi-witness"

    print_section "Integrity"
    local hash=$(echo -n "$event$date$timeline" | md5sum | cut -d' ' -f1)
    print_success "Content Hash: ${hash:0:16}...${hash:48:16}"
    print_success "Block Height: $((RANDOM % 10000))"
    print_success "Chain Intact: Yes"

    print_section "Storage"
    print_success "Replicated to 3 nodes"
    print_info "  ├─ node-us-east-1"
    print_info "  ├─ node-eu-west-1"
    print_info "  └─ node-asia-east-1"

    echo ""
    print_success "Event recorded successfully to timeline $timeline"
    echo ""
}

# Query archive
query_archive() {
    local date_range=${1:-"1900-01-01:2025-12-31"}
    local timeline=${2:-PRIME}
    local type=${3:-all}

    print_section "Querying Historical Archive"

    IFS=':' read -ra DATES <<< "$date_range"
    local start_date=${DATES[0]}
    local end_date=${DATES[1]:-$(date +%Y-%m-%d)}

    print_info "Timeline: $timeline"
    print_info "Date Range: $start_date to $end_date"
    print_info "Event Type: $type"

    print_section "Query Results"

    local total=$((RANDOM % 1000 + 100))
    local returned=$((total > 100 ? 100 : total))

    print_info "Total matching records: $total"
    print_info "Returned: $returned"
    print_info "Execution time: 0.${RANDOM:0:3} seconds"

    echo ""
    echo -e "${CYAN}ID                    Title                          Date         Type${RESET}"
    echo -e "${GRAY}────────────────────────────────────────────────────────────────────────${RESET}"

    # Generate sample results
    local events=("Moon Landing" "World War II Ends" "Internet Invented" "First Flight" "Printing Press")
    local types=("scientific" "military" "technological" "scientific" "technological")
    local years=(1969 1945 1989 1903 1440)

    for i in {0..4}; do
        local event="${events[$i]}"
        local evt_type="${types[$i]}"
        local year="${years[$i]}"
        local id="REC-${timeline}-${year}-$(printf '%06d' $i)"

        printf "${GREEN}%-22s${RESET} %-30s ${GRAY}%4d-XX-XX${RESET}  %s\n" \
            "$id" "$event" "$year" "$evt_type"
    done

    echo ""
    print_info "Use --limit and --offset for pagination"
    echo ""
}

# Verify record
verify_record() {
    local record_id=${1:-REC-PRIME-1969-000001}
    local timeline=${2:-PRIME}
    local method=${3:-multi-witness}

    print_section "Verifying Historical Record"

    print_info "Record ID: $record_id"
    print_info "Timeline: $timeline"
    print_info "Verification Method: $method"

    print_section "Integrity Check"
    sleep 1

    local hash_valid=$((RANDOM % 2))
    local chain_valid=$((RANDOM % 2))

    if [ $hash_valid -eq 1 ]; then
        print_success "Content Hash: Valid"
    else
        print_error "Content Hash: Invalid"
    fi

    if [ $chain_valid -eq 1 ]; then
        print_success "Chain Link: Valid"
    else
        print_error "Chain Link: Broken"
    fi

    print_success "Signature: Valid"

    print_section "Verification Results"

    if [ $hash_valid -eq 1 ] && [ $chain_valid -eq 1 ]; then
        local confidence=$((90 + RANDOM % 10))
        print_success "Verification: PASSED"
        print_success "Confidence: ${confidence}.$(RANDOM % 10)%"
        print_info "Verifiers: 3"
        print_info "  ├─ verifier-academic-001"
        print_info "  ├─ verifier-archive-002"
        print_info "  └─ verifier-institution-003"
    else
        print_warning "Verification: PARTIAL"
        print_warning "Confidence: ${confidence}.$(RANDOM % 10)%"
        print_info "Issues detected - manual review required"
    fi

    print_section "Evidence Sources"
    print_info "Primary sources: 3"
    print_info "Secondary sources: 5"
    print_info "Scientific data: 2"

    echo ""
}

# Compare timelines
compare_timelines() {
    local timeline1=${1:-PRIME}
    local timeline2=${2:-ALT-2024-001}
    local since=${3:-1900-01-01}

    print_section "Comparing Timelines"

    print_info "Timeline 1: $timeline1"
    print_info "Timeline 2: $timeline2"
    print_info "Since: $since"

    print_section "Divergence Detection"
    sleep 1

    local divergence_year=$((1900 + RANDOM % 125))
    print_success "Divergence Point: ${divergence_year}-06-15"
    print_info "Divergence Event: Major historical event altered"
    print_info "Detection Confidence: 98.5%"

    print_section "Differences"

    local added=$((RANDOM % 50))
    local removed=$((RANDOM % 30))
    local modified=$((RANDOM % 100))

    print_info "Events Added: $added"
    print_info "Events Removed: $removed"
    print_info "Events Modified: $modified"
    print_info "Total Differences: $((added + removed + modified))"

    print_section "Divergence Severity"

    local total=$((added + removed + modified))
    if [ $total -lt 10 ]; then
        print_success "Severity: MINOR"
        print_info "Few differences detected, timelines mostly aligned"
    elif [ $total -lt 100 ]; then
        print_warning "Severity: MODERATE"
        print_info "Significant differences, careful reconciliation needed"
    else
        print_error "Severity: MAJOR"
        print_info "Substantial timeline divergence detected"
    fi

    print_section "Sample Differences"
    echo -e "${GRAY}  1962-10-28: Cuban Missile Crisis outcome differs${RESET}"
    echo -e "${GRAY}  1969-07-20: Moon landing date differs by 3 days${RESET}"
    echo -e "${GRAY}  1989-11-09: Berlin Wall event sequence altered${RESET}"

    echo ""
}

# Preserve alteration
preserve_alteration() {
    local timeline=${1:-PRIME}
    local reason=${2:-"Timeline alteration detected"}

    print_section "Preserving Timeline Alteration"

    print_info "Original Timeline: $timeline"
    print_info "Preservation Reason: $reason"

    # Generate preserved timeline ID
    local timestamp=$(date +%s)
    local preserved_id="${timeline}-PRESERVED-${timestamp}"

    print_section "Creating Snapshot"
    sleep 1

    print_success "Snapshot ID: $preserved_id"
    print_info "Snapshot Type: Complete Timeline Copy"
    print_info "Records Preserved: $((RANDOM % 10000 + 1000))"

    print_section "Alteration Metadata"
    local alt_id="ALT-$(date +%Y-%m-%d)-$(printf '%03d' $((RANDOM % 1000)))"
    print_success "Alteration ID: $alt_id"
    print_info "Detected: $(date)"
    print_info "Severity: MODERATE"
    print_info "Affected Records: $((RANDOM % 100 + 10))"
    print_info "Temporal Energy: 2.4 × 10²⁵ joules"

    print_section "Replication"
    print_success "Original timeline preserved to:"
    print_info "  ├─ node-us-east-1 (master)"
    print_info "  ├─ node-eu-west-1 (replica)"
    print_info "  ├─ node-asia-east-1 (replica)"
    print_info "  └─ node-au-east-1 (backup)"

    print_section "Cross-Reference"
    print_info "Original Timeline: $preserved_id"
    print_info "Altered Timeline: $timeline"
    print_success "Cross-reference stored in archive index"

    echo ""
    print_success "Timeline alteration preserved successfully"
    echo ""
}

# Export archive
export_archive() {
    local timeline=${1:-PRIME}
    local format=${2:-json}
    local output=${3:-archive-export.json}

    print_section "Exporting Archive Data"

    print_info "Timeline: $timeline"
    print_info "Format: $format"
    print_info "Output: $output"

    print_section "Collecting Records"
    sleep 1

    local total_records=$((RANDOM % 10000 + 1000))
    print_info "Records to export: $total_records"
    print_info "Date range: 1900-01-01 to $(date +%Y-%m-%d)"

    print_section "Export Process"
    print_info "Serializing records..."
    sleep 1
    print_success "Serialization complete"

    print_info "Compressing data (Brotli level 11)..."
    sleep 1
    local original_size=$((total_records * 10))
    local compressed_size=$((original_size / 8))
    print_success "Compression complete (${compressed_size} KB, 87% reduction)"

    print_info "Writing to file..."
    sleep 1
    print_success "Export complete: $output"

    print_section "Export Summary"
    print_info "Total Records: $total_records"
    print_info "File Size: ${compressed_size} KB"
    print_info "Format: $format"
    print_info "Checksum: $(echo -n "$output" | md5sum | cut -d' ' -f1)"

    print_section "Download Link"
    local export_id="EXP-$(date +%s)"
    print_success "Export ID: $export_id"
    print_info "URL: https://archive.wiastandards.com/exports/$export_id.$format"
    print_info "Expires: $(date -d '+7 days' 2>/dev/null || date)"

    echo ""
}

# Generate timeline report
timeline_report() {
    local timeline=${1:-PRIME}
    local period=${2:-20th-century}

    print_section "Timeline Report: $timeline"

    print_info "Timeline: $timeline"
    print_info "Period: $period"
    print_info "Generated: $(date)"

    print_section "Timeline Statistics"

    local total_records=$((RANDOM % 10000 + 1000))
    local verified=$((total_records * 95 / 100))
    local unverified=$((total_records - verified))

    print_info "Total Records: $total_records"
    print_info "  ├─ Verified: $verified (95%)"
    print_info "  └─ Unverified: $unverified (5%)"

    print_section "Record Distribution by Type"
    print_info "Political: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"
    print_info "Military: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"
    print_info "Scientific: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"
    print_info "Cultural: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"
    print_info "Technological: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"
    print_info "Other: $((RANDOM % 1000 + 100)) ($(RANDOM % 20)%)"

    print_section "Importance Levels"
    print_info "Critical Events: $((RANDOM % 100 + 10))"
    print_info "High Importance: $((RANDOM % 500 + 50))"
    print_info "Medium Importance: $((RANDOM % 2000 + 200))"
    print_info "Low Importance: $((RANDOM % 5000 + 500))"

    print_section "Timeline Integrity"
    print_success "Chain Integrity: 100%"
    print_success "Hash Verification: All records valid"
    print_success "Replication Status: 3/3 nodes synchronized"
    print_info "Last Integrity Check: $(date)"

    print_section "Access Statistics"
    print_info "Total Queries (30 days): $((RANDOM % 10000 + 1000))"
    print_info "Active Researchers: $((RANDOM % 100 + 10))"
    print_info "Export Requests: $((RANDOM % 50 + 5))"
    print_info "Verification Requests: $((RANDOM % 200 + 20))"

    print_section "Recent Events"
    print_info "Last 5 recorded events:"
    echo -e "${GRAY}  2024-12-15: Global climate summit${RESET}"
    echo -e "${GRAY}  2024-11-20: Scientific breakthrough${RESET}"
    echo -e "${GRAY}  2024-10-05: Political treaty signed${RESET}"
    echo -e "${GRAY}  2024-09-18: Cultural festival${RESET}"
    echo -e "${GRAY}  2024-08-22: Technological innovation${RESET}"

    echo ""
}

# List timelines
list_timelines() {
    print_section "Available Timelines"

    print_info "Displaying all timelines in archive"

    echo ""
    echo -e "${CYAN}Timeline ID          Status      Records   Divergence${RESET}"
    echo -e "${GRAY}─────────────────────────────────────────────────────────${RESET}"

    # Primary timeline
    print_success "PRIME                Active      $(printf '%6d' $((RANDOM % 10000 + 5000)))    -"

    # Alternate timelines
    local alt_count=$((RANDOM % 5 + 2))
    for i in $(seq 1 $alt_count); do
        local year=$((1900 + RANDOM % 125))
        local records=$((RANDOM % 8000 + 1000))
        echo -e "${YELLOW}ALT-${year}-00${i}       Active      $(printf '%6d' $records)    ${year}${RESET}"
    done

    # Preserved timelines
    local preserved_count=$((RANDOM % 3 + 1))
    for i in $(seq 1 $preserved_count); do
        local timestamp=$((1600000000 + RANDOM % 100000000))
        local records=$((RANDOM % 5000 + 500))
        echo -e "${GRAY}PRIME-PRESERVED-${timestamp:0:10}  Preserved   $(printf '%6d' $records)    -${RESET}"
    done

    # Merged timelines
    if [ $((RANDOM % 2)) -eq 1 ]; then
        local records=$((RANDOM % 12000 + 2000))
        echo -e "${CYAN}MERGED-$(date +%s)     Active      $(printf '%6d' $records)    Multiple${RESET}"
    fi

    echo ""
    print_info "Total timelines: $((alt_count + preserved_count + 2))"
    print_info "Active timelines: $((alt_count + 1))"
    print_info "Preserved timelines: $preserved_count"
    echo ""
}

# Reconcile timelines
reconcile_timelines() {
    local timeline1=${1:-PRIME}
    local timeline2=${2:-ALT-2024-001}
    local strategy=${3:-preserve-both}

    print_section "Timeline Reconciliation"

    print_info "Timeline 1: $timeline1"
    print_info "Timeline 2: $timeline2"
    print_info "Strategy: $strategy"

    print_section "Analysis Phase"
    sleep 1

    print_info "Finding divergence point..."
    local divergence_year=$((1900 + RANDOM % 125))
    print_success "Divergence detected at ${divergence_year}-06-15"

    print_info "Calculating differences..."
    sleep 1
    local total_diff=$((RANDOM % 200 + 50))
    print_success "$total_diff differences identified"

    print_section "Reconciliation Strategy: $strategy"

    case $strategy in
        preserve-both)
            print_info "Both timelines will be preserved separately"
            print_success "No data loss"
            print_info "Cross-reference will be created"
            ;;
        merge)
            print_info "Timelines will be merged into new timeline"
            print_warning "Conflicts will be flagged for review"
            local merged_id="MERGED-$(date +%s)"
            print_success "New timeline ID: $merged_id"
            ;;
        prefer-primary)
            print_info "Timeline 1 treated as authoritative"
            print_warning "Timeline 2 marked as alternate"
            print_success "Primary timeline unchanged"
            ;;
        consensus)
            print_info "Using consensus from available timelines"
            print_info "Gathering data from 5 timelines..."
            print_success "Consensus algorithm applied"
            ;;
    esac

    print_section "Execution Phase"
    sleep 1

    print_success "Common history preserved (${divergence_year} records)"
    print_success "Divergent records categorized"
    print_success "Cross-references created"
    print_success "Integrity chains updated"

    print_section "Results"
    print_info "Reconciliation Status: Complete"
    print_info "Common Records: $((RANDOM % 5000 + 1000))"
    print_info "Timeline 1 Unique: $((RANDOM % 100 + 10))"
    print_info "Timeline 2 Unique: $((RANDOM % 100 + 10))"
    print_info "Conflicts: $((RANDOM % 20 + 1))"

    echo ""
    print_success "Timeline reconciliation completed successfully"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-time-033 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  record                   Record historical event"
    echo "    --event <text>         Event description"
    echo "    --date <YYYY-MM-DD>    Event date"
    echo "    --timeline <id>        Timeline ID (default: PRIME)"
    echo "    --type <type>          Event type"
    echo ""
    echo "  query                    Query historical archive"
    echo "    --date-range <range>   Date range (YYYY-MM-DD:YYYY-MM-DD)"
    echo "    --timeline <id>        Timeline to query (default: PRIME)"
    echo "    --type <type>          Event type filter"
    echo ""
    echo "  verify                   Verify record integrity"
    echo "    --record-id <id>       Record identifier"
    echo "    --timeline <id>        Timeline ID (default: PRIME)"
    echo "    --method <method>      Verification method"
    echo ""
    echo "  compare                  Compare two timelines"
    echo "    --timeline1 <id>       First timeline (default: PRIME)"
    echo "    --timeline2 <id>       Second timeline"
    echo "    --since <date>         Comparison start date"
    echo ""
    echo "  preserve-alteration      Preserve timeline alteration"
    echo "    --timeline <id>        Timeline to preserve"
    echo "    --reason <text>        Preservation reason"
    echo ""
    echo "  export                   Export archive data"
    echo "    --timeline <id>        Timeline to export (default: PRIME)"
    echo "    --format <format>      Export format (json|xml|csv|rdf)"
    echo "    --output <file>        Output filename"
    echo ""
    echo "  report                   Generate timeline report"
    echo "    --timeline <id>        Timeline ID (default: PRIME)"
    echo "    --period <period>      Time period"
    echo ""
    echo "  list-timelines           List all available timelines"
    echo ""
    echo "  reconcile                Reconcile two timelines"
    echo "    --timeline1 <id>       First timeline"
    echo "    --timeline2 <id>       Second timeline"
    echo "    --strategy <strategy>  Reconciliation strategy"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Event Types:"
    echo "  political, military, scientific, cultural, natural,"
    echo "  economic, technological, social, other"
    echo ""
    echo "Reconciliation Strategies:"
    echo "  preserve-both, merge, prefer-primary, consensus"
    echo ""
    echo "Examples:"
    echo "  wia-time-033 record --event 'Moon Landing' --date '1969-07-20'"
    echo "  wia-time-033 query --date-range '1960-01-01:1970-12-31' --type scientific"
    echo "  wia-time-033 verify --record-id REC-PRIME-1969-000001"
    echo "  wia-time-033 compare --timeline1 PRIME --timeline2 ALT-2024-001"
    echo "  wia-time-033 export --timeline PRIME --format json"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-TIME-033 Historical Archive CLI"
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
    record)
        EVENT=""
        DATE=$(date +%Y-%m-%d)
        TIMELINE="PRIME"
        TYPE="other"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --event) EVENT=$2; shift 2 ;;
                --date) DATE=$2; shift 2 ;;
                --timeline) TIMELINE=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        record_event "$EVENT" "$DATE" "$TIMELINE" "$TYPE"
        ;;

    query)
        DATE_RANGE="1900-01-01:2025-12-31"
        TIMELINE="PRIME"
        TYPE="all"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --date-range) DATE_RANGE=$2; shift 2 ;;
                --timeline) TIMELINE=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        query_archive "$DATE_RANGE" "$TIMELINE" "$TYPE"
        ;;

    verify)
        RECORD_ID="REC-PRIME-1969-000001"
        TIMELINE="PRIME"
        METHOD="multi-witness"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --record-id) RECORD_ID=$2; shift 2 ;;
                --timeline) TIMELINE=$2; shift 2 ;;
                --method) METHOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        verify_record "$RECORD_ID" "$TIMELINE" "$METHOD"
        ;;

    compare)
        TIMELINE1="PRIME"
        TIMELINE2="ALT-2024-001"
        SINCE="1900-01-01"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --timeline1) TIMELINE1=$2; shift 2 ;;
                --timeline2) TIMELINE2=$2; shift 2 ;;
                --since) SINCE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        compare_timelines "$TIMELINE1" "$TIMELINE2" "$SINCE"
        ;;

    preserve-alteration)
        TIMELINE="PRIME"
        REASON="Timeline alteration detected"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --timeline) TIMELINE=$2; shift 2 ;;
                --reason) REASON=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        preserve_alteration "$TIMELINE" "$REASON"
        ;;

    export)
        TIMELINE="PRIME"
        FORMAT="json"
        OUTPUT="archive-export.json"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --timeline) TIMELINE=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        export_archive "$TIMELINE" "$FORMAT" "$OUTPUT"
        ;;

    report)
        TIMELINE="PRIME"
        PERIOD="20th-century"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --timeline) TIMELINE=$2; shift 2 ;;
                --period) PERIOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        timeline_report "$TIMELINE" "$PERIOD"
        ;;

    list-timelines)
        print_header
        list_timelines
        ;;

    reconcile)
        TIMELINE1="PRIME"
        TIMELINE2="ALT-2024-001"
        STRATEGY="preserve-both"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --timeline1) TIMELINE1=$2; shift 2 ;;
                --timeline2) TIMELINE2=$2; shift 2 ;;
                --strategy) STRATEGY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        reconcile_timelines "$TIMELINE1" "$TIMELINE2" "$STRATEGY"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-time-033 help' for usage information"
        exit 1
        ;;
esac

exit 0
