#!/bin/bash

###############################################################################
# WIA-OCEAN_CONSERVATION CLI Tool
# Version 1.0
# 弘益人間 (Benefit All Humanity)
#
# Command-line interface for ocean conservation operations including:
# - Marine Protected Area monitoring
# - Species tracking
# - Illegal fishing detection
# - Coral reef status checking
# - Pollution tracking
# - Report generation
###############################################################################

set -e

# Configuration
WIA_API_BASE="${WIA_API_BASE:-https://api.wia.org/ocean-conservation/v1}"
WIA_API_KEY="${WIA_API_KEY:-}"
VERSION="1.0.0"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
error() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

success() {
    echo -e "${GREEN}$1${NC}"
}

info() {
    echo -e "${BLUE}$1${NC}"
}

warn() {
    echo -e "${YELLOW}$1${NC}"
}

check_api_key() {
    if [ -z "$WIA_API_KEY" ]; then
        error "WIA_API_KEY environment variable not set. Export your API key first."
    fi
}

api_request() {
    local method="$1"
    local endpoint="$2"
    local data="$3"

    check_api_key

    if [ -z "$data" ]; then
        curl -s -X "$method" \
            -H "Authorization: Bearer $WIA_API_KEY" \
            -H "Content-Type: application/json" \
            "$WIA_API_BASE$endpoint"
    else
        curl -s -X "$method" \
            -H "Authorization: Bearer $WIA_API_KEY" \
            -H "Content-Type: application/json" \
            -d "$data" \
            "$WIA_API_BASE$endpoint"
    fi
}

# Command: area-monitor
area_monitor() {
    local mpa_id="$1"

    if [ -z "$mpa_id" ]; then
        error "MPA ID required. Usage: area-monitor <mpa-id>"
    fi

    info "Monitoring Marine Protected Area: $mpa_id"

    # Get MPA information
    local mpa_info=$(api_request "GET" "/mpas/$mpa_id")

    if [ $? -eq 0 ]; then
        echo "$mpa_info" | jq '.'

        # Get effectiveness metrics
        local effectiveness=$(api_request "GET" "/mpas/$mpa_id/effectiveness")

        echo ""
        info "Effectiveness Metrics:"
        echo "$effectiveness" | jq '.'

        success "MPA monitoring complete"
    else
        error "Failed to retrieve MPA information"
    fi
}

# Command: species-track
species_track() {
    local species_code="$1"
    local region="${2:-}"

    if [ -z "$species_code" ]; then
        error "Species code required. Usage: species-track <species-code> [region]"
    fi

    info "Tracking species: $species_code"

    # Get observations
    local params="speciesCode=$species_code&verified=true&limit=20"
    if [ -n "$region" ]; then
        params="$params&region=$region"
    fi

    local observations=$(api_request "GET" "/species/observations?$params")

    if [ $? -eq 0 ]; then
        local count=$(echo "$observations" | jq '.total')
        info "Found $count observations"
        echo "$observations" | jq '.items[] | {date: .timestamp, location: .location, count: .count, behavior: .behavior}'

        # Get population trends
        info "\nPopulation Trends:"
        local trends=$(api_request "GET" "/species/$species_code/population")
        echo "$trends" | jq '{currentPopulation, trend, trendRate, threats}'

        success "Species tracking complete"
    else
        error "Failed to retrieve species data"
    fi
}

# Command: fishing-detect
fishing_detect() {
    local vessel_id="$1"

    if [ -z "$vessel_id" ]; then
        error "Vessel ID required. Usage: fishing-detect <vessel-id>"
    fi

    info "Detecting illegal fishing activity for vessel: $vessel_id"

    # Track vessel
    local tracking=$(api_request "GET" "/enforcement/vessels/$vessel_id/track?includeAISGaps=true")

    if [ $? -eq 0 ]; then
        local mpa_intrusions=$(echo "$tracking" | jq '.mpaIntrusions')
        local violations=$(echo "$tracking" | jq '.violations')
        local suspicious=$(echo "$tracking" | jq '.suspiciousPatterns | length')

        echo ""
        info "Analysis Results:"
        echo "  MPA Intrusions: $mpa_intrusions"
        echo "  Violations: $violations"
        echo "  Suspicious Patterns: $suspicious"

        if [ "$violations" -gt 0 ] || [ "$mpa_intrusions" -gt 0 ]; then
            warn "\n⚠ Violations detected!"
            echo "$tracking" | jq '.suspiciousPatterns'
        else
            success "No violations detected"
        fi

        success "Fishing detection complete"
    else
        error "Failed to track vessel"
    fi
}

# Command: reef-status
reef_status() {
    local reef_id="$1"

    if [ -z "$reef_id" ]; then
        error "Reef ID required. Usage: reef-status <reef-id>"
    fi

    info "Checking coral reef health: $reef_id"

    # Get reef health
    local health=$(api_request "GET" "/ecosystems/coral-reefs/$reef_id/health")

    if [ $? -eq 0 ]; then
        local score=$(echo "$health" | jq '.currentHealth.overallScore')
        local coral_cover=$(echo "$health" | jq '.currentHealth.coralCover')
        local bleaching=$(echo "$health" | jq -r '.currentHealth.bleaching')
        local trend=$(echo "$health" | jq -r '.currentHealth.trend')

        echo ""
        info "Reef Health Status:"
        echo "  Overall Score: $score/100"
        echo "  Coral Cover: $coral_cover%"
        echo "  Bleaching Level: $bleaching"
        echo "  Trend: $trend"

        # Check for alerts
        local alerts=$(echo "$health" | jq '.alerts | length')
        if [ "$alerts" -gt 0 ]; then
            warn "\n⚠ Active Alerts:"
            echo "$health" | jq '.alerts[] | "  - [\(.severity)] \(.message)"' -r
        fi

        if (( $(echo "$score < 50" | bc -l) )); then
            warn "⚠ Reef health is poor - intervention recommended"
        else
            success "Reef health check complete"
        fi
    else
        error "Failed to retrieve reef status"
    fi
}

# Command: pollution-track
pollution_track() {
    local event_type="${1:-PLASTIC_DEBRIS}"
    local region="${2:-}"

    info "Tracking pollution events: $event_type"

    # Get pollution events
    local params="eventType=$event_type&limit=10"
    if [ -n "$region" ]; then
        params="$params&region=$region"
    fi

    local events=$(api_request "GET" "/pollution/events?$params")

    if [ $? -eq 0 ]; then
        local count=$(echo "$events" | jq '.total')
        info "Found $count pollution events"
        echo ""
        echo "$events" | jq '.items[] | {
            id: .pollutionEventId,
            type: .eventType,
            severity: .severity,
            date: .detectedDate,
            area: (.affectedArea | tostring + " km²"),
            status: .response.status
        }'

        # Get hotspots
        info "\nPollution Hotspots:"
        local hotspots=$(api_request "GET" "/pollution/hotspots?type=$event_type")
        echo "$hotspots" | jq '{totalHotspots, criticalHotspots}'

        success "Pollution tracking complete"
    else
        error "Failed to retrieve pollution data"
    fi
}

# Command: report-generate
report_generate() {
    local report_type="$1"
    local region="${2:-GLOBAL}"
    local start_date="${3:-2025-01-01}"
    local end_date="${4:-2026-01-12}"

    if [ -z "$report_type" ]; then
        error "Report type required. Usage: report-generate <type> [region] [start-date] [end-date]\nTypes: SPECIES_POPULATION, ECOSYSTEM_HEALTH, MPA_EFFECTIVENESS, ENFORCEMENT_SUMMARY"
    fi

    info "Generating $report_type report for $region"

    # Generate report
    local request=$(cat <<EOF
{
    "reportType": "$report_type",
    "region": "$region",
    "period": {
        "start": "${start_date}T00:00:00Z",
        "end": "${end_date}T23:59:59Z"
    },
    "includeMetrics": ["BIODIVERSITY", "THREATS", "CONSERVATION_ACTIONS"],
    "format": "PDF"
}
EOF
)

    local response=$(api_request "POST" "/reports/generate" "$request")

    if [ $? -eq 0 ]; then
        local report_id=$(echo "$response" | jq -r '.reportId')
        local status=$(echo "$response" | jq -r '.status')

        info "Report generation started"
        echo "  Report ID: $report_id"
        echo "  Status: $status"

        # Poll for completion
        info "\nWaiting for report generation..."
        for i in {1..30}; do
            sleep 2
            local status_response=$(api_request "GET" "/reports/$report_id")
            local current_status=$(echo "$status_response" | jq -r '.status')

            if [ "$current_status" = "COMPLETED" ]; then
                local download_url=$(echo "$status_response" | jq -r '.downloadUrl')
                success "\n✓ Report generated successfully!"
                echo "  Download URL: $download_url"
                return 0
            elif [ "$current_status" = "FAILED" ]; then
                error "Report generation failed"
            fi

            echo -n "."
        done

        warn "\nReport is still generating. Check status with: report-status $report_id"
    else
        error "Failed to generate report"
    fi
}

# Command: help
show_help() {
    cat << EOF
WIA-OCEAN_CONSERVATION CLI Tool v${VERSION}
弘益人間 (Benefit All Humanity)

Usage: wia-ocean-conservation <command> [options]

Commands:
  area-monitor <mpa-id>                   Monitor Marine Protected Area
  species-track <species-code> [region]   Track marine species observations
  fishing-detect <vessel-id>              Detect illegal fishing activity
  reef-status <reef-id>                   Check coral reef health status
  pollution-track [type] [region]         Track ocean pollution events
  report-generate <type> [region] [start] [end]
                                          Generate conservation reports
  --help, -h                              Show this help message
  --version, -v                           Show version information

Environment Variables:
  WIA_API_KEY       Your WIA API key (required)
  WIA_API_BASE      API base URL (default: https://api.wia.org/ocean-conservation/v1)

Examples:
  export WIA_API_KEY="your-api-key-here"

  wia-ocean-conservation area-monitor mpa-uuid-123
  wia-ocean-conservation species-track IUCN-Chelonia-mydas CARIBBEAN_SEA
  wia-ocean-conservation fishing-detect IMO-1234567
  wia-ocean-conservation reef-status reef-uuid-456
  wia-ocean-conservation pollution-track PLASTIC_DEBRIS PACIFIC_OCEAN
  wia-ocean-conservation report-generate ECOSYSTEM_HEALTH GREAT_BARRIER_REEF

Report Types:
  - SPECIES_POPULATION
  - ECOSYSTEM_HEALTH
  - MPA_EFFECTIVENESS
  - ENFORCEMENT_SUMMARY
  - POLLUTION_ASSESSMENT
  - CONSERVATION_IMPACT

For more information: https://wia.org/standards/ocean-conservation

© 2025 SmileStory Inc. / WIA
EOF
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        area-monitor)
            shift
            area_monitor "$@"
            ;;
        species-track)
            shift
            species_track "$@"
            ;;
        fishing-detect)
            shift
            fishing_detect "$@"
            ;;
        reef-status)
            shift
            reef_status "$@"
            ;;
        pollution-track)
            shift
            pollution_track "$@"
            ;;
        report-generate)
            shift
            report_generate "$@"
            ;;
        --help|-h|help)
            show_help
            ;;
        --version|-v)
            echo "WIA-OCEAN_CONSERVATION CLI v${VERSION}"
            ;;
        *)
            error "Unknown command: $1\nRun 'wia-ocean-conservation --help' for usage information."
            ;;
    esac
}

# Run main function
main "$@"
