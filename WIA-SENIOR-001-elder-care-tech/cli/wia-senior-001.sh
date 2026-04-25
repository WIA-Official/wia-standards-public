#!/bin/bash

###############################################################################
# WIA-SENIOR-001: Elder Care Technology Standard - CLI Tool
# 弘益人間 (Benefit All Humanity)
#
# Usage: ./wia-senior-001.sh [command] [options]
###############################################################################

set -e

VERSION="1.0.0"
API_URL="${WIA_API_URL:-https://api.wia.org/senior-001}"
API_KEY="${WIA_API_KEY}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "${ORANGE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║         WIA-SENIOR-001: Elder Care Technology            ║"
    echo "║                   弘益人間 (Benefit All)                  ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API_KEY not set. Please set WIA_API_KEY environment variable."
        exit 1
    fi
}

# Command: Get Elder Profile
get_elder() {
    check_api_key
    local elder_id=$1

    if [ -z "$elder_id" ]; then
        print_error "Elder ID required"
        echo "Usage: $0 get-elder <elder_id>"
        exit 1
    fi

    print_info "Fetching elder profile: $elder_id"

    response=$(curl -s -X GET \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        "$API_URL/elders/$elder_id")

    echo "$response" | jq '.'
}

# Command: Record Vital Signs
record_vitals() {
    check_api_key
    local elder_id=$1
    local heart_rate=$2
    local bp_systolic=$3
    local bp_diastolic=$4

    if [ -z "$elder_id" ]; then
        print_error "Elder ID required"
        echo "Usage: $0 record-vitals <elder_id> <heart_rate> <bp_systolic> <bp_diastolic>"
        exit 1
    fi

    print_info "Recording vital signs for: $elder_id"

    data=$(cat <<EOF
{
  "elderId": "$elder_id",
  "timestamp": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "heartRate": {
    "bpm": ${heart_rate:-70},
    "rhythm": "regular",
    "isAbnormal": false
  },
  "bloodPressure": {
    "systolic": ${bp_systolic:-120},
    "diastolic": ${bp_diastolic:-80},
    "unit": "mmHg",
    "isAbnormal": false
  }
}
EOF
)

    response=$(curl -s -X POST \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$data" \
        "$API_URL/vitals")

    echo "$response" | jq '.'
    print_success "Vital signs recorded"
}

# Command: Get Latest Vitals
get_latest_vitals() {
    check_api_key
    local elder_id=$1

    if [ -z "$elder_id" ]; then
        print_error "Elder ID required"
        echo "Usage: $0 get-latest-vitals <elder_id>"
        exit 1
    fi

    print_info "Fetching latest vital signs: $elder_id"

    response=$(curl -s -X GET \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        "$API_URL/elders/$elder_id/vitals/latest")

    echo "$response" | jq '.'
}

# Command: Create Alert
create_alert() {
    check_api_key
    local elder_id=$1
    local alert_type=$2
    local severity=$3
    local message=$4

    if [ -z "$elder_id" ] || [ -z "$alert_type" ]; then
        print_error "Elder ID and alert type required"
        echo "Usage: $0 create-alert <elder_id> <type> <severity> <message>"
        exit 1
    fi

    print_info "Creating alert for: $elder_id"

    data=$(cat <<EOF
{
  "elderId": "$elder_id",
  "type": "$alert_type",
  "severity": "${severity:-MEDIUM}",
  "title": "Alert",
  "message": "${message:-Alert created via CLI}",
  "timestamp": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "resolved": false
}
EOF
)

    response=$(curl -s -X POST \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$data" \
        "$API_URL/alerts")

    echo "$response" | jq '.'
    print_success "Alert created"
}

# Command: Get Alerts
get_alerts() {
    check_api_key
    local elder_id=$1
    local resolved=${2:-false}

    if [ -z "$elder_id" ]; then
        print_error "Elder ID required"
        echo "Usage: $0 get-alerts <elder_id> [resolved]"
        exit 1
    fi

    print_info "Fetching alerts for: $elder_id"

    response=$(curl -s -X GET \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        "$API_URL/elders/$elder_id/alerts?resolved=$resolved")

    echo "$response" | jq '.'
}

# Command: Set Medication Reminder
set_medication() {
    check_api_key
    local elder_id=$1
    local medication_name=$2
    local schedule=$3

    if [ -z "$elder_id" ] || [ -z "$medication_name" ]; then
        print_error "Elder ID and medication name required"
        echo "Usage: $0 set-medication <elder_id> <medication_name> <schedule>"
        exit 1
    fi

    print_info "Setting medication reminder: $medication_name"

    data=$(cat <<EOF
{
  "elderId": "$elder_id",
  "name": "$medication_name",
  "dosage": "1 tablet",
  "frequency": "once-daily",
  "schedule": ["${schedule:-08:00}"],
  "startDate": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
}
EOF
)

    response=$(curl -s -X POST \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$data" \
        "$API_URL/medications")

    echo "$response" | jq '.'
    print_success "Medication reminder set"
}

# Command: Monitor (Real-time)
monitor() {
    check_api_key
    local elder_id=$1

    if [ -z "$elder_id" ]; then
        print_error "Elder ID required"
        echo "Usage: $0 monitor <elder_id>"
        exit 1
    fi

    print_info "Starting real-time monitoring for: $elder_id"
    print_info "Press Ctrl+C to stop"

    while true; do
        clear
        print_header
        echo -e "${ORANGE}Elder ID: $elder_id${NC}"
        echo -e "${ORANGE}Time: $(date)${NC}\n"

        # Fetch latest vitals
        vitals=$(curl -s -X GET \
            -H "Authorization: Bearer $API_KEY" \
            "$API_URL/elders/$elder_id/vitals/latest")

        # Display vitals
        echo "$vitals" | jq -r '
            if .data then
                "Heart Rate: " + (.data.heartRate.bpm | tostring) + " bpm\n" +
                "Blood Pressure: " + (.data.bloodPressure.systolic | tostring) + "/" +
                (.data.bloodPressure.diastolic | tostring) + " mmHg\n" +
                "Temperature: " + ((.data.temperature.value // 0) | tostring) + "°C\n" +
                "O2 Saturation: " + ((.data.oxygenSaturation.percentage // 0) | tostring) + "%"
            else
                "No data available"
            end
        '

        sleep 5
    done
}

# Command: Help
show_help() {
    print_header
    echo "Elder Care Technology Standard CLI Tool"
    echo ""
    echo "Commands:"
    echo "  get-elder <id>                    Get elder profile"
    echo "  record-vitals <id> <hr> <sys> <dia>  Record vital signs"
    echo "  get-latest-vitals <id>            Get latest vital signs"
    echo "  create-alert <id> <type> <severity> <msg>  Create alert"
    echo "  get-alerts <id> [resolved]        Get alerts"
    echo "  set-medication <id> <name> <time> Set medication reminder"
    echo "  monitor <id>                      Real-time monitoring"
    echo "  version                           Show version"
    echo "  help                              Show this help"
    echo ""
    echo "Environment Variables:"
    echo "  WIA_API_KEY      Your WIA API key (required)"
    echo "  WIA_API_URL      API endpoint (default: https://api.wia.org/senior-001)"
    echo ""
    echo "Examples:"
    echo "  $0 get-elder elder-123"
    echo "  $0 record-vitals elder-123 75 120 80"
    echo "  $0 create-alert elder-123 FALL_DETECTED HIGH 'Fall detected in bathroom'"
    echo "  $0 monitor elder-123"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
}

# Main
case "$1" in
    get-elder)
        get_elder "$2"
        ;;
    record-vitals)
        record_vitals "$2" "$3" "$4" "$5"
        ;;
    get-latest-vitals)
        get_latest_vitals "$2"
        ;;
    create-alert)
        create_alert "$2" "$3" "$4" "$5"
        ;;
    get-alerts)
        get_alerts "$2" "$3"
        ;;
    set-medication)
        set_medication "$2" "$3" "$4"
        ;;
    monitor)
        monitor "$2"
        ;;
    version)
        echo "WIA-SENIOR-001 CLI v$VERSION"
        ;;
    help|--help|-h|"")
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
