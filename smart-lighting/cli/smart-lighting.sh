#!/bin/bash

##############################################################################
# WIA-CITY-009: Smart Lighting Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Version: 1.0.0
# License: MIT
##############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_SMART_LIGHTING_ENDPOINT:-https://api.wia.org/city-009/v1}"
API_KEY="${WIA_SMART_LIGHTING_API_KEY}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_SMART_LIGHTING_API_KEY environment variable."
        exit 1
    fi
}

api_request() {
    local method="$1"
    local path="$2"
    local data="$3"

    check_api_key

    if [ -n "$data" ]; then
        curl -s -X "$method" \
            -H "Authorization: Bearer $API_KEY" \
            -H "Content-Type: application/json" \
            -d "$data" \
            "$API_ENDPOINT$path"
    else
        curl -s -X "$method" \
            -H "Authorization: Bearer $API_KEY" \
            "$API_ENDPOINT$path"
    fi
}

# Commands

cmd_list_fixtures() {
    print_info "Fetching fixtures..."

    local zone="${1:-}"
    local query=""

    if [ -n "$zone" ]; then
        query="?zone=$zone"
    fi

    local response=$(api_request GET "/api/v1/fixtures$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.fixtureId)\t\(.name)\t\(.state.on)\t\(.state.brightness)%"' | \
            column -t -s $'\t' -N "ID,Name,Status,Brightness"
        print_success "Fixtures listed successfully"
    else
        print_error "Failed to fetch fixtures"
        exit 1
    fi
}

cmd_get_fixture() {
    local fixture_id="$1"

    if [ -z "$fixture_id" ]; then
        print_error "Usage: $0 get-fixture <fixture-id>"
        exit 1
    fi

    print_info "Fetching fixture: $fixture_id"

    local response=$(api_request GET "/api/v1/fixtures/$fixture_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Fixture retrieved successfully"
    else
        print_error "Failed to fetch fixture"
        exit 1
    fi
}

cmd_turn_on() {
    local fixture_id="$1"
    local fade_time="${2:-2000}"

    if [ -z "$fixture_id" ]; then
        print_error "Usage: $0 turn-on <fixture-id> [fade-time-ms]"
        exit 1
    fi

    print_info "Turning on fixture: $fixture_id"

    local data="{\"on\":true,\"fadeTime\":$fade_time}"
    local response=$(api_request PUT "/api/v1/fixtures/$fixture_id/state" "$data")

    if [ $? -eq 0 ]; then
        print_success "Fixture turned on"
    else
        print_error "Failed to turn on fixture"
        exit 1
    fi
}

cmd_turn_off() {
    local fixture_id="$1"
    local fade_time="${2:-2000}"

    if [ -z "$fixture_id" ]; then
        print_error "Usage: $0 turn-off <fixture-id> [fade-time-ms]"
        exit 1
    fi

    print_info "Turning off fixture: $fixture_id"

    local data="{\"on\":false,\"fadeTime\":$fade_time}"
    local response=$(api_request PUT "/api/v1/fixtures/$fixture_id/state" "$data")

    if [ $? -eq 0 ]; then
        print_success "Fixture turned off"
    else
        print_error "Failed to turn off fixture"
        exit 1
    fi
}

cmd_dim() {
    local fixture_id="$1"
    local brightness="$2"
    local fade_time="${3:-2000}"

    if [ -z "$fixture_id" ] || [ -z "$brightness" ]; then
        print_error "Usage: $0 dim <fixture-id> <brightness-0-100> [fade-time-ms]"
        exit 1
    fi

    print_info "Dimming fixture: $fixture_id to $brightness%"

    local data="{\"brightness\":$brightness,\"fadeTime\":$fade_time}"
    local response=$(api_request PUT "/api/v1/fixtures/$fixture_id/state" "$data")

    if [ $? -eq 0 ]; then
        print_success "Fixture dimmed to $brightness%"
    else
        print_error "Failed to dim fixture"
        exit 1
    fi
}

cmd_set_cct() {
    local fixture_id="$1"
    local cct="$2"
    local fade_time="${3:-2000}"

    if [ -z "$fixture_id" ] || [ -z "$cct" ]; then
        print_error "Usage: $0 set-cct <fixture-id> <color-temp-K> [fade-time-ms]"
        exit 1
    fi

    print_info "Setting color temperature: $fixture_id to $cct K"

    local data="{\"colorTemperature\":$cct,\"fadeTime\":$fade_time}"
    local response=$(api_request PUT "/api/v1/fixtures/$fixture_id/state" "$data")

    if [ $? -eq 0 ]; then
        print_success "Color temperature set to $cct K"
    else
        print_error "Failed to set color temperature"
        exit 1
    fi
}

cmd_list_scenes() {
    print_info "Fetching scenes..."

    local response=$(api_request GET "/api/v1/scenes")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.scenes[] | "\(.sceneId)\t\(.name)\t\(.metadata.category)"' | \
            column -t -s $'\t' -N "ID,Name,Category"
        print_success "Scenes listed successfully"
    else
        print_error "Failed to fetch scenes"
        exit 1
    fi
}

cmd_activate_scene() {
    local scene_id="$1"
    local fade_time="${2:-3000}"

    if [ -z "$scene_id" ]; then
        print_error "Usage: $0 activate-scene <scene-id> [fade-time-ms]"
        exit 1
    fi

    print_info "Activating scene: $scene_id"

    local data="{\"fadeTime\":$fade_time}"
    local response=$(api_request POST "/api/v1/scenes/$scene_id/activate" "$data")

    if [ $? -eq 0 ]; then
        print_success "Scene activated"
    else
        print_error "Failed to activate scene"
        exit 1
    fi
}

cmd_list_schedules() {
    print_info "Fetching schedules..."

    local response=$(api_request GET "/api/v1/schedules")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.schedules[] | "\(.scheduleId)\t\(.name)\t\(.enabled)"' | \
            column -t -s $'\t' -N "ID,Name,Enabled"
        print_success "Schedules listed successfully"
    else
        print_error "Failed to fetch schedules"
        exit 1
    fi
}

cmd_enable_schedule() {
    local schedule_id="$1"

    if [ -z "$schedule_id" ]; then
        print_error "Usage: $0 enable-schedule <schedule-id>"
        exit 1
    fi

    print_info "Enabling schedule: $schedule_id"

    local data='{"enabled":true}'
    local response=$(api_request PUT "/api/v1/schedules/$schedule_id/enabled" "$data")

    if [ $? -eq 0 ]; then
        print_success "Schedule enabled"
    else
        print_error "Failed to enable schedule"
        exit 1
    fi
}

cmd_disable_schedule() {
    local schedule_id="$1"

    if [ -z "$schedule_id" ]; then
        print_error "Usage: $0 disable-schedule <schedule-id>"
        exit 1
    fi

    print_info "Disabling schedule: $schedule_id"

    local data='{"enabled":false}'
    local response=$(api_request PUT "/api/v1/schedules/$schedule_id/enabled" "$data")

    if [ $? -eq 0 ]; then
        print_success "Schedule disabled"
    else
        print_error "Failed to disable schedule"
        exit 1
    fi
}

cmd_energy_dashboard() {
    local start="${1:-$(date -d '7 days ago' +%Y-%m-%dT00:00:00Z)}"
    local end="${2:-$(date +%Y-%m-%dT23:59:59Z)}"

    print_info "Fetching energy dashboard ($start to $end)..."

    local response=$(api_request GET "/api/v1/energy/dashboard?start=$start&end=$end")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Energy dashboard retrieved"
    else
        print_error "Failed to fetch energy dashboard"
        exit 1
    fi
}

cmd_list_sensors() {
    print_info "Fetching sensors..."

    local response=$(api_request GET "/api/v1/sensors")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.sensorId)\t\(.name)\t\(.type)\t\(.state.reachable)"' | \
            column -t -s $'\t' -N "ID,Name,Type,Online"
        print_success "Sensors listed successfully"
    else
        print_error "Failed to fetch sensors"
        exit 1
    fi
}

cmd_list_alerts() {
    print_info "Fetching alerts..."

    local response=$(api_request GET "/api/v1/alerts")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.alertId)\t\(.severity)\t\(.type)\t\(.status)\t\(.message)"' | \
            column -t -s $'\t' -N "ID,Severity,Type,Status,Message"
        print_success "Alerts listed successfully"
    else
        print_error "Failed to fetch alerts"
        exit 1
    fi
}

cmd_help() {
    cat <<EOF
WIA-CITY-009 Smart Lighting CLI Tool

Usage: $0 <command> [options]

FIXTURE COMMANDS:
  list-fixtures [zone]                     List all fixtures
  get-fixture <fixture-id>                 Get fixture details
  turn-on <fixture-id> [fade-time]        Turn on fixture
  turn-off <fixture-id> [fade-time]       Turn off fixture
  dim <fixture-id> <brightness> [fade]    Set brightness (0-100)
  set-cct <fixture-id> <kelvin> [fade]    Set color temperature

SCENE COMMANDS:
  list-scenes                              List all scenes
  activate-scene <scene-id> [fade-time]   Activate scene

SCHEDULE COMMANDS:
  list-schedules                           List all schedules
  enable-schedule <schedule-id>            Enable schedule
  disable-schedule <schedule-id>           Disable schedule

SENSOR COMMANDS:
  list-sensors                             List all sensors

ENERGY COMMANDS:
  energy-dashboard [start] [end]           Get energy dashboard

ALERT COMMANDS:
  list-alerts                              List all alerts

ENVIRONMENT VARIABLES:
  WIA_SMART_LIGHTING_ENDPOINT              API endpoint (default: https://api.wia.org/city-009/v1)
  WIA_SMART_LIGHTING_API_KEY               API key (required)

EXAMPLES:
  # List all fixtures
  $0 list-fixtures

  # Turn on a fixture with 2 second fade
  $0 turn-on fixture-001 2000

  # Dim fixture to 50%
  $0 dim fixture-001 50

  # Set warm white (2700K)
  $0 set-cct fixture-001 2700

  # Activate relax scene
  $0 activate-scene relax

  # View energy dashboard for last week
  $0 energy-dashboard

弘益人間 (홍익인간) - Benefit All Humanity
© 2025 WIA (World Certification Industry Association)
EOF
}

# Main

COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
    list-fixtures)
        cmd_list_fixtures "$@"
        ;;
    get-fixture)
        cmd_get_fixture "$@"
        ;;
    turn-on)
        cmd_turn_on "$@"
        ;;
    turn-off)
        cmd_turn_off "$@"
        ;;
    dim)
        cmd_dim "$@"
        ;;
    set-cct)
        cmd_set_cct "$@"
        ;;
    list-scenes)
        cmd_list_scenes "$@"
        ;;
    activate-scene)
        cmd_activate_scene "$@"
        ;;
    list-schedules)
        cmd_list_schedules "$@"
        ;;
    enable-schedule)
        cmd_enable_schedule "$@"
        ;;
    disable-schedule)
        cmd_disable_schedule "$@"
        ;;
    energy-dashboard)
        cmd_energy_dashboard "$@"
        ;;
    list-sensors)
        cmd_list_sensors "$@"
        ;;
    list-alerts)
        cmd_list_alerts "$@"
        ;;
    help|--help|-h)
        cmd_help
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        echo
        cmd_help
        exit 1
        ;;
esac
