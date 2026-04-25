#!/bin/bash

################################################################################
# WIA-ENE-040: Deep Sea Exploration Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-040
# Deep Sea Exploration Standard API.
#
# Usage:
#   deep-sea-exploration.sh <command> [options]
#
# Commands:
#   expeditions     List/manage expeditions
#   dives           List/manage dive operations
#   observations    List/manage organism observations
#   samples         List/manage biological samples
#   vents           List/manage hydrothermal vents
#   bathymetry      Bathymetry operations
#   submersibles    List/manage submersible vehicles
#   search          Search across all data
#   stats           View statistics
#   help            Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-040/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/deep-sea-exploration.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║     🐙  WIA-ENE-040: Deep Sea Exploration CLI v${VERSION}      ║"
  echo "║                                                                ║"
  echo "║              弘익人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
}

print_success() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1" >&2
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

# Check if jq is available
has_jq() {
  command -v jq &> /dev/null
}

# Pretty print JSON (uses jq if available)
pretty_json() {
  local json="$1"
  if has_jq; then
    echo "$json" | jq '.'
  else
    echo "$json"
  fi
}

# Load configuration
load_config() {
  if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    if [ -n "${WIA_API_KEY_CONF:-}" ]; then
      API_KEY="${WIA_API_KEY_CONF}"
    fi
    if [ -n "${WIA_API_ENDPOINT_CONF:-}" ]; then
      API_ENDPOINT="${WIA_API_ENDPOINT_CONF}"
    fi
  fi
}

# Save configuration
save_config() {
  mkdir -p "$(dirname "$CONFIG_FILE")"
  cat > "$CONFIG_FILE" <<EOF
# WIA-ENE-040 Deep Sea Exploration CLI Configuration
WIA_API_KEY_CONF="${API_KEY}"
WIA_API_ENDPOINT_CONF="${API_ENDPOINT}"
EOF
  chmod 600 "$CONFIG_FILE"
  print_success "Configuration saved to $CONFIG_FILE"
}

# Make API request
api_request() {
  local method="$1"
  local path="$2"
  local data="${3:-}"

  if [ -z "$API_KEY" ]; then
    print_error "API key not set. Use: export WIA_API_KEY=your-key or run 'config' command"
    exit 1
  fi

  local url="${API_ENDPOINT}${path}"
  local response

  if [ "$method" = "GET" ]; then
    response=$(curl -s -X GET "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-040" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-040" \
      -H "X-WIA-Version: 1.0.0" \
      -d "$data")
  fi

  echo "$response"
}

################################################################################
# Commands
################################################################################

cmd_config() {
  print_header
  echo "Configuration Setup"
  echo ""

  read -p "API Endpoint [${API_ENDPOINT}]: " input_endpoint
  if [ -n "$input_endpoint" ]; then
    API_ENDPOINT="$input_endpoint"
  fi

  read -sp "API Key: " input_key
  echo ""
  if [ -n "$input_key" ]; then
    API_KEY="$input_key"
  fi

  save_config
  echo ""
  print_success "Configuration complete!"
}

cmd_list_expeditions() {
  local page="${1:-1}"
  local limit="${2:-10}"

  local path="/api/v1/expeditions?page=$page&limit=$limit"

  print_info "Listing expeditions..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_expedition() {
  local expedition_id="${1:-}"

  if [ -z "$expedition_id" ]; then
    echo "Usage: $0 expeditions get EXPEDITION_ID"
    exit 1
  fi

  print_info "Fetching expedition: $expedition_id"
  local response=$(api_request "GET" "/api/v1/expeditions/$expedition_id")

  pretty_json "$response"
}

cmd_list_dives() {
  local expedition_id="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  if [ -z "$expedition_id" ]; then
    echo "Usage: $0 dives list EXPEDITION_ID [PAGE] [LIMIT]"
    exit 1
  fi

  local path="/api/v1/expeditions/$expedition_id/dives?page=$page&limit=$limit"

  print_info "Listing dives for expedition: $expedition_id"
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_dive() {
  local dive_id="${1:-}"

  if [ -z "$dive_id" ]; then
    echo "Usage: $0 dives get DIVE_ID"
    exit 1
  fi

  print_info "Fetching dive: $dive_id"
  local response=$(api_request "GET" "/api/v1/dives/$dive_id")

  pretty_json "$response"
}

cmd_list_observations() {
  local depth_zone="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  local path="/api/v1/observations?page=$page&limit=$limit"
  if [ -n "$depth_zone" ]; then
    path="$path&depthZone=$depth_zone"
  fi

  print_info "Listing organism observations..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_observation() {
  local observation_id="${1:-}"

  if [ -z "$observation_id" ]; then
    echo "Usage: $0 observations get OBSERVATION_ID"
    exit 1
  fi

  print_info "Fetching observation: $observation_id"
  local response=$(api_request "GET" "/api/v1/observations/$observation_id")

  pretty_json "$response"
}

cmd_list_samples() {
  local page="${1:-1}"
  local limit="${2:-10}"

  local path="/api/v1/samples?page=$page&limit=$limit"

  print_info "Listing biological samples..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_sample() {
  local sample_id="${1:-}"

  if [ -z "$sample_id" ]; then
    echo "Usage: $0 samples get SAMPLE_ID"
    exit 1
  fi

  print_info "Fetching sample: $sample_id"
  local response=$(api_request "GET" "/api/v1/samples/$sample_id")

  pretty_json "$response"
}

cmd_list_vents() {
  local region="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  local path="/api/v1/vents?page=$page&limit=$limit"
  if [ -n "$region" ]; then
    path="$path&region=$region"
  fi

  print_info "Listing hydrothermal vents..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_get_vent() {
  local vent_id="${1:-}"

  if [ -z "$vent_id" ]; then
    echo "Usage: $0 vents get VENT_ID"
    exit 1
  fi

  print_info "Fetching vent: $vent_id"
  local response=$(api_request "GET" "/api/v1/vents/$vent_id")

  pretty_json "$response"
}

cmd_list_submersibles() {
  local type="${1:-}"
  local page="${2:-1}"
  local limit="${3:-10}"

  local path="/api/v1/submersibles?page=$page&limit=$limit"
  if [ -n "$type" ]; then
    path="$path&type=$type"
  fi

  print_info "Listing submersibles..."
  local response=$(api_request "GET" "$path")

  pretty_json "$response"
}

cmd_search() {
  local query="${1:-}"

  if [ -z "$query" ]; then
    echo "Usage: $0 search QUERY"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "query": "$query"
}
EOF
)

  print_info "Searching for: $query"
  local response=$(api_request "POST" "/api/v1/search" "$data")

  pretty_json "$response"
}

cmd_stats() {
  print_info "Fetching statistics..."
  local response=$(api_request "GET" "/api/v1/statistics")

  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}expeditions list${NC} [PAGE] [LIMIT]
    List research expeditions

  ${GREEN}expeditions get${NC} EXPEDITION_ID
    Get expedition details

  ${GREEN}dives list${NC} EXPEDITION_ID [PAGE] [LIMIT]
    List dives for an expedition

  ${GREEN}dives get${NC} DIVE_ID
    Get dive details

  ${GREEN}observations list${NC} [DEPTH_ZONE] [PAGE] [LIMIT]
    List organism observations
    Depth zones: hadopelagic, abyssopelagic, bathypelagic, mesopelagic

  ${GREEN}observations get${NC} OBSERVATION_ID
    Get observation details

  ${GREEN}samples list${NC} [PAGE] [LIMIT]
    List biological samples

  ${GREEN}samples get${NC} SAMPLE_ID
    Get sample details

  ${GREEN}vents list${NC} [REGION] [PAGE] [LIMIT]
    List hydrothermal vents

  ${GREEN}vents get${NC} VENT_ID
    Get vent details

  ${GREEN}submersibles list${NC} [TYPE] [PAGE] [LIMIT]
    List submersible vehicles
    Types: hov, rov, auv, lander

  ${GREEN}search${NC} QUERY
    Search across all data

  ${GREEN}stats${NC}
    View database statistics

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-040/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # List expeditions
  $0 expeditions list

  # Get expedition details
  $0 expeditions get EXP-2025-001

  # List dives
  $0 dives list EXP-2025-001

  # List hadal zone observations
  $0 observations list hadopelagic

  # List hydrothermal vents
  $0 vents list

  # Search for species
  $0 search "Hirondellea gigas"

  # View statistics
  $0 stats

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-040
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-040

${BLUE}════════════════════════════════════════════════════════════════${NC}
${GREEN}         弘익人間 (홍익인간) · Benefit All Humanity${NC}
${BLUE}════════════════════════════════════════════════════════════════${NC}
EOF
}

################################################################################
# Main
################################################################################

# Load configuration
load_config

# Parse command
COMMAND="${1:-help}"
SUBCOMMAND="${2:-}"
shift 2 &>/dev/null || shift &>/dev/null || true

case "$COMMAND" in
  config)
    cmd_config
    ;;
  expeditions)
    case "$SUBCOMMAND" in
      list) cmd_list_expeditions "$@" ;;
      get) cmd_get_expedition "$@" ;;
      *)
        echo "Usage: $0 expeditions {list|get} [options]"
        exit 1
        ;;
    esac
    ;;
  dives)
    case "$SUBCOMMAND" in
      list) cmd_list_dives "$@" ;;
      get) cmd_get_dive "$@" ;;
      *)
        echo "Usage: $0 dives {list|get} [options]"
        exit 1
        ;;
    esac
    ;;
  observations)
    case "$SUBCOMMAND" in
      list) cmd_list_observations "$@" ;;
      get) cmd_get_observation "$@" ;;
      *)
        echo "Usage: $0 observations {list|get} [options]"
        exit 1
        ;;
    esac
    ;;
  samples)
    case "$SUBCOMMAND" in
      list) cmd_list_samples "$@" ;;
      get) cmd_get_sample "$@" ;;
      *)
        echo "Usage: $0 samples {list|get} [options]"
        exit 1
        ;;
    esac
    ;;
  vents)
    case "$SUBCOMMAND" in
      list) cmd_list_vents "$@" ;;
      get) cmd_get_vent "$@" ;;
      *)
        echo "Usage: $0 vents {list|get} [options]"
        exit 1
        ;;
    esac
    ;;
  submersibles)
    case "$SUBCOMMAND" in
      list) cmd_list_submersibles "$@" ;;
      *)
        echo "Usage: $0 submersibles list [TYPE] [PAGE] [LIMIT]"
        exit 1
        ;;
    esac
    ;;
  search)
    cmd_search "$SUBCOMMAND" "$@"
    ;;
  stats)
    cmd_stats
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-040 Deep Sea Exploration CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
