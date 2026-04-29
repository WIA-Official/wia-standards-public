#!/usr/bin/env bash
################################################################################
# WIA-FINTECH CLI Tool
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
#
# Comprehensive CLI for interacting with Fintech protocols
################################################################################

set -euo pipefail

# Colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly MAGENTA='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# Configuration
readonly VERSION="1.0.0"
readonly API_BASE="${WIA_DEFI_API_BASE:-https://api.wia-fintech.org/v1}"
readonly CONFIG_FILE="${HOME}/.wia-fintech/config.json"

# API Key
WIA_API_KEY="${WIA_API_KEY:-}"

################################################################################
# Helper Functions
################################################################################

print_banner() {
  echo -e "${CYAN}"
  cat << "EOF"
╦ ╦╦╔═╗   ╔╦╗╔═╗╔═╗╦
║║║║╠═╣───║║║╣ ╠╣ ║
╚╩╝╩╩ ╩  ═╩╝╚═╝╚  ╩
EOF
  echo -e "弘益人間 (Benefit All Humanity)${NC}"
  echo -e "Version: ${VERSION}\n"
}

log() {
  echo -e "${GREEN}[INFO]${NC} $*"
}

warn() {
  echo -e "${YELLOW}[WARN]${NC} $*" >&2
}

error() {
  echo -e "${RED}[ERROR]${NC} $*" >&2
  exit 1
}

success() {
  echo -e "${GREEN}[✓]${NC} $*"
}

# Load API key from config or environment
load_api_key() {
  if [[ -n "${WIA_API_KEY}" ]]; then
    return 0
  fi

  if [[ -f "${CONFIG_FILE}" ]]; then
    WIA_API_KEY=$(jq -r '.apiKey // ""' "${CONFIG_FILE}" 2>/dev/null || echo "")
  fi

  if [[ -z "${WIA_API_KEY}" ]]; then
    error "API key not found. Set WIA_API_KEY environment variable or run: wia-fintech config set-key YOUR_API_KEY"
  fi
}

# Make API request
api_request() {
  local method="${1}"
  local endpoint="${2}"
  local data="${3:-}"

  local url="${API_BASE}${endpoint}"
  local curl_args=(-s -X "${method}" -H "X-API-Key: ${WIA_API_KEY}" -H "Content-Type: application/json")

  if [[ -n "${data}" ]]; then
    curl_args+=(-d "${data}")
  fi

  local response
  response=$(curl "${curl_args[@]}" "${url}" 2>&1)
  local exit_code=$?

  if [[ ${exit_code} -ne 0 ]]; then
    error "API request failed: ${response}"
  fi

  echo "${response}"
}

# Format JSON output
format_json() {
  if command -v jq &> /dev/null; then
    jq -C '.'
  else
    cat
  fi
}

# Format large numbers
format_number() {
  local num="${1}"
  if [[ ${num} -ge 1000000000 ]]; then
    echo "$(awk "BEGIN {printf \"%.2fB\", ${num}/1000000000}")"
  elif [[ ${num} -ge 1000000 ]]; then
    echo "$(awk "BEGIN {printf \"%.2fM\", ${num}/1000000}")"
  elif [[ ${num} -ge 1000 ]]; then
    echo "$(awk "BEGIN {printf \"%.2fK\", ${num}/1000}")"
  else
    echo "${num}"
  fi
}

################################################################################
# Configuration Commands
################################################################################

cmd_config() {
  local subcommand="${1:-}"

  case "${subcommand}" in
    set-key)
      local api_key="${2:-}"
      if [[ -z "${api_key}" ]]; then
        error "Usage: wia-fintech config set-key YOUR_API_KEY"
      fi

      mkdir -p "$(dirname "${CONFIG_FILE}")"
      echo "{\"apiKey\": \"${api_key}\"}" > "${CONFIG_FILE}"
      chmod 600 "${CONFIG_FILE}"
      success "API key saved to ${CONFIG_FILE}"
      ;;
    show)
      if [[ -f "${CONFIG_FILE}" ]]; then
        cat "${CONFIG_FILE}" | format_json
      else
        warn "No configuration file found"
      fi
      ;;
    *)
      echo "Usage: wia-fintech config {set-key|show}"
      exit 1
      ;;
  esac
}

################################################################################
# Protocol Commands
################################################################################

cmd_protocol() {
  local subcommand="${1:-list}"

  case "${subcommand}" in
    list)
      log "Fetching protocols..."
      local response
      response=$(api_request GET "/protocols")

      echo "${response}" | jq -r '.data[] |
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
        "Protocol: \(.name) (\(.protocolId))\n" +
        "Version: \(.version)\n" +
        "Chain: \(.chainId)\n" +
        "TVL: $\(.tvl)\n" +
        "24h Volume: $\(.volume24h)\n" +
        "Active Users: \(.activeUsers24h)\n" +
        "Markets: \(.markets)"'
      ;;
    info)
      local protocol_id="${2:-}"
      if [[ -z "${protocol_id}" ]]; then
        error "Usage: wia-fintech protocol info PROTOCOL_ID"
      fi

      log "Fetching protocol info for: ${protocol_id}"
      local response
      response=$(api_request GET "/protocols/${protocol_id}")

      echo "${response}" | jq '.data'
      ;;
    *)
      echo "Usage: wia-fintech protocol {list|info}"
      exit 1
      ;;
  esac
}

################################################################################
# Pool Commands
################################################################################

cmd_pool() {
  local subcommand="${1:-list}"

  case "${subcommand}" in
    list)
      local protocol="${2:-}"
      local query=""
      if [[ -n "${protocol}" ]]; then
        query="?protocol=${protocol}"
      fi

      log "Fetching pools..."
      local response
      response=$(api_request GET "/pools${query}")

      echo "${response}" | jq -r '.data.data[] |
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
        "Pool: \(.token0.symbol)/\(.token1.symbol)\n" +
        "Protocol: \(.protocol)\n" +
        "TVL: $\(.tvl)\n" +
        "24h Volume: $\(.volume24h)\n" +
        "APR: \(.apr)%\n" +
        "Fee Tier: \(.feeTier / 10000)%"'
      ;;
    info)
      local pool_id="${2:-}"
      if [[ -z "${pool_id}" ]]; then
        error "Usage: wia-fintech pool info POOL_ID"
      fi

      log "Fetching pool info for: ${pool_id}"
      local response
      response=$(api_request GET "/pools/${pool_id}")

      echo "${response}" | jq '.data'
      ;;
    top)
      log "Fetching top pools by TVL..."
      local response
      response=$(api_request GET "/pools?sortBy=tvl&order=desc&limit=10")

      echo -e "\n${CYAN}Top 10 Pools by TVL${NC}\n"
      echo "${response}" | jq -r '.data.data[] |
        "\(.token0.symbol)/\(.token1.symbol) - $\(.tvl) TVL - \(.apr)% APR"'
      ;;
    *)
      echo "Usage: wia-fintech pool {list|info|top} [PROTOCOL]"
      exit 1
      ;;
  esac
}

################################################################################
# Swap Commands
################################################################################

cmd_swap() {
  local subcommand="${1:-quote}"

  case "${subcommand}" in
    quote)
      local chain_id="${2:-1}"
      local token_in="${3:-}"
      local token_out="${4:-}"
      local amount_in="${5:-}"
      local slippage="${6:-0.5}"

      if [[ -z "${token_in}" ]] || [[ -z "${token_out}" ]] || [[ -z "${amount_in}" ]]; then
        error "Usage: wia-fintech swap quote CHAIN_ID TOKEN_IN TOKEN_OUT AMOUNT_IN [SLIPPAGE]"
      fi

      log "Getting swap quote..."
      local data
      data=$(jq -n \
        --arg chainId "${chain_id}" \
        --arg tokenIn "${token_in}" \
        --arg tokenOut "${token_out}" \
        --arg amountIn "${amount_in}" \
        --arg slippage "${slippage}" \
        '{
          chainId: ($chainId | tonumber),
          tokenIn: $tokenIn,
          tokenOut: $tokenOut,
          amountIn: $amountIn,
          slippageTolerance: ($slippage | tonumber)
        }')

      local response
      response=$(api_request POST "/swap/quote" "${data}")

      echo -e "\n${CYAN}Swap Quote${NC}\n"
      echo "${response}" | jq -r '.data |
        "From: \(.tokenIn.symbol) (\(.amountIn))\n" +
        "To: \(.tokenOut.symbol) (\(.amountOut))\n" +
        "Price Impact: \(.priceImpact)%\n" +
        "Gas Cost: $\(.gasCostUsd)\n" +
        "Route: \(.route | length) step(s)\n" +
        "Expires: \(.expiresAt)"'

      echo -e "\n${YELLOW}Route Details:${NC}"
      echo "${response}" | jq -r '.data.route[] |
        "  \(.protocol): \(.tokenIn) → \(.tokenOut) (\(.percentage)%)"'
      ;;
    recent)
      local pool_id="${2:-}"
      if [[ -z "${pool_id}" ]]; then
        error "Usage: wia-fintech swap recent POOL_ID"
      fi

      log "Fetching recent swaps for pool: ${pool_id}"
      local response
      response=$(api_request GET "/pools/${pool_id}/swaps?limit=10")

      echo -e "\n${CYAN}Recent Swaps${NC}\n"
      echo "${response}" | jq -r '.data[] |
        "\(.timestamp) - $\(.amountUsd) - \(.sender[:10])..."'
      ;;
    *)
      echo "Usage: wia-fintech swap {quote|recent}"
      exit 1
      ;;
  esac
}

################################################################################
# Stake Commands
################################################################################

cmd_stake() {
  local subcommand="${1:-list}"

  case "${subcommand}" in
    list)
      log "Fetching staking pools..."
      local response
      response=$(api_request GET "/staking/pools")

      echo "${response}" | jq -r '.data[] |
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
        "Pool: \(.protocol) - \(.asset.symbol)\n" +
        "APR: \(.apr)%\n" +
        "Total Staked: \(.totalStaked)\n" +
        "Min Stake: \(.minStake // "None")\n" +
        "Lock Period: \(if .lockPeriod then "\(.lockPeriod / 86400) days" else "None" end)"'
      ;;
    info)
      local pool_id="${2:-}"
      if [[ -z "${pool_id}" ]]; then
        error "Usage: wia-fintech stake info POOL_ID"
      fi

      log "Fetching staking pool info for: ${pool_id}"
      local response
      response=$(api_request GET "/staking/pools/${pool_id}")

      echo "${response}" | jq '.data'
      ;;
    *)
      echo "Usage: wia-fintech stake {list|info}"
      exit 1
      ;;
  esac
}

################################################################################
# Lending Commands
################################################################################

cmd_lend() {
  local subcommand="${1:-markets}"

  case "${subcommand}" in
    markets)
      local protocol="${2:-}"
      local query=""
      if [[ -n "${protocol}" ]]; then
        query="?protocol=${protocol}"
      fi

      log "Fetching lending markets..."
      local response
      response=$(api_request GET "/lending/markets${query}")

      echo "${response}" | jq -r '.data[] |
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
        "Market: \(.asset.symbol)\n" +
        "Protocol: \(.protocol)\n" +
        "Supply APY: \(.supplyApy)%\n" +
        "Borrow APY: \(.borrowApy)%\n" +
        "Utilization: \(.utilizationRate)%\n" +
        "Total Supplied: \(.totalSupplied)\n" +
        "Total Borrowed: \(.totalBorrowed)\n" +
        "Collateral Factor: \(.collateralFactor)%"'
      ;;
    info)
      local market_id="${2:-}"
      if [[ -z "${market_id}" ]]; then
        error "Usage: wia-fintech lend info MARKET_ID"
      fi

      log "Fetching market info for: ${market_id}"
      local response
      response=$(api_request GET "/lending/markets/${market_id}")

      echo "${response}" | jq '.data'
      ;;
    *)
      echo "Usage: wia-fintech lend {markets|info} [PROTOCOL]"
      exit 1
      ;;
  esac
}

################################################################################
# User Commands
################################################################################

cmd_user() {
  local subcommand="${1:-positions}"

  case "${subcommand}" in
    positions)
      local address="${2:-}"
      if [[ -z "${address}" ]]; then
        error "Usage: wia-fintech user positions ADDRESS"
      fi

      log "Fetching positions for: ${address}"
      local response
      response=$(api_request GET "/positions?address=${address}")

      echo -e "\n${CYAN}User Positions${NC}\n"
      echo "${response}" | jq -r '.data[] |
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
        "Type: \(.type)\n" +
        "Protocol: \(.protocol)\n" +
        "Value: $\(.valueUsd)\n" +
        "Created: \(.createdAt)"'
      ;;
    *)
      echo "Usage: wia-fintech user {positions} ADDRESS"
      exit 1
      ;;
  esac
}

################################################################################
# Stats Commands
################################################################################

cmd_stats() {
  local chain_id="${1:-}"
  local query=""
  if [[ -n "${chain_id}" ]]; then
    query="?chainId=${chain_id}"
  fi

  log "Fetching network statistics..."
  local response
  response=$(api_request GET "/stats${query}")

  echo -e "\n${CYAN}Network Statistics${NC}\n"
  echo "${response}" | jq -r '.data |
    "Total TVL: $\(.totalTvl)\n" +
    "24h Volume: $\(.totalVolume24h)\n" +
    "Total Users: \(.totalUsers)\n" +
    "Protocol Count: \(.protocolCount)"'
}

################################################################################
# Token Commands
################################################################################

cmd_token() {
  local subcommand="${1:-search}"

  case "${subcommand}" in
    search)
      local query="${2:-}"
      if [[ -z "${query}" ]]; then
        error "Usage: wia-fintech token search QUERY [CHAIN_ID]"
      fi

      local chain_id="${3:-}"
      local url="/tokens/search?query=${query}"
      if [[ -n "${chain_id}" ]]; then
        url="${url}&chainId=${chain_id}"
      fi

      log "Searching for tokens: ${query}"
      local response
      response=$(api_request GET "${url}")

      echo "${response}" | jq -r '.data[] |
        "\(.symbol) (\(.name))\n  Address: \(.address)\n  Price: $\(.priceUsd)"'
      ;;
    info)
      local address="${2:-}"
      local chain_id="${3:-1}"
      if [[ -z "${address}" ]]; then
        error "Usage: wia-fintech token info ADDRESS [CHAIN_ID]"
      fi

      log "Fetching token info..."
      local response
      response=$(api_request GET "/tokens/${chain_id}/${address}")

      echo "${response}" | jq '.data'
      ;;
    *)
      echo "Usage: wia-fintech token {search|info}"
      exit 1
      ;;
  esac
}

################################################################################
# Help Command
################################################################################

cmd_help() {
  print_banner
  cat << EOF
${CYAN}Usage:${NC}
  wia-fintech [command] [subcommand] [arguments]

${CYAN}Commands:${NC}
  ${GREEN}config${NC}         Configuration management
    set-key KEY    Set API key
    show           Show current configuration

  ${GREEN}protocol${NC}       Protocol information
    list           List all protocols
    info ID        Get protocol details

  ${GREEN}pool${NC}           Liquidity pool operations
    list [PROTO]   List pools (optionally filter by protocol)
    info ID        Get pool details
    top            Show top pools by TVL

  ${GREEN}swap${NC}           Swap operations
    quote CHAIN TOKEN_IN TOKEN_OUT AMOUNT [SLIPPAGE]
                   Get swap quote
    recent POOL    Show recent swaps

  ${GREEN}stake${NC}          Staking operations
    list           List staking pools
    info ID        Get staking pool details

  ${GREEN}lend${NC}           Lending operations
    markets [PROTO] List lending markets
    info ID        Get market details

  ${GREEN}user${NC}           User operations
    positions ADDR Get user positions

  ${GREEN}token${NC}          Token operations
    search QUERY [CHAIN]  Search tokens
    info ADDR [CHAIN]     Get token info

  ${GREEN}stats${NC}          Network statistics
    [CHAIN_ID]     Get stats (optionally for specific chain)

  ${GREEN}help${NC}           Show this help message
  ${GREEN}version${NC}        Show version information

${CYAN}Examples:${NC}
  # Configure API key
  wia-fintech config set-key YOUR_API_KEY

  # List all protocols
  wia-fintech protocol list

  # Get swap quote
  wia-fintech swap quote 1 \\
    0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48 \\
    0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2 \\
    1000000000 0.5

  # List staking pools
  wia-fintech stake list

  # Get user positions
  wia-fintech user positions 0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb

${CYAN}Environment Variables:${NC}
  WIA_API_KEY         API key for authentication
  WIA_DEFI_API_BASE   API base URL (default: https://api.wia-fintech.org/v1)

${CYAN}Configuration:${NC}
  Config file: ${CONFIG_FILE}

${CYAN}Philosophy:${NC}
  弘益人間 (Hongik Ingan) - Benefit All Humanity

${CYAN}More Information:${NC}
  Documentation: https://docs.wia.org/fintech
  GitHub: https://github.com/WIA-Official/wia-standards
  Support: dev@wia.org

EOF
}

cmd_version() {
  echo "WIA-FINTECH CLI v${VERSION}"
  echo "弘益人間 (Benefit All Humanity)"
}

################################################################################
# Main
################################################################################

main() {
  local command="${1:-help}"

  case "${command}" in
    config)
      shift
      cmd_config "$@"
      ;;
    protocol)
      load_api_key
      shift
      cmd_protocol "$@"
      ;;
    pool)
      load_api_key
      shift
      cmd_pool "$@"
      ;;
    swap)
      load_api_key
      shift
      cmd_swap "$@"
      ;;
    stake)
      load_api_key
      shift
      cmd_stake "$@"
      ;;
    lend)
      load_api_key
      shift
      cmd_lend "$@"
      ;;
    user)
      load_api_key
      shift
      cmd_user "$@"
      ;;
    token)
      load_api_key
      shift
      cmd_token "$@"
      ;;
    stats)
      load_api_key
      shift
      cmd_stats "$@"
      ;;
    help|--help|-h)
      cmd_help
      ;;
    version|--version|-v)
      cmd_version
      ;;
    *)
      error "Unknown command: ${command}. Run 'wia-fintech help' for usage."
      ;;
  esac
}

main "$@"
