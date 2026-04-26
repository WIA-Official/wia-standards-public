#!/bin/bash

################################################################################
# WIA-CORE-004: Interoperability Registry CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to the WIA Interoperability
# Registry including system registration, discovery, compliance verification,
# and integration template generation.
################################################################################

set -e

# Colors for output
INDIGO='\033[38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
DEFAULT_ENDPOINT="https://registry.wiastandards.com"
API_VERSION="v1"

# Configuration
REGISTRY_ENDPOINT="${WIA_REGISTRY_ENDPOINT:-$DEFAULT_ENDPOINT}"
API_KEY="${WIA_REGISTRY_API_KEY:-}"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     📚 WIA-CORE-004: Interoperability Registry CLI            ║"
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

print_result() {
    echo -e "${BLUE}→ $1${RESET}"
}

# API request helper
api_request() {
    local method=$1
    local endpoint=$2
    local data=$3

    local url="${REGISTRY_ENDPOINT}/api/${API_VERSION}${endpoint}"
    local headers=(-H "Content-Type: application/json" -H "User-Agent: WIA-CLI/1.0.0")

    if [ -n "$API_KEY" ]; then
        headers+=(-H "X-API-Key: $API_KEY")
    fi

    if [ "$method" = "GET" ]; then
        curl -s -X GET "${headers[@]}" "$url"
    elif [ "$method" = "POST" ]; then
        curl -s -X POST "${headers[@]}" -d "$data" "$url"
    elif [ "$method" = "PUT" ]; then
        curl -s -X PUT "${headers[@]}" -d "$data" "$url"
    elif [ "$method" = "DELETE" ]; then
        curl -s -X DELETE "${headers[@]}" "$url"
    fi
}

# Check if jq is available
check_jq() {
    if ! command -v jq &> /dev/null; then
        print_error "jq is required but not installed"
        print_info "Install: sudo apt-get install jq (Debian/Ubuntu)"
        print_info "Install: brew install jq (macOS)"
        exit 1
    fi
}

# Check if API key is configured
check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_warning "No API key configured"
        print_info "Set WIA_REGISTRY_API_KEY environment variable"
        print_info "export WIA_REGISTRY_API_KEY='your-api-key'"
        return 1
    fi
    return 0
}

# ============================================================================
# System Management Commands
# ============================================================================

cmd_register() {
    print_section "Register New System"

    check_jq
    check_api_key || exit 1

    local name=""
    local version=""
    local org_name=""
    local org_email=""
    local standard=""
    local endpoint=""
    local auth="API-Key"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --name) name="$2"; shift 2 ;;
            --version) version="$2"; shift 2 ;;
            --org-name) org_name="$2"; shift 2 ;;
            --org-email) org_email="$2"; shift 2 ;;
            --standard) standard="$2"; shift 2 ;;
            --endpoint) endpoint="$2"; shift 2 ;;
            --auth) auth="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    # Validate required fields
    if [ -z "$name" ] || [ -z "$version" ] || [ -z "$org_name" ] || [ -z "$org_email" ]; then
        print_error "Missing required fields"
        print_info "Usage: wia-core-004 register --name NAME --version VERSION --org-name ORG --org-email EMAIL"
        exit 1
    fi

    # Build registration JSON
    local registration=$(cat <<EOF
{
  "name": "$name",
  "version": "$version",
  "organization": {
    "name": "$org_name",
    "contact": {
      "email": "$org_email"
    }
  },
  "description": "System registered via CLI",
  "standards": [
    {
      "standardId": "${standard:-WIA-CORE-001}",
      "version": "1.0.0",
      "complianceLevel": "documented"
    }
  ],
  "endpoints": [
    {
      "type": "REST",
      "url": "${endpoint:-https://api.example.com}",
      "protocol": "https",
      "authentication": {
        "methods": ["$auth"]
      },
      "documentation": {},
      "availability": {
        "sla": 99.0,
        "regions": ["global"]
      }
    }
  ],
  "capabilities": {
    "features": [],
    "protocols": ["REST"],
    "dataFormats": ["JSON"],
    "authentication": ["$auth"],
    "compliance": {}
  },
  "dataResidency": {
    "regions": ["global"],
    "customersControlLocation": false
  }
}
EOF
)

    print_info "Registering system..."
    local response=$(api_request POST "/systems/register" "$registration")

    if echo "$response" | jq -e '.systemId' > /dev/null 2>&1; then
        local system_id=$(echo "$response" | jq -r '.systemId')
        print_success "System registered successfully"
        print_result "System ID: $system_id"
    else
        print_error "Registration failed"
        echo "$response" | jq '.'
        exit 1
    fi
}

cmd_get_system() {
    print_section "Get System Details"

    check_jq

    local system_id=$1

    if [ -z "$system_id" ]; then
        print_error "System ID required"
        print_info "Usage: wia-core-004 get-system SYSTEM_ID"
        exit 1
    fi

    print_info "Fetching system details..."
    local response=$(api_request GET "/systems/$system_id")

    if echo "$response" | jq -e '.systemId' > /dev/null 2>&1; then
        print_success "System found"
        echo "$response" | jq '.'
    else
        print_error "System not found"
        exit 1
    fi
}

cmd_list_systems() {
    print_section "List Systems"

    check_jq

    local category=""
    local limit=10

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --category) category="$2"; shift 2 ;;
            --limit) limit="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    local endpoint="/systems?limit=$limit"
    if [ -n "$category" ]; then
        endpoint="${endpoint}&category=$category"
    fi

    print_info "Fetching systems..."
    local response=$(api_request GET "$endpoint")

    if echo "$response" | jq -e '.systems' > /dev/null 2>&1; then
        local total=$(echo "$response" | jq -r '.total')
        print_success "Found $total systems"
        echo "$response" | jq -r '.systems[] | "\(.systemId) - \(.name) (\(.version))"'
    else
        print_error "Failed to fetch systems"
        exit 1
    fi
}

# ============================================================================
# Discovery Commands
# ============================================================================

cmd_discover() {
    print_section "Discover Compatible Systems"

    check_jq

    local standard=""
    local capability=""
    local min_compliance="documented"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --standard) standard="$2"; shift 2 ;;
            --capability) capability="$2"; shift 2 ;;
            --min-compliance) min_compliance="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$standard" ]; then
        print_error "Standard ID required"
        print_info "Usage: wia-core-004 discover --standard STANDARD_ID [--capability FEATURE]"
        exit 1
    fi

    # Build query JSON
    local query=$(cat <<EOF
{
  "standards": [
    {
      "id": "$standard",
      "minComplianceLevel": "$min_compliance"
    }
  ]
}
EOF
)

    if [ -n "$capability" ]; then
        query=$(echo "$query" | jq ".capabilities.features = [\"$capability\"]")
    fi

    print_info "Searching for compatible systems..."
    local response=$(api_request POST "/discovery/search" "$query")

    if echo "$response" | jq -e '.results' > /dev/null 2>&1; then
        local total=$(echo "$response" | jq -r '.total')
        print_success "Found $total compatible systems"

        echo "$response" | jq -r '.results[] | "
System: \(.name)
ID: \(.systemId)
Score: \(.score)
Standards: \(.matchedStandards | join(", "))
---"'
    else
        print_error "Discovery failed"
        exit 1
    fi
}

# ============================================================================
# Standards Commands
# ============================================================================

cmd_list_standards() {
    print_section "List Standards"

    check_jq

    local category=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --category) category="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    local endpoint="/standards"
    if [ -n "$category" ]; then
        endpoint="${endpoint}?category=$category"
    fi

    print_info "Fetching standards..."
    local response=$(api_request GET "$endpoint")

    if echo "$response" | jq -e '.standards' > /dev/null 2>&1; then
        local total=$(echo "$response" | jq -r '.total')
        print_success "Found $total standards"

        echo "$response" | jq -r '.standards[] | "
[\(.category)] \(.id) - \(.name)
Version: \(.version) | Status: \(.status)
---"'
    else
        print_error "Failed to fetch standards"
        exit 1
    fi
}

cmd_get_standard() {
    print_section "Get Standard Details"

    check_jq

    local standard_id=$1

    if [ -z "$standard_id" ]; then
        print_error "Standard ID required"
        print_info "Usage: wia-core-004 get-standard STANDARD_ID"
        exit 1
    fi

    print_info "Fetching standard details..."
    local response=$(api_request GET "/standards/$standard_id")

    if echo "$response" | jq -e '.id' > /dev/null 2>&1; then
        print_success "Standard found"
        echo "$response" | jq '.'
    else
        print_error "Standard not found"
        exit 1
    fi
}

# ============================================================================
# Compliance Commands
# ============================================================================

cmd_verify() {
    print_section "Verify Compliance"

    check_jq
    check_api_key || exit 1

    local system_id=""
    local standard_id=""
    local version="1.0.0"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --system-id) system_id="$2"; shift 2 ;;
            --standard) standard_id="$2"; shift 2 ;;
            --version) version="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$system_id" ] || [ -z "$standard_id" ]; then
        print_error "System ID and Standard ID required"
        print_info "Usage: wia-core-004 verify --system-id ID --standard STANDARD_ID"
        exit 1
    fi

    # Build verification request
    local request=$(cat <<EOF
{
  "systemId": "$system_id",
  "standardId": "$standard_id",
  "version": "$version",
  "runTests": true
}
EOF
)

    print_info "Running compliance verification..."
    local response=$(api_request POST "/compliance/verify" "$request")

    if echo "$response" | jq -e '.complianceLevel' > /dev/null 2>&1; then
        local level=$(echo "$response" | jq -r '.complianceLevel')
        local status=$(echo "$response" | jq -r '.overallStatus')
        local score=$(echo "$response" | jq -r '.score')

        print_success "Verification complete"
        print_result "Compliance Level: $level"
        print_result "Overall Status: $status"
        print_result "Score: $score/100"
    else
        print_error "Verification failed"
        echo "$response" | jq '.'
        exit 1
    fi
}

# ============================================================================
# Template Commands
# ============================================================================

cmd_generate_template() {
    print_section "Generate Integration Template"

    check_jq
    check_api_key || exit 1

    local source=""
    local target=""
    local language="typescript"
    local protocol="REST"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --source) source="$2"; shift 2 ;;
            --target) target="$2"; shift 2 ;;
            --language) language="$2"; shift 2 ;;
            --protocol) protocol="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$source" ] || [ -z "$target" ]; then
        print_error "Source and Target system IDs required"
        print_info "Usage: wia-core-004 generate-template --source ID --target ID [--language LANG]"
        exit 1
    fi

    # Build template request
    local request=$(cat <<EOF
{
  "sourceSystem": "$source",
  "targetSystem": "$target",
  "language": "$language",
  "protocol": "$protocol",
  "includeErrorHandling": true,
  "includeRetryLogic": true
}
EOF
)

    print_info "Generating integration template..."
    local response=$(api_request POST "/templates/generate" "$request")

    if echo "$response" | jq -e '.id' > /dev/null 2>&1; then
        local template_id=$(echo "$response" | jq -r '.id')
        print_success "Template generated successfully"
        print_result "Template ID: $template_id"

        echo -e "\n${CYAN}Generated Code:${RESET}"
        echo "$response" | jq -r '.code'
    else
        print_error "Template generation failed"
        exit 1
    fi
}

# ============================================================================
# Utility Commands
# ============================================================================

cmd_version() {
    print_header
    echo -e "${GRAY}Registry Endpoint: $REGISTRY_ENDPOINT${RESET}"
    echo -e "${GRAY}API Version: $API_VERSION${RESET}"
}

cmd_help() {
    print_header

    echo -e "${CYAN}Usage:${RESET}"
    echo -e "  wia-core-004 [command] [options]"
    echo

    echo -e "${CYAN}System Management:${RESET}"
    echo -e "  register          Register a new system"
    echo -e "  get-system        Get system details by ID"
    echo -e "  list-systems      List all systems"
    echo

    echo -e "${CYAN}Discovery:${RESET}"
    echo -e "  discover          Discover compatible systems"
    echo

    echo -e "${CYAN}Standards:${RESET}"
    echo -e "  list-standards    List all standards"
    echo -e "  get-standard      Get standard details"
    echo

    echo -e "${CYAN}Compliance:${RESET}"
    echo -e "  verify            Verify system compliance"
    echo

    echo -e "${CYAN}Templates:${RESET}"
    echo -e "  generate-template Generate integration code"
    echo

    echo -e "${CYAN}Utility:${RESET}"
    echo -e "  version           Show version information"
    echo -e "  help              Show this help message"
    echo

    echo -e "${CYAN}Examples:${RESET}"
    echo -e "  wia-core-004 register --name \"My API\" --version \"1.0.0\" --org-name \"ACME\""
    echo -e "  wia-core-004 discover --standard WIA-MED-001 --capability patient-data"
    echo -e "  wia-core-004 verify --system-id sys-123 --standard WIA-CORE-001"
    echo -e "  wia-core-004 generate-template --source sys-123 --target sys-456 --language typescript"
    echo

    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
}

# ============================================================================
# Main Command Router
# ============================================================================

main() {
    local command=$1
    shift

    case $command in
        register)
            cmd_register "$@"
            ;;
        get-system)
            cmd_get_system "$@"
            ;;
        list-systems)
            cmd_list_systems "$@"
            ;;
        discover)
            cmd_discover "$@"
            ;;
        list-standards)
            cmd_list_standards "$@"
            ;;
        get-standard)
            cmd_get_standard "$@"
            ;;
        verify)
            cmd_verify "$@"
            ;;
        generate-template)
            cmd_generate_template "$@"
            ;;
        --version|-v|version)
            cmd_version
            ;;
        --help|-h|help|"")
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            cmd_help
            exit 1
            ;;
    esac
}

# Run main command
main "$@"
