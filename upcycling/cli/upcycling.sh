#!/bin/bash

#############################################################################
# WIA-ENE-024: Upcycling Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Command-line interface for managing upcycling projects
#
# Usage:
#   upcycling <command> [options]
#
# Commands:
#   create         Create a new upcycling project
#   list           List upcycling projects
#   get            Get project details
#   update         Update project information
#   delete         Delete a project
#   calculate      Calculate environmental impact
#   stats          Show overall statistics
#   version        Show version information
#   help           Show this help message
#
# @version 1.0.0
# @license MIT
#############################################################################

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-024/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/upcycling.conf"

#############################################################################
# Helper Functions
#############################################################################

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                                                                ║"
    echo "║        WIA-ENE-024: Upcycling Standard CLI 🔄                 ║"
    echo "║                                                                ║"
    echo "║        弘益人間 (홍익인간) - Benefit All Humanity              ║"
    echo "║        Version: $VERSION                                          ║"
    echo "║                                                                ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ Error: $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠ Warning: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

#############################################################################
# API Functions
#############################################################################

api_request() {
    local method="$1"
    local path="$2"
    local data="$3"

    local url="${API_ENDPOINT}${path}"
    local headers=(-H "Content-Type: application/json" -H "Accept: application/json")

    if [ -n "$API_KEY" ]; then
        headers+=(-H "Authorization: Bearer $API_KEY")
    fi

    if [ "$method" = "GET" ]; then
        curl -s -X "$method" "${headers[@]}" "$url"
    else
        curl -s -X "$method" "${headers[@]}" -d "$data" "$url"
    fi
}

#############################################################################
# Commands
#############################################################################

cmd_create() {
    local project_name=""
    local material_type=""
    local material_weight=""
    local product_name=""
    local product_type=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --name)
                project_name="$2"
                shift 2
                ;;
            --material-type)
                material_type="$2"
                shift 2
                ;;
            --material-weight)
                material_weight="$2"
                shift 2
                ;;
            --product-name)
                product_name="$2"
                shift 2
                ;;
            --product-type)
                product_type="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Validate required fields
    if [ -z "$project_name" ]; then
        print_error "Project name is required (--name)"
        exit 1
    fi

    if [ -z "$material_type" ]; then
        print_error "Material type is required (--material-type)"
        print_info "Available types: textile, plastic, metal, wood, glass, paper, e_waste, construction, rubber, ceramic, leather, mixed"
        exit 1
    fi

    # Build JSON request
    local json_data=$(cat <<EOF
{
  "projectName": "$project_name",
  "sourceMaterial": {
    "materialType": "$material_type",
    "quantity": 1,
    "weight": ${material_weight:-1},
    "weightUnit": "kg",
    "condition": "B",
    "sourceChannel": "cli",
    "acquisitionCost": 0,
    "currency": "KRW"
  },
  "outputProduct": {
    "productType": "${product_type:-other}",
    "productName": "${product_name:-$project_name}",
    "quantity": 1
  }
}
EOF
    )

    print_info "Creating project: $project_name"

    local response=$(api_request "POST" "/api/v1/upcycling/projects" "$json_data")

    if echo "$response" | grep -q '"success":true'; then
        local project_id=$(echo "$response" | grep -o '"projectId":"[^"]*"' | cut -d'"' -f4)
        print_success "Project created successfully!"
        print_info "Project ID: $project_id"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
    else
        print_error "Failed to create project"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_list() {
    local status=""
    local material_type=""
    local limit=10
    local offset=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --status)
                status="$2"
                shift 2
                ;;
            --material-type)
                material_type="$2"
                shift 2
                ;;
            --limit)
                limit="$2"
                shift 2
                ;;
            --offset)
                offset="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Build query parameters
    local query="?limit=$limit&offset=$offset"
    [ -n "$status" ] && query="${query}&status=$status"
    [ -n "$material_type" ] && query="${query}&materialType=$material_type"

    print_info "Fetching projects..."

    local response=$(api_request "GET" "/api/v1/upcycling/projects${query}" "")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Projects retrieved successfully"
        echo "$response" | jq '.data.projects[] | {id: .projectId, name: .projectName, status: .status, material: .sourceMaterial.materialType}' 2>/dev/null || echo "$response"
    else
        print_error "Failed to fetch projects"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_get() {
    local project_id="$1"

    if [ -z "$project_id" ]; then
        print_error "Project ID is required"
        echo "Usage: upcycling get <project-id>"
        exit 1
    fi

    print_info "Fetching project: $project_id"

    local response=$(api_request "GET" "/api/v1/upcycling/projects/${project_id}" "")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Project details:"
        echo "$response" | jq '.data' 2>/dev/null || echo "$response"
    else
        print_error "Failed to fetch project"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_update() {
    local project_id=""
    local status=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id)
                project_id="$2"
                shift 2
                ;;
            --status)
                status="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [ -z "$project_id" ]; then
        print_error "Project ID is required (--id)"
        exit 1
    fi

    if [ -z "$status" ]; then
        print_error "Status is required (--status)"
        print_info "Available statuses: planning, sourcing, in_progress, qa, completed, sold, cancelled"
        exit 1
    fi

    local json_data="{\"status\": \"$status\"}"

    print_info "Updating project: $project_id"

    local response=$(api_request "PATCH" "/api/v1/upcycling/projects/${project_id}" "$json_data")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Project updated successfully"
        echo "$response" | jq '.data' 2>/dev/null || echo "$response"
    else
        print_error "Failed to update project"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_delete() {
    local project_id="$1"

    if [ -z "$project_id" ]; then
        print_error "Project ID is required"
        echo "Usage: upcycling delete <project-id>"
        exit 1
    fi

    read -p "Are you sure you want to delete project $project_id? (yes/no): " confirm
    if [ "$confirm" != "yes" ]; then
        print_info "Delete cancelled"
        exit 0
    fi

    print_info "Deleting project: $project_id"

    local response=$(api_request "DELETE" "/api/v1/upcycling/projects/${project_id}" "")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Project deleted successfully"
    else
        print_error "Failed to delete project"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_calculate() {
    local project_id="$1"

    if [ -z "$project_id" ]; then
        print_error "Project ID is required"
        echo "Usage: upcycling calculate <project-id>"
        exit 1
    fi

    print_info "Calculating environmental impact for: $project_id"

    local response=$(api_request "GET" "/api/v1/impact/report/${project_id}" "")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Environmental impact calculated:"
        echo "$response" | jq '.data' 2>/dev/null || echo "$response"
    else
        print_error "Failed to calculate impact"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_stats() {
    print_info "Fetching statistics..."

    local response=$(api_request "GET" "/api/v1/statistics" "")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Overall Statistics:"
        echo ""
        echo "$response" | jq '.data' 2>/dev/null || echo "$response"
    else
        print_error "Failed to fetch statistics"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        exit 1
    fi
}

cmd_version() {
    print_header
    echo "Version: $VERSION"
    echo "API Endpoint: $API_ENDPOINT"
    echo "Standard: WIA-ENE-024"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""
}

cmd_help() {
    print_header
    cat <<EOF
Usage: upcycling <command> [options]

Commands:
  ${GREEN}create${NC}         Create a new upcycling project
  ${GREEN}list${NC}           List upcycling projects
  ${GREEN}get${NC}            Get project details
  ${GREEN}update${NC}         Update project information
  ${GREEN}delete${NC}         Delete a project
  ${GREEN}calculate${NC}      Calculate environmental impact
  ${GREEN}stats${NC}          Show overall statistics
  ${GREEN}version${NC}        Show version information
  ${GREEN}help${NC}           Show this help message

Examples:
  # Create a new project
  upcycling create --name "청바지 토트백" --material-type textile --material-weight 1.5

  # List all completed projects
  upcycling list --status completed

  # Get project details
  upcycling get UP-2025-001234

  # Update project status
  upcycling update --id UP-2025-001234 --status completed

  # Calculate environmental impact
  upcycling calculate UP-2025-001234

  # Show statistics
  upcycling stats

Configuration:
  Set environment variables:
    export WIA_API_KEY="your-api-key"
    export WIA_API_ENDPOINT="https://api.wia.org/ene-024/v1"

  Or create a config file at: ${HOME}/.wia/upcycling.conf

For more information:
  https://docs.wia.org/ene-024
  https://github.com/WIA-Official/wia-standards

弘益人間 (홍익인간) - Benefit All Humanity
EOF
}

#############################################################################
# Main
#############################################################################

main() {
    # Load config file if exists
    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
    fi

    # Check if command is provided
    if [ $# -eq 0 ]; then
        cmd_help
        exit 0
    fi

    # Parse command
    local command="$1"
    shift

    case "$command" in
        create)
            cmd_create "$@"
            ;;
        list)
            cmd_list "$@"
            ;;
        get)
            cmd_get "$@"
            ;;
        update)
            cmd_update "$@"
            ;;
        delete)
            cmd_delete "$@"
            ;;
        calculate|impact)
            cmd_calculate "$@"
            ;;
        stats|statistics)
            cmd_stats "$@"
            ;;
        version|--version|-v)
            cmd_version
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            cmd_help
            exit 1
            ;;
    esac
}

# Run main
main "$@"
