#!/bin/bash

################################################################################
# WIA-CORE-010: Universal Error Handling CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to error handling utilities
# including error code validation, error parsing, and error documentation.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
ERROR_CODE_PATTERN="^WIA-[A-Z0-9]+-[A-Z0-9]+-[0-9]{3}$"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        ⚠️  WIA-CORE-010: Universal Error Handling CLI          ║"
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

# Validate error code format
validate_code() {
    local code="$1"

    print_section "Error Code Validation"
    print_info "Code: $code"

    # Check format
    if echo "$code" | grep -qE "$ERROR_CODE_PATTERN"; then
        print_success "Format: VALID (matches WIA-{DOMAIN}-{CATEGORY}-{NUMBER})"

        # Parse components
        local domain=$(echo "$code" | cut -d'-' -f2)
        local category=$(echo "$code" | cut -d'-' -f3)
        local number=$(echo "$code" | cut -d'-' -f4)

        print_section "Components"
        print_info "Prefix: WIA"
        print_info "Domain: $domain"
        print_info "Category: $category"
        print_info "Number: $number"

        # Determine severity based on category
        print_section "Metadata"
        case "$category" in
            AUTH)
                print_info "HTTP Status: 401 (Unauthorized)"
                print_info "Typical Severity: ERROR"
                print_info "Retryable: Usually No"
                ;;
            VALIDATION)
                print_info "HTTP Status: 400 (Bad Request)"
                print_info "Typical Severity: ERROR"
                print_info "Retryable: No (fix input)"
                ;;
            NETWORK)
                print_info "HTTP Status: 503 (Service Unavailable)"
                print_info "Typical Severity: WARNING"
                print_info "Retryable: Yes (with backoff)"
                ;;
            DATABASE)
                print_info "HTTP Status: 500 (Internal Server Error)"
                print_info "Typical Severity: ERROR"
                print_info "Retryable: Sometimes"
                ;;
            PERMISSION)
                print_info "HTTP Status: 403 (Forbidden)"
                print_info "Typical Severity: ERROR"
                print_info "Retryable: No"
                ;;
            SYSTEM)
                print_info "HTTP Status: 500 (Internal Server Error)"
                print_info "Typical Severity: CRITICAL/ERROR"
                print_info "Retryable: Sometimes"
                ;;
            *)
                print_info "HTTP Status: 500 (Internal Server Error)"
                print_info "Typical Severity: ERROR"
                print_info "Retryable: Unknown"
                ;;
        esac
    else
        print_error "Format: INVALID (expected WIA-{DOMAIN}-{CATEGORY}-{NUMBER})"
        print_info "Examples:"
        print_info "  WIA-CORE-AUTH-001"
        print_info "  WIA-CORE-VALIDATION-100"
        print_info "  WIA-TIME-PARADOX-001"
        return 1
    fi

    echo ""
}

# List error codes
list_codes() {
    local domain="${1:-CORE}"
    local category="${2:-}"

    print_section "Error Code Registry"
    print_info "Domain: $domain"
    if [ -n "$category" ]; then
        print_info "Category: $category"
    fi

    print_section "Standard Error Codes"

    if [ "$domain" == "CORE" ]; then
        if [ -z "$category" ] || [ "$category" == "AUTH" ]; then
            echo -e "${CYAN}Authentication (AUTH: 001-099)${RESET}"
            print_info "WIA-CORE-AUTH-001: Invalid credentials"
            print_info "WIA-CORE-AUTH-002: Token expired"
            print_info "WIA-CORE-AUTH-003: Token invalid"
            print_info "WIA-CORE-AUTH-004: Session expired"
            print_info "WIA-CORE-AUTH-005: MFA required"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "VALIDATION" ]; then
            echo -e "${CYAN}Validation (VALIDATION: 100-199)${RESET}"
            print_info "WIA-CORE-VALIDATION-100: Invalid input"
            print_info "WIA-CORE-VALIDATION-101: Missing required field"
            print_info "WIA-CORE-VALIDATION-102: Invalid format"
            print_info "WIA-CORE-VALIDATION-103: Value out of range"
            print_info "WIA-CORE-VALIDATION-104: Type mismatch"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "NETWORK" ]; then
            echo -e "${CYAN}Network (NETWORK: 200-299)${RESET}"
            print_info "WIA-CORE-NETWORK-200: Network timeout"
            print_info "WIA-CORE-NETWORK-201: Connection refused"
            print_info "WIA-CORE-NETWORK-202: DNS resolution failed"
            print_info "WIA-CORE-NETWORK-502: Bad gateway"
            print_info "WIA-CORE-NETWORK-504: Gateway timeout"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "DATABASE" ]; then
            echo -e "${CYAN}Database (DATABASE: 300-399)${RESET}"
            print_info "WIA-CORE-DATABASE-300: Database connection failed"
            print_info "WIA-CORE-DATABASE-301: Query failed"
            print_info "WIA-CORE-DATABASE-302: Transaction failed"
            print_info "WIA-CORE-DATABASE-303: Constraint violation"
            print_info "WIA-CORE-DATABASE-304: Record not found"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "RESOURCE" ]; then
            echo -e "${CYAN}Resource (RESOURCE: 400-499)${RESET}"
            print_info "WIA-CORE-RESOURCE-400: Resource not found"
            print_info "WIA-CORE-RESOURCE-429: Rate limit exceeded"
            print_info "WIA-CORE-RESOURCE-503: Service unavailable"
            print_info "WIA-CORE-RESOURCE-507: Insufficient storage"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "PERMISSION" ]; then
            echo -e "${CYAN}Permission (PERMISSION: 500-599)${RESET}"
            print_info "WIA-CORE-PERMISSION-500: Insufficient permissions"
            print_info "WIA-CORE-PERMISSION-501: Access denied"
            print_info "WIA-CORE-PERMISSION-502: Role required"
            print_info "WIA-CORE-PERMISSION-503: Resource locked"
            echo ""
        fi

        if [ -z "$category" ] || [ "$category" == "SYSTEM" ]; then
            echo -e "${CYAN}System (SYSTEM: 600-699)${RESET}"
            print_info "WIA-CORE-SYSTEM-600: Internal server error"
            print_info "WIA-CORE-SYSTEM-601: Service failure"
            print_info "WIA-CORE-SYSTEM-602: Configuration error"
            print_info "WIA-CORE-SYSTEM-603: Dependency failure"
            echo ""
        fi
    fi

    print_info "Full registry: https://errors.wiastandards.com/registry"
    echo ""
}

# Parse error from JSON
parse_error() {
    local file="$1"

    print_section "Error Parsing"
    print_info "File: $file"

    if [ ! -f "$file" ]; then
        print_error "File not found: $file"
        return 1
    fi

    # Check if jq is available
    if ! command -v jq &> /dev/null; then
        print_warning "jq not found, showing raw content"
        cat "$file"
        return 0
    fi

    # Parse JSON
    local code=$(jq -r '.error.code // .code // "N/A"' "$file")
    local message=$(jq -r '.error.message // .message // "N/A"' "$file")
    local severity=$(jq -r '.error.severity // .severity // "N/A"' "$file")
    local timestamp=$(jq -r '.error.timestamp // .timestamp // "N/A"' "$file")

    print_section "Error Details"
    print_info "Code: $code"
    print_info "Message: $message"
    print_info "Severity: $severity"
    print_info "Timestamp: $timestamp"

    # Show context if available
    local context=$(jq -r '.error.context // .context // null' "$file")
    if [ "$context" != "null" ]; then
        print_section "Context"
        echo "$context" | jq '.'
    fi

    # Show recovery info if available
    local recovery=$(jq -r '.error.recovery // .recovery // null' "$file")
    if [ "$recovery" != "null" ]; then
        print_section "Recovery Information"
        echo "$recovery" | jq '.'
    fi

    echo ""
}

# Test error recovery
test_recovery() {
    local code="$1"

    print_section "Error Recovery Test"
    print_info "Error Code: $code"

    # Extract category
    local category=$(echo "$code" | cut -d'-' -f3)

    print_section "Recovery Strategy"
    case "$category" in
        AUTH)
            print_info "Strategy: Retry with fresh credentials"
            print_info "Max Attempts: 3"
            print_info "Backoff: Exponential (1s, 2s, 4s)"
            print_success "User Action: Re-authenticate"
            ;;
        VALIDATION)
            print_info "Strategy: No automatic retry"
            print_info "Max Attempts: 0"
            print_error "User Action: Fix input and retry"
            ;;
        NETWORK)
            print_info "Strategy: Retry with exponential backoff"
            print_info "Max Attempts: 5"
            print_info "Backoff: Exponential with jitter"
            print_success "Fallback: Use cached data if available"
            ;;
        DATABASE)
            print_info "Strategy: Retry with linear backoff"
            print_info "Max Attempts: 3"
            print_info "Backoff: Linear (2s, 4s, 6s)"
            print_warning "Fallback: Read replica or cache"
            ;;
        RESOURCE)
            print_info "Strategy: Wait and retry"
            print_info "Max Attempts: 10"
            print_info "Backoff: Fixed with jitter"
            print_warning "Circuit Breaker: Open after 5 failures"
            ;;
        PERMISSION)
            print_info "Strategy: No automatic retry"
            print_info "Max Attempts: 0"
            print_error "User Action: Request permission elevation"
            ;;
        SYSTEM)
            print_info "Strategy: Limited retry with escalation"
            print_info "Max Attempts: 2"
            print_info "Backoff: Fixed (5s)"
            print_error "Escalation: Alert on-call team"
            ;;
        *)
            print_warning "Strategy: Default retry"
            print_info "Max Attempts: 3"
            print_info "Backoff: Exponential"
            ;;
    esac

    print_section "Simulation"
    for i in {1..3}; do
        print_info "Attempt $i..."
        sleep 0.5

        if [ $i -eq 3 ]; then
            print_success "Recovery successful on attempt $i"
        else
            print_warning "Attempt $i failed, retrying..."
        fi
    done

    echo ""
}

# Generate error documentation
generate_docs() {
    local output="${1:-./error-docs}"

    print_section "Generating Documentation"
    print_info "Output: $output"

    mkdir -p "$output"

    # Generate markdown documentation
    cat > "$output/ERROR_CODES.md" << 'EOF'
# WIA Error Code Reference

## Error Code Structure

All WIA error codes follow this pattern:
```
WIA-{DOMAIN}-{CATEGORY}-{NUMBER}
```

## Core Error Codes

### Authentication (AUTH: 001-099)

| Code | Description | HTTP | Severity | Retryable |
|------|-------------|------|----------|-----------|
| WIA-CORE-AUTH-001 | Invalid credentials | 401 | ERROR | Yes |
| WIA-CORE-AUTH-002 | Token expired | 401 | ERROR | Yes |
| WIA-CORE-AUTH-003 | Token invalid | 401 | ERROR | No |
| WIA-CORE-AUTH-004 | Session expired | 401 | WARNING | Yes |
| WIA-CORE-AUTH-005 | MFA required | 401 | INFO | No |

### Validation (VALIDATION: 100-199)

| Code | Description | HTTP | Severity | Retryable |
|------|-------------|------|----------|-----------|
| WIA-CORE-VALIDATION-100 | Invalid input | 400 | ERROR | No |
| WIA-CORE-VALIDATION-101 | Missing required field | 400 | ERROR | No |
| WIA-CORE-VALIDATION-102 | Invalid format | 400 | ERROR | No |
| WIA-CORE-VALIDATION-103 | Value out of range | 400 | ERROR | No |
| WIA-CORE-VALIDATION-104 | Type mismatch | 400 | ERROR | No |

### Network (NETWORK: 200-299)

| Code | Description | HTTP | Severity | Retryable |
|------|-------------|------|----------|-----------|
| WIA-CORE-NETWORK-200 | Network timeout | 408 | WARNING | Yes |
| WIA-CORE-NETWORK-201 | Connection refused | 503 | ERROR | Yes |
| WIA-CORE-NETWORK-202 | DNS resolution failed | 503 | ERROR | Yes |
| WIA-CORE-NETWORK-502 | Bad gateway | 502 | ERROR | Yes |
| WIA-CORE-NETWORK-504 | Gateway timeout | 504 | WARNING | Yes |

---

**弘益人間 (Benefit All Humanity)**

*© 2025 SmileStory Inc. / WIA - MIT License*
EOF

    print_success "Created ERROR_CODES.md"

    # Generate quick reference
    cat > "$output/QUICK_REFERENCE.txt" << 'EOF'
WIA Error Handling Quick Reference
==================================

Error Code Format: WIA-{DOMAIN}-{CATEGORY}-{NUMBER}

Severity Levels:
  5 CRITICAL  - System failure, immediate action
  4 ERROR     - Operation failed, intervention needed
  3 WARNING   - Potential issue, degraded functionality
  2 INFO      - Informational, no action required
  1 DEBUG     - Debugging information

Common Categories:
  AUTH        - Authentication/Authorization
  VALIDATION  - Input validation
  NETWORK     - Network/Communication
  DATABASE    - Data persistence
  RESOURCE    - Resource management
  PERMISSION  - Access control
  SYSTEM      - System failures

Recovery Strategies:
  retry       - Automatic retry with backoff
  fallback    - Use alternative value/function
  ignore      - Log and continue
  escalate    - Alert team/admin
  degrade     - Reduce functionality

For more information: https://wiastandards.com/core-010
EOF

    print_success "Created QUICK_REFERENCE.txt"

    print_section "Documentation Complete"
    print_info "Files created in: $output"
    print_info "  - ERROR_CODES.md"
    print_info "  - QUICK_REFERENCE.txt"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-core-010 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  validate                 Validate error code format"
    echo "    --code <error-code>    Error code to validate"
    echo ""
    echo "  list                     List error codes"
    echo "    --domain <domain>      Filter by domain (default: CORE)"
    echo "    --category <category>  Filter by category"
    echo ""
    echo "  parse                    Parse error from JSON file"
    echo "    --file <path>          Path to JSON file"
    echo ""
    echo "  test-recovery            Test error recovery strategy"
    echo "    --code <error-code>    Error code to test"
    echo ""
    echo "  docs                     Generate error documentation"
    echo "    --output <path>        Output directory (default: ./error-docs)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-core-010 validate --code 'WIA-CORE-AUTH-001'"
    echo "  wia-core-010 list --domain CORE --category AUTH"
    echo "  wia-core-010 parse --file error.json"
    echo "  wia-core-010 test-recovery --code 'WIA-CORE-NETWORK-200'"
    echo "  wia-core-010 docs --output ./docs"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-CORE-010 CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    validate)
        CODE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --code) CODE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$CODE" ]; then
            print_error "Error code required"
            echo "Usage: wia-core-010 validate --code <error-code>"
            exit 1
        fi

        print_header
        validate_code "$CODE"
        ;;

    list)
        DOMAIN="CORE"
        CATEGORY=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --domain) DOMAIN=$2; shift 2 ;;
                --category) CATEGORY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        list_codes "$DOMAIN" "$CATEGORY"
        ;;

    parse)
        FILE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --file) FILE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$FILE" ]; then
            print_error "File path required"
            echo "Usage: wia-core-010 parse --file <path>"
            exit 1
        fi

        print_header
        parse_error "$FILE"
        ;;

    test-recovery)
        CODE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --code) CODE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$CODE" ]; then
            print_error "Error code required"
            echo "Usage: wia-core-010 test-recovery --code <error-code>"
            exit 1
        fi

        print_header
        test_recovery "$CODE"
        ;;

    docs)
        OUTPUT="./error-docs"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_docs "$OUTPUT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-core-010 help' for usage information"
        exit 1
        ;;
esac

exit 0
