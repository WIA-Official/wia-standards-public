#!/bin/bash

################################################################################
# WIA-CORE-009: Universal Timestamp CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to universal timestamp operations
# including creation, parsing, validation, and timezone conversion.
################################################################################

set -e

# Colors for output
INDIGO='\033[38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
FORMAT_PREFIX="WIA-TS:"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         ⏰ WIA-CORE-009: Universal Timestamp CLI              ║"
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

# Get current timestamp
get_current_timestamp() {
    local precision=${1:-"nano"}
    local timezone=${2:-"UTC"}

    print_section "Current Universal Timestamp"

    local epoch=$(date +%s)
    local nano=$(date +%N 2>/dev/null || echo "000000000")

    case $precision in
        "second"|"s")
            local timestamp="${FORMAT_PREFIX} ${epoch}@${timezone}"
            ;;
        "milli"|"ms")
            local milli=${nano:0:3}
            local timestamp="${FORMAT_PREFIX} ${epoch}.${milli}@${timezone}"
            ;;
        "micro"|"us")
            local micro=${nano:0:6}
            local timestamp="${FORMAT_PREFIX} ${epoch}.${micro}@${timezone}"
            ;;
        "nano"|"ns"|*)
            local timestamp="${FORMAT_PREFIX} ${epoch}.${nano}@${timezone}"
            ;;
    esac

    print_success "WIA Format:"
    echo -e "  ${INDIGO}${timestamp}${RESET}"

    print_info "ISO 8601: $(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
    print_info "Unix Epoch: ${epoch}"
    print_info "Precision: ${precision}"
    print_info "Timezone: ${timezone}"

    echo ""
}

# Parse WIA-TS format
parse_timestamp() {
    local input="$1"

    print_section "Parse WIA Timestamp"
    print_info "Input: ${input}"

    # Remove prefix
    local content="${input#*WIA-TS:}"
    content="${content# }"

    # Extract components using regex
    if [[ $content =~ ^(-?[0-9]+)(\.([0-9]{1,9}))?@([A-Za-z/_+-:]+)(\[([^\]]+)\])?(\{([^}]+)\})?$ ]]; then
        local epoch="${BASH_REMATCH[1]}"
        local precision="${BASH_REMATCH[3]:-0}"
        local timezone="${BASH_REMATCH[4]}"
        local temporal="${BASH_REMATCH[6]}"
        local signature="${BASH_REMATCH[8]}"

        print_section "Parsed Components"
        print_success "Epoch: ${epoch}"

        if [ -n "$precision" ] && [ "$precision" != "0" ]; then
            print_success "Precision: ${precision} (${#precision} digits)"
        fi

        print_success "Timezone: ${timezone}"

        # Convert to human-readable date
        local date_str=$(date -d "@${epoch}" +"%Y-%m-%d %H:%M:%S %Z" 2>/dev/null || date -r "${epoch}" +"%Y-%m-%d %H:%M:%S" 2>/dev/null || echo "Invalid epoch")
        print_info "Human Date: ${date_str}"

        if [ -n "$temporal" ]; then
            print_section "Temporal Context"
            print_info "${temporal}"
        fi

        if [ -n "$signature" ]; then
            print_section "Signature"
            print_info "${signature}"
        fi

        print_section "Validation"
        print_success "Format is valid"

    else
        print_section "Validation"
        print_error "Invalid WIA-TS format"
        return 1
    fi

    echo ""
}

# Convert timezone
convert_timezone() {
    local input="$1"
    local target_tz="$2"

    print_section "Convert Timezone"
    print_info "Input: ${input}"
    print_info "Target: ${target_tz}"

    # Extract epoch from timestamp
    local content="${input#*WIA-TS:}"
    content="${content# }"

    if [[ $content =~ ^(-?[0-9]+)(\.([0-9]{1,9}))?@([A-Za-z/_+-:]+) ]]; then
        local epoch="${BASH_REMATCH[1]}"
        local precision="${BASH_REMATCH[3]}"

        # Build new timestamp with target timezone
        if [ -n "$precision" ]; then
            local new_timestamp="${FORMAT_PREFIX} ${epoch}.${precision}@${target_tz}"
        else
            local new_timestamp="${FORMAT_PREFIX} ${epoch}@${target_tz}"
        fi

        print_success "Converted:"
        echo -e "  ${INDIGO}${new_timestamp}${RESET}"

        # Show human-readable times
        print_info "Original: $(date -d "@${epoch}" +"%Y-%m-%d %H:%M:%S %Z" 2>/dev/null || date -r "${epoch}" +"%Y-%m-%d %H:%M:%S" 2>/dev/null)"

        if [ "$target_tz" != "UTC" ] && command -v TZ &>/dev/null; then
            TZ="$target_tz" print_info "Target: $(TZ=$target_tz date -d "@${epoch}" +"%Y-%m-%d %H:%M:%S %Z" 2>/dev/null || echo "Timezone conversion not supported")"
        fi
    else
        print_error "Invalid timestamp format"
        return 1
    fi

    echo ""
}

# Validate timestamp
validate_timestamp() {
    local input="$1"

    print_section "Validate Timestamp"
    print_info "Input: ${input}"

    local errors=0
    local warnings=0

    # Check format prefix
    if [[ ! $input =~ ^WIA-TS: ]]; then
        print_error "Missing WIA-TS prefix"
        ((errors++))
    else
        print_success "Format prefix is valid"
    fi

    # Extract and validate components
    local content="${input#*WIA-TS:}"
    content="${content# }"

    if [[ $content =~ ^(-?[0-9]+)(\.([0-9]{1,9}))?@([A-Za-z/_+-:]+) ]]; then
        local epoch="${BASH_REMATCH[1]}"
        local precision="${BASH_REMATCH[3]}"
        local timezone="${BASH_REMATCH[4]}"

        # Validate epoch range
        local min_epoch=-2208988800  # 1900-01-01
        local max_epoch=4102444800    # 2100-01-01

        if [ "$epoch" -lt "$min_epoch" ]; then
            print_warning "Epoch before recommended minimum (1900-01-01)"
            ((warnings++))
        elif [ "$epoch" -gt "$max_epoch" ]; then
            print_warning "Epoch after recommended maximum (2100-01-01)"
            ((warnings++))
        else
            print_success "Epoch is within valid range"
        fi

        # Validate precision
        if [ -n "$precision" ]; then
            local len=${#precision}
            if [ "$len" -eq 3 ] || [ "$len" -eq 6 ] || [ "$len" -eq 9 ]; then
                print_success "Precision format is valid ($len digits)"
            else
                print_warning "Precision has unusual length ($len digits)"
                ((warnings++))
            fi
        fi

        # Validate timezone
        if [ "$timezone" == "UTC" ]; then
            print_success "Timezone is valid (UTC)"
        elif [[ $timezone =~ ^[+-][0-9]{2}:[0-9]{2}$ ]]; then
            print_success "Timezone offset is valid"
        elif [[ $timezone =~ ^[A-Za-z/_]+$ ]]; then
            print_success "IANA timezone format is valid"
        else
            print_error "Invalid timezone format"
            ((errors++))
        fi
    else
        print_error "Invalid timestamp structure"
        ((errors++))
    fi

    print_section "Validation Summary"

    if [ "$errors" -eq 0 ] && [ "$warnings" -eq 0 ]; then
        print_success "Timestamp is valid with no issues"
    elif [ "$errors" -eq 0 ]; then
        print_warning "Timestamp is valid but has $warnings warning(s)"
    else
        print_error "Timestamp has $errors error(s) and $warnings warning(s)"
        return 1
    fi

    echo ""
}

# Generate signed timestamp (placeholder)
sign_timestamp() {
    local key_file="$1"

    print_section "Generate Signed Timestamp"

    if [ -z "$key_file" ]; then
        print_error "Private key file required"
        print_info "Usage: wia-core-009 sign --key <private-key-file>"
        return 1
    fi

    if [ ! -f "$key_file" ]; then
        print_error "Private key file not found: $key_file"
        return 1
    fi

    local epoch=$(date +%s)
    local nano=$(date +%N 2>/dev/null || echo "000000000")
    local base_timestamp="${FORMAT_PREFIX} ${epoch}.${nano}@UTC"

    # Placeholder signature generation
    local hash=$(echo -n "$base_timestamp" | sha256sum | cut -d' ' -f1)
    local sig="placeholder_ed25519_${hash:0:16}"

    local signed_timestamp="${base_timestamp}{sig=ed25519:${sig},hash=sha256:${hash}}"

    print_success "Signed Timestamp:"
    echo -e "  ${INDIGO}${signed_timestamp}${RESET}"

    print_warning "Note: This is a placeholder implementation"
    print_info "Production use requires actual cryptographic signing"

    echo ""
}

# Verify timestamp signature (placeholder)
verify_signature() {
    local timestamp="$1"
    local key_file="$2"

    print_section "Verify Timestamp Signature"

    if [ -z "$timestamp" ] || [ -z "$key_file" ]; then
        print_error "Timestamp and public key required"
        print_info "Usage: wia-core-009 verify <timestamp> --key <public-key-file>"
        return 1
    fi

    # Check if signature exists
    if [[ $timestamp =~ \{([^}]+)\} ]]; then
        local signature="${BASH_REMATCH[1]}"
        print_info "Signature found: ${signature}"

        print_warning "Note: This is a placeholder implementation"
        print_info "Production use requires actual cryptographic verification"
        print_success "Signature format is valid"
    else
        print_error "No signature found in timestamp"
        return 1
    fi

    echo ""
}

# Show help
show_help() {
    print_header

    echo "Universal Timestamp CLI Tool"
    echo ""
    echo "Usage:"
    echo "  wia-core-009 <command> [options]"
    echo ""
    echo "Commands:"
    echo -e "  ${CYAN}now${RESET}                          Get current timestamp"
    echo -e "  ${CYAN}parse <timestamp>${RESET}            Parse WIA-TS format"
    echo -e "  ${CYAN}convert <timestamp> --to <tz>${RESET} Convert timezone"
    echo -e "  ${CYAN}validate <timestamp>${RESET}         Validate timestamp"
    echo -e "  ${CYAN}sign --key <file>${RESET}            Generate signed timestamp"
    echo -e "  ${CYAN}verify <timestamp> --key <file>${RESET} Verify signature"
    echo -e "  ${CYAN}help${RESET}                         Show this help"
    echo -e "  ${CYAN}version${RESET}                      Show version"
    echo ""
    echo "Options for 'now':"
    echo "  --precision <level>       Precision: second, milli, micro, nano (default: nano)"
    echo "  --timezone <tz>           Timezone (default: UTC)"
    echo ""
    echo "Examples:"
    echo -e "  ${GRAY}# Get current timestamp with nanosecond precision${RESET}"
    echo -e "  ${CYAN}wia-core-009 now${RESET}"
    echo ""
    echo -e "  ${GRAY}# Get current timestamp in specific timezone${RESET}"
    echo -e "  ${CYAN}wia-core-009 now --timezone America/New_York${RESET}"
    echo ""
    echo -e "  ${GRAY}# Parse a WIA timestamp${RESET}"
    echo -e "  ${CYAN}wia-core-009 parse \"WIA-TS: 1735257600.123456789@UTC\"${RESET}"
    echo ""
    echo -e "  ${GRAY}# Convert timezone${RESET}"
    echo -e "  ${CYAN}wia-core-009 convert \"WIA-TS: 1735257600@UTC\" --to Asia/Tokyo${RESET}"
    echo ""
    echo -e "  ${GRAY}# Validate timestamp${RESET}"
    echo -e "  ${CYAN}wia-core-009 validate \"WIA-TS: 1735257600.123456789@UTC\"${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    echo "WIA-CORE-009 Universal Timestamp CLI v${VERSION}"
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command="$1"
    shift

    case $command in
        now)
            local precision="nano"
            local timezone="UTC"

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --precision) precision="$2"; shift 2 ;;
                    --timezone) timezone="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done

            get_current_timestamp "$precision" "$timezone"
            ;;

        parse)
            if [ -z "$1" ]; then
                print_error "Timestamp required"
                echo "Usage: wia-core-009 parse <timestamp>"
                exit 1
            fi
            parse_timestamp "$1"
            ;;

        convert)
            if [ -z "$1" ]; then
                print_error "Timestamp required"
                echo "Usage: wia-core-009 convert <timestamp> --to <timezone>"
                exit 1
            fi

            local timestamp="$1"
            local target_tz=""
            shift

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --to) target_tz="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done

            if [ -z "$target_tz" ]; then
                print_error "Target timezone required (--to)"
                exit 1
            fi

            convert_timezone "$timestamp" "$target_tz"
            ;;

        validate)
            if [ -z "$1" ]; then
                print_error "Timestamp required"
                echo "Usage: wia-core-009 validate <timestamp>"
                exit 1
            fi
            validate_timestamp "$1"
            ;;

        sign)
            local key_file=""

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --key) key_file="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done

            sign_timestamp "$key_file"
            ;;

        verify)
            if [ -z "$1" ]; then
                print_error "Timestamp required"
                echo "Usage: wia-core-009 verify <timestamp> --key <public-key-file>"
                exit 1
            fi

            local timestamp="$1"
            local key_file=""
            shift

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --key) key_file="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done

            verify_signature "$timestamp" "$key_file"
            ;;

        help|--help|-h)
            show_help
            ;;

        version|--version|-v)
            show_version
            ;;

        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-core-009 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
