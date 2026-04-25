#!/bin/bash

################################################################################
# WIA-CORE-003: Universal Data Exchange CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to universal data exchange
# operations including envelope creation, validation, transformation, and
# integrity verification.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
BLUE='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🔄 WIA-CORE-003: Universal Data Exchange CLI           ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${BLUE}▶ $1${RESET}"
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

# Generate UUID v4
generate_uuid() {
    if command -v uuidgen &> /dev/null; then
        uuidgen | tr '[:upper:]' '[:lower:]'
    else
        # Fallback UUID generation
        cat /proc/sys/kernel/random/uuid 2>/dev/null || \
        od -x /dev/urandom | head -1 | awk '{OFS="-"; print substr($2$3,1,8),substr($2$3,9,4),"4"substr($4,2,3),substr($4,5,4),substr($5$6,1,12)}'
    fi
}

# Calculate SHA-256 hash
calculate_sha256() {
    local input="$1"
    echo -n "$input" | sha256sum | awk '{print $1}'
}

# Get current ISO 8601 timestamp
get_timestamp() {
    date -u +"%Y-%m-%dT%H:%M:%S.000Z"
}

# Create envelope
create_envelope() {
    local schema="${1:-https://schema.org/Thing}"
    local data_file="${2:-}"
    local output_file="${3:-envelope.json}"

    print_section "Creating Universal Data Envelope"

    # Read data from file or use sample
    local data
    if [ -n "$data_file" ] && [ -f "$data_file" ]; then
        data=$(cat "$data_file")
        print_info "Data source: $data_file"
    else
        data='{"name":"Sample Data","value":42}'
        print_warning "No data file provided, using sample data"
    fi

    # Calculate data size
    local size=${#data}

    # Generate envelope metadata
    local envelope_id=$(generate_uuid)
    local timestamp=$(get_timestamp)

    # Calculate data checksum
    local data_checksum=$(calculate_sha256 "$data")

    print_info "Envelope ID: $envelope_id"
    print_info "Schema: $schema"
    print_info "Data size: $size bytes"
    print_info "Timestamp: $timestamp"

    # Create envelope structure
    local envelope=$(cat <<EOF
{
  "meta": {
    "id": "$envelope_id",
    "version": "1.0.0",
    "schema": "$schema",
    "schemaVersion": "1.0.0",
    "timestamp": "$timestamp",
    "format": "json",
    "encoding": "utf-8",
    "compression": "none",
    "size": $size,
    "checksum": "sha256:$data_checksum"
  },
  "data": $data,
  "integrity": {
    "algorithm": "sha256",
    "hash": ""
  }
}
EOF
)

    # Calculate integrity hash (simplified - concatenate meta and data)
    local meta_str=$(echo "$envelope" | jq -c '.meta | to_entries | sort_by(.key) | from_entries')
    local data_str=$(echo "$envelope" | jq -c '.data')
    local combined="${meta_str}${data_str}"
    local integrity_hash=$(calculate_sha256 "$combined")

    # Update envelope with integrity hash
    envelope=$(echo "$envelope" | jq --arg hash "$integrity_hash" '.integrity.hash = $hash')

    # Save to file
    echo "$envelope" | jq '.' > "$output_file"

    print_section "Envelope Created"
    print_success "Envelope saved to: $output_file"
    print_info "Integrity hash: $integrity_hash"

    echo ""
    print_info "Envelope preview:"
    echo "$envelope" | jq '.' | head -20
}

# Validate envelope
validate_envelope() {
    local envelope_file="${1:-envelope.json}"

    print_section "Validating Envelope"

    if [ ! -f "$envelope_file" ]; then
        print_error "Envelope file not found: $envelope_file"
        return 1
    fi

    print_info "File: $envelope_file"

    local envelope=$(cat "$envelope_file")
    local valid=true
    local errors=()

    # Check required fields
    if ! echo "$envelope" | jq -e '.meta' > /dev/null 2>&1; then
        errors+=("Missing 'meta' field")
        valid=false
    fi

    if ! echo "$envelope" | jq -e '.data' > /dev/null 2>&1; then
        errors+=("Missing 'data' field")
        valid=false
    fi

    if ! echo "$envelope" | jq -e '.integrity' > /dev/null 2>&1; then
        errors+=("Missing 'integrity' field")
        valid=false
    fi

    # Check meta fields
    if ! echo "$envelope" | jq -e '.meta.id' > /dev/null 2>&1; then
        errors+=("Missing 'meta.id'")
        valid=false
    fi

    if ! echo "$envelope" | jq -e '.meta.version' > /dev/null 2>&1; then
        errors+=("Missing 'meta.version'")
        valid=false
    fi

    if ! echo "$envelope" | jq -e '.meta.schema' > /dev/null 2>&1; then
        errors+=("Missing 'meta.schema'")
        valid=false
    fi

    # Verify integrity hash
    local stored_hash=$(echo "$envelope" | jq -r '.integrity.hash')
    local meta_str=$(echo "$envelope" | jq -c '.meta | to_entries | sort_by(.key) | from_entries')
    local data_str=$(echo "$envelope" | jq -c '.data')
    local combined="${meta_str}${data_str}"
    local calculated_hash=$(calculate_sha256 "$combined")

    if [ "$stored_hash" != "$calculated_hash" ]; then
        errors+=("Integrity hash mismatch")
        valid=false
    fi

    print_section "Validation Result"

    if [ "$valid" = true ]; then
        print_success "Envelope is valid"
        print_info "Integrity verified: ✓"
    else
        print_error "Envelope is invalid"
        for error in "${errors[@]}"; do
            print_error "  - $error"
        done
    fi

    echo ""
    print_info "Envelope metadata:"
    echo "$envelope" | jq '.meta'
}

# Verify integrity
verify_integrity() {
    local envelope_file="${1:-envelope.json}"

    print_section "Verifying Integrity"

    if [ ! -f "$envelope_file" ]; then
        print_error "Envelope file not found: $envelope_file"
        return 1
    fi

    local envelope=$(cat "$envelope_file")

    # Get stored hash
    local stored_hash=$(echo "$envelope" | jq -r '.integrity.hash')

    # Calculate hash
    local meta_str=$(echo "$envelope" | jq -c '.meta | to_entries | sort_by(.key) | from_entries')
    local data_str=$(echo "$envelope" | jq -c '.data')
    local combined="${meta_str}${data_str}"
    local calculated_hash=$(calculate_sha256 "$combined")

    print_info "Stored hash:     $stored_hash"
    print_info "Calculated hash: $calculated_hash"

    echo ""

    if [ "$stored_hash" = "$calculated_hash" ]; then
        print_success "Integrity verification PASSED"
        print_info "The envelope has not been tampered with"
        return 0
    else
        print_error "Integrity verification FAILED"
        print_warning "The envelope may have been modified"
        return 1
    fi
}

# Calculate compatibility score
calculate_compatibility() {
    local source_version="${1:-1.0.0}"
    local target_version="${2:-2.0.0}"

    print_section "Schema Compatibility Analysis"

    # Parse versions
    IFS='.' read -r src_major src_minor src_patch <<< "$source_version"
    IFS='.' read -r tgt_major tgt_minor tgt_patch <<< "$target_version"

    print_info "Source version: $source_version"
    print_info "Target version: $target_version"

    # Calculate version difference
    local major_diff=$((tgt_major - src_major))
    local minor_diff=$((tgt_minor - src_minor))
    local patch_diff=$((tgt_patch - src_patch))

    major_diff=${major_diff#-}  # Absolute value
    minor_diff=${minor_diff#-}
    patch_diff=${patch_diff#-}

    local version_diff=$((major_diff * 100 + minor_diff * 10 + patch_diff))
    local max_diff=1000

    # Calculate score
    local score=$(echo "scale=2; 1 - ($version_diff / $max_diff)" | bc)

    # Major version difference = reduced compatibility
    if [ $major_diff -gt 0 ]; then
        score=$(echo "scale=2; $score * 0.5" | bc)
    fi

    # Convert to percentage
    local percentage=$(echo "scale=1; $score * 100" | bc)

    print_section "Compatibility Result"
    print_info "Compatibility score: ${percentage}%"

    if (( $(echo "$score >= 0.9" | bc -l) )); then
        print_success "High compatibility - Direct use recommended"
    elif (( $(echo "$score >= 0.7" | bc -l) )); then
        print_warning "Moderate compatibility - Minor transformation needed"
    elif (( $(echo "$score >= 0.5" | bc -l) )); then
        print_warning "Low compatibility - Schema mapping required"
    else
        print_error "Incompatible - Manual conversion needed"
    fi
}

# Transform envelope
transform_envelope() {
    local input_file="${1:-envelope.json}"
    local target_format="${2:-json}"
    local output_file="${3:-transformed.json}"

    print_section "Transforming Envelope"

    if [ ! -f "$input_file" ]; then
        print_error "Input file not found: $input_file"
        return 1
    fi

    print_info "Input: $input_file"
    print_info "Target format: $target_format"
    print_info "Output: $output_file"

    # Read envelope
    local envelope=$(cat "$input_file")

    # Update format in metadata
    envelope=$(echo "$envelope" | jq --arg fmt "$target_format" '.meta.format = $fmt')

    # Recalculate integrity
    local meta_str=$(echo "$envelope" | jq -c '.meta | to_entries | sort_by(.key) | from_entries')
    local data_str=$(echo "$envelope" | jq -c '.data')
    local combined="${meta_str}${data_str}"
    local new_hash=$(calculate_sha256 "$combined")

    envelope=$(echo "$envelope" | jq --arg hash "$new_hash" '.integrity.hash = $hash')

    # Save transformed envelope
    echo "$envelope" | jq '.' > "$output_file"

    print_section "Transformation Complete"
    print_success "Transformed envelope saved to: $output_file"
    print_info "New integrity hash: $new_hash"
}

# Display help
show_help() {
    print_header
    echo "Usage: wia-core-003 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  create <schema> <data-file> <output>  Create a new envelope"
    echo "  validate <envelope-file>              Validate envelope structure"
    echo "  verify <envelope-file>                Verify envelope integrity"
    echo "  compatibility <v1> <v2>               Calculate schema compatibility"
    echo "  transform <input> <format> <output>   Transform envelope format"
    echo "  --version                             Show version"
    echo "  --help                                Show this help"
    echo ""
    echo "Examples:"
    echo "  wia-core-003 create https://schema.org/Person data.json envelope.json"
    echo "  wia-core-003 validate envelope.json"
    echo "  wia-core-003 verify envelope.json"
    echo "  wia-core-003 compatibility 1.0.0 2.0.0"
    echo "  wia-core-003 transform envelope.json xml transformed.json"
    echo ""
    echo "弘益人間 (Benefit All Humanity)"
    echo "WIA - World Certification Industry Association"
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        create)
            shift
            create_envelope "$@"
            ;;
        validate)
            shift
            validate_envelope "$@"
            ;;
        verify)
            shift
            verify_integrity "$@"
            ;;
        compatibility)
            shift
            calculate_compatibility "$@"
            ;;
        transform)
            shift
            transform_envelope "$@"
            ;;
        --version|-v)
            echo "WIA-CORE-003 CLI v$VERSION"
            ;;
        --help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Check dependencies
check_dependencies() {
    if ! command -v jq &> /dev/null; then
        print_error "jq is required but not installed."
        print_info "Install with: apt-get install jq (Debian/Ubuntu) or brew install jq (macOS)"
        exit 1
    fi

    if ! command -v bc &> /dev/null; then
        print_warning "bc is not installed. Some calculations may not work."
    fi
}

# Entry point
check_dependencies
main "$@"
