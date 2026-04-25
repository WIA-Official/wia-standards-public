#!/bin/bash

################################################################################
# WIA-CORE-008: Universal Metadata CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to universal metadata operations
# including creation, validation, enrichment, and search.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;94m'
PURPLE='\033[0;35m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SCHEMA_VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        📋 WIA-CORE-008: Universal Metadata CLI Tool          ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${PURPLE}▶ $1${RESET}"
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

# Generate unique ID
generate_id() {
    echo "wia-metadata-$(date +%s)-$(openssl rand -hex 4 2>/dev/null || echo $RANDOM)"
}

# Get current ISO timestamp
get_timestamp() {
    date -u +"%Y-%m-%dT%H:%M:%SZ"
}

# Create metadata
create_metadata() {
    local title="${1:-Untitled}"
    local description="${2:-}"
    local domain="${3:-general}"
    local data_type="${4:-dataset}"
    local format="${5:-}"

    print_section "Creating Universal Metadata"

    local id=$(generate_id)
    local timestamp=$(get_timestamp)

    print_info "ID: $id"
    print_info "Title: $title"
    print_info "Domain: $domain"
    print_info "Data Type: $data_type"

    # Generate JSON metadata
    cat > "metadata-$id.json" << EOF
{
  "id": "$id",
  "title": "$title",
  "description": "$description",
  "domain": "$domain",
  "dataType": "$data_type",
  "format": "$format",
  "created": "$timestamp",
  "modified": "$timestamp",
  "schemaVersion": "$SCHEMA_VERSION",
  "privacy": "internal",
  "validationStatus": "not-validated"
}
EOF

    print_success "Metadata created: metadata-$id.json"

    # Validate the created metadata
    validate_metadata "metadata-$id.json"
}

# Validate metadata
validate_metadata() {
    local file="${1:-metadata.json}"

    print_section "Validating Metadata"

    if [ ! -f "$file" ]; then
        print_error "File not found: $file"
        return 1
    fi

    print_info "File: $file"

    # Check if jq is available for JSON validation
    if ! command -v jq &> /dev/null; then
        print_warning "jq not found - install for detailed validation"
        print_info "Basic file check: OK"
        return 0
    fi

    # Validate JSON syntax
    if ! jq empty "$file" 2>/dev/null; then
        print_error "Invalid JSON syntax"
        return 1
    fi

    print_success "JSON syntax: Valid"

    # Check required fields
    local required_fields=("id" "title")
    local missing_fields=()

    for field in "${required_fields[@]}"; do
        if ! jq -e ".$field" "$file" &>/dev/null; then
            missing_fields+=("$field")
        fi
    done

    if [ ${#missing_fields[@]} -gt 0 ]; then
        print_error "Missing required fields: ${missing_fields[*]}"
        return 1
    fi

    print_success "Required fields: Present"

    # Calculate quality score
    local total_fields=10
    local present_fields=0

    local recommended=("description" "creator" "created" "domain" "dataType" "keywords" "tags" "license")
    for field in "${recommended[@]}"; do
        if jq -e ".$field" "$file" &>/dev/null; then
            ((present_fields++))
        fi
    done

    local completeness=$((present_fields * 100 / total_fields))
    local quality_score=$((completeness * 4 / 10 + 30))

    print_section "Quality Metrics"
    print_info "Completeness: $completeness%"
    print_info "Quality Score: $quality_score/100"

    if [ $quality_score -ge 90 ]; then
        print_success "Quality Level: EXCELLENT"
    elif [ $quality_score -ge 75 ]; then
        print_success "Quality Level: GOOD"
    elif [ $quality_score -ge 60 ]; then
        print_warning "Quality Level: ACCEPTABLE"
    else
        print_warning "Quality Level: POOR"
    fi

    # Recommendations
    print_section "Recommendations"

    if ! jq -e ".description" "$file" &>/dev/null; then
        print_info "• Add description to improve discoverability"
    fi

    if ! jq -e ".keywords" "$file" &>/dev/null; then
        print_info "• Add keywords to improve searchability"
    fi

    if ! jq -e ".creator" "$file" &>/dev/null; then
        print_info "• Add creator information for provenance"
    fi

    if ! jq -e ".license" "$file" &>/dev/null; then
        print_info "• Add license information for rights management"
    fi

    echo ""
}

# Enrich metadata
enrich_metadata() {
    local file="${1:-metadata.json}"

    print_section "Enriching Metadata"

    if [ ! -f "$file" ]; then
        print_error "File not found: $file"
        return 1
    fi

    if ! command -v jq &> /dev/null; then
        print_error "jq is required for enrichment"
        return 1
    fi

    print_info "File: $file"

    local timestamp=$(get_timestamp)

    # Add/update fields
    local enriched=$(jq --arg ts "$timestamp" '
        .modified = $ts |
        .lastValidated = $ts |
        if .keywords == null then .keywords = [] else . end |
        if .tags == null then .tags = [.domain, .dataType] | map(select(. != null)) else . end
    ' "$file")

    # Save enriched metadata
    echo "$enriched" > "$file.enriched.json"

    print_success "Metadata enriched: $file.enriched.json"
    print_info "Added: modified, lastValidated, tags"

    echo ""
}

# Search metadata
search_metadata() {
    local query="${1:-}"
    local dir="${2:-.}"

    print_section "Searching Metadata"
    print_info "Query: $query"
    print_info "Directory: $dir"

    local count=0

    # Find all JSON files
    while IFS= read -r file; do
        if [ -f "$file" ]; then
            # Simple grep search if jq not available
            if ! command -v jq &> /dev/null; then
                if grep -qi "$query" "$file" 2>/dev/null; then
                    print_success "Match: $file"
                    ((count++))
                fi
            else
                # Use jq for structured search
                local title=$(jq -r '.title // ""' "$file" 2>/dev/null)
                local desc=$(jq -r '.description // ""' "$file" 2>/dev/null)

                if echo "$title $desc" | grep -qi "$query"; then
                    print_success "Match: $file"
                    print_info "  Title: $title"
                    ((count++))
                fi
            fi
        fi
    done < <(find "$dir" -name "*.json" -type f)

    print_section "Results"
    print_info "Total matches: $count"
    echo ""
}

# Generate template
generate_template() {
    local domain="${1:-general}"
    local format="${2:-json}"

    print_section "Generating Metadata Template"
    print_info "Domain: $domain"
    print_info "Format: $format"

    local id=$(generate_id)
    local timestamp=$(get_timestamp)

    cat > "template-$domain.json" << EOF
{
  "id": "$id",
  "title": "REPLACE_WITH_TITLE",
  "description": "REPLACE_WITH_DESCRIPTION",
  "domain": "$domain",
  "dataType": "dataset",
  "format": "application/json",
  "creator": {
    "name": "REPLACE_WITH_CREATOR_NAME",
    "id": "REPLACE_WITH_CREATOR_ID"
  },
  "created": "$timestamp",
  "modified": "$timestamp",
  "keywords": ["keyword1", "keyword2", "keyword3"],
  "tags": ["$domain"],
  "license": "MIT",
  "privacy": "internal",
  "schemaVersion": "$SCHEMA_VERSION",
  "validationStatus": "not-validated"
}
EOF

    print_success "Template created: template-$domain.json"
    print_info "Edit the file and replace placeholder values"
    echo ""
}

# Check quality
check_quality() {
    local file="${1:-metadata.json}"

    print_section "Quality Assessment"

    if [ ! -f "$file" ]; then
        print_error "File not found: $file"
        return 1
    fi

    if ! command -v jq &> /dev/null; then
        print_error "jq is required for quality assessment"
        return 1
    fi

    # Count fields
    local field_count=$(jq 'keys | length' "$file")

    # Check required fields
    local has_id=$(jq -e '.id' "$file" &>/dev/null && echo "yes" || echo "no")
    local has_title=$(jq -e '.title' "$file" &>/dev/null && echo "yes" || echo "no")

    # Check recommended fields
    local has_description=$(jq -e '.description' "$file" &>/dev/null && echo "yes" || echo "no")
    local has_creator=$(jq -e '.creator' "$file" &>/dev/null && echo "yes" || echo "no")
    local has_keywords=$(jq -e '.keywords' "$file" &>/dev/null && echo "yes" || echo "no")

    # Calculate scores
    local completeness=0
    [ "$has_id" = "yes" ] && ((completeness+=10))
    [ "$has_title" = "yes" ] && ((completeness+=10))
    [ "$has_description" = "yes" ] && ((completeness+=15))
    [ "$has_creator" = "yes" ] && ((completeness+=15))
    [ "$has_keywords" = "yes" ] && ((completeness+=15))

    local discoverability=0
    [ "$has_description" = "yes" ] && ((discoverability+=20))
    [ "$has_keywords" = "yes" ] && ((discoverability+=25))

    print_info "Total Fields: $field_count"
    print_info "Completeness: $completeness%"
    print_info "Discoverability: $discoverability/100"

    print_section "Field Status"
    [ "$has_id" = "yes" ] && print_success "ID: Present" || print_error "ID: Missing"
    [ "$has_title" = "yes" ] && print_success "Title: Present" || print_error "Title: Missing"
    [ "$has_description" = "yes" ] && print_success "Description: Present" || print_warning "Description: Missing"
    [ "$has_creator" = "yes" ] && print_success "Creator: Present" || print_warning "Creator: Missing"
    [ "$has_keywords" = "yes" ] && print_success "Keywords: Present" || print_warning "Keywords: Missing"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-core-008 <command> [options]"
    echo ""
    echo "Commands:"
    echo -e "${INDIGO}  create${RESET}              Create new metadata"
    echo -e "${INDIGO}  validate${RESET}            Validate metadata file"
    echo -e "${INDIGO}  enrich${RESET}              Enrich metadata with additional fields"
    echo -e "${INDIGO}  search${RESET}              Search metadata files"
    echo -e "${INDIGO}  template${RESET}            Generate metadata template"
    echo -e "${INDIGO}  quality${RESET}             Check metadata quality"
    echo -e "${INDIGO}  version${RESET}             Show version"
    echo -e "${INDIGO}  help${RESET}                Show this help"
    echo ""
    echo "Examples:"
    echo -e "${GRAY}  # Create metadata${RESET}"
    echo -e "${INDIGO}  wia-core-008 create --title \"Research Data\" --domain science${RESET}"
    echo ""
    echo -e "${GRAY}  # Validate metadata${RESET}"
    echo -e "${INDIGO}  wia-core-008 validate --file metadata.json${RESET}"
    echo ""
    echo -e "${GRAY}  # Enrich metadata${RESET}"
    echo -e "${INDIGO}  wia-core-008 enrich --file metadata.json${RESET}"
    echo ""
    echo -e "${GRAY}  # Search metadata${RESET}"
    echo -e "${INDIGO}  wia-core-008 search --query \"genomics\"${RESET}"
    echo ""
    echo -e "${GRAY}  # Generate template${RESET}"
    echo -e "${INDIGO}  wia-core-008 template --domain healthcare${RESET}"
    echo ""
    echo -e "${GRAY}  # Check quality${RESET}"
    echo -e "${INDIGO}  wia-core-008 quality --file metadata.json${RESET}"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    echo "WIA-CORE-008 Universal Metadata CLI v$VERSION"
    echo "Schema Version: $SCHEMA_VERSION"
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    # Parse arguments
    local title=""
    local description=""
    local domain="general"
    local data_type="dataset"
    local format=""
    local file="metadata.json"
    local query=""
    local dir="."

    while [[ $# -gt 0 ]]; do
        case $1 in
            --title) title="$2"; shift 2 ;;
            --description) description="$2"; shift 2 ;;
            --domain) domain="$2"; shift 2 ;;
            --data-type) data_type="$2"; shift 2 ;;
            --format) format="$2"; shift 2 ;;
            --file) file="$2"; shift 2 ;;
            --query) query="$2"; shift 2 ;;
            --dir) dir="$2"; shift 2 ;;
            --metadata) file="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    # Execute command
    case $command in
        create)
            create_metadata "$title" "$description" "$domain" "$data_type" "$format"
            ;;
        validate)
            validate_metadata "$file"
            ;;
        enrich)
            enrich_metadata "$file"
            ;;
        search)
            search_metadata "$query" "$dir"
            ;;
        template)
            generate_template "$domain"
            ;;
        quality)
            check_quality "$file"
            ;;
        version)
            show_version
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-core-008 help' for usage"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
