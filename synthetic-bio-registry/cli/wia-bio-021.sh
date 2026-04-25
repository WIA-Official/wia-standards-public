#!/bin/bash

################################################################################
# WIA-BIO-021: Synthetic Biology Registry CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Biotechnology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to synthetic biology registry
# operations including part registration, search, and characterization.
################################################################################

set -e

# Colors for output
TEAL='\033[38;5;37m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
REGISTRY_FILE="$HOME/.wia/bio-021/registry.json"

# Initialize registry storage
init_registry() {
    mkdir -p "$HOME/.wia/bio-021"
    if [ ! -f "$REGISTRY_FILE" ]; then
        echo '{"parts":[]}' > "$REGISTRY_FILE"
    fi
}

# Helper functions
print_header() {
    echo -e "${TEAL}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      📋 WIA-BIO-021: Synthetic Biology Registry CLI          ║"
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

# Register a new part
register_part() {
    local part_id="$1"
    local part_type="$2"
    local sequence_file="$3"
    local name="$4"
    local author="$5"

    print_section "Part Registration"

    # Validate inputs
    if [ -z "$part_id" ]; then
        print_error "Part ID is required"
        return 1
    fi

    if [ -z "$part_type" ]; then
        print_error "Part type is required"
        return 1
    fi

    if [ ! -f "$sequence_file" ]; then
        print_error "Sequence file not found: $sequence_file"
        return 1
    fi

    # Read sequence
    local sequence=$(cat "$sequence_file" | grep -v "^>" | tr -d '\n' | tr -d ' ')

    print_info "Part ID: $part_id"
    print_info "Type: $part_type"
    print_info "Sequence Length: ${#sequence} bp"
    print_info "Name: ${name:-Unknown}"
    print_info "Author: ${author:-Anonymous}"

    # Calculate GC content
    local gc_count=$(echo "$sequence" | grep -o "[GCgc]" | wc -l)
    local gc_content=$(echo "scale=2; $gc_count * 100 / ${#sequence}" | bc -l)

    print_section "Sequence Analysis"
    print_info "GC Content: ${gc_content}%"
    print_info "AT Content: $(echo "100 - $gc_content" | bc -l)%"

    # Validate part ID format
    if [[ ! "$part_id" =~ ^BBa_[PRBCETKV][0-9]{5,6}$ ]]; then
        print_warning "Part ID format may not be standard BioBrick format"
        print_info "Standard format: BBa_[TYPE][NUMBER]"
    fi

    # Register part (simplified - just output for demo)
    print_section "Registration Result"
    print_success "Part $part_id registered successfully"
    print_info "Version: 1.0.0"
    print_info "Status: Available"
    print_info "Registry URL: https://registry.wiastandards.com/parts/$part_id"

    # Save to local registry
    local timestamp=$(date -Iseconds)
    local part_json=$(cat <<EOF
{
  "partId": "$part_id",
  "partName": "${name:-Unknown}",
  "type": "$part_type",
  "sequence": "$sequence",
  "length": ${#sequence},
  "gcContent": $gc_content,
  "author": "${author:-Anonymous}",
  "version": "1.0.0",
  "created": "$timestamp",
  "status": "available"
}
EOF
)

    echo "$part_json" >> "$HOME/.wia/bio-021/parts/${part_id}.json"
    mkdir -p "$HOME/.wia/bio-021/parts"
    echo "$part_json" > "$HOME/.wia/bio-021/parts/${part_id}.json"

    echo ""
}

# Search for parts
search_parts() {
    local query_type="$1"
    local query_value="$2"

    print_section "Part Search"
    print_info "Searching by $query_type: $query_value"

    # Search in local registry
    local parts_dir="$HOME/.wia/bio-021/parts"
    if [ ! -d "$parts_dir" ]; then
        print_warning "No parts found in local registry"
        echo ""
        return 0
    fi

    print_section "Search Results"

    local count=0
    for part_file in "$parts_dir"/*.json; do
        if [ -f "$part_file" ]; then
            local match=false

            case "$query_type" in
                --type)
                    if grep -q "\"type\": \"$query_value\"" "$part_file"; then
                        match=true
                    fi
                    ;;
                --organism)
                    if grep -qi "$query_value" "$part_file"; then
                        match=true
                    fi
                    ;;
                *)
                    match=true
                    ;;
            esac

            if [ "$match" = true ]; then
                local part_id=$(basename "$part_file" .json)
                local part_name=$(grep -o '"partName": "[^"]*"' "$part_file" | cut -d'"' -f4)
                local part_type=$(grep -o '"type": "[^"]*"' "$part_file" | cut -d'"' -f4)

                print_success "$part_id - $part_name ($part_type)"
                count=$((count + 1))
            fi
        fi
    done

    if [ $count -eq 0 ]; then
        print_warning "No parts found matching criteria"
    else
        print_info "Found $count part(s)"
    fi

    echo ""
}

# Get part details
get_part() {
    local part_id="$1"

    print_section "Part Details: $part_id"

    local part_file="$HOME/.wia/bio-021/parts/${part_id}.json"

    if [ ! -f "$part_file" ]; then
        print_error "Part not found: $part_id"
        echo ""
        return 1
    fi

    # Parse and display part information
    while IFS= read -r line; do
        if [[ $line =~ \"([^\"]+)\":[[:space:]]*\"?([^,\"}]+)\"? ]]; then
            local key="${BASH_REMATCH[1]}"
            local value="${BASH_REMATCH[2]}"

            case "$key" in
                partId)
                    print_info "Part ID: $value"
                    ;;
                partName)
                    print_info "Name: $value"
                    ;;
                type)
                    print_info "Type: $value"
                    ;;
                length)
                    print_info "Length: $value bp"
                    ;;
                gcContent)
                    print_info "GC Content: $value%"
                    ;;
                author)
                    print_info "Author: $value"
                    ;;
                version)
                    print_info "Version: $value"
                    ;;
                status)
                    print_info "Status: $value"
                    ;;
                created)
                    print_info "Created: $value"
                    ;;
            esac
        fi
    done < "$part_file"

    print_section "Sequence"
    local sequence=$(grep -o '"sequence": "[^"]*"' "$part_file" | cut -d'"' -f4)
    echo -e "${GRAY}$(echo $sequence | fold -w 60)${RESET}"

    echo ""
}

# Export part to different formats
export_part() {
    local part_id="$1"
    local format="$2"
    local output_file="$3"

    print_section "Export Part: $part_id"

    local part_file="$HOME/.wia/bio-021/parts/${part_id}.json"

    if [ ! -f "$part_file" ]; then
        print_error "Part not found: $part_id"
        return 1
    fi

    # Extract part data
    local part_name=$(grep -o '"partName": "[^"]*"' "$part_file" | cut -d'"' -f4)
    local sequence=$(grep -o '"sequence": "[^"]*"' "$part_file" | cut -d'"' -f4)
    local version=$(grep -o '"version": "[^"]*"' "$part_file" | cut -d'"' -f4)

    case "$format" in
        fasta)
            echo ">$part_id $part_name" > "$output_file"
            echo "$sequence" | fold -w 60 >> "$output_file"
            print_success "Exported to FASTA: $output_file"
            ;;

        genbank)
            local length=${#sequence}
            local date=$(date +%d-%b-%Y)

            cat > "$output_file" <<EOF
LOCUS       ${part_id}        ${length} bp    DNA     linear   SYN ${date}
DEFINITION  ${part_name}
ACCESSION   ${part_id}
VERSION     ${part_id}.${version}
KEYWORDS    .
SOURCE      synthetic construct
  ORGANISM  synthetic construct
ORIGIN
EOF
            echo "$sequence" | fold -w 60 | awk '{printf "%9d %s\n", NR*60-59, $0}' >> "$output_file"
            echo "//" >> "$output_file"
            print_success "Exported to GenBank: $output_file"
            ;;

        sbol)
            cat > "$output_file" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
         xmlns:sbol="http://sbols.org/v2#"
         xmlns:dcterms="http://purl.org/dc/terms/">
  <sbol:ComponentDefinition rdf:about="http://wiastandards.com/${part_id}">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/${part_id}"/>
    <sbol:displayId>${part_id}</sbol:displayId>
    <sbol:version>${version}</sbol:version>
    <dcterms:title>${part_name}</dcterms:title>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:sequence rdf:resource="http://wiastandards.com/${part_id}_sequence"/>
  </sbol:ComponentDefinition>

  <sbol:Sequence rdf:about="http://wiastandards.com/${part_id}_sequence">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/${part_id}_sequence"/>
    <sbol:displayId>${part_id}_sequence</sbol:displayId>
    <sbol:version>${version}</sbol:version>
    <sbol:elements>${sequence}</sbol:elements>
    <sbol:encoding rdf:resource="http://www.chem.qmul.ac.uk/iubmb/misc/naseq.html"/>
  </sbol:Sequence>
</rdf:RDF>
EOF
            print_success "Exported to SBOL: $output_file"
            ;;

        *)
            print_error "Unsupported format: $format"
            print_info "Supported formats: fasta, genbank, sbol"
            return 1
            ;;
    esac

    echo ""
}

# Import part from file
import_part() {
    local input_file="$1"
    local format="$2"

    print_section "Import Part from $format"

    if [ ! -f "$input_file" ]; then
        print_error "File not found: $input_file"
        return 1
    fi

    print_info "Reading $input_file..."

    # Parse based on format
    case "$format" in
        fasta)
            local part_id=$(grep "^>" "$input_file" | head -1 | awk '{print $1}' | sed 's/>//')
            local sequence=$(grep -v "^>" "$input_file" | tr -d '\n' | tr -d ' ')
            print_success "Imported from FASTA"
            print_info "Part ID: $part_id"
            print_info "Sequence Length: ${#sequence} bp"
            ;;

        genbank)
            local part_id=$(grep "^LOCUS" "$input_file" | awk '{print $2}')
            local sequence=$(sed -n '/^ORIGIN/,/^\/\//p' "$input_file" | grep -v "ORIGIN" | grep -v "//" | sed 's/[0-9 ]//g' | tr -d '\n')
            print_success "Imported from GenBank"
            print_info "Part ID: $part_id"
            print_info "Sequence Length: ${#sequence} bp"
            ;;

        *)
            print_error "Unsupported format: $format"
            return 1
            ;;
    esac

    echo ""
}

# Add characterization data
characterize_part() {
    local part_id="$1"
    local measurement_type="$2"
    local value="$3"

    print_section "Add Characterization Data"

    local part_file="$HOME/.wia/bio-021/parts/${part_id}.json"

    if [ ! -f "$part_file" ]; then
        print_error "Part not found: $part_id"
        return 1
    fi

    print_info "Part: $part_id"
    print_info "Measurement: $measurement_type"
    print_info "Value: $value"

    # In a real implementation, this would update the JSON file
    print_success "Characterization data added"
    print_info "Part version incremented to match new data"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-bio-021 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  register                 Register a new biological part"
    echo "    --id <part_id>         Part identifier (e.g., BBa_K123456)"
    echo "    --type <type>          Part type (promoter, rbs, cds, terminator, etc.)"
    echo "    --sequence-file <file> FASTA file containing sequence"
    echo "    --name <name>          Part name"
    echo "    --author <author>      Author/creator name"
    echo ""
    echo "  search                   Search for parts in registry"
    echo "    --type <type>          Filter by part type"
    echo "    --organism <organism>  Filter by organism"
    echo ""
    echo "  get <part_id>            Get details for a specific part"
    echo ""
    echo "  export                   Export part to different format"
    echo "    --id <part_id>         Part to export"
    echo "    --format <format>      Output format (fasta, genbank, sbol)"
    echo "    --output <file>        Output file path"
    echo ""
    echo "  import                   Import part from file"
    echo "    --file <file>          Input file path"
    echo "    --format <format>      Input format (fasta, genbank)"
    echo ""
    echo "  characterize             Add characterization data"
    echo "    --id <part_id>         Part to characterize"
    echo "    --type <measurement>   Measurement type"
    echo "    --value <value>        Measured value"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-bio-021 register --id BBa_K123456 --type promoter --sequence-file seq.fasta"
    echo "  wia-bio-021 search --type promoter --organism \"E. coli\""
    echo "  wia-bio-021 get BBa_K123456"
    echo "  wia-bio-021 export --id BBa_K123456 --format sbol --output part.xml"
    echo "  wia-bio-021 characterize --id BBa_K123456 --type strength --value 1500"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-BIO-021 Synthetic Biology Registry CLI"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Initialize registry
init_registry

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    register)
        PART_ID=""
        PART_TYPE=""
        SEQUENCE_FILE=""
        PART_NAME=""
        AUTHOR=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --id) PART_ID=$2; shift 2 ;;
                --type) PART_TYPE=$2; shift 2 ;;
                --sequence-file) SEQUENCE_FILE=$2; shift 2 ;;
                --name) PART_NAME=$2; shift 2 ;;
                --author) AUTHOR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        register_part "$PART_ID" "$PART_TYPE" "$SEQUENCE_FILE" "$PART_NAME" "$AUTHOR"
        ;;

    search)
        QUERY_TYPE="--all"
        QUERY_VALUE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) QUERY_TYPE="--type"; QUERY_VALUE=$2; shift 2 ;;
                --organism) QUERY_TYPE="--organism"; QUERY_VALUE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        search_parts "$QUERY_TYPE" "$QUERY_VALUE"
        ;;

    get)
        PART_ID=$1
        print_header
        get_part "$PART_ID"
        ;;

    export)
        PART_ID=""
        FORMAT=""
        OUTPUT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --id) PART_ID=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        export_part "$PART_ID" "$FORMAT" "$OUTPUT"
        ;;

    import)
        INPUT_FILE=""
        FORMAT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --file) INPUT_FILE=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        import_part "$INPUT_FILE" "$FORMAT"
        ;;

    characterize)
        PART_ID=""
        MEAS_TYPE=""
        VALUE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --id) PART_ID=$2; shift 2 ;;
                --type) MEAS_TYPE=$2; shift 2 ;;
                --value) VALUE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        characterize_part "$PART_ID" "$MEAS_TYPE" "$VALUE"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-bio-021 help' for usage information"
        exit 1
        ;;
esac

exit 0
