#!/bin/bash

###############################################################################
# WIA-LEGAL-001: Digital Court Standard - CLI Tool
# Version: 1.0.0
#
# 弘益人間 (Hongik Ingan) - Benefit All Humanity
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
API_BASE_URL="${WIA_API_URL:-https://api.digitalcourt.wia.org/v1}"
API_KEY="${WIA_API_KEY:-}"

# Functions
print_header() {
    echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${PURPLE}⚖️  WIA-LEGAL-001: Digital Court Standard${NC}"
    echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not found. Set WIA_API_KEY environment variable."
        exit 1
    fi
}

# Commands

create_case() {
    check_api_key

    local case_type="$1"
    local plaintiff="$2"
    local defendant="$3"

    print_info "Creating new case..."

    local case_number="$(date +%Y)-CV-$(printf "%06d" $RANDOM)"

    local payload=$(cat <<EOF
{
  "caseNumber": "$case_number",
  "caseType": "$case_type",
  "court": {
    "courtId": "court-digital-001",
    "jurisdiction": "WIA-GLOBAL"
  },
  "parties": {
    "plaintiffs": [{"name": "$plaintiff", "type": "individual", "role": "plaintiff"}],
    "defendants": [{"name": "$defendant", "type": "individual", "role": "defendant"}]
  },
  "description": "Case created via CLI"
}
EOF
)

    local response=$(curl -s -X POST "$API_BASE_URL/cases" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload")

    if echo "$response" | grep -q '"success":true'; then
        local case_id=$(echo "$response" | grep -o '"caseId":"[^"]*"' | cut -d'"' -f4)
        print_success "Case created successfully!"
        echo -e "  Case Number: ${CYAN}$case_number${NC}"
        echo -e "  Case ID: ${CYAN}$case_id${NC}"
    else
        print_error "Failed to create case"
        echo "$response"
        exit 1
    fi
}

file_document() {
    check_api_key

    local case_id="$1"
    local doc_type="$2"
    local file_path="$3"

    if [ ! -f "$file_path" ]; then
        print_error "File not found: $file_path"
        exit 1
    fi

    print_info "Filing document..."

    local response=$(curl -s -X POST "$API_BASE_URL/documents" \
        -H "Authorization: Bearer $API_KEY" \
        -F "caseId=$case_id" \
        -F "type=$doc_type" \
        -F "title=$(basename "$file_path")" \
        -F "filedBy=cli-user" \
        -F "confidentiality=public" \
        -F "file=@$file_path")

    if echo "$response" | grep -q '"success":true'; then
        local doc_id=$(echo "$response" | grep -o '"documentId":"[^"]*"' | cut -d'"' -f4)
        print_success "Document filed successfully!"
        echo -e "  Document ID: ${CYAN}$doc_id${NC}"
    else
        print_error "Failed to file document"
        echo "$response"
        exit 1
    fi
}

schedule_hearing() {
    check_api_key

    local case_id="$1"
    local hearing_type="$2"
    local scheduled_date="$3"

    print_info "Scheduling hearing..."

    local payload=$(cat <<EOF
{
  "caseId": "$case_id",
  "type": "$hearing_type",
  "scheduledDate": "$scheduled_date",
  "duration": 60,
  "mode": "virtual",
  "recording": true
}
EOF
)

    local response=$(curl -s -X POST "$API_BASE_URL/hearings" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload")

    if echo "$response" | grep -q '"success":true'; then
        local hearing_id=$(echo "$response" | grep -o '"hearingId":"[^"]*"' | cut -d'"' -f4)
        local virtual_url=$(echo "$response" | grep -o '"virtualRoomUrl":"[^"]*"' | cut -d'"' -f4)
        print_success "Hearing scheduled successfully!"
        echo -e "  Hearing ID: ${CYAN}$hearing_id${NC}"
        echo -e "  Virtual Room: ${CYAN}$virtual_url${NC}"
    else
        print_error "Failed to schedule hearing"
        echo "$response"
        exit 1
    fi
}

list_cases() {
    check_api_key

    print_info "Fetching cases..."

    local response=$(curl -s -X GET "$API_BASE_URL/cases" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.caseNumber) - \(.status) - \(.caseType)"'
}

get_case() {
    check_api_key

    local case_id="$1"

    print_info "Fetching case details..."

    local response=$(curl -s -X GET "$API_BASE_URL/cases/$case_id" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq '.'
}

generate_report() {
    check_api_key

    local case_id="$1"
    local format="${2:-json}"

    print_info "Generating case report..."

    local response=$(curl -s -X GET "$API_BASE_URL/cases/$case_id" \
        -H "Authorization: Bearer $API_KEY")

    if [ "$format" = "json" ]; then
        echo "$response" | jq '.'
    elif [ "$format" = "summary" ]; then
        echo "=== Case Summary ==="
        echo "$response" | jq -r '"Case: \(.data.caseNumber)\nType: \(.data.caseType)\nStatus: \(.data.status)\nFiled: \(.data.filingDate)"'
    fi
}

verify_document() {
    local file_path="$1"

    if [ ! -f "$file_path" ]; then
        print_error "File not found: $file_path"
        exit 1
    fi

    print_info "Verifying document integrity..."

    local hash=$(sha256sum "$file_path" | cut -d' ' -f1)

    print_success "Document hash calculated"
    echo -e "  SHA-256: ${CYAN}$hash${NC}"
}

show_help() {
    print_header
    echo "Usage: $0 <command> [options]"
    echo
    echo "Commands:"
    echo "  create-case --type <type> --plaintiff <name> --defendant <name>"
    echo "    Create a new case"
    echo
    echo "  file-document --case-id <id> --type <type> --file <path>"
    echo "    File a document to a case"
    echo
    echo "  schedule-hearing --case-id <id> --type <type> --date <ISO-8601>"
    echo "    Schedule a virtual hearing"
    echo
    echo "  list-cases"
    echo "    List all cases"
    echo
    echo "  get-case --case-id <id>"
    echo "    Get case details"
    echo
    echo "  generate-report --case-id <id> [--format <json|summary>]"
    echo "    Generate case report"
    echo
    echo "  verify-document --file <path>"
    echo "    Verify document integrity (calculate hash)"
    echo
    echo "Environment Variables:"
    echo "  WIA_API_KEY     - Your WIA API key (required)"
    echo "  WIA_API_URL     - API base URL (optional)"
    echo
    echo "Examples:"
    echo "  $0 create-case --type civil --plaintiff \"John Doe\" --defendant \"Acme Corp\""
    echo "  $0 file-document --case-id 550e8400 --type motion --file motion.pdf"
    echo "  $0 schedule-hearing --case-id 550e8400 --type preliminary --date \"2026-01-15T14:00:00Z\""
    echo
    echo "弘益人間 (Hongik Ingan) - Benefit All Humanity"
}

# Main
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command="$1"
    shift

    case "$command" in
        create-case)
            local case_type="" plaintiff="" defendant=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --type) case_type="$2"; shift 2;;
                    --plaintiff) plaintiff="$2"; shift 2;;
                    --defendant) defendant="$2"; shift 2;;
                    *) shift;;
                esac
            done
            create_case "$case_type" "$plaintiff" "$defendant"
            ;;
        file-document)
            local case_id="" doc_type="" file_path=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --case-id) case_id="$2"; shift 2;;
                    --type) doc_type="$2"; shift 2;;
                    --file) file_path="$2"; shift 2;;
                    *) shift;;
                esac
            done
            file_document "$case_id" "$doc_type" "$file_path"
            ;;
        schedule-hearing)
            local case_id="" hearing_type="" scheduled_date=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --case-id) case_id="$2"; shift 2;;
                    --type) hearing_type="$2"; shift 2;;
                    --date) scheduled_date="$2"; shift 2;;
                    *) shift;;
                esac
            done
            schedule_hearing "$case_id" "$hearing_type" "$scheduled_date"
            ;;
        list-cases)
            list_cases
            ;;
        get-case)
            local case_id=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --case-id) case_id="$2"; shift 2;;
                    *) shift;;
                esac
            done
            get_case "$case_id"
            ;;
        generate-report)
            local case_id="" format="json"
            while [ $# -gt 0 ]; do
                case "$1" in
                    --case-id) case_id="$2"; shift 2;;
                    --format) format="$2"; shift 2;;
                    *) shift;;
                esac
            done
            generate_report "$case_id" "$format"
            ;;
        verify-document)
            local file_path=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --file) file_path="$2"; shift 2;;
                    *) shift;;
                esac
            done
            verify_document "$file_path"
            ;;
        --help|-h|help)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            show_help
            exit 1
            ;;
    esac
}

print_header
main "$@"
