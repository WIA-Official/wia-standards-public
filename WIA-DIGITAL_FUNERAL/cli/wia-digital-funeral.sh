#!/bin/bash

###############################################################################
# WIA-DIGITAL_FUNERAL: Digital Funeral Standard - CLI Tool
# Version: 1.0.0
#
# 弘益人間 (Hongik Ingan) - Benefit All Humanity
#
# This CLI provides comprehensive tools for managing digital funeral services,
# virtual memorials, livestreamed ceremonies, and digital legacy preservation.
###############################################################################

set -e

# Colors - Purple/Violet Theme
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
VIOLET='\033[38;5;141m'
BRIGHT_PURPLE='\033[38;5;165m'
DARK_PURPLE='\033[38;5;93m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
API_BASE_URL="${WIA_API_URL:-https://api.digitalfuneral.wia.org/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_DIR="$HOME/.wia-digital-funeral"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Ensure config directory exists
mkdir -p "$CONFIG_DIR"

# Functions
print_header() {
    echo -e "${BRIGHT_PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BRIGHT_PURPLE}🕊️  WIA-DIGITAL_FUNERAL: Digital Funeral Standard${NC}"
    echo -e "${BRIGHT_PURPLE}    Version 1.0.0${NC}"
    echo -e "${BRIGHT_PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${VIOLET}    弘益人間 · Benefit All Humanity${NC}"
    echo
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${VIOLET}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_section() {
    echo -e "${BRIGHT_PURPLE}▸ $1${NC}"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not found. Set WIA_API_KEY environment variable or run 'init' command."
        exit 1
    fi
}

# Command: init - Initialize configuration
cmd_init() {
    print_section "Initializing WIA Digital Funeral configuration..."

    read -p "Enter your API key: " api_key
    read -p "Enter your organization name: " org_name
    read -p "Enter your email: " email
    read -p "Enter API base URL [${API_BASE_URL}]: " custom_url

    if [ -n "$custom_url" ]; then
        API_BASE_URL="$custom_url"
    fi

    cat > "$CONFIG_FILE" <<EOF
{
  "apiKey": "$api_key",
  "organizationName": "$org_name",
  "email": "$email",
  "apiBaseUrl": "$API_BASE_URL",
  "version": "1.0.0",
  "initialized": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    print_success "Configuration saved to $CONFIG_FILE"
    print_info "You can now use all CLI commands"
    echo
    print_info "Export your API key for this session:"
    echo -e "  ${CYAN}export WIA_API_KEY=\"$api_key\"${NC}"
}

# Command: memorial - Create and manage virtual memorials
cmd_memorial() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        create)
            memorial_create "$@"
            ;;
        update)
            memorial_update "$@"
            ;;
        delete)
            memorial_delete "$@"
            ;;
        list)
            memorial_list
            ;;
        view)
            memorial_view "$@"
            ;;
        publish)
            memorial_publish "$@"
            ;;
        *)
            print_error "Unknown memorial subcommand: $subcommand"
            echo "Available subcommands: create, update, delete, list, view, publish"
            exit 1
            ;;
    esac
}

memorial_create() {
    check_api_key

    local deceased_name="" birth_date="" death_date="" biography="" privacy=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --name) deceased_name="$2"; shift 2;;
            --birth) birth_date="$2"; shift 2;;
            --death) death_date="$2"; shift 2;;
            --bio) biography="$2"; shift 2;;
            --privacy) privacy="$2"; shift 2;;
            *) shift;;
        esac
    done

    if [ -z "$deceased_name" ]; then
        print_error "Deceased name is required (--name)"
        exit 1
    fi

    print_section "Creating virtual memorial for $deceased_name..."

    local memorial_id="mem_$(date +%s)_$(printf "%04x" $RANDOM)"

    local payload=$(cat <<EOF
{
  "memorialId": "$memorial_id",
  "deceased": {
    "name": "$deceased_name",
    "birthDate": "$birth_date",
    "deathDate": "$death_date",
    "biography": "$biography"
  },
  "privacy": "${privacy:-public}",
  "features": {
    "tributeWall": true,
    "photoGallery": true,
    "videoMemories": true,
    "condolenceBook": true,
    "memorialWebsite": true,
    "qrCode": true
  },
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
)

    local response=$(curl -s -X POST "$API_BASE_URL/memorials" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Memorial created successfully!"
        echo -e "  Memorial ID: ${VIOLET}$memorial_id${NC}"
        local website_url=$(echo "$response" | grep -o '"websiteUrl":"[^"]*"' | cut -d'"' -f4)
        if [ -n "$website_url" ]; then
            echo -e "  Website: ${CYAN}$website_url${NC}"
        fi
        local qr_url=$(echo "$response" | grep -o '"qrCodeUrl":"[^"]*"' | cut -d'"' -f4)
        if [ -n "$qr_url" ]; then
            echo -e "  QR Code: ${CYAN}$qr_url${NC}"
        fi
    else
        print_error "Failed to create memorial"
        echo "$response"
        exit 1
    fi
}

memorial_update() {
    check_api_key

    local memorial_id="" field="" value=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --id) memorial_id="$2"; shift 2;;
            --field) field="$2"; shift 2;;
            --value) value="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Updating memorial $memorial_id..."

    local payload=$(cat <<EOF
{
  "$field": "$value",
  "updatedAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
)

    curl -s -X PATCH "$API_BASE_URL/memorials/$memorial_id" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload"

    print_success "Memorial updated successfully"
}

memorial_delete() {
    check_api_key

    local memorial_id="$1"

    if [ -z "$memorial_id" ]; then
        print_error "Memorial ID is required"
        exit 1
    fi

    print_warning "Are you sure you want to delete memorial $memorial_id? (yes/no)"
    read -r confirmation

    if [ "$confirmation" != "yes" ]; then
        print_info "Deletion cancelled"
        exit 0
    fi

    print_section "Deleting memorial..."

    curl -s -X DELETE "$API_BASE_URL/memorials/$memorial_id" \
        -H "Authorization: Bearer $API_KEY"

    print_success "Memorial deleted successfully"
}

memorial_list() {
    check_api_key

    print_section "Fetching memorials..."

    local response=$(curl -s -X GET "$API_BASE_URL/memorials" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.memorialId) - \(.deceased.name) (\(.deceased.birthDate) - \(.deceased.deathDate))"' 2>/dev/null || echo "$response"
}

memorial_view() {
    check_api_key

    local memorial_id="$1"

    if [ -z "$memorial_id" ]; then
        print_error "Memorial ID is required"
        exit 1
    fi

    print_section "Fetching memorial details..."

    local response=$(curl -s -X GET "$API_BASE_URL/memorials/$memorial_id" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq '.' 2>/dev/null || echo "$response"
}

memorial_publish() {
    check_api_key

    local memorial_id="$1"

    print_section "Publishing memorial to public..."

    curl -s -X POST "$API_BASE_URL/memorials/$memorial_id/publish" \
        -H "Authorization: Bearer $API_KEY"

    print_success "Memorial published successfully"
}

# Command: tribute - Manage tribute messages and condolences
cmd_tribute() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        add)
            tribute_add "$@"
            ;;
        list)
            tribute_list "$@"
            ;;
        moderate)
            tribute_moderate "$@"
            ;;
        *)
            print_error "Unknown tribute subcommand: $subcommand"
            echo "Available subcommands: add, list, moderate"
            exit 1
            ;;
    esac
}

tribute_add() {
    check_api_key

    local memorial_id="" author="" message="" type="condolence"

    while [ $# -gt 0 ]; do
        case "$1" in
            --memorial-id) memorial_id="$2"; shift 2;;
            --author) author="$2"; shift 2;;
            --message) message="$2"; shift 2;;
            --type) type="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Adding tribute message..."

    local payload=$(cat <<EOF
{
  "memorialId": "$memorial_id",
  "author": {
    "name": "$author",
    "relationship": "friend"
  },
  "type": "$type",
  "message": "$message",
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
)

    curl -s -X POST "$API_BASE_URL/tributes" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload"

    print_success "Tribute added successfully"
}

tribute_list() {
    check_api_key

    local memorial_id="$1"

    print_section "Fetching tributes..."

    local response=$(curl -s -X GET "$API_BASE_URL/memorials/$memorial_id/tributes" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.author.name): \(.message)"' 2>/dev/null || echo "$response"
}

tribute_moderate() {
    check_api_key

    local tribute_id="" action=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --id) tribute_id="$2"; shift 2;;
            --action) action="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Moderating tribute..."

    curl -s -X POST "$API_BASE_URL/tributes/$tribute_id/moderate" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"action\": \"$action\"}"

    print_success "Tribute moderated: $action"
}

# Command: ceremony - Manage funeral ceremony details
cmd_ceremony() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        create)
            ceremony_create "$@"
            ;;
        update)
            ceremony_update "$@"
            ;;
        list)
            ceremony_list
            ;;
        cancel)
            ceremony_cancel "$@"
            ;;
        *)
            print_error "Unknown ceremony subcommand: $subcommand"
            echo "Available subcommands: create, update, list, cancel"
            exit 1
            ;;
    esac
}

ceremony_create() {
    check_api_key

    local memorial_id="" ceremony_type="" date="" location="" duration="60"

    while [ $# -gt 0 ]; do
        case "$1" in
            --memorial-id) memorial_id="$2"; shift 2;;
            --type) ceremony_type="$2"; shift 2;;
            --date) date="$2"; shift 2;;
            --location) location="$2"; shift 2;;
            --duration) duration="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Creating ceremony..."

    local ceremony_id="cer_$(date +%s)_$(printf "%04x" $RANDOM)"

    local payload=$(cat <<EOF
{
  "ceremonyId": "$ceremony_id",
  "memorialId": "$memorial_id",
  "type": "$ceremony_type",
  "scheduledDate": "$date",
  "location": {
    "type": "hybrid",
    "physical": "$location",
    "virtual": "https://stream.digitalfuneral.wia.org/$ceremony_id"
  },
  "duration": $duration,
  "features": {
    "livestream": true,
    "recording": true,
    "virtualAttendance": true,
    "chatEnabled": true,
    "tributeSlides": true
  }
}
EOF
)

    local response=$(curl -s -X POST "$API_BASE_URL/ceremonies" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Ceremony created successfully!"
        echo -e "  Ceremony ID: ${VIOLET}$ceremony_id${NC}"
        echo -e "  Stream URL: ${CYAN}https://stream.digitalfuneral.wia.org/$ceremony_id${NC}"
    else
        print_error "Failed to create ceremony"
        echo "$response"
        exit 1
    fi
}

ceremony_update() {
    check_api_key

    local ceremony_id="" field="" value=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --id) ceremony_id="$2"; shift 2;;
            --field) field="$2"; shift 2;;
            --value) value="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Updating ceremony..."

    curl -s -X PATCH "$API_BASE_URL/ceremonies/$ceremony_id" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"$field\": \"$value\"}"

    print_success "Ceremony updated successfully"
}

ceremony_list() {
    check_api_key

    print_section "Fetching ceremonies..."

    local response=$(curl -s -X GET "$API_BASE_URL/ceremonies" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.ceremonyId) - \(.type) - \(.scheduledDate)"' 2>/dev/null || echo "$response"
}

ceremony_cancel() {
    check_api_key

    local ceremony_id="$1"

    print_warning "Are you sure you want to cancel ceremony $ceremony_id? (yes/no)"
    read -r confirmation

    if [ "$confirmation" != "yes" ]; then
        print_info "Cancellation aborted"
        exit 0
    fi

    print_section "Cancelling ceremony..."

    curl -s -X POST "$API_BASE_URL/ceremonies/$ceremony_id/cancel" \
        -H "Authorization: Bearer $API_KEY"

    print_success "Ceremony cancelled successfully"
}

# Command: livestream - Manage livestream services
cmd_livestream() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        start)
            livestream_start "$@"
            ;;
        stop)
            livestream_stop "$@"
            ;;
        status)
            livestream_status "$@"
            ;;
        viewers)
            livestream_viewers "$@"
            ;;
        record)
            livestream_record "$@"
            ;;
        *)
            print_error "Unknown livestream subcommand: $subcommand"
            echo "Available subcommands: start, stop, status, viewers, record"
            exit 1
            ;;
    esac
}

livestream_start() {
    check_api_key

    local ceremony_id="" quality="1080p" backup_stream="true"

    while [ $# -gt 0 ]; do
        case "$1" in
            --ceremony-id) ceremony_id="$2"; shift 2;;
            --quality) quality="$2"; shift 2;;
            --backup) backup_stream="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Starting livestream for ceremony $ceremony_id..."

    local payload=$(cat <<EOF
{
  "ceremonyId": "$ceremony_id",
  "settings": {
    "quality": "$quality",
    "backupStream": $backup_stream,
    "lowLatency": true,
    "adaptiveBitrate": true,
    "encryption": "AES-256"
  }
}
EOF
)

    local response=$(curl -s -X POST "$API_BASE_URL/livestream/start" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload")

    if echo "$response" | grep -q '"success":true'; then
        print_success "Livestream started successfully!"
        local stream_url=$(echo "$response" | grep -o '"streamUrl":"[^"]*"' | cut -d'"' -f4)
        local stream_key=$(echo "$response" | grep -o '"streamKey":"[^"]*"' | cut -d'"' -f4)
        echo -e "  Stream URL: ${CYAN}$stream_url${NC}"
        echo -e "  Stream Key: ${YELLOW}$stream_key${NC}"
    else
        print_error "Failed to start livestream"
        echo "$response"
        exit 1
    fi
}

livestream_stop() {
    check_api_key

    local ceremony_id="$1"

    print_section "Stopping livestream..."

    curl -s -X POST "$API_BASE_URL/livestream/stop" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"ceremonyId\": \"$ceremony_id\"}"

    print_success "Livestream stopped successfully"
}

livestream_status() {
    check_api_key

    local ceremony_id="$1"

    print_section "Fetching livestream status..."

    local response=$(curl -s -X GET "$API_BASE_URL/livestream/$ceremony_id/status" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq '.' 2>/dev/null || echo "$response"
}

livestream_viewers() {
    check_api_key

    local ceremony_id="$1"

    print_section "Fetching viewer count..."

    local response=$(curl -s -X GET "$API_BASE_URL/livestream/$ceremony_id/viewers" \
        -H "Authorization: Bearer $API_KEY")

    local count=$(echo "$response" | grep -o '"currentViewers":[0-9]*' | cut -d':' -f2)
    local peak=$(echo "$response" | grep -o '"peakViewers":[0-9]*' | cut -d':' -f2)

    echo -e "  Current viewers: ${VIOLET}$count${NC}"
    echo -e "  Peak viewers: ${VIOLET}$peak${NC}"
}

livestream_record() {
    check_api_key

    local ceremony_id="" action="start"

    while [ $# -gt 0 ]; do
        case "$1" in
            --ceremony-id) ceremony_id="$2"; shift 2;;
            --action) action="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Recording control: $action..."

    curl -s -X POST "$API_BASE_URL/livestream/$ceremony_id/record" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"action\": \"$action\"}"

    print_success "Recording $action executed"
}

# Command: archive - Manage digital archives and recordings
cmd_archive() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        list)
            archive_list
            ;;
        download)
            archive_download "$@"
            ;;
        upload)
            archive_upload "$@"
            ;;
        delete)
            archive_delete "$@"
            ;;
        *)
            print_error "Unknown archive subcommand: $subcommand"
            echo "Available subcommands: list, download, upload, delete"
            exit 1
            ;;
    esac
}

archive_list() {
    check_api_key

    print_section "Fetching archives..."

    local response=$(curl -s -X GET "$API_BASE_URL/archives" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.archiveId) - \(.type) - \(.size) - \(.createdAt)"' 2>/dev/null || echo "$response"
}

archive_download() {
    check_api_key

    local archive_id="" output_path=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --id) archive_id="$2"; shift 2;;
            --output) output_path="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Downloading archive $archive_id..."

    local download_url=$(curl -s -X GET "$API_BASE_URL/archives/$archive_id/download" \
        -H "Authorization: Bearer $API_KEY" | grep -o '"url":"[^"]*"' | cut -d'"' -f4)

    if [ -n "$download_url" ]; then
        curl -L -o "$output_path" "$download_url"
        print_success "Archive downloaded to $output_path"
    else
        print_error "Failed to get download URL"
        exit 1
    fi
}

archive_upload() {
    check_api_key

    local memorial_id="" file_path="" type="recording"

    while [ $# -gt 0 ]; do
        case "$1" in
            --memorial-id) memorial_id="$2"; shift 2;;
            --file) file_path="$2"; shift 2;;
            --type) type="$2"; shift 2;;
            *) shift;;
        esac
    done

    if [ ! -f "$file_path" ]; then
        print_error "File not found: $file_path"
        exit 1
    fi

    print_section "Uploading archive..."

    local response=$(curl -s -X POST "$API_BASE_URL/archives" \
        -H "Authorization: Bearer $API_KEY" \
        -F "memorialId=$memorial_id" \
        -F "type=$type" \
        -F "file=@$file_path")

    if echo "$response" | grep -q '"success":true'; then
        local archive_id=$(echo "$response" | grep -o '"archiveId":"[^"]*"' | cut -d'"' -f4)
        print_success "Archive uploaded successfully!"
        echo -e "  Archive ID: ${VIOLET}$archive_id${NC}"
    else
        print_error "Failed to upload archive"
        echo "$response"
        exit 1
    fi
}

archive_delete() {
    check_api_key

    local archive_id="$1"

    print_warning "Are you sure you want to delete archive $archive_id? (yes/no)"
    read -r confirmation

    if [ "$confirmation" != "yes" ]; then
        print_info "Deletion cancelled"
        exit 0
    fi

    print_section "Deleting archive..."

    curl -s -X DELETE "$API_BASE_URL/archives/$archive_id" \
        -H "Authorization: Bearer $API_KEY"

    print_success "Archive deleted successfully"
}

# Command: notify - Manage notifications and invitations
cmd_notify() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        send)
            notify_send "$@"
            ;;
        template)
            notify_template "$@"
            ;;
        list)
            notify_list
            ;;
        *)
            print_error "Unknown notify subcommand: $subcommand"
            echo "Available subcommands: send, template, list"
            exit 1
            ;;
    esac
}

notify_send() {
    check_api_key

    local ceremony_id="" recipients="" message="" method="email"

    while [ $# -gt 0 ]; do
        case "$1" in
            --ceremony-id) ceremony_id="$2"; shift 2;;
            --recipients) recipients="$2"; shift 2;;
            --message) message="$2"; shift 2;;
            --method) method="$2"; shift 2;;
            *) shift;;
        esac
    done

    print_section "Sending notifications..."

    local payload=$(cat <<EOF
{
  "ceremonyId": "$ceremony_id",
  "recipients": "$recipients",
  "message": "$message",
  "method": "$method",
  "sentAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
)

    curl -s -X POST "$API_BASE_URL/notifications" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "$payload"

    print_success "Notifications sent successfully"
}

notify_template() {
    check_api_key

    local template_name="$1"

    print_section "Fetching notification template: $template_name..."

    local response=$(curl -s -X GET "$API_BASE_URL/notifications/templates/$template_name" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq '.' 2>/dev/null || echo "$response"
}

notify_list() {
    check_api_key

    print_section "Fetching sent notifications..."

    local response=$(curl -s -X GET "$API_BASE_URL/notifications" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.id) - \(.method) - \(.status) - \(.sentAt)"' 2>/dev/null || echo "$response"
}

# Command: manage - Administrative management functions
cmd_manage() {
    local subcommand="$1"
    shift

    case "$subcommand" in
        stats)
            manage_stats
            ;;
        backup)
            manage_backup "$@"
            ;;
        restore)
            manage_restore "$@"
            ;;
        audit)
            manage_audit
            ;;
        *)
            print_error "Unknown manage subcommand: $subcommand"
            echo "Available subcommands: stats, backup, restore, audit"
            exit 1
            ;;
    esac
}

manage_stats() {
    check_api_key

    print_section "Fetching statistics..."

    local response=$(curl -s -X GET "$API_BASE_URL/admin/stats" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq '.' 2>/dev/null || echo "$response"
}

manage_backup() {
    check_api_key

    local memorial_id="$1"

    print_section "Creating backup..."

    local response=$(curl -s -X POST "$API_BASE_URL/admin/backup" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"memorialId\": \"$memorial_id\"}")

    if echo "$response" | grep -q '"success":true'; then
        local backup_id=$(echo "$response" | grep -o '"backupId":"[^"]*"' | cut -d'"' -f4)
        print_success "Backup created successfully!"
        echo -e "  Backup ID: ${VIOLET}$backup_id${NC}"
    else
        print_error "Failed to create backup"
        echo "$response"
        exit 1
    fi
}

manage_restore() {
    check_api_key

    local backup_id="$1"

    print_warning "Are you sure you want to restore from backup $backup_id? (yes/no)"
    read -r confirmation

    if [ "$confirmation" != "yes" ]; then
        print_info "Restore cancelled"
        exit 0
    fi

    print_section "Restoring from backup..."

    curl -s -X POST "$API_BASE_URL/admin/restore" \
        -H "Authorization: Bearer $API_KEY" \
        -H "Content-Type: application/json" \
        -d "{\"backupId\": \"$backup_id\"}"

    print_success "Restore completed successfully"
}

manage_audit() {
    check_api_key

    print_section "Fetching audit logs..."

    local response=$(curl -s -X GET "$API_BASE_URL/admin/audit" \
        -H "Authorization: Bearer $API_KEY")

    echo "$response" | jq -r '.data[] | "\(.timestamp) - \(.action) - \(.user) - \(.resource)"' 2>/dev/null || echo "$response"
}

# Command: help - Display help information
cmd_help() {
    print_header
    echo "Usage: $0 <command> [subcommand] [options]"
    echo
    echo -e "${BRIGHT_PURPLE}Commands:${NC}"
    echo
    echo -e "${VIOLET}  init${NC}"
    echo "    Initialize configuration with API credentials"
    echo
    echo -e "${VIOLET}  memorial${NC} <subcommand>"
    echo "    Create and manage virtual memorials"
    echo "    Subcommands: create, update, delete, list, view, publish"
    echo
    echo -e "${VIOLET}  tribute${NC} <subcommand>"
    echo "    Manage tribute messages and condolences"
    echo "    Subcommands: add, list, moderate"
    echo
    echo -e "${VIOLET}  ceremony${NC} <subcommand>"
    echo "    Manage funeral ceremony details"
    echo "    Subcommands: create, update, list, cancel"
    echo
    echo -e "${VIOLET}  livestream${NC} <subcommand>"
    echo "    Manage livestream services"
    echo "    Subcommands: start, stop, status, viewers, record"
    echo
    echo -e "${VIOLET}  archive${NC} <subcommand>"
    echo "    Manage digital archives and recordings"
    echo "    Subcommands: list, download, upload, delete"
    echo
    echo -e "${VIOLET}  notify${NC} <subcommand>"
    echo "    Manage notifications and invitations"
    echo "    Subcommands: send, template, list"
    echo
    echo -e "${VIOLET}  manage${NC} <subcommand>"
    echo "    Administrative management functions"
    echo "    Subcommands: stats, backup, restore, audit"
    echo
    echo -e "${VIOLET}  help${NC}"
    echo "    Display this help message"
    echo
    echo -e "${VIOLET}  version${NC}"
    echo "    Display version information"
    echo
    echo -e "${BRIGHT_PURPLE}Environment Variables:${NC}"
    echo "  WIA_API_KEY     - Your WIA API key (required)"
    echo "  WIA_API_URL     - API base URL (optional)"
    echo
    echo -e "${BRIGHT_PURPLE}Examples:${NC}"
    echo
    echo "  # Initialize configuration"
    echo -e "  ${CYAN}$0 init${NC}"
    echo
    echo "  # Create a virtual memorial"
    echo -e "  ${CYAN}$0 memorial create --name \"John Doe\" --birth \"1950-01-01\" --death \"2025-01-01\" --privacy public${NC}"
    echo
    echo "  # Start livestream for ceremony"
    echo -e "  ${CYAN}$0 livestream start --ceremony-id cer_12345 --quality 1080p${NC}"
    echo
    echo "  # Add tribute message"
    echo -e "  ${CYAN}$0 tribute add --memorial-id mem_67890 --author \"Jane Smith\" --message \"Rest in peace\"${NC}"
    echo
    echo "  # Upload archive"
    echo -e "  ${CYAN}$0 archive upload --memorial-id mem_67890 --file recording.mp4 --type video${NC}"
    echo
    echo -e "${VIOLET}弘益人間 (Hongik Ingan) - Benefit All Humanity${NC}"
}

# Command: version - Display version information
cmd_version() {
    print_header
    echo -e "${VIOLET}Version:${NC} 1.0.0"
    echo -e "${VIOLET}API Base URL:${NC} $API_BASE_URL"
    echo -e "${VIOLET}Config Directory:${NC} $CONFIG_DIR"
    echo
    if [ -f "$CONFIG_FILE" ]; then
        echo -e "${GREEN}✓${NC} Configuration found"
        local org=$(jq -r '.organizationName' "$CONFIG_FILE" 2>/dev/null)
        if [ -n "$org" ] && [ "$org" != "null" ]; then
            echo -e "${VIOLET}Organization:${NC} $org"
        fi
    else
        echo -e "${YELLOW}⚠${NC} Configuration not found. Run 'init' command to set up."
    fi
}

# Main command dispatcher
main() {
    if [ $# -eq 0 ]; then
        cmd_help
        exit 0
    fi

    local command="$1"
    shift

    case "$command" in
        init)
            cmd_init
            ;;
        memorial)
            cmd_memorial "$@"
            ;;
        tribute)
            cmd_tribute "$@"
            ;;
        ceremony)
            cmd_ceremony "$@"
            ;;
        livestream)
            cmd_livestream "$@"
            ;;
        archive)
            cmd_archive "$@"
            ;;
        notify)
            cmd_notify "$@"
            ;;
        manage)
            cmd_manage "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        version|--version|-v)
            cmd_version
            ;;
        *)
            print_error "Unknown command: $command"
            echo
            cmd_help
            exit 1
            ;;
    esac
}

# Run main function
print_header
main "$@"
