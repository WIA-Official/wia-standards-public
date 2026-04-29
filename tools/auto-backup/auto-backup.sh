#!/bin/bash
#
# AUTO-BACKUP v1.0
# One-Click Server Backup Solution
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: curl -sSL https://wia.family/backup | bash
#    or: auto-backup [OPTIONS]
#

set -e

# ============================================================
# Version & Constants
# ============================================================
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/auto-backup.conf"
MODULES_DIR="${SCRIPT_DIR}/modules"

# ============================================================
# Colors & Formatting
# ============================================================
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    MAGENTA='\033[0;35m'
    BOLD='\033[1m'
    DIM='\033[2m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' CYAN='' MAGENTA='' BOLD='' DIM='' NC=''
fi

# ============================================================
# Default Configuration
# ============================================================
BACKUP_DIR="/var/backups/auto-backup"
LOG_FILE="/var/log/auto-backup.log"
TEMP_DIR="/tmp/auto-backup-$$"
HOSTNAME=$(hostname -f 2>/dev/null || hostname)
DATE_FORMAT="%Y-%m-%d"
DATETIME_FORMAT="%Y%m%d-%H%M%S"
CURRENT_DATE=$(date +"$DATE_FORMAT")
CURRENT_DATETIME=$(date +"$DATETIME_FORMAT")

# Backup targets
BACKUP_WEB=true
BACKUP_DB=true
BACKUP_CONFIG=true
BACKUP_SSL=true

# Directories
WEB_DIRS="/var/www"
CONFIG_DIRS="/etc/httpd /etc/nginx /etc/apache2 /etc/php* /etc/letsencrypt /etc/my.cnf* /etc/mysql"
EXCLUDE_PATTERNS="node_modules .git .cache cache tmp logs *.log"

# Database
MYSQL_USER="root"
MYSQL_PASSWORD=""
MYSQL_DATABASES="all"
PGSQL_USER="postgres"
PGSQL_DATABASES="all"
MONGO_DATABASES="all"

# Retention policy
KEEP_DAILY=7
KEEP_WEEKLY=4
KEEP_MONTHLY=3

# Cloud
CLOUD_ENABLED=false
CLOUD_PROVIDER="none"
S3_BUCKET=""
S3_REGION="ap-northeast-2"
S3_PATH="backups/"
GDRIVE_REMOTE="gdrive"
GDRIVE_PATH="Server-Backups/"

# Notifications
NOTIFY_ENABLED=false
NOTIFY_ON_SUCCESS=true
NOTIFY_ON_FAILURE=true
EMAIL_ENABLED=false
EMAIL_TO=""
SLACK_ENABLED=false
SLACK_WEBHOOK=""
TELEGRAM_ENABLED=false
TELEGRAM_BOT_TOKEN=""
TELEGRAM_CHAT_ID=""
DISCORD_ENABLED=false
DISCORD_WEBHOOK=""

# Advanced
COMPRESSION="gzip"
COMPRESSION_LEVEL=6
ENCRYPTION_ENABLED=false
ENCRYPTION_PASSWORD=""
PARALLEL_JOBS=2
DRY_RUN=false
VERBOSE=false

# Runtime variables
START_TIME=""
END_TIME=""
BACKUP_SIZE=0
BACKUP_FILE=""
DETECTED_WEBSERVER=""
DETECTED_DB=""
ERRORS=()

# ============================================================
# Load Configuration
# ============================================================
load_config() {
    if [[ -f "$CONFIG_FILE" ]]; then
        # shellcheck source=/dev/null
        source "$CONFIG_FILE"
    fi
}

# ============================================================
# Logging Functions
# ============================================================
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")

    # Log to file
    echo "[$timestamp] [$level] $message" >> "$LOG_FILE" 2>/dev/null || true

    # Console output based on level
    case "$level" in
        INFO)    [[ "$VERBOSE" == "true" ]] && echo -e "${BLUE}[INFO]${NC} $message" ;;
        SUCCESS) echo -e "${GREEN}[✓]${NC} $message" ;;
        WARN)    echo -e "${YELLOW}[!]${NC} $message" ;;
        ERROR)   echo -e "${RED}[✗]${NC} $message" ;;
        STEP)    echo -e "\n${BOLD}${CYAN}► $message${NC}" ;;
        DEBUG)   [[ "$VERBOSE" == "true" ]] && echo -e "${DIM}[DEBUG] $message${NC}" ;;
    esac
}

log_info()    { log "INFO" "$@"; }
log_success() { log "SUCCESS" "$@"; }
log_warn()    { log "WARN" "$@"; }
log_error()   { log "ERROR" "$@"; ERRORS+=("$*"); }
log_step()    { log "STEP" "$@"; }
log_debug()   { log "DEBUG" "$@"; }

# ============================================================
# Banner
# ============================================================
print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   █████╗ ██╗   ██╗████████╗ ██████╗                          ║
║  ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗                         ║
║  ███████║██║   ██║   ██║   ██║   ██║                         ║
║  ██╔══██║██║   ██║   ██║   ██║   ██║                         ║
║  ██║  ██║╚██████╔╝   ██║   ╚██████╔╝                         ║
║  ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝                          ║
║                                                              ║
║  ██████╗  █████╗  ██████╗██╗  ██╗██╗   ██╗██████╗            ║
║  ██╔══██╗██╔══██╗██╔════╝██║ ██╔╝██║   ██║██╔══██╗           ║
║  ██████╔╝███████║██║     █████╔╝ ██║   ██║██████╔╝           ║
║  ██╔══██╗██╔══██║██║     ██╔═██╗ ██║   ██║██╔═══╝            ║
║  ██████╔╝██║  ██║╚██████╗██║  ██╗╚██████╔╝██║                ║
║  ╚═════╝ ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝ ╚═════╝╚═╝                ║
║                                                              ║
║  AUTO-BACKUP v1.0 · One-Click Server Backup                  ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# ============================================================
# Help
# ============================================================
print_help() {
    cat << EOF
${BOLD}AUTO-BACKUP v${VERSION}${NC} - One-Click Server Backup Solution

${BOLD}USAGE:${NC}
    auto-backup [OPTIONS]

${BOLD}BACKUP OPTIONS:${NC}
    (none)              Full backup (web + db + config)
    --web               Backup web directories only
    --db                Backup databases only
    --config            Backup configuration files only
    --incremental       Incremental backup (changed files only)
    --dry-run           Simulate backup without making changes

${BOLD}CLOUD OPTIONS:${NC}
    --upload s3         Upload to AWS S3
    --upload gdrive     Upload to Google Drive (via rclone)
    --upload both       Upload to both S3 and Google Drive

${BOLD}SCHEDULE OPTIONS:${NC}
    --schedule daily    Set daily backup (3:00 AM)
    --schedule weekly   Set weekly backup (Sunday 3:00 AM)
    --cron "EXPR"       Set custom cron expression
    --unschedule        Remove scheduled backup

${BOLD}RESTORE OPTIONS:${NC}
    --restore latest    Restore from latest backup
    --restore DATE      Restore from specific date (YYYY-MM-DD)
    --restore FILE      Restore from specific backup file
    --db-only           Restore database only (with --restore)
    --web-only          Restore web files only (with --restore)
    --list              List available backups

${BOLD}MANAGEMENT:${NC}
    --status            Show backup status and info
    --cleanup           Remove old backups per retention policy
    --test-notify       Test notification settings
    --setup             Run interactive setup wizard
    --upgrade           Upgrade to latest version

${BOLD}OTHER OPTIONS:${NC}
    -v, --verbose       Verbose output
    -q, --quiet         Quiet mode (errors only)
    -h, --help          Show this help
    --version           Show version

${BOLD}EXAMPLES:${NC}
    auto-backup                          # Full backup
    auto-backup --web --upload s3        # Backup web, upload to S3
    auto-backup --schedule daily         # Schedule daily backup
    auto-backup --restore latest         # Restore latest backup
    auto-backup --list                   # List all backups

${BOLD}CONFIGURATION:${NC}
    Config file: ${CONFIG_FILE}
    Backup dir:  ${BACKUP_DIR}
    Log file:    ${LOG_FILE}

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utility Functions
# ============================================================
human_size() {
    local bytes=$1
    if [[ $bytes -lt 1024 ]]; then
        echo "${bytes}B"
    elif [[ $bytes -lt 1048576 ]]; then
        echo "$(( bytes / 1024 ))KB"
    elif [[ $bytes -lt 1073741824 ]]; then
        printf "%.1fMB" "$(echo "scale=1; $bytes / 1048576" | bc)"
    else
        printf "%.2fGB" "$(echo "scale=2; $bytes / 1073741824" | bc)"
    fi
}

get_dir_size() {
    local dir="$1"
    if [[ -d "$dir" ]]; then
        du -sb "$dir" 2>/dev/null | cut -f1
    else
        echo "0"
    fi
}

elapsed_time() {
    local start=$1
    local end=$2
    local diff=$((end - start))
    local mins=$((diff / 60))
    local secs=$((diff % 60))
    if [[ $mins -gt 0 ]]; then
        echo "${mins}분 ${secs}초"
    else
        echo "${secs}초"
    fi
}

check_command() {
    command -v "$1" &>/dev/null
}

require_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This operation requires root privileges. Please run with sudo."
        exit 1
    fi
}

cleanup_temp() {
    [[ -d "$TEMP_DIR" ]] && rm -rf "$TEMP_DIR"
}

trap cleanup_temp EXIT

# ============================================================
# Environment Detection
# ============================================================
detect_os() {
    if [[ -f /etc/os-release ]]; then
        # shellcheck source=/dev/null
        source /etc/os-release
        echo "$ID"
    elif [[ -f /etc/redhat-release ]]; then
        echo "rhel"
    elif [[ -f /etc/debian_version ]]; then
        echo "debian"
    else
        echo "unknown"
    fi
}

detect_webserver() {
    if systemctl is-active --quiet nginx 2>/dev/null; then
        echo "nginx"
    elif systemctl is-active --quiet apache2 2>/dev/null; then
        echo "apache"
    elif systemctl is-active --quiet httpd 2>/dev/null; then
        echo "apache"
    elif check_command nginx; then
        echo "nginx"
    elif check_command apache2 || check_command httpd; then
        echo "apache"
    else
        echo "none"
    fi
}

detect_database() {
    local dbs=()

    if systemctl is-active --quiet mysql 2>/dev/null || \
       systemctl is-active --quiet mysqld 2>/dev/null || \
       systemctl is-active --quiet mariadb 2>/dev/null; then
        dbs+=("mysql")
    fi

    if systemctl is-active --quiet postgresql 2>/dev/null; then
        dbs+=("postgresql")
    fi

    if systemctl is-active --quiet mongod 2>/dev/null; then
        dbs+=("mongodb")
    fi

    if [[ ${#dbs[@]} -eq 0 ]]; then
        echo "none"
    else
        echo "${dbs[*]}"
    fi
}

detect_web_dirs() {
    local found_dirs=()
    local check_dirs=("/var/www" "/usr/share/nginx/html" "/var/www/html" "/home/*/public_html")

    for pattern in "${check_dirs[@]}"; do
        for dir in $pattern; do
            if [[ -d "$dir" && "$(ls -A "$dir" 2>/dev/null)" ]]; then
                found_dirs+=("$dir")
            fi
        done
    done

    echo "${found_dirs[*]}"
}

detect_ssl_dirs() {
    local ssl_dirs=()

    [[ -d "/etc/letsencrypt/live" ]] && ssl_dirs+=("/etc/letsencrypt")
    [[ -d "/etc/ssl/private" ]] && ssl_dirs+=("/etc/ssl")
    [[ -d "/etc/pki/tls" ]] && ssl_dirs+=("/etc/pki/tls")

    echo "${ssl_dirs[*]}"
}

print_detection_result() {
    local os_name=$(detect_os)
    local webserver=$(detect_webserver)
    local databases=$(detect_database)
    local web_dirs=$(detect_web_dirs)
    local ssl_dirs=$(detect_ssl_dirs)

    DETECTED_WEBSERVER="$webserver"
    DETECTED_DB="$databases"

    echo -e "        ├── ${BOLD}OS:${NC} $os_name"
    echo -e "        ├── ${BOLD}웹서버:${NC} ${webserver:-없음}"

    if [[ -n "$web_dirs" ]]; then
        echo -e "        ├── ${BOLD}웹사이트:${NC}"
        for dir in $web_dirs; do
            local size=$(human_size $(get_dir_size "$dir"))
            echo -e "        │   ├── $dir (${size})"
        done
    fi

    echo -e "        ├── ${BOLD}데이터베이스:${NC} ${databases:-없음}"

    if [[ "$databases" == *"mysql"* ]]; then
        if check_command mysql; then
            local db_list=$(mysql -N -e "SHOW DATABASES" 2>/dev/null | grep -v -E "^(information_schema|performance_schema|mysql|sys)$" || true)
            if [[ -n "$db_list" ]]; then
                for db in $db_list; do
                    echo -e "        │   ├── $db"
                done
            fi
        fi
    fi

    if [[ -n "$ssl_dirs" ]]; then
        local ssl_count=0
        [[ -d "/etc/letsencrypt/live" ]] && ssl_count=$(ls -1 /etc/letsencrypt/live 2>/dev/null | wc -l)
        echo -e "        └── ${BOLD}SSL 인증서:${NC} ${ssl_count}개 도메인"
    else
        echo -e "        └── ${BOLD}SSL 인증서:${NC} 없음"
    fi
}

# ============================================================
# Backup Functions
# ============================================================
backup_web() {
    log_step "[2/5] 📂 웹 백업"

    local web_dirs=$(detect_web_dirs)
    if [[ -z "$web_dirs" ]]; then
        log_warn "백업할 웹 디렉토리가 없습니다"
        return 0
    fi

    local backup_file="${TEMP_DIR}/web-backup.tar"
    local exclude_args=""

    for pattern in $EXCLUDE_PATTERNS; do
        exclude_args="$exclude_args --exclude=$pattern"
    done

    local original_size=0
    for dir in $web_dirs; do
        original_size=$((original_size + $(get_dir_size "$dir")))
    done

    echo -e "        ├── 대상: $web_dirs"
    echo -e "        ├── 원본: $(human_size $original_size)"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        └── ${YELLOW}[DRY-RUN] 실제 백업하지 않음${NC}"
        return 0
    fi

    # Create tar archive
    local start_ts=$(date +%s)
    # shellcheck disable=SC2086
    tar -cf "$backup_file" $exclude_args $web_dirs 2>/dev/null || {
        log_error "웹 백업 실패"
        return 1
    }

    # Compress
    case "$COMPRESSION" in
        gzip)  gzip -"$COMPRESSION_LEVEL" "$backup_file" && backup_file="${backup_file}.gz" ;;
        bzip2) bzip2 -"$COMPRESSION_LEVEL" "$backup_file" && backup_file="${backup_file}.bz2" ;;
        xz)    xz -"$COMPRESSION_LEVEL" "$backup_file" && backup_file="${backup_file}.xz" ;;
    esac

    local compressed_size=$(stat -c%s "$backup_file" 2>/dev/null || stat -f%z "$backup_file")
    local savings=$((100 - (compressed_size * 100 / original_size)))
    local elapsed=$(($(date +%s) - start_ts))

    echo -e "        ├── 압축: $(human_size $compressed_size) (${savings}% 절약)"
    echo -e "        └── ${GREEN}✅ 완료${NC} (${elapsed}초)"

    return 0
}

backup_db() {
    log_step "[3/5] 🗄️ DB 백업"

    local databases=$(detect_database)
    if [[ "$databases" == "none" ]]; then
        log_warn "백업할 데이터베이스가 없습니다"
        return 0
    fi

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        └── ${YELLOW}[DRY-RUN] 실제 백업하지 않음${NC}"
        return 0
    fi

    local start_ts=$(date +%s)
    local backup_file="${TEMP_DIR}/db-backup.sql"

    # MySQL/MariaDB backup
    if [[ "$databases" == *"mysql"* ]]; then
        echo -e "        ├── MySQL/MariaDB 덤프 중..."

        local mysql_opts=""
        [[ -n "$MYSQL_PASSWORD" ]] && mysql_opts="-p${MYSQL_PASSWORD}"

        if [[ "$MYSQL_DATABASES" == "all" ]]; then
            mysqldump -u"$MYSQL_USER" $mysql_opts --all-databases --single-transaction \
                --routines --triggers > "$backup_file" 2>/dev/null || {
                log_error "MySQL 백업 실패"
                return 1
            }
        else
            mysqldump -u"$MYSQL_USER" $mysql_opts --databases $MYSQL_DATABASES \
                --single-transaction --routines --triggers > "$backup_file" 2>/dev/null || {
                log_error "MySQL 백업 실패"
                return 1
            }
        fi
    fi

    # PostgreSQL backup
    if [[ "$databases" == *"postgresql"* ]]; then
        echo -e "        ├── PostgreSQL 덤프 중..."
        local pg_file="${TEMP_DIR}/pg-backup.sql"

        if [[ "$PGSQL_DATABASES" == "all" ]]; then
            sudo -u "$PGSQL_USER" pg_dumpall > "$pg_file" 2>/dev/null || {
                log_error "PostgreSQL 백업 실패"
                return 1
            }
        else
            for db in $PGSQL_DATABASES; do
                sudo -u "$PGSQL_USER" pg_dump "$db" >> "$pg_file" 2>/dev/null
            done
        fi

        [[ -f "$pg_file" ]] && cat "$pg_file" >> "$backup_file"
    fi

    # MongoDB backup
    if [[ "$databases" == *"mongodb"* ]]; then
        echo -e "        ├── MongoDB 덤프 중..."
        mongodump --out="${TEMP_DIR}/mongodb" 2>/dev/null || {
            log_warn "MongoDB 백업 실패 (계속 진행)"
        }
    fi

    # Compress SQL dump
    local original_size=$(stat -c%s "$backup_file" 2>/dev/null || stat -f%z "$backup_file" 2>/dev/null || echo "0")
    gzip -"$COMPRESSION_LEVEL" "$backup_file"
    backup_file="${backup_file}.gz"

    local compressed_size=$(stat -c%s "$backup_file" 2>/dev/null || stat -f%z "$backup_file")
    local savings=0
    [[ $original_size -gt 0 ]] && savings=$((100 - (compressed_size * 100 / original_size)))
    local elapsed=$(($(date +%s) - start_ts))

    echo -e "        ├── 원본: $(human_size $original_size)"
    echo -e "        ├── 압축: $(human_size $compressed_size) (${savings}% 절약)"
    echo -e "        └── ${GREEN}✅ 완료${NC} (${elapsed}초)"

    return 0
}

backup_config() {
    log_step "[4/5] ⚙️ 설정 백업"

    local backup_file="${TEMP_DIR}/config-backup.tar"
    local config_list=()

    # Collect existing config directories
    for pattern in $CONFIG_DIRS; do
        for dir in $pattern; do
            [[ -e "$dir" ]] && config_list+=("$dir")
        done
    done

    if [[ ${#config_list[@]} -eq 0 ]]; then
        log_warn "백업할 설정 파일이 없습니다"
        return 0
    fi

    echo -e "        ├── 대상: ${config_list[*]}"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        └── ${YELLOW}[DRY-RUN] 실제 백업하지 않음${NC}"
        return 0
    fi

    local start_ts=$(date +%s)

    tar -cf "$backup_file" "${config_list[@]}" 2>/dev/null || {
        log_warn "일부 설정 파일 백업 실패 (계속 진행)"
    }

    gzip -"$COMPRESSION_LEVEL" "$backup_file"
    backup_file="${backup_file}.gz"

    local compressed_size=$(stat -c%s "$backup_file" 2>/dev/null || stat -f%z "$backup_file")
    local elapsed=$(($(date +%s) - start_ts))

    echo -e "        ├── 압축: $(human_size $compressed_size)"
    echo -e "        └── ${GREEN}✅ 완료${NC} (${elapsed}초)"

    return 0
}

create_final_archive() {
    log_step "[5/5] 📦 최종 패키징"

    local date_dir="${BACKUP_DIR}/${CURRENT_DATE}"
    mkdir -p "$date_dir"

    BACKUP_FILE="${date_dir}/full-backup-${CURRENT_DATETIME}.tar.gz"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        └── ${YELLOW}[DRY-RUN] 실제 패키징하지 않음${NC}"
        return 0
    fi

    echo -e "        ├── 통합 압축 중..."

    # Create final archive
    tar -czf "$BACKUP_FILE" -C "$TEMP_DIR" . 2>/dev/null || {
        log_error "최종 패키징 실패"
        return 1
    }

    # Encrypt if enabled
    if [[ "$ENCRYPTION_ENABLED" == "true" && -n "$ENCRYPTION_PASSWORD" ]]; then
        echo -e "        ├── 암호화 중..."
        openssl enc -aes-256-cbc -salt -pbkdf2 \
            -in "$BACKUP_FILE" -out "${BACKUP_FILE}.enc" \
            -pass pass:"$ENCRYPTION_PASSWORD"
        rm -f "$BACKUP_FILE"
        BACKUP_FILE="${BACKUP_FILE}.enc"
    fi

    BACKUP_SIZE=$(stat -c%s "$BACKUP_FILE" 2>/dev/null || stat -f%z "$BACKUP_FILE")

    echo -e "        └── ${GREEN}✅ 완료${NC}"

    return 0
}

# ============================================================
# Cloud Upload
# ============================================================
upload_to_s3() {
    if [[ -z "$S3_BUCKET" ]]; then
        log_error "S3 버킷이 설정되지 않았습니다"
        return 1
    fi

    if ! check_command aws; then
        log_error "AWS CLI가 설치되지 않았습니다"
        return 1
    fi

    log_info "S3 업로드 중: s3://${S3_BUCKET}/${S3_PATH}"

    local s3_path="s3://${S3_BUCKET}/${S3_PATH}$(basename "$BACKUP_FILE")"

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] aws s3 cp $BACKUP_FILE $s3_path"
        return 0
    fi

    aws s3 cp "$BACKUP_FILE" "$s3_path" --region "$S3_REGION" || {
        log_error "S3 업로드 실패"
        return 1
    }

    log_success "S3 업로드 완료: $s3_path"
    return 0
}

upload_to_gdrive() {
    if ! check_command rclone; then
        log_error "rclone이 설치되지 않았습니다"
        return 1
    fi

    log_info "Google Drive 업로드 중: ${GDRIVE_REMOTE}:${GDRIVE_PATH}"

    local gdrive_path="${GDRIVE_REMOTE}:${GDRIVE_PATH}$(basename "$BACKUP_FILE")"

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] rclone copy $BACKUP_FILE $gdrive_path"
        return 0
    fi

    rclone copy "$BACKUP_FILE" "${GDRIVE_REMOTE}:${GDRIVE_PATH}" || {
        log_error "Google Drive 업로드 실패"
        return 1
    }

    log_success "Google Drive 업로드 완료"
    return 0
}

do_upload() {
    local provider="$1"

    case "$provider" in
        s3)     upload_to_s3 ;;
        gdrive) upload_to_gdrive ;;
        both)   upload_to_s3; upload_to_gdrive ;;
        *)      log_error "알 수 없는 클라우드 제공자: $provider" ;;
    esac
}

# ============================================================
# Restore Functions
# ============================================================
list_backups() {
    echo -e "\n${BOLD}📦 사용 가능한 백업 목록${NC}\n"

    if [[ ! -d "$BACKUP_DIR" ]]; then
        log_warn "백업 디렉토리가 없습니다: $BACKUP_DIR"
        return 1
    fi

    local count=0
    for date_dir in $(ls -1r "$BACKUP_DIR" 2>/dev/null); do
        if [[ -d "${BACKUP_DIR}/${date_dir}" ]]; then
            echo -e "${CYAN}📅 ${date_dir}${NC}"
            for file in "${BACKUP_DIR}/${date_dir}"/*; do
                if [[ -f "$file" ]]; then
                    local size=$(human_size $(stat -c%s "$file" 2>/dev/null || stat -f%z "$file"))
                    echo -e "   └── $(basename "$file") (${size})"
                    ((count++))
                fi
            done
        fi
    done

    if [[ $count -eq 0 ]]; then
        log_warn "백업 파일이 없습니다"
    else
        echo -e "\n${DIM}총 ${count}개 백업 파일${NC}"
    fi
}

restore_backup() {
    local target="$1"
    local restore_opts="$2"

    require_root

    local backup_file=""

    if [[ "$target" == "latest" ]]; then
        # Find latest backup
        backup_file=$(find "$BACKUP_DIR" -name "*.tar.gz*" -type f | sort -r | head -1)
    elif [[ -f "$target" ]]; then
        backup_file="$target"
    elif [[ -d "${BACKUP_DIR}/${target}" ]]; then
        backup_file=$(find "${BACKUP_DIR}/${target}" -name "*.tar.gz*" -type f | sort -r | head -1)
    else
        log_error "백업을 찾을 수 없습니다: $target"
        return 1
    fi

    if [[ -z "$backup_file" || ! -f "$backup_file" ]]; then
        log_error "백업 파일을 찾을 수 없습니다"
        return 1
    fi

    log_step "백업 복원: $(basename "$backup_file")"

    # Decrypt if needed
    if [[ "$backup_file" == *.enc ]]; then
        if [[ -z "$ENCRYPTION_PASSWORD" ]]; then
            read -sp "암호화 비밀번호: " ENCRYPTION_PASSWORD
            echo
        fi

        local decrypted="${TEMP_DIR}/decrypted.tar.gz"
        openssl enc -aes-256-cbc -d -pbkdf2 \
            -in "$backup_file" -out "$decrypted" \
            -pass pass:"$ENCRYPTION_PASSWORD" || {
            log_error "복호화 실패"
            return 1
        }
        backup_file="$decrypted"
    fi

    # Create temp restore dir
    local restore_dir="${TEMP_DIR}/restore"
    mkdir -p "$restore_dir"

    # Extract archive
    tar -xzf "$backup_file" -C "$restore_dir" || {
        log_error "압축 해제 실패"
        return 1
    }

    # Restore based on options
    if [[ "$restore_opts" == *"db-only"* ]]; then
        log_info "데이터베이스만 복원..."
        if [[ -f "${restore_dir}/db-backup.sql.gz" ]]; then
            gunzip -c "${restore_dir}/db-backup.sql.gz" | mysql -u"$MYSQL_USER" || {
                log_error "DB 복원 실패"
                return 1
            }
            log_success "데이터베이스 복원 완료"
        else
            log_error "DB 백업 파일을 찾을 수 없습니다"
            return 1
        fi
    elif [[ "$restore_opts" == *"web-only"* ]]; then
        log_info "웹 파일만 복원..."
        if [[ -f "${restore_dir}/web-backup.tar.gz" ]]; then
            tar -xzf "${restore_dir}/web-backup.tar.gz" -C / || {
                log_error "웹 파일 복원 실패"
                return 1
            }
            log_success "웹 파일 복원 완료"
        else
            log_error "웹 백업 파일을 찾을 수 없습니다"
            return 1
        fi
    else
        # Full restore
        log_info "전체 복원 중..."

        if [[ -f "${restore_dir}/web-backup.tar.gz" ]]; then
            log_info "웹 파일 복원..."
            tar -xzf "${restore_dir}/web-backup.tar.gz" -C /
        fi

        if [[ -f "${restore_dir}/config-backup.tar.gz" ]]; then
            log_info "설정 파일 복원..."
            tar -xzf "${restore_dir}/config-backup.tar.gz" -C /
        fi

        if [[ -f "${restore_dir}/db-backup.sql.gz" ]]; then
            log_info "데이터베이스 복원..."
            gunzip -c "${restore_dir}/db-backup.sql.gz" | mysql -u"$MYSQL_USER"
        fi

        log_success "전체 복원 완료"
    fi

    # Restart services
    log_info "서비스 재시작 중..."
    systemctl reload nginx 2>/dev/null || systemctl reload apache2 2>/dev/null || systemctl reload httpd 2>/dev/null || true

    return 0
}

# ============================================================
# Cleanup (Retention Policy)
# ============================================================
cleanup_old_backups() {
    log_step "오래된 백업 정리"

    if [[ ! -d "$BACKUP_DIR" ]]; then
        log_warn "백업 디렉토리가 없습니다"
        return 0
    fi

    local deleted=0
    local saved_space=0

    # Get all backup dates sorted
    local dates=($(ls -1 "$BACKUP_DIR" 2>/dev/null | sort -r))
    local total=${#dates[@]}

    if [[ $total -le $KEEP_DAILY ]]; then
        log_info "정리할 백업이 없습니다 (${total}개 보관 중)"
        return 0
    fi

    # Keep daily
    local daily_kept=0
    local weekly_kept=0
    local monthly_kept=0

    for i in "${!dates[@]}"; do
        local date="${dates[$i]}"
        local date_path="${BACKUP_DIR}/${date}"
        local keep=false

        # Parse date
        local day_of_week=$(date -d "$date" +%u 2>/dev/null || echo "0")
        local day_of_month=$(date -d "$date" +%d 2>/dev/null || echo "00")

        # Daily (first N)
        if [[ $daily_kept -lt $KEEP_DAILY ]]; then
            keep=true
            ((daily_kept++))
        # Weekly (Sundays)
        elif [[ "$day_of_week" == "7" && $weekly_kept -lt $KEEP_WEEKLY ]]; then
            keep=true
            ((weekly_kept++))
        # Monthly (1st of month)
        elif [[ "$day_of_month" == "01" && $monthly_kept -lt $KEEP_MONTHLY ]]; then
            keep=true
            ((monthly_kept++))
        fi

        if [[ "$keep" == "false" && -d "$date_path" ]]; then
            local size=$(get_dir_size "$date_path")
            saved_space=$((saved_space + size))

            if [[ "$DRY_RUN" == "true" ]]; then
                log_info "[DRY-RUN] 삭제 예정: $date"
            else
                rm -rf "$date_path"
                log_info "삭제됨: $date"
            fi
            ((deleted++))
        fi
    done

    if [[ $deleted -gt 0 ]]; then
        log_success "${deleted}개 백업 정리 완료 ($(human_size $saved_space) 확보)"
    else
        log_info "정리할 백업이 없습니다"
    fi
}

# ============================================================
# Scheduling (Cron)
# ============================================================
setup_schedule() {
    local schedule="$1"
    local cron_expr=""

    case "$schedule" in
        daily)   cron_expr="0 3 * * *" ;;
        weekly)  cron_expr="0 3 * * 0" ;;
        *)       cron_expr="$schedule" ;;
    esac

    require_root

    local cron_file="/etc/cron.d/auto-backup"
    local script_path=$(realpath "$0")

    cat > "$cron_file" << EOF
# AUTO-BACKUP v${VERSION}
# Schedule: $schedule

SHELL=/bin/bash
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin

$cron_expr root $script_path --quiet 2>&1 | logger -t auto-backup
EOF

    chmod 644 "$cron_file"

    log_success "스케줄 설정 완료: $schedule"
    log_info "Cron: $cron_expr"
    log_info "파일: $cron_file"
}

remove_schedule() {
    require_root

    local cron_file="/etc/cron.d/auto-backup"

    if [[ -f "$cron_file" ]]; then
        rm -f "$cron_file"
        log_success "스케줄 제거 완료"
    else
        log_info "설정된 스케줄이 없습니다"
    fi
}

# ============================================================
# Notifications
# ============================================================
send_notification() {
    local status="$1"  # success or failure
    local message="$2"

    [[ "$NOTIFY_ENABLED" != "true" ]] && return 0
    [[ "$status" == "success" && "$NOTIFY_ON_SUCCESS" != "true" ]] && return 0
    [[ "$status" == "failure" && "$NOTIFY_ON_FAILURE" != "true" ]] && return 0

    local icon="✅"
    [[ "$status" == "failure" ]] && icon="❌"

    # Email
    if [[ "$EMAIL_ENABLED" == "true" && -n "$EMAIL_TO" ]]; then
        echo "$message" | mail -s "${icon} AUTO-BACKUP: ${status}" "$EMAIL_TO" 2>/dev/null || true
    fi

    # Slack
    if [[ "$SLACK_ENABLED" == "true" && -n "$SLACK_WEBHOOK" ]]; then
        local payload=$(cat << EOF
{
    "text": "${icon} *AUTO-BACKUP ${status}*\n\`\`\`${message}\`\`\`",
    "username": "AUTO-BACKUP",
    "icon_emoji": ":floppy_disk:"
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$SLACK_WEBHOOK" >/dev/null 2>&1 || true
    fi

    # Telegram
    if [[ "$TELEGRAM_ENABLED" == "true" && -n "$TELEGRAM_BOT_TOKEN" && -n "$TELEGRAM_CHAT_ID" ]]; then
        local text="${icon} *AUTO-BACKUP ${status}*%0A%0A\`${message}\`"
        curl -s "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
            -d "chat_id=${TELEGRAM_CHAT_ID}&text=${text}&parse_mode=Markdown" >/dev/null 2>&1 || true
    fi

    # Discord
    if [[ "$DISCORD_ENABLED" == "true" && -n "$DISCORD_WEBHOOK" ]]; then
        local payload=$(cat << EOF
{
    "content": "${icon} **AUTO-BACKUP ${status}**\n\`\`\`${message}\`\`\`"
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$DISCORD_WEBHOOK" >/dev/null 2>&1 || true
    fi
}

test_notifications() {
    log_step "알림 테스트"

    NOTIFY_ENABLED=true
    NOTIFY_ON_SUCCESS=true

    local test_msg="AUTO-BACKUP 알림 테스트\n서버: ${HOSTNAME}\n시간: $(date)"

    send_notification "success" "$test_msg"

    log_success "알림 테스트 전송 완료"
}

# ============================================================
# Status & Info
# ============================================================
show_status() {
    echo -e "\n${BOLD}📊 AUTO-BACKUP 상태${NC}\n"

    echo -e "${CYAN}설정${NC}"
    echo -e "  ├── 버전: v${VERSION}"
    echo -e "  ├── 설정 파일: ${CONFIG_FILE}"
    echo -e "  ├── 백업 디렉토리: ${BACKUP_DIR}"
    echo -e "  └── 로그 파일: ${LOG_FILE}"

    echo -e "\n${CYAN}백업 현황${NC}"
    if [[ -d "$BACKUP_DIR" ]]; then
        local total_backups=$(find "$BACKUP_DIR" -name "*.tar.gz*" -type f 2>/dev/null | wc -l)
        local total_size=$(du -sb "$BACKUP_DIR" 2>/dev/null | cut -f1)
        local latest=$(ls -1t "$BACKUP_DIR" 2>/dev/null | head -1)

        echo -e "  ├── 총 백업 수: ${total_backups}개"
        echo -e "  ├── 총 크기: $(human_size ${total_size:-0})"
        echo -e "  └── 최근 백업: ${latest:-없음}"
    else
        echo -e "  └── 백업 없음"
    fi

    echo -e "\n${CYAN}스케줄${NC}"
    if [[ -f "/etc/cron.d/auto-backup" ]]; then
        local cron_line=$(grep -v "^#" /etc/cron.d/auto-backup | grep -v "^$" | tail -1)
        echo -e "  └── ${cron_line}"
    else
        echo -e "  └── 설정된 스케줄 없음"
    fi

    echo -e "\n${CYAN}클라우드${NC}"
    echo -e "  ├── S3: ${S3_BUCKET:-미설정}"
    echo -e "  └── Google Drive: ${GDRIVE_REMOTE:-미설정}"

    echo
}

# ============================================================
# Setup Wizard
# ============================================================
run_setup_wizard() {
    print_banner

    echo -e "${BOLD}AUTO-BACKUP 설정 마법사${NC}\n"

    # Backup directory
    read -p "백업 디렉토리 [${BACKUP_DIR}]: " input
    BACKUP_DIR="${input:-$BACKUP_DIR}"

    # Retention policy
    read -p "일간 백업 보관 수 [${KEEP_DAILY}]: " input
    KEEP_DAILY="${input:-$KEEP_DAILY}"

    read -p "주간 백업 보관 수 [${KEEP_WEEKLY}]: " input
    KEEP_WEEKLY="${input:-$KEEP_WEEKLY}"

    read -p "월간 백업 보관 수 [${KEEP_MONTHLY}]: " input
    KEEP_MONTHLY="${input:-$KEEP_MONTHLY}"

    # Cloud
    echo -e "\n${BOLD}클라우드 업로드 설정${NC}"
    read -p "S3 버킷 (비워두면 건너뜀): " S3_BUCKET

    if [[ -n "$S3_BUCKET" ]]; then
        read -p "S3 리전 [${S3_REGION}]: " input
        S3_REGION="${input:-$S3_REGION}"
    fi

    # Notifications
    echo -e "\n${BOLD}알림 설정${NC}"
    read -p "Slack Webhook URL (비워두면 건너뜀): " SLACK_WEBHOOK
    [[ -n "$SLACK_WEBHOOK" ]] && SLACK_ENABLED=true

    read -p "알림 이메일 (비워두면 건너뜀): " EMAIL_TO
    [[ -n "$EMAIL_TO" ]] && EMAIL_ENABLED=true

    # Save configuration
    save_config

    log_success "설정이 저장되었습니다: ${CONFIG_FILE}"
}

save_config() {
    cat > "$CONFIG_FILE" << EOF
#!/bin/bash
# AUTO-BACKUP v${VERSION} Configuration
# Generated: $(date)

#===========================================
# 기본 설정
#===========================================
BACKUP_DIR="${BACKUP_DIR}"
LOG_FILE="${LOG_FILE}"

#===========================================
# 백업 대상
#===========================================
BACKUP_WEB=${BACKUP_WEB}
BACKUP_DB=${BACKUP_DB}
BACKUP_CONFIG=${BACKUP_CONFIG}
BACKUP_SSL=${BACKUP_SSL}

WEB_DIRS="${WEB_DIRS}"
CONFIG_DIRS="${CONFIG_DIRS}"
EXCLUDE_PATTERNS="${EXCLUDE_PATTERNS}"

#===========================================
# 데이터베이스
#===========================================
MYSQL_USER="${MYSQL_USER}"
MYSQL_PASSWORD="${MYSQL_PASSWORD}"
MYSQL_DATABASES="${MYSQL_DATABASES}"

PGSQL_USER="${PGSQL_USER}"
PGSQL_DATABASES="${PGSQL_DATABASES}"

#===========================================
# 보관 정책
#===========================================
KEEP_DAILY=${KEEP_DAILY}
KEEP_WEEKLY=${KEEP_WEEKLY}
KEEP_MONTHLY=${KEEP_MONTHLY}

#===========================================
# 클라우드 업로드
#===========================================
CLOUD_ENABLED=${CLOUD_ENABLED}
CLOUD_PROVIDER="${CLOUD_PROVIDER}"

S3_BUCKET="${S3_BUCKET}"
S3_REGION="${S3_REGION}"
S3_PATH="${S3_PATH}"

GDRIVE_REMOTE="${GDRIVE_REMOTE}"
GDRIVE_PATH="${GDRIVE_PATH}"

#===========================================
# 알림
#===========================================
NOTIFY_ENABLED=${NOTIFY_ENABLED}
NOTIFY_ON_SUCCESS=${NOTIFY_ON_SUCCESS}
NOTIFY_ON_FAILURE=${NOTIFY_ON_FAILURE}

EMAIL_ENABLED=${EMAIL_ENABLED}
EMAIL_TO="${EMAIL_TO}"

SLACK_ENABLED=${SLACK_ENABLED}
SLACK_WEBHOOK="${SLACK_WEBHOOK}"

TELEGRAM_ENABLED=${TELEGRAM_ENABLED}
TELEGRAM_BOT_TOKEN="${TELEGRAM_BOT_TOKEN}"
TELEGRAM_CHAT_ID="${TELEGRAM_CHAT_ID}"

DISCORD_ENABLED=${DISCORD_ENABLED}
DISCORD_WEBHOOK="${DISCORD_WEBHOOK}"

#===========================================
# 고급 설정
#===========================================
COMPRESSION="${COMPRESSION}"
COMPRESSION_LEVEL=${COMPRESSION_LEVEL}
ENCRYPTION_ENABLED=${ENCRYPTION_ENABLED}
ENCRYPTION_PASSWORD="${ENCRYPTION_PASSWORD}"
PARALLEL_JOBS=${PARALLEL_JOBS}
EOF

    chmod 600 "$CONFIG_FILE"
}

# ============================================================
# Print Summary
# ============================================================
print_summary() {
    local status="$1"
    local elapsed=$(elapsed_time "$START_TIME" "$END_TIME")

    echo -e "\n${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    if [[ "$status" == "success" ]]; then
    echo "║  ✅ 백업 완료!                                               ║"
    else
    echo "║  ❌ 백업 실패!                                               ║"
    fi
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    printf "║  📍 서버: %-49s║\n" "$HOSTNAME"
    printf "║  📁 위치: %-49s║\n" "${BACKUP_DIR}/${CURRENT_DATE}/"
    if [[ -n "$BACKUP_FILE" ]]; then
    printf "║  📦 파일: %-49s║\n" "$(basename "$BACKUP_FILE")"
    printf "║  📊 크기: %-49s║\n" "$(human_size $BACKUP_SIZE)"
    fi
    printf "║  ⏱️  소요: %-48s║\n" "$elapsed"
    echo "║                                                              ║"
    if [[ ${#ERRORS[@]} -gt 0 ]]; then
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  ⚠️  경고/오류:                                              ║"
    for err in "${ERRORS[@]}"; do
    printf "║  • %-56s║\n" "${err:0:56}"
    done
    fi
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  🔐 복원 명령어:                                             ║"
    printf "║  auto-backup --restore %-36s║\n" "$CURRENT_DATE"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  🤟 WIA AUTO-BACKUP - Free & Open Source                     ║"
    echo "║  弘益人間 · Benefit All Humanity                              ║"
    echo "║                                                              ║"
    echo "║  📖 Docs: https://wiastandards.com/auto-backup               ║"
    echo "║  ⭐ Star: https://github.com/WIA-Official/wia-standards      ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"
}

# ============================================================
# Main Backup Process
# ============================================================
do_backup() {
    local backup_web="$1"
    local backup_db="$2"
    local backup_config="$3"

    START_TIME=$(date +%s)

    print_banner

    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  🔄 AUTO-BACKUP v${VERSION}                                         ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📍 서버: %-49s║${NC}\n" "$HOSTNAME"
    printf "${BOLD}║  📅 시작: %-49s║${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╝${NC}"

    # Create temp directory
    mkdir -p "$TEMP_DIR"
    mkdir -p "$BACKUP_DIR"
    mkdir -p "$(dirname "$LOG_FILE")"
    touch "$LOG_FILE"

    # Step 1: Detection
    log_step "[1/5] 🔍 환경 감지"
    print_detection_result

    # Step 2-4: Backups
    local success=true

    if [[ "$backup_web" == "true" ]]; then
        backup_web || success=false
    fi

    if [[ "$backup_db" == "true" ]]; then
        backup_db || success=false
    fi

    if [[ "$backup_config" == "true" ]]; then
        backup_config || success=false
    fi

    # Step 5: Package
    if [[ "$success" == "true" ]]; then
        create_final_archive || success=false
    fi

    END_TIME=$(date +%s)

    # Summary
    if [[ "$success" == "true" ]]; then
        print_summary "success"
        send_notification "success" "백업 완료\n서버: ${HOSTNAME}\n크기: $(human_size $BACKUP_SIZE)\n소요: $(elapsed_time $START_TIME $END_TIME)"
    else
        print_summary "failure"
        send_notification "failure" "백업 실패\n서버: ${HOSTNAME}\n오류: ${ERRORS[*]}"
        return 1
    fi

    # Cleanup old backups
    cleanup_old_backups

    return 0
}

# ============================================================
# Main Entry Point
# ============================================================
main() {
    # Load configuration
    load_config

    # Parse arguments
    local action="backup"
    local backup_web="$BACKUP_WEB"
    local backup_db="$BACKUP_DB"
    local backup_config="$BACKUP_CONFIG"
    local upload_provider=""
    local restore_target=""
    local restore_opts=""
    local schedule_type=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --web)
                backup_web=true
                backup_db=false
                backup_config=false
                ;;
            --db)
                backup_web=false
                backup_db=true
                backup_config=false
                ;;
            --config)
                backup_web=false
                backup_db=false
                backup_config=true
                ;;
            --incremental)
                # TODO: Implement incremental backup
                log_warn "증분 백업은 아직 구현되지 않았습니다"
                ;;
            --upload)
                shift
                upload_provider="$1"
                ;;
            --schedule)
                action="schedule"
                shift
                schedule_type="$1"
                ;;
            --cron)
                action="schedule"
                shift
                schedule_type="$1"
                ;;
            --unschedule)
                action="unschedule"
                ;;
            --restore)
                action="restore"
                shift
                restore_target="$1"
                ;;
            --db-only)
                restore_opts="db-only"
                ;;
            --web-only)
                restore_opts="web-only"
                ;;
            --list)
                action="list"
                ;;
            --status)
                action="status"
                ;;
            --cleanup)
                action="cleanup"
                ;;
            --test-notify)
                action="test-notify"
                ;;
            --setup)
                action="setup"
                ;;
            --upgrade)
                action="upgrade"
                ;;
            --dry-run)
                DRY_RUN=true
                ;;
            -v|--verbose)
                VERBOSE=true
                ;;
            -q|--quiet)
                VERBOSE=false
                exec 1>/dev/null
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            --version)
                echo "AUTO-BACKUP v${VERSION}"
                exit 0
                ;;
            *)
                log_error "알 수 없는 옵션: $1"
                echo "도움말: auto-backup --help"
                exit 1
                ;;
        esac
        shift
    done

    # Execute action
    case "$action" in
        backup)
            do_backup "$backup_web" "$backup_db" "$backup_config"
            [[ -n "$upload_provider" ]] && do_upload "$upload_provider"
            ;;
        restore)
            restore_backup "$restore_target" "$restore_opts"
            ;;
        list)
            list_backups
            ;;
        schedule)
            setup_schedule "$schedule_type"
            ;;
        unschedule)
            remove_schedule
            ;;
        status)
            show_status
            ;;
        cleanup)
            cleanup_old_backups
            ;;
        test-notify)
            test_notifications
            ;;
        setup)
            run_setup_wizard
            ;;
        upgrade)
            log_info "업그레이드 기능은 추후 지원 예정입니다"
            ;;
    esac
}

# Run main
main "$@"
