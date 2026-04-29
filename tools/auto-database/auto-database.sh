#!/bin/bash
#
# AUTO-DATABASE v1.0
# One-Click Database Backup & Restore
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: auto-database [COMMAND] [OPTIONS]
#

set -e

# ============================================================
# Version & Constants
# ============================================================
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/auto-database.conf"
BACKUP_DIR="/var/backups/auto-database"
LOG_FILE="${SCRIPT_DIR}/logs/auto-database.log"
DATA_DIR="${SCRIPT_DIR}/data"
TEMP_DIR="/tmp/auto-database-$$"

# ============================================================
# Colors
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
HOSTNAME=$(hostname -f 2>/dev/null || hostname)
CURRENT_DATE=$(date +%Y-%m-%d)
CURRENT_DATETIME=$(date +%Y%m%d-%H%M%S)

# MySQL/MariaDB
MYSQL_ENABLED=true
MYSQL_HOST="localhost"
MYSQL_PORT=3306
MYSQL_USER="root"
MYSQL_PASSWORD=""
MYSQL_DATABASES="all"
MYSQL_EXCLUDE="information_schema performance_schema mysql sys"
MYSQL_OPTIONS="--single-transaction --quick --lock-tables=false --routines --triggers"

# PostgreSQL
PGSQL_ENABLED=false
PGSQL_HOST="localhost"
PGSQL_PORT=5432
PGSQL_USER="postgres"
PGSQL_PASSWORD=""
PGSQL_DATABASES="all"

# MongoDB
MONGODB_ENABLED=false
MONGODB_HOST="localhost"
MONGODB_PORT=27017
MONGODB_USER=""
MONGODB_PASSWORD=""
MONGODB_AUTH_DB="admin"
MONGODB_DATABASES="all"

# Redis
REDIS_ENABLED=false
REDIS_HOST="localhost"
REDIS_PORT=6379
REDIS_PASSWORD=""

# Backup settings
COMPRESSION="gzip"
COMPRESSION_LEVEL=6
ENCRYPTION_ENABLED=false
ENCRYPTION_PASSWORD=""

# Retention
KEEP_HOURLY=24
KEEP_DAILY=7
KEEP_WEEKLY=4
KEEP_MONTHLY=3

# Cloud
CLOUD_ENABLED=false
S3_BUCKET=""
S3_PATH="database-backups/"

# Notifications
NOTIFY_ON_SUCCESS=false
NOTIFY_ON_FAILURE=true
SLACK_WEBHOOK=""
EMAIL_TO=""

# Runtime
VERBOSE=false
DRY_RUN=false
SPECIFIC_DB=""
SPECIFIC_TYPE=""
START_TIME=""
TOTAL_ORIGINAL=0
TOTAL_COMPRESSED=0
ERRORS=()

# ============================================================
# Load Configuration
# ============================================================
load_config() {
    [[ -f "$CONFIG_FILE" ]] && source "$CONFIG_FILE"
    mkdir -p "$BACKUP_DIR" "$DATA_DIR" "$(dirname "$LOG_FILE")" "$TEMP_DIR"
}

cleanup_temp() {
    [[ -d "$TEMP_DIR" ]] && rm -rf "$TEMP_DIR"
}
trap cleanup_temp EXIT

# ============================================================
# Logging
# ============================================================
log() {
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    echo "[$timestamp] $*" >> "$LOG_FILE" 2>/dev/null || true
}

log_info()    { [[ "$VERBOSE" == "true" ]] && echo -e "${BLUE}[INFO]${NC} $1"; log "INFO: $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; log "SUCCESS: $1"; }
log_warn()    { echo -e "${YELLOW}[!]${NC} $1"; log "WARN: $1"; }
log_error()   { echo -e "${RED}[✗]${NC} $1"; log "ERROR: $1"; ERRORS+=("$1"); }

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
║  ██████╗  █████╗ ████████╗ █████╗ ██████╗  █████╗ ███████╗   ║
║  ██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔══██╗██╔══██╗██╔════╝   ║
║  ██║  ██║███████║   ██║   ███████║██████╔╝███████║███████╗   ║
║  ██║  ██║██╔══██║   ██║   ██╔══██║██╔══██╗██╔══██║╚════██║   ║
║  ██████╔╝██║  ██║   ██║   ██║  ██║██████╔╝██║  ██║███████║   ║
║  ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚══════╝   ║
║                                                              ║
║  AUTO-DATABASE v1.0 · Database Backup & Restore              ║
║  弘益人間 (홍익인간) · Benefit All Humanity                  ║
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
${BOLD}AUTO-DATABASE v${VERSION}${NC} - Database Backup & Restore

${BOLD}USAGE:${NC}
    auto-database [COMMAND] [OPTIONS]

${BOLD}COMMANDS:${NC}
    backup              Backup databases
    restore BACKUP      Restore from backup
    list                List available backups
    status              Show database status
    size                Show database sizes
    optimize            Optimize databases
    clone SRC DST       Clone database
    schedule TIME       Schedule automatic backups
    unschedule          Remove scheduled backups

${BOLD}BACKUP OPTIONS:${NC}
    --db NAME           Backup specific database only
    --mysql             Backup MySQL/MariaDB only
    --postgresql        Backup PostgreSQL only
    --mongodb           Backup MongoDB only
    --redis             Backup Redis only
    --compress          Compress backup (default)
    --no-compress       Don't compress
    --encrypt           Encrypt backup
    --upload s3         Upload to S3

${BOLD}RESTORE OPTIONS:${NC}
    restore latest      Restore latest backup
    restore DATE        Restore from date (YYYY-MM-DD)
    --db NAME           Restore specific database
    --target NAME       Restore to different database

${BOLD}OTHER OPTIONS:${NC}
    --detect            Detect installed databases
    --setup             Run setup wizard
    --dry-run           Simulate without changes
    -v, --verbose       Verbose output
    -h, --help          Show this help
    --version           Show version

${BOLD}EXAMPLES:${NC}
    auto-database backup
    auto-database backup --db myapp --encrypt
    auto-database list
    auto-database restore latest
    auto-database restore 2024-12-24 --db myapp
    auto-database schedule daily 03:00
    auto-database clone myapp myapp_dev

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utility Functions
# ============================================================
check_command() {
    command -v "$1" &>/dev/null
}

human_size() {
    local bytes=$1
    if [[ $bytes -lt 1024 ]]; then
        echo "${bytes}B"
    elif [[ $bytes -lt 1048576 ]]; then
        echo "$(( bytes / 1024 ))KB"
    elif [[ $bytes -lt 1073741824 ]]; then
        printf "%.1fMB" "$(echo "scale=1; $bytes / 1048576" | bc 2>/dev/null || echo "0")"
    else
        printf "%.2fGB" "$(echo "scale=2; $bytes / 1073741824" | bc 2>/dev/null || echo "0")"
    fi
}

elapsed_time() {
    local start=$1
    local end=$(date +%s)
    local diff=$((end - start))
    echo "${diff}초"
}

# ============================================================
# Database Detection
# ============================================================
detect_mysql() {
    if ! check_command mysql; then
        return 1
    fi

    local version=$(mysql --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
    if [[ -z "$version" ]]; then
        return 1
    fi

    # Check if service is running
    if ! systemctl is-active --quiet mysqld 2>/dev/null && \
       ! systemctl is-active --quiet mariadb 2>/dev/null && \
       ! systemctl is-active --quiet mysql 2>/dev/null; then
        return 1
    fi

    echo "$version"
    return 0
}

detect_postgresql() {
    if ! check_command psql; then
        return 1
    fi

    if ! systemctl is-active --quiet postgresql 2>/dev/null; then
        return 1
    fi

    local version=$(psql --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
    echo "$version"
    return 0
}

detect_mongodb() {
    if ! check_command mongod; then
        return 1
    fi

    if ! systemctl is-active --quiet mongod 2>/dev/null; then
        return 1
    fi

    local version=$(mongod --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
    echo "$version"
    return 0
}

detect_redis() {
    if ! check_command redis-cli; then
        return 1
    fi

    if ! systemctl is-active --quiet redis 2>/dev/null && \
       ! systemctl is-active --quiet redis-server 2>/dev/null; then
        return 1
    fi

    local version=$(redis-cli INFO 2>/dev/null | grep redis_version | cut -d: -f2 | tr -d '\r')
    echo "$version"
    return 0
}

run_detection() {
    echo -e "\n${BOLD}🔍 데이터베이스 감지${NC}\n"

    local found=false

    # MySQL/MariaDB
    local mysql_ver=$(detect_mysql)
    if [[ -n "$mysql_ver" ]]; then
        found=true
        echo -e "${GREEN}✓${NC} MySQL/MariaDB ${mysql_ver}"

        local dbs=$(mysql -N -e "SHOW DATABASES" 2>/dev/null | grep -v -E "^(information_schema|performance_schema|mysql|sys)$")
        local db_count=$(echo "$dbs" | wc -l)
        echo -e "  └── ${db_count}개 데이터베이스"

        for db in $dbs; do
            local size=$(mysql -N -e "SELECT SUM(data_length + index_length) FROM information_schema.tables WHERE table_schema='$db'" 2>/dev/null)
            echo -e "      ├── $db ($(human_size ${size:-0}))"
        done
    else
        echo -e "${DIM}○${NC} MySQL/MariaDB - 감지되지 않음"
    fi

    # PostgreSQL
    local pg_ver=$(detect_postgresql)
    if [[ -n "$pg_ver" ]]; then
        found=true
        echo -e "${GREEN}✓${NC} PostgreSQL ${pg_ver}"
    else
        echo -e "${DIM}○${NC} PostgreSQL - 감지되지 않음"
    fi

    # MongoDB
    local mongo_ver=$(detect_mongodb)
    if [[ -n "$mongo_ver" ]]; then
        found=true
        echo -e "${GREEN}✓${NC} MongoDB ${mongo_ver}"
    else
        echo -e "${DIM}○${NC} MongoDB - 감지되지 않음"
    fi

    # Redis
    local redis_ver=$(detect_redis)
    if [[ -n "$redis_ver" ]]; then
        found=true
        echo -e "${GREEN}✓${NC} Redis ${redis_ver}"
    else
        echo -e "${DIM}○${NC} Redis - 감지되지 않음"
    fi

    echo ""

    if [[ "$found" == "false" ]]; then
        log_warn "감지된 데이터베이스가 없습니다"
    fi
}

# ============================================================
# MySQL Backup/Restore
# ============================================================
get_mysql_databases() {
    if [[ "$MYSQL_DATABASES" == "all" ]]; then
        mysql -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
            ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
            -N -e "SHOW DATABASES" 2>/dev/null | \
            grep -v -E "^(information_schema|performance_schema|mysql|sys)$"
    else
        echo "$MYSQL_DATABASES" | tr ' ' '\n'
    fi
}

backup_mysql_db() {
    local db="$1"
    local output_file="${TEMP_DIR}/mysql-${db}-${CURRENT_DATETIME}.sql"

    log_info "MySQL 백업: $db"

    local size_before=$(mysql -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
        ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
        -N -e "SELECT SUM(data_length + index_length) FROM information_schema.tables WHERE table_schema='$db'" 2>/dev/null)

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        ├── ${db}... ${YELLOW}[DRY-RUN]${NC}"
        return 0
    fi

    mysqldump -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
        ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
        $MYSQL_OPTIONS "$db" > "$output_file" 2>/dev/null

    if [[ $? -ne 0 ]]; then
        log_error "MySQL 백업 실패: $db"
        return 1
    fi

    local size_after=$(stat -c%s "$output_file" 2>/dev/null || stat -f%z "$output_file")
    TOTAL_ORIGINAL=$((TOTAL_ORIGINAL + ${size_before:-0}))

    # Compress
    if [[ "$COMPRESSION" != "none" ]]; then
        case "$COMPRESSION" in
            gzip)  gzip -"$COMPRESSION_LEVEL" "$output_file" && output_file="${output_file}.gz" ;;
            bzip2) bzip2 -"$COMPRESSION_LEVEL" "$output_file" && output_file="${output_file}.bz2" ;;
            xz)    xz -"$COMPRESSION_LEVEL" "$output_file" && output_file="${output_file}.xz" ;;
        esac
    fi

    size_after=$(stat -c%s "$output_file" 2>/dev/null || stat -f%z "$output_file")
    TOTAL_COMPRESSED=$((TOTAL_COMPRESSED + size_after))

    local savings=0
    [[ ${size_before:-0} -gt 0 ]] && savings=$((100 - (size_after * 100 / size_before)))

    echo -e "        ├── ${db}... 완료 ($(human_size ${size_before:-0}) → $(human_size $size_after), ${savings}% 압축)"

    # Encrypt if enabled
    if [[ "$ENCRYPTION_ENABLED" == "true" && -n "$ENCRYPTION_PASSWORD" ]]; then
        openssl enc -aes-256-cbc -salt -pbkdf2 \
            -in "$output_file" -out "${output_file}.enc" \
            -pass pass:"$ENCRYPTION_PASSWORD"
        rm -f "$output_file"
        output_file="${output_file}.enc"
    fi

    return 0
}

backup_mysql() {
    if [[ "$MYSQL_ENABLED" != "true" ]]; then
        return 0
    fi

    local mysql_ver=$(detect_mysql)
    if [[ -z "$mysql_ver" ]]; then
        log_warn "MySQL이 실행 중이지 않습니다"
        return 0
    fi

    echo -e "        ├── ${BOLD}MySQL ${mysql_ver}${NC}"

    local databases
    if [[ -n "$SPECIFIC_DB" ]]; then
        databases="$SPECIFIC_DB"
    else
        databases=$(get_mysql_databases)
    fi

    for db in $databases; do
        # Skip excluded databases
        if echo "$MYSQL_EXCLUDE" | grep -qw "$db"; then
            continue
        fi

        backup_mysql_db "$db"
    done

    return 0
}

restore_mysql_db() {
    local backup_file="$1"
    local target_db="${2:-}"

    log_info "MySQL 복원: $backup_file"

    # Decompress if needed
    local work_file="$backup_file"
    if [[ "$backup_file" == *.gz ]]; then
        gunzip -c "$backup_file" > "${TEMP_DIR}/restore.sql"
        work_file="${TEMP_DIR}/restore.sql"
    elif [[ "$backup_file" == *.bz2 ]]; then
        bunzip2 -c "$backup_file" > "${TEMP_DIR}/restore.sql"
        work_file="${TEMP_DIR}/restore.sql"
    fi

    # Decrypt if needed
    if [[ "$work_file" == *.enc ]]; then
        if [[ -z "$ENCRYPTION_PASSWORD" ]]; then
            read -sp "암호화 비밀번호: " ENCRYPTION_PASSWORD
            echo
        fi
        local decrypted="${TEMP_DIR}/decrypted.sql"
        openssl enc -aes-256-cbc -d -pbkdf2 \
            -in "$work_file" -out "$decrypted" \
            -pass pass:"$ENCRYPTION_PASSWORD"
        work_file="$decrypted"
    fi

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] Would restore: $work_file"
        return 0
    fi

    # Create target database if specified
    if [[ -n "$target_db" ]]; then
        mysql -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
            ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
            -e "CREATE DATABASE IF NOT EXISTS \`$target_db\`" 2>/dev/null

        mysql -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
            ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
            "$target_db" < "$work_file"
    else
        mysql -h "$MYSQL_HOST" -P "$MYSQL_PORT" -u "$MYSQL_USER" \
            ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} < "$work_file"
    fi

    if [[ $? -eq 0 ]]; then
        log_success "MySQL 복원 완료"
    else
        log_error "MySQL 복원 실패"
        return 1
    fi
}

# ============================================================
# PostgreSQL Backup/Restore
# ============================================================
backup_postgresql() {
    if [[ "$PGSQL_ENABLED" != "true" ]]; then
        return 0
    fi

    local pg_ver=$(detect_postgresql)
    if [[ -z "$pg_ver" ]]; then
        return 0
    fi

    echo -e "        ├── ${BOLD}PostgreSQL ${pg_ver}${NC}"

    local output_file="${TEMP_DIR}/postgresql-${CURRENT_DATETIME}.sql"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        │   └── ${YELLOW}[DRY-RUN]${NC}"
        return 0
    fi

    if [[ "$PGSQL_DATABASES" == "all" ]]; then
        sudo -u "$PGSQL_USER" pg_dumpall > "$output_file" 2>/dev/null
    else
        for db in $PGSQL_DATABASES; do
            sudo -u "$PGSQL_USER" pg_dump "$db" >> "$output_file" 2>/dev/null
        done
    fi

    if [[ "$COMPRESSION" != "none" ]]; then
        gzip -"$COMPRESSION_LEVEL" "$output_file"
    fi

    echo -e "        │   └── 완료"
    return 0
}

# ============================================================
# MongoDB Backup/Restore
# ============================================================
backup_mongodb() {
    if [[ "$MONGODB_ENABLED" != "true" ]]; then
        return 0
    fi

    local mongo_ver=$(detect_mongodb)
    if [[ -z "$mongo_ver" ]]; then
        return 0
    fi

    echo -e "        ├── ${BOLD}MongoDB ${mongo_ver}${NC}"

    local output_dir="${TEMP_DIR}/mongodb-${CURRENT_DATETIME}"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        │   └── ${YELLOW}[DRY-RUN]${NC}"
        return 0
    fi

    local auth_opts=""
    if [[ -n "$MONGODB_USER" && -n "$MONGODB_PASSWORD" ]]; then
        auth_opts="-u $MONGODB_USER -p $MONGODB_PASSWORD --authenticationDatabase $MONGODB_AUTH_DB"
    fi

    mongodump --host "$MONGODB_HOST" --port "$MONGODB_PORT" $auth_opts \
        --out "$output_dir" 2>/dev/null

    # Compress
    tar -czf "${output_dir}.tar.gz" -C "$TEMP_DIR" "mongodb-${CURRENT_DATETIME}"
    rm -rf "$output_dir"

    echo -e "        │   └── 완료"
    return 0
}

# ============================================================
# Redis Backup
# ============================================================
backup_redis() {
    if [[ "$REDIS_ENABLED" != "true" ]]; then
        return 0
    fi

    local redis_ver=$(detect_redis)
    if [[ -z "$redis_ver" ]]; then
        return 0
    fi

    echo -e "        ├── ${BOLD}Redis ${redis_ver}${NC}"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        │   └── ${YELLOW}[DRY-RUN]${NC}"
        return 0
    fi

    # Trigger BGSAVE
    local auth_opt=""
    [[ -n "$REDIS_PASSWORD" ]] && auth_opt="-a $REDIS_PASSWORD"

    redis-cli -h "$REDIS_HOST" -p "$REDIS_PORT" $auth_opt BGSAVE >/dev/null 2>&1

    # Wait for save to complete
    sleep 2

    # Find and copy RDB file
    local rdb_file=$(redis-cli -h "$REDIS_HOST" -p "$REDIS_PORT" $auth_opt CONFIG GET dir 2>/dev/null | tail -1)
    local rdb_name=$(redis-cli -h "$REDIS_HOST" -p "$REDIS_PORT" $auth_opt CONFIG GET dbfilename 2>/dev/null | tail -1)

    if [[ -f "${rdb_file}/${rdb_name}" ]]; then
        cp "${rdb_file}/${rdb_name}" "${TEMP_DIR}/redis-${CURRENT_DATETIME}.rdb"
        local size=$(stat -c%s "${TEMP_DIR}/redis-${CURRENT_DATETIME}.rdb" 2>/dev/null || echo "0")
        echo -e "        │   └── RDB dump 완료 ($(human_size $size))"
    else
        log_warn "Redis RDB 파일을 찾을 수 없습니다"
    fi

    return 0
}

# ============================================================
# Main Backup
# ============================================================
do_backup() {
    START_TIME=$(date +%s)

    print_banner

    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  🗄️ AUTO-DATABASE v${VERSION}                                       ║${NC}"
    echo -e "${BOLD}║  🤟 WIA AUTO-DATABASE · https://wiastandards.com/auto-database║${NC}"
    echo -e "${BOLD}║  弘益人間 · Benefit All Humanity                              ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📍 서버: %-49s║${NC}\n" "$HOSTNAME"
    printf "${BOLD}║  📅 시작: %-49s║${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║                                                              ║${NC}"

    # Step 1: Detection
    echo -e "${BOLD}  [1/3] 🔍 데이터베이스 감지${NC}"

    # Step 2: Backup
    echo -e "\n${BOLD}  [2/3] 📦 백업 진행${NC}"

    case "$SPECIFIC_TYPE" in
        mysql)      backup_mysql ;;
        postgresql) backup_postgresql ;;
        mongodb)    backup_mongodb ;;
        redis)      backup_redis ;;
        *)
            backup_mysql
            backup_postgresql
            backup_mongodb
            backup_redis
            ;;
    esac

    echo -e "        └── ${GREEN}✅ 모든 DB 백업 완료${NC}"

    # Step 3: Save
    echo -e "\n${BOLD}  [3/3] 📁 저장${NC}"

    local date_dir="${BACKUP_DIR}/${CURRENT_DATE}"
    mkdir -p "$date_dir"

    if [[ "$DRY_RUN" != "true" ]]; then
        # Move all backups to final location
        mv "$TEMP_DIR"/* "$date_dir/" 2>/dev/null || true

        echo -e "        ├── 위치: ${date_dir}/"
        echo -e "        └── ${GREEN}✅ 저장 완료${NC}"

        # Upload to cloud if enabled
        if [[ "$CLOUD_ENABLED" == "true" && -n "$S3_BUCKET" ]]; then
            echo -e "\n${BOLD}  [+] ☁️ 클라우드 업로드${NC}"
            for file in "$date_dir"/*; do
                aws s3 cp "$file" "s3://${S3_BUCKET}/${S3_PATH}$(basename "$file")" 2>/dev/null && \
                    echo -e "        └── S3 업로드 완료"
            done
        fi
    fi

    # Summary
    local elapsed=$(elapsed_time $START_TIME)
    local savings=0
    [[ $TOTAL_ORIGINAL -gt 0 ]] && savings=$((100 - (TOTAL_COMPRESSED * 100 / TOTAL_ORIGINAL)))

    echo -e "\n${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║  ✅ 백업 완료!                                               ║${NC}"
    echo -e "${BOLD}║                                                              ║${NC}"
    printf "║  📊 원본: %-10s → 압축: %-10s (%d%% 절약)            ║\n" \
        "$(human_size $TOTAL_ORIGINAL)" "$(human_size $TOTAL_COMPRESSED)" "$savings"
    printf "║  ⏱️  소요: %-48s║\n" "$elapsed"
    echo -e "${BOLD}║                                                              ║${NC}"
    printf "║  🔄 복원: auto-database restore %-26s║\n" "$CURRENT_DATE"
    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo

    log "Backup completed - Original: $TOTAL_ORIGINAL, Compressed: $TOTAL_COMPRESSED"
}

# ============================================================
# List Backups
# ============================================================
list_backups() {
    echo -e "\n${BOLD}📦 사용 가능한 백업${NC}\n"

    if [[ ! -d "$BACKUP_DIR" ]]; then
        log_warn "백업 디렉토리가 없습니다"
        return 1
    fi

    local total_size=0
    local backup_count=0

    for date_dir in $(ls -1r "$BACKUP_DIR" 2>/dev/null); do
        local dir_path="${BACKUP_DIR}/${date_dir}"
        [[ ! -d "$dir_path" ]] && continue

        echo -e "${CYAN}📅 ${date_dir}${NC}"

        for file in "$dir_path"/*; do
            [[ ! -f "$file" ]] && continue
            local size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file")
            total_size=$((total_size + size))
            ((backup_count++))

            echo -e "   └── $(basename "$file") ($(human_size $size))"
        done
    done

    if [[ $backup_count -eq 0 ]]; then
        log_warn "백업 파일이 없습니다"
    else
        echo -e "\n${DIM}총 ${backup_count}개 백업, $(human_size $total_size)${NC}"
    fi
}

# ============================================================
# Restore
# ============================================================
do_restore() {
    local target="$1"
    local target_db="$2"

    echo -e "\n${BOLD}🔄 백업 복원${NC}\n"

    local backup_path=""

    if [[ "$target" == "latest" ]]; then
        local latest_dir=$(ls -1r "$BACKUP_DIR" 2>/dev/null | head -1)
        if [[ -n "$latest_dir" ]]; then
            backup_path="${BACKUP_DIR}/${latest_dir}"
        fi
    elif [[ -d "${BACKUP_DIR}/${target}" ]]; then
        backup_path="${BACKUP_DIR}/${target}"
    else
        log_error "백업을 찾을 수 없습니다: $target"
        return 1
    fi

    if [[ -z "$backup_path" || ! -d "$backup_path" ]]; then
        log_error "백업 디렉토리를 찾을 수 없습니다"
        return 1
    fi

    echo -e "백업 위치: ${backup_path}\n"

    # Restore MySQL backups
    for file in "$backup_path"/mysql-*.sql*; do
        [[ ! -f "$file" ]] && continue

        if [[ -n "$SPECIFIC_DB" ]]; then
            if [[ "$file" != *"$SPECIFIC_DB"* ]]; then
                continue
            fi
        fi

        restore_mysql_db "$file" "$target_db"
    done

    log_success "복원 완료!"
}

# ============================================================
# Database Operations
# ============================================================
show_status() {
    echo -e "\n${BOLD}📊 데이터베이스 상태${NC}\n"

    # MySQL
    if systemctl is-active --quiet mysqld 2>/dev/null || \
       systemctl is-active --quiet mariadb 2>/dev/null || \
       systemctl is-active --quiet mysql 2>/dev/null; then
        local uptime=$(mysql -N -e "SHOW STATUS LIKE 'Uptime'" 2>/dev/null | awk '{print $2}')
        local connections=$(mysql -N -e "SHOW STATUS LIKE 'Threads_connected'" 2>/dev/null | awk '{print $2}')
        echo -e "${GREEN}✓${NC} MySQL: 실행 중"
        echo -e "  ├── Uptime: ${uptime:-N/A} 초"
        echo -e "  └── Connections: ${connections:-N/A}"
    else
        echo -e "${DIM}○${NC} MySQL: 중지됨"
    fi

    # PostgreSQL
    if systemctl is-active --quiet postgresql 2>/dev/null; then
        echo -e "${GREEN}✓${NC} PostgreSQL: 실행 중"
    else
        echo -e "${DIM}○${NC} PostgreSQL: 중지됨"
    fi

    # Redis
    if systemctl is-active --quiet redis 2>/dev/null || \
       systemctl is-active --quiet redis-server 2>/dev/null; then
        local redis_info=$(redis-cli INFO 2>/dev/null | grep used_memory_human | cut -d: -f2 | tr -d '\r')
        echo -e "${GREEN}✓${NC} Redis: 실행 중"
        echo -e "  └── Memory: ${redis_info:-N/A}"
    else
        echo -e "${DIM}○${NC} Redis: 중지됨"
    fi

    echo
}

show_sizes() {
    echo -e "\n${BOLD}📊 데이터베이스 크기${NC}\n"

    # MySQL databases
    if check_command mysql; then
        echo -e "${BOLD}MySQL/MariaDB${NC}"
        mysql -N -e "SELECT table_schema AS 'Database',
            ROUND(SUM(data_length + index_length) / 1024 / 1024, 2) AS 'Size (MB)'
            FROM information_schema.tables
            WHERE table_schema NOT IN ('information_schema', 'performance_schema', 'mysql', 'sys')
            GROUP BY table_schema
            ORDER BY SUM(data_length + index_length) DESC" 2>/dev/null | \
            while read db size; do
                printf "  %-30s %10s MB\n" "$db" "$size"
            done
    fi

    echo
}

optimize_databases() {
    echo -e "\n${BOLD}⚡ 데이터베이스 최적화${NC}\n"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "${YELLOW}[DRY-RUN] 실제 최적화하지 않음${NC}"
        return 0
    fi

    # MySQL OPTIMIZE
    if check_command mysqlcheck; then
        echo -e "MySQL 테이블 최적화 중..."
        mysqlcheck -u "$MYSQL_USER" ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
            --optimize --all-databases 2>/dev/null && \
            log_success "MySQL 최적화 완료"
    fi

    echo
}

clone_database() {
    local source="$1"
    local target="$2"

    if [[ -z "$source" || -z "$target" ]]; then
        log_error "사용법: auto-database clone SOURCE TARGET"
        return 1
    fi

    echo -e "\n${BOLD}📋 데이터베이스 복제${NC}"
    echo -e "   소스: $source → 타겟: $target\n"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "${YELLOW}[DRY-RUN] 실제 복제하지 않음${NC}"
        return 0
    fi

    # Create target database
    mysql -u "$MYSQL_USER" ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
        -e "CREATE DATABASE IF NOT EXISTS \`$target\`" 2>/dev/null

    # Dump and restore
    mysqldump -u "$MYSQL_USER" ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} \
        $MYSQL_OPTIONS "$source" 2>/dev/null | \
        mysql -u "$MYSQL_USER" ${MYSQL_PASSWORD:+-p"$MYSQL_PASSWORD"} "$target"

    if [[ $? -eq 0 ]]; then
        log_success "데이터베이스 복제 완료: $source → $target"
    else
        log_error "복제 실패"
        return 1
    fi
}

# ============================================================
# Scheduling
# ============================================================
setup_schedule() {
    local schedule="$1"
    local time="${2:-03:00}"

    local cron_expr=""
    case "$schedule" in
        hourly)     cron_expr="0 * * * *" ;;
        daily)
            local hour=$(echo "$time" | cut -d: -f1)
            local min=$(echo "$time" | cut -d: -f2)
            cron_expr="$min $hour * * *"
            ;;
        *)          cron_expr="$schedule" ;;
    esac

    local cron_file="/etc/cron.d/auto-database"
    local script_path=$(realpath "$0")

    cat > "$cron_file" << EOF
# AUTO-DATABASE v${VERSION}
# 弘益人間 (홍익인간)

SHELL=/bin/bash
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin

$cron_expr root $script_path backup --quiet 2>&1 | logger -t auto-database
EOF

    chmod 644 "$cron_file"

    log_success "스케줄 설정 완료: $schedule"
    echo "Cron: $cron_expr"
}

# ============================================================
# Main
# ============================================================
main() {
    load_config

    local command=""
    local restore_target=""
    local clone_source=""
    local clone_target=""
    local schedule_type=""
    local schedule_time=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            backup)     command="backup" ;;
            restore)
                command="restore"
                shift
                restore_target="${1:-latest}"
                ;;
            list)       command="list" ;;
            status)     command="status" ;;
            size)       command="size" ;;
            optimize)   command="optimize" ;;
            clone)
                command="clone"
                shift
                clone_source="$1"
                shift
                clone_target="$1"
                ;;
            schedule)
                command="schedule"
                shift
                schedule_type="$1"
                shift
                schedule_time="$1"
                ;;
            unschedule)
                rm -f /etc/cron.d/auto-database
                log_success "스케줄 제거 완료"
                exit 0
                ;;
            --detect)   command="detect" ;;
            --setup)    command="setup" ;;
            --db)
                shift
                SPECIFIC_DB="$1"
                ;;
            --target)
                shift
                # Used with restore
                ;;
            --mysql)      SPECIFIC_TYPE="mysql" ;;
            --postgresql) SPECIFIC_TYPE="postgresql" ;;
            --mongodb)    SPECIFIC_TYPE="mongodb" ;;
            --redis)      SPECIFIC_TYPE="redis" ;;
            --compress)   COMPRESSION="gzip" ;;
            --no-compress) COMPRESSION="none" ;;
            --encrypt)    ENCRYPTION_ENABLED=true ;;
            --upload)
                shift
                CLOUD_ENABLED=true
                ;;
            --dry-run)    DRY_RUN=true ;;
            -v|--verbose) VERBOSE=true ;;
            --quiet)      exec 1>/dev/null ;;
            -h|--help)    print_help; exit 0 ;;
            --version)    echo "AUTO-DATABASE v${VERSION}"; exit 0 ;;
        esac
        shift
    done

    # Default to backup if no command
    [[ -z "$command" ]] && command="backup"

    # Execute command
    case "$command" in
        backup)   do_backup ;;
        restore)  do_restore "$restore_target" ;;
        list)     list_backups ;;
        status)   show_status ;;
        size)     show_sizes ;;
        optimize) optimize_databases ;;
        clone)    clone_database "$clone_source" "$clone_target" ;;
        schedule) setup_schedule "$schedule_type" "$schedule_time" ;;
        detect)   run_detection ;;
        setup)
            echo "Setup wizard - 추후 구현"
            ;;
    esac
}

main "$@"
