#!/bin/bash
#
# AUTO-BACKUP v1.0 Installer
# One-Click Installation Script
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: curl -sSL https://wia.family/backup/install | sudo bash
#

set -e

# ============================================================
# Configuration
# ============================================================
VERSION="1.0.0"
INSTALL_DIR="/opt/auto-backup"
BIN_LINK="/usr/local/bin/auto-backup"
GITHUB_RAW="https://raw.githubusercontent.com/WIA-Official/auto-tools/main/auto-backup"

# ============================================================
# Colors
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ============================================================
# Functions
# ============================================================
log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[!]${NC} $1"; }
log_error()   { echo -e "${RED}[✗]${NC} $1"; exit 1; }

print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   AUTO-BACKUP v1.0 Installer                                 ║
║   One-Click Server Backup Solution                           ║
║                                                              ║
║   弘益人間 (홍익인간) · Benefit All Humanity                 ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This installer must be run as root. Please use: sudo $0"
    fi
}

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

detect_package_manager() {
    if command -v apt-get &>/dev/null; then
        echo "apt"
    elif command -v dnf &>/dev/null; then
        echo "dnf"
    elif command -v yum &>/dev/null; then
        echo "yum"
    elif command -v pacman &>/dev/null; then
        echo "pacman"
    else
        echo "unknown"
    fi
}

install_dependencies() {
    log_info "Installing dependencies..."

    local pkg_mgr=$(detect_package_manager)
    local packages="tar gzip curl bc"

    case "$pkg_mgr" in
        apt)
            apt-get update -qq
            apt-get install -y -qq $packages
            ;;
        dnf)
            dnf install -y -q $packages
            ;;
        yum)
            yum install -y -q $packages
            ;;
        pacman)
            pacman -Sy --noconfirm $packages
            ;;
        *)
            log_warn "Unknown package manager. Please install manually: $packages"
            ;;
    esac

    log_success "Dependencies installed"
}

create_directories() {
    log_info "Creating directories..."

    mkdir -p "$INSTALL_DIR"/{modules,templates,logs}
    mkdir -p /var/backups/auto-backup
    mkdir -p /var/log

    log_success "Directories created"
}

install_scripts() {
    log_info "Installing scripts..."

    # Get script directory (for local install) or download from GitHub
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    if [[ -f "${script_dir}/auto-backup.sh" ]]; then
        # Local installation
        cp "${script_dir}/auto-backup.sh" "$INSTALL_DIR/"
        [[ -f "${script_dir}/auto-backup.conf" ]] && cp "${script_dir}/auto-backup.conf" "$INSTALL_DIR/"
    else
        # Download from GitHub
        curl -sSL "${GITHUB_RAW}/auto-backup.sh" -o "$INSTALL_DIR/auto-backup.sh"
        curl -sSL "${GITHUB_RAW}/auto-backup.conf" -o "$INSTALL_DIR/auto-backup.conf" 2>/dev/null || true
    fi

    # Make executable
    chmod +x "$INSTALL_DIR/auto-backup.sh"

    # Create symlink
    ln -sf "$INSTALL_DIR/auto-backup.sh" "$BIN_LINK"

    log_success "Scripts installed"
}

create_default_config() {
    if [[ ! -f "$INSTALL_DIR/auto-backup.conf" ]]; then
        log_info "Creating default configuration..."

        cat > "$INSTALL_DIR/auto-backup.conf" << 'EOF'
#!/bin/bash
# AUTO-BACKUP v1.0 Configuration
# 弘益人間 (홍익인간) · Benefit All Humanity
#
# Edit this file to customize your backup settings
# Run 'auto-backup --setup' for interactive configuration

#===========================================
# 기본 설정
#===========================================
BACKUP_DIR="/var/backups/auto-backup"
LOG_FILE="/var/log/auto-backup.log"

#===========================================
# 백업 대상
#===========================================
BACKUP_WEB=true
BACKUP_DB=true
BACKUP_CONFIG=true
BACKUP_SSL=true

# 웹 디렉토리 (공백으로 구분)
WEB_DIRS="/var/www"

# 제외할 패턴
EXCLUDE_PATTERNS="node_modules .git .cache cache tmp logs *.log"

# 설정 디렉토리
CONFIG_DIRS="/etc/httpd /etc/nginx /etc/apache2 /etc/php* /etc/letsencrypt /etc/my.cnf* /etc/mysql"

#===========================================
# 데이터베이스
#===========================================
MYSQL_USER="root"
MYSQL_PASSWORD=""  # 비워두면 ~/.my.cnf 사용
MYSQL_DATABASES="all"  # all 또는 "db1 db2 db3"

PGSQL_USER="postgres"
PGSQL_DATABASES="all"

#===========================================
# 보관 정책
#===========================================
KEEP_DAILY=7      # 일간 백업 보관 수
KEEP_WEEKLY=4     # 주간 백업 보관 수
KEEP_MONTHLY=3    # 월간 백업 보관 수

#===========================================
# 클라우드 업로드
#===========================================
CLOUD_ENABLED=false
CLOUD_PROVIDER="none"  # s3, gdrive, both, none

# AWS S3
S3_BUCKET=""
S3_REGION="ap-northeast-2"
S3_PATH="backups/"

# Google Drive (rclone 설정 필요)
GDRIVE_REMOTE="gdrive"
GDRIVE_PATH="Server-Backups/"

#===========================================
# 알림
#===========================================
NOTIFY_ENABLED=false
NOTIFY_ON_SUCCESS=true
NOTIFY_ON_FAILURE=true

# 이메일
EMAIL_ENABLED=false
EMAIL_TO=""

# Slack
SLACK_ENABLED=false
SLACK_WEBHOOK=""

# Telegram
TELEGRAM_ENABLED=false
TELEGRAM_BOT_TOKEN=""
TELEGRAM_CHAT_ID=""

# Discord
DISCORD_ENABLED=false
DISCORD_WEBHOOK=""

#===========================================
# 고급 설정
#===========================================
COMPRESSION="gzip"  # gzip, bzip2, xz
COMPRESSION_LEVEL=6  # 1-9
ENCRYPTION_ENABLED=false
ENCRYPTION_PASSWORD=""
EOF

        chmod 600 "$INSTALL_DIR/auto-backup.conf"
        log_success "Default configuration created"
    fi
}

verify_installation() {
    log_info "Verifying installation..."

    if [[ -x "$BIN_LINK" ]]; then
        local version=$("$BIN_LINK" --version 2>/dev/null || echo "unknown")
        log_success "Installation verified: $version"
        return 0
    else
        log_error "Installation verification failed"
        return 1
    fi
}

print_success() {
    echo -e "\n${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                                                              ║"
    echo "║  ✅ AUTO-BACKUP v${VERSION} 설치 완료!                           ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  설치 위치: /opt/auto-backup/                                ║"
    echo "║  실행 명령: auto-backup                                      ║"
    echo "║  설정 파일: /opt/auto-backup/auto-backup.conf                ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  빠른 시작:                                                  ║"
    echo "║                                                              ║"
    echo "║  auto-backup              # 전체 백업                        ║"
    echo "║  auto-backup --setup      # 설정 마법사                      ║"
    echo "║  auto-backup --schedule daily  # 매일 자동 백업              ║"
    echo "║  auto-backup --help       # 도움말                           ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  弘益人間 (홍익인간) · Benefit All Humanity                  ║"
    echo "║  https://wia.family                                          ║"
    echo "║                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"
}

uninstall() {
    log_info "Uninstalling AUTO-BACKUP..."

    # Remove files
    rm -rf "$INSTALL_DIR"
    rm -f "$BIN_LINK"
    rm -f /etc/cron.d/auto-backup

    log_success "AUTO-BACKUP uninstalled"
    log_info "Backup data in /var/backups/auto-backup was preserved"
}

# ============================================================
# Main
# ============================================================
main() {
    case "${1:-}" in
        --uninstall)
            check_root
            uninstall
            exit 0
            ;;
        --help|-h)
            echo "Usage: $0 [--uninstall]"
            echo ""
            echo "Options:"
            echo "  --uninstall  Remove AUTO-BACKUP"
            echo "  --help       Show this help"
            exit 0
            ;;
    esac

    print_banner
    check_root

    log_info "Starting installation..."

    install_dependencies
    create_directories
    install_scripts
    create_default_config
    verify_installation

    print_success
}

main "$@"
