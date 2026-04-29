#!/bin/bash
#
# AUTO-DEPLOY v1.0 Installer
# Zero-Downtime Deployment Solution
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#

set -e

VERSION="1.0.0"
INSTALL_DIR="/opt/auto-deploy"
BIN_LINK="/usr/local/bin/auto-deploy"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log_info()    { echo -e "${CYAN}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_error()   { echo -e "${RED}[✗]${NC} $1"; exit 1; }

print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   AUTO-DEPLOY v1.0 Installer                                 ║
║   Zero-Downtime Deployment Solution                          ║
║                                                              ║
║   弘益人間 (홍익인간) · Benefit All Humanity                 ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This installer must be run as root"
    fi
}

install_dependencies() {
    log_info "Checking dependencies..."

    local missing=()

    command -v git &>/dev/null || missing+=("git")
    command -v curl &>/dev/null || missing+=("curl")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_info "Installing: ${missing[*]}"

        if command -v apt-get &>/dev/null; then
            apt-get update -qq && apt-get install -y -qq "${missing[@]}"
        elif command -v dnf &>/dev/null; then
            dnf install -y -q "${missing[@]}"
        elif command -v yum &>/dev/null; then
            yum install -y -q "${missing[@]}"
        fi
    fi

    log_success "Dependencies OK"
}

install_scripts() {
    log_info "Installing AUTO-DEPLOY..."

    mkdir -p "$INSTALL_DIR"/{modules,templates,logs,projects}

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    if [[ -f "${script_dir}/auto-deploy.sh" ]]; then
        cp "${script_dir}/auto-deploy.sh" "$INSTALL_DIR/"
    else
        curl -sSL "https://raw.githubusercontent.com/WIA-Official/auto-tools/main/auto-deploy/auto-deploy.sh" \
            -o "$INSTALL_DIR/auto-deploy.sh"
    fi

    chmod +x "$INSTALL_DIR/auto-deploy.sh"
    ln -sf "$INSTALL_DIR/auto-deploy.sh" "$BIN_LINK"

    # Create default config
    cat > "$INSTALL_DIR/auto-deploy.conf" << 'EOF'
#!/bin/bash
# AUTO-DEPLOY v1.0 Global Configuration

DEPLOY_BASE_DIR="/var/www"
RELEASES_SUFFIX="-releases"
KEEP_RELEASES=5
LOG_FILE="/var/log/auto-deploy.log"

GIT_DEPTH=1
DEFAULT_BRANCH="main"

WEBHOOK_PORT=9000
WEBHOOK_SECRET=""

NOTIFY_ENABLED=false
SLACK_WEBHOOK=""
DISCORD_WEBHOOK=""
EOF

    chmod 600 "$INSTALL_DIR/auto-deploy.conf"

    log_success "Installed to $INSTALL_DIR"
}

print_success() {
    echo -e "\n${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ✅ AUTO-DEPLOY v${VERSION} 설치 완료!                           ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  빠른 시작:                                                  ║"
    echo "║                                                              ║"
    echo "║  # 프로젝트 초기화                                           ║"
    echo "║  auto-deploy --init https://github.com/user/app.git \\       ║"
    echo "║              --path /var/www/app                             ║"
    echo "║                                                              ║"
    echo "║  # 배포                                                      ║"
    echo "║  auto-deploy myapp                                           ║"
    echo "║                                                              ║"
    echo "║  # 롤백                                                      ║"
    echo "║  auto-deploy myapp --rollback                                ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  弘益人間 (홍익인간) · https://wia.family                    ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"
}

main() {
    print_banner
    check_root
    install_dependencies
    install_scripts
    print_success
}

main "$@"
