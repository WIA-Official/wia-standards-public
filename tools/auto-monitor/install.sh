#!/bin/bash
#
# AUTO-MONITOR v1.0 Installer
# Server Monitoring Solution
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#

set -e

VERSION="1.0.0"
INSTALL_DIR="/opt/auto-monitor"
BIN_LINK="/usr/local/bin/auto-monitor"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
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
║   AUTO-MONITOR v1.0 Installer                                ║
║   Server Monitoring Solution                                 ║
║                                                              ║
║   弘益人間 (홍익인간) · Benefit All Humanity                 ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

check_root() {
    [[ $EUID -ne 0 ]] && log_error "This installer must be run as root"
}

install_dependencies() {
    log_info "Checking dependencies..."

    local deps="curl bc"
    local missing=()

    for dep in $deps; do
        command -v "$dep" &>/dev/null || missing+=("$dep")
    done

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
    log_info "Installing AUTO-MONITOR..."

    mkdir -p "$INSTALL_DIR"/{modules,data,templates,logs}

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    if [[ -f "${script_dir}/auto-monitor.sh" ]]; then
        cp "${script_dir}/auto-monitor.sh" "$INSTALL_DIR/"
    else
        curl -sSL "https://raw.githubusercontent.com/WIA-Official/auto-tools/main/auto-monitor/auto-monitor.sh" \
            -o "$INSTALL_DIR/auto-monitor.sh"
    fi

    chmod +x "$INSTALL_DIR/auto-monitor.sh"
    ln -sf "$INSTALL_DIR/auto-monitor.sh" "$BIN_LINK"

    log_success "Installed to $INSTALL_DIR"
}

print_success() {
    echo -e "\n${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ✅ AUTO-MONITOR v${VERSION} 설치 완료!                          ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  빠른 시작:                                                  ║"
    echo "║                                                              ║"
    echo "║  auto-monitor              # 한 번 체크                      ║"
    echo "║  auto-monitor --watch      # 실시간 대시보드                 ║"
    echo "║  auto-monitor --setup      # 설정 마법사                     ║"
    echo "║  auto-monitor --install    # cron 등록 (5분마다)             ║"
    echo "║                                                              ║"
    echo "║  # 알림 설정                                                 ║"
    echo "║  auto-monitor --setup-slack https://hooks.slack.com/...      ║"
    echo "║  auto-monitor --setup-telegram TOKEN CHAT_ID                 ║"
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
