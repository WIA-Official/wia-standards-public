#!/bin/bash
#
# AUTO-HEALTH v1.0 Installer
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

VERSION="1.0.0"
INSTALL_DIR="/opt/auto-health"
BIN_LINK="/usr/local/bin/auto-health"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  AUTO-HEALTH v${VERSION} Installer                               ║"
echo "║  弘益人間 (홍익인간) · Benefit All Humanity                  ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

[[ $EUID -ne 0 ]] && echo "Please run as root" && exit 1

mkdir -p "$INSTALL_DIR"/{modules/{security,performance,services,filesystem},fixers,templates,data,logs}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${script_dir}/auto-health.sh" ]]; then
    cp "${script_dir}/auto-health.sh" "$INSTALL_DIR/"
else
    curl -sSL "https://raw.githubusercontent.com/WIA-Official/auto-tools/main/auto-health/auto-health.sh" \
        -o "$INSTALL_DIR/auto-health.sh"
fi

chmod +x "$INSTALL_DIR/auto-health.sh"
ln -sf "$INSTALL_DIR/auto-health.sh" "$BIN_LINK"

echo -e "${GREEN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ AUTO-HEALTH v${VERSION} 설치 완료!                           ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  사용법:                                                     ║"
echo "║    auto-health              # 기본 진단                      ║"
echo "║    auto-health --quick      # 빠른 진단                      ║"
echo "║    auto-health --security   # 보안 진단                      ║"
echo "║    auto-health --fix        # 자동 수정                      ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
