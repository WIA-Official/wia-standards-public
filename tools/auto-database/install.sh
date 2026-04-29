#!/bin/bash
#
# AUTO-DATABASE v1.0 Installer
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

VERSION="1.0.0"
INSTALL_DIR="/opt/auto-database"
BIN_LINK="/usr/local/bin/auto-database"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  AUTO-DATABASE v${VERSION} Installer                             ║"
echo "║  弘益人間 (홍익인간) · Benefit All Humanity                  ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

[[ $EUID -ne 0 ]] && echo "Please run as root" && exit 1

mkdir -p "$INSTALL_DIR"/{modules,data,logs}
mkdir -p /var/backups/auto-database

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${script_dir}/auto-database.sh" ]]; then
    cp "${script_dir}/auto-database.sh" "$INSTALL_DIR/"
else
    curl -sSL "https://raw.githubusercontent.com/WIA-Official/auto-tools/main/auto-database/auto-database.sh" \
        -o "$INSTALL_DIR/auto-database.sh"
fi

chmod +x "$INSTALL_DIR/auto-database.sh"
ln -sf "$INSTALL_DIR/auto-database.sh" "$BIN_LINK"

echo -e "${GREEN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ AUTO-DATABASE v${VERSION} 설치 완료!                         ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  사용법:                                                     ║"
echo "║    auto-database --detect       # DB 감지                    ║"
echo "║    auto-database backup         # 백업                       ║"
echo "║    auto-database list           # 백업 목록                  ║"
echo "║    auto-database restore latest # 복원                       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
