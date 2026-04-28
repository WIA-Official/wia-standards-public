#!/usr/bin/env bash

# WIA-AUG-015: Transhumanism Protocol Installation Script
# Version: 1.0.0
# License: MIT
# 弘益人間 (Benefit All Humanity)

set -euo pipefail

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║       WIA-AUG-015: Transhumanism Protocol Installer          ║"
echo "║       弘益人間 (Benefit All Humanity)                         ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}\n"

# Detect OS
OS="$(uname -s)"
case "${OS}" in
    Linux*)     MACHINE=Linux;;
    Darwin*)    MACHINE=Mac;;
    CYGWIN*)    MACHINE=Cygwin;;
    MINGW*)     MACHINE=MinGw;;
    *)          MACHINE="UNKNOWN:${OS}"
esac

echo -e "${CYAN}Detected OS: ${MACHINE}${NC}\n"

# Check dependencies
echo -e "${YELLOW}Checking dependencies...${NC}"

check_command() {
    if command -v "$1" &> /dev/null; then
        echo -e "  ${GREEN}✓${NC} $1 found"
        return 0
    else
        echo -e "  ${RED}✗${NC} $1 not found"
        return 1
    fi
}

DEPS_OK=true

if ! check_command "node"; then
    echo -e "${YELLOW}    Node.js is recommended for TypeScript SDK${NC}"
fi

if ! check_command "npm"; then
    echo -e "${YELLOW}    npm is recommended for TypeScript SDK${NC}"
fi

check_command "bash" || DEPS_OK=false

if [ "$DEPS_OK" = false ]; then
    echo -e "\n${RED}Error: Missing required dependencies${NC}"
    exit 1
fi

echo ""

# Install CLI tool
echo -e "${YELLOW}Installing CLI tool...${NC}"

INSTALL_DIR="/usr/local/bin"
CLI_SOURCE="./cli/wia-aug-015.sh"
CLI_TARGET="${INSTALL_DIR}/wia-aug-015"

if [ -f "$CLI_SOURCE" ]; then
    if [ -w "$INSTALL_DIR" ]; then
        cp "$CLI_SOURCE" "$CLI_TARGET"
        chmod +x "$CLI_TARGET"
        echo -e "${GREEN}✓ CLI tool installed to ${CLI_TARGET}${NC}"
    else
        echo -e "${YELLOW}⚠ Need sudo permissions to install to ${INSTALL_DIR}${NC}"
        sudo cp "$CLI_SOURCE" "$CLI_TARGET"
        sudo chmod +x "$CLI_TARGET"
        echo -e "${GREEN}✓ CLI tool installed to ${CLI_TARGET}${NC}"
    fi
else
    echo -e "${RED}✗ CLI source not found at ${CLI_SOURCE}${NC}"
    echo -e "${YELLOW}  Skipping CLI installation${NC}"
fi

echo ""

# Install TypeScript SDK (optional)
if command -v npm &> /dev/null; then
    echo -e "${YELLOW}TypeScript SDK installation${NC}"
    read -p "Install TypeScript SDK? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd api/typescript
        echo -e "${CYAN}Installing dependencies...${NC}"
        npm install
        echo -e "${CYAN}Building SDK...${NC}"
        npm run build
        echo -e "${GREEN}✓ TypeScript SDK installed${NC}"
        cd ../..
    fi
    echo ""
fi

# Verification
echo -e "${YELLOW}Verifying installation...${NC}"

if command -v wia-aug-015 &> /dev/null; then
    echo -e "${GREEN}✓ CLI tool accessible${NC}"
    wia-aug-015 version
else
    echo -e "${RED}✗ CLI tool not accessible${NC}"
    echo -e "${YELLOW}  You may need to restart your terminal or add ${INSTALL_DIR} to PATH${NC}"
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗"
echo -e "║                 Installation Complete!                       ║"
echo -e "╚══════════════════════════════════════════════════════════════╝${NC}\n"

echo -e "Quick start:"
echo -e "  ${CYAN}wia-aug-015 help${NC}                    - Show help"
echo -e "  ${CYAN}wia-aug-015 assess USER-001${NC}         - Assess enhancement stage"
echo -e "  ${CYAN}wia-aug-015 plan USER-001 H_PLUS_2${NC} - Plan transition"
echo ""
echo -e "Documentation: ${CYAN}./spec/WIA-AUG-015-v1.0.md${NC}"
echo -e "弘益人間 (Benefit All Humanity)"
echo ""
