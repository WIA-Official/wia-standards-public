#!/bin/bash

###############################################################################
# WIA-CONTACT-001: First Contact Protocol - Installation Script
# 弘益人間 (홍익인간) · Benefit All Humanity
###############################################################################

set -e

CONTACT_COLOR="\033[38;5;99m"
RESET="\033[0m"
BOLD="\033[1m"
GREEN="\033[32m"
YELLOW="\033[33m"

echo -e "${CONTACT_COLOR}${BOLD}"
echo "╔═══════════════════════════════════════════════════════╗"
echo "║   WIA-CONTACT-001: First Contact Protocol Installer  ║"
echo "║   弘益人間 · Benefit All Humanity                      ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo -e "${RESET}"

echo ""
echo "This installer will set up the First Contact Protocol tools and dependencies."
echo ""

# Check prerequisites
echo -e "${CONTACT_COLOR}Checking prerequisites...${RESET}"

check_command() {
    if command -v "$1" &> /dev/null; then
        echo -e "  ${GREEN}✓${RESET} $1 found"
        return 0
    else
        echo -e "  ${YELLOW}✗${RESET} $1 not found (optional)"
        return 1
    fi
}

check_command "node"
check_command "npm"
check_command "python3"
check_command "git"

echo ""

# Make CLI executable
echo -e "${CONTACT_COLOR}Setting up CLI tools...${RESET}"
chmod +x cli/wia-contact-001.sh
echo -e "  ${GREEN}✓${RESET} CLI tool is now executable"

# Create symlink (optional)
if [ -w "/usr/local/bin" ]; then
    ln -sf "$(pwd)/cli/wia-contact-001.sh" /usr/local/bin/wia-contact-001 2>/dev/null || true
    if [ -L "/usr/local/bin/wia-contact-001" ]; then
        echo -e "  ${GREEN}✓${RESET} Symlink created: /usr/local/bin/wia-contact-001"
    fi
fi

echo ""

# Install Node.js dependencies (if package.json exists)
if [ -f "api/typescript/package.json" ]; then
    echo -e "${CONTACT_COLOR}Installing TypeScript dependencies...${RESET}"
    cd api/typescript
    npm install --quiet
    cd ../..
    echo -e "  ${GREEN}✓${RESET} TypeScript dependencies installed"
    echo ""
fi

# Installation complete
echo -e "${GREEN}${BOLD}✓ Installation complete!${RESET}"
echo ""
echo "Quick Start:"
echo "  1. Run CLI: ./cli/wia-contact-001.sh --help"
echo "  2. Open simulator: open simulator/index.html"
echo "  3. Read docs: open ebook/en/index.html"
echo ""
echo "For more information:"
echo "  • Documentation: ./ebook/en/"
echo "  • Specifications: ./spec/"
echo "  • API Reference: ./api/"
echo ""
echo -e "${CONTACT_COLOR}Humanity's preparation for First Contact begins now.${RESET}"
echo ""
