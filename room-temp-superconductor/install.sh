#!/bin/bash

###############################################################################
# WIA-QUA-019: Room-Temperature Superconductor Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Quantum Research Group
#
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║${NC}  ${MAGENTA}❄️  WIA-QUA-019: Room-Temperature Superconductor${NC}      ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}     Installation Script                                   ${CYAN}║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

###############################################################################
# Check Dependencies
###############################################################################

echo -e "${YELLOW}Checking dependencies...${NC}"

# Check for required commands
MISSING_DEPS=()

if ! command -v node &> /dev/null; then
    MISSING_DEPS+=("node")
fi

if ! command -v npm &> /dev/null; then
    MISSING_DEPS+=("npm")
fi

if ! command -v bc &> /dev/null; then
    MISSING_DEPS+=("bc")
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${RED}Missing dependencies:${NC} ${MISSING_DEPS[*]}"
    echo ""
    echo "Please install the missing dependencies:"
    echo "  - Node.js and npm: https://nodejs.org/"
    echo "  - bc: sudo apt-get install bc (on Debian/Ubuntu)"
    exit 1
fi

echo -e "${GREEN}✓${NC} All dependencies found"
echo ""

###############################################################################
# Install TypeScript SDK
###############################################################################

echo -e "${YELLOW}Installing TypeScript SDK...${NC}"

cd "$SCRIPT_DIR/api/typescript"

if [ -f "package.json" ]; then
    echo "  Installing npm packages..."
    npm install --silent

    if [ -f "tsconfig.json" ]; then
        echo "  Building TypeScript..."
        npm run build --silent 2>/dev/null || echo "  (Build step skipped - tsconfig.json may be needed)"
    fi

    echo -e "${GREEN}✓${NC} TypeScript SDK installed"
else
    echo -e "${YELLOW}!${NC} package.json not found, skipping npm install"
fi

cd "$SCRIPT_DIR"
echo ""

###############################################################################
# Install CLI Tool
###############################################################################

echo -e "${YELLOW}Installing CLI tool...${NC}"

CLI_SOURCE="$SCRIPT_DIR/cli/wia-qua-019.sh"
CLI_DEST="/usr/local/bin/wia-qua-019"

# Make CLI executable
chmod +x "$CLI_SOURCE"

# Try to symlink to /usr/local/bin
if [ -w "/usr/local/bin" ]; then
    ln -sf "$CLI_SOURCE" "$CLI_DEST"
    echo -e "${GREEN}✓${NC} CLI installed to $CLI_DEST"
elif sudo -n true 2>/dev/null; then
    sudo ln -sf "$CLI_SOURCE" "$CLI_DEST"
    echo -e "${GREEN}✓${NC} CLI installed to $CLI_DEST (with sudo)"
else
    echo -e "${YELLOW}!${NC} Cannot install to /usr/local/bin (no write permission)"
    echo "  You can manually create a symlink:"
    echo "    sudo ln -s $CLI_SOURCE $CLI_DEST"
    echo ""
    echo "  Or add the CLI directory to your PATH:"
    echo "    export PATH=\"$SCRIPT_DIR/cli:\$PATH\""
fi

echo ""

###############################################################################
# Verify Installation
###############################################################################

echo -e "${YELLOW}Verifying installation...${NC}"

if command -v wia-qua-019 &> /dev/null; then
    VERSION=$(wia-qua-019 --version | head -1)
    echo -e "${GREEN}✓${NC} CLI tool is available: $VERSION"
else
    echo -e "${YELLOW}!${NC} CLI tool not in PATH (you may need to add it manually)"
fi

echo ""

###############################################################################
# Installation Complete
###############################################################################

echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║${NC}  Installation Complete!                                   ${GREEN}║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo "Quick Start:"
echo ""
echo "  1. Analyze hydrogen-rich hydride:"
echo "     wia-qua-019 hydride --name LaH10 --pressure 170 --temp 250"
echo ""
echo "  2. Test LK-99 material:"
echo "     wia-qua-019 lk99 --copper-doping 0.1 --characterize"
echo ""
echo "  3. Measure critical temperature:"
echo "     wia-qua-019 tc-measure --material H3S --method four-point"
echo ""
echo "  4. Perform Meissner test:"
echo "     wia-qua-019 meissner --temp 300 --field 0.01 --levitation"
echo ""
echo "  5. Show all commands:"
echo "     wia-qua-019 help"
echo ""
echo "  6. View physical constants:"
echo "     wia-qua-019 constants"
echo ""

echo -e "${CYAN}弘益人間 (Benefit All Humanity)${NC}"
echo ""
echo "Documentation: https://github.com/WIA-Official/wia-standards"
echo "Issues: https://github.com/WIA-Official/wia-standards/issues"
echo ""

###############################################################################
# 弘益人間 (홍익인간) · Benefit All Humanity
###############################################################################
