#!/bin/bash
# WIA-MENTAL-HEALTH Installation Script
#
# 弘益人間 (Benefit All Humanity)
# © 2025 WIA - World Certification Industry Association

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         🧠 WIA-MENTAL-HEALTH Installation                 ║"
echo "║                                                           ║"
echo "║              弘益人間 · Benefit All Humanity              ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Determine install location
INSTALL_DIR="/usr/local/bin"
if [ ! -w "$INSTALL_DIR" ]; then
    INSTALL_DIR="$HOME/.local/bin"
    mkdir -p "$INSTALL_DIR"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLI_SOURCE="$SCRIPT_DIR/cli/wia-mental-health.sh"
CLI_TARGET="$INSTALL_DIR/wia-mental-health"

# Check if CLI source exists
if [ ! -f "$CLI_SOURCE" ]; then
    echo -e "${RED}Error: CLI script not found at $CLI_SOURCE${NC}"
    exit 1
fi

# Install CLI
echo -e "${GREEN}Installing CLI tool...${NC}"
cp "$CLI_SOURCE" "$CLI_TARGET"
chmod +x "$CLI_TARGET"
echo -e "  ✅ CLI installed to: $CLI_TARGET"

# Check for dependencies
echo ""
echo -e "${GREEN}Checking dependencies...${NC}"

if command -v bc &> /dev/null; then
    echo -e "  ✅ bc (calculator)"
else
    echo -e "  ${YELLOW}⚠️  bc not found (install for full functionality)${NC}"
fi

if command -v jq &> /dev/null; then
    echo -e "  ✅ jq (JSON processor)"
else
    echo -e "  ${YELLOW}⚠️  jq not found (install for validation features)${NC}"
fi

# Check if install dir is in PATH
if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
    echo ""
    echo -e "${YELLOW}Note: $INSTALL_DIR is not in your PATH${NC}"
    echo "Add this to your shell profile:"
    echo ""
    echo "  export PATH=\"\$PATH:$INSTALL_DIR\""
    echo ""
fi

# Optional: Install TypeScript SDK
echo ""
echo -e "${GREEN}TypeScript SDK:${NC}"
if command -v npm &> /dev/null; then
    echo "  To install the TypeScript SDK, run:"
    echo ""
    echo "    cd $SCRIPT_DIR/api/typescript"
    echo "    npm install"
    echo ""
else
    echo -e "  ${YELLOW}⚠️  npm not found. Install Node.js for TypeScript SDK.${NC}"
fi

# Success message
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✅ Installation complete!${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Quick start:"
echo ""
echo "  wia-mental-health help"
echo "  wia-mental-health score --phq9 '2,1,2,1,0,1,0,1,0'"
echo "  wia-mental-health crisis --suicide 0 --selfharm 0"
echo ""
echo -e "${CYAN}弘益人間 · Benefit All Humanity${NC}"
