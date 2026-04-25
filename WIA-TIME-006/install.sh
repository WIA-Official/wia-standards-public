#!/bin/bash

################################################################################
# WIA-TIME-006: Universal Time Database Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

VERSION="1.0.0"

echo -e "${VIOLET}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║        🗄️  WIA-TIME-006: Universal Time Database              ║"
echo "║                   Installation Script                          ║"
echo "║                      Version $VERSION                            ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${RESET}"
echo ""

# Check prerequisites
echo -e "${VIOLET}▶ Checking Prerequisites${RESET}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check for bash
if ! command -v bash &> /dev/null; then
    echo -e "${RED}✗ bash is required but not installed${RESET}"
    exit 1
fi
echo -e "${GREEN}✓ bash found: $(bash --version | head -n1)${RESET}"

# Check for jq (optional but recommended)
if command -v jq &> /dev/null; then
    echo -e "${GREEN}✓ jq found: $(jq --version)${RESET}"
else
    echo -e "${YELLOW}⚠ jq not found (optional, for JSON parsing)${RESET}"
fi

# Check for node (for TypeScript SDK)
if command -v node &> /dev/null; then
    echo -e "${GREEN}✓ Node.js found: $(node --version)${RESET}"
else
    echo -e "${YELLOW}⚠ Node.js not found (required for TypeScript SDK)${RESET}"
fi

echo ""

# Install CLI tool
echo -e "${VIOLET}▶ Installing CLI Tool${RESET}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

INSTALL_DIR="/usr/local/bin"
CLI_SOURCE="./cli/wia-time-006.sh"
CLI_TARGET="$INSTALL_DIR/wia-time-006"

if [ -f "$CLI_SOURCE" ]; then
    # Check if we need sudo
    if [ -w "$INSTALL_DIR" ]; then
        cp "$CLI_SOURCE" "$CLI_TARGET"
        chmod +x "$CLI_TARGET"
    else
        echo "Installing to $INSTALL_DIR requires sudo privileges..."
        sudo cp "$CLI_SOURCE" "$CLI_TARGET"
        sudo chmod +x "$CLI_TARGET"
    fi

    echo -e "${GREEN}✓ CLI tool installed to $CLI_TARGET${RESET}"
else
    echo -e "${RED}✗ CLI source not found at $CLI_SOURCE${RESET}"
    exit 1
fi

echo ""

# Install TypeScript SDK
echo -e "${VIOLET}▶ Installing TypeScript SDK${RESET}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if command -v npm &> /dev/null; then
    cd api/typescript

    echo "Installing dependencies..."
    npm install

    echo "Building SDK..."
    npm run build

    echo -e "${GREEN}✓ TypeScript SDK built successfully${RESET}"
    echo ""
    echo "To use the SDK in your project:"
    echo "  npm install file:$(pwd)"
    echo ""
    echo "Or link it globally:"
    echo "  npm link"
    echo ""

    cd ../..
else
    echo -e "${YELLOW}⚠ npm not found, skipping TypeScript SDK installation${RESET}"
    echo "  Install Node.js and npm to build the TypeScript SDK"
fi

echo ""

# Create config directory
echo -e "${VIOLET}▶ Setting Up Configuration${RESET}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

CONFIG_DIR="$HOME/.wia-time-006"
mkdir -p "$CONFIG_DIR"
mkdir -p "$CONFIG_DIR/data"

echo -e "${GREEN}✓ Configuration directory created: $CONFIG_DIR${RESET}"

# Create default config
cat > "$CONFIG_DIR/config.json" << EOC
{
  "version": "$VERSION",
  "universe": "prime",
  "timeline": "alpha-001",
  "storage": {
    "type": "local",
    "replicationFactor": 1,
    "consistency": "causal"
  },
  "initialized": "$(date -Iseconds)"
}
EOC

echo -e "${GREEN}✓ Default configuration created${RESET}"

echo ""

# Verify installation
echo -e "${VIOLET}▶ Verifying Installation${RESET}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if command -v wia-time-006 &> /dev/null; then
    echo -e "${GREEN}✓ CLI tool is accessible${RESET}"
    wia-time-006 version
else
    echo -e "${RED}✗ CLI tool not found in PATH${RESET}"
    echo "  You may need to add $INSTALL_DIR to your PATH"
fi

echo ""

# Installation complete
echo -e "${GREEN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║              Installation Complete! 🎉                         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${RESET}"
echo ""
echo "Quick Start:"
echo ""
echo "  1. Initialize database:"
echo "     wia-time-006 init --storage distributed --replicas 3"
echo ""
echo "  2. Insert an event:"
echo "     wia-time-006 insert --time 'now' --event 'First event'"
echo ""
echo "  3. Query events:"
echo "     wia-time-006 query --timeline 'alpha-*'"
echo ""
echo "  4. Get help:"
echo "     wia-time-006 help"
echo ""
echo "Documentation:"
echo "  - Specification: ./spec/WIA-TIME-006-v1.0.md"
echo "  - README: ./README.md"
echo "  - TypeScript API: ./api/typescript/README.md"
echo ""
echo -e "${VIOLET}弘익人間 (Benefit All Humanity)${RESET}"
echo -e "© 2025 SmileStory Inc. / WIA - MIT License"
echo ""

exit 0
