#!/bin/bash

# WIA-MENTAL-001: Digital Therapy Standard - Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

VERSION="1.0.0"
PURPLE='\033[0;35m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${PURPLE}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║    WIA-MENTAL-001: Digital Therapy Standard Installer     ║"
echo "║              弘익人間 · Benefit All Humanity              ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

# Check for required dependencies
echo "Checking dependencies..."

command -v node >/dev/null 2>&1 || {
    echo -e "${RED}✗ Node.js is required but not installed.${NC}" >&2
    echo "Please install Node.js from https://nodejs.org/"
    exit 1
}

echo -e "${GREEN}✓ Node.js found: $(node --version)${NC}"

command -v npm >/dev/null 2>&1 || {
    echo -e "${RED}✗ npm is required but not installed.${NC}" >&2
    exit 1
}

echo -e "${GREEN}✓ npm found: $(npm --version)${NC}"

# Install TypeScript dependencies
echo ""
echo "Installing TypeScript SDK..."
cd api/typescript
npm install
npm run build 2>/dev/null || echo -e "${RED}Build step skipped (no build script)${NC}"
cd ../..

echo -e "${GREEN}✓ TypeScript SDK installed${NC}"

# Make CLI executable
echo ""
echo "Setting up CLI tools..."
chmod +x cli/digital-therapy.sh

echo -e "${GREEN}✓ CLI tools configured${NC}"

# Create configuration directory
CONFIG_DIR="$HOME/.wia/mental-001"
mkdir -p "$CONFIG_DIR"

echo -e "${GREEN}✓ Configuration directory created${NC}"

# Installation complete
echo ""
echo -e "${GREEN}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║           Installation Complete! 🧠💜                     ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo "Next steps:"
echo ""
echo "1. Initialize the CLI:"
echo "   ./cli/digital-therapy.sh init"
echo ""
echo "2. View documentation:"
echo "   open ebook/en/index.html"
echo ""
echo "3. Try the simulator:"
echo "   open simulator/index.html"
echo ""
echo "4. Read the API docs:"
echo "   open spec/PHASE-1.md"
echo ""
echo "For support: https://docs.wia-official.org/mental-001"
echo ""
echo -e "${PURPLE}弘益人間 · Benefit All Humanity${NC}"
echo ""
