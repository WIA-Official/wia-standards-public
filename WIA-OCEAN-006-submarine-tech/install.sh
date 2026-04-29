#!/bin/bash

################################################################################
# WIA-OCEAN-006: Submarine Technology Standard
# Installation Script
#
# Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
################################################################################

set -e

VERSION="1.0.0"
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║    WIA-OCEAN-006: Submarine Technology Standard Setup     ║"
echo "║              弘益人間 · Benefit All Humanity                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo "Installing WIA-OCEAN-006 Submarine Technology Standard..."

# Check for Node.js
if command -v node &> /dev/null; then
    echo -e "${GREEN}✓ Node.js found: $(node --version)${NC}"

    # Install TypeScript SDK
    if [ -d "api/typescript" ]; then
        echo "Installing TypeScript SDK dependencies..."
        cd api/typescript
        npm install
        npm run build
        cd ../..
        echo -e "${GREEN}✓ TypeScript SDK installed${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Node.js not found. Skipping TypeScript SDK installation.${NC}"
fi

# Make CLI executable
if [ -f "cli/submarine-tech.sh" ]; then
    chmod +x cli/submarine-tech.sh
    echo -e "${GREEN}✓ CLI tools configured${NC}"
fi

# Create symlink for easy access
INSTALL_DIR="$HOME/.local/bin"
if [ -d "$INSTALL_DIR" ]; then
    ln -sf "$(pwd)/cli/submarine-tech.sh" "$INSTALL_DIR/wia-ocean-006"
    echo -e "${GREEN}✓ Command 'wia-ocean-006' available${NC}"
else
    echo -e "${YELLOW}⚠ $INSTALL_DIR not found. Add cli/ to your PATH manually.${NC}"
fi

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "Quick start:"
echo "  1. View status: ./cli/submarine-tech.sh status"
echo "  2. Read sensors: ./cli/submarine-tech.sh sensor read"
echo "  3. Start mission: ./cli/submarine-tech.sh mission start"
echo ""
echo "Documentation:"
echo "  - Main docs: ./index.html"
echo "  - Simulator: ./simulator/index.html"
echo "  - E-book: ./ebook/en/index.html"
echo "  - Specs: ./spec/PHASE-1.md"
echo ""
echo "TypeScript usage:"
echo "  npm install @wia/ocean-006"
echo ""
echo "弘益人間 (홍익인간) - Benefit All Humanity"
