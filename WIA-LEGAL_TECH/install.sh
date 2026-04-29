#!/usr/bin/env bash

###############################################################################
# WIA-LEGAL_TECH Installation Script
# Version: 1.0.0
# Philosophy: 弘益人間 (Hongik Ingan - Benefit All Humanity)
###############################################################################

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         WIA-LEGAL_TECH Installation Script              ║${NC}"
echo -e "${BLUE}║         弘益人間 (Benefit All Humanity)                   ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check prerequisites
echo -e "${BLUE}[1/5]${NC} Checking prerequisites..."

if ! command -v node &> /dev/null; then
    echo -e "${RED}Error: Node.js is not installed${NC}"
    echo "Please install Node.js 16+ from https://nodejs.org"
    exit 1
fi

if ! command -v npm &> /dev/null; then
    echo -e "${RED}Error: npm is not installed${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Prerequisites check passed"
echo ""

# Install TypeScript SDK
echo -e "${BLUE}[2/5]${NC} Installing TypeScript SDK dependencies..."
cd api/typescript
npm install
npm run build
echo -e "${GREEN}✓${NC} TypeScript SDK installed"
echo ""

# Install CLI tool
echo -e "${BLUE}[3/5]${NC} Setting up CLI tool..."
cd ../..
chmod +x cli/wia-legal-tech.sh

# Create symlink for global access (optional)
if [ -w /usr/local/bin ]; then
    ln -sf "$(pwd)/cli/wia-legal-tech.sh" /usr/local/bin/wia-legal-tech
    echo -e "${GREEN}✓${NC} CLI tool installed globally as 'wia-legal-tech'"
else
    echo -e "${BLUE}ℹ${NC} Run with sudo to install CLI globally, or use ./cli/wia-legal-tech.sh"
fi
echo ""

# Configure environment
echo -e "${BLUE}[4/5]${NC} Environment configuration..."
if [ ! -f .env ]; then
    cat > .env << ENV
# WIA-LEGAL_TECH Configuration
WIA_LEGAL_TECH_API_KEY=your_api_key_here
WIA_LEGAL_TECH_API_URL=https://api.wia-legal.tech/v1
ENV
    echo -e "${GREEN}✓${NC} Created .env file (please update with your API key)"
else
    echo -e "${BLUE}ℹ${NC} .env file already exists"
fi
echo ""

# Verify installation
echo -e "${BLUE}[5/5]${NC} Verifying installation..."
if [ -f api/typescript/dist/index.js ]; then
    echo -e "${GREEN}✓${NC} TypeScript SDK build successful"
fi

if [ -x cli/wia-legal-tech.sh ]; then
    echo -e "${GREEN}✓${NC} CLI tool executable"
fi
echo ""

echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║          Installation Complete!                          ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Next steps:"
echo "1. Update .env with your API key"
echo "2. Try: wia-legal-tech --help"
echo "3. Read documentation: ebook/en/index.html"
echo ""
echo -e "${BLUE}弘益人間 (Hongik Ingan) - Benefit All Humanity${NC}"
