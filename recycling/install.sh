#!/bin/bash

#############################################################################
# WIA-ENE-023: Recycling System Standard - Installation Script
#
# This script installs the WIA-ENE-023 recycling standard tools and SDK
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# © 2025 SmileStory Inc. / WIA
# License: CC BY 4.0
#############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Banner
echo -e "${CYAN}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║         WIA-ENE-023: Recycling System Standard ♻️            ║"
echo "║                                                               ║"
echo "║         弘益人間 (홍익인간) - Benefit All Humanity           ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}📦 Starting WIA-ENE-023 installation...${NC}\n"

# Check for Node.js
echo -e "${YELLOW}🔍 Checking prerequisites...${NC}"
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    echo -e "${GREEN}✓ Node.js found: ${NODE_VERSION}${NC}"
else
    echo -e "${RED}✗ Node.js not found${NC}"
    echo -e "${YELLOW}  Please install Node.js 18+ from https://nodejs.org${NC}"
    exit 1
fi

# Check for npm
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    echo -e "${GREEN}✓ npm found: ${NPM_VERSION}${NC}"
else
    echo -e "${RED}✗ npm not found${NC}"
    echo -e "${YELLOW}  Please install npm${NC}"
    exit 1
fi

echo ""

# Install TypeScript SDK
if [ -d "api/typescript" ]; then
    echo -e "${BLUE}📚 Installing TypeScript SDK dependencies...${NC}"
    cd api/typescript

    if [ -f "package.json" ]; then
        npm install
        echo -e "${GREEN}✓ TypeScript SDK dependencies installed${NC}"

        # Build the SDK
        if [ -f "tsconfig.json" ] || grep -q "\"build\"" package.json; then
            echo -e "${BLUE}🔨 Building TypeScript SDK...${NC}"
            npm run build 2>/dev/null || echo -e "${YELLOW}  Note: Build script not configured yet${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ package.json not found in api/typescript${NC}"
    fi

    cd "$SCRIPT_DIR"
    echo ""
fi

# Make CLI executable
if [ -f "cli/recycling.sh" ]; then
    echo -e "${BLUE}🛠️  Setting up CLI tool...${NC}"
    chmod +x cli/recycling.sh
    echo -e "${GREEN}✓ CLI tool is now executable${NC}"
    echo ""
fi

# Create symlink for global access (optional)
echo -e "${YELLOW}❓ Would you like to install the CLI tool globally? (y/n)${NC}"
read -r INSTALL_GLOBAL

if [[ "$INSTALL_GLOBAL" =~ ^[Yy]$ ]]; then
    INSTALL_DIR="/usr/local/bin"
    CLI_NAME="wia-recycling"

    if [ -w "$INSTALL_DIR" ]; then
        ln -sf "$SCRIPT_DIR/cli/recycling.sh" "$INSTALL_DIR/$CLI_NAME"
        echo -e "${GREEN}✓ CLI tool installed globally as '${CLI_NAME}'${NC}"
    else
        echo -e "${YELLOW}  Installing globally requires sudo access${NC}"
        sudo ln -sf "$SCRIPT_DIR/cli/recycling.sh" "$INSTALL_DIR/$CLI_NAME"
        echo -e "${GREEN}✓ CLI tool installed globally as '${CLI_NAME}'${NC}"
    fi
    echo ""
fi

# Check for API key
echo -e "${BLUE}🔑 API Key Configuration${NC}"
if [ -z "$WIA_API_KEY" ]; then
    echo -e "${YELLOW}⚠ WIA_API_KEY environment variable not set${NC}"
    echo -e "  To use the WIA-ENE-023 API, you need an API key."
    echo -e "  Get your key at: ${CYAN}https://wia.org/api/keys${NC}"
    echo -e ""
    echo -e "  To set your API key, add this to your shell profile:"
    echo -e "  ${CYAN}export WIA_API_KEY='your-api-key-here'${NC}"
else
    echo -e "${GREEN}✓ WIA_API_KEY is configured${NC}"
fi

echo ""

# Display installation summary
echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}║            Installation completed successfully! ✓            ║${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"

echo ""
echo -e "${CYAN}📖 Next Steps:${NC}"
echo ""
echo -e "  1. Read the specification:"
echo -e "     ${CYAN}cat spec/WIA-ENE-023-v1.0.md${NC}"
echo ""
echo -e "  2. Try the CLI tool:"
echo -e "     ${CYAN}./cli/recycling.sh --help${NC}"
echo ""
echo -e "  3. Use the TypeScript SDK:"
echo -e "     ${CYAN}cd api/typescript && npm link${NC}"
echo -e "     ${CYAN}# Then in your project: npm link @wia/ene-023${NC}"
echo ""
echo -e "  4. View examples:"
echo -e "     ${CYAN}cat README.md${NC}"
echo ""
echo -e "${CYAN}📚 Documentation:${NC}"
echo -e "  Website:  ${BLUE}https://wia.org/standards/ene-023${NC}"
echo -e "  Docs:     ${BLUE}https://docs.wia.org/ene-023${NC}"
echo -e "  GitHub:   ${BLUE}https://github.com/WIA-Official/wia-standards${NC}"
echo ""
echo -e "${CYAN}💬 Support:${NC}"
echo -e "  Email:    ${BLUE}standards@wia.org${NC}"
echo -e "  Forum:    ${BLUE}https://forum.wia.org/ene-023${NC}"
echo ""
echo -e "${GREEN}弘益人間 (홍익인간) · Benefit All Humanity${NC}"
echo ""
