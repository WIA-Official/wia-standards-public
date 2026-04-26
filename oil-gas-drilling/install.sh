#!/bin/bash

###############################################################################
# WIA-ENE-036: Oil & Gas Drilling Standard - Installation Script
#
# Description: Automated installation script for SDK and CLI tools
#
# Version: 1.0.0
# License: CC BY 4.0
#
# 弘益人間 (홍익인간) - Benefit All Humanity
###############################################################################

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

# Emoji
OIL="🛢️ "
CHECK="✅"
CROSS="❌"
INFO="ℹ️ "
ROCKET="🚀"
FIRE="🔥"

echo -e "${BOLD}${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ${OIL}WIA-ENE-036: Oil & Gas Drilling Standard          ║"
echo "║  Installation Script                                         ║"
echo "║  弘益人間 (홍익인간) - Benefit All Humanity                 ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}\n"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

###############################################################################
# Check Dependencies
###############################################################################

echo -e "${BLUE}${INFO} Checking dependencies...${NC}"

# Check Node.js
if ! command -v node &> /dev/null; then
    echo -e "${RED}${CROSS} Node.js is not installed.${NC}"
    echo -e "${YELLOW}Please install Node.js (>= 16.0.0) from https://nodejs.org/${NC}"
    exit 1
fi

NODE_VERSION=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
if [ "$NODE_VERSION" -lt 16 ]; then
    echo -e "${RED}${CROSS} Node.js version must be >= 16.0.0${NC}"
    echo -e "${YELLOW}Current version: $(node --version)${NC}"
    exit 1
fi

echo -e "${GREEN}${CHECK} Node.js $(node --version)${NC}"

# Check npm
if ! command -v npm &> /dev/null; then
    echo -e "${RED}${CROSS} npm is not installed.${NC}"
    exit 1
fi

echo -e "${GREEN}${CHECK} npm $(npm --version)${NC}"

# Check curl (for CLI)
if ! command -v curl &> /dev/null; then
    echo -e "${YELLOW}⚠️  curl is not installed. CLI tool will not work properly.${NC}"
    echo -e "${YELLOW}Please install curl to use the CLI tool.${NC}"
fi

# Check jq (for CLI)
if ! command -v jq &> /dev/null; then
    echo -e "${YELLOW}⚠️  jq is not installed. CLI tool will not work properly.${NC}"
    echo -e "${YELLOW}Please install jq to use the CLI tool.${NC}"
fi

# Check bc (for calculations in CLI)
if ! command -v bc &> /dev/null; then
    echo -e "${YELLOW}⚠️  bc is not installed. Some CLI calculations may not work.${NC}"
    echo -e "${YELLOW}Please install bc for full CLI functionality.${NC}"
fi

###############################################################################
# Install TypeScript SDK
###############################################################################

echo -e "\n${BLUE}${ROCKET} Installing TypeScript SDK...${NC}"

cd "$SCRIPT_DIR/api/typescript"

if [ -f "package.json" ]; then
    echo -e "${INFO} Running npm install..."
    npm install

    echo -e "${INFO} Building TypeScript SDK..."
    npm run build || true  # Continue even if build fails (dev dependencies might be missing)

    echo -e "${GREEN}${CHECK} TypeScript SDK installed${NC}"
else
    echo -e "${YELLOW}⚠️  package.json not found. Skipping SDK installation.${NC}"
fi

###############################################################################
# Install CLI Tool
###############################################################################

echo -e "\n${BLUE}${ROCKET} Installing CLI tool...${NC}"

CLI_PATH="$SCRIPT_DIR/cli/oil-gas-drilling.sh"

if [ -f "$CLI_PATH" ]; then
    chmod +x "$CLI_PATH"
    echo -e "${GREEN}${CHECK} CLI tool executable permissions set${NC}"

    # Ask user if they want to create a symlink
    read -p "Create symlink in /usr/local/bin? (requires sudo) [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo ln -sf "$CLI_PATH" /usr/local/bin/oil-gas-drilling
        echo -e "${GREEN}${CHECK} Symlink created: /usr/local/bin/oil-gas-drilling${NC}"
    else
        echo -e "${INFO} Skipping symlink creation."
        echo -e "${INFO} You can run the CLI tool directly: $CLI_PATH"
    fi
else
    echo -e "${YELLOW}⚠️  CLI tool not found at: $CLI_PATH${NC}"
fi

###############################################################################
# Setup Environment Variables
###############################################################################

echo -e "\n${BLUE}${INFO} Environment Variable Setup${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

echo -e "\nTo use the WIA-ENE-036 tools, you need to set the following environment variables:"
echo -e "\n${BOLD}Required:${NC}"
echo -e "  export WIA_ENE036_API_KEY=\"your-api-key-here\""

echo -e "\n${BOLD}Optional:${NC}"
echo -e "  export WIA_ENE036_ENDPOINT=\"https://api.wia.org/ene-036/v1\""
echo -e "  export WIA_ENE036_OPERATOR_ID=\"your-operator-id\""

echo -e "\n${INFO} You can add these to your shell configuration file:"
echo -e "  - Bash: ~/.bashrc or ~/.bash_profile"
echo -e "  - Zsh: ~/.zshrc"
echo -e "  - Fish: ~/.config/fish/config.fish"

read -p "Would you like to add these to your ~/.bashrc now? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    read -p "Enter your WIA API Key: " api_key
    read -p "Enter your Operator ID (optional): " operator_id

    {
        echo ""
        echo "# WIA-ENE-036: Oil & Gas Drilling Standard"
        echo "export WIA_ENE036_API_KEY=\"$api_key\""
        [ -n "$operator_id" ] && echo "export WIA_ENE036_OPERATOR_ID=\"$operator_id\""
    } >> ~/.bashrc

    echo -e "${GREEN}${CHECK} Environment variables added to ~/.bashrc${NC}"
    echo -e "${INFO} Run 'source ~/.bashrc' to apply changes"
fi

###############################################################################
# Test Installation
###############################################################################

echo -e "\n${BLUE}${INFO} Testing installation...${NC}"

# Test CLI
if command -v oil-gas-drilling &> /dev/null || [ -x "$CLI_PATH" ]; then
    CLI_CMD="oil-gas-drilling"
    if ! command -v oil-gas-drilling &> /dev/null; then
        CLI_CMD="$CLI_PATH"
    fi

    VERSION_OUTPUT=$($CLI_CMD version 2>&1 || true)
    if echo "$VERSION_OUTPUT" | grep -q "WIA-ENE-036"; then
        echo -e "${GREEN}${CHECK} CLI tool is working${NC}"
    else
        echo -e "${YELLOW}⚠️  CLI tool installed but may not be configured correctly${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  CLI tool not found in PATH${NC}"
fi

###############################################################################
# Installation Complete
###############################################################################

echo -e "\n${GREEN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ${CHECK} Installation Complete!                                ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BOLD}Next Steps:${NC}"
echo -e "1. Set your API key (if not already done):"
echo -e "   ${YELLOW}export WIA_ENE036_API_KEY=\"your-api-key\"${NC}"
echo -e ""
echo -e "2. Set your Operator ID (optional but recommended):"
echo -e "   ${YELLOW}export WIA_ENE036_OPERATOR_ID=\"your-operator-id\"${NC}"
echo -e ""
echo -e "3. Test the CLI tool:"
echo -e "   ${YELLOW}oil-gas-drilling version${NC}"
echo -e "   ${YELLOW}oil-gas-drilling help${NC}"
echo -e ""
echo -e "4. Create your first well:"
echo -e "   ${YELLOW}oil-gas-drilling create-well${NC}"
echo -e ""
echo -e "5. Use the TypeScript SDK in your project:"
echo -e "   ${YELLOW}import { OilGasDrillingClient } from '@wia/ene-036';${NC}"
echo -e ""
echo -e "6. Read the documentation:"
echo -e "   ${YELLOW}cat $SCRIPT_DIR/README.md${NC}"
echo -e "   ${YELLOW}cat $SCRIPT_DIR/spec/WIA-ENE-036-v1.0.md${NC}"
echo -e ""
echo -e "${BOLD}Resources:${NC}"
echo -e "  📖 Documentation: https://wia.org/standards/ene-036"
echo -e "  💻 GitHub: https://github.com/WIA-Official/wia-standards"
echo -e "  📧 Support: oil-gas-drilling@wia.org"
echo -e ""
echo -e "${BOLD}Quick Start Examples:${NC}"
echo -e "  ${CYAN}# List your wells${NC}"
echo -e "  oil-gas-drilling list-wells"
echo -e ""
echo -e "  ${CYAN}# Submit daily production${NC}"
echo -e "  oil-gas-drilling submit-production WIA-OG-2025-TX-001234"
echo -e ""
echo -e "  ${CYAN}# Report emissions${NC}"
echo -e "  oil-gas-drilling report-emissions WIA-OG-2025-TX-001234"
echo -e ""
echo -e "  ${CYAN}# View operator dashboard${NC}"
echo -e "  oil-gas-drilling dashboard"
echo -e ""
echo -e "${CYAN}${BOLD}弘益人間 (홍익인간) - Benefit All Humanity${NC}"
echo -e "${OIL}${FIRE}Thank you for using WIA-ENE-036 Oil & Gas Drilling Standard!\n"
