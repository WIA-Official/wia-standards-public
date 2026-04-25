#!/bin/bash
################################################################################
# WIA-HERITAGE-009: Cultural Repatriation Data Standard - Installation Script
# 弘益人間 (Benefit All Humanity)
# © 2025 SmileStory Inc. / WIA
################################################################################

set -e

# Colors
AMBER='\033[38;5;214m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${AMBER}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║    WIA-HERITAGE-009: Cultural Repatriation Data Standard  ║"
echo "║                   弘益人間 (Benefit All Humanity)               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Detect OS
OS="$(uname -s)"
case "$OS" in
    Linux*)  PLATFORM="linux" ;;
    Darwin*) PLATFORM="macos" ;;
    *)       echo -e "${RED}Unsupported OS: $OS${NC}"; exit 1 ;;
esac

echo -e "${GREEN}Installing WIA-HERITAGE-009 CLI...${NC}"

# Install CLI tool
INSTALL_DIR="/usr/local/bin"
CLI_SCRIPT="wia-heritage-009"

# Download CLI
if command -v curl &> /dev/null; then
    curl -fsSL "https://raw.githubusercontent.com/WIA-Official/wia-standards/main/standards/WIA-HERITAGE-009-cultural-repatriation/cli/${CLI_SCRIPT}.sh" -o "/tmp/${CLI_SCRIPT}"
else
    wget -q "https://raw.githubusercontent.com/WIA-Official/wia-standards/main/standards/WIA-HERITAGE-009-cultural-repatriation/cli/${CLI_SCRIPT}.sh" -O "/tmp/${CLI_SCRIPT}"
fi

# Make executable
chmod +x "/tmp/${CLI_SCRIPT}"

# Install (may require sudo)
if [[ -w "$INSTALL_DIR" ]]; then
    mv "/tmp/${CLI_SCRIPT}" "$INSTALL_DIR/"
else
    echo "Installing to $INSTALL_DIR requires sudo permissions..."
    sudo mv "/tmp/${CLI_SCRIPT}" "$INSTALL_DIR/"
fi

echo -e "${GREEN}✓ CLI tool installed successfully!${NC}"

# Install TypeScript SDK
if command -v npm &> /dev/null; then
    echo -e "${GREEN}Installing TypeScript SDK...${NC}"
    npm install -g @wia/heritage-009-cultural-repatriation
    echo -e "${GREEN}✓ TypeScript SDK installed successfully!${NC}"
else
    echo -e "${AMBER}npm not found. Skipping TypeScript SDK installation.${NC}"
    echo "To install SDK later, run: npm install -g @wia/heritage-009-cultural-repatriation"
fi

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo -e "${AMBER}Get started:${NC}"
echo "  1. Set your API key: wia-heritage-009 config --set-api-key YOUR_KEY"
echo "  2. View help: wia-heritage-009 --help"
echo "  3. Scan artifact: wia-heritage-009 scan --input photos/ --output artifact.gltf"
echo ""
echo -e "${AMBER}Learn more:${NC}"
echo "  Documentation: https://docs.wia.org/heritage/009"
echo "  GitHub: https://github.com/WIA-Official/wia-standards"
echo ""
echo -e "${AMBER}弘益人間 - Benefit All Humanity${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
