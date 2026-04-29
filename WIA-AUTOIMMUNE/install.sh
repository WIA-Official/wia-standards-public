#!/bin/bash
#
# WIA-AUTOIMMUNE Installation Script
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

# Colors
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔═══════════════════════════════════════════════════════════════════════════╗"
echo "║                      WIA-AUTOIMMUNE INSTALLER                             ║"
echo "║               Treg-Microbiome Axis Autoimmune Standard                    ║"
echo "╚═══════════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Detect OS
OS="$(uname -s)"
ARCH="$(uname -m)"

echo "Detected: $OS ($ARCH)"
echo ""

# Installation directory
INSTALL_DIR="${WIA_INSTALL_DIR:-$HOME/.wia}"
BIN_DIR="$INSTALL_DIR/bin"

# Create directories
mkdir -p "$BIN_DIR"
mkdir -p "$INSTALL_DIR/autoimmune"

# Download CLI
echo -e "${YELLOW}Installing WIA-AUTOIMMUNE CLI...${NC}"

CLI_URL="https://github.com/WIA-Official/wia-standards/raw/main/standards/WIA-AUTOIMMUNE/cli/wia-autoimmune.sh"

if command -v curl &> /dev/null; then
    curl -sL "$CLI_URL" -o "$BIN_DIR/wia-autoimmune" 2>/dev/null || cp "$(dirname "$0")/cli/wia-autoimmune.sh" "$BIN_DIR/wia-autoimmune"
elif command -v wget &> /dev/null; then
    wget -q "$CLI_URL" -O "$BIN_DIR/wia-autoimmune" 2>/dev/null || cp "$(dirname "$0")/cli/wia-autoimmune.sh" "$BIN_DIR/wia-autoimmune"
else
    # Local install
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    cp "$SCRIPT_DIR/cli/wia-autoimmune.sh" "$BIN_DIR/wia-autoimmune"
fi

chmod +x "$BIN_DIR/wia-autoimmune"

# Add to PATH
SHELL_RC=""
if [ -n "$ZSH_VERSION" ] || [ -f "$HOME/.zshrc" ]; then
    SHELL_RC="$HOME/.zshrc"
elif [ -n "$BASH_VERSION" ] || [ -f "$HOME/.bashrc" ]; then
    SHELL_RC="$HOME/.bashrc"
fi

if [ -n "$SHELL_RC" ]; then
    if ! grep -q "WIA_INSTALL_DIR" "$SHELL_RC" 2>/dev/null; then
        echo "" >> "$SHELL_RC"
        echo "# WIA Standards" >> "$SHELL_RC"
        echo "export WIA_INSTALL_DIR=\"$INSTALL_DIR\"" >> "$SHELL_RC"
        echo "export PATH=\"\$WIA_INSTALL_DIR/bin:\$PATH\"" >> "$SHELL_RC"
    fi
fi

# Install Node.js SDK (optional)
if command -v npm &> /dev/null; then
    echo -e "${YELLOW}Installing TypeScript SDK...${NC}"
    npm install -g @wia/autoimmune-sdk 2>/dev/null || echo "  (SDK available via: npm install @wia/autoimmune-sdk)"
fi

# Verify installation
echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "CLI installed to: $BIN_DIR/wia-autoimmune"
echo ""
echo "To get started:"
echo "  1. Restart your shell or run: source $SHELL_RC"
echo "  2. Configure API key: wia-autoimmune config"
echo "  3. Run: wia-autoimmune help"
echo ""
echo -e "${CYAN}弘益人間 - Benefit All Humanity${NC}"
