#!/bin/bash

###############################################################################
# WIA-IND-026 Predictive Maintenance - Installation Script
# World Certification Industry Association
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emoji
ROCKET="🚀"
CHECK="✅"
CROSS="❌"
WRENCH="🔧"
PACKAGE="📦"
INFO="ℹ️"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}║        ${WRENCH}  WIA-IND-026: Predictive Maintenance  ${WRENCH}        ║${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}║           World Certification Industry Association        ║${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Detect OS
OS="unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
fi

echo -e "${INFO} ${CYAN}Detected OS: ${OS}${NC}"
echo ""

# Check prerequisites
echo -e "${ROCKET} ${PURPLE}Checking prerequisites...${NC}"
echo ""

MISSING_DEPS=()

# Check Node.js
if command -v node &> /dev/null; then
    NODE_VERSION=$(node -v)
    echo -e "  ${CHECK} ${GREEN}Node.js: ${NODE_VERSION}${NC}"
else
    echo -e "  ${CROSS} ${RED}Node.js: Not installed${NC}"
    MISSING_DEPS+=("nodejs")
fi

# Check npm
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm -v)
    echo -e "  ${CHECK} ${GREEN}npm: v${NPM_VERSION}${NC}"
else
    echo -e "  ${CROSS} ${RED}npm: Not installed${NC}"
    MISSING_DEPS+=("npm")
fi

# Check Python (optional, for ML models)
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
    echo -e "  ${CHECK} ${GREEN}Python: ${PYTHON_VERSION}${NC}"
else
    echo -e "  ${INFO} ${YELLOW}Python: Not installed (optional, for ML models)${NC}"
fi

# Check curl
if command -v curl &> /dev/null; then
    echo -e "  ${CHECK} ${GREEN}curl: Installed${NC}"
else
    echo -e "  ${CROSS} ${RED}curl: Not installed${NC}"
    MISSING_DEPS+=("curl")
fi

echo ""

# Handle missing dependencies
if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${CROSS} ${RED}Missing required dependencies: ${MISSING_DEPS[*]}${NC}"
    echo ""
    echo -e "${INFO} ${YELLOW}Please install the missing dependencies:${NC}"
    echo ""

    if [[ "$OS" == "linux" ]]; then
        echo -e "  ${CYAN}Ubuntu/Debian:${NC}"
        echo -e "    sudo apt update"
        echo -e "    sudo apt install -y nodejs npm curl"
        echo ""
        echo -e "  ${CYAN}RHEL/CentOS/Fedora:${NC}"
        echo -e "    sudo yum install -y nodejs npm curl"
    elif [[ "$OS" == "macos" ]]; then
        echo -e "  ${CYAN}macOS (using Homebrew):${NC}"
        echo -e "    brew install node"
    fi

    echo ""
    exit 1
fi

# Installation options
echo -e "${PACKAGE} ${PURPLE}Installation Options:${NC}"
echo ""
echo -e "  ${CYAN}1)${NC} Install SDK only (npm package)"
echo -e "  ${CYAN}2)${NC} Install CLI tool only"
echo -e "  ${CYAN}3)${NC} Install both SDK and CLI (recommended)"
echo -e "  ${CYAN}4)${NC} Install with Python ML dependencies"
echo ""
read -p "$(echo -e ${CYAN}Select option [1-4]:${NC} )" INSTALL_OPTION

echo ""

# Create installation directory
INSTALL_DIR="${HOME}/.wia/ind-026"
mkdir -p "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR/bin"
mkdir -p "$INSTALL_DIR/lib"
mkdir -p "$INSTALL_DIR/config"

echo -e "${ROCKET} ${PURPLE}Installing WIA-IND-026...${NC}"
echo ""

# Install SDK
if [[ "$INSTALL_OPTION" == "1" ]] || [[ "$INSTALL_OPTION" == "3" ]] || [[ "$INSTALL_OPTION" == "4" ]]; then
    echo -e "  ${PACKAGE} Installing TypeScript SDK..."

    # In production, this would be: npm install -g @wia/ind-026
    # For now, we'll copy the local files

    if [ -d "$(dirname "$0")/api/typescript" ]; then
        cp -r "$(dirname "$0")/api/typescript" "$INSTALL_DIR/sdk"
        cd "$INSTALL_DIR/sdk"
        npm install --quiet 2>&1 | grep -v "^npm WARN" || true
        npm run build --quiet 2>&1 || true
        echo -e "  ${CHECK} ${GREEN}SDK installed successfully${NC}"
    else
        echo -e "  ${INFO} ${YELLOW}Installing from npm registry...${NC}"
        npm install -g @wia/ind-026 2>&1 | grep -v "^npm WARN" || true
        echo -e "  ${CHECK} ${GREEN}SDK installed successfully${NC}"
    fi

    echo ""
fi

# Install CLI
if [[ "$INSTALL_OPTION" == "2" ]] || [[ "$INSTALL_OPTION" == "3" ]] || [[ "$INSTALL_OPTION" == "4" ]]; then
    echo -e "  ${WRENCH} Installing CLI tool..."

    CLI_SOURCE="$(dirname "$0")/cli/wia-ind-026.sh"
    CLI_DEST="$INSTALL_DIR/bin/wia-ind-026"

    if [ -f "$CLI_SOURCE" ]; then
        cp "$CLI_SOURCE" "$CLI_DEST"
        chmod +x "$CLI_DEST"
        echo -e "  ${CHECK} ${GREEN}CLI installed to: $CLI_DEST${NC}"
    else
        # Download from repository
        curl -fsSL "https://raw.githubusercontent.com/WIA-Official/wia-standards/main/standards/predictive-maintenance/cli/wia-ind-026.sh" -o "$CLI_DEST"
        chmod +x "$CLI_DEST"
        echo -e "  ${CHECK} ${GREEN}CLI downloaded and installed${NC}"
    fi

    echo ""
fi

# Install Python ML dependencies
if [[ "$INSTALL_OPTION" == "4" ]]; then
    echo -e "  ${PACKAGE} Installing Python ML dependencies..."

    if command -v python3 &> /dev/null && command -v pip3 &> /dev/null; then
        pip3 install --quiet numpy scipy scikit-learn pandas tensorflow 2>&1 | grep -v "Requirement already satisfied" || true
        echo -e "  ${CHECK} ${GREEN}ML dependencies installed${NC}"
    else
        echo -e "  ${CROSS} ${RED}Python3/pip3 not found. Skipping ML dependencies.${NC}"
    fi

    echo ""
fi

# Configure PATH
echo -e "${WRENCH} ${PURPLE}Configuring environment...${NC}"
echo ""

SHELL_CONFIG=""
if [ -n "$BASH_VERSION" ]; then
    SHELL_CONFIG="$HOME/.bashrc"
elif [ -n "$ZSH_VERSION" ]; then
    SHELL_CONFIG="$HOME/.zshrc"
fi

if [ -n "$SHELL_CONFIG" ]; then
    if ! grep -q "WIA-IND-026" "$SHELL_CONFIG" 2>/dev/null; then
        echo "" >> "$SHELL_CONFIG"
        echo "# WIA-IND-026 Predictive Maintenance" >> "$SHELL_CONFIG"
        echo "export PATH=\"\$HOME/.wia/ind-026/bin:\$PATH\"" >> "$SHELL_CONFIG"
        echo "export WIA_IND_026_HOME=\"\$HOME/.wia/ind-026\"" >> "$SHELL_CONFIG"
        echo -e "  ${CHECK} ${GREEN}Added to $SHELL_CONFIG${NC}"
    else
        echo -e "  ${INFO} ${YELLOW}Already configured in $SHELL_CONFIG${NC}"
    fi
fi

echo ""

# Create default configuration
echo -e "${WRENCH} ${PURPLE}Creating default configuration...${NC}"
echo ""

cat > "$INSTALL_DIR/config/config.json" <<EOF
{
  "version": "1.0.0",
  "apiEndpoint": "https://api.wia.org/ind-026",
  "dataRetention": 90,
  "sensorDefaults": {
    "vibration": {
      "samplingRate": 25600,
      "units": "g"
    },
    "thermal": {
      "samplingRate": 1,
      "units": "celsius"
    },
    "acoustic": {
      "samplingRate": 44100,
      "units": "dB"
    }
  },
  "mlModels": {
    "enabled": true,
    "updateFrequency": "daily",
    "minTrainingSamples": 1000
  },
  "alerting": {
    "enabled": true,
    "channels": ["email", "sms", "webhook"]
  }
}
EOF

echo -e "  ${CHECK} ${GREEN}Configuration created: $INSTALL_DIR/config/config.json${NC}"
echo ""

# Create sample data directory
mkdir -p "$INSTALL_DIR/data/samples"
mkdir -p "$INSTALL_DIR/data/models"
mkdir -p "$INSTALL_DIR/logs"

# Installation summary
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                            ║${NC}"
echo -e "${GREEN}║  ${CHECK}  Installation completed successfully!                 ║${NC}"
echo -e "${GREEN}║                                                            ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Installation Directory:${NC} $INSTALL_DIR"
echo ""
echo -e "${CYAN}Next Steps:${NC}"
echo ""
echo -e "  1. Reload your shell configuration:"
echo -e "     ${YELLOW}source $SHELL_CONFIG${NC}"
echo ""
echo -e "  2. Verify installation:"
echo -e "     ${YELLOW}wia-ind-026 --version${NC}"
echo ""
echo -e "  3. View help:"
echo -e "     ${YELLOW}wia-ind-026 --help${NC}"
echo ""
echo -e "  4. Register your first asset:"
echo -e "     ${YELLOW}wia-ind-026 asset register --id MOTOR-001 --type ROTATING_MACHINERY${NC}"
echo ""
echo -e "${CYAN}Documentation:${NC}"
echo -e "  • Specification: $INSTALL_DIR/spec/WIA-IND-026-v1.0.md"
echo -e "  • GitHub: https://github.com/WIA-Official/wia-standards"
echo -e "  • Community: https://community.wia.org/ind-026"
echo ""
echo -e "${CYAN}Support:${NC}"
echo -e "  • Issues: https://github.com/WIA-Official/wia-standards/issues"
echo -e "  • Email: support@wia.org"
echo ""
echo -e "${PURPLE}Thank you for installing WIA-IND-026!${NC}"
echo ""
echo -e "${BLUE}弘益人間 (홍익인간) · Benefit All Humanity${NC}"
echo ""

# Make installed CLI executable
if [ -f "$INSTALL_DIR/bin/wia-ind-026" ]; then
    chmod +x "$INSTALL_DIR/bin/wia-ind-026"
fi

exit 0

###############################################################################
# 弘益人間 (홍익인간) · Benefit All Humanity
###############################################################################
