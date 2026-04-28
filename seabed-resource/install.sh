#!/usr/bin/env bash

#######################################################################
# WIA-ENE-037: Seabed Resource Development Standard - Installation Script
#
# @version 1.0.0
# @license CC BY 4.0
# @description Automated installation script for WIA-ENE-037 standard
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#######################################################################

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_header() {
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║  WIA-ENE-037: 해저 자원 개발 표준 설치 프로그램 🌊        ║${NC}"
    echo -e "${CYAN}║  弘益人間 (홍익인간) - Benefit All Humanity               ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_step() {
    echo -e "\n${CYAN}▶ $1${NC}"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        print_success "$1 is installed"
        return 0
    else
        print_error "$1 is not installed"
        return 1
    fi
}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

print_header

# Check prerequisites
print_step "Checking prerequisites..."

MISSING_DEPS=0

if ! check_command "node"; then
    print_warning "Node.js is required for TypeScript SDK"
    MISSING_DEPS=1
fi

if ! check_command "npm"; then
    print_warning "npm is required for TypeScript SDK"
    MISSING_DEPS=1
fi

if ! check_command "jq"; then
    print_warning "jq is required for CLI tool"
    MISSING_DEPS=1
fi

if ! check_command "curl"; then
    print_error "curl is required"
    exit 1
fi

if [[ $MISSING_DEPS -eq 1 ]]; then
    echo ""
    print_warning "Some optional dependencies are missing. Installation will continue, but some features may not work."
    read -p "Continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install TypeScript SDK
if command -v npm &> /dev/null; then
    print_step "Installing TypeScript SDK..."

    cd "$SCRIPT_DIR/api/typescript"

    print_info "Running npm install..."
    npm install

    print_info "Building SDK..."
    npm run build

    print_success "TypeScript SDK installed successfully"
else
    print_warning "Skipping TypeScript SDK installation (npm not found)"
fi

# Install CLI tool
print_step "Installing CLI tool..."

CLI_SOURCE="$SCRIPT_DIR/cli/seabed-resource.sh"
CLI_TARGET="/usr/local/bin/seabed-resource"

# Make CLI executable
chmod +x "$CLI_SOURCE"
print_success "CLI script made executable"

# Create symlink
if [[ -w "/usr/local/bin" ]]; then
    ln -sf "$CLI_SOURCE" "$CLI_TARGET"
    print_success "CLI tool installed to $CLI_TARGET"
elif sudo -n true 2>/dev/null; then
    sudo ln -sf "$CLI_SOURCE" "$CLI_TARGET"
    print_success "CLI tool installed to $CLI_TARGET (with sudo)"
else
    print_warning "Cannot install CLI tool to $CLI_TARGET (permission denied)"
    print_info "You can still use the CLI tool directly:"
    print_info "  $CLI_SOURCE"
    echo ""
    read -p "Try installing with sudo? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo ln -sf "$CLI_SOURCE" "$CLI_TARGET"
        print_success "CLI tool installed to $CLI_TARGET"
    fi
fi

# Configure environment variables
print_step "Configuring environment variables..."

SHELL_RC=""
if [[ -n "${BASH_VERSION:-}" ]]; then
    SHELL_RC="$HOME/.bashrc"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
    SHELL_RC="$HOME/.zshrc"
else
    SHELL_RC="$HOME/.profile"
fi

print_info "Detected shell configuration: $SHELL_RC"

if [[ ! -f "$SHELL_RC" ]]; then
    touch "$SHELL_RC"
fi

# Check if environment variables are already set
if ! grep -q "WIA_ENE037_API_KEY" "$SHELL_RC" 2>/dev/null; then
    echo ""
    echo "# WIA-ENE-037: Seabed Resource Development Standard" >> "$SHELL_RC"
    echo "# export WIA_ENE037_API_KEY=\"your-api-key\"" >> "$SHELL_RC"
    echo "# export WIA_ENE037_OPERATOR_ID=\"your-operator-id\"" >> "$SHELL_RC"
    echo "# export WIA_ENE037_API_ENDPOINT=\"https://api.wia.org/ene-037/v1\"" >> "$SHELL_RC"
    print_success "Environment variable templates added to $SHELL_RC"
    print_info "Please edit $SHELL_RC and set your API key and operator ID"
else
    print_info "Environment variables already configured in $SHELL_RC"
fi

# Installation summary
print_step "Installation Summary"
echo ""
echo -e "${GREEN}✓ Installation completed successfully!${NC}"
echo ""
echo -e "${CYAN}What's installed:${NC}"
echo "  • TypeScript SDK: $SCRIPT_DIR/api/typescript"
echo "  • CLI Tool: $CLI_TARGET"
echo "  • Specification: $SCRIPT_DIR/spec/WIA-ENE-037-v1.0.md"
echo ""
echo -e "${CYAN}Next steps:${NC}"
echo "  1. Set your API credentials in $SHELL_RC:"
echo "     export WIA_ENE037_API_KEY=\"your-api-key\""
echo "     export WIA_ENE037_OPERATOR_ID=\"your-operator-id\""
echo ""
echo "  2. Reload your shell configuration:"
echo "     source $SHELL_RC"
echo ""
echo "  3. Test the CLI tool:"
echo "     seabed-resource help"
echo ""
echo "  4. Or use the TypeScript SDK:"
echo "     npm install @wia/ene-037"
echo ""
echo -e "${CYAN}Documentation:${NC}"
echo "  • Standard Spec: $SCRIPT_DIR/spec/WIA-ENE-037-v1.0.md"
echo "  • README: $SCRIPT_DIR/README.md"
echo "  • Website: https://wia.org/standards/ene-037"
echo ""
echo -e "${CYAN}Support:${NC}"
echo "  • Email: seabed-resource@wia.org"
echo "  • GitHub: https://github.com/WIA-Official/wia-standards"
echo ""
echo -e "${CYAN}弘益人間 (홍익인간) - Benefit All Humanity${NC} 🌊"
echo ""
