#!/bin/bash

################################################################################
# WIA-COMP-016: Software License - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-COMP-016 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       📜 WIA-COMP-016: Software License Installer             ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${RESET}"
}

print_error() {
    echo -e "${RED}✗ $1${RESET}"
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-comp-016.sh"
    local install_dir="/usr/local/bin"

    chmod +x "$cli_path"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-comp-016"
        print_success "CLI installed to $install_dir/wia-comp-016"
    else
        sudo cp "$cli_path" "$install_dir/wia-comp-016"
        print_success "CLI installed (with sudo)"
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if command -v npm &> /dev/null; then
        cd "$ts_dir" && npm install && npm run build
        print_success "TypeScript SDK built"
    else
        print_warning "npm not found, skipping TypeScript SDK"
    fi

    cd "$SCRIPT_DIR"
}

# Main
print_header
install_cli
install_typescript

print_section "Installation Complete!"
echo ""
print_success "WIA-COMP-016 Software License standard installed!"
echo ""
echo -e "${CYAN}  wia-comp-016 help${RESET}               - Show help"
echo ""
echo -e "${BLUE}弘익인간 (Benefit All Humanity)${RESET}"
echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
echo ""
