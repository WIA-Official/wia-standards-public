#!/bin/bash

################################################################################
# WIA-IND-002: Smart Textile Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industrial Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-002 Smart Textile Standard tools and SDK.
################################################################################

set -e

# Colors
INDIGO='\033[0;94m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Version
VERSION="1.0.0"

# Directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${HOME}/.wia/standards/smart-textile"
BIN_DIR="${HOME}/.wia/bin"

# Print functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    👕 WIA-IND-002: Smart Textile Standard Installer           ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    echo ""
}

print_step() {
    echo -e "${CYAN}▶ $1${RESET}"
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

# Check dependencies
check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for bc (calculator)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    fi

    # Check for node (optional, for TypeScript SDK)
    if ! command -v node &> /dev/null; then
        print_warning "Node.js not found (optional for TypeScript SDK)"
    else
        local node_version=$(node --version)
        print_info "Node.js version: $node_version"
    fi

    # Check for npm (optional)
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found (optional for TypeScript SDK)"
    else
        local npm_version=$(npm --version)
        print_info "npm version: $npm_version"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install the missing dependencies:"
        echo "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        echo "  macOS: brew install ${missing_deps[*]}"
        echo "  RHEL/CentOS: sudo yum install ${missing_deps[*]}"
        echo ""
        exit 1
    fi

    print_success "All required dependencies are installed"
    echo ""
}

# Create directories
create_directories() {
    print_step "Creating installation directories..."

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$BIN_DIR"
    mkdir -p "$INSTALL_DIR/cli"
    mkdir -p "$INSTALL_DIR/spec"
    mkdir -p "$INSTALL_DIR/api"

    print_success "Directories created"
    print_info "Installation directory: $INSTALL_DIR"
    print_info "Binary directory: $BIN_DIR"
    echo ""
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool..."

    # Copy CLI script
    cp "$SCRIPT_DIR/cli/wia-ind-002.sh" "$INSTALL_DIR/cli/"
    chmod +x "$INSTALL_DIR/cli/wia-ind-002.sh"

    # Create symlink in bin directory
    ln -sf "$INSTALL_DIR/cli/wia-ind-002.sh" "$BIN_DIR/wia-ind-002"

    print_success "CLI tool installed"
    print_info "Command: wia-ind-002"
    echo ""
}

# Install specification
install_spec() {
    print_step "Installing specification..."

    # Copy spec files
    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec"/* "$INSTALL_DIR/spec/"
        print_success "Specification installed"
        print_info "Location: $INSTALL_DIR/spec/"
    else
        print_warning "Specification directory not found"
    fi
    echo ""
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        echo ""
        return
    fi

    # Copy API files
    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cp -r "$SCRIPT_DIR/api/typescript" "$INSTALL_DIR/api/"

        cd "$INSTALL_DIR/api/typescript"

        print_info "Installing npm dependencies..."
        npm install --silent

        print_info "Building TypeScript SDK..."
        npm run build --silent

        print_success "TypeScript SDK installed"
        print_info "Location: $INSTALL_DIR/api/typescript/"
        print_info ""
        print_info "To use in your project:"
        print_info "  npm install $INSTALL_DIR/api/typescript"
        print_info "  # or link locally:"
        print_info "  cd $INSTALL_DIR/api/typescript && npm link"
        print_info "  cd your-project && npm link @wia/ind-002"
    else
        print_warning "TypeScript SDK directory not found"
    fi
    echo ""
}

# Setup PATH
setup_path() {
    print_step "Setting up PATH..."

    # Detect shell
    local shell_rc=""
    if [ -n "$BASH_VERSION" ]; then
        shell_rc="$HOME/.bashrc"
    elif [ -n "$ZSH_VERSION" ]; then
        shell_rc="$HOME/.zshrc"
    else
        shell_rc="$HOME/.profile"
    fi

    # Check if PATH already contains BIN_DIR
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        echo "" >> "$shell_rc"
        echo "# WIA Standards - Smart Textile" >> "$shell_rc"
        echo "export PATH=\"\$PATH:$BIN_DIR\"" >> "$shell_rc"

        print_success "PATH updated in $shell_rc"
        print_warning "Please run: source $shell_rc"
        print_info "Or restart your terminal to use 'wia-ind-002' command"
    else
        print_success "PATH already configured"
    fi
    echo ""
}

# Verify installation
verify_installation() {
    print_step "Verifying installation..."

    local errors=0

    # Check CLI tool
    if [ -x "$INSTALL_DIR/cli/wia-ind-002.sh" ]; then
        print_success "CLI tool: OK"
    else
        print_error "CLI tool: NOT FOUND"
        errors=$((errors + 1))
    fi

    # Check symlink
    if [ -L "$BIN_DIR/wia-ind-002" ]; then
        print_success "Symlink: OK"
    else
        print_error "Symlink: NOT FOUND"
        errors=$((errors + 1))
    fi

    # Check specification
    if [ -d "$INSTALL_DIR/spec" ]; then
        print_success "Specification: OK"
    else
        print_warning "Specification: NOT FOUND"
    fi

    # Check TypeScript SDK
    if [ -d "$INSTALL_DIR/api/typescript" ]; then
        print_success "TypeScript SDK: OK"
    else
        print_warning "TypeScript SDK: NOT FOUND"
    fi

    echo ""

    if [ $errors -eq 0 ]; then
        print_success "Installation verified successfully!"
    else
        print_error "Installation completed with $errors error(s)"
        return 1
    fi
}

# Print usage instructions
print_usage() {
    echo -e "${CYAN}Usage Instructions:${RESET}"
    echo ""
    echo "CLI Tool:"
    echo "  wia-ind-002 --help                     Show help"
    echo "  wia-ind-002 calc-conductivity          Calculate fabric conductivity"
    echo "  wia-ind-002 calc-sensor                Calculate sensor performance"
    echo "  wia-ind-002 calc-thermal               Calculate temperature regulation"
    echo "  wia-ind-002 calc-energy                Calculate energy harvesting"
    echo "  wia-ind-002 simulate-wash              Simulate washability"
    echo "  wia-ind-002 calc-power                 Calculate power budget"
    echo ""
    echo "TypeScript SDK:"
    echo "  import { SmartTextileSDK } from '@wia/ind-002';"
    echo ""
    echo "Documentation:"
    echo "  Specification: $INSTALL_DIR/spec/WIA-IND-002-v1.0.md"
    echo "  README: $SCRIPT_DIR/README.md"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity) - WIA Industrial Research Group${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    print_header
    print_step "Uninstalling WIA-IND-002 Smart Textile Standard..."

    # Remove installation directory
    if [ -d "$INSTALL_DIR" ]; then
        rm -rf "$INSTALL_DIR"
        print_success "Removed installation directory"
    fi

    # Remove symlink
    if [ -L "$BIN_DIR/wia-ind-002" ]; then
        rm "$BIN_DIR/wia-ind-002"
        print_success "Removed CLI symlink"
    fi

    print_success "Uninstallation complete"
    echo ""
}

# Main installation process
main() {
    # Check for uninstall flag
    if [ "$1" = "--uninstall" ] || [ "$1" = "-u" ]; then
        uninstall
        exit 0
    fi

    print_header

    check_dependencies
    create_directories
    install_cli
    install_spec
    install_typescript_sdk
    setup_path
    verify_installation

    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${GREEN}║                 Installation Complete! 🎉                     ║${RESET}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${RESET}"
    echo ""

    print_usage
}

# Run main
main "$@"
