#!/bin/bash

################################################################################
# WIA-IND-007: Food Tech Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Food Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-007 Food Tech Standard CLI tool and
# TypeScript SDK on your system.
################################################################################

set -e

# Colors
INDIGO='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

# Configuration
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"
STANDARD_NAME="WIA-IND-007"
CLI_NAME="wia-ind-007"
VERSION="1.0.0"

# Print functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🍽️  WIA-IND-007 Food Tech Standard Installer           ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
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
    echo -e "  $1"
}

# Check if running with sufficient privileges
check_privileges() {
    if [ "$EUID" -ne 0 ] && [ ! -w "$INSTALL_DIR" ]; then
        print_warning "Installation to $INSTALL_DIR requires sudo privileges"
        print_info "You may be prompted for your password"
        USE_SUDO=1
    else
        USE_SUDO=0
    fi
}

# Check dependencies
check_dependencies() {
    echo ""
    print_info "Checking dependencies..."

    local missing_deps=0

    # Check for bash
    if command -v bash &> /dev/null; then
        print_success "bash: $(bash --version | head -n1)"
    else
        print_error "bash: Not found"
        missing_deps=1
    fi

    # Check for bc (required for calculations)
    if command -v bc &> /dev/null; then
        print_success "bc: $(bc --version | head -n1)"
    else
        print_error "bc: Not found (required for calculations)"
        missing_deps=1
    fi

    # Check for git (optional)
    if command -v git &> /dev/null; then
        print_success "git: $(git --version)"
    else
        print_warning "git: Not found (optional, for version control)"
    fi

    # Check for Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "node: $(node --version)"
        HAS_NODE=1
    else
        print_warning "node: Not found (optional, for TypeScript SDK)"
        HAS_NODE=0
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
        HAS_NPM=1
    else
        print_warning "npm: Not found (optional, for TypeScript SDK)"
        HAS_NPM=0
    fi

    if [ $missing_deps -eq 1 ]; then
        echo ""
        print_error "Missing required dependencies"
        print_info "Please install missing dependencies and try again"
        print_info ""
        print_info "On Ubuntu/Debian: sudo apt-get install bc"
        print_info "On macOS: brew install bc"
        print_info "On CentOS/RHEL: sudo yum install bc"
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    echo ""
    print_info "Installing CLI tool..."

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local cli_source="$script_dir/cli/$CLI_NAME.sh"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source file not found: $cli_source"
        exit 1
    fi

    # Make executable
    chmod +x "$cli_source"

    # Copy to install directory
    if [ $USE_SUDO -eq 1 ]; then
        sudo cp "$cli_source" "$INSTALL_DIR/$CLI_NAME"
        sudo chmod +x "$INSTALL_DIR/$CLI_NAME"
    else
        cp "$cli_source" "$INSTALL_DIR/$CLI_NAME"
        chmod +x "$INSTALL_DIR/$CLI_NAME"
    fi

    print_success "CLI tool installed to $INSTALL_DIR/$CLI_NAME"
}

# Install TypeScript SDK
install_typescript_sdk() {
    if [ $HAS_NODE -eq 0 ] || [ $HAS_NPM -eq 0 ]; then
        print_warning "Skipping TypeScript SDK installation (Node.js/npm not found)"
        return
    fi

    echo ""
    print_info "Installing TypeScript SDK..."

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local api_dir="$script_dir/api/typescript"

    if [ ! -d "$api_dir" ]; then
        print_error "TypeScript SDK directory not found: $api_dir"
        return
    fi

    cd "$api_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install --silent

    # Build TypeScript
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript..."
        npm run build --silent 2>&1 | grep -v "npm WARN" || true
        print_success "TypeScript SDK built successfully"
    else
        print_warning "tsconfig.json not found, skipping build"
    fi

    # Optional: Link for local development
    read -p "Link SDK for local development (npm link)? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if [ $USE_SUDO -eq 1 ]; then
            sudo npm link
        else
            npm link
        fi
        print_success "TypeScript SDK linked globally"
        print_info "You can now use: import { ... } from '@wia/ind-007'"
    fi

    cd - > /dev/null
}

# Verify installation
verify_installation() {
    echo ""
    print_info "Verifying installation..."

    if command -v "$CLI_NAME" &> /dev/null; then
        print_success "CLI tool is accessible"
        print_info "Version: $("$CLI_NAME" --version 2>&1 | head -n1)"
    else
        print_error "CLI tool not found in PATH"
        print_info "You may need to add $INSTALL_DIR to your PATH"
        print_info "Add this to your ~/.bashrc or ~/.zshrc:"
        print_info "  export PATH=\"\$PATH:$INSTALL_DIR\""
    fi
}

# Show usage examples
show_examples() {
    echo ""
    echo -e "${INDIGO}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${GREEN}Installation Complete!${RESET}"
    echo -e "${INDIGO}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo ""
    echo "Try these commands to get started:"
    echo ""
    echo "  # Show help"
    echo "  $CLI_NAME --help"
    echo ""
    echo "  # Calculate protein efficiency"
    echo "  $CLI_NAME calc-protein cultured-beef 2.5 1.2"
    echo ""
    echo "  # Analyze nutritional density"
    echo "  $CLI_NAME analyze-nutrition tofu 100"
    echo ""
    echo "  # Assess food safety"
    echo "  $CLI_NAME assess-safety 95 98 92"
    echo ""
    echo "  # Predict shelf life"
    echo "  $CLI_NAME predict-shelf yogurt 4 14"
    echo ""
    echo "  # Optimize fermentation"
    echo "  $CLI_NAME optimize-ferment kombucha 25 3.5"
    echo ""
    echo "  # Calculate carbon footprint"
    echo "  $CLI_NAME calc-carbon plant-burger full"
    echo ""
    echo "  # Personalized nutrition"
    echo "  $CLI_NAME calc-macros 35 male 75 175 moderate"
    echo ""
    echo "Documentation:"
    echo "  - Full spec: ./spec/WIA-IND-007-v1.0.md"
    echo "  - README: ./README.md"
    echo "  - TypeScript API: ./api/typescript/src/types.ts"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    echo ""
    print_info "Uninstalling $STANDARD_NAME..."

    if [ -f "$INSTALL_DIR/$CLI_NAME" ]; then
        if [ $USE_SUDO -eq 1 ]; then
            sudo rm "$INSTALL_DIR/$CLI_NAME"
        else
            rm "$INSTALL_DIR/$CLI_NAME"
        fi
        print_success "CLI tool removed from $INSTALL_DIR"
    else
        print_warning "CLI tool not found in $INSTALL_DIR"
    fi

    # Unlink npm package if linked
    if [ $HAS_NPM -eq 1 ]; then
        if npm ls -g @wia/ind-007 &> /dev/null; then
            if [ $USE_SUDO -eq 1 ]; then
                sudo npm unlink -g @wia/ind-007 2>&1 | grep -v "npm WARN" || true
            else
                npm unlink -g @wia/ind-007 2>&1 | grep -v "npm WARN" || true
            fi
            print_success "TypeScript SDK unlinked"
        fi
    fi

    print_success "Uninstallation complete"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Parse arguments
    if [ "$1" = "--uninstall" ] || [ "$1" = "uninstall" ]; then
        check_privileges
        uninstall
        exit 0
    fi

    if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
        echo "Usage: ./install.sh [options]"
        echo ""
        echo "Options:"
        echo "  --uninstall    Uninstall the WIA-IND-007 standard"
        echo "  --help         Show this help message"
        echo ""
        echo "Environment variables:"
        echo "  INSTALL_DIR    Installation directory (default: /usr/local/bin)"
        echo ""
        exit 0
    fi

    check_privileges
    check_dependencies
    install_cli
    install_typescript_sdk
    verify_installation
    show_examples
}

# Run main
main "$@"
