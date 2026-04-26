#!/usr/bin/env bash

################################################################################
# WIA-AUG-009: Bionic Ear Standard - Installation Script
#
# This script installs the WIA-AUG-009 standard components including:
# - CLI tool (wia-aug-009)
# - TypeScript SDK
# - Documentation
#
# Version: 1.0.0
# License: MIT
# Author: WIA Human Augmentation Auditory Bionics Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -euo pipefail

# Colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# Paths
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CLI_SOURCE="${SCRIPT_DIR}/cli/wia-aug-009.sh"
readonly INSTALL_DIR="${HOME}/.local/bin"

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}  ${BLUE}WIA-AUG-009: Bionic Ear Standard - Installation${NC}       ${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

################################################################################
# Installation Steps
################################################################################

install_cli() {
    print_info "Installing CLI tool..."

    # Create install directory if it doesn't exist
    if [[ ! -d "$INSTALL_DIR" ]]; then
        mkdir -p "$INSTALL_DIR"
        print_info "Created directory: $INSTALL_DIR"
    fi

    # Copy CLI script
    if [[ -f "$CLI_SOURCE" ]]; then
        cp "$CLI_SOURCE" "${INSTALL_DIR}/wia-aug-009"
        chmod +x "${INSTALL_DIR}/wia-aug-009"
        print_success "CLI tool installed to ${INSTALL_DIR}/wia-aug-009"
    else
        print_error "CLI source file not found: $CLI_SOURCE"
        return 1
    fi

    # Check if install directory is in PATH
    if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
        print_warning "Add $INSTALL_DIR to your PATH to use wia-aug-009 command"
        echo ""
        echo "Add this line to your ~/.bashrc or ~/.zshrc:"
        echo "  export PATH=\"\$PATH:$INSTALL_DIR\""
    fi
}

install_typescript_sdk() {
    print_info "Installing TypeScript SDK..."

    local ts_dir="${SCRIPT_DIR}/api/typescript"

    if [[ -d "$ts_dir" ]]; then
        cd "$ts_dir"

        # Check if npm is available
        if command -v npm &> /dev/null; then
            print_info "Installing npm dependencies..."
            npm install
            print_success "TypeScript SDK dependencies installed"

            print_info "Building TypeScript SDK..."
            npm run build
            print_success "TypeScript SDK built successfully"
        else
            print_warning "npm not found. Skipping TypeScript SDK installation."
            print_info "Install Node.js and npm to use the TypeScript SDK"
        fi

        cd "$SCRIPT_DIR"
    else
        print_error "TypeScript SDK directory not found: $ts_dir"
        return 1
    fi
}

verify_installation() {
    print_info "Verifying installation..."

    # Check CLI
    if [[ -x "${INSTALL_DIR}/wia-aug-009" ]]; then
        print_success "CLI tool is executable"

        # Test CLI
        if "${INSTALL_DIR}/wia-aug-009" version &> /dev/null; then
            print_success "CLI tool is working"
        else
            print_warning "CLI tool may have issues"
        fi
    else
        print_error "CLI tool is not executable"
        return 1
    fi
}

show_next_steps() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}Installation Complete!${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Next Steps:"
    echo ""
    echo "1. Verify installation:"
    echo "   ${INSTALL_DIR}/wia-aug-009 version"
    echo ""
    echo "2. View available commands:"
    echo "   ${INSTALL_DIR}/wia-aug-009 help"
    echo ""
    echo "3. Classify a device:"
    echo "   ${INSTALL_DIR}/wia-aug-009 classify --type cochlear --strategy ACE --electrodes 22"
    echo ""
    echo "4. Configure a processor:"
    echo "   ${INSTALL_DIR}/wia-aug-009 configure --device-id CI-001 --strategy ACE --rate 900"
    echo ""
    echo "5. Read the documentation:"
    echo "   ${SCRIPT_DIR}/README.md"
    echo "   ${SCRIPT_DIR}/spec/WIA-AUG-009-v1.0.md"
    echo ""
    echo "For TypeScript SDK usage:"
    echo "   npm install @wia/aug-009"
    echo ""
    echo "Documentation: https://wiastandards.com/aug-009"
    echo ""
    echo -e "${CYAN}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
}

################################################################################
# Main Installation
################################################################################

main() {
    print_header

    print_info "Starting WIA-AUG-009 installation..."
    echo ""

    # Install components
    install_cli || exit 1
    echo ""

    # Install TypeScript SDK (optional)
    if [[ "${SKIP_TYPESCRIPT:-false}" != "true" ]]; then
        install_typescript_sdk || print_warning "TypeScript SDK installation had issues (non-critical)"
        echo ""
    fi

    # Verify
    verify_installation || exit 1
    echo ""

    # Show next steps
    show_next_steps
}

# Run main installation
main "$@"
