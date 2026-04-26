#!/bin/bash

################################################################################
# WIA-DEF-007: Laser Weapon Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Systems Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-DEF-007 CLI tool and optionally the TypeScript SDK
################################################################################

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
SLATE='\033[38;5;109m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Configuration
INSTALL_DIR="${HOME}/.local/bin"
STANDARD_NAME="WIA-DEF-007"
CLI_NAME="wia-def-007"

# Print functions
print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🔴 WIA-DEF-007: Laser Weapon Standard                ║"
    echo "║                    Installation Script                        ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_step() {
    echo -e "\n${CYAN}▶ $1${RESET}"
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
    print_step "Checking dependencies"

    local missing_deps=()

    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_warning "Missing dependencies: ${missing_deps[*]}"
        print_info "Please install: sudo apt-get install ${missing_deps[*]}"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_success "All dependencies satisfied"
    fi
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool"

    # Create install directory if it doesn't exist
    if [ ! -d "$INSTALL_DIR" ]; then
        print_info "Creating directory: $INSTALL_DIR"
        mkdir -p "$INSTALL_DIR"
    fi

    # Get the script directory
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    CLI_SOURCE="${SCRIPT_DIR}/cli/${CLI_NAME}.sh"

    if [ ! -f "$CLI_SOURCE" ]; then
        print_error "CLI source not found: $CLI_SOURCE"
        exit 1
    fi

    # Copy CLI tool
    print_info "Copying ${CLI_NAME}.sh to ${INSTALL_DIR}/${CLI_NAME}"
    cp "$CLI_SOURCE" "${INSTALL_DIR}/${CLI_NAME}"
    chmod +x "${INSTALL_DIR}/${CLI_NAME}"

    print_success "CLI tool installed to ${INSTALL_DIR}/${CLI_NAME}"
}

# Check PATH
check_path() {
    print_step "Checking PATH configuration"

    if [[ ":$PATH:" == *":${INSTALL_DIR}:"* ]]; then
        print_success "$INSTALL_DIR is in PATH"
    else
        print_warning "$INSTALL_DIR is not in PATH"
        print_info "Add the following to your ~/.bashrc or ~/.zshrc:"
        echo ""
        echo -e "${CYAN}    export PATH=\"\$PATH:${INSTALL_DIR}\"${RESET}"
        echo ""

        # Offer to add to shell rc
        read -p "Add to ~/.bashrc now? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "" >> ~/.bashrc
            echo "# WIA Standards CLI tools" >> ~/.bashrc
            echo "export PATH=\"\$PATH:${INSTALL_DIR}\"" >> ~/.bashrc
            print_success "Added to ~/.bashrc (restart shell or run: source ~/.bashrc)"
        fi
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_step "Installing TypeScript SDK"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    TS_DIR="${SCRIPT_DIR}/api/typescript"

    if [ ! -d "$TS_DIR" ]; then
        print_error "TypeScript SDK not found: $TS_DIR"
        return 1
    fi

    cd "$TS_DIR"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    print_info "Installing npm dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK built successfully"
    print_info "To use in your project: npm install ${TS_DIR}"
}

# Verify installation
verify_installation() {
    print_step "Verifying installation"

    if command -v "$CLI_NAME" &> /dev/null; then
        print_success "CLI tool is accessible"
        print_info "Testing CLI tool..."
        "$CLI_NAME" version | head -5
    else
        print_warning "CLI tool not immediately accessible"
        print_info "You may need to restart your shell or run: source ~/.bashrc"
        print_info "Or run directly: ${INSTALL_DIR}/${CLI_NAME}"
    fi
}

# Show usage examples
show_examples() {
    print_step "Usage Examples"

    echo ""
    echo -e "${CYAN}Calculate beam parameters:${RESET}"
    echo -e "${GRAY}  ${CLI_NAME} calc-beam --power 100000 --range 2000${RESET}"
    echo ""
    echo -e "${CYAN}Validate engagement:${RESET}"
    echo -e "${GRAY}  ${CLI_NAME} validate --target mortar --range 2000 --weather clear${RESET}"
    echo ""
    echo -e "${CYAN}Simulate atmospheric propagation:${RESET}"
    echo -e "${GRAY}  ${CLI_NAME} simulate-atmosphere --range 5000 --humidity 70${RESET}"
    echo ""
    echo -e "${CYAN}Generate thermal plan:${RESET}"
    echo -e "${GRAY}  ${CLI_NAME} thermal-plan --power 150000 --duty-cycle 0.3${RESET}"
    echo ""
    echo -e "${CYAN}Show help:${RESET}"
    echo -e "${GRAY}  ${CLI_NAME} help${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    echo -e "${GRAY}This script will install the WIA-DEF-007 Laser Weapon Standard tools${RESET}"
    echo -e "${GRAY}Installation directory: ${INSTALL_DIR}${RESET}"
    echo ""

    read -p "Continue with installation? (Y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Nn]$ ]]; then
        echo "Installation cancelled"
        exit 0
    fi

    # Run installation steps
    check_dependencies
    install_cli
    check_path

    # Optional TypeScript SDK
    echo ""
    read -p "Install TypeScript SDK? (requires Node.js) (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_typescript
    fi

    verify_installation
    show_examples

    echo ""
    print_success "Installation complete!"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Run main installation
main "$@"
