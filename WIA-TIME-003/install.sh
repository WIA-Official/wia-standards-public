#!/bin/bash

################################################################################
# WIA-TIME-003: Quantum Time Theory - Installation Script
#
# 弘益人間 (Hongik Ingan) - Benefit All Humanity
#
# @version 1.0.0
# @license MIT
# @copyright 2025 SmileStory Inc. / WIA
################################################################################

set -e

# Colors (Violet theme #8B5CF6)
VIOLET='\033[38;5;141m'
BOLD='\033[1m'
RESET='\033[0m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'

# Configuration
INSTALL_DIR="${HOME}/.wia/time-003"
BIN_DIR="${HOME}/.local/bin"
VERSION="1.0.0"

################################################################################
# Functions
################################################################################

print_header() {
    echo -e "${VIOLET}${BOLD}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║      WIA-TIME-003: Quantum Time Theory - Installation        ║"
    echo "║                   弘益人間 - Benefit All Humanity                ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    echo ""
}

print_step() {
    echo -e "${VIOLET}${BOLD}[Step]${RESET} $1"
}

print_success() {
    echo -e "${GREEN}${BOLD}✓${RESET} $1"
}

print_warning() {
    echo -e "${YELLOW}${BOLD}⚠${RESET} $1"
}

print_error() {
    echo -e "${RED}${BOLD}✗${RESET} $1"
}

check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check Node.js
    if ! command -v node &> /dev/null; then
        missing_deps+=("node")
    else
        print_success "Node.js: $(node --version)"
    fi

    # Check npm
    if ! command -v npm &> /dev/null; then
        missing_deps+=("npm")
    else
        print_success "npm: $(npm --version)"
    fi

    # Check bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    else
        print_success "Bash: $(bash --version | head -n1)"
    fi

    # Check git
    if ! command -v git &> /dev/null; then
        print_warning "Git not found (optional)"
    else
        print_success "Git: $(git --version)"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install the missing dependencies and try again."
        exit 1
    fi

    echo ""
}

create_directories() {
    print_step "Creating directories..."

    mkdir -p "${INSTALL_DIR}"
    mkdir -p "${BIN_DIR}"

    print_success "Install directory: ${INSTALL_DIR}"
    print_success "Binary directory: ${BIN_DIR}"
    echo ""
}

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    local api_dir="$(dirname "$0")/api/typescript"

    if [ -d "${api_dir}" ]; then
        cd "${api_dir}"

        # Install dependencies
        print_success "Installing npm dependencies..."
        npm install --silent

        # Build SDK
        print_success "Building TypeScript SDK..."
        npm run build --silent

        # Copy to install directory
        cp -r dist "${INSTALL_DIR}/"
        cp package.json "${INSTALL_DIR}/"
        cp -r src "${INSTALL_DIR}/"

        print_success "TypeScript SDK installed successfully"
    else
        print_warning "TypeScript SDK directory not found, skipping..."
    fi

    echo ""
}

install_cli() {
    print_step "Installing CLI tool..."

    local cli_script="$(dirname "$0")/cli/wia-time-003.sh"

    if [ -f "${cli_script}" ]; then
        # Copy CLI script
        cp "${cli_script}" "${INSTALL_DIR}/wia-time-003"
        chmod +x "${INSTALL_DIR}/wia-time-003"

        # Create symlink in bin directory
        if [ -L "${BIN_DIR}/wia-time-003" ]; then
            rm "${BIN_DIR}/wia-time-003"
        fi
        ln -s "${INSTALL_DIR}/wia-time-003" "${BIN_DIR}/wia-time-003"

        print_success "CLI tool installed: wia-time-003"
        print_success "Symlink created: ${BIN_DIR}/wia-time-003"
    else
        print_error "CLI script not found: ${cli_script}"
        exit 1
    fi

    echo ""
}

copy_documentation() {
    print_step "Copying documentation..."

    local base_dir="$(dirname "$0")"

    # Copy README
    if [ -f "${base_dir}/README.md" ]; then
        cp "${base_dir}/README.md" "${INSTALL_DIR}/"
        print_success "README.md copied"
    fi

    # Copy spec
    if [ -d "${base_dir}/spec" ]; then
        cp -r "${base_dir}/spec" "${INSTALL_DIR}/"
        print_success "Specification copied"
    fi

    echo ""
}

update_path() {
    print_step "Checking PATH configuration..."

    if [[ ":$PATH:" != *":${BIN_DIR}:"* ]]; then
        print_warning "${BIN_DIR} is not in your PATH"
        echo ""
        echo "Add this to your ~/.bashrc or ~/.zshrc:"
        echo -e "${VIOLET}export PATH=\"\$PATH:${BIN_DIR}\"${RESET}"
        echo ""
    else
        print_success "${BIN_DIR} is in your PATH"
    fi

    echo ""
}

print_completion() {
    echo -e "${GREEN}${BOLD}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                   Installation Complete! 🎉                   ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    echo ""
    echo -e "${BOLD}Installation Details:${RESET}"
    echo "  Install Directory: ${INSTALL_DIR}"
    echo "  CLI Command: wia-time-003"
    echo "  Version: ${VERSION}"
    echo ""
    echo -e "${BOLD}Quick Start:${RESET}"
    echo "  wia-time-003 help                    # Show help"
    echo "  wia-time-003 create-state --count 50 # Create quantum state"
    echo "  wia-time-003 tunnel --particle electron --energy 1.0"
    echo ""
    echo -e "${BOLD}TypeScript SDK:${RESET}"
    echo "  import { createQuantumTimeState } from '@wia/time-003';"
    echo ""
    echo -e "${BOLD}Documentation:${RESET}"
    echo "  README: ${INSTALL_DIR}/README.md"
    echo "  Spec: ${INSTALL_DIR}/spec/WIA-TIME-003-v1.0.md"
    echo ""
    echo -e "${VIOLET}${BOLD}弘益人間 (Hongik Ingan) - Benefit All Humanity${RESET}"
    echo ""
}

print_uninstall_info() {
    echo -e "${BLUE}${BOLD}To uninstall:${RESET}"
    echo "  rm -rf ${INSTALL_DIR}"
    echo "  rm ${BIN_DIR}/wia-time-003"
    echo ""
}

################################################################################
# Main Installation
################################################################################

main() {
    print_header

    # Check if running as root
    if [ "$EUID" -eq 0 ]; then
        print_error "Do not run this script as root!"
        exit 1
    fi

    # Run installation steps
    check_dependencies
    create_directories
    install_typescript_sdk
    install_cli
    copy_documentation
    update_path

    # Print completion message
    print_completion
    print_uninstall_info
}

# Handle script arguments
case "${1:-install}" in
    install)
        main
        ;;
    uninstall)
        print_header
        print_step "Uninstalling WIA-TIME-003..."
        rm -rf "${INSTALL_DIR}"
        rm -f "${BIN_DIR}/wia-time-003"
        print_success "Uninstalled successfully"
        echo ""
        ;;
    --help|-h)
        echo "Usage: $0 [install|uninstall]"
        echo ""
        echo "Commands:"
        echo "  install     Install WIA-TIME-003 (default)"
        echo "  uninstall   Remove WIA-TIME-003"
        echo ""
        ;;
    *)
        print_error "Unknown command: $1"
        echo "Use --help for usage information"
        exit 1
        ;;
esac
