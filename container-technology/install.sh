#!/bin/bash

################################################################################
# WIA-COMP-006: Container Technology - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-COMP-006 standard components:
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
    echo "║     🐳 WIA-COMP-006: Container Technology Installer           ║"
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

# Check prerequisites
check_prerequisites() {
    print_section "Checking Prerequisites"

    local all_ok=true

    # Check bash
    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1)"
    else
        print_error "Bash not found"
        all_ok=false
    fi

    # Check Node.js (optional)
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional, for TypeScript SDK)"
    fi

    # Check npm (optional)
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional, for TypeScript SDK)"
    fi

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-comp-006.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI script executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-comp-006"
        print_success "Installed CLI to $install_dir/wia-comp-006"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-comp-006"
        print_success "Installed CLI to $install_dir/wia-comp-006 (with sudo)"
    fi

    if command -v wia-comp-006 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-comp-006'"
        print_info "Try: wia-comp-006 --help"
    else
        print_warning "CLI installed but not in PATH. You may need to restart your terminal."
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found at $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK built successfully"
    print_info "To use: npm install @wia/comp-006"

    cd "$SCRIPT_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/comp-006/docs"

    mkdir -p "$docs_dir"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "Installed README.md"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Installed specification files"
    fi

    print_info "Documentation available at: $docs_dir"
}

# Show completion message
show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}The WIA-COMP-006 Container Technology standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-comp-006 run${RESET}             - Run a container"
    echo -e "${CYAN}  wia-comp-006 build${RESET}           - Build an image"
    echo -e "${CYAN}  wia-comp-006 ps${RESET}              - List containers"
    echo -e "${CYAN}  wia-comp-006 logs${RESET}            - Show container logs"
    echo -e "${CYAN}  wia-comp-006 stats${RESET}           - Show container stats"
    echo -e "${CYAN}  wia-comp-006 network create${RESET}  - Create a network"
    echo -e "${CYAN}  wia-comp-006 volume create${RESET}   - Create a volume"
    echo -e "${CYAN}  wia-comp-006 help${RESET}            - Show help message"
    echo ""
    echo "Quick start:"
    echo -e "${CYAN}  wia-comp-006 run --image nginx:latest --port 8080:80${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { runContainer } from '@wia/comp-006';${RESET}"
    echo ""
    echo -e "${BLUE}弘익인간 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    SKIP_CLI=false
    SKIP_TS=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-cli) SKIP_CLI=true; shift ;;
            --skip-typescript) SKIP_TS=true; shift ;;
            --help)
                echo "Usage: ./install.sh [options]"
                echo ""
                echo "Options:"
                echo "  --skip-cli          Skip CLI installation"
                echo "  --skip-typescript   Skip TypeScript SDK installation"
                echo "  --help              Show this help message"
                exit 0
                ;;
            *) shift ;;
        esac
    done

    check_prerequisites

    if [ "$SKIP_CLI" = false ]; then
        install_cli
    fi

    if [ "$SKIP_TS" = false ]; then
        install_typescript
    fi

    install_docs
    show_completion
}

main "$@"
