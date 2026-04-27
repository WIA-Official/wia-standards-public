#!/bin/bash

################################################################################
# WIA-IND-010: Restaurant Tech Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-010 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🍴 WIA-IND-010: Restaurant Tech Installer                ║"
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

    # Check bc (for calculations)
    if command -v bc &> /dev/null; then
        print_success "bc (calculator): installed"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Install with: apt-get install bc (Debian/Ubuntu) or brew install bc (macOS)"
        all_ok=false
    fi

    # Check Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional, for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    fi

    # Check npm (optional, for TypeScript SDK)
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

    local cli_path="$SCRIPT_DIR/cli/wia-ind-010.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    # Make executable
    chmod +x "$cli_path"
    print_success "Made CLI script executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-ind-010"
        print_success "Installed CLI to $install_dir/wia-ind-010"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-ind-010"
        print_success "Installed CLI to $install_dir/wia-ind-010 (with sudo)"
    fi

    # Verify installation
    if command -v wia-ind-010 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-ind-010'"
        print_info "Try: wia-ind-010 --help"
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
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK built successfully"
    print_info "To use the SDK globally: npm link"
    print_info "Or in your project: npm install $ts_dir"

    cd "$SCRIPT_DIR"
}

# Create symlinks for documentation
install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/ind-010/docs"

    # Create directory
    mkdir -p "$docs_dir"

    # Copy documentation
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

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-ind-010 &> /dev/null; then
        print_info "Testing CLI tool..."
        wia-ind-010 version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
    fi

    # Test TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK build verified"
    fi
}

# Show post-installation message
show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}The WIA-IND-010 standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-ind-010 pos total${RESET}        - Calculate order total with tax and tip"
    echo -e "${CYAN}  wia-ind-010 turnover${RESET}         - Calculate table turnover rate"
    echo -e "${CYAN}  wia-ind-010 revpash${RESET}          - Calculate Revenue Per Available Seat Hour"
    echo -e "${CYAN}  wia-ind-010 food-cost${RESET}        - Calculate food cost percentage"
    echo -e "${CYAN}  wia-ind-010 labor-cost${RESET}       - Calculate labor cost percentage"
    echo -e "${CYAN}  wia-ind-010 prime-cost${RESET}       - Calculate prime cost (food + labor)"
    echo -e "${CYAN}  wia-ind-010 menu-profit${RESET}      - Calculate menu item profitability"
    echo -e "${CYAN}  wia-ind-010 avg-check${RESET}        - Calculate average check size"
    echo -e "${CYAN}  wia-ind-010 ticket-time${RESET}      - Calculate kitchen ticket time"
    echo -e "${CYAN}  wia-ind-010 schedule${RESET}         - Optimize staff schedule"
    echo -e "${CYAN}  wia-ind-010 help${RESET}             - Show help message"
    echo ""
    echo "Quick start:"
    echo -e "${GRAY}  # Calculate order total${RESET}"
    echo -e "${CYAN}  wia-ind-010 pos total --subtotal 45.50 --tax 8 --tip 18${RESET}"
    echo ""
    echo -e "${GRAY}  # Calculate table turnover${RESET}"
    echo -e "${CYAN}  wia-ind-010 turnover --parties 85 --tables 25 --hours 8${RESET}"
    echo ""
    echo -e "${GRAY}  # Calculate food cost percentage${RESET}"
    echo -e "${CYAN}  wia-ind-010 food-cost --cogs 3500 --revenue 10000${RESET}"
    echo ""
    echo -e "${GRAY}  # Calculate RevPASH${RESET}"
    echo -e "${CYAN}  wia-ind-010 revpash --revenue 12500 --seats 100 --hours 8${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { OrderType, calculateOrderTotal } from '@wia/ind-010';${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/ind-010/docs/${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Check if user wants to skip certain steps
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
                echo ""
                exit 0
                ;;
            *) shift ;;
        esac
    done

    # Run installation steps
    check_prerequisites

    if [ "$SKIP_CLI" = false ]; then
        install_cli
    else
        print_warning "Skipping CLI installation"
    fi

    if [ "$SKIP_TS" = false ]; then
        install_typescript
    else
        print_warning "Skipping TypeScript SDK installation"
    fi

    install_docs
    run_tests
    show_completion
}

# Run main
main "$@"
