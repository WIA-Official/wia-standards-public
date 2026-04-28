#!/bin/bash

################################################################################
# WIA-COMM-016: VPN Protocol - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-COMM-016 standard components:
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
    echo "║       🔐 WIA-COMM-016: VPN Protocol Installer                 ║"
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

    # Check WireGuard tools (optional)
    if command -v wg &> /dev/null; then
        print_success "WireGuard tools: installed"
    else
        print_warning "WireGuard tools not found (optional)"
        print_info "Install from: https://www.wireguard.com/install/"
    fi

    # Check OpenVPN (optional)
    if command -v openvpn &> /dev/null; then
        print_success "OpenVPN: installed"
    else
        print_warning "OpenVPN not found (optional)"
    fi

    # Check strongSwan/IPsec (optional)
    if command -v ipsec &> /dev/null; then
        print_success "IPsec tools: installed"
    else
        print_warning "IPsec tools not found (optional)"
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

    local cli_path="$SCRIPT_DIR/cli/wia-comm-016.sh"
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
        cp "$cli_path" "$install_dir/wia-comm-016"
        print_success "Installed CLI to $install_dir/wia-comm-016"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-comm-016"
        print_success "Installed CLI to $install_dir/wia-comm-016 (with sudo)"
    fi

    # Verify installation
    if command -v wia-comm-016 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-comm-016'"
        print_info "Try: wia-comm-016 --help"
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

    local docs_dir="$HOME/.wia/comm-016/docs"

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
    if command -v wia-comm-016 &> /dev/null; then
        print_info "Testing CLI tool..."
        wia-comm-016 version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
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
    echo -e "${GREEN}The WIA-COMM-016 VPN Protocol standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-comm-016 config-ipsec${RESET}       - Configure IPsec VPN"
    echo -e "${CYAN}  wia-comm-016 setup-wireguard${RESET}    - Setup WireGuard VPN"
    echo -e "${CYAN}  wia-comm-016 config-openvpn${RESET}     - Configure OpenVPN"
    echo -e "${CYAN}  wia-comm-016 validate${RESET}           - Validate VPN configuration"
    echo -e "${CYAN}  wia-comm-016 calc-overhead${RESET}      - Calculate tunnel overhead"
    echo -e "${CYAN}  wia-comm-016 generate-keys${RESET}      - Generate VPN keys"
    echo -e "${CYAN}  wia-comm-016 security-audit${RESET}     - Perform security audit"
    echo -e "${CYAN}  wia-comm-016 help${RESET}               - Show help message"
    echo ""
    echo "Quick start:"
    echo -e "${GRAY}  # Configure IPsec with IKEv2${RESET}"
    echo -e "${CYAN}  wia-comm-016 config-ipsec --protocol IKEv2 --encryption AES-256-GCM${RESET}"
    echo ""
    echo -e "${GRAY}  # Setup WireGuard VPN${RESET}"
    echo -e "${CYAN}  wia-comm-016 setup-wireguard --port 51820 --address 10.0.0.1/24${RESET}"
    echo ""
    echo -e "${GRAY}  # Generate WireGuard keys${RESET}"
    echo -e "${CYAN}  wia-comm-016 generate-keys --protocol wireguard --output keys/${RESET}"
    echo ""
    echo -e "${GRAY}  # Security audit${RESET}"
    echo -e "${CYAN}  wia-comm-016 security-audit --protocol ipsec-ikev2 --encryption AES-256-GCM${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { configureIPsec, setupWireGuard } from '@wia/comm-016';${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/comm-016/docs/${RESET}"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${RESET}"
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
