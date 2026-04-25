#!/bin/bash

################################################################################
# WIA-AUTO-023: Vehicle Cybersecurity - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Automotive Security Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-AUTO-023 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${ORANGE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    🛡️  WIA-AUTO-023: Vehicle Cybersecurity Installer         ║"
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

    # Check for basic tools
    if command -v awk &> /dev/null; then
        print_success "awk: installed"
    else
        print_warning "awk not found (optional)"
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

    # Check openssl (for security operations)
    if command -v openssl &> /dev/null; then
        print_success "OpenSSL: $(openssl version)"
    else
        print_warning "OpenSSL not found (recommended for security operations)"
    fi

    # Check for CAN utilities (optional)
    if command -v candump &> /dev/null; then
        print_success "CAN utilities: installed"
    else
        print_info "CAN utilities not found (optional, for CAN bus monitoring)"
        print_info "Install with: apt-get install can-utils (Debian/Ubuntu)"
    fi

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some required prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-auto-023.sh"
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
        cp "$cli_path" "$install_dir/wia-auto-023"
        print_success "Installed CLI to $install_dir/wia-auto-023"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-auto-023"
        print_success "Installed CLI to $install_dir/wia-auto-023 (with sudo)"
    fi

    # Verify installation
    if command -v wia-auto-023 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-auto-023'"
        print_info "Try: wia-auto-023 --help"
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

    local docs_dir="$HOME/.wia/auto-023/docs"

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

# Setup security configuration
setup_security() {
    print_section "Setting Up Security Configuration"

    local config_dir="$HOME/.wia/auto-023"
    mkdir -p "$config_dir"

    # Create default configuration
    cat > "$config_dir/config.json" << EOF
{
  "version": "1.0.0",
  "security": {
    "ids_enabled": true,
    "auto_update_signatures": true,
    "log_level": "info",
    "monitoring": {
      "can": true,
      "ethernet": true,
      "v2x": false,
      "obd": true
    }
  },
  "compliance": {
    "standards": ["ISO21434", "UNECE_WP29"],
    "auto_check": false
  },
  "reporting": {
    "backend_enabled": false,
    "local_storage": true,
    "retention_days": 90
  }
}
EOF

    print_success "Created configuration file: $config_dir/config.json"

    # Create logs directory
    mkdir -p "$config_dir/logs"
    print_success "Created logs directory: $config_dir/logs"

    # Create reports directory
    mkdir -p "$config_dir/reports"
    print_success "Created reports directory: $config_dir/reports"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-auto-023 &> /dev/null; then
        print_info "Testing CLI tool..."
        wia-auto-023 version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
    fi

    # Test TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK build verified"
    fi

    # Check security tools
    if command -v openssl &> /dev/null; then
        print_success "Cryptographic tools available"
    fi
}

# Show post-installation message
show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}The WIA-AUTO-023 standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-auto-023 scan${RESET}              - Scan vehicle for vulnerabilities"
    echo -e "${CYAN}  wia-auto-023 monitor-can${RESET}       - Monitor CAN bus traffic"
    echo -e "${CYAN}  wia-auto-023 validate-ota${RESET}      - Validate OTA update package"
    echo -e "${CYAN}  wia-auto-023 report${RESET}            - Generate security report"
    echo -e "${CYAN}  wia-auto-023 pentest${RESET}           - Run penetration tests"
    echo -e "${CYAN}  wia-auto-023 compliance${RESET}        - Check ISO 21434 compliance"
    echo -e "${CYAN}  wia-auto-023 help${RESET}              - Show help message"
    echo ""
    echo "Quick start:"
    echo -e "${GRAY}  # Scan a vehicle${RESET}"
    echo -e "${CYAN}  wia-auto-023 scan --vin WBA12345678901234${RESET}"
    echo ""
    echo -e "${GRAY}  # Monitor CAN bus for 5 minutes${RESET}"
    echo -e "${CYAN}  wia-auto-023 monitor-can --interface can0 --duration 300${RESET}"
    echo ""
    echo -e "${GRAY}  # Validate OTA update package${RESET}"
    echo -e "${CYAN}  wia-auto-023 validate-ota --package update.bin${RESET}"
    echo ""
    echo -e "${GRAY}  # Check ISO 21434 compliance${RESET}"
    echo -e "${CYAN}  wia-auto-023 compliance --standard ISO21434${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { VehicleSecuritySDK } from '@wia/auto-023';${RESET}"
    echo ""
    echo "Configuration:"
    echo -e "${GRAY}  $HOME/.wia/auto-023/config.json${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/auto-023/docs/${RESET}"
    echo ""
    echo "Security Features:"
    echo -e "${GREEN}  ✓${RESET} ECU Security (Secure Boot, HSM, Code Signing)"
    echo -e "${GREEN}  ✓${RESET} Network Security (CAN, Ethernet, V2X)"
    echo -e "${GREEN}  ✓${RESET} OTA Update Security"
    echo -e "${GREEN}  ✓${RESET} Intrusion Detection System"
    echo -e "${GREEN}  ✓${RESET} ISO/SAE 21434 Compliance"
    echo -e "${GREEN}  ✓${RESET} UNECE WP.29 Support"
    echo ""
    echo -e "${ORANGE}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Check if user wants to skip certain steps
    SKIP_CLI=false
    SKIP_TS=false
    SKIP_SECURITY=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-cli) SKIP_CLI=true; shift ;;
            --skip-typescript) SKIP_TS=true; shift ;;
            --skip-security) SKIP_SECURITY=true; shift ;;
            --help)
                echo "Usage: ./install.sh [options]"
                echo ""
                echo "Options:"
                echo "  --skip-cli          Skip CLI installation"
                echo "  --skip-typescript   Skip TypeScript SDK installation"
                echo "  --skip-security     Skip security configuration"
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

    if [ "$SKIP_SECURITY" = false ]; then
        setup_security
    else
        print_warning "Skipping security configuration"
    fi

    install_docs
    run_tests
    show_completion
}

# Run main
main "$@"
