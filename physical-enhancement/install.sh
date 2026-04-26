#!/bin/bash

################################################################################
# WIA-AUG-006: Physical Enhancement - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Physical Augmentation Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-AUG-006 Physical Enhancement standard tools
################################################################################

set -e

VERSION="1.0.0"
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"
CONFIG_DIR="$HOME/.wia/aug-006"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${CYAN}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  WIA-AUG-006: Physical Enhancement Installation"
    echo "  Version: ${VERSION}"
    echo "  弘益人間 (Benefit All Humanity)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${NC}"
}

error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
    exit 1
}

warning() {
    echo -e "${YELLOW}WARNING: $1${NC}"
}

success() {
    echo -e "${GREEN}✓ $1${NC}"
}

info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

step() {
    echo -e "${CYAN}▸ $1${NC}"
}

################################################################################
# Installation Functions
################################################################################

check_prerequisites() {
    step "Checking prerequisites..."

    # Check for bash
    if ! command -v bash &> /dev/null; then
        error "Bash is required but not installed"
    fi

    # Check for bc (for calculations)
    if ! command -v bc &> /dev/null; then
        warning "bc is not installed. Some calculations may not work. Install with: apt-get install bc"
    fi

    # Check Node.js for TypeScript SDK
    if command -v node &> /dev/null; then
        local node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [ "$node_version" -ge 16 ]; then
            success "Node.js $(node --version) detected"
        else
            warning "Node.js version 16+ recommended for TypeScript SDK (current: $(node --version))"
        fi
    else
        info "Node.js not found. TypeScript SDK will not be available."
    fi

    success "Prerequisites check completed"
}

create_directories() {
    step "Creating configuration directories..."

    mkdir -p "$CONFIG_DIR"
    mkdir -p "$CONFIG_DIR/profiles"
    mkdir -p "$CONFIG_DIR/sessions"
    mkdir -p "$CONFIG_DIR/reports"

    success "Configuration directories created at $CONFIG_DIR"
}

install_cli() {
    step "Installing CLI tool..."

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local cli_script="$script_dir/cli/wia-aug-006.sh"

    if [ ! -f "$cli_script" ]; then
        error "CLI script not found at $cli_script"
    fi

    # Check if we need sudo
    if [ -w "$INSTALL_DIR" ]; then
        cp "$cli_script" "$INSTALL_DIR/wia-aug-006"
        chmod +x "$INSTALL_DIR/wia-aug-006"
    else
        info "Installing to $INSTALL_DIR requires sudo privileges"
        sudo cp "$cli_script" "$INSTALL_DIR/wia-aug-006"
        sudo chmod +x "$INSTALL_DIR/wia-aug-006"
    fi

    success "CLI tool installed to $INSTALL_DIR/wia-aug-006"
}

install_typescript_sdk() {
    step "Installing TypeScript SDK..."

    if ! command -v npm &> /dev/null; then
        warning "npm not found. Skipping TypeScript SDK installation."
        return
    fi

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local sdk_dir="$script_dir/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        warning "TypeScript SDK directory not found. Skipping."
        return
    fi

    cd "$sdk_dir"

    info "Installing dependencies..."
    npm install

    info "Building TypeScript SDK..."
    npm run build

    success "TypeScript SDK built successfully"
    info "To use in your project: npm install $sdk_dir"

    cd - > /dev/null
}

create_config_file() {
    step "Creating default configuration..."

    cat > "$CONFIG_DIR/config.json" << EOF
{
  "version": "$VERSION",
  "defaultSafetyLevel": "standard",
  "defaultEnhancementFactor": 2.0,
  "alertsEnabled": true,
  "loggingEnabled": true,
  "maxSessionDuration": 7200,
  "recoveryReminders": true,
  "dataRetentionDays": 90
}
EOF

    success "Default configuration created at $CONFIG_DIR/config.json"
}

create_sample_profile() {
    step "Creating sample user profile..."

    cat > "$CONFIG_DIR/profiles/sample.json" << EOF
{
  "id": "sample-user",
  "name": "Sample User",
  "age": 30,
  "weight": 75,
  "height": 175,
  "fitnessLevel": "moderate",
  "medicalHistory": [],
  "certificationLevel": "Level2",
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    success "Sample profile created at $CONFIG_DIR/profiles/sample.json"
}

verify_installation() {
    step "Verifying installation..."

    if command -v wia-aug-006 &> /dev/null; then
        success "CLI tool is accessible in PATH"
        wia-aug-006 version
    else
        warning "CLI tool not found in PATH. You may need to add $INSTALL_DIR to your PATH"
        echo "Add to your ~/.bashrc or ~/.zshrc:"
        echo "  export PATH=\"$INSTALL_DIR:\$PATH\""
    fi
}

print_next_steps() {
    echo
    echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}Installation Complete!${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
    echo
    echo "Next steps:"
    echo
    echo "1. Verify installation:"
    echo "   $ wia-aug-006 version"
    echo
    echo "2. Assess your physical baseline:"
    echo "   $ wia-aug-006 assess-baseline --age 30 --weight 75 --height 175 --fitness moderate"
    echo
    echo "3. Configure enhancement:"
    echo "   $ wia-aug-006 enhance --domain strength --tech exoskeleton --factor 2.5 --baseline 1500"
    echo
    echo "4. Monitor during activity:"
    echo "   $ wia-aug-006 monitor-load --current 100 --max 250 --duration 300"
    echo
    echo "5. Check fatigue levels:"
    echo "   $ wia-aug-006 check-fatigue --session 3600 --intensity 75 --hr 160 --max-hr 185"
    echo
    echo "6. View help for all commands:"
    echo "   $ wia-aug-006 help"
    echo
    echo "Configuration directory: $CONFIG_DIR"
    echo "Documentation: https://wiastandards.com/standards/physical-enhancement"
    echo
    echo -e "${CYAN}弘益人間 (Benefit All Humanity)${NC}"
    echo
}

################################################################################
# Main Installation
################################################################################

main() {
    print_header

    # Check if running as root
    if [ "$EUID" -eq 0 ]; then
        warning "Running as root. Installation will proceed with elevated privileges."
    fi

    # Run installation steps
    check_prerequisites
    create_directories
    install_cli
    install_typescript_sdk
    create_config_file
    create_sample_profile
    verify_installation
    print_next_steps

    success "Installation completed successfully!"
}

# Handle command line arguments
case "${1:-}" in
    --help|-h)
        echo "WIA-AUG-006 Physical Enhancement Installation Script"
        echo
        echo "Usage: $0 [options]"
        echo
        echo "Options:"
        echo "  --help, -h          Show this help message"
        echo "  --cli-only          Install CLI tool only (skip TypeScript SDK)"
        echo "  --uninstall         Uninstall WIA-AUG-006"
        echo
        echo "Environment variables:"
        echo "  INSTALL_DIR         Installation directory (default: /usr/local/bin)"
        echo
        exit 0
        ;;
    --cli-only)
        print_header
        check_prerequisites
        create_directories
        install_cli
        create_config_file
        verify_installation
        success "CLI-only installation completed!"
        ;;
    --uninstall)
        print_header
        step "Uninstalling WIA-AUG-006..."

        if [ -w "$INSTALL_DIR" ]; then
            rm -f "$INSTALL_DIR/wia-aug-006"
        else
            sudo rm -f "$INSTALL_DIR/wia-aug-006"
        fi

        read -p "Remove configuration directory $CONFIG_DIR? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$CONFIG_DIR"
            success "Configuration removed"
        fi

        success "Uninstallation completed"
        ;;
    "")
        main
        ;;
    *)
        error "Unknown option: $1. Use --help for usage information."
        ;;
esac
