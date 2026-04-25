#!/bin/bash

################################################################################
# WIA-TIME-012: Matter Transmission - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

# Configuration
INSTALL_DIR="${INSTALL_DIR:-$HOME/.wia/time-012}"
BIN_DIR="${BIN_DIR:-/usr/local/bin}"

################################################################################
# Helper Functions
################################################################################

print_banner() {
    echo -e "${PURPLE}"
    echo "╔════════════════════════════════════════════════╗"
    echo "║  📦 WIA-TIME-012: Matter Transmission         ║"
    echo "║  Installation Script v1.0.0                   ║"
    echo "║  弘益人間 (Benefit All Humanity)                ║"
    echo "╚════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        return 1
    fi
    return 0
}

################################################################################
# Installation Steps
################################################################################

step_check_requirements() {
    echo ""
    print_info "Checking requirements..."

    local missing_deps=()

    # Check for required commands
    if ! check_command curl; then
        missing_deps+=("curl")
    fi

    if ! check_command jq; then
        missing_deps+=("jq")
    fi

    if ! check_command bc; then
        missing_deps+=("bc")
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install missing dependencies:"
        echo "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        echo "  macOS:         brew install ${missing_deps[*]}"
        echo "  Fedora/RHEL:   sudo dnf install ${missing_deps[*]}"
        exit 1
    fi

    print_success "All requirements satisfied"
}

step_create_directories() {
    echo ""
    print_info "Creating installation directories..."

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$INSTALL_DIR/cli"
    mkdir -p "$INSTALL_DIR/api"
    mkdir -p "$INSTALL_DIR/spec"
    mkdir -p "$HOME/.wia/backups"
    mkdir -p "$HOME/.wia/logs"

    print_success "Directories created"
}

step_install_cli() {
    echo ""
    print_info "Installing CLI tool..."

    # Copy CLI script
    cp cli/wia-time-012.sh "$INSTALL_DIR/cli/"
    chmod +x "$INSTALL_DIR/cli/wia-time-012.sh"

    # Create symlink
    if [ -w "$BIN_DIR" ]; then
        ln -sf "$INSTALL_DIR/cli/wia-time-012.sh" "$BIN_DIR/wia-time-012"
        print_success "CLI tool installed to $BIN_DIR/wia-time-012"
    else
        print_warning "Cannot write to $BIN_DIR (requires sudo)"
        echo ""
        echo "To complete installation, run:"
        echo "  sudo ln -sf $INSTALL_DIR/cli/wia-time-012.sh $BIN_DIR/wia-time-012"
        echo ""
        echo "Or add to your PATH:"
        echo "  export PATH=\"\$PATH:$INSTALL_DIR/cli\""
    fi
}

step_install_typescript_sdk() {
    echo ""
    print_info "Installing TypeScript SDK..."

    if check_command npm; then
        cp -r api/typescript "$INSTALL_DIR/api/"
        cd "$INSTALL_DIR/api/typescript"

        npm install
        npm run build

        print_success "TypeScript SDK installed"
        print_info "To use in your project: npm install $INSTALL_DIR/api/typescript"
    else
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
    fi
}

step_install_spec() {
    echo ""
    print_info "Installing specification files..."

    cp -r spec "$INSTALL_DIR/"
    cp README.md "$INSTALL_DIR/"

    print_success "Specification files installed"
}

step_configure() {
    echo ""
    print_info "Configuring WIA-TIME-012..."

    # Create configuration file
    cat > "$HOME/.wia/time-012.conf" <<EOF
# WIA-TIME-012 Configuration
# Generated: $(date)

# API Configuration
WIA_API_URL=https://api.wiastandards.com/time-012
WIA_API_KEY=

# Installation Paths
INSTALL_DIR=$INSTALL_DIR
BIN_DIR=$BIN_DIR

# Defaults
DEFAULT_RESOLUTION=atomic
DEFAULT_ERROR_CORRECTION=maximum
DEFAULT_PRIORITY=high
DEFAULT_QUANTUM_PRESERVATION=true

# Safety Settings
SAFETY_LEVEL=maximum
REQUIRE_BACKUP=true
BACKUP_REDUNDANCY=3

# Logging
LOG_DIR=$HOME/.wia/logs
LOG_LEVEL=info
EOF

    print_success "Configuration file created: $HOME/.wia/time-012.conf"
}

step_verify_installation() {
    echo ""
    print_info "Verifying installation..."

    # Check CLI
    if [ -x "$BIN_DIR/wia-time-012" ] || [ -x "$INSTALL_DIR/cli/wia-time-012.sh" ]; then
        print_success "CLI tool installed correctly"
    else
        print_error "CLI tool installation failed"
        return 1
    fi

    # Check TypeScript SDK
    if [ -d "$INSTALL_DIR/api/typescript" ]; then
        print_success "TypeScript SDK installed correctly"
    else
        print_warning "TypeScript SDK not installed"
    fi

    # Check spec files
    if [ -f "$INSTALL_DIR/spec/WIA-TIME-012-v1.0.md" ]; then
        print_success "Specification files installed correctly"
    else
        print_error "Specification files installation failed"
        return 1
    fi

    print_success "Installation verified"
}

################################################################################
# Main Installation
################################################################################

main() {
    print_banner

    echo "This script will install WIA-TIME-012: Matter Transmission"
    echo ""
    echo "Installation directory: $INSTALL_DIR"
    echo "Binary directory:       $BIN_DIR"
    echo ""

    read -p "Continue with installation? [Y/n] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]] && [[ ! -z $REPLY ]]; then
        echo "Installation cancelled"
        exit 0
    fi

    # Run installation steps
    step_check_requirements
    step_create_directories
    step_install_cli
    step_install_typescript_sdk
    step_install_spec
    step_configure
    step_verify_installation

    # Print success message
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  ✓ Installation Complete!                     ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
    echo ""

    print_info "Next steps:"
    echo ""
    echo "1. Set your API key:"
    echo "   export WIA_API_KEY=your_key_here"
    echo ""
    echo "2. Try the CLI:"
    echo "   wia-time-012 --help"
    echo ""
    echo "3. Run a test analysis:"
    echo "   wia-time-012 analyze TEST-001 atomic true json"
    echo ""
    echo "4. Read the documentation:"
    echo "   $INSTALL_DIR/README.md"
    echo "   $INSTALL_DIR/spec/WIA-TIME-012-v1.0.md"
    echo ""

    print_info "For TypeScript projects:"
    echo "   npm install $INSTALL_DIR/api/typescript"
    echo ""

    print_info "Configuration file:"
    echo "   $HOME/.wia/time-012.conf"
    echo ""

    echo -e "${PURPLE}弘익人間 (Benefit All Humanity)${NC}"
    echo ""
}

################################################################################
# Uninstall Function
################################################################################

uninstall() {
    print_banner
    echo "This will remove WIA-TIME-012 from your system"
    echo ""

    read -p "Are you sure? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Uninstall cancelled"
        exit 0
    fi

    print_info "Removing installation..."

    # Remove directories
    rm -rf "$INSTALL_DIR"

    # Remove symlink
    if [ -L "$BIN_DIR/wia-time-012" ]; then
        rm -f "$BIN_DIR/wia-time-012"
    fi

    # Remove config (ask first)
    if [ -f "$HOME/.wia/time-012.conf" ]; then
        read -p "Remove configuration file? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -f "$HOME/.wia/time-012.conf"
        fi
    fi

    print_success "WIA-TIME-012 has been removed"
}

################################################################################
# Script Entry Point
################################################################################

if [ "$1" = "uninstall" ]; then
    uninstall
else
    main
fi
