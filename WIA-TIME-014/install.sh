#!/bin/bash

################################################################################
# WIA-TIME-014: Data Time Transport - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-014 standard including:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Configuration
INSTALL_DIR="${INSTALL_DIR:-$HOME/.local/bin}"
CONFIG_DIR="$HOME/.wia/time-014"
VERSION="1.0.0"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       💾 WIA-TIME-014: Data Time Transport Installer         ║"
    echo "║                      Version $VERSION                            ║"
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
    exit 1
}

print_info() {
    echo -e "${BLUE}ℹ $1${RESET}"
}

# Check prerequisites
check_prerequisites() {
    print_section "Checking Prerequisites"

    local missing_deps=()

    # Check for required commands
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    else
        print_success "bash found ($(bash --version | head -n1))"
    fi

    if ! command -v jq &> /dev/null; then
        print_warning "jq not found (optional, but recommended)"
    else
        print_success "jq found ($(jq --version))"
    fi

    # Check for Node.js (for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js found ($(node --version))"

        if command -v npm &> /dev/null; then
            print_success "npm found ($(npm --version))"
        else
            print_warning "npm not found (required for TypeScript SDK)"
        fi
    else
        print_warning "Node.js not found (required for TypeScript SDK)"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"

    # Copy CLI script
    local cli_source="$SCRIPT_DIR/cli/wia-time-014.sh"
    local cli_target="$INSTALL_DIR/wia-time-014"

    if [[ ! -f "$cli_source" ]]; then
        print_error "CLI script not found: $cli_source"
    fi

    cp "$cli_source" "$cli_target"
    chmod +x "$cli_target"

    print_success "CLI tool installed to: $cli_target"

    # Check if install directory is in PATH
    if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
        print_warning "Install directory not in PATH"
        print_info "Add this line to your ~/.bashrc or ~/.zshrc:"
        echo -e "  ${GRAY}export PATH=\"\$PATH:$INSTALL_DIR\"${RESET}"
    else
        print_success "Install directory is in PATH"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local sdk_dir="$SCRIPT_DIR/api/typescript"

    if [[ ! -d "$sdk_dir" ]]; then
        print_error "TypeScript SDK directory not found: $sdk_dir"
    fi

    cd "$sdk_dir"

    # Check for package.json
    if [[ ! -f "package.json" ]]; then
        print_error "package.json not found"
    fi

    # Install dependencies
    if command -v npm &> /dev/null; then
        print_info "Installing npm dependencies..."
        npm install

        print_success "Dependencies installed"

        # Build TypeScript
        print_info "Building TypeScript..."
        npm run build

        print_success "TypeScript SDK built successfully"

        # Optional: Link globally
        read -p "Install SDK globally? (npm link) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            npm link
            print_success "SDK linked globally"
            print_info "You can now use: npm link @wia/time-014"
        fi
    else
        print_warning "npm not available, skipping TypeScript SDK installation"
    fi

    cd - > /dev/null
}

# Setup configuration
setup_config() {
    print_section "Setting Up Configuration"

    # Create config directories
    mkdir -p "$CONFIG_DIR"
    mkdir -p "$CONFIG_DIR/capsules"
    mkdir -p "$CONFIG_DIR/messages"
    mkdir -p "$CONFIG_DIR/data"

    print_success "Configuration directories created"
    echo -e "  ${GRAY}Config:${RESET}   $CONFIG_DIR"
    echo -e "  ${GRAY}Capsules:${RESET} $CONFIG_DIR/capsules"
    echo -e "  ${GRAY}Messages:${RESET} $CONFIG_DIR/messages"
    echo -e "  ${GRAY}Data:${RESET}     $CONFIG_DIR/data"

    # Create default config file
    cat > "$CONFIG_DIR/config.json" <<EOF
{
  "version": "$VERSION",
  "installedAt": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "apiEndpoint": "https://api.wia-time-014.org",
  "defaultTimeline": "TL-01-PRIME-ALPHA-0000000001",
  "defaultCompression": "zstd",
  "defaultRedundancy": 3
}
EOF

    print_success "Default configuration created: $CONFIG_DIR/config.json"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$CONFIG_DIR/docs"
    mkdir -p "$docs_dir"

    # Copy documentation
    if [[ -f "$SCRIPT_DIR/README.md" ]]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "README.md copied"
    fi

    if [[ -f "$SCRIPT_DIR/spec/WIA-TIME-014-v1.0.md" ]]; then
        cp "$SCRIPT_DIR/spec/WIA-TIME-014-v1.0.md" "$docs_dir/"
        print_success "Specification copied"
    fi

    print_info "Documentation available at: $docs_dir"
}

# Verify installation
verify_installation() {
    print_section "Verifying Installation"

    # Check CLI
    if command -v wia-time-014 &> /dev/null; then
        print_success "CLI tool accessible"
        wia-time-014 version
    else
        print_warning "CLI tool not in PATH (restart shell or update PATH)"
    fi

    # Check TypeScript SDK
    if [[ -f "$SCRIPT_DIR/api/typescript/dist/index.js" ]]; then
        print_success "TypeScript SDK built"
    else
        print_warning "TypeScript SDK not built"
    fi
}

# Print installation summary
print_summary() {
    print_section "Installation Summary"

    cat <<EOF
${GREEN}Installation completed successfully!${RESET}

${CYAN}CLI Tool:${RESET}
  Command:    wia-time-014
  Location:   $INSTALL_DIR/wia-time-014
  Version:    $VERSION

${CYAN}TypeScript SDK:${RESET}
  Package:    @wia/time-014
  Location:   $SCRIPT_DIR/api/typescript
  Version:    $VERSION

${CYAN}Configuration:${RESET}
  Directory:  $CONFIG_DIR
  Config:     $CONFIG_DIR/config.json

${CYAN}Quick Start:${RESET}
  # Create a time capsule
  wia-time-014 create-capsule data.json '2030-01-01T00:00:00Z' 'My Capsule'

  # Send a temporal message
  wia-time-014 send 'Hello future!' '2026-01-01T00:00:00Z'

  # Encode data
  wia-time-014 encode myfile.txt

  # Check bandwidth
  wia-time-014 bandwidth status

  # List timelines
  wia-time-014 timeline-list

  # Get help
  wia-time-014 help

${CYAN}TypeScript Usage:${RESET}
  ${GRAY}import { DataTimeTransportSDK } from '@wia/time-014';${RESET}

  ${GRAY}const sdk = new DataTimeTransportSDK();${RESET}
  ${GRAY}const capsule = await sdk.createTimeCapsule({${RESET}
    ${GRAY}data: { message: 'Hello!' },${RESET}
    ${GRAY}deliveryTime: new Date('2030-01-01')${RESET}
  ${GRAY}});${RESET}

${CYAN}Documentation:${RESET}
  README:     $CONFIG_DIR/docs/README.md
  Spec:       $CONFIG_DIR/docs/WIA-TIME-014-v1.0.md
  Website:    https://wiastandards.com/standards/WIA-TIME-014

${CYAN}Support:${RESET}
  GitHub:     https://github.com/WIA-Official/wia-standards
  Issues:     https://github.com/WIA-Official/wia-standards/issues
  Docs:       https://docs.wiastandards.com

${VIOLET}弘益人間 (홍익인간) · Benefit All Humanity${RESET}

${GRAY}WIA - World Certification Industry Association${RESET}
${GRAY}© 2025 SmileStory Inc. / WIA${RESET}
${GRAY}MIT License${RESET}
EOF
}

# Uninstall function
uninstall() {
    print_header
    print_section "Uninstalling WIA-TIME-014"

    read -p "Are you sure you want to uninstall WIA-TIME-014? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Uninstallation cancelled"
        exit 0
    fi

    # Remove CLI
    if [[ -f "$INSTALL_DIR/wia-time-014" ]]; then
        rm "$INSTALL_DIR/wia-time-014"
        print_success "CLI tool removed"
    fi

    # Remove config (ask for confirmation)
    read -p "Remove configuration and data? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$CONFIG_DIR"
        print_success "Configuration removed"
    else
        print_info "Configuration preserved at: $CONFIG_DIR"
    fi

    # Unlink TypeScript SDK
    if command -v npm &> /dev/null; then
        cd "$SCRIPT_DIR/api/typescript" 2>/dev/null || true
        npm unlink 2>/dev/null || true
        cd - > /dev/null 2>&1 || true
    fi

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    # Check for uninstall flag
    if [[ "$1" == "--uninstall" || "$1" == "uninstall" ]]; then
        uninstall
        exit 0
    fi

    print_header

    # Check if running with sudo (not recommended)
    if [[ $EUID -eq 0 ]]; then
        print_warning "Running as root is not recommended"
        print_info "Installing to user directory: $INSTALL_DIR"
    fi

    # Run installation steps
    check_prerequisites
    install_cli
    setup_config
    install_docs

    # Ask about TypeScript SDK
    if command -v npm &> /dev/null; then
        read -p "Install TypeScript SDK? [Y/n] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            install_typescript_sdk
        fi
    fi

    verify_installation
    print_summary

    echo ""
    print_success "Installation complete!"
    print_info "Run 'wia-time-014 help' to get started"
}

# Run main installation
main "$@"
