#!/bin/bash

###############################################################################
# WIA-TIME-002: Spacetime Manipulation Standard - Installation Script
#
# Installs the WIA-TIME-002 SDK, CLI tools, and dependencies
#
# Version: 1.0.0
# License: MIT
# 弘益人間 · Benefit All Humanity
# WIA - World Certification Industry Association
###############################################################################

set -e

# Color codes
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly VIOLET='\033[0;38;5;93m'
readonly NC='\033[0m'

# Installation paths
readonly INSTALL_DIR="${HOME}/.wia/time-002"
readonly BIN_DIR="${HOME}/.local/bin"
readonly VERSION="1.0.0"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${VIOLET}"
    cat << "EOF"
╔═══════════════════════════════════════════════════════════════════════╗
║                                                                       ║
║   ██╗    ██╗██╗ █████╗       ████████╗██╗███╗   ███╗███████╗        ║
║   ██║    ██║██║██╔══██╗      ╚══██╔══╝██║████╗ ████║██╔════╝        ║
║   ██║ █╗ ██║██║███████║         ██║   ██║██╔████╔██║█████╗          ║
║   ██║███╗██║██║██╔══██║         ██║   ██║██║╚██╔╝██║██╔══╝          ║
║   ╚███╔███╔╝██║██║  ██║         ██║   ██║██║ ╚═╝ ██║███████╗        ║
║    ╚══╝╚══╝ ╚═╝╚═╝  ╚═╝         ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝        ║
║                                                                       ║
║              WIA-TIME-002: Spacetime Manipulation                    ║
║                                                                       ║
║                    🌌 Installer v1.0.0 🌌                            ║
║                                                                       ║
╚═══════════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ Error: $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠ Warning: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_section() {
    echo -e "\n${VIOLET}═══ $1 ═══${NC}\n"
}

###############################################################################
# Dependency Checks
###############################################################################

check_dependencies() {
    print_section "Checking Dependencies"

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    else
        print_success "bash: $(bash --version | head -n1)"
    fi

    # Check for bc (for calculations)
    if ! command -v bc &> /dev/null; then
        print_warning "bc not found (optional, for advanced calculations)"
    else
        print_success "bc: $(bc --version | head -n1)"
    fi

    # Check for Node.js (for TypeScript SDK)
    if ! command -v node &> /dev/null; then
        print_warning "Node.js not found (required for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    else
        print_success "Node.js: $(node --version)"
    fi

    # Check for npm (for TypeScript SDK)
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found (required for TypeScript SDK)"
    else
        print_success "npm: $(npm --version)"
    fi

    # Check for git
    if ! command -v git &> /dev/null; then
        missing_deps+=("git")
    else
        print_success "git: $(git --version)"
    fi

    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        print_info "Please install missing dependencies and try again."
        exit 1
    fi

    echo ""
}

###############################################################################
# Installation Functions
###############################################################################

create_directories() {
    print_section "Creating Directories"

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$BIN_DIR"

    print_success "Created: $INSTALL_DIR"
    print_success "Created: $BIN_DIR"
}

install_cli() {
    print_section "Installing CLI Tool"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local cli_source="${script_dir}/cli/wia-time-002.sh"

    if [[ ! -f "$cli_source" ]]; then
        print_error "CLI script not found: $cli_source"
        exit 1
    fi

    # Copy CLI script
    cp "$cli_source" "$BIN_DIR/wia-time-002"
    chmod +x "$BIN_DIR/wia-time-002"

    print_success "Installed CLI: $BIN_DIR/wia-time-002"

    # Check if BIN_DIR is in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        print_warning "$BIN_DIR is not in your PATH"
        print_info "Add this to your ~/.bashrc or ~/.zshrc:"
        echo ""
        echo "    export PATH=\"\$PATH:$BIN_DIR\""
        echo ""
    fi
}

install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    if ! command -v npm &> /dev/null; then
        print_warning "Skipping TypeScript SDK (npm not found)"
        return
    fi

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local sdk_dir="${script_dir}/api/typescript"

    if [[ ! -d "$sdk_dir" ]]; then
        print_error "TypeScript SDK directory not found: $sdk_dir"
        exit 1
    fi

    # Copy SDK to install directory
    cp -r "$sdk_dir" "$INSTALL_DIR/typescript"

    print_success "Copied SDK to: $INSTALL_DIR/typescript"

    # Install dependencies
    cd "$INSTALL_DIR/typescript"

    print_info "Installing npm dependencies..."
    npm install --silent

    print_success "Installed npm dependencies"

    # Build SDK
    print_info "Building TypeScript SDK..."
    npm run build --silent

    print_success "Built TypeScript SDK"

    echo ""
    print_info "To use the SDK in your project:"
    echo ""
    echo "    npm install $INSTALL_DIR/typescript"
    echo ""
    echo "Or link it globally:"
    echo ""
    echo "    cd $INSTALL_DIR/typescript"
    echo "    npm link"
    echo ""
}

install_documentation() {
    print_section "Installing Documentation"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Copy README
    if [[ -f "${script_dir}/README.md" ]]; then
        cp "${script_dir}/README.md" "$INSTALL_DIR/"
        print_success "Installed: README.md"
    fi

    # Copy specification
    if [[ -d "${script_dir}/spec" ]]; then
        cp -r "${script_dir}/spec" "$INSTALL_DIR/"
        print_success "Installed: spec/"
    fi
}

create_config() {
    print_section "Creating Configuration"

    local config_file="$INSTALL_DIR/config.json"

    cat > "$config_file" << EOF
{
  "version": "$VERSION",
  "standard": "WIA-TIME-002",
  "name": "Spacetime Manipulation",
  "category": "TIME",
  "color": "#8B5CF6",
  "installed_at": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "paths": {
    "install_dir": "$INSTALL_DIR",
    "bin_dir": "$BIN_DIR",
    "cli": "$BIN_DIR/wia-time-002"
  },
  "constants": {
    "SPEED_OF_LIGHT": 299792458,
    "GRAVITATIONAL_CONSTANT": 6.67430e-11,
    "PLANCK_CONSTANT": 6.62607015e-34
  }
}
EOF

    print_success "Created configuration: $config_file"
}

###############################################################################
# Post-Installation
###############################################################################

run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-time-002 &> /dev/null || [[ -x "$BIN_DIR/wia-time-002" ]]; then
        print_info "Testing CLI..."
        "$BIN_DIR/wia-time-002" version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
    fi

    # Test TypeScript SDK
    if [[ -d "$INSTALL_DIR/typescript/dist" ]]; then
        print_success "TypeScript SDK build verified"
    fi
}

print_completion() {
    print_section "Installation Complete"

    cat << EOF
${GREEN}✓ WIA-TIME-002: Spacetime Manipulation Standard installed successfully!${NC}

${VIOLET}Installation Summary:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Standard:      WIA-TIME-002
  Version:       $VERSION
  Install Dir:   $INSTALL_DIR
  CLI Tool:      $BIN_DIR/wia-time-002

${VIOLET}Next Steps:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. Add CLI to PATH (if not already):
     ${BLUE}export PATH="\$PATH:$BIN_DIR"${NC}

  2. Test the CLI:
     ${BLUE}wia-time-002 help${NC}
     ${BLUE}wia-time-002 version${NC}

  3. Try creating a gravity well:
     ${BLUE}wia-time-002 gravity-well --mass 1.989e30 --radius 6.96e8${NC}

  4. Generate a warp bubble:
     ${BLUE}wia-time-002 warp-bubble --velocity "1.5e8,0,0" --radius 100${NC}

  5. Use the TypeScript SDK:
     ${BLUE}cd $INSTALL_DIR/typescript${NC}
     ${BLUE}npm link${NC}

${VIOLET}Documentation:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  README:        $INSTALL_DIR/README.md
  Spec:          $INSTALL_DIR/spec/WIA-TIME-002-v1.0.md
  GitHub:        https://github.com/WIA-Official/wia-standards

${VIOLET}Resources:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  WIA Portal:    https://wiastandards.com
  Email:         standards@wia.org
  Discord:       https://discord.gg/wia-standards

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
${VIOLET}弘益人間 · Benefit All Humanity${NC}
${VIOLET}WIA - World Certification Industry Association${NC}
${VIOLET}© 2025 MIT License${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
EOF
}

###############################################################################
# Cleanup and Uninstallation
###############################################################################

uninstall() {
    print_section "Uninstalling WIA-TIME-002"

    if [[ -d "$INSTALL_DIR" ]]; then
        rm -rf "$INSTALL_DIR"
        print_success "Removed: $INSTALL_DIR"
    fi

    if [[ -f "$BIN_DIR/wia-time-002" ]]; then
        rm -f "$BIN_DIR/wia-time-002"
        print_success "Removed: $BIN_DIR/wia-time-002"
    fi

    print_success "WIA-TIME-002 uninstalled successfully"
}

###############################################################################
# Main Installation Flow
###############################################################################

main() {
    # Parse arguments
    if [[ "$1" == "--uninstall" ]]; then
        print_header
        uninstall
        exit 0
    fi

    if [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
        cat << EOF
WIA-TIME-002 Installation Script

Usage:
  ./install.sh              Install WIA-TIME-002
  ./install.sh --uninstall  Uninstall WIA-TIME-002
  ./install.sh --help       Show this help

This script will:
  1. Check dependencies
  2. Install CLI tool to ~/.local/bin
  3. Install TypeScript SDK to ~/.wia/time-002
  4. Install documentation
  5. Run verification tests

Requirements:
  - bash
  - git
  - Node.js and npm (for TypeScript SDK)
  - bc (optional, for CLI calculations)

弘益人間 · Benefit All Humanity
© 2025 WIA - MIT License
EOF
        exit 0
    fi

    # Run installation
    print_header

    check_dependencies
    create_directories
    install_cli
    install_typescript_sdk
    install_documentation
    create_config
    run_tests
    print_completion
}

# Run main function
main "$@"
