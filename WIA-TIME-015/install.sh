#!/bin/bash

################################################################################
# WIA-TIME-015: Time Machine Hardware - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Temporal Hardware Engineering Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-015 Time Machine Hardware standard,
# including the CLI tool and TypeScript SDK.
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
INSTALL_DIR="${INSTALL_DIR:-$HOME/.wia/time-015}"
BIN_DIR="${BIN_DIR:-$HOME/.local/bin}"

# Functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     ⚙️  WIA-TIME-015: Time Machine Hardware Installer        ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
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
    echo -e "  $1"
}

# Check prerequisites
check_prerequisites() {
    print_section "Checking Prerequisites"

    local has_error=0

    # Check for bash
    if command -v bash >/dev/null 2>&1; then
        print_success "bash $(bash --version | head -n1 | cut -d' ' -f4)"
    else
        print_error "bash not found"
        has_error=1
    fi

    # Check for node (optional, for TypeScript SDK)
    if command -v node >/dev/null 2>&1; then
        print_success "node $(node --version)"
    else
        print_warning "node not found (TypeScript SDK will not be available)"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command -v npm >/dev/null 2>&1; then
        print_success "npm $(npm --version)"
    else
        print_warning "npm not found (TypeScript SDK will not be available)"
    fi

    # Check for bc (for CLI calculations)
    if command -v bc >/dev/null 2>&1; then
        print_success "bc found"
    else
        print_warning "bc not found (some CLI calculations may not work)"
    fi

    if [ $has_error -eq 1 ]; then
        print_error "Prerequisites check failed"
        exit 1
    fi
}

# Create directories
create_directories() {
    print_section "Creating Directories"

    mkdir -p "$INSTALL_DIR"
    print_success "Created $INSTALL_DIR"

    mkdir -p "$BIN_DIR"
    print_success "Created $BIN_DIR"

    mkdir -p "$INSTALL_DIR/spec"
    mkdir -p "$INSTALL_DIR/api/typescript"
    mkdir -p "$INSTALL_DIR/cli"
    print_success "Created subdirectories"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    # Get the directory of this script
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Copy CLI script
    if [ -f "$SCRIPT_DIR/cli/wia-time-015.sh" ]; then
        cp "$SCRIPT_DIR/cli/wia-time-015.sh" "$INSTALL_DIR/cli/"
        chmod +x "$INSTALL_DIR/cli/wia-time-015.sh"
        print_success "Copied CLI script"

        # Create symlink in bin directory
        ln -sf "$INSTALL_DIR/cli/wia-time-015.sh" "$BIN_DIR/wia-time-015"
        print_success "Created symlink: $BIN_DIR/wia-time-015"
    else
        print_error "CLI script not found at $SCRIPT_DIR/cli/wia-time-015.sh"
        exit 1
    fi
}

# Install specification
install_spec() {
    print_section "Installing Specification"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if [ -f "$SCRIPT_DIR/spec/WIA-TIME-015-v1.0.md" ]; then
        cp "$SCRIPT_DIR/spec/WIA-TIME-015-v1.0.md" "$INSTALL_DIR/spec/"
        print_success "Copied specification"
    else
        print_warning "Specification not found (skipping)"
    fi

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$INSTALL_DIR/"
        print_success "Copied README"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if ! command -v npm >/dev/null 2>&1; then
        print_warning "npm not available, skipping TypeScript SDK"
        return
    fi

    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cp -r "$SCRIPT_DIR/api/typescript"/* "$INSTALL_DIR/api/typescript/"
        print_success "Copied TypeScript SDK"

        # Install dependencies
        cd "$INSTALL_DIR/api/typescript"
        print_info "Installing npm dependencies..."
        npm install --silent 2>&1 | grep -v "npm WARN" || true
        print_success "Installed dependencies"

        # Build SDK
        if [ -f "package.json" ]; then
            print_info "Building TypeScript SDK..."
            npm run build --silent 2>&1 | grep -v "npm WARN" || true
            print_success "Built TypeScript SDK"
        fi

        cd - >/dev/null
    else
        print_warning "TypeScript SDK not found (skipping)"
    fi
}

# Configure environment
configure_environment() {
    print_section "Configuring Environment"

    # Check if bin directory is in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        print_warning "$BIN_DIR is not in PATH"
        print_info "Add this line to your ~/.bashrc or ~/.zshrc:"
        echo -e "\n  ${CYAN}export PATH=\"\$PATH:$BIN_DIR\"${RESET}\n"
    else
        print_success "$BIN_DIR is in PATH"
    fi

    # Create configuration file
    cat > "$INSTALL_DIR/config.sh" << EOF
# WIA-TIME-015 Configuration
export WIA_TIME_015_VERSION="$VERSION"
export WIA_TIME_015_HOME="$INSTALL_DIR"
export WIA_TIME_015_SPEC="$INSTALL_DIR/spec/WIA-TIME-015-v1.0.md"
export WIA_TIME_015_CLI="$INSTALL_DIR/cli/wia-time-015.sh"
EOF

    print_success "Created configuration file"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    if [ -x "$BIN_DIR/wia-time-015" ]; then
        print_info "Testing CLI tool..."
        "$BIN_DIR/wia-time-015" --version >/dev/null 2>&1
        print_success "CLI tool working"
    else
        print_warning "CLI tool not executable"
    fi
}

# Print installation summary
print_summary() {
    print_section "Installation Summary"

    echo ""
    print_success "WIA-TIME-015 Time Machine Hardware installed successfully!"
    echo ""
    print_info "Installation directory: $INSTALL_DIR"
    print_info "CLI tool: $BIN_DIR/wia-time-015"
    echo ""

    print_section "Quick Start"
    echo ""
    print_info "1. Check hardware status:"
    echo -e "   ${CYAN}wia-time-015 status${RESET}"
    echo ""
    print_info "2. Run diagnostics:"
    echo -e "   ${CYAN}wia-time-015 diagnostics${RESET}"
    echo ""
    print_info "3. Charge flux capacitor:"
    echo -e "   ${CYAN}wia-time-015 flux-charge --level 1.0${RESET}"
    echo ""
    print_info "4. Check navigation to target date:"
    echo -e "   ${CYAN}wia-time-015 nav-check --target \"1985-11-05T01:21:00Z\"${RESET}"
    echo ""
    print_info "5. Get help:"
    echo -e "   ${CYAN}wia-time-015 --help${RESET}"
    echo ""

    if command -v npm >/dev/null 2>&1; then
        print_section "TypeScript SDK"
        echo ""
        print_info "Install via npm:"
        echo -e "   ${CYAN}npm install @wia/time-015${RESET}"
        echo ""
        print_info "Or use locally:"
        echo -e "   ${CYAN}cd $INSTALL_DIR/api/typescript${RESET}"
        echo -e "   ${CYAN}npm link${RESET}"
        echo ""
    fi

    print_section "Documentation"
    echo ""
    print_info "Specification: $INSTALL_DIR/spec/WIA-TIME-015-v1.0.md"
    print_info "README: $INSTALL_DIR/README.md"
    print_info "Website: https://wiastandards.com/standards/WIA-TIME-015"
    echo ""

    print_section "Philosophy"
    echo ""
    echo -e "  ${VIOLET}弘益人間 (홍익인간) · Benefit All Humanity${RESET}"
    echo ""
    echo -e "  ${CYAN}WIA - World Certification Industry Association${RESET}"
    echo -e "  ${CYAN}© 2025 SmileStory Inc. / WIA${RESET}"
    echo -e "  ${CYAN}MIT License${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    print_header
    print_section "Uninstalling WIA-TIME-015"

    read -p "Are you sure you want to uninstall WIA-TIME-015? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Uninstall cancelled"
        exit 0
    fi

    # Remove installation directory
    if [ -d "$INSTALL_DIR" ]; then
        rm -rf "$INSTALL_DIR"
        print_success "Removed $INSTALL_DIR"
    fi

    # Remove symlink
    if [ -L "$BIN_DIR/wia-time-015" ]; then
        rm "$BIN_DIR/wia-time-015"
        print_success "Removed $BIN_DIR/wia-time-015"
    fi

    print_success "WIA-TIME-015 uninstalled"
    echo ""
}

# Main installation
main() {
    # Handle uninstall
    if [ "$1" = "uninstall" ] || [ "$1" = "--uninstall" ]; then
        uninstall
        exit 0
    fi

    print_header

    check_prerequisites
    create_directories
    install_cli
    install_spec
    install_typescript_sdk
    configure_environment
    run_tests
    print_summary
}

# Run main
main "$@"
