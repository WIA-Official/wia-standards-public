#!/bin/bash

################################################################################
# WIA-IND-003: Wearable Fashion Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Fashion Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-003 CLI tool and TypeScript SDK
################################################################################

set -e

# Colors
INDIGO='\033[0;35m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
RESET='\033[0m'

# Version
VERSION="1.0.0"

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLI_SOURCE="${SCRIPT_DIR}/cli/wia-ind-003.sh"
INSTALL_DIR="/usr/local/bin"
CLI_TARGET="${INSTALL_DIR}/wia-ind-003"

# Print functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    👔 WIA-IND-003: Wearable Fashion Standard Installer       ║"
    echo "║                      Version ${VERSION}                           ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    echo ""
}

print_step() {
    echo -e "${CYAN}▶ $1${RESET}"
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

# Check if running as root (for system-wide install)
check_permissions() {
    if [ "$EUID" -ne 0 ] && [ "$1" != "--user" ]; then
        print_warning "Not running as root. Will attempt user installation."
        print_info "Run with 'sudo ./install.sh' for system-wide installation."
        print_info "Or use './install.sh --user' for user-only installation."
        echo ""
        USER_INSTALL=true
        INSTALL_DIR="$HOME/.local/bin"
        CLI_TARGET="${INSTALL_DIR}/wia-ind-003"
    else
        USER_INSTALL=false
    fi
}

# Check dependencies
check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for bc (required for calculations)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    fi

    # Check for bash (should always be present)
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    if [ ${#missing_deps[@]} -eq 0 ]; then
        print_success "All dependencies satisfied"
    else
        print_error "Missing dependencies: ${missing_deps[*]}"
        print_info "Please install missing dependencies:"
        print_info "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        print_info "  macOS: brew install ${missing_deps[*]}"
        print_info "  RedHat/CentOS: sudo yum install ${missing_deps[*]}"
        exit 1
    fi
    echo ""
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool..."

    # Create install directory if it doesn't exist
    if [ ! -d "$INSTALL_DIR" ]; then
        print_info "Creating directory: $INSTALL_DIR"
        mkdir -p "$INSTALL_DIR"
    fi

    # Check if CLI source exists
    if [ ! -f "$CLI_SOURCE" ]; then
        print_error "CLI source not found: $CLI_SOURCE"
        exit 1
    fi

    # Copy CLI to install location
    print_info "Copying wia-ind-003 to $INSTALL_DIR"
    cp "$CLI_SOURCE" "$CLI_TARGET"

    # Make executable
    chmod +x "$CLI_TARGET"

    print_success "CLI tool installed to $CLI_TARGET"
    echo ""
}

# Setup PATH
setup_path() {
    if [ "$USER_INSTALL" = true ]; then
        print_step "Setting up PATH..."

        # Check if INSTALL_DIR is in PATH
        if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
            print_warning "$INSTALL_DIR is not in your PATH"

            # Determine shell config file
            if [ -n "$ZSH_VERSION" ]; then
                SHELL_CONFIG="$HOME/.zshrc"
            elif [ -n "$BASH_VERSION" ]; then
                if [ -f "$HOME/.bashrc" ]; then
                    SHELL_CONFIG="$HOME/.bashrc"
                else
                    SHELL_CONFIG="$HOME/.bash_profile"
                fi
            else
                SHELL_CONFIG="$HOME/.profile"
            fi

            # Add to PATH
            echo "" >> "$SHELL_CONFIG"
            echo "# WIA-IND-003 CLI" >> "$SHELL_CONFIG"
            echo "export PATH=\"\$PATH:$INSTALL_DIR\"" >> "$SHELL_CONFIG"

            print_success "Added $INSTALL_DIR to PATH in $SHELL_CONFIG"
            print_warning "Please restart your shell or run: source $SHELL_CONFIG"
        else
            print_success "$INSTALL_DIR is already in PATH"
        fi
        echo ""
    fi
}

# Install TypeScript SDK
install_sdk() {
    print_step "Installing TypeScript SDK..."

    local SDK_DIR="${SCRIPT_DIR}/api/typescript"

    if [ ! -d "$SDK_DIR" ]; then
        print_warning "TypeScript SDK directory not found. Skipping SDK installation."
        return
    fi

    cd "$SDK_DIR"

    # Check if npm is available
    if command -v npm &> /dev/null; then
        print_info "Installing npm dependencies..."
        npm install

        print_info "Building TypeScript SDK..."
        npm run build 2>/dev/null || print_warning "Build failed. TypeScript may not be installed."

        print_success "TypeScript SDK installed"
    else
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "Install Node.js to use the TypeScript SDK:"
        print_info "  https://nodejs.org/"
    fi

    cd "$SCRIPT_DIR"
    echo ""
}

# Verify installation
verify_installation() {
    print_step "Verifying installation..."

    if [ -x "$CLI_TARGET" ]; then
        print_success "CLI tool is executable"

        # Test CLI
        if "$CLI_TARGET" --version &> /dev/null; then
            local version=$("$CLI_TARGET" --version)
            print_success "CLI tool works: $version"
        else
            print_warning "CLI tool installed but --version check failed"
        fi
    else
        print_error "CLI tool installation failed"
        exit 1
    fi
    echo ""
}

# Print post-installation instructions
print_instructions() {
    print_step "Installation Complete!"
    echo ""
    echo -e "${GREEN}The WIA-IND-003 CLI tool has been installed successfully.${RESET}"
    echo ""
    echo "Usage examples:"
    echo "  wia-ind-003 --help"
    echo "  wia-ind-003 calc-led-power --count 100 --current 20"
    echo "  wia-ind-003 calc-battery --capacity 1000 --current 50"
    echo "  wia-ind-003 design-jewelry --type bracelet --features heartRate,ledDisplay"
    echo "  wia-ind-003 calc-thermal --power 15 --area 500"
    echo "  wia-ind-003 calc-harvest --type solar --area 200"
    echo ""

    if [ "$USER_INSTALL" = true ]; then
        print_warning "User installation: You may need to restart your shell or run:"
        print_info "source $SHELL_CONFIG"
        echo ""
    fi

    echo "Documentation:"
    print_info "README: ${SCRIPT_DIR}/README.md"
    print_info "Spec: ${SCRIPT_DIR}/spec/WIA-IND-003-v1.0.md"
    print_info "TypeScript: ${SCRIPT_DIR}/api/typescript/"
    echo ""

    echo "Resources:"
    print_info "Website: https://wiastandards.com"
    print_info "GitHub: https://github.com/WIA-Official/wia-standards"
    print_info "Docs: https://docs.wiastandards.com"
    echo ""

    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${CYAN}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    print_header
    print_step "Uninstalling WIA-IND-003..."

    if [ -f "$CLI_TARGET" ]; then
        rm "$CLI_TARGET"
        print_success "Removed $CLI_TARGET"
    else
        print_warning "CLI tool not found at $CLI_TARGET"
    fi

    print_success "Uninstallation complete"
    echo ""
}

# Main installation
main() {
    print_header

    # Parse arguments
    case "${1:-}" in
        --uninstall)
            check_permissions "$@"
            uninstall
            exit 0
            ;;
        --user)
            check_permissions "$1"
            ;;
        --help|-h)
            echo "WIA-IND-003 Installation Script"
            echo ""
            echo "Usage: ./install.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --user        Install for current user only"
            echo "  --uninstall   Uninstall WIA-IND-003"
            echo "  --help        Show this help message"
            echo ""
            echo "Examples:"
            echo "  sudo ./install.sh          # System-wide installation"
            echo "  ./install.sh --user        # User installation"
            echo "  sudo ./install.sh --uninstall"
            echo ""
            exit 0
            ;;
        *)
            check_permissions "$@"
            ;;
    esac

    # Run installation steps
    check_dependencies
    install_cli
    setup_path
    install_sdk
    verify_installation
    print_instructions
}

# Run main function
main "$@"
