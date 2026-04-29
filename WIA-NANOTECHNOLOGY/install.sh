#!/bin/bash
###############################################################################
# WIA-NANOTECHNOLOGY Installation Script
#
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-NANOTECHNOLOGY standard components including:
# - TypeScript SDK
# - CLI tools
# - Documentation (ebooks)
# - Standard specifications
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

###############################################################################
# Utility Functions
###############################################################################

print_banner() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                                                               ║"
    echo "║              WIA-NANOTECHNOLOGY INSTALLER                     ║"
    echo "║                                                               ║"
    echo "║                  Version 1.0.0                                ║"
    echo "║                                                               ║"
    echo "║              弘益人間 (Benefit All Humanity)                   ║"
    echo "║                                                               ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

check_command() {
    if command -v $1 &> /dev/null; then
        print_success "$1 is installed"
        return 0
    else
        print_warning "$1 is not installed"
        return 1
    fi
}

###############################################################################
# Dependency Checking
###############################################################################

check_dependencies() {
    print_info "Checking dependencies..."

    local all_ok=true

    # Required dependencies
    if ! check_command "node"; then
        print_error "Node.js is required but not installed"
        print_info "Please install Node.js 16+ from https://nodejs.org"
        all_ok=false
    else
        local node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [ "$node_version" -lt 16 ]; then
            print_error "Node.js version 16+ is required (found v$node_version)"
            all_ok=false
        else
            print_success "Node.js version is compatible"
        fi
    fi

    if ! check_command "npm"; then
        print_error "npm is required but not installed"
        all_ok=false
    fi

    # Optional dependencies
    check_command "git" || print_info "Git is recommended for version control"
    check_command "jq" || print_info "jq is recommended for CLI output formatting"
    check_command "curl" || print_warning "curl is recommended for API testing"

    if [ "$all_ok" = false ]; then
        print_error "Please install required dependencies and try again"
        exit 1
    fi

    print_success "All required dependencies are installed"
}

###############################################################################
# TypeScript SDK Installation
###############################################################################

install_typescript_sdk() {
    print_info "Installing TypeScript SDK..."

    cd "$SCRIPT_DIR/api/typescript"

    if [ -d "node_modules" ]; then
        print_warning "node_modules exists, cleaning..."
        rm -rf node_modules
    fi

    print_info "Installing npm packages..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    if [ $? -eq 0 ]; then
        print_success "TypeScript SDK installed and built successfully"
    else
        print_error "Failed to build TypeScript SDK"
        exit 1
    fi

    cd "$SCRIPT_DIR"
}

###############################################################################
# CLI Tools Installation
###############################################################################

install_cli_tools() {
    print_info "Installing CLI tools..."

    local cli_script="$SCRIPT_DIR/cli/wia-nanotechnology.sh"
    local install_path="/usr/local/bin/wia-nanotechnology"

    # Check if script is executable
    if [ ! -x "$cli_script" ]; then
        chmod +x "$cli_script"
        print_info "Made CLI script executable"
    fi

    # Try to install globally
    if [ -w "/usr/local/bin" ]; then
        cp "$cli_script" "$install_path"
        chmod +x "$install_path"
        print_success "CLI tool installed to $install_path"
        print_info "You can now run: wia-nanotechnology --help"
    else
        print_warning "Cannot install to /usr/local/bin (requires sudo)"
        print_info "You can install manually with:"
        print_info "  sudo cp $cli_script /usr/local/bin/wia-nanotechnology"
        print_info "  sudo chmod +x /usr/local/bin/wia-nanotechnology"
        print_info ""
        print_info "Or use directly: $cli_script"
    fi
}

###############################################################################
# Documentation Setup
###############################################################################

setup_documentation() {
    print_info "Setting up documentation..."

    # Check if ebooks exist
    if [ -d "$SCRIPT_DIR/ebook/en" ] && [ -d "$SCRIPT_DIR/ebook/ko" ]; then
        local en_count=$(ls -1 "$SCRIPT_DIR/ebook/en"/*.html 2>/dev/null | wc -l)
        local ko_count=$(ls -1 "$SCRIPT_DIR/ebook/ko"/*.html 2>/dev/null | wc -l)

        print_success "Found $en_count English ebook files"
        print_success "Found $ko_count Korean ebook files"

        # Offer to open in browser
        print_info "To view the ebook, open:"
        print_info "  English: file://$SCRIPT_DIR/ebook/en/index.html"
        print_info "  Korean:  file://$SCRIPT_DIR/ebook/ko/index.html"
    else
        print_warning "Ebook files not found"
    fi

    # Check specifications
    if [ -d "$SCRIPT_DIR/spec" ]; then
        local spec_count=$(ls -1 "$SCRIPT_DIR/spec"/*.md 2>/dev/null | wc -l)
        print_success "Found $spec_count specification files"
    fi
}

###############################################################################
# Environment Setup
###############################################################################

setup_environment() {
    print_info "Setting up environment..."

    local shell_config=""
    if [ -f "$HOME/.bashrc" ]; then
        shell_config="$HOME/.bashrc"
    elif [ -f "$HOME/.zshrc" ]; then
        shell_config="$HOME/.zshrc"
    fi

    if [ -n "$shell_config" ]; then
        # Add WIA-NANO environment variable examples to shell config
        if ! grep -q "WIA_NANO_API_KEY" "$shell_config"; then
            echo "" >> "$shell_config"
            echo "# WIA-NANOTECHNOLOGY Configuration" >> "$shell_config"
            echo "# export WIA_NANO_API_KEY=\"your-api-key-here\"" >> "$shell_config"
            echo "# export WIA_NANO_API_URL=\"https://api.wia-nanotechnology.org/v1\"" >> "$shell_config"
            print_info "Added environment variable examples to $shell_config"
            print_warning "Please edit $shell_config to set your API key"
        else
            print_info "Environment variables already configured in $shell_config"
        fi
    fi
}

###############################################################################
# Post-Installation Tests
###############################################################################

run_tests() {
    print_info "Running post-installation tests..."

    # Test CLI
    local cli_path="/usr/local/bin/wia-nanotechnology"
    if [ ! -f "$cli_path" ]; then
        cli_path="$SCRIPT_DIR/cli/wia-nanotechnology.sh"
    fi

    if [ -x "$cli_path" ]; then
        "$cli_path" version &> /dev/null
        if [ $? -eq 0 ]; then
            print_success "CLI tool is functional"
        else
            print_warning "CLI tool may have issues"
        fi
    fi

    # Test TypeScript SDK
    cd "$SCRIPT_DIR/api/typescript"
    if [ -d "dist" ]; then
        print_success "TypeScript SDK build artifacts present"
    else
        print_warning "TypeScript SDK build artifacts missing"
    fi
    cd "$SCRIPT_DIR"
}

###############################################################################
# Installation Summary
###############################################################################

print_summary() {
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                 INSTALLATION COMPLETE!                        ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    print_info "WIA-NANOTECHNOLOGY v1.0.0 has been installed"
    echo ""
    echo -e "${BLUE}Next Steps:${NC}"
    echo ""
    echo "  1. Set your API key:"
    echo "     export WIA_NANO_API_KEY=\"your-api-key\""
    echo ""
    echo "  2. Try the CLI:"
    echo "     wia-nanotechnology --help"
    echo "     wia-nanotechnology material-search --type CNT"
    echo ""
    echo "  3. Use the TypeScript SDK:"
    echo "     cd api/typescript"
    echo "     npm link"
    echo "     # Then in your project: npm link @wia/nanotechnology"
    echo ""
    echo "  4. Read the documentation:"
    echo "     - Ebook: file://$SCRIPT_DIR/ebook/en/index.html"
    echo "     - Specs: $SCRIPT_DIR/spec/"
    echo "     - README: $SCRIPT_DIR/README.md"
    echo ""
    echo -e "${YELLOW}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
}

###############################################################################
# Main Installation Flow
###############################################################################

main() {
    print_banner

    print_info "Starting installation..."
    echo ""

    # Run installation steps
    check_dependencies
    echo ""

    install_typescript_sdk
    echo ""

    install_cli_tools
    echo ""

    setup_documentation
    echo ""

    setup_environment
    echo ""

    run_tests
    echo ""

    print_summary
}

# Trap errors
trap 'print_error "Installation failed at line $LINENO"' ERR

# Run main installation
main "$@"
