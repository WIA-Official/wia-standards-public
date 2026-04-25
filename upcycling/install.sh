#!/bin/bash

#############################################################################
# WIA-ENE-024: Upcycling Standard - Installation Script
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This script installs the WIA-ENE-024 upcycling standard toolkit
#
# Usage:
#   chmod +x install.sh
#   ./install.sh
#
# @version 1.0.0
# @license MIT
#############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#############################################################################
# Helper Functions
#############################################################################

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                                                                ║"
    echo "║        WIA-ENE-024: Upcycling Standard 🔄                     ║"
    echo "║                                                                ║"
    echo "║        弘益人間 (홍익인간) - Benefit All Humanity              ║"
    echo "║                                                                ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

check_command() {
    if command -v $1 &> /dev/null; then
        print_success "$1 is installed"
        return 0
    else
        print_error "$1 is not installed"
        return 1
    fi
}

#############################################################################
# Installation Steps
#############################################################################

install_dependencies() {
    echo ""
    echo "📦 Checking dependencies..."
    echo ""

    local missing_deps=0

    # Check for Node.js
    if check_command "node"; then
        NODE_VERSION=$(node --version)
        print_info "Node.js version: $NODE_VERSION"
    else
        missing_deps=$((missing_deps + 1))
        print_info "Install Node.js from: https://nodejs.org/"
    fi

    # Check for npm
    if check_command "npm"; then
        NPM_VERSION=$(npm --version)
        print_info "npm version: $NPM_VERSION"
    else
        missing_deps=$((missing_deps + 1))
        print_info "Install npm with Node.js"
    fi

    # Check for git (optional)
    if check_command "git"; then
        GIT_VERSION=$(git --version)
        print_info "Git version: $GIT_VERSION"
    else
        print_info "Git is optional but recommended"
    fi

    # Check for jq (optional, for CLI)
    if check_command "jq"; then
        JQ_VERSION=$(jq --version)
        print_info "jq version: $JQ_VERSION"
    else
        print_info "jq is optional (used by CLI for JSON processing)"
        print_info "Install: apt-get install jq (Ubuntu) or brew install jq (macOS)"
    fi

    echo ""
    if [ $missing_deps -gt 0 ]; then
        print_error "Please install missing dependencies before continuing"
        return 1
    else
        print_success "All required dependencies are installed"
        return 0
    fi
}

install_typescript_sdk() {
    echo ""
    echo "📚 Installing TypeScript SDK..."
    echo ""

    cd "$SCRIPT_DIR/api/typescript"

    if [ -f "package.json" ]; then
        npm install
        print_success "TypeScript SDK dependencies installed"

        # Build TypeScript
        if [ -f "tsconfig.json" ]; then
            npm run build 2>/dev/null || print_info "Build script not found (optional)"
        fi
    else
        print_error "package.json not found in api/typescript/"
        return 1
    fi

    cd "$SCRIPT_DIR"
}

install_cli() {
    echo ""
    echo "🔧 Installing CLI tool..."
    echo ""

    CLI_PATH="$SCRIPT_DIR/cli/upcycling.sh"

    if [ -f "$CLI_PATH" ]; then
        chmod +x "$CLI_PATH"

        # Optionally create symlink in /usr/local/bin
        if [ -w "/usr/local/bin" ]; then
            ln -sf "$CLI_PATH" /usr/local/bin/upcycling
            print_success "CLI installed to /usr/local/bin/upcycling"
            print_info "You can now run: upcycling --help"
        else
            print_info "To use CLI globally, run:"
            print_info "  sudo ln -sf $CLI_PATH /usr/local/bin/upcycling"
            print_info "Or add to PATH:"
            print_info "  export PATH=\$PATH:$SCRIPT_DIR/cli"
        fi
    else
        print_error "CLI script not found at $CLI_PATH"
        return 1
    fi
}

verify_installation() {
    echo ""
    echo "🔍 Verifying installation..."
    echo ""

    local errors=0

    # Check spec file
    if [ -f "$SCRIPT_DIR/spec/WIA-ENE-024-v1.0.md" ]; then
        print_success "Specification file found"
    else
        print_error "Specification file not found"
        errors=$((errors + 1))
    fi

    # Check TypeScript types
    if [ -f "$SCRIPT_DIR/api/typescript/src/types.ts" ]; then
        print_success "TypeScript types found"
    else
        print_error "TypeScript types not found"
        errors=$((errors + 1))
    fi

    # Check TypeScript SDK
    if [ -f "$SCRIPT_DIR/api/typescript/src/index.ts" ]; then
        print_success "TypeScript SDK found"
    else
        print_error "TypeScript SDK not found"
        errors=$((errors + 1))
    fi

    # Check CLI
    if [ -f "$SCRIPT_DIR/cli/upcycling.sh" ]; then
        print_success "CLI tool found"
    else
        print_error "CLI tool not found"
        errors=$((errors + 1))
    fi

    echo ""
    if [ $errors -eq 0 ]; then
        print_success "Installation verified successfully!"
        return 0
    else
        print_error "Installation incomplete ($errors errors)"
        return 1
    fi
}

show_next_steps() {
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                                                                ║${NC}"
    echo -e "${GREEN}║  Installation Complete! 🎉                                     ║${NC}"
    echo -e "${GREEN}║                                                                ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "📖 Next Steps:"
    echo ""
    echo "1. Read the specification:"
    echo "   cat $SCRIPT_DIR/spec/WIA-ENE-024-v1.0.md"
    echo ""
    echo "2. Try the CLI:"
    echo "   upcycling --help"
    echo "   upcycling create --name \"My First Upcycling Project\""
    echo ""
    echo "3. Use the TypeScript SDK:"
    echo "   import { UpcyclingClient } from '@wia/ene-024';"
    echo ""
    echo "4. Explore example projects:"
    echo "   cd $SCRIPT_DIR/examples"
    echo ""
    echo "📚 Documentation:"
    echo "   https://docs.wia.org/ene-024"
    echo ""
    echo "🌐 Community:"
    echo "   https://github.com/WIA-Official/wia-standards"
    echo ""
    echo -e "${BLUE}弘益人間 (홍익인간) - Benefit All Humanity${NC}"
    echo ""
}

#############################################################################
# Main Installation Flow
#############################################################################

main() {
    print_header

    echo "Starting WIA-ENE-024 Upcycling Standard installation..."
    echo ""

    # Step 1: Check dependencies
    if ! install_dependencies; then
        print_error "Installation failed at dependency check"
        exit 1
    fi

    # Step 2: Install TypeScript SDK
    if ! install_typescript_sdk; then
        print_error "Installation failed at TypeScript SDK setup"
        exit 1
    fi

    # Step 3: Install CLI
    if ! install_cli; then
        print_error "Installation failed at CLI setup"
        exit 1
    fi

    # Step 4: Verify installation
    if ! verify_installation; then
        print_error "Installation verification failed"
        exit 1
    fi

    # Step 5: Show next steps
    show_next_steps

    exit 0
}

# Run main installation
main
