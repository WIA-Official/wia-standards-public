#!/bin/bash

################################################################################
# WIA-TIME-025: Temporal Verification - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-025 Temporal Verification tools including:
# - TypeScript SDK
# - CLI tool
# - Documentation
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
STANDARD_NAME="WIA-TIME-025"
STANDARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       ✅ WIA-TIME-025: Temporal Verification Installer        ║"
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
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

# Check if command exists
command_exists() {
    command -v "$1" &> /dev/null
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$STANDARD_DIR/cli/wia-time-025.sh"
    local cli_dest="/usr/local/bin/wia-time-025"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        return 1
    fi

    print_info "Source: $cli_source"
    print_info "Destination: $cli_dest"

    # Check if we have permission to install
    if [ ! -w "/usr/local/bin" ]; then
        print_warning "Need sudo permission to install to /usr/local/bin"
        sudo cp "$cli_source" "$cli_dest"
        sudo chmod +x "$cli_dest"
    else
        cp "$cli_source" "$cli_dest"
        chmod +x "$cli_dest"
    fi

    print_success "CLI tool installed: $cli_dest"

    # Verify installation
    if command_exists wia-time-025; then
        local installed_version=$(wia-time-025 version 2>&1 | grep "Version:" | awk '{print $2}' || echo "$VERSION")
        print_success "Verification: wia-time-025 v$installed_version"
    else
        print_warning "CLI tool installed but not in PATH"
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$STANDARD_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript source not found: $ts_dir"
        return 1
    fi

    cd "$ts_dir"

    # Check for Node.js
    if ! command_exists node; then
        print_warning "Node.js not found. Please install Node.js 18+ to use the TypeScript SDK"
        print_info "Visit: https://nodejs.org/"
        return 0
    fi

    local node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
    print_info "Node.js version: $(node --version)"

    if [ "$node_version" -lt 18 ]; then
        print_warning "Node.js 18+ required. Current version: $(node --version)"
        return 0
    fi

    # Check for npm
    if ! command_exists npm; then
        print_error "npm not found. Please install npm"
        return 1
    fi

    print_info "npm version: $(npm --version)"

    # Install dependencies
    print_info "Installing dependencies..."
    if npm install --silent; then
        print_success "Dependencies installed"
    else
        print_warning "Failed to install dependencies (may need to run manually)"
    fi

    # Build TypeScript
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript..."
        if npm run build --silent 2>/dev/null; then
            print_success "TypeScript compiled successfully"
        else
            print_info "TypeScript build skipped (run 'npm run build' manually)"
        fi
    fi

    print_success "TypeScript SDK ready: $ts_dir"
    print_info "Import with: import { TemporalVerifier } from '@wia/time-025';"

    cd "$STANDARD_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local docs=("README.md" "spec/WIA-TIME-025-v1.0.md")

    for doc in "${docs[@]}"; do
        if [ -f "$STANDARD_DIR/$doc" ]; then
            print_success "$doc"
        else
            print_warning "$doc not found"
        fi
    done

    print_info "Documentation location: $STANDARD_DIR"
}

# Run verification tests
run_tests() {
    print_section "Running Verification Tests"

    # Test CLI
    if command_exists wia-time-025; then
        print_info "Testing CLI tool..."
        if wia-time-025 version &>/dev/null; then
            print_success "CLI test passed"
        else
            print_warning "CLI test failed"
        fi
    else
        print_warning "CLI tool not found, skipping test"
    fi

    # Test TypeScript SDK
    if [ -f "$STANDARD_DIR/api/typescript/package.json" ]; then
        print_info "TypeScript SDK available"
        if [ -d "$STANDARD_DIR/api/typescript/node_modules" ]; then
            print_success "TypeScript SDK dependencies installed"
        else
            print_info "TypeScript SDK dependencies not installed (run 'npm install' in api/typescript/)"
        fi
    fi
}

# Display installation summary
show_summary() {
    print_section "Installation Summary"

    echo ""
    echo -e "${CYAN}Installed Components:${RESET}"

    if command_exists wia-time-025; then
        print_success "CLI Tool: /usr/local/bin/wia-time-025"
    else
        print_warning "CLI Tool: Not installed or not in PATH"
    fi

    if [ -d "$STANDARD_DIR/api/typescript/node_modules" ]; then
        print_success "TypeScript SDK: $STANDARD_DIR/api/typescript"
    else
        print_info "TypeScript SDK: $STANDARD_DIR/api/typescript (dependencies not installed)"
    fi

    print_success "Documentation: $STANDARD_DIR/spec/"

    echo ""
    echo -e "${CYAN}Quick Start:${RESET}"
    echo ""
    echo "  1. Verify a journey:"
    echo "     $ wia-time-025 verify-journey --id J-2024-001 --traveler TR-123456"
    echo ""
    echo "  2. Validate a signature:"
    echo "     $ wia-time-025 validate-signature --signature 0x... --key traveler.pub"
    echo ""
    echo "  3. Check travel log:"
    echo "     $ wia-time-025 check-log --file journey-log.json --blockchain true"
    echo ""
    echo "  4. TypeScript usage:"
    echo "     $ cd $STANDARD_DIR/api/typescript"
    echo "     $ npm install"
    echo "     $ npm run build"
    echo ""
    echo -e "${CYAN}Documentation:${RESET}"
    echo ""
    echo "  - README: $STANDARD_DIR/README.md"
    echo "  - Specification: $STANDARD_DIR/spec/WIA-TIME-025-v1.0.md"
    echo "  - Website: https://wiastandards.com/standards/WIA-TIME-025"
    echo ""
    echo -e "${CYAN}Need Help?${RESET}"
    echo ""
    echo "  $ wia-time-025 help"
    echo "  $ wia-time-025 version"
    echo ""
}

# Main installation flow
main() {
    print_header

    print_section "Installation Information"
    print_info "Standard: $STANDARD_NAME"
    print_info "Version: $VERSION"
    print_info "Installation Directory: $STANDARD_DIR"
    print_info "System: $(uname -s) $(uname -m)"
    echo ""

    # Check prerequisites
    print_section "Checking Prerequisites"

    if command_exists bash; then
        print_success "Bash: $(bash --version | head -n1)"
    else
        print_error "Bash not found"
        exit 1
    fi

    if command_exists node; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional, required for TypeScript SDK)"
    fi

    if command_exists npm; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional, required for TypeScript SDK)"
    fi

    # Perform installation
    install_cli
    install_typescript
    install_docs
    run_tests
    show_summary

    print_section "Installation Complete"
    print_success "WIA-TIME-025 Temporal Verification installed successfully!"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Run main installation
main "$@"
