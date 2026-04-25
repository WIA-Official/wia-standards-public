#!/bin/bash

################################################################################
# WIA-IND-028: Digital Factory - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-028 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
AMBER='\033[0;33m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🏭 WIA-IND-028: Digital Factory Installer               ║"
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

    # Check bc (for calculations)
    if command -v bc &> /dev/null; then
        print_success "bc (calculator): installed"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Install with: apt-get install bc (Debian/Ubuntu) or brew install bc (macOS)"
        all_ok=false
    fi

    # Check jq (for JSON processing)
    if command -v jq &> /dev/null; then
        print_success "jq (JSON processor): $(jq --version)"
    else
        print_warning "jq not found (recommended for JSON processing)"
        print_info "Install with: apt-get install jq (Debian/Ubuntu) or brew install jq (macOS)"
    fi

    # Check curl (for API calls)
    if command -v curl &> /dev/null; then
        print_success "curl: $(curl --version | head -n1)"
    else
        print_warning "curl not found (required for API calls)"
        all_ok=false
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

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some required dependencies are missing"
        print_info "Install missing dependencies and run this script again"
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-ind-028.sh"
    local cli_target="/usr/local/bin/wia-ind-028"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI tool not found at $cli_source"
        return 1
    fi

    # Make CLI executable
    chmod +x "$cli_source"
    print_success "Made CLI executable"

    # Check if /usr/local/bin is writable
    if [ -w "/usr/local/bin" ]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
        print_success "CLI installed to $cli_target"
    else
        print_warning "/usr/local/bin is not writable"
        print_info "Run with sudo: sudo ./install.sh"
        print_info "Or use directly: $cli_source"

        # Create symlink in user's local bin
        local user_bin="$HOME/.local/bin"
        mkdir -p "$user_bin"

        if [ -d "$user_bin" ]; then
            ln -sf "$cli_source" "$user_bin/wia-ind-028"
            print_success "Created symlink in $user_bin/wia-ind-028"

            # Check if user's bin is in PATH
            if [[ ":$PATH:" != *":$user_bin:"* ]]; then
                print_warning "$user_bin is not in your PATH"
                print_info "Add to PATH: echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc"
            fi
        fi
    fi

    # Test CLI
    if command -v wia-ind-028 &> /dev/null; then
        print_success "CLI tool is available: $(which wia-ind-028)"
    else
        print_warning "CLI tool not in PATH, use directly: $cli_source"
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript SDK not found at $ts_dir"
        return 1
    fi

    # Check if npm is available
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        return 0
    fi

    # Install dependencies
    cd "$ts_dir"

    if [ -f "package.json" ]; then
        print_info "Installing npm dependencies..."
        npm install --quiet
        print_success "TypeScript SDK dependencies installed"

        # Build TypeScript
        if [ -f "tsconfig.json" ]; then
            print_info "Building TypeScript..."
            npm run build --if-present 2>/dev/null || print_warning "Build script not found, skipping"
        fi

        print_success "TypeScript SDK ready"
        print_info "Import with: import { DigitalFactorySDK } from '@wia/ind-028';"
    else
        print_warning "package.json not found, skipping npm install"
    fi

    cd "$SCRIPT_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        print_success "README.md: Installation and usage guide"
    fi

    if [ -f "$SCRIPT_DIR/spec/WIA-IND-028-v1.0.md" ]; then
        print_success "WIA-IND-028-v1.0.md: Complete specification"
    fi

    print_info "Documentation available in: $SCRIPT_DIR"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-ind-028 &> /dev/null; then
        print_info "Testing CLI tool..."
        if wia-ind-028 --version &> /dev/null; then
            print_success "CLI tool working"
        else
            print_warning "CLI tool installed but --version failed"
        fi
    fi

    # Test TypeScript SDK
    if command -v npm &> /dev/null && [ -d "$SCRIPT_DIR/api/typescript/node_modules" ]; then
        cd "$SCRIPT_DIR/api/typescript"
        if npm test --if-present 2>/dev/null; then
            print_success "TypeScript SDK tests passed"
        else
            print_info "TypeScript SDK tests not configured"
        fi
        cd "$SCRIPT_DIR"
    fi
}

# Print usage information
print_usage() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${AMBER}🏭 Digital Factory CLI${RESET}"
    echo ""
    echo "Usage examples:"
    echo "  wia-ind-028 --version                      # Check version"
    echo "  wia-ind-028 twin create --name \"My Factory\" # Create digital twin"
    echo "  wia-ind-028 monitor --factory FAC-001      # Monitor factory"
    echo "  wia-ind-028 simulate --scenario peak       # Run simulation"
    echo "  wia-ind-028 energy current                 # Check energy usage"
    echo ""

    echo -e "${AMBER}📦 TypeScript SDK${RESET}"
    echo ""
    echo "Installation:"
    echo "  npm install @wia/ind-028"
    echo ""
    echo "Usage:"
    echo "  import { DigitalFactorySDK } from '@wia/ind-028';"
    echo "  const factory = new DigitalFactorySDK({ factoryId: 'FAC-001' });"
    echo ""

    echo -e "${AMBER}📚 Documentation${RESET}"
    echo ""
    echo "  README.md               : $SCRIPT_DIR/README.md"
    echo "  Full Specification      : $SCRIPT_DIR/spec/WIA-IND-028-v1.0.md"
    echo "  TypeScript API          : $SCRIPT_DIR/api/typescript/src/"
    echo ""

    echo -e "${AMBER}🔗 Resources${RESET}"
    echo ""
    echo "  Website         : https://wiastandards.com"
    echo "  GitHub          : https://github.com/WIA-Official/wia-standards"
    echo "  Documentation   : https://docs.wiastandards.com"
    echo "  Certification   : https://cert.wiastandards.com"
    echo ""

    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${AMBER}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    check_prerequisites
    install_cli
    install_typescript
    install_docs
    run_tests
    print_usage
}

# Run main installation
main

# Exit successfully
exit 0

# **弘익人間 (홍익인간) · Benefit All Humanity**
