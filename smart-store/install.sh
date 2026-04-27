#!/bin/bash

################################################################################
# WIA-IND-021: Smart Store - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-021 standard components:
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
AMBER='\033[38;5;214m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🏪 WIA-IND-021: Smart Store Installer                   ║"
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

    # Check openssl (for random ID generation)
    if command -v openssl &> /dev/null; then
        print_success "OpenSSL: $(openssl version)"
    else
        print_warning "OpenSSL not found (optional, will use fallback)"
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
        print_error "Required dependencies missing"
        exit 1
    fi

    echo ""
    print_success "All required prerequisites met"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-ind-021.sh"
    local cli_dest="/usr/local/bin/wia-ind-021"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI tool not found: $cli_source"
        return 1
    fi

    # Make executable
    chmod +x "$cli_source"
    print_success "Made CLI tool executable"

    # Try to install to /usr/local/bin
    if [ -w "/usr/local/bin" ]; then
        cp "$cli_source" "$cli_dest"
        chmod +x "$cli_dest"
        print_success "CLI tool installed to: $cli_dest"
    else
        print_warning "Cannot write to /usr/local/bin (need sudo)"
        print_info "Run manually: sudo cp $cli_source $cli_dest"
        print_info "Or add to PATH: export PATH=\"$SCRIPT_DIR/cli:\$PATH\""

        # Offer to add to PATH
        echo ""
        read -p "Add CLI to PATH in ~/.bashrc? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "" >> ~/.bashrc
            echo "# WIA-IND-021 Smart Store CLI" >> ~/.bashrc
            echo "export PATH=\"$SCRIPT_DIR/cli:\$PATH\"" >> ~/.bashrc
            print_success "Added to ~/.bashrc"
            print_info "Run: source ~/.bashrc"
        fi
    fi

    # Verify installation
    if command -v wia-ind-021 &> /dev/null; then
        print_success "CLI tool verified: $(wia-ind-021 --version | grep Version)"
    else
        print_warning "CLI not in PATH yet, restart shell or source ~/.bashrc"
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript SDK not found: $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not installed, skipping TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    if npm install --silent; then
        print_success "Dependencies installed"
    else
        print_warning "Dependency installation failed (continuing...)"
    fi

    print_info "Building SDK..."
    if npm run build --silent 2>/dev/null; then
        print_success "SDK built successfully"
    else
        print_warning "SDK build failed (may need to run manually)"
        print_info "Run: cd $ts_dir && npm install && npm run build"
    fi

    cd "$SCRIPT_DIR"

    echo ""
    print_info "To use the TypeScript SDK in your project:"
    print_info "  npm install $ts_dir"
    print_info "Or publish to npm registry:"
    print_info "  cd $ts_dir && npm publish"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-IND-021-v1.0.md"
    local readme_file="$SCRIPT_DIR/README.md"

    if [ -f "$spec_file" ]; then
        print_success "Specification: $spec_file"
        print_info "  $(wc -l < "$spec_file") lines"
    else
        print_warning "Specification not found"
    fi

    if [ -f "$readme_file" ]; then
        print_success "README: $readme_file"
    else
        print_warning "README not found"
    fi

    echo ""
    print_info "View documentation:"
    print_info "  Specification: cat $spec_file | less"
    print_info "  README: cat $readme_file | less"
    print_info "  Online: https://wiastandards.com/standards/WIA-IND-021"
}

# Setup environment
setup_environment() {
    print_section "Setting Up Environment"

    print_info "Setting environment variables..."

    # Check if already set
    if [ -z "$WIA_STORE_ID" ]; then
        echo ""
        read -p "Enter Store ID (default: store-001): " store_id
        store_id=${store_id:-store-001}

        echo "" >> ~/.bashrc
        echo "# WIA-IND-021 Smart Store Configuration" >> ~/.bashrc
        echo "export WIA_STORE_ID=\"$store_id\"" >> ~/.bashrc

        print_success "Store ID set to: $store_id"
        print_info "Added to ~/.bashrc"
    else
        print_success "Store ID already set: $WIA_STORE_ID"
    fi

    echo ""
    print_info "Environment variables:"
    print_info "  WIA_STORE_ID      - Store identifier"
    print_info "  WIA_API_ENDPOINT  - API endpoint URL"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-ind-021 &> /dev/null; then
        print_info "Testing CLI tool..."
        if wia-ind-021 --version &> /dev/null; then
            print_success "CLI test passed"
        else
            print_warning "CLI test failed"
        fi
    fi

    # Test TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK build exists"
    else
        print_warning "TypeScript SDK not built"
    fi
}

# Print installation summary
print_summary() {
    print_section "Installation Summary"

    echo ""
    print_success "WIA-IND-021 Smart Store installation complete!"

    echo ""
    echo -e "${CYAN}Quick Start:${RESET}"
    echo ""
    echo "1. Create a checkout session:"
    echo "   $ wia-ind-021 checkout create customer-123 app"
    echo ""
    echo "2. Monitor inventory:"
    echo "   $ wia-ind-021 inventory monitor shelf-42 5"
    echo ""
    echo "3. View analytics:"
    echo "   $ wia-ind-021 analytics heatmap dairy today"
    echo ""
    echo "4. Generate report:"
    echo "   $ wia-ind-021 report generate daily"
    echo ""
    echo -e "${CYAN}TypeScript Usage:${RESET}"
    echo ""
    echo "  import { SmartStoreSDK } from '@wia/ind-021';"
    echo ""
    echo "  const store = new SmartStoreSDK({"
    echo "    storeId: 'my-store',"
    echo "    enableVision: true"
    echo "  });"
    echo ""
    echo "  const session = await store.startCheckoutSession({"
    echo "    customerId: 'customer-123',"
    echo "    authMethod: 'app'"
    echo "  });"
    echo ""
    echo -e "${CYAN}Documentation:${RESET}"
    echo ""
    echo "  Specification: $SCRIPT_DIR/spec/WIA-IND-021-v1.0.md"
    echo "  README:        $SCRIPT_DIR/README.md"
    echo "  Online:        https://wiastandards.com/standards/WIA-IND-021"
    echo ""
    echo -e "${CYAN}Support:${RESET}"
    echo ""
    echo "  GitHub:   https://github.com/WIA-Official/wia-standards"
    echo "  Website:  https://wiastandards.com"
    echo "  Docs:     https://docs.wiastandards.com"
    echo ""
    echo -e "${GRAY}弘益人間 (홍익인간) · Benefit All Humanity${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    check_prerequisites
    install_cli
    install_typescript
    install_docs
    setup_environment
    run_tests
    print_summary
}

# Run main function
main "$@"

################################################################################
# 弘익人間 (홍익인간) · Benefit All Humanity
################################################################################
