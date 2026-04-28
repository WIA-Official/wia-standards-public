#!/bin/bash

################################################################################
# WIA-COMM-005: Satellite Internet - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Communications Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-COMM-005 standard components:
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
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🛰️ WIA-COMM-005: Satellite Internet Installer          ║"
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
        print_error "Missing required dependencies"
        return 1
    fi

    return 0
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-comm-005.sh"
    local cli_target="/usr/local/bin/wia-comm-005"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        return 1
    fi

    # Make executable
    chmod +x "$cli_source"
    print_success "Made CLI executable"

    # Check if we can install to /usr/local/bin
    if [ -w "/usr/local/bin" ]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
        print_success "Installed CLI to: $cli_target"
    else
        print_warning "Cannot write to /usr/local/bin (permission denied)"
        print_info "Run with sudo, or manually copy:"
        print_info "  sudo cp $cli_source $cli_target"
        print_info "  sudo chmod +x $cli_target"
        print_info ""
        print_info "Alternative: Add to PATH in current directory"
        print_info "  export PATH=\"\$PATH:$SCRIPT_DIR/cli\""
    fi

    # Verify installation
    if command -v wia-comm-005 &> /dev/null; then
        print_success "CLI installed successfully!"
        print_info "Run: wia-comm-005 --help"
    else
        print_warning "CLI not in PATH, use full path: $cli_source"
    fi
}

# Install TypeScript SDK
install_sdk() {
    print_section "Installing TypeScript SDK"

    local sdk_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_error "SDK directory not found: $sdk_dir"
        return 1
    fi

    cd "$sdk_dir"

    if command -v npm &> /dev/null; then
        print_info "Installing npm dependencies..."
        npm install

        print_info "Building TypeScript SDK..."
        npm run build

        print_success "TypeScript SDK built successfully!"
        print_info "Import with: import { SatelliteNetwork } from '@wia/comm-005';"
    else
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm, then run:"
        print_info "  cd $sdk_dir"
        print_info "  npm install"
        print_info "  npm run build"
    fi

    cd "$SCRIPT_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-COMM-005-v1.0.md"
    local readme_file="$SCRIPT_DIR/README.md"

    if [ -f "$spec_file" ]; then
        print_success "Specification: $spec_file"
    else
        print_warning "Specification not found: $spec_file"
    fi

    if [ -f "$readme_file" ]; then
        print_success "README: $readme_file"
    else
        print_warning "README not found: $readme_file"
    fi

    print_info "Documentation available at:"
    print_info "  Specification: $spec_file"
    print_info "  README: $readme_file"
    print_info "  Online: https://wiastandards.com/standards/WIA-COMM-005"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    local cli_path
    if command -v wia-comm-005 &> /dev/null; then
        cli_path="wia-comm-005"
    else
        cli_path="$SCRIPT_DIR/cli/wia-comm-005.sh"
        chmod +x "$cli_path"
    fi

    print_info "Testing CLI version..."
    if $cli_path version &> /dev/null; then
        print_success "CLI test passed"
    else
        print_warning "CLI test failed"
    fi

    # Test SDK (if available)
    local sdk_dir="$SCRIPT_DIR/api/typescript"
    if [ -d "$sdk_dir/node_modules" ]; then
        print_info "Running SDK tests..."
        cd "$sdk_dir"
        if npm test &> /dev/null; then
            print_success "SDK tests passed"
        else
            print_warning "SDK tests failed (may be normal if no tests defined)"
        fi
        cd "$SCRIPT_DIR"
    fi
}

# Show usage examples
show_examples() {
    print_section "Usage Examples"

    echo ""
    echo "CLI Tool:"
    echo "  wia-comm-005 link-budget --distance 600 --frequency 28 --power 10"
    echo "  wia-comm-005 doppler --velocity 7.5 --frequency 28"
    echo "  wia-comm-005 predict --lat 37.7749 --lon -122.4194"
    echo "  wia-comm-005 debris-check --altitude 550 --inclination 53"
    echo ""
    echo "TypeScript SDK:"
    echo "  import { SatelliteNetwork, calculateLinkBudget } from '@wia/comm-005';"
    echo ""
    echo "  const network = new SatelliteNetwork({"
    echo "    constellation: 'LEO',"
    echo "    altitude: 550,"
    echo "    totalSatellites: 1584"
    echo "  });"
    echo ""
    echo "  const linkBudget = calculateLinkBudget({"
    echo "    distance: 600,"
    echo "    frequency: 28,"
    echo "    txPower: 10"
    echo "  });"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Check prerequisites
    if ! check_prerequisites; then
        print_error "Installation aborted due to missing prerequisites"
        exit 1
    fi

    # Install components
    install_cli
    install_sdk
    install_docs

    # Run tests
    run_tests

    # Show examples
    show_examples

    # Final message
    echo ""
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${GREEN}✓ Installation Complete!${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo ""
    echo -e "${CYAN}Quick Start:${RESET}"
    echo "  wia-comm-005 --help"
    echo "  wia-comm-005 link-budget 600 28 10"
    echo ""
    echo -e "${CYAN}Documentation:${RESET}"
    echo "  README: $SCRIPT_DIR/README.md"
    echo "  Spec: $SCRIPT_DIR/spec/WIA-COMM-005-v1.0.md"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA · MIT License${RESET}"
    echo ""
}

# Run main function
main "$@"
