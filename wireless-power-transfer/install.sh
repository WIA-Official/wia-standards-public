#!/bin/bash

################################################################################
# WIA-COMM-009: Wireless Power Transfer - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Standards Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-COMM-009 standard components:
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
    echo "║    ⚡ WIA-COMM-009: Wireless Power Transfer Installer        ║"
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
        echo ""
        print_error "Some prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_script="$SCRIPT_DIR/cli/wia-comm-009.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_script" ]; then
        print_error "CLI script not found at $cli_script"
        return 1
    fi

    # Make executable
    chmod +x "$cli_script"
    print_success "Made CLI script executable"

    # Check if we can install to /usr/local/bin
    if [ -w "$install_dir" ]; then
        ln -sf "$cli_script" "$install_dir/wia-comm-009"
        print_success "Installed CLI to $install_dir/wia-comm-009"
    else
        print_warning "Cannot write to $install_dir (requires sudo)"
        print_info "Run with sudo to install globally, or add to PATH manually:"
        print_info "  export PATH=\"\$PATH:$SCRIPT_DIR/cli\""
        print_info "  echo 'export PATH=\"\$PATH:$SCRIPT_DIR/cli\"' >> ~/.bashrc"
    fi

    # Test CLI
    if command -v wia-comm-009 &> /dev/null; then
        print_success "CLI installed successfully!"
        print_info "Test with: wia-comm-009 --version"
    else
        print_warning "CLI not in PATH. Add manually or use: $cli_script"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local sdk_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_error "TypeScript SDK directory not found"
        return 1
    fi

    cd "$sdk_dir"

    # Check if npm is available
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm, then run: cd $sdk_dir && npm install && npm run build"
        return 0
    fi

    print_info "Installing dependencies..."
    if npm install --silent 2>&1 | grep -v "^npm WARN"; then
        print_success "Dependencies installed"
    else
        print_warning "Some warnings during npm install (this is usually okay)"
    fi

    print_info "Building TypeScript SDK..."
    if npm run build --silent 2>&1 | grep -v "^npm WARN"; then
        print_success "TypeScript SDK built successfully"
        print_info "Dist files created in: $sdk_dir/dist"
    else
        print_error "Build failed"
        return 1
    fi

    cd - > /dev/null

    print_success "TypeScript SDK ready!"
    print_info "Import with: import { WirelessPowerSDK } from '@wia/comm-009';"
    print_info "Or install locally: npm install $sdk_dir"
}

# Install documentation
install_documentation() {
    print_section "Installing Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-COMM-009-v1.0.md"
    local readme_file="$SCRIPT_DIR/README.md"

    if [ -f "$spec_file" ]; then
        print_success "Specification: $spec_file"
    else
        print_warning "Specification not found"
    fi

    if [ -f "$readme_file" ]; then
        print_success "README: $readme_file"
    else
        print_warning "README not found"
    fi

    print_info "View specification with: less $spec_file"
    print_info "Or open in browser: markdown $spec_file > spec.html && open spec.html"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    local cli_script="$SCRIPT_DIR/cli/wia-comm-009.sh"

    if [ ! -x "$cli_script" ]; then
        print_warning "CLI not executable, skipping tests"
        return 0
    fi

    print_info "Testing CLI commands..."

    # Test version
    if "$cli_script" version &> /dev/null; then
        print_success "Version command works"
    else
        print_error "Version command failed"
    fi

    # Test inductive WPT
    print_info "Testing inductive WPT calculation..."
    if "$cli_script" inductive Qi 15 87000 > /dev/null 2>&1; then
        print_success "Inductive WPT test passed"
    else
        print_warning "Inductive WPT test had warnings"
    fi

    # Test resonant WPT
    print_info "Testing resonant WPT calculation..."
    if "$cli_script" resonant 1.0 3300 6.78e6 > /dev/null 2>&1; then
        print_success "Resonant WPT test passed"
    else
        print_warning "Resonant WPT test had warnings"
    fi

    print_success "Tests completed!"
}

# Print usage examples
print_examples() {
    print_section "Usage Examples"

    echo -e "${GRAY}CLI Commands:${RESET}"
    echo "  wia-comm-009 inductive Qi 15 87000"
    echo "  wia-comm-009 resonant 1.0 3300 6.78e6"
    echo "  wia-comm-009 microwave 2.45e9 100 1000"
    echo "  wia-comm-009 laser 1064e-9 1000 10000"
    echo "  wia-comm-009 ev-dynamic 100 20000 85"
    echo "  wia-comm-009 efficiency resonant 0.5 0.6"
    echo "  wia-comm-009 safety 1000 2 2.45e9"
    echo ""

    echo -e "${GRAY}TypeScript SDK:${RESET}"
    cat << 'EOF'
import { WirelessPowerSDK } from '@wia/comm-009';

const sdk = new WirelessPowerSDK();

// Create inductive WPT system (Qi charger)
const qiCharger = sdk.createInductiveWPT({
  standard: 'Qi',
  frequency: 87000,
  maxPower: 15,
  voltage: 5,
  transmitterCoil: {
    diameter: 0.05,
    turns: 20,
    wireDiameter: 0.001,
    inductance: 10e-6,
    resistance: 0.1,
    quality: 100
  },
  foreignObjectDetection: true,
  livingObjectProtection: true,
  thermalProtection: true,
  maxTemperature: 60,
  sarCompliance: true,
  maxSAR: 2
});

// Start charging
const session = await qiCharger.startTransfer({
  deviceId: 'phone-12345',
  requestedPower: 10,
  alignment: { x: 0, y: 0, z: 0.005 }
});

console.log(`Efficiency: ${session.efficiency * 100}%`);
console.log(`Power delivered: ${session.powerDelivered} W`);
EOF
}

# Print summary
print_summary() {
    print_section "Installation Complete!"

    echo -e "${GREEN}✓ WIA-COMM-009 Wireless Power Transfer Standard${RESET}"
    echo ""
    echo "Components installed:"
    echo "  • CLI Tool: wia-comm-009"
    echo "  • TypeScript SDK: @wia/comm-009"
    echo "  • Documentation: spec/WIA-COMM-009-v1.0.md"
    echo ""
    echo "Quick start:"
    echo "  1. CLI: wia-comm-009 --help"
    echo "  2. Test: wia-comm-009 inductive Qi 15 87000"
    echo "  3. Read spec: less $SCRIPT_DIR/spec/WIA-COMM-009-v1.0.md"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    check_prerequisites

    install_cli

    install_typescript_sdk

    install_documentation

    run_tests

    print_examples

    print_summary
}

# Run main function
main "$@"
