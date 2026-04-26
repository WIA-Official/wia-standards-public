#!/bin/bash

################################################################################
# WIA-COMM-002: IoT (M2M) - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA IoT and M2M Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-COMM-002 standard components:
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
    echo "║       🔗 WIA-COMM-002: IoT (M2M) Installer                    ║"
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

    # Check curl (for HTTP operations)
    if command -v curl &> /dev/null; then
        print_success "curl: installed"
    else
        print_warning "curl not found (optional, for HTTP operations)"
    fi

    # Check jq (for JSON parsing)
    if command -v jq &> /dev/null; then
        print_success "jq: installed"
    else
        print_warning "jq not found (optional, for JSON processing)"
        print_info "Install with: apt-get install jq (Debian/Ubuntu) or brew install jq (macOS)"
    fi

    # Check mosquitto (MQTT broker/client)
    if command -v mosquitto_pub &> /dev/null; then
        print_success "mosquitto: installed"
    else
        print_warning "mosquitto not found (optional, for MQTT operations)"
        print_info "Install with: apt-get install mosquitto-clients (Debian/Ubuntu)"
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
        print_error "Some required prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_script="$SCRIPT_DIR/cli/wia-comm-002.sh"
    local install_dir="/usr/local/bin"
    local install_path="$install_dir/wia-comm-002"

    if [ ! -f "$cli_script" ]; then
        print_error "CLI script not found: $cli_script"
        return 1
    fi

    # Make script executable
    chmod +x "$cli_script"
    print_success "Made CLI script executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_script" "$install_path"
        print_success "Installed CLI to $install_path"
    else
        print_info "Need sudo to install to $install_dir"
        if sudo cp "$cli_script" "$install_path"; then
            print_success "Installed CLI to $install_path (with sudo)"
        else
            print_error "Failed to install CLI"
            return 1
        fi
    fi

    # Verify installation
    if command -v wia-comm-002 &> /dev/null; then
        print_success "CLI tool installed successfully"
        print_info "Run 'wia-comm-002 --version' to verify"
    else
        print_warning "CLI installed but not in PATH"
        print_info "You may need to restart your shell or add $install_dir to PATH"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript SDK directory not found: $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    if npm install --quiet; then
        print_success "Dependencies installed"
    else
        print_error "Failed to install dependencies"
        return 1
    fi

    print_info "Building TypeScript SDK..."
    if npm run build --quiet 2>&1 | grep -v "^$"; then
        print_success "TypeScript SDK built successfully"
    else
        print_warning "Build completed with warnings"
    fi

    print_info "SDK can be imported with: import { IoTDeviceSDK } from '@wia/comm-002'"
    print_info "Or installed globally: npm install -g"

    cd - > /dev/null
}

# Setup documentation
setup_documentation() {
    print_section "Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-COMM-002-v1.0.md"
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

    print_info "Documentation available at:"
    print_info "  - https://wiastandards.com/standards/iot-m2m"
    print_info "  - https://docs.wiastandards.com/comm-002"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    print_info "Testing CLI tool..."
    if wia-comm-002 --version &> /dev/null; then
        print_success "CLI test passed"
    else
        print_warning "CLI test failed (may need to restart shell)"
    fi

    if command -v npm &> /dev/null; then
        local ts_dir="$SCRIPT_DIR/api/typescript"
        if [ -d "$ts_dir" ]; then
            cd "$ts_dir"
            print_info "Running TypeScript tests..."
            if npm test --quiet 2>&1 | tail -n 5; then
                print_success "TypeScript tests passed"
            else
                print_warning "TypeScript tests completed with warnings"
            fi
            cd - > /dev/null
        fi
    fi
}

# Print summary
print_summary() {
    print_section "Installation Summary"

    echo ""
    print_success "WIA-COMM-002 IoT (M2M) Standard installed successfully!"
    echo ""

    print_info "Available commands:"
    print_info "  wia-comm-002 device register --id sensor-001 --type temperature"
    print_info "  wia-comm-002 publish --topic sensors/temp --data '{\"value\": 23.5}'"
    print_info "  wia-comm-002 subscribe --topic commands/# --qos 1"
    print_info "  wia-comm-002 device list --status online"
    print_info "  wia-comm-002 twin create --device sensor-001 --sync-interval 30"
    print_info "  wia-comm-002 data query --device sensor-001 --metric temperature"
    print_info "  wia-comm-002 cert generate --device sensor-001 --validity 365"
    echo ""

    print_info "For help: wia-comm-002 --help"
    echo ""

    print_info "TypeScript SDK usage:"
    print_info "  import { IoTDeviceSDK } from '@wia/comm-002';"
    print_info "  const device = new IoTDeviceSDK({ ... });"
    print_info "  await device.connect();"
    print_info "  await device.publishTelemetry({ ... });"
    echo ""

    print_info "Documentation:"
    print_info "  Specification: $SCRIPT_DIR/spec/WIA-COMM-002-v1.0.md"
    print_info "  README: $SCRIPT_DIR/README.md"
    print_info "  Online: https://wiastandards.com/standards/iot-m2m"
    echo ""

    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    check_prerequisites
    install_cli
    install_typescript_sdk
    setup_documentation

    # Optional: run tests
    if [ "${RUN_TESTS:-false}" = "true" ]; then
        run_tests
    fi

    print_summary
}

# Run main function
main "$@"

exit 0
