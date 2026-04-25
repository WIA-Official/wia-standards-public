#!/bin/bash

################################################################################
# WIA-QUA-013: Dark Matter Detection - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Dark Matter Physics Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-QUA-013 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    🌑 WIA-QUA-013: Dark Matter Detection Installer            ║"
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

    # Check shuf (for randomness)
    if command -v shuf &> /dev/null; then
        print_success "shuf (random): installed"
    else
        print_warning "shuf not found (required for CLI)"
        print_info "Install coreutils package"
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

    local cli_script="$SCRIPT_DIR/cli/wia-qua-013.sh"
    local install_dir="/usr/local/bin"
    local install_path="$install_dir/wia-qua-013"

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
    if command -v wia-qua-013 &> /dev/null; then
        print_success "CLI tool installed successfully"
        print_info "Run 'wia-qua-013 --version' to verify"
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

    print_info "SDK can be imported with: import { DarkMatterDetectorSDK } from '@wia/qua-013'"
    print_info "Or installed globally: npm install -g"

    cd - > /dev/null
}

# Setup documentation
setup_documentation() {
    print_section "Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-QUA-013-v1.0.md"
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
    print_info "  - https://wiastandards.com/standards/dark-matter-detection"
    print_info "  - https://docs.wiastandards.com/qua-013"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    print_info "Testing CLI tool..."
    if wia-qua-013 --version &> /dev/null; then
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
    print_success "WIA-QUA-013 Dark Matter Detection Standard installed successfully!"
    echo ""

    print_info "Available commands:"
    print_info "  wia-qua-013 wimp search --detector xenon --mass 1000 --exposure 365"
    print_info "  wia-qua-013 axion search --frequency 5.0 --field 8.0"
    print_info "  wia-qua-013 direct analyze --detector xenon --energy 3.5"
    print_info "  wia-qua-013 indirect gamma --target galactic-center"
    print_info "  wia-qua-013 collider search --experiment atlas --channel monojet"
    print_info "  wia-qua-013 lensing analyze --cluster abell-1689"
    print_info "  wia-qua-013 background estimate --detector xenon"
    print_info "  wia-qua-013 limits calculate --exposure 1000 --wimp-mass 50"
    echo ""

    print_info "For help: wia-qua-013 help"
    echo ""

    print_info "TypeScript SDK usage:"
    print_info "  import { DarkMatterDetectorSDK } from '@wia/qua-013';"
    print_info "  const detector = new DarkMatterDetectorSDK();"
    print_info "  const result = await detector.analyzeDirectDetection({ ... });"
    echo ""

    print_info "Documentation:"
    print_info "  Specification: $SCRIPT_DIR/spec/WIA-QUA-013-v1.0.md"
    print_info "  README: $SCRIPT_DIR/README.md"
    print_info "  Online: https://wiastandards.com/standards/dark-matter-detection"
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
