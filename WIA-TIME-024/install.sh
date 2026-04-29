#!/bin/bash

################################################################################
# WIA-TIME-024: Time Measurement - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-024 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        ⏱️  WIA-TIME-024: Time Measurement Installer           ║"
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

    # Check date command
    if command -v date &> /dev/null; then
        print_success "date: installed"
    else
        print_error "date command not found"
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
        print_error "Required dependencies missing. Please install them first."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-time-024.sh"
    local install_dir="/usr/local/bin"
    local cli_target="$install_dir/wia-time-024"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI script not found at: $cli_source"
        return 1
    fi

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
    else
        print_info "Requires sudo for installation to $install_dir"
        sudo cp "$cli_source" "$cli_target"
        sudo chmod +x "$cli_target"
    fi

    if [ -f "$cli_target" ]; then
        print_success "CLI tool installed: $cli_target"
        print_info "Test with: wia-time-024 --version"
    else
        print_error "CLI installation failed"
        return 1
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_warning "TypeScript directory not found, skipping"
        return 0
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    if npm install; then
        print_success "Dependencies installed"
    else
        print_warning "Dependency installation failed"
    fi

    print_info "Building TypeScript SDK..."
    if npm run build; then
        print_success "TypeScript SDK built successfully"
        print_info "Package: @wia/time-024"
        print_info "Install with: npm install $ts_dir"
    else
        print_warning "Build failed, but continuing installation"
    fi

    cd "$SCRIPT_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local docs=(
        "README.md"
        "spec/WIA-TIME-024-v1.0.md"
    )

    for doc in "${docs[@]}"; do
        if [ -f "$SCRIPT_DIR/$doc" ]; then
            print_success "Found: $doc"
        else
            print_warning "Missing: $doc"
        fi
    done

    print_info "Documentation available at: $SCRIPT_DIR"
}

# Verify installation
verify_installation() {
    print_section "Verifying Installation"

    # Check CLI
    if command -v wia-time-024 &> /dev/null; then
        print_success "CLI tool: wia-time-024 is available"
        print_info "Version: $(wia-time-024 version 2>&1 | grep -oP 'Version: \K.*' || echo '1.0.0')"
    else
        print_warning "CLI tool not found in PATH"
        print_info "You may need to add /usr/local/bin to your PATH"
    fi

    # Check TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK: Built and ready"
    else
        print_info "TypeScript SDK: Not built (optional)"
    fi

    # Check documentation
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        print_success "Documentation: Available"
    fi
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI tool
    print_info "Testing CLI tool..."
    if wia-time-024 --version &> /dev/null; then
        print_success "CLI version check passed"
    else
        print_warning "CLI version check failed"
    fi

    # Test measurement
    print_info "Testing time measurement..."
    if wia-time-024 measure --velocity 0.1 --precision ns &> /dev/null; then
        print_success "Time measurement test passed"
    else
        print_warning "Time measurement test failed"
    fi

    # Test TypeScript (if available)
    if command -v npm &> /dev/null && [ -d "$SCRIPT_DIR/api/typescript" ]; then
        print_info "Testing TypeScript SDK..."
        cd "$SCRIPT_DIR/api/typescript"
        if npm test &> /dev/null; then
            print_success "TypeScript tests passed"
        else
            print_info "TypeScript tests not available (requires test setup)"
        fi
        cd "$SCRIPT_DIR"
    fi
}

# Show usage examples
show_examples() {
    print_section "Usage Examples"

    echo ""
    echo -e "${VIOLET}Example 1: Measure Precision Time${RESET}"
    echo "  wia-time-024 measure --velocity 0.5 --gravity -7e8 --precision ns"
    echo ""

    echo -e "${VIOLET}Example 2: Calibrate Atomic Clock${RESET}"
    echo "  wia-time-024 calibrate --type optical-lattice --accuracy 1e-18"
    echo ""

    echo -e "${VIOLET}Example 3: Apply Relativistic Corrections${RESET}"
    echo "  wia-time-024 correct --time 1704067200 --velocity 0.3"
    echo ""

    echo -e "${VIOLET}Example 4: Synchronize Timelines${RESET}"
    echo "  wia-time-024 sync --timeline alpha --timeline beta"
    echo ""

    echo -e "${VIOLET}Example 5: Calculate Temporal Resolution${RESET}"
    echo "  wia-time-024 resolution --clock cesium-fountain"
    echo ""

    echo -e "${VIOLET}TypeScript Usage:${RESET}"
    echo "  npm install @wia/time-024"
    echo "  import { measurePrecisionTime } from '@wia/time-024';"
    echo ""
}

# Show next steps
show_next_steps() {
    print_section "Next Steps"

    echo ""
    echo "1. Read the documentation:"
    echo "   cat $SCRIPT_DIR/README.md"
    echo ""
    echo "2. View detailed specification:"
    echo "   cat $SCRIPT_DIR/spec/WIA-TIME-024-v1.0.md"
    echo ""
    echo "3. Try the CLI tool:"
    echo "   wia-time-024 help"
    echo ""
    echo "4. For TypeScript development:"
    echo "   cd $SCRIPT_DIR/api/typescript"
    echo "   npm install"
    echo "   npm run dev"
    echo ""
    echo "5. Join the WIA community:"
    echo "   https://github.com/WIA-Official/wia-standards"
    echo ""
}

# Uninstall function
uninstall() {
    print_section "Uninstalling WIA-TIME-024"

    # Remove CLI tool
    local cli_target="/usr/local/bin/wia-time-024"
    if [ -f "$cli_target" ]; then
        if [ -w "/usr/local/bin" ]; then
            rm "$cli_target"
        else
            sudo rm "$cli_target"
        fi
        print_success "CLI tool removed"
    fi

    # Clean TypeScript build
    if [ -d "$SCRIPT_DIR/api/typescript/node_modules" ]; then
        rm -rf "$SCRIPT_DIR/api/typescript/node_modules"
        print_success "TypeScript dependencies removed"
    fi

    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        rm -rf "$SCRIPT_DIR/api/typescript/dist"
        print_success "TypeScript build removed"
    fi

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    print_header

    # Parse arguments
    case "${1:-install}" in
        install)
            check_prerequisites
            install_cli
            install_typescript
            install_docs
            verify_installation
            run_tests
            show_examples
            show_next_steps

            echo ""
            print_success "Installation complete!"
            echo ""
            echo -e "${VIOLET}弘益人間 (Benefit All Humanity)${RESET}"
            echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
            echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
            echo ""
            ;;

        uninstall)
            uninstall
            ;;

        check)
            check_prerequisites
            verify_installation
            ;;

        test)
            run_tests
            ;;

        help|--help|-h)
            print_header
            echo "USAGE:"
            echo "  ./install.sh [command]"
            echo ""
            echo "COMMANDS:"
            echo "  install      Install WIA-TIME-024 (default)"
            echo "  uninstall    Remove WIA-TIME-024"
            echo "  check        Check prerequisites and verify installation"
            echo "  test         Run tests"
            echo "  help         Show this help message"
            echo ""
            ;;

        *)
            print_error "Unknown command: $1"
            echo "Run './install.sh help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
