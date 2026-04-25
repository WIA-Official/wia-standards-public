#!/bin/bash

###############################################################################
# WIA-TIME-011: Historical Integrity Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"
CLI_NAME="wia-time-011"

print_banner() {
    echo -e "${MAGENTA}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║      WIA-TIME-011: Historical Integrity Installer        ║"
    echo "║                                                           ║"
    echo "║         弘益人間 (Benefit All Humanity)                   ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo ""
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_requirements() {
    print_info "Checking requirements..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for openssl (used for random generation)
    if ! command -v openssl &> /dev/null; then
        print_warning "openssl not found (optional for advanced features)"
    fi

    # Check for bc (used for calculations)
    if ! command -v bc &> /dev/null; then
        print_warning "bc not found (optional for calculations)"
    fi

    # Check for node (for TypeScript SDK)
    if ! command -v node &> /dev/null; then
        print_warning "Node.js not found (required for TypeScript SDK)"
    else
        local node_version=$(node --version | sed 's/v//')
        print_info "Node.js version: $node_version"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        exit 1
    fi

    print_success "All required dependencies found"
}

install_cli() {
    print_info "Installing CLI tool..."

    local cli_source="$SCRIPT_DIR/cli/$CLI_NAME.sh"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        exit 1
    fi

    # Make CLI executable
    chmod +x "$cli_source"

    # Try to create symlink or copy
    if [ -w "$INSTALL_DIR" ]; then
        ln -sf "$cli_source" "$INSTALL_DIR/$CLI_NAME"
        print_success "CLI installed to $INSTALL_DIR/$CLI_NAME"
    else
        print_warning "No write permission to $INSTALL_DIR"
        print_info "Trying with sudo..."

        if command -v sudo &> /dev/null; then
            sudo ln -sf "$cli_source" "$INSTALL_DIR/$CLI_NAME"
            print_success "CLI installed to $INSTALL_DIR/$CLI_NAME (with sudo)"
        else
            print_error "Cannot install CLI. Please run with appropriate permissions."
            print_info "Alternatively, add to PATH: export PATH=\"$SCRIPT_DIR/cli:\$PATH\""
            exit 1
        fi
    fi
}

install_typescript_sdk() {
    print_info "Installing TypeScript SDK..."

    local sdk_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_warning "TypeScript SDK directory not found"
        return
    fi

    cd "$sdk_dir"

    # Check if npm is available
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "To install manually: cd $sdk_dir && npm install && npm run build"
        return
    fi

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK installed and built"
    print_info "To use in your project: npm install @wia/time-011"
}

verify_installation() {
    print_info "Verifying installation..."

    if command -v "$CLI_NAME" &> /dev/null; then
        print_success "CLI tool is accessible in PATH"

        local version=$("$CLI_NAME" version | head -n 1)
        print_info "Version: $version"
    else
        print_warning "CLI tool not in PATH. You may need to add $INSTALL_DIR to PATH"
    fi
}

print_next_steps() {
    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                Installation Complete! 🎉                 ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "Next Steps:"
    echo ""
    echo "1. Verify Installation:"
    echo "   $ $CLI_NAME version"
    echo ""
    echo "2. View Help:"
    echo "   $ $CLI_NAME help"
    echo ""
    echo "3. Try Examples:"
    echo "   $ $CLI_NAME verify-event --event 'test-event' --hash '0x123...'"
    echo "   $ $CLI_NAME check-integrity --timeline 'my-timeline'"
    echo "   $ $CLI_NAME create-checkpoint --timeline 'my-timeline' --name 'backup'"
    echo ""
    echo "4. TypeScript SDK Usage:"
    echo "   $ npm install @wia/time-011"
    echo ""
    echo "Documentation:"
    echo "  - README: $SCRIPT_DIR/README.md"
    echo "  - Spec:   $SCRIPT_DIR/spec/WIA-TIME-011-v1.0.md"
    echo "  - Web:    https://wiastandards.com/standards/WIA-TIME-011"
    echo ""
    echo -e "${MAGENTA}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
}

main() {
    print_banner

    check_requirements
    echo ""

    install_cli
    echo ""

    if [ "${SKIP_SDK:-0}" != "1" ]; then
        install_typescript_sdk
        echo ""
    fi

    verify_installation
    echo ""

    print_next_steps
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-sdk)
            SKIP_SDK=1
            shift
            ;;
        --install-dir)
            INSTALL_DIR="$2"
            shift 2
            ;;
        --help)
            echo "Usage: ./install.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --skip-sdk         Skip TypeScript SDK installation"
            echo "  --install-dir DIR  Install CLI to specified directory (default: /usr/local/bin)"
            echo "  --help             Show this help message"
            echo ""
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

main
