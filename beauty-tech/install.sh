#!/bin/bash

################################################################################
# WIA-IND-004: Beauty Tech Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Beauty Technology Research Group
#
# 弘익人間 (Benefit All Humanity)
################################################################################

set -e

STANDARD_NAME="WIA-IND-004"
STANDARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLI_NAME="wia-ind-004"
VERSION="1.0.0"

# 색상 정의 (Color definitions)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

################################################################################
# Utility Functions
################################################################################

print_header() {
    echo ""
    echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${MAGENTA}║${NC}  💄 WIA-IND-004: Beauty Tech Standard Installer         ${MAGENTA}║${NC}"
    echo -e "${MAGENTA}║${NC}  Version ${VERSION}                                          ${MAGENTA}║${NC}"
    echo -e "${MAGENTA}║${NC}  弘益人間 (Benefit All Humanity)                          ${MAGENTA}║${NC}"
    echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${CYAN}ℹ${NC} $1"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        print_success "$1 is installed"
        return 0
    else
        print_warning "$1 is not installed"
        return 1
    fi
}

################################################################################
# Installation Steps
################################################################################

check_prerequisites() {
    echo -e "${BLUE}Checking prerequisites...${NC}"
    echo ""

    local all_good=true

    # Check for bash
    if check_command "bash"; then
        BASH_VERSION_NUM=$(bash --version | head -n1 | grep -oP '\d+\.\d+' | head -n1)
        echo "  Version: $BASH_VERSION_NUM"
    else
        print_error "Bash is required but not found"
        all_good=false
    fi

    # Check for bc (calculator)
    if check_command "bc"; then
        BC_VERSION=$(bc --version | head -n1 | grep -oP '\d+\.\d+(\.\d+)?' || echo "unknown")
        echo "  Version: $BC_VERSION"
    else
        print_error "bc (calculator) is required for calculations"
        all_good=false
    fi

    # Optional: Check for Node.js (for TypeScript SDK)
    if check_command "node"; then
        NODE_VERSION=$(node --version)
        echo "  Version: $NODE_VERSION"
    else
        print_warning "Node.js not found (optional, needed for TypeScript SDK)"
    fi

    # Optional: Check for npm
    if check_command "npm"; then
        NPM_VERSION=$(npm --version)
        echo "  Version: $NPM_VERSION"
    else
        print_warning "npm not found (optional, needed for TypeScript SDK)"
    fi

    echo ""

    if [ "$all_good" = false ]; then
        print_error "Missing required dependencies. Please install them and try again."
        echo ""
        echo "On Ubuntu/Debian:"
        echo "  sudo apt-get update"
        echo "  sudo apt-get install -y bash bc"
        echo ""
        echo "On macOS:"
        echo "  brew install bash bc"
        echo ""
        exit 1
    fi

    print_success "All prerequisites satisfied"
    echo ""
}

install_cli() {
    echo -e "${BLUE}Installing CLI tool...${NC}"
    echo ""

    local cli_source="${STANDARD_DIR}/cli/${CLI_NAME}.sh"
    local install_dir="/usr/local/bin"
    local cli_dest="${install_dir}/${CLI_NAME}"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI script not found at $cli_source"
        exit 1
    fi

    # Make CLI script executable
    chmod +x "$cli_source"
    print_success "Made CLI script executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_source" "$cli_dest"
    else
        print_info "Need sudo permission to install to $install_dir"
        sudo cp "$cli_source" "$cli_dest"
    fi

    print_success "Installed CLI to $cli_dest"

    # Verify installation
    if command -v "$CLI_NAME" &> /dev/null; then
        print_success "CLI tool is now available: $CLI_NAME"

        # Test the CLI
        local version_output=$("$CLI_NAME" version 2>&1 | grep -i "version" || echo "")
        if [ -n "$version_output" ]; then
            print_success "CLI verification passed"
        fi
    else
        print_warning "CLI installed but not in PATH. You may need to restart your terminal."
    fi

    echo ""
}

install_typescript_sdk() {
    echo -e "${BLUE}Installing TypeScript SDK...${NC}"
    echo ""

    local sdk_dir="${STANDARD_DIR}/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_error "TypeScript SDK directory not found at $sdk_dir"
        exit 1
    fi

    # Check if npm is available
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "To install the SDK later, run: cd $sdk_dir && npm install"
        echo ""
        return
    fi

    cd "$sdk_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    if npm install --silent; then
        print_success "npm dependencies installed"
    else
        print_warning "npm install had some issues, but continuing..."
    fi

    # Build TypeScript
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript..."
        if npm run build --silent 2>/dev/null; then
            print_success "TypeScript build successful"
        else
            print_warning "TypeScript build failed (tsconfig may need adjustment)"
        fi
    fi

    cd - > /dev/null

    echo ""
}

create_symlinks() {
    echo -e "${BLUE}Creating helpful symlinks...${NC}"
    echo ""

    local bin_dir="$HOME/.local/bin"

    # Create user bin directory if it doesn't exist
    if [ ! -d "$bin_dir" ]; then
        mkdir -p "$bin_dir"
        print_success "Created $bin_dir"
    fi

    # Create symlink in user bin
    local user_cli="$bin_dir/$CLI_NAME"
    local cli_source="${STANDARD_DIR}/cli/${CLI_NAME}.sh"

    if [ -f "$user_cli" ] || [ -L "$user_cli" ]; then
        rm -f "$user_cli"
    fi

    ln -s "$cli_source" "$user_cli"
    print_success "Created symlink: $user_cli"

    # Check if user bin is in PATH
    if [[ ":$PATH:" != *":$bin_dir:"* ]]; then
        print_warning "$bin_dir is not in your PATH"
        print_info "Add this to your ~/.bashrc or ~/.zshrc:"
        echo ""
        echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
        echo ""
    fi

    echo ""
}

setup_completion() {
    echo -e "${BLUE}Setting up shell completion (optional)...${NC}"
    echo ""

    print_info "Shell completion is not yet implemented"
    print_info "You can still use --help with any command"

    echo ""
}

print_installation_summary() {
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  ✨ Installation Complete!                               ${GREEN}║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Standard:${NC} $STANDARD_NAME (Beauty Tech)"
    echo -e "${CYAN}Version:${NC} $VERSION"
    echo -e "${CYAN}Location:${NC} $STANDARD_DIR"
    echo ""
    echo -e "${YELLOW}Quick Start:${NC}"
    echo ""
    echo "  # Show version"
    echo "  $CLI_NAME version"
    echo ""
    echo "  # Calculate skin health score"
    echo "  $CLI_NAME calc-skin-health --hydration 75 --elasticity 80 --texture 70"
    echo ""
    echo "  # Calculate foundation color match"
    echo "  $CLI_NAME calc-color-match --l1 68 --a1 8 --b1 18 --l2 70 --a2 7 --b2 19"
    echo ""
    echo "  # Calculate melanin index"
    echo "  $CLI_NAME calc-melanin-index --red 180 --green 150 --blue 120"
    echo ""
    echo "  # Show all available commands"
    echo "  $CLI_NAME help"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo "  • README: $STANDARD_DIR/README.md"
    echo "  • Specification: $STANDARD_DIR/spec/WIA-IND-004-v1.0.md"
    echo "  • TypeScript SDK: $STANDARD_DIR/api/typescript/"
    echo ""
    echo -e "${CYAN}Resources:${NC}"
    echo "  • Website: https://wiastandards.com/standards/WIA-IND-004"
    echo "  • GitHub: https://github.com/WIA-Official/wia-standards"
    echo "  • Documentation: https://docs.wiastandards.com"
    echo ""
    echo -e "${MAGENTA}弘益人間 (홍익인간) · Benefit All Humanity${NC}"
    echo ""
}

################################################################################
# Main Installation Flow
################################################################################

main() {
    print_header

    # Parse command line arguments
    local skip_cli=false
    local skip_sdk=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-cli)
                skip_cli=true
                shift
                ;;
            --skip-sdk)
                skip_sdk=true
                shift
                ;;
            --help|-h)
                echo "WIA-IND-004 Beauty Tech Standard Installer"
                echo ""
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --skip-cli    Skip CLI installation"
                echo "  --skip-sdk    Skip TypeScript SDK installation"
                echo "  --help        Show this help message"
                echo ""
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done

    # Run installation steps
    check_prerequisites

    if [ "$skip_cli" = false ]; then
        install_cli
        create_symlinks
    else
        print_info "Skipping CLI installation (--skip-cli)"
        echo ""
    fi

    if [ "$skip_sdk" = false ]; then
        install_typescript_sdk
    else
        print_info "Skipping TypeScript SDK installation (--skip-sdk)"
        echo ""
    fi

    setup_completion

    print_installation_summary
}

# Run main installation
main "$@"
