#!/bin/bash

################################################################################
# WIA-COMP-011: DevOps - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘익人間 (Benefit All Humanity)
################################################################################

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
GRAY='\033[0;90m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🔄 WIA-COMP-011: DevOps Installer                     ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${BLUE}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

check_prerequisites() {
    print_section "Checking Prerequisites"

    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1)"
    fi

    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    fi
}

install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-comp-011.sh"
    local install_dir="/usr/local/bin"

    chmod +x "$cli_path"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-comp-011"
    else
        sudo cp "$cli_path" "$install_dir/wia-comp-011"
    fi

    print_success "CLI installed to $install_dir/wia-comp-011"
}

install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if command -v npm &> /dev/null && [ -d "$ts_dir" ]; then
        cd "$ts_dir"
        npm install
        npm run build
        print_success "TypeScript SDK built successfully"
        cd "$SCRIPT_DIR"
    fi
}

show_completion() {
    print_section "Installation Complete!"
    echo ""
    echo -e "${GREEN}WIA-COMP-011 DevOps standard installed successfully!${RESET}"
    echo ""
    echo "Try: wia-comp-011 --help"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo ""
}

main() {
    print_header
    check_prerequisites
    install_cli
    install_typescript
    show_completion
}

main "$@"
