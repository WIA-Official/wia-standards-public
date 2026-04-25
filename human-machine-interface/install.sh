#!/bin/bash

################################################################################
# WIA-AUG-014: Human-Machine Interface - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Human Augmentation Interface Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🔌 WIA-AUG-014: Human-Machine Interface Installer        ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() { echo -e "\n${CYAN}▶ $1${RESET}"; }
print_success() { echo -e "${GREEN}✓ $1${RESET}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${RESET}"; }
print_error() { echo -e "${RED}✗ $1${RESET}"; }
print_info() { echo -e "${GRAY}  $1${RESET}"; }

check_prerequisites() {
    print_section "Checking Prerequisites"

    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1 | cut -d' ' -f1-4)"
    else
        print_error "Bash not found"
        exit 1
    fi

    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional)"
    fi
}

install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-aug-014.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-014"
    else
        sudo cp "$cli_path" "$install_dir/wia-aug-014"
    fi
    print_success "Installed to $install_dir/wia-aug-014"
}

install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript"
        return 0
    fi

    cd "$ts_dir"
    npm install
    npm run build 2>/dev/null || print_warning "Build skipped (tsup not configured)"
    print_success "TypeScript SDK ready"
    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-014/docs"
    mkdir -p "$docs_dir"

    [ -f "$SCRIPT_DIR/README.md" ] && cp "$SCRIPT_DIR/README.md" "$docs_dir/"
    [ -d "$SCRIPT_DIR/spec" ] && cp -r "$SCRIPT_DIR/spec" "$docs_dir/"

    print_success "Documentation at: $docs_dir"
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}WIA-AUG-014 Human-Machine Interface installed!${RESET}"
    echo ""
    echo "Commands:"
    echo -e "${CYAN}  wia-aug-014 connect${RESET}    - Connect to device"
    echo -e "${CYAN}  wia-aug-014 calibrate${RESET}  - Start calibration"
    echo -e "${CYAN}  wia-aug-014 monitor${RESET}    - Monitor metrics"
    echo -e "${CYAN}  wia-aug-014 test${RESET}       - Test communication"
    echo -e "${CYAN}  wia-aug-014 haptic${RESET}     - Send haptic feedback"
    echo -e "${CYAN}  wia-aug-014 state${RESET}      - Get device state"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

main() {
    print_header
    check_prerequisites
    install_cli
    install_typescript
    install_docs
    show_completion
}

main "$@"
