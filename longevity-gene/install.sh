#!/bin/bash

################################################################################
# WIA-AUG-017: Longevity Gene Editing - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Human Augmentation Longevity Group
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
    echo "║   🧬 WIA-AUG-017: Longevity Gene Editing Installer            ║"
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

    if command -v bc &> /dev/null; then
        print_success "bc: installed"
    else
        print_warning "bc not found (install for calculations)"
        print_info "  Ubuntu/Debian: sudo apt-get install bc"
        print_info "  macOS: brew install bc"
    fi

    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional for TypeScript SDK)"
    fi

    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional for TypeScript SDK)"
    fi
}

install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-aug-017.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-017"
        print_success "Installed to $install_dir/wia-aug-017"
    else
        if command -v sudo &> /dev/null; then
            sudo cp "$cli_path" "$install_dir/wia-aug-017"
            print_success "Installed to $install_dir/wia-aug-017 (with sudo)"
        else
            print_warning "Cannot install to $install_dir (no write permission)"
            print_info "  Run: sudo cp $cli_path $install_dir/wia-aug-017"
        fi
    fi
}

install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK"
        return 0
    fi

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found"
        return 1
    fi

    cd "$ts_dir"
    print_info "Installing dependencies..."
    npm install --silent 2>/dev/null || npm install

    print_info "Building SDK..."
    npm run build 2>/dev/null || print_warning "Build skipped (tsup may not be configured)"

    print_success "TypeScript SDK ready at: $ts_dir"
    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-017/docs"
    mkdir -p "$docs_dir"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "README copied"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Specification copied"
    fi

    print_success "Documentation installed at: $docs_dir"
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}🧬 WIA-AUG-017 Longevity Gene Editing installed successfully!${RESET}"
    echo ""
    echo "CLI Commands:"
    echo -e "${CYAN}  wia-aug-017 assess-age${RESET}      - Assess biological age from biomarkers"
    echo -e "${CYAN}  wia-aug-017 select-genes${RESET}    - Select optimal target genes"
    echo -e "${CYAN}  wia-aug-017 design-protocol${RESET} - Design gene editing protocol"
    echo -e "${CYAN}  wia-aug-017 off-target${RESET}      - Evaluate off-target effects"
    echo -e "${CYAN}  wia-aug-017 monitor${RESET}         - Monitor treatment efficacy"
    echo -e "${CYAN}  wia-aug-017 healthspan${RESET}      - Track healthspan extension"
    echo -e "${CYAN}  wia-aug-017 risks${RESET}           - Assess treatment risks"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${CYAN}  npm install @wia/aug-017${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/aug-017/docs/${RESET}"
    echo ""
    echo "Quick Start:"
    echo -e "${CYAN}  wia-aug-017 assess-age --epigenetic 52 --telomere 6.8 --senescent 12 --chronological 45${RESET}"
    echo -e "${CYAN}  wia-aug-017 select-genes --age 45 --goals healthspan --risk moderate${RESET}"
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
