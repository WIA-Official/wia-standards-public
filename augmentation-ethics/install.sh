#!/bin/bash

################################################################################
# WIA-AUG-012: Augmentation Ethics - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Human Augmentation Ethics Group
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
    echo "║      💠 WIA-AUG-012: Augmentation Ethics Installer            ║"
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
        print_warning "Node.js not found (optional for SDK)"
    fi

    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional for SDK)"
    fi
}

install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-aug-012.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-012"
        print_success "Installed to $install_dir/wia-aug-012"
    else
        if command -v sudo &> /dev/null; then
            sudo cp "$cli_path" "$install_dir/wia-aug-012"
            print_success "Installed to $install_dir/wia-aug-012 (with sudo)"
        else
            print_warning "Cannot install to $install_dir (no sudo available)"
            print_info "You can manually copy $cli_path to a directory in your PATH"
        fi
    fi
}

install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        return 0
    fi

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found"
        return 1
    fi

    cd "$ts_dir"
    print_info "Running npm install..."
    npm install 2>&1 | grep -v "^npm WARN" || true
    print_success "Dependencies installed"

    if npm run build &> /dev/null 2>&1; then
        print_success "TypeScript SDK built successfully"
    else
        print_warning "Build skipped (tsup may not be configured)"
    fi

    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-012/docs"
    mkdir -p "$docs_dir"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "README.md installed"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Specification installed"
    fi

    print_info "Documentation at: $docs_dir"
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}WIA-AUG-012 Augmentation Ethics installed successfully!${RESET}"
    echo ""
    echo "Available Commands:"
    echo -e "${CYAN}  wia-aug-012 assess${RESET}              - Assess ethical compliance"
    echo -e "${CYAN}  wia-aug-012 validate-consent${RESET}    - Validate informed consent"
    echo -e "${CYAN}  wia-aug-012 check-coercion${RESET}      - Check for coercion"
    echo -e "${CYAN}  wia-aug-012 evaluate-equity${RESET}     - Evaluate equity and access"
    echo -e "${CYAN}  wia-aug-012 assess-identity${RESET}     - Assess identity impact"
    echo -e "${CYAN}  wia-aug-012 review-reversibility${RESET} - Review reversibility"
    echo -e "${CYAN}  wia-aug-012 report${RESET}              - Generate ethics report"
    echo ""
    echo "Examples:"
    echo -e "${GRAY}  # Assess enhancement for 25-year-old${RESET}"
    echo -e "${GRAY}  wia-aug-012 assess --type ENHANCEMENT --age 25 --reversible yes${RESET}"
    echo ""
    echo -e "${GRAY}  # Check for occupational coercion${RESET}"
    echo -e "${GRAY}  wia-aug-012 check-coercion --context occupational --mandatory no${RESET}"
    echo ""
    echo -e "${GRAY}  # Evaluate equity with cost and access parameters${RESET}"
    echo -e "${GRAY}  wia-aug-012 evaluate-equity --cost 75000 --access limited${RESET}"
    echo ""
    echo "TypeScript SDK:"
    if command -v npm &> /dev/null; then
        echo -e "${CYAN}  npm install @wia/aug-012${RESET}"
        echo ""
        echo "  import { AugmentationEthicsSDK } from '@wia/aug-012';"
        echo "  const sdk = new AugmentationEthicsSDK();"
    else
        echo -e "${YELLOW}  Install Node.js and npm to use TypeScript SDK${RESET}"
    fi
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
