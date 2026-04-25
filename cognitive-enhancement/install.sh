#!/bin/bash

################################################################################
# WIA-AUG-005: Cognitive Enhancement - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Cognitive Enhancement Research Group
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
    echo "║      🧠 WIA-AUG-005: Cognitive Enhancement Installer          ║"
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
        print_info "Ubuntu/Debian: sudo apt-get install bc"
        print_info "macOS: brew install bc"
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

    local cli_path="$SCRIPT_DIR/cli/wia-aug-005.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-005"
        print_success "Installed to $install_dir/wia-aug-005"
    else
        if sudo cp "$cli_path" "$install_dir/wia-aug-005" 2>/dev/null; then
            print_success "Installed to $install_dir/wia-aug-005 (sudo)"
        else
            print_warning "Could not install to $install_dir"
            print_info "You can manually copy: cp $cli_path $install_dir/wia-aug-005"
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

    print_info "Installing dependencies..."
    npm install 2>&1 | grep -v "^npm WARN" || true

    if command -v npx &> /dev/null && [ -f "node_modules/.bin/tsup" ]; then
        print_info "Building SDK..."
        npm run build 2>/dev/null || print_warning "Build skipped (tsup configuration needed)"
    else
        print_warning "Build tools not available, skipping build"
    fi

    print_success "TypeScript SDK ready at: $ts_dir"
    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-005/docs"
    mkdir -p "$docs_dir"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "Copied README.md"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Copied specification files"
    fi

    print_success "Documentation installed at: $docs_dir"
}

verify_installation() {
    print_section "Verifying Installation"

    if command -v wia-aug-005 &> /dev/null; then
        print_success "CLI tool installed successfully"
        wia-aug-005 version
    else
        print_warning "CLI tool not in PATH"
        print_info "You may need to restart your terminal or add /usr/local/bin to PATH"
    fi
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}WIA-AUG-005 Cognitive Enhancement installed successfully!${RESET}"
    echo ""
    echo "Quick Start:"
    echo -e "${CYAN}  1. Assess your baseline:${RESET}"
    echo "     wia-aug-005 assess-baseline --user YOUR-ID"
    echo ""
    echo -e "${CYAN}  2. Enhance a cognitive domain:${RESET}"
    echo "     wia-aug-005 enhance --domain MEMORY --method COMPUTATIONAL"
    echo ""
    echo -e "${CYAN}  3. Measure performance:${RESET}"
    echo "     wia-aug-005 measure --user YOUR-ID --domain ALL"
    echo ""
    echo -e "${CYAN}  4. Optimize attention (convenience):${RESET}"
    echo "     wia-aug-005 optimize-attention --user YOUR-ID"
    echo ""
    echo "Available Commands:"
    echo -e "${CYAN}  wia-aug-005 assess-baseline${RESET}    - Baseline cognitive assessment"
    echo -e "${CYAN}  wia-aug-005 enhance${RESET}            - Enhance cognitive domain"
    echo -e "${CYAN}  wia-aug-005 measure${RESET}            - Measure performance"
    echo -e "${CYAN}  wia-aug-005 monitor-load${RESET}       - Monitor cognitive load"
    echo -e "${CYAN}  wia-aug-005 manage-fatigue${RESET}     - Manage fatigue"
    echo -e "${CYAN}  wia-aug-005 optimize-attention${RESET} - Optimize attention"
    echo -e "${CYAN}  wia-aug-005 decision-support${RESET}   - Decision support"
    echo -e "${CYAN}  wia-aug-005 help${RESET}               - Show full help"
    echo ""
    echo "TypeScript SDK:"
    if command -v npm &> /dev/null; then
        echo "  cd $SCRIPT_DIR/api/typescript"
        echo "  npm link (to use globally)"
        echo ""
        echo "  Or in your project:"
        echo "  npm install @wia/aug-005"
    else
        echo "  Install Node.js and npm to use the TypeScript SDK"
    fi
    echo ""
    echo "Documentation:"
    echo "  Local: $HOME/.wia/aug-005/docs/"
    echo "  Online: https://wiastandards.com/standards/cognitive-enhancement"
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
    verify_installation
    show_completion
}

main "$@"
