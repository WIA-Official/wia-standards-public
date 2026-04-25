#!/bin/bash

################################################################################
# WIA-AUG-003: Neural Enhancement - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Neural Enhancement Working Group
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
    echo "║      🧠 WIA-AUG-003: Neural Enhancement Installer              ║"
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

    local cli_path="$SCRIPT_DIR/cli/wia-aug-003.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-003"
    else
        sudo cp "$cli_path" "$install_dir/wia-aug-003"
    fi
    print_success "Installed to $install_dir/wia-aug-003"

    # Verify installation
    if command -v wia-aug-003 &> /dev/null; then
        print_success "CLI tool verified: $(wia-aug-003 version | grep Version | cut -d: -f2)"
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
        print_warning "TypeScript directory not found, skipping"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install --silent 2>/dev/null || npm install

    print_info "Building SDK..."
    npm run build 2>/dev/null || print_warning "Build skipped (tsup not configured)"

    print_success "TypeScript SDK ready at: $ts_dir"
    print_info "Install globally: npm install -g ."
    print_info "Or link locally: npm link"

    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-003/docs"
    mkdir -p "$docs_dir"

    # Copy documentation files
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "README.md installed"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Specification documents installed"
    fi

    print_success "Documentation installed at: $docs_dir"
}

create_examples() {
    print_section "Creating Example Files"

    local examples_dir="$HOME/.wia/aug-003/examples"
    mkdir -p "$examples_dir"

    # Create basic example
    cat > "$examples_dir/basic_classification.sh" << 'EOF'
#!/bin/bash
# WIA-AUG-003 Example: Basic Interface Classification

echo "=== Neural Interface Classification Example ==="
wia-aug-003 classify \
    --type cortical \
    --location motor_cortex \
    --electrodes 128 \
    --resolution high

echo -e "\n=== Signal Processing Example ==="
wia-aug-003 process \
    --signal ecog \
    --sampling-rate 2000 \
    --filter 0.5-200

echo -e "\n=== BCI Calibration Example ==="
wia-aug-003 calibrate \
    --sessions 15 \
    --convergence 0.90 \
    --adaptation 0.1
EOF

    chmod +x "$examples_dir/basic_classification.sh"
    print_success "Example scripts created at: $examples_dir"
}

run_tests() {
    print_section "Running Basic Tests"

    # Test CLI availability
    if command -v wia-aug-003 &> /dev/null; then
        print_success "CLI tool is accessible"

        # Test version command
        if wia-aug-003 version &> /dev/null; then
            print_success "Version command works"
        fi

        # Test classify command
        print_info "Testing classification command..."
        wia-aug-003 classify --type cortical --electrodes 64 > /dev/null 2>&1
        print_success "Classification command works"

    else
        print_warning "CLI tool not in PATH, may need to restart shell"
    fi
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}WIA-AUG-003 Neural Enhancement installed successfully!${RESET}"
    echo ""
    echo "📚 Documentation:"
    echo -e "${CYAN}  ~/.wia/aug-003/docs/${RESET}"
    echo ""
    echo "🔧 Available Commands:"
    echo -e "${CYAN}  wia-aug-003 classify${RESET}   - Classify neural interface"
    echo -e "${CYAN}  wia-aug-003 process${RESET}    - Process neural signals"
    echo -e "${CYAN}  wia-aug-003 map${RESET}        - Map neural pathways"
    echo -e "${CYAN}  wia-aug-003 load${RESET}       - Check cognitive load"
    echo -e "${CYAN}  wia-aug-003 protect${RESET}    - Validate neuroprotection"
    echo -e "${CYAN}  wia-aug-003 calibrate${RESET}  - Calibrate BCI"
    echo -e "${CYAN}  wia-aug-003 report${RESET}     - Generate assessment report"
    echo -e "${CYAN}  wia-aug-003 help${RESET}       - Show detailed help"
    echo ""
    echo "📦 TypeScript SDK:"
    echo -e "${GRAY}  cd $SCRIPT_DIR/api/typescript${RESET}"
    echo -e "${GRAY}  npm link${RESET}"
    echo ""
    echo "📖 Examples:"
    echo -e "${GRAY}  ~/.wia/aug-003/examples/basic_classification.sh${RESET}"
    echo ""
    echo "🧪 Quick Test:"
    echo -e "${CYAN}  wia-aug-003 classify --type cortical --electrodes 128${RESET}"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

show_usage() {
    print_header
    echo "Usage: ./install.sh [options]"
    echo ""
    echo "Options:"
    echo "  --cli-only          Install CLI tool only"
    echo "  --sdk-only          Install TypeScript SDK only"
    echo "  --no-examples       Skip example creation"
    echo "  --no-tests          Skip running tests"
    echo "  --help              Show this help"
    echo ""
}

main() {
    local install_cli=true
    local install_sdk=true
    local create_examples_flag=true
    local run_tests_flag=true

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --cli-only)
                install_sdk=false
                shift
                ;;
            --sdk-only)
                install_cli=false
                shift
                ;;
            --no-examples)
                create_examples_flag=false
                shift
                ;;
            --no-tests)
                run_tests_flag=false
                shift
                ;;
            --help)
                show_usage
                exit 0
                ;;
            *)
                echo -e "${RED}Unknown option: $1${RESET}"
                show_usage
                exit 1
                ;;
        esac
    done

    print_header
    check_prerequisites

    if [ "$install_cli" = true ]; then
        install_cli
    fi

    if [ "$install_sdk" = true ]; then
        install_typescript
    fi

    install_docs

    if [ "$create_examples_flag" = true ]; then
        create_examples
    fi

    if [ "$run_tests_flag" = true ]; then
        run_tests
    fi

    show_completion
}

main "$@"
