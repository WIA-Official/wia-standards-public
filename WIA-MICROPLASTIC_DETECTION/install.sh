#!/usr/bin/env bash

# WIA-MICROPLASTIC_DETECTION Installation Script
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
STANDARD_NAME="WIA-MICROPLASTIC_DETECTION"
STANDARD_VERSION="1.0.0"
INSTALL_DIR="${HOME}/.wia/${STANDARD_NAME}"
BIN_DIR="${HOME}/.local/bin"

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                                                            ║"
    echo "║        WIA-MICROPLASTIC_DETECTION Installer v1.0           ║"
    echo "║                                                            ║"
    echo "║        弘益人間 (Benefit All Humanity)                      ║"
    echo "║                                                            ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

check_prerequisites() {
    print_info "Checking prerequisites..."

    local missing=()

    # Check bash version
    if [[ "${BASH_VERSINFO[0]}" -lt 4 ]]; then
        missing+=("bash>=4.0")
    fi

    # Check required commands
    for cmd in curl jq; do
        if ! command -v "$cmd" &> /dev/null; then
            missing+=("$cmd")
        fi
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        print_error "Missing prerequisites: ${missing[*]}"
        print_info "Please install missing dependencies:"
        print_info "  Ubuntu/Debian: sudo apt-get install ${missing[*]}"
        print_info "  macOS: brew install ${missing[*]}"
        exit 1
    fi

    print_success "All prerequisites satisfied"
}

install_cli() {
    print_info "Installing CLI tool..."

    # Create bin directory
    mkdir -p "$BIN_DIR"

    # Copy CLI script
    local cli_source="./cli/wia-microplastic-detection.sh"
    local cli_target="$BIN_DIR/wia-microplastic-detection"

    if [[ -f "$cli_source" ]]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
        print_success "CLI tool installed to $cli_target"
    else
        print_warning "CLI source not found: $cli_source"
    fi

    # Check if bin directory is in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        print_warning "$BIN_DIR is not in PATH"
        print_info "Add the following to your ~/.bashrc or ~/.zshrc:"
        echo "    export PATH=\"\$PATH:$BIN_DIR\""
    fi
}

install_typescript_sdk() {
    print_info "Installing TypeScript SDK..."

    if command -v npm &> /dev/null; then
        local sdk_dir="./api/typescript"

        if [[ -d "$sdk_dir" ]]; then
            cd "$sdk_dir"

            print_info "Installing dependencies..."
            npm install

            print_info "Building SDK..."
            npm run build

            print_success "TypeScript SDK built successfully"

            cd - > /dev/null

            print_info "To use the SDK in your project:"
            echo "    npm install @wia/microplastic-detection"
        else
            print_warning "TypeScript SDK directory not found"
        fi
    else
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js to use the TypeScript SDK: https://nodejs.org/"
    fi
}

configure_api_key() {
    print_info "Configuring API access..."

    local config_dir="${HOME}/.wia"
    local config_file="${config_dir}/microplastic-detection.conf"

    mkdir -p "$config_dir"

    if [[ -f "$config_file" ]]; then
        print_info "Configuration file already exists: $config_file"
        read -p "Do you want to update the API key? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            return
        fi
    fi

    echo -e "${YELLOW}"
    read -p "Enter your WIA API key (or press Enter to skip): " api_key
    echo -e "${NC}"

    if [[ -n "$api_key" ]]; then
        echo "API_KEY=\"$api_key\"" > "$config_file"
        chmod 600 "$config_file"
        print_success "API key saved to $config_file"
    else
        print_info "API key configuration skipped"
        print_info "You can configure it later by setting WIA_API_KEY environment variable"
        print_info "Or create $config_file with: API_KEY=\"your_api_key\""
    fi
}

install_documentation() {
    print_info "Installing documentation..."

    mkdir -p "$INSTALL_DIR"

    # Copy ebook files
    if [[ -d "./ebook" ]]; then
        cp -r ./ebook "$INSTALL_DIR/"
        print_success "eBook installed to $INSTALL_DIR/ebook"
        print_info "View English guide: file://$INSTALL_DIR/ebook/en/index.html"
        print_info "View Korean guide: file://$INSTALL_DIR/ebook/ko/index.html"
    fi

    # Copy spec files
    if [[ -d "./spec" ]]; then
        cp -r ./spec "$INSTALL_DIR/"
        print_success "Specifications installed to $INSTALL_DIR/spec"
    fi
}

run_tests() {
    print_info "Running installation tests..."

    # Test CLI
    if command -v wia-microplastic-detection &> /dev/null; then
        local version
        version=$(wia-microplastic-detection --version 2>&1 | head -n1)
        print_success "CLI test passed: $version"
    else
        print_warning "CLI not found in PATH (restart shell or add $BIN_DIR to PATH)"
    fi

    # Test TypeScript SDK
    if [[ -f "./api/typescript/dist/index.js" ]]; then
        print_success "TypeScript SDK build verified"
    else
        print_warning "TypeScript SDK not built"
    fi
}

show_next_steps() {
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                                                            ║${NC}"
    echo -e "${GREEN}║        Installation Complete! 🎉                           ║${NC}"
    echo -e "${GREEN}║                                                            ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    print_info "Next Steps:"
    echo ""
    echo "  1. Restart your shell or run:"
    echo "     source ~/.bashrc   # or ~/.zshrc"
    echo ""
    echo "  2. View documentation:"
    echo "     open $INSTALL_DIR/ebook/en/index.html"
    echo ""
    echo "  3. Try the CLI:"
    echo "     wia-microplastic-detection --help"
    echo ""
    echo "  4. Use the SDK:"
    echo "     npm install @wia/microplastic-detection"
    echo ""
    echo "  5. Get an API key:"
    echo "     Visit: https://wia.org/get-api-key"
    echo ""
    print_info "Documentation: https://wia.org/standards/microplastic-detection"
    print_info "Support: standards@wia.org"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${NC}"
    echo -e "${BLUE}© 2025 SmileStory Inc. / WIA${NC}"
    echo ""
}

uninstall() {
    print_info "Uninstalling WIA-MICROPLASTIC_DETECTION..."

    # Remove CLI
    rm -f "$BIN_DIR/wia-microplastic-detection"
    print_success "CLI removed"

    # Remove installation directory
    rm -rf "$INSTALL_DIR"
    print_success "Installation directory removed"

    # Remove config
    rm -f "${HOME}/.wia/microplastic-detection.conf"
    print_success "Configuration removed"

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    print_header

    # Handle uninstall
    if [[ "$1" == "uninstall" ]]; then
        uninstall
        exit 0
    fi

    # Check if running from correct directory
    if [[ ! -f "./README.md" ]] || [[ ! -d "./cli" ]]; then
        print_error "Please run this script from the WIA-MICROPLASTIC_DETECTION directory"
        exit 1
    fi

    print_info "Installing $STANDARD_NAME v$STANDARD_VERSION"
    echo ""

    check_prerequisites
    echo ""

    install_cli
    echo ""

    install_typescript_sdk
    echo ""

    configure_api_key
    echo ""

    install_documentation
    echo ""

    run_tests
    echo ""

    show_next_steps
}

# Run main function
main "$@"
